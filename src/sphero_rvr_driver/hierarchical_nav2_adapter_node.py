"""Live semantic-goal to Nav2 action adapter.

This node publishes no velocity command.  It accepts only digest-bound
semantic decisions plus their captured/current snapshots, resolves geometry
with project-owned deterministic code, and sends ``NavigateThroughPoses``
goals while a fresh physical authority heartbeat remains valid.
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Mapping, Optional

from .hierarchical_physical_binding import (
    AUTHORITY_HEARTBEAT_MAX_AGE_S,
    AUTHORITY_TOPIC,
    CONTROLLER_STATUS_TOPIC,
    GOAL_DISPATCH_TOPIC,
    NAV2_ACTION,
    resolve_goal_dispatch,
    transient_authority_hold,
    validate_authority_heartbeat,
)


def controller_status_cancel_mode(
    payload: Mapping[str, Any],
    *,
    source_sha: str,
    mission_id: str,
) -> str:
    """Map controller state to a stop-only adapter action."""

    if (
        payload.get("schema")
        != "sphero_rvr.hierarchical_controller_status.v1"
        or str(payload.get("source_sha", "")).strip() != source_sha
        or str(payload.get("mission_id", "")).strip() != mission_id
    ):
        return "veto"
    state = str(payload.get("state", "")).strip()
    if (
        state == "wait_planning"
        and payload.get("cancel_active_goal") is True
    ):
        return "replan"
    if state == "complete":
        return "complete"
    if state == "recovery_required":
        return "veto"
    return ""


def nav2_result_state(status: int, cancel_reason: str) -> tuple[str, str]:
    if status == 4:
        return "wait_planning", "nav2_result_status_4"
    if status == 6:
        # Nav2 uses ABORTED for ordinary planning/execution outcomes such as
        # "no valid path".  Exploration must discard that target and choose a
        # new one; it is not a loss of motor safety or process integrity.
        return "wait_planning", "nav2_result_status_6"
    if status == 5 and cancel_reason == "controller_replan":
        return "wait_planning", "controller_replan_cancelled"
    if status == 5 and cancel_reason == "controller_complete":
        return "complete", "controller_complete_cancelled"
    return "recovery_required", f"nav2_result_status_{status}"


def stronger_cancel_reason(current: str, requested: str) -> str:
    priority = {
        "": 0,
        "controller_replan": 1,
        "controller_complete": 2,
        "veto": 3,
    }
    return (
        requested
        if priority.get(requested, 3) > priority.get(current, 3)
        else current
    )


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateThroughPoses
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    class HierarchicalNav2AdapterNode(Node):
        def __init__(self):
            super().__init__("hierarchical_nav2_adapter")
            for name, default in {
                "enabled": False,
                "source_sha": "",
                "deployed_sha": "",
                "reviewed_sha": "",
                "authority_topic": AUTHORITY_TOPIC,
                "goal_dispatch_topic": GOAL_DISPATCH_TOPIC,
                "controller_status_topic": CONTROLLER_STATUS_TOPIC,
                "nav2_action": NAV2_ACTION,
                "authority_max_age_s": AUTHORITY_HEARTBEAT_MAX_AGE_S,
                "status_topic": "/mission_api/v2/hierarchical/status",
            }.items():
                self.declare_parameter(name, default)
            if not bool(self.get_parameter("enabled").value):
                raise ValueError(
                    "hierarchical Nav2 adapter is default-off; enabled must be explicit"
                )
            self._source_sha = self._required_provenance(
                "source_sha", "RVR_SOURCE_SHA"
            )
            self._deployed_sha = self._required_provenance(
                "deployed_sha", "RVR_DEPLOYED_SHA"
            )
            self._reviewed_sha = self._required_provenance(
                "reviewed_sha", "RVR_HIERARCHICAL_REVIEWED_SHA"
            )
            if not (
                self._source_sha
                == self._deployed_sha
                == self._reviewed_sha
            ):
                raise ValueError(
                    "hierarchical Nav2 adapter requires matching exact SHAs"
                )
            if (
                str(self.get_parameter("authority_topic").value)
                != AUTHORITY_TOPIC
                or str(self.get_parameter("goal_dispatch_topic").value)
                != GOAL_DISPATCH_TOPIC
                or str(
                    self.get_parameter("controller_status_topic").value
                )
                != CONTROLLER_STATUS_TOPIC
                or str(self.get_parameter("nav2_action").value) != NAV2_ACTION
            ):
                raise ValueError(
                    "hierarchical ROS authority, dispatch, status, and Nav2 action names are fixed"
                )
            self._authority: Optional[dict[str, Any]] = None
            self._authority_received_at_s: Optional[float] = None
            self._goal_handle = None
            self._last_batch_digest = ""
            self._pending_batch_digest = ""
            self._active_batch_digest = ""
            self._cancel_reasons: dict[str, str] = {}
            self._distance_remaining_m: Optional[float] = None
            self._poses_remaining: Optional[int] = None
            self._client = ActionClient(
                self, NavigateThroughPoses, NAV2_ACTION
            )
            self._status_pub = self.create_publisher(
                String, str(self.get_parameter("status_topic").value), 10
            )
            self.create_subscription(
                String, AUTHORITY_TOPIC, self._on_authority, 10
            )
            self.create_subscription(
                String, GOAL_DISPATCH_TOPIC, self._on_dispatch, 10
            )
            self.create_subscription(
                String,
                CONTROLLER_STATUS_TOPIC,
                self._on_controller_status,
                10,
            )
            self.create_timer(0.05, self._check_authority)
            self._publish_status("locked", "awaiting_fresh_authority")

        def _required_provenance(
            self, parameter: str, environment: str
        ) -> str:
            value = (
                str(self.get_parameter(parameter).value).strip()
                or os.environ.get(environment, "").strip()
            )
            if not value:
                raise ValueError(
                    f"{parameter} must come from reviewed deployment provenance"
                )
            return value

        def _on_authority(self, msg) -> None:
            try:
                payload = json.loads(str(msg.data))
                if not isinstance(payload, dict):
                    raise ValueError("authority must be an object")
            except (TypeError, ValueError, json.JSONDecodeError):
                self._authority = None
                self._authority_received_at_s = None
                self._cancel_for_veto("malformed_authority")
                return
            self._authority = payload
            self._authority_received_at_s = time.time()

        def _authority_valid(self) -> tuple[bool, str]:
            if (
                self._authority is None
                or self._authority_received_at_s is None
            ):
                return False, "authority_missing"
            return validate_authority_heartbeat(
                self._authority,
                now_s=time.time(),
                received_at_s=self._authority_received_at_s,
                source_sha=self._source_sha,
                deployed_sha=self._deployed_sha,
                reviewed_sha=self._reviewed_sha,
                max_age_s=float(
                    self.get_parameter("authority_max_age_s").value
                ),
            )

        def _on_controller_status(self, msg) -> None:
            try:
                payload = json.loads(str(msg.data))
                if not isinstance(payload, dict):
                    raise ValueError("controller status must be an object")
            except (TypeError, ValueError, json.JSONDecodeError):
                self._cancel_for_veto("malformed_controller_status")
                return
            mission_id = (
                ""
                if self._authority is None
                else str(self._authority.get("mission_id", "")).strip()
            )
            mode = controller_status_cancel_mode(
                payload,
                source_sha=self._source_sha,
                mission_id=mission_id,
            )
            if mode == "replan":
                self._cancel_for_replan(str(payload.get("reason", "")))
            elif mode == "complete":
                self._cancel_for_completion(
                    str(payload.get("reason", ""))
                )
            elif mode == "veto":
                self._cancel_for_veto("controller_status_veto")

        def _check_authority(self) -> None:
            valid, reason = self._authority_valid()
            if not valid:
                if transient_authority_hold(reason):
                    # The private command bridge independently publishes zero
                    # while authority is stale.  Preserve the Nav2 action so a
                    # transient Pi scheduling delay does not turn a safe hold
                    # into a terminal route cancellation.
                    self._publish_status("holding", reason)
                else:
                    self._cancel_for_veto(reason)

        def _on_dispatch(self, msg) -> None:
            valid, reason = self._authority_valid()
            if not valid:
                self._cancel_for_veto(reason)
                self._publish_status("rejected", reason)
                return
            try:
                raw = json.loads(str(msg.data))
                if not isinstance(raw, dict):
                    raise ValueError("dispatch must be an object")
                batch = resolve_goal_dispatch(
                    raw,
                    authority=self._authority or {},
                    now_s=time.time(),
                )
                payload = batch.to_json_dict()
            except Exception as exc:
                self._cancel_for_veto("dispatch_invalid")
                self._publish_status(
                    "rejected",
                    f"dispatch_invalid:{exc.__class__.__name__}",
                )
                return
            digest = str(payload["batch_digest"])
            if digest in {
                self._last_batch_digest,
                self._pending_batch_digest,
                self._active_batch_digest,
            }:
                return
            if not self._client.wait_for_server(timeout_sec=0.0):
                self._publish_status(
                    "recovery_required", "nav2_action_unavailable"
                )
                return
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self._pose(item) for item in batch.poses]
            self._pending_batch_digest = digest
            future = self._client.send_goal_async(
                goal,
                feedback_callback=(
                    lambda message, batch_digest=digest: self._on_feedback(
                        message, batch_digest
                    )
                ),
            )
            future.add_done_callback(
                lambda result, batch_digest=digest: self._on_goal_response(
                    result, batch_digest
                )
            )
            self._publish_status(
                "dispatching",
                str(payload["reason"]),
                batch=payload,
            )

        def _pose(self, item) -> Any:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(item.x_m)
            pose.pose.position.y = float(item.y_m)
            pose.pose.orientation.z = math.sin(float(item.yaw_rad) / 2.0)
            pose.pose.orientation.w = math.cos(float(item.yaw_rad) / 2.0)
            return pose

        def _on_goal_response(self, future, digest: str) -> None:
            try:
                handle = future.result()
            except Exception as exc:
                if digest != self._pending_batch_digest:
                    self._cancel_reasons.pop(digest, None)
                    return
                self._pending_batch_digest = ""
                self._cancel_reasons.pop(digest, None)
                self._publish_status(
                    "recovery_required",
                    f"nav2_goal_error:{exc.__class__.__name__}",
                )
                return
            if digest != self._pending_batch_digest:
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
                self._cancel_reasons.pop(digest, None)
                return
            self._pending_batch_digest = ""
            if handle is None or not handle.accepted:
                self._cancel_reasons.pop(digest, None)
                self._publish_status(
                    "recovery_required", "nav2_goal_rejected"
                )
                return
            self._goal_handle = handle
            self._last_batch_digest = digest
            self._active_batch_digest = digest
            result = handle.get_result_async()
            result.add_done_callback(
                lambda wrapped, batch_digest=digest: self._on_result(
                    wrapped, batch_digest
                )
            )
            cancel_reason = self._cancel_reasons.get(digest, "")
            if cancel_reason:
                handle.cancel_goal_async()
                self._publish_cancel_status(cancel_reason)
            else:
                self._publish_status(
                    "navigating", "nav2_goal_accepted"
                )

        def _on_feedback(self, message, digest: str) -> None:
            if digest != self._active_batch_digest:
                return
            try:
                feedback = message.feedback
                distance = float(feedback.distance_remaining)
                poses_remaining = int(feedback.number_of_poses_remaining)
                if not math.isfinite(distance) or distance < 0.0:
                    raise ValueError("invalid Nav2 distance")
                self._distance_remaining_m = distance
                self._poses_remaining = poses_remaining
            except (AttributeError, TypeError, ValueError):
                self._distance_remaining_m = None
                self._poses_remaining = None
            self._publish_status("navigating", "nav2_feedback")

        def _on_result(self, future, digest: str) -> None:
            cancel_reason = self._cancel_reasons.pop(digest, "")
            if digest != self._active_batch_digest:
                return
            try:
                wrapped = future.result()
                status = int(wrapped.status)
            except Exception as exc:
                self._goal_handle = None
                self._active_batch_digest = ""
                self._publish_status(
                    "recovery_required",
                    f"nav2_result_error:{exc.__class__.__name__}",
                )
                return
            self._goal_handle = None
            self._active_batch_digest = ""
            self._distance_remaining_m = 0.0
            self._poses_remaining = 0
            state, reason = nav2_result_state(status, cancel_reason)
            self._publish_status(state, reason)

        def _cancel_for_replan(self, reason: str) -> None:
            self._cancel_current("controller_replan")
            self._publish_status("wait_planning", str(reason))

        def _cancel_for_completion(self, reason: str) -> None:
            self._cancel_current("controller_complete")
            self._publish_status("complete", str(reason))

        def _cancel_for_veto(self, reason: str) -> None:
            self._cancel_current("veto")
            self._publish_status("locked", str(reason))

        def _cancel_current(self, requested_reason: str) -> None:
            digests = {
                self._pending_batch_digest,
                self._active_batch_digest,
            } - {""}
            active_was_cancelled = bool(
                self._cancel_reasons.get(self._active_batch_digest, "")
            )
            for digest in digests:
                self._cancel_reasons[digest] = stronger_cancel_reason(
                    self._cancel_reasons.get(digest, ""),
                    requested_reason,
                )
            if self._goal_handle is not None and not active_was_cancelled:
                self._goal_handle.cancel_goal_async()

        def _publish_cancel_status(self, cancel_reason: str) -> None:
            if cancel_reason == "controller_replan":
                self._publish_status(
                    "wait_planning",
                    "controller_replan_pending_acceptance",
                )
            elif cancel_reason == "controller_complete":
                self._publish_status(
                    "complete",
                    "controller_complete_pending_acceptance",
                )
            else:
                self._publish_status(
                    "locked", "veto_pending_acceptance"
                )

        def _publish_status(
            self,
            state: str,
            reason: str,
            *,
            batch: Optional[dict[str, Any]] = None,
        ) -> None:
            payload = {
                "schema": "sphero_rvr.hierarchical_nav2_adapter_status.v1",
                "state": str(state),
                "reason": str(reason),
                "source_sha": self._source_sha,
                "goal_active": self._goal_handle is not None,
                "last_batch_digest": self._last_batch_digest,
                "distance_remaining_m": self._distance_remaining_m,
                "poses_remaining": self._poses_remaining,
                "batch": batch,
                "direct_twist_publisher": False,
                "physical_execution_enabled": (
                    self._authority_valid()[0]
                    if self._authority is not None
                    else False
                ),
            }
            message = String()
            message.data = json.dumps(payload, sort_keys=True)
            self._status_pub.publish(message)

    rclpy.init(args=args)
    node = HierarchicalNav2AdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
