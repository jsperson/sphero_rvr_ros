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
from typing import Any, Optional

from .hierarchical_physical_binding import (
    AUTHORITY_HEARTBEAT_MAX_AGE_S,
    AUTHORITY_TOPIC,
    GOAL_DISPATCH_TOPIC,
    NAV2_ACTION,
    resolve_goal_dispatch,
    validate_authority_heartbeat,
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
                or str(self.get_parameter("nav2_action").value) != NAV2_ACTION
            ):
                raise ValueError(
                    "hierarchical ROS authority, dispatch, and Nav2 action names are fixed"
                )
            self._authority: Optional[dict[str, Any]] = None
            self._authority_received_at_s: Optional[float] = None
            self._goal_handle = None
            self._last_batch_digest = ""
            self._cancel_requested = False
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

        def _check_authority(self) -> None:
            valid, reason = self._authority_valid()
            if not valid:
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
            if digest == self._last_batch_digest:
                return
            if not self._client.wait_for_server(timeout_sec=0.0):
                self._publish_status("wait_planning", "nav2_action_unavailable")
                return
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self._pose(item) for item in batch.poses]
            self._cancel_requested = False
            future = self._client.send_goal_async(
                goal, feedback_callback=self._on_feedback
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
                self._publish_status(
                    "wait_planning",
                    f"nav2_goal_error:{exc.__class__.__name__}",
                )
                return
            if handle is None or not handle.accepted:
                self._publish_status("wait_planning", "nav2_goal_rejected")
                return
            self._goal_handle = handle
            self._last_batch_digest = digest
            self._cancel_requested = False
            result = handle.get_result_async()
            result.add_done_callback(self._on_result)
            self._publish_status("navigating", "nav2_goal_accepted")

        def _on_feedback(self, message) -> None:
            del message
            self._publish_status("navigating", "nav2_feedback")

        def _on_result(self, future) -> None:
            try:
                wrapped = future.result()
                status = int(wrapped.status)
            except Exception as exc:
                self._publish_status(
                    "recovery_required",
                    f"nav2_result_error:{exc.__class__.__name__}",
                )
                return
            self._goal_handle = None
            self._cancel_requested = False
            self._publish_status(
                "wait_planning" if status == 4 else "recovery_required",
                f"nav2_result_status_{status}",
            )

        def _cancel_for_veto(self, reason: str) -> None:
            if self._goal_handle is not None and not self._cancel_requested:
                self._cancel_requested = True
                self._goal_handle.cancel_goal_async()
            self._publish_status("locked", str(reason))

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
