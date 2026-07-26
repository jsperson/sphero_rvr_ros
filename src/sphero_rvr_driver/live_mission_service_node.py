"""Production ROS 2 owner for the persistent Mission Service seam.

This node observes existing safety-path topics. It never publishes Twist, sensor
commands, or a hardware command surface. A route-request publisher exists only
behind the explicit reviewed-SHA execution gate.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Optional

from .live_mission_service import (
    LiveRouteProgressExecutor,
    LiveStateCache,
    LiveStatusExecutor,
    SafetyGatedPromptRouteExecutor,
    live_executor_bindings,
)
from .mission_api import build_default_registry
from .mission_service import MissionService, MissionServiceServer
from .perception_navigation import (
    GoalRegion,
    LocalizationEstimate,
    RESULT_SCHEMA as NAVIGATION_RESULT_SCHEMA,
)
from .prompt_drive import CodexOAuthPromptDriveProvider, PromptDriveLimits, PromptDrivePlanner
from .prompt_drive_ros import RosLiveRouteExecutor
from .prompt_mission_controller import PromptMissionController
from .stationary_perception import (
    CodexOAuthStationaryIntentProvider,
    StationaryPerceptionController,
)
from .adaptive_mission_controller import CodexOAuthAdaptiveMissionIntentProvider, AdaptiveMissionLimits
from .adaptive_mission_live_controller import LiveAdaptiveMissionController
from .adaptive_mission_physical import PhysicalAdaptiveMissionExecutor
from .adaptive_mission_session import SystemdAdaptiveMissionSession


def _json_mapping(value: Any) -> dict[str, Any]:
    try:
        parsed = json.loads(str(value))
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError("status payload must be a JSON object") from exc
    if not isinstance(parsed, Mapping):
        raise ValueError("status payload must be a JSON object")
    return dict(parsed)


def _localization_mapping(value: Any) -> dict[str, Any]:
    """Validate the selected lidar-localization status topic contract."""

    parsed = _json_mapping(value)
    if parsed.get("schema") == NAVIGATION_RESULT_SCHEMA:
        localization = parsed.get("localization")
        goal = parsed.get("goal")
        if not isinstance(localization, Mapping) or not isinstance(goal, Mapping):
            raise ValueError("navigation result requires localization and goal objects")
        parsed["localization"] = LocalizationEstimate.from_mapping(
            localization
        ).to_json_dict()
        parsed["goal"] = GoalRegion.from_mapping(goal).to_json_dict()
        if bool(parsed.get("motion_authority", True)):
            raise ValueError("navigation status must not claim motion authority")
        if bool(parsed.get("physical_execution_enabled", True)):
            raise ValueError("Perception navigation navigation status must keep physical execution disabled")
        return parsed
    return LocalizationEstimate.from_mapping(parsed).to_json_dict()


def _collision_mapping(value: Any) -> dict[str, Any]:
    raw = str(value).strip()
    if not raw:
        raise ValueError("collision status payload is empty")
    try:
        return _json_mapping(raw)
    except ValueError:
        result: dict[str, Any] = {"state": raw.split(maxsplit=1)[0].upper(), "raw": raw}
        numeric_fields = {
            "scan_age": "scan_age_s",
            "front": "front_clearance_m",
            "front_slow": "forward_corridor_clearance_m",
            "front_slow_min_angle_deg": "forward_corridor_min_angle_deg",
            "front_slow_max_angle_deg": "forward_corridor_max_angle_deg",
            "stop_distance_m": "collision_stop_distance_m",
            "slow_distance_m": "collision_slow_distance_m",
            "rear": "rear_clearance_m",
            "left": "left_clearance_m",
            "right": "right_clearance_m",
            "trajectory_clearance_margin_m": "trajectory_clearance_margin_m",
            "trajectory_horizon_s": "trajectory_horizon_s",
            "trajectory_min_clearance_m": "trajectory_min_clearance_m",
            "trajectory_collision_time_s": "trajectory_collision_time_s",
        }
        boolean_fields = {
            "scan_healthy": "scan_healthy",
            "tf_available": "tf_available",
        }
        text_fields = {
            "reason": "reason",
            "scan_reason": "scan_reason",
            "tf_reason": "tf_reason",
        }
        for token in raw.split()[1:]:
            key, separator, token_value = token.partition("=")
            output_key = numeric_fields.get(key)
            if not separator:
                continue
            boolean_key = boolean_fields.get(key)
            if boolean_key is not None and token_value.lower() in {"true", "false"}:
                result[boolean_key] = token_value.lower() == "true"
                continue
            text_key = text_fields.get(key)
            if text_key is not None:
                result[text_key] = token_value
                continue
            if key in {"requested", "output"}:
                pair = token_value.strip("()").split(",", 1)
                if len(pair) == 2:
                    try:
                        linear = float(pair[0])
                        angular = float(pair[1])
                    except ValueError:
                        continue
                    if math.isfinite(linear) and math.isfinite(angular):
                        prefix = "requested" if key == "requested" else "supervised"
                        result[f"{prefix}_linear_mps"] = linear
                        result[f"{prefix}_angular_rad_s"] = angular
                continue
            if output_key is None or token_value in {"", "None"}:
                continue
            try:
                parsed_value = float(token_value)
            except ValueError:
                continue
            if math.isfinite(parsed_value):
                result[output_key] = parsed_value
        return result


def _control_mapping(value: Any) -> dict[str, Any]:
    raw = str(value).strip()
    if not raw:
        raise ValueError("control status payload is empty")
    try:
        parsed = _json_mapping(raw)
    except ValueError:
        state = raw.split(maxsplit=1)[0].upper()
        if state not in {
            "READY",
            "CLEAR",
            "RUNNING",
            "STOP",
            "STOPPED",
            "ESTOP",
            "ESTOPPED",
            "LATCHED",
            "CANCEL",
            "CANCELLED",
        }:
            raise ValueError("control status payload has an unsupported state")
        return {"state": state, "raw": raw}
    state = str(parsed.get("state", "")).strip().upper()
    has_boolean_state = isinstance(parsed.get("stop_active"), bool) or isinstance(
        parsed.get("estop_latched"), bool
    )
    if state:
        if state not in {
            "READY",
            "CLEAR",
            "RUNNING",
            "STOP",
            "STOPPED",
            "ESTOP",
            "ESTOPPED",
            "LATCHED",
            "CANCEL",
            "CANCELLED",
        }:
            raise ValueError("control status payload has an unsupported state")
        parsed["state"] = state
    elif not has_boolean_state:
        raise ValueError("control status payload lacks an authoritative state")
    return parsed


def _odom_mapping(msg: Any) -> Optional[dict[str, Any]]:
    try:
        pose = msg.pose.pose
        twist = msg.twist.twist
        stamp = msg.header.stamp
        orientation = pose.orientation
        yaw_rad = math.atan2(
            2.0 * (float(orientation.w) * float(orientation.z) + float(orientation.x) * float(orientation.y)),
            1.0 - 2.0 * (float(orientation.y) ** 2 + float(orientation.z) ** 2),
        )
        values = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_rad,
            float(twist.linear.x),
            float(twist.angular.z),
        )
        if not all(math.isfinite(value) for value in values):
            return None
        return {
            "stamp_s": float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0,
            "frame_id": str(msg.header.frame_id),
            "child_frame_id": str(msg.child_frame_id),
            "x_m": values[0],
            "y_m": values[1],
            "yaw_rad": values[2],
            "heading_deg": math.degrees(values[2]),
            "linear_mps": values[3],
            "angular_rad_s": values[4],
        }
    except (AttributeError, TypeError, ValueError):
        return None


def _validated_execution_gate(
    *,
    enabled: bool,
    reviewed_sha: str,
    source_sha: str,
    deployed_sha: str,
    planning_enabled: bool,
) -> bool:
    """Require an explicit exact-SHA review before installing route authority."""

    if not bool(enabled):
        return False
    if not bool(planning_enabled):
        raise ValueError("live execution requires the prompt planner")
    reviewed = str(reviewed_sha).strip()
    source = str(source_sha).strip()
    deployed = str(deployed_sha).strip()
    if not reviewed or reviewed != source or reviewed != deployed:
        raise ValueError(
            "live execution reviewed SHA must exactly match the source and deployed SHAs"
        )
    return True


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class LiveMissionServiceNode(Node):
        def __init__(self):
            super().__init__("live_mission_service")
            self._declare_parameters()
            source_sha = self._required_provenance("source_sha", "RVR_SOURCE_SHA")
            deployed_sha = self._required_provenance("deployed_sha", "RVR_DEPLOYED_SHA")
            max_source_age_s = float(self.get_parameter("max_source_age_s").value)
            max_binding_age_s = float(self.get_parameter("max_binding_age_s").value)

            self._cache = LiveStateCache()
            self._status_executor = LiveStatusExecutor(
                self._cache,
                source_sha=source_sha,
                deployed_sha=deployed_sha,
                max_source_age_s=max_source_age_s,
            )
            self._route_executor = LiveRouteProgressExecutor(self._status_executor)
            now = time.time()
            bindings = live_executor_bindings(
                self._status_executor,
                self._route_executor,
                heartbeat_at_s=now,
                max_binding_age_s=max_binding_age_s,
            )
            database = Path(str(self.get_parameter("database_path").value)).expanduser()
            socket_path = Path(str(self.get_parameter("socket_path").value)).expanduser()
            registry = build_default_registry(detector_classes=("shoe", "backpack"))
            planning_enabled = bool(self.get_parameter("planning_enabled").value)
            stationary_perception_enabled = bool(
                self.get_parameter("stationary_perception_enabled").value
            )
            adaptive_mission_enabled = bool(
                self.get_parameter("adaptive_mission_enabled").value
            )
            approval_activation_enabled = bool(
                self.get_parameter("approval_activation_enabled").value
            )
            legacy_live_execution_enabled = bool(
                self.get_parameter("live_execution_enabled").value
            )
            if (
                approval_activation_enabled
                and legacy_live_execution_enabled
            ):
                raise ValueError(
                    "approval-time activation and legacy always-unlocked execution "
                    "cannot be enabled together"
                )
            reviewed_sha = str(
                self.get_parameter(
                    "approval_activation_reviewed_sha"
                    if approval_activation_enabled
                    else "live_execution_reviewed_sha"
                ).value
            )
            live_execution_enabled = _validated_execution_gate(
                enabled=(
                    approval_activation_enabled
                    or legacy_live_execution_enabled
                ),
                reviewed_sha=reviewed_sha,
                source_sha=source_sha,
                deployed_sha=deployed_sha,
                planning_enabled=planning_enabled,
            )
            if approval_activation_enabled and not adaptive_mission_enabled:
                raise ValueError(
                    "approval-time activation requires the Adaptive mission controller"
                )
            if stationary_perception_enabled and live_execution_enabled:
                raise ValueError(
                    "stationary perception cannot coexist with live execution authority"
                )
            if stationary_perception_enabled and adaptive_mission_enabled:
                raise ValueError(
                    "stationary perception and Adaptive mission cannot own the prompt "
                    "controller simultaneously"
                )
            model_id = str(self.get_parameter("planning_model").value).strip() or None
            reasoning_effort = str(self.get_parameter("planning_reasoning_effort").value)
            planning_limits = PromptDriveLimits(
                max_motion_calls=int(self.get_parameter("planning_max_motion_calls").value),
                max_translation_m=float(self.get_parameter("planning_max_translation_m").value),
                max_translation_per_call_m=float(
                    self.get_parameter("planning_max_translation_per_call_m").value
                ),
                max_abs_turn_deg=float(self.get_parameter("planning_max_abs_turn_deg").value),
                max_runtime_s=float(self.get_parameter("planning_max_runtime_s").value),
                linear_speed_mps=float(self.get_parameter("planning_linear_speed_mps").value),
                angular_speed_deg_s=float(
                    self.get_parameter("planning_angular_speed_deg_s").value
                ),
            )
            adaptive_limits = AdaptiveMissionLimits(
                mission_lease_s=float(
                    self.get_parameter("adaptive_mission_lease_s").value
                )
            )
            ros_route_executor = (
                RosLiveRouteExecutor(
                    request_topic=str(self.get_parameter("route_request_topic").value),
                    status_topic=str(self.get_parameter("route_status_topic").value),
                    cancel_service=str(self.get_parameter("route_cancel_service").value),
                    collision_state_topic=str(
                        self.get_parameter("collision_state_topic").value
                    ),
                    graph_timeout_s=float(self.get_parameter("route_graph_timeout_s").value),
                    cleanup_timeout_s=float(self.get_parameter("route_cleanup_timeout_s").value),
                )
                if live_execution_enabled
                else None
            )
            prompt_route_executor = (
                SafetyGatedPromptRouteExecutor(self._status_executor, ros_route_executor)
                if ros_route_executor is not None
                else None
            )
            adaptive_mission_executor = (
                PhysicalAdaptiveMissionExecutor(
                    self._cache,
                    source_sha=source_sha,
                    deployed_sha=deployed_sha,
                    reviewed_sha=reviewed_sha,
                    execution_enabled=live_execution_enabled,
                    transport=ros_route_executor,
                    limits=adaptive_limits,
                    max_source_age_s=min(max_source_age_s, 0.30),
                    cleanup_timeout_s=float(
                        self.get_parameter(
                            "route_cleanup_timeout_s"
                        ).value
                    ),
                )
                if adaptive_mission_enabled
                else None
            )
            adaptive_session = None
            if adaptive_mission_enabled:
                adaptive_session = SystemdAdaptiveMissionSession(
                    activation_capable=live_execution_enabled
                )
                adaptive_session.ensure_locked()

            def service_factory() -> MissionService:
                return MissionService(
                    database,
                    source_sha=source_sha,
                    deployed_sha=deployed_sha,
                    registry=registry,
                    adapters=self._status_executor,
                    mode="live",
                    executor_bindings=bindings,
                    live_execution_enabled=live_execution_enabled,
                    adaptive_mission_limits=adaptive_limits.to_json_dict(),
                )

            controller_factory = None
            if stationary_perception_enabled:
                def controller_factory(
                    service: MissionService,
                ) -> StationaryPerceptionController:
                    provider = CodexOAuthStationaryIntentProvider(
                        model=model_id,
                        reasoning_effort=reasoning_effort,
                    )
                    return StationaryPerceptionController(
                        service,
                        provider,
                        self._cache,
                        tick_s=float(
                            self.get_parameter("stationary_perception_tick_s").value
                        ),
                        max_source_age_s=float(
                            self.get_parameter(
                                "stationary_perception_max_source_age_s"
                            ).value
                        ),
                    )
            elif adaptive_mission_enabled:
                def controller_factory(
                    service: MissionService,
                ) -> LiveAdaptiveMissionController:
                    return LiveAdaptiveMissionController(
                        service,
                        CodexOAuthAdaptiveMissionIntentProvider(
                            model=model_id,
                            reasoning_effort=reasoning_effort,
                            limits=adaptive_limits,
                        ),
                        adaptive_mission_executor,  # type: ignore[arg-type]
                        execution_enabled=live_execution_enabled,
                        limits=adaptive_limits,
                        session_lifecycle=adaptive_session,
                        activation_timeout_s=float(
                            self.get_parameter(
                                "approval_activation_timeout_s"
                            ).value
                        ),
                    )
            elif planning_enabled:
                def controller_factory(service: MissionService) -> PromptMissionController:
                    provider = CodexOAuthPromptDriveProvider(
                        model=model_id,
                        reasoning_effort=reasoning_effort,
                    )
                    return PromptMissionController(
                        service,
                        PromptDrivePlanner(
                            provider,
                            limits=planning_limits,
                            source_sha=source_sha,
                        ),
                        route_executor=prompt_route_executor,
                        execution_enabled=live_execution_enabled,
                        approval_ttl_s=float(self.get_parameter("approval_ttl_s").value),
                    )

            self._server = MissionServiceServer(
                socket_path,
                service_factory,
                prompt_controller_factory=controller_factory,
            )
            self._server_thread = threading.Thread(
                target=self._server.serve_forever,
                name="live-mission-service-socket",
                daemon=True,
            )
            self._server_thread.start()
            self._status_pub = self.create_publisher(
                String,
                str(self.get_parameter("capability_status_topic").value),
                10,
            )
            self.create_subscription(
                Odometry,
                str(self.get_parameter("odom_topic").value),
                self._on_odom,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("collision_state_topic").value),
                self._on_collision,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("control_state_topic").value),
                self._on_control,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("route_status_topic").value),
                self._on_route,
                10,
            )
            for source_name, parameter_name in (
                ("camera", "camera_status_topic"),
                ("lidar", "lidar_status_topic"),
                ("localization", "localization_status_topic"),
                ("semantic_map", "semantic_map_status_topic"),
            ):
                self.create_subscription(
                    String,
                    str(self.get_parameter(parameter_name).value),
                    lambda msg, name=source_name: self._on_json_source(name, msg),
                    10,
                )
            self.create_service(
                Trigger,
                str(self.get_parameter("status_service").value),
                self._on_status,
            )
            self.create_timer(
                float(self.get_parameter("publish_period_s").value),
                self._heartbeat_and_publish,
            )
            self._heartbeat_and_publish()

        def _declare_parameters(self) -> None:
            self.declare_parameter("database_path", "~/.local/state/sphero_rvr/missions.sqlite3")
            self.declare_parameter("socket_path", "~/.local/state/sphero_rvr/mission-service.sock")
            self.declare_parameter("source_sha", "")
            self.declare_parameter("deployed_sha", "")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("collision_state_topic", "/collision_stop/state")
            self.declare_parameter("control_state_topic", "/mission_api/v2/control_state")
            self.declare_parameter("route_request_topic", "/mission_api/v2/live_route/request")
            self.declare_parameter("route_status_topic", "/mission_api/v2/live_route/status")
            self.declare_parameter("route_cancel_service", "live_route/cancel")
            self.declare_parameter("camera_status_topic", "/mission_api/v2/camera/status")
            self.declare_parameter("lidar_status_topic", "/mission_api/v2/lidar/status")
            self.declare_parameter("localization_status_topic", "/mission_api/v2/localization/status")
            self.declare_parameter("semantic_map_status_topic", "/mission_api/v2/map/status")
            self.declare_parameter("capability_status_topic", "/mission_api/v2/service/status")
            self.declare_parameter("status_service", "/mission_api/v2/service/read_status")
            self.declare_parameter("publish_period_s", 0.5)
            self.declare_parameter("max_source_age_s", 1.0)
            self.declare_parameter("max_binding_age_s", 2.0)
            self.declare_parameter("planning_enabled", True)
            self.declare_parameter("stationary_perception_enabled", False)
            self.declare_parameter("adaptive_mission_enabled", False)
            self.declare_parameter("stationary_perception_tick_s", 0.2)
            self.declare_parameter("stationary_perception_max_source_age_s", 1.5)
            self.declare_parameter("planning_model", "gpt-5.6-sol")
            self.declare_parameter("planning_reasoning_effort", "low")
            self.declare_parameter("planning_max_motion_calls", 3)
            self.declare_parameter("planning_max_translation_m", 0.5)
            self.declare_parameter("planning_max_translation_per_call_m", 0.5)
            self.declare_parameter("planning_max_abs_turn_deg", 180.0)
            self.declare_parameter("planning_max_runtime_s", 45.0)
            self.declare_parameter("planning_linear_speed_mps", 0.08)
            self.declare_parameter("planning_angular_speed_deg_s", 30.0)
            self.declare_parameter("live_execution_enabled", False)
            self.declare_parameter("live_execution_reviewed_sha", "")
            self.declare_parameter("approval_activation_enabled", False)
            self.declare_parameter("approval_activation_reviewed_sha", "")
            self.declare_parameter("approval_activation_timeout_s", 30.0)
            self.declare_parameter("adaptive_mission_lease_s", 900.0)
            self.declare_parameter("approval_ttl_s", 60.0)
            self.declare_parameter("route_graph_timeout_s", 5.0)
            self.declare_parameter("route_cleanup_timeout_s", 3.0)

        def _required_provenance(self, parameter: str, environment: str) -> str:
            value = str(self.get_parameter(parameter).value).strip() or os.environ.get(environment, "").strip()
            if not value or value.lower() == "unknown":
                raise ValueError(
                    f"{parameter} must be injected from reviewed deployment provenance"
                )
            return value

        def _on_odom(self, msg) -> None:
            value = _odom_mapping(msg)
            now = time.time()
            if value is None:
                self._cache.mark_invalid("odom", "malformed or non-finite odometry", received_at_s=now)
                return
            self._cache.update(
                "odom",
                value,
                received_at_s=now,
                source_timestamp_s=value.get("stamp_s"),
            )

        def _on_collision(self, msg) -> None:
            now = time.time()
            try:
                value = _collision_mapping(getattr(msg, "data", ""))
                self._cache.update("collision", value, received_at_s=now)
            except ValueError as exc:
                self._cache.mark_invalid("collision", str(exc), received_at_s=now)

        def _on_route(self, msg) -> None:
            self._on_json_source("route_progress", msg)

        def _on_control(self, msg) -> None:
            now = time.time()
            try:
                value = _control_mapping(getattr(msg, "data", ""))
                self._cache.update("control", value, received_at_s=now)
            except ValueError as exc:
                self._cache.mark_invalid("control", str(exc), received_at_s=now)

        def _on_json_source(self, name: str, msg) -> None:
            now = time.time()
            try:
                value = (
                    _localization_mapping(getattr(msg, "data", ""))
                    if name == "localization"
                    else _json_mapping(getattr(msg, "data", ""))
                )
                source_timestamp = value.get("stamp_s")
                if name == "localization" and source_timestamp is None:
                    localization = value.get("localization", value)
                    if isinstance(localization, Mapping):
                        pose = localization.get("pose")
                        if isinstance(pose, Mapping):
                            source_timestamp = pose.get("stamp_s")
                self._cache.update(
                    name,
                    value,
                    received_at_s=now,
                    source_timestamp_s=source_timestamp,
                )
            except (TypeError, ValueError) as exc:
                self._cache.mark_invalid(name, str(exc), received_at_s=now)

        def _heartbeat_and_publish(self) -> None:
            now = time.time()
            service = self._server.service
            service.heartbeat_executor(
                "query_status_telemetry",
                evidence=self._status_executor.evidence(now_s=now),
            )
            route_evidence = self._route_executor.evidence(now_s=now)
            service.heartbeat_executor("move_distance", evidence=route_evidence)
            service.heartbeat_executor("turn_angle", evidence=route_evidence)
            message = String()
            message.data = json.dumps(self._status_payload(now_s=now), sort_keys=True)
            self._status_pub.publish(message)

        def _status_payload(self, *, now_s: Optional[float] = None) -> dict[str, Any]:
            return {
                "api_version": "mission_api.v2",
                "node": self.get_name(),
                "mode": "live",
                "motion_authority": self._server.service.live_execution_enabled,
                "status": self._status_executor.evidence(now_s=now_s),
                "capabilities": self._server.service.capabilities(),
            }

        def _on_status(self, request, response):
            del request
            response.success = True
            response.message = json.dumps(self._status_payload(), sort_keys=True)
            return response

        def destroy_node(self):
            self._server.shutdown()
            self._server.server_close()
            self._server_thread.join(timeout=2.0)
            super().destroy_node()

    rclpy.init(args=args)
    node = LiveMissionServiceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
