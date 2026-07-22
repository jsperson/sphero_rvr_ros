"""Production ROS 2 owner for the persistent no-motion Mission Service seam.

This node observes existing safety-path topics. It does not publish Twist, route
requests, sensor commands, or any hardware command surface.
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
    live_executor_bindings,
)
from .mission_api import build_default_registry
from .mission_service import MissionService, MissionServiceServer
from .prompt_drive import CodexOAuthPromptDriveProvider, PromptDrivePlanner
from .prompt_mission_controller import PromptMissionController


def _json_mapping(value: Any) -> dict[str, Any]:
    try:
        parsed = json.loads(str(value))
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError("status payload must be a JSON object") from exc
    if not isinstance(parsed, Mapping):
        raise ValueError("status payload must be a JSON object")
    return dict(parsed)


def _collision_mapping(value: Any) -> dict[str, Any]:
    raw = str(value).strip()
    if not raw:
        raise ValueError("collision status payload is empty")
    try:
        return _json_mapping(raw)
    except ValueError:
        return {"state": raw.split(maxsplit=1)[0].upper(), "raw": raw}


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
            model_id = str(self.get_parameter("planning_model").value).strip() or None
            reasoning_effort = str(self.get_parameter("planning_reasoning_effort").value)

            def service_factory() -> MissionService:
                return MissionService(
                    database,
                    source_sha=source_sha,
                    deployed_sha=deployed_sha,
                    registry=registry,
                    adapters=self._status_executor,
                    mode="live",
                    executor_bindings=bindings,
                    live_execution_enabled=False,
                )

            controller_factory = None
            if planning_enabled:
                def controller_factory(service: MissionService) -> PromptMissionController:
                    provider = CodexOAuthPromptDriveProvider(
                        model=model_id,
                        reasoning_effort=reasoning_effort,
                    )
                    return PromptMissionController(
                        service,
                        PromptDrivePlanner(provider, source_sha=source_sha),
                        execution_enabled=False,
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
            self.declare_parameter("route_status_topic", "/mission_api/v2/live_route/status")
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
            self.declare_parameter("planning_model", "gpt-5.6-sol")
            self.declare_parameter("planning_reasoning_effort", "high")

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

        def _on_json_source(self, name: str, msg) -> None:
            now = time.time()
            try:
                value = _json_mapping(getattr(msg, "data", ""))
                self._cache.update(
                    name,
                    value,
                    received_at_s=now,
                    source_timestamp_s=value.get("stamp_s"),
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
                "motion_authority": False,
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
