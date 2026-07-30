"""ROS 2 node for the live Mission API v2 odometry route runner.

The node subscribes to live state, accepts typed bounded route JSON, and publishes
only the supervisor input topic. It never opens serial transport or a direct
motor driver command surface.
"""

from __future__ import annotations

import json
import math
import subprocess
from typing import Any, Optional

from .collision_stop import (
    CollisionStopConfig,
    CollisionState,
    ScanInput,
    ScanStampTracker,
    Transform2D,
)
from .live_route_runner import (
    LiveRouteConfig,
    LiveRouteRequest,
    LiveRouteRunner,
    LiveRouteState,
    TrackEncoderState,
    _normalize_exception_terminal,
    route_request_from_json,
)
from .hierarchical_exploration import (
    HierarchicalBridgeConfig,
    HierarchicalCommandBridge,
    PRIVATE_NAV2_CMD_TOPIC,
)
from .hierarchical_physical_binding import (
    AUTHORITY_HEARTBEAT_MAX_AGE_S,
    AUTHORITY_TOPIC,
    validate_authority_heartbeat,
)
from .odometry import MotionPrimitiveConfig, OdomMotionState
from .range_motion_node import _stamp_seconds, _tf_error_reason, _transform2d_from_transform_stamped

MAX_TURN_CORRECTION_CONTROL_PERIOD_S = 0.05


def _odom_state(msg: Any) -> Optional[OdomMotionState]:
    header = getattr(msg, "header", None)
    stamp = _stamp_seconds(getattr(header, "stamp", None))
    try:
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        qx = float(getattr(orientation, "x", 0.0))
        qy = float(getattr(orientation, "y", 0.0))
        qz = float(getattr(orientation, "z", 0.0))
        qw = float(getattr(orientation, "w", 1.0))
        import math

        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return OdomMotionState(
            stamp=float(stamp) if stamp is not None else 0.0,
            x_m=float(position.x),
            y_m=float(position.y),
            yaw_rad=yaw,
        )
    except Exception:
        return None


def _encoder_state(raw: Any) -> Optional[TrackEncoderState]:
    try:
        payload = json.loads(str(raw))
        if not isinstance(payload, dict) or payload.get("schema") != "sphero_rvr.encoder_counts.v1":
            return None
        stamp = float(payload["stamp"])
        counts_per_meter = float(payload["counts_per_meter"])
        left = payload["left_count"]
        right = payload["right_count"]
        if not math.isfinite(stamp) or stamp < 0.0:
            return None
        if not math.isfinite(counts_per_meter) or counts_per_meter <= 0.0:
            return None
        if isinstance(left, bool) or isinstance(right, bool):
            return None
        if not isinstance(left, int) or not isinstance(right, int):
            return None
        return TrackEncoderState(stamp, left, right, counts_per_meter)
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def _collision_state_value(raw: Any) -> Optional[str]:
    """Parse the supervisor's state token from JSON or its diagnostic text."""

    state = raw
    try:
        payload = json.loads(str(raw))
        if isinstance(payload, dict):
            state = payload.get("state", payload.get("collision_state"))
        else:
            state = payload
    except Exception:
        pass
    tokens = str(state).strip().split(maxsplit=1) if state is not None else []
    token = tokens[0] if tokens else ""
    try:
        return CollisionState(token.upper()).value
    except Exception:
        return None


def _collision_reason_value(raw: Any) -> str:
    """Parse the supervisor reason without weakening its state token."""

    try:
        payload = json.loads(str(raw))
        if isinstance(payload, dict):
            return str(payload.get("reason", "")).strip().lower()
    except Exception:
        pass
    for token in str(raw).strip().split():
        key, separator, value = token.partition("=")
        if separator and key.strip().lower() == "reason":
            return value.strip().lower()
    return ""


def _source_sha() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return "unknown"


def main(args=None):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    class LiveRouteRunnerNode(Node):
        def __init__(self):
            super().__init__("live_route_runner")
            self._declare_parameters()
            control_period_s = float(self.get_parameter("control_period_s").value)
            if (
                not math.isfinite(control_period_s)
                or control_period_s <= 0.0
                or control_period_s > MAX_TURN_CORRECTION_CONTROL_PERIOD_S
            ):
                raise ValueError(
                    "live route control_period_s must be positive and no greater than 0.05"
                )
            self._runner = LiveRouteRunner(self._read_config())
            self._hierarchical_mode = bool(
                self.get_parameter("hierarchical_mode_enabled").value
            )
            self._hierarchical_physical = bool(
                self.get_parameter(
                    "hierarchical_physical_binding_enabled"
                ).value
            )
            use_sim_time = bool(self.get_parameter("use_sim_time").value)
            if self._hierarchical_physical and not self._hierarchical_mode:
                raise ValueError(
                    "physical hierarchical binding requires hierarchical mode"
                )
            if self._hierarchical_mode and not self._hierarchical_physical and not use_sim_time:
                raise ValueError(
                    "Phase 1 hierarchical mode is replay-only and requires use_sim_time"
                )
            if self._hierarchical_physical and use_sim_time:
                raise ValueError(
                    "physical hierarchical binding requires live time"
                )
            self._deployed_sha = str(
                self.get_parameter("deployed_sha").value
            ).strip()
            self._reviewed_sha = str(
                self.get_parameter(
                    "hierarchical_physical_reviewed_sha"
                ).value
            ).strip()
            if self._hierarchical_physical and not (
                self._source_sha_value()
                == self._deployed_sha
                == self._reviewed_sha
            ):
                raise ValueError(
                    "physical hierarchical binding requires matching exact source, deployed, and reviewed SHAs"
                )
            self._hierarchical_bridge = HierarchicalCommandBridge(
                HierarchicalBridgeConfig(
                    enabled=self._hierarchical_mode,
                    command_lease_s=float(
                        self.get_parameter("nav2_cmd_lease_s").value
                    ),
                    max_linear_mps=float(
                        self.get_parameter("hierarchical_max_linear_mps").value
                    ),
                    max_angular_rad_s=float(
                        self.get_parameter("hierarchical_max_angular_rad_s").value
                    ),
                    clear_breakaway_linear_mps=float(
                        self.get_parameter(
                            "hierarchical_clear_breakaway_linear_mps"
                        ).value
                    ),
                    clear_breakaway_angular_rad_s=float(
                        self.get_parameter(
                            "hierarchical_clear_breakaway_angular_rad_s"
                        ).value
                    ),
                    reverse_escape_linear_mps=float(
                        self.get_parameter(
                            "hierarchical_reverse_escape_linear_mps"
                        ).value
                    ),
                )
            )
            self._latest_nav2_command_received_at: Optional[float] = None
            self._hierarchical_authority: Optional[dict[str, Any]] = None
            self._hierarchical_authority_received_at: Optional[float] = None
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._latest_request: Optional[LiveRouteRequest] = None
            self._latest_scan: Optional[ScanInput] = None
            self._scan_stamp_tracker = ScanStampTracker()
            self._latest_odom: Optional[OdomMotionState] = None
            self._latest_odom_received_at: Optional[float] = None
            self._latest_encoder_counts: Optional[TrackEncoderState] = None
            self._collision_state: Optional[str] = None
            self._collision_reason = ""
            self._collision_received_at: Optional[float] = None
            self._stop = False
            self._estop = False
            self._cancel = False
            self._source_sha = str(self.get_parameter("source_sha").value) or _source_sha()

            self._cmd_pub = self.create_publisher(Twist, self._supervisor_cmd_topic(), 10)
            self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
            self._diagnostics_pub = self.create_publisher(DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10)
            if self._hierarchical_mode:
                self.create_subscription(
                    Twist,
                    self._private_nav2_cmd_topic(),
                    self._on_nav2_command,
                    10,
                )
                if self._hierarchical_physical:
                    self.create_subscription(
                        String,
                        self._hierarchical_authority_topic(),
                        self._on_hierarchical_authority,
                        10,
                    )
            else:
                self.create_subscription(String, str(self.get_parameter("route_request_topic").value), self._on_route_request, 10)
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self._on_scan,
                qos_profile_sensor_data,
            )
            self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10)
            self.create_subscription(String, str(self.get_parameter("encoder_counts_topic").value), self._on_encoder_counts, 10)
            self.create_subscription(String, str(self.get_parameter("collision_state_topic").value), self._on_collision_state, 10)
            self.create_subscription(String, str(self.get_parameter("stop_state_topic").value), self._on_stop_state, 10)
            self.create_service(Trigger, "live_route/cancel", self._on_cancel)
            self.create_timer(control_period_s, self._tick)

        def _declare_parameters(self) -> None:
            odom_defaults = MotionPrimitiveConfig()
            scan_defaults = CollisionStopConfig()
            for name, value in {
                "route_request_topic": "/mission_api/v2/live_route/request",
                "status_topic": "/mission_api/v2/live_route/status",
                "diagnostics_topic": "/diagnostics",
                "cmd_vel_topic": "/cmd_vel",
                "hierarchical_mode_enabled": False,
                "hierarchical_physical_binding_enabled": False,
                "hierarchical_authority_topic": AUTHORITY_TOPIC,
                "hierarchical_authority_max_age_s": (
                    AUTHORITY_HEARTBEAT_MAX_AGE_S
                ),
                "hierarchical_physical_reviewed_sha": "",
                "deployed_sha": "",
                "nav2_cmd_topic": PRIVATE_NAV2_CMD_TOPIC,
                "nav2_cmd_lease_s": 0.25,
                "hierarchical_max_linear_mps": 0.10,
                "hierarchical_max_angular_rad_s": 0.4,
                "hierarchical_clear_breakaway_linear_mps": 0.0,
                "hierarchical_clear_breakaway_angular_rad_s": 0.0,
                "hierarchical_reverse_escape_linear_mps": 0.0,
                "scan_topic": "/scan",
                "odom_topic": "/odom",
                "encoder_counts_topic": "/encoder_counts",
                "collision_state_topic": "/collision_stop/state",
                "stop_state_topic": "/mission_api/v2/control_state",
                "control_period_s": 0.05,
                "source_sha": "",
                "base_frame": scan_defaults.base_frame,
                "laser_frame": scan_defaults.laser_frame,
                "fail_on_missing_tf": scan_defaults.fail_on_missing_tf,
                "tf_timeout_s": scan_defaults.tf_timeout_s,
                "min_valid_ranges": scan_defaults.min_valid_ranges,
                "min_valid_fraction": scan_defaults.min_valid_fraction,
                "min_range_m": scan_defaults.min_range_m,
                "max_range_m": scan_defaults.max_range_m,
                "sector_unknown_policy": scan_defaults.sector_unknown_policy,
                "clearance_margin_m": 0.40,
                "min_translation_cap_m": 0.01,
                "max_translation_segment_m": 0.75,
                "collision_state_max_age_s": scan_defaults.max_scan_age_s,
                "track_counts_per_meter": 4337.768,
                "terminal_settle_time_s": 0.50,
                "terminal_settle_timeout_s": 2.0,
                "terminal_settle_distance_m": 0.005,
                "terminal_settle_angle_rad": math.radians(1.0),
                "terminal_settle_encoder_counts": 8,
                "max_terminal_distance_error_m": 0.03,
                "max_terminal_angle_error_rad": math.radians(10.0),
                "max_turn_corrections": 3,
                "distance_tolerance_m": odom_defaults.distance_tolerance_m,
                "angle_tolerance_rad": odom_defaults.angle_tolerance_rad,
                "target_stop_horizon_s": odom_defaults.target_stop_horizon_s,
                "turn_target_stop_horizon_s": odom_defaults.turn_target_stop_horizon_s,
                "max_turn_speed_rad_s": odom_defaults.max_turn_speed_rad_s,
                "max_turn_progress_rate_rad_s": odom_defaults.max_turn_progress_rate_rad_s,
                "heading_kp": odom_defaults.heading_kp,
                "max_heading_correction_rad_s": odom_defaults.max_heading_correction_rad_s,
                "max_sample_age_s": odom_defaults.max_sample_age_s,
                "stall_timeout_s": odom_defaults.stall_timeout_s,
                "startup_grace_s": odom_defaults.startup_grace_s,
                "min_progress_m": odom_defaults.min_progress_m,
                "min_angle_progress_rad": odom_defaults.min_angle_progress_rad,
            }.items():
                self.declare_parameter(name, value)

        def _read_config(self) -> LiveRouteConfig:
            odom = MotionPrimitiveConfig(
                distance_tolerance_m=float(self.get_parameter("distance_tolerance_m").value),
                angle_tolerance_rad=float(self.get_parameter("angle_tolerance_rad").value),
                target_stop_horizon_s=float(
                    self.get_parameter("target_stop_horizon_s").value
                ),
                turn_target_stop_horizon_s=float(
                    self.get_parameter("turn_target_stop_horizon_s").value
                ),
                max_turn_speed_rad_s=float(self.get_parameter("max_turn_speed_rad_s").value),
                max_turn_progress_rate_rad_s=float(
                    self.get_parameter("max_turn_progress_rate_rad_s").value
                ),
                heading_kp=float(self.get_parameter("heading_kp").value),
                max_heading_correction_rad_s=float(self.get_parameter("max_heading_correction_rad_s").value),
                max_sample_age_s=float(self.get_parameter("max_sample_age_s").value),
                stall_timeout_s=float(self.get_parameter("stall_timeout_s").value),
                startup_grace_s=float(self.get_parameter("startup_grace_s").value),
                min_progress_m=float(self.get_parameter("min_progress_m").value),
                min_angle_progress_rad=float(self.get_parameter("min_angle_progress_rad").value),
            )
            scan = CollisionStopConfig(
                base_frame=str(self.get_parameter("base_frame").value),
                laser_frame=str(self.get_parameter("laser_frame").value),
                fail_on_missing_tf=bool(self.get_parameter("fail_on_missing_tf").value),
                tf_timeout_s=float(self.get_parameter("tf_timeout_s").value),
                min_valid_ranges=int(self.get_parameter("min_valid_ranges").value),
                min_valid_fraction=float(self.get_parameter("min_valid_fraction").value),
                min_range_m=float(self.get_parameter("min_range_m").value),
                max_range_m=float(self.get_parameter("max_range_m").value),
                sector_unknown_policy=str(self.get_parameter("sector_unknown_policy").value),
            )
            return LiveRouteConfig(
                odom=odom,
                scan=scan,
                clearance_margin_m=float(self.get_parameter("clearance_margin_m").value),
                min_translation_cap_m=float(self.get_parameter("min_translation_cap_m").value),
                max_translation_segment_m=float(self.get_parameter("max_translation_segment_m").value),
                collision_state_max_age_s=float(self.get_parameter("collision_state_max_age_s").value),
                track_counts_per_meter=float(self.get_parameter("track_counts_per_meter").value),
                terminal_settle_time_s=float(self.get_parameter("terminal_settle_time_s").value),
                terminal_settle_timeout_s=float(self.get_parameter("terminal_settle_timeout_s").value),
                terminal_settle_distance_m=float(self.get_parameter("terminal_settle_distance_m").value),
                terminal_settle_angle_rad=float(self.get_parameter("terminal_settle_angle_rad").value),
                terminal_settle_encoder_counts=int(self.get_parameter("terminal_settle_encoder_counts").value),
                max_terminal_distance_error_m=float(self.get_parameter("max_terminal_distance_error_m").value),
                max_terminal_angle_error_rad=float(self.get_parameter("max_terminal_angle_error_rad").value),
                max_turn_corrections=int(self.get_parameter("max_turn_corrections").value),
            )

        def _supervisor_cmd_topic(self) -> str:
            topic = str(self.get_parameter("cmd_vel_topic").value)
            if topic != "/cmd_vel":
                raise ValueError("live_route_runner cmd_vel_topic must remain /cmd_vel")
            return "/cmd_vel"

        def _private_nav2_cmd_topic(self) -> str:
            topic = str(self.get_parameter("nav2_cmd_topic").value)
            if topic != PRIVATE_NAV2_CMD_TOPIC:
                raise ValueError(
                    f"hierarchical Nav2 command topic must remain {PRIVATE_NAV2_CMD_TOPIC}"
                )
            return PRIVATE_NAV2_CMD_TOPIC

        def _source_sha_value(self) -> str:
            return str(self.get_parameter("source_sha").value).strip() or _source_sha()

        def _hierarchical_authority_topic(self) -> str:
            topic = str(
                self.get_parameter("hierarchical_authority_topic").value
            )
            if topic != AUTHORITY_TOPIC:
                raise ValueError(
                    f"hierarchical authority topic must remain {AUTHORITY_TOPIC}"
                )
            return AUTHORITY_TOPIC

        def _on_nav2_command(self, msg) -> None:
            now_s = self._now_seconds()
            self._latest_nav2_command_received_at = now_s
            self._hierarchical_bridge.accept(
                float(msg.linear.x),
                float(msg.angular.z),
                received_at_s=now_s,
            )

        def _on_hierarchical_authority(self, msg) -> None:
            now_s = self._now_seconds()
            try:
                payload = json.loads(str(msg.data))
                if not isinstance(payload, dict):
                    raise ValueError("authority payload must be an object")
            except (TypeError, ValueError, json.JSONDecodeError):
                self._hierarchical_authority = None
                self._hierarchical_authority_received_at = None
                return
            self._hierarchical_authority = payload
            self._hierarchical_authority_received_at = now_s

        def _on_route_request(self, msg) -> None:
            try:
                request = route_request_from_json(str(msg.data), source_sha=self._source_sha)
            except Exception as exc:
                self._publish_zero_status("invalid_route", {"message": str(exc)})
                return
            self._cancel = False
            self._latest_request = request
            try:
                command = self._runner.start(request, self._current_state())
            except Exception as exc:
                reason = _normalize_exception_terminal(exc)
                self._runner.abort(reason, self._current_state())
                self._publish_zero_status(reason, {"message": str(exc)})
                self._publish_manifest()
                return
            self._publish_command(command)
            if not self._runner.active:
                self._publish_manifest()

        def _on_scan(self, msg) -> None:
            header = getattr(msg, "header", None)
            stamp = _stamp_seconds(getattr(header, "stamp", None))
            frame_id = str(getattr(header, "frame_id", "")) or str(self.get_parameter("laser_frame").value)
            transform, transform_error = self._lookup_scan_transform(frame_id, getattr(header, "stamp", None))
            self._latest_scan = ScanInput(
                ranges=tuple(getattr(msg, "ranges", []) or []),
                angle_min=float(msg.angle_min),
                angle_increment=float(msg.angle_increment),
                range_min=float(msg.range_min),
                range_max=float(msg.range_max),
                stamp=stamp,
                received_at=self._now_seconds(),
                frame_id=frame_id,
                transform_to_base=transform,
                transform_error=transform_error,
                stamp_progress_error=self._scan_stamp_tracker.check(stamp),
            )

        def _lookup_scan_transform(self, frame_id: str, stamp_msg) -> tuple[Optional[Transform2D], Optional[str]]:
            base_frame = str(self.get_parameter("base_frame").value)
            if frame_id == base_frame:
                return Transform2D(), None
            try:
                time = Time.from_msg(stamp_msg) if stamp_msg is not None else Time()
                stamped = self._tf_buffer.lookup_transform(
                    base_frame,
                    frame_id or str(self.get_parameter("laser_frame").value),
                    time,
                    timeout=Duration(seconds=float(self.get_parameter("tf_timeout_s").value)),
                )
                return _transform2d_from_transform_stamped(stamped), None
            except Exception as exc:
                reason = _tf_error_reason(exc)
                self.get_logger().warn(f"live route TF lookup failed: {reason}: {exc}")
                return None, reason

        def _on_odom(self, msg) -> None:
            self._latest_odom = _odom_state(msg)
            self._latest_odom_received_at = self._now_seconds()

        def _on_encoder_counts(self, msg) -> None:
            self._latest_encoder_counts = _encoder_state(getattr(msg, "data", None))

        def _on_collision_state(self, msg) -> None:
            raw = getattr(msg, "data", None)
            self._collision_state = _collision_state_value(raw)
            self._collision_reason = _collision_reason_value(raw)
            if self._collision_state is not None:
                self._collision_received_at = self._now_seconds()
            else:
                self._collision_received_at = None

        def _on_stop_state(self, msg) -> None:
            text = str(getattr(msg, "data", "")).lower()
            self._stop = "stop" in text and "estop" not in text
            self._estop = "estop" in text
            self._cancel = "cancel" in text

        def _on_cancel(self, request, response):
            self._cancel = True
            if self._hierarchical_mode:
                self._publish_command(
                    type("Zero", (), {"linear_x": 0.0, "angular_z": 0.0})()
                )
                response.success = True
                response.message = (
                    "hierarchical replay cancel requested; zero command published"
                )
                return response
            try:
                command = self._runner.update(self._current_state())
            except Exception as exc:
                reason = _normalize_exception_terminal(exc)
                self._runner.abort(reason, self._current_state())
                self._publish_zero_status(reason, {"message": str(exc)})
                self._publish_manifest()
                command = type("Zero", (), {"linear_x": 0.0, "angular_z": 0.0})()
            self._publish_command(command)
            self._publish_manifest()
            response.success = True
            response.message = "live route cancel requested; zero command published"
            return response

        def _tick(self) -> None:
            if self._hierarchical_mode:
                now_s = self._now_seconds()
                authority_valid = not self._hierarchical_physical
                if self._hierarchical_physical:
                    if (
                        self._hierarchical_authority is not None
                        and self._hierarchical_authority_received_at is not None
                    ):
                        authority_valid, _ = validate_authority_heartbeat(
                            self._hierarchical_authority,
                            now_s=now_s,
                            received_at_s=(
                                self._hierarchical_authority_received_at
                            ),
                            source_sha=self._source_sha_value(),
                            deployed_sha=self._deployed_sha,
                            reviewed_sha=self._reviewed_sha,
                            max_age_s=float(
                                self.get_parameter(
                                    "hierarchical_authority_max_age_s"
                                ).value
                            ),
                        )
                evidence_fresh = bool(
                    self._latest_odom is not None
                    and self._latest_odom_received_at is not None
                    and now_s >= self._latest_odom_received_at
                    and now_s - self._latest_odom_received_at
                    <= float(self.get_parameter("max_sample_age_s").value)
                    and self._collision_received_at is not None
                    and now_s >= self._collision_received_at
                    and now_s - self._collision_received_at
                    <= float(
                        self.get_parameter("collision_state_max_age_s").value
                    )
                )
                decision = self._hierarchical_bridge.evaluate(
                    now_s=now_s,
                    goal_active=(
                        self._latest_nav2_command_received_at is not None
                        and authority_valid
                    ),
                    mission_lease_valid=authority_valid,
                    motion_evidence_fresh=evidence_fresh,
                    collision_state=self._collision_state or "UNKNOWN",
                    collision_reason=self._collision_reason,
                    stop=self._stop,
                    estop=self._estop,
                    cancelled=self._cancel,
                )
                self._publish_command(
                    type(
                        "HierarchicalCommand",
                        (),
                        {
                            "linear_x": decision.bridged_linear_mps,
                            "angular_z": decision.bridged_angular_rad_s,
                        },
                    )()
                )
                return
            if not self._runner.active:
                return
            try:
                command = self._runner.update(self._current_state())
            except Exception as exc:
                reason = _normalize_exception_terminal(exc)
                self._runner.abort(reason, self._current_state())
                self._publish_zero_status(reason, {"message": str(exc)})
                self._publish_manifest()
                return
            self._publish_command(command)
            if not self._runner.active:
                self._publish_manifest()

        def _current_state(self) -> LiveRouteState:
            return LiveRouteState(
                stamp=self._now_seconds(),
                odom=self._latest_odom,
                scan=self._latest_scan,
                collision_state=self._collision_state,
                collision_received_at=self._collision_received_at,
                stop=self._stop,
                estop=self._estop,
                cancel=self._cancel,
                encoder_counts=self._latest_encoder_counts,
            )

        def _publish_command(self, command) -> None:
            twist = Twist()
            try:
                linear_x = float(command.linear_x)
                angular_z = float(command.angular_z)
            except (TypeError, ValueError):
                linear_x = 0.0
                angular_z = 0.0
            if not math.isfinite(linear_x) or not math.isfinite(angular_z):
                self.get_logger().error("live route runner rejected non-finite command; publishing zero")
                linear_x = 0.0
                angular_z = 0.0
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self._cmd_pub.publish(twist)

        def _publish_zero_status(self, reason: str, error: dict[str, Any]) -> None:
            self._publish_command(type("Zero", (), {"linear_x": 0.0, "angular_z": 0.0})())
            msg = String()
            msg.data = json.dumps({"status": "failed", "terminal_reason": reason, "error": error}, sort_keys=True)
            self._status_pub.publish(msg)

        def _publish_manifest(self) -> None:
            try:
                manifest = self._runner.manifest()
                msg = String()
                msg.data = manifest.to_json()
                self._status_pub.publish(msg)
                self._publish_diagnostics(manifest)
            except Exception as exc:
                self.get_logger().error(f"failed to publish live route manifest: {exc}")

        def _publish_diagnostics(self, manifest) -> None:
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "live_route_runner"
            status.hardware_id = "sphero_rvr_live_route_runner"
            status.level = DiagnosticStatus.OK if manifest.status.value in {"complete", "running"} else DiagnosticStatus.WARN
            status.message = f"live route: {manifest.terminal_reason}"
            status.values = [
                KeyValue(key="route_id", value=manifest.route_id),
                KeyValue(key="status", value=manifest.status.value),
                KeyValue(key="terminal_reason", value=manifest.terminal_reason),
                KeyValue(key="measured_distance_m", value=f"{manifest.measured_distance_m:.3f}"),
                KeyValue(key="measured_angle_deg", value=f"{manifest.measured_angle_deg:.3f}"),
                KeyValue(key="final_heading_deg", value=str(manifest.final_heading_deg)),
                KeyValue(key="encoder_start_stamp", value=str(manifest.encoder_start_stamp)),
                KeyValue(key="encoder_final_stamp", value=str(manifest.encoder_final_stamp)),
                KeyValue(key="left_encoder_delta_counts", value=str(manifest.left_encoder_delta_counts)),
                KeyValue(key="right_encoder_delta_counts", value=str(manifest.right_encoder_delta_counts)),
                KeyValue(key="left_track_distance_m", value=str(manifest.left_track_distance_m)),
                KeyValue(key="right_track_distance_m", value=str(manifest.right_track_distance_m)),
                KeyValue(key="terminal_settled", value=str(manifest.terminal_settled).lower()),
                KeyValue(key="terminal_settle_duration_s", value=str(manifest.terminal_settle_duration_s)),
                KeyValue(key="collision_state", value=manifest.collision_state),
                KeyValue(key="source_sha", value=manifest.source_sha),
            ]
            array.status = [status]
            self._diagnostics_pub.publish(array)

        def _now_seconds(self) -> float:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0

    rclpy.init(args=args)
    node = LiveRouteRunnerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
