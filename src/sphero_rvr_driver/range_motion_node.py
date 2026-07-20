"""ROS 2 seam for the closed-loop lidar range-motion controller.

Goals are accepted either as JSON on a String topic or via a Trigger start
service that reads goal parameters.  The node publishes only to the public
supervisor input (`/cmd_vel` by default), never to `/cmd_vel_motor`; the
independent collision-stop supervisor remains the final motor gate.
"""

from __future__ import annotations

import json
import math
from typing import Any, Mapping, Optional

from .collision_stop import (
    CollisionStopConfig,
    ScanInput,
    Transform2D,
    _resolve_scan_transform,
    _sector_samples,
    evaluate_scan,
)
from .range_motion import (
    AngularRangeCandidate,
    MotionDirection,
    MotionGoal,
    MotionMode,
    RangeMotionConfig,
    RangeMotionController,
    RangeMotionSample,
    RangeMotionTelemetry,
    track_stable_surface,
)


def _goal_from_json(payload: str) -> MotionGoal:
    data = json.loads(payload)
    return MotionGoal(
        direction=MotionDirection(str(data["direction"])),
        mode=MotionMode(str(data.get("mode", "approach"))),
        target_clearance_m=float(data["target_clearance_m"]),
        max_measured_displacement_m=_optional_float(data.get("max_measured_displacement_m")),
        timeout_s=_optional_float(data.get("timeout_s")),
    )


def _goal_from_parameters(parameters: Mapping[str, Any]) -> MotionGoal:
    return MotionGoal(
        direction=MotionDirection(str(parameters["service_goal_direction"])),
        mode=MotionMode(str(parameters["service_goal_mode"])),
        target_clearance_m=float(parameters["service_goal_target_clearance_m"]),
        max_measured_displacement_m=_optional_float(parameters.get("service_goal_max_measured_displacement_m")),
        timeout_s=_optional_float(parameters.get("service_goal_timeout_s")),
    )


def _scan_sector_candidates(scan: ScanInput, config: CollisionStopConfig, sector: str) -> tuple[float, ...]:
    transform, _tf_available, _tf_reason = _resolve_scan_transform(scan, config)
    values = _sector_samples(scan, config, transform).get(sector, [])
    return tuple(float(value) for value in values if value is not None)


def _scan_sector_angular_candidates(
    scan: ScanInput, config: CollisionStopConfig, sector: str
) -> tuple[AngularRangeCandidate, ...]:
    transform, _tf_available, _tf_reason = _resolve_scan_transform(scan, config)
    min_range = max(float(scan.range_min), config.min_range_m)
    max_range = min(float(scan.range_max), config.max_range_m)
    candidates: list[AngularRangeCandidate] = []
    for index, raw_value in enumerate(scan.ranges):
        try:
            range_m = float(raw_value)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(range_m) or range_m < min_range or range_m > max_range:
            continue
        scan_angle = scan.angle_min + index * scan.angle_increment
        point_x = range_m * math.cos(scan_angle)
        point_y = range_m * math.sin(scan_angle)
        base_x, base_y = transform.transform_point(point_x, point_y)
        angle_rad = math.atan2(base_y, base_x)
        angle_deg = ((math.degrees(angle_rad) + 180.0) % 360.0) - 180.0
        if _angle_in_sector(angle_deg, config, sector):
            candidates.append(AngularRangeCandidate(range_m=range_m, angle_rad=angle_rad))
    return tuple(candidates)


def _angle_in_sector(angle_deg: float, config: CollisionStopConfig, sector: str) -> bool:
    if sector == "front":
        return config.front_stop_min_angle_deg <= angle_deg <= config.front_stop_max_angle_deg
    if sector == "front_slow":
        return config.front_slow_min_angle_deg <= angle_deg <= config.front_slow_max_angle_deg
    if sector == "rear":
        rear_width = abs(config.rear_stop_angle_width_deg)
        return angle_deg >= 180.0 - rear_width or angle_deg <= -180.0 + rear_width
    if sector == "left":
        return config.left_spin_min_angle_deg <= angle_deg <= config.left_spin_max_angle_deg
    if sector == "right":
        return config.right_spin_min_angle_deg <= angle_deg <= config.right_spin_max_angle_deg
    return False


def _telemetry_to_json(telemetry: RangeMotionTelemetry) -> str:
    return json.dumps(
        {
            "stop_reason": telemetry.stop_reason.value,
            "requested_velocity_mps": telemetry.requested_velocity_mps,
            "forwarded_velocity_mps": telemetry.forwarded_velocity_mps,
            "lidar_range_rate_mps": telemetry.lidar_range_rate_mps,
            "odom_velocity_mps": telemetry.odom_velocity_mps,
            "measured_displacement_m": telemetry.measured_displacement_m,
            "confidence": telemetry.confidence,
            "target_clearance_m": telemetry.target_clearance_m,
            "current_clearance_m": telemetry.current_clearance_m,
            "health": telemetry.health,
        },
        sort_keys=True,
    )


def _optional_float(value: Any) -> Optional[float]:
    return None if value is None else float(value)


def _stamp_seconds(stamp: Any) -> Optional[float]:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return float(sec) + float(nanosec) / 1_000_000_000.0


def _odom_x(msg: Any) -> Optional[float]:
    try:
        return float(msg.pose.pose.position.x)
    except Exception:
        return None


def _transform2d_from_transform_stamped(transform_stamped: Any) -> Transform2D:
    transform = transform_stamped.transform
    translation = transform.translation
    rotation = transform.rotation
    x = float(getattr(translation, "x", 0.0))
    y = float(getattr(translation, "y", 0.0))
    qx = float(getattr(rotation, "x", 0.0))
    qy = float(getattr(rotation, "y", 0.0))
    qz = float(getattr(rotation, "z", 0.0))
    qw = float(getattr(rotation, "w", 1.0))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    result = Transform2D(x=x, y=y, yaw=yaw)
    if not result.is_finite():
        raise ValueError("malformed_tf")
    return result


def _tf_error_reason(exc: Exception) -> str:
    name = exc.__class__.__name__.lower()
    message = str(exc).lower()
    if "extrapolation" in name or "extrapolation" in message or "stale" in message:
        return "stale_tf"
    if "value" in name or "malformed" in message:
        return "malformed_tf"
    return "missing_tf"


def main(args=None):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    class RangeMotionControllerNode(Node):
        def __init__(self):
            super().__init__("range_motion_controller")
            self._declare_parameters()
            self._motion_config = self._read_motion_config()
            self._scan_config = self._read_scan_config()
            self._controller = RangeMotionController(self._motion_config)
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._active_goal: Optional[MotionGoal] = None
            self._latest_scan_eval = None
            self._latest_scan: Optional[ScanInput] = None
            self._latest_odom_x: Optional[float] = None
            self._tracked_target_clearance_m: Optional[float] = None
            self._last_status = "idle"

            self._cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
            self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
            self._diagnostics_pub = self.create_publisher(DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10)
            self.create_subscription(String, str(self.get_parameter("goal_topic").value), self._on_goal, 10)
            self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan, 10)
            self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10)
            self.create_service(Trigger, "range_motion/start", self._on_start_service)
            self.create_service(Trigger, "range_motion/cancel", self._on_cancel)

        def _declare_parameters(self):
            motion_defaults = RangeMotionConfig()
            scan_defaults = CollisionStopConfig()
            for name, value in {
                "goal_topic": "/range_motion/goal",
                "status_topic": "/range_motion/status",
                "cmd_vel_topic": "/cmd_vel",
                "scan_topic": "/scan",
                "odom_topic": "/odom",
                "diagnostics_topic": "/diagnostics",
                "base_frame": scan_defaults.base_frame,
                "laser_frame": scan_defaults.laser_frame,
                "max_speed_mps": motion_defaults.max_speed_mps,
                "min_speed_mps": motion_defaults.min_speed_mps,
                "acceleration_mps2": motion_defaults.acceleration_mps2,
                "slowdown_distance_m": motion_defaults.slowdown_distance_m,
                "target_tolerance_m": motion_defaults.target_tolerance_m,
                "max_sample_age_s": motion_defaults.max_sample_age_s,
                "rate_window_s": motion_defaults.rate_window_s,
                "min_front_clearance_m": motion_defaults.min_front_clearance_m,
                "min_rear_clearance_m": motion_defaults.min_rear_clearance_m,
                "min_side_clearance_m": motion_defaults.min_side_clearance_m,
                "max_range_jump_m": motion_defaults.max_range_jump_m,
                "stall_timeout_s": motion_defaults.stall_timeout_s,
                "min_progress_m": motion_defaults.min_progress_m,
                "max_odom_lidar_disagreement_m": motion_defaults.max_odom_lidar_disagreement_m,
                "min_valid_ranges": scan_defaults.min_valid_ranges,
                "min_valid_fraction": scan_defaults.min_valid_fraction,
                "min_range_m": scan_defaults.min_range_m,
                "max_range_m": scan_defaults.max_range_m,
                "sector_unknown_policy": scan_defaults.sector_unknown_policy,
                "fail_on_missing_tf": scan_defaults.fail_on_missing_tf,
                "tf_timeout_s": scan_defaults.tf_timeout_s,
                "service_goal_direction": "forward",
                "service_goal_mode": "approach",
                "service_goal_target_clearance_m": 0.1016,
                "service_goal_max_measured_displacement_m": 1.0,
                "service_goal_timeout_s": 8.0,
            }.items():
                self.declare_parameter(name, value)

        def _read_motion_config(self) -> RangeMotionConfig:
            return RangeMotionConfig(
                max_speed_mps=float(self.get_parameter("max_speed_mps").value),
                min_speed_mps=float(self.get_parameter("min_speed_mps").value),
                acceleration_mps2=float(self.get_parameter("acceleration_mps2").value),
                slowdown_distance_m=float(self.get_parameter("slowdown_distance_m").value),
                target_tolerance_m=float(self.get_parameter("target_tolerance_m").value),
                max_sample_age_s=float(self.get_parameter("max_sample_age_s").value),
                rate_window_s=float(self.get_parameter("rate_window_s").value),
                min_front_clearance_m=float(self.get_parameter("min_front_clearance_m").value),
                min_rear_clearance_m=float(self.get_parameter("min_rear_clearance_m").value),
                min_side_clearance_m=float(self.get_parameter("min_side_clearance_m").value),
                max_range_jump_m=float(self.get_parameter("max_range_jump_m").value),
                stall_timeout_s=float(self.get_parameter("stall_timeout_s").value),
                min_progress_m=float(self.get_parameter("min_progress_m").value),
                max_odom_lidar_disagreement_m=float(self.get_parameter("max_odom_lidar_disagreement_m").value),
            )

        def _read_scan_config(self) -> CollisionStopConfig:
            return CollisionStopConfig(
                min_valid_ranges=int(self.get_parameter("min_valid_ranges").value),
                min_valid_fraction=float(self.get_parameter("min_valid_fraction").value),
                min_range_m=float(self.get_parameter("min_range_m").value),
                max_range_m=float(self.get_parameter("max_range_m").value),
                sector_unknown_policy=str(self.get_parameter("sector_unknown_policy").value),
                fail_on_missing_tf=bool(self.get_parameter("fail_on_missing_tf").value),
                base_frame=str(self.get_parameter("base_frame").value),
                laser_frame=str(self.get_parameter("laser_frame").value),
                tf_timeout_s=float(self.get_parameter("tf_timeout_s").value),
            )

        def _on_goal(self, msg):
            try:
                goal = _goal_from_json(str(msg.data))
            except Exception as exc:
                self.get_logger().error(f"invalid range-motion goal: {exc}")
                self._publish_zero_status("invalid_goal")
                return
            self._start_goal(goal)

        def _on_start_service(self, request, response):
            try:
                goal = _goal_from_parameters(
                    {
                        "service_goal_direction": self.get_parameter("service_goal_direction").value,
                        "service_goal_mode": self.get_parameter("service_goal_mode").value,
                        "service_goal_target_clearance_m": self.get_parameter("service_goal_target_clearance_m").value,
                        "service_goal_max_measured_displacement_m": self.get_parameter(
                            "service_goal_max_measured_displacement_m"
                        ).value,
                        "service_goal_timeout_s": self.get_parameter("service_goal_timeout_s").value,
                    }
                )
            except Exception as exc:
                self.get_logger().error(f"invalid range-motion service goal: {exc}")
                self._publish_zero_status("invalid_goal")
                response.success = False
                response.message = f"invalid range-motion service goal: {exc}"
                return response
            telemetry = self._start_goal(goal)
            response.success = telemetry is not None and telemetry.stop_reason.value == "running"
            response.message = self._last_status
            return response

        def _start_goal(self, goal: MotionGoal) -> Optional[RangeMotionTelemetry]:
            self._tracked_target_clearance_m = None
            sample = self._current_sample(goal)
            if sample is None:
                self._publish_zero_status("missing_scan")
                return None
            self._active_goal = goal
            telemetry = self._controller.start(goal, sample)
            self._publish_telemetry(telemetry)
            return telemetry

        def _on_scan(self, msg):
            header = getattr(msg, "header", None)
            stamp = _stamp_seconds(getattr(header, "stamp", None))
            frame_id = str(getattr(header, "frame_id", "")) or self._scan_config.laser_frame
            transform_to_base, transform_error = self._lookup_scan_transform(frame_id, getattr(header, "stamp", None))
            scan = ScanInput(
                ranges=tuple(getattr(msg, "ranges", []) or []),
                angle_min=float(msg.angle_min),
                angle_increment=float(msg.angle_increment),
                range_min=float(msg.range_min),
                range_max=float(msg.range_max),
                stamp=stamp,
                received_at=self._now_seconds(),
                frame_id=frame_id,
                transform_to_base=transform_to_base,
                transform_error=transform_error,
            )
            self._latest_scan = scan
            self._latest_scan_eval = evaluate_scan(scan, self._scan_config, now=self._now_seconds())
            if self._active_goal is None:
                return
            sample = self._current_sample(self._active_goal)
            if sample is None:
                self._publish_zero_status("missing_scan")
                return
            telemetry = self._controller.update(sample, now=self._now_seconds())
            self._publish_telemetry(telemetry)
            if telemetry.stop_reason.value != "running":
                self._active_goal = None
                self._tracked_target_clearance_m = None

        def _lookup_scan_transform(self, frame_id: str, stamp_msg) -> tuple[Optional[Transform2D], Optional[str]]:
            if frame_id == self._scan_config.base_frame:
                return Transform2D(), None
            try:
                time = Time.from_msg(stamp_msg) if stamp_msg is not None else Time()
                stamped = self._tf_buffer.lookup_transform(
                    self._scan_config.base_frame,
                    frame_id or self._scan_config.laser_frame,
                    time,
                    timeout=Duration(seconds=self._scan_config.tf_timeout_s),
                )
                return _transform2d_from_transform_stamped(stamped), None
            except Exception as exc:
                reason = _tf_error_reason(exc)
                self.get_logger().warn(f"range motion TF lookup failed: {reason}: {exc}")
                return None, reason

        def _on_odom(self, msg):
            self._latest_odom_x = _odom_x(msg)

        def _on_cancel(self, request, response):
            self._active_goal = None
            self._tracked_target_clearance_m = None
            self._publish_zero_status("operator_stop")
            response.success = True
            response.message = "range motion cancelled; zero command published"
            return response

        def _current_sample(self, goal: MotionGoal) -> Optional[RangeMotionSample]:
            eval_result = self._latest_scan_eval
            if eval_result is None or not eval_result.healthy:
                return None
            nearest = eval_result.nearest
            target_sector = "front"
            if goal.direction.value == "backward" and goal.mode.value == "approach":
                target_sector = "rear"
            target_clearance = nearest.get(target_sector)
            if self._latest_scan is not None:
                track = track_stable_surface(
                    previous_clearance_m=self._tracked_target_clearance_m,
                    candidate_clearances_m=_scan_sector_candidates(self._latest_scan, self._scan_config, target_sector),
                    association_gate_m=self._motion_config.max_range_jump_m,
                )
                target_clearance = track.clearance_m
                self._tracked_target_clearance_m = track.clearance_m
            return RangeMotionSample(
                stamp=self._now_seconds(),
                target_clearance_m=target_clearance,
                front_clearance_m=nearest.get("front"),
                rear_clearance_m=nearest.get("rear"),
                left_clearance_m=nearest.get("left"),
                right_clearance_m=nearest.get("right"),
                odom_displacement_m=self._latest_odom_x,
                target_candidates=()
                if self._latest_scan is None
                else _scan_sector_angular_candidates(self._latest_scan, self._scan_config, target_sector),
            )

        def _publish_zero_status(self, reason: str):
            twist = Twist()
            self._cmd_pub.publish(twist)
            msg = String()
            msg.data = json.dumps({"stop_reason": reason, "forwarded_velocity_mps": 0.0}, sort_keys=True)
            self._status_pub.publish(msg)
            self._last_status = reason

        def _publish_telemetry(self, telemetry: RangeMotionTelemetry):
            twist = Twist()
            twist.linear.x = telemetry.command.linear_x
            twist.angular.z = telemetry.command.angular_z
            self._cmd_pub.publish(twist)
            msg = String()
            msg.data = _telemetry_to_json(telemetry)
            self._status_pub.publish(msg)
            self._publish_diagnostics(telemetry)
            self._last_status = telemetry.stop_reason.value

        def _publish_diagnostics(self, telemetry: RangeMotionTelemetry):
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "range_motion_controller"
            status.hardware_id = "sphero_rvr_range_motion"
            status.level = DiagnosticStatus.OK if telemetry.stop_reason.value == "running" else DiagnosticStatus.WARN
            status.message = f"range motion: {telemetry.stop_reason.value}"
            fields = {
                "stop_reason": telemetry.stop_reason.value,
                "requested_velocity_mps": f"{telemetry.requested_velocity_mps:.3f}",
                "forwarded_velocity_mps": f"{telemetry.forwarded_velocity_mps:.3f}",
                "lidar_range_rate_mps": f"{telemetry.lidar_range_rate_mps:.3f}",
                "odom_velocity_mps": "" if telemetry.odom_velocity_mps is None else f"{telemetry.odom_velocity_mps:.3f}",
                "measured_displacement_m": f"{telemetry.measured_displacement_m:.3f}",
                "confidence": f"{telemetry.confidence:.3f}",
                "target_clearance_m": "" if telemetry.target_clearance_m is None else f"{telemetry.target_clearance_m:.3f}",
                "current_clearance_m": "" if telemetry.current_clearance_m is None else f"{telemetry.current_clearance_m:.3f}",
                "health": telemetry.health,
            }
            status.values = [KeyValue(key=k, value=v) for k, v in fields.items()]
            array.status = [status]
            self._diagnostics_pub.publish(array)

        def _now_seconds(self) -> float:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0

    rclpy.init(args=args)
    node = RangeMotionControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
