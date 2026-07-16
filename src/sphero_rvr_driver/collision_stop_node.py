"""ROS 2 wrapper for the lidar collision-stop supervisor.

The safety state machine is implemented in :mod:`sphero_rvr_driver.collision_stop`
so unit tests can run without ROS.  This module imports ROS lazily and owns the
public STOP/ESTOP services plus the final /cmd_vel_motor publisher in supervised
launches.
"""

from __future__ import annotations

from typing import Any, Optional

from .collision_stop import CollisionStopConfig, CollisionStopSupervisor, ScanInput, TwistCommand


def _stamp_seconds(stamp: Any) -> Optional[float]:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return float(sec) + float(nanosec) / 1_000_000_000.0


def main(args=None):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class LidarCollisionStopSupervisorNode(Node):
        def __init__(self):
            super().__init__("lidar_collision_stop_supervisor")
            self._declare_parameters()
            self._config = self._read_config()
            now = self._now_seconds()
            self._supervisor = CollisionStopSupervisor(self._config, now=now)
            self._last_event = "STARTUP_WAITING_FOR_SCAN"

            requested_cmd_topic = str(self.get_parameter("requested_cmd_topic").value)
            motor_cmd_topic = str(self.get_parameter("motor_cmd_topic").value)
            scan_topic = str(self.get_parameter("scan_topic").value)
            state_topic = str(self.get_parameter("state_topic").value)
            events_topic = str(self.get_parameter("events_topic").value)
            diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)

            self._cmd_pub = self.create_publisher(Twist, motor_cmd_topic, 10)
            self._state_pub = self.create_publisher(String, state_topic, 10)
            self._events_pub = self.create_publisher(String, events_topic, 10)
            self._diagnostics_pub = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)
            self.create_subscription(Twist, requested_cmd_topic, self._on_cmd_vel, 10)
            self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

            self._driver_stop_client = self.create_client(Trigger, str(self.get_parameter("driver_stop_service").value))
            self._driver_estop_client = self.create_client(Trigger, str(self.get_parameter("driver_estop_service").value))
            self._driver_clear_estop_client = self.create_client(Trigger, str(self.get_parameter("driver_clear_estop_service").value))
            self.create_service(Trigger, "stop", self._on_stop)
            self.create_service(Trigger, "estop", self._on_estop)
            self.create_service(Trigger, "clear_estop", self._on_clear_estop)
            self.create_service(Trigger, "collision_stop/reset", self._on_reset)
            self.create_timer(self._config.zero_publish_period_s, self._on_timer)

        def _declare_parameters(self):
            defaults = CollisionStopConfig()
            for name, value in {
                "requested_cmd_topic": "/cmd_vel",
                "motor_cmd_topic": "/cmd_vel_motor",
                "scan_topic": "/scan",
                "diagnostics_topic": "/diagnostics",
                "state_topic": "/collision_stop/state",
                "events_topic": "/collision_stop/events",
                "driver_stop_service": "/rvr_driver/stop",
                "driver_estop_service": "/rvr_driver/estop",
                "driver_clear_estop_service": "/rvr_driver/clear_estop",
                "base_frame": "base_link",
                "laser_frame": "laser",
                "max_scan_age_s": defaults.max_scan_age_s,
                "startup_grace_s": defaults.startup_grace_s,
                "min_valid_ranges": defaults.min_valid_ranges,
                "min_valid_fraction": defaults.min_valid_fraction,
                "min_range_m": defaults.min_range_m,
                "max_range_m": defaults.max_range_m,
                "sector_unknown_policy": defaults.sector_unknown_policy,
                "footprint_front_m": defaults.footprint_front_m,
                "footprint_rear_m": defaults.footprint_rear_m,
                "footprint_left_m": defaults.footprint_left_m,
                "footprint_right_m": defaults.footprint_right_m,
                "payload_margin_m": defaults.payload_margin_m,
                "front_stop_min_angle_deg": defaults.front_stop_min_angle_deg,
                "front_stop_max_angle_deg": defaults.front_stop_max_angle_deg,
                "front_slow_min_angle_deg": defaults.front_slow_min_angle_deg,
                "front_slow_max_angle_deg": defaults.front_slow_max_angle_deg,
                "rear_stop_angle_width_deg": defaults.rear_stop_angle_width_deg,
                "stop_distance_m": defaults.stop_distance_m,
                "slow_distance_m": defaults.slow_distance_m,
                "reverse_stop_distance_m": defaults.reverse_stop_distance_m,
                "release_distance_m": defaults.release_distance_m,
                "release_time_s": defaults.release_time_s,
                "min_forward_scale": defaults.min_forward_scale,
                "max_forward_mps": defaults.max_forward_mps,
                "max_angular_rad_s": defaults.max_angular_rad_s,
                "reset_policy": defaults.reset_policy.value,
                "zero_publish_period_s": defaults.zero_publish_period_s,
                "allow_disable": defaults.allow_disable,
                "fail_on_missing_tf": defaults.fail_on_missing_tf,
                "requested_cmd_timeout_s": defaults.requested_cmd_timeout_s,
            }.items():
                self.declare_parameter(name, value)

        def _read_config(self) -> CollisionStopConfig:
            return CollisionStopConfig(
                requested_cmd_timeout_s=float(self.get_parameter("requested_cmd_timeout_s").value),
                max_scan_age_s=float(self.get_parameter("max_scan_age_s").value),
                startup_grace_s=float(self.get_parameter("startup_grace_s").value),
                min_valid_ranges=int(self.get_parameter("min_valid_ranges").value),
                min_valid_fraction=float(self.get_parameter("min_valid_fraction").value),
                min_range_m=float(self.get_parameter("min_range_m").value),
                max_range_m=float(self.get_parameter("max_range_m").value),
                sector_unknown_policy=str(self.get_parameter("sector_unknown_policy").value),
                footprint_front_m=float(self.get_parameter("footprint_front_m").value),
                footprint_rear_m=float(self.get_parameter("footprint_rear_m").value),
                footprint_left_m=float(self.get_parameter("footprint_left_m").value),
                footprint_right_m=float(self.get_parameter("footprint_right_m").value),
                payload_margin_m=float(self.get_parameter("payload_margin_m").value),
                front_stop_min_angle_deg=float(self.get_parameter("front_stop_min_angle_deg").value),
                front_stop_max_angle_deg=float(self.get_parameter("front_stop_max_angle_deg").value),
                front_slow_min_angle_deg=float(self.get_parameter("front_slow_min_angle_deg").value),
                front_slow_max_angle_deg=float(self.get_parameter("front_slow_max_angle_deg").value),
                rear_stop_angle_width_deg=float(self.get_parameter("rear_stop_angle_width_deg").value),
                stop_distance_m=float(self.get_parameter("stop_distance_m").value),
                slow_distance_m=float(self.get_parameter("slow_distance_m").value),
                reverse_stop_distance_m=float(self.get_parameter("reverse_stop_distance_m").value),
                release_distance_m=float(self.get_parameter("release_distance_m").value),
                release_time_s=float(self.get_parameter("release_time_s").value),
                min_forward_scale=float(self.get_parameter("min_forward_scale").value),
                max_forward_mps=float(self.get_parameter("max_forward_mps").value),
                max_angular_rad_s=float(self.get_parameter("max_angular_rad_s").value),
                reset_policy=str(self.get_parameter("reset_policy").value),
                zero_publish_period_s=float(self.get_parameter("zero_publish_period_s").value),
                allow_disable=bool(self.get_parameter("allow_disable").value),
                fail_on_missing_tf=bool(self.get_parameter("fail_on_missing_tf").value),
            )

        def _on_scan(self, msg):
            stamp = _stamp_seconds(getattr(getattr(msg, "header", None), "stamp", None))
            decision = self._supervisor.update_scan(
                ScanInput(
                    ranges=tuple(getattr(msg, "ranges", []) or []),
                    angle_min=float(msg.angle_min),
                    angle_increment=float(msg.angle_increment),
                    range_min=float(msg.range_min),
                    range_max=float(msg.range_max),
                    stamp=stamp,
                    received_at=self._now_seconds(),
                    frame_id=str(getattr(getattr(msg, "header", None), "frame_id", "")),
                ),
                now=self._now_seconds(),
            )
            self._publish_decision(decision)

        def _on_cmd_vel(self, msg):
            decision = self._supervisor.apply_command(
                TwistCommand(float(msg.linear.x), float(msg.angular.z)), now=self._now_seconds()
            )
            self._publish_decision(decision)

        def _on_timer(self):
            decision = self._supervisor.tick(now=self._now_seconds())
            self._publish_decision(decision)

        def _on_stop(self, request, response):
            decision = self._supervisor.stop(now=self._now_seconds())
            self._publish_decision(decision)
            ok, message = self._call_driver(self._driver_stop_client, "driver stop")
            response.success = ok
            response.message = message
            return response

        def _on_estop(self, request, response):
            decision = self._supervisor.estop(now=self._now_seconds())
            self._publish_decision(decision)
            ok, message = self._call_driver(self._driver_estop_client, "driver estop")
            response.success = ok
            response.message = message
            return response

        def _on_clear_estop(self, request, response):
            decision = self._supervisor.clear_estop(now=self._now_seconds())
            self._publish_decision(decision)
            ok, message = self._call_driver(self._driver_clear_estop_client, "driver clear_estop")
            response.success = ok
            response.message = message
            return response

        def _on_reset(self, request, response):
            result = self._supervisor.reset(now=self._now_seconds())
            self._publish_decision(result.decision)
            response.success = result.accepted
            response.message = result.reason
            return response

        def _call_driver(self, client, label: str) -> tuple[bool, str]:
            if not client.service_is_ready():
                return False, f"{label} unavailable; local supervisor state already forced zero"
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if not future.done():
                return False, f"{label} timed out; local supervisor state already forced zero"
            result = future.result()
            return bool(getattr(result, "success", False)), str(getattr(result, "message", label))

        def _publish_decision(self, decision):
            msg = Twist()
            msg.linear.x = decision.output.linear_x
            msg.angular.z = decision.output.angular_z
            self._cmd_pub.publish(msg)

            state = String()
            state.data = (
                f"{decision.state.value} reason={decision.reason} "
                f"scan_age={decision.scan_health.age_s} front={decision.nearest.get('front')} "
                f"output=({decision.output.linear_x:.3f},{decision.output.angular_z:.3f})"
            )
            self._state_pub.publish(state)
            event_text = f"{decision.state.value} {decision.reason}"
            if event_text != self._last_event:
                event = String()
                event.data = event_text
                self._events_pub.publish(event)
                self._last_event = event_text
            self._publish_diagnostics(decision)

        def _publish_diagnostics(self, decision):
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "lidar_collision_stop_supervisor"
            status.hardware_id = "sphero_rvr_collision_stop"
            status.level = DiagnosticStatus.OK if decision.state.value in {"CLEAR", "SLOW"} else DiagnosticStatus.WARN
            status.message = f"{decision.state.value}: {decision.reason}"
            fields = {
                "state": decision.state.value,
                "previous_state": decision.previous_state.value,
                "reason": decision.reason,
                "scan_healthy": str(decision.scan_health.healthy).lower(),
                "scan_reason": decision.scan_health.reason,
                "scan_age_s": "" if decision.scan_health.age_s is None else f"{decision.scan_health.age_s:.3f}",
                "scan_frame": decision.scan_health.frame_id,
                "valid_ranges": str(decision.scan_health.valid_count),
                "considered_ranges": str(decision.scan_health.considered_count),
                "nearest_front_m": _fmt_optional(decision.nearest.get("front")),
                "nearest_rear_m": _fmt_optional(decision.nearest.get("rear")),
                "nearest_left_m": _fmt_optional(decision.nearest.get("left")),
                "nearest_right_m": _fmt_optional(decision.nearest.get("right")),
                "requested_linear_x": f"{decision.requested.linear_x:.3f}",
                "requested_angular_z": f"{decision.requested.angular_z:.3f}",
                "output_linear_x": f"{decision.output.linear_x:.3f}",
                "output_angular_z": f"{decision.output.angular_z:.3f}",
                "scale": f"{decision.scale:.3f}",
                "reset_required": str(decision.reset_required).lower(),
                "tf_available": "not_checked_ros_free_sector_mode",
                "placeholder_lidar_transform": "unknown",
            }
            status.values = [KeyValue(key=k, value=v) for k, v in fields.items()]
            array.status = [status]
            self._diagnostics_pub.publish(array)

        def _now_seconds(self) -> float:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _fmt_optional(value):
        return "" if value is None else f"{float(value):.3f}"

    rclpy.init(args=args)
    node = LidarCollisionStopSupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
