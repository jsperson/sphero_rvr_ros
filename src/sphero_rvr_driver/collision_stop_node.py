"""ROS 2 wrapper for the lidar collision-stop supervisor.

The safety state machine is implemented in :mod:`sphero_rvr_driver.collision_stop`
so unit tests can run without ROS.  This module imports ROS lazily and owns the
public STOP/ESTOP services plus the final /cmd_vel_motor publisher in supervised
launches.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional

from .collision_stop import CollisionStopConfig, CollisionStopSupervisor, ScanInput, Transform2D, TwistCommand


@dataclass(frozen=True)
class DriverForwardResult:
    success: bool
    message: str


class DriverServiceForwarder:
    """Forward public safety services to the driver with bounded confirmation.

    The public supervisor services are only truthful if the downstream driver
    response has actually arrived.  This helper waits on a Future completion
    event; it does not spin an executor.  The ROS node must therefore run under
    an executor that can make client futures progress while a service callback is
    waiting, e.g. MultiThreadedExecutor with separate callback groups.
    """

    def __init__(self, *, timeout_s: float):
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self._timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self._pending: dict[Any, threading.Event] = {}
        self._shutdown = False

    def call(self, client: Any, label: str, request_factory: Callable[[], Any]) -> DriverForwardResult:
        if self.is_shutdown:
            return DriverForwardResult(False, f"{label} not forwarded during shutdown; local supervisor state already forced zero")
        if not client.service_is_ready():
            return DriverForwardResult(False, f"{label} unavailable; local supervisor state already forced zero")
        try:
            future = client.call_async(request_factory())
        except Exception as exc:
            return DriverForwardResult(False, f"{label} request failed: {exc}; local supervisor state already forced zero")

        done = threading.Event()
        holder: dict[str, DriverForwardResult] = {}
        with self._lock:
            if self._shutdown:
                self._cancel_future(future)
                return DriverForwardResult(False, f"{label} not forwarded during shutdown; local supervisor state already forced zero")
            self._pending[future] = done

        def _complete(done_future: Any) -> None:
            try:
                if self.is_shutdown:
                    holder.setdefault(
                        "result",
                        DriverForwardResult(False, f"{label} aborted during shutdown; local supervisor state already forced zero"),
                    )
                    return
                result = done_future.result()
                success = bool(getattr(result, "success", False))
                message = str(getattr(result, "message", label))
                holder["result"] = DriverForwardResult(success, message)
            except Exception as exc:
                if self.is_shutdown:
                    holder["result"] = DriverForwardResult(
                        False, f"{label} aborted during shutdown; local supervisor state already forced zero"
                    )
                else:
                    holder["result"] = DriverForwardResult(
                        False, f"{label} response failed: {exc}; local supervisor state already forced zero"
                    )
            finally:
                with self._lock:
                    self._pending.pop(done_future, None)
                done.set()

        future.add_done_callback(_complete)
        if not done.wait(self._timeout_s):
            with self._lock:
                self._pending.pop(future, None)
            self._cancel_future(future)
            return DriverForwardResult(False, f"{label} timed out; local supervisor state already forced zero")
        if self.is_shutdown:
            return DriverForwardResult(False, f"{label} aborted during shutdown; local supervisor state already forced zero")
        return holder.get(
            "result",
            DriverForwardResult(False, f"{label} completed without a response; local supervisor state already forced zero"),
        )

    def shutdown(self) -> None:
        with self._lock:
            self._shutdown = True
            pending = list(self._pending.items())
            self._pending.clear()
        for future, done in pending:
            self._cancel_future(future)
            done.set()

    @property
    def is_shutdown(self) -> bool:
        with self._lock:
            return self._shutdown

    @staticmethod
    def _cancel_future(future: Any) -> None:
        cancel = getattr(future, "cancel", None)
        if cancel is not None:
            try:
                cancel()
            except Exception:
                pass


def _stamp_seconds(stamp: Any) -> Optional[float]:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return float(sec) + float(nanosec) / 1_000_000_000.0


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
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    class LidarCollisionStopSupervisorNode(Node):
        def __init__(self):
            super().__init__("lidar_collision_stop_supervisor")
            self._shutting_down = False
            self._declare_parameters()
            self._config = self._read_config()
            self._driver_forwarder = DriverServiceForwarder(
                timeout_s=float(self.get_parameter("driver_service_timeout_s").value)
            )
            self._state_lock = threading.RLock()
            self._service_group = ReentrantCallbackGroup()
            self._driver_client_group = ReentrantCallbackGroup()
            now = self._now_seconds()
            self._supervisor = CollisionStopSupervisor(self._config, now=now)
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
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

            self._driver_stop_client = self.create_client(
                Trigger,
                str(self.get_parameter("driver_stop_service").value),
                callback_group=self._driver_client_group,
            )
            self._driver_estop_client = self.create_client(
                Trigger,
                str(self.get_parameter("driver_estop_service").value),
                callback_group=self._driver_client_group,
            )
            self._driver_clear_estop_client = self.create_client(
                Trigger,
                str(self.get_parameter("driver_clear_estop_service").value),
                callback_group=self._driver_client_group,
            )
            self.create_service(Trigger, "stop", self._on_stop, callback_group=self._service_group)
            self.create_service(Trigger, "estop", self._on_estop, callback_group=self._service_group)
            self.create_service(Trigger, "clear_estop", self._on_clear_estop, callback_group=self._service_group)
            self.create_service(Trigger, "collision_stop/reset", self._on_reset)
            self.create_timer(self._config.zero_publish_period_s, self._on_timer)

        def destroy_node(self):
            self._begin_shutdown()
            super().destroy_node()

        def _begin_shutdown(self):
            self._shutting_down = True
            self._driver_forwarder.shutdown()

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
                "driver_service_timeout_s": 1.0,
                "base_frame": "base_link",
                "laser_frame": "laser",
                "max_scan_age_s": defaults.max_scan_age_s,
                "max_scan_stamp_age_s": defaults.max_scan_stamp_age_s,
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
                "left_spin_min_angle_deg": defaults.left_spin_min_angle_deg,
                "left_spin_max_angle_deg": defaults.left_spin_max_angle_deg,
                "right_spin_min_angle_deg": defaults.right_spin_min_angle_deg,
                "right_spin_max_angle_deg": defaults.right_spin_max_angle_deg,
                "stop_distance_m": defaults.stop_distance_m,
                "slow_distance_m": defaults.slow_distance_m,
                "reverse_stop_distance_m": defaults.reverse_stop_distance_m,
                "trajectory_clearance_margin_m": defaults.trajectory_clearance_margin_m,
                "measured_stop_time_s": defaults.measured_stop_time_s,
                "braking_distance_margin_m": defaults.braking_distance_margin_m,
                "release_distance_m": defaults.release_distance_m,
                "release_time_s": defaults.release_time_s,
                "min_forward_scale": defaults.min_forward_scale,
                "max_forward_mps": defaults.max_forward_mps,
                "max_angular_rad_s": defaults.max_angular_rad_s,
                "reset_policy": defaults.reset_policy.value,
                "zero_publish_period_s": defaults.zero_publish_period_s,
                "allow_disable": defaults.allow_disable,
                "fail_on_missing_tf": defaults.fail_on_missing_tf,
                "tf_timeout_s": defaults.tf_timeout_s,
                "requested_cmd_timeout_s": defaults.requested_cmd_timeout_s,
            }.items():
                self.declare_parameter(name, value)

        def _read_config(self) -> CollisionStopConfig:
            return CollisionStopConfig(
                requested_cmd_timeout_s=float(self.get_parameter("requested_cmd_timeout_s").value),
                max_scan_age_s=float(self.get_parameter("max_scan_age_s").value),
                max_scan_stamp_age_s=float(self.get_parameter("max_scan_stamp_age_s").value),
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
                left_spin_min_angle_deg=float(self.get_parameter("left_spin_min_angle_deg").value),
                left_spin_max_angle_deg=float(self.get_parameter("left_spin_max_angle_deg").value),
                right_spin_min_angle_deg=float(self.get_parameter("right_spin_min_angle_deg").value),
                right_spin_max_angle_deg=float(self.get_parameter("right_spin_max_angle_deg").value),
                stop_distance_m=float(self.get_parameter("stop_distance_m").value),
                slow_distance_m=float(self.get_parameter("slow_distance_m").value),
                reverse_stop_distance_m=float(self.get_parameter("reverse_stop_distance_m").value),
                trajectory_clearance_margin_m=float(self.get_parameter("trajectory_clearance_margin_m").value),
                measured_stop_time_s=float(self.get_parameter("measured_stop_time_s").value),
                braking_distance_margin_m=float(self.get_parameter("braking_distance_margin_m").value),
                release_distance_m=float(self.get_parameter("release_distance_m").value),
                release_time_s=float(self.get_parameter("release_time_s").value),
                min_forward_scale=float(self.get_parameter("min_forward_scale").value),
                max_forward_mps=float(self.get_parameter("max_forward_mps").value),
                max_angular_rad_s=float(self.get_parameter("max_angular_rad_s").value),
                reset_policy=str(self.get_parameter("reset_policy").value),
                zero_publish_period_s=float(self.get_parameter("zero_publish_period_s").value),
                allow_disable=bool(self.get_parameter("allow_disable").value),
                fail_on_missing_tf=bool(self.get_parameter("fail_on_missing_tf").value),
                base_frame=str(self.get_parameter("base_frame").value),
                laser_frame=str(self.get_parameter("laser_frame").value),
                tf_timeout_s=float(self.get_parameter("tf_timeout_s").value),
            )

        def _on_scan(self, msg):
            with self._state_lock:
                header = getattr(msg, "header", None)
                stamp_msg = getattr(header, "stamp", None)
                stamp = _stamp_seconds(stamp_msg)
                frame_id = str(getattr(header, "frame_id", "")) or self._config.laser_frame
                transform_to_base, transform_error = self._lookup_scan_transform(frame_id, stamp_msg)
                decision = self._supervisor.update_scan(
                    ScanInput(
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
                    ),
                    now=self._now_seconds(),
                )
                self._publish_decision(decision)

        def _lookup_scan_transform(self, frame_id: str, stamp_msg) -> tuple[Optional[Transform2D], Optional[str]]:
            if frame_id == self._config.base_frame:
                return Transform2D(), None
            try:
                time = Time.from_msg(stamp_msg) if stamp_msg is not None else Time()
                stamped = self._tf_buffer.lookup_transform(
                    self._config.base_frame,
                    frame_id or self._config.laser_frame,
                    time,
                    timeout=Duration(seconds=self._config.tf_timeout_s),
                )
                return _transform2d_from_transform_stamped(stamped), None
            except Exception as exc:
                reason = _tf_error_reason(exc)
                if self._context_ok():
                    self.get_logger().warn(f"collision stop TF lookup failed: {reason}: {exc}")
                return None, reason

        def _on_cmd_vel(self, msg):
            with self._state_lock:
                decision = self._supervisor.apply_command(
                    TwistCommand(float(msg.linear.x), float(msg.angular.z)), now=self._now_seconds()
                )
                self._publish_decision(decision)

        def _on_timer(self):
            with self._state_lock:
                decision = self._supervisor.tick(now=self._now_seconds())
                self._publish_decision(decision)

        def _on_stop(self, request, response):
            with self._state_lock:
                decision = self._supervisor.stop(now=self._now_seconds())
                self._publish_decision(decision)
            ok, message = self._call_driver(self._driver_stop_client, "driver stop")
            response.success = ok
            response.message = message
            return response

        def _on_estop(self, request, response):
            with self._state_lock:
                decision = self._supervisor.estop(now=self._now_seconds())
                self._publish_decision(decision)
            ok, message = self._call_driver(self._driver_estop_client, "driver estop")
            response.success = ok
            response.message = message
            return response

        def _on_clear_estop(self, request, response):
            with self._state_lock:
                decision = self._supervisor.clear_estop(now=self._now_seconds())
                self._publish_decision(decision)
            ok, message = self._call_driver(self._driver_clear_estop_client, "driver clear_estop")
            response.success = ok
            response.message = message
            return response

        def _on_reset(self, request, response):
            with self._state_lock:
                result = self._supervisor.reset(now=self._now_seconds())
                self._publish_decision(result.decision)
            response.success = result.accepted
            response.message = result.reason
            return response

        def _call_driver(self, client, label: str) -> tuple[bool, str]:
            result = self._driver_forwarder.call(client, label, Trigger.Request)
            return result.success, result.message

        def _publish_decision(self, decision):
            if not self._context_ok():
                return
            msg = Twist()
            msg.linear.x = decision.output.linear_x
            msg.angular.z = decision.output.angular_z
            self._cmd_pub.publish(msg)

            state = String()
            state.data = (
                f"{decision.state.value} reason={decision.reason} "
                f"scan_healthy={str(decision.scan_health.healthy).lower()} "
                f"scan_reason={decision.scan_health.reason} "
                f"scan_age={decision.scan_health.age_s} "
                f"scan_stamp_age={decision.scan_health.stamp_age_s} "
                f"tf_available={str(decision.scan_health.tf_available).lower()} "
                f"tf_reason={decision.scan_health.tf_reason} "
                f"front={decision.nearest.get('front')} "
                f"front_slow={decision.nearest.get('front_slow')} "
                f"front_slow_min_angle_deg={self._config.front_slow_min_angle_deg} "
                f"front_slow_max_angle_deg={self._config.front_slow_max_angle_deg} "
                f"stop_distance_m={self._config.stop_distance_m} "
                f"slow_distance_m={self._config.slow_distance_m} "
                f"rear={decision.nearest.get('rear')} "
                f"left={decision.nearest.get('left')} "
                f"right={decision.nearest.get('right')} "
                f"trajectory_clearance_margin_m={self._config.trajectory_clearance_margin_m} "
                f"trajectory_horizon_s={None if decision.trajectory is None else decision.trajectory.horizon_s} "
                f"trajectory_min_clearance_m={None if decision.trajectory is None else decision.trajectory.minimum_clearance_m} "
                f"trajectory_collision_time_s={None if decision.trajectory is None else decision.trajectory.collision_time_s} "
                f"requested=({decision.requested.linear_x:.3f},{decision.requested.angular_z:.3f}) "
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
            if not self._context_ok():
                return
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
                "scan_stamp_age_s": (
                    ""
                    if decision.scan_health.stamp_age_s is None
                    else f"{decision.scan_health.stamp_age_s:.3f}"
                ),
                "scan_frame": decision.scan_health.frame_id,
                "base_frame": decision.scan_health.base_frame,
                "valid_ranges": str(decision.scan_health.valid_count),
                "considered_ranges": str(decision.scan_health.considered_count),
                "nearest_front_m": _fmt_optional(decision.nearest.get("front")),
                "nearest_front_slow_m": _fmt_optional(decision.nearest.get("front_slow")),
                "front_slow_min_angle_deg": f"{self._config.front_slow_min_angle_deg:.3f}",
                "front_slow_max_angle_deg": f"{self._config.front_slow_max_angle_deg:.3f}",
                "stop_distance_m": f"{self._config.stop_distance_m:.3f}",
                "slow_distance_m": f"{self._config.slow_distance_m:.3f}",
                "trajectory_clearance_margin_m": f"{self._config.trajectory_clearance_margin_m:.3f}",
                "trajectory_horizon_s": _fmt_optional(
                    None if decision.trajectory is None else decision.trajectory.horizon_s
                ),
                "trajectory_min_clearance_m": _fmt_optional(
                    None
                    if decision.trajectory is None
                    else decision.trajectory.minimum_clearance_m
                ),
                "trajectory_collision_time_s": _fmt_optional(
                    None
                    if decision.trajectory is None
                    else decision.trajectory.collision_time_s
                ),
                "trajectory_collision_point_x_m": _fmt_optional(
                    None
                    if decision.trajectory is None
                    else decision.trajectory.collision_point_x_m
                ),
                "trajectory_collision_point_y_m": _fmt_optional(
                    None
                    if decision.trajectory is None
                    else decision.trajectory.collision_point_y_m
                ),
                "nearest_rear_m": _fmt_optional(decision.nearest.get("rear")),
                "nearest_left_m": _fmt_optional(decision.nearest.get("left")),
                "nearest_right_m": _fmt_optional(decision.nearest.get("right")),
                "requested_linear_x": f"{decision.requested.linear_x:.3f}",
                "requested_angular_z": f"{decision.requested.angular_z:.3f}",
                "output_linear_x": f"{decision.output.linear_x:.3f}",
                "output_angular_z": f"{decision.output.angular_z:.3f}",
                "scale": f"{decision.scale:.3f}",
                "reset_required": str(decision.reset_required).lower(),
                "tf_available": str(decision.scan_health.tf_available).lower(),
                "tf_reason": decision.scan_health.tf_reason,
                "tf_timeout_s": f"{self._config.tf_timeout_s:.3f}",
            }
            status.values = [KeyValue(key=k, value=v) for k, v in fields.items()]
            array.status = [status]
            self._diagnostics_pub.publish(array)

        def _now_seconds(self) -> float:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0

        def _context_ok(self) -> bool:
            return not self._shutting_down and self.context.ok()

    def _fmt_optional(value):
        return "" if value is None else f"{float(value):.3f}"

    rclpy.init(args=args)
    node = LidarCollisionStopSupervisorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._begin_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
