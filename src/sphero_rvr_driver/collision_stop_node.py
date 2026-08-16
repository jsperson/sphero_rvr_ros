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
from sphero_rvr_core.low_obstacle_brake import (
    BlindBandHold,
    forward_speed_scale,
    nearest_forward_obstacle,
    points_in_swept_path,
    swept_path_obstacle,
)
from sphero_rvr_core.tof_frame import TofConfig, blind_band_outer_range_m


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
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan, PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
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
            # D22. The camera cloud gets its OWN callback group. Without one it
            # landed in the node's DEFAULT (mutually exclusive) group alongside
            # /scan and /cmd_vel, which meant all three serialized onto a single
            # slot no matter how many executor threads existed -- and scan and
            # cmd_vel are the two highest-rate topics in the stack. Under load the
            # cloud callback simply did not get scheduled: measured 2026-08-10 with
            # an independent observer receiving every cloud while the supervisor's
            # own copy aged past 0.6 s, 18 times in one mission, twice while the
            # rover was DRIVING (once at full cruise). Both camera gates fail OPEN
            # on a stale cloud, so that is the sub-lidar protection silently off.
            # This callback takes no state lock, so it is safe to run in parallel
            # with the arbitration path.
            self._low_obstacle_group = ReentrantCallbackGroup()
            now = self._now_seconds()
            self._supervisor = CollisionStopSupervisor(self._config, now=now)
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._last_event = "STARTUP_WAITING_FOR_SCAN"

            # Camera low-obstacle brake (additive; can only slow/stop forward motion,
            # never speeds it up, never touches reverse/rotation, and only acts on a
            # fresh cloud so a dead camera leaves the lidar behaviour unchanged).
            self._lowobs_enable = bool(self.get_parameter("low_obstacle_brake_enable").value)
            self._lowobs_half_angle = math.radians(float(self.get_parameter("low_obstacle_half_angle_deg").value))
            self._lowobs_stop_m = float(self.get_parameter("low_obstacle_stop_distance_m").value)
            self._lowobs_slow_m = float(self.get_parameter("low_obstacle_slow_distance_m").value)
            self._lowobs_min_r = float(self.get_parameter("low_obstacle_min_range_m").value)
            self._lowobs_max_r = float(self.get_parameter("low_obstacle_max_range_m").value)
            self._lowobs_min_scale = float(self.get_parameter("low_obstacle_min_forward_scale").value)
            self._lowobs_max_age = float(self.get_parameter("low_obstacle_max_age_s").value)
            self._lowobs_swept = bool(self.get_parameter("low_obstacle_swept_path").value)
            self._lowobs_half_width = float(self.get_parameter("low_obstacle_half_width_m").value)
            self._lowobs_lock = threading.Lock()
            self._lowobs_points: list = []
            self._lowobs_stamp: Optional[float] = None
            self._lowobs_nearest: Optional[float] = None
            self._lowobs_scale: float = 1.0

            # HOLD-ON-VANISH (D39). Both operands are DERIVED here, not configured:
            # the band edge from the shipped sensor geometry, the closure allowance
            # from this brake's own freshness contract. A YAML knob for either would
            # let the deployed config disagree with the sensor it describes.
            self._lowobs_hold_enable = bool(
                self.get_parameter("low_obstacle_hold_on_vanish_enable").value)
            self._lowobs_odom_frame = str(self.get_parameter("odom_frame").value)
            band_outer_m = blind_band_outer_range_m(TofConfig())
            one_frame_closure_m = (
                float(self.get_parameter("max_forward_mps").value) * self._lowobs_max_age)
            self._lowobs_hold = BlindBandHold(
                band_outer_m, one_frame_closure_m, self._lowobs_half_width,
                self._lowobs_min_r, self._lowobs_max_r,
            )
            self._lowobs_hold_pose: Optional[Transform2D] = None
            self._lowobs_hold_active: bool = False
            self._lowobs_hold_reason: str = "clear"
            self.get_logger().info(
                f"low-obstacle hold-on-vanish {'ENABLED' if self._lowobs_hold_enable else 'disabled'}: "
                f"band_outer={band_outer_m:.4f} m (derived), "
                f"one_frame_closure={one_frame_closure_m:.4f} m "
                f"(max_forward_mps x low_obstacle_max_age_s), "
                f"arms below {band_outer_m + one_frame_closure_m:.4f} m"
            )

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
            self.create_subscription(
                LaserScan,
                scan_topic,
                self._on_scan,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("low_obstacle_topic").value),
                self._on_low_obstacle_cloud,
                qos_profile_sensor_data,
                callback_group=self._low_obstacle_group,
            )

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
                # Camera low-obstacle brake (node-level, additive to the lidar core).
                "low_obstacle_brake_enable": True,
                "low_obstacle_topic": "/camera/low_obstacles",
                "low_obstacle_stop_distance_m": 0.50,  # >= camera near-vision limit (~0.45 m)
                "low_obstacle_slow_distance_m": 0.70,
                "low_obstacle_half_angle_deg": 25.0,
                "low_obstacle_min_range_m": 0.40,  # below this the monocular detector is unreliable
                "low_obstacle_max_range_m": 1.20,
                "low_obstacle_min_forward_scale": 0.60,  # slow-band floor; avoid sub-breakaway creep
                "low_obstacle_max_age_s": 0.6,  # stale cloud -> no camera limit (lidar-only)
                # Swept-path check: use the arc actually commanded instead of a fixed
                # cone. half_width is the robot half-width plus a little margin.
                "low_obstacle_swept_path": True,
                "low_obstacle_half_width_m": 0.16,
                # HOLD-ON-VANISH (D39). Default TRUE: Scott's bar is that contact with
                # anything the rangefinder detected or could have detected is a defect,
                # and this brake's fail direction is over-caution. The band edge and the
                # closure allowance are DERIVED at construction, so there is deliberately
                # no knob for either -- only for whether the mechanism runs at all.
                "low_obstacle_hold_on_vanish_enable": True,
                # Frame the hold measures its own motion against. Only ever read through
                # TF: a belief transported by COMMANDED motion would be retired by moves
                # the arbiter refused, which is D40's defect relocated into the brake.
                "odom_frame": "odom",
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

        def _on_low_obstacle_cloud(self, msg):
            try:
                pts = [
                    (float(p[0]), float(p[1]))
                    for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
                ]
            except Exception:
                pts = []
            with self._lowobs_lock:
                self._lowobs_points = pts
                self._lowobs_stamp = self._now_seconds()

        def _low_obstacle_blocks_pivot(self, pts, cloud_age):
            """True when a FRESH camera cloud has a point inside the swept circle.

            Same geometry as the supervisor's pivot gate: the circumscribed corner
            radius of the footprint, because a pivot sweeps that circle in every
            direction. Fails OPEN on a stale or absent cloud, exactly like the forward
            camera brake -- a dead camera must degrade to lidar-only behaviour, never
            freeze the rover.

            Takes the cloud snapshot as arguments rather than re-reading it, so the
            caller can report the SAME snapshot in telemetry: an age read twice can
            straddle a cloud arrival, and then the state line disagrees with what the
            veto actually saw.
            """
            if not self._lowobs_enable:
                return False
            if cloud_age is None or cloud_age > self._lowobs_max_age or not pts:
                return False
            radius = math.hypot(
                max(float(self.get_parameter("footprint_front_m").value),
                    float(self.get_parameter("footprint_rear_m").value)),
                max(float(self.get_parameter("footprint_left_m").value),
                    float(self.get_parameter("footprint_right_m").value)),
            ) + float(self.get_parameter("payload_margin_m").value)
            for px, py in ((p[0], p[1]) for p in pts):
                if math.hypot(px, py) <= radius:
                    return True
            return False

        def _apply_low_obstacle_brake(self, linear_x, angular_z, now):
            """Additive forward limit from a FRESH sub-lidar obstacle cloud. Returns
            (limited_linear_x, nearest_m, scale, considered_points). Only reduces
            positive forward speed; reverse/zero and a stale/absent cloud pass
            through unchanged (lidar-only).

            Checks the arc the rover is actually about to drive, not a fixed cone
            ahead: turning, a differential drive pivots about a centre off to one
            side, so the flank leads and sweeps ground the nose never covers. A
            straight-ahead cone reports CLEAR while the flank hits a chair leg.

            `considered_points` is the count AFTER the range window and the swept
            filter, and it is returned rather than recomputed by the caller so it can
            never describe a different set from the `nearest` beside it. `None` means
            the brake did not look at all -- disabled, not driving forward, or no
            fresh cloud -- which is a different fact from looking and finding zero.
            """
            if not self._lowobs_enable:
                return linear_x, None, 1.0, None
            with self._lowobs_lock:
                pts = self._lowobs_points
                stamp = self._lowobs_stamp
            fresh = stamp is not None and (now - stamp) <= self._lowobs_max_age

            # THE HOLD RUNS ON EVERY CYCLE, INCLUDING THE ONES THE BRAKE SITS OUT.
            # The three early returns this replaced -- reverse, zero command, stale
            # cloud -- are exactly the cycles during which a held belief must be
            # carried and retired. Skipping them would mean a belief could only be
            # cleared while driving forward INTO the thing it represents, and a stale
            # cloud would release the hold, which is the released-into-contact defect
            # rebuilt out of a freshness check.
            hold = None
            if self._lowobs_hold_enable and self._lowobs_swept:
                hold = self._lowobs_hold.update(
                    pts if fresh else [], fresh, linear_x, angular_z,
                    self._pose_delta_since_last(),
                )
                self._lowobs_hold_active = hold.active
                self._lowobs_hold_reason = hold.reason

            if linear_x <= 0.0:
                return linear_x, None, 1.0, None
            if not fresh and hold is None:
                return linear_x, None, 1.0, None

            if self._lowobs_swept:
                considered = points_in_swept_path(
                    pts, linear_x, angular_z, self._lowobs_half_width,
                    self._lowobs_min_r, self._lowobs_max_r,
                ) if fresh else None
                nearest = hold.nearest_m if hold is not None else swept_path_obstacle(
                    pts, linear_x, angular_z, self._lowobs_half_width,
                    self._lowobs_min_r, self._lowobs_max_r,
                )
            elif fresh:
                nearest = nearest_forward_obstacle(
                    pts, self._lowobs_half_angle, self._lowobs_min_r, self._lowobs_max_r
                )
                considered = None
            else:
                return linear_x, None, 1.0, None
            scale = forward_speed_scale(nearest, self._lowobs_stop_m, self._lowobs_slow_m, self._lowobs_min_scale)
            return linear_x * scale, nearest, scale, considered

        def _pose_delta_since_last(self) -> Optional[tuple]:
            """Measured motion since the previous call as `(dx, dy, dyaw)` in the
            PREVIOUS base_link frame, or None when it could not be measured.

            None is not an error path to be smoothed over -- it is the input that makes
            the hold refuse to retire a belief. A gap in TF means the rover's own
            displacement is unknown, and a belief that cannot be placed cannot be shown
            to be gone. Returning a zero delta instead would silently assert the rover
            had not moved, which is a claim nothing measured.
            """
            # TIMEOUT ZERO, DELIBERATELY, AND IT IS NOT THE SCAN PATH'S 0.05 s.
            # This asks for the LATEST available transform, not one at a particular
            # stamp, so a timeout can only ever be spent in the case where no transform
            # exists at all -- precisely when waiting buys nothing. And this runs inside
            # `_state_lock` on every publish cycle, against a 0.10 s timer: blocking
            # 0.05 s per cycle on a dead TF tree would eat half the arbitration budget
            # to learn something the failure path already handles. Fail fast, and the
            # hold holds -- the safe direction costs nothing here.
            try:
                stamped = self._tf_buffer.lookup_transform(
                    self._lowobs_odom_frame, self._config.base_frame, Time(),
                    timeout=Duration(seconds=0.0),
                )
            except Exception:
                self._lowobs_hold_pose = None
                return None
            pose = _transform2d_from_transform_stamped(stamped)
            if not pose.is_finite():
                self._lowobs_hold_pose = None
                return None
            previous, self._lowobs_hold_pose = self._lowobs_hold_pose, pose
            if previous is None:
                # First fix after a gap: a delta needs two poses, and inventing a zero
                # here would claim the rover held still across the whole outage.
                return None
            wx, wy = pose.x - previous.x, pose.y - previous.y
            cos_y, sin_y = math.cos(previous.yaw), math.sin(previous.yaw)
            dyaw = math.atan2(math.sin(pose.yaw - previous.yaw),
                              math.cos(pose.yaw - previous.yaw))
            return (cos_y * wx + sin_y * wy, -sin_y * wx + cos_y * wy, dyaw)

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
            cam_linear, cam_nearest, cam_scale, cam_considered = (
                self._apply_low_obstacle_brake(
                    decision.output.linear_x, decision.output.angular_z,
                    self._now_seconds(),
                )
            )
            self._lowobs_nearest, self._lowobs_scale = cam_nearest, cam_scale
            msg.linear.x = cam_linear
            msg.angular.z = decision.output.angular_z
            # A PIVOT is the one motion the camera used to be blind to. The forward
            # brake early-returns on linear_x <= 0, and a pivot escape emits exactly
            # linear_x == 0, so the cloud was never even read -- while the lidar, at
            # 0.19 m, reports the wall behind a shoe rather than the shoe. That is the
            # precise failure class low_obstacle was built to prevent, re-opened for a
            # new motion type. A pivot sweeps the footprint's corner circle, so any
            # camera point inside that circle blocks the turn.
            # ONE snapshot of the cloud feeds both the veto and the telemetry, so
            # the state line reports exactly what the veto judged.
            with self._lowobs_lock:
                cam_pts = self._lowobs_points
                cam_stamp = self._lowobs_stamp
            cam_cloud_age = (
                None if cam_stamp is None else self._now_seconds() - cam_stamp
            )
            pivot_veto = False
            if msg.linear.x == 0.0 and msg.angular.z != 0.0:
                pivot_veto = self._low_obstacle_blocks_pivot(cam_pts, cam_cloud_age)
                if pivot_veto:
                    msg.angular.z = 0.0
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
                f"trajectory_moving_away_point_count={0 if decision.trajectory is None else decision.trajectory.moving_away_point_count} "
                f"requested=({decision.requested.linear_x:.3f},{decision.requested.angular_z:.3f}) "
                f"output=({decision.output.linear_x:.3f},{decision.output.angular_z:.3f}) "
                f"cam_nearest={_fmt_optional(cam_nearest)} cam_scale={cam_scale:.2f} "
                # HOW MANY POINTS THE BRAKE ACTUALLY CONSIDERED, after its range
                # window and swept-path filter. Absent until 2026-08-15, and its
                # absence cost two sessions: /tof/state's `obstacle_zones` counts
                # rule-B zones over the SENSOR'S WHOLE REACH, so "zones cycled 0->10
                # while cam_scale never left 1.00" read as detections being lost
                # between the sensor and the brake. They were not lost. They sat at
                # 0.54-1.56 m against a 0.60 m brake reach -- a correct number about
                # the wrong population, with no field on this line able to say so.
                # EMPTY means the brake did not look (disabled / not driving forward
                # / no fresh cloud); 0 means it looked and the swept path was clear.
                f"cam_considered={_fmt_optional(cam_considered)} "
                # WHETHER THIS FRAME'S LIMIT CAME FROM A SIGHTING OR A BELIEF, and
                # why. Without these, a hold is indistinguishable on the recording
                # from an ordinary stop, and the one question the next autopsy will
                # ask -- "was the brake looking at something, or remembering it?" --
                # would again be answerable only by replaying the bag through the
                # production code. `cam_hold_reason` names the specific silence:
                # vanished_in_band / held_no_look / held_no_pose / retired_sight_through,
                # with an `_off_path` suffix when a belief is held but not clamping.
                f"cam_hold_active={str(self._lowobs_hold_active).lower()} "
                f"cam_hold_reason={self._lowobs_hold_reason} "
                f"cam_output_linear={msg.linear.x:.3f} "
                # The published command differs from decision.output whenever the
                # pivot veto zeroed the turn, and the veto silently disengages when
                # the cloud goes stale -- measured happening while clouds were still
                # ARRIVING, because a saturated executor starved the cloud
                # subscription. A recording without these fields cannot distinguish
                # "camera vetoed the pivot" from "gate granted it", nor see the
                # fail-open window at all.
                f"pivot_veto={str(pivot_veto).lower()} "
                f"cam_cloud_age={_fmt_optional(None if cam_cloud_age is None else round(cam_cloud_age, 3))} "
                f"output_angular_published={msg.angular.z:.3f}"
            )
            self._state_pub.publish(state)
            event_text = f"{decision.state.value} {decision.reason}"
            if self._lowobs_enable and cam_scale < 1.0:
                event_text += f" +CAMERA_LOW_OBSTACLE_{'STOP' if cam_scale == 0.0 else 'SLOW'}@{cam_nearest:.2f}m"
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
    # 4, not 2: the arbitration path (scan + cmd_vel, mutually exclusive by
    # design and serialized on the state lock anyway), the camera cloud in its own
    # group, and the service/driver-client groups all need to make progress
    # concurrently. Two threads made the camera cloud compete with the very
    # callbacks that must never be delayed (D22).
    executor = MultiThreadedExecutor(num_threads=4)
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
