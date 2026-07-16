"""ROS 2 client wrapper for the Sphero RVR terminal UI.

Imports ROS 2 modules lazily so non-ROS unit tests can import the TUI helpers.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class RVRStatus:
    connected: bool = False
    emergency_stopped: bool = False
    diagnostic_message: str = "waiting for diagnostics"
    battery_percentage: Optional[float] = None
    battery_voltage: Optional[float] = None
    mode: str = "live"
    battery_received_at: Optional[float] = None
    odom_received_at: Optional[float] = None
    odom_x: Optional[float] = None
    odom_y: Optional[float] = None
    odom_yaw: Optional[float] = None
    odom_distance_m: Optional[float] = None
    scan_received_at: Optional[float] = None
    scan_range_count: int = 0
    scan_valid_count: int = 0
    scan_min_range: Optional[float] = None
    scan_max_range: Optional[float] = None
    collision_stop_state: str = "waiting"
    collision_stop_reason: str = "waiting"
    collision_stop_received_at: Optional[float] = None
    collision_stop_fresh: bool = False
    collision_stop_reset_required: bool = False
    cmd_vel_available: bool = False
    cmd_vel_publisher_count: int = 0
    service_available: Dict[str, bool] = field(default_factory=dict)
    tf_available: Dict[str, Optional[bool]] = field(default_factory=dict)

FRESH_SECONDS = 2.0
STATUS_SERVICES = ("/stop", "/estop", "/clear_estop")
STATUS_TF_FRAMES = (
    ("odom", "base_link"),
    ("base_link", "laser"),
    ("map", "odom"),
)


class DryRunRVRClient:
    """ROS-free fake client used to exercise the TUI without robot hardware."""

    def __init__(self):
        now = time.monotonic()
        self.status = RVRStatus(
            connected=True,
            diagnostic_message="DRY RUN: fake ROS surfaces active",
            battery_percentage=0.87,
            battery_voltage=7.8,
            mode="dry-run",
            battery_received_at=now,
            odom_received_at=now,
            odom_x=0.0,
            odom_y=0.0,
            odom_yaw=0.0,
            odom_distance_m=0.0,
            scan_received_at=now,
            scan_range_count=360,
            scan_valid_count=360,
            scan_min_range=0.42,
            scan_max_range=6.0,
            collision_stop_state="CLEAR",
            collision_stop_reason="dry_run_clear",
            collision_stop_received_at=now,
            collision_stop_fresh=True,
            cmd_vel_available=False,
            cmd_vel_publisher_count=0,
            service_available={name: True for name in STATUS_SERVICES},
            tf_available={f"{parent}->{child}": True for parent, child in STATUS_TF_FRAMES},
        )
        self.published_commands: list[tuple[float, float]] = []

    def start(self) -> None:
        self.status.diagnostic_message = "DRY RUN: fake ROS surfaces active"

    def close(self) -> None:
        return None

    def publish_velocity(self, linear_mps: float, angular_rad_s: float) -> None:
        self.published_commands.append((float(linear_mps), float(angular_rad_s)))

    def stop(self, timeout_sec: float = 2.0) -> str:
        self.publish_velocity(0.0, 0.0)
        return "DRY-RUN stop: would publish zero /cmd_vel and call /stop"

    def estop(self, timeout_sec: float = 2.0) -> str:
        self.status.emergency_stopped = True
        self.publish_velocity(0.0, 0.0)
        return "DRY-RUN estop: would call /estop"

    def clear_estop(self, timeout_sec: float = 2.0) -> str:
        self.status.emergency_stopped = False
        return "DRY-RUN clear-estop: would call /clear_estop"


def update_battery_status(status: RVRStatus, msg, now: Optional[float] = None) -> None:
    status.battery_percentage = None if msg.percentage < 0 else float(msg.percentage)
    status.battery_voltage = None if msg.voltage == 0.0 else float(msg.voltage)
    status.battery_received_at = time.monotonic() if now is None else now


def update_odom_status(status: RVRStatus, msg, now: Optional[float] = None) -> None:
    pose = msg.pose.pose
    status.odom_x = float(pose.position.x)
    status.odom_y = float(pose.position.y)
    status.odom_yaw = _yaw_from_quaternion(pose.orientation)
    status.odom_distance_m = math.hypot(status.odom_x, status.odom_y)
    status.odom_received_at = time.monotonic() if now is None else now


def update_scan_status(status: RVRStatus, msg, now: Optional[float] = None) -> None:
    ranges = list(getattr(msg, "ranges", []) or [])
    valid = [float(value) for value in ranges if math.isfinite(value)]
    status.scan_range_count = len(ranges)
    status.scan_valid_count = len(valid)
    status.scan_min_range = min(valid) if valid else None
    status.scan_max_range = max(valid) if valid else None
    status.scan_received_at = time.monotonic() if now is None else now


def update_collision_stop_status(status: RVRStatus, msg, now: Optional[float] = None) -> None:
    text = str(getattr(msg, "data", ""))
    parts = text.split()
    status.collision_stop_state = parts[0] if parts else "waiting"
    reason = "waiting"
    for part in parts[1:]:
        if part.startswith("reason="):
            reason = part.split("=", 1)[1]
            break
    status.collision_stop_reason = reason
    status.collision_stop_received_at = time.monotonic() if now is None else now
    status.collision_stop_fresh = status.collision_stop_state in {"CLEAR", "SLOW"}
    status.collision_stop_reset_required = "reset_required" in text


def collision_stop_allows_motion(status: RVRStatus, now: Optional[float] = None) -> bool:
    current_time = time.monotonic() if now is None else now
    received_at = getattr(status, "collision_stop_received_at", None)
    fresh = received_at is not None and current_time - received_at <= FRESH_SECONDS
    return fresh and getattr(status, "collision_stop_state", "") in {"CLEAR", "SLOW"}


def format_status_lines(
    status: RVRStatus,
    *,
    armed: bool,
    speed: float,
    turn: float,
    now: Optional[float] = None,
) -> list[str]:
    current_time = time.monotonic() if now is None else now
    service_available = getattr(status, "service_available", {}) or {}
    tf_available = getattr(status, "tf_available", {}) or {}
    cmd_vel_publisher_count = getattr(status, "cmd_vel_publisher_count", 0)
    return [
        "RVR Control Console",
        "────────────────────────────────────────",
        f"Hardware mode: {getattr(status, 'mode', 'live')}",
        (
            f"RVR driver: {_presence_text(getattr(status, 'connected', False))}    "
            f"/cmd_vel: {_cmd_vel_text(getattr(status, 'cmd_vel_available', False), cmd_vel_publisher_count)}"
        ),
        _battery_line(status, current_time),
        _odom_line(status, current_time),
        _scan_line(status, current_time),
        _collision_stop_line(status, current_time),
        (
            "Services: "
            + "  ".join(f"{name} {_availability_text(service_available.get(name, False))}" for name in STATUS_SERVICES)
        ),
        "TF: " + "  ".join(_tf_text(tf_available, parent, child) for parent, child in STATUS_TF_FRAMES),
        f"Armed: {armed}    Estop: {getattr(status, 'emergency_stopped', False)}",
        f"Speed: {speed:.2f} m/s    Turn: {turn:.2f} rad/s",
        f"Diagnostics: {getattr(status, 'diagnostic_message', 'waiting for diagnostics')}",
    ]


def _yaw_from_quaternion(quat) -> float:
    x = float(getattr(quat, "x", 0.0))
    y = float(getattr(quat, "y", 0.0))
    z = float(getattr(quat, "z", 0.0))
    w = float(getattr(quat, "w", 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _presence_text(present: bool) -> str:
    return "present" if present else "missing"


def _availability_text(available: bool) -> str:
    return "ok" if available else "missing"


def _cmd_vel_text(available: bool, publisher_count: int) -> str:
    exposure = "available" if available else "not-exposed"
    return f"{exposure} (publishers={publisher_count})"


def freshness_text(received_at: Optional[float], now: float) -> str:
    if received_at is None:
        return "waiting"
    age = max(0.0, now - received_at)
    freshness = "fresh" if age <= FRESH_SECONDS else "stale"
    return f"{freshness} {age:.1f}s"


def _battery_line(status: RVRStatus, now: float) -> str:
    parts = []
    percentage = getattr(status, "battery_percentage", None)
    voltage = getattr(status, "battery_voltage", None)
    if percentage is not None:
        parts.append(f"{percentage * 100:.0f}%")
    if voltage is not None:
        parts.append(f"{voltage:.2f} V")
    value = " / ".join(parts) if parts else "waiting"
    return f"Battery: {value} ({freshness_text(getattr(status, 'battery_received_at', None), now)})"


def _odom_line(status: RVRStatus, now: float) -> str:
    if getattr(status, "odom_received_at", None) is None:
        return "Odom: waiting"
    freshness = freshness_text(status.odom_received_at, now)
    return (
        f"Odom: {freshness} "
        f"pose=({status.odom_x:.2f}, {status.odom_y:.2f}, yaw={status.odom_yaw:.2f}) "
        f"distance={status.odom_distance_m:.2f} m"
    )


def _scan_line(status: RVRStatus, now: float) -> str:
    if getattr(status, "scan_received_at", None) is None:
        return "Scan: waiting"
    freshness = freshness_text(status.scan_received_at, now)
    line = (
        f"Scan: {freshness} "
        f"ranges={status.scan_range_count} valid={status.scan_valid_count}"
    )
    if status.scan_min_range is not None and status.scan_max_range is not None:
        line += f" min={status.scan_min_range:.2f} m max={status.scan_max_range:.2f} m"
    return line


def _collision_stop_line(status: RVRStatus, now: float) -> str:
    freshness = freshness_text(getattr(status, "collision_stop_received_at", None), now)
    return (
        f"Collision stop: {getattr(status, 'collision_stop_state', 'waiting')} "
        f"reason={getattr(status, 'collision_stop_reason', 'waiting')} {freshness}"
    )


def _tf_text(tf_available: Dict[str, Optional[bool]], parent: str, child: str) -> str:
    key = f"{parent}->{child}"
    available = tf_available.get(key)
    if available is None:
        state = "waiting"
    elif available:
        state = "ok"
    else:
        state = "missing"
    return f"{key} {state}"


class RVRROSClient:
    """Small ROS-facing API used by the curses TUI."""

    def __init__(self):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from sensor_msgs.msg import BatteryState, LaserScan
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
        self._Twist = Twist
        self._Trigger = Trigger
        self.status = RVRStatus()
        self._node: Node = rclpy.create_node("sphero_rvr_tui")
        self._tf_buffer = None
        try:
            import tf2_ros

            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        except Exception:
            self._tf_listener = None
        self._cmd_pub = None
        self._battery_sub = self._node.create_subscription(BatteryState, "battery_state", self._on_battery, 10)
        self._diagnostics_sub = self._node.create_subscription(DiagnosticArray, "diagnostics", self._on_diagnostics, 10)
        self._odom_sub = self._node.create_subscription(Odometry, "odom", self._on_odom, 10)
        self._scan_sub = self._node.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self._collision_stop_sub = self._node.create_subscription(String, "/collision_stop/state", self._on_collision_stop, 10)
        self._stop_client = self._node.create_client(Trigger, "stop")
        self._estop_client = self._node.create_client(Trigger, "estop")
        self._clear_estop_client = self._node.create_client(Trigger, "clear_estop")
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._running = False

    def start(self) -> None:
        self._running = True
        self._spin_thread.start()

    def close(self) -> None:
        self._running = False
        self.disable_velocity_publisher()
        self._rclpy.shutdown()
        self._spin_thread.join(timeout=2)

    @property
    def velocity_publisher_enabled(self) -> bool:
        return self._cmd_pub is not None

    def enable_velocity_publisher(self):
        if self._cmd_pub is None:
            self._cmd_pub = self._node.create_publisher(self._Twist, "cmd_vel", 10)
        self.status.cmd_vel_available = True
        self.status.cmd_vel_publisher_count = int(self._node.count_publishers("/cmd_vel"))
        return self._cmd_pub

    def disable_velocity_publisher(self) -> None:
        if self._cmd_pub is None:
            self.status.cmd_vel_available = False
            self.status.cmd_vel_publisher_count = 0
            return
        destroy = getattr(self._node, "destroy_publisher", None)
        if destroy is not None:
            destroy(self._cmd_pub)
        self._cmd_pub = None
        self.status.cmd_vel_available = False
        self.status.cmd_vel_publisher_count = 0

    def publish_velocity(self, linear_mps: float, angular_rad_s: float) -> None:
        if self._cmd_pub is None:
            if abs(linear_mps) < 1e-9 and abs(angular_rad_s) < 1e-9:
                self.status.diagnostic_message = "cmd_vel publisher not enabled; zero velocity skipped"
                return
            raise RuntimeError(
                "cmd_vel publisher is not enabled (velocity publisher is not enabled; use motor-capable mapping first)"
            )
        msg = self._Twist()
        msg.linear.x = float(linear_mps)
        msg.angular.z = float(angular_rad_s)
        self._cmd_pub.publish(msg)

    def stop(self, timeout_sec: float = 2.0) -> str:
        return self._call_trigger(self._stop_client, timeout_sec=timeout_sec)

    def estop(self, timeout_sec: float = 2.0) -> str:
        return self._call_trigger(self._estop_client, timeout_sec=timeout_sec)

    def clear_estop(self, timeout_sec: float = 2.0) -> str:
        return self._call_trigger(self._clear_estop_client, timeout_sec=timeout_sec)

    def _spin(self) -> None:
        while self._running:
            self._rclpy.spin_once(self._node, timeout_sec=0.1)
            self._refresh_graph_status()

    def _call_trigger(self, client, timeout_sec: float) -> str:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(f"service {client.srv_name} is unavailable")
        future = client.call_async(self._Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            # The ROS subscriptions/status path is already handled by the background
            # spin thread. Spinning this same node again here can raise
            # RuntimeError("Executor is already spinning") in live TUI use; wait for
            # the background spinner to service the future instead.
            time.sleep(0.01)
        if not future.done():
            raise TimeoutError(f"service {client.srv_name} timed out")
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response.message

    def _on_battery(self, msg) -> None:
        update_battery_status(self.status, msg)

    def _on_odom(self, msg) -> None:
        update_odom_status(self.status, msg)

    def _on_scan(self, msg) -> None:
        update_scan_status(self.status, msg)

    def _on_collision_stop(self, msg) -> None:
        update_collision_stop_status(self.status, msg)

    def _on_diagnostics(self, msg) -> None:
        if not msg.status:
            return
        status = msg.status[0]
        self.status.diagnostic_message = status.message
        for item in status.values:
            if item.key == "connected":
                self.status.connected = item.value.lower() == "true"
            elif item.key == "emergency_stopped":
                self.status.emergency_stopped = item.value.lower() == "true"

    def _refresh_graph_status(self) -> None:
        try:
            topics = {name for name, _types in self._node.get_topic_names_and_types()}
            self.status.cmd_vel_available = self.velocity_publisher_enabled and "/cmd_vel" in topics
            self.status.cmd_vel_publisher_count = (
                int(self._node.count_publishers("/cmd_vel")) if self.velocity_publisher_enabled else 0
            )
            self.status.service_available = {
                "/stop": self._stop_client.service_is_ready(),
                "/estop": self._estop_client.service_is_ready(),
                "/clear_estop": self._clear_estop_client.service_is_ready(),
            }
            if self._tf_buffer is not None:
                self.status.tf_available = {
                    f"{parent}->{child}": bool(
                        self._tf_buffer.can_transform(parent, child, self._rclpy.time.Time())
                    )
                    for parent, child in STATUS_TF_FRAMES
                }
        except Exception as exc:
            self.status.diagnostic_message = f"status refresh failed: {exc}"
