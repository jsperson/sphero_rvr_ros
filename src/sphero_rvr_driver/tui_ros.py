"""ROS 2 client wrapper for the Sphero RVR terminal UI.

Imports ROS 2 modules lazily so non-ROS unit tests can import the TUI helpers.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional


@dataclass
class RVRStatus:
    connected: bool = False
    emergency_stopped: bool = False
    diagnostic_message: str = "waiting for diagnostics"
    battery_percentage: Optional[float] = None
    battery_voltage: Optional[float] = None


class RVRROSClient:
    """Small ROS-facing API used by the curses TUI."""

    def __init__(self):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from sensor_msgs.msg import BatteryState
        from std_srvs.srv import Trigger

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
        self._Twist = Twist
        self._Trigger = Trigger
        self.status = RVRStatus()
        self._node: Node = rclpy.create_node("sphero_rvr_tui")
        self._cmd_pub = self._node.create_publisher(Twist, "cmd_vel", 10)
        self._battery_sub = self._node.create_subscription(BatteryState, "battery_state", self._on_battery, 10)
        self._diagnostics_sub = self._node.create_subscription(DiagnosticArray, "diagnostics", self._on_diagnostics, 10)
        self._stop_client = self._node.create_client(Trigger, "stop")
        self._estop_client = self._node.create_client(Trigger, "estop")
        self._clear_estop_client = self._node.create_client(Trigger, "clear_estop")
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._running = False

    def start(self) -> None:
        self._running = True
        self._spin_thread.start()

    def close(self) -> None:
        try:
            self.stop(timeout_sec=1.0)
        except Exception:
            pass
        self._running = False
        self._rclpy.shutdown()
        self._spin_thread.join(timeout=2)

    def publish_velocity(self, linear_mps: float, angular_rad_s: float) -> None:
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

    def _call_trigger(self, client, timeout_sec: float) -> str:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(f"service {client.srv_name} is unavailable")
        future = client.call_async(self._Trigger.Request())
        deadline = timeout_sec
        while not future.done() and deadline > 0:
            self._rclpy.spin_once(self._node, timeout_sec=0.05)
            deadline -= 0.05
        if not future.done():
            raise TimeoutError(f"service {client.srv_name} timed out")
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response.message

    def _on_battery(self, msg) -> None:
        self.status.battery_percentage = None if msg.percentage < 0 else float(msg.percentage)
        self.status.battery_voltage = None if msg.voltage == 0.0 else float(msg.voltage)

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
