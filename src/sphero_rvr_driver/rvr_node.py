"""ROS 2 node wrapper for the concurrency-safe RVR driver.

This module avoids importing rclpy at package import time so the core test suite
can run on machines without ROS 2 installed. Run it inside a ROS 2 environment.
"""

from __future__ import annotations

import asyncio
import threading
from dataclasses import dataclass
from typing import Optional

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.serial_transport import SerialTransport
from sphero_rvr_core.transport import Transport
from .diagnostics import BatterySnapshot, battery_state_fields, summarize_state
from .twist_mapper import TwistLike, map_twist_to_velocity


@dataclass(frozen=True)
class RVRNodeConfig:
    serial_port: str = "/dev/ttyAMA0"
    baud_rate: int = 115200
    cmd_vel_timeout: float = 0.5
    control_period: float = 0.05
    max_linear_mps: float = 0.25
    max_angular_rad_s: float = 0.4
    max_raw_motor_duty: int = 96
    battery_publish_period: float = 5.0
    diagnostics_publish_period: float = 1.0


def create_driver(config: RVRNodeConfig, transport: Optional[Transport] = None) -> RVRDriver:
    """Build the core driver from ROS-node configuration.

    Kept importable/testable without ROS 2 so safety defaults do not regress.
    """
    if transport is None:
        transport = SerialTransport(port=config.serial_port, baud_rate=config.baud_rate)
    return RVRDriver(
        transport=transport,
        control_period=config.control_period,
        command_timeout=config.cmd_vel_timeout,
        max_linear_mps=config.max_linear_mps,
        max_angular_rad_s=config.max_angular_rad_s,
        max_raw_motor_duty=config.max_raw_motor_duty,
    )


class AsyncDriverThread:
    """Run the async driver loop behind ROS 2's synchronous callback model."""

    def __init__(self, driver: RVRDriver):
        self.driver = driver
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)

    def start(self) -> None:
        self.thread.start()
        self.run(self.driver.connect()).result(timeout=10)

    def stop(self) -> None:
        try:
            self.run(self.driver.disconnect()).result(timeout=10)
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(timeout=5)

    def run(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()


def main(args=None):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState
    from std_srvs.srv import Trigger

    class SpheroRVRNode(Node):
        def __init__(self):
            super().__init__("sphero_rvr_driver")
            self._declare_parameters()
            self._config = self._read_config()
            self._driver_thread = AsyncDriverThread(create_driver(self._config))
            self._driver_thread.start()

            self._battery_future = None
            self._battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
            self._diagnostics_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)

            self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
            self.create_service(Trigger, "stop", self._on_stop)
            self.create_service(Trigger, "estop", self._on_estop)
            self.create_service(Trigger, "clear_estop", self._on_clear_estop)
            self.create_timer(self._config.battery_publish_period, self._poll_battery)
            self.create_timer(self._config.diagnostics_publish_period, self._publish_diagnostics)

        def _declare_parameters(self):
            defaults = RVRNodeConfig()
            self.declare_parameter("serial_port", defaults.serial_port)
            self.declare_parameter("baud_rate", defaults.baud_rate)
            self.declare_parameter("cmd_vel_timeout", defaults.cmd_vel_timeout)
            self.declare_parameter("control_period", defaults.control_period)
            self.declare_parameter("max_linear_mps", defaults.max_linear_mps)
            self.declare_parameter("max_angular_rad_s", defaults.max_angular_rad_s)
            self.declare_parameter("max_raw_motor_duty", defaults.max_raw_motor_duty)
            self.declare_parameter("battery_publish_period", defaults.battery_publish_period)
            self.declare_parameter("diagnostics_publish_period", defaults.diagnostics_publish_period)

        def _read_config(self) -> RVRNodeConfig:
            return RVRNodeConfig(
                serial_port=str(self.get_parameter("serial_port").value),
                baud_rate=int(self.get_parameter("baud_rate").value),
                cmd_vel_timeout=float(self.get_parameter("cmd_vel_timeout").value),
                control_period=float(self.get_parameter("control_period").value),
                max_linear_mps=float(self.get_parameter("max_linear_mps").value),
                max_angular_rad_s=float(self.get_parameter("max_angular_rad_s").value),
                max_raw_motor_duty=int(self.get_parameter("max_raw_motor_duty").value),
                battery_publish_period=float(self.get_parameter("battery_publish_period").value),
                diagnostics_publish_period=float(self.get_parameter("diagnostics_publish_period").value),
            )

        def destroy_node(self):
            self._driver_thread.stop()
            super().destroy_node()

        def _on_cmd_vel(self, msg):
            velocity = map_twist_to_velocity(
                TwistLike(linear_x=float(msg.linear.x), angular_z=float(msg.angular.z)),
                max_linear_mps=self._config.max_linear_mps,
                max_angular_rad_s=self._config.max_angular_rad_s,
            )
            future = self._driver_thread.run(
                self._driver_thread.driver.set_velocity(velocity.linear_mps, velocity.angular_rad_s)
            )
            future.add_done_callback(lambda fut: self._log_future_exception(fut, "cmd_vel"))

        def _on_stop(self, request, response):
            self._driver_thread.run(self._driver_thread.driver.stop()).result(timeout=5)
            response.success = True
            response.message = "RVR stopped"
            return response

        def _on_estop(self, request, response):
            self._driver_thread.run(self._driver_thread.driver.emergency_stop()).result(timeout=5)
            response.success = True
            response.message = "RVR emergency stop active"
            return response

        def _on_clear_estop(self, request, response):
            self._driver_thread.run(self._driver_thread.driver.clear_emergency_stop()).result(timeout=5)
            response.success = True
            response.message = "RVR emergency stop cleared"
            return response

        def _poll_battery(self):
            if self._battery_future is not None and not self._battery_future.done():
                return
            self._battery_future = self._driver_thread.run(self._read_battery_snapshot())
            self._battery_future.add_done_callback(self._publish_battery_from_future)

        async def _read_battery_snapshot(self) -> BatterySnapshot:
            percentage = await self._driver_thread.driver.get_battery_percentage()
            voltage = None
            try:
                voltage = await self._driver_thread.driver.get_battery_voltage()
            except Exception as exc:  # voltage is nice-to-have; percent keeps topic alive.
                self.get_logger().warn(f"battery voltage query failed: {exc}")
            return BatterySnapshot(percentage=percentage, voltage=voltage)

        def _publish_battery_from_future(self, future):
            try:
                fields = battery_state_fields(future.result())
            except Exception as exc:
                self.get_logger().warn(f"battery query failed: {exc}")
                return

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.present = fields.present
            msg.percentage = fields.percentage
            msg.voltage = fields.voltage
            self._battery_pub.publish(msg)

        def _publish_diagnostics(self):
            state = self._driver_thread.driver.get_state()
            summary = summarize_state(state)
            status = DiagnosticStatus()
            status.name = "sphero_rvr_driver"
            status.hardware_id = "sphero_rvr"
            status.level = {"OK": DiagnosticStatus.OK, "WARN": DiagnosticStatus.WARN, "ERROR": DiagnosticStatus.ERROR}[
                summary.level
            ]
            status.message = summary.message
            status.values = [
                KeyValue(key="connected", value=str(state.connected).lower()),
                KeyValue(key="emergency_stopped", value=str(state.emergency_stopped).lower()),
            ]
            if state.latest_velocity is not None:
                status.values.extend(
                    [
                        KeyValue(key="linear_mps", value=f"{state.latest_velocity.linear_mps:.3f}"),
                        KeyValue(key="angular_rad_s", value=f"{state.latest_velocity.angular_rad_s:.3f}"),
                    ]
                )

            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status = [status]
            self._diagnostics_pub.publish(msg)

        def _log_future_exception(self, future, label: str):
            try:
                future.result()
            except Exception as exc:
                self.get_logger().error(f"{label} command failed: {exc}")

    rclpy.init(args=args)
    node = SpheroRVRNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
