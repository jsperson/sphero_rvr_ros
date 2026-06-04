"""ROS 2 node wrapper for the concurrency-safe RVR driver.

This module avoids importing rclpy at package import time so the core test suite
can run on machines without ROS 2 installed. Run it inside a ROS 2 environment.
"""

import asyncio
import threading

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.serial_transport import SerialTransport
from .twist_mapper import TwistLike, map_twist_to_velocity


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
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    class SpheroRVRNode(Node):
        def __init__(self):
            super().__init__("sphero_rvr_driver")
            self.declare_parameter("serial_port", "/dev/ttyAMA0")
            self.declare_parameter("baud_rate", 115200)
            self.declare_parameter("cmd_vel_timeout", 0.5)
            self.declare_parameter("control_period", 0.05)
            self.declare_parameter("max_linear_mps", 0.5)
            self.declare_parameter("max_angular_rad_s", 2.0)

            max_linear = float(self.get_parameter("max_linear_mps").value)
            max_angular = float(self.get_parameter("max_angular_rad_s").value)
            transport = SerialTransport(
                port=str(self.get_parameter("serial_port").value),
                baud_rate=int(self.get_parameter("baud_rate").value),
            )
            self._driver_thread = AsyncDriverThread(
                RVRDriver(
                    transport=transport,
                    control_period=float(self.get_parameter("control_period").value),
                    command_timeout=float(self.get_parameter("cmd_vel_timeout").value),
                    max_linear_mps=max_linear,
                    max_angular_rad_s=max_angular,
                )
            )
            self._max_linear = max_linear
            self._max_angular = max_angular
            self._driver_thread.start()

            self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
            self.create_service(Trigger, "stop", self._on_stop)
            self.create_service(Trigger, "estop", self._on_estop)
            self.create_service(Trigger, "clear_estop", self._on_clear_estop)

        def destroy_node(self):
            self._driver_thread.stop()
            super().destroy_node()

        def _on_cmd_vel(self, msg):
            velocity = map_twist_to_velocity(
                TwistLike(linear_x=float(msg.linear.x), angular_z=float(msg.angular.z)),
                max_linear_mps=self._max_linear,
                max_angular_rad_s=self._max_angular,
            )
            self._driver_thread.run(
                self._driver_thread.driver.set_velocity(velocity.linear_mps, velocity.angular_rad_s)
            )

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

    rclpy.init(args=args)
    node = SpheroRVRNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
