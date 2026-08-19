"""ROS 2 node wrapper for the concurrency-safe RVR driver.

This module avoids importing rclpy at package import time so the core test suite
can run on machines without ROS 2 installed. Run it inside a ROS 2 environment.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import threading
import time
from dataclasses import dataclass, replace
from typing import Optional

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.serial_transport import SerialTransport
from sphero_rvr_core.transport import Transport
from .diagnostics import (
    BatterySnapshot,
    DiagnosticTelemetry,
    battery_state_fields,
    diagnostic_key_values,
    summarize_state,
)
from .led import normalize_rgb255
from .odometry import DifferentialOdomConfig, DifferentialOdomTracker, OdomSample
from .twist_mapper import TwistLike, map_twist_to_velocity

DEFAULT_ODOM_COUNTS_PER_METER = 4337.768


#: The only value of `serial_port` that selects the curve-faithful simulator instead of a
#: real serial device. Deliberately not a boolean flag and deliberately not a plausible
#: path -- a config file cannot enable a fake robot by accident, and a guard test asserts
#: no flight launch or config names it.
SIMULATED_CHASSIS_PORT = "SIMULATED_CHASSIS_NOT_A_REAL_ROBOT"


@dataclass(frozen=True)
class RVRNodeConfig:
    serial_port: str = "/dev/ttyAMA0"
    baud_rate: int = 115200
    cmd_vel_timeout: float = 0.5
    control_period: float = 0.05
    safety_dispatch_timeout_s: float = 0.10
    max_linear_mps: float = 0.25
    max_angular_rad_s: float = 0.4
    max_raw_motor_duty: int = 160
    max_linear_raw_motor_duty: int = 64
    max_angular_raw_motor_duty: int = 255
    pivot_target_rate_rad_s: float = 1.3
    pivot_max_duty: int = 32
    pivot_min_duty: int = 23
    pivot_duty_gain: float = 0.6
    closed_loop_pivot: bool = True
    heading_max_speed: int = 60
    #: Ships FALSE. Flips only when docs/bench_card_2026-08-19.md items (i)-(iv)
    #: are MEASURED (estop preempts an in-flight firmware heading turn above
    #: all). The precise-turn primitive is un-runnable-by-construction until
    #: then -- the camera-charter pattern applied to a motor verb.
    precise_turn_bench_verified: bool = False
    velocity_control_mode: str = RVRDriver.VELOCITY_CONTROL_RAW_MOTOR
    battery_publish_period: float = 5.0
    temperature_publish_period: float = 2.0
    diagnostics_publish_period: float = 1.0
    diagnostics_metadata_period: float = 30.0
    motor_diagnostics_poll_period: float = 0.5
    ambient_light_publish_period: float = 2.0
    odom_publish_period: float = 0.1
    odom_counts_per_meter: float = DEFAULT_ODOM_COUNTS_PER_METER
    # Attempt-2 full-scan registration estimated 0.250696 m. Keep the rounded,
    # explicitly reviewed value pending the restrained confirmation run.
    odom_wheel_track_m: float = 0.2507
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"
    odom_publish_tf: bool = True
    odom_pose_xy_covariance: float = 0.05
    odom_pose_yaw_covariance: float = 0.25
    odom_twist_linear_covariance: float = 0.10
    odom_twist_angular_covariance: float = 0.50
    # IMU sensor streaming (Stage B fusion). publish_imu gates the whole feature;
    # when off the driver never starts the stream (bringup stays as before).
    publish_imu: bool = False
    imu_stream_interval_ms: int = 50
    imu_publish_period: float = 0.05
    imu_frame_id: str = "imu_link"
    imu_orientation_covariance: float = 0.05
    imu_angular_velocity_covariance: float = 0.02
    imu_linear_acceleration_covariance: float = 0.20


def create_driver(config: RVRNodeConfig, transport: Optional[Transport] = None) -> RVRDriver:
    """Build the core driver from ROS-node configuration.

    Kept importable/testable without ROS 2 so safety defaults do not regress.
    """
    if transport is None:
        if config.serial_port == SIMULATED_CHASSIS_PORT:
            # CHASSIS-OFF CLOSED-LOOP TESTING ONLY. Selected by an impossible "port"
            # rather than a boolean, so it cannot be switched on by a stray `true` in a
            # YAML file: a real deployment names a real device, and no real device is
            # called this. Un-startable by accident, per the camera charter's pattern.
            #
            # Everything above this line is production code -- the driver, the velocity
            # clamp, plan_pivot and the odom pipeline all run unchanged. Only the wire
            # is simulated. See sphero_rvr_core.chassis_sim.
            from sphero_rvr_core.chassis_sim import BANNER, SimTransport

            logging.getLogger(__name__).warning(
                "SIMULATED CHASSIS -- NOT A REAL ROBOT. %s", BANNER
            )
            transport = SimTransport(clock=time.monotonic)
        else:
            transport = SerialTransport(port=config.serial_port, baud_rate=config.baud_rate)
    return RVRDriver(
        transport=transport,
        control_period=config.control_period,
        command_timeout=config.cmd_vel_timeout,
        safety_dispatch_timeout_s=config.safety_dispatch_timeout_s,
        max_linear_mps=config.max_linear_mps,
        max_angular_rad_s=config.max_angular_rad_s,
        max_raw_motor_duty=config.max_raw_motor_duty,
        max_linear_raw_motor_duty=config.max_linear_raw_motor_duty,
        max_angular_raw_motor_duty=config.max_angular_raw_motor_duty,
        pivot_target_rate_rad_s=config.pivot_target_rate_rad_s,
        pivot_max_duty=config.pivot_max_duty,
        pivot_min_duty=config.pivot_min_duty,
        pivot_duty_gain=config.pivot_duty_gain,
        closed_loop_pivot=config.closed_loop_pivot,
        heading_max_speed=config.heading_max_speed,
        precise_turn_verified=config.precise_turn_bench_verified,
        velocity_control_mode=config.velocity_control_mode,
        wheel_track_m=config.odom_wheel_track_m,
        imu_stream_interval_ms=(
            config.imu_stream_interval_ms if config.publish_imu else None
        ),
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
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState, Illuminance, Imu, Temperature
    from std_msgs.msg import ColorRGBA, Float32, String
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster

    class SpheroRVRNode(Node):
        def __init__(self):
            super().__init__("sphero_rvr_driver")
            self._declare_parameters()
            self._config = self._read_config()
            self._diagnostic_lock = threading.Lock()
            self._diagnostic_telemetry = DiagnosticTelemetry()
            driver = create_driver(self._config)
            self._motor_stall_subscription = driver.on_motor_stall_notify(
                self._on_motor_stall_notify
            )
            self._motor_fault_subscription = driver.on_motor_fault_notify(
                self._on_motor_fault_notify
            )
            self._motor_thermal_subscription = (
                driver.on_motor_thermal_protection_status_notify(
                    self._on_motor_thermal_notify
                )
            )
            self._driver_thread = AsyncDriverThread(driver)
            self._driver_thread.start()
            self._enable_motor_diagnostic_notifications()

            self._battery_future = None
            self._temperature_future = None
            self._ambient_light_future = None
            self._odom_future = None
            self._metadata_future = None
            self._motor_diagnostics_future = None
            self._odom_tracker = DifferentialOdomTracker(
                DifferentialOdomConfig(
                    counts_per_meter=self._config.odom_counts_per_meter,
                    wheel_track_m=self._config.odom_wheel_track_m,
                    frame_id=self._config.odom_frame_id,
                    child_frame_id=self._config.base_frame_id,
                    pose_xy_covariance=self._config.odom_pose_xy_covariance,
                    pose_yaw_covariance=self._config.odom_pose_yaw_covariance,
                    twist_linear_covariance=self._config.odom_twist_linear_covariance,
                    twist_angular_covariance=self._config.odom_twist_angular_covariance,
                )
            )
            self._battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
            self._left_motor_temperature_pub = self.create_publisher(Temperature, "left_motor_temperature", 10)
            self._right_motor_temperature_pub = self.create_publisher(Temperature, "right_motor_temperature", 10)
            self._ambient_light_pub = self.create_publisher(Illuminance, "ambient_light", 10)
            self._odom_pub = self.create_publisher(Odometry, "odom", 10)
            self._encoder_counts_pub = self.create_publisher(String, "encoder_counts", 10)
            self._tf_broadcaster = TransformBroadcaster(self)
            self._diagnostics_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
            self._imu_pub = None
            self._last_published_imu = None
            if self._config.publish_imu:
                self._imu_pub = self.create_publisher(Imu, "imu", 10)

            self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
            self.create_subscription(ColorRGBA, "set_all_leds", self._on_set_all_leds, 10)
            self.create_service(Trigger, "stop", self._on_stop)
            self.create_service(Trigger, "estop", self._on_estop)
            self.create_service(Trigger, "clear_estop", self._on_clear_estop)
            self.create_service(Trigger, "clear_fail_safe", self._on_clear_fail_safe)
            self.create_service(Trigger, "reset_yaw", self._on_reset_yaw)
            # PRECISE-TURN LANE (option-D-era hybrid, 2026-08-18 night). A thin
            # cmd/event pair, NOT an action: this node runs a single-threaded
            # executor and a blocking action here would stall cmd_vel. The
            # SUPERVISOR GATEWAY is the only intended caller (the safety layer
            # is the only door) -- it owns admission, settle monitoring, the
            # timeout, and cancel->stop; this side only fires the firmware loop
            # and reports STARTED/REFUSED on the event topic. The driver method
            # itself refuses until the bench card flips the flag.
            self._precise_turn_event_pub = self.create_publisher(
                String, "/rvr_driver/precise_turn_event", 10
            )
            self.create_subscription(
                Float32, "/rvr_driver/precise_turn_cmd", self._on_precise_turn_cmd, 10
            )
            self.create_service(Trigger, "reset_locator", self._on_reset_locator)
            self.create_service(Trigger, "release_led_requests", self._on_release_led_requests)
            self.create_timer(self._config.battery_publish_period, self._poll_battery)
            self.create_timer(self._config.temperature_publish_period, self._poll_temperature)
            self.create_timer(self._config.ambient_light_publish_period, self._poll_ambient_light)
            self.create_timer(self._config.odom_publish_period, self._poll_odom)
            if self._config.publish_imu:
                self.create_timer(self._config.imu_publish_period, self._poll_imu)
            self.create_timer(self._config.diagnostics_metadata_period, self._poll_diagnostic_metadata)
            self.create_timer(
                self._config.motor_diagnostics_poll_period,
                self._poll_motor_diagnostics,
            )
            self.create_timer(self._config.diagnostics_publish_period, self._publish_diagnostics)

        def _declare_parameters(self):
            defaults = RVRNodeConfig()
            self.declare_parameter("serial_port", defaults.serial_port)
            self.declare_parameter("baud_rate", defaults.baud_rate)
            self.declare_parameter("cmd_vel_timeout", defaults.cmd_vel_timeout)
            self.declare_parameter("control_period", defaults.control_period)
            self.declare_parameter(
                "safety_dispatch_timeout_s",
                defaults.safety_dispatch_timeout_s,
            )
            self.declare_parameter("max_linear_mps", defaults.max_linear_mps)
            self.declare_parameter("max_angular_rad_s", defaults.max_angular_rad_s)
            self.declare_parameter("max_raw_motor_duty", defaults.max_raw_motor_duty)
            self.declare_parameter("max_linear_raw_motor_duty", defaults.max_linear_raw_motor_duty)
            self.declare_parameter("max_angular_raw_motor_duty", defaults.max_angular_raw_motor_duty)
            self.declare_parameter("pivot_target_rate_rad_s", defaults.pivot_target_rate_rad_s)
            self.declare_parameter("pivot_max_duty", defaults.pivot_max_duty)
            self.declare_parameter("pivot_min_duty", defaults.pivot_min_duty)
            self.declare_parameter("pivot_duty_gain", defaults.pivot_duty_gain)
            self.declare_parameter("closed_loop_pivot", defaults.closed_loop_pivot)
            self.declare_parameter("heading_max_speed", defaults.heading_max_speed)
            self.declare_parameter("precise_turn_bench_verified", defaults.precise_turn_bench_verified)
            self.declare_parameter("velocity_control_mode", defaults.velocity_control_mode)
            self.declare_parameter("battery_publish_period", defaults.battery_publish_period)
            self.declare_parameter("temperature_publish_period", defaults.temperature_publish_period)
            self.declare_parameter("diagnostics_publish_period", defaults.diagnostics_publish_period)
            self.declare_parameter("diagnostics_metadata_period", defaults.diagnostics_metadata_period)
            self.declare_parameter(
                "motor_diagnostics_poll_period",
                defaults.motor_diagnostics_poll_period,
            )
            self.declare_parameter("ambient_light_publish_period", defaults.ambient_light_publish_period)
            self.declare_parameter("odom_publish_period", defaults.odom_publish_period)
            self.declare_parameter("odom_counts_per_meter", defaults.odom_counts_per_meter)
            self.declare_parameter("odom_wheel_track_m", defaults.odom_wheel_track_m)
            self.declare_parameter("odom_frame_id", defaults.odom_frame_id)
            self.declare_parameter("base_frame_id", defaults.base_frame_id)
            self.declare_parameter("odom_publish_tf", defaults.odom_publish_tf)
            self.declare_parameter("odom_pose_xy_covariance", defaults.odom_pose_xy_covariance)
            self.declare_parameter("odom_pose_yaw_covariance", defaults.odom_pose_yaw_covariance)
            self.declare_parameter("odom_twist_linear_covariance", defaults.odom_twist_linear_covariance)
            self.declare_parameter("odom_twist_angular_covariance", defaults.odom_twist_angular_covariance)
            self.declare_parameter("publish_imu", defaults.publish_imu)
            self.declare_parameter("imu_stream_interval_ms", defaults.imu_stream_interval_ms)
            self.declare_parameter("imu_publish_period", defaults.imu_publish_period)
            self.declare_parameter("imu_frame_id", defaults.imu_frame_id)
            self.declare_parameter("imu_orientation_covariance", defaults.imu_orientation_covariance)
            self.declare_parameter(
                "imu_angular_velocity_covariance", defaults.imu_angular_velocity_covariance
            )
            self.declare_parameter(
                "imu_linear_acceleration_covariance", defaults.imu_linear_acceleration_covariance
            )

        def _read_config(self) -> RVRNodeConfig:
            return RVRNodeConfig(
                serial_port=str(self.get_parameter("serial_port").value),
                baud_rate=int(self.get_parameter("baud_rate").value),
                cmd_vel_timeout=float(self.get_parameter("cmd_vel_timeout").value),
                control_period=float(self.get_parameter("control_period").value),
                safety_dispatch_timeout_s=float(
                    self.get_parameter("safety_dispatch_timeout_s").value
                ),
                max_linear_mps=float(self.get_parameter("max_linear_mps").value),
                max_angular_rad_s=float(self.get_parameter("max_angular_rad_s").value),
                max_raw_motor_duty=int(self.get_parameter("max_raw_motor_duty").value),
                max_linear_raw_motor_duty=int(self.get_parameter("max_linear_raw_motor_duty").value),
                max_angular_raw_motor_duty=int(self.get_parameter("max_angular_raw_motor_duty").value),
                pivot_target_rate_rad_s=float(self.get_parameter("pivot_target_rate_rad_s").value),
                pivot_max_duty=int(self.get_parameter("pivot_max_duty").value),
                pivot_min_duty=int(self.get_parameter("pivot_min_duty").value),
                pivot_duty_gain=float(self.get_parameter("pivot_duty_gain").value),
                closed_loop_pivot=bool(self.get_parameter("closed_loop_pivot").value),
                heading_max_speed=int(self.get_parameter("heading_max_speed").value),
                precise_turn_bench_verified=bool(self.get_parameter("precise_turn_bench_verified").value),
                velocity_control_mode=str(self.get_parameter("velocity_control_mode").value),
                battery_publish_period=float(self.get_parameter("battery_publish_period").value),
                temperature_publish_period=float(self.get_parameter("temperature_publish_period").value),
                diagnostics_publish_period=float(self.get_parameter("diagnostics_publish_period").value),
                diagnostics_metadata_period=float(self.get_parameter("diagnostics_metadata_period").value),
                motor_diagnostics_poll_period=float(
                    self.get_parameter("motor_diagnostics_poll_period").value
                ),
                ambient_light_publish_period=float(self.get_parameter("ambient_light_publish_period").value),
                odom_publish_period=float(self.get_parameter("odom_publish_period").value),
                odom_counts_per_meter=float(self.get_parameter("odom_counts_per_meter").value),
                odom_wheel_track_m=float(self.get_parameter("odom_wheel_track_m").value),
                odom_frame_id=str(self.get_parameter("odom_frame_id").value),
                base_frame_id=str(self.get_parameter("base_frame_id").value),
                odom_publish_tf=bool(self.get_parameter("odom_publish_tf").value),
                odom_pose_xy_covariance=float(self.get_parameter("odom_pose_xy_covariance").value),
                odom_pose_yaw_covariance=float(self.get_parameter("odom_pose_yaw_covariance").value),
                odom_twist_linear_covariance=float(self.get_parameter("odom_twist_linear_covariance").value),
                odom_twist_angular_covariance=float(self.get_parameter("odom_twist_angular_covariance").value),
                publish_imu=bool(self.get_parameter("publish_imu").value),
                imu_stream_interval_ms=int(self.get_parameter("imu_stream_interval_ms").value),
                imu_publish_period=float(self.get_parameter("imu_publish_period").value),
                imu_frame_id=str(self.get_parameter("imu_frame_id").value),
                imu_orientation_covariance=float(self.get_parameter("imu_orientation_covariance").value),
                imu_angular_velocity_covariance=float(
                    self.get_parameter("imu_angular_velocity_covariance").value
                ),
                imu_linear_acceleration_covariance=float(
                    self.get_parameter("imu_linear_acceleration_covariance").value
                ),
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

        def _on_clear_fail_safe(self, request, response):
            self._driver_thread.run(self._driver_thread.driver.clear_fail_safe_fault()).result(timeout=5)
            response.success = True
            response.message = "RVR fail-safe fault cleared"
            return response

        def _on_precise_turn_cmd(self, msg):
            delta_deg = float(msg.data)

            def report(fut):
                try:
                    current, target = fut.result()
                    self._precise_turn_event_pub.publish(String(data=(
                        f"STARTED delta_deg={delta_deg:.1f} "
                        f"current_deg={current:.1f} target_deg={target:.1f}"
                    )))
                except Exception as exc:               # noqa: BLE001
                    self._precise_turn_event_pub.publish(
                        String(data=f"REFUSED {exc}")
                    )

            future = self._driver_thread.run(
                self._driver_thread.driver.turn_by_degrees(delta_deg)
            )
            future.add_done_callback(report)

        def _on_reset_yaw(self, request, response):
            self._driver_thread.run(self._driver_thread.driver.reset_yaw()).result(timeout=5)
            response.success = True
            response.message = "RVR yaw reference reset"
            return response

        def _on_reset_locator(self, request, response):
            self._driver_thread.run(self._driver_thread.driver.reset_locator()).result(timeout=5)
            self._odom_tracker.reset()
            response.success = True
            response.message = "RVR locator and local odometry estimate reset"
            return response

        def _on_release_led_requests(self, request, response):
            self._driver_thread.run(self._driver_thread.driver.release_led_requests()).result(timeout=5)
            response.success = True
            response.message = "RVR LED requests released"
            return response

        def _on_set_all_leds(self, msg):
            r, g, b = normalize_rgb255(msg.r * 255.0, msg.g * 255.0, msg.b * 255.0)
            future = self._driver_thread.run(self._driver_thread.driver.set_all_leds(r, g, b))
            future.add_done_callback(lambda fut: self._log_future_exception(fut, "set_all_leds"))

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
                snapshot = future.result()
                fields = battery_state_fields(snapshot)
            except Exception as exc:
                self.get_logger().warn(f"battery query failed: {exc}")
                return
            if not self._context_ok():
                return

            with self._diagnostic_lock:
                self._diagnostic_telemetry = replace(
                    self._diagnostic_telemetry,
                    battery=snapshot,
                )
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.present = fields.present
            msg.percentage = fields.percentage
            msg.voltage = fields.voltage
            self._battery_pub.publish(msg)

        def _poll_temperature(self):
            if self._temperature_future is not None and not self._temperature_future.done():
                return
            self._temperature_future = self._driver_thread.run(self._driver_thread.driver.get_temperature())
            self._temperature_future.add_done_callback(self._publish_temperature_from_future)

        def _publish_temperature_from_future(self, future):
            try:
                readings = future.result()
            except Exception as exc:
                self.get_logger().warn(f"temperature query failed: {exc}")
                return
            if not self._context_ok():
                return

            stamp = self.get_clock().now().to_msg()
            if readings.left_motor is not None:
                self._publish_temperature(
                    self._left_motor_temperature_pub,
                    frame_id="left_motor",
                    stamp=stamp,
                    temperature=readings.left_motor,
                )
            if readings.right_motor is not None:
                self._publish_temperature(
                    self._right_motor_temperature_pub,
                    frame_id="right_motor",
                    stamp=stamp,
                    temperature=readings.right_motor,
                )

        @staticmethod
        def _publish_temperature(publisher, *, frame_id: str, stamp, temperature: float) -> None:
            msg = Temperature()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.temperature = float(temperature)
            msg.variance = 0.0
            publisher.publish(msg)

        def _poll_ambient_light(self):
            if self._ambient_light_future is not None and not self._ambient_light_future.done():
                return
            self._ambient_light_future = self._driver_thread.run(self._driver_thread.driver.get_ambient_light())
            self._ambient_light_future.add_done_callback(self._publish_ambient_light_from_future)

        def _publish_ambient_light_from_future(self, future):
            try:
                illuminance = float(future.result())
            except Exception as exc:
                self.get_logger().warn(f"ambient light query failed: {exc}")
                return
            if not self._context_ok():
                return

            msg = Illuminance()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._config.base_frame_id
            msg.illuminance = illuminance
            msg.variance = 0.0
            self._ambient_light_pub.publish(msg)

        def _poll_imu(self):
            if self._imu_pub is None or not self._context_ok():
                return
            sample = self._driver_thread.driver.get_imu_sample()
            # Publish each streamed sample at most once (identity changes only
            # when the driver decodes a new packet) to avoid feeding the EKF
            # duplicate measurements between stream updates.
            if sample is None or sample is self._last_published_imu:
                return
            self._last_published_imu = sample
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._config.imu_frame_id
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (
                sample.orientation
            )
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
                sample.angular_velocity
            )
            (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ) = sample.linear_acceleration
            oc = self._config.imu_orientation_covariance
            wc = self._config.imu_angular_velocity_covariance
            ac = self._config.imu_linear_acceleration_covariance
            msg.orientation_covariance = [oc, 0.0, 0.0, 0.0, oc, 0.0, 0.0, 0.0, oc]
            msg.angular_velocity_covariance = [wc, 0.0, 0.0, 0.0, wc, 0.0, 0.0, 0.0, wc]
            msg.linear_acceleration_covariance = [ac, 0.0, 0.0, 0.0, ac, 0.0, 0.0, 0.0, ac]
            self._imu_pub.publish(msg)

        def _poll_odom(self):
            if self._odom_future is not None and not self._odom_future.done():
                return
            self._odom_future = self._driver_thread.run(self._driver_thread.driver.get_encoder_counts())
            self._odom_future.add_done_callback(self._publish_odom_from_future)

        def _publish_odom_from_future(self, future):
            try:
                counts = future.result()
            except Exception as exc:
                self.get_logger().warn(f"encoder odometry query failed: {exc}")
                return
            if not self._context_ok():
                return
            stamp_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
            encoder_msg = String()
            encoder_msg.data = json.dumps(
                {
                    "schema": "sphero_rvr.encoder_counts.v1",
                    "stamp": stamp_seconds,
                    "left_count": int(counts.left),
                    "right_count": int(counts.right),
                    "counts_per_meter": self._config.odom_counts_per_meter,
                    "frame_id": self._config.odom_frame_id,
                },
                sort_keys=True,
                separators=(",", ":"),
            )
            self._encoder_counts_pub.publish(encoder_msg)
            sample = self._odom_tracker.update(counts, stamp=stamp_seconds)
            if sample is None:
                return
            self._publish_odom_sample(sample)

        def _publish_odom_sample(self, sample: OdomSample) -> None:
            # Feed measured yaw rate back to the driver for closed-loop pivots.
            self._driver_thread.driver.set_measured_yaw_rate(sample.angular_rad_s)
            stamp = self.get_clock().now().to_msg()
            qz = math.sin(sample.yaw / 2.0)
            qw = math.cos(sample.yaw / 2.0)

            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = sample.frame_id
            msg.child_frame_id = sample.child_frame_id
            msg.pose.pose.position.x = sample.x
            msg.pose.pose.position.y = sample.y
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.twist.twist.linear.x = sample.linear_mps
            msg.twist.twist.angular.z = sample.angular_rad_s
            msg.pose.covariance = list(sample.pose_covariance)
            msg.twist.covariance = list(sample.twist_covariance)
            self._odom_pub.publish(msg)

            if not self._config.odom_publish_tf:
                return

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = sample.frame_id
            tf_msg.child_frame_id = sample.child_frame_id
            tf_msg.transform.translation.x = sample.x
            tf_msg.transform.translation.y = sample.y
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf_msg)

        def _poll_diagnostic_metadata(self):
            if self._metadata_future is not None and not self._metadata_future.done():
                return
            self._metadata_future = self._driver_thread.run(self._read_diagnostic_metadata())
            self._metadata_future.add_done_callback(self._store_diagnostic_metadata_from_future)

        async def _read_diagnostic_metadata(self) -> DiagnosticTelemetry:
            voltage_state = None
            motor_fault = None
            firmware_version = None
            board_revision = None
            processor_name = None
            core_uptime_s = None
            try:
                state = await self._driver_thread.driver.get_battery_voltage_state()
                voltage_state = state.state_name
            except Exception as exc:
                self.get_logger().warn(f"battery voltage state query failed: {exc}")
            try:
                motor_fault = await self._driver_thread.driver.get_motor_fault_state()
            except Exception as exc:
                self.get_logger().warn(f"motor fault query failed: {exc}")
            try:
                version = await self._driver_thread.driver.get_firmware_version()
                firmware_version = f"{version.major}.{version.minor}.{version.revision}"
            except Exception as exc:
                self.get_logger().warn(f"firmware version query failed: {exc}")
            try:
                board_revision = await self._driver_thread.driver.get_board_revision()
            except Exception as exc:
                self.get_logger().warn(f"board revision query failed: {exc}")
            try:
                processor_name = await self._driver_thread.driver.get_processor_name()
            except Exception as exc:
                self.get_logger().warn(f"processor name query failed: {exc}")
            try:
                core_uptime_s = await self._driver_thread.driver.get_core_uptime()
            except Exception as exc:
                self.get_logger().warn(f"core uptime query failed: {exc}")
            with self._diagnostic_lock:
                current = self._diagnostic_telemetry
            return replace(
                current,
                battery_voltage_state=voltage_state,
                motor_fault=motor_fault,
                firmware_version=firmware_version,
                board_revision=board_revision,
                processor_name=processor_name,
                core_uptime_s=core_uptime_s,
            )

        def _store_diagnostic_metadata_from_future(self, future):
            try:
                telemetry = future.result()
            except Exception as exc:
                self.get_logger().warn(f"diagnostic metadata query failed: {exc}")
                return
            with self._diagnostic_lock:
                # Preserve any notification or fast-poll fields updated while
                # the slower metadata query was in flight.
                self._diagnostic_telemetry = replace(
                    self._diagnostic_telemetry,
                    battery_voltage_state=telemetry.battery_voltage_state,
                    motor_fault=telemetry.motor_fault,
                    firmware_version=telemetry.firmware_version,
                    board_revision=telemetry.board_revision,
                    processor_name=telemetry.processor_name,
                    core_uptime_s=telemetry.core_uptime_s,
                )

        def _enable_motor_diagnostic_notifications(self) -> None:
            async def enable() -> None:
                await self._driver_thread.driver.enable_motor_stall_notify()
                await self._driver_thread.driver.enable_motor_fault_notify()
                await (
                    self._driver_thread.driver.enable_motor_thermal_protection_status_notify()
                )

            try:
                self._driver_thread.run(enable()).result(timeout=5)
            except Exception as exc:
                self.get_logger().warn(
                    f"motor diagnostic notification setup failed: {exc}"
                )
                return
            with self._diagnostic_lock:
                self._diagnostic_telemetry = replace(
                    self._diagnostic_telemetry,
                    motor_diagnostics_notification_enabled=True,
                )

        def _on_motor_stall_notify(self, event) -> None:
            with self._diagnostic_lock:
                self._diagnostic_telemetry = replace(
                    self._diagnostic_telemetry,
                    motor_stall_event_count=(
                        self._diagnostic_telemetry.motor_stall_event_count + 1
                    ),
                    last_motor_stall_index=int(event.motor_index),
                    last_motor_stall_active=bool(event.is_triggered),
                )

        def _on_motor_fault_notify(self, event) -> None:
            with self._diagnostic_lock:
                self._diagnostic_telemetry = replace(
                    self._diagnostic_telemetry,
                    motor_fault=bool(event.is_fault),
                )

        def _on_motor_thermal_notify(self, status) -> None:
            with self._diagnostic_lock:
                self._diagnostic_telemetry = replace(
                    self._diagnostic_telemetry,
                    left_motor_temperature_c=float(status.left_temp),
                    left_motor_thermal_status=int(status.left_status),
                    right_motor_temperature_c=float(status.right_temp),
                    right_motor_thermal_status=int(status.right_status),
                )

        def _poll_motor_diagnostics(self) -> None:
            if (
                self._motor_diagnostics_future is not None
                and not self._motor_diagnostics_future.done()
            ):
                return
            self._motor_diagnostics_future = self._driver_thread.run(
                self._read_motor_diagnostics()
            )
            self._motor_diagnostics_future.add_done_callback(
                self._store_motor_diagnostics_from_future
            )

        async def _read_motor_diagnostics(self) -> dict:
            result = {}
            try:
                result["battery_voltage"] = (
                    await self._driver_thread.driver.get_battery_voltage()
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"runtime battery voltage query failed: {exc}"
                )
            try:
                state = await self._driver_thread.driver.get_battery_voltage_state()
                result["battery_voltage_state"] = state.state_name
            except Exception as exc:
                self.get_logger().warn(
                    f"runtime battery voltage state query failed: {exc}"
                )
            try:
                result["motor_fault"] = (
                    await self._driver_thread.driver.get_motor_fault_state()
                )
            except Exception as exc:
                self.get_logger().warn(f"runtime motor fault query failed: {exc}")
            try:
                result["thermal"] = (
                    await self._driver_thread.driver.get_thermal_protection_status()
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"runtime thermal protection query failed: {exc}"
                )
            return result

        def _store_motor_diagnostics_from_future(self, future) -> None:
            try:
                result = future.result()
            except Exception as exc:
                self.get_logger().warn(f"runtime motor diagnostics failed: {exc}")
                return
            with self._diagnostic_lock:
                current = self._diagnostic_telemetry
                battery = current.battery
                if "battery_voltage" in result and battery is not None:
                    battery = replace(
                        battery,
                        voltage=float(result["battery_voltage"]),
                    )
                thermal = result.get("thermal")
                self._diagnostic_telemetry = replace(
                    current,
                    battery=battery,
                    battery_voltage_state=result.get(
                        "battery_voltage_state",
                        current.battery_voltage_state,
                    ),
                    motor_fault=result.get("motor_fault", current.motor_fault),
                    left_motor_temperature_c=(
                        float(thermal.left_temp)
                        if thermal is not None
                        else current.left_motor_temperature_c
                    ),
                    left_motor_thermal_status=(
                        int(thermal.left_status)
                        if thermal is not None
                        else current.left_motor_thermal_status
                    ),
                    right_motor_temperature_c=(
                        float(thermal.right_temp)
                        if thermal is not None
                        else current.right_motor_temperature_c
                    ),
                    right_motor_thermal_status=(
                        int(thermal.right_status)
                        if thermal is not None
                        else current.right_motor_thermal_status
                    ),
                )

        def _publish_diagnostics(self):
            if not self._context_ok():
                return
            state = self._driver_thread.driver.get_state()
            with self._diagnostic_lock:
                telemetry = self._diagnostic_telemetry
            summary = summarize_state(state, telemetry)
            status = DiagnosticStatus()
            status.name = "sphero_rvr_driver"
            status.hardware_id = "sphero_rvr"
            status.level = {"OK": DiagnosticStatus.OK, "WARN": DiagnosticStatus.WARN, "ERROR": DiagnosticStatus.ERROR}[
                summary.level
            ]
            status.message = summary.message
            status.values = [
                KeyValue(key=key, value=value)
                for key, value in diagnostic_key_values(state, telemetry).items()
            ]

            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status = [status]
            self._diagnostics_pub.publish(msg)

        def _log_future_exception(self, future, label: str):
            try:
                future.result()
            except Exception as exc:
                self.get_logger().error(f"{label} command failed: {exc}")

        def _context_ok(self) -> bool:
            return self.context.ok()

    rclpy.init(args=args)
    node = SpheroRVRNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
