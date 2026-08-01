"""High-level concurrency-safe RVR driver."""

import asyncio
import logging
import time
from typing import Any, Callable, Optional

from .command_queue import CommandPriority, PriorityCommandQueue
from .commands import RVRCommands
from .dispatcher import Dispatcher, Subscription
from . import responses
from .packet import (
    DID_DRIVE,
    DID_POWER,
    DID_SENSOR,
    FLAG_REQUEST_ERROR_ONLY,
    FLAG_REQUEST_RESPONSE,
    TARGET_BT,
    TARGET_MCU,
)
from .safety import clamp_velocity, is_stale, now_seconds
from .state import RVRState, VelocityCommand
from .transport import Transport

LOGGER = logging.getLogger(__name__)


class _StaleMotionCommand(RuntimeError):
    """Internal signal that a queued motion packet was invalidated before write."""


class RVRDriver:
    VELOCITY_CONTROL_RAW_MOTOR = "raw_motor"
    VELOCITY_CONTROL_NATIVE_TANK_SI = "native_tank_si"
    # Retained only so callers receive an explicit safety error instead of an
    # ambiguous "unknown mode" failure. The RC-SI mapping was measured at
    # roughly 10x its requested straight-line velocity on 2026-08-01.
    VELOCITY_CONTROL_NATIVE_RC_SI = "native_rc_si"
    _VELOCITY_CONTROL_MODES = frozenset(
        {VELOCITY_CONTROL_RAW_MOTOR, VELOCITY_CONTROL_NATIVE_TANK_SI}
    )
    _MOTOR_CAPABLE_COMMAND_IDS = frozenset(
        {
            RVRCommands.CID_RAW_MOTORS,
            RVRCommands.CID_DRIVE_WITH_HEADING,
            RVRCommands.CID_DRIVE_TANK_SI_UNITS,
            RVRCommands.CID_DRIVE_TANK_NORMALIZED,
            RVRCommands.CID_DRIVE_RC_SI_UNITS,
            RVRCommands.CID_DRIVE_RC_NORMALIZED,
            RVRCommands.CID_DRIVE_TO_POSITION_SI,
        }
    )
    _SENSOR_MOTOR_CAPABLE_COMMAND_IDS = frozenset(
        {
            RVRCommands.CID_START_IR_FOLLOWING,
            RVRCommands.CID_START_IR_EVADING,
        }
    )

    def __init__(
        self,
        transport: Transport,
        control_period: float = 0.05,
        command_timeout: float = 0.5,
        max_linear_mps: float = 1.0,
        max_angular_rad_s: float = 3.0,
        max_raw_motor_duty: int = 64,
        max_linear_raw_motor_duty: Optional[int] = None,
        max_angular_raw_motor_duty: Optional[int] = None,
        velocity_control_mode: str = VELOCITY_CONTROL_RAW_MOTOR,
        safe_stop_attempts: int = 2,
        safe_stop_retry_delay: float = 0.02,
        safety_dispatch_timeout_s: float = 0.10,
        wheel_track_m: float = 0.2507,
        pivot_raw_motor_duty: int = 30,
    ):
        self.commands = RVRCommands()
        self._dispatcher = Dispatcher(transport)
        self._queue = PriorityCommandQueue()
        self._control_period = control_period
        self._command_timeout = command_timeout
        self._safe_stop_attempts = max(1, int(safe_stop_attempts))
        self._safe_stop_retry_delay = max(0.0, float(safe_stop_retry_delay))
        self._safety_dispatch_timeout_s = max(0.001, float(safety_dispatch_timeout_s))
        self._max_linear_mps = max_linear_mps
        self._max_angular_rad_s = max_angular_rad_s
        self._wheel_track_m = float(wheel_track_m)
        if not 0.0 < self._wheel_track_m < float("inf"):
            raise ValueError("wheel_track_m must be positive and finite")
        self._max_raw_motor_duty = max(0, min(255, int(max_raw_motor_duty)))
        self._max_linear_raw_motor_duty = max(
            0,
            min(
                255,
                int(max_linear_raw_motor_duty if max_linear_raw_motor_duty is not None else self._max_raw_motor_duty),
            ),
        )
        self._max_angular_raw_motor_duty = max(
            0,
            min(
                255,
                int(max_angular_raw_motor_duty if max_angular_raw_motor_duty is not None else self._max_raw_motor_duty),
            ),
        )
        self._pivot_raw_motor_duty = max(0, min(127, int(pivot_raw_motor_duty)))
        normalized_control_mode = str(velocity_control_mode).strip().lower()
        if normalized_control_mode == self.VELOCITY_CONTROL_NATIVE_RC_SI:
            raise ValueError(
                "velocity_control_mode native_rc_si is quarantined: its "
                "drive_rc_si_units straight-speed mapping is unsafe/miscalibrated"
            )
        if normalized_control_mode not in self._VELOCITY_CONTROL_MODES:
            raise ValueError(
                "velocity_control_mode must be one of: "
                + ", ".join(sorted(self._VELOCITY_CONTROL_MODES))
            )
        self._velocity_control_mode = normalized_control_mode
        self._desired_velocity: Optional[VelocityCommand] = None
        self._last_velocity_update: Optional[float] = None
        self._connected = False
        self._emergency_stopped = False
        self._motion_generation = 0
        self._fail_safe_active = False
        self._fail_safe_reason: Optional[str] = None
        self._control_task: Optional[asyncio.Task] = None
        self._sequence_id = 0
        self._motor_transport_write_count = 0
        self._motion_transport_write_count = 0
        self._last_motor_command_id: Optional[int] = None
        self._last_motor_sequence_id: Optional[int] = None
        self._last_motor_payload_hex: Optional[str] = None
        self._last_motor_transport_write_epoch_s: Optional[float] = None
        self._last_motion_transport_write_epoch_s: Optional[float] = None

    async def connect(self) -> None:
        await self._dispatcher.start()
        await self._queue.start()
        await self._send(self.commands.connect, CommandPriority.HIGH)
        self._connected = True
        self._control_task = asyncio.create_task(self._control_loop())

    async def disconnect(self) -> None:
        if self._control_task is not None:
            self._control_task.cancel()
            try:
                await self._control_task
            except asyncio.CancelledError:
                pass
            self._control_task = None
        if self._connected and not self._emergency_stopped and not self._fail_safe_active:
            try:
                await self.stop()
            except Exception:
                pass
        await self._queue.stop()
        await self._dispatcher.stop()
        self._connected = False

    async def set_velocity(self, linear_mps: float, angular_rad_s: float) -> None:
        self._raise_if_emergency_stopped()
        if self._fail_safe_active:
            raise RuntimeError("fail-safe fault active; clear safe stop before driving")
        velocity = clamp_velocity(
            VelocityCommand(linear_mps, angular_rad_s),
            max_linear_mps=self._max_linear_mps,
            max_angular_rad_s=self._max_angular_rad_s,
        )
        if velocity.linear_mps == 0.0 and velocity.angular_rad_s == 0.0:
            # A zero Twist is the terminal command used by every supervised
            # motion controller.  Do not translate it into a slew-enabled RC
            # command: that can leave unloaded tracks coasting after the route
            # has already reported completion.  Transition once through the
            # validated immediate stop path, which also invalidates queued
            # motor packets; repeated idle zeros remain transport-silent.
            if self._desired_velocity is not None:
                await self.stop()
            return
        self._desired_velocity = velocity
        self._last_velocity_update = now_seconds()

    async def stop(self) -> None:
        self._desired_velocity = None
        self._last_velocity_update = None
        self._invalidate_motion_commands()
        await self._send_immediate_safety(self.commands.stop)

    async def emergency_stop(self) -> None:
        self._emergency_stopped = True
        self._desired_velocity = None
        self._last_velocity_update = None
        self._invalidate_motion_commands()
        await self._send_immediate_safety(self.commands.emergency_stop)

    async def clear_emergency_stop(self) -> None:
        self._emergency_stopped = False

    async def clear_fail_safe_fault(self) -> None:
        """Clear a transport fail-safe only after a stop command is accepted."""
        self._invalidate_motion_commands()
        await self._send_immediate_safety(self.commands.stop)
        self._fail_safe_active = False
        self._fail_safe_reason = None

    async def reset_yaw(self) -> None:
        await self._send(self.commands.reset_yaw, CommandPriority.HIGH)

    async def reset_locator(self) -> None:
        await self._send(self.commands.reset_locator, CommandPriority.HIGH)

    async def set_all_leds(self, r: int, g=None, b: Optional[int] = None) -> None:
        await self._send(lambda seq: self.commands.set_all_leds(seq, r, g, b), CommandPriority.LOW)

    async def set_led_group(self, group_name: str, r: int, g: int, b: int) -> None:
        await self._send(lambda seq: self.commands.set_led_group(seq, group_name, r, g, b), CommandPriority.LOW)

    async def drive_with_heading(self, speed: int, heading: int, reverse: bool = False) -> None:
        flags = 1 if reverse else 0
        await self._send(
            lambda seq: self.commands.drive_with_heading(seq, speed, heading, flags),
            CommandPriority.NORMAL,
            motor_capable=True,
        )

    async def raw_motors(self, left_mode: int, left_speed: int, right_mode: int, right_speed: int) -> None:
        await self._send(
            lambda seq: self.commands.raw_motors(seq, left_mode, left_speed, right_mode, right_speed),
            CommandPriority.NORMAL,
            motor_capable=True,
        )

    async def drive_to_position_si(
        self,
        yaw_angle: float,
        x: float,
        y: float,
        linear_speed: float,
        flags: int = 0,
    ) -> None:
        await self._send(
            lambda seq: self.commands.drive_to_position_si(seq, yaw_angle, x, y, linear_speed, flags),
            CommandPriority.NORMAL,
            motor_capable=True,
        )

    async def echo(self, data: bytes, target: int = TARGET_BT) -> bytes:
        response = await self._send(lambda seq: self.commands.echo(seq, data, target), CommandPriority.LOW)
        return responses.parse_echo(response.payload)

    async def sleep(self) -> None:
        await self._send(self.commands.sleep, CommandPriority.HIGH)

    async def get_battery_percentage(self) -> int:
        response = await self._send(self.commands.get_battery_percentage, CommandPriority.LOW)
        return responses.parse_battery_percentage(response.payload)

    async def get_battery_voltage(self, reading_type: int = 0) -> float:
        response = await self._send(lambda seq: self.commands.get_battery_voltage(seq, reading_type), CommandPriority.LOW)
        return responses.parse_battery_voltage(response.payload)

    async def get_battery_voltage_state(self) -> responses.BatteryVoltageState:
        response = await self._send(self.commands.get_battery_voltage_state, CommandPriority.LOW)
        return responses.parse_battery_voltage_state(response.payload)

    async def get_battery_thresholds(self) -> responses.BatteryThresholds:
        response = await self._send(self.commands.get_battery_thresholds, CommandPriority.LOW)
        return responses.parse_battery_thresholds(response.payload)

    async def enable_battery_voltage_state_change_notify(self, enabled: bool = True) -> None:
        await self._send(lambda seq: self.commands.enable_battery_voltage_state_change_notify(seq, enabled), CommandPriority.LOW)

    async def get_current_sense_amplifier_current(self, amplifier_id: int) -> float:
        response = await self._send(
            lambda seq: self.commands.get_current_sense_amplifier_current(seq, amplifier_id),
            CommandPriority.LOW,
        )
        return responses.parse_current_sense_amplifier_current(response.payload)

    async def get_rgbc_sensor_values(self) -> responses.RGBCSensorValues:
        response = await self._send(self.commands.get_rgbc_sensor_values, CommandPriority.LOW)
        return responses.parse_rgbc_sensor_values(response.payload)

    async def enable_gyro_max_notify(self, enabled: bool = True) -> None:
        await self._send(lambda seq: self.commands.enable_gyro_max_notify(seq, enabled), CommandPriority.LOW)

    async def set_locator_flags(self, flags: int) -> None:
        await self._send(lambda seq: self.commands.set_locator_flags(seq, flags), CommandPriority.LOW)

    async def get_ambient_light(self) -> float:
        response = await self._send(self.commands.get_ambient_light, CommandPriority.LOW)
        return responses.parse_ambient_light(response.payload)

    async def enable_color_detection(self, enabled: bool = True) -> None:
        await self._send(lambda seq: self.commands.enable_color_detection(seq, enabled), CommandPriority.LOW)

    async def enable_color_detection_notify(
        self,
        enabled: bool = True,
        interval_ms: int = 250,
        confidence: int = 0,
    ) -> None:
        await self._send(
            lambda seq: self.commands.enable_color_detection_notify(seq, enabled, interval_ms, confidence),
            CommandPriority.LOW,
        )

    async def get_current_detected_color(self) -> responses.DetectedColor:
        response = await self._send(self.commands.get_current_detected_color, CommandPriority.LOW)
        return responses.parse_current_detected_color(response.payload)

    async def configure_streaming_service(self, token: int, configuration: bytes, target: int = TARGET_BT) -> None:
        await self._send(
            lambda seq: self.commands.configure_streaming_service(seq, token, configuration, target),
            CommandPriority.LOW,
        )

    async def start_streaming_service(self, period: int, target: int = TARGET_BT) -> None:
        await self._send(lambda seq: self.commands.start_streaming_service(seq, period, target), CommandPriority.LOW)

    async def stop_streaming_service(self, target: int = TARGET_BT) -> None:
        await self._send(lambda seq: self.commands.stop_streaming_service(seq, target), CommandPriority.LOW)

    async def clear_streaming_service(self, target: int = TARGET_BT) -> None:
        await self._send(lambda seq: self.commands.clear_streaming_service(seq, target), CommandPriority.LOW)

    async def enable_robot_infrared_message_notify(self, enabled: bool = True) -> None:
        await self._send(lambda seq: self.commands.enable_robot_infrared_message_notify(seq, enabled), CommandPriority.LOW)

    async def get_temperature(self) -> responses.TemperatureReadings:
        response = await self._send(self.commands.get_temperature, CommandPriority.LOW)
        return responses.parse_temperature(response.payload)

    async def get_thermal_protection_status(self) -> responses.ThermalProtectionStatus:
        response = await self._send(self.commands.get_thermal_protection_status, CommandPriority.LOW)
        return responses.parse_thermal_protection_status(response.payload)

    async def enable_motor_thermal_protection_status_notify(self, enabled: bool = True) -> None:
        await self._send(
            lambda seq: self.commands.enable_motor_thermal_protection_status_notify(seq, enabled),
            CommandPriority.LOW,
        )

    async def get_encoder_counts(self) -> responses.EncoderCounts:
        response = await self._send(self.commands.get_encoder_counts, CommandPriority.LOW)
        return responses.parse_encoder_counts(response.payload)

    async def get_magnetometer(self) -> responses.MagnetometerReadings:
        response = await self._send(self.commands.get_magnetometer, CommandPriority.LOW)
        return responses.parse_magnetometer(response.payload)

    async def calibrate_magnetometer(self) -> None:
        await self._send(self.commands.calibrate_magnetometer, CommandPriority.LOW)

    async def get_motor_fault_state(self) -> bool:
        response = await self._send(self.commands.get_motor_fault_state, CommandPriority.LOW)
        return responses.parse_motor_fault_state(response.payload)

    async def enable_motor_stall_notify(self, enabled: bool = True) -> None:
        await self._send(lambda seq: self.commands.enable_motor_stall_notify(seq, enabled), CommandPriority.LOW)

    async def enable_motor_fault_notify(self, enabled: bool = True) -> None:
        await self._send(lambda seq: self.commands.enable_motor_fault_notify(seq, enabled), CommandPriority.LOW)

    async def get_firmware_version(self, target: int = TARGET_BT) -> responses.FirmwareVersion:
        response = await self._send(lambda seq: self.commands.get_main_app_version(seq, target), CommandPriority.LOW)
        return responses.parse_firmware_version(response.payload)

    async def get_bootloader_version(self, target: int = TARGET_BT) -> responses.FirmwareVersion:
        response = await self._send(lambda seq: self.commands.get_bootloader_version(seq, target), CommandPriority.LOW)
        return responses.parse_firmware_version(response.payload)

    async def get_mac_address(self) -> str:
        response = await self._send(self.commands.get_mac_address, CommandPriority.LOW)
        return responses.parse_null_terminated_ascii(response.payload)

    async def get_stats_id(self) -> int:
        response = await self._send(self.commands.get_stats_id, CommandPriority.LOW)
        return responses.parse_stats_id(response.payload)

    async def get_board_revision(self, target: int = TARGET_BT) -> int:
        response = await self._send(lambda seq: self.commands.get_board_revision(seq, target), CommandPriority.LOW)
        return responses.parse_board_revision(response.payload)

    async def get_processor_name(self, target: int = TARGET_BT) -> str:
        response = await self._send(lambda seq: self.commands.get_processor_name(seq, target), CommandPriority.LOW)
        return responses.parse_null_terminated_ascii(response.payload)

    async def get_sku(self) -> str:
        response = await self._send(self.commands.get_sku, CommandPriority.LOW)
        return responses.parse_null_terminated_ascii(response.payload)

    async def get_core_uptime(self, target: int = TARGET_BT) -> int:
        response = await self._send(lambda seq: self.commands.get_core_uptime(seq, target), CommandPriority.LOW)
        return responses.parse_core_uptime(response.payload)

    async def get_bluetooth_advertising_name(self) -> str:
        response = await self._send(self.commands.get_bluetooth_advertising_name, CommandPriority.LOW)
        return responses.parse_bluetooth_advertising_name(response.payload)

    async def get_active_color_palette(self) -> responses.ActiveColorPalette:
        response = await self._send(self.commands.get_active_color_palette, CommandPriority.LOW)
        return responses.parse_active_color_palette(response.payload)

    async def set_active_color_palette(self, rgb_index_bytes: bytes) -> None:
        await self._send(lambda seq: self.commands.set_active_color_palette(seq, rgb_index_bytes), CommandPriority.LOW)

    async def get_color_identification_report(
        self,
        red: int,
        green: int,
        blue: int,
        confidence_threshold: int,
    ) -> responses.ColorIdentificationReport:
        response = await self._send(
            lambda seq: self.commands.get_color_identification_report(seq, red, green, blue, confidence_threshold),
            CommandPriority.LOW,
        )
        return responses.parse_color_identification_report(response.payload)

    async def load_color_palette(self, palette_index: int) -> None:
        await self._send(lambda seq: self.commands.load_color_palette(seq, palette_index), CommandPriority.LOW)

    async def save_color_palette(self, palette_index: int) -> None:
        await self._send(lambda seq: self.commands.save_color_palette(seq, palette_index), CommandPriority.LOW)

    async def release_led_requests(self) -> None:
        await self._send(self.commands.release_led_requests, CommandPriority.LOW)

    async def send_ir_message(self, code: int, strength: int = 32) -> None:
        await self._send(lambda seq: self.commands.send_ir_message(seq, code, strength), CommandPriority.LOW)

    async def start_ir_broadcast(self, far_code: int, near_code: int) -> None:
        await self._send(lambda seq: self.commands.start_ir_broadcast(seq, far_code, near_code), CommandPriority.LOW)

    async def stop_ir_broadcast(self) -> None:
        await self._send(self.commands.stop_ir_broadcast, CommandPriority.LOW)

    async def start_ir_following(self, far_code: int, near_code: int) -> None:
        await self._send(
            lambda seq: self.commands.start_ir_following(seq, far_code, near_code),
            CommandPriority.LOW,
            motor_capable=True,
        )

    async def stop_ir_following(self) -> None:
        await self._send(self.commands.stop_ir_following, CommandPriority.LOW)

    async def start_ir_evading(self, far_code: int, near_code: int) -> None:
        await self._send(
            lambda seq: self.commands.start_ir_evading(seq, far_code, near_code),
            CommandPriority.LOW,
            motor_capable=True,
        )

    async def stop_ir_evading(self) -> None:
        await self._send(self.commands.stop_ir_evading, CommandPriority.LOW)

    async def get_ir_readings(self) -> responses.IRReadings:
        response = await self._send(self.commands.get_ir_readings, CommandPriority.LOW)
        return responses.parse_ir_readings(response.payload)

    def on_will_sleep_notify(self, callback: Callable[[responses.SleepEvent], Any]) -> Subscription:
        return self._subscribe(DID_POWER, 0x19, TARGET_BT, callback)

    def on_did_sleep_notify(self, callback: Callable[[responses.SleepEvent], Any]) -> Subscription:
        return self._subscribe(DID_POWER, 0x1A, TARGET_BT, callback)

    def on_battery_voltage_state_change_notify(
        self,
        callback: Callable[[responses.BatteryVoltageState], Any],
    ) -> Subscription:
        return self._subscribe(DID_POWER, 0x1C, TARGET_BT, callback)

    def get_cached_battery_voltage_state_change(self) -> Optional[responses.BatteryVoltageState]:
        return self._dispatcher.get_cached_event(DID_POWER, 0x1C, TARGET_BT)

    def on_motor_stall_notify(self, callback: Callable[[responses.MotorStallEvent], Any]) -> Subscription:
        return self._subscribe(DID_DRIVE, 0x26, TARGET_MCU, callback)

    def on_motor_fault_notify(self, callback: Callable[[responses.MotorFaultEvent], Any]) -> Subscription:
        return self._subscribe(DID_DRIVE, 0x28, TARGET_MCU, callback)

    def on_gyro_max_notify(self, callback: Callable[[responses.GyroMaxEvent], Any]) -> Subscription:
        return self._subscribe(DID_SENSOR, 0x10, TARGET_MCU, callback)

    def on_robot_to_robot_infrared_message_received_notify(
        self,
        callback: Callable[[responses.InfraredMessageEvent], Any],
    ) -> Subscription:
        return self._subscribe(DID_SENSOR, 0x2C, TARGET_MCU, callback)

    def on_color_detection_notify(self, callback: Callable[[responses.DetectedColor], Any]) -> Subscription:
        return self._subscribe(DID_SENSOR, 0x36, TARGET_BT, callback)

    def on_streaming_service_data_notify(
        self,
        callback: Callable[[responses.StreamingServiceData], Any],
        target: int = TARGET_BT,
    ) -> Subscription:
        return self._subscribe(DID_SENSOR, 0x3D, target, callback)

    def on_motor_thermal_protection_status_notify(
        self,
        callback: Callable[[responses.ThermalProtectionStatus], Any],
    ) -> Subscription:
        return self._subscribe(DID_SENSOR, 0x4D, TARGET_MCU, callback)

    def get_state(self) -> RVRState:
        return RVRState(
            connected=self._connected,
            emergency_stopped=self._emergency_stopped,
            latest_velocity=self._desired_velocity,
            fail_safe_active=self._fail_safe_active,
            fail_safe_reason=self._fail_safe_reason,
            motor_transport_write_count=self._motor_transport_write_count,
            motion_transport_write_count=self._motion_transport_write_count,
            last_motor_command_id=self._last_motor_command_id,
            last_motor_sequence_id=self._last_motor_sequence_id,
            last_motor_payload_hex=self._last_motor_payload_hex,
            last_motor_transport_write_epoch_s=self._last_motor_transport_write_epoch_s,
            last_motion_transport_write_epoch_s=self._last_motion_transport_write_epoch_s,
        )

    def _subscribe(
        self,
        device_id: int,
        command_id: int,
        source: Optional[int],
        callback: Callable[[Any], Any],
    ) -> Subscription:
        return self._dispatcher.subscribe(device_id, command_id, source, callback)

    async def _control_loop(self) -> None:
        stop_sent_for_stale = False
        while True:
            await asyncio.sleep(self._control_period)
            if self._emergency_stopped:
                continue
            if self._fail_safe_active:
                continue
            if self._desired_velocity is None:
                continue
            if is_stale(self._last_velocity_update, self._command_timeout):
                if not stop_sent_for_stale:
                    await self._attempt_control_safe_stop("stale velocity command")
                    stop_sent_for_stale = not self._fail_safe_active
                continue
            stop_sent_for_stale = False
            velocity = self._desired_velocity
            motion_generation = self._motion_generation
            linear_fraction = velocity.linear_mps / self._max_linear_mps if self._max_linear_mps else 0.0
            angular_fraction = velocity.angular_rad_s / self._max_angular_rad_s if self._max_angular_rad_s else 0.0
            if self._velocity_control_mode == self.VELOCITY_CONTROL_RAW_MOTOR:
                # This is the physically measured ROS mission backend.  Both
                # requested components are normalized against their configured
                # maxima and then mapped through independently bounded raw-duty
                # caps.  Unlike native RC-SI, these caps were present during the
                # June 24 floor calibration and make short-run calibration
                # packets explicit and reproducible.
                await self._send_from_control_loop(
                    lambda seq: self.commands.drive_rc(
                        seq,
                        linear_fraction,
                        angular_fraction,
                        max_speed=self._max_raw_motor_duty,
                        max_linear_speed=self._max_linear_raw_motor_duty,
                        max_angular_speed=self._max_angular_raw_motor_duty,
                    ),
                    motion_generation=motion_generation,
                )
                continue
            if abs(velocity.linear_mps) < 0.005 and abs(velocity.angular_rad_s) > 0.0:
                signed_duty = (
                    self._pivot_raw_motor_duty
                    if velocity.angular_rad_s > 0.0
                    else -self._pivot_raw_motor_duty
                )
                await self._send_from_control_loop(
                    lambda seq: self.commands.drive_tank_normalized(
                        seq,
                        left_velocity=-signed_duty,
                        right_velocity=signed_duty,
                    ),
                    motion_generation=motion_generation,
                )
                continue
            half_track = self._wheel_track_m / 2.0
            left_mps = velocity.linear_mps - velocity.angular_rad_s * half_track
            right_mps = velocity.linear_mps + velocity.angular_rad_s * half_track
            await self._send_from_control_loop(
                lambda seq: self.commands.drive_tank_si_units(
                    seq,
                    left_velocity=left_mps,
                    right_velocity=right_mps,
                ),
                motion_generation=motion_generation,
            )

    async def _send_from_control_loop(self, packet_factory, *, motion_generation: int) -> None:
        try:
            await self._send(
                packet_factory,
                CommandPriority.NORMAL,
                motor_capable=True,
                expected_motion_generation=motion_generation,
            )
        except asyncio.CancelledError:
            raise
        except Exception:
            LOGGER.exception("RVR control loop send failed; attempting safe stop")
            await self._attempt_control_safe_stop("control loop send failure")

    async def _attempt_control_safe_stop(self, reason: str) -> None:
        self._desired_velocity = None
        self._last_velocity_update = None
        self._invalidate_motion_commands()
        last_exc: Optional[BaseException] = None
        for attempt in range(1, self._safe_stop_attempts + 1):
            try:
                await self._send_immediate_safety(self.commands.stop)
                return
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                last_exc = exc
                if attempt < self._safe_stop_attempts:
                    LOGGER.exception(
                        "RVR safe stop delivery failed; retrying (%s/%s)",
                        attempt,
                        self._safe_stop_attempts,
                    )
                    if self._safe_stop_retry_delay:
                        await asyncio.sleep(self._safe_stop_retry_delay)
                    continue
                self._fail_safe_active = True
                self._fail_safe_reason = f"{reason}: {exc}"
                LOGGER.exception("RVR fail-safe fault active; safe stop delivery failed")
        if last_exc is not None and not self._fail_safe_active:
            self._fail_safe_active = True
            self._fail_safe_reason = f"{reason}: {last_exc}"

    async def _send(
        self,
        packet_factory,
        priority: CommandPriority,
        *,
        motor_capable: bool = False,
        expected_motion_generation: Optional[int] = None,
    ):
        sequence_id = self._next_sequence_id()
        packet = packet_factory(sequence_id)
        motor_capable = motor_capable or self._is_motor_capable_packet(packet)
        generation = None
        if motor_capable:
            generation = (
                self._motion_generation
                if expected_motion_generation is None
                else int(expected_motion_generation)
            )
        if motor_capable:
            self._raise_if_emergency_stopped()
        expects_response = packet.flags & (FLAG_REQUEST_RESPONSE | FLAG_REQUEST_ERROR_ONLY)
        return await self._queue.submit(
            lambda: self._dispatch_if_motion_current(
                packet,
                expects_response=bool(expects_response),
                motion_generation=generation,
            ),
            priority=priority,
        )

    async def _send_immediate_safety(self, packet_factory) -> None:
        sequence_id = self._next_sequence_id()
        packet = packet_factory(sequence_id)
        try:
            await asyncio.wait_for(self._dispatcher.send(packet), timeout=self._safety_dispatch_timeout_s)
            self._record_motor_transport_write(packet)
        except asyncio.TimeoutError as exc:
            self._fail_safe_active = True
            self._fail_safe_reason = "safety stop dispatch exceeded software budget"
            raise TimeoutError("safety stop dispatch exceeded software budget") from exc

    async def _dispatch_if_motion_current(self, packet, *, expects_response: bool, motion_generation: Optional[int]):
        if motion_generation is not None:
            if not self._motion_dispatch_allowed(motion_generation):
                return None
        before_write = None
        if motion_generation is not None:
            before_write = lambda: self._raise_if_motion_dispatch_blocked(motion_generation)
        try:
            if expects_response:
                result = await self._dispatcher.request(packet, before_write=before_write)
            else:
                result = await self._dispatcher.send(packet, before_write=before_write)
            if motion_generation is not None:
                self._record_motor_transport_write(packet)
            return result
        except _StaleMotionCommand:
            return None

    def _record_motor_transport_write(self, packet) -> None:
        """Record successful host transport writes without implying firmware ACK.

        Raw motor commands are fire-and-forget in the RVR protocol. Reaching
        this method proves that the transport write completed; it does not
        prove that the MCU accepted or applied the command.
        """
        if not self._is_motor_capable_packet(packet):
            return
        written_at = time.time()
        self._motor_transport_write_count += 1
        self._last_motor_command_id = int(packet.command_id)
        self._last_motor_sequence_id = int(packet.sequence_id)
        self._last_motor_payload_hex = packet.payload.hex()
        self._last_motor_transport_write_epoch_s = written_at
        is_motion_raw_motor = (
            packet.command_id == RVRCommands.CID_RAW_MOTORS
            and packet.payload != b"\x00\x00\x00\x00"
        )
        if packet.command_id != RVRCommands.CID_RAW_MOTORS or is_motion_raw_motor:
            self._motion_transport_write_count += 1
            self._last_motion_transport_write_epoch_s = written_at

    def _invalidate_motion_commands(self) -> None:
        self._motion_generation += 1

    def _raise_if_emergency_stopped(self) -> None:
        if self._emergency_stopped:
            raise RuntimeError("emergency stop active; clear emergency stop before driving")

    def _motion_dispatch_allowed(self, motion_generation: int) -> bool:
        return not self._emergency_stopped and motion_generation == self._motion_generation

    def _raise_if_motion_dispatch_blocked(self, motion_generation: int) -> None:
        if not self._motion_dispatch_allowed(motion_generation):
            raise _StaleMotionCommand("stale motion command invalidated before dispatch")

    def _is_motor_capable_packet(self, packet) -> bool:
        if packet.device_id == DID_DRIVE:
            return packet.command_id in self._MOTOR_CAPABLE_COMMAND_IDS
        if packet.device_id == DID_SENSOR:
            return packet.command_id in self._SENSOR_MOTOR_CAPABLE_COMMAND_IDS
        return False

    def _next_sequence_id(self) -> int:
        self._sequence_id = (self._sequence_id + 1) % 256
        return self._sequence_id
