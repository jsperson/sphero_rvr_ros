"""High-level concurrency-safe RVR driver."""

import asyncio
from typing import Optional

from .command_queue import CommandPriority, PriorityCommandQueue
from .commands import RVRCommands
from .dispatcher import Dispatcher
from . import responses
from .packet import FLAG_REQUEST_ERROR_ONLY, FLAG_REQUEST_RESPONSE, TARGET_BT, TARGET_MCU
from .safety import clamp_velocity, is_stale, now_seconds
from .state import RVRState, VelocityCommand
from .transport import Transport


class RVRDriver:
    def __init__(
        self,
        transport: Transport,
        control_period: float = 0.05,
        command_timeout: float = 0.5,
        max_linear_mps: float = 1.0,
        max_angular_rad_s: float = 3.0,
        max_raw_motor_duty: int = 64,
    ):
        self.commands = RVRCommands()
        self._dispatcher = Dispatcher(transport)
        self._queue = PriorityCommandQueue()
        self._control_period = control_period
        self._command_timeout = command_timeout
        self._max_linear_mps = max_linear_mps
        self._max_angular_rad_s = max_angular_rad_s
        self._max_raw_motor_duty = max(0, min(255, int(max_raw_motor_duty)))
        self._desired_velocity: Optional[VelocityCommand] = None
        self._last_velocity_update: Optional[float] = None
        self._connected = False
        self._emergency_stopped = False
        self._control_task: Optional[asyncio.Task] = None
        self._sequence_id = 0

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
        if self._connected and not self._emergency_stopped:
            try:
                await self.stop()
            except Exception:
                pass
        await self._queue.stop()
        await self._dispatcher.stop()
        self._connected = False

    async def set_velocity(self, linear_mps: float, angular_rad_s: float) -> None:
        self._desired_velocity = clamp_velocity(
            VelocityCommand(linear_mps, angular_rad_s),
            max_linear_mps=self._max_linear_mps,
            max_angular_rad_s=self._max_angular_rad_s,
        )
        self._last_velocity_update = now_seconds()

    async def stop(self) -> None:
        self._desired_velocity = None
        self._last_velocity_update = None
        await self._send(self.commands.stop, CommandPriority.HIGH)

    async def emergency_stop(self) -> None:
        self._emergency_stopped = True
        self._desired_velocity = None
        self._last_velocity_update = None
        await self._send(self.commands.emergency_stop, CommandPriority.EMERGENCY)

    async def clear_emergency_stop(self) -> None:
        await self._send(self.commands.clear_emergency_stop, CommandPriority.HIGH)
        self._emergency_stopped = False

    async def reset_yaw(self) -> None:
        await self._send(self.commands.reset_yaw, CommandPriority.HIGH)

    async def reset_locator(self) -> None:
        await self._send(self.commands.reset_locator, CommandPriority.HIGH)

    async def set_all_leds(self, r: int, g: int, b: int) -> None:
        await self._send(lambda seq: self.commands.set_all_leds(seq, r, g, b), CommandPriority.LOW)

    async def set_led_group(self, group_name: str, r: int, g: int, b: int) -> None:
        await self._send(lambda seq: self.commands.set_led_group(seq, group_name, r, g, b), CommandPriority.LOW)

    async def drive_with_heading(self, speed: int, heading: int, reverse: bool = False) -> None:
        flags = 1 if reverse else 0
        await self._send(lambda seq: self.commands.drive_with_heading(seq, speed, heading, flags), CommandPriority.NORMAL)

    async def raw_motors(self, left_mode: int, left_speed: int, right_mode: int, right_speed: int) -> None:
        await self._send(
            lambda seq: self.commands.raw_motors(seq, left_mode, left_speed, right_mode, right_speed),
            CommandPriority.NORMAL,
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
        )

    async def get_battery_percentage(self) -> int:
        response = await self._send(self.commands.get_battery_percentage, CommandPriority.LOW)
        return responses.parse_battery_percentage(response.payload)

    async def get_battery_voltage(self) -> float:
        response = await self._send(self.commands.get_battery_voltage, CommandPriority.LOW)
        return responses.parse_battery_voltage(response.payload)

    async def get_battery_voltage_state(self) -> responses.BatteryVoltageState:
        response = await self._send(self.commands.get_battery_voltage_state, CommandPriority.LOW)
        return responses.parse_battery_voltage_state(response.payload)

    async def get_battery_thresholds(self) -> responses.BatteryThresholds:
        response = await self._send(self.commands.get_battery_thresholds, CommandPriority.LOW)
        return responses.parse_battery_thresholds(response.payload)

    async def get_rgbc_sensor_values(self) -> responses.RGBCSensorValues:
        response = await self._send(self.commands.get_rgbc_sensor_values, CommandPriority.LOW)
        return responses.parse_rgbc_sensor_values(response.payload)

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

    async def get_temperature(self) -> responses.TemperatureReadings:
        response = await self._send(self.commands.get_temperature, CommandPriority.LOW)
        return responses.parse_temperature(response.payload)

    async def get_thermal_protection_status(self) -> responses.ThermalProtectionStatus:
        response = await self._send(self.commands.get_thermal_protection_status, CommandPriority.LOW)
        return responses.parse_thermal_protection_status(response.payload)

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

    async def get_mac_address(self) -> str:
        response = await self._send(self.commands.get_mac_address, CommandPriority.LOW)
        return responses.parse_null_terminated_ascii(response.payload)

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

    async def send_ir_message(self, code: int, strength: int = 32) -> None:
        await self._send(lambda seq: self.commands.send_ir_message(seq, code, strength), CommandPriority.LOW)

    async def start_ir_broadcast(self, far_code: int, near_code: int) -> None:
        await self._send(lambda seq: self.commands.start_ir_broadcast(seq, far_code, near_code), CommandPriority.LOW)

    async def stop_ir_broadcast(self) -> None:
        await self._send(self.commands.stop_ir_broadcast, CommandPriority.LOW)

    async def start_ir_following(self, far_code: int, near_code: int) -> None:
        await self._send(lambda seq: self.commands.start_ir_following(seq, far_code, near_code), CommandPriority.LOW)

    async def stop_ir_following(self) -> None:
        await self._send(self.commands.stop_ir_following, CommandPriority.LOW)

    async def start_ir_evading(self, far_code: int, near_code: int) -> None:
        await self._send(lambda seq: self.commands.start_ir_evading(seq, far_code, near_code), CommandPriority.LOW)

    async def stop_ir_evading(self) -> None:
        await self._send(self.commands.stop_ir_evading, CommandPriority.LOW)

    async def get_ir_readings(self) -> responses.IRReadings:
        response = await self._send(self.commands.get_ir_readings, CommandPriority.LOW)
        return responses.parse_ir_readings(response.payload)

    def get_state(self) -> RVRState:
        return RVRState(
            connected=self._connected,
            emergency_stopped=self._emergency_stopped,
            latest_velocity=self._desired_velocity,
        )

    async def _control_loop(self) -> None:
        stop_sent_for_stale = False
        while True:
            await asyncio.sleep(self._control_period)
            if self._emergency_stopped:
                continue
            if self._desired_velocity is None:
                continue
            if is_stale(self._last_velocity_update, self._command_timeout):
                if not stop_sent_for_stale:
                    await self.stop()
                    stop_sent_for_stale = True
                continue
            stop_sent_for_stale = False
            velocity = self._desired_velocity
            await self._send(
                lambda seq: self.commands.drive_rc(
                    seq,
                    velocity.linear_mps,
                    velocity.angular_rad_s,
                    max_speed=self._max_raw_motor_duty,
                ),
                CommandPriority.NORMAL,
            )

    async def _send(self, packet_factory, priority: CommandPriority):
        sequence_id = self._next_sequence_id()
        packet = packet_factory(sequence_id)
        expects_response = packet.flags & (FLAG_REQUEST_RESPONSE | FLAG_REQUEST_ERROR_ONLY)
        if expects_response:
            return await self._queue.submit(lambda: self._dispatcher.request(packet), priority=priority)
        return await self._queue.submit(lambda: self._dispatcher.send(packet), priority=priority)

    def _next_sequence_id(self) -> int:
        self._sequence_id = (self._sequence_id + 1) % 256
        return self._sequence_id
