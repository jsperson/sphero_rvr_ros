"""Pure Sphero RVR command builders.

The builders in this module produce :class:`sphero_rvr_core.packet.Packet`
objects only. They do not own serial I/O, sleep, talk to ROS, or mutate global
sequence state. The driver supplies sequence IDs and sends packets through the
single dispatcher/queue path.
"""

import struct
from typing import ClassVar

from .packet import (
    DID_DRIVE,
    DID_IO,
    DID_IR,
    DID_POWER,
    DID_SENSOR,
    DID_SYSTEM_INFO,
    FLAG_HAS_TARGET,
    FLAG_IS_ACTIVITY,
    FLAG_REQUEST_RESPONSE,
    TARGET_BT,
    TARGET_MCU,
    Packet,
)

# LED group mappings: each group has 3 consecutive bitmap bits/channels.
LED_GROUPS = {
    "headlight_right": 0,
    "headlight_left": 1,
    "battery_door_front": 2,
    "battery_door_rear": 3,
    "power_button_front": 4,
    "power_button_rear": 5,
    "brakelight_left": 6,
    "brakelight_right": 7,
    "status_indication_left": 8,
    "status_indication_right": 9,
}


class RVRCommands:
    """Command catalog for the Sphero RVR direct serial protocol."""

    # Backwards-compatible names used by the initial concurrency-slice tests.
    DEVICE: ClassVar[int] = DID_POWER
    CONNECT: ClassVar[int] = 0x0D
    EMERGENCY_STOP: ClassVar[int] = 0xFE
    CLEAR_EMERGENCY_STOP: ClassVar[int] = 0xFD

    # Drive command IDs.
    CID_RAW_MOTORS: ClassVar[int] = 0x01
    CID_RESET_YAW: ClassVar[int] = 0x06
    CID_DRIVE_WITH_HEADING: ClassVar[int] = 0x07
    CID_RESET_LOCATOR: ClassVar[int] = 0x13
    CID_DRIVE_TO_POSITION_SI: ClassVar[int] = 0x38

    # Legacy test aliases. `drive_rc` remains a high-level/core convenience for
    # now; full ROS velocity mapping can be revisited after command cataloging.
    STOP: ClassVar[int] = CID_RAW_MOTORS
    DRIVE_RC: ClassVar[int] = 0xF0

    # IO command IDs.
    CID_SET_ALL_LEDS: ClassVar[int] = 0x1A

    # Power command IDs.
    CID_WAKE: ClassVar[int] = 0x0D
    CID_GET_BATTERY_PERCENTAGE: ClassVar[int] = 0x10
    CID_GET_BATTERY_VOLTAGE_STATE: ClassVar[int] = 0x17
    CID_GET_BATTERY_VOLTAGE: ClassVar[int] = 0x25
    CID_GET_BATTERY_THRESHOLDS: ClassVar[int] = 0x26

    # Sensor command IDs.
    CID_GET_IR_READINGS: ClassVar[int] = 0x22
    CID_GET_RGBC_SENSOR: ClassVar[int] = 0x23
    CID_CALIBRATE_MAGNETOMETER: ClassVar[int] = 0x25
    CID_START_IR_FOLLOWING: ClassVar[int] = 0x28
    CID_GET_AMBIENT_LIGHT: ClassVar[int] = 0x30
    CID_STOP_IR_FOLLOWING: ClassVar[int] = 0x32
    CID_START_IR_EVADING: ClassVar[int] = 0x33
    CID_STOP_IR_EVADING: ClassVar[int] = 0x34
    CID_ENABLE_COLOR_DETECTION_NOTIFY: ClassVar[int] = 0x35
    CID_GET_CURRENT_DETECTED_COLOR: ClassVar[int] = 0x37
    CID_ENABLE_COLOR_DETECTION: ClassVar[int] = 0x38
    CID_GET_TEMPERATURE: ClassVar[int] = 0x4A
    CID_GET_THERMAL_PROTECTION: ClassVar[int] = 0x4B
    CID_GET_MAGNETOMETER: ClassVar[int] = 0x52
    CID_GET_ENCODER_COUNTS: ClassVar[int] = 0x53

    # System info command IDs.
    CID_GET_MAIN_APP_VERSION: ClassVar[int] = 0x00
    CID_GET_BOARD_REVISION: ClassVar[int] = 0x03
    CID_GET_MAC_ADDRESS: ClassVar[int] = 0x06
    CID_GET_PROCESSOR_NAME: ClassVar[int] = 0x1F
    CID_GET_SKU: ClassVar[int] = 0x38
    CID_GET_CORE_UPTIME: ClassVar[int] = 0x39

    # IR command IDs.
    CID_SEND_IR_MESSAGE: ClassVar[int] = 0x38
    CID_STOP_IR_BROADCAST: ClassVar[int] = 0x39
    CID_START_IR_BROADCAST: ClassVar[int] = 0x3A

    # Motor protection command IDs live under DID_DRIVE.
    CID_ENABLE_MOTOR_STALL_NOTIFY: ClassVar[int] = 0x25
    CID_ENABLE_MOTOR_FAULT_NOTIFY: ClassVar[int] = 0x27
    CID_GET_MOTOR_FAULT: ClassVar[int] = 0x29

    def connect(self, sequence_id: int) -> Packet:
        """Connect-time wake command retained for driver compatibility."""
        return self.wake(sequence_id)

    def wake(self, sequence_id: int) -> Packet:
        return self._packet(DID_POWER, self.CID_WAKE, sequence_id, TARGET_BT)

    def drive_with_heading(
        self,
        sequence_id: int,
        speed: int,
        heading: int,
        flags: int = 0,
    ) -> Packet:
        payload = struct.pack(">BHB", speed & 0xFF, heading & 0xFFFF, flags & 0xFF)
        return self._packet(DID_DRIVE, self.CID_DRIVE_WITH_HEADING, sequence_id, TARGET_MCU, payload)

    def raw_motors(
        self,
        sequence_id: int,
        left_mode: int,
        left_speed: int,
        right_mode: int,
        right_speed: int,
    ) -> Packet:
        payload = struct.pack(">BBBB", left_mode & 0xFF, left_speed & 0xFF, right_mode & 0xFF, right_speed & 0xFF)
        return self._packet(DID_DRIVE, self.CID_RAW_MOTORS, sequence_id, TARGET_MCU, payload)

    def stop(self, sequence_id: int) -> Packet:
        return self.raw_motors(sequence_id, 0, 0, 0, 0)

    def emergency_stop(self, sequence_id: int) -> Packet:
        return self.raw_motors(sequence_id, 0, 0, 0, 0)

    def clear_emergency_stop(self, sequence_id: int) -> Packet:
        return Packet(DID_DRIVE, self.CLEAR_EMERGENCY_STOP, sequence_id, target=TARGET_MCU)

    def drive_rc(self, sequence_id: int, linear_mps: float, angular_rad_s: float, max_speed: int = 255) -> Packet:
        """Map ROS-style velocity intent to hardware-real raw motor command.

        Both inputs are expected to be clamped by the driver before reaching
        this method. Values are normalized into -1.0..1.0 tank-track commands:
        left = linear + angular, right = linear - angular.

        `max_speed` caps the generated raw-motor duty cycle. Keep this lower
        for floor testing so ordinary `/cmd_vel` inputs cannot jump straight to
        the full 255 duty range.
        """
        left = float(linear_mps) + float(angular_rad_s)
        right = float(linear_mps) - float(angular_rad_s)
        max_value = max(abs(left), abs(right), 1.0)
        left /= max_value
        right /= max_value
        max_speed = max(0, min(255, int(max_speed)))
        left_mode, left_speed = self._track_mode_and_speed(left, max_speed)
        right_mode, right_speed = self._track_mode_and_speed(right, max_speed)
        return self.raw_motors(sequence_id, left_mode, left_speed, right_mode, right_speed)

    @staticmethod
    def _track_mode_and_speed(value: float, max_speed: int = 255):
        speed = max(0, min(max_speed, int(abs(value) * max_speed)))
        if speed == 0:
            return 0, 0
        return (2 if value < 0 else 1), speed

    def reset_yaw(self, sequence_id: int) -> Packet:
        return self._packet(DID_DRIVE, self.CID_RESET_YAW, sequence_id, TARGET_MCU)

    def reset_locator(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_RESET_LOCATOR, sequence_id, TARGET_MCU)

    def drive_to_position_si(
        self,
        sequence_id: int,
        yaw_angle: float,
        x: float,
        y: float,
        linear_speed: float,
        flags: int = 0,
    ) -> Packet:
        payload = struct.pack(">ffffB", float(yaw_angle), float(x), float(y), float(linear_speed), flags & 0xFF)
        return Packet(
            DID_DRIVE,
            self.CID_DRIVE_TO_POSITION_SI,
            sequence_id,
            payload=payload,
            target=None,
            flags=0x06,
        )

    def set_all_leds(self, sequence_id: int, r: int, g: int, b: int) -> Packet:
        payload = struct.pack(">I", 0x3FFFFFFF) + bytes([r & 0xFF, g & 0xFF, b & 0xFF] * 10)
        return self._packet(DID_IO, self.CID_SET_ALL_LEDS, sequence_id, TARGET_BT, payload)

    def set_led_group(self, sequence_id: int, group_name: str, r: int, g: int, b: int) -> Packet:
        if group_name not in LED_GROUPS:
            raise ValueError(f"Unknown LED group: {group_name}. Valid groups: {list(LED_GROUPS.keys())}")
        base_bit = LED_GROUPS[group_name] * 3
        led_bitmap = (1 << base_bit) | (1 << (base_bit + 1)) | (1 << (base_bit + 2))
        brightness = bytearray(30)
        brightness[base_bit] = r & 0xFF
        brightness[base_bit + 1] = g & 0xFF
        brightness[base_bit + 2] = b & 0xFF
        payload = struct.pack(">I", led_bitmap) + bytes(brightness)
        return self._packet(DID_IO, self.CID_SET_ALL_LEDS, sequence_id, TARGET_BT, payload)

    def get_battery_percentage(self, sequence_id: int) -> Packet:
        return self._packet(DID_POWER, self.CID_GET_BATTERY_PERCENTAGE, sequence_id, TARGET_BT, request_response=True)

    def get_battery_voltage(self, sequence_id: int) -> Packet:
        return self._packet(
            DID_POWER,
            self.CID_GET_BATTERY_VOLTAGE,
            sequence_id,
            TARGET_BT,
            payload=struct.pack(">B", 0),
            request_response=True,
        )

    def get_battery_voltage_state(self, sequence_id: int) -> Packet:
        return self._packet(DID_POWER, self.CID_GET_BATTERY_VOLTAGE_STATE, sequence_id, TARGET_BT, request_response=True)

    def get_battery_thresholds(self, sequence_id: int) -> Packet:
        return self._packet(DID_POWER, self.CID_GET_BATTERY_THRESHOLDS, sequence_id, TARGET_BT, request_response=True)

    def get_ambient_light(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_GET_AMBIENT_LIGHT, sequence_id, TARGET_BT, request_response=True)

    def get_rgbc_sensor_values(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_GET_RGBC_SENSOR, sequence_id, TARGET_BT, request_response=True)

    def enable_color_detection(self, sequence_id: int, enabled: bool = True) -> Packet:
        return self._packet(
            DID_SENSOR,
            self.CID_ENABLE_COLOR_DETECTION,
            sequence_id,
            TARGET_BT,
            payload=struct.pack(">B", 1 if enabled else 0),
            request_response=True,
        )

    def enable_color_detection_notify(
        self,
        sequence_id: int,
        enabled: bool = True,
        interval_ms: int = 250,
        confidence: int = 0,
    ) -> Packet:
        payload = struct.pack(">BHB", 1 if enabled else 0, interval_ms & 0xFFFF, confidence & 0xFF)
        return self._packet(DID_SENSOR, self.CID_ENABLE_COLOR_DETECTION_NOTIFY, sequence_id, TARGET_BT, payload)

    def get_current_detected_color(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_GET_CURRENT_DETECTED_COLOR, sequence_id, TARGET_BT, request_response=True)

    def get_temperature(self, sequence_id: int) -> Packet:
        return self._packet(
            DID_SENSOR,
            self.CID_GET_TEMPERATURE,
            sequence_id,
            TARGET_MCU,
            payload=struct.pack(">BB", 4, 5),
            request_response=True,
        )

    def get_thermal_protection_status(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_GET_THERMAL_PROTECTION, sequence_id, TARGET_MCU, request_response=True)

    def get_encoder_counts(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_GET_ENCODER_COUNTS, sequence_id, TARGET_MCU, request_response=True)

    def get_magnetometer(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_GET_MAGNETOMETER, sequence_id, TARGET_MCU, request_response=True)

    def calibrate_magnetometer(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_CALIBRATE_MAGNETOMETER, sequence_id, TARGET_MCU)

    def get_motor_fault_state(self, sequence_id: int) -> Packet:
        return self._packet(DID_DRIVE, self.CID_GET_MOTOR_FAULT, sequence_id, TARGET_MCU, request_response=True)

    def enable_motor_stall_notify(self, sequence_id: int, enabled: bool = True) -> Packet:
        return self._packet(
            DID_DRIVE,
            self.CID_ENABLE_MOTOR_STALL_NOTIFY,
            sequence_id,
            TARGET_MCU,
            payload=struct.pack(">B", 1 if enabled else 0),
        )

    def enable_motor_fault_notify(self, sequence_id: int, enabled: bool = True) -> Packet:
        return self._packet(
            DID_DRIVE,
            self.CID_ENABLE_MOTOR_FAULT_NOTIFY,
            sequence_id,
            TARGET_MCU,
            payload=struct.pack(">B", 1 if enabled else 0),
        )

    def get_main_app_version(self, sequence_id: int, target: int = TARGET_BT) -> Packet:
        return self._packet(DID_SYSTEM_INFO, self.CID_GET_MAIN_APP_VERSION, sequence_id, target, request_response=True)

    def get_mac_address(self, sequence_id: int) -> Packet:
        return self._packet(DID_SYSTEM_INFO, self.CID_GET_MAC_ADDRESS, sequence_id, TARGET_BT, request_response=True)

    def get_board_revision(self, sequence_id: int, target: int = TARGET_BT) -> Packet:
        return self._packet(DID_SYSTEM_INFO, self.CID_GET_BOARD_REVISION, sequence_id, target, request_response=True)

    def get_processor_name(self, sequence_id: int, target: int = TARGET_BT) -> Packet:
        return self._packet(DID_SYSTEM_INFO, self.CID_GET_PROCESSOR_NAME, sequence_id, target, request_response=True)

    def get_sku(self, sequence_id: int) -> Packet:
        return self._packet(DID_SYSTEM_INFO, self.CID_GET_SKU, sequence_id, TARGET_BT, request_response=True)

    def get_core_uptime(self, sequence_id: int, target: int = TARGET_BT) -> Packet:
        return self._packet(DID_SYSTEM_INFO, self.CID_GET_CORE_UPTIME, sequence_id, target, request_response=True)

    def send_ir_message(self, sequence_id: int, code: int, strength: int = 32) -> Packet:
        return self._packet(DID_IR, self.CID_SEND_IR_MESSAGE, sequence_id, TARGET_BT, struct.pack(">BB", code & 0xFF, strength & 0xFF))

    def start_ir_broadcast(self, sequence_id: int, far_code: int, near_code: int) -> Packet:
        return self._packet(DID_IR, self.CID_START_IR_BROADCAST, sequence_id, TARGET_BT, struct.pack(">BB", far_code & 0xFF, near_code & 0xFF))

    def stop_ir_broadcast(self, sequence_id: int) -> Packet:
        return self._packet(DID_IR, self.CID_STOP_IR_BROADCAST, sequence_id, TARGET_BT)

    def start_ir_following(self, sequence_id: int, far_code: int, near_code: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_START_IR_FOLLOWING, sequence_id, TARGET_BT, struct.pack(">BB", far_code & 0xFF, near_code & 0xFF))

    def stop_ir_following(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_STOP_IR_FOLLOWING, sequence_id, TARGET_BT)

    def start_ir_evading(self, sequence_id: int, far_code: int, near_code: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_START_IR_EVADING, sequence_id, TARGET_BT, struct.pack(">BB", far_code & 0xFF, near_code & 0xFF))

    def stop_ir_evading(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_STOP_IR_EVADING, sequence_id, TARGET_BT)

    def get_ir_readings(self, sequence_id: int) -> Packet:
        return self._packet(DID_SENSOR, self.CID_GET_IR_READINGS, sequence_id, TARGET_MCU, request_response=True)

    def _packet(
        self,
        did: int,
        cid: int,
        sequence_id: int,
        target: int,
        payload: bytes = b"",
        request_response: bool = False,
    ) -> Packet:
        flags = FLAG_HAS_TARGET | FLAG_IS_ACTIVITY
        if request_response:
            flags |= FLAG_REQUEST_RESPONSE
        return Packet(did, cid, sequence_id, payload=payload, target=target, flags=flags)
