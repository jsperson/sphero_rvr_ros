"""Typed parsers for Sphero RVR response payloads."""

from dataclasses import dataclass
import struct
from typing import Optional


@dataclass(frozen=True)
class BatteryVoltageState:
    state: int
    state_name: str


@dataclass(frozen=True)
class BatteryThresholds:
    critical: float
    low: float
    hysteresis: float


@dataclass(frozen=True)
class RGBCSensorValues:
    red: int
    green: int
    blue: int
    clear: int


@dataclass(frozen=True)
class DetectedColor:
    red: int
    green: int
    blue: int
    confidence: int
    color_classification_id: int


@dataclass(frozen=True)
class TemperatureReadings:
    left_motor: Optional[float] = None
    right_motor: Optional[float] = None
    nordic_die: Optional[float] = None


@dataclass(frozen=True)
class ThermalProtectionStatus:
    left_temp: float
    left_status: int
    right_temp: float
    right_status: int


@dataclass(frozen=True)
class EncoderCounts:
    left: int
    right: int


@dataclass(frozen=True)
class MagnetometerReadings:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class FirmwareVersion:
    major: int
    minor: int
    revision: int


@dataclass(frozen=True)
class IRReadings:
    front_left: int
    front_right: int
    back_right: int
    back_left: int


def require_length(payload: bytes, minimum: int, name: str) -> None:
    if len(payload) < minimum:
        raise ValueError(f"{name} payload too short: need {minimum}, got {len(payload)}")


def parse_battery_percentage(payload: bytes) -> int:
    require_length(payload, 1, "battery percentage")
    return payload[0]


def parse_battery_voltage(payload: bytes) -> float:
    require_length(payload, 4, "battery voltage")
    return struct.unpack(">f", payload[:4])[0]


def parse_battery_voltage_state(payload: bytes) -> BatteryVoltageState:
    require_length(payload, 1, "battery voltage state")
    state = payload[0]
    state_names = {0: "unknown", 1: "ok", 2: "low", 3: "critical"}
    return BatteryVoltageState(state=state, state_name=state_names.get(state, "unknown"))


def parse_battery_thresholds(payload: bytes) -> BatteryThresholds:
    require_length(payload, 12, "battery thresholds")
    critical, low, hysteresis = struct.unpack(">fff", payload[:12])
    return BatteryThresholds(critical=critical, low=low, hysteresis=hysteresis)


def parse_rgbc_sensor_values(payload: bytes) -> RGBCSensorValues:
    require_length(payload, 8, "RGBC sensor")
    red, green, blue, clear = struct.unpack(">HHHH", payload[:8])
    return RGBCSensorValues(red=red, green=green, blue=blue, clear=clear)


def parse_ambient_light(payload: bytes) -> float:
    require_length(payload, 4, "ambient light")
    return struct.unpack(">f", payload[:4])[0]


def parse_current_detected_color(payload: bytes) -> DetectedColor:
    require_length(payload, 5, "current detected color")
    return DetectedColor(
        red=payload[0],
        green=payload[1],
        blue=payload[2],
        confidence=payload[3],
        color_classification_id=payload[4],
    )


def parse_temperature(payload: bytes) -> TemperatureReadings:
    sensor_names = {4: "left_motor", 5: "right_motor", 8: "nordic_die"}
    values = {}
    index = 0
    while index + 5 <= len(payload):
        sensor_id = payload[index]
        temp = struct.unpack(">f", payload[index + 1:index + 5])[0]
        name = sensor_names.get(sensor_id)
        if name is not None:
            values[name] = temp
        index += 5
    if not values:
        raise ValueError("temperature payload contains no known sensor readings")
    return TemperatureReadings(**values)


def parse_thermal_protection_status(payload: bytes) -> ThermalProtectionStatus:
    require_length(payload, 10, "thermal protection")
    left_temp = struct.unpack(">f", payload[0:4])[0]
    left_status = payload[4]
    right_temp = struct.unpack(">f", payload[5:9])[0]
    right_status = payload[9]
    return ThermalProtectionStatus(left_temp, left_status, right_temp, right_status)


def parse_encoder_counts(payload: bytes) -> EncoderCounts:
    require_length(payload, 8, "encoder counts")
    left, right = struct.unpack(">ii", payload[:8])
    return EncoderCounts(left=left, right=right)


def parse_magnetometer(payload: bytes) -> MagnetometerReadings:
    require_length(payload, 12, "magnetometer")
    x, y, z = struct.unpack(">fff", payload[:12])
    return MagnetometerReadings(x=x, y=y, z=z)


def parse_firmware_version(payload: bytes) -> FirmwareVersion:
    require_length(payload, 6, "firmware version")
    major, minor, revision = struct.unpack(">HHH", payload[:6])
    return FirmwareVersion(major=major, minor=minor, revision=revision)


def parse_null_terminated_ascii(payload: bytes) -> str:
    require_length(payload, 1, "ASCII string")
    return payload.split(b"\x00")[0].decode("ascii", errors="ignore")


def parse_board_revision(payload: bytes) -> int:
    require_length(payload, 1, "board revision")
    return payload[0]


def parse_core_uptime(payload: bytes) -> int:
    require_length(payload, 8, "core uptime")
    return struct.unpack(">Q", payload[:8])[0]


def parse_motor_fault_state(payload: bytes) -> bool:
    require_length(payload, 1, "motor fault state")
    return payload[0] != 0


def parse_ir_readings(payload: bytes) -> IRReadings:
    require_length(payload, 4, "IR readings")
    sensor_data = struct.unpack(">I", payload[:4])[0]
    return IRReadings(
        front_left=sensor_data & 0xFF,
        front_right=(sensor_data >> 8) & 0xFF,
        back_right=(sensor_data >> 16) & 0xFF,
        back_left=(sensor_data >> 24) & 0xFF,
    )
