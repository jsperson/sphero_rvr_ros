"""Diagnostics helpers kept independent of ROS imports for unit testing."""

from dataclasses import dataclass
from typing import Optional

from sphero_rvr_core.safety import clamp
from sphero_rvr_core.state import RVRState


@dataclass(frozen=True)
class DiagnosticSummary:
    level: str
    message: str


@dataclass(frozen=True)
class BatterySnapshot:
    percentage: int
    voltage: Optional[float] = None


@dataclass(frozen=True)
class BatteryStateFields:
    percentage: float
    voltage: float
    present: bool


@dataclass(frozen=True)
class DiagnosticTelemetry:
    battery: Optional[BatterySnapshot] = None
    battery_voltage_state: Optional[str] = None
    motor_fault: Optional[bool] = None
    firmware_version: Optional[str] = None
    board_revision: Optional[int] = None
    processor_name: Optional[str] = None
    core_uptime_s: Optional[int] = None


def battery_state_fields(snapshot: BatterySnapshot) -> BatteryStateFields:
    """Return ROS BatteryState-compatible primitive values.

    `sensor_msgs/BatteryState.percentage` is a 0.0..1.0 fraction, while the RVR
    API reports a whole-number percent.
    """
    return BatteryStateFields(
        percentage=clamp(float(snapshot.percentage) / 100.0, 0.0, 1.0),
        voltage=float("nan") if snapshot.voltage is None else float(snapshot.voltage),
        present=True,
    )


def diagnostic_key_values(
    state: RVRState,
    telemetry: Optional[DiagnosticTelemetry] = None,
) -> dict[str, str]:
    values = {
        "connected": str(state.connected).lower(),
        "emergency_stopped": str(state.emergency_stopped).lower(),
    }
    if state.latest_velocity is not None:
        values.update(
            {
                "linear_mps": f"{state.latest_velocity.linear_mps:.3f}",
                "angular_rad_s": f"{state.latest_velocity.angular_rad_s:.3f}",
            }
        )
    if telemetry is None:
        return values
    if telemetry.battery is not None:
        values["battery_percent"] = str(int(telemetry.battery.percentage))
        if telemetry.battery.voltage is not None:
            values["battery_voltage"] = f"{telemetry.battery.voltage:.3f}"
    if telemetry.battery_voltage_state is not None:
        values["battery_voltage_state"] = telemetry.battery_voltage_state
    if telemetry.motor_fault is not None:
        values["motor_fault"] = str(telemetry.motor_fault).lower()
    if telemetry.firmware_version is not None:
        values["firmware_version"] = telemetry.firmware_version
    if telemetry.board_revision is not None:
        values["board_revision"] = str(telemetry.board_revision)
    if telemetry.processor_name is not None:
        values["processor_name"] = telemetry.processor_name
    if telemetry.core_uptime_s is not None:
        values["core_uptime_s"] = str(telemetry.core_uptime_s)
    return values


def summarize_state(
    state: RVRState,
    telemetry: Optional[DiagnosticTelemetry] = None,
) -> DiagnosticSummary:
    if state.emergency_stopped:
        return DiagnosticSummary(level="ERROR", message="RVR emergency stop is active")
    if telemetry is not None and telemetry.motor_fault:
        return DiagnosticSummary(level="ERROR", message="RVR motor fault is active")
    if not state.connected:
        return DiagnosticSummary(level="WARN", message="RVR is disconnected")
    if telemetry is not None and telemetry.battery_voltage_state in {"low", "critical"}:
        return DiagnosticSummary(level="WARN", message=f"RVR battery voltage is {telemetry.battery_voltage_state}")
    return DiagnosticSummary(level="OK", message="RVR driver connected")
