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


def summarize_state(state: RVRState) -> DiagnosticSummary:
    if state.emergency_stopped:
        return DiagnosticSummary(level="ERROR", message="RVR emergency stop is active")
    if not state.connected:
        return DiagnosticSummary(level="WARN", message="RVR is disconnected")
    return DiagnosticSummary(level="OK", message="RVR driver connected")
