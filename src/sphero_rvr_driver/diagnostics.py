"""Diagnostics helpers kept independent of ROS imports for unit testing."""

from dataclasses import dataclass

from sphero_rvr_core.state import RVRState


@dataclass(frozen=True)
class DiagnosticSummary:
    level: str
    message: str


def summarize_state(state: RVRState) -> DiagnosticSummary:
    if state.emergency_stopped:
        return DiagnosticSummary(level="ERROR", message="RVR emergency stop is active")
    if not state.connected:
        return DiagnosticSummary(level="WARN", message="RVR is disconnected")
    return DiagnosticSummary(level="OK", message="RVR driver connected")
