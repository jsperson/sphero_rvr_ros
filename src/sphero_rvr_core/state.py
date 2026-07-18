"""Driver state models."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class VelocityCommand:
    linear_mps: float
    angular_rad_s: float


@dataclass(frozen=True)
class RVRState:
    connected: bool = False
    emergency_stopped: bool = False
    latest_velocity: Optional[VelocityCommand] = None
    fail_safe_active: bool = False
    fail_safe_reason: Optional[str] = None
