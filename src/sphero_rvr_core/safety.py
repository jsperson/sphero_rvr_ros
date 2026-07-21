"""Safety helpers for clamping and timeout decisions."""

import math
import time
from typing import Optional

from .state import VelocityCommand


def now_seconds() -> float:
    return time.monotonic()


def clamp(value: float, lower: float, upper: float) -> float:
    if not math.isfinite(float(value)):
        raise ValueError("cannot clamp non-finite value")
    return max(lower, min(upper, value))


def finite_or_zero(value: float) -> float:
    """Return a finite motion value or fail closed to zero."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return 0.0
    return result if math.isfinite(result) else 0.0


def clamp_velocity(command: VelocityCommand, max_linear_mps: float, max_angular_rad_s: float) -> VelocityCommand:
    return VelocityCommand(
        linear_mps=clamp(finite_or_zero(command.linear_mps), -max_linear_mps, max_linear_mps),
        angular_rad_s=clamp(finite_or_zero(command.angular_rad_s), -max_angular_rad_s, max_angular_rad_s),
    )


def is_stale(last_update: Optional[float], timeout_seconds: float, now: Optional[float] = None) -> bool:
    if last_update is None:
        return True
    current = now_seconds() if now is None else now
    return current - last_update > timeout_seconds
