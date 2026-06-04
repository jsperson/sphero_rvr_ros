"""Safety helpers for clamping and timeout decisions."""

import time
from typing import Optional

from .state import VelocityCommand


def now_seconds() -> float:
    return time.monotonic()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp_velocity(command: VelocityCommand, max_linear_mps: float, max_angular_rad_s: float) -> VelocityCommand:
    return VelocityCommand(
        linear_mps=clamp(command.linear_mps, -max_linear_mps, max_linear_mps),
        angular_rad_s=clamp(command.angular_rad_s, -max_angular_rad_s, max_angular_rad_s),
    )


def is_stale(last_update: Optional[float], timeout_seconds: float, now: Optional[float] = None) -> bool:
    if last_update is None:
        return True
    current = now_seconds() if now is None else now
    return current - last_update > timeout_seconds
