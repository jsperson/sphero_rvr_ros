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


def is_pivot_command(linear_mps: float, angular_rad_s: float, linear_epsilon_mps: float) -> bool:
    """Would this command take the in-place pivot path? Sanitised inputs only.

    One definition of "pivot", used by the clamp and by the control loop alike. When those
    two disagreed about which path a command was on, the clamp governed a path the command
    never took -- which is the whole D45 story in one sentence.
    """
    return abs(finite_or_zero(linear_mps)) < linear_epsilon_mps and abs(
        finite_or_zero(angular_rad_s)
    ) > 0.0


def clamp_velocity_for_path(
    command: VelocityCommand,
    *,
    max_linear_mps: float,
    max_angular_rad_s: float,
    max_pivot_rate_rad_s: float,
    is_pivot: bool,
) -> VelocityCommand:
    """Clamp against the authority for the path this command will ACTUALLY take.

    Two regimes, two measurements, two limits:

    * **In-place pivots** go through ``drive_tank_normalized`` and are governed by the
      measured curve (``pivot_curve``). Their ceiling is ``max_pivot_rate_rad_s``, derived
      from the curve at the deployed ``pivot_max_duty``.
    * **Arcs while driving** mix linear and angular into tread speeds and are governed by
      ``max_angular_rad_s``, which is **UNMEASURED** and stays at its current value.

    Why the arc limit is not raised to the curve's ceiling, since it is tempting and
    wrong: the curve was measured on in-place pivots only. Applying it to arcs would set a
    tank differential of ``angular * wheel_track`` = 5.83 * 0.2507 ≈ **±0.73 m/s** against
    a ``max_linear_mps`` of 0.20 -- nearly 4x the rover's own linear limit, on a regime
    nobody has measured. That is precisely the error class that put a raw-motor 0-255
    figure onto a ±127 tank scale and cost this project a wrong autopsy. The gap is
    admitted and has a close path (an arc-rate run in the run-card family); it is not
    silently absorbed.
    """
    angular_limit = max_pivot_rate_rad_s if is_pivot else max_angular_rad_s
    return VelocityCommand(
        linear_mps=clamp(finite_or_zero(command.linear_mps), -max_linear_mps, max_linear_mps),
        angular_rad_s=clamp(finite_or_zero(command.angular_rad_s), -angular_limit, angular_limit),
    )


def is_stale(last_update: Optional[float], timeout_seconds: float, now: Optional[float] = None) -> bool:
    if last_update is None:
        return True
    current = now_seconds() if now is None else now
    return current - last_update > timeout_seconds
