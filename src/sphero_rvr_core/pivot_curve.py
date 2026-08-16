"""The measured relationship between tank duty and in-place yaw rate. ONE AUTHORITY.

Every layer in this stack used to hold its own opinion about pivot rate -- the decisive
controller's 0.9, the collision supervisor's 0.4, ``rvr_node``'s second 0.4, and the pivot
loop's internal 1.3 target -- and **not one of them was executable by this drivetrain.**
2026-08-16 measured why, on the rover's operating surface (rubber gym flooring, 80-83 %
battery, binary 4180c2c). Evidence and four run reports:
``03_validation/breakaway_2026-08-16/``.

    duty 23 -> 2.895 rad/s      duty 32 -> 4.034 rad/s
    duty 28 -> 3.568 rad/s      duty 45 -> 5.852 rad/s

    fit over that band:  rate ~= 0.1344 * duty - 0.213

Below the band the relationship is **not a line, it is a cliff**:

* duties 2..10 produce **exactly 0.000 rad/s** -- mean AND peak -- while 41 motor packets
  per burst reach the transport. Dead, not slow.
* duties 12 and 16 are **bimodal**: each produced a clean pivot once and a one-tread arc
  the other time, walking 15.9 cm and 21.8 cm respectively. The upper edge of that walk
  band is somewhere between 16 and 23; 23..45 all pivoted cleanly (<= 2.5 cm).

So the drivetrain's rate curve **jumps from zero to ~0.8-1.5 rad/s**, and the old
constants 0.4 and 0.9 fall in the gap: *no duty produces them*. 1.3 lands in the walk
band. That is why this module exists and why nothing may extrapolate the fit downward.

WHAT THIS MODULE IS NOT: it describes IN-PLACE PIVOTS driven through
``drive_tank_normalized``. It says nothing about arcs while driving, which are a different
regime on a different command path and remain UNMEASURED.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# --------------------------------------------------------------------------------------
# The measurement. Changing any number here without a new run 4 is falsifying data.
# --------------------------------------------------------------------------------------

CURVE_SLOPE_RAD_S_PER_DUTY = 0.13440
CURVE_INTERCEPT_RAD_S = -0.21361

#: The duties the fit was measured over. It is valid here and nowhere else.
CURVE_VALID_DUTY_MIN = 23
CURVE_VALID_DUTY_MAX = 45

#: Duties measured at EXACTLY 0.000 rad/s with motor packets provably written.
DEAD_ZONE_MAX_DUTY = 10

#: Duties measured bimodal -- clean pivot once, one-tread walk the other time. The upper
#: edge is unknown; it is above 16 and at or below 23. Treated as the whole 11..22 range,
#: because a band whose edge is unmeasured must be assumed hostile up to the first duty
#: proven clean.
WALK_BAND_MIN_DUTY = 11
WALK_BAND_MAX_DUTY = CURVE_VALID_DUTY_MIN - 1

#: Conditions the curve was taken under. A rate derived from it is only valid near these.
MEASURED_SURFACE = "rubber gym flooring"
MEASURED_BATTERY_PCT = 80

CURVE_CITATION = "03_validation/breakaway_2026-08-16/README_run4.md"

#: What counts as "in place". ONE definition, shared by the driver's control loop and by
#: the velocity clamp -- when those two disagreed about which path a command was on, the
#: clamp governed a path the command never took.
PIVOT_LINEAR_EPSILON_MPS = 0.005


def rate_for_duty(duty: float) -> float:
    """Yaw rate (rad/s, magnitude) the curve predicts for a tank duty magnitude.

    Only meaningful inside ``CURVE_VALID_DUTY_MIN..CURVE_VALID_DUTY_MAX``. Callers that
    need a duty from a rate must use :func:`plan_pivot`, which refuses to leave the band.
    """
    return CURVE_SLOPE_RAD_S_PER_DUTY * float(duty) + CURVE_INTERCEPT_RAD_S


def duty_for_rate(rate_rad_s: float) -> float:
    """Inverse of :func:`rate_for_duty`, unclamped. NOT a public planning entry point.

    Deliberately returns a float and does not clamp: every caller must decide what to do
    about a rate that lands in the cliff, and :func:`plan_pivot` is where that policy
    lives. Using this directly is how the extrapolation error gets made.
    """
    return (float(rate_rad_s) - CURVE_INTERCEPT_RAD_S) / CURVE_SLOPE_RAD_S_PER_DUTY


@dataclass(frozen=True)
class PivotPlan:
    """A signed tank duty, the rate it will actually produce, and what happened."""

    duty: int
    achieved_rate_rad_s: float
    requested_rate_rad_s: float
    #: "stopped" | "exact" | "raised_to_minimum" | "capped_at_maximum"
    policy: str
    note: Optional[str] = None

    @property
    def is_honoured(self) -> bool:
        return self.policy in ("stopped", "exact")


def minimum_clean_rate(min_duty: int) -> float:
    return rate_for_duty(min_duty)


def maximum_clean_rate(max_duty: int) -> float:
    return rate_for_duty(max_duty)


def plan_pivot(
    angular_rad_s: float,
    *,
    min_duty: int,
    max_duty: int,
    zero_epsilon: float = 1e-9,
) -> PivotPlan:
    """Turn a requested yaw rate into the duty that PRODUCES it.

    ``min_duty``/``max_duty`` come from the deployed config (``pivot_min_duty`` /
    ``pivot_max_duty``), so the band is the operator's to set and this module's to honour.

    **Policy for a request below the minimum producible rate: RAISE IT, and say so.**
    Refusing would leave the robot unable to pivot at all whenever an upstream layer asks
    for a rate written before the curve existed -- and every such layer currently does.
    Raising is also strictly *less* aggressive than the behaviour it replaces: the old
    closed loop drove the floor duty regardless of the request, so a 0.4 rad/s ask already
    executed at ``rate_for_duty(min_duty)``. This changes nothing about how fast the robot
    turns at the floor; it only makes the number honest and logs the substitution.

    **No duty between 1 and ``min_duty`` is ever emitted.** That range is the measured
    dead zone plus the bimodal walk band, and a pivot that walks is how the rover ends up
    somewhere nobody commanded.
    """
    if min_duty > max_duty:
        raise ValueError(f"min_duty {min_duty} exceeds max_duty {max_duty}")

    requested = float(angular_rad_s)
    if abs(requested) <= zero_epsilon:
        return PivotPlan(0, 0.0, requested, "stopped")

    sign = 1 if requested > 0.0 else -1
    magnitude = abs(requested)

    floor_rate = minimum_clean_rate(min_duty)
    ceil_rate = maximum_clean_rate(max_duty)

    if magnitude < floor_rate:
        return PivotPlan(
            sign * min_duty,
            floor_rate,
            requested,
            "raised_to_minimum",
            note=(
                f"{magnitude:.3f} rad/s is below the slowest clean pivot this drivetrain "
                f"can make ({floor_rate:.3f} rad/s at duty {min_duty}); duties under "
                f"{min_duty} are the measured dead zone and walk band. Raised. "
                f"See {CURVE_CITATION}."
            ),
        )
    if magnitude > ceil_rate:
        return PivotPlan(
            sign * max_duty,
            ceil_rate,
            requested,
            "capped_at_maximum",
            note=(
                f"{magnitude:.3f} rad/s is above the fastest pivot the deployed band "
                f"allows ({ceil_rate:.3f} rad/s at duty {max_duty}). Capped."
            ),
        )

    duty = int(round(duty_for_rate(magnitude)))
    # Belt and braces, and NOT load-bearing: the two branches above already bound
    # `magnitude` to [floor_rate, ceil_rate], so the rounded duty is inside [min, max] by
    # construction -- proven by `test_the_in_band_rounding_cannot_escape_the_band`. Kept
    # because a silent one-line clamp is cheap and the cost of being wrong here is a duty
    # in the walk band; recorded as unreachable so nobody mistakes it for the thing doing
    # the work. This repo has been bitten three times by code that never runs.
    duty = max(min_duty, min(max_duty, duty))
    return PivotPlan(sign * duty, rate_for_duty(duty), requested, "exact")
