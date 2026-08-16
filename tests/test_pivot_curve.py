"""The velocity truth layer: a commanded rate must map to a duty that PRODUCES it.

These tests pin the module to the 2026-08-16 measurement
(`03_validation/breakaway_2026-08-16/README_run4.md`). If someone edits a curve constant,
these go red and name the run that would have to be re-flown to justify it.

The behaviour being replaced had NO tests at all -- the closed-loop pivot's ramp was never
exercised by this suite, which is part of why it survived three weeks while pinned at its
floor. The replacement does not get to be in that position.
"""

import pytest

from sphero_rvr_core import pivot_curve as pc


# --------------------------------------------------------------------------------------
# The curve is the measurement, and the measurement is four numbers on a floor
# --------------------------------------------------------------------------------------

MEASURED = {23: 2.895, 28: 3.568, 32: 4.034, 45: 5.852}


@pytest.mark.parametrize("duty, measured", sorted(MEASURED.items()))
def test_the_fit_reproduces_every_rung_it_was_fitted_to(duty, measured):
    # A least-squares fit through 4 points sits within a few hundredths of each. If a
    # constant is edited, this is the test that says "which run says so?".
    assert pc.rate_for_duty(duty) == pytest.approx(measured, abs=0.06)


def test_the_curve_is_monotonic_and_increasing_across_the_valid_band():
    duties = range(pc.CURVE_VALID_DUTY_MIN, pc.CURVE_VALID_DUTY_MAX + 1)
    rates = [pc.rate_for_duty(d) for d in duties]
    assert all(b > a for a, b in zip(rates, rates[1:]))


def test_duty_for_rate_inverts_rate_for_duty_inside_the_band():
    for duty in range(pc.CURVE_VALID_DUTY_MIN, pc.CURVE_VALID_DUTY_MAX + 1):
        assert pc.duty_for_rate(pc.rate_for_duty(duty)) == pytest.approx(duty, abs=1e-6)


def test_the_band_boundaries_are_the_ones_that_were_measured_clean():
    # 23..45 pivoted cleanly (<= 2.5 cm translation). 12 and 16 walked. The walk band's
    # upper edge is UNKNOWN, so it must extend up to the first duty proven clean.
    assert pc.CURVE_VALID_DUTY_MIN == 23
    assert pc.CURVE_VALID_DUTY_MAX == 45
    assert pc.DEAD_ZONE_MAX_DUTY == 10
    assert pc.WALK_BAND_MAX_DUTY == pc.CURVE_VALID_DUTY_MIN - 1, (
        "the walk band must reach the first proven-clean duty; leaving a gap would let "
        "a duty nobody has measured be treated as safe"
    )
    assert pc.WALK_BAND_MIN_DUTY > pc.DEAD_ZONE_MAX_DUTY


# --------------------------------------------------------------------------------------
# The three impossible rates -- the whole reason this module exists
# --------------------------------------------------------------------------------------

# 0.4 = the collision supervisor's clamp AND rvr_node's second copy of it.
# 0.9 = the decisive controller's pivot rate.
# 1.3 = the retired closed loop's own internal target.
# Measured: duties 2..10 give EXACTLY 0.000 rad/s, duty 12 gives 1.477. None of these
# three rates is producible by any duty; 1.3 lands inside the bimodal walk band.
IMPOSSIBLE_RATES = (0.4, 0.9, 1.3)


@pytest.mark.parametrize("rate", IMPOSSIBLE_RATES)
def test_the_historically_commanded_rates_are_below_anything_the_drivetrain_can_do(rate):
    assert rate < pc.minimum_clean_rate(pc.CURVE_VALID_DUTY_MIN)


@pytest.mark.parametrize("rate", IMPOSSIBLE_RATES)
def test_an_impossible_rate_is_raised_to_the_floor_and_says_so(rate):
    plan = pc.plan_pivot(rate, min_duty=28, max_duty=45)

    assert plan.policy == "raised_to_minimum"
    assert plan.duty == 28
    assert plan.achieved_rate_rad_s == pytest.approx(pc.rate_for_duty(28))
    assert plan.requested_rate_rad_s == rate
    assert not plan.is_honoured, "a substituted rate must never report as honoured"
    assert plan.note and "below the slowest clean pivot" in plan.note
    assert pc.CURVE_CITATION in plan.note


@pytest.mark.parametrize("rate", IMPOSSIBLE_RATES)
def test_raising_is_not_faster_than_the_behaviour_it_replaces(rate):
    # The retired loop drove pivot_min_duty regardless of the request, so the floor duty
    # is exactly what these rates already produced. This change makes the number honest;
    # it must not make the robot turn faster than it already did.
    plan = pc.plan_pivot(rate, min_duty=28, max_duty=45)
    assert abs(plan.duty) == 28


# --------------------------------------------------------------------------------------
# No duty in the cliff, ever
# --------------------------------------------------------------------------------------

@pytest.mark.parametrize("rate", [0.0001, 0.05, 0.4, 1.0, 2.0, 2.8])
def test_no_sub_floor_request_can_produce_a_duty_inside_the_dead_or_walk_band(rate):
    plan = pc.plan_pivot(rate, min_duty=23, max_duty=45)
    assert abs(plan.duty) == 0 or abs(plan.duty) >= pc.CURVE_VALID_DUTY_MIN


def test_the_planner_never_emits_a_duty_between_zero_and_the_floor():
    # Sweep the whole plausible request range, both signs, at both deployed floors.
    for min_duty in (23, 28):
        r = -8.0
        while r <= 8.0:
            plan = pc.plan_pivot(r, min_duty=min_duty, max_duty=45)
            assert plan.duty == 0 or min_duty <= abs(plan.duty) <= 45, (
                f"rate {r} produced duty {plan.duty}, inside the measured walk band"
            )
            r += 0.01


def test_a_zero_request_stops_rather_than_creeping():
    plan = pc.plan_pivot(0.0, min_duty=23, max_duty=45)
    assert plan.duty == 0 and plan.policy == "stopped" and plan.is_honoured


# --------------------------------------------------------------------------------------
# Ordinary in-band behaviour
# --------------------------------------------------------------------------------------

# NB 5.83, not the measured 5.852: the least-squares fit at duty 45 predicts 5.834, a
# little under what that rung actually did. Asking for the measured value is therefore
# ABOVE the band and correctly caps -- a small thing, but exactly the kind of gap between
# "what we measured" and "what the model says" that should live in a test rather than in
# someone's memory.
@pytest.mark.parametrize("rate", [2.9, 3.0, 3.568, 4.034, 5.0, 5.83])
def test_a_producible_rate_is_honoured_and_lands_within_half_a_duty_of_the_ask(rate):
    plan = pc.plan_pivot(rate, min_duty=23, max_duty=45)

    assert plan.policy == "exact" and plan.is_honoured
    # Duty is an integer, so the achieved rate can differ by up to half a duty step.
    assert plan.achieved_rate_rad_s == pytest.approx(
        rate, abs=pc.CURVE_SLOPE_RAD_S_PER_DUTY / 2 + 1e-9
    )


@pytest.mark.parametrize("rate, expected_sign", [(3.5, 1), (-3.5, -1)])
def test_sign_is_preserved_and_only_the_sign_comes_from_the_command(rate, expected_sign):
    plan = pc.plan_pivot(rate, min_duty=23, max_duty=45)
    assert plan.duty * expected_sign > 0
    assert abs(plan.duty) == abs(pc.plan_pivot(-rate, min_duty=23, max_duty=45).duty)


def test_a_rate_above_the_band_is_capped_and_flagged():
    plan = pc.plan_pivot(12.0, min_duty=23, max_duty=45)

    assert plan.policy == "capped_at_maximum"
    assert plan.duty == 45
    assert plan.achieved_rate_rad_s == pytest.approx(pc.rate_for_duty(45))
    assert not plan.is_honoured


def test_the_deployed_band_sets_the_floor_not_this_module():
    # config/lean_rvr_tank_si.yaml runs 28..45; the dataclass defaults are 23..32.
    missions = pc.plan_pivot(1.0, min_duty=28, max_duty=45)
    defaults = pc.plan_pivot(1.0, min_duty=23, max_duty=32)

    assert abs(missions.duty) == 28 and abs(defaults.duty) == 23
    assert missions.achieved_rate_rad_s > defaults.achieved_rate_rad_s


def test_an_inverted_band_is_a_configuration_error_not_a_silent_clamp():
    with pytest.raises(ValueError):
        pc.plan_pivot(3.0, min_duty=45, max_duty=23)


def test_the_module_records_the_conditions_its_numbers_are_only_valid_under():
    # A rate derived from this curve is voltage- and surface-dependent. If the provenance
    # is deleted, the numbers become folklore again -- which is how "<=128 does not move"
    # survived onto a scale it was never measured on.
    assert "rubber gym" in pc.MEASURED_SURFACE
    assert pc.MEASURED_BATTERY_PCT >= 25
    assert "breakaway_2026-08-16" in pc.CURVE_CITATION


def test_the_measured_top_rung_sits_just_above_what_the_fit_promises():
    # Documenting the residual rather than hiding it: duty 45 measured 5.852, the fit says
    # 5.834. Requests between the two cap at maximum, which is the conservative direction.
    assert pc.rate_for_duty(45) < 5.852
    assert 5.852 - pc.rate_for_duty(45) < 0.05


def test_the_in_band_rounding_cannot_escape_the_band():
    # This is why the final clamp in plan_pivot survives mutation: it is UNREACHABLE.
    # The floor/ceiling branches bound the magnitude first, so the rounded duty is already
    # inside the band for every in-band rate. Proving it here turns "a mutation survived"
    # from an unknown into a documented invariant.
    for min_duty, max_duty in ((23, 45), (28, 45), (23, 32)):
        lo, hi = pc.minimum_clean_rate(min_duty), pc.maximum_clean_rate(max_duty)
        steps = 400
        for i in range(steps + 1):
            rate = lo + (hi - lo) * i / steps
            duty = int(round(pc.duty_for_rate(rate)))
            assert min_duty <= duty <= max_duty, (
                f"rate {rate} rounded to duty {duty}, outside [{min_duty},{max_duty}] -- "
                "the clamp IS load-bearing after all and this comment is wrong"
            )
