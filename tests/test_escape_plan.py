"""The plan stage, against the THREE REAL RECORDED WEDGES and the REAL supervisor.

One rule, three poses, and a heading-blind or side-biased rule fails at least one:

    mission 2 (08-14)   forward 0.386 m open, left object 0.166 m at 9   -> 12 o'clock
    08-14b              forward 0.263 m blocked, 2.180 m at 8 o'clock    ->  8 o'clock
    2026-08-15 postspin forward 1.763 m open after the 180 degree spin   -> 12 o'clock

The first two are near mirror images -- mission 2 opens RIGHT, 08-14b opens LEFT --
so a side-biased rule fails one of them. The third is the heading-blindness test, and
it is asserted BOTH with and without a trail, because the trail producer (A1) does not
exist yet and the answer must not depend on machinery that has not landed.

Where a number here is quoted from the archive it is the archive's, not this code's.
Note the population when comparing: the design note's "0.387 m open dead ahead" and
"1.77 m" are different measurements -- the first is the arbiter's +/-30 degree FRONT
SECTOR minimum, the second is the 12 o'clock bucket. Both appear below, labelled.
"""

import json
import math
import os

import pytest

from sphero_rvr_core.escape_survey import (
    VOUCH_LIDAR,
    VOUCH_ORDER,
    VOUCH_TRAIL,
    VOUCH_TOF,
    VOUCH_UNVOUCHED,
    SurveyConfig,
    survey_from_scan,
)
from sphero_rvr_core.escape_plan import (
    ARC_TO_OPENING,
    CAUSE_FREEZE,
    CAUSE_VISIBLE,
    FORWARD_DRIVE,
    TRAIL_RETRACE,
    PlanConfig,
    PlanGates,
    format_plan,
    front_stop_distance_m,
    rank_candidates,
    rear_stop_distance_m,
    shape_is_grantable,
)

FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")
CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")


def _deployed_params():
    """`ros__parameters` out of the DEPLOYED supervisor YAML.

    Never dataclass defaults: thirteen fields have differed between the two, and a
    verdict flipped between them. `stop_distance_m` is one of them right now -- 0.30
    deployed against 0.35 in the dataclass -- and a plan built on 0.35 would withhold
    forward moves the arbiter grants, which is D40 running backwards.
    """
    yaml = pytest.importorskip("yaml")
    raw = yaml.safe_load(open(os.path.join(CONFIG, "collision_stop.yaml")))

    def walk(node):
        if isinstance(node, dict):
            if "ros__parameters" in node:
                return node["ros__parameters"]
            for value in node.values():
                found = walk(value)
                if found:
                    return found
        return None

    params = walk(raw)
    assert params, "no ros__parameters block in the deployed collision_stop.yaml"
    return params


def deployed_gates():
    p = _deployed_params()
    return PlanGates(
        stop_distance_m=float(p["stop_distance_m"]),
        reverse_stop_distance_m=float(p["reverse_stop_distance_m"]),
        footprint_front_m=float(p["footprint_front_m"]),
        footprint_rear_m=float(p["footprint_rear_m"]),
        payload_margin_m=float(p["payload_margin_m"]),
        measured_stop_time_s=float(p["measured_stop_time_s"]),
        braking_distance_margin_m=float(p["braking_distance_margin_m"]),
    )


def load(name):
    with open(os.path.join(FIXTURES, name + ".json")) as fh:
        return json.load(fh)


def survey_of(name, **kwargs):
    fx = load(name)
    return survey_from_scan(
        ranges=fx["ranges"],
        angle_min=fx["angle_min"],
        angle_increment=fx["angle_increment"],
        range_min=fx["range_min"],
        range_max=fx["range_max"],
        laser_yaw_deg=fx["laser_yaw_deg"],
        **kwargs,
    )


# name, cause, the clock the archive says is the way out, the kind that should carry it
ACCEPTANCE = (
    ("wedge_mission2_2026-08-14", CAUSE_VISIBLE, 12, FORWARD_DRIVE),
    ("wedge_20260814b", CAUSE_VISIBLE, 8, ARC_TO_OPENING),
    ("wedge_20260815_postspin", CAUSE_FREEZE, 12, FORWARD_DRIVE),
)


# --------------------------------------------------------------------------
# THE THREE-SPECIMEN ACCEPTANCE
# --------------------------------------------------------------------------

@pytest.mark.parametrize("name,cause,clock,kind", ACCEPTANCE)
def test_the_ranked_first_candidate_matches_the_archived_way_out(name, cause, clock, kind):
    survey = survey_of(name, cause=cause)
    plan = rank_candidates(survey, cause, deployed_gates())

    assert plan, f"{name}: the plan proposed nothing at a pose with a known way out"
    assert plan[0].clock == clock, (
        f"{name}: ranked #1 is {plan[0].kind}@{plan[0].clock} o'clock; the archive says "
        f"{clock} o'clock. Line: {format_plan(plan)}"
    )
    assert plan[0].kind == kind
    assert plan[0].grantable, (
        f"{name}: the top candidate is one the supervisor must refuse -- proposing it "
        f"is the un-grantable-by-construction defect, one layer up"
    )


def test_specimen_3_reaches_forward_WITH_a_trail_and_WITHOUT_one():
    """The 2026-08-15 pose, both ways, because the trail producer does not exist yet.

    After the 180 degree spin THE TRAIL LIES AHEAD: the rover drove into that corner
    nose-first and reoriented, so retracing its entry corridor means driving FORWARD.
    Scott's "it 100% can drive forward clear across the room" and the trail rule give
    the same answer at this pose for the same underlying reason -- that corridor is
    body-proven. Both derivations must land on 12 o'clock, or one of them is wrong.
    """
    gates = deployed_gates()

    without = rank_candidates(
        survey_of("wedge_20260815_postspin", cause=CAUSE_FREEZE),
        CAUSE_FREEZE, gates)
    assert without[0].clock == 12
    assert without[0].kind == FORWARD_DRIVE

    with_trail = rank_candidates(
        survey_of("wedge_20260815_postspin", cause=CAUSE_FREEZE,
                  trail_available=True, trail_first_clock=12, trail_length_m=1.72),
        CAUSE_FREEZE, gates)
    assert with_trail[0].clock == 12
    assert with_trail[0].kind == TRAIL_RETRACE, (
        "on a FREEZE the trail must lead: a freeze proves unvouched space is actively "
        "hostile, and ranking a lidar-open bearing first is ranking the sensor that "
        "just demonstrably missed something"
    )
    assert with_trail[0].vouched_by == VOUCH_TRAIL


def test_the_rule_is_not_side_biased_across_the_mirror_pair():
    """Mission 2 opens RIGHT, 08-14b opens LEFT. One rule, opposite answers.

    Compared at the ARC level so the comparison is like-for-like: mission 2's overall
    winner is a forward drive, which would mask a side bias in the arc ranking.
    """
    gates = deployed_gates()
    m2 = [c for c in rank_candidates(survey_of("wedge_mission2_2026-08-14"),
                                     CAUSE_VISIBLE, gates)
          if c.kind == ARC_TO_OPENING and c.grantable]
    b = [c for c in rank_candidates(survey_of("wedge_20260814b"),
                                    CAUSE_VISIBLE, gates)
         if c.kind == ARC_TO_OPENING and c.grantable]

    assert m2 and b
    assert m2[0].bearing_deg < 0, (
        f"mission 2's best arc should point RIGHT (negative bearing); got "
        f"{m2[0].clock} o'clock at {m2[0].bearing_deg:+.1f} deg"
    )
    assert b[0].bearing_deg > 0, (
        f"08-14b's best arc should point LEFT; got {b[0].clock} o'clock at "
        f"{b[0].bearing_deg:+.1f} deg"
    )
    assert b[0].clock == 8, "08-14b's 2.180 m of floor sits at 8 o'clock"


# --------------------------------------------------------------------------
# PREMISE TRIPWIRES -- these assert the ARBITER's contract, not the plan's.
# They pass with the plan reverted, and that is their job. If one goes red, the
# supervisor moved and every grantability claim beside it is measuring something else.
# --------------------------------------------------------------------------

def _supervisor_with_deployed_config():
    from sphero_rvr_driver.collision_stop import CollisionStopConfig, CollisionStopSupervisor
    from dataclasses import fields

    known = {f.name for f in fields(CollisionStopConfig)}
    params = {k: v for k, v in _deployed_params().items() if k in known}
    # The fixtures are single frames, so scan-health minimums that assume a stream
    # would reject them for a reason that has nothing to do with geometry.
    params["min_valid_ranges"] = 1
    params["min_valid_fraction"] = 0.0
    return CollisionStopSupervisor(CollisionStopConfig(**params), now=0.0)


def _scan_input_from_fixture(name, stamp=0.0):
    from sphero_rvr_driver.collision_stop import ScanInput, Transform2D

    fx = load(name)
    return ScanInput(
        ranges=[r if r is not None else float("inf") for r in fx["ranges"]],
        angle_min=fx["angle_min"],
        angle_increment=fx["angle_increment"],
        range_min=fx["range_min"],
        range_max=fx["range_max"],
        stamp=stamp,
        received_at=stamp,
        transform_to_base=Transform2D(yaw=math.radians(fx["laser_yaw_deg"])),
    )


def test_the_plans_stop_distance_arithmetic_still_equals_the_supervisors():
    """PREMISE TRIPWIRE. Survives its own mutation on purpose.

    The plan lives in `sphero_rvr_core` and the arbiter in `sphero_rvr_driver`, so the
    stop-distance arithmetic is restated rather than imported downward. This is what
    keeps the restatement honest. Going red means the supervisor's arithmetic moved
    and the plan is now withholding or proposing moves against a stale model.
    """
    sup = _supervisor_with_deployed_config()
    from sphero_rvr_driver.collision_stop import TwistCommand

    gates = deployed_gates()
    for speed in (0.05, 0.10, 0.20, -0.05, -0.10, -0.20):
        command = TwistCommand(speed, 0.0)
        assert front_stop_distance_m(gates, speed) == pytest.approx(
            sup._front_stop_distance(command)), f"front, at {speed} m/s"
        assert rear_stop_distance_m(gates, speed) == pytest.approx(
            sup._rear_stop_distance(command)), f"rear, at {speed} m/s"


@pytest.mark.parametrize("name,cause,_clock,_kind", ACCEPTANCE)
def test_every_shape_the_plan_calls_ungrantable_really_yields_no_motion(name, cause, _clock, _kind):
    """PREMISE TRIPWIRE, and the load-bearing one.

    The plan predicts refusal from the survey alone; this runs the SAME shapes through
    the REAL `CollisionStopSupervisor` carrying the DEPLOYED config, fed the SAME
    recorded scan. Only one direction is asserted, deliberately: everything the plan
    calls un-grantable must genuinely produce (0.0, 0.0).

    The converse is NOT asserted, because `shape_is_grantable` is necessary and not
    sufficient -- it answers the two sector gates and cannot answer the trajectory
    projection, which is what actually pinned the arc escape at 08-14b after ~13
    degrees of rotation. Asserting the converse would be asserting a claim the
    function does not make.
    """
    from sphero_rvr_driver.collision_stop import TwistCommand

    survey = survey_of(name, cause=cause)
    plan = rank_candidates(survey, cause, deployed_gates())
    refused = [c for c in plan if not c.grantable]
    if not refused:
        pytest.skip(f"{name}: the plan refused nothing, nothing to cross-check")

    for candidate in refused:
        sup = _supervisor_with_deployed_config()
        sup.update_scan(_scan_input_from_fixture(name, stamp=0.0), now=0.0)
        decision = sup.apply_command(
            TwistCommand(*candidate.first_command), now=0.1)
        assert (decision.output.linear_x, decision.output.angular_z) == (0.0, 0.0), (
            f"{name}: the plan withheld {candidate.kind}@{candidate.clock} o'clock as "
            f"'{candidate.gate_reason}', but the supervisor granted "
            f"({decision.output.linear_x}, {decision.output.angular_z}) with reason "
            f"'{decision.reason}'. Withholding a move the arbiter would have granted "
            f"is the D40 defect running one layer up."
        )


def test_the_08_14b_top_candidate_is_granted_by_the_real_supervisor():
    """The positive half, at the one pose where the archive proves the answer.

    08-14b is the strongest specimen: static geometry, +/-7 mm across the whole
    episode, and `rear_hold` fires with certainty (rear sector 0.150 m inside the
    deployed 0.25 m). The plan's top candidate must obtain MOTION there -- which is
    exactly what the straight reverse could not, four times, over 21 s.
    """
    from sphero_rvr_driver.collision_stop import TwistCommand

    plan = rank_candidates(survey_of("wedge_20260814b"), CAUSE_VISIBLE, deployed_gates())
    top = plan[0]
    sup = _supervisor_with_deployed_config()
    sup.update_scan(_scan_input_from_fixture("wedge_20260814b"), now=0.0)
    decision = sup.apply_command(TwistCommand(*top.first_command), now=0.1)

    assert (decision.output.linear_x, decision.output.angular_z) != (0.0, 0.0), (
        f"the top candidate delivers nothing at 08-14b (reason '{decision.reason}') -- "
        f"the same 0.000 m the field recorded four times"
    )


def test_the_recorded_geometry_still_trips_the_gate_it_is_supposed_to():
    """PREMISE TRIPWIRE on the EVIDENCE, not on the code.

    If a config moves, this should say "this pose no longer reproduces the failure"
    rather than quietly passing for a new reason.
    """
    gates = deployed_gates()
    b = survey_of("wedge_20260814b")
    assert b.rear_sector_min_m is not None
    assert b.rear_sector_min_m < rear_stop_distance_m(gates, -0.10), (
        "08-14b's rear must sit INSIDE the deployed rear-hold distance or this "
        "specimen no longer demonstrates the un-grantable straight reverse at all"
    )
    assert b.front_sector_min_m < front_stop_distance_m(gates, 0.10), (
        "08-14b's forward must be genuinely blocked, or its answer would trivially "
        "be a forward drive and it would stop being the mirror of mission 2"
    )
    m2 = survey_of("wedge_mission2_2026-08-14")
    assert m2.front_sector_min_m > front_stop_distance_m(gates, 0.10), (
        "mission 2's forward must be genuinely available -- 0.387 m in the archive, "
        "and zero forward commands in 683 rows is the whole indictment"
    )


# --------------------------------------------------------------------------
# THE SHAPE RULES
# --------------------------------------------------------------------------

def test_a_straight_reverse_into_a_blocked_rear_is_never_proposed():
    """The D40 defect, refused at plan time. Reverse arcs are fine; straight is not.

    `rear_hold` zeroes linear and passes angular through UNTOUCHED, so a reverse with
    no angular term delivers exactly nothing wherever the rear sector is inside the
    stop distance. Four attempts, four refusals, 0.000 m each (2026-08-14b).
    """
    gates = deployed_gates()
    for name, _cause, _clock, _kind in ACCEPTANCE:
        plan = rank_candidates(survey_of(name), CAUSE_VISIBLE, gates)
        for candidate in plan:
            linear, angular = candidate.first_command
            if linear < 0.0 and candidate.grantable:
                assert angular != 0.0, (
                    f"{name}: {candidate.kind}@{candidate.clock} proposes a STRAIGHT "
                    f"reverse and calls it grantable -- the un-grantable-by-"
                    f"construction shape, re-adopted"
                )


def test_a_reverse_is_grantable_only_because_of_its_angular_term():
    survey = survey_of("wedge_20260814b")
    gates = deployed_gates()
    straight_ok, straight_why = shape_is_grantable((-0.10, 0.0), survey, gates)
    arc_ok, arc_why = shape_is_grantable((-0.10, 0.40), survey, gates)

    assert straight_ok is False and straight_why == "rear_hold"
    assert arc_ok is True and arc_why == "rear_hold_passes_angular"


def test_an_unread_sector_is_a_refusal_not_a_clearance():
    """`sector_unknown_policy: blocked`. No ray is not "no obstacle"."""
    empty = survey_from_scan(
        ranges=[None] * 720, angle_min=-3.14159, angle_increment=0.008726,
        range_min=0.15, range_max=12.0, laser_yaw_deg=178.99,
    )
    gates = deployed_gates()
    assert shape_is_grantable((0.10, 0.0), empty, gates) == (False, "front_sector_unknown")
    assert shape_is_grantable((-0.10, 0.40), empty, gates) == (False, "rear_sector_unknown")


def test_a_survey_that_sees_nothing_proposes_nothing():
    """Degenerate input. Better to propose no exit than a confident one into a wall."""
    empty = survey_from_scan(
        ranges=[None] * 720, angle_min=-3.14159, angle_increment=0.008726,
        range_min=0.15, range_max=12.0, laser_yaw_deg=178.99,
    )
    plan = rank_candidates(empty, CAUSE_FREEZE, deployed_gates())
    assert plan == ()
    assert "NO_EXIT_PROPOSED" in format_plan(plan)


# --------------------------------------------------------------------------
# VOUCHING
# --------------------------------------------------------------------------

def test_the_vouching_order_is_trail_then_tof_then_lidar_then_unvouched():
    """The ruling, pinned. ToF outranks LIDAR because it speaks for the sub-lidar
    band -- the object class that actually pins this robot. All five freezes on
    2026-08-15 were "an obstacle no sensor on this robot can see", and the 0.19 m
    scan plane is structurally blind to exactly that class.

    Inverting ToF and LIDAR here inverts the plan's ranking, so this is not
    documentation.
    """
    assert VOUCH_ORDER == (VOUCH_TRAIL, VOUCH_TOF, VOUCH_LIDAR, VOUCH_UNVOUCHED)


def test_unvouched_ranks_last_but_is_never_forbidden():
    """Most of any room is unvouched at any instant, so a rule that refused unvouched
    space would deadlock a real room. It ranks last and goes slower."""
    survey = survey_of("wedge_20260815_postspin")
    # Make one genuinely open direction speak for nobody.
    from dataclasses import replace as dc_replace
    survey.directions[11] = dc_replace(survey.directions[11], vouched_by=VOUCH_UNVOUCHED)

    plan = rank_candidates(survey, CAUSE_VISIBLE, deployed_gates())
    unvouched = [c for c in plan if c.vouched_by == VOUCH_UNVOUCHED]
    vouched = [c for c in plan if c.vouched_by == VOUCH_LIDAR and c.grantable]

    assert unvouched, "the unvouched direction was dropped instead of ranked last"
    assert unvouched[0].reduced_speed is True
    last_vouched = max(plan.index(c) for c in vouched)
    assert plan.index(unvouched[0]) > last_vouched, (
        "an unvouched direction outranked a vouched one"
    )


def test_a_trail_that_cannot_be_aimed_is_not_proposed():
    """A1 has not landed. A candidate that can never be aimed is the never-triggered
    form of the recovery-defect family, and this project has shipped that twice."""
    survey = survey_of("wedge_20260815_postspin", cause=CAUSE_FREEZE,
                       trail_available=True, trail_first_clock=None)
    plan = rank_candidates(survey, CAUSE_FREEZE, deployed_gates())
    assert not any(c.kind == TRAIL_RETRACE for c in plan)
    assert plan[0].kind == FORWARD_DRIVE, "and the pose still gets its real answer"


def test_a_visible_cause_does_not_hand_the_lead_to_the_trail():
    """Cause-conditioned, both branches. When the sensor CAN see the problem, the
    sensor that can see it chooses the exit."""
    gates = deployed_gates()
    kwargs = dict(trail_available=True, trail_first_clock=6, trail_length_m=1.72)

    frozen = rank_candidates(
        survey_of("wedge_20260815_postspin", cause=CAUSE_FREEZE, **kwargs),
        CAUSE_FREEZE, gates)
    visible = rank_candidates(
        survey_of("wedge_20260815_postspin", cause=CAUSE_VISIBLE, **kwargs),
        CAUSE_VISIBLE, gates)

    assert frozen[0].kind == TRAIL_RETRACE and frozen[0].clock == 6
    assert visible[0].kind == FORWARD_DRIVE, (
        "a visible cause should let the 1.763 m of measured open floor lead, not a "
        "trail pointing back the way the rover came"
    )


# --------------------------------------------------------------------------
# STRUCTURE
# --------------------------------------------------------------------------

def test_plan_gates_cannot_be_built_without_the_deployed_numbers():
    """No defaults, on purpose. A default here is a quiet claim about the deployed
    supervisor config, and thirteen fields have differed between the two before."""
    with pytest.raises(TypeError):
        PlanGates()


def test_the_open_threshold_a_reading_was_judged_against_travels_with_it():
    """`is_open` used to read the CLASS attribute, so it answered against a hardcoded
    0.30 no matter what `SurveyConfig` the caller passed, while `open_clocks` answered
    against the config. Two definitions of "open" that agreed only until somebody
    supplied a real config -- and the plan's adequate-opening test is this predicate.
    """
    tight = survey_of("wedge_20260815_postspin", config=SurveyConfig(open_m=1.0))
    for clock, reading in tight.directions.items():
        assert reading.is_open == (clock in tight.open_clocks), (
            f"clock {clock}: is_open and open_clocks disagree about what open means"
        )
    assert tight.directions[12].is_open       # 1.763 m, above a 1.0 m threshold
    assert not tight.directions[10].is_open   # 0.687 m, below it
    loose = survey_of("wedge_20260815_postspin", config=SurveyConfig(open_m=0.20))
    assert loose.directions[10].is_open, (
        "the same reading must flip with the threshold, or is_open is still reading "
        "a constant instead of the config"
    )


def test_plan_is_pure_and_needs_no_ros():
    """The proofs must bind to this on a machine with no rclpy -- commit 1's lesson,
    applied up front rather than discovered."""
    import sphero_rvr_core.escape_plan as mod

    assert "rclpy" not in getattr(mod, "__dict__", {})


def test_the_plan_line_carries_what_an_autopsy_needs():
    plan = rank_candidates(survey_of("wedge_20260814b"), CAUSE_VISIBLE, deployed_gates())
    line = format_plan(plan)
    assert line.startswith("PLAN ")
    assert "first=arc_to_opening@8oclock" in line
    assert "\n" not in line, "the plan must be ONE greppable line, beside the survey's"
    assert "!front_stop" in line, (
        "the line must say what was CONSIDERED and refused, not only what was chosen"
    )
