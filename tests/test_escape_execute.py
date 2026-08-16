"""The execute stage, proved against the three recorded wedges and the rules' poses.

Every specimen here is an ARCHIVED STUCK POSE replayed through the production survey
and plan, not a hand-drawn scenario -- the same fixtures 2a and 2b are measured on.
"""

import json
import os

import pytest

from sphero_rvr_core.escape_execute import (
    ABORTED, ASK_HUMAN, ESCAPED, Attempt, EscapeExecution, Result, StaleSurvey,
)
from sphero_rvr_core.escape_outcome import CLEARED, DECLINED, FROZEN, REFUSED
from sphero_rvr_core.escape_plan import CAUSE_FREEZE, CAUSE_VISIBLE, PlanGates
from sphero_rvr_core.escape_survey import survey_from_scan

FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")


def load(name):
    with open(os.path.join(FIXTURES, f"{name}.json")) as fh:
        return json.load(fh)


def survey_of(name, cause=CAUSE_VISIBLE, **kwargs):
    fx = load(name)
    survey = survey_from_scan(
        ranges=fx["ranges"], angle_min=fx["angle_min"],
        angle_increment=fx["angle_increment"], range_min=fx["range_min"],
        range_max=fx["range_max"], laser_yaw_deg=fx["laser_yaw_deg"], **kwargs,
    )
    survey.cause = cause
    return survey


def gates():
    """The DEPLOYED gates, never dataclass defaults -- 13 fields have differed between
    them and several decide these verdicts."""
    import re
    from pathlib import Path
    cfg = (Path(__file__).resolve().parents[1] / "config" / "collision_stop.yaml").read_text()

    def val(name):
        m = re.search(rf"^\s*{name}:\s*([0-9.]+)", cfg, re.M)
        assert m, name
        return float(m.group(1))

    return PlanGates(
        stop_distance_m=val("stop_distance_m"),
        reverse_stop_distance_m=val("reverse_stop_distance_m"),
        footprint_front_m=val("footprint_front_m"),
        footprint_rear_m=val("footprint_rear_m"),
        payload_margin_m=val("payload_margin_m"),
        measured_stop_time_s=val("measured_stop_time_s"),
        braking_distance_margin_m=val("braking_distance_margin_m"),
    )


def run(name, outcomes, cause=CAUSE_VISIBLE):
    """Drive an escape through a scripted list of outcomes, re-surveying the SAME pose
    each time (the pessimal case: nothing about the world improved)."""
    ex = EscapeExecution(gates=gates())
    step = ex.begin(survey_of(name, cause), cause)
    seen = []
    for outcome in outcomes:
        if isinstance(step, Result):
            break
        seen.append(step)
        step = ex.report(outcome, survey=survey_of(name, cause))
    return ex, seen, step


# --- the sequencing rules -----------------------------------------------------------

def test_a_failure_without_a_fresh_survey_is_refused():
    """RULE 1, and it is a RAISE rather than a default on purpose. `survey=None`
    silently re-ranking the old snapshot is how an adaptive sequence becomes the fixed
    ladder that produced four identical attempts at one pose."""
    ex = EscapeExecution(gates=gates())
    step = ex.begin(survey_of("wedge_20260815_postspin"), CAUSE_VISIBLE)
    assert isinstance(step, Attempt)
    with pytest.raises(StaleSurvey):
        ex.report(REFUSED)


def test_a_clearing_outcome_needs_no_survey_because_the_escape_is_over():
    ex = EscapeExecution(gates=gates())
    ex.begin(survey_of("wedge_20260815_postspin"), CAUSE_VISIBLE)
    result = ex.report(CLEARED)
    assert result.result == ESCAPED
    assert not ex.in_progress


def _two_candidates_at_one_clock(cause=CAUSE_FREEZE):
    """A pose where 12 o'clock carries TWO candidate kinds, which is what makes rules 2
    and 3 distinguishable at all.

    CONSTRUCTED, AND SAID SO. On all three archived specimens every clock carries
    exactly one candidate, so a freeze and a refusal produce identical sequences there
    and a test over them would pass whether or not rule 2 existed. The trail fields are
    set here because A1 has not landed and nothing fills them live yet -- the rule is
    real and the plan already emits the candidate, so the rule gets tested now rather
    than after the thing that will exercise it.
    """
    survey = survey_of("wedge_20260815_postspin", cause)
    survey.trail_available = True
    survey.trail_first_clock = 12
    survey.trail_length_m = 1.7
    return survey


def test_a_freeze_retires_the_WHOLE_DIRECTION():
    """RULE 2. The fresh survey CANNOT see what froze us -- every sensor will keep
    calling that heading open -- so the belief has to outlive the evidence. Same
    principle as the D39 hold one layer up.

    The discriminating assertion is that `forward_drive` at 12 o'clock, which the plan
    ranks second and which a refusal WOULD go on to try, never appears.
    """
    ex = EscapeExecution(gates=gates())
    first = ex.begin(_two_candidates_at_one_clock(), CAUSE_FREEZE)
    assert (first.candidate.kind, first.candidate.clock) == ("trail_retrace", 12)

    second = ex.report(FROZEN, survey=_two_candidates_at_one_clock())
    assert 12 in ex.frozen_clocks
    assert second.candidate.clock != 12, (
        "a frozen direction was proposed again after a fresh survey called it open")


def test_a_refusal_retires_only_that_shape_at_that_clock():
    """RULE 3, and it is the same pose as rule 2's test so the two are compared rather
    than merely stated. The arbiter refused this SHAPE here and said nothing about the
    rest of the vocabulary; retiring the whole direction would discard most of the
    escape for a reason the arbiter never gave."""
    ex = EscapeExecution(gates=gates())
    first = ex.begin(_two_candidates_at_one_clock(), CAUSE_FREEZE)
    assert (first.candidate.kind, first.candidate.clock) == ("trail_retrace", 12)

    second = ex.report(REFUSED, survey=_two_candidates_at_one_clock())
    assert not ex.frozen_clocks
    assert (second.candidate.kind, second.candidate.clock) == ("forward_drive", 12), (
        "a refusal retired the whole direction instead of just the shape")


def test_no_shape_is_retried_at_the_same_clock():
    ex, seen, _ = run("wedge_20260815_postspin", [REFUSED, REFUSED, REFUSED, REFUSED])
    shapes = [(a.candidate.kind, a.candidate.clock) for a in seen]
    assert len(shapes) == len(set(shapes)), "the same shape was tried twice at one clock"


def test_declined_aborts_and_is_never_retried():
    """RULE 4. Two nodes disagreeing about which is driving is a wiring bug; a quiet
    retry rebuilds the give-up livelock from the other side, and a human cannot fix it
    either -- so it aborts rather than escalating."""
    ex = EscapeExecution(gates=gates())
    ex.begin(survey_of("wedge_20260815_postspin"), CAUSE_VISIBLE)
    result = ex.report(DECLINED)
    assert result.result == ABORTED
    assert len(result.attempts) == 1
    assert not ex.in_progress


def test_no_candidate_is_ever_tried_twice():
    """The livelock guard, stated over the whole sequence rather than per rule. Run 1
    spent 21 s on four attempts that were the same attempt."""
    _, seen, final = run("wedge_20260814b", [REFUSED] * 8, cause=CAUSE_FREEZE)
    tried = [(a.candidate.kind, a.candidate.clock) for a in seen]
    assert len(tried) == len(set(tried))
    assert isinstance(final, Result)


def test_exhaustion_escalates_to_a_human_carrying_the_survey():
    """The plea's content is the survey -- one artifact, three consumers. The honest
    blocked ending sits BEHIND the human, never in front of them."""
    _, _, final = run("wedge_20260815_postspin", [REFUSED] * 12)
    assert final.result == ASK_HUMAN
    assert final.survey is not None
    assert final.survey.min_range_m is not None


def test_an_ungrantable_candidate_is_never_executed():
    """Plan-time grantability exists so a shape the arbiter must refuse is never
    PROPOSED. Executing one anyway would spend an attempt to learn what the plan
    already knew -- the un-grantable-by-construction family, form 3, which this
    project has now shipped twice."""
    for name in ("wedge_20260814b", "wedge_20260815_postspin", "wedge_mission2_2026-08-14"):
        _, seen, _ = run(name, [REFUSED] * 10)
        assert all(a.candidate.grantable for a in seen), name


# --- D34 composition ----------------------------------------------------------------

def test_in_progress_spans_exactly_the_escape():
    """D34's hook. A node asks THIS rather than inferring from whether a controller
    looks busy -- inferring another component's state by proxy is what produced three
    defects in two days."""
    ex = EscapeExecution(gates=gates())
    assert not ex.in_progress                       # nothing started
    ex.begin(survey_of("wedge_20260815_postspin"), CAUSE_VISIBLE)
    assert ex.in_progress                           # no goal may start here
    ex.report(REFUSED, survey=survey_of("wedge_20260815_postspin"))
    assert ex.in_progress                           # still mid-escape between attempts
    ex.report(CLEARED)
    assert not ex.in_progress


def test_an_execution_cannot_be_restarted():
    """One execution per escape. Reusing one would carry a previous pose's frozen
    directions into a new place and silently forbid headings that are fine there."""
    ex = EscapeExecution(gates=gates())
    ex.begin(survey_of("wedge_20260815_postspin"), CAUSE_VISIBLE)
    with pytest.raises(RuntimeError):
        ex.begin(survey_of("wedge_20260815_postspin"), CAUSE_VISIBLE)


def test_reporting_without_an_outstanding_attempt_is_an_error():
    ex = EscapeExecution(gates=gates())
    with pytest.raises(RuntimeError):
        ex.report(REFUSED, survey=survey_of("wedge_20260815_postspin"))


# --- the three specimens ------------------------------------------------------------

def test_specimen3_drives_forward_first_and_never_reverses_into_the_corner():
    """SPECIMEN 3, the purest. The rover had spun 180 degrees to face out with 1.77 m
    open dead ahead, and its reverse-only vocabulary backed it into the corner it had
    just escaped. Scott: "not using logic... just flailing."

    The execute stage must hand out the forward drive FIRST, at the CURRENT heading.
    """
    _, seen, _ = run("wedge_20260815_postspin", [REFUSED])
    first = seen[0].candidate
    assert first.kind == "forward_drive"
    assert first.clock == 12
    assert first.first_command[0] > 0.0, "the first move out of specimen 3 must go FORWARD"


def test_mission2_is_not_answered_by_the_widest_opening():
    """Mission 2 is the pose that breaks a width-only rule: 12 o'clock reads far less
    than 3 o'clock, and the archive still says 12 o'clock is the answer. Asserted here
    at the EXECUTE layer so the ordering cannot regress behind the plan's back."""
    _, seen, _ = run("wedge_mission2_2026-08-14", [REFUSED] * 3)
    assert seen, "mission 2 must produce at least one executable candidate"
    assert seen[0].candidate.clock == 12


def test_every_specimen_produces_an_executable_first_move():
    """The acceptance that matters: at all three archived poses the rover has something
    to try. Zero candidates at a real stuck pose is the failure this whole batch
    exists to end."""
    for name in ("wedge_20260814b", "wedge_20260815_postspin", "wedge_mission2_2026-08-14"):
        ex = EscapeExecution(gates=gates())
        step = ex.begin(survey_of(name), CAUSE_VISIBLE)
        assert isinstance(step, Attempt), f"{name} produced no executable candidate"
        assert step.candidate.grantable, name
