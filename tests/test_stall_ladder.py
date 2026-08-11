"""Revert-proofs for the stall survival ladder.

Every test here replays a signature that was RECORDED on the robot on 2026-08-10 and
that ended a mission. The bar for this file is the house rule: a test must fail
against the bug it hunts. `test_progress_guard_is_blind_to_a_blocked_pivot` is the
control -- it documents the defect in the CURRENT code and is expected to keep
passing after the ladder lands, because ProgressGuard's blindness is real and the
ladder is what compensates for it.
"""

import math

import pytest

from sphero_rvr_core.decisive_control import BackOffConfig, ProgressGuard
from sphero_rvr_core.stall_ladder import (
    DRIVE_OPEN, PIVOT_OPEN, REVERSE_ARC, REVERSE_STRAIGHT, RUNG_ORDER,
    LadderConfig, StallLadder,
)

HZ = 20.0


def _cycles(seconds, hz=HZ):
    return int(seconds * hz)


# ---------------------------------------------------------------- the control case

def test_progress_guard_is_blind_to_a_blocked_pivot():
    """THE DEFECT, stated as an executable fact.

    Run 185048 spent its last 14 s commanding (0.0, -0.9) against an output of
    (0.0, 0.0) with yaw pinned at -136.4 deg. `translating` is False for a pivot, so
    ProgressGuard early-returns "drive" and resets its own progress clock every cycle.
    Swept to 300 s to show this is unbounded blindness rather than a slow timer.
    """
    for seconds in (14, 30, 300):
        guard = ProgressGuard(BackOffConfig())
        actions = set()
        for i in range(_cycles(seconds)):
            result = guard.step(0.411, -0.310, i / HZ, False, output_moving=False)
            actions.add(result.action)
        assert actions == {"drive"}, (
            f"ProgressGuard escalated a blocked pivot after {seconds}s; if this now "
            "fails, the guard learned to see yaw and the ladder's rationale changed"
        )


# ------------------------------------------------- class A: the refused pivot

def test_ladder_escalates_run_185048_blocked_pivot():
    """Replay of run 185048's dead 14 s: commanding, never permitted, never moving."""
    ladder = StallLadder()
    fired = None
    for i in range(_cycles(14)):
        result = ladder.step(x=0.411, y=-0.310, yaw=math.radians(-136.4), now=i / HZ,
                             commanding=True, output_moving=False)
        if result.action == "rung":
            fired = result
            break
    assert fired is not None, "the ladder never triggered on run 185048's signature"
    assert fired.rung == REVERSE_STRAIGHT
    assert "output_suppressed" in fired.reason


def test_output_suppression_fires_before_the_explorer_drops_the_goal():
    """The explorer drops a goal after 6 s of no progress. A ladder that needs longer
    than that to notice can never run, which is exactly how the existing unstick
    became unreachable."""
    ladder = StallLadder()
    for i in range(_cycles(6)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=i / HZ,
                             commanding=True, output_moving=False)
        if result.action == "rung":
            assert i / HZ < 6.0
            return
    pytest.fail("ladder did not trigger within the explorer's 6 s goal-drop window")


# ------------------------------------------------ class B: the refused reverse

def test_run_190528_rear_hold_escalates_to_the_reverse_arc():
    """Run 190528 aborted three goals here. Rung 1 is refused (rear_hold zeroes a
    straight reverse); the ladder must reach rung 2, which the supervisor grants."""
    ladder = StallLadder()
    seen = []
    for i in range(_cycles(20)):
        result = ladder.step(x=-1.238, y=0.103, yaw=math.radians(-90.5), now=i / HZ,
                             commanding=True, output_moving=False)
        if result.action == "rung" and result.rung not in seen:
            seen.append(result.rung)
        if result.exhausted:
            break
    assert REVERSE_STRAIGHT in seen
    assert REVERSE_ARC in seen, f"never escalated past straight reverse: {seen}"


def test_ladder_reaches_the_forward_rung_that_was_available_all_along():
    """At run 190528's abort pose the supervisor was granting forward at 0.093 m/s
    with 0.58 m clear ahead. The mission died anyway. The ladder must get there."""
    ladder = StallLadder()
    seen = []
    for i in range(_cycles(30)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=i / HZ,
                             commanding=True, output_moving=False)
        if result.action == "rung" and result.rung not in seen:
            seen.append(result.rung)
        if result.exhausted:
            break
    assert seen == list(RUNG_ORDER), f"did not try every escape in order: {seen}"


# -------------------------------------------------------- the contract change

def test_failure_is_counted_once_per_exhausted_ladder_not_per_refused_action():
    """The whole objective in one assertion: a stall may end a goal only after every
    escape has been tried."""
    ladder = StallLadder()
    exhausted_at = None
    for i in range(_cycles(30)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=i / HZ,
                             commanding=True, output_moving=False)
        if result.exhausted:
            exhausted_at = i / HZ
            break
    assert exhausted_at is not None
    cfg = LadderConfig()
    # Four rungs, each given its full budget, and not one instant sooner.
    assert exhausted_at >= cfg.rung_budget_s * len(RUNG_ORDER) - 0.5


def test_a_working_rung_ends_the_ladder_without_a_failure():
    """A successful escape is not a failure and must not tick anything."""
    ladder = StallLadder()
    cfg = LadderConfig()
    x = 0.0
    result = None
    for i in range(_cycles(10)):
        now = i / HZ
        if ladder.active:            # the rung is driving us out
            x += cfg.reverse_speed_mps / HZ
        result = ladder.step(x=x, y=0.0, yaw=0.0, now=now,
                             commanding=True, output_moving=ladder.active)
        if result.reason.endswith("_cleared"):
            break
    assert result.reason == f"{REVERSE_STRAIGHT}_cleared"
    assert not result.exhausted


# ------------------------------------------------------------ anti-livelock

def test_repeated_escapes_on_one_goal_are_bounded():
    """Escape-then-restall is thrash WITH motion, and is bounded nowhere without the
    per-goal invocation budget."""
    cfg = LadderConfig(max_invocations_per_goal=2)
    ladder = StallLadder(cfg)
    x, now, exhausted = 0.0, 0.0, False
    for _ in range(_cycles(120)):
        if ladder.active:
            x += cfg.reverse_speed_mps / HZ      # every rung succeeds...
        result = ladder.step(x=x, y=0.0, yaw=0.0, now=now,
                             commanding=True, output_moving=ladder.active)
        if result.exhausted:
            exhausted = True
            break
        now += 1.0 / HZ
    assert exhausted, "a goal that escapes and re-stalls forever was never bounded"
    assert ladder.invocations > cfg.max_invocations_per_goal


def test_grind_yaw_does_not_count_as_progress():
    """A pivot against an invisible pin throws +/-80 deg in 0.3 s as the tracks slip.
    That is not motion, but it is excellent-looking yaw progress, and it would keep
    the stall clock re-arming forever."""
    ladder = StallLadder()
    fired = False
    for i in range(_cycles(10)):
        yaw = math.radians(80.0 if i % 2 else -80.0)   # physically impossible rate
        result = ladder.step(x=0.0, y=0.0, yaw=yaw, now=i / HZ,
                             commanding=True, output_moving=True)
        if result.action == "rung":
            fired = True
            break
    assert fired, "grind yaw masqueraded as progress and the ladder never triggered"


def test_genuine_slow_turning_is_not_a_stall():
    """The mirror image: a real pivot at the supervisor's cap must NOT be escalated,
    or the ladder would interrupt every legitimate turn."""
    ladder = StallLadder()
    for i in range(_cycles(10)):
        yaw = 0.4 * (i / HZ)                    # 0.4 rad/s, exactly what is permitted
        result = ladder.step(x=0.0, y=0.0, yaw=yaw, now=i / HZ,
                             commanding=True, output_moving=True)
        assert result.action == "drive", f"interrupted a legitimate turn at t={i/HZ}"


def test_not_commanding_never_triggers_the_ladder():
    """A robot standing still because nothing asked it to move is not stalled."""
    ladder = StallLadder()
    for i in range(_cycles(30)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=i / HZ,
                             commanding=False, output_moving=False)
        assert result.action == "drive"


def test_pivot_rung_is_judged_by_yaw_not_distance():
    """A pivot moves no distance by design. Judging it by position would mark a
    perfectly good escape as ineffective and escalate past it."""
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    now, yaw, reached_pivot = 0.0, 0.0, False
    for _ in range(_cycles(60)):
        result = ladder.step(x=0.0, y=0.0, yaw=yaw, now=now,
                             commanding=True, output_moving=True,
                             open_bearing_rad=1.0)
        if result.rung == PIVOT_OPEN:
            reached_pivot = True
            yaw += cfg.pivot_rate_rad_s / HZ     # turning nicely, going nowhere
        if reached_pivot and result.reason.endswith("_cleared"):
            assert result.reason == f"{PIVOT_OPEN}_cleared"
            return
        now += 1.0 / HZ
    pytest.fail("a turning-but-stationary pivot rung was never credited as working")


def test_pivot_turns_toward_the_open_bearing():
    """Escaping toward the wall is not an escape."""
    cfg = LadderConfig()
    for bearing, expect_positive in ((1.2, True), (-1.2, False)):
        ladder = StallLadder(cfg)
        now, seen = 0.0, None
        for _ in range(_cycles(60)):
            result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now,
                                 commanding=True, output_moving=False,
                                 open_bearing_rad=bearing)
            if result.rung == PIVOT_OPEN and result.action == "rung":
                seen = result.angular_z
                break
            now += 1.0 / HZ
        assert seen is not None
        assert (seen > 0) is expect_positive, (
            f"pivot rung turned the wrong way for open bearing {bearing}")


def test_drive_open_is_the_last_resort_not_the_first():
    """Driving forward out of a stall is the most dangerous rung and must come after
    the retreats, not before them."""
    assert RUNG_ORDER.index(DRIVE_OPEN) == len(RUNG_ORDER) - 1
    assert RUNG_ORDER.index(REVERSE_STRAIGHT) == 0


# ------------------------------------------------------------- freeze classification

def test_refused_motion_is_not_a_freeze():
    """Run 185048: the supervisor refused every pivot for 14 s. The stall is fully
    explained by the refusal, so there is nothing invisible to mark. The empty
    `freeze_marks` in that mission report was the CORRECT answer."""
    ladder = StallLadder()
    for i in range(_cycles(14)):
        result = ladder.step(x=0.411, y=-0.310, yaw=math.radians(-136.4), now=i / HZ,
                             commanding=True, output_moving=False)
        if result.action == "rung":
            assert not result.freeze
            return
    pytest.fail("ladder never triggered")


def test_permitted_but_immobile_is_a_freeze_and_still_escalates():
    """The supervisor said yes, the robot did not move: something is physically there
    that no sensor can see. Mark it — and still run the ladder, because discovering an
    invisible obstacle does not excuse us from escaping it."""
    ladder = StallLadder()
    for i in range(_cycles(14)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=i / HZ,
                             commanding=True, output_moving=True)
        if result.action == "rung":
            assert result.freeze, "an invisible obstacle was not reported as a freeze"
            assert result.rung == REVERSE_STRAIGHT, "freeze suppressed the escape"
            return
    pytest.fail("ladder never triggered")


# ---------------------------------------------- honest exhaustion reporting (D30)

def test_every_rung_refused_reports_genuinely_wedged():
    """The bench probe found a real pose (returns at the swept-circle corners at
    0.20 m) where the supervisor refuses reverse, arc AND pivot. That is correct
    behaviour -- the rover IS surrounded -- but the report must say so, rather than
    imply the ladder manoeuvred and it did not help."""
    ladder = StallLadder()
    result = None
    for i in range(_cycles(40)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=i / HZ,
                             commanding=True, output_moving=False)
        if result.exhausted:
            break
    assert result.exhausted
    assert result.genuinely_wedged, "surrounded pose was not reported as wedged"
    assert result.reason == "genuinely_wedged"


def test_granted_but_ineffective_is_not_reported_as_wedged():
    """The mirror image, and the one that matters for blame: if the supervisor let us
    move and the manoeuvres simply did not free us, that is NOT the room having us
    surrounded, and calling it wedged would hide a real ladder or odometry problem."""
    ladder = StallLadder()
    result = None
    for i in range(_cycles(40)):
        # Permitted throughout, but the robot never actually goes anywhere.
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=i / HZ,
                             commanding=True, output_moving=True)
        if result.exhausted:
            break
    assert result.exhausted
    assert not result.genuinely_wedged
    assert result.reason == "all_rungs_ineffective"
