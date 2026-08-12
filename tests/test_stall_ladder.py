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
    VISIBLE_RUNG_ORDER, LadderConfig, StallLadder,
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
    """A successful escape is not a failure and must not tick anything.

    The escape here goes SIDEWAYS -- the supervisor granting the reverse an arc, or
    the rover slipping off the thing it was against. Straight-back-and-forward is no
    longer an escape and has its own test.
    """
    ladder = StallLadder()
    cfg = LadderConfig()
    y = 0.0
    result = None
    for i in range(_cycles(10)):
        now = i / HZ
        if ladder.active:            # the rung is taking us off the approach line
            y += cfg.reverse_speed_mps / HZ
        result = ladder.step(x=0.0, y=y, yaw=0.0, now=now,
                             commanding=True, output_moving=ladder.active)
        if result.reason.endswith("_cleared"):
            break
    assert result.reason == f"{REVERSE_STRAIGHT}_cleared"
    assert not result.exhausted


# ------------------------------------------------------------ anti-livelock

def test_repeated_escapes_on_one_goal_are_bounded():
    """Escape-then-restall-in-the-same-place is thrash WITH motion, and is bounded
    nowhere without the per-goal escape budget.

    Every rung "succeeds" here -- the rover slides 0.14 m off its approach line, the
    ladder credits it, and then the rover ends up right back where it stalled. That is
    the field signature (run 211237: 14 escapes, 14 re-stalls, median 0.033 m from
    where the escape started), so the budget must run out rather than the ladder
    handing back forever.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    y, now, exhausted = 0.0, 0.0, False
    for _ in range(_cycles(300)):
        if ladder.active:
            y += cfg.reverse_speed_mps / HZ      # every rung succeeds...
        else:
            y = 0.0                              # ...and we end up back at the stall
        result = ladder.step(x=0.0, y=y, yaw=0.0, now=now,
                             commanding=True, output_moving=ladder.active)
        if result.exhausted:
            exhausted = True
            break
        now += 1.0 / HZ
    assert exhausted, "a goal that escapes and re-stalls forever was never bounded"
    assert result.budget_exhausted, (
        "the bound fired, but as the wrong outcome -- every rung had been tried")
    assert set(ladder.tried_rungs) == set(RUNG_ORDER), (
        f"the budget ran out before every escape had been tried: {ladder.tried_rungs}")


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


def test_pivot_rung_is_judged_by_where_the_gap_is_not_by_distance():
    """A pivot moves no distance by design, so position cannot judge it -- and wheel
    yaw must not, because the driver self-regulates a pivot at ~2.9 rad/s, five times
    the rate the grind filter calls physically possible (D32). What judges it is the
    ROOM: the gap comes round to the nose as the body turns.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    now, bearing, reached_pivot = 0.0, 1.2, False
    for _ in range(_cycles(60)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now,
                             commanding=True, output_moving=True,
                             open_bearing_rad=bearing)
        if result.rung == PIVOT_OPEN:
            reached_pivot = True
            # The body turns toward the gap at the driver's real rate; the wheel-yaw
            # this test never advances would be rejected by the rate filter anyway.
            bearing = max(0.0, bearing - 2.9 / HZ)
        if reached_pivot and result.reason.endswith("_cleared"):
            assert result.reason == f"{PIVOT_OPEN}_cleared"
            assert bearing <= cfg.pivot_target_tolerance_rad, (
                "the pivot was credited before the gap was in front of the robot")
            return
        now += 1.0 / HZ
    pytest.fail("a pivot that brought the gap to the nose was never credited")


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


def test_a_long_successful_drive_does_not_make_the_next_stop_a_freeze():
    """F2. The freeze vote covers the CURRENT no-progress window, not the whole goal.

    60 s of granted driving followed by a legitimate refusal must classify as an
    ordinary brake. Without the tally reset the drive banks ~1200 'permitted' votes
    and outvotes the 20 refused cycles, planting a permanent phantom mark and
    blaming the room for a wall the lidar sees perfectly well.
    """
    ladder = StallLadder()
    x = 0.0
    for i in range(_cycles(60)):                 # driving, permitted, progressing
        x += 0.2 / HZ
        ladder.step(x=x, y=0.0, yaw=0.0, now=i / HZ,
                    commanding=True, output_moving=True)
    base = _cycles(60)
    for j in range(_cycles(10)):                 # now the supervisor brakes us
        result = ladder.step(x=x, y=0.0, yaw=0.0, now=(base + j) / HZ,
                             commanding=True, output_moving=False)
        if result.action == "rung":
            assert not result.freeze, (
                "a normal brake after a long drive was classified as a FREEZE — "
                "the vote window is not being reset on progress")
            return
    pytest.fail("ladder never triggered after the brake")


# --------------------------------------------------------------------------- F4

# The real chair-pin trace, run 20260810_142641, t=47.6..51.2 at 10 Hz: odom_yaw_deg
# recorded while the rover was pinned against a chair leg. It bursts 102 deg in 0.6 s
# (peak 5.04 rad/s) as the tracks slip and the estimator catches up, then settles and
# sits. Position moved 15 mm across the whole window -- the rover went nowhere.
# Inlined rather than shipped as a CSV: one trace, one consumer, no parser.
CHAIR_PIN_YAW_DEG = [
    105.3, 105.3, 105.3, 105.3, 105.3, 112.1, 120.3, 137.2, 154.5, -176.6,
    -154.1, -152.8, -152.6, -152.6, -152.6, -152.6, -152.6, -152.6, -152.6,
    -152.6, -152.6, -152.6, -152.6, -152.6, -152.6, -152.6, -152.6, -152.6,
    -152.6, -152.6, -152.6, -152.6, -152.6, -152.6, -152.6, -152.6,
]
CHAIR_PIN_HZ = 10.0


def test_f4_real_chair_pin_burst_does_not_mask_the_stall():
    """A slip burst must not buy the rover more time.

    Zeroing `turned` for the burst CYCLES alone was not enough: on the next settled
    cycle the burst is still inside (yaw - ref_yaw), so it reads as 1.78 rad of
    progress, re-arms the stall clock, and the ladder never fires. The filter was
    defeated by the exact signature that motivated it, which is why this replays the
    recorded trace rather than a synthetic square wave.
    """
    def fire_time(yaws):
        ladder = StallLadder()
        for i, deg in enumerate(yaws):
            result = ladder.step(x=0.0, y=0.0, yaw=math.radians(deg),
                                 now=i / CHAIR_PIN_HZ,
                                 commanding=True, output_moving=True)
            if result.action == "rung":
                return i / CHAIR_PIN_HZ
        return None

    tail = [CHAIR_PIN_YAW_DEG[-1]] * int(10 * CHAIR_PIN_HZ)
    with_burst = fire_time(CHAIR_PIN_YAW_DEG + tail)
    # The same pinned rover, without the estimator's slip burst: yaw simply holds.
    without_burst = fire_time([CHAIR_PIN_YAW_DEG[0]] * (len(CHAIR_PIN_YAW_DEG)
                                                       + len(tail)))
    assert with_burst is not None, "the ladder never fired while the rover sat pinned"
    assert without_burst is not None

    # ASSERT THE TIMING, not merely that it eventually fires. Without the reference
    # poison the ladder still fires -- just later, because the burst re-arms the stall
    # clock once and buys the pin an extra stall_time_s. "Eventually" is exactly the
    # assertion a grind can satisfy while the rover sits against a chair leg, so the
    # burst must cost NOTHING.
    assert with_burst <= without_burst + 1.0 / CHAIR_PIN_HZ, (
        f"the recorded burst delayed the stall detection ({with_burst:.1f}s vs "
        f"{without_burst:.1f}s without it) — it bought the pin extra time")


def test_f4_a_slip_burst_does_not_credit_the_pivot_rung():
    """_run_rung applied no rate filter at all, so one slip burst mid-grind marked
    the pivot rung 'cleared' and handed control back with the rover still pinned.

    The bearing is held STATIC throughout, which is the physical claim: tracks
    slipping against a pin do not turn the body, so the room does not move.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    now, yaw, in_pivot = 0.0, 0.0, False
    for i in range(_cycles(60)):
        result = ladder.step(x=0.0, y=0.0, yaw=yaw, now=now,
                             commanding=True, output_moving=True,
                             open_bearing_rad=1.2)
        if result.rung == PIVOT_OPEN and result.action == "rung":
            in_pivot = True
            # A single impossible burst, then pinned again — the grind signature.
            yaw += math.radians(60.0) if i % 20 == 0 else 0.0
        assert not (in_pivot and result.reason == f"{PIVOT_OPEN}_cleared"), (
            "a slip burst credited the pivot rung as a successful escape")
        now += 1.0 / HZ


# ------------------------------------------------------------------------ F6/F7

def test_f6_reverse_arc_is_credited_when_the_supervisor_grants_only_rotation():
    """Under `rear_hold` the supervisor refuses the linear half of a reverse arc and
    passes the ANGULAR through, so what reaches the motors is a pure rotation. Judged
    by position alone that rung can never be credited in the exact geometry it was
    added for -- it burns its full budget and escalates past a working escape."""
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    now, yaw, x, seen_arc = 0.0, 0.0, 0.0, False
    for _ in range(_cycles(60)):
        result = ladder.step(x=x, y=0.0, yaw=yaw, now=now,
                             commanding=True, output_moving=ladder.active)
        if result.rung == REVERSE_ARC and result.action == "rung":
            seen_arc = True
            yaw += cfg.pivot_rate_rad_s / HZ      # rotation granted, no translation
        if seen_arc and result.reason == f"{REVERSE_ARC}_cleared":
            return
        now += 1.0 / HZ
    pytest.fail("a rotation-only reverse arc was never credited as a working escape")


def test_f7_drive_open_steers_toward_the_open_bearing():
    """Rung 4 ignored open_bearing and drove straight at the heading we stalled on.
    In a freeze that is powered contact with the obstacle that stopped us."""
    cfg = LadderConfig()
    for bearing, expect_positive in ((1.2, True), (-1.2, False)):
        ladder = StallLadder(cfg)
        now, seen = 0.0, None
        for _ in range(_cycles(60)):
            result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now,
                                 commanding=True, output_moving=False,
                                 open_bearing_rad=bearing)
            if result.rung == DRIVE_OPEN and result.action == "rung":
                seen = result.angular_z
                break
            now += 1.0 / HZ
        assert seen is not None, "never reached the drive rung"
        assert abs(seen) > 1e-9, "drive rung drove straight ahead, ignoring open space"
        assert (seen > 0) is expect_positive, "drive rung steered the wrong way"


# -------------------------------------------------------------------------- F10

def test_f10_abandon_rung_drops_the_escape_but_keeps_the_goal_budget():
    """Pins the intended split. The ladder SURVIVES a same-destination replan (else
    bt_navigator's ~1 Hz replan clears the anti-livelock budget forever), but a
    half-run rung must not be inherited by the next execute loop with a stale
    reference pose and a stale clock."""
    ladder = StallLadder()
    for i in range(_cycles(5)):
        if ladder.step(x=0.0, y=0.0, yaw=0.0, now=i / HZ,
                       commanding=True, output_moving=False).action == "rung":
            break
    assert ladder.active, "setup failed: no rung was running"
    spent = ladder.invocations
    assert spent >= 1

    ladder.abandon_rung()
    assert not ladder.active, "a rung survived the end of its goal"
    assert ladder.invocations == spent, (
        "abandoning a rung reset the per-goal invocation budget — a replanning "
        "goal could then livelock forever")


# --------------------------------------------------------------------------- N3

def test_n3_rocking_does_not_fake_clear_the_pivot_rung():
    """Zero-net rocking is not an escape.

    A rover pinned against something compliant oscillates within the rate-sane band.
    Accumulating |step| banked those wobbles and "cleared" the pivot rung having
    turned nowhere -- burning an invocation and publishing pivot_open_cleared, which
    is telemetry that lies about the rover's state.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    now, yaw, in_pivot = 0.0, 0.0, False
    # RATE-SANE by construction: at 20 Hz this is 0.52 rad/s against a 0.6 bound, so
    # F4's grind filter passes it and N3's signed accumulation is the only thing that
    # can reject it. My first attempt used 2.0 deg/cycle = 0.70 rad/s, which the rate
    # filter rejected outright -- so the test passed with N3 mutated out and proved
    # nothing about signing at all.
    swing = math.radians(1.5)
    assert swing * HZ < LadderConfig().max_yaw_rate_rad_s, "oscillation must be rate-sane"
    for i in range(_cycles(60)):
        # Rocking does not turn the body, so the gap stays exactly where it was.
        result = ladder.step(x=0.0, y=0.0, yaw=yaw, now=now,
                             commanding=True, output_moving=True,
                             open_bearing_rad=1.2)
        if result.rung == PIVOT_OPEN and result.action == "rung":
            in_pivot = True
            yaw += swing if (i // 2) % 2 == 0 else -swing   # rocking, net zero
        assert not (in_pivot and result.reason == f"{PIVOT_OPEN}_cleared"), (
            "rocking in place was credited as a completed pivot escape")
        now += 1.0 / HZ


def test_n3_genuine_rotation_still_clears_the_pivot_rung():
    """Paired negative: rejecting rocking must not stop crediting a real turn.

    A real turn moves the room, so the bearing to the gap closes as the body swings.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    now, bearing, in_pivot = 0.0, -1.2, False
    for _ in range(_cycles(60)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now,
                             commanding=True, output_moving=True,
                             open_bearing_rad=bearing)
        if result.rung == PIVOT_OPEN and result.action == "rung":
            in_pivot = True
            bearing = min(0.0, bearing + cfg.pivot_rate_rad_s / HZ)
        if in_pivot and result.reason == f"{PIVOT_OPEN}_cleared":
            return
        now += 1.0 / HZ
    pytest.fail("a genuine sustained rotation was never credited")


# ------------------------------------------------- budget-spent is its own outcome

def test_budget_spent_reports_as_itself_not_as_ineffective():
    """The third state, from gauntlet run 20260811_103337.

    Five consecutive goals each exhausted instantly with the budget already spent.
    Nothing was tried, so nothing was permitted or refused -- yet the result carried
    genuinely_wedged=False, which made the controller log "we WERE permitted to move
    and it did not help": blaming the ROBOT for trying when it never tried. The
    recorder for that window shows zero commands, zero output and one pose.

    Under the traversal budget this state is RARER but not gone: it is what a stall
    gets once every escape on this goal has already had its honest attempt.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    now, result = 0.0, None
    for _ in range(_cycles(40)):                 # one full traversal, nothing works
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now,
                             commanding=True, output_moving=True)
        now += 1.0 / HZ
        if result.exhausted:
            break
    assert result.exhausted and result.reason == "all_rungs_ineffective", (
        "a traversal in which every rung was tried must report what it found")

    # The NEXT stall on the same goal is the budget-spent case, and it must emit
    # nothing at all -- the field's zero-command signature.
    cmds = []
    for _ in range(_cycles(10)):
        r = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now,
                        commanding=True, output_moving=False)
        now += 1.0 / HZ
        if r.action == "rung":
            cmds.append(r)
        if r.exhausted:
            assert r.reason == "ladder_budget_exhausted"
            assert r.budget_exhausted, "budget-spent did not report as itself"
            assert not r.genuinely_wedged
            assert not cmds, "a rung was emitted despite the budget being spent"
            return
    pytest.fail("never exhausted")


# ============================================================================
# PART TWO -- THE DANCE. Revert-proofs 1-4 of docs/turning_batch_design.md §11.
#
# Two missions, 39 ladder invocations, every one of them starting at straight
# reverse, and rung 3 -- the pivot toward open floor -- never once ran on hardware.
# At the second mission's death pose the rover's own lidar reported 2.07 m of open
# floor 71 deg to its right while it gave up five times in a row.
# ============================================================================

# THE 14 EPISODES of run 20260811_211237, measured from its recorder.
#
# Rung windows were taken from the RECORDER, not the log: rung 1 commands exactly
# (-0.10, 0.00), a signature nothing else in this stack emits, so a contiguous run of
# those rows IS the rung. (travel, lateral, net heading) per episode, where lateral is
# perpendicular to the heading the rover stalled on. The whole population is inlined
# rather than summarised, because the claim being pinned is about ALL of them.
DANCE_EPISODES = [
    # travel_m, lateral_m, dyaw_deg
    (0.105, 0.004, +4.1), (0.106, 0.004, -4.4), (0.123, 0.000, -0.2),
    (0.119, 0.001, -1.1), (0.108, 0.003, -4.9), (0.125, 0.004, +3.2),
    (0.112, 0.001, +0.8), (0.110, 0.005, -4.7), (0.112, 0.004, +2.4),
    (0.130, 0.006, -6.5), (0.119, 0.000, -0.2), (0.111, 0.000, +0.0),
    (0.120, 0.001, -0.4), (0.115, 0.005, +3.8),
]


def _stall_until_rung(ladder, *, x=0.0, y=0.0, yaw=0.0, t0=0.0,
                      output_moving=False, bearing=None, hz=HZ):
    """Drive the no-progress predicate until a rung fires. Returns (result, now)."""
    for i in range(_cycles(20, hz)):
        now = t0 + i / hz
        result = ladder.step(x=x, y=y, yaw=yaw, now=now, commanding=True,
                             output_moving=output_moving, open_bearing_rad=bearing)
        if result.action in ("rung", "exhausted"):
            return result, now
    pytest.fail("the ladder never triggered")


def test_visible_stall_pivots_first():
    """REVERT-PROOF 1. A stall the sensors can explain, with a gap to turn toward,
    must PIVOT first -- not reverse.

    The death pose, exactly: the supervisor refusing every command (so the freeze vote
    says "a gate can see it") and the controller's own gap search reporting -71.4 deg.
    Against HEAD this returns reverse_straight, as it did 39 times in the field.
    """
    ladder = StallLadder()
    result, _ = _stall_until_rung(ladder, output_moving=False,
                                  bearing=math.radians(-71.4))
    assert result.rung == PIVOT_OPEN, (
        f"a lidar-visible stall with 2 m of open floor at -71 deg opened with "
        f"{result.rung} — this is the dance")
    assert result.angular_z < 0.0, "pivoted away from the gap"
    assert result.linear_x == 0.0, "a pivot must not acquire translation"


def test_blind_contact_still_reverses_first():
    """REVERT-PROOF 2. The pairing test that stops proof 1 from breaking D25.

    A freeze -- the supervisor permitted motion and the rover did not move -- means
    NOTHING can see what stopped us. The path we came in on is then the only route
    known to be clear, and reverse-first is preserved exactly, gap or no gap.
    """
    ladder = StallLadder()
    result, _ = _stall_until_rung(ladder, output_moving=True,
                                  bearing=math.radians(-71.4))
    assert result.freeze, "setup failed: this stall was not classified as a freeze"
    assert result.rung == REVERSE_STRAIGHT, (
        f"a blind contact opened with {result.rung} instead of backing out along the "
        "entry path — D25's rationale is gone")


def test_an_unknown_bearing_reverses_rather_than_guessing():
    """The seam, asserted from the consumer's side: 'no bearing' is not 'dead ahead'.

    The controller answers None when the scan is stale or unplaceable. Pivoting toward
    a bearing nobody measured is exactly the class of mistake this project keeps
    paying for, so an unknown bearing takes the order that needs no bearing.
    """
    ladder = StallLadder()
    result, _ = _stall_until_rung(ladder, output_moving=False, bearing=None)
    assert result.rung == REVERSE_STRAIGHT


def test_a_gap_already_dead_ahead_is_not_somewhere_to_pivot_to():
    """The other half of the same guard: a bearing inside the pivot tolerance is not a
    turn worth making, and pivoting 'toward' it would be a no-op that burns a rung."""
    ladder = StallLadder()
    cfg = LadderConfig()
    result, _ = _stall_until_rung(
        ladder, output_moving=False,
        bearing=cfg.pivot_target_tolerance_rad * 0.5)
    assert result.rung == REVERSE_STRAIGHT


def test_a_reverse_that_changes_nothing_does_not_clear():
    """REVERT-PROOF 3. Scott: "backing up and rolling forward shouldn't clear the
    ladder."

    Replays all 14 rung-1 episodes of run 20260811_211237. Every one of them backed
    straight down its own approach line -- median 100.0% of the travel axial, max
    lateral 0.006 m, max net heading 6.5 deg -- and every one was credited, handing the
    rover back to the same approach: 14 re-stalls within 12 s, a median of 0.033 m from
    where the escape began.

    Against HEAD (`escape_distance_m` 0.12, any direction) episodes 3, 6, 10 and 13
    clear on this recorder data alone, so this test fails there; in the field, where
    the ladder read TF rather than the recorder's odom, the log shows it credited all
    fourteen (14 invocations, zero `_failed->` escalations).
    """
    cfg = LadderConfig()
    for n, (travel, lateral, dyaw_deg) in enumerate(DANCE_EPISODES, 1):
        ladder = StallLadder(cfg)
        result, now = _stall_until_rung(ladder, output_moving=False)
        assert result.rung == REVERSE_STRAIGHT
        # The rung runs: the rover backs down its own approach line. Heading here is
        # 0, so axial is x and lateral is y.
        cycles = _cycles(cfg.rung_budget_s * 0.9)
        for i in range(1, cycles + 1):
            frac = i / cycles
            r = ladder.step(x=-travel * frac, y=lateral * frac,
                            yaw=math.radians(dyaw_deg) * frac,
                            now=now + i / HZ, commanding=True, output_moving=True,
                            open_bearing_rad=None)
            assert not r.reason.endswith("_cleared"), (
                f"episode {n}: {travel:.3f} m of straight reverse ({lateral:.3f} m "
                f"lateral, {dyaw_deg:+.1f} deg) was credited as an escape")


def test_a_long_straight_reverse_still_changes_nothing():
    """The part of proof 3 the RECORDED episodes cannot prove, stated separately
    rather than smuggled in.

    Every one of the 14 field episodes travelled 0.105-0.130 m, all of it axial -- so
    they are rejected by the 0.14 m threshold alone, and they would be rejected even
    by a criterion that counted displacement in ANY direction. A mutation run proved
    exactly that: replacing the lateral test with a total-displacement test left the
    replay green. The claim that AXIAL TRAVEL COUNTS FOR NOTHING therefore needs a
    case the recording does not contain -- and the new code produces one, because a
    rung 1 that is no longer credited early now runs its full budget: 3.0 s at
    0.10 m/s is 0.30 m of straight reverse, more than twice the threshold, and it
    still leaves the rover facing the same obstacle on the same line.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    result, now = _stall_until_rung(ladder, output_moving=False)
    assert result.rung == REVERSE_STRAIGHT
    x = 0.0
    for i in range(1, _cycles(cfg.rung_budget_s) + 1):
        x -= cfg.reverse_speed_mps / HZ          # straight back, heading 0
        r = ladder.step(x=x, y=0.0, yaw=0.0, now=now + i / HZ, commanding=True,
                        output_moving=True, open_bearing_rad=None)
        assert not r.reason.endswith("_cleared"), (
            f"{abs(x):.2f} m of straight reverse was credited as an escape")
    assert abs(x) >= 2 * cfg.escape_lateral_m, (
        f"the reverse under test only reached {abs(x):.2f} m — not long enough to "
        "distinguish a lateral test from a distance test")


def test_a_bearing_that_goes_unknown_cannot_credit_a_pivot():
    """The seam again, where it actually bites: the pivot is credited by comparing
    two lidar bearings, so a bearing that stops existing must stop the comparison.

    Coercing an absent bearing to 0.0 -- the controller's old behaviour -- reads as
    "the gap is dead ahead now", which is indistinguishable from a completed pivot
    and would credit an escape the robot never made.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    now, in_pivot = 0.0, False
    for _ in range(_cycles(60)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now, commanding=True,
                             output_moving=True,
                             open_bearing_rad=(None if in_pivot else 1.2))
        if result.rung == PIVOT_OPEN and result.action == "rung":
            in_pivot = True          # the scan goes stale the moment we start turning
        assert not (in_pivot and result.reason == f"{PIVOT_OPEN}_cleared"), (
            "a pivot was credited off a bearing the caller said it did not have")
        now += 1.0 / HZ


def test_second_stall_resumes_at_the_next_rung():
    """REVERT-PROOF 4. Two stalls on one goal must reach rung 2.

    `_begin` used to set the rung index to 0 unconditionally, so the second stall on a
    goal ran rung 1 AGAIN. With a rung 1 that false-succeeds, that made rungs 2-4
    unreachable by construction -- which is why 39 field invocations produced zero
    escalations past the first rung.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)

    # Stall one: rung 1 fires and "works" -- the rover slides off its approach line.
    first, now = _stall_until_rung(ladder, output_moving=False)
    assert first.rung == REVERSE_STRAIGHT
    y = 0.0
    for i in range(1, _cycles(5)):
        y = min(cfg.escape_lateral_m, y + cfg.reverse_speed_mps / HZ)
        r = ladder.step(x=0.0, y=y, yaw=0.0, now=now + i / HZ,
                        commanding=True, output_moving=True, open_bearing_rad=None)
        if r.reason.endswith("_cleared"):
            now = now + i / HZ
            break
    else:
        pytest.fail("setup failed: rung 1 never cleared")

    # ...and the rover ends up back where it stalled, which is what the field shows.
    second, _ = _stall_until_rung(ladder, t0=now + 0.1, output_moving=False)
    assert second.rung == REVERSE_ARC, (
        f"the second stall on this goal ran {second.rung} again — the ladder has no "
        "memory and rungs 2-4 stay unreachable")
    assert ladder.tried_rungs == (REVERSE_STRAIGHT, REVERSE_ARC)


def test_real_progress_between_stalls_earns_the_whole_ladder_again():
    """The paired negative for proof 4: memory is per stall REGION, not per goal.

    A stall 0.6 m from the last one is a different problem, and starting it half way
    up the ladder would skip the retreats for no reason. The threshold is the robot's
    own radius, and the recorded populations sit either side of it with room to spare:
    across both gauntlet runs, consecutive invocations at the SAME stall are at most
    0.107 m apart and every genuinely different place is at least 0.278 m away.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    first, now = _stall_until_rung(ladder, output_moving=False)
    assert first.rung == REVERSE_STRAIGHT
    ladder.abandon_rung()
    second, _ = _stall_until_rung(ladder, x=0.6, t0=now + 1.0, output_moving=False)
    assert second.rung == REVERSE_STRAIGHT, (
        "a stall 0.6 m away inherited the last stall's escalation")


def test_the_budget_bounds_complete_traversals_not_repeats_of_rung_one():
    """§9's budget ruling, as an executable fact.

    Under the old per-invocation budget, two invocations of a false-succeeding rung 1
    spent a goal's entire escape allowance on two identical reverses: 39 field
    invocations, every exhaustion `budget_exhausted`, zero `all_rungs_ineffective`.
    A goal must now get every rung once before it can be refused an escape.
    """
    cfg = LadderConfig()
    assert cfg.max_ladder_traversals_per_goal == 1
    ladder = StallLadder(cfg)
    now, result, rungs = 0.0, None, []
    for _ in range(_cycles(60)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now,
                             commanding=True, output_moving=True)
        if result.action == "rung" and result.rung not in rungs:
            rungs.append(result.rung)
        if result.exhausted:
            break
        now += 1.0 / HZ
    assert rungs == list(RUNG_ORDER), f"a goal was refused escapes it never had: {rungs}"
    assert result.reason == "all_rungs_ineffective"
    assert not result.budget_exhausted


def _one_full_traversal(ladder, cfg, x, t0):
    """Stall at (x, 0), run every rung, let none of them work. Returns the result on
    the cycle the ladder gives up and the time it happened."""
    for i in range(_cycles(60)):
        now = t0 + i / HZ
        r = ladder.step(x=x, y=0.0, yaw=0.0, now=now, commanding=True,
                        output_moving=True, open_bearing_rad=None)
        if r.exhausted:
            return r, now
    pytest.fail("a full traversal never exhausted")


def test_a_goal_that_keeps_stalling_somewhere_new_is_still_bounded():
    """THE OUTER BOUND, and the reason it has to exist.

    `max_ladder_traversals_per_goal` is a budget per STALL REGION: the escalation
    memory resets whenever the rover genuinely gets somewhere, and the budget resets
    with it. That is the right rule -- a stall 0.6 m from the last one is a different
    problem -- but on its own it bounds nothing at the goal level, because a rover
    that escapes, drives 0.2 m, stalls again and escapes again can repeat that
    forever without ever thrashing in one place.

    So complete traversals are ALSO counted monotonically across every reset. Four
    full ladders in four different places on one goal is a verdict on the goal, and
    the fifth stall is refused honestly rather than escaped a fifth time.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    t, x = 0.0, 0.0
    for n in range(cfg.max_total_traversals_per_goal):
        r, t = _one_full_traversal(ladder, cfg, x, t + 1.0)
        assert r.reason == "all_rungs_ineffective", (
            f"traversal {n + 1} ended as {r.reason} — this test is not measuring "
            "what it claims")
        x += 0.6                      # escaped, drove on, stalled somewhere new

    r, _ = _one_full_traversal(ladder, cfg, x, t + 1.0)
    assert r.reason == "goal_traversal_ceiling", (
        f"a goal that has had {cfg.max_total_traversals_per_goal} complete ladders "
        f"in different places got another one ({r.reason}) — 'bounded per goal' is "
        "not true")
    assert r.exhausted and r.budget_exhausted
    assert not r.genuinely_wedged


def test_the_outer_bound_does_not_touch_a_goal_that_recovers():
    """The paired negative. The ceiling must be unreachable in ordinary operation:
    three full ladders in three places, then a normal stall, must still get escapes.
    """
    cfg = LadderConfig()
    assert cfg.max_total_traversals_per_goal == 4
    ladder = StallLadder(cfg)
    t, x = 0.0, 0.0
    for _ in range(cfg.max_total_traversals_per_goal - 1):
        _, t = _one_full_traversal(ladder, cfg, x, t + 1.0)
        x += 0.6
    result, _ = _stall_until_rung(ladder, x=x, t0=t + 1.0, output_moving=False)
    assert result.action == "rung" and result.rung == REVERSE_STRAIGHT, (
        f"the fourth stall on this goal was refused an escape ({result.reason}) — "
        "the outer bound is set so tight it fires during normal recovery")


def test_a_new_goal_clears_the_outer_bound():
    """It is a bound on the GOAL, so the goal boundary is what clears it -- the same
    `goal_generation` signal that already owns every other per-goal reset."""
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    t, x = 0.0, 0.0
    for _ in range(cfg.max_total_traversals_per_goal):
        _, t = _one_full_traversal(ladder, cfg, x, t + 1.0)
        x += 0.6
    ladder.reset_goal()
    result, _ = _stall_until_rung(ladder, x=x, t0=t + 1.0, output_moving=False)
    assert result.action == "rung", (
        "a brand new goal inherited the last one's traversal ceiling")


def test_a_visible_stall_walks_its_own_order_not_the_blind_one():
    """Ordering is a whole SEQUENCE, not just a first move: after the pivot comes the
    drive out along the bearing, and the retreats stay available behind them."""
    ladder = StallLadder()
    now, rungs = 0.0, []
    for _ in range(_cycles(60)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now, commanding=True,
                             output_moving=False, open_bearing_rad=math.radians(-71.4))
        if result.action == "rung" and result.rung not in rungs:
            rungs.append(result.rung)
        if result.exhausted:
            break
        now += 1.0 / HZ
    assert rungs == list(VISIBLE_RUNG_ORDER), f"visible stall walked {rungs}"


def test_a_pivot_that_never_turned_is_not_credited():
    """The other half of the pivot's clear test: BEING pointed at a gap is not the
    same as having TURNED to point at one.

    A traversal that reaches the pivot rung with the gap already ahead -- a blind
    contact, say, whose lidar can see straight past whatever is physically holding the
    wheels -- would otherwise clear the rung on its first cycle having commanded
    nothing, hand back, and re-stall against the same invisible thing.
    """
    cfg = LadderConfig()
    ladder = StallLadder(cfg)
    now, in_pivot = 0.0, False
    for _ in range(_cycles(60)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now, commanding=True,
                             output_moving=True,      # freeze -> blind order
                             open_bearing_rad=0.1)    # already inside the tolerance
        if result.rung == PIVOT_OPEN and result.action == "rung":
            in_pivot = True
        assert not (in_pivot and result.reason == f"{PIVOT_OPEN}_cleared"), (
            "the pivot rung was credited without the room having moved at all")
        now += 1.0 / HZ


def test_a_freeze_part_way_through_a_visible_traversal_retreats_again():
    """The cause can CHANGE between two stalls on one goal, and the dangerous
    direction is visible -> blind.

    A traversal that opened with the pivot has the drive-out queued next. If the very
    next stall is a freeze -- nothing explains our immobility, so something is
    physically there that no sensor sees -- resuming that plan would power the rover
    forward into it. A freeze re-opens the retreats, whatever the traversal intended.
    """
    ladder = StallLadder()
    first, now = _stall_until_rung(ladder, output_moving=False,
                                   bearing=math.radians(-71.4))
    assert first.rung == PIVOT_OPEN
    ladder.abandon_rung()
    second, _ = _stall_until_rung(ladder, t0=now + 0.1, output_moving=True,
                                  bearing=math.radians(-71.4))
    assert second.freeze, "setup failed: the second stall was not a freeze"
    assert second.rung == REVERSE_STRAIGHT, (
        f"a blind contact resumed at {second.rung} — the escalation memory carried a "
        "plan made when a sensor could still explain the stall")


def test_a_blind_contact_never_escalates_into_driving_forward_first():
    """The safety-relevant half of ordering, stated on its own.

    Whatever the lidar can see, a stall where nothing explains the immobility must try
    both retreats before it powers forward -- driving into an obstacle no sensor can
    detect is the one escape that makes contact worse.
    """
    ladder = StallLadder()
    now, rungs = 0.0, []
    for _ in range(_cycles(60)):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=now, commanding=True,
                             output_moving=True,  # permitted, immobile -> freeze
                             open_bearing_rad=math.radians(-71.4))
        if result.action == "rung" and result.rung not in rungs:
            rungs.append(result.rung)
        if result.exhausted:
            break
        now += 1.0 / HZ
    assert rungs.index(DRIVE_OPEN) > rungs.index(REVERSE_STRAIGHT)
    assert rungs.index(DRIVE_OPEN) > rungs.index(REVERSE_ARC)
    assert rungs == list(RUNG_ORDER)
