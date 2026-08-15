"""The give-up escape's geometry, asserted against the DEPLOYED config.

docs/reverse_before_give_up_design.md §4. Gauntlet mission 2 (2026-08-12) ended
`INCOMPLETE_NO_PLANNABLE_TARGETS` standing among five of its own freeze marks, with
0.781 m of measured clear floor behind it. The escape that breaks that trap has to
travel far enough to get the robot's footprint out of its own mark's inflation, and
"far enough" is a function of three deployed numbers, not a choice.

So the arithmetic lives here rather than in a comment: a future edit to
`inflation_radius` or `freeze_mark_radius_m` that makes 0.30 m too short fails in CI,
where it costs a test run, instead of on carpet, where it costs a mission and a rover
parked against a chair. Same discipline as the death-pose margin test.
"""

import math
import os

import pytest

from sphero_rvr_core.decisive_control import freeze_mark_pose


def _params(path, *keys):
    """Read `ros__parameters` out of a deployed YAML, wherever the node key sits."""
    yaml = pytest.importorskip("yaml")
    here = os.path.dirname(__file__)
    raw = yaml.safe_load(open(os.path.join(here, "..", "config", path)))

    found = {}

    def walk(d):
        if not isinstance(d, dict):
            return
        for k, v in d.items():
            if k == "ros__parameters" and isinstance(v, dict):
                for key in keys:
                    if key in v:
                        found.setdefault(key, v[key])
            walk(v) if isinstance(v, dict) else None

    walk(raw)
    return found


def _nav2_costmap_value(key):
    """`global_costmap` values, read from the deployed nav2 YAML by scanning for the
    key inside the global costmap block — the file nests ros__parameters twice."""
    yaml = pytest.importorskip("yaml")
    here = os.path.dirname(__file__)
    raw = yaml.safe_load(open(os.path.join(here, "..", "config", "lean_nav2.yaml")))
    block = raw.get("global_costmap", {}).get("global_costmap", {})
    params = block.get("ros__parameters", {})
    if key in params:
        return params[key]
    for v in params.values():
        if isinstance(v, dict) and key in v:
            return v[key]
    pytest.skip(f"{key} not found in the deployed global_costmap block")


# The three numbers that build the trap, as DEPLOYED.
MARK_RADIUS_M = 0.14          # freeze_mark_radius_m, decisive_controller_node default
COMMANDED_ESCAPE_M = 0.30     # give_up_escape_distance_m, same node
FOOTPRINT_FRONT_M = 0.11      # where a forward mark is planted, relative to centre


def test_the_escape_is_long_enough_to_leave_the_robots_own_mark():
    """THE DERIVED-CONSTANTS ASSERTION.

    A mark is a lethal disc of `freeze_mark_radius_m` planted `footprint_front_m`
    ahead of the robot's centre, so at the instant of planting the centre sits INSIDE
    its own mark. Two distances then matter, and the escape must beat the larger:

      inscribed clear  centre-to-mark >= mark_radius + robot_radius
      inflation clear  centre-to-mark >= mark_radius + inflation_radius
    """
    robot_radius = float(_nav2_costmap_value("robot_radius"))
    inflation = float(_nav2_costmap_value("inflation_radius"))

    inscribed_needed = MARK_RADIUS_M + robot_radius - FOOTPRINT_FRONT_M
    inflation_needed = MARK_RADIUS_M + inflation - FOOTPRINT_FRONT_M

    assert inscribed_needed == pytest.approx(0.17, abs=0.005), (
        f"inscribed-clearance reverse moved to {inscribed_needed:.3f} m — the design "
        "note's §4 table is stale, re-derive it")
    assert inflation_needed == pytest.approx(0.19, abs=0.005), (
        f"inflation-clearance reverse moved to {inflation_needed:.3f} m — re-derive §4")
    assert COMMANDED_ESCAPE_M > inflation_needed, (
        f"the commanded escape ({COMMANDED_ESCAPE_M:.2f} m) no longer clears the "
        f"robot's own freeze mark ({inflation_needed:.3f} m needed at "
        f"inflation_radius={inflation}, freeze_mark_radius={MARK_RADIUS_M}). A rover "
        "that escapes and is still inside its own inflation cannot plan, which is "
        "exactly the trap this escape exists to break.")
    margin = COMMANDED_ESCAPE_M - inflation_needed
    assert margin >= 0.10, (
        f"only {margin:.3f} m of margin against odometry error; the design allowed "
        "0.11 m and anything much tighter is a coin toss on carpet")


def test_one_escape_is_not_claimed_to_clear_a_whole_mark_field():
    """The honest half of §4: mission 2's trap was five marks inside about a metre,
    and a single 0.30 m reverse can move AWAY from one mark and TOWARD another.

    This test does not assert that the escape always works — it asserts that the
    geometry does not support such a claim, so nobody later reads the margin above as
    a convergence guarantee. Convergence comes from the monotonic escape budget and
    the re-plan loop, not from this number.
    """
    marks = [(-1.135, 0.840), (-1.134, 0.295), (-0.853, 0.559),
             (-0.779, -0.199), (-0.488, 0.010)]          # mission 2, as recorded
    x, y = -1.270, 0.625                                  # where it gave up
    inflation = float(_nav2_costmap_value("inflation_radius"))
    blocked = MARK_RADIUS_M + inflation

    # Reverse 0.30 m along every heading and count how many headings land clear of
    # every mark. If that were all of them, the "one escape always works" reading
    # would be defensible; it is not.
    clear = 0
    for i in range(72):
        th = i * math.pi / 36.0
        nx, ny = x - COMMANDED_ESCAPE_M * math.cos(th), y - COMMANDED_ESCAPE_M * math.sin(th)
        if all(math.hypot(nx - mx, ny - my) > blocked for mx, my in marks):
            clear += 1
    assert clear < 72, (
        "every heading escapes this recorded five-mark field in one move — if that "
        "is really true the design note's non-convergence caveat should be deleted, "
        "not left as folklore")
    assert clear > 0, (
        "no heading escapes the recorded field in one move, which would make the "
        "single-escape design useless here rather than merely unguaranteed")


# ------------------------------------------------------- R1: which edge gets marked

def test_a_reverse_freeze_marks_behind_the_robot():
    """REVERT-PROOF 7. A freeze while REVERSING means the obstacle is BEHIND.

    Marking the leading edge there plants a lethal disc on the clear floor ahead and
    leaves the real obstacle unmarked — poisoning the costmap in the one situation
    where the rover most needs the floor ahead to stay plannable. Fails against HEAD,
    which projects `+footprint_front_m` along yaw regardless of direction.
    """
    x, y, yaw = 1.0, 2.0, 0.0            # facing +x
    fx, fy = freeze_mark_pose(x, y, yaw, front_m=0.11, rear_m=0.16, reversing=True)
    assert fx < x, (
        f"a reverse freeze marked {fx - x:+.3f} m along the heading — that is IN "
        "FRONT of the robot, on the floor it was backing away from")
    assert fx == pytest.approx(x - 0.16, abs=1e-9), "trailing edge is footprint_rear_m"
    assert fy == pytest.approx(y, abs=1e-9)


def test_a_forward_freeze_still_marks_the_leading_edge():
    """The PAIRED NEGATIVE. D25's leading-edge correction (run 20260811_093818's
    face-walking against the chair) must not be traded away by the reverse fix."""
    x, y, yaw = 1.0, 2.0, 0.0
    fx, fy = freeze_mark_pose(x, y, yaw, front_m=0.11, rear_m=0.16, reversing=False)
    assert fx == pytest.approx(x + 0.11, abs=1e-9)
    assert fy == pytest.approx(y, abs=1e-9)


def test_the_mark_follows_the_heading_not_the_world_axes():
    """Both edges rotate with the robot; a mark placed along a world axis would be
    wrong everywhere except due east."""
    x, y, yaw = 0.0, 0.0, math.pi / 2.0        # facing +y
    fwd = freeze_mark_pose(x, y, yaw, 0.11, 0.16, reversing=False)
    rev = freeze_mark_pose(x, y, yaw, 0.11, 0.16, reversing=True)
    assert fwd[1] == pytest.approx(0.11, abs=1e-9) and abs(fwd[0]) < 1e-9
    assert rev[1] == pytest.approx(-0.16, abs=1e-9) and abs(rev[0]) < 1e-9


# --------------------------------------------------- F-A: the freeze predicate rolls

def test_a_creeping_pinned_reverse_is_still_a_freeze():
    """F-A, and it is mission 1's own measurement.

    A rover pinned against something invisible does not sit perfectly still: 13
    recorded straight-reverse windows against a blind contact, supervisor granting
    79% of cycles, achieved a MEAN of 0.086 m — nearly 3x `progress_epsilon_m` 0.03.
    A freeze test that compares TOTAL travel since the escape began therefore reads
    "we are moving" for the whole escape and never fires, in exactly the case the
    escape exists for: no mark is planted behind the obstacle and the seam publishes
    `refused` while the supervisor was in fact permitting motion.

    The rule has to be the ladder's rolling one. Fails against the cumulative version.
    """
    from sphero_rvr_core.decisive_control import WindowedFreezeMonitor

    mon = WindowedFreezeMonitor(window_cycles=20, expected_per_cycle_m=0.01)
    # 0.086 m per 3 s window at 10 Hz = 0.0029 m per cycle against a commanded
    # 0.01 m per cycle: 29% delivered, exactly the signature the recorder caught.
    x = 0.0
    fired_at = None
    for i in range(60):
        x -= 0.086 / 30.0
        if mon.update(x, 0.0, output_moving=True):
            fired_at = i
            break
    assert fired_at is not None, (
        "a pinned, creeping, PERMITTED reverse was never classified as a freeze — "
        "the mark goes unplanted and the outcome lies about what happened")
    assert fired_at < 30, f"took {fired_at} cycles to notice; the window is 20"

    # AND IT MUST KEEP SAYING SO. This is what separates a SLIDING window from a
    # measurement taken since the escape began: cumulative travel grows without
    # bound, so a cumulative test eventually passes its own bar and reports the pin
    # as motion again — 0.0029 m/cycle crosses 50% of a 0.19 m window budget at about
    # cycle 33, which is inside a 6 s escape. The rover is still pinned; only the
    # arithmetic moved on.
    for _ in range(80):
        x -= 0.086 / 30.0
        assert mon.update(x, 0.0, output_moving=True), (
            "the classifier stopped calling a still-pinned rover frozen — the window "
            "is not sliding")


def test_a_genuinely_moving_reverse_is_never_a_freeze():
    """The paired negative: an escape that is actually backing out must not be
    reported as frozen, or every successful escape plants a phantom mark behind it."""
    from sphero_rvr_core.decisive_control import WindowedFreezeMonitor

    mon = WindowedFreezeMonitor(window_cycles=20, expected_per_cycle_m=0.01)
    x = 0.0
    for _ in range(120):                      # 0.10 m/s at 10 Hz = 0.01 m per cycle
        x -= 0.01
        assert not mon.update(x, 0.0, output_moving=True), (
            "a reverse travelling at the commanded speed was called a freeze")


def test_a_refused_reverse_is_not_a_freeze_either():
    """Refused is not frozen: if the supervisor is zeroing us, the stall is explained
    and there is nothing invisible to mark. Same distinction the ladder draws."""
    from sphero_rvr_core.decisive_control import WindowedFreezeMonitor

    mon = WindowedFreezeMonitor(window_cycles=20, expected_per_cycle_m=0.01)
    for _ in range(120):
        assert not mon.update(0.0, 0.0, output_moving=False)


def test_a_supervisor_SLOWED_reverse_is_not_a_freeze():
    """The margin that makes the rate test safe, asserted rather than asserted-at.

    The supervisor legitimately scales a granted command down -- 0.70 in the lidar
    SLOW band, 0.60 for the camera's. A rover delivering 60% of what it asked for is
    being throttled, not pinned, and calling that a freeze would plant phantom marks
    every time the escape passed near anything. The pin delivered 29%; the threshold
    sits at 50%, between the two on purpose.
    """
    from sphero_rvr_core.decisive_control import WindowedFreezeMonitor

    mon = WindowedFreezeMonitor(window_cycles=20, expected_per_cycle_m=0.01)
    x = 0.0
    for _ in range(120):
        x -= 0.01 * 0.60                      # the slowest legitimate scaling
        assert not mon.update(x, 0.0, output_moving=True), (
            "a supervisor-SLOWED reverse was classified as a freeze")


# ---------------------------------------------------------------------------
# REVERT-PROOF 1c (D40) — the give-up escape's COMMAND SHAPE must be grantable
# at the poses where it is meant to fire.
#
# Gauntlet 2026-08-14b: four give-up escapes, four refusals, all at one pose,
# "refused: 0.000 m in 6.0 s; supervisor: rear_hold". The by-bearing table at
# all four refusal stamps (re-derived from the bag, odom_yaw -146.1 deg at every
# one, geometry stable to 7 mm) read 7 of 12 bearings OPEN with 2.18 m of floor
# at 4 o'clock -- while the rear, the ONLY direction the escape ever tried, was
# genuinely blocked at 0.150 m.
#
# Mechanism, arithmetic: rear_hold zeroes linear_x and passes angular_z through
# UNTOUCHED. The escape commanded (-speed, 0.0). Zero linear from the gate plus
# zero angular from the escape is exactly (0.0, 0.0), forever, at any pose where
# the rear sector sits inside reverse_stop_distance_m.
#
# This is the UN-GRANTABLE-BY-CONSTRUCTION form of the recovery-defect family:
# the code runs, the trigger fires, and the arbiter must refuse it anyway because
# of the command's shape. The check is asked of the ARBITER, not the caller.
# ---------------------------------------------------------------------------

WEDGE_REAR_M = 0.150          # 7 o'clock, min over all four refusal frames
WEDGE_OPEN_M = 2.18           # 4 o'clock, the floor that was never asked for


def _wedge_scan(stamp=0.0, count=360):
    """Tonight's refusal geometry: rear pinned at 0.150 m, open elsewhere."""
    from tests.test_collision_stop import scan_with
    return scan_with(rear=WEDGE_REAR_M, stamp=stamp, count=count)


def _supervisor_at_the_wedge():
    """A supervisor carrying the DEPLOYED reverse stop distance, not a default.

    The deployed value is the one that decides whether rear_hold fires, so it is
    read from config/collision_stop.yaml rather than trusted from the dataclass --
    13 fields have differed between the two before, and a verdict flipped between
    them.
    """
    from sphero_rvr_driver.collision_stop import (
        CollisionStopConfig,
        CollisionStopSupervisor,
    )

    deployed = _params("collision_stop.yaml", "reverse_stop_distance_m")
    reverse_stop = deployed.get("reverse_stop_distance_m")
    assert reverse_stop is not None, "deployed reverse_stop_distance_m not found"
    assert WEDGE_REAR_M < reverse_stop, (
        f"the recorded wedge rear ({WEDGE_REAR_M} m) must sit INSIDE the deployed "
        f"reverse stop distance ({reverse_stop} m) or this pose does not reproduce "
        f"the rear_hold refusal at all"
    )

    cfg = CollisionStopConfig(
        reverse_stop_distance_m=float(reverse_stop),
        min_valid_ranges=1,
        min_valid_fraction=0.0,
    )
    sup = CollisionStopSupervisor(cfg, now=0.0)
    sup.update_scan(_wedge_scan(stamp=0.0), now=0.0)
    return sup


def test_the_straight_reverse_the_escape_used_to_send_is_ungrantable_here():
    """The defect, pinned as arithmetic so it cannot be argued about.

    Not a regression guard -- a statement of WHY the shape had to change. If this
    ever stops holding, the supervisor's rear_hold contract moved and revert-proof
    1c below is measuring something else.
    """
    from sphero_rvr_driver.collision_stop import TwistCommand

    sup = _supervisor_at_the_wedge()
    decision = sup.apply_command(TwistCommand(-0.10, 0.0), now=0.1)

    assert decision.reason == "rear_hold"
    assert decision.output.linear_x == 0.0
    assert decision.output.angular_z == 0.0, (
        "a straight reverse at this pose yields NO motion on either axis -- "
        "which is why four give-up escapes delivered 0.000 m in 6.0 s"
    )


def test_the_give_up_escape_command_shape_is_grantable_at_a_rear_hold_pose():
    """REVERT-PROOF 1c. Fails against the straight-reverse escape.

    The shape is taken from PRODUCTION (`escape_arc_command`), not restated here,
    so this binds to what the escape actually sends. Mutating that function back to
    a zero angular term must fail this test -- that mutation was run and it does.
    """
    from sphero_rvr_core.decisive_control import escape_arc_command
    from sphero_rvr_driver.collision_stop import TwistCommand

    linear, angular = escape_arc_command(
        speed_mps=0.10, arc_rate_rad_s=0.40, open_bearing_rad=-0.9,
    )
    assert linear < 0.0, "the escape must still be a REVERSE"

    sup = _supervisor_at_the_wedge()
    decision = sup.apply_command(TwistCommand(linear, angular), now=0.1)

    granted = (decision.output.linear_x, decision.output.angular_z)
    assert granted != (0.0, 0.0), (
        "the give-up escape's commanded shape is refused to a dead stop at the very "
        "pose class it exists for -- un-grantable by construction (D40)"
    )
    assert decision.output.angular_z != 0.0, (
        "rear_hold passes angular through untouched, so the arc term is what survives"
    )


def test_escape_arc_command_never_degenerates_to_a_straight_reverse():
    """The shape invariant, across every direction input including the empty ones.

    A None or zero bearing must NOT collapse back to angular 0.0 -- that is the
    broken shape, and "no preference" is the case most likely to reintroduce it.
    """
    from sphero_rvr_core.decisive_control import escape_arc_command

    for bearing in (-1.2, -0.01, 0.0, 0.01, 1.2, None):
        linear, angular = escape_arc_command(0.10, 0.40, bearing)
        assert linear < 0.0
        assert angular != 0.0, f"degenerated to a straight reverse at bearing={bearing}"
        assert abs(angular) == pytest.approx(0.40)


def test_escape_arc_turns_toward_the_open_bearing():
    """Sign follows the open side. Direction is the FALLBACK source (lidar-height);
    the trail is the preferred one and arrives with A1."""
    from sphero_rvr_core.decisive_control import escape_arc_command

    assert escape_arc_command(0.10, 0.40, -0.9)[1] < 0.0     # open to the right
    assert escape_arc_command(0.10, 0.40, +0.9)[1] > 0.0     # open to the left
