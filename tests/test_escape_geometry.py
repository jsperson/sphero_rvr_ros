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
