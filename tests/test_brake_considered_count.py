"""The count the brake actually considered, and its binding to the distance beside it.

Autopsy #2 (2026-08-15) spent a night on a defect that did not exist, because
`/tof/state`'s `obstacle_zones` counts rule-B zones over the SENSOR'S WHOLE REACH
while `cam_scale` acts only inside the brake's range window and swept path. Nothing
published the second population, so the first stood in for it and "zones cycled
0 -> 10 while cam_scale never left 1.00" read as detections being lost between the
sensor and the brake. They were at 0.542-1.563 m against a deployed 0.60 m reach.

These tests exist so the new count cannot drift into being a THIRD population.
"""

import math

import pytest

from sphero_rvr_core.low_obstacle_brake import (
    NoSweptPath,
    points_in_swept_path,
    swept_path_obstacle,
)

HALF_W = 0.16          # deployed low_obstacle_half_width_m
MIN_R, MAX_R = 0.14, 0.60   # deployed low_obstacle_min/max_range_m


def _grid():
    """A spread of points wide and deep enough to fall on both sides of every
    filter: inside and outside the range window, on and off the swept corridor,
    ahead and behind."""
    pts = []
    for x in (-0.40, -0.10, 0.05, 0.20, 0.35, 0.50, 0.75, 1.20):
        for y in (-0.50, -0.20, -0.10, 0.0, 0.10, 0.20, 0.50):
            pts.append((x, y))
    return pts


COMMANDS = [
    (0.20, 0.0),      # straight
    (0.20, 0.40),     # forward-left arc
    (0.20, -0.40),    # forward-right arc
    (0.10, 0.05),     # nearly straight, but `turning` by the 1e-3 test
    (-0.10, 0.40),    # reverse arc -- the give-up escape's shape
]


@pytest.mark.parametrize("v,w", COMMANDS)
def test_the_count_and_the_distance_describe_THE_SAME_set(v, w):
    """The load-bearing property. A count filtered differently from the distance it
    sits beside would be a NEW instance of the very defect it was added to detect --
    this time inside the instrument.
    """
    pts = _grid()
    nearest = swept_path_obstacle(pts, v, w, HALF_W, MIN_R, MAX_R)
    count = points_in_swept_path(pts, v, w, HALF_W, MIN_R, MAX_R)

    assert (nearest is None) == (count == 0), (
        f"({v},{w}): nearest={nearest} and count={count} disagree about whether the "
        f"swept path holds anything at all"
    )
    if nearest is not None:
        assert count >= 1
        assert nearest == pytest.approx(min(
            math.hypot(x, y) for (x, y) in pts
            if _survives(x, y, v, w)
        )), "the reported distance is not the minimum of the counted set"
        assert count == sum(1 for (x, y) in pts if _survives(x, y, v, w))


def _survives(x, y, v, w):
    """Membership expressed through PRODUCTION, one point at a time, rather than by
    restating the swept geometry here. A test that hardcoded the corridor maths would
    prove only its own consistency."""
    return points_in_swept_path([(x, y)], v, w, HALF_W, MIN_R, MAX_R) == 1


@pytest.mark.parametrize("v,w", COMMANDS)
def test_nothing_outside_the_range_window_is_ever_counted(v, w):
    """The autopsy's own mechanism, pinned. Points beyond the brake's reach are not
    a lost detection -- they are out of scope, and the count must say so."""
    far = [(0.90, 0.0), (1.20, 0.05), (1.50, -0.05)]     # all > MAX_R
    near = [(0.05, 0.0), (0.10, 0.02)]                   # all < MIN_R
    assert points_in_swept_path(far, v, w, HALF_W, MIN_R, MAX_R) == 0
    assert points_in_swept_path(near, v, w, HALF_W, MIN_R, MAX_R) == 0
    assert swept_path_obstacle(far + near, v, w, HALF_W, MIN_R, MAX_R) is None


def test_the_08_15_geometry_reproduces_the_autopsys_numbers():
    """The recorded specimen, as a fixture rather than as a story.

    During the zone sequence the nearest ToF point sat at 0.542 m -- inside the
    0.60 m reach, and STILL correctly ignored, because for the commanded arc
    (v=0.20, w=-0.40) it was off the swept corridor. A straight-ahead cone would have
    returned it. That single frame is why the population argument needed the SWEPT
    filter and not just the range window.
    """
    # A point at 0.542 m off to the left while the rover arcs RIGHT.
    bearing = math.radians(35.0)
    p = [(0.542 * math.cos(bearing), 0.542 * math.sin(bearing))]

    assert points_in_swept_path(p, 0.20, -0.40, HALF_W, MIN_R, MAX_R) == 0, (
        "a point off the commanded arc must not be counted as considered"
    )
    assert swept_path_obstacle(p, 0.20, -0.40, HALF_W, MIN_R, MAX_R) is None


def test_zero_considered_is_not_the_same_fact_as_not_looking():
    """`0` means the brake looked and the swept path was clear. The node reports an
    EMPTY field when it did not look at all -- disabled, not driving forward, or no
    fresh cloud. Conflating them is how "the brake never fired" gets written down
    when the truth is "the brake was never asked"."""
    assert points_in_swept_path([], 0.20, 0.0, HALF_W, MIN_R, MAX_R) == 0
    assert swept_path_obstacle([], 0.20, 0.0, HALF_W, MIN_R, MAX_R) is None


@pytest.mark.parametrize("v,w", [
    (0.0, 0.0),        # stationary
    (0.0, 0.40),       # PURE PIVOT -- the shape 2c's execute stage will ask about
    (0.0, -0.40),
    (0.0005, 0.40),    # under the translation threshold: a pivot with drift
    (-0.0005, 0.0),
])
def test_a_non_translating_command_is_REFUSED_not_answered(v, w):
    """The edge pinned as a recording on 2026-08-15, now closed.

    UNTIL THIS GUARD a non-translating command fell through the `turning` test
    (which needs |v| > 1e-3) into the STRAIGHT branch, where `linear_mps >= 0.0`
    admitted the forward corridor. A pivot query therefore got a confident,
    plausible answer about a motion it was not asking about. A pivot sweeps the
    footprint's CORNER CIRCLE; a forward corridor is a different question, and
    modelling the rotation annulus is deliberately not in scope -- refusal is.

    THE GUARD MOVED INTO THE PRIMITIVE rather than staying in the caller, because
    the caller that would misuse it is not hypothetical: the escape planner's
    execute stage asks plan-time "would this shape be granted" questions and a pivot
    is one of the shapes. The guard that lives in the caller is the guard the next
    caller forgets -- the arbiter-not-caller lesson from the other direction.

    IT RAISES RATHER THAN RETURNING None, and that is the load-bearing choice.
    `None` already means CLEAR here, and `forward_speed_scale(None, ...)` is 1.0 --
    full speed. Returning None for an unanswerable query would hand out the most
    permissive answer in the API for the one input the function cannot model:
    fail-open wearing a refusal's clothes.

    The threshold is the SAME constant the turning test uses, so the two can never
    disagree about where translation begins. A second threshold would leave a band
    of near-pivots still getting the corridor answer.
    """
    pts = _grid()
    with pytest.raises(NoSweptPath):
        swept_path_obstacle(pts, v, w, HALF_W, MIN_R, MAX_R)
    with pytest.raises(NoSweptPath):
        points_in_swept_path(pts, v, w, HALF_W, MIN_R, MAX_R)


def test_the_refusal_never_eats_a_translating_command():
    """The guard must not swallow the real work. Anything genuinely translating still
    answers, including the slowest speed the escape actually commands (0.10 m/s)."""
    pts = _grid()
    for v, w in ((0.10, 0.0), (0.10, 0.40), (-0.10, 0.40), (0.0011, 0.0)):
        swept_path_obstacle(pts, v, w, HALF_W, MIN_R, MAX_R)
        points_in_swept_path(pts, v, w, HALF_W, MIN_R, MAX_R)


def test_the_production_caller_can_never_trigger_the_refusal():
    """PREMISE TRIPWIRE -- survives its own mutation, on purpose.

    Raising inside a safety-path primitive is only safe because
    `_apply_low_obstacle_brake` returns early on `linear_x <= 0.0`, so the flying
    system never reaches it. This asserts that early return still exists. If it ever
    goes, an exception becomes reachable inside the publish loop, and this test is
    where that gets noticed rather than in a room.
    """
    import re
    from pathlib import Path

    node = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver"
            / "collision_stop_node.py").read_text()
    body = node[node.index("def _apply_low_obstacle_brake"):]
    body = body[:body.index("def _on_timer")]
    assert re.search(r"if not self\._lowobs_enable or linear_x <= 0\.0:\s*\n\s*return", body), (
        "the low-obstacle brake's non-positive-speed early return is gone, so "
        "NoSweptPath is now reachable from the publish path"
    )
