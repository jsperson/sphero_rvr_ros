"""The body->map transform, and the refusals that are part of the verb.

Scenario 8 scored "no relative motion at all" as one of Scott's three missing
instruction classes. This is the arithmetic half; the node wiring is separate and
deliberately so -- a transform that cannot be tested without ROS would be untestable
on the machine where the work happens (norm 16's lesson, applied before the fact).
"""

from __future__ import annotations

import math

import pytest

from sphero_rvr_core.relative_motion import (MAX_DISTANCE_M, MIN_DISTANCE_M,
                                             RelativeMotionError, describe,
                                             relative_goal)


def test_forward_from_the_origin_facing_east():
    assert relative_goal((0.0, 0.0, 0.0), 2.0) == pytest.approx((2.0, 0.0))


def test_forward_respects_the_rovers_yaw():
    """THE WHOLE POINT OF THE VERB. `goto` is absolute; "forward" is not, and the
    difference is the yaw the agent would otherwise have to apply itself."""
    got = relative_goal((1.0, 1.0, math.pi / 2), 2.0)         # facing north
    assert got == pytest.approx((1.0, 3.0), abs=1e-9)
    got = relative_goal((1.0, 1.0, math.pi), 2.0)             # facing west
    assert got == pytest.approx((-1.0, 1.0), abs=1e-9)


def test_heading_is_measured_in_the_body_frame_not_the_map():
    """+90 is the ROVER's left, whatever the rover is facing. A heading that meant
    map-north would be a different verb wearing the same name."""
    facing_north = relative_goal((0.0, 0.0, math.pi / 2), 1.0, heading_deg=90.0)
    assert facing_north == pytest.approx((-1.0, 0.0), abs=1e-9)   # left of north = west
    facing_east = relative_goal((0.0, 0.0, 0.0), 1.0, heading_deg=90.0)
    assert facing_east == pytest.approx((0.0, 1.0), abs=1e-9)     # left of east = north


def test_the_compose_case_scott_asked_for():
    """"Pivot 180 then forward 1 m" -- the pivot is the EXISTING `turn` verb, so by
    the time this runs the rover is already facing the other way. The destination is
    then plain forward, and it lands where "180 then forward" should put it."""
    start = (2.0, 0.0, 0.0)
    after_pivot = (2.0, 0.0, math.pi)          # what turn(180) leaves behind
    assert relative_goal(after_pivot, 1.0) == pytest.approx((1.0, 0.0), abs=1e-9)
    # and asking for it in ONE call, as a backward move, agrees -- the two spellings
    # of the same destination must not disagree
    assert relative_goal(start, 1.0, heading_deg=180.0) == pytest.approx(
        (1.0, 0.0), abs=1e-9)


def test_no_pose_is_refused_in_words_not_assumed():
    with pytest.raises(RelativeMotionError, match="no pose"):
        relative_goal(None, 1.0)


def test_a_move_the_goal_tolerance_would_swallow_is_refused():
    """`xy_goal_tolerance` is 0.12 m deployed: a 0.05 m request would report SUCCESS
    without the rover moving. Refusing is the honest answer; clamping up to the
    minimum would be answering a different instruction."""
    with pytest.raises(RelativeMotionError, match="minimum"):
        relative_goal((0.0, 0.0, 0.0), 0.05)
    assert relative_goal((0.0, 0.0, 0.0), MIN_DISTANCE_M)[0] == pytest.approx(MIN_DISTANCE_M)


def test_absurd_distances_and_headings_are_refused_at_the_contract():
    with pytest.raises(RelativeMotionError, match="exceeds"):
        relative_goal((0.0, 0.0, 0.0), MAX_DISTANCE_M + 0.01)
    for bad in (float("nan"), float("inf")):
        with pytest.raises(RelativeMotionError):
            relative_goal((0.0, 0.0, 0.0), bad)
    with pytest.raises(RelativeMotionError, match="heading"):
        relative_goal((0.0, 0.0, 0.0), 1.0, heading_deg=181.0)


def test_nothing_here_clamps():
    """A silently-adjusted distance is a different instruction from the one given.
    Every out-of-range input raises; none returns a nearby legal answer."""
    for distance in (0.01, MAX_DISTANCE_M * 2):
        with pytest.raises(RelativeMotionError):
            relative_goal((0.0, 0.0, 0.0), distance)


def test_it_says_back_what_it_understood():
    assert describe(2.0) == "forward 2.00 m"
    assert describe(1.0, 180.0) == "backward 1.00 m"
    assert "left" in describe(1.0, 90.0) and "right" in describe(1.0, -90.0)
