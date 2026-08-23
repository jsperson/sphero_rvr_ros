"""D64: the gate keeps its teeth, the operator stops guessing.

Hand-built rooms, no ROS. The load-bearing case is IN_SHADOW vs BEYOND_HORIZON:
they were one sentence until 2026-08-22 and they call for opposite actions.
"""

import math

import pytest

from sphero_rvr_core.goal_legality import (
    BEYOND_HORIZON, Grid, INFLATED, IN_SHADOW, LEGAL, OCCUPIED, OFF_GRID,
    assess, classify, nearest_legal,
)

RES = 0.05
W = H = 60          # 3 m x 3 m, origin at (-1.5, -1.5)
OX = OY = -1.5


def rooms(free_to_x=1.0, obstacle=None, inflate_around=None):
    """(map, cost) grids: free floor out to `free_to_x`, unknown beyond.

    `obstacle`: (x, y) mapped-occupied cell. `inflate_around`: (x, y) whose
    neighbourhood gets nonzero cost while /map still reads free.
    """
    m = [-1] * (W * H)
    c = [0] * (W * H)
    for cy in range(H):
        for cx in range(W):
            wx = OX + (cx + 0.5) * RES
            wy = OY + (cy + 0.5) * RES
            if -1.0 <= wx <= free_to_x and -1.0 <= wy <= 1.0:
                m[cy * W + cx] = 0
    map_grid = Grid(m, W, H, RES, OX, OY)
    cost_grid = Grid(c, W, H, RES, OX, OY)
    if obstacle is not None:
        i = map_grid.index(*obstacle)
        m[i] = 100
        for d in (-1, 0, 1):
            for e in (-1, 0, 1):
                m[i + d + e * W] = 100
    if inflate_around is not None:
        i = cost_grid.index(*inflate_around)
        for d in (-2, -1, 0, 1, 2):
            for e in (-2, -1, 0, 1, 2):
                c[i + d + e * W] = 99
    return map_grid, cost_grid


# --- the classification, which is the point ---------------------------------

def test_open_floor_is_legal():
    m, c = rooms()
    v = classify(m, c, (0.0, 0.0), (0.5, 0.0))
    assert v.code == LEGAL and v.legal


def test_unknown_with_nothing_in_the_way_is_BEYOND_HORIZON():
    """The map simply ends. Driving closer makes this cell legal later."""
    m, c = rooms(free_to_x=1.0)
    v = classify(m, c, (0.0, 0.0), (1.3, 0.0))
    assert v.code == BEYOND_HORIZON
    assert v.improves_with_motion
    assert "drive closer" in v.reason.lower()


def test_unknown_BEHIND_AN_OBSTACLE_is_IN_SHADOW():
    """The 2026-08-22 case: a staged object shadows every cell behind it, so no
    goal there can EVER be legal until the rover moves. This is the distinction
    that cost minutes of guessing."""
    m, c = rooms(free_to_x=1.0, obstacle=(0.55, 0.0))
    v = classify(m, c, (0.0, 0.0), (1.3, 0.0))
    assert v.code == IN_SHADOW
    assert v.improves_with_motion
    assert "will not help" in v.reason.lower()
    # names the blocker's range — the ray's FIRST hit, i.e. the obstacle's near
    # EDGE (0.45 m for a 3-cell block centred at 0.55), not its centre
    blocker_m = float(v.reason.split("blocker is ")[1].split(" m")[0])
    assert 0.35 < blocker_m < 0.60


def test_shadow_and_horizon_are_not_confused_at_the_same_range():
    """Same goal, same distance, same /map value — only an obstacle differs."""
    goal = (1.3, 0.0)
    plain = classify(*rooms(free_to_x=1.0), (0.0, 0.0), goal)
    shadowed = classify(*rooms(free_to_x=1.0, obstacle=(0.55, 0.0)), (0.0, 0.0), goal)
    assert plain.code == BEYOND_HORIZON and shadowed.code == IN_SHADOW


def test_occupied_and_inflated_and_off_grid():
    m, c = rooms(obstacle=(0.5, 0.0))
    assert classify(m, c, (0.0, 0.0), (0.5, 0.0)).code == OCCUPIED
    m2, c2 = rooms(inflate_around=(0.5, 0.0))
    v = classify(m2, c2, (0.0, 0.0), (0.5, 0.0))
    assert v.code == INFLATED and "cost 99" in v.reason
    assert classify(m2, c2, (0.0, 0.0), (99.0, 99.0)).code == OFF_GRID


def test_only_legal_says_legal():
    """A gate that can only pass is not a gate: every non-LEGAL code must be
    refused by `legal`."""
    m, c = rooms(free_to_x=1.0, obstacle=(0.55, 0.0))
    for goal in ((1.3, 0.0), (0.55, 0.0), (99.0, 0.0)):
        assert not classify(m, c, (0.0, 0.0), goal).legal


# --- the proposal ------------------------------------------------------------

def test_nearest_legal_is_near_the_REQUEST_not_the_robot():
    m, c = rooms(free_to_x=1.0)
    found = nearest_legal(m, c, (0.0, 0.0), (1.3, 0.0))
    assert found is not None
    (nx, ny), offset = found
    assert offset < 0.35, "the proposal should be a small change to the request"
    assert nx > 0.9, "it should be near the requested end of the room, not at the robot"
    assert classify(m, c, (0.0, 0.0), (nx, ny)).legal


def test_assess_attaches_the_proposal_to_the_refusal():
    m, c = rooms(free_to_x=1.0)
    v = assess(m, c, (0.0, 0.0), (1.3, 0.0))
    assert v.code == BEYOND_HORIZON and not v.legal
    assert v.nearest is not None and v.nearest_offset_m is not None
    assert classify(m, c, (0.0, 0.0), v.nearest).legal, (
        "the tool must never propose a goal its own gate would refuse")


def test_assess_leaves_a_legal_goal_alone():
    m, c = rooms()
    v = assess(m, c, (0.0, 0.0), (0.5, 0.0))
    assert v.legal and v.nearest is None


def test_no_proposal_when_nothing_legal_is_within_reach():
    m, c = rooms(free_to_x=1.0)
    v = assess(m, c, (0.0, 0.0), (1.45, 1.45), max_offset_m=0.10)
    assert v.nearest is None, "a distant proposal is a different errand"


def test_the_proposal_respects_inflation():
    """The band is exactly what the operator cannot see; a proposal inside it
    would hand back the same refusal."""
    m, c = rooms(inflate_around=(0.5, 0.0))
    v = assess(m, c, (0.0, 0.0), (0.5, 0.0))
    assert v.code == INFLATED
    assert v.nearest is not None
    assert c.at(*v.nearest) == 0
    assert math.hypot(v.nearest[0] - 0.5, v.nearest[1]) >= 0.10
