"""The goal tool's trinity, held against the day it was invented to prevent.

Goal 1 of run 3c was "1.5 m dead ahead" -- and physically inside the couch, in
SLAM-unknown space, so the planner's refusal read as a navigation failure until
Scott looked at the room. The trinity makes that mistake un-sendable: mapped free,
cost 0, dry-run plans, or no goal leaves the tool. These tests fix the policy to
the field's own cases; the ROS plumbing around it is verified by use on the Pi.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
try:
    from fly_stock_goal import LIVENESS_NODES, trinity_verdict
finally:
    sys.path.pop(0)


def test_the_couch_goal_is_refused_for_being_unmapped():
    """Run 3c goal 1's exact cell shape: /map = -1 (nobody has seen this floor).
    The reason string names the lesson so the operator learns it at refusal time."""
    ok, reason = trinity_verdict(-1, 0)
    assert not ok
    assert "unknown" in reason.lower()


def test_the_flown_goal_2_shape_passes():
    """Mapped free and cost 0 -- the instrument-picked goal that produced the stock
    middle's first success."""
    ok, reason = trinity_verdict(0, 0)
    assert ok


def test_a_goal_in_the_inflation_band_is_refused_not_graded():
    """Cost 56 at the cell (the 3d nose-line band): a goal there invites RPP to
    finish inside cost it will flinch at. Refused outright."""
    ok, reason = trinity_verdict(0, 56)
    assert not ok
    assert "56" in reason


def test_a_goal_in_mapped_occupancy_is_refused():
    ok, reason = trinity_verdict(100, 0)
    assert not ok


def test_a_goal_off_the_grid_is_refused():
    """Off the grid entirely (None from the cell reader) -- the fresh-SLAM-map case
    where the room simply is not that big yet."""
    for map_val, cost_val in ((None, 0), (0, None), (None, None)):
        ok, _ = trinity_verdict(map_val, cost_val)
        assert not ok


def test_the_liveness_roster_is_the_goal_paths_nodes():
    """bt_navigator (the owner), controller_server (the acknowledge-orphan seam),
    slam_toolbox (the card's P2 MUST). planner_server is proven by the dry-run
    itself -- the roster is deliberate, not exhaustive."""
    assert set(LIVENESS_NODES) == {"bt_navigator", "controller_server",
                                   "slam_toolbox"}
