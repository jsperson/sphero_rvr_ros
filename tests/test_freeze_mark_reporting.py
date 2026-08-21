"""D35 — a count of EVENTS must not be published under a name that reads as PLACES.

Run 112721 (gauntlet reset mission 1, 2026-08-12) filed nine `freeze_marks` for six
distinct positions: `(-0.847,-1.094)` appears four times, because the explorer appended
one dict per freeze event while the controller's `FreezeMarkSet` had already merged at
0.15 m. The run's own author read the report an hour later and said nine obstacles.
That is the D24 family -- the report was not wrong about anything it measured, it was
wrong about what it was called.

The fix reports BOTH, and RENAMES. Supplementing alone would have left every
`len(report["freeze_marks"])` in existence still quietly wrong; a rename makes that
reader raise KeyError and get fixed.

WHY THE REAL RECORDING AND NOT A HAND-BUILT FIXTURE. Mission 1's marks constrain the
merge radius from BOTH sides, which is luck no invented fixture would have had:

  * four identical points prove merging happens at all;
  * the closest pair that must NOT merge -- (-0.797,-1.366) and (-0.847,-1.094) -- sit
    **0.277 m** apart, so a radius widened past that silently reports five places for
    six, and this test catches it.

A fixture with all its distinct points far apart would pass at any radius from 0.001 to
0.25 and prove only that identical points are identical.
"""

import json
import math
import re
from pathlib import Path

import pytest

from fixtures.mission_reports import (  # noqa: E402
    MISSION_1_FREEZE_EVENTS,
    MISSION_2_FREEZE_EVENTS,
)
from sphero_rvr_core.freeze_marks import FreezeMarkSet, merge_positions
from sphero_rvr_core.mission_report import (
    DEFAULT_FREEZE_MARK_MERGE_RADIUS_M,
    OUTCOME_BLOCKED_BY_UNSEEN_OBSTACLES,
    OUTCOME_COMPLETE,
    build_report,
)

ROOT = Path(__file__).resolve().parents[1]
EXPLORER = ROOT / "src" / "sphero_rvr_driver" / "coverage_explorer_node.py"
CONTROLLER = ROOT / "src" / "sphero_rvr_driver" / "decisive_controller_node.py"

DEPLOYED_RADIUS_M = 0.15


def _report(events, radius=DEPLOYED_RADIUS_M):
    return build_report(
        OUTCOME_BLOCKED_BY_UNSEEN_OBSTACLES, covered_cells=3473, resolution=0.05,
        duration_s=518.6, freeze_events=events, freeze_mark_merge_radius_m=radius,
    )


# --------------------------------------------------------------- the replay


def test_mission_1_replays_as_nine_events_and_six_positions():
    """THE REGRESSION. Replays run 112721's recorded marks through the report builder.

    Fails against efc52bc, where the report had a single `freeze_marks` list of nine
    and no notion of a place at all.
    """
    r = _report(MISSION_1_FREEZE_EVENTS)

    assert r["freeze_mark_counts"]["events"] == 9
    assert r["freeze_mark_counts"]["distinct_positions"] == 6
    assert len(r["freeze_events"]) == 9
    assert len(r["freeze_positions"]) == 6

    # The four-times-repeated point survives as exactly one place, and it is still
    # the same place -- merging must not move a mark to some centroid of the cluster.
    assert {"x": -0.847, "y": -1.094} in r["freeze_positions"]
    assert sum(1 for m in r["freeze_events"]
               if (m["x"], m["y"]) == (-0.847, -1.094)) == 4


def test_the_two_places_277_mm_apart_stay_two_places():
    """THE RADIUS IS LOAD-BEARING, and this is the pair that proves it.

    (-0.797,-1.366) and (-0.847,-1.094) are 0.277 m apart -- the closest distinct pair
    in mission 1. At the deployed 0.15 m they are two obstacles, which is what the room
    contained. Widen the radius past 0.277 and the report quietly claims five.

    This is the assertion that makes the headline count mean something: without it,
    `distinct_positions == 6` is satisfied by any radius below 0.277, and a future
    change that widened the merge would pass.
    """
    a, b = (-0.797, -1.366), (-0.847, -1.094)
    assert math.hypot(a[0] - b[0], a[1] - b[1]) == pytest.approx(0.277, abs=5e-4)

    pts = [(m["x"], m["y"]) for m in MISSION_1_FREEZE_EVENTS]
    assert len(merge_positions(pts, 0.15)) == 6
    # ... and the failure mode it guards, made explicit rather than implied:
    assert len(merge_positions(pts, 0.30)) == 5


def test_mission_2_is_the_control_case_where_both_counts_agree():
    """Run 125305's five marks are five places (closest pair 0.358 m). A report whose
    counts agree must not be made to disagree by the fix -- the defect was never
    "reports over-count", it was "reports do not say which number they mean"."""
    r = _report(MISSION_2_FREEZE_EVENTS)
    assert r["freeze_mark_counts"]["events"] == 5
    assert r["freeze_mark_counts"]["distinct_positions"] == 5


# --------------------------------------------------------------- the boundary


@pytest.mark.parametrize("separation_m,expected_places", [
    (0.14, 1),   # inside the radius -> one place
    (0.16, 2),   # outside -> two
])
def test_the_merge_boundary_sits_where_the_radius_says(separation_m, expected_places):
    events = [{"x": 0.0, "y": 0.0}, {"x": separation_m, "y": 0.0}]
    r = _report(events, radius=0.15)
    assert r["freeze_mark_counts"]["events"] == 2
    assert r["freeze_mark_counts"]["distinct_positions"] == expected_places


def test_a_mission_with_no_freezes_reports_zero_of_each_not_absent_fields():
    """An empty run must still answer the question. run 185048's empty freeze list was
    the CORRECT answer (stall_survival_ladder.md), and a report that omits the fields
    entirely forces the reader back to guessing."""
    r = build_report(OUTCOME_COMPLETE, covered_cells=10, resolution=0.05,
                     duration_s=1.0)
    assert r["freeze_events"] == []
    assert r["freeze_positions"] == []
    assert r["freeze_mark_counts"]["events"] == 0
    assert r["freeze_mark_counts"]["distinct_positions"] == 0


# --------------------------------------------------------------- the rename


def test_the_ambiguous_field_is_GONE_not_merely_supplemented():
    """The rename IS the fix's second half.

    Had `freeze_marks` been left in place beside the new counts, every existing reader
    doing `len(report["freeze_marks"])` would still get 9 and still say nine obstacles
    -- the exact misreading D35 is about, now with a correct number sitting next to it
    unread. Removing the name converts a silent misread into a KeyError.
    """
    r = _report(MISSION_1_FREEZE_EVENTS)
    assert "freeze_marks" not in r
    assert json.loads(json.dumps(r)) == r


def test_the_merge_radius_travels_with_the_count():
    """"Six distinct positions" is not a fact about a room until the separation that
    made two freezes one place is stated. An archived report must not require its
    reader to go and find out which config produced it."""
    r = _report(MISSION_1_FREEZE_EVENTS, radius=0.15)
    assert r["freeze_mark_counts"]["merge_radius_m"] == 0.15


# --------------------------------------------- same rule as the controller's


def test_merging_IS_the_controllers_rule_not_a_second_copy_of_it():
    """The report's merge must be the controller's merge, or "distinct positions"
    describes a set nothing on the robot ever computed.

    So this feeds the same events to `FreezeMarkSet` directly -- the class the
    controller runs -- and demands the identical answer, INCLUDING the greedy
    first-match-wins order (a later point merges into the first mark it is near, and
    the retained coordinates are the FIRST one's, not an average).
    """
    events = [(0.0, 0.0), (0.10, 0.0), (0.20, 0.0), (5.0, 5.0)]

    direct = FreezeMarkSet(ttl_s=1e9, merge_radius_m=0.15)
    for x, y in events:
        direct.add(x, y, 0.0)
    expected = [(m.x, m.y) for m in direct.live(0.0)]

    assert merge_positions(events, 0.15) == expected
    # Greedy, and pinned explicitly: (0.20,0) is 0.20 from (0,0) and 0.10 from
    # (0.10,0), but (0.10,0) was already absorbed INTO (0,0) and no longer exists as a
    # mark -- so (0.20,0) becomes its own place. Chaining would have given one.
    assert expected == [(0.0, 0.0), (0.20, 0.0), (5.0, 5.0)]


# NOT TESTED HERE, DELIBERATELY, and the reason is worth more than the test was.
#
# `FreezeMarkSet` carries a TTL, and mission 1 ran 518.6 s -- longer than the 300 s
# publication TTL -- so "does expiry silently drop a place from a long mission?" looks
# like it needs an assertion. It does not. `merge_positions` stamps every event at
# now=0.0, so no mark can ever be older than the moment it is pruned against and the
# TTL is inert BY CONSTRUCTION, not by luck.
#
# There was a test here asserting exactly that. Mutation testing killed it: re-enabling
# expiry at 300 s left all 11 tests passing, because the mutant and the original are
# the same program. A test that cannot fail against the bug it names is decoration, and
# this file would rather be one assertion shorter than one assertion less honest --
# D20's lesson, applied to my own work rather than someone else's.


# ------------------------------------------------- the cross-file radius pin


def _declared_default(path, name):
    m = re.search(
        r'declare_parameter\(\s*["\']%s["\']\s*,\s*([0-9.]+)\s*\)' % re.escape(name),
        path.read_text())
    assert m, f"{name} is not declared in {path.name}"
    return float(m.group(1))


def test_the_explorer_and_the_controller_merge_at_the_SAME_radius():
    """D15 class: a constant that lives in two files drifts, and this one drifts
    SILENTLY -- nothing would raise, the report would simply describe a merge nobody
    performed. The controller merges before publishing; the explorer counts places
    afterwards. If those radii differ, the count is fiction.

    The honest better answer is deferred on purpose (message-contract change, Pi ten
    commits behind): the controller should publish its radius on the freeze event so
    the explorer merges at the radius actually used. Until then this test is the seam.
    """
    explorer = _declared_default(EXPLORER, "freeze_mark_merge_radius_m")
    controller = _declared_default(CONTROLLER, "freeze_mark_merge_radius_m")

    assert explorer == controller, (
        f"explorer merges at {explorer} m, controller at {controller} m -- the "
        "report's distinct-position count would describe neither"
    )
    assert explorer == DEFAULT_FREEZE_MARK_MERGE_RADIUS_M, (
        "mission_report's fallback default disagrees with the deployed nodes"
    )
    assert explorer == DEPLOYED_RADIUS_M
