"""The viewpoint standoff: a goal the safety stack won't let the rover reach is not a goal.

Cert attempt 2 (2026-08-19): the mission covered 37 m² and then died retrying
wall-adjacent cells — approach points the PLANNER approved parked the rover
inside its own reflex envelope (supervisor stop_distance 0.30 m + footprint
front 0.0965 m), the stall kill burned five goals in a row, and the breaker
ended an honest mission the geometry made unwinnable. These tests hold the
ratified fix (PM pins 1–4, 2026-08-19): a DERIVED standoff with receipts, the
excluded counted, and the easy 90% of goals untouched.
"""

import math
import re
from pathlib import Path

from sphero_rvr_core.coverage_exploration import (
    VIEWPOINT_STANDOFF_M,
    CoverageConfig,
    candidate_goals,
    cluster_has_viewpoint,
    point_clears_standoff,
)

ROOT = Path(__file__).resolve().parents[1]
RES = 0.05


def grid(w, h):
    return bytearray(w * h)  # all free (0)


def wall(occ, w, cells):
    for cx, cy in cells:
        occ[cy * w + cx] = 100


def test_the_standoff_is_derived_from_the_deployed_config_with_receipts():
    """Pin 1: stop_distance_m + footprint_front_m from config/collision_stop.yaml,
    never a folk number. If the deployed envelope changes, this fails and the
    constant gets re-derived rather than silently drifting."""
    text = (ROOT / "config" / "collision_stop.yaml").read_text()
    stop = float(re.search(r"stop_distance_m:\s*([\d.]+)", text).group(1))
    front = float(re.search(r"footprint_front_m:\s*([\d.]+)", text).group(1))
    assert math.isclose(VIEWPOINT_STANDOFF_M, stop + front, abs_tol=1e-9), (
        f"VIEWPOINT_STANDOFF_M {VIEWPOINT_STANDOFF_M} != deployed "
        f"stop_distance_m {stop} + footprint_front_m {front}")


def test_a_wall_adjacent_point_fails_and_open_floor_clears():
    w = h = 60
    occ = grid(w, h)
    wall(occ, w, [(x, y) for x in (0, 1) for y in range(h)])
    # 0.15 m from the wall: inside the envelope, the exact pose cert 2 parked at
    assert not point_clears_standoff(occ, w, h, 3, 30, RES, VIEWPOINT_STANDOFF_M)
    # 0.55 m out: the supervisor permits standing here
    assert point_clears_standoff(occ, w, h, 12, 30, RES, VIEWPOINT_STANDOFF_M)


def test_a_wall_adjacent_cluster_keeps_a_viewpoint_away_from_the_wall():
    """The cert-2 tail case: wall-adjacent target cells must STAY candidates —
    covering them only requires standing within coverage_radius, off the wall."""
    w = h = 60
    occ = grid(w, h)
    wall(occ, w, [(x, y) for x in (0, 1) for y in range(h)])
    cluster = [(x, y) for x in (3, 4, 5) for y in (28, 29, 30)]
    covered = {(x, y) for y in range(h) for x in range(w)} - set(cluster)
    cfg = CoverageConfig(free_threshold=0)
    selection = candidate_goals(occ, w, h, 0.0, 0.0, RES, 30, 30, covered, set(),
                                cfg, viewpoint_standoff_m=VIEWPOINT_STANDOFF_M)
    assert selection.excluded_no_viewpoint == 0
    assert len(selection.candidates) == 1, (
        "the wall-adjacent cluster fell out of candidates -- the clamp became "
        "the old goal-clearance filter, the exact regression the pins forbid")
    assert cluster_has_viewpoint(occ, w, h, cluster, RES,
                                 cfg.coverage_radius_m, VIEWPOINT_STANDOFF_M, 0)


def test_a_sub_standoff_pocket_is_excluded_and_counted():
    """A 0.15 m-wide dead-end lane deep inside a wall block THICKER than
    coverage_radius: every free cell inside the lane sits within the envelope,
    and (unlike a thin wall, where through-wall stamping legitimately covers
    from the far side) no free floor outside the block is within coverage
    range. No pose the safety stack permits can cover it -- excluded, COUNTED
    (pin 2)."""
    w = h = 60
    occ = grid(w, h)
    # solid block x=0..30, y=10..50; a 3-cell (0.15 m) lane carved at y=29..31
    # from the block's right edge to a dead end at x=1
    wall(occ, w, [(x, y) for x in range(0, 31) for y in range(10, 51)
                  if not (1 <= x <= 30 and 29 <= y <= 31)])
    target = [(x, y) for x in (2, 3, 4) for y in (29, 30, 31)]
    covered = {(x, y) for y in range(h) for x in range(w)} - set(target)
    cfg = CoverageConfig(free_threshold=0)
    selection = candidate_goals(occ, w, h, 0.0, 0.0, RES, 45, 30, covered, set(),
                                cfg, viewpoint_standoff_m=VIEWPOINT_STANDOFF_M)
    assert selection.candidates == []
    assert selection.excluded_no_viewpoint == 1, (
        "the unviewable pocket was not counted -- a COMPLETE could silently "
        "absorb it (measure-the-right-population, wearing a medal)")


def test_open_floor_goals_are_untouched_by_the_clamp():
    """Pin 3's no-regression case: the easy 90% of goals cert 2 already drove
    must select identically with the clamp on and off."""
    w = h = 60
    occ = grid(w, h)
    cluster = [(x, y) for x in (40, 41, 42) for y in (40, 41, 42)]
    covered = {(x, y) for y in range(h) for x in range(w)} - set(cluster)
    cfg = CoverageConfig(free_threshold=0)
    without = candidate_goals(occ, w, h, 0.0, 0.0, RES, 10, 10, covered, set(), cfg)
    with_clamp = candidate_goals(occ, w, h, 0.0, 0.0, RES, 10, 10, covered, set(),
                                 cfg, viewpoint_standoff_m=VIEWPOINT_STANDOFF_M)
    assert without.candidates == with_clamp.candidates
    assert with_clamp.excluded_no_viewpoint == 0


def test_the_report_carries_the_count_without_fabricating_zero():
    from sphero_rvr_core.mission_report import OUTCOME_COMPLETE, build_report

    unset = build_report(OUTCOME_COMPLETE, covered_cells=1, resolution=RES,
                         duration_s=1.0)
    assert unset["cells_excluded_no_viewpoint"] is None, (
        "an uncounted exclusion must publish as UNKNOWN, never a fabricated 0 "
        "-- remaining_candidates' own 2026-08-16 lesson")
    counted = build_report(OUTCOME_COMPLETE, covered_cells=1, resolution=RES,
                           duration_s=1.0, cells_excluded_no_viewpoint=2)
    assert counted["cells_excluded_no_viewpoint"] == 2


def test_the_ladder_squeeze_still_yields_a_goal_via_the_proven_viewpoint():
    """THE CERT-3 MUST-FLIP (second campaign, 2026-08-19): a candidate whose
    ENTIRE approach ladder sits inside the safety envelope must still carry a
    goal — the selection's proven viewpoint. On 5c7f330 CandidateSelection had
    no viewpoints field and the node discarded the proof after the exclusion
    check: the single-cluster selection starved at 12% coverage with the
    planner never asked.

    Geometry reproduces the squeeze: a long wall slab, the target cluster 0.10 m
    beneath it, the robot far along the SAME wall line — so the ladder poses
    (target, 0.375 m back, 0.675 m back, all on the robot-ward line) every one
    hug the slab and fail the standoff, while open floor south of the cluster
    holds a legal viewpoint within coverage radius."""
    w = h = 70
    occ = grid(w, h)
    wall(occ, w, [(x, y) for x in range(20, 56) for y in (30, 31, 32)])
    cluster = [(x, y) for x in (25, 26, 27) for y in (27, 28)]
    covered = {(x, y) for y in range(h) for x in range(w)} - set(cluster)
    cfg = CoverageConfig(free_threshold=0)
    robot = (58, 28)
    selection = candidate_goals(occ, w, h, 0.0, 0.0, RES, robot[0], robot[1],
                                covered, set(), cfg,
                                viewpoint_standoff_m=VIEWPOINT_STANDOFF_M)
    assert len(selection.candidates) == 1
    rep = selection.candidates[0]
    # the whole robot-ward ladder fails the envelope: the squeeze is real
    import math as _math
    rx, ry = robot
    d = _math.hypot(rep[0] - rx, rep[1] - ry)
    ux, uy = (rx - rep[0]) / d, (ry - rep[1]) / d
    ladder = [rep] + [
        (rep[0] + ux * (0.75 * frac / RES), rep[1] + uy * (0.75 * frac / RES))
        for frac in (0.5, 0.9)
    ]
    for px, py in ladder:
        assert not point_clears_standoff(occ, w, h, int(px), int(py), RES,
                                         VIEWPOINT_STANDOFF_M), (
            f"ladder pose ({px:.0f},{py:.0f}) clears -- the squeeze geometry "
            f"is wrong and this test proves nothing")
    # ...and the selection still carries the way out
    vp = selection.viewpoints[rep]
    assert point_clears_standoff(occ, w, h, vp[0], vp[1], RES,
                                 VIEWPOINT_STANDOFF_M)
    assert occ[vp[1] * w + vp[0]] == 0, "the viewpoint must be free floor"


def test_open_floor_viewpoints_ride_along_without_changing_selection():
    """No-regression, extended for the fallback: candidates identical with and
    without the clamp (as before), and the viewpoint dict is a benign passenger
    for open-floor clusters (the proof is simply one of the cluster's own
    cells or a neighbor -- present, legal, unused unless the ladder starves)."""
    w = h = 60
    occ = grid(w, h)
    cluster = [(x, y) for x in (40, 41, 42) for y in (40, 41, 42)]
    covered = {(x, y) for y in range(h) for x in range(w)} - set(cluster)
    cfg = CoverageConfig(free_threshold=0)
    without = candidate_goals(occ, w, h, 0.0, 0.0, RES, 10, 10, covered, set(), cfg)
    with_clamp = candidate_goals(occ, w, h, 0.0, 0.0, RES, 10, 10, covered, set(),
                                 cfg, viewpoint_standoff_m=VIEWPOINT_STANDOFF_M)
    assert without.candidates == with_clamp.candidates
    rep = with_clamp.candidates[0]
    vp = with_clamp.viewpoints[rep]
    assert point_clears_standoff(occ, w, h, vp[0], vp[1], RES,
                                 VIEWPOINT_STANDOFF_M)


def test_the_node_falls_back_to_the_viewpoint_and_the_planner_still_gates_it():
    """Source guard: the fallback consults selection.viewpoints ONLY after the
    ladder found nothing, and the planner gates the viewpoint pose exactly as
    it gates every pose. The revert-proof's two-gate loop shape is untouched
    (verified by that test passing unamended, ratified pin 3)."""
    from pathlib import Path

    node_src = (Path(__file__).resolve().parents[1] / "src" /
                "sphero_rvr_driver" / "coverage_explorer_node.py").read_text()
    assert "selection.viewpoints.get(cell)" in node_src
    idx = node_src.index("selection.viewpoints.get(cell)")
    assert "if goal_cell is None:" in node_src[idx - 1500:idx], (
        "the fallback no longer waits for the ladder to fail first")
    tail = node_src[idx:idx + 600]
    assert "_planner_can_reach(vwx, vwy, frame)" in tail, (
        "the viewpoint pose skips the planner -- reachability lost its gate")


def test_planner_rejections_means_the_planner_said_no():
    """Cert 3's conflation, closed at the source: the only increment of
    _planner_rejections lives inside _planner_can_reach on a genuine empty
    answer, and the old candidates-without-goal accumulation is gone."""
    from pathlib import Path

    node_src = (Path(__file__).resolve().parents[1] / "src" /
                "sphero_rvr_driver" / "coverage_explorer_node.py").read_text()
    assert node_src.count("self._planner_rejections +=") == 1, (
        "planner_rejections has more than one incrementer again -- the "
        "conflation that wrote 24 refusals into a report whose planner log "
        "shows zero")
    assert "self._planner_rejections += self._unplannable_last_cycle" \
        not in node_src
    assert '"standoff_skips"' in node_src, (
        "the envelope's own counter left the status payload")
