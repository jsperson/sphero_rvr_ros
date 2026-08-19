"""Tests for the coverage + frontier exploration core."""

from sphero_rvr_core.coverage_exploration import (
    CoverageConfig,
    candidate_goals,
    cell_center_world,
    is_frontier,
    stamp_coverage,
    world_grid,
)


def candidate_goals_list(*args, **kwargs):
    """Return-shape shim (2026-08-19 viewpoint-standoff batch): candidate_goals
    now returns CandidateSelection(candidates, excluded_no_viewpoint); these
    tests assert on the candidate list."""
    return candidate_goals(*args, **kwargs).candidates


def first(*args, **kwargs):
    """The goal the node would try first — candidate_goals is nearest-first."""
    cells = candidate_goals(*args, **kwargs).candidates
    return cells[0] if cells else None


def build(rows):
    """'.'=free(0) '#'=occupied(100) '?'=unknown(-1). Returns (occ, w, h)."""
    h = len(rows)
    w = len(rows[0])
    occ = []
    for r in rows:
        for c in r:
            occ.append(0 if c == "." else (100 if c == "#" else -1))
    return occ, w, h


def cover_all(w, h, origin=(0.0, 0.0), res=1.0):
    covered = set()
    for cy in range(h):
        for cx in range(w):
            wx, wy = cell_center_world(cx, cy, origin[0], origin[1], res)
            covered.add(world_grid(wx, wy, res))
    return covered


def cover_cells(cells, origin=(0.0, 0.0), res=1.0):
    covered = set()
    for cx, cy in cells:
        wx, wy = cell_center_world(cx, cy, origin[0], origin[1], res)
        covered.add(world_grid(wx, wy, res))
    return covered


CFG = CoverageConfig(min_cluster_cells=1)


# --- coverage stamping ---


def test_stamp_covers_near_and_not_far():
    covered = set()
    stamp_coverage(covered, 0.0, 0.0, res=0.5, radius_m=0.75)
    assert (0, 0) in covered  # cell center (0.25,0.25), ~0.35 m away
    assert (2, 2) not in covered  # cell center (1.25,1.25), ~1.77 m away


# --- frontier detection ---


def test_is_frontier():
    occ, w, h = build(["..?"])  # free, free, unknown
    assert is_frontier(occ, w, h, 1, 0)  # free cell next to unknown
    assert not is_frontier(occ, w, h, 0, 0)  # free, only free neighbor
    assert not is_frontier(occ, w, h, 2, 0)  # the unknown cell itself


# --- goal selection: frontier ---


def test_selects_frontier_when_all_covered():
    occ, w, h = build([".....?"])  # frontier at cell (4,0)
    covered = cover_all(w, h)  # nothing uncovered -> only a frontier can be a target
    goal = first(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), CFG)
    assert goal == (4, 0)


# --- goal selection: coverage (nearest uncovered) ---


def test_selects_nearest_uncovered():
    occ, w, h = build(["......"])  # all free, no unknown
    covered = cover_cells([(0, 0), (1, 0)])  # only near the robot
    cfg = CoverageConfig(min_cluster_cells=4, include_frontiers=True)
    goal = first(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert goal == (2, 0)  # nearest uncovered, cluster (2..5) has 4 cells


# --- mission complete ---


def test_none_when_all_covered_and_no_frontier():
    occ, w, h = build(["......"])
    covered = cover_all(w, h)
    goal = first(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), CFG)
    assert goal is None


# --- reachability: walled-off free space is never chosen ---


def test_unreachable_free_not_selected():
    occ, w, h = build([".#..."])  # robot region | wall | free free free (uncovered)
    covered = cover_cells([(0, 0)])
    goal = first(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), CFG)
    assert goal is None  # (2,0)+ are uncovered but walled off from the robot


# --- min cluster filters noise ---


def test_min_cluster_skips_lone_speck():
    occ, w, h = build(["......"])
    covered = cover_all(w, h) - {(3, 0)}  # exactly one uncovered cell
    cfg = CoverageConfig(min_cluster_cells=5)
    goal = first(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert goal is None  # lone speck (cluster size 1) skipped


# --- suppression (goals that planned but would not drive) ---


def test_suppressed_cells_are_not_proposed():
    occ, w, h = build(["......"])
    covered = cover_cells([(0, 0), (1, 0)])
    # Suppress the whole uncovered run -> nothing left to pursue.
    goal = first(
        occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, {(2, 0), (3, 0), (4, 0), (5, 0)}, CFG
    )
    assert goal is None


# --- frontiers disabled: cover only ---


def test_include_frontiers_false_ignores_frontier():
    occ, w, h = build([".....?"])
    covered = cover_all(w, h)
    cfg = CoverageConfig(min_cluster_cells=1, include_frontiers=False)
    goal = first(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert goal is None  # frontier present but not pursued; nothing uncovered


def test_prefers_coverage_and_frontier_together():
    # Uncovered near cells AND a frontier far right; nearest target wins.
    occ, w, h = build(["......?"])
    covered = cover_cells([(0, 0)])
    cfg = CoverageConfig(min_cluster_cells=1, include_frontiers=True)
    goal = first(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert goal == (1, 0)  # nearest uncovered comes before the far frontier


# --- reachability is the PLANNER's call, not this module's ---


def test_narrow_passage_is_still_proposed():
    """A passage too narrow for the robot IS proposed, deliberately.

    This module used to erode the map by an inscribed radius and refuse to look
    through gaps narrower than the robot. That estimate disagreed with the real
    costmap in both directions -- it rejected passages the planner would happily
    take, and still let through targets the planner refused -- which is what the
    blacklist, its TTL and its radius all existed to contain. The node now asks
    ComputePathToPose about each proposal, so proposing an impassable gap costs
    exactly one planner query and no memory.
    """
    cfg = CoverageConfig(min_cluster_cells=1, include_frontiers=False)
    covered_left = cover_cells([(x, y) for y in range(5) for x in range(7) if x < 4])
    narrow, w, h = build(["...#...", "...#...", ".......", "...#...", "...#..."])
    goal = first(narrow, w, h, 0.0, 0.0, 1.0, 1, 2, covered_left, set(), cfg)
    assert goal is not None and goal[0] >= 4  # offered; the planner decides


def test_walled_off_space_is_still_excluded():
    """Connectivity is a fact the map states, so it stays here: a region with no
    free path at all is never offered, and never costs a planner query."""
    cfg = CoverageConfig(min_cluster_cells=1, include_frontiers=False)
    covered_left = cover_cells([(x, y) for y in range(5) for x in range(7) if x < 4])
    sealed, w, h = build(["...#...", "...#...", "...#...", "...#...", "...#..."])
    assert first(sealed, w, h, 0.0, 0.0, 1.0, 1, 2, covered_left, set(), cfg) is None


# --- candidate list: ordering, one per cluster, bounded ---


def test_candidates_are_nearest_first_and_one_per_cluster():
    # Two uncovered clusters either side of the robot at (5,0): near (3..4) and
    # far (8..9). Expect one representative each, nearest first.
    occ, w, h = build(["............"])
    covered = cover_all(w, h) - {(3, 0), (4, 0), (8, 0), (9, 0)}
    cfg = CoverageConfig(min_cluster_cells=2, include_frontiers=False)
    cells = candidate_goals_list(occ, w, h, 0.0, 0.0, 1.0, 5, 0, covered, set(), cfg)
    assert len(cells) == 2
    assert cells[0] == (4, 0)  # nearest cell of the near cluster
    assert cells[1] == (8, 0)  # nearest cell of the far cluster


def test_cluster_too_close_offers_a_farther_cell_instead():
    # D14: the flood reaches a cluster at its NEAREST cell, but a cell inside
    # min_offer_distance yields no goal at the caller -- offering it silences the
    # whole cluster. The representative must move outward to the nearest usable
    # cell, not vanish.
    occ, w, h = build(["........."])
    covered = cover_all(w, h) - {(1, 0), (2, 0), (3, 0), (4, 0), (5, 0)}
    # res=1.0, robot at (0,0): cluster spans distances 1..5.
    cfg = CoverageConfig(min_cluster_cells=2, include_frontiers=False,
                         min_offer_distance_m=1.5)
    cells = candidate_goals_list(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    # min distance 1.5 m + one cell of live-pose margin = 2.5 cells -> (3,0).
    assert cells == [(3, 0)]


def test_cluster_entirely_too_close_is_skipped_for_this_pose():
    # A cluster with no cell at usable distance is unofferable FROM HERE -- it must
    # yield nothing rather than a goal the caller will refuse forever. Distances
    # are re-measured from the live pose each cycle, so moving re-offers it.
    occ, w, h = build(["...."])
    covered = cover_all(w, h) - {(1, 0), (2, 0)}
    cfg = CoverageConfig(min_cluster_cells=2, include_frontiers=False,
                         min_offer_distance_m=3.0)
    cells = candidate_goals_list(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert cells == []


def test_min_offer_distance_zero_keeps_the_nearest_cell():
    # Default config: the representative stays the flood's first-touched (nearest)
    # cell, exactly as before D14.
    occ, w, h = build(["......"])
    covered = cover_all(w, h) - {(2, 0), (3, 0)}
    cfg = CoverageConfig(min_cluster_cells=2, include_frontiers=False)
    cells = candidate_goals_list(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert cells == [(2, 0)]


def test_max_candidates_bounds_the_planner_queries():
    # Many separate single-cell clusters; the cap is what stops one selection
    # cycle from firing an unbounded number of ComputePathToPose queries.
    occ, w, h = build(["." * 40])
    covered = cover_all(w, h) - {(i, 0) for i in range(0, 40, 2)}
    cfg = CoverageConfig(min_cluster_cells=1, include_frontiers=False, max_candidates=3)
    cells = candidate_goals_list(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert len(cells) == 3


# --- start-pose blocked check (the 2026-08-07 invalid-run guard) ---

def _flat_costmap(w, h, fill=0):
    return [fill] * (w * h)


def test_start_blocked_when_robot_cell_is_lethal():
    from sphero_rvr_core.coverage_exploration import robot_start_blocked
    data = _flat_costmap(10, 10)
    data[5 * 10 + 5] = 100
    assert robot_start_blocked(data, 10, 10, 0.0, 0.0, 0.1, 0.55, 0.55) is True


def test_start_blocked_at_inscribed_ring():
    from sphero_rvr_core.coverage_exploration import robot_start_blocked
    data = _flat_costmap(10, 10)
    data[5 * 10 + 5] = 99  # inscribed: footprint is in collision
    assert robot_start_blocked(data, 10, 10, 0.0, 0.0, 0.1, 0.55, 0.55) is True


def test_start_clear_on_free_cell():
    from sphero_rvr_core.coverage_exploration import robot_start_blocked
    data = _flat_costmap(10, 10)
    assert robot_start_blocked(data, 10, 10, 0.0, 0.0, 0.1, 0.55, 0.55) is False


def test_start_mild_inflation_is_not_blocked():
    from sphero_rvr_core.coverage_exploration import robot_start_blocked
    data = _flat_costmap(10, 10)
    data[5 * 10 + 5] = 60  # inflated but the footprint still fits
    assert robot_start_blocked(data, 10, 10, 0.0, 0.0, 0.1, 0.55, 0.55) is False


def test_start_unknown_cell_is_inconclusive():
    from sphero_rvr_core.coverage_exploration import robot_start_blocked
    data = _flat_costmap(10, 10, -1)
    assert robot_start_blocked(data, 10, 10, 0.0, 0.0, 0.1, 0.55, 0.55) is None


def test_start_outside_costmap_is_inconclusive():
    from sphero_rvr_core.coverage_exploration import robot_start_blocked
    data = _flat_costmap(10, 10)
    assert robot_start_blocked(data, 10, 10, 0.0, 0.0, 0.1, 99.0, 99.0) is None


def test_start_blocked_handles_degenerate_costmap():
    from sphero_rvr_core.coverage_exploration import robot_start_blocked
    assert robot_start_blocked([], 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0) is None
