"""Tests for the coverage + frontier exploration core."""

from sphero_rvr_core.coverage_exploration import (
    CoverageConfig,
    cell_center_world,
    compute_navigable,
    is_frontier,
    select_next_goal,
    stamp_coverage,
    world_grid,
)


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
    goal = select_next_goal(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), CFG)
    assert goal == (4, 0)


# --- goal selection: coverage (nearest uncovered) ---


def test_selects_nearest_uncovered():
    occ, w, h = build(["......"])  # all free, no unknown
    covered = cover_cells([(0, 0), (1, 0)])  # only near the robot
    cfg = CoverageConfig(min_cluster_cells=4, include_frontiers=True)
    goal = select_next_goal(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert goal == (2, 0)  # nearest uncovered, cluster (2..5) has 4 cells


# --- mission complete ---


def test_none_when_all_covered_and_no_frontier():
    occ, w, h = build(["......"])
    covered = cover_all(w, h)
    goal = select_next_goal(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), CFG)
    assert goal is None


# --- reachability: walled-off free space is never chosen ---


def test_unreachable_free_not_selected():
    occ, w, h = build([".#..."])  # robot region | wall | free free free (uncovered)
    covered = cover_cells([(0, 0)])
    goal = select_next_goal(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), CFG)
    assert goal is None  # (2,0)+ are uncovered but walled off from the robot


# --- min cluster filters noise ---


def test_min_cluster_skips_lone_speck():
    occ, w, h = build(["......"])
    covered = cover_all(w, h) - {(3, 0)}  # exactly one uncovered cell
    cfg = CoverageConfig(min_cluster_cells=5)
    goal = select_next_goal(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert goal is None  # lone speck (cluster size 1) skipped


# --- blacklist ---


def test_blacklist_skips_cluster():
    occ, w, h = build(["......"])
    covered = cover_cells([(0, 0), (1, 0)])
    # Blacklist the whole uncovered run -> nothing left to pursue.
    goal = select_next_goal(
        occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, {(2, 0), (3, 0), (4, 0), (5, 0)}, CFG
    )
    assert goal is None


# --- frontiers disabled: cover only ---


def test_include_frontiers_false_ignores_frontier():
    occ, w, h = build([".....?"])
    covered = cover_all(w, h)
    cfg = CoverageConfig(min_cluster_cells=1, include_frontiers=False)
    goal = select_next_goal(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert goal is None  # frontier present but not pursued; nothing uncovered


def test_prefers_coverage_and_frontier_together():
    # Uncovered near cells AND a frontier far right; nearest target wins.
    occ, w, h = build(["......?"])
    covered = cover_cells([(0, 0)])
    cfg = CoverageConfig(min_cluster_cells=1, include_frontiers=True)
    goal = select_next_goal(occ, w, h, 0.0, 0.0, 1.0, 0, 0, covered, set(), cfg)
    assert goal == (1, 0)  # nearest uncovered comes before the far frontier


# --- navigability: don't target/route through cells too close to obstacles ---


def test_compute_navigable_erodes_near_walls():
    occ, w, h = build([".....", ".....", "..#..", ".....", "....."])
    nav = compute_navigable(occ, w, h, inscribed_cells=1, occupied_threshold=50, free_threshold=0)
    at = lambda x, y: nav[y * w + x]
    assert at(2, 2) == 0  # the obstacle cell
    assert at(1, 2) == 0 and at(3, 2) == 0 and at(2, 1) == 0 and at(2, 3) == 0  # 4-neighbors within inscribed
    assert at(0, 0) == 1 and at(2, 0) == 1  # cells >1 from the obstacle are navigable


def test_narrow_passage_blocks_but_wide_passage_allows():
    # Left region (robot) connected to an uncovered right region only via an opening
    # at x=3. Require 1-cell clearance (inscribed_radius_m 1.0 at res 1.0).
    cfg = CoverageConfig(min_cluster_cells=1, include_frontiers=False, inscribed_radius_m=1.0)
    # left + wall column covered; right region (x>=4) uncovered
    covered_left = cover_cells([(x, y) for y in range(5) for x in range(7) if x < 4])

    # Narrow: single opening cell (3,2), flanked above/below by walls -> not navigable.
    narrow, w, h = build(["...#...", "...#...", ".......", "...#...", "...#..."])
    assert select_next_goal(narrow, w, h, 0.0, 0.0, 1.0, 1, 2, covered_left, set(), cfg) is None

    # Wide: opening spans y=1..3, so (3,2) is >1 from any wall -> navigable -> right reachable.
    wide, w2, h2 = build(["...#...", ".......", ".......", ".......", "...#..."])
    goal = select_next_goal(wide, w2, h2, 0.0, 0.0, 1.0, 1, 2, covered_left, set(), cfg)
    assert goal is not None and goal[0] >= 4  # an uncovered cell in the right region


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
