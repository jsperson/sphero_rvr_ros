"""Tests for the coverage + frontier exploration core."""

from sphero_rvr_core.coverage_exploration import (
    CoverageConfig,
    cell_center_world,
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
