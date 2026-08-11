"""Revert-proofs for goal-pose clearance — prevention, the ladder's first line.

Run 185048 drove itself to a pose with 0.22 m on two sides and was then unable to
plan, pivot or reverse. That pose was an ordinary GOAL when it was selected. These
tests pin the filter that declines to send the rover somewhere it cannot leave.
"""

from sphero_rvr_core.coverage_exploration import INSCRIBED_COST, pose_clearance_m

W = H = 40
RES = 0.05
ORIGIN = (0.0, 0.0)


def _grid(fill=0):
    return [fill] * (W * H)


def _set(grid, cx, cy, value=INSCRIBED_COST):
    grid[cy * W + cx] = value


def _clear_at(grid, wx, wy, **kw):
    return pose_clearance_m(grid, W, H, ORIGIN[0], ORIGIN[1], RES, wx, wy, **kw)


def test_open_pose_reports_the_probe_ceiling():
    """Nothing nearby: we deliberately stop measuring once the answer stops mattering."""
    assert _clear_at(_grid(), 1.0, 1.0) == 0.60


def test_a_wall_one_cell_away_is_measured():
    grid = _grid()
    _set(grid, 21, 20)                      # one cell east of (1.0, 1.0)
    assert abs(_clear_at(grid, 1.0, 1.0) - RES) < 1e-9


def test_run_185048_pocket_is_rejected_by_a_035_filter():
    """The real geometry: hemmed at ~0.22 m on two sides. This is the pose the rover
    chose, drove to, and could not leave."""
    grid = _grid()
    for i in range(W):                      # a wall 0.20 m to the west...
        _set(grid, 16, i)
    for i in range(W):                      # ...and one 0.20 m to the south
        _set(grid, i, 16)
    clearance = _clear_at(grid, 1.0, 1.0)
    assert clearance is not None
    assert clearance < 0.35, (
        "the 185048 pocket passed a 0.35 m clearance filter; prevention is inert")


def test_unknown_pose_is_not_reported_as_open():
    """Fail safe: 'cannot tell' must be distinguishable from 'clear', or the filter
    silently approves everything outside the mapped area."""
    grid = _grid()
    grid[20 * W + 20] = -1
    assert _clear_at(grid, 1.0, 1.0) is None


def test_off_map_pose_is_not_reported_as_open():
    assert _clear_at(_grid(), 99.0, 99.0) is None


def test_unknown_space_along_a_ray_does_not_count_as_an_obstacle():
    """Unmapped is not blocked. Counting it as an obstruction would reject every goal
    near the frontier — which is exactly where exploration needs to go."""
    grid = _grid()
    for i in range(W):
        grid[i * W + 21] = -1
    assert _clear_at(grid, 1.0, 1.0) == 0.60


def test_map_edge_is_not_an_obstacle():
    """The costmap boundary is the limit of knowledge, not a wall."""
    assert _clear_at(_grid(), 0.05, 0.05) == 0.60


def test_below_inscribed_cost_does_not_count():
    """Only genuine obstruction counts; inflation gradient is not a wall."""
    grid = _grid()
    _set(grid, 21, 20, INSCRIBED_COST - 1)
    assert _clear_at(grid, 1.0, 1.0) == 0.60
