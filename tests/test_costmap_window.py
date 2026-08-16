"""The D43 dump: the one measurement that can separate two indistinguishable causes.

`START POSE BLOCKED` fired on 2026-08-15 over floor that measured OPEN on all twelve
bearings at a minimum of 0.410 m, with no freeze marks in range. Two mechanisms
explain that identically from outside -- a stale SLAM static layer, or a map-frame
pose offset -- and neither can be separated from a bag that never recorded the grid.

These tests assert the dump can actually tell them apart, because a dump that cannot
is instrumentation that will produce another inconclusive autopsy.
"""

import pytest

from sphero_rvr_core.costmap_window import (
    INSCRIBED_COST,
    extract_window,
    format_window,
)
from sphero_rvr_core.coverage_exploration import robot_start_blocked

RES = 0.05
W = H = 40
ORIGIN_X = ORIGIN_Y = -1.0


def _grid(fill=0):
    return [fill] * (W * H)


def _set(data, cx, cy, value):
    data[cy * W + cx] = value


def _cell_of(world_x, world_y):
    return int((world_x - ORIGIN_X) / RES), int((world_y - ORIGIN_Y) / RES)


def test_the_dump_indexes_the_costmap_the_SAME_WAY_the_check_that_fired_does():
    """PREMISE TRIPWIRE, and the most important test here.

    The dump exists to show the cell the planner refused. If it indexed differently
    from `robot_start_blocked` -- the function that produced the refusal -- it would
    faithfully display a DIFFERENT cell, and every conclusion drawn from it would be
    about the wrong square. That failure would be invisible: the picture would look
    perfectly plausible.

    So this asserts agreement against production rather than restating the indexing.
    """
    data = _grid()
    for wx, wy in ((0.0, 0.0), (-0.42, 0.31), (0.55, -0.60), (-0.95, 0.95)):
        cx, cy = _cell_of(wx, wy)
        _set(data, cx, cy, INSCRIBED_COST)

        window = extract_window(data, W, H, ORIGIN_X, ORIGIN_Y, RES, wx, wy, 0.20)
        blocked = robot_start_blocked(data, W, H, ORIGIN_X, ORIGIN_Y, RES, wx, wy)

        assert window is not None
        assert window.centre_is_blocked == blocked, (
            f"at ({wx},{wy}) the dump says centre_blocked={window.centre_is_blocked} "
            f"while the check that fires says {blocked} -- they are reading different "
            f"cells, and the dump would picture the wrong square"
        )
        _set(data, cx, cy, 0)


def test_stale_occupancy_and_a_pose_offset_are_DISTINGUISHABLE_in_the_picture():
    """The whole justification for the dump, as an assertion rather than a hope.

    STALE OCCUPANCY: a lethal patch sitting exactly where the robot is, with clear
    cells all around it -- the floor is open and the map disagrees.
    POSE OFFSET: the robot's marker landing on the edge of real structure, with the
    lethal cells forming a wall that continues away from it.

    If these two produced the same picture, the dump would settle nothing.
    """
    wx, wy = 0.0, 0.0
    cx, cy = _cell_of(wx, wy)

    stale = _grid()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            _set(stale, cx + dx, cy + dy, INSCRIBED_COST)

    offset = _grid()
    for gy in range(H):                      # a wall three cells to the right
        _set(offset, cx + 3, gy, INSCRIBED_COST)
    _set(offset, cx, cy, INSCRIBED_COST)     # and the robot's own cell caught on it

    a = format_window(extract_window(stale, W, H, ORIGIN_X, ORIGIN_Y, RES, wx, wy, 0.30),
                      wx, wy)
    b = format_window(extract_window(offset, W, H, ORIGIN_X, ORIGIN_Y, RES, wx, wy, 0.30),
                      wx, wy)

    assert a != b, "the two mechanisms must not draw the same picture"
    # The blob is bounded; the wall runs the full height of the window.
    a_rows = [r for r in a.splitlines() if r.startswith("  ")]
    b_rows = [r for r in b.splitlines() if r.startswith("  ")]
    assert sum("#" in r for r in a_rows) <= 3, "stale occupancy should be a small patch"
    assert sum("#" in r for r in b_rows) == len(b_rows), (
        "a pose offset against real structure should show that structure spanning "
        "the window, which is what separates it from a floating blob"
    )


def test_off_map_cells_are_blank_and_NOT_free_floor():
    """Padding with zeros would draw open floor around a robot at the map's edge --
    inventing the very evidence the dump exists to collect."""
    wx, wy = ORIGIN_X + RES, ORIGIN_Y + RES        # hard against the corner
    window = extract_window(_grid(), W, H, ORIGIN_X, ORIGIN_Y, RES, wx, wy, 0.25)
    text = format_window(window, wx, wy)
    rows = [r for r in text.splitlines() if r.startswith("  ")]
    assert any(" " in r.strip("  ") or r.rstrip() != r for r in rows) or any(
        "  " in r[2:] for r in rows), "off-map cells must render blank, not as '.'"
    assert any(cell is None for row in window.cells for cell in row)


def test_unknown_is_its_own_character_not_free():
    """-1 means nobody has looked. Drawing it as free is how a costmap dump grows a
    room the robot has never seen."""
    data = _grid()
    cx, cy = _cell_of(0.0, 0.0)
    _set(data, cx + 1, cy, -1)
    text = format_window(extract_window(data, W, H, ORIGIN_X, ORIGIN_Y, RES, 0.0, 0.0, 0.15),
                         0.0, 0.0)
    assert "?" in text


def test_the_legend_travels_with_the_dump():
    """A dump whose legend lives in a source file is a dump misread out of an archive
    six months later."""
    text = format_window(extract_window(_grid(), W, H, ORIGIN_X, ORIGIN_Y, RES, 0.0, 0.0, 0.1),
                         0.0, 0.0)
    assert "legend:" in text
    for token in ("COSTMAP_DUMP", "centre_blocked=", f"inscribed={INSCRIBED_COST}"):
        assert token in text


def test_the_robot_cell_is_drawn_over_whatever_it_holds():
    data = _grid()
    cx, cy = _cell_of(0.0, 0.0)
    _set(data, cx, cy, INSCRIBED_COST)
    window = extract_window(data, W, H, ORIGIN_X, ORIGIN_Y, RES, 0.0, 0.0, 0.10)
    text = format_window(window, 0.0, 0.0)
    rows = [r[2:] for r in text.splitlines() if r.startswith("  ")]
    mid = window.radius_cells
    assert rows[mid][mid] == "R"
    assert window.centre_is_blocked is True, (
        "the marker must not hide the value -- the header still reports it"
    )


def test_north_is_up():
    """A dump printed upside-down is how a reader concludes the obstacle is behind
    the robot when it is in front. This project has already made a mirrored-convention
    error in both the control path and the analysis layer."""
    data = _grid()
    cx, cy = _cell_of(0.0, 0.0)
    _set(data, cx, cy + 2, INSCRIBED_COST)          # +y, i.e. ahead/up
    window = extract_window(data, W, H, ORIGIN_X, ORIGIN_Y, RES, 0.0, 0.0, 0.15)
    rows = [r[2:] for r in format_window(window, 0.0, 0.0).splitlines()
            if r.startswith("  ")]
    mid = window.radius_cells
    assert rows[mid - 2][mid] == "#", "a cell at +y must print ABOVE the robot"
    assert rows[mid + 2][mid] != "#"


def test_a_degenerate_grid_is_refused_rather_than_drawn():
    for bad in (
        dict(resolution=0.0), dict(width=0), dict(height=0), dict(radius_m=-1.0),
    ):
        kwargs = dict(width=W, height=H, resolution=RES, radius_m=0.2)
        kwargs.update(bad)
        assert extract_window(
            _grid(), kwargs["width"], kwargs["height"], ORIGIN_X, ORIGIN_Y,
            kwargs["resolution"], 0.0, 0.0, kwargs["radius_m"]) is None
    assert "unavailable" in format_window(None, 0.0, 0.0)


def test_a_robot_outside_the_map_reports_unknown_not_clear():
    window = extract_window(_grid(), W, H, ORIGIN_X, ORIGIN_Y, RES, 99.0, 99.0, 0.2)
    assert window is not None
    assert window.centre_value is None
    assert window.centre_is_blocked is None, (
        "off-map must be 'cannot tell', never 'not blocked'"
    )


def test_it_is_pure_and_needs_no_ros():
    import sphero_rvr_core.costmap_window as mod

    assert "rclpy" not in getattr(mod, "__dict__", {})
