"""The simulated lidar's geometry, tested where a mirrored world would be caught.

A simulator that flips the map produces a plausible-looking scan of the wrong room, and
every downstream result is confident nonsense. That is the N1 class of defect, and it is
why these tests care about handedness and origin as much as about distances.
"""

import math

import numpy as np
import pytest

from sphero_rvr_core.occupancy_raycast import (
    OccupancyMap,
    load_map,
    parse_map_yaml,
    parse_pgm,
    raycast,
)


def _grid(blocked_cells, width=20, height=20, resolution=0.05, origin=(0.0, 0.0)):
    blocked = np.zeros((height, width), dtype=bool)
    for row, col in blocked_cells:
        blocked[row, col] = True
    return OccupancyMap(blocked, resolution, origin[0], origin[1])


def test_an_empty_grid_returns_inf_everywhere():
    grid = _grid([])
    ranges = raycast(grid, 0.5, 0.5, 0.0, np.array([0.0, 1.0, -1.0]), max_range=0.4)
    assert np.all(np.isinf(ranges))


def test_a_wall_straight_ahead_is_measured_at_its_distance():
    # Cell (row 10, col 14) centre is at x = 14*0.05 = 0.70, y = 10*0.05 = 0.50.
    grid = _grid([(10, 14)])
    ranges = raycast(grid, 0.50, 0.50, 0.0, np.array([0.0]), max_range=1.0)
    assert ranges[0] == pytest.approx(0.20, abs=0.03)


def test_the_beam_angle_is_measured_from_the_robot_heading():
    grid = _grid([(10, 14)])
    # Facing +y, the same obstacle must appear 90 degrees to the RIGHT, not ahead.
    ahead = raycast(grid, 0.50, 0.50, math.pi / 2, np.array([0.0]), max_range=1.0)
    right = raycast(grid, 0.50, 0.50, math.pi / 2, np.array([-math.pi / 2]), max_range=1.0)
    assert np.isinf(ahead[0])
    assert right[0] == pytest.approx(0.20, abs=0.03)


def test_handedness_is_not_mirrored():
    # An obstacle at +y must be seen to the LEFT (positive angle) of a robot facing +x.
    grid = _grid([(14, 10)])  # x = 0.50, y = 0.70
    left = raycast(grid, 0.50, 0.50, 0.0, np.array([math.pi / 2]), max_range=1.0)
    right = raycast(grid, 0.50, 0.50, 0.0, np.array([-math.pi / 2]), max_range=1.0)
    assert left[0] == pytest.approx(0.20, abs=0.03)
    assert np.isinf(right[0]), "the world is mirrored -- a simulated room of the wrong hand"


def test_the_map_origin_offsets_the_world():
    grid = _grid([(10, 14)], origin=(-1.0, -1.0))
    # Same cell, but the grid now starts at (-1,-1), so the obstacle is at (-0.30,-0.50).
    ranges = raycast(grid, -0.50, -0.50, 0.0, np.array([0.0]), max_range=1.0)
    assert ranges[0] == pytest.approx(0.20, abs=0.03)


def test_range_is_capped_and_reports_inf_beyond_it():
    grid = _grid([(10, 19)])
    assert np.isinf(raycast(grid, 0.50, 0.50, 0.0, np.array([0.0]), max_range=0.20)[0])
    assert np.isfinite(raycast(grid, 0.50, 0.50, 0.0, np.array([0.0]), max_range=1.0)[0])


def test_a_beam_leaving_the_grid_does_not_wrap_around():
    grid = _grid([(0, 0)])
    ranges = raycast(grid, 0.50, 0.50, 0.0, np.array([0.0]), max_range=2.0)
    assert np.isinf(ranges[0]), "a beam that exits the map must not find the far edge"


# --- map loading --------------------------------------------------------------------


def _pgm(width, height, pixels):
    header = f"P5\n{width} {height}\n255\n".encode()
    return header + bytes(pixels)


YAML = """image: test.pgm
mode: trinary
resolution: 0.05
origin: [-1.0, -2.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""


def test_map_yaml_fields_are_read():
    meta = parse_map_yaml(YAML)
    assert meta["resolution"] == 0.05
    assert meta["origin"] == [-1.0, -2.0, 0.0]
    assert meta["occupied_thresh"] == 0.65


def test_pgm_parses_dimensions_and_pixels():
    image = parse_pgm(_pgm(3, 2, [0, 128, 255, 255, 128, 0]))
    assert image.shape == (2, 3)
    assert image[0, 0] == 0 and image[1, 2] == 0


def test_dark_pixels_are_obstacles_and_the_image_is_flipped_upright():
    # map_server: dark = occupied. PGM row 0 is the TOP of the image, but grid row 0 is
    # the BOTTOM (origin corner) -- getting this backwards flips the room upside down.
    grid = load_map(YAML, _pgm(2, 2, [0, 255, 255, 255]))
    assert grid.blocked[1, 0], "the dark pixel was on the TOP row, so it belongs at high y"
    assert not grid.blocked[0, 0]


def test_a_real_recorded_mission_map_loads_if_present(tmp_path):
    # Shape check only -- the real map lives on the Pi, so this stays a unit test.
    grid = load_map(YAML, _pgm(4, 4, [255] * 16))
    assert grid.resolution == 0.05
    assert (grid.origin_x, grid.origin_y) == (-1.0, -2.0)
    assert grid.blocked.shape == (4, 4)
    assert not grid.blocked.any()
