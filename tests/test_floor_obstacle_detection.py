"""Tests for the monocular floor-boundary obstacle detector."""

import numpy as np

from sphero_rvr_core.floor_obstacle_detection import detect_floor_boundary, floor_reference


def test_all_floor_finds_no_obstacle():
    img = np.full((20, 10, 3), 100, np.uint8)
    assert all(r is None for r in detect_floor_boundary(img))


def test_floor_reference_is_bottom_band_color():
    img = np.zeros((20, 10, 3), np.uint8)
    img[:10] = 200  # top
    img[10:] = 50   # bottom (floor)
    ref = floor_reference(img)
    assert np.allclose(ref, [50, 50, 50])


def test_obstacle_above_floor_gives_contact_row():
    img = np.zeros((20, 10, 3), np.uint8)
    img[:10] = 200   # top half = obstacle
    img[10:] = 50    # bottom half = floor
    res = detect_floor_boundary(img, min_run=4)
    # lowest non-floor pixel in each column is row 9 (the obstacle's floor contact)
    assert all(r == 9 for r in res)


def test_obstacle_bar_only_in_some_columns():
    img = np.full((20, 10, 3), 50, np.uint8)  # all floor
    img[5:12, 3:7] = 200  # obstacle bar in columns 3..6, rows 5..11
    res = detect_floor_boundary(img, min_run=4)
    assert res[3] == 11 and res[6] == 11  # contact at the bar's bottom
    assert res[0] is None and res[9] is None  # clear columns


def test_noise_below_min_run_is_ignored():
    img = np.full((20, 10, 3), 50, np.uint8)
    img[8, 5] = 200  # a single stray non-floor pixel
    res = detect_floor_boundary(img, min_run=4)
    assert res[5] is None  # one pixel < min_run -> not an obstacle
