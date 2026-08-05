"""Tests for the monocular floor-boundary obstacle detector."""

import numpy as np

from sphero_rvr_core.floor_obstacle_detection import (
    adaptive_threshold,
    detect_floor_boundary,
    floor_reference,
)


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


def test_adaptive_threshold_within_bounds_and_scales_with_spread():
    # A near-uniform floor -> low percentile distance -> clamped to lower bound.
    calm = np.zeros((100, 50), np.float32)
    assert adaptive_threshold(calm, 0.12, bounds=(25.0, 60.0)) == 25.0
    # A noisy band -> large percentile distance -> clamped to upper bound.
    noisy = np.zeros((100, 50), np.float32)
    noisy[88:, :] = 100.0  # the floor band is all far-from-ref
    assert adaptive_threshold(noisy, 0.12, bounds=(25.0, 60.0)) == 60.0


def test_adaptive_color_thresh_none_still_detects_obstacle():
    img = np.zeros((20, 10, 3), np.uint8)
    img[:10] = 200  # obstacle
    img[10:] = 50   # floor (uniform -> adaptive thr hits lower bound, well below 150)
    res = detect_floor_boundary(img, color_thresh=None, min_run=4)
    assert all(r == 9 for r in res)


def test_min_rise_tolerates_specks_in_obstacle():
    # Obstacle rows 5..11 with a floor-coloured speck; strict min_run would still
    # find the contact, min_rise fractional window also does.
    img = np.full((20, 10, 3), 50, np.uint8)
    img[5:12, 3:7] = 200
    img[8, 4] = 50  # a floor-coloured speck inside the obstacle
    res = detect_floor_boundary(img, min_rise=6, rise_frac=0.6)
    assert res[4] == 11  # contact still at the obstacle base despite the speck
