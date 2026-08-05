"""Tests for the camera low-obstacle contribution to the collision brake."""

import math

from sphero_rvr_core.low_obstacle_brake import forward_speed_scale, nearest_forward_obstacle


def test_nearest_forward_picks_closest_in_cone():
    pts = [(0.8, 0.0), (0.5, 0.05), (1.2, 0.0)]
    assert nearest_forward_obstacle(pts, math.radians(30)) == math.hypot(0.5, 0.05)


def test_nearest_forward_ignores_out_of_cone():
    # A near point far off to the side is outside a 20 deg cone.
    pts = [(0.5, 0.5)]  # bearing 45 deg
    assert nearest_forward_obstacle(pts, math.radians(20)) is None


def test_nearest_forward_ignores_behind_and_out_of_range():
    pts = [(-0.5, 0.0), (0.02, 0.0), (5.0, 0.0)]
    assert nearest_forward_obstacle(pts, math.radians(30), min_range_m=0.05, max_range_m=2.0) is None


def test_nearest_forward_clear_cone_is_none():
    assert nearest_forward_obstacle([], math.radians(30)) is None


def test_scale_clear_is_full_speed():
    assert forward_speed_scale(None, 0.25, 0.45) == 1.0
    assert forward_speed_scale(0.9, 0.25, 0.45) == 1.0


def test_scale_inside_stop_is_zero():
    assert forward_speed_scale(0.20, 0.25, 0.45) == 0.0
    assert forward_speed_scale(0.25, 0.25, 0.45) == 0.0


def test_scale_ramps_in_slow_band_with_floor():
    # midpoint of the slow band with a 0.5 floor -> 0.75
    s = forward_speed_scale(0.35, 0.25, 0.45, min_forward_scale=0.5)
    assert s == 0.75


def test_scale_degenerate_band_stops():
    assert forward_speed_scale(0.30, 0.30, 0.30) == 0.0
