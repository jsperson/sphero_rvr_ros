"""Tests for the camera low-obstacle contribution to the collision brake."""

import math

import pytest

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


def test_swept_path_straight_is_a_corridor():
    from sphero_rvr_core.low_obstacle_brake import swept_path_obstacle
    pts = [(0.8, 0.0), (0.5, 0.9)]          # one ahead, one far off to the side
    got = swept_path_obstacle(pts, linear_mps=0.1, angular_rad_s=0.0, half_width_m=0.15)
    assert got == pytest.approx(0.8)


def test_swept_path_straight_ignores_behind():
    from sphero_rvr_core.low_obstacle_brake import swept_path_obstacle
    assert swept_path_obstacle([(-0.5, 0.0)], 0.1, 0.0, 0.15) is None


def test_left_turn_sees_obstacle_off_the_nose_that_a_cone_misses():
    """The whole point: turning left, the swept arc covers ground a straight
    corridor does not, which is how the chair leg was hit."""
    from sphero_rvr_core.low_obstacle_brake import nearest_forward_obstacle, swept_path_obstacle
    # left turn: v=0.1, w=0.5 -> radius 0.2 m, centre at (0, +0.2)
    # a point on that circle, ahead and to the left, well outside a narrow corridor
    pt = (0.2, 0.2)
    assert swept_path_obstacle([pt], 0.1, 0.5, half_width_m=0.15) is not None
    # a straight corridor of the same width misses it entirely
    assert swept_path_obstacle([pt], 0.1, 0.0, half_width_m=0.15) is None


def test_turn_ignores_points_outside_the_swept_annulus():
    from sphero_rvr_core.low_obstacle_brake import swept_path_obstacle
    # radius 0.2, half width 0.05 -> annulus 0.15..0.25 from centre (0,0.2)
    far_inside = (0.0, 0.2)       # distance 0 from centre -> not swept
    assert swept_path_obstacle([far_inside], 0.1, 0.5, half_width_m=0.05) is None


def test_swept_path_respects_range_gate():
    from sphero_rvr_core.low_obstacle_brake import swept_path_obstacle
    assert swept_path_obstacle([(3.0, 0.0)], 0.1, 0.0, 0.15, max_range_m=1.5) is None


def test_reverse_looks_behind():
    from sphero_rvr_core.low_obstacle_brake import swept_path_obstacle
    assert swept_path_obstacle([(-0.4, 0.0)], -0.1, 0.0, 0.15) == pytest.approx(0.4)
    assert swept_path_obstacle([(0.4, 0.0)], -0.1, 0.0, 0.15) is None
