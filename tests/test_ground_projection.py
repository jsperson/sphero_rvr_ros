"""The surviving clear-ray function (bucket zero, 2026-08-21, retired the rest:
the monocular IPM core died with the optical low-obstacle path; clear_ray_point
lives because semantic_map's stale-mark clearing depends on it)."""

import math

import pytest

from sphero_rvr_core.ground_projection import clear_ray_point


def test_clear_ray_point_centre_is_straight_ahead():
    fwd, left = clear_ray_point(u=100.0, cx=100.0, fx=170.0, range_m=1.8)
    assert fwd == pytest.approx(1.8) and left == pytest.approx(0.0)


def test_clear_ray_point_right_column_is_negative_left():
    _, left = clear_ray_point(u=150.0, cx=100.0, fx=170.0, range_m=1.8)
    assert left < 0  # right of centre -> -y, the optical-frame convention


def test_clear_ray_point_preserves_range():
    fwd, left = clear_ray_point(u=40.0, cx=100.0, fx=170.0, range_m=1.8)
    assert math.hypot(fwd, left) == pytest.approx(1.8)
