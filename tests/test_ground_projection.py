"""Tests for the inverse-perspective-mapping ground projection core."""

import math

import pytest

from sphero_rvr_core.ground_projection import ground_to_cell, pixel_to_ground

FX = FY = 500.0
CX = CY = 250.0


def test_center_ray_down_tilt_hits_expected_distance():
    # Camera 1 m up, optical axis 0.5 rad below horizontal; the center pixel meets
    # the floor at forward = h / tan(tilt), directly ahead.
    fwd, left = pixel_to_ground(CX, CY, FX, FY, CX, CY, cam_height_m=1.0, cam_tilt_down_rad=0.5)
    assert fwd == pytest.approx(1.0 / math.tan(0.5), rel=1e-3)
    assert left == pytest.approx(0.0, abs=1e-6)


def test_up_tilted_center_ray_has_no_ground_hit():
    # Our real camera is tilted ~3 deg UP: the center ray points above the horizon.
    assert pixel_to_ground(CX, CY, FX, FY, CX, CY, cam_height_m=0.114, cam_tilt_down_rad=-0.0524) is None


def test_lower_pixel_hits_ground_even_when_up_tilted():
    # A pixel well below the principal point still points down -> hits the floor
    # ahead (this is how the low, slightly-up camera sees near low obstacles).
    res = pixel_to_ground(CX, 450.0, FX, FY, CX, CY, cam_height_m=0.114, cam_tilt_down_rad=-0.0524)
    assert res is not None
    fwd, left = res
    assert fwd > 0.0
    assert left == pytest.approx(0.0, abs=1e-6)


def test_pixel_to_right_maps_to_negative_left():
    # u to the right of center -> the ground point is to the robot's right (y_left<0).
    fwd, left = pixel_to_ground(350.0, CY, FX, FY, CX, CY, cam_height_m=1.0, cam_tilt_down_rad=0.5)
    assert fwd > 0.0
    assert left < 0.0


def test_ground_to_cell_at_origin():
    assert ground_to_cell(1.0, 0.5, origin_x=0.0, origin_y=0.0, res=0.05) == (20, 10)


def test_ground_to_cell_with_robot_pose():
    # Robot at (2,0) facing +y (yaw 90deg): a point 1 m forward is at world (2,1).
    cell = ground_to_cell(1.0, 0.0, origin_x=0.0, origin_y=0.0, res=0.5, robot_x=2.0, robot_y=0.0, robot_yaw=math.pi / 2)
    assert cell == (4, 2)  # world (2.0, 1.0) / 0.5
