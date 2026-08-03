"""Tests for the pragmatic drive decision (straight / arc / pivot)."""

import math

import pytest

from sphero_rvr_core.decisive_control import (
    DecisiveControlConfig,
    compute_drive_command,
    heading_error_to_point,
    select_target_point,
)

CFG = DecisiveControlConfig()


def test_aligned_drives_straight_without_turning():
    cmd = compute_drive_command(0.05, distance_to_target_m=1.0, config=CFG)
    assert cmd.mode == "straight"
    assert cmd.linear_mps == CFG.cruise_speed_mps
    assert cmd.angular_rad_s == 0.0  # the deadband: do not turn when roughly aligned


def test_moderate_error_arcs_while_moving():
    cmd = compute_drive_command(0.5, distance_to_target_m=1.0, config=CFG)  # ~29 deg left
    assert cmd.mode == "arc"
    assert cmd.linear_mps == CFG.cruise_speed_mps  # keeps rolling -> no grind
    assert cmd.angular_rad_s == pytest.approx(CFG.arc_gain * 0.5)
    assert cmd.angular_rad_s > 0  # target to the left -> turn left (+)


def test_arc_turns_right_for_right_target():
    cmd = compute_drive_command(-0.5, distance_to_target_m=1.0, config=CFG)
    assert cmd.mode == "arc"
    assert cmd.angular_rad_s < 0  # target to the right -> turn right (-)


def test_arc_angular_is_capped():
    # Just under the pivot threshold, arc_gain would exceed the cap.
    cmd = compute_drive_command(1.0, distance_to_target_m=1.0, config=CFG)
    assert cmd.mode == "arc"
    assert cmd.angular_rad_s == pytest.approx(CFG.max_arc_angular_rad_s)


def test_large_error_pivots_in_place_decisively():
    cmd = compute_drive_command(2.0, distance_to_target_m=1.0, config=CFG)  # ~115 deg
    assert cmd.mode == "pivot"
    assert cmd.linear_mps == 0.0
    # decisive rate above breakaway, never a slow creep
    assert cmd.angular_rad_s == pytest.approx(CFG.pivot_rate_rad_s)


def test_large_negative_error_pivots_right():
    cmd = compute_drive_command(-2.5, distance_to_target_m=1.0, config=CFG)
    assert cmd.mode == "pivot"
    assert cmd.angular_rad_s == pytest.approx(-CFG.pivot_rate_rad_s)


def test_arrived_stops():
    cmd = compute_drive_command(1.5, distance_to_target_m=0.05, config=CFG)
    assert cmd.mode == "arrived"
    assert cmd.linear_mps == 0.0 and cmd.angular_rad_s == 0.0


def test_heading_error_wraps_around_pi():
    # A target nearly behind should not read as a huge unwrapped angle.
    cmd = compute_drive_command(math.pi + 0.1, distance_to_target_m=1.0, config=CFG)
    assert cmd.mode == "pivot"  # ~180 deg behind -> pivot, and wrapped (not straight)


def test_select_target_point_picks_lookahead_ahead():
    path = [(0.0, 0.0), (0.2, 0.0), (0.4, 0.0), (0.6, 0.0), (0.8, 0.0)]
    # Robot at origin, lookahead 0.5 -> first point >= 0.5 away is (0.6, 0.0).
    assert select_target_point(path, 0.0, 0.0, 0.5) == (0.6, 0.0)


def test_select_target_point_returns_last_when_path_shorter_than_lookahead():
    path = [(0.0, 0.0), (0.2, 0.0)]
    # Near the goal: nothing is 1.0 away, so aim at the last point (the goal).
    assert select_target_point(path, 0.0, 0.0, 1.0) == (0.2, 0.0)


def test_select_target_point_starts_from_nearest_not_index_zero():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    # Robot already at 1.0: nearest is index 1, lookahead 0.5 -> (2.0, 0.0).
    assert select_target_point(path, 1.0, 0.0, 0.5) == (2.0, 0.0)


def test_select_target_point_empty_path():
    assert select_target_point([], 0.0, 0.0, 0.5) is None


def test_heading_error_to_point_geometry():
    # Robot at origin facing +x; target dead ahead -> ~0 error.
    err, dist = heading_error_to_point(0.0, 0.0, 0.0, 1.0, 0.0)
    assert err == pytest.approx(0.0, abs=1e-9)
    assert dist == pytest.approx(1.0)
    # Target to the left (+y) -> positive (CCW) error.
    err_left, _ = heading_error_to_point(0.0, 0.0, 0.0, 1.0, 1.0)
    assert err_left == pytest.approx(math.pi / 4)
    # Robot already facing the target -> zero error regardless of position.
    err0, d0 = heading_error_to_point(2.0, 3.0, math.atan2(1.0, 1.0), 3.0, 4.0)
    assert err0 == pytest.approx(0.0, abs=1e-9)
    assert d0 == pytest.approx(math.hypot(1.0, 1.0))
