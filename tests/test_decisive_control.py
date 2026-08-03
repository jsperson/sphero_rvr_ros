"""Tests for the pragmatic drive decision (straight / arc / pivot)."""

import math

import pytest

from sphero_rvr_core.decisive_control import (
    BackOffConfig,
    DecisiveControlConfig,
    ProgressGuard,
    compute_drive_command,
    heading_error_to_point,
    select_target_point,
)

CFG = DecisiveControlConfig()

# Small, easy-to-reason-about back-off config for the ProgressGuard tests.
BO = BackOffConfig(
    stall_time_s=1.0,
    progress_epsilon_m=0.03,
    back_off_speed_mps=0.10,
    back_off_distance_m=0.10,
    back_off_timeout_s=2.0,
    max_back_offs=2,
)


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


# --- ProgressGuard: the back-off reflex (reverse out of a boxed-in stall) ---


def test_guard_moving_forward_never_backs_off():
    guard = ProgressGuard(BO)
    x = 0.0
    for i in range(10):
        x += 0.05  # advancing > epsilon each cycle
        assert guard.step(x, 0.0, i * 0.5, translating=True).action == "drive"


def test_guard_stall_while_translating_triggers_reverse():
    guard = ProgressGuard(BO)
    assert guard.step(0.0, 0.0, 0.0, translating=True).action == "drive"  # arms clock
    assert guard.step(0.0, 0.0, 0.5, translating=True).action == "drive"  # < stall_time
    result = guard.step(0.0, 0.0, 1.0, translating=True)  # stalled >= 1.0 s
    assert result.action == "reverse"
    assert result.reverse_speed_mps == BO.back_off_speed_mps


def test_guard_pivot_does_not_accrue_stall():
    # Not translating (pivoting): position never changes, but that is expected —
    # it must never be mistaken for a boxed-in stall.
    guard = ProgressGuard(BO)
    for i in range(10):
        assert guard.step(0.0, 0.0, i * 0.5, translating=False).action == "drive"


def test_guard_back_off_completes_then_resumes():
    guard = ProgressGuard(BO)
    guard.step(0.0, 0.0, 0.0, translating=True)
    guard.step(0.0, 0.0, 0.5, translating=True)
    assert guard.step(0.0, 0.0, 1.0, translating=True).action == "reverse"  # enter back-off
    # Reversing, but not yet the full distance -> keep reversing.
    assert guard.step(-0.05, 0.0, 1.2, translating=True).action == "reverse"
    # Reached back_off_distance (0.10 m) -> resume normal control.
    assert guard.step(-0.10, 0.0, 1.5, translating=True).action == "drive"


def test_guard_back_off_timeout_aborts_when_cannot_reverse():
    # Rear blocked: robot cannot actually back up, so the back-off times out.
    guard = ProgressGuard(BO)
    guard.step(0.0, 0.0, 0.0, translating=True)
    guard.step(0.0, 0.0, 0.5, translating=True)
    assert guard.step(0.0, 0.0, 1.0, translating=True).action == "reverse"  # bo starts at t=1.0
    assert guard.step(0.0, 0.0, 2.0, translating=True).action == "reverse"  # 1.0 s < timeout
    assert guard.step(0.0, 0.0, 3.0, translating=True).action == "abort"    # 2.0 s >= timeout


def test_guard_aborts_after_max_back_offs():
    cfg = BackOffConfig(
        stall_time_s=1.0,
        progress_epsilon_m=0.03,
        back_off_speed_mps=0.10,
        back_off_distance_m=0.10,
        back_off_timeout_s=2.0,
        max_back_offs=1,
    )
    guard = ProgressGuard(cfg)
    guard.step(0.0, 0.0, 0.0, translating=True)
    guard.step(0.0, 0.0, 0.5, translating=True)
    assert guard.step(0.0, 0.0, 1.0, translating=True).action == "reverse"  # back-off #1
    assert guard.step(-0.10, 0.0, 1.5, translating=True).action == "drive"  # completed, resume
    guard.step(-0.10, 0.0, 2.0, translating=True)  # < stall_time from resume
    # Second stall would be back-off #2 > max_back_offs (1) -> abort instead.
    assert guard.step(-0.10, 0.0, 2.5, translating=True).action == "abort"
