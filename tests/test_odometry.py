import math

import pytest

from sphero_rvr_core.responses import EncoderCounts
from sphero_rvr_driver.odometry import (
    COVARIANCE_SIZE,
    DifferentialOdomConfig,
    DifferentialOdomTracker,
    MotionPrimitiveConfig,
    MotionPrimitiveController,
    MotionPrimitiveGoal,
    MotionPrimitiveKind,
    MotionPrimitiveStopReason,
    OdomMotionState,
    encoder_delta,
    planar_pose_covariance,
    planar_twist_covariance,
)


def test_encoder_delta_unwraps_signed_int32_rollover():
    assert encoder_delta(-2_147_483_640, 2_147_483_640) == 16
    assert encoder_delta(2_147_483_640, -2_147_483_640) == -16


def test_encoder_delta_can_use_raw_subtraction_when_modulus_disabled():
    assert encoder_delta(-2_147_483_640, 2_147_483_640, modulus=None) == -4_294_967_280


def test_differential_odom_tracker_integrates_encoder_counts():
    tracker = DifferentialOdomTracker(
        DifferentialOdomConfig(counts_per_meter=1000.0, wheel_track_m=0.25)
    )

    assert tracker.update(EncoderCounts(left=100, right=100), stamp=1.0) is None
    sample = tracker.update(EncoderCounts(left=200, right=300), stamp=3.0)

    assert sample is not None
    assert sample.frame_id == "odom"
    assert sample.child_frame_id == "base_link"
    assert sample.source == "encoder_counts"
    assert "Open-loop" in sample.quality_note
    assert sample.x == pytest.approx(0.14701, abs=1e-5)
    assert sample.y == pytest.approx(0.02980, abs=1e-5)
    assert sample.yaw == pytest.approx(0.4)
    assert sample.linear_mps == pytest.approx(0.075)
    assert sample.angular_rad_s == pytest.approx(0.2)


def test_differential_odom_tracker_wraps_heading_and_moves_in_current_heading():
    tracker = DifferentialOdomTracker(
        DifferentialOdomConfig(counts_per_meter=100.0, wheel_track_m=0.5)
    )
    tracker.update(EncoderCounts(left=0, right=0), stamp=0.0)

    sample = tracker.update(EncoderCounts(left=-400, right=400), stamp=1.0)
    assert sample is not None
    assert -math.pi <= sample.yaw <= math.pi

    moved = tracker.update(EncoderCounts(left=-300, right=500), stamp=2.0)
    assert moved is not None
    assert abs(moved.y) > 0.01


def test_differential_odom_tracker_handles_encoder_count_rollover():
    tracker = DifferentialOdomTracker(
        DifferentialOdomConfig(counts_per_meter=100.0, wheel_track_m=0.5)
    )
    tracker.update(EncoderCounts(left=2_147_483_640, right=2_147_483_640), stamp=10.0)
    sample = tracker.update(EncoderCounts(left=-2_147_483_640, right=-2_147_483_640), stamp=11.0)

    assert sample is not None
    assert sample.x == pytest.approx(0.16)
    assert sample.y == pytest.approx(0.0)
    assert sample.yaw == pytest.approx(0.0)
    assert sample.linear_mps == pytest.approx(0.16)


def test_differential_odom_tracker_drops_non_monotonic_samples_but_rebaselines():
    tracker = DifferentialOdomTracker(
        DifferentialOdomConfig(counts_per_meter=100.0, wheel_track_m=0.5)
    )
    tracker.update(EncoderCounts(left=0, right=0), stamp=2.0)
    assert tracker.update(EncoderCounts(left=10, right=10), stamp=2.0) is None

    sample = tracker.update(EncoderCounts(left=20, right=20), stamp=3.0)
    assert sample is not None
    assert sample.x == pytest.approx(0.10)


def test_covariance_helpers_populate_planar_ros_slots_only():
    pose = planar_pose_covariance(0.11, 0.22)
    twist = planar_twist_covariance(0.33, 0.44)

    assert len(pose) == COVARIANCE_SIZE
    assert len(twist) == COVARIANCE_SIZE
    assert pose[0] == 0.11
    assert pose[7] == 0.11
    assert pose[35] == 0.22
    assert sum(1 for value in pose if value) == 3
    assert twist[0] == 0.33
    assert twist[35] == 0.44
    assert sum(1 for value in twist if value) == 2


def test_config_rejects_invalid_covariance_and_modulus():
    with pytest.raises(ValueError, match="odom_pose_xy_covariance|pose_xy_covariance"):
        DifferentialOdomConfig(
            counts_per_meter=100.0,
            wheel_track_m=0.5,
            pose_xy_covariance=-0.1,
        )
    with pytest.raises(ValueError, match="encoder_count_modulus"):
        DifferentialOdomConfig(
            counts_per_meter=100.0,
            wheel_track_m=0.5,
            encoder_count_modulus=0,
        )


def test_move_distance_primitive_uses_measured_odom_progress_and_heading_hold():
    controller = MotionPrimitiveController(MotionPrimitiveConfig(min_progress_m=0.01, startup_grace_s=1.0))
    goal = MotionPrimitiveGoal.move_distance(distance_m=0.4572, speed_mps=0.10, timeout_s=10.0)

    telemetry = controller.start(goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.05))
    assert telemetry.stop_reason is MotionPrimitiveStopReason.RUNNING

    early = controller.update(OdomMotionState(stamp=0.8, x_m=0.008, y_m=0.0, yaw_rad=0.08))
    assert early.stop_reason is MotionPrimitiveStopReason.RUNNING
    assert early.command.linear_x > 0.0
    assert early.command.angular_z < 0.0

    done = controller.update(OdomMotionState(stamp=4.8, x_m=0.46, y_m=0.0, yaw_rad=0.05))
    assert done.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
    assert done.measured_distance_m == pytest.approx(0.459425)
    assert done.command.linear_x == 0.0


def test_move_distance_primitive_treats_attended_slow_progress_as_motion():
    """Regression for the 2026-07-24 attended 10 cm physical trace."""
    observed_progress = (
        (0.20, 0.00196),
        (0.30, 0.00887),
        (0.40, 0.01670),
        (0.50, 0.02320),
        (0.60, 0.02820),
        (0.70, 0.03240),
        (0.80, 0.03520),
        (0.90, 0.03650),
        (1.00, 0.03930),
        (1.10, 0.04070),
        (1.20, 0.04120),
        (1.30, 0.04160),
        (1.40, 0.04210),
        (1.50, 0.04300),
        (1.60, 0.04438),
    )
    goal = MotionPrimitiveGoal.move_distance(
        distance_m=0.10,
        speed_mps=0.08,
        timeout_s=5.25,
    )

    controller = MotionPrimitiveController(MotionPrimitiveConfig())
    controller.start(goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0))
    for stamp, distance_m in observed_progress:
        telemetry = controller.update(
            OdomMotionState(
                stamp=stamp,
                x_m=distance_m,
                y_m=0.0,
                yaw_rad=0.0,
            )
        )
        assert telemetry.stop_reason is MotionPrimitiveStopReason.RUNNING
        assert telemetry.command.linear_x == pytest.approx(0.08)

    # The same evidence really did trip the former 15 mm checkpoint policy.
    legacy = MotionPrimitiveController(MotionPrimitiveConfig(min_progress_m=0.015))
    legacy.start(goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0))
    legacy_telemetry = None
    for stamp, distance_m in observed_progress:
        legacy_telemetry = legacy.update(
            OdomMotionState(
                stamp=stamp,
                x_m=distance_m,
                y_m=0.0,
                yaw_rad=0.0,
            )
        )
        if legacy_telemetry.stop_reason is not MotionPrimitiveStopReason.RUNNING:
            break
    assert legacy_telemetry is not None
    assert legacy_telemetry.stop_reason is MotionPrimitiveStopReason.STALL


def test_move_distance_primitive_still_stops_a_real_stall_with_smaller_activity_quantum():
    controller = MotionPrimitiveController(MotionPrimitiveConfig())
    goal = MotionPrimitiveGoal.move_distance(
        distance_m=0.10,
        speed_mps=0.08,
        timeout_s=5.25,
    )
    controller.start(goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0))

    moving = controller.update(
        OdomMotionState(stamp=0.4, x_m=0.006, y_m=0.0, yaw_rad=0.0)
    )
    stalled = controller.update(
        OdomMotionState(stamp=1.51, x_m=0.006, y_m=0.0, yaw_rad=0.0)
    )

    assert moving.stop_reason is MotionPrimitiveStopReason.RUNNING
    assert stalled.stop_reason is MotionPrimitiveStopReason.STALL
    assert stalled.command.linear_x == 0.0
    assert stalled.command.angular_z == 0.0


def test_move_distance_primitive_reserves_measured_target_stop_horizon():
    controller = MotionPrimitiveController(
        MotionPrimitiveConfig(target_stop_horizon_s=0.25)
    )
    goal = MotionPrimitiveGoal.move_distance(
        distance_m=0.10,
        speed_mps=0.10,
        timeout_s=5.0,
    )
    controller.start(
        goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0)
    )

    outside = controller.update(
        OdomMotionState(stamp=0.1, x_m=0.053, y_m=0.0, yaw_rad=0.0)
    )
    braking = controller.update(
        OdomMotionState(stamp=0.2, x_m=0.075, y_m=0.0, yaw_rad=0.0)
    )

    assert outside.stop_reason is MotionPrimitiveStopReason.RUNNING
    assert braking.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
    assert braking.measured_distance_m == pytest.approx(0.075)
    assert braking.command.linear_x == 0.0


def test_target_stop_horizon_ignores_stationary_and_wrong_way_samples():
    controller = MotionPrimitiveController(
        MotionPrimitiveConfig(target_stop_horizon_s=0.25)
    )
    goal = MotionPrimitiveGoal.move_distance(
        distance_m=0.10,
        speed_mps=0.10,
        timeout_s=5.0,
    )
    controller.start(
        goal, OdomMotionState(stamp=0.0, x_m=0.05, y_m=0.0, yaw_rad=0.0)
    )

    stationary = controller.update(
        OdomMotionState(stamp=0.1, x_m=0.05, y_m=0.0, yaw_rad=0.0)
    )
    wrong_way = controller.update(
        OdomMotionState(stamp=0.2, x_m=0.04, y_m=0.0, yaw_rad=0.0)
    )

    assert stationary.stop_reason is MotionPrimitiveStopReason.RUNNING
    assert wrong_way.stop_reason is MotionPrimitiveStopReason.RUNNING


def test_turn_angle_primitive_uses_measured_heading_and_stops_at_angle():
    controller = MotionPrimitiveController(
        MotionPrimitiveConfig(
            angle_tolerance_rad=math.radians(2.0),
            max_turn_speed_rad_s=0.35,
        )
    )
    goal = MotionPrimitiveGoal.turn_angle(angle_rad=math.radians(90.0), angular_speed_rad_s=0.5, timeout_s=8.0)

    controller.start(goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0))
    mid = controller.update(OdomMotionState(stamp=1.0, x_m=0.0, y_m=0.0, yaw_rad=math.radians(40.0)))
    done = controller.update(OdomMotionState(stamp=2.2, x_m=0.0, y_m=0.0, yaw_rad=math.radians(89.0)))

    assert mid.kind is MotionPrimitiveKind.TURN_ANGLE
    assert mid.command.angular_z == pytest.approx(0.35)
    assert done.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
    assert done.measured_angle_rad == pytest.approx(math.radians(89.0))
    assert done.command.angular_z == 0.0


def test_turn_angle_primitive_reserves_measured_rate_target_stop_horizon():
    controller = MotionPrimitiveController(
        MotionPrimitiveConfig(
            turn_target_stop_horizon_s=0.10,
            max_turn_speed_rad_s=0.35,
        )
    )
    goal = MotionPrimitiveGoal.turn_angle(
        angle_rad=math.radians(45.0),
        angular_speed_rad_s=0.35,
        timeout_s=5.0,
    )
    controller.start(
        goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0)
    )

    for stamp, yaw_deg in (
        (0.1, 2.8),
        (0.2, 8.0),
        (0.3, 13.0),
        (0.4, 19.0),
        (0.5, 25.0),
    ):
        moving = controller.update(
            OdomMotionState(
                stamp=stamp,
                x_m=0.0,
                y_m=0.0,
                yaw_rad=math.radians(yaw_deg),
            )
        )
        assert moving.stop_reason is MotionPrimitiveStopReason.RUNNING

    outside = controller.update(
        OdomMotionState(
            stamp=0.6,
            x_m=0.0,
            y_m=0.0,
            yaw_rad=math.radians(31.0),
        )
    )
    braking = controller.update(
        OdomMotionState(
            stamp=0.7,
            x_m=0.0,
            y_m=0.0,
            yaw_rad=math.radians(37.0),
        )
    )

    assert outside.stop_reason is MotionPrimitiveStopReason.RUNNING
    assert outside.command.angular_z == pytest.approx(0.35)
    assert braking.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
    assert braking.measured_angle_rad == pytest.approx(math.radians(37.0))
    assert braking.command.angular_z == 0.0


def test_turn_angle_primitive_retains_rate_to_brake_between_odom_samples():
    controller = MotionPrimitiveController(
        MotionPrimitiveConfig(
            turn_target_stop_horizon_s=0.10,
            max_turn_speed_rad_s=0.35,
        )
    )
    goal = MotionPrimitiveGoal.turn_angle(
        angle_rad=math.radians(45.0),
        angular_speed_rad_s=0.35,
        timeout_s=5.0,
    )
    controller.start(
        goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0)
    )
    controller.update(
        OdomMotionState(
            stamp=0.10, x_m=0.0, y_m=0.0, yaw_rad=math.radians(6.5)
        ),
        now=0.10,
    )
    latest = OdomMotionState(
        stamp=0.20, x_m=0.0, y_m=0.0, yaw_rad=math.radians(18.75)
    )
    moving = controller.update(latest, now=0.20)
    still_moving = controller.update(latest, now=0.25)
    braking = controller.update(latest, now=0.30)

    assert moving.command.angular_z == pytest.approx(0.35)
    assert still_moving.command.angular_z == pytest.approx(0.35)
    assert braking.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
    assert braking.command.angular_z == 0.0


def test_turn_angle_primitive_can_resume_same_goal_after_stationary_undershoot():
    controller = MotionPrimitiveController(
        MotionPrimitiveConfig(max_turn_speed_rad_s=0.35)
    )
    goal = MotionPrimitiveGoal.turn_angle(
        angle_rad=math.radians(45.0),
        angular_speed_rad_s=0.35,
        timeout_s=5.0,
    )
    controller.start(
        goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0)
    )
    controller.update(
        OdomMotionState(
            stamp=0.2, x_m=0.0, y_m=0.0, yaw_rad=math.radians(20.0)
        )
    )
    braking = controller.update(
        OdomMotionState(
            stamp=0.4, x_m=0.0, y_m=0.0, yaw_rad=math.radians(36.0)
        )
    )
    assert braking.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED

    correction = controller.resume_target_correction(
        OdomMotionState(
            stamp=1.0, x_m=0.0, y_m=0.0, yaw_rad=math.radians(36.0)
        )
    )
    corrected = controller.update(
        OdomMotionState(
            stamp=1.1, x_m=0.0, y_m=0.0, yaw_rad=math.radians(44.0)
        )
    )

    assert correction.stop_reason is MotionPrimitiveStopReason.RUNNING
    assert correction.command.angular_z == pytest.approx(0.35)
    assert corrected.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
    assert corrected.measured_angle_rad == pytest.approx(math.radians(44.0))


def test_motion_primitive_refuses_correction_after_non_target_terminal():
    controller = MotionPrimitiveController(MotionPrimitiveConfig())
    controller.start(
        MotionPrimitiveGoal.turn_angle(
            angle_rad=math.radians(45.0),
            angular_speed_rad_s=0.35,
            timeout_s=5.0,
        ),
        OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0),
    )
    state = OdomMotionState(stamp=0.1, x_m=0.0, y_m=0.0, yaw_rad=0.0)
    controller.update(state, collision_veto=True)

    with pytest.raises(RuntimeError, match="target-reached"):
        controller.resume_target_correction(state)


def test_turn_correction_pause_publishes_zero_and_requires_active_turn():
    controller = MotionPrimitiveController(MotionPrimitiveConfig())
    controller.start(
        MotionPrimitiveGoal.turn_angle(
            angle_rad=math.radians(45.0),
            angular_speed_rad_s=0.35,
            timeout_s=5.0,
        ),
        OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0),
    )
    controller.update(
        OdomMotionState(
            stamp=0.3, x_m=0.0, y_m=0.0, yaw_rad=math.radians(36.0)
        )
    )
    controller.resume_target_correction(
        OdomMotionState(
            stamp=1.0, x_m=0.0, y_m=0.0, yaw_rad=math.radians(36.0)
        )
    )
    paused = controller.pause_target_correction(
        OdomMotionState(
            stamp=1.05, x_m=0.0, y_m=0.0, yaw_rad=math.radians(36.0)
        )
    )

    assert paused.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
    assert paused.command.linear_x == paused.command.angular_z == 0.0
    with pytest.raises(RuntimeError, match="active motion"):
        controller.pause_target_correction(
            OdomMotionState(
                stamp=1.1, x_m=0.0, y_m=0.0, yaw_rad=math.radians(36.0)
            )
        )


def test_motion_primitive_timeout_uses_wall_time_when_odom_is_fresh_but_lagged():
    controller = MotionPrimitiveController(MotionPrimitiveConfig())
    controller.start(
        MotionPrimitiveGoal.turn_angle(
            angle_rad=math.radians(45.0),
            angular_speed_rad_s=0.35,
            timeout_s=1.0,
        ),
        OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0),
    )
    timed_out = controller.update(
        OdomMotionState(
            stamp=0.9, x_m=0.0, y_m=0.0, yaw_rad=math.radians(10.0)
        ),
        now=1.1,
    )

    assert timed_out.stop_reason is MotionPrimitiveStopReason.TIMEOUT
    assert timed_out.command.linear_x == timed_out.command.angular_z == 0.0


def test_turn_angle_primitive_applies_configured_speed_ceiling_in_both_directions():
    config = MotionPrimitiveConfig(max_turn_speed_rad_s=0.2)

    left = MotionPrimitiveController(config)
    left.start(
        MotionPrimitiveGoal.turn_angle(
            angle_rad=math.radians(45.0),
            angular_speed_rad_s=0.5,
            timeout_s=8.0,
        ),
        OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0),
    )
    left_command = left.update(
        OdomMotionState(stamp=0.1, x_m=0.0, y_m=0.0, yaw_rad=0.0)
    ).command

    right = MotionPrimitiveController(config)
    right.start(
        MotionPrimitiveGoal.turn_angle(
            angle_rad=math.radians(-45.0),
            angular_speed_rad_s=0.5,
            timeout_s=8.0,
        ),
        OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0),
    )
    right_command = right.update(
        OdomMotionState(stamp=0.1, x_m=0.0, y_m=0.0, yaw_rad=0.0)
    ).command

    assert left_command.angular_z == pytest.approx(0.2)
    assert right_command.angular_z == pytest.approx(-0.2)


def test_motion_primitive_config_rejects_nonpositive_turn_speed_ceiling():
    with pytest.raises(ValueError, match="max_turn_speed_rad_s"):
        MotionPrimitiveConfig(max_turn_speed_rad_s=0.0)


def test_motion_primitive_config_rejects_nonpositive_turn_progress_rate_ceiling():
    with pytest.raises(ValueError, match="max_turn_progress_rate_rad_s"):
        MotionPrimitiveConfig(max_turn_progress_rate_rad_s=0.0)


def test_motion_primitive_config_rejects_nonpositive_turn_stop_horizon():
    with pytest.raises(ValueError, match="turn_target_stop_horizon_s"):
        MotionPrimitiveConfig(turn_target_stop_horizon_s=0.0)


def test_motion_primitive_config_rejects_nonpositive_target_stop_horizon():
    with pytest.raises(ValueError, match="target_stop_horizon_s"):
        MotionPrimitiveConfig(target_stop_horizon_s=0.0)


def test_turn_angle_primitive_preserves_signed_direction_and_rejects_wrong_way_rotation():
    controller = MotionPrimitiveController(MotionPrimitiveConfig(angle_tolerance_rad=math.radians(2.0), startup_grace_s=0.5))
    goal = MotionPrimitiveGoal.turn_angle(angle_rad=math.radians(-90.0), angular_speed_rad_s=0.5, timeout_s=8.0)

    controller.start(goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0))
    wrong_way = controller.update(OdomMotionState(stamp=1.0, x_m=0.0, y_m=0.0, yaw_rad=math.radians(89.0)))

    assert wrong_way.stop_reason is MotionPrimitiveStopReason.STALL
    assert wrong_way.measured_angle_rad == pytest.approx(0.0)
