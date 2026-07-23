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


def test_turn_angle_primitive_uses_measured_heading_and_stops_at_angle():
    controller = MotionPrimitiveController(MotionPrimitiveConfig(angle_tolerance_rad=math.radians(2.0)))
    goal = MotionPrimitiveGoal.turn_angle(angle_rad=math.radians(90.0), angular_speed_rad_s=0.5, timeout_s=8.0)

    controller.start(goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0))
    mid = controller.update(OdomMotionState(stamp=1.0, x_m=0.0, y_m=0.0, yaw_rad=math.radians(40.0)))
    done = controller.update(OdomMotionState(stamp=2.2, x_m=0.0, y_m=0.0, yaw_rad=math.radians(89.0)))

    assert mid.kind is MotionPrimitiveKind.TURN_ANGLE
    assert mid.command.angular_z == pytest.approx(0.25)
    assert done.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
    assert done.measured_angle_rad == pytest.approx(math.radians(89.0))
    assert done.command.angular_z == 0.0


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


def test_turn_angle_primitive_preserves_signed_direction_and_rejects_wrong_way_rotation():
    controller = MotionPrimitiveController(MotionPrimitiveConfig(angle_tolerance_rad=math.radians(2.0), startup_grace_s=0.5))
    goal = MotionPrimitiveGoal.turn_angle(angle_rad=math.radians(-90.0), angular_speed_rad_s=0.5, timeout_s=8.0)

    controller.start(goal, OdomMotionState(stamp=0.0, x_m=0.0, y_m=0.0, yaw_rad=0.0))
    wrong_way = controller.update(OdomMotionState(stamp=1.0, x_m=0.0, y_m=0.0, yaw_rad=math.radians(89.0)))

    assert wrong_way.stop_reason is MotionPrimitiveStopReason.STALL
    assert wrong_way.measured_angle_rad == pytest.approx(0.0)
