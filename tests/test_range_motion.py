import pytest

from sphero_rvr_driver.range_motion import (
    AngularRangeCandidate,
    MotionDirection,
    MotionGoal,
    MotionMode,
    RangeMotionConfig,
    RangeMotionController,
    RangeMotionSample,
    StopReason,
    TargetAssociationState,
    track_stable_surface,
    track_target_cluster,
)


def sample(t, clearance, *, front=1.0, rear=1.0, left=1.0, right=1.0, odom_x=0.0):
    return RangeMotionSample(
        stamp=t,
        target_clearance_m=clearance,
        front_clearance_m=front,
        rear_clearance_m=rear,
        left_clearance_m=left,
        right_clearance_m=right,
        odom_displacement_m=odom_x,
    )


def clustered_sample(t, candidates, *, front=1.0, odom_x=0.0):
    return RangeMotionSample(
        stamp=t,
        target_clearance_m=None,
        front_clearance_m=front,
        rear_clearance_m=1.0,
        left_clearance_m=1.0,
        right_clearance_m=1.0,
        odom_displacement_m=odom_x,
        target_candidates=tuple(candidates),
    )


def test_forward_approach_ramps_slows_and_stops_at_measured_target_not_duration():
    cfg = RangeMotionConfig(max_speed_mps=0.30, acceleration_mps2=1.0, target_tolerance_m=0.01)
    controller = RangeMotionController(cfg)
    controller.start(MotionGoal(direction=MotionDirection.FORWARD, mode=MotionMode.APPROACH, target_clearance_m=0.10), sample(0.0, 0.50))

    early = controller.update(sample(0.1, 0.46))
    mid = controller.update(sample(0.5, 0.25, odom_x=0.20))
    near = controller.update(sample(0.8, 0.13, odom_x=0.34))
    stopped = controller.update(sample(1.0, 0.105, odom_x=0.395))

    assert early.command.linear_x > 0.0
    assert mid.command.linear_x <= cfg.max_speed_mps
    assert 0.0 < near.command.linear_x < mid.command.linear_x
    assert stopped.stop_reason is StopReason.TARGET_REACHED
    assert stopped.command.linear_x == 0.0
    assert stopped.measured_displacement_m == pytest.approx(0.395)
    assert stopped.lidar_range_rate_mps < 0.0


def test_range_motion_default_speed_ceiling_matches_collision_supervisor_cap():
    assert RangeMotionConfig().max_speed_mps == pytest.approx(0.10)
    controller = RangeMotionController(RangeMotionConfig(acceleration_mps2=10.0))
    controller.start(MotionGoal(direction=MotionDirection.FORWARD, mode=MotionMode.APPROACH, target_clearance_m=0.10), sample(0.0, 0.50))

    telemetry = controller.update(sample(1.0, 0.40, odom_x=0.01))

    assert telemetry.requested_velocity_mps <= 0.10
    assert telemetry.forwarded_velocity_mps == 0.0
    assert telemetry.collision_state == "unknown"


def test_reverse_release_uses_measured_progress_when_actual_speed_is_below_commanded():
    cfg = RangeMotionConfig(max_speed_mps=0.25, acceleration_mps2=2.0, target_tolerance_m=0.01)
    controller = RangeMotionController(cfg)
    controller.start(MotionGoal(direction=MotionDirection.BACKWARD, mode=MotionMode.RETREAT, target_clearance_m=0.30), sample(0.0, 0.12))

    first = controller.update(sample(0.2, 0.14, odom_x=-0.01))
    still_moving = controller.update(sample(1.0, 0.22, odom_x=-0.04))
    stopped = controller.update(sample(2.0, 0.302, odom_x=-0.095))

    assert first.command.linear_x < 0.0
    assert still_moving.stop_reason is StopReason.RUNNING
    assert still_moving.command.linear_x < 0.0
    assert stopped.stop_reason is StopReason.TARGET_REACHED
    assert stopped.command.linear_x == 0.0
    assert stopped.measured_displacement_m == pytest.approx(0.095)


def test_stall_detection_fails_closed_when_range_and_odom_do_not_progress():
    cfg = RangeMotionConfig(stall_timeout_s=0.4, min_progress_m=0.02, max_speed_mps=0.20)
    controller = RangeMotionController(cfg)
    controller.start(MotionGoal(direction=MotionDirection.FORWARD, mode=MotionMode.APPROACH, target_clearance_m=0.10), sample(0.0, 0.50))

    controller.update(sample(0.1, 0.499, odom_x=0.0))
    stalled = controller.update(sample(0.6, 0.498, odom_x=0.0))

    assert stalled.stop_reason is StopReason.STALL
    assert stalled.command.linear_x == 0.0
    assert stalled.confidence < 0.5


def test_loaded_rover_startup_slew_is_not_misclassified_as_stall():
    cfg = RangeMotionConfig(
        max_speed_mps=0.10,
        stall_timeout_s=0.75,
        min_progress_m=0.015,
        startup_grace_s=1.75,
        min_odom_progress_m=0.005,
    )
    controller = RangeMotionController(cfg)
    controller.start(
        MotionGoal(
            direction=MotionDirection.FORWARD,
            mode=MotionMode.APPROACH,
            target_clearance_m=0.4572,
            max_measured_displacement_m=0.508,
            timeout_s=30.0,
        ),
        sample(0.0, 0.9480, front=0.82375, odom_x=0.0),
    )

    telemetry = controller.update(sample(1.3396, 0.9470, front=0.81725, odom_x=0.0124479))

    assert telemetry.stop_reason is StopReason.RUNNING
    assert telemetry.measured_displacement_m == pytest.approx(0.0124479)
    assert telemetry.current_clearance_m == pytest.approx(0.9470)
    assert telemetry.command.linear_x > 0.0


def test_safety_stale_jump_unsafe_rear_and_odom_disagreement_fail_closed():
    base_goal = MotionGoal(direction=MotionDirection.FORWARD, mode=MotionMode.APPROACH, target_clearance_m=0.10)

    stale = RangeMotionController(RangeMotionConfig(max_sample_age_s=0.3))
    stale.start(base_goal, sample(0.0, 0.50))
    assert stale.update(sample(0.5, 0.45), now=1.0).stop_reason is StopReason.STALE_SENSOR

    jump = RangeMotionController(RangeMotionConfig(max_range_jump_m=0.20))
    jump.start(base_goal, sample(0.0, 0.50))
    assert jump.update(sample(0.1, 1.00)).stop_reason is StopReason.TARGET_JUMP

    rear = RangeMotionController(RangeMotionConfig(min_rear_clearance_m=0.25))
    rear.start(base_goal, sample(0.0, 0.50, rear=1.0))
    assert rear.update(sample(0.1, 0.49, rear=0.20)).stop_reason is StopReason.UNSAFE_CLEARANCE

    disagree = RangeMotionController(RangeMotionConfig(max_odom_lidar_disagreement_m=0.05))
    disagree.start(base_goal, sample(0.0, 0.50, odom_x=0.0))
    assert disagree.update(sample(1.0, 0.40, odom_x=0.25)).stop_reason is StopReason.ODOM_DISAGREEMENT


def test_stable_surface_tracker_filters_out_single_scan_outlier_and_reports_confidence():
    result = track_stable_surface(
        previous_clearance_m=0.50,
        candidate_clearances_m=[0.12, 0.48, 0.49, 0.50, 0.51, 1.20],
        association_gate_m=0.08,
    )

    assert result.clearance_m == pytest.approx(0.495)
    assert result.confidence > 0.7
    assert result.associated_count == 4


def test_stable_surface_tracker_initializes_from_nearest_compact_cluster_not_single_speckle():
    result = track_stable_surface(
        previous_clearance_m=None,
        candidate_clearances_m=[0.18, 0.48, 0.49, 0.50, 0.51, 1.8, 2.0, 2.2],
        association_gate_m=0.08,
    )

    assert result.clearance_m == pytest.approx(0.495)
    assert result.confidence > 0.4
    assert result.associated_count == 4


def test_target_cluster_tracker_preserves_angular_identity_through_nearby_clutter():
    previous = TargetAssociationState(range_m=0.95, angle_rad=0.02, confidence=0.9)

    result = track_target_cluster(
        previous=previous,
        candidates=(
            AngularRangeCandidate(range_m=0.80, angle_rad=-0.42),
            AngularRangeCandidate(range_m=0.81, angle_rad=-0.39),
            AngularRangeCandidate(range_m=0.946, angle_rad=0.01),
            AngularRangeCandidate(range_m=0.952, angle_rad=0.03),
        ),
        range_gate_m=0.08,
        angle_gate_rad=0.12,
    )

    assert result.range_m == pytest.approx(0.949)
    assert result.angle_rad == pytest.approx(0.02)
    assert result.confidence > 0.7
    assert result.associated_count == 2


def test_controller_uses_target_cluster_identity_instead_of_front_collision_clutter():
    cfg = RangeMotionConfig(max_speed_mps=0.10, startup_grace_s=1.0)
    controller = RangeMotionController(cfg)
    first = controller.start(
        MotionGoal(direction=MotionDirection.FORWARD, mode=MotionMode.APPROACH, target_clearance_m=0.4572),
        clustered_sample(
            0.0,
            (
                AngularRangeCandidate(range_m=0.80, angle_rad=-0.40),
                AngularRangeCandidate(range_m=0.95, angle_rad=0.0),
                AngularRangeCandidate(range_m=0.96, angle_rad=0.03),
            ),
            front=0.81,
        ),
    )

    telemetry = controller.update(
        clustered_sample(
            0.8,
            (
                AngularRangeCandidate(range_m=0.79, angle_rad=-0.41),
                AngularRangeCandidate(range_m=0.94, angle_rad=0.01),
                AngularRangeCandidate(range_m=0.95, angle_rad=0.03),
            ),
            front=0.80,
            odom_x=0.02,
        )
    )

    assert first.current_clearance_m == pytest.approx(0.955)
    assert telemetry.stop_reason is StopReason.RUNNING
    assert telemetry.current_clearance_m == pytest.approx(0.945)
