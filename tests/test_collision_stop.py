import math

import pytest

from sphero_rvr_driver.collision_stop import (
    CollisionStopConfig,
    CollisionStopSupervisor,
    CollisionState,
    ResetPolicy,
    ScanInput,
    Transform2D,
    TwistCommand,
    evaluate_scan,
    evaluate_projected_trajectory,
)


def scan_with(front=None, rear=None, left=None, right=None, *, stamp=0.0, count=360):
    ranges = [2.0] * count
    angle_min = -math.pi
    angle_increment = (2.0 * math.pi) / count
    for i in range(count):
        deg = math.degrees(angle_min + i * angle_increment)
        if front is not None and -10 <= deg <= 10:
            ranges[i] = front
        if rear is not None and (deg <= -170 or deg >= 170):
            ranges[i] = rear
        if left is not None and 60 <= deg <= 100:
            ranges[i] = left
        if right is not None and -100 <= deg <= -60:
            ranges[i] = right
    return ScanInput(
        ranges=tuple(ranges),
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.05,
        range_max=8.0,
        stamp=stamp,
        received_at=stamp,
        frame_id="laser",
        transform_to_base=Transform2D(),
    )


def scan_with_point(angle_deg, distance_m, *, stamp=0.0, count=360):
    scan = scan_with(stamp=stamp, count=count)
    ranges = list(scan.ranges)
    index = round((angle_deg - math.degrees(scan.angle_min)) / math.degrees(scan.angle_increment))
    ranges[index % count] = distance_m
    return ScanInput(**{**scan.__dict__, "ranges": tuple(ranges)})


def test_evaluate_scan_uses_base_link_sectors_after_calibrated_pi_lidar_yaw():
    cfg = CollisionStopConfig(min_valid_ranges=1, min_valid_fraction=0.0)
    scan = scan_with(rear=0.30, stamp=1.0)
    scan = ScanInput(**{**scan.__dict__, "transform_to_base": Transform2D(yaw=3.1239668018215028)})

    result = evaluate_scan(scan, cfg, now=1.0)

    assert result.healthy is True
    assert result.nearest["front"] == pytest.approx(0.30)
    assert result.nearest["rear"] == pytest.approx(2.0)
    assert result.health.tf_available is True
    assert result.health.tf_reason == "ok"


def test_nonzero_lidar_yaw_moves_scan_frame_front_out_of_base_link_front_sector():
    cfg = CollisionStopConfig(min_valid_ranges=1, min_valid_fraction=0.0)
    scan = scan_with(front=0.30, stamp=1.0)
    scan = ScanInput(**{**scan.__dict__, "transform_to_base": Transform2D(yaw=math.pi / 2.0)})

    result = evaluate_scan(scan, cfg, now=1.0)

    assert result.healthy is True
    assert result.nearest["front"] == pytest.approx(2.0)
    assert result.nearest["left"] == pytest.approx(0.30)


def test_missing_and_malformed_required_tf_fail_closed_truthfully():
    cfg = CollisionStopConfig(min_valid_ranges=1, min_valid_fraction=0.0, fail_on_missing_tf=True)
    missing = scan_with(stamp=1.0)
    missing = ScanInput(**{**missing.__dict__, "frame_id": "laser", "transform_to_base": None})
    malformed = ScanInput(**{**missing.__dict__, "transform_to_base": Transform2D(yaw=math.nan)})

    missing_result = evaluate_scan(missing, cfg, now=1.0)
    malformed_result = evaluate_scan(malformed, cfg, now=1.0)

    assert missing_result.healthy is False
    assert missing_result.reason == "missing_tf"
    assert missing_result.health.tf_available is False
    assert missing_result.health.tf_reason == "missing_tf"
    assert malformed_result.healthy is False
    assert malformed_result.reason == "malformed_tf"
    assert malformed_result.health.tf_available is False
    assert malformed_result.health.tf_reason == "malformed_tf"


def test_clear_scan_passes_bounded_command_and_reports_sector_distances():
    cfg = CollisionStopConfig(max_forward_mps=0.10, max_angular_rad_s=0.4)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    decision = supervisor.update_scan(scan_with(stamp=0.0), now=0.0)

    decision = supervisor.apply_command(TwistCommand(0.5, 1.0), now=0.1)

    assert decision.state is CollisionState.CLEAR
    assert decision.output == TwistCommand(0.10, 0.4)
    assert decision.scan_health.healthy is True
    assert decision.nearest["front"] == pytest.approx(2.0)


@pytest.mark.parametrize("command", (TwistCommand(math.nan, 0.0), TwistCommand(math.inf, 0.0), TwistCommand(0.0, -math.inf)))
def test_non_finite_collision_stop_command_fails_closed_to_zero(command):
    cfg = CollisionStopConfig(max_forward_mps=0.10, max_angular_rad_s=0.4)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    supervisor.update_scan(scan_with(stamp=0.0), now=0.0)

    decision = supervisor.apply_command(command, now=0.1)

    assert decision.state is CollisionState.STOPPED
    assert decision.reason == "non_finite_command"
    assert decision.output == TwistCommand(0.0, 0.0)
    assert decision.reset_required is True


def test_non_finite_collision_command_fails_closed_without_motion():
    supervisor = CollisionStopSupervisor(CollisionStopConfig(), now=0.0)
    supervisor.update_scan(scan_with(stamp=0.0), now=0.0)

    for value in (math.nan, math.inf, -math.inf):
        decision = supervisor.apply_command(TwistCommand(value, value), now=0.1)

        assert decision.state is CollisionState.STOPPED
        assert decision.reason == "non_finite_command"
        assert decision.output == TwistCommand(0.0, 0.0)


def test_collision_stop_uses_footprint_payload_and_speed_dependent_braking_distance():
    cfg = CollisionStopConfig(
        min_valid_ranges=1,
        min_valid_fraction=0.0,
        stop_distance_m=0.10,
        slow_distance_m=0.60,
        footprint_front_m=0.30,
        payload_margin_m=0.10,
        measured_stop_time_s=1.0,
        braking_distance_margin_m=0.0,
        max_forward_mps=0.10,
    )
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    supervisor.update_scan(scan_with(front=0.49, stamp=0.0), now=0.0)

    decision = supervisor.apply_command(TwistCommand(0.10, 0.0), now=0.0)

    assert decision.state is CollisionState.STOPPED
    assert decision.reason == "front_stop"
    assert decision.output == TwistCommand(0.0, 0.0)


def test_missing_stale_and_malformed_scans_fail_closed():
    cfg = CollisionStopConfig(startup_grace_s=0.5, max_scan_age_s=0.3)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)

    assert supervisor.apply_command(TwistCommand(0.1, 0.0), now=0.1).output == TwistCommand(0.0, 0.0)
    stale = supervisor.apply_command(TwistCommand(0.1, 0.0), now=0.6)
    assert stale.state is CollisionState.SENSOR_STALE
    assert stale.reason == "missing_scan"

    bad = evaluate_scan(
        ScanInput(ranges=(1.0,), angle_min=0.0, angle_increment=0.0, range_min=0.0, range_max=6.0, stamp=1.0, received_at=1.0),
        cfg,
        now=1.0,
    )
    assert bad.healthy is False
    assert bad.reason == "angle_increment"


def test_scan_freshness_separates_receipt_age_from_acquisition_start_stamp():
    cfg = CollisionStopConfig(max_scan_age_s=0.3, max_scan_stamp_age_s=0.75)
    acquiring = scan_with(stamp=10.0)
    acquiring = ScanInput(**{**acquiring.__dict__, "received_at": 10.34})

    fresh = evaluate_scan(acquiring, cfg, now=10.34)
    receipt_stale = evaluate_scan(acquiring, cfg, now=10.65)
    stamp_stale = evaluate_scan(
        ScanInput(**{**acquiring.__dict__, "received_at": 10.76}),
        cfg,
        now=10.76,
    )

    assert fresh.healthy is True
    assert fresh.health.age_s == pytest.approx(0.0)
    assert fresh.health.stamp_age_s == pytest.approx(0.34)
    assert receipt_stale.healthy is False
    assert receipt_stale.reason == "stale_scan"
    assert stamp_stale.healthy is False
    assert stamp_stale.reason == "stale_scan_stamp"


def test_scan_freshness_rejects_invalid_threshold_relationship():
    with pytest.raises(
        ValueError,
        match="max_scan_stamp_age_s must be at least max_scan_age_s",
    ):
        CollisionStopConfig(max_scan_age_s=0.3, max_scan_stamp_age_s=0.2)


def test_scan_freshness_rejects_frozen_or_replayed_source_stamps():
    supervisor = CollisionStopSupervisor(CollisionStopConfig(), now=10.0)
    first = scan_with(stamp=10.0)
    repeated = ScanInput(**{**first.__dict__, "received_at": 10.1})

    assert supervisor.update_scan(first, now=10.0).state is CollisionState.CLEAR
    frozen = supervisor.update_scan(repeated, now=10.1)

    assert frozen.state is CollisionState.SENSOR_STALE
    assert frozen.reason == "non_advancing_scan_stamp"
    assert frozen.output == TwistCommand()


@pytest.mark.parametrize(
    ("stamp", "received_at", "reason"),
    (
        (math.nan, 1.0, "stale_scan_stamp"),
        (1.0, math.nan, "stale_scan"),
    ),
)
def test_scan_freshness_rejects_non_finite_timestamps(stamp, received_at, reason):
    scan = scan_with(stamp=1.0)
    result = evaluate_scan(
        ScanInput(**{**scan.__dict__, "stamp": stamp, "received_at": received_at}),
        CollisionStopConfig(),
        now=1.0,
    )

    assert result.healthy is False
    assert result.reason == reason


def test_invalid_unknown_front_sector_is_blocked_not_clear():
    cfg = CollisionStopConfig(min_valid_ranges=12, min_valid_fraction=0.05)
    scan = scan_with(stamp=0.0)
    ranges = list(scan.ranges)
    for i in range(len(ranges)):
        deg = math.degrees(scan.angle_min + i * scan.angle_increment)
        if -30 <= deg <= 30:
            ranges[i] = math.nan
    result = evaluate_scan(ScanInput(**{**scan.__dict__, "ranges": tuple(ranges)}), cfg, now=0.0)

    assert result.healthy is False
    assert result.reason == "front_unknown"


def test_front_stop_latches_until_clear_release_time_and_manual_reset():
    cfg = CollisionStopConfig(reset_policy=ResetPolicy.MANUAL, release_time_s=0.5)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)

    supervisor.update_scan(scan_with(front=0.30, stamp=0.0), now=0.0)
    stopped = supervisor.apply_command(TwistCommand(0.1, 0.0), now=0.0)
    assert stopped.state is CollisionState.STOPPED
    assert stopped.output == TwistCommand(0.0, 0.0)

    supervisor.update_scan(scan_with(front=0.50, stamp=0.1), now=0.1)
    early = supervisor.reset(now=0.4)
    assert early.accepted is False
    assert early.decision.state is CollisionState.STOPPED

    supervisor.update_scan(scan_with(front=0.50, stamp=0.7), now=0.7)
    accepted = supervisor.reset(now=0.7)
    assert accepted.accepted is True
    assert accepted.decision.state is CollisionState.CLEAR
    assert accepted.decision.output == TwistCommand(0.0, 0.0)


def test_slow_band_scales_forward_without_changing_turn():
    cfg = CollisionStopConfig(stop_distance_m=0.35, slow_distance_m=0.60, min_forward_scale=0.0)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    supervisor.update_scan(scan_with(front=0.475, stamp=0.0), now=0.0)

    decision = supervisor.apply_command(TwistCommand(0.10, 0.2), now=0.0)

    assert decision.state is CollisionState.SLOW
    assert decision.output.linear_x == pytest.approx(0.05)
    assert decision.output.angular_z == pytest.approx(0.2)


def test_reverse_and_side_obstacles_hold_unsafe_components():
    supervisor = CollisionStopSupervisor(CollisionStopConfig(), now=0.0)
    supervisor.update_scan(scan_with(rear=0.20, left=0.20, right=0.20, stamp=0.0), now=0.0)

    assert supervisor.apply_command(TwistCommand(-0.1, 0.0), now=0.0).output.linear_x == 0.0
    assert supervisor.apply_command(TwistCommand(0.0, 0.3), now=0.0).output.angular_z == 0.0
    assert supervisor.apply_command(TwistCommand(0.0, -0.3), now=0.0).output.angular_z == 0.0


def test_projected_turn_allows_rear_left_point_outside_actual_swept_path():
    cfg = CollisionStopConfig()
    scan = scan_with_point(134.0, 0.316, stamp=0.0)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    supervisor.update_scan(scan, now=0.0)

    decision = supervisor.apply_command(TwistCommand(0.0, math.radians(30.0)), now=0.0)

    assert decision.state is CollisionState.CLEAR
    assert decision.output == TwistCommand(0.0, cfg.max_angular_rad_s)
    assert decision.trajectory is not None
    assert decision.trajectory.blocked is False
    assert decision.trajectory.horizon_s == pytest.approx(
        cfg.requested_cmd_timeout_s + cfg.measured_stop_time_s
    )
    assert decision.trajectory.minimum_clearance_m is not None
    assert decision.trajectory.minimum_clearance_m > 0.0


def test_projected_forward_motion_ignores_obstacle_behind_current_trajectory():
    cfg = CollisionStopConfig()
    scan = scan_with_point(134.0, 0.316, stamp=0.0)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    supervisor.update_scan(scan, now=0.0)

    decision = supervisor.apply_command(TwistCommand(0.08, 0.0), now=0.0)

    assert decision.state is CollisionState.CLEAR
    assert decision.output == TwistCommand(0.08, 0.0)
    assert decision.trajectory is not None
    assert decision.trajectory.blocked is False


def test_projected_forward_motion_blocks_obstacle_entering_corner_sweep():
    cfg = CollisionStopConfig()
    scan = scan_with_point(31.0, 0.30, stamp=0.0)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    supervisor.update_scan(scan, now=0.0)

    decision = supervisor.apply_command(TwistCommand(0.08, 0.0), now=0.0)

    assert decision.state is CollisionState.SLOW
    assert decision.reason == "forward_trajectory_blocked"
    assert decision.output == TwistCommand()
    assert decision.trajectory is not None
    assert decision.trajectory.blocked is True


@pytest.mark.parametrize(
    ("angle_deg", "angular_z", "reason"),
    (
        (50.0, 0.4, "left_trajectory_blocked"),
        (-50.0, -0.4, "right_trajectory_blocked"),
    ),
)
def test_projected_turn_blocks_only_obstacle_entering_swept_footprint(
    angle_deg,
    angular_z,
    reason,
):
    cfg = CollisionStopConfig()
    scan = scan_with_point(angle_deg, 0.27, stamp=0.0)
    projected = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(0.0, angular_z),
    )
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    supervisor.update_scan(scan, now=0.0)

    decision = supervisor.apply_command(TwistCommand(0.0, angular_z), now=0.0)

    assert projected.blocked is True
    assert projected.collision_time_s is not None
    assert 0.0 < projected.collision_time_s <= projected.horizon_s
    assert decision.state is CollisionState.SLOW
    assert decision.reason == reason
    assert decision.output == TwistCommand()
    assert decision.trajectory == projected


def test_trajectory_margin_and_footprint_validation_fail_closed_at_configuration():
    with pytest.raises(ValueError, match="trajectory_clearance_margin_m"):
        CollisionStopConfig(trajectory_clearance_margin_m=-0.01)
    with pytest.raises(ValueError, match="trajectory_clearance_margin_m"):
        CollisionStopConfig(trajectory_clearance_margin_m=math.nan)
    with pytest.raises(ValueError, match="footprint_front_m"):
        CollisionStopConfig(footprint_front_m=0.0)


def test_stop_estop_clear_estop_do_not_replay_old_command():
    supervisor = CollisionStopSupervisor(CollisionStopConfig(reset_policy=ResetPolicy.AUTO_AFTER_CLEAR), now=0.0)
    supervisor.update_scan(scan_with(stamp=0.0), now=0.0)
    supervisor.apply_command(TwistCommand(0.1, 0.0), now=0.0)

    stop = supervisor.stop(now=0.1)
    assert stop.output == TwistCommand(0.0, 0.0)
    assert supervisor.apply_command(TwistCommand(0.1, 0.0), now=0.2).output == TwistCommand(0.0, 0.0)

    estop = supervisor.estop(now=0.3)
    assert estop.state is CollisionState.ESTOPPED
    assert supervisor.apply_command(TwistCommand(0.1, 0.0), now=0.4).output == TwistCommand(0.0, 0.0)

    cleared = supervisor.clear_estop(now=0.5)
    assert cleared.state is CollisionState.SENSOR_STALE
    assert cleared.output == TwistCommand(0.0, 0.0)


def test_stale_command_outputs_zero_before_driver_timeout():
    supervisor = CollisionStopSupervisor(CollisionStopConfig(requested_cmd_timeout_s=0.25), now=0.0)
    supervisor.update_scan(scan_with(stamp=0.0), now=0.0)
    supervisor.apply_command(TwistCommand(0.1, 0.0), now=0.0)

    decision = supervisor.tick(now=0.3)

    assert decision.state is CollisionState.CLEAR
    assert decision.reason == "stale_command"
    assert decision.output == TwistCommand(0.0, 0.0)
