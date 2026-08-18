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


def geometry_config(**kwargs):
    """A config whose FOOTPRINT IS STATED, for the tests that place obstacles relative
    to it.

    These are mechanism tests: they check that the trajectory projection blocks a point
    entering the swept rectangle and clears one that recedes, with obstacles hand-placed
    a few centimetres outside a specific footprint. They used bare `CollisionStopConfig()`
    and so silently inherited whatever the dataclass defaults happened to be -- which
    made them tests of the mechanism AND of the defaults at once, without saying so.

    On 2026-08-15 the defaults were corrected to Scott's measured extents (front 0.22 ->
    0.0965, lateral 0.14 -> 0.098/0.106, payload 0.05 -> 0.02) and eight of these went
    red: the robot had shrunk out from under points chosen for a larger one. Nothing was
    wrong with the mechanism.

    The numbers below are the OLD defaults, preserved deliberately so each test keeps
    exactly the geometry it was written against. They describe no real robot and are not
    meant to -- which is the point of naming them here instead of inheriting them.
    """
    return CollisionStopConfig(
        footprint_front_m=0.22, footprint_rear_m=0.16,
        footprint_left_m=0.14, footprint_right_m=0.14,
        payload_margin_m=0.05, **kwargs,
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


def test_front_stop_allows_only_rear_clear_supervised_reverse_escape():
    supervisor = CollisionStopSupervisor(geometry_config(), now=0.0)
    supervisor.update_scan(
        # Put the front return inside the expanded projected footprint. This
        # proves the same known point that latches STOP is excluded only from
        # the bounded reverse trajectory that increases its separation.
        scan_with(front=0.15, rear=1.20, stamp=0.0),
        now=0.0,
    )
    stopped = supervisor.apply_command(
        TwistCommand(0.10, 0.0),
        now=0.0,
    )
    assert stopped.state is CollisionState.STOPPED
    assert stopped.reason == "front_stop"

    supervisor.update_scan(
        scan_with(front=0.15, rear=1.20, stamp=0.1),
        now=0.1,
    )
    escaped = supervisor.apply_command(
        TwistCommand(-0.10, 0.0),
        now=0.1,
    )

    assert escaped.state is CollisionState.SLOW
    assert escaped.reason == "front_stop_reverse_escape"
    assert escaped.output == TwistCommand(-0.10, 0.0)
    assert escaped.trajectory is not None
    assert escaped.trajectory.blocked is False
    assert escaped.trajectory.moving_away_point_count > 0


def test_front_stop_reverse_escape_remains_zero_when_rear_is_blocked():
    supervisor = CollisionStopSupervisor(CollisionStopConfig(), now=0.0)
    supervisor.update_scan(
        scan_with(front=0.30, rear=0.20, stamp=0.0),
        now=0.0,
    )
    supervisor.apply_command(TwistCommand(0.10, 0.0), now=0.0)
    supervisor.update_scan(
        scan_with(front=0.30, rear=0.20, stamp=0.1),
        now=0.1,
    )

    held = supervisor.apply_command(TwistCommand(-0.10, 0.0), now=0.1)

    assert held.state is CollisionState.STOPPED
    assert held.reason == "reverse_escape_rear_blocked"
    assert held.output == TwistCommand()
    assert held.reset_required is True


def test_nonfinite_stop_latch_cannot_be_released_by_reverse_escape():
    supervisor = CollisionStopSupervisor(CollisionStopConfig(), now=0.0)
    supervisor.update_scan(
        scan_with(front=1.20, rear=1.20, stamp=0.0),
        now=0.0,
    )
    stopped = supervisor.apply_command(
        TwistCommand(math.nan, 0.0),
        now=0.0,
    )
    assert stopped.reason == "non_finite_command"

    held = supervisor.apply_command(TwistCommand(-0.10, 0.0), now=0.1)

    assert held.state is CollisionState.STOPPED
    assert held.reason == "reset_required"
    assert held.output == TwistCommand()


def test_slow_band_scales_forward_without_changing_turn():
    cfg = CollisionStopConfig(stop_distance_m=0.35, slow_distance_m=0.60, min_forward_scale=0.0)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    supervisor.update_scan(scan_with(front=0.475, stamp=0.0), now=0.0)

    decision = supervisor.apply_command(TwistCommand(0.10, 0.2), now=0.0)

    assert decision.state is CollisionState.SLOW
    assert decision.output.linear_x == pytest.approx(0.05)
    assert decision.output.angular_z == pytest.approx(0.2)


def test_reverse_and_side_obstacles_hold_unsafe_components():
    supervisor = CollisionStopSupervisor(geometry_config(), now=0.0)
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
    # 2026-08-18 clamp split: a pure pivot under the curve ceiling now passes AT ITS
    # COMMANDED RATE (the old `== max_angular_rad_s` here was the blind clamp this
    # test never meant to assert -- its subject is the trajectory projection below).
    assert decision.output == TwistCommand(0.0, math.radians(30.0))
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


def test_projected_motion_is_directional_for_a_known_overlapped_rear_point():
    cfg = CollisionStopConfig()
    # This models the surveyed M7.3 box return: it intersects the lidar-height
    # rear-left rectangle at the current pose.
    scan = scan_with_point(135.0, 0.15, stamp=0.0)

    forward = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(0.08, 0.0),
    )
    reverse = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(-0.08, 0.0),
    )

    assert forward.blocked is False
    assert forward.moving_away_point_count == 1
    assert reverse.blocked is True
    assert reverse.collision_time_s == pytest.approx(0.0)


def test_projected_turn_in_place_is_directional_next_to_overlapped_point():
    cfg = CollisionStopConfig()
    scan = scan_with_point(135.0, 0.15, stamp=0.0)

    turning_toward = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(0.0, -0.1),
    )
    turning_away = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(0.0, 0.1),
    )

    assert turning_toward.blocked is True
    assert turning_toward.collision_time_s == pytest.approx(0.0)
    assert turning_away.blocked is False
    assert turning_away.moving_away_point_count == 1


def test_projected_curved_escape_requires_sampled_clearance_to_recede():
    cfg = geometry_config()
    # With the point directly beside the rover, forward translation alone is
    # tangential and cannot use the radial shortcut. The left curve qualifies
    # only because every sampled footprint clearance increases; the mirrored
    # curve approaches the same point and must remain blocked.
    scan = scan_with_point(90.0, 0.15, stamp=0.0)

    curving_away = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(0.08, 0.2),
    )
    curving_toward = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(0.08, -0.2),
    )

    assert curving_away.blocked is False
    assert curving_away.moving_away_point_count == 1
    assert curving_toward.blocked is True
    assert curving_toward.collision_time_s == pytest.approx(0.0)


def test_projected_motion_is_directional_for_a_known_overlapped_front_point():
    cfg = geometry_config()
    scan = scan_with_point(0.0, 0.15, stamp=0.0)

    forward = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(0.08, 0.0),
    )
    reverse = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(-0.08, 0.0),
    )

    assert forward.blocked is True
    assert forward.collision_time_s == pytest.approx(0.0)
    assert reverse.blocked is False
    assert reverse.moving_away_point_count == 1


def test_tangential_motion_does_not_claim_to_move_away_from_known_obstacle():
    cfg = geometry_config()
    scan = scan_with_point(90.0, 0.15, stamp=0.0)

    projected = evaluate_projected_trajectory(
        scan,
        cfg,
        TwistCommand(0.08, 0.0),
    )

    assert projected.blocked is True
    assert projected.collision_time_s == pytest.approx(0.0)
    assert projected.moving_away_point_count == 0


def test_projected_trajectory_fails_closed_when_every_point_would_be_excluded():
    cfg = CollisionStopConfig()
    scan = scan_with_point(135.0, 0.15, stamp=0.0)
    only_overlapped_point = ScanInput(
        **{
            **scan.__dict__,
            "ranges": tuple(
                value if value < 1.0 else math.inf
                for value in scan.ranges
            ),
        }
    )

    projected = evaluate_projected_trajectory(
        only_overlapped_point,
        cfg,
        TwistCommand(0.08, 0.0),
    )

    assert projected.blocked is True
    assert projected.collision_time_s == pytest.approx(0.0)
    assert projected.minimum_clearance_m is None
    assert projected.moving_away_point_count == 1


def test_projected_forward_motion_blocks_obstacle_entering_corner_sweep():
    cfg = geometry_config()
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
    cfg = geometry_config()
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


def _front_stopped(front_m):
    """A supervisor latched in a front stop, with the DEPLOYED footprint."""
    cfg = CollisionStopConfig(
        max_forward_mps=0.20, max_angular_rad_s=0.4,
        stop_distance_m=0.30, slow_distance_m=0.50, release_distance_m=0.40,
        # All four extents, from YAML. Omitting the lateral pair silently falls back
        # to 0.14 and quietly changes the robot the test is describing -- which is how
        # the first version of this test came to certify a collision as acceptable.
        footprint_front_m=0.11, footprint_rear_m=0.16,
        footprint_left_m=0.10, footprint_right_m=0.10, payload_margin_m=0.02,
        reset_policy=ResetPolicy.AUTO_AFTER_CLEAR,
    )
    sup = CollisionStopSupervisor(cfg, now=0.0)
    sup.update_scan(scan_with(front=front_m, rear=2.0, left=2.0, right=2.0, stamp=0.0), now=0.0)
    sup.apply_command(TwistCommand(0.2, 0.0), now=0.1)
    assert sup.state is CollisionState.STOPPED
    return sup


# The circle a pivot actually sweeps: hypot(max(front,rear), max(left,right)) + margin
# = hypot(0.16, 0.10) + 0.02 = 0.2087 m.
SWEPT_RADIUS = math.hypot(0.16, 0.10) + 0.02


def test_front_stop_allows_turning_toward_open_floor():
    """A latched front stop used to zero rotation as well as translation, leaving the
    rover facing an obstacle unable to turn toward open floor beside it (2026-08-09:
    stopped 0.28 m from an obstacle with 0.93 m clear on its left, stuck until the
    mission gave up). Forward stays zeroed; rotation gets out."""
    sup = _front_stopped(0.25)                       # stopped, but outside the sweep
    assert 0.25 > SWEPT_RADIUS
    decision = sup.apply_command(TwistCommand(0.0, 0.4), now=0.2)
    assert decision.output.linear_x == 0.0           # forward still dead
    assert decision.output.angular_z != 0.0          # but it can turn


def test_pivot_refused_when_the_obstacle_is_inside_the_swept_circle():
    """The pivot gate must measure the circle a pivot really sweeps. It first used
    max(front, rear) + margin = 0.180 m, measuring a rectangle along one axis, and
    granted pivots at 0.181-0.209 m that sweep the corner straight through the
    obstacle."""
    sup = _front_stopped(0.20)                       # inside the swept circle
    assert 0.20 < SWEPT_RADIUS
    decision = sup.apply_command(TwistCommand(0.0, 0.4), now=0.2)
    assert decision.output.angular_z == 0.0
    assert decision.state is CollisionState.STOPPED


def test_pivot_refused_for_an_obstacle_between_the_named_sectors():
    """front/rear/left/right leave 60 deg unread, and the footprint corners sit at
    +/-42.3 and +/-148.0 deg -- squarely in those gaps. An obstacle at a corner
    bearing must still refuse the pivot."""
    cfg = CollisionStopConfig(
        max_forward_mps=0.20, max_angular_rad_s=0.4,
        stop_distance_m=0.30, slow_distance_m=0.50, release_distance_m=0.40,
        footprint_front_m=0.11, footprint_rear_m=0.16,
        footprint_left_m=0.10, footprint_right_m=0.10, payload_margin_m=0.02,
        reset_policy=ResetPolicy.AUTO_AFTER_CLEAR,
    )
    sup = CollisionStopSupervisor(cfg, now=0.0)
    # 0.09 m at ~143 deg: rear-left corner, inside every sector's blind gap.
    ranges = [2.0] * 360
    for i in range(360):
        deg = math.degrees(-math.pi + i * (2.0 * math.pi / 360))
        if 137.0 <= deg <= 149.0:
            ranges[i] = 0.09
        if -10.0 <= deg <= 10.0:
            ranges[i] = 0.20
    scan = ScanInput(ranges=tuple(ranges), angle_min=-math.pi,
                     angle_increment=(2.0 * math.pi) / 360, range_min=0.05,
                     range_max=8.0, stamp=0.0, received_at=0.0, frame_id="laser",
                     transform_to_base=Transform2D())
    sup.update_scan(scan, now=0.0)
    sup.apply_command(TwistCommand(0.2, 0.0), now=0.1)
    decision = sup.apply_command(TwistCommand(0.0, 0.4), now=0.2)
    assert decision.output.angular_z == 0.0, "pivot granted into a corner obstacle"


def test_front_stop_does_not_synthesize_a_pivot_from_an_amputated_arc():
    """D25. A front stop zeroes forward motion. It must not then hand back the
    residual angular of an ARC, because that turns "drive forward while curving
    left" into "pivot in place" -- a motion the caller never asked for.

    Measured in the field 2026-08-10 at 14:29:01: the controller commanded a forward
    arc (0.20, -0.80) into a chair, the front stop correctly zeroed linear, and the
    escape passed the rotation through. The rover ground against the chair for 2.5 s
    -- audible strain, tracks slipping, reported yaw swinging +/-40 deg per 0.3 s
    while position never changed -- until the back-off clock expired and reversed it
    out cleanly on the first attempt.

    The obstacle here sits OUTSIDE the swept circle, so the geometry gates would all
    permit this rotation; only the pure-pivot condition refuses it. That is what makes
    this a real regression test rather than a restatement of D17/D18.
    """
    sup = _front_stopped(0.25)
    assert 0.25 > SWEPT_RADIUS, "obstacle must be outside the sweep, or D18 refuses it"
    decision = sup.apply_command(TwistCommand(0.2, 0.4), now=0.2)
    assert decision.output.linear_x == 0.0, "the front stop must still kill forward"
    assert decision.output.angular_z == 0.0, (
        "an arc whose forward half was amputated must NOT become an in-place pivot"
    )


def test_front_stop_still_grants_a_deliberate_pure_pivot():
    """The other half of D25: a caller that genuinely wants to rotate out of a stop
    sends a PURE pivot, and must still get it. Same supervisor, same scan, same
    obstacle -- only the request differs. Without this pairing the fix above could be
    'refuse all rotation', which is the D17-era bug that stranded the rover."""
    sup = _front_stopped(0.25)
    decision = sup.apply_command(TwistCommand(0.0, 0.4), now=0.2)
    assert decision.output.linear_x == 0.0
    assert decision.output.angular_z != 0.0


def _latched_by(reason_command, front_m=2.0):
    """A supervisor latched STOPPED by something OTHER than a front stop."""
    cfg = CollisionStopConfig(
        max_forward_mps=0.20, max_angular_rad_s=0.4,
        stop_distance_m=0.30, slow_distance_m=0.50, release_distance_m=0.40,
        footprint_front_m=0.11, footprint_rear_m=0.16,
        footprint_left_m=0.10, footprint_right_m=0.10, payload_margin_m=0.02,
        reset_policy=ResetPolicy.AUTO_AFTER_CLEAR,
    )
    sup = CollisionStopSupervisor(cfg, now=0.0)
    sup.update_scan(scan_with(front=front_m, rear=2.0, left=2.0, right=2.0, stamp=0.0),
                    now=0.0)
    sup.apply_command(reason_command, now=0.1)
    assert sup.state is CollisionState.STOPPED
    return sup


def test_reverse_arc_under_a_non_front_stop_latch_does_not_become_a_pivot():
    """D25, second door. The pure-pivot condition was first written `<= 0.0` on the
    reasoning that a reverse arc could never reach the turn escape, because the
    reverse-escape branch returns first for linear_x < 0. That reasoning omitted
    that branch's OTHER precondition: it only fires when the latch is `front_stop`.

    Under a `non_finite_command` latch the reverse-escape branch is skipped, a
    reverse arc falls through to the turn escape, and `<= 0.0` handed the rotation
    back -- output (0.0, 0.4), the same synthesized motion entered by another door.

    MUST FAIL against the `<= 0.0` version.
    """
    sup = _latched_by(TwistCommand(float("nan"), 0.0))
    decision = sup.apply_command(TwistCommand(-0.15, 0.5), now=0.2)
    assert decision.output.linear_x == 0.0
    assert decision.output.angular_z == 0.0, (
        "a reverse ARC must not be turned into an in-place pivot, whatever latched "
        "the stop"
    )


def test_pure_pivot_under_a_non_front_stop_latch_is_still_granted():
    """The pairing again: the fix must stay a scalpel. A deliberate pure pivot is
    still allowed to rotate out from under a non-front-stop latch."""
    sup = _latched_by(TwistCommand(float("nan"), 0.0))
    decision = sup.apply_command(TwistCommand(0.0, 0.4), now=0.2)
    assert decision.output.angular_z != 0.0


# --- the clamp seam, split by path (D45 wearing supervisor clothes, 2026-08-18) ------
#
# Field receipt (run 3c, goals 3/4): RPP commanded pivots at 3.55 rad/s, _bound
# squashed them to 0.40 on /cmd_vel_motor, and the driver's plan_pivot raised them
# straight back to the 3.55 floor -- the wire lied while the wheels did the right
# thing, and every commanded pivot rate in (0.4, 5.83] collapsed to the floor. The
# clamp governed a path the command never took; these tests hold the split.

def _clear_supervisor(**kwargs):
    cfg = CollisionStopConfig(max_forward_mps=0.10, max_angular_rad_s=0.4, **kwargs)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    supervisor.update_scan(scan_with(stamp=0.0), now=0.0)
    return supervisor


def test_a_pure_pivot_passes_at_its_commanded_rate_not_the_arc_cap():
    """The goal-3 field receipt as a test: (0, 3.55) must leave the supervisor at
    3.55 -- the rate RPP asked for and the drivetrain measurably produces -- not
    0.40, which the driver would silently raise back to the floor anyway."""
    supervisor = _clear_supervisor()
    decision = supervisor.apply_command(TwistCommand(0.0, 3.55), now=0.1)
    assert decision.state is CollisionState.CLEAR
    assert decision.output.angular_z == pytest.approx(3.55)


def test_a_pivot_above_the_curves_ceiling_gets_the_curves_cap_not_the_arcs():
    """(0, 9.0) clamps to maximum_clean_rate(pivot_max_duty) ~ 5.834 -- the fastest
    CLEAN pivot the deployed band produces (run 4: 5.852 rad/s measured) -- and
    emphatically not to 0.4."""
    from sphero_rvr_core.pivot_curve import maximum_clean_rate

    supervisor = _clear_supervisor()
    decision = supervisor.apply_command(TwistCommand(0.0, 9.0), now=0.1)
    assert decision.output.angular_z == pytest.approx(maximum_clean_rate(45))
    assert decision.output.angular_z > 0.4


def test_an_arc_keeps_the_unmeasured_arc_authority():
    """(0.1, 3.55) is an ARC: its angular clamps to max_angular_rad_s = 0.4 exactly
    as before the split. The arc regime is UNMEASURED (curve was measured on
    in-place pivots only; see safety.clamp_velocity_for_path) and its limit does not
    move until the arc-rate run card closes the gap."""
    supervisor = _clear_supervisor()
    decision = supervisor.apply_command(TwistCommand(0.1, 3.55), now=0.1)
    assert decision.output.angular_z == pytest.approx(0.4)
    assert decision.output.linear_x == pytest.approx(0.1)


def test_a_sub_epsilon_linear_is_bounded_as_the_pivot_the_driver_will_run():
    """(0.0005, 3.55): the driver's own path test (is_pivot_command with
    PIVOT_LINEAR_EPSILON_MPS = 0.005) sends this down the pivot branch, so _bound
    must clamp it as a pivot. Clamping it as an arc would govern a path the command
    never takes -- D45's sentence, the supervisor edition."""
    supervisor = _clear_supervisor()
    decision = supervisor.apply_command(TwistCommand(0.0005, 3.55), now=0.1)
    assert decision.output.angular_z == pytest.approx(3.55)


def test_the_stopped_state_grant_rule_is_untouched_by_the_bound_split():
    """_bound's epsilon is for CEILINGS; the STOPPED-state rotation GRANT keeps
    D25's exact-zero rule. A sub-epsilon linear command under a stop must NOT have
    its rotation granted as if it were a pure pivot -- that is the amputated-arc
    synthesis the grant rule exists to refuse."""
    supervisor = _clear_supervisor()
    supervisor.update_scan(scan_with(front=0.2, stamp=0.2), now=0.2)
    decision = supervisor.apply_command(TwistCommand(0.0005, 3.55), now=0.3)
    assert decision.state is CollisionState.STOPPED
    assert decision.output == TwistCommand(0.0, 0.0)


def test_the_cross_config_guard_the_literal_exists_only_while_this_proves_it():
    """THE HOUSE SOLUTION TO THE TWO-AUTHORITIES TRAP: collision_stop.yaml carries
    max_pivot_rate_rad_s as a literal because the supervisor cannot read the
    driver's config at runtime -- and that literal is allowed to exist ONLY while
    this test proves it equals the curve's answer for the duty band the DRIVER
    actually deploys (lean_rvr_tank_si.yaml's pivot_max_duty). Change the band and
    this fails a test, not a flight."""
    from pathlib import Path

    import yaml

    from sphero_rvr_core.pivot_curve import maximum_clean_rate

    root = Path(__file__).resolve().parents[1]
    sup = yaml.safe_load((root / "config" / "collision_stop.yaml").read_text())
    rvr = yaml.safe_load((root / "config" / "lean_rvr_tank_si.yaml").read_text())

    def find(tree, key):
        if isinstance(tree, dict):
            if key in tree:
                return tree[key]
            for v in tree.values():
                got = find(v, key)
                if got is not None:
                    return got
        return None

    declared = find(sup, "max_pivot_rate_rad_s")
    duty = find(rvr, "pivot_max_duty")
    assert declared is not None, "supervisor yaml lost its pivot ceiling"
    assert duty is not None, "driver yaml lost pivot_max_duty"
    assert declared == pytest.approx(maximum_clean_rate(int(duty)), abs=1e-4), (
        f"collision_stop.yaml says {declared} but the curve says "
        f"{maximum_clean_rate(int(duty)):.5f} for the deployed duty {duty} -- the "
        f"literal has come loose from its author"
    )
