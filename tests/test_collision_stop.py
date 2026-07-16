import math

import pytest

from sphero_rvr_driver.collision_stop import (
    CollisionStopConfig,
    CollisionStopSupervisor,
    CollisionState,
    ResetPolicy,
    ScanInput,
    TwistCommand,
    evaluate_scan,
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
    )


def test_clear_scan_passes_bounded_command_and_reports_sector_distances():
    cfg = CollisionStopConfig(max_forward_mps=0.10, max_angular_rad_s=0.4)
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    decision = supervisor.update_scan(scan_with(stamp=0.0), now=0.0)

    decision = supervisor.apply_command(TwistCommand(0.5, 1.0), now=0.1)

    assert decision.state is CollisionState.CLEAR
    assert decision.output == TwistCommand(0.10, 0.4)
    assert decision.scan_health.healthy is True
    assert decision.nearest["front"] == pytest.approx(2.0)


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
