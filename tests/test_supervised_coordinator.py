import pytest

from sphero_rvr_driver.range_motion import StopReason
from sphero_rvr_driver.supervised_coordinator import (
    CoordinatorConfig,
    CoordinatorPhase,
    CoordinatorStopReason,
    DeterministicSegmentSelector,
    MissionEvent,
    MissionEventKind,
    SegmentStatus,
    SupervisedCoordinator,
)


def test_coordinator_starts_first_deterministic_bounded_segment_through_range_motion_only():
    selector = DeterministicSegmentSelector(
        clearances_m=(0.55, 0.42, 0.70, 0.80),
        config=CoordinatorConfig(segment_target_clearance_m=0.20, max_segment_displacement_m=0.30),
    )
    coordinator = SupervisedCoordinator(config=CoordinatorConfig(max_segments=3), selector=selector)

    telemetry = coordinator.start(now=10.0)
    command = coordinator.next_range_motion_goal()

    assert telemetry.phase is CoordinatorPhase.RUNNING_SEGMENT
    assert telemetry.active_segment is not None
    assert telemetry.active_segment.index == 0
    assert telemetry.active_segment.direction == "forward"
    assert command is not None
    assert command.channel == "range_motion"
    assert command.topic == "/range_motion/goal"
    assert command.motor_topic is None
    assert command.goal.direction.value == "forward"
    assert command.goal.target_clearance_m == 0.20
    assert command.goal.max_measured_displacement_m == 0.30


def test_coordinator_advances_segments_in_deterministic_order_after_target_reached():
    selector = DeterministicSegmentSelector(
        clearances_m=(0.90, 0.60),
        config=CoordinatorConfig(segment_target_clearance_m=0.25, max_segment_displacement_m=0.40),
    )
    coordinator = SupervisedCoordinator(config=CoordinatorConfig(max_segments=2), selector=selector)
    coordinator.start(now=0.0)

    first_done = coordinator.handle_segment_status(
        SegmentStatus(stop_reason=StopReason.TARGET_REACHED, measured_displacement_m=0.34, current_clearance_m=0.24),
        now=1.0,
    )
    second_command = coordinator.next_range_motion_goal()
    second_done = coordinator.handle_segment_status(
        SegmentStatus(stop_reason=StopReason.TARGET_REACHED, measured_displacement_m=0.12, current_clearance_m=0.24),
        now=2.0,
    )

    assert first_done.phase is CoordinatorPhase.RUNNING_SEGMENT
    assert second_command is not None
    assert second_command.goal.direction.value == "backward"
    assert second_command.goal.max_measured_displacement_m == pytest.approx(0.35)
    assert second_done.phase is CoordinatorPhase.COMPLETE
    assert second_done.stop_reason is CoordinatorStopReason.COMPLETE
    assert second_done.completed_segments == 2
    assert second_done.total_measured_displacement_m == pytest.approx(0.46)


def test_coordinator_fails_closed_and_latches_on_stop_estop_collision_and_shutdown_events():
    for event, expected in [
        (MissionEvent(MissionEventKind.STOP), CoordinatorStopReason.STOP),
        (MissionEvent(MissionEventKind.ESTOP), CoordinatorStopReason.ESTOP),
        (MissionEvent(MissionEventKind.COLLISION_STOP, detail="front_stop"), CoordinatorStopReason.COLLISION_STOP),
        (MissionEvent(MissionEventKind.SHUTDOWN), CoordinatorStopReason.SHUTDOWN),
    ]:
        coordinator = SupervisedCoordinator(
            config=CoordinatorConfig(max_segments=2),
            selector=DeterministicSegmentSelector(clearances_m=(0.8, 0.8, 0.8, 0.8)),
        )
        coordinator.start(now=0.0)

        stopped = coordinator.handle_event(event, now=0.5)
        after_latch = coordinator.handle_segment_status(
            SegmentStatus(stop_reason=StopReason.TARGET_REACHED, measured_displacement_m=0.2, current_clearance_m=0.2),
            now=1.0,
        )

        assert stopped.phase is CoordinatorPhase.FAILED_CLOSED
        assert stopped.stop_reason is expected
        assert stopped.cancellation_required is True
        assert stopped.observable_safety_state == event.kind.value
        assert coordinator.next_range_motion_goal() is None
        assert after_latch.stop_reason is expected
        assert after_latch.phase is CoordinatorPhase.FAILED_CLOSED
        assert after_latch.completed_segments == 0


def test_coordinator_fails_closed_on_range_motion_failure_without_retry_wandering():
    coordinator = SupervisedCoordinator(
        config=CoordinatorConfig(max_segments=5),
        selector=DeterministicSegmentSelector(clearances_m=(0.8, 0.8, 0.8, 0.8)),
    )
    coordinator.start(now=0.0)

    telemetry = coordinator.handle_segment_status(
        SegmentStatus(stop_reason=StopReason.STALE_SENSOR, measured_displacement_m=0.0, current_clearance_m=None),
        now=1.0,
    )

    assert telemetry.phase is CoordinatorPhase.FAILED_CLOSED
    assert telemetry.stop_reason is CoordinatorStopReason.RANGE_MOTION_FAILED
    assert telemetry.range_motion_stop_reason is StopReason.STALE_SENSOR
    assert telemetry.cancellation_required is True
    assert coordinator.next_range_motion_goal() is None
    assert telemetry.completed_segments == 0


def test_coordinator_telemetry_is_mission_api_and_read_only_ui_friendly():
    coordinator = SupervisedCoordinator(
        config=CoordinatorConfig(max_segments=1, mission_id="shoe-map-demo"),
        selector=DeterministicSegmentSelector(clearances_m=(0.60, 0.50, 0.50, 0.50)),
    )

    payload = coordinator.start(now=12.5).to_dict()

    assert payload["mission_id"] == "shoe-map-demo"
    assert payload["phase"] == "RUNNING_SEGMENT"
    assert payload["active_segment"]["direction"] == "forward"
    assert payload["command_path"] == ["range_motion", "/cmd_vel", "collision_stop", "/cmd_vel_motor"]
    assert payload["safety"]["fail_closed"] is True
    assert payload["safety"]["stop_estop_collision_authoritative"] is True
    assert "cmd_vel_motor_publish_allowed" not in payload or payload["cmd_vel_motor_publish_allowed"] is False
