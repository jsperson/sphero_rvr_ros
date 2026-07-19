from __future__ import annotations

import pytest

from sphero_rvr_driver.mission_api import (
    ArtifactContract,
    CapabilitySet,
    MissionApiVersion,
    MissionCommand,
    MissionEventKind,
    MissionRequest,
    MissionResultStatus,
    MissionState,
    MissionStateMachine,
    MissionValidationError,
    SafetyContract,
    build_canonical_shoe_mapping_request,
    validate_mission_request,
)
from sphero_rvr_driver.range_motion import StopReason


def test_canonical_shoe_mapping_mission_validates_to_versioned_contract() -> None:
    request = build_canonical_shoe_mapping_request(mission_id="mission-001")
    capabilities = CapabilitySet(
        semantic_mapping=True,
        slam_replay_or_live_mapping=True,
        shoe_detection=True,
        supervised_motion=True,
        collision_stop=True,
        estop=True,
        artifacts=True,
    )

    command = validate_mission_request(request, capabilities)

    assert command.api_version is MissionApiVersion.V1
    assert command.mission_type == "semantic_room_shoe_mapping"
    assert command.mission_id == "mission-001"
    assert command.room_mapping.map_name == "shoe_room_map"
    assert command.room_mapping.semantic_labels == ("shoe",)
    assert command.room_mapping.require_artifact_references is True
    assert command.capability_checks == capabilities
    assert command.safety == SafetyContract(
        start_requires_supervised_motion=True,
        cancel_supported=True,
        estop_supported=True,
        max_runtime_s=600.0,
        max_segments=8,
        allow_direct_ros_commands=False,
    )
    assert ArtifactContract("occupancy_map", "application/x-yaml", required=True) in command.artifacts
    assert ArtifactContract("semantic_map", "application/json", required=True) in command.artifacts
    assert ArtifactContract("shoe_detections", "application/json", required=True) in command.artifacts


def test_validation_rejects_unsafe_or_generic_ros_command_requests() -> None:
    capabilities = CapabilitySet.all_enabled()
    request = MissionRequest(
        api_version=MissionApiVersion.V1,
        mission_id="unsafe-1",
        mission_type="semantic_room_shoe_mapping",
        room_mapping={"map_name": "room", "semantic_labels": ["shoe"]},
        safety={"allow_direct_ros_commands": False},
        artifacts=[{"kind": "semantic_map", "mime_type": "application/json"}],
        requested_ros_topics=("/cmd_vel",),
        raw_ros_command={"topic": "/cmd_vel", "linear": 0.2},
    )

    with pytest.raises(MissionValidationError, match="direct ROS command"):
        validate_mission_request(request, capabilities)

    with pytest.raises(MissionValidationError, match="Unsupported mission_type"):
        validate_mission_request(
            MissionRequest(
                api_version=MissionApiVersion.V1,
                mission_id="unsafe-2",
                mission_type="publish_ros_topic",
                room_mapping={"map_name": "room", "semantic_labels": ["shoe"]},
                safety={},
                artifacts=[],
            ),
            capabilities,
        )


def test_validation_rejects_missing_capabilities_and_unbounded_safety() -> None:
    request = build_canonical_shoe_mapping_request(mission_id="mission-002")

    with pytest.raises(MissionValidationError, match="shoe_detection"):
        validate_mission_request(request, CapabilitySet.all_enabled(shoe_detection=False))

    unsafe = build_canonical_shoe_mapping_request(
        mission_id="mission-003",
        safety={"max_runtime_s": 0, "max_segments": 0, "cancel_supported": False},
    )
    with pytest.raises(MissionValidationError, match="max_runtime_s"):
        validate_mission_request(unsafe, CapabilitySet.all_enabled())


def test_state_machine_transitions_through_mapping_exploring_detecting_finalizing_complete() -> None:
    command = validate_mission_request(build_canonical_shoe_mapping_request(), CapabilitySet.all_enabled())
    machine = MissionStateMachine(command)

    observed = [machine.snapshot().state]
    for event in (
        MissionEventKind.START_REQUESTED,
        MissionEventKind.VALIDATED,
        MissionEventKind.MAPPING_STARTED,
        MissionEventKind.EXPLORATION_STARTED,
        MissionEventKind.DETECTION_STARTED,
        MissionEventKind.FINALIZE_STARTED,
    ):
        observed.append(machine.apply(event).state)
    complete = machine.complete(
        artifacts={
            "occupancy_map": "maps/shoe_room_map.yaml",
            "semantic_map": "maps/shoe_room_semantic.json",
            "shoe_detections": "detections/shoe_room.json",
        }
    )

    assert observed == [
        MissionState.IDLE,
        MissionState.VALIDATING,
        MissionState.MAPPING,
        MissionState.EXPLORING,
        MissionState.DETECTING,
        MissionState.FINALIZING,
        MissionState.COMPLETE,
    ]
    assert complete.result is not None
    assert complete.result.status is MissionResultStatus.COMPLETE
    assert complete.result.artifacts["semantic_map"] == "maps/shoe_room_semantic.json"
    assert complete.telemetry["state"] == "COMPLETE"
    assert complete.telemetry["api_version"] == "mission_api.v1"


def test_state_machine_cancel_estop_failure_and_block_paths_are_latched() -> None:
    command = validate_mission_request(build_canonical_shoe_mapping_request(), CapabilitySet.all_enabled())

    cancelled = MissionStateMachine(command)
    cancelled.apply(MissionEventKind.START_REQUESTED)
    cancel_snapshot = cancelled.cancel(reason="operator requested cancel")
    assert cancel_snapshot.state is MissionState.CANCELLED
    assert cancel_snapshot.result is not None
    assert cancel_snapshot.result.status is MissionResultStatus.CANCELLED
    assert cancelled.apply(MissionEventKind.VALIDATED).state is MissionState.CANCELLED

    estopped = MissionStateMachine(command)
    estopped.apply(MissionEventKind.START_REQUESTED)
    estop_snapshot = estopped.estop(reason="physical estop")
    assert estop_snapshot.state is MissionState.ESTOPPED
    assert estop_snapshot.result is not None
    assert estop_snapshot.result.status is MissionResultStatus.ESTOPPED

    failed = MissionStateMachine(command)
    failed.apply(MissionEventKind.START_REQUESTED)
    failed.apply(MissionEventKind.VALIDATED)
    failure_snapshot = failed.fail(reason="range motion timeout", range_motion_stop_reason=StopReason.TIMEOUT)
    assert failure_snapshot.state is MissionState.FAILED
    assert failure_snapshot.result is not None
    assert failure_snapshot.result.status is MissionResultStatus.FAILED
    assert failure_snapshot.telemetry["range_motion_stop_reason"] == "timeout"

    blocked = MissionStateMachine(command)
    blocked.apply(MissionEventKind.START_REQUESTED)
    blocked_snapshot = blocked.block(reason="localization prerequisite missing")
    assert blocked_snapshot.state is MissionState.BLOCKED
    assert blocked_snapshot.result is not None
    assert blocked_snapshot.result.status is MissionResultStatus.BLOCKED


def test_state_machine_supports_paused_state_without_terminal_result() -> None:
    command = validate_mission_request(build_canonical_shoe_mapping_request(), CapabilitySet.all_enabled())
    machine = MissionStateMachine(command)
    machine.apply(MissionEventKind.START_REQUESTED)
    machine.apply(MissionEventKind.VALIDATED)

    paused = machine.apply(MissionEventKind.PAUSE_REQUESTED, reason="operator pause")
    resumed = machine.apply(MissionEventKind.RESUME_REQUESTED)

    assert paused.state is MissionState.PAUSED
    assert paused.result is None
    assert paused.telemetry["terminal"] is False
    assert resumed.state is MissionState.EXPLORING


def test_result_rejects_missing_required_artifact_references() -> None:
    command = validate_mission_request(build_canonical_shoe_mapping_request(), CapabilitySet.all_enabled())
    machine = MissionStateMachine(command)
    for event in (
        MissionEventKind.START_REQUESTED,
        MissionEventKind.VALIDATED,
        MissionEventKind.MAPPING_STARTED,
        MissionEventKind.EXPLORATION_STARTED,
        MissionEventKind.DETECTION_STARTED,
        MissionEventKind.FINALIZE_STARTED,
    ):
        machine.apply(event)

    with pytest.raises(MissionValidationError, match="missing required result artifacts"):
        machine.complete(artifacts={"semantic_map": "maps/semantic.json"})
