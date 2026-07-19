"""Versioned Mission API schema and deterministic mission state machine.

This module is intentionally ROS-free.  It validates one allowlisted semantic
mission contract for the vertical slice and exposes stable JSON-friendly events,
telemetry, and result payloads for web/PWA controls and a constrained language
translator.  It is not a generic ROS bridge and it never accepts arbitrary ROS
commands.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Sequence

from .range_motion import StopReason


class MissionValidationError(ValueError):
    """Raised when a Mission API request cannot become a deterministic mission."""


class MissionApiVersion(str, Enum):
    V1 = "mission_api.v1"


class MissionState(str, Enum):
    IDLE = "IDLE"
    VALIDATING = "VALIDATING"
    MAPPING = "MAPPING"
    EXPLORING = "EXPLORING"
    DETECTING = "DETECTING"
    FINALIZING = "FINALIZING"
    COMPLETE = "COMPLETE"
    PAUSED = "PAUSED"
    CANCELLED = "CANCELLED"
    BLOCKED = "BLOCKED"
    ESTOPPED = "ESTOPPED"
    FAILED = "FAILED"


class MissionEventKind(str, Enum):
    START_REQUESTED = "start_requested"
    VALIDATED = "validated"
    MAPPING_STARTED = "mapping_started"
    EXPLORATION_STARTED = "exploration_started"
    DETECTION_STARTED = "detection_started"
    FINALIZE_STARTED = "finalize_started"
    COMPLETED = "completed"
    PAUSE_REQUESTED = "pause_requested"
    RESUME_REQUESTED = "resume_requested"
    CANCEL_REQUESTED = "cancel_requested"
    BLOCKED = "blocked"
    ESTOP = "estop"
    FAILED = "failed"


class MissionResultStatus(str, Enum):
    COMPLETE = "complete"
    CANCELLED = "cancelled"
    BLOCKED = "blocked"
    ESTOPPED = "estopped"
    FAILED = "failed"


SUPPORTED_MISSION_TYPE = "semantic_room_shoe_mapping"
DEFAULT_SEMANTIC_LABELS = ("shoe",)
UNSAFE_ROS_SURFACES = (
    "/cmd_vel",
    "/cmd_vel_motor",
    "cmd_vel",
    "cmd_vel_motor",
    "raw_motor",
    "motor",
    "teleop",
)


@dataclass(frozen=True)
class CapabilitySet:
    semantic_mapping: bool = False
    slam_replay_or_live_mapping: bool = False
    shoe_detection: bool = False
    supervised_motion: bool = False
    collision_stop: bool = False
    estop: bool = False
    artifacts: bool = False

    @classmethod
    def all_enabled(cls, **overrides: bool) -> "CapabilitySet":
        values = {
            "semantic_mapping": True,
            "slam_replay_or_live_mapping": True,
            "shoe_detection": True,
            "supervised_motion": True,
            "collision_stop": True,
            "estop": True,
            "artifacts": True,
        }
        values.update(overrides)
        return cls(**values)

    def missing_for_semantic_shoe_mapping(self) -> tuple[str, ...]:
        return tuple(name for name, enabled in asdict(self).items() if not enabled)

    def to_json_dict(self) -> dict[str, bool]:
        return dict(asdict(self))


@dataclass(frozen=True)
class RoomMappingContract:
    map_name: str = "shoe_room_map"
    semantic_labels: tuple[str, ...] = DEFAULT_SEMANTIC_LABELS
    frame_id: str = "map"
    source_frame_id: str = "base_link"
    occupancy_resolution_m: float = 0.05
    require_artifact_references: bool = True

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "RoomMappingContract":
        labels = tuple(str(item).strip().lower() for item in value.get("semantic_labels", DEFAULT_SEMANTIC_LABELS))
        return cls(
            map_name=str(value.get("map_name", "shoe_room_map")).strip(),
            semantic_labels=labels,
            frame_id=str(value.get("frame_id", "map")).strip(),
            source_frame_id=str(value.get("source_frame_id", "base_link")).strip(),
            occupancy_resolution_m=float(value.get("occupancy_resolution_m", 0.05)),
            require_artifact_references=bool(value.get("require_artifact_references", True)),
        )

    def __post_init__(self) -> None:
        if not self.map_name:
            raise MissionValidationError("room_mapping.map_name is required")
        if not self.semantic_labels:
            raise MissionValidationError("room_mapping.semantic_labels is required")
        if tuple(label.lower() for label in self.semantic_labels) != DEFAULT_SEMANTIC_LABELS:
            raise MissionValidationError("semantic_room_shoe_mapping only supports the shoe semantic label")
        if not math.isfinite(self.occupancy_resolution_m) or self.occupancy_resolution_m <= 0.0:
            raise MissionValidationError("room_mapping.occupancy_resolution_m must be positive and finite")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "map_name": self.map_name,
            "semantic_labels": list(self.semantic_labels),
            "frame_id": self.frame_id,
            "source_frame_id": self.source_frame_id,
            "occupancy_resolution_m": self.occupancy_resolution_m,
            "require_artifact_references": self.require_artifact_references,
        }


@dataclass(frozen=True)
class SafetyContract:
    start_requires_supervised_motion: bool = True
    cancel_supported: bool = True
    estop_supported: bool = True
    max_runtime_s: float = 600.0
    max_segments: int = 8
    allow_direct_ros_commands: bool = False

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "SafetyContract":
        return cls(
            start_requires_supervised_motion=bool(value.get("start_requires_supervised_motion", True)),
            cancel_supported=bool(value.get("cancel_supported", True)),
            estop_supported=bool(value.get("estop_supported", True)),
            max_runtime_s=float(value.get("max_runtime_s", 600.0)),
            max_segments=int(value.get("max_segments", 8)),
            allow_direct_ros_commands=bool(value.get("allow_direct_ros_commands", False)),
        )

    def __post_init__(self) -> None:
        if not math.isfinite(self.max_runtime_s) or self.max_runtime_s <= 0.0:
            raise MissionValidationError("safety.max_runtime_s must be positive and finite")
        if self.max_segments <= 0:
            raise MissionValidationError("safety.max_segments must be positive")
        if not self.start_requires_supervised_motion:
            raise MissionValidationError("safety.start_requires_supervised_motion must be true")
        if not self.cancel_supported:
            raise MissionValidationError("safety.cancel_supported must be true")
        if not self.estop_supported:
            raise MissionValidationError("safety.estop_supported must be true")
        if self.allow_direct_ros_commands:
            raise MissionValidationError("safety.allow_direct_ros_commands must be false")

    def to_json_dict(self) -> dict[str, Any]:
        return dict(asdict(self))


@dataclass(frozen=True)
class ArtifactContract:
    kind: str
    mime_type: str
    required: bool = True

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "ArtifactContract":
        return cls(
            kind=str(value.get("kind", "")).strip(),
            mime_type=str(value.get("mime_type", "")).strip(),
            required=bool(value.get("required", True)),
        )

    def __post_init__(self) -> None:
        if not self.kind:
            raise MissionValidationError("artifact.kind is required")
        if not self.mime_type:
            raise MissionValidationError("artifact.mime_type is required")

    def to_json_dict(self) -> dict[str, Any]:
        return dict(asdict(self))


REQUIRED_ARTIFACTS = (
    ArtifactContract("occupancy_map", "application/x-yaml", required=True),
    ArtifactContract("semantic_map", "application/json", required=True),
    ArtifactContract("shoe_detections", "application/json", required=True),
)


@dataclass(frozen=True)
class MissionRequest:
    api_version: MissionApiVersion
    mission_id: str
    mission_type: str
    room_mapping: Mapping[str, Any]
    safety: Mapping[str, Any] = field(default_factory=dict)
    artifacts: Sequence[Mapping[str, Any]] = field(default_factory=list)
    requested_ros_topics: Sequence[str] = field(default_factory=tuple)
    raw_ros_command: Optional[Mapping[str, Any]] = None

    def __post_init__(self) -> None:
        if isinstance(self.api_version, str):
            object.__setattr__(self, "api_version", MissionApiVersion(self.api_version))
        if not self.mission_id:
            raise MissionValidationError("mission_id is required")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": self.api_version.value,
            "mission_id": self.mission_id,
            "mission_type": self.mission_type,
            "room_mapping": dict(self.room_mapping),
            "safety": dict(self.safety),
            "artifacts": [dict(item) for item in self.artifacts],
            "requested_ros_topics": list(self.requested_ros_topics),
            "raw_ros_command": None if self.raw_ros_command is None else dict(self.raw_ros_command),
        }


@dataclass(frozen=True)
class MissionCommand:
    api_version: MissionApiVersion
    mission_id: str
    mission_type: str
    room_mapping: RoomMappingContract
    capability_checks: CapabilitySet
    safety: SafetyContract
    artifacts: tuple[ArtifactContract, ...]
    command_path: tuple[str, ...] = ("mission_api", "supervised_coordinator", "range_motion", "/cmd_vel", "collision_stop")
    generic_ros_bridge: bool = False

    def required_artifact_kinds(self) -> tuple[str, ...]:
        return tuple(artifact.kind for artifact in self.artifacts if artifact.required)

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": self.api_version.value,
            "mission_id": self.mission_id,
            "mission_type": self.mission_type,
            "room_mapping": self.room_mapping.to_json_dict(),
            "capability_checks": self.capability_checks.to_json_dict(),
            "safety": self.safety.to_json_dict(),
            "artifacts": [artifact.to_json_dict() for artifact in self.artifacts],
            "command_path": list(self.command_path),
            "generic_ros_bridge": self.generic_ros_bridge,
        }


@dataclass(frozen=True)
class MissionResult:
    mission_id: str
    status: MissionResultStatus
    artifacts: Mapping[str, str] = field(default_factory=dict)
    reason: str = ""

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "mission_id": self.mission_id,
            "status": self.status.value,
            "artifacts": dict(self.artifacts),
            "reason": self.reason,
        }


@dataclass(frozen=True)
class MissionSnapshot:
    mission_id: str
    api_version: MissionApiVersion
    state: MissionState
    event_log: tuple[str, ...]
    telemetry: Mapping[str, Any]
    result: Optional[MissionResult] = None

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "mission_id": self.mission_id,
            "api_version": self.api_version.value,
            "state": self.state.value,
            "event_log": list(self.event_log),
            "telemetry": dict(self.telemetry),
            "result": None if self.result is None else self.result.to_json_dict(),
        }


def build_canonical_shoe_mapping_request(
    *,
    mission_id: str = "shoe-room-map",
    safety: Optional[Mapping[str, Any]] = None,
) -> MissionRequest:
    safety_values = {
        "start_requires_supervised_motion": True,
        "cancel_supported": True,
        "estop_supported": True,
        "max_runtime_s": 600.0,
        "max_segments": 8,
        "allow_direct_ros_commands": False,
    }
    if safety:
        safety_values.update(safety)
    return MissionRequest(
        api_version=MissionApiVersion.V1,
        mission_id=mission_id,
        mission_type=SUPPORTED_MISSION_TYPE,
        room_mapping={
            "map_name": "shoe_room_map",
            "semantic_labels": ["shoe"],
            "frame_id": "map",
            "source_frame_id": "base_link",
            "occupancy_resolution_m": 0.05,
            "require_artifact_references": True,
        },
        safety=safety_values,
        artifacts=[artifact.to_json_dict() for artifact in REQUIRED_ARTIFACTS],
        requested_ros_topics=(),
        raw_ros_command=None,
    )


def validate_mission_request(request: MissionRequest, capabilities: CapabilitySet) -> MissionCommand:
    if request.api_version is not MissionApiVersion.V1:
        raise MissionValidationError(f"Unsupported api_version: {request.api_version}")
    if request.mission_type != SUPPORTED_MISSION_TYPE:
        raise MissionValidationError(f"Unsupported mission_type: {request.mission_type}")
    _reject_direct_ros_surfaces(request)
    missing = capabilities.missing_for_semantic_shoe_mapping()
    if missing:
        raise MissionValidationError("Missing required capabilities: " + ", ".join(missing))
    room_mapping = RoomMappingContract.from_mapping(request.room_mapping)
    safety = SafetyContract.from_mapping(request.safety)
    artifacts = _normalize_artifacts(request.artifacts)
    _require_artifact_contracts(artifacts)
    return MissionCommand(
        api_version=request.api_version,
        mission_id=request.mission_id,
        mission_type=request.mission_type,
        room_mapping=room_mapping,
        capability_checks=capabilities,
        safety=safety,
        artifacts=artifacts,
    )


def _reject_direct_ros_surfaces(request: MissionRequest) -> None:
    if request.raw_ros_command is not None:
        raise MissionValidationError("direct ROS command payloads are not allowed by Mission API")
    for topic in request.requested_ros_topics:
        lowered = str(topic).lower()
        if any(surface in lowered for surface in UNSAFE_ROS_SURFACES):
            raise MissionValidationError(f"direct ROS command/topic is not allowed: {topic}")


def _normalize_artifacts(values: Sequence[Mapping[str, Any]]) -> tuple[ArtifactContract, ...]:
    artifacts = tuple(ArtifactContract.from_mapping(value) for value in values)
    if not artifacts:
        return REQUIRED_ARTIFACTS
    known = {artifact.kind: artifact for artifact in artifacts}
    merged = list(artifacts)
    for required in REQUIRED_ARTIFACTS:
        if required.kind not in known:
            merged.append(required)
    return tuple(merged)


def _require_artifact_contracts(artifacts: Sequence[ArtifactContract]) -> None:
    available = {artifact.kind for artifact in artifacts if artifact.required}
    missing = [artifact.kind for artifact in REQUIRED_ARTIFACTS if artifact.kind not in available]
    if missing:
        raise MissionValidationError("missing required artifact contracts: " + ", ".join(missing))


class MissionStateMachine:
    TERMINAL_STATES = {
        MissionState.COMPLETE,
        MissionState.CANCELLED,
        MissionState.BLOCKED,
        MissionState.ESTOPPED,
        MissionState.FAILED,
    }

    _TRANSITIONS = {
        (MissionState.IDLE, MissionEventKind.START_REQUESTED): MissionState.VALIDATING,
        (MissionState.VALIDATING, MissionEventKind.VALIDATED): MissionState.MAPPING,
        (MissionState.MAPPING, MissionEventKind.MAPPING_STARTED): MissionState.EXPLORING,
        (MissionState.EXPLORING, MissionEventKind.EXPLORATION_STARTED): MissionState.DETECTING,
        (MissionState.DETECTING, MissionEventKind.DETECTION_STARTED): MissionState.FINALIZING,
        (MissionState.FINALIZING, MissionEventKind.FINALIZE_STARTED): MissionState.COMPLETE,
        (MissionState.MAPPING, MissionEventKind.PAUSE_REQUESTED): MissionState.PAUSED,
        (MissionState.EXPLORING, MissionEventKind.PAUSE_REQUESTED): MissionState.PAUSED,
        (MissionState.DETECTING, MissionEventKind.PAUSE_REQUESTED): MissionState.PAUSED,
        (MissionState.FINALIZING, MissionEventKind.PAUSE_REQUESTED): MissionState.PAUSED,
        (MissionState.PAUSED, MissionEventKind.RESUME_REQUESTED): MissionState.EXPLORING,
    }

    def __init__(self, command: MissionCommand):
        self.command = command
        self._state = MissionState.IDLE
        self._event_log: list[str] = []
        self._result: Optional[MissionResult] = None
        self._reason = ""
        self._range_motion_stop_reason: Optional[StopReason] = None

    def snapshot(self) -> MissionSnapshot:
        return MissionSnapshot(
            mission_id=self.command.mission_id,
            api_version=self.command.api_version,
            state=self._state,
            event_log=tuple(self._event_log),
            telemetry=self._telemetry(),
            result=self._result,
        )

    def apply(self, event: MissionEventKind, *, reason: str = "") -> MissionSnapshot:
        if isinstance(event, str):
            event = MissionEventKind(event)
        if self._state in self.TERMINAL_STATES:
            return self.snapshot()
        if event is MissionEventKind.CANCEL_REQUESTED:
            return self.cancel(reason=reason or "cancel requested")
        if event is MissionEventKind.ESTOP:
            return self.estop(reason=reason or "estop")
        if event is MissionEventKind.BLOCKED:
            return self.block(reason=reason or "blocked")
        if event is MissionEventKind.FAILED:
            return self.fail(reason=reason or "failed")
        next_state = self._TRANSITIONS.get((self._state, event))
        if next_state is None:
            return self.fail(reason=f"invalid mission transition: {self._state.value} + {event.value}")
        self._state = next_state
        self._event_log.append(event.value)
        if self._state is MissionState.COMPLETE and self._result is None:
            self._result = MissionResult(self.command.mission_id, MissionResultStatus.COMPLETE, artifacts={})
        return self.snapshot()

    def complete(self, *, artifacts: Mapping[str, str]) -> MissionSnapshot:
        missing = [kind for kind in self.command.required_artifact_kinds() if not artifacts.get(kind)]
        if missing:
            raise MissionValidationError("missing required result artifacts: " + ", ".join(missing))
        self._state = MissionState.COMPLETE
        self._event_log.append(MissionEventKind.COMPLETED.value)
        self._result = MissionResult(
            self.command.mission_id,
            MissionResultStatus.COMPLETE,
            artifacts=dict(artifacts),
        )
        return self.snapshot()

    def cancel(self, *, reason: str) -> MissionSnapshot:
        self._finish(MissionState.CANCELLED, MissionResultStatus.CANCELLED, reason)
        return self.snapshot()

    def estop(self, *, reason: str) -> MissionSnapshot:
        self._finish(MissionState.ESTOPPED, MissionResultStatus.ESTOPPED, reason)
        return self.snapshot()

    def block(self, *, reason: str) -> MissionSnapshot:
        self._finish(MissionState.BLOCKED, MissionResultStatus.BLOCKED, reason)
        return self.snapshot()

    def fail(self, *, reason: str, range_motion_stop_reason: Optional[StopReason] = None) -> MissionSnapshot:
        if isinstance(range_motion_stop_reason, str):
            range_motion_stop_reason = StopReason(range_motion_stop_reason)
        self._range_motion_stop_reason = range_motion_stop_reason
        self._finish(MissionState.FAILED, MissionResultStatus.FAILED, reason)
        return self.snapshot()

    def _finish(self, state: MissionState, status: MissionResultStatus, reason: str) -> None:
        if self._state in self.TERMINAL_STATES:
            return
        self._state = state
        self._reason = reason
        self._event_log.append(status.value)
        self._result = MissionResult(self.command.mission_id, status, artifacts={}, reason=reason)

    def _telemetry(self) -> dict[str, Any]:
        return {
            "api_version": self.command.api_version.value,
            "mission_id": self.command.mission_id,
            "mission_type": self.command.mission_type,
            "state": self._state.value,
            "terminal": self._state in self.TERMINAL_STATES,
            "cancel_supported": self.command.safety.cancel_supported,
            "estop_supported": self.command.safety.estop_supported,
            "max_runtime_s": self.command.safety.max_runtime_s,
            "max_segments": self.command.safety.max_segments,
            "command_path": list(self.command.command_path),
            "generic_ros_bridge": False,
            "direct_ros_commands_allowed": False,
            "required_artifacts": list(self.command.required_artifact_kinds()),
            "result_artifacts": {} if self._result is None else dict(self._result.artifacts),
            "range_motion_stop_reason": None
            if self._range_motion_stop_reason is None
            else self._range_motion_stop_reason.value,
            "reason": self._reason,
        }
