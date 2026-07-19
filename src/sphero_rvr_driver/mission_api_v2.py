"""Mission API v2 rover capability/tool registry.

This module is intentionally ROS-free.  It defines typed, allowlisted rover tool
contracts for low-rate supervisory planners.  The validation boundary is fail-
closed: unknown tools, malformed arguments, missing capabilities/approvals,
physical/replay confusion, direct ROS/motor surfaces, budget expansion, adapter
schema drift, and timeout violations are rejected before they can become
execution authority.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Callable, Mapping, Optional, Sequence

from .mission_api import MissionValidationError
from .mission_controls import MissionExecutionMode


DIRECT_SURFACE_TOKENS = (
    "/cmd_vel",
    "/cmd_vel_motor",
    "cmd_vel",
    "cmd_vel_motor",
    "raw_motor",
    "raw motor",
    "motor duty",
    "generic ros bridge",
    "ros bridge",
    "ros topic",
    "publish",
    "shell",
    "filesystem",
    "credential",
    "env var",
    "clear_estop",
    "clear estop",
)

MAX_RUNTIME_CAP_S = 900.0
MAX_TOOL_CALLS_CAP = 32
MAX_TRAVEL_CAP_M = 5.0
MAX_SEGMENTS_CAP = 16


class ToolResultStatus(str, Enum):
    OK = "ok"
    PARTIAL = "partial"
    FAILED = "failed"
    CANCELLED = "cancelled"
    ESTOPPED = "estopped"


class ApprovalState(str, Enum):
    NONE = "none"
    REPLAY_ONLY = "replay_only"
    PHYSICAL_APPROVED = "physical_approved"


@dataclass(frozen=True)
class CapabilityState:
    semantic_mapping: bool = False
    object_detection: bool = False
    supervised_motion: bool = False
    collision_stop: bool = False
    estop: bool = False
    artifacts: bool = False
    telemetry: bool = False

    @classmethod
    def all_enabled(cls, **overrides: bool) -> "CapabilityState":
        values = {name: True for name in cls.__dataclass_fields__}
        values.update(overrides)
        return cls(**values)

    def enabled(self, name: str) -> bool:
        return bool(getattr(self, name, False))

    def to_json_dict(self) -> dict[str, bool]:
        return dict(asdict(self))


@dataclass(frozen=True)
class MissionBudgets:
    max_iterations: int = 8
    max_runtime_s: float = 120.0
    max_tool_calls: int = 12
    max_travel_m: float = 2.0
    max_segments: int = 8

    def __post_init__(self) -> None:
        _require_positive_int("max_iterations", self.max_iterations, MAX_TOOL_CALLS_CAP)
        _require_positive_float("max_runtime_s", self.max_runtime_s, MAX_RUNTIME_CAP_S)
        _require_positive_int("max_tool_calls", self.max_tool_calls, MAX_TOOL_CALLS_CAP)
        _require_nonnegative_float("max_travel_m", self.max_travel_m, MAX_TRAVEL_CAP_M)
        _require_positive_int("max_segments", self.max_segments, MAX_SEGMENTS_CAP)

    def to_json_dict(self) -> dict[str, Any]:
        return dict(asdict(self))


@dataclass(frozen=True)
class RemainingBudgets:
    iterations: int
    runtime_s: float
    tool_calls: int
    travel_m: float
    segments: int

    def to_json_dict(self) -> dict[str, Any]:
        return dict(asdict(self))


@dataclass(frozen=True)
class ToolDefinition:
    name: str
    description: str
    input_schema: Mapping[str, Any]
    result_schema: Mapping[str, Any]
    timeout_s: float
    capabilities_required: Sequence[str] = field(default_factory=tuple)
    approval_required: ApprovalState = ApprovalState.NONE
    max_travel_m: float = 0.0
    counts_as_segment: bool = False

    def __post_init__(self) -> None:
        if not self.name or not self.name.replace("_", "").isalnum():
            raise MissionValidationError("tool name must be a non-empty safe identifier")
        if _contains_direct_surface(self.name) or _contains_direct_surface(self.description):
            raise MissionValidationError(f"tool {self.name} exposes a forbidden direct surface")
        if isinstance(self.approval_required, str):
            object.__setattr__(self, "approval_required", ApprovalState(self.approval_required))
        _require_positive_float("timeout_s", self.timeout_s, MAX_RUNTIME_CAP_S)
        _require_nonnegative_float("max_travel_m", self.max_travel_m, MAX_TRAVEL_CAP_M)
        _validate_object_schema(self.input_schema, schema_name=f"{self.name}.input_schema")
        _validate_object_schema(self.result_schema, schema_name=f"{self.name}.result_schema")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "input_schema": dict(self.input_schema),
            "result_schema": dict(self.result_schema),
            "timeout_s": self.timeout_s,
            "capabilities_required": list(self.capabilities_required),
            "approval_required": self.approval_required.value,
            "max_travel_m": self.max_travel_m,
            "counts_as_segment": self.counts_as_segment,
        }


@dataclass(frozen=True)
class ToolCall:
    tool_name: str
    arguments: Mapping[str, Any] = field(default_factory=dict)
    call_id: str = ""

    def __post_init__(self) -> None:
        if not str(self.tool_name).strip():
            raise MissionValidationError("tool_name is required")
        if not isinstance(self.arguments, Mapping):
            raise MissionValidationError("tool arguments must be an object")
        _reject_direct_surface_payload(self.tool_name, self.arguments)

    def to_json_dict(self) -> dict[str, Any]:
        return {"tool_name": self.tool_name, "arguments": dict(self.arguments), "call_id": self.call_id}


@dataclass(frozen=True)
class ToolResult:
    status: ToolResultStatus
    observation: Mapping[str, Any] = field(default_factory=dict)
    artifacts: Mapping[str, str] = field(default_factory=dict)
    reason: str = ""
    duration_s: float = 0.0
    travel_m: float = 0.0

    def __post_init__(self) -> None:
        if isinstance(self.status, str):
            object.__setattr__(self, "status", ToolResultStatus(self.status))
        if not isinstance(self.observation, Mapping):
            raise MissionValidationError("tool result observation must be an object")
        if not isinstance(self.artifacts, Mapping):
            raise MissionValidationError("tool result artifacts must be an object")
        _reject_direct_surface_payload(self.reason, self.observation, self.artifacts)
        _require_nonnegative_float("duration_s", self.duration_s, MAX_RUNTIME_CAP_S)
        _require_nonnegative_float("travel_m", self.travel_m, MAX_TRAVEL_CAP_M)

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "status": self.status.value,
            "observation": dict(self.observation),
            "artifacts": dict(self.artifacts),
            "reason": self.reason,
            "duration_s": self.duration_s,
            "travel_m": self.travel_m,
        }


ToolAdapter = Callable[[Mapping[str, Any]], ToolResult]


class RoverToolRegistry:
    """Allowlisted tool definitions plus deterministic adapters."""

    def __init__(self, *, registry_version: str = "mission_api.v2"):
        self.registry_version = registry_version
        self._definitions: dict[str, ToolDefinition] = {}
        self._adapters: dict[str, ToolAdapter] = {}

    def register(self, definition: ToolDefinition, adapter: ToolAdapter) -> None:
        if definition.name in self._definitions:
            raise MissionValidationError(f"duplicate tool registered: {definition.name}")
        self._definitions[definition.name] = definition
        self._adapters[definition.name] = adapter

    def definition(self, name: str) -> ToolDefinition:
        try:
            return self._definitions[name]
        except KeyError as exc:
            raise MissionValidationError(f"unknown rover tool: {name}") from exc

    def definitions(self) -> tuple[ToolDefinition, ...]:
        return tuple(self._definitions[name] for name in sorted(self._definitions))

    def tool_definitions_json(self) -> list[dict[str, Any]]:
        return [definition.to_json_dict() for definition in self.definitions()]

    def validate_tool_call(
        self,
        call: ToolCall,
        *,
        capabilities: CapabilityState,
        approval_state: ApprovalState,
        execution_mode: MissionExecutionMode,
        remaining: RemainingBudgets,
    ) -> ToolDefinition:
        definition = self.definition(call.tool_name)
        for capability in definition.capabilities_required:
            if not capabilities.enabled(capability):
                raise MissionValidationError(f"tool {call.tool_name} requires unavailable capability: {capability}")
        approval_state = ApprovalState(approval_state)
        execution_mode = MissionExecutionMode(execution_mode)
        if definition.approval_required is ApprovalState.PHYSICAL_APPROVED:
            if execution_mode is not MissionExecutionMode.PHYSICAL:
                raise MissionValidationError(f"tool {call.tool_name} requires physical execution mode")
            if approval_state is not ApprovalState.PHYSICAL_APPROVED:
                raise MissionValidationError(f"tool {call.tool_name} requires physical approval")
        if execution_mode is MissionExecutionMode.PHYSICAL and approval_state is not ApprovalState.PHYSICAL_APPROVED:
            raise MissionValidationError("physical execution mode requires physical approval")
        if execution_mode is not MissionExecutionMode.PHYSICAL and approval_state is ApprovalState.REPLAY_ONLY:
            pass
        _validate_value_against_schema(call.arguments, definition.input_schema, path=f"{call.tool_name}.arguments")
        if remaining.tool_calls <= 0:
            raise MissionValidationError("tool-call budget exhausted")
        if definition.max_travel_m > remaining.travel_m:
            raise MissionValidationError("travel budget exhausted")
        if definition.counts_as_segment and remaining.segments <= 0:
            raise MissionValidationError("segment budget exhausted")
        return definition

    def execute_tool(self, call: ToolCall, definition: ToolDefinition) -> ToolResult:
        started = time.monotonic()
        result = self._adapters[definition.name](call.arguments)
        elapsed = time.monotonic() - started
        if not isinstance(result, ToolResult):
            raise MissionValidationError(f"tool {definition.name} adapter returned non-ToolResult")
        duration = max(result.duration_s, elapsed)
        if duration > definition.timeout_s:
            raise MissionValidationError(f"tool {definition.name} exceeded timeout_s={definition.timeout_s}")
        if result.travel_m > definition.max_travel_m:
            raise MissionValidationError(f"tool {definition.name} exceeded declared travel limit")
        _validate_value_against_schema(result.observation, definition.result_schema, path=f"{definition.name}.result")
        return ToolResult(
            status=result.status,
            observation=result.observation,
            artifacts=result.artifacts,
            reason=result.reason,
            duration_s=duration,
            travel_m=result.travel_m,
        )


class ScriptedToolAdapter:
    """Deterministic replay/mock adapter for tests and offline demos."""

    def __init__(self, results: Sequence[ToolResult]):
        if not results:
            raise MissionValidationError("scripted adapter requires at least one result")
        self._results = list(results)
        self.calls: list[Mapping[str, Any]] = []

    def __call__(self, arguments: Mapping[str, Any]) -> ToolResult:
        self.calls.append(dict(arguments))
        index = min(len(self.calls) - 1, len(self._results) - 1)
        return self._results[index]


def build_default_rover_tool_registry(*, registry_version: str = "mission_api.v2") -> RoverToolRegistry:
    registry = RoverToolRegistry(registry_version=registry_version)
    registry.register(
        ToolDefinition(
            name="create_room_map",
            description="Build or load a bounded replay/mock room occupancy map artifact.",
            input_schema=_schema({"map_name": "string"}, required=("map_name",)),
            result_schema=_schema({"map_name": "string", "occupancy_map": "string", "coverage": "number"}, required=("map_name", "occupancy_map")),
            timeout_s=5.0,
            capabilities_required=("semantic_mapping", "artifacts"),
        ),
        lambda args: ToolResult(
            ToolResultStatus.OK,
            observation={"map_name": str(args["map_name"]), "occupancy_map": f"maps/{args['map_name']}.yaml", "coverage": 0.86},
            artifacts={"occupancy_map": f"maps/{args['map_name']}.yaml"},
            duration_s=0.01,
        ),
    )
    registry.register(
        ToolDefinition(
            name="detect_objects",
            description="Detect an allowlisted object class in replay/mock observations.",
            input_schema=_schema({"object_class": "string", "source": "string"}, required=("object_class",)),
            result_schema=_schema({"object_class": "string", "count": "integer", "detections": "array"}, required=("object_class", "count", "detections")),
            timeout_s=5.0,
            capabilities_required=("object_detection",),
        ),
        lambda args: ToolResult(
            ToolResultStatus.OK,
            observation={"object_class": str(args["object_class"]), "count": 2, "detections": [{"label": str(args["object_class"]), "confidence": 0.91}]},
            artifacts={f"{args['object_class']}_detections": f"detections/{args['object_class']}.json"},
            duration_s=0.01,
        ),
    )
    registry.register(
        ToolDefinition(
            name="project_semantic_map",
            description="Project allowlisted object detections into a semantic map artifact.",
            input_schema=_schema({"map_name": "string", "object_class": "string"}, required=("map_name", "object_class")),
            result_schema=_schema({"semantic_map": "string", "labels": "array"}, required=("semantic_map", "labels")),
            timeout_s=5.0,
            capabilities_required=("semantic_mapping", "artifacts"),
        ),
        lambda args: ToolResult(
            ToolResultStatus.OK,
            observation={"semantic_map": f"maps/{args['map_name']}_{args['object_class']}.json", "labels": [str(args["object_class"])]},
            artifacts={"semantic_map": f"maps/{args['map_name']}_{args['object_class']}.json"},
            duration_s=0.01,
        ),
    )
    registry.register(
        ToolDefinition(
            name="approach_clearance",
            description="Request one bounded supervised-motion segment to approach a target clearance; no direct motor control.",
            input_schema=_schema({"target_clearance_m": "number"}, required=("target_clearance_m",)),
            result_schema=_schema({"clearance_m": "number", "segment_complete": "boolean"}, required=("clearance_m", "segment_complete")),
            timeout_s=10.0,
            capabilities_required=("supervised_motion", "collision_stop", "estop", "telemetry"),
            approval_required=ApprovalState.REPLAY_ONLY,
            max_travel_m=1.0,
            counts_as_segment=True,
        ),
        lambda args: ToolResult(
            ToolResultStatus.OK,
            observation={"clearance_m": float(args["target_clearance_m"]), "segment_complete": True},
            duration_s=0.01,
            travel_m=0.25,
        ),
    )
    registry.register(
        ToolDefinition(
            name="capture_observation",
            description="Capture a replay/mock observation and artifact reference after deterministic motion settles.",
            input_schema=_schema({"label": "string"}, required=("label",)),
            result_schema=_schema({"label": "string", "artifact": "string"}, required=("label", "artifact")),
            timeout_s=5.0,
            capabilities_required=("telemetry", "artifacts"),
        ),
        lambda args: ToolResult(
            ToolResultStatus.OK,
            observation={"label": str(args["label"]), "artifact": f"observations/{args['label']}.json"},
            artifacts={"observation": f"observations/{args['label']}.json"},
            duration_s=0.01,
        ),
    )
    registry.register(
        ToolDefinition(
            name="report_artifacts",
            description="Return final bounded mission artifact references and summary.",
            input_schema=_schema({"summary": "string"}, required=("summary",)),
            result_schema=_schema({"summary": "string", "complete": "boolean"}, required=("summary", "complete")),
            timeout_s=2.0,
            capabilities_required=("artifacts",),
        ),
        lambda args: ToolResult(
            ToolResultStatus.OK,
            observation={"summary": str(args["summary"]), "complete": True},
            duration_s=0.01,
        ),
    )
    return registry


def remaining_budgets(budgets: MissionBudgets, *, iterations_used: int, runtime_used_s: float, tool_calls_used: int, travel_used_m: float, segments_used: int) -> RemainingBudgets:
    return RemainingBudgets(
        iterations=max(0, budgets.max_iterations - iterations_used),
        runtime_s=max(0.0, budgets.max_runtime_s - runtime_used_s),
        tool_calls=max(0, budgets.max_tool_calls - tool_calls_used),
        travel_m=max(0.0, budgets.max_travel_m - travel_used_m),
        segments=max(0, budgets.max_segments - segments_used),
    )


def _schema(properties: Mapping[str, str], *, required: Sequence[str] = ()) -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {name: {"type": type_name} for name, type_name in properties.items()},
        "required": list(required),
        "additionalProperties": False,
    }


def _validate_object_schema(schema: Mapping[str, Any], *, schema_name: str) -> None:
    if schema.get("type") != "object":
        raise MissionValidationError(f"{schema_name} must be an object schema")
    if not isinstance(schema.get("properties", {}), Mapping):
        raise MissionValidationError(f"{schema_name}.properties must be an object")
    for name, spec in schema.get("properties", {}).items():
        if _contains_direct_surface(str(name)) or _contains_direct_surface(str(spec)):
            raise MissionValidationError(f"{schema_name} exposes a forbidden direct surface")
        if not isinstance(spec, Mapping) or spec.get("type") not in {"string", "number", "integer", "boolean", "array", "object"}:
            raise MissionValidationError(f"{schema_name}.{name} has unsupported type")


def _validate_value_against_schema(value: Mapping[str, Any], schema: Mapping[str, Any], *, path: str) -> None:
    if not isinstance(value, Mapping):
        raise MissionValidationError(f"{path} must be an object")
    properties = schema.get("properties", {})
    required = set(schema.get("required", ()))
    missing = [name for name in required if name not in value]
    if missing:
        raise MissionValidationError(f"{path} missing required fields: {', '.join(sorted(missing))}")
    if schema.get("additionalProperties") is False:
        extra = [name for name in value if name not in properties]
        if extra:
            raise MissionValidationError(f"{path} has unexpected fields: {', '.join(sorted(extra))}")
    for name, item in value.items():
        if name not in properties:
            continue
        expected = properties[name]["type"]
        if expected == "string" and not isinstance(item, str):
            raise MissionValidationError(f"{path}.{name} must be a string")
        if expected == "number" and (not isinstance(item, (int, float)) or isinstance(item, bool) or not math.isfinite(float(item))):
            raise MissionValidationError(f"{path}.{name} must be a finite number")
        if expected == "integer" and (not isinstance(item, int) or isinstance(item, bool)):
            raise MissionValidationError(f"{path}.{name} must be an integer")
        if expected == "boolean" and not isinstance(item, bool):
            raise MissionValidationError(f"{path}.{name} must be a boolean")
        if expected == "array" and not isinstance(item, Sequence):
            raise MissionValidationError(f"{path}.{name} must be an array")
        if expected == "object" and not isinstance(item, Mapping):
            raise MissionValidationError(f"{path}.{name} must be an object")


def _reject_direct_surface_payload(*values: Any) -> None:
    for value in values:
        if _contains_direct_surface(repr(value).lower()):
            raise MissionValidationError("direct ROS/motor/system surfaces are not accepted by mission_api.v2")


def _contains_direct_surface(text: str) -> bool:
    lowered = str(text).lower()
    return any(token in lowered for token in DIRECT_SURFACE_TOKENS)


def _require_positive_int(name: str, value: int, cap: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0 or value > cap:
        raise MissionValidationError(f"{name} must be an integer in 1..{cap}")


def _require_positive_float(name: str, value: float, cap: float) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0.0 or float(value) > cap:
        raise MissionValidationError(f"{name} must be positive, finite, and <= {cap}")


def _require_nonnegative_float(name: str, value: float, cap: float) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)) or float(value) < 0.0 or float(value) > cap:
        raise MissionValidationError(f"{name} must be finite, non-negative, and <= {cap}")
