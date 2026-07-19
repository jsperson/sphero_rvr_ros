"""ROS-free mission_api.v2 typed rover capability registry and runtime.

The v2 layer treats planner output as untrusted JSON.  A planner may select only
registered deterministic tools, with bounded schemas, explicit availability,
approval classes, resource ownership, and auditable results.  This is not a ROS
bridge; adapters invoke project capabilities by stable tool ids only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Sequence

from .mission_api import MissionApiVersion, MissionValidationError


UNSAFE_ROS_SURFACES = (
    "/cmd_vel",
    "/cmd_vel_motor",
    "cmd_vel",
    "cmd_vel_motor",
    "raw_motor",
    "raw motor",
    "motor command",
    "teleop",
    "ros topic",
    "generic ros bridge",
)


class CapabilityAvailability(str, Enum):
    AVAILABLE = "available"
    UNAVAILABLE = "unavailable"
    UNSUPPORTED = "unsupported"


class ToolResultStatus(str, Enum):
    COMPLETE = "complete"
    FAILED = "failed"
    BLOCKED = "blocked"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    STOPPED = "stopped"
    ESTOPPED = "estopped"


class MissionRuntimeStatus(str, Enum):
    COMPLETE = "complete"
    FAILED = "failed"
    BLOCKED = "blocked"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    STOPPED = "stopped"
    ESTOPPED = "estopped"


def _positive_finite(value: float) -> bool:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(numeric) and numeric > 0.0


@dataclass(frozen=True)
class MissionBudgets:
    max_steps: int
    max_runtime_s: float
    max_travel_m: Optional[float] = None

    def __post_init__(self) -> None:
        if isinstance(self.max_steps, bool) or not isinstance(self.max_steps, int) or self.max_steps <= 0:
            raise MissionValidationError("mission budget max_steps must be positive")
        if isinstance(self.max_runtime_s, bool) or not isinstance(self.max_runtime_s, (int, float)):
            raise MissionValidationError("mission budget max_runtime_s must be positive and finite")
        if not _positive_finite(self.max_runtime_s):
            raise MissionValidationError("mission budget max_runtime_s must be positive and finite")
        if self.max_travel_m is not None and (isinstance(self.max_travel_m, bool) or not isinstance(self.max_travel_m, (int, float))):
            raise MissionValidationError("mission budget max_travel_m must be positive and finite when set")
        if self.max_travel_m is not None and not _positive_finite(self.max_travel_m):
            raise MissionValidationError("mission budget max_travel_m must be positive and finite when set")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "max_steps": self.max_steps,
            "max_runtime_s": self.max_runtime_s,
            "max_travel_m": self.max_travel_m,
        }


@dataclass(frozen=True)
class MissionGoal:
    goal_id: str
    objective: str
    success_criteria: Sequence[str]
    constraints: Mapping[str, Any] = field(default_factory=dict)
    execution_mode: str = "replay"
    budgets: MissionBudgets = field(default_factory=lambda: MissionBudgets(max_steps=8, max_runtime_s=120.0))
    requested_artifacts: Sequence[str] = field(default_factory=tuple)
    api_version: MissionApiVersion = MissionApiVersion.V2

    def __post_init__(self) -> None:
        if isinstance(self.api_version, str):
            object.__setattr__(self, "api_version", MissionApiVersion(self.api_version))
        if self.api_version is not MissionApiVersion.V2:
            raise MissionValidationError("MissionGoal requires mission_api.v2")
        if not str(self.goal_id).strip():
            raise MissionValidationError("goal_id is required")
        if not str(self.objective).strip():
            raise MissionValidationError("objective is required")
        if not tuple(self.success_criteria):
            raise MissionValidationError("success_criteria are required")
        object.__setattr__(self, "success_criteria", tuple(str(item) for item in self.success_criteria))
        object.__setattr__(self, "requested_artifacts", tuple(str(item) for item in self.requested_artifacts))

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": self.api_version.value,
            "goal_id": self.goal_id,
            "objective": self.objective,
            "constraints": dict(self.constraints),
            "success_criteria": list(self.success_criteria),
            "execution_mode": self.execution_mode,
            "budgets": self.budgets.to_json_dict(),
            "requested_artifacts": list(self.requested_artifacts),
        }


@dataclass(frozen=True)
class ApprovalGrant:
    approval_id: str
    approved_by: str
    approved_at_s: float
    expires_at_s: float
    approval_class: str

    def __post_init__(self) -> None:
        if not self.approval_id or not self.approved_by:
            raise MissionValidationError("approval id and approver are required")
        if not math.isfinite(float(self.approved_at_s)) or not math.isfinite(float(self.expires_at_s)):
            raise MissionValidationError("approval timestamps must be finite")

    def valid_for(self, approval_class: str, *, now_s: float) -> bool:
        return self.approval_class == approval_class and self.approved_at_s <= now_s <= self.expires_at_s

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "approval_id": self.approval_id,
            "approved_by": self.approved_by,
            "approved_at_s": self.approved_at_s,
            "expires_at_s": self.expires_at_s,
            "approval_class": self.approval_class,
        }


@dataclass(frozen=True)
class ToolDefinition:
    tool_id: str
    version: str
    argument_schema: Mapping[str, Any]
    result_schema: Mapping[str, Any]
    preconditions: Sequence[str] = field(default_factory=tuple)
    availability: CapabilityAvailability = CapabilityAvailability.AVAILABLE
    timeout_s: float = 5.0
    cancellation: str = "cooperative"
    safety_class: str = "read_only"
    approval_class: str = "none"
    resource_ownership: Sequence[str] = field(default_factory=tuple)
    effects: Sequence[str] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if isinstance(self.availability, str):
            object.__setattr__(self, "availability", CapabilityAvailability(self.availability))
        if not self.tool_id or not self.version:
            raise MissionValidationError("tool id and version are required")
        if not _positive_finite(self.timeout_s):
            raise MissionValidationError(f"{self.tool_id} timeout_s must be positive and finite")
        object.__setattr__(self, "preconditions", tuple(str(item) for item in self.preconditions))
        object.__setattr__(self, "resource_ownership", tuple(str(item) for item in self.resource_ownership))
        object.__setattr__(self, "effects", tuple(str(item) for item in self.effects))

    def requires_approval(self) -> bool:
        return self.approval_class != "none"

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "tool_id": self.tool_id,
            "version": self.version,
            "argument_schema": dict(self.argument_schema),
            "result_schema": dict(self.result_schema),
            "preconditions": list(self.preconditions),
            "availability": self.availability.value,
            "timeout_s": self.timeout_s,
            "cancellation": self.cancellation,
            "safety_class": self.safety_class,
            "approval_class": self.approval_class,
            "resource_ownership": list(self.resource_ownership),
            "effects": list(self.effects),
        }


@dataclass(frozen=True)
class ToolInvocation:
    correlation_id: str
    tool_id: str
    tool_version: str
    arguments: Mapping[str, Any]
    approval: Optional[ApprovalGrant] = None
    requested_at_s: float = 0.0
    provenance: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not str(self.correlation_id).strip():
            raise MissionValidationError("correlation_id is required")
        if not str(self.tool_id).strip() or not str(self.tool_version).strip():
            raise MissionValidationError("tool id and version are required")
        if not isinstance(self.arguments, Mapping):
            raise MissionValidationError("tool invocation arguments must be an object")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "correlation_id": self.correlation_id,
            "tool_id": self.tool_id,
            "tool_version": self.tool_version,
            "arguments": dict(self.arguments),
            "approval": None if self.approval is None else self.approval.to_json_dict(),
            "requested_at_s": self.requested_at_s,
            "provenance": dict(self.provenance),
        }


@dataclass(frozen=True)
class ToolResult:
    invocation: ToolInvocation
    status: ToolResultStatus
    started_at_s: float
    completed_at_s: float
    observation: Mapping[str, Any] = field(default_factory=dict)
    error: Mapping[str, Any] = field(default_factory=dict)
    artifact_refs: Mapping[str, str] = field(default_factory=dict)
    provenance: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if isinstance(self.status, str):
            object.__setattr__(self, "status", ToolResultStatus(self.status))

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "correlation_id": self.invocation.correlation_id,
            "tool_id": self.invocation.tool_id,
            "tool_version": self.invocation.tool_version,
            "status": self.status.value,
            "started_at_s": self.started_at_s,
            "completed_at_s": self.completed_at_s,
            "observation": dict(self.observation),
            "error": dict(self.error),
            "artifact_refs": dict(self.artifact_refs),
            "provenance": dict(self.provenance),
        }


@dataclass(frozen=True)
class MissionPlan:
    goal: MissionGoal
    invocations: Sequence[ToolInvocation]
    plan_id: str = "mission-plan"
    dependencies: Sequence[tuple[str, str]] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if not tuple(self.invocations):
            raise MissionValidationError("mission plan requires at least one invocation")
        object.__setattr__(self, "invocations", tuple(self.invocations))
        object.__setattr__(self, "dependencies", tuple(self.dependencies))
        if self.dependencies:
            raise MissionValidationError("mission plan dependencies are not supported yet")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "plan_id": self.plan_id,
            "goal": self.goal.to_json_dict(),
            "invocations": [invocation.to_json_dict() for invocation in self.invocations],
            "dependencies": [list(edge) for edge in self.dependencies],
        }


@dataclass(frozen=True)
class MissionRuntimeResult:
    plan: MissionPlan
    status: MissionRuntimeStatus
    results: Sequence[ToolResult]
    audit: Sequence[Mapping[str, Any]]

    def __post_init__(self) -> None:
        if isinstance(self.status, str):
            object.__setattr__(self, "status", MissionRuntimeStatus(self.status))
        object.__setattr__(self, "results", tuple(self.results))
        object.__setattr__(self, "audit", tuple(dict(item) for item in self.audit))

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": MissionApiVersion.V2.value,
            "status": self.status.value,
            "plan": self.plan.to_json_dict(),
            "results": [result.to_json_dict() for result in self.results],
            "audit": [dict(item) for item in self.audit],
        }


class CapabilityRegistry:
    def __init__(self, definitions: Sequence[ToolDefinition]):
        self._definitions = {(definition.tool_id, definition.version): definition for definition in definitions}

    def require(self, tool_id: str, version: str) -> ToolDefinition:
        try:
            return self._definitions[(tool_id, version)]
        except KeyError as exc:
            raise MissionValidationError(f"unknown tool: {tool_id}@{version}") from exc

    def definitions(self) -> tuple[ToolDefinition, ...]:
        return tuple(self._definitions[key] for key in sorted(self._definitions))

    def to_json_dict(self) -> dict[str, Any]:
        return {f"{definition.tool_id}@{definition.version}": definition.to_json_dict() for definition in self.definitions()}


@dataclass
class FakeCapabilityAdapters:
    fail_tools: Mapping[str, str] = field(default_factory=dict)
    block_tools: Mapping[str, str] = field(default_factory=dict)
    duration_by_tool: Mapping[str, float] = field(default_factory=dict)
    observation_by_tool: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    artifact_refs_by_tool: Mapping[str, Mapping[str, str]] = field(default_factory=dict)
    provenance_by_tool: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    cancel_after: Optional[int] = None
    stop_before: Optional[str] = None
    estop_before: Optional[str] = None

    def execute(self, invocation: ToolInvocation, definition: ToolDefinition, *, started_at_s: float, index: int) -> ToolResult:
        duration = float(self.duration_by_tool.get(invocation.tool_id, min(0.25, definition.timeout_s)))
        completed_at_s = started_at_s + duration
        if self.cancel_after is not None and index >= self.cancel_after:
            return _tool_result(invocation, ToolResultStatus.CANCELLED, started_at_s, completed_at_s, error="cancel requested")
        if self.stop_before == invocation.tool_id:
            return _tool_result(invocation, ToolResultStatus.STOPPED, started_at_s, completed_at_s, error="STOP propagated")
        if self.estop_before == invocation.tool_id:
            return _tool_result(invocation, ToolResultStatus.ESTOPPED, started_at_s, completed_at_s, error="ESTOP propagated")
        if invocation.tool_id in self.block_tools:
            return _tool_result(invocation, ToolResultStatus.BLOCKED, started_at_s, completed_at_s, error=self.block_tools[invocation.tool_id])
        if invocation.tool_id in self.fail_tools:
            return _tool_result(invocation, ToolResultStatus.FAILED, started_at_s, completed_at_s, error=self.fail_tools[invocation.tool_id])
        if invocation.tool_id == "pause_cancel_stop_estop":
            action = str(invocation.arguments["action"])
            status_by_action = {
                "cancel": ToolResultStatus.CANCELLED,
                "stop": ToolResultStatus.STOPPED,
                "estop": ToolResultStatus.ESTOPPED,
            }
            status = status_by_action.get(action, ToolResultStatus.BLOCKED)
            return ToolResult(
                invocation=invocation,
                status=status,
                started_at_s=started_at_s,
                completed_at_s=completed_at_s,
                observation={"latched_state": action.upper()},
                provenance={"adapter": "fake/replay", "deterministic": True},
            )
        observation, artifact_refs = _fake_observation(invocation)
        if invocation.tool_id in self.observation_by_tool:
            observation = dict(self.observation_by_tool[invocation.tool_id])
        if invocation.tool_id in self.artifact_refs_by_tool:
            artifact_refs = dict(self.artifact_refs_by_tool[invocation.tool_id])
        provenance = {"adapter": "fake/replay", "deterministic": True}
        if invocation.tool_id in self.provenance_by_tool:
            provenance.update(self.provenance_by_tool[invocation.tool_id])
        return ToolResult(
            invocation=invocation,
            status=ToolResultStatus.COMPLETE,
            started_at_s=started_at_s,
            completed_at_s=completed_at_s,
            observation=observation,
            artifact_refs=artifact_refs,
            provenance=provenance,
        )


class DeterministicMissionRuntime:
    def __init__(
        self,
        registry: CapabilityRegistry,
        adapters: FakeCapabilityAdapters,
        *,
        now_s: float = 0.0,
        budget_ceilings: Optional[MissionBudgets] = None,
    ):
        self.registry = registry
        self.adapters = adapters
        self.now_s = float(now_s)
        self.budget_ceilings = budget_ceilings or MissionBudgets(max_steps=8, max_runtime_s=120.0, max_travel_m=2.0)

    def execute_plan(self, plan: MissionPlan) -> MissionRuntimeResult:
        self._validate_plan_budgets(plan)
        results: list[ToolResult] = []
        audit: list[dict[str, Any]] = []
        elapsed_s = 0.0
        for index, invocation in enumerate(plan.invocations):
            started_at = self.now_s + elapsed_s
            definition = self._validate_invocation(invocation, plan, now_s=started_at)
            remaining_runtime_s = plan.goal.budgets.max_runtime_s - elapsed_s
            if _effective_timeout_s(definition, invocation) > remaining_runtime_s:
                result = _tool_result(
                    invocation,
                    ToolResultStatus.TIMEOUT,
                    started_at,
                    started_at,
                    error="tool timeout exceeds remaining mission runtime budget",
                )
                results.append(result)
                audit.append(self._audit_entry(invocation, definition, result, elapsed_s))
                return MissionRuntimeResult(plan, MissionRuntimeStatus.TIMEOUT, tuple(results), tuple(audit))
            result = self.adapters.execute(invocation, definition, started_at_s=started_at, index=index)
            elapsed_s += max(0.0, result.completed_at_s - result.started_at_s)
            if result.completed_at_s - result.started_at_s > _effective_timeout_s(definition, invocation):
                result = _tool_result(
                    invocation,
                    ToolResultStatus.TIMEOUT,
                    result.started_at_s,
                    result.started_at_s + _effective_timeout_s(definition, invocation),
                    error=f"tool exceeded timeout_s={_effective_timeout_s(definition, invocation):g}; cancellation cleanup completed",
                )
                elapsed_s = max(0.0, result.completed_at_s - result.started_at_s)
            else:
                result = _validate_result_boundary(result, definition)
            if elapsed_s > plan.goal.budgets.max_runtime_s and result.status is ToolResultStatus.COMPLETE:
                result = _tool_result(
                    invocation,
                    ToolResultStatus.TIMEOUT,
                    result.started_at_s,
                    result.completed_at_s,
                    error="mission runtime budget exceeded",
                )
            results.append(result)
            audit.append(self._audit_entry(invocation, definition, result, elapsed_s))
            if result.status is not ToolResultStatus.COMPLETE:
                return MissionRuntimeResult(plan, _mission_status_for(result.status), tuple(results), tuple(audit))
        return MissionRuntimeResult(plan, MissionRuntimeStatus.COMPLETE, tuple(results), tuple(audit))

    def _validate_plan_budgets(self, plan: MissionPlan) -> None:
        _reject_direct_ros_surfaces(plan.goal.to_json_dict())
        if plan.goal.budgets.max_steps > self.budget_ceilings.max_steps:
            raise MissionValidationError("mission plan exceeds trusted max_steps ceiling")
        if plan.goal.budgets.max_runtime_s > self.budget_ceilings.max_runtime_s:
            raise MissionValidationError("mission plan exceeds trusted max_runtime_s ceiling")
        if len(plan.invocations) > plan.goal.budgets.max_steps:
            raise MissionValidationError("mission plan exceeds max_steps budget")
        travel = 0.0
        for invocation in plan.invocations:
            if invocation.tool_id in {"move_to_clearance", "bounded_exploration_segment"}:
                if "max_travel_m" not in invocation.arguments:
                    raise MissionValidationError(f"{invocation.tool_id} requires bounded max_travel_m")
                try:
                    max_travel_m = float(invocation.arguments["max_travel_m"])
                except (TypeError, ValueError) as exc:
                    raise MissionValidationError(f"{invocation.tool_id}.max_travel_m must be finite") from exc
                if not math.isfinite(max_travel_m):
                    raise MissionValidationError(f"{invocation.tool_id}.max_travel_m must be finite")
                travel += max_travel_m
        if self.budget_ceilings.max_travel_m is not None and travel > 0.0:
            if plan.goal.budgets.max_travel_m is None:
                raise MissionValidationError("mission plan requires max_travel_m within trusted ceiling")
            if plan.goal.budgets.max_travel_m > self.budget_ceilings.max_travel_m:
                raise MissionValidationError("mission plan exceeds trusted max_travel_m ceiling")
        if plan.goal.budgets.max_travel_m is not None and travel > plan.goal.budgets.max_travel_m:
            raise MissionValidationError("mission plan exceeds max_travel_m budget")

    def _validate_invocation(self, invocation: ToolInvocation, plan: MissionPlan, *, now_s: float) -> ToolDefinition:
        definition = self.registry.require(invocation.tool_id, invocation.tool_version)
        if definition.availability is not CapabilityAvailability.AVAILABLE:
            raise MissionValidationError(f"tool {invocation.tool_id} is {definition.availability.value}/unavailable")
        _reject_direct_ros_surfaces(invocation.arguments)
        _validate_schema(invocation.arguments, definition.argument_schema, path=invocation.tool_id)
        if definition.requires_approval():
            if invocation.approval is None or not invocation.approval.valid_for(definition.approval_class, now_s=now_s):
                raise MissionValidationError(f"approval is stale or missing for {invocation.tool_id}")
        del plan
        return definition

    @staticmethod
    def _audit_entry(
        invocation: ToolInvocation,
        definition: ToolDefinition,
        result: ToolResult,
        elapsed_s: float,
    ) -> dict[str, Any]:
        return {
            "api_version": MissionApiVersion.V2.value,
            "correlation_id": invocation.correlation_id,
            "tool_id": invocation.tool_id,
            "tool_version": invocation.tool_version,
            "status": result.status.value,
            "safety_class": definition.safety_class,
            "approval_class": definition.approval_class,
            "resource_ownership": list(definition.resource_ownership),
            "elapsed_s": elapsed_s,
            "direct_ros_surface_exposed": False,
        }


def build_default_v2_registry(
    *,
    detector_classes: Sequence[str] = ("shoe",),
    availability: Optional[Mapping[str, CapabilityAvailability]] = None,
) -> CapabilityRegistry:
    available = dict(availability or {})

    def avail(tool_id: str) -> CapabilityAvailability:
        value = available.get(tool_id, CapabilityAvailability.AVAILABLE)
        return CapabilityAvailability(value)

    definitions = (
        ToolDefinition(
            "map_localize",
            "1.0",
            _schema({"mode": {"type": "string", "enum": ["replay", "live"]}}, required=()),
            _schema({"map_frame": {"type": "string"}}),
            preconditions=("map/localization adapter installed",),
            availability=avail("map_localize"),
            timeout_s=10.0,
            safety_class="localization",
            resource_ownership=("map", "localizer"),
            effects=("loads or replays an allowlisted map/localization workflow",),
        ),
        ToolDefinition(
            "bounded_exploration_segment",
            "1.0",
            _schema(
                {
                    "max_segments": {"type": "integer", "minimum": 1, "maximum": 8},
                    "segment_timeout_s": {"type": "number", "minimum": 0.1, "maximum": 30.0},
                    "max_travel_m": {"type": "number", "minimum": 0.01, "maximum": 2.0},
                }
            ),
            _schema({"completed_segments": {"type": "integer"}}),
            preconditions=("supervised coordinator and collision stop are available",),
            availability=avail("bounded_exploration_segment"),
            timeout_s=30.0,
            safety_class="supervised_motion",
            approval_class="supervised_motion",
            resource_ownership=("supervised_coordinator", "range_motion"),
            effects=("runs bounded deterministic exploration through range_motion and collision_stop",),
        ),
        ToolDefinition(
            "move_to_clearance",
            "1.0",
            _schema(
                {
                    "clearance_m": {"type": "number", "minimum": 0.05, "maximum": 2.0},
                    "speed_mps": {"type": "number", "minimum": 0.01, "maximum": 0.2},
                    "timeout_s": {"type": "number", "minimum": 0.1, "maximum": 30.0},
                    "max_travel_m": {"type": "number", "minimum": 0.01, "maximum": 2.0},
                }
            ),
            _schema({"target_clearance_m": {"type": "number"}}),
            preconditions=("range target visible", "collision stop clear"),
            availability=avail("move_to_clearance"),
            timeout_s=30.0,
            safety_class="supervised_motion",
            approval_class="supervised_motion",
            resource_ownership=("range_motion",),
            effects=("requests bounded range_motion only; no direct motor writes",),
        ),
        ToolDefinition(
            "rotate_scan",
            "1.0",
            _schema({"angle_deg": {"type": "number", "minimum": -180.0, "maximum": 180.0}}),
            _schema({"scan_ref": {"type": "string"}}),
            availability=avail("rotate_scan"),
            timeout_s=10.0,
            safety_class="supervised_motion",
            approval_class="supervised_motion",
            resource_ownership=("range_motion",),
            effects=("bounded rotate/scan when adapter exists",),
        ),
        ToolDefinition(
            "capture_observation",
            "1.0",
            _schema({"sensor": {"type": "string", "enum": ["replay", "camera", "lidar"]}}, required=()),
            _schema({"observation_ref": {"type": "string"}}),
            availability=avail("capture_observation"),
            timeout_s=5.0,
            safety_class="perception",
            resource_ownership=("camera", "lidar"),
            effects=("captures or replays an observation from allowlisted sensors",),
        ),
        ToolDefinition(
            "detect_objects",
            "1.0",
            _schema({"object_class": {"type": "string", "enum": list(detector_classes)}}),
            _schema({"detections_ref": {"type": "string"}}),
            preconditions=("detector plugin installed for requested object_class",),
            availability=avail("detect_objects"),
            timeout_s=10.0,
            safety_class="perception",
            resource_ownership=("detector",),
            effects=("runs an installed detector plugin for the requested object class",),
        ),
        ToolDefinition(
            "project_detections_to_map",
            "1.0",
            _schema({"target_frame": {"type": "string", "enum": ["map"]}}, required=()),
            _schema({"map_observations_ref": {"type": "string"}}),
            availability=avail("project_detections_to_map"),
            timeout_s=5.0,
            safety_class="semantic_mapping",
            resource_ownership=("semantic_projector",),
            effects=("projects detector observations into the map frame",),
        ),
        ToolDefinition(
            "generate_semantic_artifacts",
            "1.0",
            _schema(
                {
                    "artifact_kinds": {
                        "type": "array",
                        "items": {
                            "type": "string",
                            "enum": ["semantic_map", "geojson", "annotated_map", "coverage_report", "mission_summary"],
                        },
                        "minItems": 1,
                    }
                }
            ),
            _schema({"artifact_refs": {"type": "object"}}),
            availability=avail("generate_semantic_artifacts"),
            timeout_s=10.0,
            safety_class="artifact_generation",
            resource_ownership=("artifact_writer",),
            effects=("writes semantic/artifact references from validated pipeline outputs",),
        ),
        ToolDefinition(
            "query_status_telemetry",
            "1.0",
            _schema({}, required=()),
            _schema({"state": {"type": "string"}}),
            availability=avail("query_status_telemetry"),
            timeout_s=2.0,
            safety_class="read_only",
            resource_ownership=("mission_runtime",),
            effects=("returns read-only mission telemetry",),
        ),
        ToolDefinition(
            "pause_cancel_stop_estop",
            "1.0",
            _schema({"action": {"type": "string", "enum": ["pause", "cancel", "stop", "estop"]}}),
            _schema({"latched_state": {"type": "string"}}),
            availability=avail("pause_cancel_stop_estop"),
            timeout_s=2.0,
            cancellation="authoritative",
            safety_class="safety_control",
            approval_class="none",
            resource_ownership=("mission_runtime", "robot_side_safety"),
            effects=("propagates cancellation/STOP/ESTOP semantics to deterministic runtime boundary",),
        ),
    )
    return CapabilityRegistry(definitions)


def build_canonical_shoe_mapping_v2_plan(*, goal_id: str = "shoe-room-map", approval: Optional[ApprovalGrant] = None) -> MissionPlan:
    goal = MissionGoal(
        goal_id=goal_id,
        objective="Map the room and identify every shoe; produce semantic artifacts bounded to observed coverage.",
        constraints={"area": "room", "coverage": "observed_only"},
        success_criteria=(
            "occupancy/localization workflow completes",
            "shoe detections are projected into map frame",
            "semantic artifacts are referenced",
        ),
        execution_mode="replay",
        budgets=MissionBudgets(max_steps=8, max_runtime_s=120.0, max_travel_m=2.0),
        requested_artifacts=("semantic_map", "mission_summary"),
    )
    return MissionPlan(
        plan_id=f"{goal_id}-v2-plan",
        goal=goal,
        invocations=(
            ToolInvocation("shoe-1", "map_localize", "1.0", {"mode": "replay"}),
            ToolInvocation(
                "shoe-2",
                "bounded_exploration_segment",
                "1.0",
                {"max_segments": 2, "segment_timeout_s": 8.0, "max_travel_m": 1.0},
                approval=approval,
            ),
            ToolInvocation("shoe-3", "capture_observation", "1.0", {"sensor": "replay"}),
            ToolInvocation("shoe-4", "detect_objects", "1.0", {"object_class": "shoe"}),
            ToolInvocation("shoe-5", "project_detections_to_map", "1.0", {"target_frame": "map"}),
            ToolInvocation(
                "shoe-6",
                "generate_semantic_artifacts",
                "1.0",
                {"artifact_kinds": ["semantic_map", "geojson", "coverage_report", "mission_summary"]},
            ),
        ),
    )


def _schema(properties: Mapping[str, Any], *, required: Optional[Sequence[str]] = None) -> dict[str, Any]:
    return {
        "type": "object",
        "properties": dict(properties),
        "required": list(properties if required is None else required),
        "additionalProperties": False,
    }


def _validate_schema(value: Any, schema: Mapping[str, Any], *, path: str) -> None:
    expected = schema.get("type")
    if expected == "object":
        if not isinstance(value, Mapping):
            raise MissionValidationError(f"{path} must be an object")
        properties = schema.get("properties", {})
        required = schema.get("required", [])
        for key in required:
            if key not in value:
                raise MissionValidationError(f"{path}.{key} is required")
        if schema.get("additionalProperties") is False:
            extras = [key for key in value if key not in properties]
            if extras:
                raise MissionValidationError(f"{path} contains unsupported argument: {extras[0]}")
        for key, item in value.items():
            if key in properties:
                _validate_schema(item, properties[key], path=f"{path}.{key}")
        return
    if expected == "array":
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise MissionValidationError(f"{path} must be an array")
        min_items = schema.get("minItems")
        if min_items is not None and len(value) < int(min_items):
            raise MissionValidationError(f"{path} must contain at least {min_items} items")
        for index, item in enumerate(value):
            _validate_schema(item, schema.get("items", {}), path=f"{path}[{index}]")
        return
    if expected == "string":
        if not isinstance(value, str):
            raise MissionValidationError(f"{path} must be a string")
    elif expected == "number":
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise MissionValidationError(f"{path} must be a finite number")
        if "minimum" in schema and float(value) < float(schema["minimum"]):
            raise MissionValidationError(f"{path} below minimum")
        if "maximum" in schema and float(value) > float(schema["maximum"]):
            raise MissionValidationError(f"{path} above maximum")
    elif expected == "integer":
        if isinstance(value, bool) or not isinstance(value, int):
            raise MissionValidationError(f"{path} must be an integer")
        if "minimum" in schema and value < int(schema["minimum"]):
            raise MissionValidationError(f"{path} below minimum")
        if "maximum" in schema and value > int(schema["maximum"]):
            raise MissionValidationError(f"{path} above maximum")
    elif expected is not None:
        raise MissionValidationError(f"unsupported schema type at {path}: {expected}")
    if "enum" in schema and value not in schema["enum"]:
        raise MissionValidationError(f"{path} value {value!r} is not allowed")


def _validate_result_boundary(result: ToolResult, definition: ToolDefinition) -> ToolResult:
    try:
        _reject_direct_ros_surfaces(result.observation)
        _reject_direct_ros_surfaces(result.artifact_refs)
        _reject_direct_ros_surfaces(result.error)
        _reject_direct_ros_surfaces(result.provenance)
    except MissionValidationError:
        return _tool_result(
            result.invocation,
            ToolResultStatus.FAILED,
            result.started_at_s,
            result.completed_at_s,
            error="adapter result failed boundary validation",
        )
    if result.status is not ToolResultStatus.COMPLETE:
        return result
    try:
        _validate_schema(result.observation, definition.result_schema, path=f"{definition.tool_id}.result")
    except MissionValidationError as exc:
        return _tool_result(
            result.invocation,
            ToolResultStatus.FAILED,
            result.started_at_s,
            result.completed_at_s,
            error=f"adapter result failed schema validation: {exc}",
        )
    if result.artifact_refs:
        expected_artifacts = result.observation.get("artifact_refs") if isinstance(result.observation, Mapping) else None
        if expected_artifacts != result.artifact_refs:
            return _tool_result(
                result.invocation,
                ToolResultStatus.FAILED,
                result.started_at_s,
                result.completed_at_s,
                error="adapter artifact_refs do not match declared result schema",
            )
    return result


def _reject_direct_ros_surfaces(value: Any) -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            _reject_direct_ros_surfaces(key)
            _reject_direct_ros_surfaces(item)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for item in value:
            _reject_direct_ros_surfaces(item)
    elif isinstance(value, str):
        lowered = value.lower()
        if any(surface in lowered for surface in UNSAFE_ROS_SURFACES):
            raise MissionValidationError(f"direct ROS surface is not allowed: {value}")


def _effective_timeout_s(definition: ToolDefinition, invocation: ToolInvocation) -> float:
    candidates = [float(definition.timeout_s)]
    for key in ("timeout_s", "segment_timeout_s"):
        if key not in invocation.arguments:
            continue
        try:
            value = float(invocation.arguments[key])
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and value > 0.0:
            candidates.append(value)
    return min(candidates)


def _fake_observation(invocation: ToolInvocation) -> tuple[dict[str, Any], dict[str, str]]:
    tool_id = invocation.tool_id
    if tool_id == "map_localize":
        return {"map_frame": "map"}, {}
    if tool_id == "bounded_exploration_segment":
        return {"completed_segments": int(invocation.arguments["max_segments"])}, {}
    if tool_id == "move_to_clearance":
        return {"target_clearance_m": float(invocation.arguments["clearance_m"])}, {}
    if tool_id == "rotate_scan":
        return {"scan_ref": "artifacts/replay/rotate_scan.json"}, {}
    if tool_id == "capture_observation":
        return {"observation_ref": "artifacts/replay/observation.json"}, {}
    if tool_id == "detect_objects":
        object_class = str(invocation.arguments["object_class"])
        return {"detections_ref": f"artifacts/replay/{object_class}_detections.json"}, {}
    if tool_id == "project_detections_to_map":
        return {"map_observations_ref": "artifacts/replay/map_observations.json"}, {}
    if tool_id == "generate_semantic_artifacts":
        refs = _artifact_refs(invocation.arguments["artifact_kinds"])
        return {"artifact_refs": refs}, refs
    if tool_id == "query_status_telemetry":
        return {"state": "RUNNING"}, {}
    return {"tool_id": tool_id}, {}


def _artifact_refs(kinds: Sequence[str]) -> dict[str, str]:
    suffixes = {
        "semantic_map": "semantic_map.json",
        "geojson": "semantic_map.geojson",
        "annotated_map": "annotated_semantic_map.ppm",
        "coverage_report": "coverage_uncertainty_report.md",
        "mission_summary": "mission_summary.md",
    }
    return {kind: f"artifacts/vs06_semantic_map/{suffixes[kind]}" for kind in kinds}


def _tool_result(
    invocation: ToolInvocation,
    status: ToolResultStatus,
    started_at_s: float,
    completed_at_s: float,
    *,
    error: str,
) -> ToolResult:
    return ToolResult(
        invocation=invocation,
        status=status,
        started_at_s=started_at_s,
        completed_at_s=completed_at_s,
        error={"message": error},
        provenance={"adapter": "fake/replay", "deterministic": True},
    )


def _mission_status_for(status: ToolResultStatus) -> MissionRuntimeStatus:
    return MissionRuntimeStatus(status.value)
