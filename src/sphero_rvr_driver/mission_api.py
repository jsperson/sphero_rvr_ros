"""ROS-free typed rover Mission API capability registry and runtime.

The canonical Mission API treats planner output as untrusted JSON.  A planner may
select only registered deterministic tools, with bounded schemas, explicit
availability, approval classes, resource ownership, and auditable results.  This
is not a ROS bridge; adapters invoke project capabilities by stable tool ids
only. The serialized wire/schema identifier remains ``mission_api.v2`` for
contract evolution even though there is only one implementation module.
"""

from __future__ import annotations

import math
import multiprocessing as mp
from pathlib import Path
import queue
import subprocess
import hashlib
import json
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Protocol, Sequence

class MissionValidationError(ValueError):
    """Raised when a Mission API request or runtime boundary is invalid."""


class MissionApiVersion(str, Enum):
    V2 = "mission_api.v2"


_PHYSICAL_ADAPTER_AUTHORITY = object()


def physical_adapter_authority() -> object:
    """Return the process-local marker used by reviewed physical adapters."""

    return _PHYSICAL_ADAPTER_AUTHORITY


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


class CriterionKind(str, Enum):
    TOOL_COMPLETE = "tool_complete"
    ARTIFACT_PRESENT = "artifact_present"
    OBSERVATION_EQUALS = "observation_equals"
    TEXT_EVIDENCE = "text_evidence"


@dataclass(frozen=True)
class SuccessCriterion:
    """A deterministic predicate evaluated only against validated runtime evidence."""

    criterion_id: str
    description: str
    kind: CriterionKind
    tool_id: str = ""
    field: str = ""
    expected: Any = None

    def __post_init__(self) -> None:
        if isinstance(self.kind, str):
            object.__setattr__(self, "kind", CriterionKind(self.kind))
        if not str(self.criterion_id).strip() or not str(self.description).strip():
            raise MissionValidationError("success criterion id and description are required")
        if self.kind in {CriterionKind.TOOL_COMPLETE, CriterionKind.OBSERVATION_EQUALS} and not str(self.tool_id).strip():
            raise MissionValidationError(f"{self.kind.value} criterion requires tool_id")
        if self.kind in {CriterionKind.ARTIFACT_PRESENT, CriterionKind.OBSERVATION_EQUALS} and not str(self.field).strip():
            raise MissionValidationError(f"{self.kind.value} criterion requires field")

    def to_json_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "criterion_id": self.criterion_id,
            "description": self.description,
            "kind": self.kind.value,
        }
        if self.tool_id:
            payload["tool_id"] = self.tool_id
        if self.field:
            payload["field"] = self.field
        if self.kind is CriterionKind.OBSERVATION_EQUALS:
            payload["expected"] = self.expected
        return payload


@dataclass(frozen=True)
class CriterionResult:
    criterion_id: str
    satisfied: bool
    evidence: Mapping[str, Any] = field(default_factory=dict)
    reason: str = ""

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "criterion_id": self.criterion_id,
            "satisfied": self.satisfied,
            "evidence": dict(self.evidence),
            "reason": self.reason,
        }


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
    max_observations: Optional[int] = None
    max_artifacts: Optional[int] = None
    max_tool_calls: Optional[int] = None
    max_provider_calls: Optional[int] = None

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
        for field_name in ("max_observations", "max_artifacts", "max_tool_calls", "max_provider_calls"):
            value = getattr(self, field_name)
            if value is not None and (isinstance(value, bool) or not isinstance(value, int) or value <= 0):
                raise MissionValidationError(f"mission budget {field_name} must be positive when set")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "max_steps": self.max_steps,
            "max_runtime_s": self.max_runtime_s,
            "max_travel_m": self.max_travel_m,
            "max_observations": self.max_observations,
            "max_artifacts": self.max_artifacts,
            "max_tool_calls": self.max_tool_calls,
            "max_provider_calls": self.max_provider_calls,
        }


@dataclass(frozen=True)
class MissionGoal:
    goal_id: str
    objective: str
    success_criteria: Sequence[SuccessCriterion]
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
        criteria = tuple(
            item if isinstance(item, SuccessCriterion) else SuccessCriterion(
                criterion_id=f"criterion-{index + 1}",
                description=str(item),
                kind=CriterionKind.TEXT_EVIDENCE,
            )
            for index, item in enumerate(self.success_criteria)
        )
        if not criteria:
            raise MissionValidationError("success_criteria are required")
        criterion_ids = [item.criterion_id for item in criteria]
        if len(set(criterion_ids)) != len(criterion_ids):
            raise MissionValidationError("success criterion ids must be unique")
        object.__setattr__(self, "success_criteria", criteria)
        object.__setattr__(self, "requested_artifacts", tuple(str(item) for item in self.requested_artifacts))

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": self.api_version.value,
            "goal_id": self.goal_id,
            "objective": self.objective,
            "constraints": dict(self.constraints),
            "success_criteria": [criterion.to_json_dict() for criterion in self.success_criteria],
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
    mission_id: Optional[str] = None
    issued_to: Optional[str] = None
    tool_id: Optional[str] = None
    correlation_id: Optional[str] = None
    arguments_digest: Optional[str] = None
    principal: Optional[str] = None

    def __post_init__(self) -> None:
        if not self.approval_id or not self.approved_by:
            raise MissionValidationError("approval id and approver are required")
        if not math.isfinite(float(self.approved_at_s)) or not math.isfinite(float(self.expires_at_s)):
            raise MissionValidationError("approval timestamps must be finite")

    def valid_for(
        self,
        approval_class: str,
        *,
        now_s: float,
        mission_id: Optional[str] = None,
        issued_to: str = "mission-runtime",
    ) -> bool:
        if self.approval_class != approval_class or not (self.approved_at_s <= now_s <= self.expires_at_s):
            return False
        if self.mission_id != mission_id:
            return False
        if self.issued_to != issued_to:
            return False
        return True

    def rejection_reason(
        self,
        approval_class: str,
        *,
        now_s: float,
        mission_id: Optional[str] = None,
        issued_to: str = "mission-runtime",
    ) -> str:
        if self.approval_class != approval_class or not (self.approved_at_s <= now_s <= self.expires_at_s):
            return "approval is stale or missing"
        if self.mission_id != mission_id:
            return "approval mission binding mismatch"
        if self.issued_to != issued_to:
            return "approval identity binding mismatch"
        return "approval is stale or missing"

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "approval_id": self.approval_id,
            "approved_by": self.approved_by,
            "approved_at_s": self.approved_at_s,
            "expires_at_s": self.expires_at_s,
            "approval_class": self.approval_class,
            "mission_id": self.mission_id,
            "issued_to": self.issued_to,
            "tool_id": self.tool_id,
            "correlation_id": self.correlation_id,
            "arguments_digest": self.arguments_digest,
            "principal": self.principal,
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
    criterion_results: Sequence[CriterionResult] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if isinstance(self.status, str):
            object.__setattr__(self, "status", MissionRuntimeStatus(self.status))
        object.__setattr__(self, "results", tuple(self.results))
        object.__setattr__(self, "audit", tuple(dict(item) for item in self.audit))
        object.__setattr__(self, "criterion_results", tuple(self.criterion_results))

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": MissionApiVersion.V2.value,
            "status": self.status.value,
            "plan": self.plan.to_json_dict(),
            "results": [result.to_json_dict() for result in self.results],
            "audit": [dict(item) for item in self.audit],
            "criterion_results": [item.to_json_dict() for item in self.criterion_results],
        }


class CapabilityRegistry:
    def __init__(self, definitions: Sequence[ToolDefinition]):
        self._definitions: dict[tuple[str, str], ToolDefinition] = {}
        for definition in definitions:
            key = (definition.tool_id, definition.version)
            if key in self._definitions:
                raise MissionValidationError(f"duplicate tool definition: {definition.tool_id}@{definition.version}")
            self._definitions[key] = definition

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
    cooperative_execution: bool = field(default=True, init=False)
    execution_mode: str = "replay"
    authority_kind: str = "replay"
    healthy: bool = True
    evidence_level: str = "deterministic_replay"
    deployed_sha: str = field(default_factory=lambda: _source_sha())
    satisfied_preconditions: Optional[Sequence[str]] = field(default_factory=lambda: _default_satisfied_preconditions())
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


    def begin_execution(self, invocation: ToolInvocation, definition: ToolDefinition, *, started_at_s: float, index: int) -> "CompletedExecutionHandle":
        return CompletedExecutionHandle(self.execute(invocation, definition, started_at_s=started_at_s, index=index))


class AdapterExecutionHandle(Protocol):
    def wait(self, timeout_s: float) -> Optional[ToolResult]: ...
    def cancel(self) -> None: ...
    def cleanup(self, timeout_s: float) -> bool: ...
    def wait_idle(self, timeout_s: float) -> bool: ...


@dataclass
class CompletedExecutionHandle:
    """Already-quiescent handle for a bounded synchronous adapter."""

    result: ToolResult

    def wait(self, timeout_s: float) -> Optional[ToolResult]:
        del timeout_s
        return self.result

    def cancel(self) -> None:
        return None

    def cleanup(self, timeout_s: float) -> bool:
        del timeout_s
        return True

    def wait_idle(self, timeout_s: float) -> bool:
        del timeout_s
        return True


class CapabilityAdaptersProtocol(Protocol):
    cooperative_execution: bool

    def begin_execution(self, invocation: ToolInvocation, definition: ToolDefinition, *, started_at_s: float, index: int) -> AdapterExecutionHandle:
        ...


class DeterministicMissionRuntime:
    def __init__(
        self,
        registry: CapabilityRegistry,
        adapters: CapabilityAdaptersProtocol,
        *,
        now_s: float = 0.0,
        budget_ceilings: Optional[MissionBudgets] = None,
    ):
        self.registry = registry
        self.adapters = adapters
        self.now_s = float(now_s)
        self._explicit_budget_ceilings = budget_ceilings is not None
        self.budget_ceilings = budget_ceilings or MissionBudgets(max_steps=8, max_runtime_s=120.0, max_travel_m=2.0)
        self._ledger_steps = 0
        self._ledger_runtime_s = 0.0
        self._ledger_travel_m = 0.0
        self._ledger_observations = 0
        self._ledger_artifacts = 0
        self._ledger_tool_calls = 0
        self._ledger_provider_calls = 0
        self._ledger_approvals: set[str] = set()
        self._resource_owner: dict[str, str] = {}

    def capability_state(self) -> dict[str, dict[str, Any]]:
        mode = str(getattr(self.adapters, "execution_mode", "unknown"))
        healthy = bool(getattr(self.adapters, "healthy", False))
        evidence_level = str(getattr(self.adapters, "evidence_level", "unspecified"))
        deployed_sha = str(getattr(self.adapters, "deployed_sha", _source_sha()))
        supported = getattr(self.adapters, "supported_tool_ids", None)
        supported_tool_ids = None if supported is None else {str(item) for item in supported}
        return {
            f"{definition.tool_id}@{definition.version}": {
                "declared": True,
                "bound": definition.availability is CapabilityAvailability.AVAILABLE and (supported_tool_ids is None or definition.tool_id in supported_tool_ids),
                "healthy": healthy and definition.availability is CapabilityAvailability.AVAILABLE and (supported_tool_ids is None or definition.tool_id in supported_tool_ids),
                "mode": mode,
                "deployed_sha": deployed_sha,
                "evidence_level": evidence_level,
                "availability": definition.availability.value,
            }
            for definition in self.registry.definitions()
        }

    def execute_plan(self, plan: MissionPlan) -> MissionRuntimeResult:
        plan_travel_m = self._validate_plan_budgets(plan)
        self._validate_plan_resources(plan)
        physical_mode = plan.goal.execution_mode == "physical"
        if physical_mode and not _is_reviewed_physical_adapter(self.adapters):
            raise MissionValidationError("physical execution requires physical adapters")
        if not bool(getattr(self.adapters, "cooperative_execution", False)) or not callable(getattr(self.adapters, "begin_execution", None)):
            raise MissionValidationError("adapter unavailable: cooperative cancel/cleanup/quiescence contract required")
        results: list[ToolResult] = []
        audit: list[dict[str, Any]] = []
        elapsed_s = 0.0
        used_approvals: set[str] = set(self._ledger_approvals)
        self._validate_cumulative_ledger(plan, plan_travel_m)
        for index, invocation in enumerate(plan.invocations):
            started_at = self.now_s + elapsed_s
            definition = self._validate_invocation(invocation, plan, now_s=started_at, used_approvals=used_approvals, physical_mode=physical_mode)
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
            self._acquire_resources(definition, invocation)
            try:
                result = self._execute_adapter(invocation, definition, started_at_s=started_at, index=index)
            finally:
                self._release_resources(definition, invocation)
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
                result = _validate_result_boundary(result, definition, physical_mode=physical_mode)
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
                self._record_ledger(plan, results, elapsed_s, used_approvals)
                return MissionRuntimeResult(plan, _mission_status_for(result.status), tuple(results), tuple(audit))
        completion_budget_error = self._completion_budget_error(plan, results)
        if completion_budget_error:
            result = _tool_result(
                plan.invocations[-1],
                ToolResultStatus.FAILED,
                results[-1].started_at_s if results else self.now_s,
                results[-1].completed_at_s if results else self.now_s,
                error=completion_budget_error,
            )
            results[-1] = result
            audit[-1] = self._audit_entry(result.invocation, self.registry.require(result.invocation.tool_id, result.invocation.tool_version), result, elapsed_s)
            self._record_ledger(plan, results, elapsed_s, used_approvals)
            return MissionRuntimeResult(plan, MissionRuntimeStatus.FAILED, tuple(results), tuple(audit))
        missing_artifacts = _missing_requested_artifacts(plan.goal, results)
        if missing_artifacts:
            result = _tool_result(
                plan.invocations[-1],
                ToolResultStatus.FAILED,
                results[-1].started_at_s if results else self.now_s,
                results[-1].completed_at_s if results else self.now_s,
                error="mission completion missing requested artifacts: " + ", ".join(missing_artifacts),
            )
            results[-1] = result
            audit[-1] = self._audit_entry(result.invocation, self.registry.require(result.invocation.tool_id, result.invocation.tool_version), result, elapsed_s)
            self._record_ledger(plan, results, elapsed_s, used_approvals)
            return MissionRuntimeResult(plan, MissionRuntimeStatus.FAILED, tuple(results), tuple(audit))
        criterion_results = _evaluate_success_criteria(plan.goal, results)
        missing_criteria = tuple(item for item in criterion_results if not item.satisfied)
        if missing_criteria:
            result = _tool_result(
                plan.invocations[-1],
                ToolResultStatus.FAILED,
                results[-1].started_at_s if results else self.now_s,
                results[-1].completed_at_s if results else self.now_s,
                error="mission completion missing success criteria evidence: " + ", ".join(item.criterion_id for item in missing_criteria),
            )
            results[-1] = result
            audit[-1] = self._audit_entry(result.invocation, self.registry.require(result.invocation.tool_id, result.invocation.tool_version), result, elapsed_s)
            self._record_ledger(plan, results, elapsed_s, used_approvals)
            return MissionRuntimeResult(plan, MissionRuntimeStatus.FAILED, tuple(results), tuple(audit), tuple(criterion_results))
        self._record_ledger(plan, results, elapsed_s, used_approvals)
        return MissionRuntimeResult(plan, MissionRuntimeStatus.COMPLETE, tuple(results), tuple(audit), tuple(criterion_results))

    def _validate_plan_budgets(self, plan: MissionPlan) -> float:
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
            if invocation.tool_id == "move_distance":
                try:
                    distance_m = abs(float(invocation.arguments["distance_m"]))
                except (KeyError, TypeError, ValueError) as exc:
                    raise MissionValidationError("move_distance.distance_m must be finite") from exc
                if not math.isfinite(distance_m):
                    raise MissionValidationError("move_distance.distance_m must be finite")
                travel += distance_m
        if self.budget_ceilings.max_travel_m is not None and travel > 0.0:
            if plan.goal.budgets.max_travel_m is None:
                raise MissionValidationError("mission plan requires max_travel_m within trusted ceiling")
            if plan.goal.budgets.max_travel_m > self.budget_ceilings.max_travel_m:
                raise MissionValidationError("mission plan exceeds trusted max_travel_m ceiling")
        if plan.goal.budgets.max_travel_m is not None and travel > plan.goal.budgets.max_travel_m:
            raise MissionValidationError("mission plan exceeds max_travel_m budget")
        return travel

    def _validate_cumulative_ledger(self, plan: MissionPlan, plan_travel_m: float) -> None:
        if self._explicit_budget_ceilings and self._ledger_steps + len(plan.invocations) > self.budget_ceilings.max_steps:
            raise MissionValidationError("mission session exceeds cumulative max_steps budget")
        if self._explicit_budget_ceilings and self._ledger_runtime_s + plan.goal.budgets.max_runtime_s > self.budget_ceilings.max_runtime_s:
            raise MissionValidationError("mission session exceeds cumulative max_runtime_s budget")
        if self.budget_ceilings.max_travel_m is not None and self._ledger_travel_m + plan_travel_m > self.budget_ceilings.max_travel_m:
            raise MissionValidationError("mission session exceeds cumulative max_travel_m budget")
        plan_tool_calls = len(plan.invocations)
        if self.budget_ceilings.max_tool_calls is not None and self._ledger_tool_calls + plan_tool_calls > self.budget_ceilings.max_tool_calls:
            raise MissionValidationError("mission session exceeds cumulative max_tool_calls budget")
        if plan.goal.budgets.max_tool_calls is not None and plan_tool_calls > plan.goal.budgets.max_tool_calls:
            raise MissionValidationError("mission plan exceeds max_tool_calls budget")
        plan_provider_calls = _planned_provider_calls(plan)
        if self.budget_ceilings.max_provider_calls is not None and self._ledger_provider_calls + plan_provider_calls > self.budget_ceilings.max_provider_calls:
            raise MissionValidationError("mission session exceeds cumulative max_provider_calls budget")
        if plan.goal.budgets.max_provider_calls is not None and plan_provider_calls > plan.goal.budgets.max_provider_calls:
            raise MissionValidationError("mission plan exceeds max_provider_calls budget")

    def _record_ledger(self, plan: MissionPlan, results: Sequence[ToolResult], elapsed_s: float, used_approvals: set[str]) -> None:
        self._ledger_steps += len(results)
        self._ledger_runtime_s += elapsed_s
        self._ledger_travel_m += _planned_travel_m(plan)
        self._ledger_observations += sum(1 for result in results if result.observation)
        self._ledger_artifacts += sum(len(result.artifact_refs) for result in results)
        self._ledger_tool_calls += len(results)
        self._ledger_provider_calls += _provider_calls_from_results(results)
        self._ledger_approvals = set(used_approvals)

    def _validate_plan_resources(self, plan: MissionPlan) -> None:
        owners: dict[str, tuple[str, str]] = {}
        for invocation in plan.invocations:
            definition = self.registry.require(invocation.tool_id, invocation.tool_version)
            duplicates = {resource for resource in definition.resource_ownership if definition.resource_ownership.count(resource) > 1}
            if duplicates:
                raise MissionValidationError(f"duplicate exclusive resource declaration: {sorted(duplicates)[0]}")
            for resource in definition.resource_ownership:
                owner = owners.get(resource)
                if owner is not None:
                    owner_correlation_id, owner_tool_id = owner
                    if owner_correlation_id != invocation.correlation_id:
                        raise MissionValidationError(f"exclusive resource conflict: {resource} requested by {owner_correlation_id} and {invocation.correlation_id}")
                owners[resource] = (invocation.correlation_id, invocation.tool_id)

    def _validate_invocation(
        self,
        invocation: ToolInvocation,
        plan: MissionPlan,
        *,
        now_s: float,
        used_approvals: set[str],
        physical_mode: bool,
    ) -> ToolDefinition:
        definition = self.registry.require(invocation.tool_id, invocation.tool_version)
        if definition.availability is not CapabilityAvailability.AVAILABLE:
            raise MissionValidationError(f"tool {invocation.tool_id} is {definition.availability.value}/unavailable")
        capability = self.capability_state()[f"{definition.tool_id}@{definition.version}"]
        if not capability["bound"] or not capability["healthy"]:
            raise MissionValidationError(f"tool {invocation.tool_id} is not bound to a healthy adapter")
        self._validate_preconditions(definition, physical_mode=physical_mode)
        _reject_direct_ros_surfaces(invocation.arguments)
        _validate_schema(invocation.arguments, definition.argument_schema, path=invocation.tool_id)
        if invocation.tool_id == "turn_angle" and float(invocation.arguments["angle_deg"]) == 0.0:
            raise MissionValidationError("turn_angle.angle_deg must be non-zero")
        if definition.requires_approval():
            if invocation.approval is None:
                raise MissionValidationError(f"approval is stale or missing for {invocation.tool_id}")
            if invocation.approval.mission_id is None or invocation.approval.issued_to is None:
                raise MissionValidationError(f"approval requires mission and runtime identity binding for {invocation.tool_id}")
            if invocation.approval.mission_id != plan.goal.goal_id:
                raise MissionValidationError(f"approval mission binding mismatch for {invocation.tool_id}")
            if invocation.approval.issued_to != "mission-runtime":
                raise MissionValidationError(f"approval identity binding mismatch for {invocation.tool_id}")
            if not invocation.approval.valid_for(definition.approval_class, now_s=now_s, mission_id=plan.goal.goal_id):
                raise MissionValidationError(f"{invocation.approval.rejection_reason(definition.approval_class, now_s=now_s, mission_id=plan.goal.goal_id)} for {invocation.tool_id}")
            expected_digest = _arguments_digest(invocation.arguments)
            if invocation.approval.tool_id != invocation.tool_id:
                raise MissionValidationError(f"approval tool binding mismatch for {invocation.tool_id}")
            if invocation.approval.correlation_id != invocation.correlation_id:
                raise MissionValidationError(f"approval correlation binding mismatch for {invocation.tool_id}")
            if invocation.approval.arguments_digest != expected_digest:
                raise MissionValidationError(f"approval argument binding mismatch for {invocation.tool_id}")
            if invocation.approval.principal is None:
                raise MissionValidationError(f"approval principal binding required for {invocation.tool_id}")
            if invocation.approval.approval_id in used_approvals:
                raise MissionValidationError(f"approval replay detected for {invocation.tool_id}")
            used_approvals.add(invocation.approval.approval_id)
        return definition

    def _validate_preconditions(self, definition: ToolDefinition, *, physical_mode: bool) -> None:
        if not definition.preconditions:
            return
        satisfied = getattr(self.adapters, "satisfied_preconditions", None)
        if satisfied is None:
            if physical_mode:
                raise MissionValidationError(f"tool {definition.tool_id} preconditions are not attested by physical adapter")
            return
        satisfied_set = {str(item) for item in satisfied}
        missing = tuple(item for item in definition.preconditions if item not in satisfied_set)
        if missing:
            raise MissionValidationError(f"tool {definition.tool_id} precondition not satisfied: {missing[0]}")

    def _acquire_resources(self, definition: ToolDefinition, invocation: ToolInvocation) -> None:
        acquired: list[str] = []
        for resource in definition.resource_ownership:
            owner = self._resource_owner.get(resource)
            if owner is not None:
                for prior in acquired:
                    self._resource_owner.pop(prior, None)
                raise MissionValidationError(f"exclusive resource conflict: {resource} owned by {owner}")
            self._resource_owner[resource] = invocation.correlation_id
            acquired.append(resource)

    def _release_resources(self, definition: ToolDefinition, invocation: ToolInvocation) -> None:
        for resource in definition.resource_ownership:
            if self._resource_owner.get(resource) == invocation.correlation_id:
                self._resource_owner.pop(resource, None)

    def _completion_budget_error(self, plan: MissionPlan, results: Sequence[ToolResult]) -> str:
        observations = sum(1 for result in results if result.observation)
        artifacts = sum(len(result.artifact_refs) for result in results)
        provider_calls = _provider_calls_from_results(results)
        if plan.goal.budgets.max_observations is not None and observations > plan.goal.budgets.max_observations:
            return "mission plan exceeds max_observations budget"
        if self.budget_ceilings.max_observations is not None and self._ledger_observations + observations > self.budget_ceilings.max_observations:
            return "mission session exceeds cumulative max_observations budget"
        if plan.goal.budgets.max_artifacts is not None and artifacts > plan.goal.budgets.max_artifacts:
            return "mission plan exceeds max_artifacts budget"
        if self.budget_ceilings.max_artifacts is not None and self._ledger_artifacts + artifacts > self.budget_ceilings.max_artifacts:
            return "mission session exceeds cumulative max_artifacts budget"
        if plan.goal.budgets.max_provider_calls is not None and provider_calls > plan.goal.budgets.max_provider_calls:
            return "mission plan exceeds max_provider_calls budget"
        if self.budget_ceilings.max_provider_calls is not None and self._ledger_provider_calls + provider_calls > self.budget_ceilings.max_provider_calls:
            return "mission session exceeds cumulative max_provider_calls budget"
        return ""

    def _execute_adapter(self, invocation: ToolInvocation, definition: ToolDefinition, *, started_at_s: float, index: int) -> ToolResult:
        timeout_s = _effective_timeout_s(definition, invocation)
        begin_execution = getattr(self.adapters, "begin_execution", None)
        if not callable(begin_execution):
            return _execute_adapter_in_process(self.adapters, invocation, definition, started_at_s=started_at_s, index=index, timeout_s=timeout_s)
        handle = begin_execution(invocation, definition, started_at_s=started_at_s, index=index)
        result = handle.wait(timeout_s)
        if result is None:
            handle.cancel()
            cleanup_done = handle.cleanup(min(0.25, timeout_s))
            idle = handle.wait_idle(min(0.25, timeout_s))
            if not cleanup_done or not idle:
                raise MissionValidationError("adapter unavailable: timeout cleanup could not prove quiescence")
            return _tool_result(
                invocation,
                ToolResultStatus.TIMEOUT,
                started_at_s,
                started_at_s + timeout_s,
                error=f"tool exceeded timeout_s={timeout_s:g}; cooperative cancellation cleanup completed and adapter is idle",
            )
        return result

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


def build_default_registry(
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
            "move_distance",
            "1.0",
            _schema(
                {
                    "distance_m": {"type": "number", "minimum": 0.01, "maximum": 2.0},
                    "speed_mps": {"type": "number", "minimum": 0.01, "maximum": 0.2},
                    "timeout_s": {"type": "number", "minimum": 0.1, "maximum": 30.0},
                }
            ),
            _schema({"measured_distance_m": {"type": "number"}, "stop_reason": {"type": "string"}}),
            preconditions=("fresh odometry", "collision stop clear"),
            availability=avail("move_distance"),
            timeout_s=30.0,
            safety_class="supervised_motion",
            approval_class="supervised_motion",
            resource_ownership=("odom_motion",),
            effects=("requests bounded odometry distance primitive with heading hold; no direct motor writes",),
        ),
        ToolDefinition(
            "turn_angle",
            "1.0",
            _schema(
                {
                    "angle_deg": {"type": "number", "minimum": -180.0, "maximum": 180.0},
                    "angular_speed_deg_s": {"type": "number", "minimum": 1.0, "maximum": 90.0},
                    "timeout_s": {"type": "number", "minimum": 0.1, "maximum": 30.0},
                }
            ),
            _schema({"measured_angle_deg": {"type": "number"}, "stop_reason": {"type": "string"}}),
            preconditions=("fresh heading odometry", "collision stop clear"),
            availability=avail("turn_angle"),
            timeout_s=30.0,
            safety_class="supervised_motion",
            approval_class="supervised_motion",
            resource_ownership=("odom_motion",),
            effects=("requests bounded odometry turn primitive; no direct motor writes",),
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


def build_canonical_shoe_mapping_plan(*, goal_id: str = "shoe-room-map", approval: Optional[ApprovalGrant] = None) -> MissionPlan:
    goal = MissionGoal(
        goal_id=goal_id,
        objective="Map the room and identify every shoe; produce semantic artifacts bounded to observed coverage.",
        constraints={"area": "room", "coverage": "observed_only"},
        success_criteria=(
            SuccessCriterion("localized", "occupancy/localization workflow completes", CriterionKind.TOOL_COMPLETE, tool_id="map_localize"),
            SuccessCriterion("projected", "shoe detections are projected into map frame", CriterionKind.TOOL_COMPLETE, tool_id="project_detections_to_map"),
            SuccessCriterion("semantic-map", "semantic map artifact is referenced", CriterionKind.ARTIFACT_PRESENT, field="semantic_map"),
            SuccessCriterion("mission-summary", "mission summary artifact is referenced", CriterionKind.ARTIFACT_PRESENT, field="mission_summary"),
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


def _validate_result_boundary(result: ToolResult, definition: ToolDefinition, *, physical_mode: bool = False) -> ToolResult:
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
    if physical_mode:
        provenance_adapter = str(result.provenance.get("adapter", ""))
        if not provenance_adapter.startswith("physical/") or result.provenance.get("deterministic") is not True:
            return _tool_result(
                result.invocation,
                ToolResultStatus.FAILED,
                result.started_at_s,
                result.completed_at_s,
                error="physical adapter result lacks live physical provenance",
            )
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
        missing = _invalid_artifact_refs(result.artifact_refs)
        if missing:
            return _tool_result(
                result.invocation,
                ToolResultStatus.FAILED,
                result.started_at_s,
                result.completed_at_s,
                error="adapter artifact_refs failed provenance validation: " + ", ".join(missing),
            )
    return result


def _arguments_digest(arguments: Mapping[str, Any]) -> str:
    payload = json.dumps(dict(arguments), sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _planned_travel_m(plan: MissionPlan) -> float:
    travel = 0.0
    for invocation in plan.invocations:
        if invocation.tool_id in {"move_to_clearance", "bounded_exploration_segment"}:
            travel += float(invocation.arguments.get("max_travel_m", 0.0))
        elif invocation.tool_id == "move_distance":
            travel += abs(float(invocation.arguments.get("distance_m", 0.0)))
    return travel


def _is_reviewed_physical_adapter(adapters: Any) -> bool:
    try:
        from .physical_capability_adapters import PhysicalCapabilityAdapters
    except ImportError:
        return False
    return (
        type(adapters) is PhysicalCapabilityAdapters
        and getattr(adapters, "execution_mode", "") == "physical"
        and getattr(adapters, "authority_kind", "") == "physical"
        and getattr(adapters, "physical_authority", None) is _PHYSICAL_ADAPTER_AUTHORITY
    )


def _planned_provider_calls(plan: MissionPlan) -> int:
    total = 0
    for invocation in plan.invocations:
        raw = invocation.provenance.get("provider_calls", 0)
        try:
            calls = int(raw)
        except (TypeError, ValueError) as exc:
            raise MissionValidationError("provider_calls provenance must be an integer") from exc
        if calls < 0:
            raise MissionValidationError("provider_calls provenance must be non-negative")
        total += calls
    return total


def _provider_calls_from_results(results: Sequence[ToolResult]) -> int:
    total = 0
    for result in results:
        raw = result.provenance.get("provider_calls", 0)
        try:
            calls = int(raw)
        except (TypeError, ValueError) as exc:
            raise MissionValidationError("provider_calls result provenance must be an integer") from exc
        if calls < 0:
            raise MissionValidationError("provider_calls result provenance must be non-negative")
        total += calls
    return total


def _default_satisfied_preconditions() -> tuple[str, ...]:
    return (
        "map/localization adapter installed",
        "supervised coordinator and collision stop are available",
        "range target visible",
        "collision stop clear",
        "fresh odometry",
        "fresh heading odometry",
        "detector plugin installed for requested object_class",
    )


def _invalid_artifact_refs(artifact_refs: Mapping[str, str]) -> tuple[str, ...]:
    repo_root = Path(__file__).resolve().parents[2]
    invalid: list[str] = []
    for kind, ref in artifact_refs.items():
        if not isinstance(ref, str) or not ref.strip():
            invalid.append(str(kind))
            continue
        path = Path(ref)
        if path.is_absolute() or ".." in path.parts or path.parts[:1] != ("artifacts",):
            invalid.append(str(kind))
            continue
        if not (repo_root / path).is_file():
            invalid.append(str(kind))
    return tuple(invalid)


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
    if tool_id == "move_distance":
        return {"measured_distance_m": abs(float(invocation.arguments["distance_m"])), "stop_reason": "target_reached"}, {}
    if tool_id == "turn_angle":
        return {"measured_angle_deg": float(invocation.arguments["angle_deg"]), "stop_reason": "target_reached"}, {}
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


def _missing_requested_artifacts(goal: MissionGoal, results: Sequence[ToolResult]) -> tuple[str, ...]:
    artifact_refs: dict[str, str] = {}
    for result in results:
        artifact_refs.update(result.artifact_refs)
    return tuple(kind for kind in goal.requested_artifacts if not artifact_refs.get(kind))


def _evaluate_success_criteria(goal: MissionGoal, results: Sequence[ToolResult]) -> tuple[CriterionResult, ...]:
    artifact_refs: dict[str, str] = {}
    completed_by_tool: dict[str, ToolResult] = {}
    for result in results:
        artifact_refs.update(result.artifact_refs)
        if result.status is ToolResultStatus.COMPLETE:
            completed_by_tool[result.invocation.tool_id] = result
    return tuple(_evaluate_success_criterion(criterion, artifact_refs, completed_by_tool) for criterion in goal.success_criteria)


def _adapter_process_entry(adapter: Any, invocation: ToolInvocation, definition: ToolDefinition, started_at_s: float, index: int, result_queue: Any) -> None:
    try:
        result_queue.put(("ok", adapter.execute(invocation, definition, started_at_s=started_at_s, index=index)))
    except BaseException as exc:  # pragma: no cover - exercised through parent process result
        result_queue.put(("error", str(exc)))


def _execute_adapter_in_process(
    adapter: Any,
    invocation: ToolInvocation,
    definition: ToolDefinition,
    *,
    started_at_s: float,
    index: int,
    timeout_s: float,
) -> ToolResult:
    context = mp.get_context("fork") if "fork" in mp.get_all_start_methods() else mp.get_context()
    result_queue = context.Queue(maxsize=1)
    process = context.Process(target=_adapter_process_entry, args=(adapter, invocation, definition, started_at_s, index, result_queue))
    process.start()
    process.join(timeout_s)
    if process.is_alive():
        process.terminate()
        process.join(1.0)
        if process.is_alive():
            process.kill()
            process.join(1.0)
        return _tool_result(
            invocation,
            ToolResultStatus.TIMEOUT,
            started_at_s,
            started_at_s + timeout_s,
            error=f"tool exceeded timeout_s={timeout_s:g}; cancellation cleanup completed",
        )
    try:
        kind, payload = result_queue.get_nowait()
    except queue.Empty:
        return _tool_result(invocation, ToolResultStatus.FAILED, started_at_s, started_at_s, error="adapter process exited without a result")
    if kind == "ok":
        return payload
    return _tool_result(invocation, ToolResultStatus.FAILED, started_at_s, started_at_s, error=str(payload))


def _evaluate_success_criterion(
    criterion: SuccessCriterion,
    artifact_refs: Mapping[str, str],
    completed_by_tool: Mapping[str, ToolResult],
) -> CriterionResult:
    if criterion.kind is CriterionKind.TOOL_COMPLETE:
        result = completed_by_tool.get(criterion.tool_id)
        return CriterionResult(
            criterion.criterion_id,
            result is not None,
            {"tool_id": criterion.tool_id, "correlation_id": "" if result is None else result.invocation.correlation_id},
            "tool did not complete" if result is None else "",
        )
    if criterion.kind is CriterionKind.ARTIFACT_PRESENT:
        ref = artifact_refs.get(criterion.field)
        return CriterionResult(
            criterion.criterion_id,
            bool(ref),
            {"artifact": criterion.field, "ref": ref or ""},
            "artifact is missing" if not ref else "",
        )
    if criterion.kind is CriterionKind.OBSERVATION_EQUALS:
        result = completed_by_tool.get(criterion.tool_id)
        actual = None if result is None else result.observation.get(criterion.field)
        return CriterionResult(
            criterion.criterion_id,
            actual == criterion.expected,
            {"tool_id": criterion.tool_id, "field": criterion.field, "actual": actual},
            "observation evidence mismatch" if actual != criterion.expected else "",
        )
    evidence_text = _success_evidence_text(completed_by_tool, artifact_refs)
    tokens = tuple(_significant_tokens(criterion.description))
    satisfied = bool(tokens) and all(token in evidence_text for token in tokens)
    return CriterionResult(
        criterion.criterion_id,
        satisfied,
        {"description": criterion.description},
        "no validated runtime evidence matched criterion text" if not satisfied else "",
    )


def _success_evidence_text(completed_by_tool: Mapping[str, ToolResult], artifact_refs: Mapping[str, str]) -> str:
    chunks: list[str] = []
    for tool_id, result in completed_by_tool.items():
        chunks.append(tool_id.replace("_", " "))
        chunks.extend(str(key).replace("_", " ") for key in result.observation.keys())
        chunks.extend(str(value).replace("_", " ") for value in result.observation.values() if isinstance(value, (str, int, float)))
        chunks.extend(str(key).replace("_", " ") for key in result.artifact_refs.keys())
    chunks.extend(str(key).replace("_", " ") for key in artifact_refs.keys())
    return " ".join(chunks).lower()


def _significant_tokens(text: str) -> tuple[str, ...]:
    stopwords = {"the", "and", "are", "into", "with", "that", "this", "from", "has", "have", "mission", "rover"}
    normalized = "".join(ch.lower() if ch.isalnum() else " " for ch in text)
    return tuple(token for token in normalized.split() if len(token) >= 4 and token not in stopwords)


def _source_sha() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL, timeout=2).strip()
    except Exception:
        return "unknown"


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
