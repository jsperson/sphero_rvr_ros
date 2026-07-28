"""Persistent fail-closed owner for Mission API session state and execution.

The service is deliberately ROS-free.  A deployment binds one reviewed adapter
set, while the durable SQLite ledger survives MCP/CLI processes and machine
restarts.  Motion is never resumed from persisted state.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import fcntl
import json
import math
import os
from pathlib import Path
import socket
import socketserver
import sqlite3
import threading
import time
import stat
from typing import Any, Callable, Mapping, Optional, Sequence

from .mission_api import (
    CapabilityAvailability,
    CapabilityRegistry,
    ApprovalGrant,
    CriterionKind,
    DeterministicMissionRuntime,
    FakeCapabilityAdapters,
    MissionBudgets,
    MissionPlan,
    MissionGoal,
    MissionRuntimeResult,
    MissionValidationError,
    SuccessCriterion,
    ToolInvocation,
    ToolResult,
    _arguments_digest,
    build_default_registry,
    _issue_approval_grant,
)

_TERMINAL_STATUSES = {
    "complete",
    "failed",
    "blocked",
    "cancelled",
    "timeout",
    "stopped",
    "estopped",
    "rejected",
    "recovery_required",
}

_PROMPT_TERMINAL_STATUSES = {
    "complete",
    "failed",
    "blocked",
    "cancelled",
    "stopped",
    "estopped",
    "timeout",
    "rejected",
    "recovery_required",
}

_DEFAULT_ADAPTIVE_LIMITS = {
    "mission_lease_s": 900.0,
    "max_translation_per_intent_m": 0.25,
    "max_rotation_per_intent_deg": 45.0,
    "max_intent_timeout_s": 5.0,
    "max_intent_lease_s": 5.0,
    "linear_speed_mps": 0.10,
    "angular_speed_rad_s": 0.4,
}


def _finite_equal(left: Any, right: Any) -> bool:
    if isinstance(left, bool) or isinstance(right, bool):
        return False
    try:
        left_value = float(left)
        right_value = float(right)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(left_value)
        and math.isfinite(right_value)
        and left_value == right_value
    )


@dataclass(frozen=True)
class ExecutorBinding:
    """Process-local authority binding for one declared capability."""

    executor: Any
    mode: str
    credential_namespace: str
    heartbeat_at_s: float
    max_age_s: float
    evidence: Mapping[str, Any]

    def __post_init__(self) -> None:
        if self.mode not in {"replay", "live"}:
            raise MissionValidationError("executor binding mode must be replay or live")
        if not str(self.credential_namespace).strip():
            raise MissionValidationError("executor binding credential namespace is required")
        if not isinstance(self.heartbeat_at_s, (int, float)) or not math.isfinite(
            float(self.heartbeat_at_s)
        ):
            raise MissionValidationError("executor binding heartbeat must be finite")
        if (
            not isinstance(self.max_age_s, (int, float))
            or not math.isfinite(float(self.max_age_s))
            or float(self.max_age_s) <= 0.0
        ):
            raise MissionValidationError(
                "executor binding max age must be positive and finite"
            )
        if not isinstance(self.evidence, Mapping) or not self.evidence:
            raise MissionValidationError("executor binding evidence is required")
        try:
            evidence = json.loads(_json_dump(dict(self.evidence)))
        except (TypeError, ValueError) as exc:
            raise MissionValidationError(
                "executor binding evidence must be bounded JSON data"
            ) from exc
        object.__setattr__(self, "heartbeat_at_s", float(self.heartbeat_at_s))
        object.__setattr__(self, "max_age_s", float(self.max_age_s))
        object.__setattr__(self, "evidence", evidence)


class _ExecutorRouter:
    """Dispatch replay invocations to the executor bound for each tool id."""

    cooperative_execution = True
    execution_mode = "replay"
    authority_kind = "replay"
    healthy = True
    evidence_level = "mission_service_binding"

    def __init__(self, service: "MissionService", tool_ids: Sequence[str]) -> None:
        self._service = service
        self.supported_tool_ids = tuple(tool_ids)
        preconditions: set[str] = set()
        for tool_id in tool_ids:
            binding = service._require_healthy_binding(tool_id)
            preconditions.update(
                str(item)
                for item in (getattr(binding.executor, "satisfied_preconditions", ()) or ())
            )
        self.satisfied_preconditions = tuple(sorted(preconditions))

    def begin_execution(self, invocation: ToolInvocation, definition: Any, **kwargs: Any) -> Any:
        binding = self._service._require_healthy_binding(invocation.tool_id)
        return binding.executor.begin_execution(invocation, definition, **kwargs)


class MissionService:
    """Own persistent session authority and bind one execution mode."""

    def __init__(
        self,
        database: str | Path,
        *,
        source_sha: str,
        deployed_sha: str,
        registry: Optional[CapabilityRegistry] = None,
        adapters: Any = None,
        mode: str = "replay",
        executor_bindings: Optional[Mapping[str, ExecutorBinding]] = None,
        live_execution_enabled: bool = False,
        adaptive_mission_limits: Optional[Mapping[str, Any]] = None,
        session_budgets: MissionBudgets = MissionBudgets(
            max_steps=8,
            max_runtime_s=120.0,
            max_travel_m=2.0,
            max_observations=16,
            max_artifacts=16,
            max_tool_calls=8,
            max_provider_calls=8,
        ),
        clock_s: Any = None,
    ) -> None:
        if mode not in {"replay", "live"}:
            raise MissionValidationError("mission service mode must be replay or live")
        source_provenance = _require_provenance(source_sha, "source_sha")
        deployed_provenance = _require_provenance(deployed_sha, "deployed_sha")
        database_target = str(database)
        self._database_owner_lock = None
        if database_target != ":memory:":
            self.database = Path(database_target).expanduser().resolve(strict=False)
            database_target = str(self.database)
            self.database.parent.mkdir(parents=True, exist_ok=True)
            database_owner_lock = open(f"{self.database}.owner.lock", "a+b")
            try:
                fcntl.flock(
                    database_owner_lock.fileno(),
                    fcntl.LOCK_EX | fcntl.LOCK_NB,
                )
            except OSError as exc:
                database_owner_lock.close()
                raise MissionValidationError(
                    f"mission service database already owned: {self.database}"
                ) from exc
            self._database_owner_lock = database_owner_lock
        else:
            self.database = Path(database_target)
        self.registry = registry or build_default_registry(detector_classes=("shoe", "backpack"))
        self.adapters = adapters or FakeCapabilityAdapters()
        self.mode = mode
        self.source_sha = source_provenance
        self.deployed_sha = deployed_provenance
        self.session_budgets = session_budgets
        self.live_execution_enabled = bool(live_execution_enabled)
        self.adaptive_mission_limits = (
            json.loads(_json_dump(dict(adaptive_mission_limits)))
            if adaptive_mission_limits is not None
            else None
        )
        self._clock_s = clock_s or time.time
        self._lock = threading.RLock()
        self._executor_bindings: dict[str, ExecutorBinding] = {}
        try:
            if executor_bindings is None and self.mode == "replay":
                now = self._now()
                evidence = {
                    "executor": self.adapters.__class__.__name__,
                    "evidence_level": str(getattr(self.adapters, "evidence_level", "unspecified")),
                    "deployed_sha": str(getattr(self.adapters, "deployed_sha", self.deployed_sha)),
                }
                executor_bindings = {
                    definition.tool_id: ExecutorBinding(
                        executor=self.adapters,
                        mode="replay",
                        credential_namespace="replay",
                        heartbeat_at_s=now,
                        max_age_s=60.0,
                        evidence=evidence,
                    )
                    for definition in self.registry.definitions()
                    if definition.availability is CapabilityAvailability.AVAILABLE
                }
            for tool_id, binding in (executor_bindings or {}).items():
                self._bind_executor(str(tool_id), binding)
        except Exception:
            self._release_database_owner()
            raise
        try:
            self._connection = sqlite3.connect(database_target, check_same_thread=False)
            self._connection.row_factory = sqlite3.Row
            self._connection.execute("PRAGMA journal_mode=WAL")
            self._connection.execute("PRAGMA synchronous=FULL")
            self._create_schema()
            self._recover_interrupted_missions()
        except Exception:
            if hasattr(self, "_connection"):
                self._connection.close()
            self._release_database_owner()
            raise

    def close(self) -> None:
        with self._lock:
            self._connection.close()
            self._release_database_owner()

    def __enter__(self) -> "MissionService":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def _release_database_owner(self) -> None:
        if self._database_owner_lock is not None and not self._database_owner_lock.closed:
            fcntl.flock(self._database_owner_lock.fileno(), fcntl.LOCK_UN)
            self._database_owner_lock.close()

    def bind_executor(self, tool_id: str, binding: ExecutorBinding) -> None:
        """Bind one declared tool to a live process-local executor authority."""

        with self._lock:
            self._bind_executor(tool_id, binding)

    def heartbeat_executor(
        self,
        tool_id: str,
        *,
        evidence: Optional[Mapping[str, Any]] = None,
    ) -> None:
        """Refresh one binding without changing its authority or credentials."""

        with self._lock:
            try:
                binding = self._executor_bindings[tool_id]
            except KeyError as exc:
                raise MissionValidationError(
                    f"executor binding is missing for {tool_id}"
                ) from exc
            refreshed_evidence = binding.evidence if evidence is None else evidence
            refreshed = replace(
                binding,
                heartbeat_at_s=self._now(),
                evidence=dict(refreshed_evidence),
            )
            self._bind_executor(tool_id, refreshed)

    def capabilities(self) -> dict[str, dict[str, Any]]:
        """Return declared and currently usable capabilities with fresh evidence."""

        with self._lock:
            return {
                f"{definition.tool_id}@{definition.version}": self._capability_state(
                    definition.tool_id,
                    definition.availability,
                )
                for definition in self.registry.definitions()
            }

    def _bind_executor(self, tool_id: str, binding: ExecutorBinding) -> None:
        definitions = [
            definition
            for definition in self.registry.definitions()
            if definition.tool_id == tool_id
        ]
        if not definitions:
            raise MissionValidationError(f"cannot bind undeclared capability: {tool_id}")
        expected_namespace = "physical" if self.mode == "live" else "replay"
        if binding.mode != self.mode or binding.credential_namespace != expected_namespace:
            raise MissionValidationError(
                "executor binding mode/credential namespace cannot cross service authority"
            )
        expected_executor_mode = "physical" if self.mode == "live" else "replay"
        executor_mode = str(getattr(binding.executor, "execution_mode", "unknown"))
        if executor_mode != expected_executor_mode:
            raise MissionValidationError(
                "executor mode does not match mission service authority"
            )
        if float(binding.heartbeat_at_s) > self._now():
            raise MissionValidationError("executor binding heartbeat cannot be in the future")
        self._executor_bindings[tool_id] = binding

    def _capability_state(
        self,
        tool_id: str,
        availability: CapabilityAvailability,
    ) -> dict[str, Any]:
        binding = self._executor_bindings.get(tool_id)
        if binding is None:
            return {
                "declared": True,
                "bound": False,
                "healthy": False,
                "mode": self.mode,
                "credential_namespace": "physical" if self.mode == "live" else "replay",
                "availability": availability.value,
                "fresh": False,
                "heartbeat_at_s": None,
                "age_s": None,
                "max_age_s": None,
                "evidence": {},
                "health_reason": "executor binding is missing",
            }
        age_s = max(0.0, self._now() - float(binding.heartbeat_at_s))
        fresh = age_s <= float(binding.max_age_s)
        protocol_ready = bool(getattr(binding.executor, "cooperative_execution", False)) and callable(
            getattr(binding.executor, "begin_execution", None)
        )
        bound = availability is CapabilityAvailability.AVAILABLE and protocol_ready
        executor_healthy = bool(getattr(binding.executor, "healthy", False))
        healthy = bound and fresh and executor_healthy
        if availability is not CapabilityAvailability.AVAILABLE:
            reason = f"capability is declared {availability.value}"
        elif not protocol_ready:
            reason = "executor lacks cooperative execution contract"
        elif not fresh:
            reason = "executor binding evidence is stale"
        elif not executor_healthy:
            reason = str(
                getattr(binding.executor, "health_reason", "executor reported unhealthy")
            )
        else:
            reason = "healthy"
        return {
            "declared": True,
            "bound": bound,
            "healthy": healthy,
            "mode": binding.mode,
            "credential_namespace": binding.credential_namespace,
            "availability": availability.value,
            "fresh": fresh,
            "heartbeat_at_s": float(binding.heartbeat_at_s),
            "age_s": age_s,
            "max_age_s": float(binding.max_age_s),
            "evidence": dict(binding.evidence),
            "health_reason": reason,
        }

    def _require_healthy_binding(self, tool_id: str) -> ExecutorBinding:
        definition = next(
            (
                item
                for item in self.registry.definitions()
                if item.tool_id == tool_id
            ),
            None,
        )
        if definition is None:
            raise MissionValidationError(f"unknown tool: {tool_id}")
        state = self._capability_state(tool_id, definition.availability)
        if not state["healthy"]:
            raise MissionValidationError(
                f"tool {tool_id} executor {state['health_reason']}"
            )
        return self._executor_bindings[tool_id]

    def _validate_executor_bindings(self, plan: MissionPlan) -> None:
        for invocation in plan.invocations:
            binding = self._require_healthy_binding(invocation.tool_id)
            definition = self.registry.require(
                invocation.tool_id,
                invocation.tool_version,
            )
            satisfied = {
                str(item)
                for item in (
                    getattr(binding.executor, "satisfied_preconditions", ()) or ()
                )
            }
            missing = [item for item in definition.preconditions if item not in satisfied]
            if missing:
                raise MissionValidationError(
                    f"tool {invocation.tool_id} precondition not attested by bound executor: {missing[0]}"
                )

    def _execution_adapters(self, plan: MissionPlan) -> Any:
        bindings = [
            self._require_healthy_binding(invocation.tool_id)
            for invocation in plan.invocations
        ]
        executors = {id(binding.executor): binding.executor for binding in bindings}
        if len(executors) == 1:
            return next(iter(executors.values()))
        if self.mode == "live":
            raise MissionValidationError(
                "live capability bindings require one reviewed physical adapter authority"
            )
        return _ExecutorRouter(self, tuple(invocation.tool_id for invocation in plan.invocations))

    def submit_plan(
        self,
        plan: MissionPlan,
        *,
        session_id: str,
        source: str,
        credential_namespace: Optional[str] = None,
        submission_id: Optional[str] = None,
    ) -> dict[str, Any]:
        if not str(session_id).strip():
            raise MissionValidationError("session_id is required")
        if source not in {"mcp", "planner", "cli", "api"}:
            raise MissionValidationError("mission source must be mcp, planner, cli, or api")
        if self.mode == "live" and not self.live_execution_enabled:
            raise MissionValidationError(
                "live execution is disabled by reviewed service configuration"
            )
        namespace = credential_namespace or ("physical" if self.mode == "live" else "replay")
        mission_id = submission_id or plan.goal.goal_id
        with self._lock:
            self._ensure_session(session_id)
            self._insert_mission(mission_id, session_id, plan, source, namespace)
            with self._connection:
                self._append_event(mission_id, session_id, "proposal", {
                    "source": source,
                    "mode": self.mode,
                    "credential_namespace": namespace,
                    "plan": plan.to_json_dict(),
                })
            execution_started = False
            try:
                plan = self._bind_replay_approvals(
                    plan,
                    session_id=session_id,
                    source=source,
                )
                self._validate_authority(plan, namespace)
                self._validate_session_latch(session_id)
                self._validate_executor_bindings(plan)
                ledger = self._session_ledger(session_id)
                self._validate_cumulative_budget(plan, ledger)
                execution_adapters = self._execution_adapters(plan)
                runtime = DeterministicMissionRuntime(
                    self.registry,
                    execution_adapters,
                    now_s=self._now(),
                    budget_ceilings=self.session_budgets,
                )
                runtime.validate_plan(plan)
                with self._connection:
                    self._connection.execute(
                        "UPDATE missions SET status = 'running', updated_at_s = ? WHERE mission_id = ?",
                        (self._now(), mission_id),
                    )
                    for invocation in plan.invocations:
                        if invocation.approval is not None:
                            self._append_event(
                                mission_id,
                                session_id,
                                "approval",
                                {"approval": invocation.approval.to_json_dict()},
                            )
                        self._append_event(
                            mission_id,
                            session_id,
                            "invocation",
                            {"invocation": invocation.to_json_dict(), "source": source},
                        )
                    self._append_event(
                        mission_id,
                        session_id,
                        "running",
                        {"status": "running"},
                    )
                execution_started = True
                result = runtime.execute_plan(plan)
                return self._record_result(mission_id, session_id, result, ledger)
            except Exception as exc:
                if isinstance(exc, MissionValidationError) and not execution_started:
                    self._record_rejection(mission_id, session_id, str(exc))
                else:
                    self._record_process_failure(mission_id, session_id, exc)
                raise

    def cancel(self, session_id: str, *, reason: str = "operator cancel") -> dict[str, Any]:
        with self._lock, self._connection:
            self._ensure_session(session_id)
            now = self._now()
            self._connection.execute(
                "UPDATE sessions SET cancel_latched = 1, terminal_reason = ?, updated_at_s = ? WHERE session_id = ?",
                (str(reason), now, session_id),
            )
            rows = self._connection.execute(
                "SELECT mission_id FROM missions WHERE session_id = ?", (session_id,)
            ).fetchall()
            for row in rows:
                self._append_event(row["mission_id"], session_id, "cancel", {"reason": str(reason)})
            return self.session_status(session_id)

    def status(self, mission_id: str) -> dict[str, Any]:
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM missions WHERE mission_id = ?", (mission_id,)
            ).fetchone()
            if row is None:
                raise MissionValidationError(f"unknown mission: {mission_id}")
            session = self.session_status(row["session_id"])
            return {
                "mission_id": mission_id,
                "session_id": row["session_id"],
                "status": row["status"],
                "mode": row["mode"],
                "source": row["source"],
                "source_sha": row["source_sha"],
                "deployed_sha": row["deployed_sha"],
                "terminal_reason": row["terminal_reason"],
                "recovery_required": bool(row["recovery_required"]),
                "cancel_latched": session["cancel_latched"],
                "stop_latched": session["stop_latched"],
                "estop_latched": session["estop_latched"],
                "auto_resume": False,
                "ledger": session["ledger"],
                "route": _json_load(row["route_json"], _empty_route()),
                "artifacts": _json_load(row["artifacts_json"], {}),
            }

    def session_status(self, session_id: str) -> dict[str, Any]:
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM sessions WHERE session_id = ?", (session_id,)
            ).fetchone()
            if row is None:
                raise MissionValidationError(f"unknown session: {session_id}")
            return {
                "session_id": session_id,
                "mode": row["mode"],
                "credential_namespace": row["credential_namespace"],
                "cancel_latched": bool(row["cancel_latched"]),
                "stop_latched": bool(row["stop_latched"]),
                "estop_latched": bool(row["estop_latched"]),
                "terminal_reason": row["terminal_reason"],
                "ledger": _json_load(row["ledger_json"], _empty_ledger()),
                "auto_resume": False,
            }

    def events(self, mission_id: Optional[str] = None) -> list[dict[str, Any]]:
        with self._lock:
            if mission_id is None:
                rows = self._connection.execute("SELECT * FROM events ORDER BY event_id").fetchall()
            else:
                rows = self._connection.execute(
                    "SELECT * FROM events WHERE mission_id = ? ORDER BY event_id", (mission_id,)
                ).fetchall()
            return [
                {
                    "event_id": row["event_id"],
                    "mission_id": row["mission_id"],
                    "session_id": row["session_id"],
                    "kind": row["kind"],
                    "created_at_s": row["created_at_s"],
                    "source_sha": row["source_sha"],
                    "deployed_sha": row["deployed_sha"],
                    "payload": _json_load(row["payload_json"], {}),
                }
                for row in rows
            ]

    def begin_prompt_mission(
        self,
        *,
        mission_id: str,
        session_id: str,
        prompt: str,
        source: str = "api",
        provider_call_started: bool = True,
    ) -> dict[str, Any]:
        """Persist receipt and planning before any provider call begins."""

        mission = str(mission_id).strip()
        session = str(session_id).strip()
        objective = str(prompt).strip()
        if not mission or not session or not objective:
            raise MissionValidationError("mission_id, session_id, and prompt are required")
        if source not in {"api", "web", "cli", "mcp"}:
            raise MissionValidationError("prompt mission source must be api, web, cli, or mcp")
        namespace = "physical" if self.mode == "live" else "replay"
        with self._lock:
            self._ensure_session(session)
            now = self._now()
            try:
                with self._connection:
                    self._connection.execute(
                        "INSERT INTO prompt_missions("
                        "mission_id,session_id,status,mode,source,credential_namespace,prompt,"
                        "source_sha,deployed_sha,created_at_s,updated_at_s"
                        ") VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                        (
                            mission,
                            session,
                            "received",
                            self.mode,
                            source,
                            namespace,
                            objective,
                            self.source_sha,
                            self.deployed_sha,
                            now,
                            now,
                        ),
                    )
                    self._append_event(mission, session, "received", {"prompt": objective, "source": source})
                    self._connection.execute(
                        "UPDATE prompt_missions SET status='planning', updated_at_s=? WHERE mission_id=?",
                        (self._now(), mission),
                    )
                    self._append_event(
                        mission,
                        session,
                        "planning",
                        {
                            "provider_call_started": bool(
                                provider_call_started
                            )
                        },
                    )
            except sqlite3.IntegrityError as exc:
                raise MissionValidationError(f"mission id already exists: {mission}") from exc
            return self.prompt_status(mission)

    def record_prompt_proposal(
        self,
        mission_id: str,
        proposal: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist one typed proposal or rejection after independent validation."""

        payload = json.loads(_json_dump(dict(proposal)))
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "planning":
                raise MissionValidationError("prompt mission is not awaiting a provider decision")
            if str(payload.get("prompt", "")).strip() != row["prompt"]:
                raise MissionValidationError("proposal prompt does not match the persisted mission prompt")
            if str(payload.get("source_sha", "")).strip() != self.source_sha:
                raise MissionValidationError("proposal source SHA does not match the service source SHA")
            digest = str(payload.get("proposal_digest", "")).strip().lower()
            if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
                raise MissionValidationError("proposal digest must be a full SHA-256 hex value")
            decision = str(payload.get("decision", ""))
            segments = payload.get("segments", ())
            executable = decision == "propose" and isinstance(segments, list) and bool(segments)
            if decision not in {"propose", "reject"}:
                raise MissionValidationError("proposal decision must be propose or reject")
            if decision == "reject" and segments:
                raise MissionValidationError("a rejected proposal cannot contain motion segments")
            status = "proposed" if executable else "rejected"
            reason = "" if executable else str(payload.get("summary", "model rejected mission"))
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status=?, proposal_json=?, proposal_digest=?, "
                    "terminal_reason=?, updated_at_s=? WHERE mission_id=?",
                    (status, _json_dump(payload), digest, reason, self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "proposal" if executable else "proposal_rejected",
                    {"proposal": payload, "executable": executable},
                )
                if not executable:
                    self._append_event(
                        mission_id,
                        row["session_id"],
                        "terminal",
                        {"status": "rejected", "reason": reason},
                    )
            return self.prompt_status(mission_id)

    def record_hierarchical_physical_proposal(
        self,
        mission_id: str,
        proposal: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist the semantic-only canonical proposal before M7.6 approval."""

        from .hierarchical_physical_binding import (
            PHYSICAL_PROPOSAL_SCHEMA,
            canonical_digest,
        )

        payload = json.loads(_json_dump(dict(proposal)))
        supplied_digest = str(
            payload.get("proposal_digest", "")
        ).strip().lower()
        unsigned = dict(payload)
        unsigned.pop("proposal_digest", None)
        expected_keys = {
            "schema",
            "mission_id",
            "objective",
            "objective_revision",
            "requested_object_classes",
            "source_sha",
            "created_at_s",
        }
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "planning":
                raise MissionValidationError(
                    "canonical hierarchical mission is not awaiting a proposal"
                )
            if (
                set(unsigned) != expected_keys
                or unsigned.get("schema") != PHYSICAL_PROPOSAL_SCHEMA
                or str(unsigned.get("mission_id", "")).strip()
                != row["mission_id"]
                or str(unsigned.get("objective", "")).strip()
                != row["prompt"]
                or str(unsigned.get("source_sha", "")).strip()
                != self.source_sha
                or unsigned.get("requested_object_classes")
                != ["shoe", "person"]
                or unsigned.get("objective_revision") != 1
            ):
                raise MissionValidationError(
                    "canonical hierarchical proposal semantics or provenance are invalid"
                )
            if canonical_digest(unsigned) != supplied_digest:
                raise MissionValidationError(
                    "canonical hierarchical proposal digest is invalid"
                )
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='proposed', "
                    "proposal_json=?, proposal_digest=?, updated_at_s=? "
                    "WHERE mission_id=?",
                    (
                        _json_dump(payload),
                        supplied_digest,
                        self._now(),
                        mission_id,
                    ),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "hierarchical_physical_proposal",
                    {
                        "proposal": payload,
                        "semantic_only": True,
                        "motion_authority": False,
                    },
                )
            return self.prompt_status(mission_id)

    def approve_hierarchical_physical_mission(
        self,
        mission_id: str,
        *,
        approval: Mapping[str, Any],
        now_s: float,
    ) -> dict[str, Any]:
        """Validate and consume one exact M7.6 approval into durable state."""

        from .hierarchical_physical_binding import (
            HierarchicalPhysicalApproval,
        )

        payload = json.loads(_json_dump(dict(approval)))
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "proposed":
                raise MissionValidationError(
                    "only a proposed canonical hierarchical mission can be approved"
                )
            proposal = _json_load(row["proposal_json"], {})
            validated = HierarchicalPhysicalApproval.validated(
                payload,
                now_s=float(now_s),
                source_sha=self.source_sha,
                deployed_sha=self.deployed_sha,
                reviewed_sha=self.source_sha,
            )
            if (
                validated.mission_id != row["mission_id"]
                or validated.proposal_digest
                != str(proposal.get("proposal_digest", ""))
            ):
                raise MissionValidationError(
                    "M7.6 approval does not bind the persisted canonical proposal"
                )
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='approved', "
                    "approval_json=?, approval_expires_at_s=?, updated_at_s=? "
                    "WHERE mission_id=?",
                    (
                        _json_dump(payload),
                        validated.expires_at_s,
                        self._now(),
                        mission_id,
                    ),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "hierarchical_m7_6_approval",
                    {
                        "approval": payload,
                        "authenticated": True,
                        "restart_resume_allowed": False,
                    },
                )
            return self.prompt_status(mission_id)

    def record_hierarchical_checkpoint(
        self,
        mission_id: str,
        checkpoint: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Append bounded controller evidence while the canonical run is live."""

        payload = json.loads(_json_dump(dict(checkpoint)))
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "running":
                raise MissionValidationError(
                    "hierarchical checkpoints require a running canonical mission"
                )
            with self._connection:
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "hierarchical_checkpoint",
                    payload,
                )
            return self.prompt_status(mission_id)

    def reject_prompt_planning(self, mission_id: str, reason: str) -> dict[str, Any]:
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] not in {"received", "planning"}:
                raise MissionValidationError("prompt mission planning is no longer active")
            message = str(reason).strip() or "prompt planning failed"
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='rejected', terminal_reason=?, updated_at_s=? WHERE mission_id=?",
                    (message, self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "terminal",
                    {"status": "rejected", "reason": message},
                )
            return self.prompt_status(mission_id)

    def record_rolling_replay_proposal(
        self,
        mission_id: str,
        proposal: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist the replay-only rolling-intent contract before it can run.

        This is intentionally separate from the physical prompt-route approval
        path.  A rolling replay proposal contains no primitive route and can
        never authorize the live executor.
        """

        payload = json.loads(_json_dump(dict(proposal)))
        with self._lock:
            if self.mode != "replay":
                raise MissionValidationError(
                    "rolling replay proposals require replay service authority"
                )
            row = self._prompt_row(mission_id)
            if row["status"] != "planning":
                raise MissionValidationError(
                    "prompt mission is not awaiting a rolling replay proposal"
                )
            proposal_schema = str(payload.get("schema", ""))
            if proposal_schema not in {
                "sphero_rvr.rolling_replay_proposal.v1",
                "sphero_rvr.adaptive_mission_proposal.v1",
                "sphero_rvr.hierarchical_phase4_proposal.v1",
            }:
                raise MissionValidationError("adaptive replay proposal schema is invalid")
            if str(payload.get("prompt", "")).strip() != row["prompt"]:
                raise MissionValidationError(
                    "rolling replay proposal prompt does not match the persisted mission"
                )
            if str(payload.get("source_sha", "")).strip() != self.source_sha:
                raise MissionValidationError(
                    "rolling replay proposal source SHA does not match the service"
                )
            if payload.get("segments") not in ([], ()):
                raise MissionValidationError(
                    "rolling replay proposal cannot contain a fixed primitive route"
                )
            digest = str(payload.get("proposal_digest", "")).strip().lower()
            if len(digest) != 64 or any(
                character not in "0123456789abcdef" for character in digest
            ):
                raise MissionValidationError(
                    "rolling replay proposal digest must be a full SHA-256 value"
                )
            digest_payload = dict(payload)
            digest_payload.pop("proposal_digest", None)
            if _arguments_digest(digest_payload) != digest:
                raise MissionValidationError(
                    "rolling replay proposal digest does not bind the complete proposal"
                )
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='proposed', proposal_json=?, "
                    "proposal_digest=?, updated_at_s=? WHERE mission_id=?",
                    (_json_dump(payload), digest, self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    (
                        "adaptive_mission_proposal"
                        if proposal_schema == "sphero_rvr.adaptive_mission_proposal.v1"
                        else "hierarchical_phase4_proposal"
                        if proposal_schema
                        == "sphero_rvr.hierarchical_phase4_proposal.v1"
                        else "rolling_replay_proposal"
                    ),
                    {"proposal": payload, "motion_authority": False},
                )
            return self.prompt_status(mission_id)

    def record_adaptive_mission_proposal(
        self,
        mission_id: str,
        proposal: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist an adaptive mission proposal in replay or reviewed live mode."""

        payload = json.loads(_json_dump(dict(proposal)))
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "planning":
                raise MissionValidationError(
                    "prompt mission is not awaiting an adaptive mission proposal"
                )
            if payload.get("schema") != "sphero_rvr.adaptive_mission_proposal.v1":
                raise MissionValidationError("Adaptive mission proposal schema is invalid")
            if str(payload.get("prompt", "")).strip() != row["prompt"]:
                raise MissionValidationError(
                    "Adaptive mission proposal prompt does not match the persisted mission"
                )
            if str(payload.get("source_sha", "")).strip() != self.source_sha:
                raise MissionValidationError(
                    "Adaptive mission proposal source SHA does not match the service"
                )
            if str(payload.get("deployed_sha", "")).strip() != self.deployed_sha:
                raise MissionValidationError(
                    "Adaptive mission proposal deployed SHA does not match the service"
                )
            if payload.get("segments") not in ([], ()):
                raise MissionValidationError(
                    "Adaptive mission proposal cannot contain a fixed primitive route"
                )
            if payload.get("safety_policy") != "lidar_collision_stop.v1":
                raise MissionValidationError(
                    "Adaptive mission proposal safety policy is not the installed policy"
                )
            from .adaptive_mission_controller import AdaptiveMissionLimits

            installed_limits = (
                self.adaptive_mission_limits
                if self.adaptive_mission_limits is not None
                else AdaptiveMissionLimits().to_json_dict()
            )
            proposal_limits = payload.get("limits", {})
            lease_s = (
                proposal_limits.get("mission_lease_s")
                if isinstance(proposal_limits, Mapping)
                else None
            )
            installed_lease_s = installed_limits.get(
                "mission_lease_s"
            )
            nonlease_limits_match = isinstance(
                proposal_limits, Mapping
            ) and all(
                (
                    proposal_limits.get(name)
                    == installed_limits.get(name)
                )
                if installed_limits.get(name) is None
                else _finite_equal(
                    proposal_limits.get(name),
                    installed_limits.get(name),
                )
                for name in installed_limits
                if name != "mission_lease_s"
            )
            try:
                lease_within_profile = (
                    not isinstance(lease_s, bool)
                    and math.isfinite(float(lease_s))
                    and 0.0 < float(lease_s)
                    <= float(installed_lease_s)
                )
            except (TypeError, ValueError):
                lease_within_profile = False
            if not lease_within_profile or not nonlease_limits_match:
                raise MissionValidationError(
                    "Adaptive mission proposal exceeds or changes the installed authority profile"
                )
            contract = payload.get("contract")
            if not isinstance(contract, Mapping):
                raise MissionValidationError(
                    "Adaptive mission proposal contract is missing"
                )
            expected_physical = bool(
                self.mode == "live" and self.live_execution_enabled
            )
            if (
                contract.get("fixed_route") is not False
                or contract.get("replanning_after_every_intent") is not True
                or contract.get("one_authenticated_approval") is not True
                or contract.get("per_intent_approval") is not False
                or contract.get("approval_activates_supervised_graph")
                is not True
                or contract.get("telemetry_managed_for_lease") is not True
                or contract.get("telemetry_stops_when_lease_ends") is not True
                or contract.get(
                    "first_intent_requires_fresh_post_approval_evidence"
                )
                is not True
                or contract.get("motion_authority") is not False
                or contract.get("physical_execution_enabled")
                is not expected_physical
                or contract.get("drop_off_detection") is not False
            ):
                raise MissionValidationError(
                    "Adaptive mission proposal contract does not match service authority"
                )
            if self.mode == "live" and payload.get("executor_mode") != (
                "physical-supervised-live-route"
            ):
                raise MissionValidationError(
                    "live Adaptive mission requires the supervised physical executor mode"
                )
            if expected_physical and (
                payload.get("starting_snapshot_id")
                != "pending-approval-activation"
                or payload.get("first_intent") != {}
            ):
                raise MissionValidationError(
                    "physical Adaptive mission must defer its first snapshot and "
                    "LLM intent until authenticated approval activation"
                )
            digest = str(payload.get("proposal_digest", "")).strip().lower()
            body = dict(payload)
            body.pop("proposal_digest", None)
            from .rolling_replay import canonical_digest

            if digest != canonical_digest(body):
                raise MissionValidationError(
                    "Adaptive mission proposal digest does not match its envelope"
                )
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='proposed', "
                    "proposal_json=?, proposal_digest=?, updated_at_s=? "
                    "WHERE mission_id=?",
                    (_json_dump(payload), digest, self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "adaptive_mission_proposal",
                    {
                        "proposal_digest": digest,
                        "lease_id": payload.get("lease_id"),
                        "first_intent": payload.get("first_intent"),
                        "motion_authority": False,
                    },
                )
            return self.prompt_status(mission_id)

    def approve_adaptive_mission(
        self,
        mission_id: str,
        *,
        supplied_approval: str,
        operator: str,
        authentication_source: str,
        expires_at_s: float,
    ) -> dict[str, Any]:
        """Bind one authenticated live approval to the persisted Adaptive mission envelope."""

        with self._lock:
            if self.mode != "live" or not self.live_execution_enabled:
                raise MissionValidationError(
                    "physical Adaptive mission execution is disabled by reviewed service configuration"
                )
            row = self._prompt_row(mission_id)
            if row["status"] != "proposed":
                raise MissionValidationError(
                    "only a proposed adaptive mission can be approved"
                )
            proposal = _json_load(row["proposal_json"], {})
            if proposal.get("schema") != "sphero_rvr.adaptive_mission_proposal.v1":
                raise MissionValidationError(
                    "persisted mission is not an adaptive mission proposal"
                )
            approved_by = str(operator).strip()
            identity_source = str(authentication_source).strip()
            if not approved_by or identity_source != "tailscale-serve":
                raise MissionValidationError(
                    "physical Adaptive mission requires a Tailscale-authenticated operator"
                )
            digest = str(row["proposal_digest"])
            if str(supplied_approval).strip() != f"APPROVE ADAPTIVE MISSION {digest}":
                raise MissionValidationError(
                    "Adaptive mission approval does not match the persisted proposal"
                )
            now = self._now()
            requested_expires = float(expires_at_s)
            lease_s = float(
                _json_load(row["proposal_json"], {})
                .get("limits", {})
                .get("mission_lease_s", 0.0)
            )
            expected_lease_s = float(
                (
                    self.adaptive_mission_limits
                    or _DEFAULT_ADAPTIVE_LIMITS
                ).get("mission_lease_s", 0.0)
            )
            proposal_limits = proposal.get("limits", {})
            configured_limits = (
                self.adaptive_mission_limits
                or _DEFAULT_ADAPTIVE_LIMITS
            )
            safety_limits_match = isinstance(
                proposal_limits, Mapping
            ) and all(
                _finite_equal(
                    proposal_limits.get(name),
                    configured_limits.get(name),
                )
                for name in (
                    "max_translation_per_intent_m",
                    "max_rotation_per_intent_deg",
                    "max_intent_timeout_s",
                    "max_intent_lease_s",
                    "linear_speed_mps",
                    "angular_speed_rad_s",
                )
            )
            if (
                not math.isfinite(requested_expires)
                or not math.isfinite(lease_s)
                or not math.isfinite(expected_lease_s)
                or not 0.0 < lease_s <= 900.0
                or lease_s > expected_lease_s
                or not safety_limits_match
                or requested_expires < now + lease_s - 5.0
                or requested_expires > now + lease_s
            ):
                raise MissionValidationError(
                    "Adaptive mission approval must match the persisted lease "
                    "and every reviewed safety limit"
                )
            expires = now + lease_s
            approval = {
                "approved": True,
                "authenticated": True,
                "authentication_source": identity_source,
                "operator": approved_by,
                "proposal_digest": digest,
                "approval_id": f"{approved_by}:{digest}",
                "approved_at_s": now,
                "expires_at_s": expires,
                "lease_id": proposal.get("lease_id"),
            }
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='approved', "
                    "approval_json=?, approval_expires_at_s=?, updated_at_s=? "
                    "WHERE mission_id=?",
                    (_json_dump(approval), expires, self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "adaptive_mission_approval",
                    approval,
                )
            return self.prompt_status(mission_id)

    def update_adaptive_mission_objective(
        self,
        mission_id: str,
        *,
        prompt: str,
        operator: str,
        authentication_source: str,
    ) -> dict[str, Any]:
        """Update the objective without replacing or extending its active lease."""

        objective = str(prompt).strip()
        principal = str(operator).strip()
        identity_source = str(authentication_source).strip()
        if not objective or len(objective) > 500:
            raise MissionValidationError(
                "updated Adaptive mission objective must contain 1 to 500 characters"
            )
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] not in {"approved", "queued", "running"}:
                raise MissionValidationError(
                    "only an active Adaptive mission lease may update its objective"
                )
            proposal = _json_load(row["proposal_json"], {})
            approval = _json_load(row["approval_json"], {})
            if (
                proposal.get("schema")
                != "sphero_rvr.adaptive_mission_proposal.v1"
                or principal != str(approval.get("operator", ""))
                or identity_source != "tailscale-serve"
            ):
                raise MissionValidationError(
                    "active objective update is not bound to the authenticated lease operator"
                )
            previous = str(row["prompt"])
            if objective == previous:
                return self.prompt_status(mission_id)
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET prompt=?, updated_at_s=? "
                    "WHERE mission_id=?",
                    (objective, self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "adaptive_objective_updated",
                    {
                        "reason": (
                            f"Authenticated operator {principal} updated the "
                            f"objective from {previous!r} to {objective!r}; "
                            "the active lease expiry is unchanged."
                        ),
                        "objective": objective,
                        "operator": principal,
                    },
                )
            return self.prompt_status(mission_id)

    def record_adaptive_mission_checkpoint(
        self,
        mission_id: str,
        *,
        kind: str,
        checkpoint: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist the latest live Adaptive mission projection without granting authority."""

        payload = json.loads(_json_dump(dict(checkpoint)))
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] not in {
                "approved",
                "queued",
                "running",
                "cancel_requested",
            }:
                raise MissionValidationError(
                    "Adaptive mission checkpoint requires an active approved mission"
                )
            if str(payload.get("mission_id", "")) != str(mission_id):
                raise MissionValidationError(
                    "Adaptive mission checkpoint mission identity changed"
                )
            if payload.get("schema") != "sphero_rvr.adaptive_mission_result.v1":
                raise MissionValidationError(
                    "Adaptive mission checkpoint schema is invalid"
                )
            if payload.get("motion_authority") is not False:
                raise MissionValidationError(
                    "Adaptive mission checkpoint cannot claim browser motion authority"
                )
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET result_json=?, updated_at_s=? "
                    "WHERE mission_id=?",
                    (_json_dump(payload), self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "adaptive_mission_checkpoint",
                    {
                        "kind": str(kind),
                        "status": payload.get("status"),
                        "snapshot_id": _json_load(
                            _json_dump(payload.get("world_snapshot", {})), {}
                        ).get("snapshot_id"),
                        "active_intent": payload.get("active_intent"),
                        "metrics": payload.get("metrics"),
                    },
                )
            return self.prompt_status(mission_id)

    def approve_rolling_replay_mission(
        self,
        mission_id: str,
        *,
        proposal_digest: str,
        operator: str,
        authentication_source: str = "replay-local",
        authenticated: bool = False,
    ) -> dict[str, Any]:
        """Approve a no-authority replay without crossing the live gate."""

        supplied_digest = str(proposal_digest).strip().lower()
        approved_by = str(operator).strip()
        identity_source = str(authentication_source).strip()
        with self._lock:
            if self.mode != "replay" or self.live_execution_enabled:
                raise MissionValidationError(
                    "rolling replay approval requires no-authority replay mode"
                )
            row = self._prompt_row(mission_id)
            if row["status"] != "proposed":
                raise MissionValidationError(
                    "only a proposed rolling replay mission can be approved"
                )
            if not approved_by:
                raise MissionValidationError("replay approval operator is required")
            if not identity_source:
                raise MissionValidationError(
                    "replay approval authentication source is required"
                )
            if supplied_digest != str(row["proposal_digest"]):
                raise MissionValidationError(
                    "rolling replay approval does not match the persisted proposal"
                )
            approval = {
                "approved": True,
                "operator": approved_by,
                "authenticated": bool(authenticated),
                "authentication_source": identity_source,
                "proposal_digest": supplied_digest,
                "approved_at_s": self._now(),
                "simulation_only": True,
            }
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='running', approval_json=?, "
                    "updated_at_s=? WHERE mission_id=?",
                    (_json_dump(approval), self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "rolling_replay_started",
                    {
                        "approval": approval,
                        "motion_authority": False,
                        "physical_execution_enabled": False,
                    },
                )
            return self.prompt_status(mission_id)

    def record_rolling_replay_checkpoint(
        self,
        mission_id: str,
        *,
        kind: str,
        checkpoint: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Durably replace the latest replay projection and append evidence."""

        event_kind = str(kind).strip()
        if not event_kind:
            raise MissionValidationError("rolling replay checkpoint kind is required")
        payload = json.loads(_json_dump(dict(checkpoint)))
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "running":
                raise MissionValidationError(
                    "rolling replay checkpoint requires a running mission"
                )
            if bool(payload.get("motion_authority", True)):
                raise MissionValidationError(
                    "rolling replay checkpoint cannot claim motion authority"
                )
            if bool(payload.get("physical_execution_enabled", True)):
                raise MissionValidationError(
                    "rolling replay checkpoint cannot enable physical execution"
                )
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET result_json=?, updated_at_s=? "
                    "WHERE mission_id=?",
                    (_json_dump(payload), self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    event_kind,
                    payload,
                )
            return self.prompt_status(mission_id)

    def finish_rolling_replay_mission(
        self,
        mission_id: str,
        *,
        status: str,
        reason: str,
        result: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist one replay terminal result after deterministic stop."""

        destination = str(status).strip().lower()
        if destination not in _PROMPT_TERMINAL_STATUSES - {"rejected"}:
            raise MissionValidationError("rolling replay terminal status is invalid")
        payload = json.loads(_json_dump(dict(result)))
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "running":
                raise MissionValidationError(
                    "rolling replay terminal result requires a running mission"
                )
            if bool(payload.get("motion_authority", True)):
                raise MissionValidationError(
                    "rolling replay terminal result cannot claim motion authority"
                )
            if bool(payload.get("physical_execution_enabled", True)):
                raise MissionValidationError(
                    "rolling replay terminal result cannot enable physical execution"
                )
            terminal_reason = str(reason).strip()
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status=?, result_json=?, "
                    "terminal_reason=?, updated_at_s=? WHERE mission_id=?",
                    (
                        destination,
                        _json_dump(payload),
                        terminal_reason,
                        self._now(),
                        mission_id,
                    ),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "terminal",
                    {
                        "status": destination,
                        "reason": terminal_reason,
                        "result": payload,
                    },
                )
            return self.prompt_status(mission_id)

    def record_stationary_perception_proposal(
        self,
        mission_id: str,
        proposal: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist a live-sensor proposal that can never authorize motion."""

        payload = json.loads(_json_dump(dict(proposal)))
        with self._lock:
            if self.mode != "live" or self.live_execution_enabled:
                raise MissionValidationError(
                    "stationary perception requires live mode with execution disabled"
                )
            row = self._prompt_row(mission_id)
            if row["status"] != "planning":
                raise MissionValidationError(
                    "prompt mission is not awaiting a stationary perception proposal"
                )
            if (
                str(payload.get("schema", ""))
                != "sphero_rvr.stationary_perception_proposal.v1"
            ):
                raise MissionValidationError(
                    "stationary perception proposal schema is invalid"
                )
            if str(payload.get("prompt", "")).strip() != row["prompt"]:
                raise MissionValidationError(
                    "stationary perception prompt does not match the persisted mission"
                )
            if str(payload.get("source_sha", "")).strip() != self.source_sha:
                raise MissionValidationError(
                    "stationary perception source SHA does not match the service"
                )
            if payload.get("segments") not in ([], ()):
                raise MissionValidationError(
                    "stationary perception cannot contain a route or motion segments"
                )
            contract = payload.get("contract", {})
            if (
                not isinstance(contract, Mapping)
                or bool(contract.get("motion_authority", True))
                or bool(contract.get("physical_execution_enabled", True))
            ):
                raise MissionValidationError(
                    "stationary perception proposal must explicitly deny physical authority"
                )
            digest = str(payload.get("proposal_digest", "")).strip().lower()
            digest_payload = dict(payload)
            digest_payload.pop("proposal_digest", None)
            if (
                len(digest) != 64
                or any(character not in "0123456789abcdef" for character in digest)
                or _arguments_digest(digest_payload) != digest
            ):
                raise MissionValidationError(
                    "stationary perception digest must bind the complete proposal"
                )
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='proposed', proposal_json=?, "
                    "proposal_digest=?, updated_at_s=? WHERE mission_id=?",
                    (_json_dump(payload), digest, self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "stationary_perception_proposal",
                    {
                        "proposal": payload,
                        "motion_authority": False,
                        "physical_execution_enabled": False,
                    },
                )
            return self.prompt_status(mission_id)

    def approve_stationary_perception_mission(
        self,
        mission_id: str,
        *,
        proposal_digest: str,
        operator: str,
    ) -> dict[str, Any]:
        """Approve only live observation; no executor or route grant is created."""

        digest = str(proposal_digest).strip().lower()
        approved_by = str(operator).strip()
        with self._lock:
            if self.mode != "live" or self.live_execution_enabled:
                raise MissionValidationError(
                    "stationary perception approval requires execution-locked live mode"
                )
            row = self._prompt_row(mission_id)
            if row["status"] != "proposed":
                raise MissionValidationError(
                    "only a proposed stationary perception mission can be approved"
                )
            if not approved_by:
                raise MissionValidationError(
                    "stationary perception approval operator is required"
                )
            if digest != str(row["proposal_digest"]):
                raise MissionValidationError(
                    "stationary perception approval does not match the persisted proposal"
                )
            approval = {
                "approved": True,
                "operator": approved_by,
                "proposal_digest": digest,
                "approved_at_s": self._now(),
                "stationary_perception_only": True,
                "motion_authority": False,
                "physical_execution_enabled": False,
            }
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='running', approval_json=?, "
                    "updated_at_s=? WHERE mission_id=?",
                    (_json_dump(approval), self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "stationary_perception_started",
                    approval,
                )
            return self.prompt_status(mission_id)

    def record_stationary_perception_checkpoint(
        self,
        mission_id: str,
        *,
        kind: str,
        checkpoint: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist fresh live perception and LLM revisions without route state."""

        event_kind = str(kind).strip()
        payload = json.loads(_json_dump(dict(checkpoint)))
        if not event_kind:
            raise MissionValidationError(
                "stationary perception checkpoint kind is required"
            )
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "running":
                raise MissionValidationError(
                    "stationary perception checkpoint requires a running mission"
                )
            if bool(payload.get("motion_authority", True)) or bool(
                payload.get("physical_execution_enabled", True)
            ):
                raise MissionValidationError(
                    "stationary perception checkpoint cannot claim physical authority"
                )
            world_snapshot = payload.get("world_snapshot", {})
            if not isinstance(world_snapshot, Mapping):
                world_snapshot = {}
            compact_event = {
                "schema": payload.get("schema"),
                "mission_id": payload.get("mission_id", mission_id),
                "status": payload.get("status"),
                "terminal": bool(payload.get("terminal", False)),
                "terminal_reason": payload.get("terminal_reason", ""),
                "progress": payload.get("progress", 0.0),
                "snapshot_id": world_snapshot.get("snapshot_id", ""),
                "active_intent": payload.get("active_intent"),
                "inference": payload.get("inference", {}),
                "metrics": payload.get("metrics", {}),
                "motion_authority": False,
                "physical_execution_enabled": False,
            }
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET result_json=?, updated_at_s=? "
                    "WHERE mission_id=?",
                    (_json_dump(payload), self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    event_kind,
                    compact_event,
                )
            return self.prompt_status(mission_id)

    def finish_stationary_perception_mission(
        self,
        mission_id: str,
        *,
        status: str,
        reason: str,
        result: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Persist a deterministic stationary terminal result."""

        destination = str(status).strip().lower()
        if destination not in _PROMPT_TERMINAL_STATUSES - {"rejected"}:
            raise MissionValidationError(
                "stationary perception terminal status is invalid"
            )
        payload = json.loads(_json_dump(dict(result)))
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "running":
                raise MissionValidationError(
                    "stationary perception terminal result requires a running mission"
                )
            if bool(payload.get("motion_authority", True)) or bool(
                payload.get("physical_execution_enabled", True)
            ):
                raise MissionValidationError(
                    "stationary perception terminal result cannot claim physical authority"
                )
            terminal_reason = str(reason).strip()
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status=?, result_json=?, "
                    "terminal_reason=?, updated_at_s=? WHERE mission_id=?",
                    (
                        destination,
                        _json_dump(payload),
                        terminal_reason,
                        self._now(),
                        mission_id,
                    ),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "terminal",
                    {
                        "status": destination,
                        "reason": terminal_reason,
                        "result": payload,
                        "motion_authority": False,
                        "physical_execution_enabled": False,
                    },
                )
            return self.prompt_status(mission_id)

    def approve_prompt_mission(
        self,
        mission_id: str,
        *,
        supplied_approval: str,
        operator: str,
        expires_at_s: float,
    ) -> dict[str, Any]:
        """Verify the persisted proposal and record server-owned approval authority."""

        from .prompt_drive import approved_live_route, prompt_drive_proposal_from_json

        with self._lock:
            if not self.live_execution_enabled:
                raise MissionValidationError("live execution is disabled by reviewed service configuration")
            row = self._prompt_row(mission_id)
            if row["status"] != "proposed":
                raise MissionValidationError("only a proposed prompt mission can be approved")
            approved_by = str(operator).strip()
            if not approved_by:
                raise MissionValidationError("approval operator identity is required")
            expires = float(expires_at_s)
            now = self._now()
            if not math.isfinite(expires) or expires <= now or expires > now + 300.0:
                raise MissionValidationError("approval expiry must be within the next 300 seconds")
            proposal = prompt_drive_proposal_from_json(_json_load(row["proposal_json"], {}))
            route = approved_live_route(proposal, supplied_approval, operator=approved_by)
            approval = {
                "approved": True,
                "operator": approved_by,
                "proposal_digest": proposal.proposal_digest,
                "approval_id": route.approval_id,
                "approved_at_s": now,
                "expires_at_s": expires,
            }
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='approved', approval_json=?, route_json=?, "
                    "approval_expires_at_s=?, updated_at_s=? WHERE mission_id=?",
                    (_json_dump(approval), _json_dump(route.to_json_dict()), expires, now, mission_id),
                )
                self._append_event(mission_id, row["session_id"], "approval", approval)
            return self.prompt_status(mission_id)

    def transition_prompt_mission(
        self,
        mission_id: str,
        status: str,
        *,
        reason: str = "",
        result: Optional[Mapping[str, Any]] = None,
    ) -> dict[str, Any]:
        transitions = {
            "approved": {
                "queued",
                "cancelled",
                "failed",
                "recovery_required",
            },
            "queued": {"running", "cancelled", "recovery_required"},
            "running": _PROMPT_TERMINAL_STATUSES | {"cancel_requested"},
            "cancel_requested": _PROMPT_TERMINAL_STATUSES,
        }
        destination = str(status).strip().lower()
        with self._lock:
            row = self._prompt_row(mission_id)
            if destination not in transitions.get(row["status"], set()):
                raise MissionValidationError(
                    f"invalid prompt mission transition: {row['status']} -> {destination}"
                )
            payload = {} if result is None else json.loads(_json_dump(dict(result)))
            terminal_reason = str(reason).strip()
            recovery = destination == "recovery_required"
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status=?, result_json=?, terminal_reason=?, "
                    "recovery_required=?, updated_at_s=? WHERE mission_id=?",
                    (
                        destination,
                        _json_dump(payload),
                        terminal_reason,
                        int(recovery),
                        self._now(),
                        mission_id,
                    ),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "terminal" if destination in _PROMPT_TERMINAL_STATUSES else destination,
                    {"status": destination, "reason": terminal_reason, "result": payload},
                )
                if recovery:
                    self._latch_session_recovery(row["session_id"], terminal_reason)
            return self.prompt_status(mission_id)

    def request_prompt_cancel(self, mission_id: str, *, reason: str) -> dict[str, Any]:
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] != "running":
                raise MissionValidationError("cancel request requires a running prompt mission")
            message = str(reason).strip() or "operator requested cancellation"
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='cancel_requested', terminal_reason=?, updated_at_s=? WHERE mission_id=?",
                    (message, self._now(), mission_id),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "cancel_requested",
                    {"reason": message},
                )
            return self.prompt_status(mission_id)

    def cancel_prompt_mission(self, mission_id: str, *, reason: str) -> dict[str, Any]:
        with self._lock:
            row = self._prompt_row(mission_id)
            if row["status"] in _PROMPT_TERMINAL_STATUSES:
                return self.prompt_status(mission_id)
            message = str(reason).strip() or "operator cancelled mission"
            with self._connection:
                self._connection.execute(
                    "UPDATE prompt_missions SET status='cancelled', terminal_reason=?, updated_at_s=? WHERE mission_id=?",
                    (message, self._now(), mission_id),
                )
                self._connection.execute(
                    "UPDATE sessions SET cancel_latched=1, terminal_reason=?, updated_at_s=? WHERE session_id=?",
                    (message, self._now(), row["session_id"]),
                )
                self._append_event(
                    mission_id,
                    row["session_id"],
                    "terminal",
                    {"status": "cancelled", "reason": message},
                )
            return self.prompt_status(mission_id)

    def prompt_status(self, mission_id: str) -> dict[str, Any]:
        with self._lock:
            row = self._prompt_row(mission_id)
            proposal = _json_load(row["proposal_json"], {})
            events = self.events(row["mission_id"])
            if (
                isinstance(proposal, Mapping)
                and proposal.get("schema")
                in {
                    "sphero_rvr.stationary_perception_proposal.v1",
                    "sphero_rvr.adaptive_mission_proposal.v1",
                }
            ):
                compact_events = []
                for event in events:
                    payload = event.get("payload", {})
                    if not isinstance(payload, Mapping):
                        payload = {}
                    compact_events.append(
                        {
                            **event,
                            "payload": {
                                key: payload[key]
                                for key in (
                                    "schema",
                                    "mission_id",
                                    "status",
                                    "terminal",
                                    "terminal_reason",
                                    "reason",
                                    "source",
                                    "progress",
                                    "snapshot_id",
                                    "active_intent",
                                    "inference",
                                    "metrics",
                                    "motion_authority",
                                    "physical_execution_enabled",
                                    "provider_call_started",
                                    "objective",
                                    "operator",
                                )
                                if key in payload
                            },
                        }
                    )
                events = compact_events
            return {
                "api_version": "mission_api.v2",
                "mission_id": row["mission_id"],
                "session_id": row["session_id"],
                "status": row["status"],
                "mode": row["mode"],
                "source": row["source"],
                "prompt": row["prompt"],
                "proposal": proposal,
                "proposal_digest": row["proposal_digest"],
                "approval": _json_load(row["approval_json"], {}),
                "approval_expires_at_s": row["approval_expires_at_s"],
                "route": _json_load(row["route_json"], {}),
                "result": _json_load(row["result_json"], {}),
                "terminal_reason": row["terminal_reason"],
                "recovery_required": bool(row["recovery_required"]),
                "source_sha": row["source_sha"],
                "deployed_sha": row["deployed_sha"],
                "live_execution_enabled": self.live_execution_enabled,
                "auto_resume": False,
                "events": events,
            }

    def latest_prompt_status(self, session_id: str) -> Optional[dict[str, Any]]:
        with self._lock:
            row = self._connection.execute(
                "SELECT mission_id FROM prompt_missions WHERE session_id=? ORDER BY created_at_s DESC LIMIT 1",
                (str(session_id),),
            ).fetchone()
            return None if row is None else self.prompt_status(row["mission_id"])

    def _prompt_row(self, mission_id: str) -> sqlite3.Row:
        row = self._connection.execute(
            "SELECT * FROM prompt_missions WHERE mission_id=?",
            (str(mission_id),),
        ).fetchone()
        if row is None:
            raise MissionValidationError(f"unknown prompt mission: {mission_id}")
        return row

    def _create_schema(self) -> None:
        with self._connection:
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS sessions (
                    session_id TEXT PRIMARY KEY,
                    mode TEXT NOT NULL,
                    credential_namespace TEXT NOT NULL,
                    ledger_json TEXT NOT NULL,
                    cancel_latched INTEGER NOT NULL DEFAULT 0,
                    stop_latched INTEGER NOT NULL DEFAULT 0,
                    estop_latched INTEGER NOT NULL DEFAULT 0,
                    terminal_reason TEXT NOT NULL DEFAULT '',
                    created_at_s REAL NOT NULL,
                    updated_at_s REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS missions (
                    mission_id TEXT PRIMARY KEY,
                    session_id TEXT NOT NULL REFERENCES sessions(session_id),
                    status TEXT NOT NULL,
                    mode TEXT NOT NULL,
                    source TEXT NOT NULL,
                    credential_namespace TEXT NOT NULL,
                    plan_json TEXT NOT NULL,
                    result_json TEXT NOT NULL DEFAULT '{}',
                    route_json TEXT NOT NULL DEFAULT '{}',
                    artifacts_json TEXT NOT NULL DEFAULT '{}',
                    terminal_reason TEXT NOT NULL DEFAULT '',
                    recovery_required INTEGER NOT NULL DEFAULT 0,
                    source_sha TEXT NOT NULL,
                    deployed_sha TEXT NOT NULL,
                    created_at_s REAL NOT NULL,
                    updated_at_s REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS events (
                    event_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    mission_id TEXT NOT NULL,
                    session_id TEXT NOT NULL,
                    kind TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    source_sha TEXT NOT NULL,
                    deployed_sha TEXT NOT NULL,
                    created_at_s REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS prompt_missions (
                    mission_id TEXT PRIMARY KEY,
                    session_id TEXT NOT NULL REFERENCES sessions(session_id),
                    status TEXT NOT NULL,
                    mode TEXT NOT NULL,
                    source TEXT NOT NULL,
                    credential_namespace TEXT NOT NULL,
                    prompt TEXT NOT NULL,
                    proposal_json TEXT NOT NULL DEFAULT '{}',
                    proposal_digest TEXT NOT NULL DEFAULT '',
                    approval_json TEXT NOT NULL DEFAULT '{}',
                    approval_expires_at_s REAL,
                    route_json TEXT NOT NULL DEFAULT '{}',
                    result_json TEXT NOT NULL DEFAULT '{}',
                    terminal_reason TEXT NOT NULL DEFAULT '',
                    recovery_required INTEGER NOT NULL DEFAULT 0,
                    source_sha TEXT NOT NULL,
                    deployed_sha TEXT NOT NULL,
                    created_at_s REAL NOT NULL,
                    updated_at_s REAL NOT NULL
                );
                CREATE TRIGGER IF NOT EXISTS events_no_update
                BEFORE UPDATE ON events BEGIN SELECT RAISE(ABORT, 'events are append-only'); END;
                CREATE TRIGGER IF NOT EXISTS events_no_delete
                BEFORE DELETE ON events BEGIN SELECT RAISE(ABORT, 'events are append-only'); END;
                """
            )

    def _recover_interrupted_missions(self) -> None:
        with self._lock, self._connection:
            rows = self._connection.execute(
                "SELECT mission_id, session_id FROM missions WHERE status NOT IN "
                "('complete','failed','blocked','cancelled','timeout','stopped','estopped','rejected','recovery_required')"
            ).fetchall()
            for row in rows:
                reason = "mission service restarted during execution; motion remains cancelled"
                self._connection.execute(
                    "UPDATE missions SET status = 'recovery_required', recovery_required = 1, terminal_reason = ?, updated_at_s = ? WHERE mission_id = ?",
                    (reason, self._now(), row["mission_id"]),
                )
                self._connection.execute(
                    "UPDATE sessions SET cancel_latched = 1, terminal_reason = ?, updated_at_s = ? WHERE session_id = ?",
                    (reason, self._now(), row["session_id"]),
                )
                self._append_event(row["mission_id"], row["session_id"], "recovery_required", {"reason": reason, "auto_resume": False})
            planning_rows = self._connection.execute(
                "SELECT mission_id,session_id FROM prompt_missions WHERE status IN ('received','planning')"
            ).fetchall()
            for row in planning_rows:
                reason = "mission service restarted before planning completed; no approval or motion exists"
                self._connection.execute(
                    "UPDATE prompt_missions SET status='rejected', terminal_reason=?, updated_at_s=? WHERE mission_id=?",
                    (reason, self._now(), row["mission_id"]),
                )
                self._append_event(
                    row["mission_id"], row["session_id"], "terminal", {"status": "rejected", "reason": reason}
                )
            active_rows = self._connection.execute(
                "SELECT mission_id,session_id FROM prompt_missions WHERE status IN ('approved','queued','running','cancel_requested')"
            ).fetchall()
            for row in active_rows:
                reason = "mission service restarted after approval; execution is not resumed"
                self._connection.execute(
                    "UPDATE prompt_missions SET status='recovery_required', recovery_required=1, "
                    "terminal_reason=?, updated_at_s=? WHERE mission_id=?",
                    (reason, self._now(), row["mission_id"]),
                )
                self._latch_session_recovery(row["session_id"], reason)
                self._append_event(
                    row["mission_id"], row["session_id"], "recovery_required", {"reason": reason, "auto_resume": False}
                )

    def _latch_session_recovery(self, session_id: str, reason: str) -> None:
        self._connection.execute(
            "UPDATE sessions SET cancel_latched=1, terminal_reason=?, updated_at_s=? WHERE session_id=?",
            (str(reason), self._now(), session_id),
        )

    def _ensure_session(self, session_id: str) -> None:
        row = self._connection.execute(
            "SELECT mode, credential_namespace FROM sessions WHERE session_id = ?", (session_id,)
        ).fetchone()
        namespace = "physical" if self.mode == "live" else "replay"
        if row is None:
            now = self._now()
            self._connection.execute(
                "INSERT INTO sessions(session_id, mode, credential_namespace, ledger_json, created_at_s, updated_at_s) VALUES(?,?,?,?,?,?)",
                (session_id, self.mode, namespace, _json_dump(_empty_ledger()), now, now),
            )
            self._connection.commit()
            return
        if row["mode"] != self.mode or row["credential_namespace"] != namespace:
            raise MissionValidationError("session authority mode/credential namespace cannot cross")

    def _insert_mission(
        self,
        mission_id: str,
        session_id: str,
        plan: MissionPlan,
        source: str,
        namespace: str,
    ) -> None:
        now = self._now()
        try:
            self._connection.execute(
                "INSERT INTO missions(mission_id, session_id, status, mode, source, credential_namespace, plan_json, route_json, source_sha, deployed_sha, created_at_s, updated_at_s) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
                (
                    mission_id,
                    session_id,
                    "proposed",
                    self.mode,
                    source,
                    namespace,
                    _json_dump(plan.to_json_dict()),
                    _json_dump(_empty_route()),
                    self.source_sha,
                    self.deployed_sha,
                    now,
                    now,
                ),
            )
            self._connection.commit()
        except sqlite3.IntegrityError as exc:
            raise MissionValidationError(f"mission id already exists: {mission_id}") from exc

    def _validate_authority(self, plan: MissionPlan, namespace: str) -> None:
        expected_namespace = "physical" if self.mode == "live" else "replay"
        if namespace != expected_namespace:
            raise MissionValidationError("replay and physical credential namespace cannot cross")
        expected_plan_mode = "physical" if self.mode == "live" else "replay"
        if plan.goal.execution_mode != expected_plan_mode:
            raise MissionValidationError("plan execution mode does not match mission service authority")
        adapter_mode = str(getattr(self.adapters, "execution_mode", "unknown"))
        if adapter_mode != expected_plan_mode:
            raise MissionValidationError("bound executor mode does not match mission service authority")

    def _bind_replay_approvals(self, plan: MissionPlan, *, session_id: str, source: str) -> MissionPlan:
        """Mint process-local replay grants after the socket trust boundary.

        Physical mode never enters this path; physical approval remains an
        operator-owned service concern and cannot be supplied by MCP credentials.
        """

        if self.mode != "replay":
            return plan
        invocations: list[ToolInvocation] = []
        now = self._now()
        for invocation in plan.invocations:
            definition = self.registry.require(invocation.tool_id, invocation.tool_version)
            if not definition.requires_approval() or invocation.approval is not None:
                invocations.append(invocation)
                continue
            grant = _issue_approval_grant(
                approval_id=f"service-replay:{session_id}:{invocation.correlation_id}",
                approved_by="mission-service-replay-supervisor",
                approved_at_s=now,
                expires_at_s=now + max(1.0, float(definition.timeout_s) + 1.0),
                approval_class=definition.approval_class,
                mission_id=plan.goal.goal_id,
                issued_to="mission-runtime",
                tool_id=invocation.tool_id,
                correlation_id=invocation.correlation_id,
                arguments_digest=_arguments_digest(invocation.arguments),
                principal=f"{source}:{session_id}",
                execution_mode="replay",
            )
            invocations.append(replace(invocation, approval=grant))
        return replace(plan, invocations=tuple(invocations))

    def _validate_session_latch(self, session_id: str) -> None:
        status = self.session_status(session_id)
        if status["estop_latched"]:
            raise MissionValidationError("terminal runtime state ESTOPPED is latched; mission session recovery/cancel latch is set")
        if status["stop_latched"]:
            raise MissionValidationError("terminal runtime state STOPPED is latched; mission session recovery/cancel latch is set")
        if status["cancel_latched"]:
            raise MissionValidationError("terminal runtime state CANCELLED is latched; mission session recovery/cancel latch is set")

    def _validate_cumulative_budget(self, plan: MissionPlan, ledger: Mapping[str, Any]) -> None:
        planned_steps = len(plan.invocations)
        planned_travel = _plan_travel_m(plan)
        checks = (
            ("max_steps", "steps", planned_steps),
            ("max_tool_calls", "tool_calls", planned_steps),
        )
        for budget_name, ledger_name, increment in checks:
            ceiling = getattr(self.session_budgets, budget_name)
            if ceiling is not None and int(ledger[ledger_name]) + increment > ceiling:
                raise MissionValidationError(f"mission session exceeds cumulative {budget_name} budget")
        if self.session_budgets.max_travel_m is not None and float(ledger["travel_m"]) + planned_travel > self.session_budgets.max_travel_m:
            raise MissionValidationError("mission session exceeds cumulative max_travel_m budget")
        if float(ledger["runtime_s"]) + plan.goal.budgets.max_runtime_s > self.session_budgets.max_runtime_s:
            raise MissionValidationError("mission session exceeds cumulative max_runtime_s budget")
        if (
            self.session_budgets.max_observations is not None
            and int(ledger["observations"]) + planned_steps > self.session_budgets.max_observations
        ):
            raise MissionValidationError("mission session exceeds cumulative max_observations budget")
        planned_artifacts = sum(
            len(invocation.arguments.get("artifact_kinds", ()))
            for invocation in plan.invocations
            if invocation.tool_id == "generate_semantic_artifacts"
        )
        if (
            self.session_budgets.max_artifacts is not None
            and int(ledger["artifacts"]) + planned_artifacts > self.session_budgets.max_artifacts
        ):
            raise MissionValidationError("mission session exceeds cumulative max_artifacts budget")
        used_approval_ids = {str(item) for item in ledger["approval_ids"]}
        for invocation in plan.invocations:
            if invocation.approval is not None and invocation.approval.approval_id in used_approval_ids:
                raise MissionValidationError(f"approval replay detected for {invocation.tool_id}")

    def _record_result(
        self,
        mission_id: str,
        session_id: str,
        result: MissionRuntimeResult,
        prior_ledger: Mapping[str, Any],
    ) -> dict[str, Any]:
        route = _route_from_results(result.results)
        artifacts: dict[str, str] = {}
        runtime_s = 0.0
        for item in result.results:
            runtime_s += max(0.0, item.completed_at_s - item.started_at_s)
            if item.observation:
                self._append_event(mission_id, session_id, "observation", {
                    "correlation_id": item.invocation.correlation_id,
                    "status": item.status.value,
                    "observation": dict(item.observation),
                })
            if item.artifact_refs:
                artifacts.update(item.artifact_refs)
                self._append_event(mission_id, session_id, "artifact", {
                    "correlation_id": item.invocation.correlation_id,
                    "artifact_refs": dict(item.artifact_refs),
                    "provenance": dict(item.provenance),
                })
        ledger = dict(prior_ledger)
        ledger["steps"] += len(result.results)
        ledger["tool_calls"] += len(result.results)
        ledger["runtime_s"] += runtime_s
        ledger["travel_m"] += route["measured_distance_m"]
        ledger["observations"] += sum(bool(item.observation) for item in result.results)
        ledger["artifacts"] += sum(len(item.artifact_refs) for item in result.results)
        approval_ids = list(ledger["approval_ids"])
        approval_ids.extend(
            item.invocation.approval.approval_id
            for item in result.results
            if item.invocation.approval is not None
        )
        ledger["approval_ids"] = sorted(set(approval_ids))
        status = result.status.value
        terminal_reason = ""
        if result.results and result.results[-1].error:
            terminal_reason = str(result.results[-1].error.get("message", result.results[-1].error))
        with self._connection:
            self._connection.execute(
                "UPDATE sessions SET ledger_json = ?, cancel_latched = ?, stop_latched = ?, estop_latched = ?, terminal_reason = ?, updated_at_s = ? WHERE session_id = ?",
                (
                    _json_dump(ledger),
                    int(status == "cancelled"),
                    int(status == "stopped"),
                    int(status == "estopped"),
                    terminal_reason,
                    self._now(),
                    session_id,
                ),
            )
            self._connection.execute(
                "UPDATE missions SET status = ?, result_json = ?, route_json = ?, artifacts_json = ?, terminal_reason = ?, updated_at_s = ? WHERE mission_id = ?",
                (
                    status,
                    _json_dump(result.to_json_dict()),
                    _json_dump(route),
                    _json_dump(artifacts),
                    terminal_reason,
                    self._now(),
                    mission_id,
                ),
            )
            self._append_event(mission_id, session_id, "terminal", {
                "status": status,
                "reason": terminal_reason,
                "ledger": ledger,
                "route": route,
                "artifacts": artifacts,
            })
        return {
            "api_version": "mission_api.v2",
            "mission_id": mission_id,
            "session_id": session_id,
            "status": status,
            "source_sha": self.source_sha,
            "deployed_sha": self.deployed_sha,
            "ledger": ledger,
            "route": route,
            "artifacts": artifacts,
            "result": result.to_json_dict(),
            "auto_resume": False,
        }

    def _record_rejection(self, mission_id: str, session_id: str, reason: str) -> None:
        with self._connection:
            self._connection.execute(
                "UPDATE missions SET status = 'rejected', terminal_reason = ?, updated_at_s = ? WHERE mission_id = ?",
                (reason, self._now(), mission_id),
            )
            self._append_event(mission_id, session_id, "terminal", {"status": "rejected", "reason": reason})

    def _record_process_failure(self, mission_id: str, session_id: str, exc: Exception) -> None:
        reason = f"executor process failure: {exc.__class__.__name__}; operator recovery required"
        with self._connection:
            self._connection.execute(
                "UPDATE missions SET status = 'recovery_required', recovery_required = 1, "
                "terminal_reason = ?, updated_at_s = ? WHERE mission_id = ?",
                (reason, self._now(), mission_id),
            )
            self._connection.execute(
                "UPDATE sessions SET cancel_latched = 1, terminal_reason = ?, updated_at_s = ? "
                "WHERE session_id = ?",
                (reason, self._now(), session_id),
            )
            self._append_event(
                mission_id,
                session_id,
                "recovery_required",
                {"reason": reason, "auto_resume": False},
            )

    def _session_ledger(self, session_id: str) -> dict[str, Any]:
        return dict(self.session_status(session_id)["ledger"])

    def _append_event(self, mission_id: str, session_id: str, kind: str, payload: Mapping[str, Any]) -> None:
        self._connection.execute(
            "INSERT INTO events(mission_id, session_id, kind, payload_json, source_sha, deployed_sha, created_at_s) VALUES(?,?,?,?,?,?,?)",
            (
                mission_id,
                session_id,
                kind,
                _json_dump(payload),
                self.source_sha,
                self.deployed_sha,
                self._now(),
            ),
        )

    def _now(self) -> float:
        return float(self._clock_s())


def _empty_ledger() -> dict[str, Any]:
    return {
        "steps": 0,
        "runtime_s": 0.0,
        "travel_m": 0.0,
        "observations": 0,
        "artifacts": 0,
        "tool_calls": 0,
        "provider_calls": 0,
        "approval_ids": [],
    }


def _empty_route() -> dict[str, Any]:
    return {"measured_distance_m": 0.0, "measured_angle_deg": 0.0, "completed_segments": 0}


def _route_from_results(results: Sequence[ToolResult]) -> dict[str, Any]:
    route = _empty_route()
    for result in results:
        observation = result.observation
        route["measured_distance_m"] += abs(float(observation.get("measured_distance_m", 0.0)))
        route["measured_angle_deg"] += float(observation.get("measured_angle_deg", 0.0))
        route["completed_segments"] += int(observation.get("completed_segments", 0))
    return route


def _plan_travel_m(plan: MissionPlan) -> float:
    travel = 0.0
    for invocation in plan.invocations:
        if invocation.tool_id == "move_distance":
            travel += abs(float(invocation.arguments.get("distance_m", 0.0)))
        elif invocation.tool_id in {"move_to_clearance", "bounded_exploration_segment"}:
            travel += float(invocation.arguments.get("max_travel_m", 0.0))
    return travel


def _json_dump(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _json_load(value: str, default: Any) -> Any:
    if not value:
        return default
    return json.loads(value)


def _require_provenance(value: str, name: str) -> str:
    provenance = str(value).strip()
    if not provenance or provenance.lower() == "unknown":
        raise MissionValidationError(f"{name} must be injected from reviewed build provenance")
    return provenance


def mission_plan_from_json(payload: Mapping[str, Any]) -> MissionPlan:
    """Parse the complete Mission API JSON envelope at the service boundary."""

    goal_payload = payload.get("goal")
    invocations_payload = payload.get("invocations")
    if not isinstance(goal_payload, Mapping):
        raise MissionValidationError("plan goal must be an object")
    if not isinstance(invocations_payload, Sequence) or isinstance(invocations_payload, (str, bytes)):
        raise MissionValidationError("plan invocations must be an array")
    budgets_payload = goal_payload.get("budgets", {})
    if not isinstance(budgets_payload, Mapping):
        raise MissionValidationError("goal budgets must be an object")
    criteria_payload = goal_payload.get("success_criteria", ())
    if not isinstance(criteria_payload, Sequence) or isinstance(criteria_payload, (str, bytes)):
        raise MissionValidationError("goal success_criteria must be an array")
    criteria: list[SuccessCriterion] = []
    for criterion in criteria_payload:
        if not isinstance(criterion, Mapping):
            raise MissionValidationError("success criterion must be an object")
        criteria.append(
            SuccessCriterion(
                criterion_id=str(criterion.get("criterion_id", "")),
                description=str(criterion.get("description", "")),
                kind=CriterionKind(str(criterion.get("kind", ""))),
                tool_id=str(criterion.get("tool_id", "")),
                field=str(criterion.get("field", "")),
                expected=criterion.get("expected"),
            )
        )
    goal = MissionGoal(
        goal_id=str(goal_payload.get("goal_id", "")),
        objective=str(goal_payload.get("objective", "")),
        success_criteria=tuple(criteria),
        constraints=goal_payload.get("constraints", {}) if isinstance(goal_payload.get("constraints", {}), Mapping) else {},
        execution_mode=str(goal_payload.get("execution_mode", "replay")),
        budgets=MissionBudgets(
            max_steps=budgets_payload.get("max_steps"),
            max_runtime_s=budgets_payload.get("max_runtime_s"),
            max_travel_m=budgets_payload.get("max_travel_m"),
            max_observations=budgets_payload.get("max_observations"),
            max_artifacts=budgets_payload.get("max_artifacts"),
            max_tool_calls=budgets_payload.get("max_tool_calls"),
            max_provider_calls=budgets_payload.get("max_provider_calls"),
        ),
        requested_artifacts=tuple(str(item) for item in goal_payload.get("requested_artifacts", ())),
    )
    invocations: list[ToolInvocation] = []
    for invocation in invocations_payload:
        if not isinstance(invocation, Mapping):
            raise MissionValidationError("tool invocation must be an object")
        approval_payload = invocation.get("approval")
        approval = None
        if approval_payload is not None:
            if not isinstance(approval_payload, Mapping):
                raise MissionValidationError("approval must be an object")
            approval = ApprovalGrant(**dict(approval_payload))
        arguments = invocation.get("arguments", {})
        if not isinstance(arguments, Mapping):
            raise MissionValidationError("tool invocation arguments must be an object")
        provenance = invocation.get("provenance", {})
        if not isinstance(provenance, Mapping):
            raise MissionValidationError("tool invocation provenance must be an object")
        invocations.append(
            ToolInvocation(
                correlation_id=str(invocation.get("correlation_id", "")),
                tool_id=str(invocation.get("tool_id", "")),
                tool_version=str(invocation.get("tool_version", "")),
                arguments=dict(arguments),
                approval=approval,
                requested_at_s=float(invocation.get("requested_at_s", 0.0)),
                provenance=dict(provenance),
            )
        )
    dependencies = payload.get("dependencies", ())
    if not isinstance(dependencies, Sequence) or isinstance(dependencies, (str, bytes)):
        raise MissionValidationError("plan dependencies must be an array")
    return MissionPlan(
        goal=goal,
        invocations=tuple(invocations),
        plan_id=str(payload.get("plan_id", "mission-plan")),
        dependencies=tuple(tuple(str(part) for part in edge) for edge in dependencies),
    )


class _MissionRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        line = self.rfile.readline(4_000_001)
        if not line or len(line) > 4_000_000:
            return
        try:
            request = json.loads(line.decode("utf-8"))
            if not isinstance(request, Mapping):
                raise MissionValidationError("request must be an object")
            result = self.server.dispatch(request)  # type: ignore[attr-defined]
            response = {"ok": True, "result": result}
        except (MissionValidationError, ValueError, TypeError, json.JSONDecodeError) as exc:
            response = {"ok": False, "error": {"type": exc.__class__.__name__, "message": str(exc)}}
        except Exception as exc:  # pragma: no cover - process boundary must fail closed.
            response = {"ok": False, "error": {"type": "ServiceError", "message": f"mission service failure: {exc.__class__.__name__}"}}
        self.wfile.write((_json_dump(response) + "\n").encode("utf-8"))


class MissionServiceServer(socketserver.ThreadingUnixStreamServer):
    """Owner process API over a user-only Unix-domain socket."""

    daemon_threads = True

    def __init__(
        self,
        socket_path: str | Path,
        service_factory: Callable[[], MissionService],
        prompt_controller_factory: Optional[Callable[[MissionService], Any]] = None,
    ) -> None:
        self.socket_path = Path(socket_path)
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self._owner_lock = open(f"{self.socket_path}.lock", "a+b")
        self._server_resources_closed = False
        try:
            fcntl.flock(self._owner_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._owner_lock.close()
            raise MissionValidationError(
                f"mission service already running for socket: {self.socket_path}"
            ) from exc
        try:
            if self.socket_path.exists():
                if not stat.S_ISSOCK(self.socket_path.stat().st_mode):
                    raise MissionValidationError(
                        f"mission service socket path is not a socket: {self.socket_path}"
                    )
                self.socket_path.unlink()
            self.service = service_factory()
            self.prompt_controller = (
                None
                if prompt_controller_factory is None
                else prompt_controller_factory(self.service)
            )
            super().__init__(str(self.socket_path), _MissionRequestHandler)
        except Exception:
            if not self._server_resources_closed:
                if getattr(self, "prompt_controller", None) is not None:
                    self.prompt_controller.close()
                if hasattr(self, "service"):
                    self.service.close()
                if not self._owner_lock.closed:
                    fcntl.flock(self._owner_lock.fileno(), fcntl.LOCK_UN)
                    self._owner_lock.close()
                self._server_resources_closed = True
            raise
        os.chmod(self.socket_path, 0o600)

    def server_close(self) -> None:
        if self._server_resources_closed:
            return
        self._server_resources_closed = True
        super().server_close()
        if self.prompt_controller is not None:
            self.prompt_controller.close()
        self.service.close()
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass
        if not self._owner_lock.closed:
            fcntl.flock(self._owner_lock.fileno(), fcntl.LOCK_UN)
            self._owner_lock.close()

    def dispatch(self, request: Mapping[str, Any]) -> Any:
        operation = str(request.get("operation", ""))
        if operation == "submit":
            plan_payload = request.get("plan")
            if not isinstance(plan_payload, Mapping):
                raise MissionValidationError("submit requires a plan object")
            return self.service.submit_plan(
                mission_plan_from_json(plan_payload),
                session_id=str(request.get("session_id", "")),
                source=str(request.get("source", "api")),
                credential_namespace=request.get("credential_namespace"),
            )
        if operation == "status":
            return self.service.status(str(request.get("mission_id", "")))
        if operation == "session_status":
            return self.service.session_status(str(request.get("session_id", "")))
        if operation == "cancel":
            return self.service.cancel(str(request.get("session_id", "")), reason=str(request.get("reason", "operator cancel")))
        if operation == "capabilities":
            return self.service.capabilities()
        if operation == "events":
            mission_id = request.get("mission_id")
            return self.service.events(None if mission_id is None else str(mission_id))
        if operation == "service_snapshot":
            if self.prompt_controller is None:
                return {
                    "api_version": "mission_api.v2",
                    "mode": self.service.mode,
                    "source_sha": self.service.source_sha,
                    "deployed_sha": self.service.deployed_sha,
                    "planning_enabled": False,
                    "live_execution_enabled": self.service.live_execution_enabled,
                    "capabilities": self.service.capabilities(),
                }
            return self.prompt_controller.service_snapshot()
        if operation.startswith("prompt_") and self.prompt_controller is None:
            raise MissionValidationError("prompt mission controller is not configured")
        if operation == "prompt_submit":
            submit_arguments: dict[str, Any] = {
                "session_id": str(request.get("session_id", "")),
                "source": str(request.get("source", "web")),
                "mission_id": (
                    None
                    if request.get("mission_id") is None
                    else str(request["mission_id"])
                ),
                "operator": str(request.get("operator", "")),
                "authentication_source": str(
                    request.get("authentication_source", "")
                ),
            }
            if "mission_lease_s" in request:
                submit_arguments["mission_lease_s"] = request[
                    "mission_lease_s"
                ]
            return self.prompt_controller.submit(
                str(request.get("prompt", "")),
                **submit_arguments,
            )
        if operation == "prompt_status":
            return self.prompt_controller.status(str(request.get("mission_id", "")))
        if operation == "prompt_latest":
            return self.service.latest_prompt_status(str(request.get("session_id", "")))
        if operation == "prompt_approve":
            approve_arguments: dict[str, Any] = {
                "supplied_approval": str(
                    request.get("approval_phrase", "")
                ),
                "operator": str(request.get("operator", "")),
                "authentication_source": str(
                    request.get("authentication_source", "")
                ),
            }
            room = request.get("physical_room_confirmation")
            if room is not None:
                if not isinstance(room, Mapping):
                    raise MissionValidationError(
                        "physical room confirmation must be an object"
                    )
                approve_arguments["physical_room_confirmation"] = dict(room)
            return self.prompt_controller.approve(
                str(request.get("mission_id", "")),
                **approve_arguments,
            )
        if operation == "prompt_cancel":
            return self.prompt_controller.cancel(
                str(request.get("mission_id", "")),
                reason=str(request.get("reason", "operator cancelled mission")),
            )
        raise MissionValidationError(f"unsupported mission service operation: {operation}")
