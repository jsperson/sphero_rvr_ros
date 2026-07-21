"""Persistent fail-closed owner for Mission API session state and execution.

The service is deliberately ROS-free.  A deployment binds one reviewed adapter
set, while the durable SQLite ledger survives MCP/CLI processes and machine
restarts.  Motion is never resumed from persisted state.
"""

from __future__ import annotations

from dataclasses import replace
import fcntl
import json
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
        self._clock_s = clock_s or time.time
        self._lock = threading.RLock()
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
                ledger = self._session_ledger(session_id)
                self._validate_cumulative_budget(plan, ledger)
                runtime = DeterministicMissionRuntime(
                    self.registry,
                    self.adapters,
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
    ) -> None:
        self.socket_path = Path(socket_path)
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self._owner_lock = open(f"{self.socket_path}.lock", "a+b")
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
            super().__init__(str(self.socket_path), _MissionRequestHandler)
        except Exception:
            if hasattr(self, "service"):
                self.service.close()
            fcntl.flock(self._owner_lock.fileno(), fcntl.LOCK_UN)
            self._owner_lock.close()
            raise
        os.chmod(self.socket_path, 0o600)

    def server_close(self) -> None:
        super().server_close()
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
        if operation == "events":
            mission_id = request.get("mission_id")
            return self.service.events(None if mission_id is None else str(mission_id))
        raise MissionValidationError(f"unsupported mission service operation: {operation}")
