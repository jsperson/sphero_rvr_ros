"""Authenticated controls for canonical Mission API plans with physical-start gating.

This module is ROS-free and dependency-free. It wraps a validated Mission API
plan with explicit authentication/authorization, audit records, and a physical
approval gate before any motor-capable physical start is allowed. It is not a
generic ROS bridge and it exposes no direct motor command route.
"""

from __future__ import annotations

import html
import hmac
import hashlib
import json
import secrets
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Mapping, Optional, Sequence

from .mission_api import MissionApiVersion, MissionPlan

_PHYSICAL_GATE_SECRET = secrets.token_bytes(32)
_USED_PHYSICAL_GATE_SIGNATURES: set[str] = set()


class MissionControlError(ValueError):
    """Raised when an authenticated Mission API control request is rejected."""


class MissionExecutionMode(str, Enum):
    REPLAY = "replay"
    MOCK = "mock"
    PHYSICAL = "physical"


class MissionControlState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    CANCELLED = "CANCELLED"
    BLOCKED = "BLOCKED"
    ESTOPPED = "ESTOPPED"


@dataclass(frozen=True)
class MissionPrincipal:
    subject: str
    permissions: Sequence[str] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if not str(self.subject).strip():
            raise MissionControlError("authenticated principal required")
        object.__setattr__(self, "subject", str(self.subject).strip())
        object.__setattr__(self, "permissions", tuple(str(permission) for permission in self.permissions))

    def has_permission(self, permission: str) -> bool:
        return permission in self.permissions

    def to_json_dict(self) -> dict[str, Any]:
        return {"subject": self.subject, "permissions": list(self.permissions)}


@dataclass(frozen=True)
class PhysicalStartApproval:
    approved_by: str
    approved_at: str
    gate_id: str
    expires_at: str = ""
    mission_id: str = ""
    issued_to: str = "mission-runtime"
    reason: str = ""
    gate_signature: str = ""

    def __post_init__(self) -> None:
        if not str(self.approved_by).strip():
            raise MissionControlError("physical start approval requires approved_by")
        if not str(self.approved_at).strip():
            raise MissionControlError("physical start approval requires approved_at")
        if not str(self.gate_id).strip():
            raise MissionControlError("physical start approval requires gate_id")

    @property
    def trusted_gate(self) -> bool:
        return self.is_trusted_for(mission_id=self.mission_id, issued_to=self.issued_to)

    def is_trusted_for(self, *, mission_id: str, issued_to: str = "mission-runtime", now: Optional[datetime] = None) -> bool:
        if self.mission_id != mission_id or self.issued_to != issued_to or not self.gate_signature:
            return False
        try:
            approved_at = _parse_timestamp(self.approved_at)
            expires_at = _parse_timestamp(self.expires_at)
        except MissionControlError:
            return False
        now_utc = datetime.now(timezone.utc) if now is None else now.astimezone(timezone.utc)
        if expires_at <= approved_at or not (approved_at <= now_utc <= expires_at):
            return False
        return hmac.compare_digest(self.gate_signature, _physical_gate_signature(self))

    def replay_key(self) -> str:
        return self.gate_signature

    def to_json_dict(self) -> dict[str, str | bool]:
        payload = dict(asdict(self))
        payload["trusted_gate"] = self.trusted_gate
        return payload


def issue_physical_start_approval(
    *,
    approved_by: str,
    approved_at: str,
    expires_at: str,
    gate_id: str,
    mission_id: str,
    reason: str = "",
    issued_to: str = "mission-runtime",
) -> PhysicalStartApproval:
    """Issue a process-local trusted physical gate approval.

    Browser/API payloads can request physical start, but they cannot mint this
    signature because the signing key never serializes into the controls API.
    """

    approval = PhysicalStartApproval(
        approved_by=approved_by,
        approved_at=approved_at,
        expires_at=expires_at,
        gate_id=gate_id,
        mission_id=mission_id,
        issued_to=issued_to,
        reason=reason,
    )
    return PhysicalStartApproval(
        approved_by=approval.approved_by,
        approved_at=approval.approved_at,
        expires_at=approval.expires_at,
        gate_id=approval.gate_id,
        mission_id=approval.mission_id,
        issued_to=approval.issued_to,
        reason=approval.reason,
        gate_signature=_physical_gate_signature(approval),
    )


def _physical_gate_signature(approval: PhysicalStartApproval) -> str:
    message = "|".join(
        (
            approval.approved_by,
            approval.approved_at,
            approval.expires_at,
            approval.gate_id,
            approval.mission_id,
            approval.issued_to,
            approval.reason,
        )
    ).encode("utf-8")
    return hmac.new(_PHYSICAL_GATE_SECRET, message, hashlib.sha256).hexdigest()


@dataclass(frozen=True)
class StaticMissionControlsBundle:
    index_html: str
    manifest: Mapping[str, Any]
    service_worker_js: str


@dataclass(frozen=True)
class MissionControlSnapshot:
    plan: MissionPlan
    state: MissionControlState
    event_log: tuple[str, ...]

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": MissionApiVersion.V2.value,
            "plan_id": self.plan.plan_id,
            "mission_id": self.plan.goal.goal_id,
            "state": self.state.value,
            "event_log": list(self.event_log),
            "telemetry": {
                "api_version": MissionApiVersion.V2.value,
                "state": self.state.value,
                "terminal": self.state in {MissionControlState.CANCELLED, MissionControlState.BLOCKED, MissionControlState.ESTOPPED},
                "direct_ros_commands_allowed": False,
                "generic_ros_bridge": False,
                "command_path": [],
                "allowed_tool_ids": [invocation.tool_id for invocation in self.plan.invocations],
                "required_artifacts": list(self.plan.goal.requested_artifacts),
            },
        }


@dataclass(frozen=True)
class MissionAuditEvent:
    sequence: int
    api_version: MissionApiVersion
    action: str
    actor: str
    mission_id: str
    decision: str
    reason: str
    execution_mode: MissionExecutionMode
    physical_gate: Mapping[str, Any]
    linked_mission_events: tuple[str, ...]

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "api_version": self.api_version.value,
            "action": self.action,
            "actor": self.actor,
            "mission_id": self.mission_id,
            "decision": self.decision,
            "reason": self.reason,
            "execution_mode": self.execution_mode.value,
            "physical_gate": dict(self.physical_gate),
            "linked_mission_events": list(self.linked_mission_events),
        }


class MissionControlSession:
    """Stateful authenticated controls over one canonical Mission API plan."""

    REQUIRED_PERMISSIONS = {
        "mission.start": "mission:start",
        "mission.cancel": "mission:cancel",
        "mission.pause": "mission:pause",
    }

    def __init__(self, plan: MissionPlan):
        self.plan = plan
        self._state = MissionControlState.IDLE
        self._event_log: list[str] = []
        self._audit_log: list[MissionAuditEvent] = []
        self._execution_mode = MissionExecutionMode.REPLAY
        self._physical_approval: Optional[PhysicalStartApproval] = None

    def snapshot(self) -> MissionControlSnapshot:
        return MissionControlSnapshot(self.plan, self._state, tuple(self._event_log))

    @property
    def audit_log(self) -> tuple[MissionAuditEvent, ...]:
        return tuple(self._audit_log)

    def start(
        self,
        principal: Optional[MissionPrincipal],
        *,
        mode: MissionExecutionMode,
        physical_approval: Optional[PhysicalStartApproval] = None,
        reason: str = "mission start requested",
    ) -> MissionControlSnapshot:
        mode = self._normalize_mode(mode)
        if self._state in {MissionControlState.CANCELLED, MissionControlState.ESTOPPED}:
            self._audit("mission.start", "anonymous" if principal is None else principal.subject, "denied", f"cannot start from terminal state {self._state.value}", mode, None)
            raise MissionControlError(f"cannot start from terminal state {self._state.value}")
        if self._state is not MissionControlState.IDLE:
            self._audit("mission.start", "anonymous" if principal is None else principal.subject, "denied", f"cannot start from state {self._state.value}", mode, None)
            raise MissionControlError(f"cannot start from state {self._state.value}")
        self._execution_mode = mode
        principal = self._authorize(principal, "mission.start", mode=mode)
        if mode is MissionExecutionMode.PHYSICAL and physical_approval is None:
            self._audit("mission.start", principal.subject, "denied", "physical start approval required", mode, None)
            raise MissionControlError("physical start approval required for motor-capable mission start")
        if mode is MissionExecutionMode.PHYSICAL and physical_approval is not None and not physical_approval.is_trusted_for(mission_id=self.plan.goal.goal_id):
            self._audit("mission.start", principal.subject, "denied", "trusted physical gate approval required", mode, None)
            raise MissionControlError("trusted physical gate approval required for physical mission start")
        if mode is MissionExecutionMode.PHYSICAL and physical_approval is not None and physical_approval.replay_key() in _USED_PHYSICAL_GATE_SIGNATURES:
            self._audit("mission.start", principal.subject, "denied", "trusted physical gate approval required", mode, None)
            raise MissionControlError("trusted physical gate approval required for physical mission start")

        self._physical_approval = physical_approval if mode is MissionExecutionMode.PHYSICAL else None
        if self._state is MissionControlState.IDLE:
            self._event_log.extend(("start_requested", "validated"))
        self._state = MissionControlState.RUNNING
        if mode is MissionExecutionMode.PHYSICAL and physical_approval is not None:
            _USED_PHYSICAL_GATE_SIGNATURES.add(physical_approval.replay_key())
        self._audit("mission.start", principal.subject, "allowed", reason, mode, self._physical_approval)
        return self.snapshot()

    def pause(self, principal: Optional[MissionPrincipal], *, reason: str = "pause requested") -> MissionControlSnapshot:
        principal = self._authorize(principal, "mission.pause", mode=self._execution_mode)
        if self._state is not MissionControlState.RUNNING:
            self._audit("mission.pause", principal.subject, "denied", f"cannot pause from state {self._state.value}", self._execution_mode, self._physical_approval)
            raise MissionControlError(f"cannot pause from state {self._state.value}")
        self._state = MissionControlState.PAUSED
        self._event_log.append("pause_requested")
        self._audit("mission.pause", principal.subject, "allowed", reason, self._execution_mode, self._physical_approval)
        return self.snapshot()

    def cancel(self, principal: Optional[MissionPrincipal], *, reason: str = "cancel requested") -> MissionControlSnapshot:
        principal = self._authorize(principal, "mission.cancel", mode=self._execution_mode)
        if self._state in {MissionControlState.CANCELLED, MissionControlState.ESTOPPED}:
            self._audit("mission.cancel", principal.subject, "denied", f"cannot cancel from terminal state {self._state.value}", self._execution_mode, self._physical_approval)
            raise MissionControlError(f"cannot cancel from terminal state {self._state.value}")
        self._state = MissionControlState.CANCELLED
        self._event_log.append("cancelled")
        self._audit("mission.cancel", principal.subject, "allowed", reason, self._execution_mode, self._physical_approval)
        return self.snapshot()

    def robot_safety_event(self, event: str, *, reason: str) -> MissionControlSnapshot:
        normalized = str(event).strip().lower()
        if normalized == "estop":
            self._state = MissionControlState.ESTOPPED
            self._event_log.append("estopped")
        elif normalized == "blocked":
            self._state = MissionControlState.BLOCKED
            self._event_log.append("blocked")
        else:
            raise MissionControlError(f"unsupported robot-side safety event: {event}")
        self._audit(f"robot_safety.{normalized}", "robot-side-supervisor", "latched", reason, self._execution_mode, self._physical_approval)
        return self.snapshot()

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": MissionApiVersion.V2.value,
            "mission": self.snapshot().to_json_dict(),
            "execution_mode": self._execution_mode.value,
            "physical_gate": self._physical_gate_payload(self._physical_approval),
            "audit_log": [event.to_json_dict() for event in self._audit_log],
            "allowed_methods": ["POST"],
            "write_endpoints": ["/api/mission/start", "/api/mission/pause", "/api/mission/cancel"],
            "motor_command_route_exposed": False,
            "generic_ros_bridge": False,
            "browser_stop_is_sole_safety_mechanism": False,
            "robot_side_safety": {
                "independent_stop_estop_collision_supervisor": True,
                "browser_stop_is_advisory_control_only": True,
                "direct_motor_bypass_allowed": False,
            },
        }

    def _authorize(self, principal: Optional[MissionPrincipal], action: str, *, mode: MissionExecutionMode) -> MissionPrincipal:
        permission = self.REQUIRED_PERMISSIONS[action]
        if principal is None:
            self._audit(action, "anonymous", "denied", "authenticated principal required", mode, None)
            raise MissionControlError("authenticated principal required")
        if not principal.has_permission(permission):
            self._audit(action, principal.subject, "denied", f"missing permission: {permission}", mode, None)
            raise MissionControlError(f"missing permission: {permission}")
        return principal

    def _audit(
        self,
        action: str,
        actor: str,
        decision: str,
        reason: str,
        mode: MissionExecutionMode,
        physical_approval: Optional[PhysicalStartApproval],
    ) -> None:
        self._audit_log.append(
            MissionAuditEvent(
                sequence=len(self._audit_log) + 1,
                api_version=MissionApiVersion.V2,
                action=action,
                actor=actor,
                mission_id=self.plan.goal.goal_id,
                decision=decision,
                reason=reason,
                execution_mode=mode,
                physical_gate=self._physical_gate_payload(physical_approval),
                linked_mission_events=tuple(self._event_log),
            )
        )

    @staticmethod
    def _normalize_mode(mode: MissionExecutionMode) -> MissionExecutionMode:
        if isinstance(mode, str):
            return MissionExecutionMode(mode)
        return mode

    @staticmethod
    def _physical_gate_payload(approval: Optional[PhysicalStartApproval]) -> dict[str, Any]:
        return {
            "required_for_physical_start": True,
            "approved": approval is not None,
            "approval": None if approval is None else approval.to_json_dict(),
        }


def handle_mission_control_request(
    method: str,
    path: str,
    body: str,
    session: MissionControlSession,
    principal: Optional[MissionPrincipal],
) -> tuple[int, str, str]:
    """Tiny dependency-free router for authenticated Mission API controls."""

    normalized_method = method.upper()
    normalized_path = path.rstrip("/") or "/"
    if normalized_path not in {"/api/mission/start", "/api/mission/cancel", "/api/mission/pause"}:
        raise MissionControlError(f"{normalized_path} is not exposed by authenticated Mission API controls")
    if normalized_method != "POST":
        raise MissionControlError("authenticated Mission API controls only support POST")

    payload = json.loads(body or "{}")
    api_version = payload.get("api_version", MissionApiVersion.V2.value)
    if api_version != MissionApiVersion.V2.value:
        raise MissionControlError(f"unsupported Mission API version: {api_version}")

    if normalized_path == "/api/mission/start":
        approval = _physical_approval_from_payload(payload.get("physical_approval"))
        session.start(
            principal,
            mode=MissionExecutionMode(payload.get("execution_mode", MissionExecutionMode.REPLAY.value)),
            physical_approval=approval,
            reason=str(payload.get("reason", "mission start requested")),
        )
    elif normalized_path == "/api/mission/pause":
        session.pause(principal, reason=str(payload.get("reason", "pause requested")))
    else:
        session.cancel(principal, reason=str(payload.get("reason", "cancel requested")))

    return 200, "application/json", json.dumps(session.to_json_dict(), sort_keys=True)


def build_static_controls_bundle(*, app_name: str = "RVR Mission Controls") -> StaticMissionControlsBundle:
    """Build a static shell with only Mission API controls and visible safety caveats."""

    safe_name = html.escape(app_name, quote=True)
    index_html = f"""<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\">
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
  <title>{safe_name}</title>
</head>
<body>
  <main data-api-version=\"mission_api.v2\">
    <h1>{safe_name}</h1>
    <p>Controls call only the canonical Mission API. The robot-side STOP/ESTOP/collision supervisor remains independent.</p>
    <section aria-label=\"Mission controls\">
      <button data-action=\"start-replay\" data-route=\"/api/mission/start\">Start replay mission</button>
      <button data-action=\"start-physical\" data-route=\"/api/mission/start\">Request physical start</button>
      <button data-action=\"pause\" data-route=\"/api/mission/pause\">Pause</button>
      <button data-action=\"cancel\" data-route=\"/api/mission/cancel\">Cancel / STOP mission</button>
    </section>
    <output id=\"mission-status\" aria-live=\"polite\"></output>
  </main>
</body>
</html>
"""
    return StaticMissionControlsBundle(
        index_html=index_html,
        manifest={"name": app_name, "short_name": "RVR Control", "display": "standalone"},
        service_worker_js="self.addEventListener('install', () => self.skipWaiting());\n",
    )


def _physical_approval_from_payload(value: Any) -> Optional[PhysicalStartApproval]:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise MissionControlError("physical_approval must be an object")
    return PhysicalStartApproval(
        approved_by=str(value.get("approved_by", "")),
        approved_at=str(value.get("approved_at", "")),
        gate_id=str(value.get("gate_id", "")),
        expires_at=str(value.get("expires_at", "")),
        mission_id=str(value.get("mission_id", "")),
        issued_to=str(value.get("issued_to", "mission-runtime")),
        reason=str(value.get("reason", "")),
    )


def _parse_timestamp(value: str) -> datetime:
    try:
        normalized = value.replace("Z", "+00:00")
        parsed = datetime.fromisoformat(normalized)
    except ValueError as exc:
        raise MissionControlError("physical start approval timestamps must be ISO-8601") from exc
    if parsed.tzinfo is None:
        raise MissionControlError("physical start approval timestamps must include timezone")
    return parsed.astimezone(timezone.utc)
