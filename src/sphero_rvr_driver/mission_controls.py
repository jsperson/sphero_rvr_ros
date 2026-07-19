"""Authenticated Mission API controls with physical-start gating.

This module is ROS-free and dependency-free.  It wraps the versioned Mission API
state machine with explicit authentication/authorization, audit records, and a
physical approval gate before any motor-capable mission start is allowed.  It is
not a generic ROS bridge and it exposes no direct motor command route.
"""

from __future__ import annotations

import html
import json
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Sequence

from .mission_api import MissionApiVersion, MissionCommand, MissionEventKind, MissionSnapshot, MissionStateMachine


class MissionControlError(ValueError):
    """Raised when an authenticated Mission API control request is rejected."""


class MissionExecutionMode(str, Enum):
    REPLAY = "replay"
    MOCK = "mock"
    PHYSICAL = "physical"


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
    reason: str = ""

    def __post_init__(self) -> None:
        if not str(self.approved_by).strip():
            raise MissionControlError("physical start approval requires approved_by")
        if not str(self.approved_at).strip():
            raise MissionControlError("physical start approval requires approved_at")
        if not str(self.gate_id).strip():
            raise MissionControlError("physical start approval requires gate_id")

    def to_json_dict(self) -> dict[str, str]:
        return dict(asdict(self))


@dataclass(frozen=True)
class StaticMissionControlsBundle:
    index_html: str
    manifest: Mapping[str, Any]
    service_worker_js: str


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
    """Stateful authenticated controls over one Mission API command."""

    REQUIRED_PERMISSIONS = {
        "mission.start": "mission:start",
        "mission.cancel": "mission:cancel",
        "mission.pause": "mission:pause",
    }

    def __init__(self, command: MissionCommand):
        self.command = command
        self._machine = MissionStateMachine(command)
        self._audit_log: list[MissionAuditEvent] = []
        self._execution_mode = MissionExecutionMode.REPLAY
        self._physical_approval: Optional[PhysicalStartApproval] = None

    def snapshot(self) -> MissionSnapshot:
        return self._machine.snapshot()

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
    ) -> MissionSnapshot:
        mode = self._normalize_mode(mode)
        self._execution_mode = mode
        principal = self._authorize(principal, "mission.start", mode=mode)
        if mode is MissionExecutionMode.PHYSICAL and physical_approval is None:
            self._audit(
                action="mission.start",
                actor=principal.subject,
                decision="denied",
                reason="physical start approval required",
                mode=mode,
                physical_approval=None,
            )
            raise MissionControlError("physical start approval required for motor-capable mission start")

        self._physical_approval = physical_approval if mode is MissionExecutionMode.PHYSICAL else None
        self._machine.apply(MissionEventKind.START_REQUESTED, reason=reason)
        snapshot = self._machine.apply(MissionEventKind.VALIDATED, reason="mission api contract validated")
        self._audit(
            action="mission.start",
            actor=principal.subject,
            decision="allowed",
            reason=reason,
            mode=mode,
            physical_approval=self._physical_approval,
        )
        return snapshot

    def pause(self, principal: Optional[MissionPrincipal], *, reason: str = "pause requested") -> MissionSnapshot:
        principal = self._authorize(principal, "mission.pause", mode=self._execution_mode)
        snapshot = self._machine.apply(MissionEventKind.PAUSE_REQUESTED, reason=reason)
        self._audit(
            action="mission.pause",
            actor=principal.subject,
            decision="allowed",
            reason=reason,
            mode=self._execution_mode,
            physical_approval=self._physical_approval,
        )
        return snapshot

    def cancel(self, principal: Optional[MissionPrincipal], *, reason: str = "cancel requested") -> MissionSnapshot:
        principal = self._authorize(principal, "mission.cancel", mode=self._execution_mode)
        snapshot = self._machine.cancel(reason=reason)
        self._audit(
            action="mission.cancel",
            actor=principal.subject,
            decision="allowed",
            reason=reason,
            mode=self._execution_mode,
            physical_approval=self._physical_approval,
        )
        return snapshot

    def robot_safety_event(self, event: str, *, reason: str) -> MissionSnapshot:
        normalized = str(event).strip().lower()
        if normalized == "estop":
            snapshot = self._machine.estop(reason=reason)
        elif normalized == "blocked":
            snapshot = self._machine.block(reason=reason)
        else:
            raise MissionControlError(f"unsupported robot-side safety event: {event}")
        self._audit(
            action=f"robot_safety.{normalized}",
            actor="robot-side-supervisor",
            decision="latched",
            reason=reason,
            mode=self._execution_mode,
            physical_approval=self._physical_approval,
        )
        return snapshot

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": self.command.api_version.value,
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

    def _authorize(
        self,
        principal: Optional[MissionPrincipal],
        action: str,
        *,
        mode: MissionExecutionMode,
    ) -> MissionPrincipal:
        permission = self.REQUIRED_PERMISSIONS[action]
        if principal is None:
            self._audit(
                action=action,
                actor="anonymous",
                decision="denied",
                reason="authenticated principal required",
                mode=mode,
                physical_approval=None,
            )
            raise MissionControlError("authenticated principal required")
        if not principal.has_permission(permission):
            self._audit(
                action=action,
                actor=principal.subject,
                decision="denied",
                reason=f"missing permission: {permission}",
                mode=mode,
                physical_approval=None,
            )
            raise MissionControlError(f"missing permission: {permission}")
        return principal

    def _audit(
        self,
        *,
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
                api_version=self.command.api_version,
                action=action,
                actor=actor,
                mission_id=self.command.mission_id,
                decision=decision,
                reason=reason,
                execution_mode=mode,
                physical_gate=self._physical_gate_payload(physical_approval),
                linked_mission_events=tuple(self.snapshot().event_log),
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
    api_version = payload.get("api_version", MissionApiVersion.V1.value)
    if api_version != MissionApiVersion.V1.value:
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
  <main data-api-version=\"mission_api.v1\">
    <h1>{safe_name}</h1>
    <p>Controls call only the versioned Mission API. The robot-side STOP/ESTOP/collision supervisor remains independent.</p>
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
        reason=str(value.get("reason", "")),
    )
