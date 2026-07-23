"""Mission web console over typed mock/replay or Pi-local service contracts.

The browser-facing router in this module has no ROS, serial, motor, OAuth, or
live-execution adapter. Natural-language prompts are turned into deterministic
fixture proposals by the same :class:`PromptDrivePlanner` that validates the
Pi-local model output. Digest approval is checked by the existing prompt-drive
approval contract, then a server-owned mock scenario advances mission state.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import threading
from dataclasses import asdict, dataclass, field
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Mapping, Optional, Protocol, Sequence
from urllib.parse import urlsplit

from .mission_api import MissionApiVersion, MissionValidationError
from .mission_service_client import MissionServiceClient
from .prompt_drive import (
    PROMPT_DRIVE_API_VERSION,
    PromptDriveDecision,
    PromptDriveLimits,
    PromptDrivePlanner,
    PromptDriveProposal,
    PromptDriveProviderResponse,
    approval_phrase,
    approved_live_route,
    prompt_drive_proposal_from_json,
)

WEB_API_VERSION = "rvr_mission_web.v1"
MAX_REQUEST_BYTES = 64 * 1024
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
TERMINAL_ARTIFACT_PATH = "/api/web/artifacts/terminal-result"


class MissionWebError(ValueError):
    """Raised when a web-console request fails closed."""


class WebMissionState(str, Enum):
    READY = "READY"
    RECEIVED = "RECEIVED"
    PLANNING = "PLANNING"
    PROPOSED = "PROPOSED"
    APPROVED = "APPROVED"
    QUEUED = "QUEUED"
    REJECTED = "REJECTED"
    RUNNING = "RUNNING"
    COMPLETE = "COMPLETE"
    CANCELLED = "CANCELLED"
    STOPPED = "STOPPED"
    ESTOPPED = "ESTOPPED"
    BLOCKED = "BLOCKED"
    FAILED = "FAILED"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"


class MockScenario(str, Enum):
    SUCCESS = "success"
    REJECTION = "rejection"
    CANCELLATION = "cancellation"
    STOP = "stop"
    ESTOP = "estop"
    COLLISION = "collision_blocked"
    STALE = "stale_telemetry"


class LiveScenario(str, Enum):
    LIVE = "live"


TERMINAL_STATES = {
    WebMissionState.REJECTED,
    WebMissionState.COMPLETE,
    WebMissionState.CANCELLED,
    WebMissionState.STOPPED,
    WebMissionState.ESTOPPED,
    WebMissionState.BLOCKED,
    WebMissionState.FAILED,
    WebMissionState.RECOVERY_REQUIRED,
}


@dataclass(frozen=True)
class ScenarioDefinition:
    scenario: Enum
    label: str
    description: str

    def to_json_dict(self) -> dict[str, str]:
        return {
            "id": self.scenario.value,
            "label": self.label,
            "description": self.description,
        }


SCENARIOS: tuple[ScenarioDefinition, ...] = (
    ScenarioDefinition(MockScenario.SUCCESS, "Successful mission", "Proposal runs to a proven mock terminal result."),
    ScenarioDefinition(MockScenario.REJECTION, "Model rejection", "Unsupported reverse motion is rejected without approval or execution."),
    ScenarioDefinition(MockScenario.CANCELLATION, "Cancellation", "An approved mock mission is cancelled while running."),
    ScenarioDefinition(MockScenario.STOP, "STOP", "The independent safety path reports a stopped terminal result."),
    ScenarioDefinition(MockScenario.ESTOP, "ESTOP", "The independent emergency stop latches the mock mission."),
    ScenarioDefinition(MockScenario.COLLISION, "Collision blocked", "The collision supervisor blocks progress before completion."),
    ScenarioDefinition(MockScenario.STALE, "Stale telemetry", "Old sensor evidence blocks execution fail-closed."),
)

LIVE_SCENARIOS: tuple[ScenarioDefinition, ...] = (
    ScenarioDefinition(
        LiveScenario.LIVE,
        "Pi mission service",
        "Real Pi-local OAuth planning; physical execution follows the deployed service gate.",
    ),
)


@dataclass(frozen=True)
class MapPoint:
    x_m: float
    y_m: float

    def to_json_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class MapObstacle:
    obstacle_id: str
    x_m: float
    y_m: float
    width_m: float
    height_m: float
    label: str

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class MapObject:
    object_id: str
    label: str
    x_m: float
    y_m: float
    confidence: float
    evidence_ref: str

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class MapFixture:
    frame: str = "map"
    width_m: float = 6.0
    height_m: float = 4.0
    origin: MapPoint = MapPoint(0.0, 0.0)
    proposed_route: Sequence[MapPoint] = field(
        default_factory=lambda: (
            MapPoint(0.8, 3.15),
            MapPoint(1.65, 3.15),
            MapPoint(2.45, 2.55),
            MapPoint(3.35, 2.55),
            MapPoint(4.15, 1.75),
            MapPoint(5.15, 1.15),
        )
    )
    obstacles: Sequence[MapObstacle] = field(
        default_factory=lambda: (
            MapObstacle("table", 2.15, 0.55, 1.35, 0.75, "table"),
            MapObstacle("chair", 4.15, 2.65, 0.65, 0.65, "chair"),
            MapObstacle("box", 0.65, 1.25, 0.55, 0.85, "box"),
        )
    )
    objects: Sequence[MapObject] = field(
        default_factory=lambda: (
            MapObject("shoe-001", "shoe", 3.78, 1.36, 0.91, "fixture://camera/shoe-001"),
            MapObject("shoe-002", "shoe", 5.12, 2.82, 0.84, "fixture://camera/shoe-002"),
        )
    )

    def to_json_dict(self, *, progress: float) -> dict[str, Any]:
        route = tuple(self.proposed_route)
        visible_count = 0 if progress <= 0.0 else max(1, math.ceil(progress * (len(route) - 1)) + 1)
        traveled = route[: min(len(route), visible_count)]
        rover = traveled[-1] if traveled else route[0]
        return {
            "frame": self.frame,
            "bounds": {
                "origin": self.origin.to_json_dict(),
                "width_m": self.width_m,
                "height_m": self.height_m,
            },
            "rover": {**rover.to_json_dict(), "yaw_deg": 18.0 if progress else 0.0},
            "proposed_route": [point.to_json_dict() for point in route],
            "traveled_path": [point.to_json_dict() for point in traveled],
            "obstacles": [obstacle.to_json_dict() for obstacle in self.obstacles],
            "objects": [item.to_json_dict() for item in self.objects],
            "fixture_only": True,
        }


@dataclass(frozen=True)
class MissionEvent:
    sequence: int
    event_type: str
    message: str

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class MissionWebResponse:
    status: int
    content_type: str
    body: str


class MissionWebAdapter(Protocol):
    """Typed browser/service boundary; future live adapters must implement it server-side."""

    def scenarios(self) -> Sequence[ScenarioDefinition]: ...

    def snapshot(self) -> Mapping[str, Any]: ...

    def propose(self, prompt: str, scenario: str) -> Mapping[str, Any]: ...

    def approve(
        self,
        supplied_approval: str,
        *,
        confirm_current_proposal: bool = False,
    ) -> Mapping[str, Any]: ...

    def advance(self) -> Mapping[str, Any]: ...

    def cancel(self) -> Mapping[str, Any]: ...


class _FixturePromptProvider:
    provider_id = "mock-replay"
    model_id = "fixture-model-not-live"
    reasoning_effort = "fixture"

    def __init__(self, scenario: MockScenario):
        self.scenario = scenario

    def propose(self, prompt: str, limits: PromptDriveLimits) -> PromptDriveProviderResponse:
        del prompt, limits
        if self.scenario is MockScenario.REJECTION:
            return PromptDriveProviderResponse(
                PromptDriveDecision.REJECT,
                "Reverse motion is outside the bounded prompt-drive MVP envelope.",
            )
        return PromptDriveProviderResponse(
            PromptDriveDecision.PROPOSE,
            "Move through the fixture room and turn toward the final observation point.",
            (
                {"tool_name": "move_distance", "value": 0.20},
                {"tool_name": "turn_angle", "value": 45.0},
                {"tool_name": "move_distance", "value": 0.15},
            ),
        )


class MockReplayMissionAdapter:
    """Stateful deterministic adapter with no path to live execution."""

    mode = "mock/replay"
    live_execution_enabled = False
    direct_ros_commands_allowed = False
    credentials_accepted = False

    def __init__(self, *, source_sha: str = "mock-replay-fixture", map_fixture: MapFixture = MapFixture()):
        self.source_sha = str(source_sha)
        self.map_fixture = map_fixture
        self._lock = threading.RLock()
        self._proposal: Optional[PromptDriveProposal] = None
        self._scenario = MockScenario.SUCCESS
        self._state = WebMissionState.READY
        self._progress = 0.0
        self._terminal_reason = ""
        self._approval_granted = False
        self._events: list[MissionEvent] = []
        self._safety = self._base_safety()

    def scenarios(self) -> Sequence[ScenarioDefinition]:
        return SCENARIOS

    def snapshot(self) -> Mapping[str, Any]:
        with self._lock:
            return self._snapshot_unlocked()

    def propose(self, prompt: str, scenario: str) -> Mapping[str, Any]:
        with self._lock:
            try:
                chosen = MockScenario(str(scenario))
            except ValueError as exc:
                raise MissionWebError(f"unsupported mock scenario: {scenario}") from exc
            planner = PromptDrivePlanner(_FixturePromptProvider(chosen), source_sha=self.source_sha)
            try:
                proposal = planner.propose(prompt)
            except MissionValidationError as exc:
                raise MissionWebError(str(exc)) from exc
            self._scenario = chosen
            self._proposal = proposal
            self._progress = 0.0
            self._terminal_reason = ""
            self._approval_granted = False
            self._events = []
            self._safety = self._base_safety()
            self._record("prompt_received", "Natural-language mission received by mock/replay adapter.")
            self._record("proposal_validated", "Typed prompt-drive decision validated server-side.")
            if proposal.executable:
                self._state = WebMissionState.PROPOSED
            else:
                self._state = WebMissionState.REJECTED
                self._terminal_reason = "model_rejected"
                self._record("mission_rejected", proposal.summary)
            return self._snapshot_unlocked()

    def approve(
        self,
        supplied_approval: str,
        *,
        confirm_current_proposal: bool = False,
    ) -> Mapping[str, Any]:
        del confirm_current_proposal
        with self._lock:
            if self._proposal is None or self._state is not WebMissionState.PROPOSED:
                raise MissionWebError("an executable proposal is required before approval")
            try:
                # This validates the unchanged full digest using the production
                # prompt-drive contract. The resulting route is deliberately
                # discarded; this adapter exposes no executor.
                approved_live_route(self._proposal, supplied_approval, operator="mock-web-operator")
            except MissionValidationError as exc:
                raise MissionWebError(str(exc)) from exc
            self._approval_granted = True
            self._state = WebMissionState.RUNNING
            self._record("approval_granted", "Exact digest-bound approval accepted for simulation only.")
            self._record("simulation_started", "Mock/replay mission entered RUNNING state.")
            return self._snapshot_unlocked()

    def advance(self) -> Mapping[str, Any]:
        with self._lock:
            if self._state in TERMINAL_STATES:
                return self._snapshot_unlocked()
            if self._state is not WebMissionState.RUNNING:
                raise MissionWebError("only a running mock mission can advance")
            if self._scenario is MockScenario.SUCCESS:
                self._progress = min(1.0, self._progress + 0.34)
                self._record("progress", f"Replay progress reached {round(self._progress * 100)}%.")
                if self._progress >= 1.0:
                    self._finish(WebMissionState.COMPLETE, "target_reached", "Mock mission completed with evidence.")
            elif self._scenario is MockScenario.CANCELLATION:
                self._progress = 0.32
                self._finish(WebMissionState.CANCELLED, "cancelled", "Mock operator cancellation acknowledged.")
            elif self._scenario is MockScenario.STOP:
                self._progress = 0.24
                self._safety["stop_active"] = True
                self._finish(WebMissionState.STOPPED, "stop_requested", "Independent STOP reported by fixture.")
            elif self._scenario is MockScenario.ESTOP:
                self._progress = 0.18
                self._safety["estop_latched"] = True
                self._finish(WebMissionState.ESTOPPED, "estop_latched", "Independent ESTOP latched by fixture.")
            elif self._scenario is MockScenario.COLLISION:
                self._progress = 0.41
                self._safety["collision_state"] = "BLOCKED"
                self._finish(WebMissionState.BLOCKED, "collision_veto", "Collision supervisor vetoed the route.")
            elif self._scenario is MockScenario.STALE:
                self._progress = 0.12
                self._safety["telemetry_fresh"] = False
                self._finish(WebMissionState.BLOCKED, "stale_telemetry", "Telemetry freshness gate failed closed.")
            else:
                raise MissionWebError("rejected scenarios cannot enter execution")
            return self._snapshot_unlocked()

    def cancel(self) -> Mapping[str, Any]:
        with self._lock:
            if self._state not in {WebMissionState.PROPOSED, WebMissionState.RUNNING}:
                raise MissionWebError("only a proposed or running mock mission can be cancelled")
            self._finish(WebMissionState.CANCELLED, "cancelled", "Mock operator cancelled the mission.")
            return self._snapshot_unlocked()

    def _finish(self, state: WebMissionState, reason: str, message: str) -> None:
        self._state = state
        self._terminal_reason = reason
        self._record(state.value.lower(), message)

    def _record(self, event_type: str, message: str) -> None:
        self._events.append(MissionEvent(len(self._events) + 1, event_type, message))

    @staticmethod
    def _base_safety() -> dict[str, Any]:
        return {
            "stop_active": False,
            "estop_latched": False,
            "collision_state": "CLEAR",
            "telemetry_fresh": True,
            "independent_robot_safety": True,
            "browser_is_sole_safety_mechanism": False,
        }

    def _snapshot_unlocked(self) -> dict[str, Any]:
        proposal_payload = None if self._proposal is None else self._proposal.to_json_dict()
        phrase = ""
        if self._proposal is not None and self._proposal.executable:
            phrase = approval_phrase(self._proposal)
        result = (
            {
                "status": self._state.value.lower(),
                "terminal_reason": self._terminal_reason,
                "evidence_mode": "mock/replay",
            }
            if self._state in TERMINAL_STATES
            else {}
        )
        return {
            "web_api_version": WEB_API_VERSION,
            "mission_api_version": MissionApiVersion.V2.value,
            "prompt_drive_api_version": PROMPT_DRIVE_API_VERSION,
            "adapter": {
                "mode": self.mode,
                "fixture_only": True,
                "live_execution_enabled": self.live_execution_enabled,
                "direct_ros_commands_allowed": self.direct_ros_commands_allowed,
                "credentials_accepted": self.credentials_accepted,
                "future_live_boundary": "Pi-hosted authenticated mission service",
            },
            "scenario": self._scenario.value,
            "proposal": proposal_payload,
            "approval": {
                "required": bool(self._proposal and self._proposal.executable),
                "enabled": bool(self._proposal and self._proposal.executable),
                "approved": self._approval_granted,
                "proposal_digest": "" if self._proposal is None else self._proposal.proposal_digest,
                "required_phrase": phrase,
                "simulation_only": True,
            },
            "mission": {
                "state": self._state.value,
                "progress": self._progress,
                "terminal": self._state in TERMINAL_STATES,
                "terminal_reason": self._terminal_reason,
                "result": result,
            },
            "artifacts": _terminal_artifacts(result, fixture_only=True),
            "safety": dict(self._safety),
            "events": [event.to_json_dict() for event in self._events],
            "map": self.map_fixture.to_json_dict(progress=self._progress),
        }


class LiveMissionWebAdapter:
    """Default-disabled browser adapter over the Pi-local Unix socket only."""

    direct_ros_commands_allowed = False
    credentials_accepted = False

    def __init__(
        self,
        client: MissionServiceClient,
        *,
        session_id: str = "rvr-web-console",
        operator: str = "tailscale-operator",
    ) -> None:
        self.client = client
        self.session_id = str(session_id).strip()
        self.operator = str(operator).strip()
        if not self.session_id or not self.operator:
            raise MissionWebError("live web session and operator identity are required")
        self._lock = threading.RLock()
        self._request_context = threading.local()
        self._mission_id: Optional[str] = None
        try:
            service = dict(self.client.service_snapshot())
        except MissionValidationError as exc:
            raise MissionWebError(str(exc)) from exc
        if service.get("mode") != "live":
            raise MissionWebError("live web adapter requires a live-mode Pi mission service")
        self._service_snapshot = service
        self.live_execution_enabled = bool(service.get("live_execution_enabled", False))
        self.mode = "live" if self.live_execution_enabled else "live/proposal-only"

    def set_request_identity(self, identity: str) -> None:
        self._request_context.operator = str(identity).strip()

    def clear_request_identity(self) -> None:
        self._request_context.operator = ""

    def _operator_identity(self) -> str:
        return str(getattr(self._request_context, "operator", "")).strip() or self.operator

    def scenarios(self) -> Sequence[ScenarioDefinition]:
        return LIVE_SCENARIOS

    def snapshot(self) -> Mapping[str, Any]:
        with self._lock:
            try:
                self._service_snapshot = dict(self.client.service_snapshot())
                mission = (
                    self.client.prompt_status(self._mission_id)
                    if self._mission_id is not None
                    else self.client.latest_prompt_status(self.session_id)
                )
            except MissionValidationError as exc:
                raise MissionWebError(str(exc)) from exc
            if mission is not None:
                self._mission_id = str(mission["mission_id"])
            self.live_execution_enabled = bool(
                self._service_snapshot.get("live_execution_enabled", False)
            )
            self.mode = "live" if self.live_execution_enabled else "live/proposal-only"
            return self._translate(None if mission is None else dict(mission))

    def propose(self, prompt: str, scenario: str) -> Mapping[str, Any]:
        if str(scenario) != LiveScenario.LIVE.value:
            raise MissionWebError("live adapter accepts only the Pi mission-service scenario")
        with self._lock:
            try:
                snapshot = self.client.submit_prompt(
                    prompt,
                    session_id=self.session_id,
                    source="web",
                )
            except MissionValidationError as exc:
                raise MissionWebError(str(exc)) from exc
            self._mission_id = str(snapshot["mission_id"])
            return self._translate(dict(snapshot))

    def approve(
        self,
        supplied_approval: str,
        *,
        confirm_current_proposal: bool = False,
    ) -> Mapping[str, Any]:
        del supplied_approval
        with self._lock:
            if self._mission_id is None:
                raise MissionWebError("a persisted live proposal is required before approval")
            if not confirm_current_proposal:
                raise MissionWebError("explicit confirmation of the current proposal is required")
            try:
                current = self.client.prompt_status(self._mission_id)
                if not isinstance(current, Mapping) or str(current.get("status", "")).lower() != "proposed":
                    raise MissionWebError("the current live mission is not awaiting approval")
                proposal_payload = current.get("proposal", {})
                if not isinstance(proposal_payload, Mapping):
                    raise MissionWebError("the current live proposal is unavailable")
                # The browser confirms the proposal it is displaying. The Pi
                # recomputes the exact full-digest phrase from freshly read
                # persisted state, so a user no longer copies a hash and the
                # unchanged-proposal binding remains server-owned and audited.
                server_approval = approval_phrase(
                    prompt_drive_proposal_from_json(proposal_payload)
                )
                snapshot = self.client.approve_prompt(
                    self._mission_id,
                    approval_phrase=server_approval,
                    operator=self._operator_identity(),
                )
            except MissionValidationError as exc:
                raise MissionWebError(str(exc)) from exc
            return self._translate(dict(snapshot))

    def advance(self) -> Mapping[str, Any]:
        # Polling observes the Pi owner; it never advances or executes live state.
        return self.snapshot()

    def cancel(self) -> Mapping[str, Any]:
        with self._lock:
            if self._mission_id is None:
                raise MissionWebError("a persisted live mission is required before cancellation")
            try:
                snapshot = self.client.cancel_prompt(
                    self._mission_id,
                    reason=f"authenticated operator {self._operator_identity()} cancelled mission",
                )
            except MissionValidationError as exc:
                raise MissionWebError(str(exc)) from exc
            return self._translate(dict(snapshot))

    def _translate(self, mission: Optional[Mapping[str, Any]]) -> dict[str, Any]:
        capabilities = self._service_snapshot.get("capabilities", {})
        capability = (
            capabilities.get("query_status_telemetry@1.0", {})
            if isinstance(capabilities, Mapping)
            else {}
        )
        live_evidence = capability.get("evidence", {}) if isinstance(capability, Mapping) else {}
        if not isinstance(live_evidence, Mapping):
            live_evidence = {}
        safety = live_evidence.get("safety", {})
        if not isinstance(safety, Mapping):
            safety = {}
        odom = live_evidence.get("odom", {})
        collision = live_evidence.get("collision", {})
        route_progress = live_evidence.get("route_progress", {})
        required_fresh = bool(
            isinstance(odom, Mapping)
            and isinstance(collision, Mapping)
            and odom.get("fresh", False)
            and collision.get("fresh", False)
        )
        stop_state = str(safety.get("stop_state", "UNKNOWN")).upper()
        estop_state = str(safety.get("estop_state", "UNKNOWN")).upper()
        execution_ready = bool(
            self.live_execution_enabled
            and required_fresh
            and str(safety.get("collision_state", "UNKNOWN")).upper() == "CLEAR"
            and stop_state == "READY"
            and estop_state == "CLEAR"
        )

        if mission is None:
            state = WebMissionState.READY.value
            proposal: Mapping[str, Any] = {}
            terminal_reason = ""
            events: Sequence[Mapping[str, Any]] = ()
            result: Mapping[str, Any] = {}
            mission_id = ""
            approval: Mapping[str, Any] = {}
        else:
            state = _web_state(str(mission.get("status", "failed")))
            proposal = mission.get("proposal", {}) if isinstance(mission.get("proposal", {}), Mapping) else {}
            terminal_reason = str(mission.get("terminal_reason", ""))
            events = mission.get("events", ()) if isinstance(mission.get("events", ()), Sequence) else ()
            result = mission.get("result", {}) if isinstance(mission.get("result", {}), Mapping) else {}
            mission_id = str(mission.get("mission_id", ""))
            approval = mission.get("approval", {}) if isinstance(mission.get("approval", {}), Mapping) else {}

        progress_value = 0.0
        if isinstance(route_progress, Mapping):
            route_value = route_progress.get("value", {})
            if isinstance(route_value, Mapping):
                try:
                    progress_value = max(0.0, min(1.0, float(route_value.get("progress", 0.0))))
                except (TypeError, ValueError):
                    progress_value = 0.0
        if state == WebMissionState.COMPLETE.value:
            progress_value = 1.0

        translated_events = []
        for index, event in enumerate(events, start=1):
            if not isinstance(event, Mapping):
                continue
            payload = event.get("payload", {})
            if not isinstance(payload, Mapping):
                payload = {}
            translated_events.append(
                {
                    "sequence": int(event.get("event_id", index)),
                    "event_type": str(event.get("kind", "event")),
                    "message": str(
                        payload.get("reason")
                        or payload.get("status")
                        or payload.get("source")
                        or event.get("kind", "mission event")
                    ),
                }
            )

        return {
            "web_api_version": WEB_API_VERSION,
            "mission_api_version": MissionApiVersion.V2.value,
            "prompt_drive_api_version": PROMPT_DRIVE_API_VERSION,
            "adapter": {
                "mode": self.mode,
                "fixture_only": False,
                "live_execution_enabled": self.live_execution_enabled,
                "direct_ros_commands_allowed": False,
                "credentials_accepted": False,
                "service_source_sha": self._service_snapshot.get("source_sha", ""),
                "service_deployed_sha": self._service_snapshot.get("deployed_sha", ""),
                "boundary": "Pi-local MissionService Unix socket",
            },
            "scenario": LiveScenario.LIVE.value,
            "proposal": dict(proposal) if proposal else None,
            "approval": {
                "required": bool(proposal) and state == WebMissionState.PROPOSED.value,
                "enabled": execution_ready and state == WebMissionState.PROPOSED.value,
                "approved": bool(approval.get("approved", False)),
                "proposal_digest": str(proposal.get("proposal_digest", "")) if proposal else "",
                "required_phrase": "",
                "method": "authenticated_one_click",
                "server_digest_bound": True,
                "simulation_only": False,
            },
            "mission": {
                "mission_id": mission_id,
                "state": state,
                "progress": progress_value,
                "terminal": state in {item.value for item in TERMINAL_STATES},
                "terminal_reason": terminal_reason,
                "result": dict(result),
            },
            "artifacts": _terminal_artifacts(
                result if state in {item.value for item in TERMINAL_STATES} else {},
                fixture_only=False,
            ),
            "safety": {
                "stop_active": bool(safety.get("stop_active", False)),
                "estop_latched": bool(safety.get("estop_latched", False)),
                "stop_state": stop_state,
                "estop_state": estop_state,
                "collision_state": str(safety.get("collision_state", "UNKNOWN")),
                "front_clearance_m": safety.get("front_clearance_m"),
                "forward_corridor_clearance_m": safety.get(
                    "forward_corridor_clearance_m"
                ),
                "forward_corridor_min_angle_deg": safety.get(
                    "forward_corridor_min_angle_deg"
                ),
                "forward_corridor_max_angle_deg": safety.get(
                    "forward_corridor_max_angle_deg"
                ),
                "collision_stop_distance_m": safety.get(
                    "collision_stop_distance_m"
                ),
                "collision_slow_distance_m": safety.get(
                    "collision_slow_distance_m"
                ),
                "telemetry_fresh": required_fresh,
                "independent_robot_safety": True,
                "browser_is_sole_safety_mechanism": False,
            },
            "events": translated_events,
            "map": _authoritative_live_map(live_evidence),
        }


def _web_state(status: str) -> str:
    normalized = str(status).strip().lower()
    mapping = {
        "received": WebMissionState.RECEIVED,
        "planning": WebMissionState.PLANNING,
        "proposed": WebMissionState.PROPOSED,
        "approved": WebMissionState.APPROVED,
        "queued": WebMissionState.QUEUED,
        "running": WebMissionState.RUNNING,
        "cancel_requested": WebMissionState.RUNNING,
        "complete": WebMissionState.COMPLETE,
        "cancelled": WebMissionState.CANCELLED,
        "stopped": WebMissionState.STOPPED,
        "estopped": WebMissionState.ESTOPPED,
        "blocked": WebMissionState.BLOCKED,
        "rejected": WebMissionState.REJECTED,
        "failed": WebMissionState.FAILED,
        "recovery_required": WebMissionState.RECOVERY_REQUIRED,
    }
    return mapping.get(normalized, WebMissionState.FAILED).value


def _terminal_artifacts(
    result: Mapping[str, Any], *, fixture_only: bool
) -> list[dict[str, Any]]:
    if not result:
        return []
    return [
        {
            "artifact_id": "terminal-result",
            "label": "Terminal result (JSON)",
            "href": TERMINAL_ARTIFACT_PATH,
            "media_type": "application/json",
            "fixture_only": bool(fixture_only),
        }
    ]


def _authoritative_live_map(live_evidence: Mapping[str, Any]) -> dict[str, Any]:
    semantic = live_evidence.get("semantic_map", {})
    if isinstance(semantic, Mapping) and semantic.get("fresh") and semantic.get("valid"):
        value = semantic.get("value", {})
        if isinstance(value, Mapping):
            candidate = value.get("map", value)
            required = {"bounds", "rover", "proposed_route", "traveled_path", "obstacles", "objects"}
            if isinstance(candidate, Mapping) and required <= set(candidate):
                result = json.loads(json.dumps(dict(candidate), allow_nan=False))
                result.update({"available": True, "fixture_only": False, "source": "Pi mission service"})
                return result
    odom = live_evidence.get("odom", {})
    odom_value = odom.get("value", {}) if isinstance(odom, Mapping) else {}
    rover = {
        "x_m": float(odom_value.get("x_m", 0.0)) if isinstance(odom_value, Mapping) else 0.0,
        "y_m": float(odom_value.get("y_m", 0.0)) if isinstance(odom_value, Mapping) else 0.0,
        "yaw_deg": float(odom_value.get("heading_deg", 0.0)) if isinstance(odom_value, Mapping) else 0.0,
    }
    return {
        "available": False,
        "unavailable_reason": "authoritative semantic map is missing, invalid, or stale",
        "frame": "unavailable",
        "bounds": {"origin": {"x_m": 0.0, "y_m": 0.0}, "width_m": 1.0, "height_m": 1.0},
        "rover": rover,
        "proposed_route": [],
        "traveled_path": [],
        "obstacles": [],
        "objects": [],
        "fixture_only": False,
    }


def build_mission_web_bundle(*, app_name: str = "RVR Mission Console") -> Mapping[str, Any]:
    """Return the dependency-free responsive UI assets."""

    safe_name = html.escape(app_name, quote=True)
    index_html = _INDEX_HTML.replace("__APP_NAME__", safe_name)
    return {
        "index_html": index_html,
        "manifest": {
            "name": app_name,
            "short_name": "RVR Console",
            "start_url": ".",
            "display": "standalone",
            "background_color": "#07111f",
            "theme_color": "#5de4c7",
            "icons": [],
        },
        "service_worker_js": "self.addEventListener('install', () => self.skipWaiting());\n",
    }


def handle_mission_web_request(
    method: str,
    path: str,
    body: str,
    adapter: MissionWebAdapter,
) -> MissionWebResponse:
    """Dependency-free router used by tests and the local HTTP wrapper."""

    normalized_method = str(method).upper()
    normalized_path = urlsplit(path).path.rstrip("/") or "/"
    bundle = build_mission_web_bundle()
    if normalized_method == "GET":
        if normalized_path == "/":
            return MissionWebResponse(200, "text/html; charset=utf-8", str(bundle["index_html"]))
        if normalized_path == "/manifest.webmanifest":
            return _json_response(200, bundle["manifest"], content_type="application/manifest+json")
        if normalized_path == "/service-worker.js":
            return MissionWebResponse(200, "text/javascript; charset=utf-8", str(bundle["service_worker_js"]))
        if normalized_path == "/favicon.ico":
            return MissionWebResponse(204, "image/x-icon", "")
        if normalized_path == "/api/web/state":
            return _json_response(200, adapter.snapshot())
        if normalized_path == "/api/web/scenarios":
            return _json_response(200, {"scenarios": [item.to_json_dict() for item in adapter.scenarios()]})
        if normalized_path == TERMINAL_ARTIFACT_PATH:
            snapshot = adapter.snapshot()
            mission = snapshot.get("mission", {})
            if not isinstance(mission, Mapping) or not bool(mission.get("terminal", False)):
                raise MissionWebError("terminal mission evidence is not available")
            result = mission.get("result", {})
            if not isinstance(result, Mapping) or not result:
                raise MissionWebError("terminal mission result is not available")
            return _json_response(
                200,
                {
                    "mission_id": str(mission.get("mission_id", "")),
                    "state": str(mission.get("state", "")),
                    "terminal_reason": str(mission.get("terminal_reason", "")),
                    "result": dict(result),
                },
            )
        raise MissionWebError(f"GET route is not exposed: {normalized_path}")
    if normalized_method != "POST":
        raise MissionWebError("mission web adapter only supports GET and bounded mock POST routes")
    if normalized_path in {"/api/motor", "/api/ros", "/api/write", "/cmd_vel", "/cmd_vel_motor"}:
        raise MissionWebError(f"live or direct command route is not exposed: {normalized_path}")
    payload = _json_object(body)
    if normalized_path == "/api/web/mission/propose":
        result = adapter.propose(str(payload.get("prompt", "")), str(payload.get("scenario", "success")))
    elif normalized_path == "/api/web/mission/approve":
        result = adapter.approve(
            str(payload.get("approval_phrase", "")),
            confirm_current_proposal=bool(payload.get("confirm_current_proposal", False)),
        )
    elif normalized_path == "/api/web/mission/advance":
        result = adapter.advance()
    elif normalized_path == "/api/web/mission/cancel":
        result = adapter.cancel()
    else:
        raise MissionWebError(f"POST route is not exposed: {normalized_path}")
    return _json_response(200, result)


def _json_object(body: str) -> Mapping[str, Any]:
    try:
        payload = json.loads(body or "{}")
    except json.JSONDecodeError as exc:
        raise MissionWebError("request body must be valid JSON") from exc
    if not isinstance(payload, Mapping):
        raise MissionWebError("request body must be a JSON object")
    return payload


def _json_response(status: int, payload: Mapping[str, Any], *, content_type: str = "application/json") -> MissionWebResponse:
    return MissionWebResponse(status, content_type, json.dumps(dict(payload), sort_keys=True, separators=(",", ":")))


class _MissionWebHttpHandler(BaseHTTPRequestHandler):
    server_version = "RvrMissionWeb/1.0"

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        self._dispatch("")

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        normalized_path = urlsplit(self.path).path.rstrip("/") or "/"
        if normalized_path in {"/api/motor", "/api/ros", "/api/write", "/cmd_vel", "/cmd_vel_motor"}:
            self._send_error(400, f"live or direct command route is not exposed: {normalized_path}")
            return
        if bool(getattr(self.server, "enforce_same_origin", False)):
            expected_origin = str(getattr(self.server, "allowed_origin", ""))
            if self.headers.get("Origin", "") != expected_origin:
                self._send_error(403, "state-changing request origin is not authorized")
                return
            fetch_site = self.headers.get("Sec-Fetch-Site", "same-origin")
            if fetch_site not in {"same-origin", "none"}:
                self._send_error(403, "cross-site state-changing request is not authorized")
                return
        identity = self.headers.get("Tailscale-User-Login", "").strip()
        if bool(getattr(self.server, "require_tailscale_identity", False)) and not identity:
            self._send_error(401, "an authenticated Tailscale user identity is required")
            return
        self._request_identity = identity
        content_type = self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower()
        if content_type != "application/json":
            self._send_error(415, "state-changing requests require application/json")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._send_error(400, "invalid Content-Length")
            return
        if length < 0 or length > MAX_REQUEST_BYTES:
            self._send_error(413, "request body is too large")
            return
        try:
            body = self.rfile.read(length).decode("utf-8", errors="strict")
        except UnicodeDecodeError:
            self._send_error(400, "request body must be UTF-8")
            return
        self._dispatch(body)

    def _dispatch(self, body: str) -> None:
        adapter = getattr(self.server, "mission_web_adapter")
        set_identity = getattr(adapter, "set_request_identity", None)
        clear_identity = getattr(adapter, "clear_request_identity", None)
        if callable(set_identity) and self.command == "POST":
            set_identity(str(getattr(self, "_request_identity", "")))
        try:
            response = handle_mission_web_request(self.command, self.path, body, adapter)
        except (MissionWebError, UnicodeDecodeError) as exc:
            self._send_error(400, str(exc))
            return
        finally:
            if callable(clear_identity) and self.command == "POST":
                clear_identity()
        self.send_response(response.status)
        self.send_header("Content-Type", response.content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self' 'unsafe-inline'; connect-src 'self'")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        encoded = response.body.encode("utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_error(self, status: int, message: str) -> None:
        response = _json_response(status, {"error": message})
        self.send_response(status)
        self.send_header("Content-Type", response.content_type)
        encoded = response.body.encode("utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def log_message(self, format: str, *args: Any) -> None:
        del format, args


def make_server(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    *,
    adapter: Optional[MissionWebAdapter] = None,
    allowed_origin: Optional[str] = None,
    require_tailscale_identity: bool = False,
) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, int(port)), _MissionWebHttpHandler)
    setattr(server, "mission_web_adapter", adapter or MockReplayMissionAdapter())
    setattr(server, "enforce_same_origin", allowed_origin is not None)
    setattr(server, "allowed_origin", "" if allowed_origin is None else str(allowed_origin).rstrip("/"))
    setattr(server, "require_tailscale_identity", bool(require_tailscale_identity))
    return server


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Serve the loopback-only RVR mission web console.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--mode", choices=("mock", "live"), default="mock")
    parser.add_argument(
        "--mission-socket",
        default="~/.local/state/sphero_rvr/mission-service.sock",
        help="Pi-local MissionService Unix socket used only in live mode",
    )
    parser.add_argument("--session-id", default="rvr-web-console")
    parser.add_argument("--operator", default="tailscale-operator")
    parser.add_argument(
        "--public-origin",
        default=None,
        help="Exact authenticated HTTPS origin required for live POST requests",
    )
    args = parser.parse_args(argv)
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        parser.error("the mission web service may bind only to a loopback host")
    if args.mode == "live":
        if not args.public_origin:
            parser.error("live mode requires --public-origin for same-origin enforcement")
        adapter: MissionWebAdapter = LiveMissionWebAdapter(
            MissionServiceClient(args.mission_socket),
            session_id=args.session_id,
            operator=args.operator,
        )
        allowed_origin = args.public_origin
    else:
        adapter = MockReplayMissionAdapter()
        allowed_origin = None
    server = make_server(
        args.host,
        args.port,
        adapter=adapter,
        allowed_origin=allowed_origin,
        require_tailscale_identity=args.mode == "live",
    )
    print(
        f"RVR {args.mode} web console on loopback: http://{args.host}:{server.server_address[1]} "
        f"(live execution enabled: {adapter.live_execution_enabled})"
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


_INDEX_HTML = r'''<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="theme-color" content="#5de4c7">
  <link rel="manifest" href="/manifest.webmanifest">
  <title>__APP_NAME__</title>
  <style>
    :root { color-scheme: dark; --ink:#ecf5f4; --muted:#91a9aa; --panel:#0e1b2b; --line:#21364a; --teal:#5de4c7; --amber:#ffca6b; --red:#ff7b72; --blue:#78a9ff; font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background:#07111f; color:var(--ink); }
    * { box-sizing:border-box; }
    [hidden] { display:none !important; }
    body { margin:0; min-height:100vh; background:radial-gradient(circle at 15% -10%, #123952 0, transparent 34rem), #07111f; }
    button, textarea, select, input { font:inherit; }
    button { cursor:pointer; }
    button:disabled { cursor:not-allowed; opacity:.45; }
    header { display:flex; justify-content:space-between; gap:1rem; align-items:center; padding:1.1rem clamp(1rem,3vw,2.5rem); border-bottom:1px solid var(--line); background:rgba(7,17,31,.88); position:sticky; top:0; z-index:5; backdrop-filter:blur(12px); }
    h1 { font-size:clamp(1.15rem,2vw,1.55rem); margin:0; letter-spacing:.02em; }
    h2 { margin:0 0 .85rem; font-size:.9rem; color:#b9cccc; text-transform:uppercase; letter-spacing:.12em; }
    p { line-height:1.55; }
    .mode-badge { display:inline-flex; align-items:center; gap:.45rem; padding:.48rem .75rem; border:1px solid #2a7c72; border-radius:99px; color:var(--teal); background:#0b292b; font-size:.78rem; font-weight:800; letter-spacing:.08em; }
    .mode-badge::before { content:""; width:.52rem; height:.52rem; border-radius:50%; background:var(--teal); box-shadow:0 0 .8rem var(--teal); }
    .mode-badge.live { color:var(--amber); border-color:#8f6729; background:#30220d; }
    .mode-badge.live::before { background:var(--amber); box-shadow:0 0 .8rem var(--amber); }
    .mode-badge.execution { color:var(--red); border-color:#8f3d43; background:#32161b; }
    .mode-badge.execution::before { background:var(--red); box-shadow:0 0 .8rem var(--red); }
    .shell { width:min(1500px,100%); max-width:100%; margin:auto; padding:clamp(.8rem,2vw,1.5rem); overflow:hidden; }
    .safety-strip { display:grid; grid-template-columns:repeat(5,1fr); gap:.65rem; margin-bottom:1rem; }
    .safety-cell { padding:.7rem .85rem; border:1px solid var(--line); border-radius:.75rem; background:rgba(14,27,43,.92); }
    .safety-cell span { display:block; color:var(--muted); font-size:.7rem; text-transform:uppercase; letter-spacing:.08em; }
    .safety-cell strong { display:block; margin-top:.28rem; font-size:.92rem; }
    .workspace { display:grid; grid-template-columns:minmax(310px,.85fr) minmax(480px,1.65fr) minmax(290px,.8fr); gap:1rem; align-items:start; min-width:0; }
    .column { display:grid; gap:1rem; min-width:0; }
    .panel { min-width:0; border:1px solid var(--line); border-radius:1rem; padding:1rem; background:linear-gradient(145deg,rgba(17,35,52,.97),rgba(11,24,39,.97)); box-shadow:0 14px 45px rgba(0,0,0,.16); }
    .field-label { display:block; margin:.7rem 0 .35rem; color:#bbcccd; font-size:.8rem; font-weight:700; }
    textarea, select, input { width:100%; color:var(--ink); background:#071421; border:1px solid #294258; border-radius:.7rem; padding:.75rem; outline:none; }
    textarea { resize:vertical; min-height:7rem; }
    textarea:focus, select:focus, input:focus { border-color:var(--teal); box-shadow:0 0 0 3px rgba(93,228,199,.1); }
    .actions { display:flex; gap:.55rem; flex-wrap:wrap; margin-top:.8rem; }
    .primary, .secondary, .danger { border-radius:.7rem; border:1px solid transparent; padding:.65rem .9rem; font-weight:800; }
    .primary { color:#041a18; background:var(--teal); }
    .secondary { color:var(--ink); background:#182a3d; border-color:#30485e; }
    .danger { color:#ffd9d6; background:#3b1d25; border-color:#7e3541; }
    .hint, .empty { color:var(--muted); font-size:.83rem; }
    .digest { display:block; overflow-wrap:anywhere; padding:.6rem; border-radius:.55rem; background:#071421; color:var(--amber); font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:.72rem; }
    .plan-meta { display:grid; grid-template-columns:1fr 1fr; gap:.6rem; }
    .meta { padding:.6rem; background:#0a1726; border-radius:.65rem; }
    .meta span { display:block; color:var(--muted); font-size:.68rem; text-transform:uppercase; }
    .meta strong { display:block; margin-top:.2rem; overflow-wrap:anywhere; font-size:.82rem; }
    .segments { display:grid; gap:.5rem; margin:.8rem 0; }
    .segment { display:flex; justify-content:space-between; gap:.8rem; min-width:0; padding:.65rem .7rem; border-left:3px solid var(--blue); background:#0a1726; border-radius:.3rem .65rem .65rem .3rem; }
    .segment code { color:#bdd2ff; }
    .segment strong { min-width:0; overflow-wrap:anywhere; text-align:right; }
    .limits { display:flex; gap:.4rem; flex-wrap:wrap; }
    .chip { border:1px solid #314a60; color:#b7cbd0; border-radius:99px; padding:.28rem .5rem; font-size:.68rem; }
    .map-frame { position:relative; aspect-ratio:3/2; min-height:360px; border:1px solid #29465a; overflow:hidden; border-radius:.8rem; background:#08131e; }
    #mission-map { width:100%; height:100%; display:block; }
    .legend { display:flex; flex-wrap:wrap; gap:.8rem; margin-top:.65rem; color:var(--muted); font-size:.72rem; }
    .legend i { width:.7rem; height:.7rem; border-radius:50%; display:inline-block; margin-right:.28rem; vertical-align:-.05rem; }
    .status-line { display:flex; justify-content:space-between; align-items:center; gap:.7rem; }
    .state { font-size:1.25rem; color:var(--teal); font-weight:900; letter-spacing:.04em; }
    .terminal { color:var(--amber); font-size:.78rem; }
    .result-json { max-height:18rem; overflow:auto; white-space:pre-wrap; overflow-wrap:anywhere; padding:.65rem; border-radius:.55rem; background:#071421; color:#bdd2ff; font-size:.7rem; }
    .artifact-list { display:grid; gap:.45rem; margin:.7rem 0 0; padding:0; list-style:none; }
    .artifact-list a { color:var(--teal); font-weight:800; }
    progress { width:100%; height:.65rem; accent-color:var(--teal); }
    .event-list { list-style:none; margin:0; padding:0; display:grid; gap:.65rem; max-height:25rem; overflow:auto; }
    .event-list li { position:relative; padding-left:1.2rem; color:#cad7d7; font-size:.8rem; line-height:1.35; }
    .event-list li::before { content:""; position:absolute; left:.1rem; top:.35rem; width:.45rem; height:.45rem; border-radius:50%; background:var(--blue); }
    .event-list small { display:block; color:var(--muted); margin-bottom:.12rem; }
    .error { min-height:1.2rem; color:#ffaba5; font-size:.8rem; margin-top:.5rem; }
    .rejected { border-color:#7e3541; color:#ffd3d0; }
    @media (max-width:1100px) { .workspace { grid-template-columns:minmax(300px,.85fr) minmax(430px,1.3fr); } .workspace > .column:last-child { grid-column:1/-1; grid-template-columns:1fr 1fr; } }
    @media (max-width:760px) { header { position:static; align-items:flex-start; flex-direction:column; } .shell { padding:.7rem; } .safety-strip { grid-template-columns:1fr 1fr; } .workspace, .workspace > .column:last-child { grid-template-columns:minmax(0,1fr); } .workspace > .column:last-child { grid-column:auto; } .map-frame { min-height:0; } .plan-meta { grid-template-columns:1fr; } .segment { flex-direction:column; gap:.35rem; } .segment strong { text-align:left; font-size:.76rem; } }
  </style>
</head>
<body>
  <header>
    <div><h1>__APP_NAME__</h1><div class="hint">Map-driven mission planning preview</div></div>
    <div class="mode-badge" data-testid="mode-badge">MOCK / REPLAY — NO LIVE EXECUTION</div>
  </header>
  <main class="shell">
    <section class="safety-strip" aria-label="Safety state">
      <div class="safety-cell"><span>Collision</span><strong id="safety-collision">CLEAR</strong></div>
      <div class="safety-cell"><span>Forward corridor</span><strong id="safety-corridor">UNAVAILABLE</strong></div>
      <div class="safety-cell"><span>Telemetry</span><strong id="safety-telemetry">FRESH</strong></div>
      <div class="safety-cell"><span>STOP</span><strong id="safety-stop">READY</strong></div>
      <div class="safety-cell"><span>ESTOP</span><strong id="safety-estop">CLEAR</strong></div>
    </section>
    <div class="workspace">
      <div class="column">
        <section class="panel" aria-labelledby="mission-heading">
          <h2 id="mission-heading">Mission prompt</h2>
          <label class="field-label" for="mission-prompt">Tell the rover what to do</label>
          <textarea id="mission-prompt" data-testid="mission-prompt">Move forward 20 centimeters, turn left 45 degrees, then move forward 15 centimeters.</textarea>
          <label class="field-label" for="scenario" id="scenario-label">Replay outcome</label>
          <select id="scenario" data-testid="scenario"></select>
          <div class="actions"><button class="primary" id="propose" data-testid="propose">Generate proposal</button></div>
          <div class="error" id="request-error" role="alert"></div>
        </section>
        <section class="panel" aria-labelledby="approval-heading">
          <h2 id="approval-heading">Simulation approval</h2>
          <p class="hint" id="approval-hint">Approval is digest-bound and authorizes only the mock adapter.</p>
          <label class="field-label" for="approval-input" id="approval-input-label">Type the exact phrase shown with the proposal</label>
          <input id="approval-input" data-testid="approval-input" autocomplete="off" disabled>
          <div class="actions">
            <button class="primary" id="approve" data-testid="approve" disabled>Approve simulation</button>
            <button class="danger" id="cancel" data-testid="cancel" disabled>Cancel mission</button>
          </div>
        </section>
      </div>
      <div class="column">
        <section class="panel" aria-labelledby="map-heading">
          <h2 id="map-heading">Room map</h2>
          <div class="map-frame"><svg id="mission-map" role="img" aria-label="Fixture room map showing rover, route, path, obstacles, and objects"></svg></div>
          <div class="legend"><span><i style="background:#78a9ff"></i>Proposed route</span><span><i style="background:#5de4c7"></i>Traveled path</span><span><i style="background:#ffca6b"></i>Objects</span><span><i style="background:#64788c"></i>Obstacles</span></div>
        </section>
        <section class="panel" aria-labelledby="proposal-heading">
          <h2 id="proposal-heading">LLM proposal</h2>
          <div id="proposal-view" class="empty">Submit a mission to see a typed route proposal or rejection.</div>
        </section>
      </div>
      <div class="column">
        <section class="panel" aria-labelledby="status-heading">
          <h2 id="status-heading">Mission status</h2>
          <div class="status-line"><div class="state" id="mission-state" data-testid="mission-state">READY</div><div class="terminal" id="terminal-reason"></div></div>
          <progress id="mission-progress" max="100" value="0"></progress>
          <div class="hint" id="progress-label">0% complete</div>
        </section>
        <section class="panel" aria-labelledby="events-heading">
          <h2 id="events-heading">Event history</h2>
          <ol class="event-list" id="event-list"><li class="empty">No mission events yet.</li></ol>
        </section>
        <section class="panel" aria-labelledby="result-heading">
          <h2 id="result-heading">Terminal evidence</h2>
          <div id="result-view" class="empty" data-testid="result-view">No terminal evidence yet.</div>
          <ul id="artifact-list" class="artifact-list"></ul>
        </section>
        <section class="panel">
          <h2>Authority boundary</h2>
          <p class="hint" id="authority-copy">The browser uses a typed mock/replay adapter. Planning, approval authority, and any future execution remain server-side on the Pi. Independent robot safety is never replaced by this page.</p>
        </section>
      </div>
    </div>
  </main>
  <script>
    const $ = (id) => document.getElementById(id);
    let current = null;
    let timer = null;
    let hydratedMissionId = null;
    let promptDirty = false;

    async function api(path, options = {}) {
      const response = await fetch(path, {headers:{'Content-Type':'application/json'}, ...options});
      const payload = await response.json();
      if (!response.ok) throw new Error(payload.error || `Request failed: ${response.status}`);
      return payload;
    }

    function escapeHtml(value) {
      return String(value).replace(/[&<>'"]/g, (char) => ({'&':'&amp;','<':'&lt;','>':'&gt;',"'":'&#39;','"':'&quot;'}[char]));
    }

    function render(snapshot) {
      current = snapshot;
      const proposal = snapshot.proposal;
      const missionId = snapshot.mission.mission_id || null;
      if (proposal && missionId !== hydratedMissionId) {
        if (!promptDirty) $('mission-prompt').value = proposal.prompt || '';
        hydratedMissionId = missionId;
      }
      const live = !snapshot.adapter.fixture_only;
      const execution = live && snapshot.adapter.live_execution_enabled;
      const badge = document.querySelector('[data-testid="mode-badge"]');
      badge.className = `mode-badge${live ? ' live' : ''}${execution ? ' execution' : ''}`;
      badge.textContent = live ? (execution ? 'LIVE — PHYSICAL EXECUTION ENABLED' : 'LIVE — PROPOSAL ONLY / EXECUTION LOCKED') : 'MOCK / REPLAY — NO LIVE EXECUTION';
      $('scenario-label').textContent = live ? 'Service target' : 'Replay outcome';
      $('approval-heading').textContent = live ? 'Run confirmation' : 'Simulation approval';
      $('approval-hint').textContent = live ? (execution ? 'Review the current route, then click once to run it. No code or hash entry is required.' : 'Physical execution is locked by the deployed Pi configuration.') : 'Approval is digest-bound and authorizes only the mock adapter.';
      $('authority-copy').textContent = live ? 'The browser uses the Pi-local mission-service boundary. Planning, OAuth, persistence, approval authority, and any physical execution remain on the Pi. Independent robot safety is never replaced by this page.' : 'The browser uses a typed mock/replay adapter. Planning, approval authority, and any future execution remain server-side on the Pi. Independent robot safety is never replaced by this page.';
      $('approve').textContent = live ? 'Approve and run' : 'Approve simulation';
      $('mission-state').textContent = snapshot.mission.state;
      $('terminal-reason').textContent = snapshot.mission.terminal_reason || '';
      $('mission-progress').value = Math.round(snapshot.mission.progress * 100);
      $('progress-label').textContent = `${Math.round(snapshot.mission.progress * 100)}% complete`;
      $('safety-collision').textContent = snapshot.safety.collision_state;
      const corridorClearance = snapshot.safety.forward_corridor_clearance_m == null
        ? Number.NaN : Number(snapshot.safety.forward_corridor_clearance_m);
      const corridorMin = snapshot.safety.forward_corridor_min_angle_deg == null
        ? Number.NaN : Number(snapshot.safety.forward_corridor_min_angle_deg);
      const corridorMax = snapshot.safety.forward_corridor_max_angle_deg == null
        ? Number.NaN : Number(snapshot.safety.forward_corridor_max_angle_deg);
      $('safety-corridor').textContent = Number.isFinite(corridorClearance)
        && Number.isFinite(corridorMin) && Number.isFinite(corridorMax)
        ? `${corridorClearance.toFixed(2)} m (${corridorMin.toFixed(0)}°…${corridorMax.toFixed(0)}°)`
        : 'UNAVAILABLE';
      $('safety-telemetry').textContent = snapshot.safety.telemetry_fresh ? 'FRESH' : 'STALE — BLOCKED';
      $('safety-stop').textContent = snapshot.safety.stop_state || (snapshot.safety.stop_active ? 'ACTIVE' : 'READY');
      $('safety-estop').textContent = snapshot.safety.estop_state || (snapshot.safety.estop_latched ? 'LATCHED' : 'CLEAR');
      $('approve').disabled = snapshot.mission.state !== 'PROPOSED' || !snapshot.approval.enabled;
      $('approval-input').hidden = live;
      $('approval-input-label').hidden = live;
      $('approval-input').disabled = live || snapshot.mission.state !== 'PROPOSED' || !snapshot.approval.enabled;
      $('cancel').disabled = !['RECEIVED','PLANNING','PROPOSED','APPROVED','QUEUED','RUNNING'].includes(snapshot.mission.state);
      if (proposal) {
        const segments = proposal.segments.map((segment, index) => `<div class="segment"><span>${index + 1}. <code>${escapeHtml(segment.tool_id)}</code></span><strong>${escapeHtml(JSON.stringify(segment.arguments))}</strong></div>`).join('');
        const limits = Object.entries(proposal.limits).map(([key,value]) => `<span class="chip">${escapeHtml(key)}: ${escapeHtml(value)}</span>`).join('');
        const decisionClass = proposal.decision === 'reject' ? ' rejected' : '';
        $('proposal-view').className = decisionClass;
        const approvalAudit = live ? `<p class="hint">The Pi records the exact route you confirmed.</p><details><summary>Technical approval audit</summary><code class="digest">${escapeHtml(proposal.proposal_digest)}</code></details>` : `<p class="field-label">Approval digest</p><code class="digest">${escapeHtml(proposal.proposal_digest)}</code><p class="field-label">Required phrase</p><code class="digest">${escapeHtml(snapshot.approval.required_phrase || 'Not approvable')}</code>`;
        $('proposal-view').innerHTML = `<div class="plan-meta"><div class="meta"><span>Decision</span><strong>${escapeHtml(proposal.decision)}</strong></div><div class="meta"><span>Model</span><strong>${escapeHtml(proposal.provider_id)}/${escapeHtml(proposal.model_id)}</strong></div></div><p>${escapeHtml(proposal.summary)}</p>${segments}<div class="limits">${limits}</div>${approvalAudit}`;
      } else {
        $('proposal-view').className = 'empty';
        $('proposal-view').textContent = 'Submit a mission to see a typed route proposal or rejection.';
      }
      const events = snapshot.events || [];
      $('event-list').innerHTML = events.length ? events.map((event) => `<li><small>#${event.sequence} · ${escapeHtml(event.event_type)}</small>${escapeHtml(event.message)}</li>`).join('') : '<li class="empty">No mission events yet.</li>';
      const result = snapshot.mission.result || {};
      const artifacts = snapshot.artifacts || [];
      if (Object.keys(result).length) {
        $('result-view').className = '';
        $('result-view').innerHTML = `<pre class="result-json">${escapeHtml(JSON.stringify(result, null, 2))}</pre>`;
      } else {
        $('result-view').className = 'empty';
        $('result-view').textContent = 'No terminal evidence yet.';
      }
      $('artifact-list').innerHTML = artifacts.map((artifact) => `<li><a href="${escapeHtml(artifact.href)}" target="_blank" rel="noopener">${escapeHtml(artifact.label)}</a> <span class="hint">${escapeHtml(artifact.media_type)}</span></li>`).join('');
      renderMap(snapshot.map);
      if (snapshot.mission.terminal && timer) { clearInterval(timer); timer = null; }
    }

    function renderMap(map) {
      const svg = $('mission-map');
      const W = 900, H = 600, pad = 42;
      const sx = (x) => pad + (x / map.bounds.width_m) * (W - pad * 2);
      const sy = (y) => H - pad - (y / map.bounds.height_m) * (H - pad * 2);
      const points = (items) => items.map((p) => `${sx(p.x_m)},${sy(p.y_m)}`).join(' ');
      const grid = Array.from({length:11}, (_,i) => `<line x1="${pad + i*(W-pad*2)/10}" y1="${pad}" x2="${pad + i*(W-pad*2)/10}" y2="${H-pad}" stroke="#14283a"/><line x1="${pad}" y1="${pad + i*(H-pad*2)/10}" x2="${W-pad}" y2="${pad + i*(H-pad*2)/10}" stroke="#14283a"/>`).join('');
      const obstacles = map.obstacles.map((o) => `<g><rect x="${sx(o.x_m)}" y="${sy(o.y_m + o.height_m)}" width="${o.width_m/map.bounds.width_m*(W-pad*2)}" height="${o.height_m/map.bounds.height_m*(H-pad*2)}" rx="8" fill="#42576b" stroke="#6f8497"/><text x="${sx(o.x_m)+8}" y="${sy(o.y_m + o.height_m)+20}" fill="#b8c8d4" font-size="14">${escapeHtml(o.label)}</text></g>`).join('');
      const objects = map.objects.map((o) => `<g><circle cx="${sx(o.x_m)}" cy="${sy(o.y_m)}" r="10" fill="#ffca6b"/><circle cx="${sx(o.x_m)}" cy="${sy(o.y_m)}" r="18" fill="none" stroke="#ffca6b" opacity=".45"/><text x="${sx(o.x_m)+15}" y="${sy(o.y_m)-10}" fill="#ffdf9d" font-size="15">${escapeHtml(o.label)} ${Math.round(o.confidence*100)}%</text></g>`).join('');
      svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
      const frame = `${grid}<rect x="${pad}" y="${pad}" width="${W-pad*2}" height="${H-pad*2}" fill="none" stroke="#385168" stroke-width="3"/>`;
      if (map.available === false) {
        svg.setAttribute('aria-label', 'Authoritative live room map unavailable');
        svg.innerHTML = `${frame}<text x="${W/2}" y="${H/2}" text-anchor="middle" fill="#91a9aa" font-size="20">${escapeHtml(map.unavailable_reason || 'Authoritative map unavailable')}</text>`;
        return;
      }
      svg.setAttribute('aria-label', 'Room map showing authoritative rover, route, path, obstacles, and objects');
      svg.innerHTML = `${frame}${obstacles}<polyline points="${points(map.proposed_route)}" fill="none" stroke="#78a9ff" stroke-width="5" stroke-dasharray="10 10"/>${map.traveled_path.length > 1 ? `<polyline points="${points(map.traveled_path)}" fill="none" stroke="#5de4c7" stroke-width="8" stroke-linecap="round"/>` : ''}${objects}<g transform="translate(${sx(map.rover.x_m)} ${sy(map.rover.y_m)}) rotate(${map.rover.yaw_deg})"><path d="M 18 0 L -12 -12 L -7 0 L -12 12 Z" fill="#5de4c7" stroke="#d4fff7" stroke-width="2"/></g>`;
    }

    async function loadScenarios() {
      const payload = await api('/api/web/scenarios');
      $('scenario').innerHTML = payload.scenarios.map((item) => `<option value="${escapeHtml(item.id)}">${escapeHtml(item.label)} — ${escapeHtml(item.description)}</option>`).join('');
    }

    async function propose() {
      stopTimer();
      $('request-error').textContent = '';
      try {
        const snapshot = await api('/api/web/mission/propose', {method:'POST', body:JSON.stringify({prompt:$('mission-prompt').value, scenario:$('scenario').value})});
        $('approval-input').value = '';
        promptDirty = false;
        render(snapshot);
        if (!snapshot.adapter.fixture_only && !snapshot.mission.terminal) startTimer();
      } catch (error) { $('request-error').textContent = error.message; }
    }

    async function approve() {
      $('request-error').textContent = '';
      try {
        const body = current && !current.adapter.fixture_only
          ? {confirm_current_proposal:true}
          : {approval_phrase:$('approval-input').value};
        const snapshot = await api('/api/web/mission/approve', {method:'POST', body:JSON.stringify(body)});
        render(snapshot);
        startTimer();
      } catch (error) { $('request-error').textContent = error.message; }
    }

    async function cancelMission() {
      stopTimer();
      try { render(await api('/api/web/mission/cancel', {method:'POST', body:'{}'})); }
      catch (error) { $('request-error').textContent = error.message; }
    }

    function startTimer() {
      stopTimer();
      timer = setInterval(async () => {
        try { render(current && current.adapter.fixture_only ? await api('/api/web/mission/advance', {method:'POST', body:'{}'}) : await api('/api/web/state')); }
        catch (error) { stopTimer(); $('request-error').textContent = error.message; }
      }, 650);
    }
    function stopTimer() { if (timer) clearInterval(timer); timer = null; }

    $('propose').addEventListener('click', propose);
    $('approve').addEventListener('click', approve);
    $('cancel').addEventListener('click', cancelMission);
    $('mission-prompt').addEventListener('input', () => { promptDirty = true; });
    Promise.all([loadScenarios(), api('/api/web/state')]).then(([,snapshot]) => render(snapshot)).catch((error) => $('request-error').textContent = error.message);
  </script>
</body>
</html>
'''


if __name__ == "__main__":
    raise SystemExit(main())
