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
import os
from pathlib import Path
import subprocess
import threading
import uuid
from dataclasses import asdict, dataclass, field
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Mapping, Optional, Protocol, Sequence
from urllib.parse import urlsplit

from .mission_api import MissionApiVersion, MissionValidationError
from .mission_service import MissionService
from .mission_service_client import MissionServiceClient
from .prompt_drive import (
    ALLOWED_REASONING_EFFORTS,
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
from .rolling_replay import (
    ROLLING_PROPOSAL_SCHEMA,
    CodexOAuthRollingIntentProvider,
    RollingIntentProvider,
    RollingReplayEngine,
    canonical_digest,
)

WEB_API_VERSION = "rvr_mission_web.v1"
MAX_REQUEST_BYTES = 64 * 1024
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
TERMINAL_ARTIFACT_PATH = "/api/web/artifacts/terminal-result"
STATIONARY_SENSOR_CONTROL_PATH = "/api/web/stationary-sensors"
STATIONARY_SENSOR_UNIT = "rvr-stationary-perception.service"


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


class RollingReplayScenario(str, Enum):
    LLM_DRIVING = "rolling_llm_replay"


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

ROLLING_REPLAY_SCENARIOS: tuple[ScenarioDefinition, ...] = (
    ScenarioDefinition(
        RollingReplayScenario.LLM_DRIVING,
        "Rolling LLM driving replay",
        "Continuous pose, perception, tracking, and mapping while real OAuth LLM revisions run asynchronously.",
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


class StationarySensorControl(Protocol):
    """Bounded control for the fixed no-motion stationary perception unit."""

    def status(self) -> Mapping[str, Any]: ...

    def set_active(
        self,
        active: bool,
        *,
        authority_snapshot: Mapping[str, Any],
    ) -> Mapping[str, Any]: ...


class SystemdStationarySensorControl:
    """Start or stop only the installed Stage C stationary sensor unit."""

    def __init__(self, unit: str = STATIONARY_SENSOR_UNIT) -> None:
        if str(unit).strip() != STATIONARY_SENSOR_UNIT:
            raise MissionWebError("only the stationary perception service may be controlled")
        self.unit = STATIONARY_SENSOR_UNIT
        self._lock = threading.RLock()

    def status(self) -> Mapping[str, Any]:
        with self._lock:
            try:
                completed = subprocess.run(
                    [
                        "systemctl",
                        "--user",
                        "show",
                        "--property=LoadState",
                        "--property=ActiveState",
                        "--property=SubState",
                        self.unit,
                    ],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=3.0,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                return {
                    "available": False,
                    "active": False,
                    "transitioning": False,
                    "state": "unavailable",
                    "detail": f"Unable to query stationary sensors: {exc}",
                    "unit": self.unit,
                }
            properties = {}
            for line in str(completed.stdout).splitlines():
                key, separator, value = line.partition("=")
                if separator:
                    properties[key.strip()] = value.strip()
            active_state = properties.get("ActiveState", "unknown").lower()
            sub_state = properties.get("SubState", "unknown").lower()
            loaded = properties.get("LoadState", "not-found").lower() == "loaded"
            available = completed.returncode == 0 and loaded
            detail = (
                f"{active_state} / {sub_state}"
                if available
                else str(completed.stderr).strip() or "Stationary sensor service is not installed."
            )
            return {
                "available": available,
                "active": active_state == "active",
                "transitioning": active_state in {"activating", "deactivating", "reloading"},
                "state": active_state if available else "unavailable",
                "detail": detail,
                "unit": self.unit,
            }

    def set_active(
        self,
        active: bool,
        *,
        authority_snapshot: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        with self._lock:
            if active and not _stationary_sensor_start_permitted(authority_snapshot):
                raise MissionWebError(
                    "stationary sensors may start only while physical execution and motion authority are disabled"
                )
            if active:
                _assert_stationary_no_motion_runtime()
            action = "start" if active else "stop"
            try:
                completed = subprocess.run(
                    ["systemctl", "--user", action, self.unit],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=20.0,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise MissionWebError(f"unable to {action} stationary sensors: {exc}") from exc
            if completed.returncode != 0:
                detail = str(completed.stderr).strip() or str(completed.stdout).strip()
                raise MissionWebError(
                    f"unable to {action} stationary sensors: {detail or 'systemd request failed'}"
                )
            status = dict(self.status())
            if bool(status.get("active", False)) != active:
                raise MissionWebError(
                    f"stationary sensors did not reach the requested {'active' if active else 'inactive'} state"
                )
            return status


def _stationary_sensor_start_permitted(snapshot: Mapping[str, Any]) -> bool:
    adapter = snapshot.get("adapter", {})
    if not isinstance(adapter, Mapping):
        return False
    return bool(
        not adapter.get("fixture_only", True)
        and adapter.get("stationary_perception", False)
        and not adapter.get("live_execution_enabled", False)
        and adapter.get("motion_authority") is False
        and adapter.get("physical_execution_enabled") is False
    )


def _assert_stationary_no_motion_runtime() -> None:
    """Reject fixed-odom sensing when a motor-capable runtime may be present."""

    try:
        processes = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise MissionWebError(
            f"unable to verify the no-motion process boundary: {exc}"
        ) from exc
    if processes.returncode != 0:
        detail = str(processes.stderr).strip() or "process inspection failed"
        raise MissionWebError(f"unable to verify the no-motion process boundary: {detail}")
    forbidden = (
        "rvr_node",
        "live_route_runner",
        "range_motion_controller",
        "lidar_collision_stop_supervisor",
        "supervised_rvr.launch.py",
        "rvr.launch.py",
        "rvr-console",
        "/cmd_vel",
        "/cmd_vel_motor",
    )
    conflicts = [
        line.strip()
        for line in str(processes.stdout).splitlines()
        if any(token in line for token in forbidden)
    ]
    if conflicts:
        raise MissionWebError(
            "stationary sensors refused because a driver, route, or motion process is present"
        )
    try:
        serial_owner = subprocess.run(
            ["fuser", "/dev/ttyAMA0"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise MissionWebError(f"unable to verify rover serial ownership: {exc}") from exc
    if serial_owner.returncode == 0:
        raise MissionWebError(
            "stationary sensors refused because the rover serial device has an owner"
        )
    if serial_owner.returncode not in {1}:
        detail = str(serial_owner.stderr).strip() or "serial ownership check failed"
        raise MissionWebError(f"unable to verify rover serial ownership: {detail}")


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
            "left_clearance_m": None,
            "right_clearance_m": None,
            "trajectory_clearance_margin_m": None,
            "trajectory_horizon_s": None,
            "trajectory_min_clearance_m": None,
            "trajectory_collision_time_s": None,
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


class RollingReplayMissionAdapter:
    """Persistent replay-only web adapter for the Stage B vertical slice."""

    mode = "rolling-llm-replay"
    live_execution_enabled = False
    direct_ros_commands_allowed = False
    credentials_accepted = False

    def __init__(
        self,
        provider: RollingIntentProvider,
        *,
        database: str | Path = ":memory:",
        source_sha: str = "rolling-replay-local",
        tick_s: float = 0.1,
        session_id: str = "rolling-replay-web",
    ) -> None:
        self.provider = provider
        self.source_sha = str(source_sha).strip()
        self.tick_s = float(tick_s)
        self.session_id = str(session_id).strip()
        if not self.source_sha or not self.session_id:
            raise MissionWebError(
                "rolling replay source SHA and session ID are required"
            )
        self._lock = threading.RLock()
        self._service = MissionService(
            database,
            source_sha=self.source_sha,
            deployed_sha=self.source_sha,
            mode="replay",
            live_execution_enabled=False,
        )
        self._mission_id: Optional[str] = None
        self._engine: Optional[RollingReplayEngine] = None
        self._proposal: Optional[dict[str, Any]] = None

    def close(self) -> None:
        with self._lock:
            if self._engine is not None:
                self._engine.close()
                self._engine = None
            self._service.close()

    def scenarios(self) -> Sequence[ScenarioDefinition]:
        return ROLLING_REPLAY_SCENARIOS

    def snapshot(self) -> Mapping[str, Any]:
        with self._lock:
            return self._snapshot_unlocked()

    def propose(self, prompt: str, scenario: str) -> Mapping[str, Any]:
        if str(scenario) != RollingReplayScenario.LLM_DRIVING.value:
            raise MissionWebError("rolling replay accepts only its LLM-driving scenario")
        objective = str(prompt).strip()
        if not objective:
            raise MissionWebError("mission prompt is required")
        with self._lock:
            if self._engine is not None:
                self._engine.close()
            mission_id = f"rolling-replay-{uuid.uuid4().hex}"
            self._mission_id = mission_id
            self._service.begin_prompt_mission(
                mission_id=mission_id,
                session_id=self.session_id,
                prompt=objective,
                source="web",
            )
            proposal_body = {
                "schema": ROLLING_PROPOSAL_SCHEMA,
                "mission_id": mission_id,
                "prompt": objective,
                "source_sha": self.source_sha,
                "provider_id": self.provider.provider_id,
                "model_id": self.provider.model_id,
                "reasoning_effort": self.provider.reasoning_effort,
                "contract": {
                    "output": "one rolling finite leased intent per fresh snapshot",
                    "fixed_route": False,
                    "asynchronous": True,
                    "motion_authority": False,
                    "physical_execution_enabled": False,
                },
                "decision": "propose",
                "summary": (
                    "Run one continuous no-authority replay whose steering and "
                    "observation intent is repeatedly revised by the authenticated LLM."
                ),
                "segments": [],
                "limits": {
                    "max_speed_mps": 0.18,
                    "lease_s": "8–90",
                    "max_provider_calls": 8,
                    "physical_execution": False,
                },
                "prompt_drive_api_version": PROMPT_DRIVE_API_VERSION,
            }
            self._proposal = {
                **proposal_body,
                "proposal_digest": canonical_digest(proposal_body),
            }
            self._service.record_rolling_replay_proposal(
                mission_id, self._proposal
            )
            self._engine = RollingReplayEngine(
                mission_id,
                objective,
                self.provider,
                tick_s=self.tick_s,
                checkpoint=self._persist_checkpoint,
            )
            return self._snapshot_unlocked()

    def approve(
        self,
        supplied_approval: str,
        *,
        confirm_current_proposal: bool = False,
    ) -> Mapping[str, Any]:
        del confirm_current_proposal
        with self._lock:
            if (
                self._mission_id is None
                or self._proposal is None
                or self._engine is None
            ):
                raise MissionWebError(
                    "a rolling replay proposal is required before approval"
                )
            expected = self._approval_phrase()
            if str(supplied_approval).strip() != expected:
                raise MissionWebError(
                    "rolling replay approval phrase does not match the current proposal"
                )
            self._service.approve_rolling_replay_mission(
                self._mission_id,
                proposal_digest=str(self._proposal["proposal_digest"]),
                operator="browser-replay-operator",
            )
            self._engine.start()
            return self._snapshot_unlocked()

    def advance(self) -> Mapping[str, Any]:
        return self.snapshot()

    def cancel(self) -> Mapping[str, Any]:
        with self._lock:
            if self._engine is None or self._mission_id is None:
                raise MissionWebError("a rolling replay mission is required")
            status = str(
                self._service.prompt_status(self._mission_id).get("status", "")
            )
            if status == "running":
                self._engine.cancel()
            elif status == "proposed":
                self._service.cancel_prompt_mission(
                    self._mission_id, reason="browser replay operator cancelled mission"
                )
            else:
                raise MissionWebError(
                    "only a proposed or running rolling replay can be cancelled"
                )
            return self._snapshot_unlocked()

    def _persist_checkpoint(
        self, kind: str, projection: Mapping[str, Any]
    ) -> None:
        mission_id = self._mission_id
        if mission_id is None:
            raise MissionWebError("rolling replay mission identity is unavailable")
        if str(kind) == "terminal":
            result = projection.get("result", {})
            if not isinstance(result, Mapping):
                raise MissionWebError("rolling replay terminal result is unavailable")
            self._service.finish_rolling_replay_mission(
                mission_id,
                status=str(projection.get("status", "failed")),
                reason=str(projection.get("terminal_reason", "")),
                result=result,
            )
            return
        checkpoint = {
            "schema": projection.get("schema"),
            "mission_id": mission_id,
            "status": projection.get("status"),
            "world_snapshot": projection.get("world_snapshot"),
            "active_intent": projection.get("active_intent"),
            "intent_revisions": projection.get("intent_revisions"),
            "inference": projection.get("inference"),
            "metrics": projection.get("metrics"),
            "motion_authority": False,
            "physical_execution_enabled": False,
        }
        self._service.record_rolling_replay_checkpoint(
            mission_id, kind=str(kind), checkpoint=checkpoint
        )

    def _approval_phrase(self) -> str:
        if self._proposal is None:
            return ""
        return (
            "APPROVE ROLLING REPLAY "
            f"{str(self._proposal['proposal_digest'])}"
        )

    def _snapshot_unlocked(self) -> dict[str, Any]:
        mission = (
            None
            if self._mission_id is None
            else self._service.prompt_status(self._mission_id)
        )
        projection = None if self._engine is None else self._engine.snapshot()
        status = "ready" if mission is None else str(mission["status"])
        state = _web_state(status)
        if projection is not None and projection.get("terminal"):
            state = _web_state(str(projection.get("status", "failed")))
        terminal = state in {item.value for item in TERMINAL_STATES}
        result = (
            projection.get("result", {})
            if projection is not None and terminal
            else {}
        )
        if not isinstance(result, Mapping):
            result = {}
        approval = {} if mission is None else mission.get("approval", {})
        if not isinstance(approval, Mapping):
            approval = {}
        rolling_events = [] if projection is None else projection.get("events", [])
        map_payload = (
            MapFixture().to_json_dict(progress=0.0)
            if projection is None
            else projection["map"]
        )
        world = {} if projection is None else projection["world_snapshot"]
        obstacles = world.get("obstacles", {}) if isinstance(world, Mapping) else {}
        localization = (
            world.get("localization", {}) if isinstance(world, Mapping) else {}
        )
        replay_safety = (
            world.get("safety", {}) if isinstance(world, Mapping) else {}
        )
        return {
            "web_api_version": WEB_API_VERSION,
            "mission_api_version": MissionApiVersion.V2.value,
            "prompt_drive_api_version": PROMPT_DRIVE_API_VERSION,
            "adapter": {
                "mode": self.mode,
                "fixture_only": True,
                "rolling_replay": True,
                "real_llm_provider": self.provider.provider_id
                == "openai-codex-oauth",
                "provider_id": self.provider.provider_id,
                "model_id": self.provider.model_id,
                "reasoning_effort": self.provider.reasoning_effort,
                "live_execution_enabled": False,
                "direct_ros_commands_allowed": False,
                "credentials_accepted": False,
                "mission_service_persistence": True,
                "source_sha": self.source_sha,
            },
            "scenario": RollingReplayScenario.LLM_DRIVING.value,
            "proposal": self._proposal,
            "approval": {
                "required": self._proposal is not None,
                "enabled": state == WebMissionState.PROPOSED.value,
                "approved": bool(approval.get("approved", False)),
                "proposal_digest": (
                    "" if self._proposal is None else self._proposal["proposal_digest"]
                ),
                "required_phrase": self._approval_phrase(),
                "simulation_only": True,
            },
            "mission": {
                "mission_id": "" if mission is None else mission["mission_id"],
                "state": state,
                "progress": 0.0
                if projection is None
                else float(projection["progress"]),
                "terminal": terminal,
                "terminal_reason": (
                    ""
                    if projection is None
                    else str(projection.get("terminal_reason", ""))
                ),
                "result": dict(result),
            },
            "artifacts": _terminal_artifacts(result, fixture_only=True),
            "safety": {
                "stop_active": bool(replay_safety.get("stop_active", False)),
                "estop_latched": bool(
                    replay_safety.get("estop_latched", False)
                ),
                "collision_state": str(
                    replay_safety.get("collision_state", "CLEAR")
                ),
                "front_clearance_m": obstacles.get("forward_clearance_m"),
                "forward_corridor_clearance_m": obstacles.get(
                    "forward_clearance_m"
                ),
                "forward_corridor_min_angle_deg": -25.0,
                "forward_corridor_max_angle_deg": 25.0,
                "left_clearance_m": obstacles.get("left_clearance_m"),
                "right_clearance_m": obstacles.get("right_clearance_m"),
                "trajectory_min_clearance_m": obstacles.get(
                    "forward_clearance_m"
                ),
                "trajectory_horizon_s": 0.75,
                "telemetry_fresh": bool(localization.get("fresh", True)),
                "independent_robot_safety": True,
                "browser_is_sole_safety_mechanism": False,
            },
            "events": list(rolling_events),
            "map": map_payload,
            "rolling": (
                {
                    "world_snapshot": None,
                    "decision_snapshots": [],
                    "active_intent": None,
                    "intent_revisions": [],
                    "inference": {
                        "in_flight": False,
                        "provider_calls_started": 0,
                        "provider_calls_completed": 0,
                        "movement_updates_during_calls": 0,
                    },
                    "metrics": {},
                }
                if projection is None
                else {
                    "world_snapshot": projection["world_snapshot"],
                    "decision_snapshots": projection["decision_snapshots"],
                    "active_intent": projection["active_intent"],
                    "intent_revisions": projection["intent_revisions"],
                    "inference": projection["inference"],
                    "metrics": projection["metrics"],
                }
            ),
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
        self.stationary_perception_enabled = bool(
            service.get("stationary_perception_enabled", False)
        )
        self.mode = (
            "live/stationary-perception"
            if self.stationary_perception_enabled
            else "live"
            if self.live_execution_enabled
            else "live/proposal-only"
        )

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
            self.stationary_perception_enabled = bool(
                self._service_snapshot.get("stationary_perception_enabled", False)
            )
            self.mode = (
                "live/stationary-perception"
                if self.stationary_perception_enabled
                else "live"
                if self.live_execution_enabled
                else "live/proposal-only"
            )
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
                if self.stationary_perception_enabled:
                    server_approval = (
                        "APPROVE STATIONARY PERCEPTION "
                        f"{str(proposal_payload.get('proposal_digest', ''))}"
                    )
                else:
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
        if self.stationary_perception_enabled:
            required_fresh = all(
                isinstance(live_evidence.get(name), Mapping)
                and bool(live_evidence[name].get("fresh", False))
                for name in ("camera", "lidar", "localization", "semantic_map")
            )
        else:
            required_fresh = bool(
                isinstance(odom, Mapping)
                and isinstance(collision, Mapping)
                and odom.get("fresh", False)
                and collision.get("fresh", False)
            )
        stop_state = str(safety.get("stop_state", "UNKNOWN")).upper()
        estop_state = str(safety.get("estop_state", "UNKNOWN")).upper()
        execution_ready = bool(
            (
                self.stationary_perception_enabled
                and not self.live_execution_enabled
                and required_fresh
            )
            or (
                self.live_execution_enabled
                and required_fresh
                and str(safety.get("collision_state", "UNKNOWN")).upper() == "CLEAR"
                and stop_state == "READY"
                and estop_state == "CLEAR"
            )
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
                **(
                    {
                        "rolling_replay": True,
                        "stationary_perception": True,
                        "real_llm_provider": self._service_snapshot.get(
                            "provider_id"
                        )
                        == "openai-codex-oauth",
                        "provider_id": self._service_snapshot.get("provider_id", ""),
                        "model_id": self._service_snapshot.get("model_id", ""),
                        "reasoning_effort": self._service_snapshot.get(
                            "reasoning_effort", ""
                        ),
                        "motion_authority": False,
                        "physical_execution_enabled": False,
                    }
                    if self.stationary_perception_enabled
                    else {}
                ),
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
                "stationary_perception_only": self.stationary_perception_enabled,
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
                "left_clearance_m": safety.get("left_clearance_m"),
                "right_clearance_m": safety.get("right_clearance_m"),
                "trajectory_clearance_margin_m": safety.get(
                    "trajectory_clearance_margin_m"
                ),
                "trajectory_horizon_s": safety.get(
                    "trajectory_horizon_s"
                ),
                "trajectory_min_clearance_m": safety.get(
                    "trajectory_min_clearance_m"
                ),
                "trajectory_collision_time_s": safety.get(
                    "trajectory_collision_time_s"
                ),
                "telemetry_fresh": required_fresh,
                "independent_robot_safety": True,
                "browser_is_sole_safety_mechanism": False,
            },
            "events": translated_events,
            "map": _authoritative_live_map(live_evidence),
            "camera_preview": _live_camera_preview(live_evidence),
            "rolling": (
                _stationary_projection(result)
                if self.stationary_perception_enabled
                else {}
            ),
        }


def _stationary_projection(result: Mapping[str, Any]) -> dict[str, Any]:
    """Normalize running checkpoints and terminal stationary results for the UI."""

    world = result.get("world_snapshot", result.get("final_snapshot", {}))
    if not isinstance(world, Mapping):
        world = {}
    revisions = result.get("intent_revisions", [])
    decisions = result.get("decision_snapshots", [])
    inference = result.get("inference", {})
    metrics = result.get("metrics", {})
    return {
        "world_snapshot": dict(world),
        "decision_snapshots": list(decisions) if isinstance(decisions, list) else [],
        "active_intent": (
            result.get("active_intent")
            if isinstance(result.get("active_intent"), Mapping)
            else None
        ),
        "intent_revisions": list(revisions) if isinstance(revisions, list) else [],
        "inference": dict(inference) if isinstance(inference, Mapping) else {},
        "metrics": dict(metrics) if isinstance(metrics, Mapping) else {},
    }


def _live_camera_preview(live_evidence: Mapping[str, Any]) -> dict[str, Any]:
    camera = live_evidence.get("camera", {})
    if not isinstance(camera, Mapping):
        return {
            "available": False,
            "present": False,
            "valid": False,
            "fresh": False,
            "state": "unavailable",
        }
    value = camera.get("value", {})
    if not isinstance(value, Mapping):
        value = {}
    data_url = str(value.get("thumbnail_data_url", ""))
    has_frame = data_url.startswith("data:image/jpeg;base64,")
    present = bool(camera.get("present", bool(value)))
    valid = bool(camera.get("valid", bool(value)))
    fresh = bool(camera.get("fresh", False))
    error = str(camera.get("error", "")).strip()
    if error:
        state = "interrupted"
    elif has_frame and fresh:
        state = "fresh"
    elif has_frame:
        state = "stale"
    elif present:
        state = "empty"
    else:
        state = "unavailable"
    return {
        "available": has_frame,
        "present": present,
        "valid": valid,
        "fresh": fresh,
        "state": state,
        "error": error,
        "age_s": camera.get("age_s"),
        "received_at_s": camera.get("received_at_s"),
        "source_timestamp_s": camera.get("source_timestamp_s"),
        "frame_id": str(value.get("frame_id", "")),
        "stamp_s": value.get("stamp_s"),
        "width": value.get("width"),
        "height": value.get("height"),
        "data_url": data_url,
        "detections": value.get("detections", []),
        "tracks": value.get("tracks", []),
        "uncertain_track_id": str(value.get("uncertain_track_id", "")),
    }


def _web_state(status: str) -> str:
    normalized = str(status).strip().lower()
    mapping = {
        "ready": WebMissionState.READY,
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
    navigation_map = _authoritative_navigation_map(live_evidence)
    if navigation_map is not None:
        return navigation_map
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


def _authoritative_navigation_map(
    live_evidence: Mapping[str, Any],
) -> Optional[dict[str, Any]]:
    """Render fresh lidar-localized navigation without inventing map layers."""

    source = live_evidence.get("localization", {})
    if not (
        isinstance(source, Mapping)
        and source.get("fresh")
        and source.get("valid")
    ):
        return None
    value = source.get("value", {})
    if not isinstance(value, Mapping):
        return None
    navigation = value if value.get("schema") == "sphero_rvr.perception_navigation_result.v1" else {}
    localization = navigation.get("localization", value)
    if not isinstance(localization, Mapping):
        return None
    pose = localization.get("pose")
    if (
        not isinstance(pose, Mapping)
        or str(localization.get("state", "")).lower() not in {"valid", "degraded"}
        or not bool(localization.get("authoritative", False))
    ):
        return None
    try:
        rover = {
            "x_m": float(pose["x_m"]),
            "y_m": float(pose["y_m"]),
            "yaw_deg": float(pose.get("heading_deg", math.degrees(float(pose["yaw_rad"])))),
        }
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(number) for number in rover.values()):
        return None

    traveled_path: list[dict[str, float]] = []
    raw_path = navigation.get("path", [])
    if isinstance(raw_path, list):
        for item in raw_path:
            if not isinstance(item, Mapping):
                continue
            try:
                point = {"x_m": float(item["x_m"]), "y_m": float(item["y_m"])}
            except (KeyError, TypeError, ValueError):
                continue
            if all(math.isfinite(number) for number in point.values()):
                traveled_path.append(point)

    goal_region = None
    goal = navigation.get("goal")
    if isinstance(goal, Mapping):
        try:
            candidate = {
                "x_m": float(goal["x_m"]),
                "y_m": float(goal["y_m"]),
                "radius_m": float(goal["radius_m"]),
            }
        except (KeyError, TypeError, ValueError):
            candidate = {}
        if candidate and all(math.isfinite(number) for number in candidate.values()):
            goal_region = candidate

    proposed_route = [{"x_m": rover["x_m"], "y_m": rover["y_m"]}]
    horizon = navigation.get("next_horizon")
    if isinstance(horizon, Mapping) and str(horizon.get("kind", "")) == "translate":
        try:
            distance = float(horizon.get("distance_m", 0.0))
        except (TypeError, ValueError):
            distance = 0.0
        if math.isfinite(distance):
            yaw = math.radians(rover["yaw_deg"])
            proposed_route.append(
                {
                    "x_m": rover["x_m"] + distance * math.cos(yaw),
                    "y_m": rover["y_m"] + distance * math.sin(yaw),
                }
            )

    extent_points = list(traveled_path) + list(proposed_route)
    if goal_region is not None:
        extent_points.append(goal_region)
    xs = [rover["x_m"]] + [float(point["x_m"]) for point in extent_points]
    ys = [rover["y_m"]] + [float(point["y_m"]) for point in extent_points]
    margin = 0.5
    minimum_x, maximum_x = min(xs) - margin, max(xs) + margin
    minimum_y, maximum_y = min(ys) - margin, max(ys) + margin
    return {
        "available": True,
        "navigation_available": True,
        "occupancy_available": False,
        "semantic_objects_available": False,
        "unavailable_layers": ["occupancy", "semantic_objects"],
        "frame": str(pose.get("frame_id", "map")),
        "bounds": {
            "origin": {"x_m": minimum_x, "y_m": minimum_y},
            "width_m": max(1.0, maximum_x - minimum_x),
            "height_m": max(1.0, maximum_y - minimum_y),
        },
        "rover": rover,
        "goal_region": goal_region,
        "proposed_route": proposed_route,
        "traveled_path": traveled_path,
        "obstacles": [],
        "objects": [],
        "localization": {
            "state": str(localization.get("state", "unknown")),
            "quality": localization.get("quality"),
            "source": str(localization.get("source", "")),
            "odom_translation_disagreement_m": localization.get(
                "odom_translation_disagreement_m"
            ),
            "odom_heading_disagreement_rad": localization.get(
                "odom_heading_disagreement_rad"
            ),
        },
        "fixture_only": False,
        "source": "Pi mission service lidar localization",
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
    sensor_control: Optional[StationarySensorControl] = None,
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
            return _json_response(200, _with_sensor_control(adapter.snapshot(), sensor_control))
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
    elif normalized_path == STATIONARY_SENSOR_CONTROL_PATH:
        if sensor_control is None:
            raise MissionWebError("stationary sensor control is not available")
        if "active" not in payload or not isinstance(payload["active"], bool):
            raise MissionWebError("stationary sensor request requires a boolean active value")
        authority_snapshot = adapter.snapshot()
        sensor_control.set_active(
            bool(payload["active"]),
            authority_snapshot=authority_snapshot,
        )
        result = adapter.snapshot()
    else:
        raise MissionWebError(f"POST route is not exposed: {normalized_path}")
    return _json_response(200, _with_sensor_control(result, sensor_control))


def _with_sensor_control(
    snapshot: Mapping[str, Any],
    sensor_control: Optional[StationarySensorControl],
) -> dict[str, Any]:
    result = dict(snapshot)
    if sensor_control is None:
        status: dict[str, Any] = {
            "available": False,
            "active": False,
            "transitioning": False,
            "state": "unavailable",
            "detail": "Stationary sensor control is not configured.",
            "unit": STATIONARY_SENSOR_UNIT,
        }
    else:
        status = dict(sensor_control.status())
    status["start_permitted"] = _stationary_sensor_start_permitted(snapshot)
    result["sensor_control"] = status
    return result


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
        sensor_control = getattr(self.server, "stationary_sensor_control", None)
        set_identity = getattr(adapter, "set_request_identity", None)
        clear_identity = getattr(adapter, "clear_request_identity", None)
        if callable(set_identity) and self.command == "POST":
            set_identity(str(getattr(self, "_request_identity", "")))
        try:
            response = handle_mission_web_request(
                self.command,
                self.path,
                body,
                adapter,
                sensor_control,
            )
        except (MissionWebError, UnicodeDecodeError) as exc:
            self._send_error(400, str(exc))
            return
        finally:
            if callable(clear_identity) and self.command == "POST":
                clear_identity()
        self.send_response(response.status)
        self.send_header("Content-Type", response.content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; img-src 'self' data:; style-src 'self' 'unsafe-inline'; "
            "script-src 'self' 'unsafe-inline'; connect-src 'self'",
        )
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
    sensor_control: Optional[StationarySensorControl] = None,
    allowed_origin: Optional[str] = None,
    require_tailscale_identity: bool = False,
) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, int(port)), _MissionWebHttpHandler)
    setattr(server, "mission_web_adapter", adapter or MockReplayMissionAdapter())
    setattr(server, "stationary_sensor_control", sensor_control)
    setattr(server, "enforce_same_origin", allowed_origin is not None)
    setattr(server, "allowed_origin", "" if allowed_origin is None else str(allowed_origin).rstrip("/"))
    setattr(server, "require_tailscale_identity", bool(require_tailscale_identity))
    return server


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Serve the loopback-only RVR mission web console.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--mode",
        choices=("mock", "live", "rolling-replay"),
        default="mock",
    )
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
    parser.add_argument(
        "--replay-database",
        default="~/.local/state/sphero_rvr/rolling-replay.sqlite3",
        help="Persistent SQLite evidence used only by rolling-replay mode",
    )
    parser.add_argument("--replay-model", default=None)
    parser.add_argument(
        "--replay-reasoning-effort",
        choices=ALLOWED_REASONING_EFFORTS,
        default="low",
    )
    args = parser.parse_args(argv)
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        parser.error("the mission web service may bind only to a loopback host")
    if args.mode == "live":
        if not args.public_origin:
            parser.error("live mode requires --public-origin for same-origin enforcement")
        adapter: MissionWebAdapter = LiveMissionWebAdapter(
            # Stationary checkpoints deliberately retain exact world snapshots and
            # can exceed the generic command-response ceiling during a sustained
            # live run. Keep the browser boundary finite while allowing the
            # evidence-rich Stage C result to remain visible through termination.
            MissionServiceClient(
                args.mission_socket,
                max_response_bytes=128_000_000,
            ),
            session_id=args.session_id,
            operator=args.operator,
        )
        allowed_origin = args.public_origin
        sensor_control: Optional[StationarySensorControl] = SystemdStationarySensorControl()
    elif args.mode == "rolling-replay":
        adapter = RollingReplayMissionAdapter(
            CodexOAuthRollingIntentProvider(
                model=args.replay_model,
                reasoning_effort=args.replay_reasoning_effort,
            ),
            database=args.replay_database,
            source_sha=_local_source_sha(),
        )
        allowed_origin = None
        sensor_control = None
    else:
        adapter = MockReplayMissionAdapter()
        allowed_origin = None
        sensor_control = None
    server = make_server(
        args.host,
        args.port,
        adapter=adapter,
        sensor_control=sensor_control,
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
        close_adapter = getattr(adapter, "close", None)
        if callable(close_adapter):
            close_adapter()
    return 0


def _local_source_sha() -> str:
    configured = str(os.environ.get("RVR_SOURCE_SHA", "")).strip()
    if configured:
        return configured
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        completed = None
    value = "" if completed is None else str(completed.stdout).strip()
    return value or "rolling-replay-local-source"


_INDEX_HTML = r'''<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="theme-color" content="#5de4c7">
  <link rel="manifest" href="/manifest.webmanifest">
  <title>__APP_NAME__</title>
  <style>
    :root { color-scheme:dark; --ink:#edf5f5; --muted:#91a7ad; --panel:#0d1a28; --line:#294052; --teal:#65e0c2; --amber:#f4c36a; --red:#ff8179; --blue:#7da7f7; --slate:#64788c; font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; background:#07111b; color:var(--ink); }
    * { box-sizing:border-box; }
    [hidden] { display:none !important; }
    html { min-width:0; background:#07111b; }
    body { margin:0; min-width:0; min-height:100vh; overflow-x:hidden; background:linear-gradient(180deg,#0a1825 0,#07111b 18rem); }
    button, textarea, select, input { font:inherit; }
    button { cursor:pointer; }
    button:disabled { cursor:not-allowed; opacity:.45; }
    button:focus-visible, textarea:focus-visible, select:focus-visible, input:focus-visible, summary:focus-visible { outline:3px solid rgba(101,224,194,.75); outline-offset:3px; }
    header { display:flex; justify-content:space-between; gap:1rem; align-items:center; padding:.68rem clamp(.85rem,2vw,1.4rem); border-bottom:1px solid var(--line); background:rgba(7,17,27,.95); position:sticky; top:0; z-index:8; backdrop-filter:blur(12px); }
    h1 { font-size:clamp(1.05rem,1.6vw,1.35rem); margin:0; letter-spacing:.02em; }
    h2 { margin:0; font-size:.82rem; color:#c4d2d4; text-transform:uppercase; letter-spacing:.12em; }
    p { line-height:1.55; }
    .mode-badge { display:inline-flex; align-items:center; gap:.42rem; max-width:100%; padding:.38rem .66rem; border:1px solid #2a7c72; border-radius:99px; color:var(--teal); background:#0b292b; font-size:.7rem; font-weight:850; letter-spacing:.07em; text-align:center; }
    .mode-badge::before { content:""; width:.52rem; height:.52rem; border-radius:50%; background:var(--teal); box-shadow:0 0 .8rem var(--teal); }
    .mode-badge.live { color:var(--amber); border-color:#8f6729; background:#30220d; }
    .mode-badge.live::before { background:var(--amber); box-shadow:0 0 .8rem var(--amber); }
    .mode-badge.execution { color:var(--red); border-color:#8f3d43; background:#32161b; }
    .mode-badge.execution::before { background:var(--red); box-shadow:0 0 .8rem var(--red); }
    .shell { width:min(1680px,100%); max-width:100%; margin:auto; padding:clamp(.6rem,1.3vw,1rem); }
    .safety-strip { display:grid; grid-template-columns:repeat(8,minmax(0,1fr)); gap:.5rem; margin-bottom:.65rem; }
    .safety-cell { min-width:0; padding:.52rem .62rem; border:1px solid var(--line); border-radius:.58rem; background:#0d1a28; }
    .safety-cell span { display:block; color:var(--muted); font-size:.61rem; text-transform:uppercase; letter-spacing:.08em; }
    .safety-cell strong { display:block; margin-top:.2rem; overflow-wrap:anywhere; font-size:.76rem; line-height:1.25; }
    .safety-cell.warning { border-color:#91652d; background:#2b2112; }
    .safety-cell.unsafe { border-color:#91414a; background:#31191e; }
    .sensor-control button { width:100%; margin-top:.28rem; padding:.32rem .42rem; border:1px solid #40617a; border-radius:.42rem; color:var(--ink); background:#182a3d; font-size:.7rem; font-weight:800; }
    .sensor-control small { display:block; margin-top:.22rem; color:var(--muted); font-size:.62rem; line-height:1.25; }
    .workspace { display:grid; grid-template-columns:minmax(0,2.35fr) minmax(320px,.75fr); gap:.75rem; align-items:start; min-width:0; }
    .visual-column, .ops-sidebar { display:grid; gap:.75rem; min-width:0; }
    .ops-sidebar { position:sticky; top:4.15rem; max-height:calc(100vh - 4.8rem); overflow-y:auto; overscroll-behavior:contain; padding-right:.15rem; scrollbar-width:thin; }
    .panel { min-width:0; border:1px solid var(--line); border-radius:.72rem; padding:.8rem; background:linear-gradient(145deg,rgba(16,32,47,.98),rgba(10,22,34,.98)); box-shadow:0 10px 30px rgba(0,0,0,.13); }
    .panel-heading { display:flex; align-items:center; justify-content:space-between; gap:.7rem; margin-bottom:.65rem; }
    .panel-heading .hint { margin:0; text-align:right; }
    .field-label { display:block; margin:.7rem 0 .35rem; color:#bbcccd; font-size:.8rem; font-weight:700; }
    textarea, select, input { width:100%; color:var(--ink); background:#071421; border:1px solid #294258; border-radius:.7rem; padding:.75rem; outline:none; }
    textarea { resize:vertical; min-height:5.5rem; }
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
    .map-panel { padding:.65rem; }
    .map-frame { position:relative; aspect-ratio:16/10; min-height:520px; border:1px solid #355168; overflow:hidden; border-radius:.58rem; background:#07121d; }
    #mission-map { width:100%; height:100%; display:block; }
    .visual-status { display:flex; flex-wrap:wrap; gap:.38rem; align-items:center; }
    .status-pill { display:inline-flex; align-items:center; gap:.3rem; min-width:0; padding:.25rem .45rem; border:1px solid #355168; border-radius:99px; background:rgba(7,18,29,.88); color:#c8d7da; font-size:.65rem; font-weight:750; }
    .status-pill::before { content:""; flex:0 0 auto; width:.42rem; height:.42rem; border-radius:50%; background:var(--teal); }
    .status-pill.stale::before, .status-pill.degraded::before { background:var(--amber); }
    .status-pill.unavailable::before, .status-pill.interrupted::before { background:var(--red); }
    .map-overlay { position:absolute; inset:.55rem .55rem auto; display:flex; flex-wrap:wrap; gap:.35rem; z-index:2; pointer-events:none; }
    .visual-alert { position:absolute; inset:auto .55rem .55rem; z-index:2; padding:.52rem .65rem; border:1px solid #8f6729; border-radius:.5rem; background:rgba(48,34,13,.94); color:#f7d99c; font-size:.72rem; font-weight:750; }
    .legend { display:flex; flex-wrap:wrap; gap:.72rem; margin-top:.55rem; color:var(--muted); font-size:.68rem; }
    .legend i { width:.7rem; height:.7rem; border-radius:50%; display:inline-block; margin-right:.28rem; vertical-align:-.05rem; }
    .camera-panel { padding:.65rem; }
    .camera-frame { position:relative; aspect-ratio:16/9; overflow:hidden; border:1px solid #355168; border-radius:.58rem; background:#050b12; }
    #live-camera-preview { width:100%; height:100%; display:block; object-fit:contain; background:#050b12; }
    .camera-empty { position:absolute; inset:0; display:grid; place-items:center; padding:2rem; text-align:center; color:#a7bac0; background:radial-gradient(circle at 50% 35%,#142d3e 0,#07121d 60%); }
    .camera-empty strong { display:block; color:#dce7e8; font-size:1rem; }
    .camera-empty span { display:block; max-width:38rem; margin-top:.35rem; font-size:.78rem; line-height:1.45; }
    .camera-overlay { position:absolute; inset:0; pointer-events:none; }
    .detection-box { position:absolute; border:2px solid var(--amber); border-radius:.3rem; box-shadow:0 0 0 1px rgba(0,0,0,.4); }
    .detection-box span { position:absolute; left:-2px; bottom:100%; max-width:14rem; padding:.18rem .3rem; overflow:hidden; border-radius:.22rem .22rem 0 0; background:rgba(35,27,8,.92); color:#ffdf9d; font-size:.62rem; font-weight:800; white-space:nowrap; text-overflow:ellipsis; }
    .camera-interruption { position:absolute; inset:0; display:grid; place-items:center; padding:2rem; text-align:center; background:rgba(33,10,14,.72); color:#ffd1cd; font-weight:850; }
    .camera-meta { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:.45rem; margin-top:.52rem; }
    .camera-meta div { min-width:0; padding:.45rem .52rem; border-radius:.45rem; background:#081521; }
    .camera-meta span { display:block; color:var(--muted); font-size:.6rem; text-transform:uppercase; letter-spacing:.06em; }
    .camera-meta strong { display:block; margin-top:.18rem; overflow-wrap:anywhere; font-size:.72rem; }
    .detection-summary { display:flex; flex-wrap:wrap; gap:.38rem; margin-top:.5rem; }
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
    .rolling-grid { display:grid; gap:.6rem; grid-template-columns:repeat(2,minmax(0,1fr)); }
    .rolling-card { min-width:0; padding:.65rem; border:1px solid #294258; border-radius:.65rem; background:#071421; }
    .rolling-card span { display:block; color:var(--muted); font-size:.67rem; text-transform:uppercase; letter-spacing:.06em; }
    .rolling-card strong { display:block; margin-top:.24rem; overflow-wrap:anywhere; color:#d8f5ef; font-size:.8rem; }
    .rolling-json { margin:.65rem 0 0; max-height:16rem; overflow:auto; white-space:pre-wrap; overflow-wrap:anywhere; padding:.65rem; border-radius:.55rem; background:#071421; color:#bdd2ff; font-size:.68rem; }
    .revision-list { display:grid; gap:.55rem; max-height:22rem; overflow:auto; }
    .revision { border-left:3px solid var(--teal); padding:.6rem .7rem; border-radius:.25rem .6rem .6rem .25rem; background:#091827; font-size:.76rem; line-height:1.4; }
    .revision small { color:var(--muted); display:block; margin-bottom:.18rem; }
    .in-flight { color:var(--amber) !important; }
    .error { min-height:1.2rem; color:#ffaba5; font-size:.8rem; margin-top:.5rem; }
    .rejected { border-color:#7e3541; color:#ffd3d0; }
    details.panel { padding:0; }
    details.panel > summary { cursor:pointer; list-style:none; padding:.75rem .8rem; color:#c4d2d4; font-size:.78rem; font-weight:800; text-transform:uppercase; letter-spacing:.1em; }
    details.panel > summary::-webkit-details-marker { display:none; }
    details.panel > summary::after { content:"+"; float:right; color:var(--teal); font-size:1rem; }
    details.panel[open] > summary::after { content:"−"; }
    details.panel > .detail-body { padding:0 .8rem .8rem; }
    @media (max-width:1180px) {
      .workspace { grid-template-columns:minmax(0,1.8fr) minmax(310px,.8fr); }
      .map-frame { min-height:420px; }
      .safety-strip { grid-template-columns:repeat(4,minmax(0,1fr)); }
    }
    @media (max-width:1100px) {
      header { position:static; align-items:flex-start; flex-direction:column; }
      .shell { padding:.55rem; }
      .safety-strip { grid-template-columns:1fr 1fr; }
      .workspace { grid-template-columns:minmax(0,1fr); }
      .visual-column { grid-row:1; }
      .ops-sidebar { grid-row:2; position:static; max-height:none; overflow:visible; padding:0; }
      .map-frame { min-height:0; aspect-ratio:4/3; }
      .camera-frame { aspect-ratio:4/3; }
      .camera-meta { grid-template-columns:1fr; }
      .plan-meta, .rolling-grid { grid-template-columns:1fr; }
      .segment { flex-direction:column; gap:.35rem; }
      .segment strong { text-align:left; font-size:.76rem; }
    }
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
      <div class="safety-cell"><span>Projected path</span><strong id="safety-trajectory">UNAVAILABLE</strong></div>
      <div class="safety-cell"><span>Telemetry</span><strong id="safety-telemetry">FRESH</strong></div>
      <div class="safety-cell sensor-control" id="sensor-control" hidden>
        <span>Telemetry + camera</span>
        <button id="sensors-toggle" type="button" aria-pressed="false" disabled>Turn sensors on</button>
        <small id="sensors-status" role="status">Control unavailable</small>
      </div>
      <div class="safety-cell"><span>STOP</span><strong id="safety-stop">READY</strong></div>
      <div class="safety-cell"><span>ESTOP</span><strong id="safety-estop">CLEAR</strong></div>
      <div class="safety-cell"><span>Motion authority</span><strong id="safety-authority">NONE</strong></div>
    </section>
    <div class="workspace">
      <div class="visual-column">
        <section class="panel map-panel" aria-labelledby="map-heading">
          <div class="panel-heading">
            <h2 id="map-heading">Live spatial map</h2>
            <div class="visual-status" id="map-status" aria-live="polite"></div>
          </div>
          <div class="map-frame">
            <svg id="mission-map" role="img" aria-label="Fixture room map showing rover, route, path, obstacles, and objects"></svg>
            <div class="map-overlay" id="map-overlay" aria-hidden="true"></div>
            <div class="visual-alert" id="map-alert" hidden></div>
          </div>
          <div class="legend" aria-label="Map legend">
            <span><i style="background:#7da7f7"></i>Goal / safe corridor</span>
            <span><i style="background:#65e0c2"></i>Rover / traveled path</span>
            <span><i style="background:#f4c36a"></i>Semantic tracks</span>
            <span><i style="background:#64788c"></i>Occupancy / obstacles</span>
          </div>
        </section>
        <section class="panel camera-panel" aria-labelledby="camera-heading">
          <div class="panel-heading">
            <h2 id="camera-heading">Camera</h2>
            <div class="visual-status" id="camera-status" aria-live="polite"></div>
          </div>
          <div class="camera-frame" id="camera-frame">
            <img id="live-camera-preview" hidden alt="Latest rover camera evidence">
            <div class="camera-empty" id="camera-empty">
              <div><strong>No camera frame available</strong><span>The console will not substitute a fixture for a missing image source.</span></div>
            </div>
            <div class="camera-overlay" id="camera-overlay" aria-hidden="true"></div>
            <div class="camera-interruption" id="camera-interruption" hidden></div>
          </div>
          <div class="camera-meta" aria-label="Camera evidence metadata">
            <div><span>Frame ID</span><strong id="camera-frame-id">Unavailable</strong></div>
            <div><span>Evidence timestamp</span><strong id="camera-timestamp">Unavailable</strong></div>
            <div><span>Observation focus</span><strong id="camera-focus">No active request</strong></div>
          </div>
          <div class="detection-summary" id="camera-detections" aria-live="polite"></div>
        </section>
      </div>
      <aside class="ops-sidebar" aria-label="Mission controls and details">
        <section class="panel" aria-labelledby="status-heading">
          <div class="panel-heading"><h2 id="status-heading">Mission status</h2><span class="terminal" id="terminal-reason"></span></div>
          <div class="status-line"><div class="state" id="mission-state" data-testid="mission-state">READY</div><div class="hint" id="progress-label">0% complete</div></div>
          <progress id="mission-progress" max="100" value="0"></progress>
        </section>
        <section class="panel" aria-labelledby="mission-heading">
          <h2 id="mission-heading">Mission prompt</h2>
          <label class="field-label" for="mission-prompt">Tell the rover what to do</label>
          <textarea id="mission-prompt" data-testid="mission-prompt">Explore this room, identify and map the shoes and any recognized people, inspect uncertain findings from another viewpoint, then stop safely.</textarea>
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
        <section class="panel" id="rolling-intent-panel" hidden>
          <h2>Current finite leased intent</h2>
          <div id="rolling-intent" class="empty">No validated intent yet.</div>
        </section>
        <section class="panel" aria-labelledby="proposal-heading">
          <h2 id="proposal-heading">LLM proposal</h2>
          <div id="proposal-view" class="empty">Submit a mission to see a typed route proposal or rejection.</div>
        </section>
        <section class="panel" id="rolling-loop-panel" hidden>
          <h2>Asynchronous LLM loop</h2>
          <div class="rolling-grid" id="rolling-metrics"></div>
          <div class="revision-list" id="rolling-revisions"></div>
        </section>
        <details class="panel" id="rolling-world-panel" hidden>
          <summary>Fresh world snapshot &amp; detections</summary>
          <div class="detail-body"><pre class="rolling-json" id="rolling-world"></pre></div>
        </details>
        <details class="panel" id="terminal-panel">
          <summary id="result-heading">Terminal evidence &amp; artifacts</summary>
          <div class="detail-body">
            <div id="result-view" class="empty" data-testid="result-view">No terminal evidence yet.</div>
            <ul id="artifact-list" class="artifact-list"></ul>
          </div>
        </details>
        <details class="panel">
          <summary id="events-heading">Event history</summary>
          <div class="detail-body"><ol class="event-list" id="event-list"><li class="empty">No mission events yet.</li></ol></div>
        </details>
        <details class="panel">
          <summary>Authority boundary &amp; diagnostics</summary>
          <div class="detail-body"><p class="hint" id="authority-copy">The browser uses a typed mock/replay adapter. Planning, approval authority, and any future execution remain server-side on the Pi. Independent robot safety is never replaced by this page.</p></div>
        </details>
      </aside>
    </div>
  </main>
  <script>
    const $ = (id) => document.getElementById(id);
    let current = null;
    let timer = null;
    let hydratedMissionId = null;
    let promptDirty = false;
    let sensorRequestInFlight = false;

    async function api(path, options = {}) {
      const response = await fetch(path, {headers:{'Content-Type':'application/json'}, ...options});
      const payload = await response.json();
      if (!response.ok) throw new Error(payload.error || `Request failed: ${response.status}`);
      return payload;
    }

    function escapeHtml(value) {
      return String(value).replace(/[&<>'"]/g, (char) => ({'&':'&amp;','<':'&lt;','>':'&gt;',"'":'&#39;','"':'&quot;'}[char]));
    }

    function statusPill(label, value, tone = '') {
      return `<span class="status-pill ${escapeHtml(tone)}">${escapeHtml(label)}: ${escapeHtml(value)}</span>`;
    }

    function formatEvidenceTime(value) {
      const stamp = finiteNumber(value);
      if (stamp === null) return 'Unavailable';
      if (stamp < 1e9) return `${stamp.toFixed(3)} s replay time`;
      const milliseconds = stamp > 1e12 ? stamp : stamp * 1000;
      const date = new Date(milliseconds);
      return Number.isNaN(date.getTime()) ? `${stamp.toFixed(3)} s` : date.toISOString();
    }

    function finiteNumber(value) {
      if (value === null || value === '' || typeof value === 'boolean') return null;
      const number = Number(value);
      return Number.isFinite(number) ? number : null;
    }

    function setSafetyTone(id, tone) {
      const cell = $(id).closest('.safety-cell');
      cell.classList.toggle('warning', tone === 'warning');
      cell.classList.toggle('unsafe', tone === 'unsafe');
    }

    function shouldContinuouslyPoll(snapshot) {
      return !snapshot.adapter.fixture_only
        && (!snapshot.mission.terminal || Boolean(snapshot.adapter.stationary_perception));
    }

    function renderSensorControl(snapshot) {
      const stationary = Boolean(snapshot.adapter.stationary_perception);
      const panel = $('sensor-control');
      const button = $('sensors-toggle');
      const label = $('sensors-status');
      const control = snapshot.sensor_control || {};
      panel.hidden = !stationary;
      if (!stationary) return;
      const active = Boolean(control.active);
      const transitioning = sensorRequestInFlight || Boolean(control.transitioning);
      button.setAttribute('aria-pressed', active ? 'true' : 'false');
      button.textContent = transitioning
        ? (active ? 'Stopping…' : 'Starting…')
        : (active ? 'Turn sensors off' : 'Turn sensors on');
      button.disabled = transitioning
        || !control.available
        || (!active && !control.start_permitted);
      label.textContent = transitioning
        ? (active ? 'Stopping stationary sensors' : 'Starting stationary sensors')
        : active
          ? (snapshot.safety.telemetry_fresh ? 'On · data fresh' : 'On · waiting for fresh data')
          : control.available
            ? 'Off · no live map or camera'
            : (control.detail || 'Control unavailable');
      panel.classList.toggle('warning', !active || !snapshot.safety.telemetry_fresh);
      panel.classList.toggle('unsafe', active && control.state === 'failed');
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
      const rollingReplay = Boolean(snapshot.adapter.rolling_replay);
      const stationary = Boolean(snapshot.adapter.stationary_perception);
      const execution = live && snapshot.adapter.live_execution_enabled;
      const badge = document.querySelector('[data-testid="mode-badge"]');
      badge.className = `mode-badge${live || rollingReplay ? ' live' : ''}${execution ? ' execution' : ''}`;
      badge.textContent = stationary ? 'LIVE STATIONARY PERCEPTION — NO MOTION AUTHORITY' : rollingReplay ? 'ROLLING LLM REPLAY — NO MOTION AUTHORITY' : live ? (execution ? 'LIVE — PHYSICAL EXECUTION ENABLED' : 'LIVE — PROPOSAL ONLY / EXECUTION LOCKED') : 'MOCK / REPLAY — NO LIVE EXECUTION';
      $('scenario-label').textContent = stationary ? 'Stationary sensor target' : live ? 'Service target' : rollingReplay ? 'Replay demonstration' : 'Replay outcome';
      $('approval-heading').textContent = stationary ? 'Stationary perception confirmation' : live ? 'Run confirmation' : rollingReplay ? 'Replay confirmation' : 'Simulation approval';
      $('approval-hint').textContent = stationary ? 'Starts only continuous live sensing and leased observation intent. Physical execution remains locked.' : live ? (execution ? 'Review the current route, then click once to run it. No code or hash entry is required.' : 'Physical execution is locked by the deployed Pi configuration.') : rollingReplay ? 'Digest-bound confirmation starts only the persistent no-authority replay and real asynchronous LLM loop.' : 'Approval is digest-bound and authorizes only the mock adapter.';
      $('authority-copy').textContent = stationary ? 'Live lidar, camera, tracking, semantic mapping, persistence, and OAuth inference run concurrently on the Pi. The rover driver, serial transport, motion topics, motor graph, and physical authority are absent.' : live ? 'The browser uses the Pi-local mission-service boundary. Planning, OAuth, persistence, approval authority, and any physical execution remain on the Pi. Independent robot safety is never replaced by this page.' : rollingReplay ? 'MissionService persists this replay. The authenticated LLM may revise only typed finite leased intent; deterministic freshness and safety own immediate stop. ROS, sensors, serial, and motor authority are absent.' : 'The browser uses a typed mock/replay adapter. Planning, approval authority, and any future execution remain server-side on the Pi. Independent robot safety is never replaced by this page.';
      $('approve').textContent = stationary ? 'Start stationary perception' : live ? 'Approve and run' : rollingReplay ? 'Start rolling replay' : 'Approve simulation';
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
      const trajectoryClearance = snapshot.safety.trajectory_min_clearance_m == null
        ? Number.NaN : Number(snapshot.safety.trajectory_min_clearance_m);
      const trajectoryHorizon = snapshot.safety.trajectory_horizon_s == null
        ? Number.NaN : Number(snapshot.safety.trajectory_horizon_s);
      const leftClearance = snapshot.safety.left_clearance_m == null
        ? Number.NaN : Number(snapshot.safety.left_clearance_m);
      const rightClearance = snapshot.safety.right_clearance_m == null
        ? Number.NaN : Number(snapshot.safety.right_clearance_m);
      $('safety-trajectory').textContent = Number.isFinite(trajectoryClearance)
        ? `${trajectoryClearance.toFixed(2)} m over ${trajectoryHorizon.toFixed(2)} s`
        : Number.isFinite(leftClearance) && Number.isFinite(rightClearance)
          ? `L ${leftClearance.toFixed(2)} / R ${rightClearance.toFixed(2)} m`
          : 'UNAVAILABLE';
      $('safety-telemetry').textContent = snapshot.safety.telemetry_fresh ? 'FRESH' : 'STALE — BLOCKED';
      $('safety-stop').textContent = snapshot.safety.stop_state || (snapshot.safety.stop_active ? 'ACTIVE' : 'READY');
      $('safety-estop').textContent = snapshot.safety.estop_state || (snapshot.safety.estop_latched ? 'LATCHED' : 'CLEAR');
      $('safety-authority').textContent = execution ? 'PHYSICAL ENABLED' : stationary ? 'STATIONARY ONLY' : rollingReplay ? 'REPLAY ONLY' : live ? 'PHYSICAL LOCKED' : 'SIMULATION ONLY';
      const collisionState = String(snapshot.safety.collision_state || 'UNKNOWN').toUpperCase();
      const stopState = String($('safety-stop').textContent).toUpperCase();
      const estopState = String($('safety-estop').textContent).toUpperCase();
      setSafetyTone('safety-collision', ['CLEAR','SLOW'].includes(collisionState) ? (collisionState === 'SLOW' ? 'warning' : '') : 'unsafe');
      setSafetyTone('safety-telemetry', snapshot.safety.telemetry_fresh ? '' : 'unsafe');
      setSafetyTone('safety-stop', ['READY','CLEAR'].includes(stopState) ? '' : stopState === 'UNKNOWN' ? 'warning' : 'unsafe');
      setSafetyTone('safety-estop', estopState === 'CLEAR' ? '' : estopState === 'UNKNOWN' ? 'warning' : 'unsafe');
      setSafetyTone('safety-authority', execution ? 'unsafe' : stationary || rollingReplay || live ? 'warning' : '');
      renderSensorControl(snapshot);
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
      $('terminal-panel').open = Boolean(snapshot.mission.terminal);
      renderRolling(snapshot);
      renderMap(snapshot.map, snapshot);
      renderCamera(snapshot);
      if (!shouldContinuouslyPoll(snapshot) && snapshot.mission.terminal && timer) {
        clearInterval(timer);
        timer = null;
      }
    }

    function renderRolling(snapshot) {
      const enabled = Boolean(snapshot.adapter.rolling_replay);
      const stationary = Boolean(snapshot.adapter.stationary_perception);
      $('rolling-intent-panel').hidden = !enabled;
      $('rolling-loop-panel').hidden = !enabled;
      $('rolling-world-panel').hidden = !enabled;
      if (!enabled) return;
      const rolling = snapshot.rolling || {};
      const intent = rolling.active_intent;
      if (intent) {
        $('rolling-intent').className = '';
        $('rolling-intent').innerHTML = stationary
          ? `<div class="rolling-grid"><div class="rolling-card"><span>Revision</span><strong>#${escapeHtml(intent.revision)}</strong></div><div class="rolling-card"><span>Lease</span><strong>${escapeHtml(intent.lease_s)} s · expires ${Number(intent.expires_at_s).toFixed(1)}</strong></div><div class="rolling-card"><span>Action</span><strong>${escapeHtml(intent.action)}</strong></div><div class="rolling-card"><span>Observation focus</span><strong>${escapeHtml(intent.observation_focus)}</strong></div><div class="rolling-card"><span>Viewpoint recommendation</span><strong>${escapeHtml(intent.viewpoint_recommendation)}</strong></div><div class="rolling-card"><span>Search targets</span><strong>${escapeHtml((intent.search_targets || []).join(', '))}</strong></div></div><p>${escapeHtml(intent.rationale)}</p><code class="digest">snapshot ${escapeHtml(intent.snapshot_id)} · motion_authority=false</code>`
          : `<div class="rolling-grid"><div class="rolling-card"><span>Revision</span><strong>#${escapeHtml(intent.revision)}</strong></div><div class="rolling-card"><span>Lease</span><strong>${escapeHtml(intent.lease_s)} s · expires ${Number(intent.expires_at_s).toFixed(1)}</strong></div><div class="rolling-card"><span>Steering / speed</span><strong>${Number(intent.steering).toFixed(2)} · ${Number(intent.speed_limit_mps).toFixed(2)} m/s</strong></div><div class="rolling-card"><span>Safe corridor</span><strong>${escapeHtml(intent.safe_corridor)}</strong></div><div class="rolling-card"><span>Observation focus</span><strong>${escapeHtml(intent.observation_focus)}</strong></div><div class="rolling-card"><span>Viewpoint</span><strong>${escapeHtml(intent.viewpoint)}</strong></div></div><p>${escapeHtml(intent.rationale)}</p><code class="digest">snapshot ${escapeHtml(intent.snapshot_id)}</code>`;
      } else {
        $('rolling-intent').className = 'empty';
        $('rolling-intent').textContent = rolling.inference && rolling.inference.in_flight ? 'First LLM intent is being produced while replay perception updates.' : 'No validated intent yet.';
      }
      const inference = rolling.inference || {};
      const metrics = rolling.metrics || {};
      const metricItems = stationary ? [
        ['LLM state', inference.in_flight ? `CALL ${inference.call} IN FLIGHT` : 'IDLE'],
        ['Intent revisions', metrics.intent_revision_count || 0],
        ['Sensor updates during LLM', metrics.sensor_updates_while_llm_in_flight || 0],
        ['Camera / lidar updates', `${metrics.camera_updates || 0} / ${metrics.lidar_updates || 0}`],
        ['Semantic-map updates', metrics.semantic_map_updates || 0],
        ['Enrolled / unknown faces', `${metrics.enrolled_face_track_count || 0} / ${metrics.unknown_face_track_count || 0}`],
      ] : [
        ['LLM state', inference.in_flight ? `CALL ${inference.call} IN FLIGHT` : 'IDLE'],
        ['Intent revisions', metrics.intent_revision_count || 0],
        ['Motion ticks during LLM', metrics.motion_updates_while_llm_in_flight || 0],
        ['Artificial zero gaps', metrics.artificial_zero_motion_gaps || 0],
        ['Perception / map updates', `${metrics.perception_updates || 0} / ${metrics.semantic_map_updates || 0}`],
        ['Object / face tracking', `${metrics.continuous_object_tracking ? 'continuous' : 'warming'} / ${metrics.continuous_face_tracking ? 'continuous' : 'warming'}`],
      ];
      $('rolling-metrics').innerHTML = metricItems.map(([label,value], index) => `<div class="rolling-card"><span>${escapeHtml(label)}</span><strong class="${index === 0 && inference.in_flight ? 'in-flight' : ''}">${escapeHtml(value)}</strong></div>`).join('');
      const revisions = rolling.intent_revisions || [];
      $('rolling-revisions').innerHTML = revisions.length ? revisions.slice().reverse().map((item) => stationary
        ? `<div class="revision"><small>Revision ${escapeHtml(item.revision)} · snapshot ${escapeHtml(String(item.snapshot_id).slice(0,12))} · ${escapeHtml(item.sensor_updates_during_call)} live sensor updates during call</small><strong>${escapeHtml(item.action)} · ${escapeHtml(item.observation_focus)} / ${escapeHtml(item.viewpoint_recommendation)}</strong><br>${escapeHtml(item.rationale)}</div>`
        : `<div class="revision"><small>Revision ${escapeHtml(item.revision)} · snapshot ${escapeHtml(String(item.snapshot_id).slice(0,12))} · ${escapeHtml(item.movement_updates_during_call)} replay updates during call</small><strong>${Number(item.steering).toFixed(2)} steering · ${escapeHtml(item.safe_corridor)} · ${escapeHtml(item.observation_focus)} / ${escapeHtml(item.viewpoint)}</strong><br>${escapeHtml(item.rationale)}</div>`).join('') : '<p class="empty">No completed LLM revisions yet.</p>';
      const world = rolling.world_snapshot || {};
      const visible = stationary ? {
        snapshot_id: world.snapshot_id,
        version: world.version,
        observed_at_s: world.observed_at_s,
        sources: world.sources,
        occupancy: world.occupancy,
        detections: world.detections,
        semantic_tracks: world.semantic_tracks,
        uncertain_track_id: world.uncertain_track_id,
        safety: world.safety,
      } : {
        snapshot_id: world.snapshot_id,
        tick: world.tick,
        elapsed_s: world.elapsed_s,
        localization: world.localization,
        obstacles: world.obstacles,
        progress: world.progress,
        camera: world.camera,
        semantic_map: world.semantic_map,
      };
      $('rolling-world').textContent = JSON.stringify(visible, null, 2);
    }

    function renderCamera(snapshot) {
      const stationary = Boolean(snapshot.adapter.stationary_perception);
      const rolling = snapshot.rolling || {};
      const world = rolling.world_snapshot || {};
      const worldCamera = world.camera || {};
      const preview = snapshot.camera_preview || {};
      const frameId = preview.frame_id || worldCamera.frame_id || '';
      const stamp = preview.stamp_s ?? preview.source_timestamp_s ?? worldCamera.stamp_s;
      const detections = Array.isArray(preview.detections)
        ? preview.detections
        : Array.isArray(worldCamera.detections) ? worldCamera.detections : [];
      const intent = rolling.active_intent || {};
      const focus = intent.observation_focus || 'No active request';
      const viewpoint = intent.viewpoint_recommendation || intent.viewpoint || '';
      const hasPixels = Boolean(preview.available && preview.data_url);
      const state = preview.state || (frameId ? 'metadata-only' : 'unavailable');
      const image = $('live-camera-preview');
      const empty = $('camera-empty');
      const interruption = $('camera-interruption');

      image.hidden = !hasPixels;
      if (hasPixels) {
        image.src = preview.data_url;
        image.alt = `Rover camera frame ${frameId || 'without identifier'} with detection overlays`;
      } else {
        image.removeAttribute('src');
      }
      empty.hidden = hasPixels || state === 'interrupted';
      if (!hasPixels && state !== 'interrupted') {
        const title = frameId ? 'Frame pixels not supplied' : 'Camera source unavailable';
        const explanation = frameId
          ? `Evidence metadata for ${frameId} is available, but this ${stationary ? 'live source' : 'replay'} supplied no image pixels.`
          : 'The console will not substitute a fixture for a missing image source.';
        empty.innerHTML = `<div><strong>${escapeHtml(title)}</strong><span>${escapeHtml(explanation)}</span></div>`;
      }

      interruption.hidden = state !== 'interrupted' && !(hasPixels && !preview.fresh);
      if (!interruption.hidden) {
        interruption.textContent = state === 'interrupted'
          ? `CAMERA INTERRUPTED — ${preview.error || 'source error'}`
          : 'STALE CAMERA EVIDENCE — latest frame retained for review';
      }

      $('camera-frame-id').textContent = frameId || 'Unavailable';
      $('camera-timestamp').textContent = formatEvidenceTime(stamp);
      $('camera-focus').textContent = viewpoint ? `${focus} · ${viewpoint}` : focus;
      const cameraTone = hasPixels && preview.fresh ? '' : state === 'interrupted' ? 'interrupted' : state === 'stale' ? 'stale' : 'unavailable';
      const age = finiteNumber(preview.age_s);
      $('camera-status').innerHTML = statusPill(
        'Input',
        hasPixels ? (preview.fresh ? 'fresh' : state) : frameId ? 'metadata only' : state,
        cameraTone
      ) + (age !== null ? statusPill('Age', `${age.toFixed(2)} s`, age > 1.5 ? 'stale' : '') : '');

      const width = finiteNumber(preview.width);
      const height = finiteNumber(preview.height);
      $('camera-overlay').innerHTML = hasPixels && width !== null && width > 0 && height !== null && height > 0
        ? detections.filter((item) => item && item.bbox).map((item) => {
            const box = item.bbox;
            const left = Math.max(0, Math.min(100, Number(box.x) / width * 100));
            const top = Math.max(0, Math.min(100, Number(box.y) / height * 100));
            const boxWidth = Math.max(0, Math.min(100 - left, Number(box.width) / width * 100));
            const boxHeight = Math.max(0, Math.min(100 - top, Number(box.height) / height * 100));
            const confidence = finiteNumber(item.confidence);
            const label = `${item.track_id || item.detection_id || 'detection'} · ${item.label || 'unknown'}${confidence !== null ? ` ${Math.round(confidence * 100)}%` : ''}`;
            return `<div class="detection-box" style="left:${left}%;top:${top}%;width:${boxWidth}%;height:${boxHeight}%"><span>${escapeHtml(label)}</span></div>`;
          }).join('')
        : '';
      $('camera-detections').innerHTML = detections.length
        ? detections.map((item) => {
            const confidence = finiteNumber(item.confidence);
            const label = `${item.track_id || item.detection_id || 'detection'} · ${item.label || 'unknown'}${confidence !== null ? ` · ${Math.round(confidence * 100)}%` : ''}`;
            return `<span class="chip">${escapeHtml(label)}</span>`;
          }).join('')
        : '<span class="hint">No detections reported for this frame.</span>';
    }

    function renderMap(map, snapshot) {
      const svg = $('mission-map');
      const W = 900, H = 600, pad = 42;
      const bounds = map.bounds || {origin:{x_m:0,y_m:0},width_m:1,height_m:1};
      const origin = bounds.origin || {x_m:0, y_m:0};
      const widthM = Math.max(.001, Number(bounds.width_m) || 1);
      const heightM = Math.max(.001, Number(bounds.height_m) || 1);
      const rover = map.rover || {x_m:0,y_m:0,yaw_deg:0};
      const sx = (x) => pad + ((Number(x) - Number(origin.x_m || 0)) / widthM) * (W - pad * 2);
      const sy = (y) => H - pad - ((Number(y) - Number(origin.y_m || 0)) / heightM) * (H - pad * 2);
      const points = (items) => items.map((p) => `${sx(p.x_m)},${sy(p.y_m)}`).join(' ');
      const grid = Array.from({length:11}, (_,i) => `<line x1="${pad + i*(W-pad*2)/10}" y1="${pad}" x2="${pad + i*(W-pad*2)/10}" y2="${H-pad}" stroke="#14283a"/><line x1="${pad}" y1="${pad + i*(H-pad*2)/10}" x2="${W-pad}" y2="${pad + i*(H-pad*2)/10}" stroke="#14283a"/>`).join('');
      const world = (snapshot.rolling || {}).world_snapshot || {};
      const tracks = Array.isArray(world.semantic_tracks)
        ? world.semantic_tracks
        : Array.isArray((world.semantic_map || {}).tracks) ? world.semantic_map.tracks : [];
      const trackById = new Map(tracks.map((item) => [String(item.track_id || item.object_id || ''), item]));
      const obstacles = (map.obstacles || []).map((o) => {
        const occupancy = String(o.label || '').toLowerCase().includes('occupancy');
        if (occupancy) return `<circle cx="${sx(o.x_m)}" cy="${sy(o.y_m)}" r="2.6" fill="#75899a" opacity=".8"/>`;
        const obstacleWidth = Number(o.width_m || .05) / widthM * (W-pad*2);
        const obstacleHeight = Number(o.height_m || .05) / heightM * (H-pad*2);
        return `<g><rect x="${sx(o.x_m)}" y="${sy(Number(o.y_m) + Number(o.height_m || .05))}" width="${obstacleWidth}" height="${obstacleHeight}" rx="7" fill="#42576b" stroke="#8294a3"/><text x="${sx(o.x_m)+8}" y="${sy(Number(o.y_m) + Number(o.height_m || .05))+20}" fill="#d2dde3" font-size="13">${escapeHtml(o.label || 'obstacle')}</text></g>`;
      }).join('');
      const objects = (map.objects || []).map((o) => {
        const track = trackById.get(String(o.object_id || o.track_id || '')) || {};
        const confidence = finiteNumber(o.confidence ?? track.confidence);
        const uncertaintyM = finiteNumber(o.uncertainty_m ?? track.uncertainty_m);
        const uncertaintyRadius = uncertaintyM !== null ? Math.max(16, uncertaintyM / widthM * (W-pad*2)) : 18;
        const confidenceLabel = confidence !== null ? ` ${Math.round(confidence*100)}%` : '';
        return `<g><circle cx="${sx(o.x_m)}" cy="${sy(o.y_m)}" r="${uncertaintyRadius}" fill="rgba(244,195,106,.08)" stroke="#f4c36a" stroke-dasharray="5 5" opacity=".75"/><circle cx="${sx(o.x_m)}" cy="${sy(o.y_m)}" r="9" fill="#f4c36a"/><text x="${sx(o.x_m)+14}" y="${sy(o.y_m)-11}" fill="#ffe2a5" font-size="14">${escapeHtml(o.label || track.label || 'track')}${escapeHtml(confidenceLabel)}</text></g>`;
      }).join('');
      const goal = map.goal_region ? `<circle cx="${sx(map.goal_region.x_m)}" cy="${sy(map.goal_region.y_m)}" r="${Math.max(7, Number(map.goal_region.radius_m || 0)/widthM*(W-pad*2))}" fill="rgba(125,167,247,.13)" stroke="#7da7f7" stroke-width="3"/>` : '';
      const intent = (snapshot.rolling || {}).active_intent || {};
      let corridorMin = finiteNumber(snapshot.safety.forward_corridor_min_angle_deg);
      let corridorMax = finiteNumber(snapshot.safety.forward_corridor_max_angle_deg);
      if ((corridorMin === null || corridorMax === null) && intent.safe_corridor) {
        const corridor = String(intent.safe_corridor || '');
        [corridorMin, corridorMax] = corridor === 'left' ? [10,55] : corridor === 'right' ? [-55,-10] : [-25,25];
      }
      const clearance = finiteNumber(snapshot.safety.forward_corridor_clearance_m);
      const corridorAvailable = corridorMin !== null && corridorMax !== null && clearance !== null && clearance > 0;
      const corridorRange = corridorAvailable ? Math.max(.1, Math.min(clearance, Math.min(widthM,heightM)*.45)) : 0;
      const endpoint = (offset) => {
        const angle = (Number(rover.yaw_deg || 0) + offset) * Math.PI / 180;
        return `${sx(Number(rover.x_m) + corridorRange*Math.cos(angle))},${sy(Number(rover.y_m) + corridorRange*Math.sin(angle))}`;
      };
      const corridorShape = map.available === false || !corridorAvailable ? '' : `<path d="M ${sx(rover.x_m)} ${sy(rover.y_m)} L ${endpoint(corridorMin)} L ${endpoint(corridorMax)} Z" fill="rgba(125,167,247,.10)" stroke="#7da7f7" stroke-width="2" stroke-dasharray="6 6"/>`;
      const layerNotice = map.navigation_available && map.unavailable_layers && map.unavailable_layers.length ? `<text x="${pad+8}" y="${H-pad-10}" fill="#91a9aa" font-size="15">Unavailable layers: ${escapeHtml(map.unavailable_layers.join(', '))}</text>` : '';
      svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
      const frame = `${grid}<rect x="${pad}" y="${pad}" width="${W-pad*2}" height="${H-pad*2}" fill="none" stroke="#385168" stroke-width="3"/>`;
      const sources = world.sources || {};
      const localizationSource = sources.localization || {};
      const localization = map.localization || world.localization || localizationSource.value || {};
      const localizationState = String(localization.state || (map.available === false ? 'unavailable' : snapshot.adapter.fixture_only ? 'replay' : 'unknown')).toLowerCase();
      const localizationFresh = localizationSource.fresh ?? localization.fresh ?? snapshot.safety.telemetry_fresh;
      const localizationQuality = finiteNumber(localization.quality);
      const qualityLabel = localizationQuality !== null ? `${Math.round(localizationQuality*100)}%` : 'not reported';
      const freshnessLabel = localizationFresh ? 'fresh' : 'stale';
      const mapTone = map.available === false ? 'unavailable' : !localizationFresh ? 'stale' : localizationState === 'degraded' ? 'degraded' : '';
      const sourceLabel = map.source || (snapshot.adapter.fixture_only ? 'replay fixture' : 'authoritative service');
      $('map-status').innerHTML = statusPill('Map', map.available === false ? 'unavailable' : freshnessLabel, mapTone);
      $('map-overlay').innerHTML = [
        statusPill('Localization', localizationState, mapTone),
        statusPill('Quality', qualityLabel, localizationQuality !== null && localizationQuality < .55 ? 'degraded' : ''),
        statusPill('Source', sourceLabel),
        world.snapshot_id ? statusPill('Snapshot', String(world.snapshot_id).slice(0,12)) : '',
      ].join('');
      const alert = $('map-alert');
      alert.hidden = map.available !== false && localizationFresh && localizationState !== 'degraded';
      alert.textContent = map.available === false
        ? map.unavailable_reason || 'Authoritative spatial data unavailable'
        : !localizationFresh ? 'STALE LOCALIZATION — spatial evidence is not current'
        : 'DEGRADED LOCALIZATION — use pose and tracks with caution';
      if (map.available === false) {
        svg.setAttribute('aria-label', 'Authoritative live room map unavailable');
        svg.innerHTML = `${frame}<text x="${W/2}" y="${H/2}" text-anchor="middle" fill="#91a9aa" font-size="20">${escapeHtml(map.unavailable_reason || 'Authoritative map unavailable')}</text>`;
        return;
      }
      svg.setAttribute('aria-label', `Room map showing ${sourceLabel} occupancy, rover heading, safe corridor, route, path, goal, and semantic tracks`);
      svg.innerHTML = `${frame}${corridorShape}${goal}${obstacles}<polyline points="${points(map.proposed_route || [])}" fill="none" stroke="#7da7f7" stroke-width="5" stroke-dasharray="10 10"/>${(map.traveled_path || []).length > 1 ? `<polyline points="${points(map.traveled_path)}" fill="none" stroke="#65e0c2" stroke-width="8" stroke-linecap="round"/>` : ''}${objects}<g transform="translate(${sx(rover.x_m)} ${sy(rover.y_m)}) rotate(${Number(rover.yaw_deg || 0)})"><path d="M 18 0 L -12 -12 L -7 0 L -12 12 Z" fill="#65e0c2" stroke="#d4fff7" stroke-width="2"/></g>${layerNotice}`;
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
        if (shouldContinuouslyPoll(snapshot)) startTimer();
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
      try {
        const snapshot = await api('/api/web/mission/cancel', {method:'POST', body:'{}'});
        render(snapshot);
        if (shouldContinuouslyPoll(snapshot)) startTimer();
      }
      catch (error) { $('request-error').textContent = error.message; }
    }

    async function toggleSensors() {
      if (!current || sensorRequestInFlight) return;
      sensorRequestInFlight = true;
      $('request-error').textContent = '';
      renderSensorControl(current);
      try {
        const active = !Boolean(current.sensor_control && current.sensor_control.active);
        const snapshot = await api('/api/web/stationary-sensors', {
          method:'POST',
          body:JSON.stringify({active}),
        });
        sensorRequestInFlight = false;
        render(snapshot);
        if (shouldContinuouslyPoll(snapshot)) startTimer();
      } catch (error) {
        sensorRequestInFlight = false;
        renderSensorControl(current);
        $('request-error').textContent = error.message;
      }
    }

    function startTimer() {
      stopTimer();
      timer = setInterval(async () => {
        const livePoll = Boolean(current && !current.adapter.fixture_only);
        try {
          render(livePoll
            ? await api('/api/web/state')
            : await api('/api/web/mission/advance', {method:'POST', body:'{}'}));
          $('request-error').textContent = '';
        } catch (error) {
          if (!livePoll) stopTimer();
          $('request-error').textContent = error.message;
        }
      }, 650);
    }
    function stopTimer() { if (timer) clearInterval(timer); timer = null; }

    $('propose').addEventListener('click', propose);
    $('approve').addEventListener('click', approve);
    $('cancel').addEventListener('click', cancelMission);
    $('sensors-toggle').addEventListener('click', toggleSensors);
    $('mission-prompt').addEventListener('input', () => { promptDirty = true; });
    Promise.all([loadScenarios(), api('/api/web/state')]).then(([,snapshot]) => {
      render(snapshot);
      if (shouldContinuouslyPoll(snapshot)) startTimer();
    }).catch((error) => $('request-error').textContent = error.message);
  </script>
</body>
</html>
'''


if __name__ == "__main__":
    raise SystemExit(main())
