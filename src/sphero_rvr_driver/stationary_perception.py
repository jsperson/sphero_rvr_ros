"""Live stationary perception with asynchronous, leased LLM observation intent.

This module is deliberately ROS-free.  A ROS sensor node populates ``LiveStateCache``;
the engine snapshots that cache while a single authenticated provider worker runs.
No type in this module can express motion or acquire physical execution authority.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Any, Callable, Mapping, Optional, Protocol
import uuid

from .live_mission_service import LiveStateCache, snapshot_evidence
from .mission_api import MissionValidationError
from .mission_service import MissionService
from .prompt_drive import ALLOWED_REASONING_EFFORTS, DEFAULT_CODEX_MODEL_ID
from .rolling_replay import canonical_digest


STATIONARY_PROPOSAL_SCHEMA = "sphero_rvr.stationary_perception_proposal.v1"
STATIONARY_INTENT_SCHEMA = "sphero_rvr.stationary_observation_intent.v1"
STATIONARY_RESULT_SCHEMA = "sphero_rvr.stationary_perception_result.v1"
STATIONARY_SNAPSHOT_SCHEMA = "sphero_rvr.stationary_world_snapshot.v1"

_ACTIONS = {"observe", "inspect", "search", "wait", "finish"}
_VIEWPOINTS = {"forward", "left", "right", "wider", "closer", "hold"}
_TERMINAL_STATES = {
    "complete",
    "failed",
    "blocked",
    "cancelled",
    "stopped",
    "estopped",
}


@dataclass(frozen=True)
class StationaryObservationIntent:
    revision: int
    snapshot_id: str
    action: str
    observation_focus: str
    viewpoint_recommendation: str
    search_targets: tuple[str, ...]
    rationale: str
    lease_s: float
    issued_at_s: float
    expires_at_s: float
    provider_id: str
    model_id: str

    @classmethod
    def validated(
        cls,
        raw: Mapping[str, Any],
        *,
        revision: int,
        snapshot: Mapping[str, Any],
        issued_at_s: float,
        provider_id: str,
        model_id: str,
    ) -> "StationaryObservationIntent":
        if str(raw.get("snapshot_id", "")) != str(snapshot.get("snapshot_id", "")):
            raise MissionValidationError(
                "stationary intent is not bound to the exact live world snapshot"
            )
        action = str(raw.get("action", "")).strip().lower()
        viewpoint = str(raw.get("viewpoint_recommendation", "")).strip().lower()
        if action not in _ACTIONS:
            raise MissionValidationError("stationary observation action is invalid")
        if viewpoint not in _VIEWPOINTS:
            raise MissionValidationError("stationary viewpoint recommendation is invalid")
        focus = str(raw.get("observation_focus", "")).strip()
        rationale = str(raw.get("rationale", "")).strip()
        if not focus or len(focus) > 160:
            raise MissionValidationError("stationary observation focus is required and bounded")
        if not rationale or len(rationale) > 800:
            raise MissionValidationError("stationary intent rationale is required and bounded")
        targets_value = raw.get("search_targets", [])
        if not isinstance(targets_value, list) or len(targets_value) > 8:
            raise MissionValidationError("stationary search targets must be a bounded list")
        targets = tuple(str(item).strip() for item in targets_value)
        if any(not item or len(item) > 80 for item in targets):
            raise MissionValidationError("stationary search targets must be non-empty and bounded")
        try:
            lease_s = float(raw.get("lease_s"))
        except (TypeError, ValueError) as exc:
            raise MissionValidationError("stationary intent lease must be finite") from exc
        if lease_s != 90.0:
            raise MissionValidationError("stationary intent lease must be exactly 90 seconds")
        if action == "finish" and revision < 3:
            raise MissionValidationError(
                "stationary terminal intent requires at least three grounded revisions"
            )

        uncertain_track = str(snapshot.get("uncertain_track_id", "")).strip()
        if uncertain_track and (
            focus != uncertain_track or viewpoint not in {"left", "right", "wider", "closer"}
        ):
            raise MissionValidationError(
                "uncertain evidence requires its exact track and another viewpoint"
            )
        return cls(
            revision=revision,
            snapshot_id=str(snapshot["snapshot_id"]),
            action=action,
            observation_focus=focus,
            viewpoint_recommendation=viewpoint,
            search_targets=targets,
            rationale=rationale,
            lease_s=lease_s,
            issued_at_s=float(issued_at_s),
            expires_at_s=float(issued_at_s) + lease_s,
            provider_id=str(provider_id),
            model_id=str(model_id),
        )

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "schema": STATIONARY_INTENT_SCHEMA,
            "revision": self.revision,
            "snapshot_id": self.snapshot_id,
            "action": self.action,
            "observation_focus": self.observation_focus,
            "viewpoint_recommendation": self.viewpoint_recommendation,
            "search_targets": list(self.search_targets),
            "rationale": self.rationale,
            "lease_s": self.lease_s,
            "issued_at_s": self.issued_at_s,
            "expires_at_s": self.expires_at_s,
            "provider_id": self.provider_id,
            "model_id": self.model_id,
            "motion_authority": False,
            "physical_execution_enabled": False,
        }


class StationaryIntentProvider(Protocol):
    provider_id: str
    model_id: str
    reasoning_effort: str

    def revise(
        self, mission: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]: ...


class ScriptedStationaryIntentProvider:
    """Deterministic delayed provider used only by tests."""

    provider_id = "scripted-test-provider"
    model_id = "deterministic-not-live"
    reasoning_effort = "fixture"

    def __init__(self, *, delay_s: float = 0.08) -> None:
        self.delay_s = float(delay_s)

    def revise(
        self, mission: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        del mission
        time.sleep(max(0.0, self.delay_s))
        uncertain = str(snapshot.get("uncertain_track_id", ""))
        return {
            "snapshot_id": snapshot["snapshot_id"],
            "action": "inspect" if uncertain else "observe",
            "observation_focus": uncertain or "live-room-survey",
            "viewpoint_recommendation": "wider" if uncertain else "forward",
            "search_targets": ["shoe", "face"],
            "lease_s": 90.0,
            "rationale": (
                f"Track {uncertain} is uncertain, so a wider view is recommended."
                if uncertain
                else "Fresh lidar and camera evidence support continued stationary observation."
            ),
        }


class CodexOAuthStationaryIntentProvider:
    """Real ChatGPT-OAuth boundary for one typed stationary observation revision."""

    provider_id = "openai-codex-oauth"

    def __init__(
        self,
        *,
        model: Optional[str] = None,
        reasoning_effort: str = "high",
        codex_command: str = "codex",
        timeout_s: float = 90.0,
    ) -> None:
        if reasoning_effort not in ALLOWED_REASONING_EFFORTS:
            raise MissionValidationError(
                f"unsupported reasoning effort: {reasoning_effort}"
            )
        self.model_id = model or os.environ.get("OPENAI_MODEL", DEFAULT_CODEX_MODEL_ID)
        self.reasoning_effort = reasoning_effort
        self.codex_command = str(codex_command)
        self.timeout_s = float(timeout_s)
        self._oauth_checked = False
        self._oauth_lock = threading.Lock()

    def revise(
        self, mission: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        executable = shutil.which(self.codex_command)
        if executable is None:
            raise MissionValidationError(
                "Codex CLI is not installed; stationary perception requires the real OAuth provider"
            )
        environment = dict(os.environ)
        environment.pop("OPENAI_API_KEY", None)
        environment.pop("CODEX_API_KEY", None)
        self._require_chatgpt_oauth(executable, environment)
        with tempfile.TemporaryDirectory(prefix="rvr-stationary-perception-") as directory:
            root = Path(directory)
            schema_path = root / "intent-schema.json"
            output_path = root / "intent.json"
            schema_path.write_text(
                json.dumps(_stationary_intent_output_schema(), sort_keys=True),
                encoding="utf-8",
            )
            command = [
                executable,
                "exec",
                "--ignore-user-config",
                "--ignore-rules",
                "--strict-config",
                "--skip-git-repo-check",
                "--ephemeral",
                "--sandbox",
                "read-only",
                "--disable",
                "shell_tool",
                "--disable",
                "unified_exec",
                "--disable",
                "apps",
                "--disable",
                "multi_agent",
                "--disable",
                "web_search_request",
                "-c",
                'web_search="disabled"',
                "-c",
                f'model_reasoning_effort="{self.reasoning_effort}"',
                "-C",
                str(root),
                "-m",
                self.model_id,
                "--output-schema",
                str(schema_path),
                "--output-last-message",
                str(output_path),
                "-",
            ]
            try:
                completed = subprocess.run(
                    command,
                    input=_stationary_provider_prompt(mission, snapshot),
                    text=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    timeout=self.timeout_s,
                    env=environment,
                    cwd=root,
                    check=False,
                )
            except subprocess.TimeoutExpired as exc:
                raise MissionValidationError(
                    "Codex OAuth stationary-intent call timed out"
                ) from exc
            if completed.returncode != 0:
                detail = str(completed.stderr).strip().splitlines()
                suffix = f": {detail[-1]}" if detail else ""
                raise MissionValidationError(
                    "Codex OAuth stationary-intent call failed with exit code "
                    f"{completed.returncode}{suffix}"
                )
            try:
                payload = json.loads(output_path.read_text(encoding="utf-8"))
            except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise MissionValidationError(
                    "Codex OAuth stationary-intent call returned malformed output"
                ) from exc
        if not isinstance(payload, Mapping):
            raise MissionValidationError(
                "Codex OAuth stationary-intent output must be an object"
            )
        return dict(payload)

    def _require_chatgpt_oauth(
        self, executable: str, environment: Mapping[str, str]
    ) -> None:
        with self._oauth_lock:
            if self._oauth_checked:
                return
            try:
                status = subprocess.run(
                    [executable, "login", "status"],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=15.0,
                    env=dict(environment),
                    check=False,
                )
            except subprocess.TimeoutExpired as exc:
                raise MissionValidationError("Codex OAuth status check timed out") from exc
            if (
                status.returncode != 0
                or "logged in using chatgpt" not in str(status.stdout).lower()
            ):
                raise MissionValidationError(
                    "Codex CLI is not authenticated with ChatGPT OAuth; "
                    "run `codex login --device-auth`"
                )
            self._oauth_checked = True


def _stationary_intent_output_schema() -> dict[str, Any]:
    return {
        "$schema": "http://json-schema.org/draft-07/schema#",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "snapshot_id": {"type": "string"},
            "action": {"type": "string", "enum": sorted(_ACTIONS)},
            "observation_focus": {
                "type": "string",
                "minLength": 1,
                "maxLength": 160,
            },
            "viewpoint_recommendation": {
                "type": "string",
                "enum": sorted(_VIEWPOINTS),
            },
            "search_targets": {
                "type": "array",
                "maxItems": 8,
                "items": {"type": "string", "minLength": 1, "maxLength": 80},
            },
            "lease_s": {"type": "number", "minimum": 90.0, "maximum": 90.0},
            "rationale": {
                "type": "string",
                "minLength": 1,
                "maxLength": 800,
            },
        },
        "required": [
            "snapshot_id",
            "action",
            "observation_focus",
            "viewpoint_recommendation",
            "search_targets",
            "lease_s",
            "rationale",
        ],
    }


def _stationary_provider_prompt(
    mission: str, snapshot: Mapping[str, Any]
) -> str:
    request = {
        "role": "You direct stationary observation for a rover with no motion authority.",
        "operator_mission": str(mission),
        "world_snapshot": dict(snapshot),
        "rules": [
            "Return exactly one finite observation intent bound to world_snapshot.snapshot_id.",
            "The rover is physically stationary. Never emit motion, steering, speed, routes, primitives, cmd_vel, ROS commands, serial access, or claims of execution.",
            "Use lease_s 90.",
            "Use only detection, track, face identity, occupancy, and freshness evidence in the snapshot.",
            "A face label is authoritative only when recognized_from_enrollment is true; every other face remains unknown.",
            "When uncertain_track_id is non-empty, focus on that exact track and recommend left, right, wider, or closer.",
            "Recommend a viewpoint for an operator or later reviewed mission; never claim the rover moved.",
            "Keep observing while sensors are fresh. Do not choose finish merely because one revision is complete.",
            "Ground the concise rationale in evidence IDs and track IDs present in the snapshot.",
        ],
    }
    return json.dumps(request, sort_keys=True, separators=(",", ":"), allow_nan=False)


class StationaryPerceptionEngine:
    """Continuously snapshot live sensors while one provider worker revises intent."""

    def __init__(
        self,
        mission_id: str,
        mission: str,
        provider: StationaryIntentProvider,
        cache: LiveStateCache,
        *,
        tick_s: float = 0.2,
        max_source_age_s: float = 1.5,
        checkpoint: Optional[Callable[[str, Mapping[str, Any]], None]] = None,
    ) -> None:
        self.mission_id = str(mission_id)
        self.mission = str(mission)
        self.provider = provider
        self.cache = cache
        self.tick_s = float(tick_s)
        self.max_source_age_s = float(max_source_age_s)
        if self.tick_s <= 0.0 or self.max_source_age_s <= 0.0:
            raise ValueError("stationary tick and freshness limit must be positive")
        self._checkpoint = checkpoint
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="stationary-intent-provider"
        )
        self._future: Optional[Future[Mapping[str, Any]]] = None
        self._future_snapshot: Optional[dict[str, Any]] = None
        self._started_monotonic = time.monotonic()
        self._first_intent_deadline = time.time() + 90.0
        self._tick = 0
        self._active_intent: Optional[StationaryObservationIntent] = None
        self._revisions: list[dict[str, Any]] = []
        self._decision_snapshots: list[dict[str, Any]] = []
        self._events: list[dict[str, Any]] = []
        self._provider_calls_started = 0
        self._provider_calls_completed = 0
        self._provider_rejections = 0
        self._sensor_updates = 0
        self._sensor_updates_while_inference = 0
        self._camera_updates = 0
        self._lidar_updates = 0
        self._map_updates = 0
        self._last_versions: dict[str, Any] = {}
        self._terminal = False
        self._terminal_state = ""
        self._terminal_reason = ""
        self._latest_world = self._build_world_snapshot()
        self._append_event(
            "stationary_ready",
            "Live stationary perception initialized with no motion authority.",
        )

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                raise RuntimeError("stationary perception already started")
            initial_reason = self._sensor_terminal_reason(self._latest_world)
            if initial_reason:
                raise MissionValidationError(
                    f"stationary perception sensors are not fresh: {initial_reason}"
                )
            self._thread = threading.Thread(
                target=self._run,
                name=f"stationary-perception-{self.mission_id}",
                daemon=True,
            )
            self._thread.start()

    def cancel(self) -> None:
        with self._lock:
            if not self._terminal:
                self._finish("cancelled", "operator_cancelled")

    def stop(self) -> None:
        with self._lock:
            if not self._terminal:
                self._finish("stopped", "stop_requested")

    def estop(self) -> None:
        with self._lock:
            if not self._terminal:
                self._finish("estopped", "estop_latched")

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        self._pool.shutdown(wait=False, cancel_futures=True)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._projection()

    def _run(self) -> None:
        try:
            while not self._stop.wait(self.tick_s):
                with self._lock:
                    if self._terminal:
                        break
                    self._tick += 1
                    self._latest_world = self._build_world_snapshot()
                    self._measure_sensor_updates()
                    reason = self._sensor_terminal_reason(self._latest_world)
                    if reason:
                        self._finish("blocked", reason)
                        continue
                    control_reason = self._control_terminal_reason(self._latest_world)
                    if control_reason:
                        state = "estopped" if control_reason == "estop_latched" else "stopped"
                        self._finish(state, control_reason)
                        continue
                    deadline = (
                        self._first_intent_deadline
                        if self._active_intent is None
                        else self._active_intent.expires_at_s
                    )
                    if time.time() >= deadline:
                        self._finish("blocked", "intent_lease_expired")
                        continue
                    self._collect_provider_result()
                    if self._terminal:
                        continue
                    if self._future is None and self._provider_calls_started < 12:
                        self._dispatch_provider_call()
                    if self._tick % max(1, round(1.0 / self.tick_s)) == 0:
                        self._emit_checkpoint("world_snapshot", self._projection())
        finally:
            self._stop.set()

    def _dispatch_provider_call(self) -> None:
        snapshot = json.loads(json.dumps(self._latest_world))
        self._provider_calls_started += 1
        self._future_snapshot = snapshot
        self._decision_snapshots.append(snapshot)
        self._future = self._pool.submit(self.provider.revise, self.mission, snapshot)
        self._append_event(
            "llm_call_started",
            f"Provider call {self._provider_calls_started} started from snapshot "
            f"{snapshot['snapshot_id'][:12]} while live sensors continued.",
        )
        self._emit_checkpoint("llm_call_started", self._projection())

    def _collect_provider_result(self) -> None:
        if self._future is None or not self._future.done():
            return
        future = self._future
        source_snapshot = self._future_snapshot
        self._future = None
        self._future_snapshot = None
        self._provider_calls_completed += 1
        assert source_snapshot is not None
        try:
            intent = StationaryObservationIntent.validated(
                future.result(),
                revision=len(self._revisions) + 1,
                snapshot=source_snapshot,
                issued_at_s=time.time(),
                provider_id=self.provider.provider_id,
                model_id=self.provider.model_id,
            )
        except Exception as exc:
            self._provider_rejections += 1
            self._append_event(
                "llm_response_rejected",
                f"Provider response rejected by deterministic validation: {exc}",
            )
            self._emit_checkpoint("llm_response_rejected", self._projection())
            if self._provider_calls_started >= 12:
                self._finish("failed", "provider_validation_exhausted")
            return
        self._active_intent = intent
        revision = intent.to_json_dict()
        revision["applied_tick"] = self._tick
        revision["sensor_updates_during_call"] = self._sensor_updates_while_inference
        self._revisions.append(revision)
        self._append_event(
            "intent_revision",
            f"Revision {intent.revision} atomically selected {intent.action}: "
            f"{intent.observation_focus} / {intent.viewpoint_recommendation}.",
        )
        self._emit_checkpoint("intent_revision", self._projection())
        if intent.action == "finish":
            self._finish("complete", "llm_terminal_intent")

    def _build_world_snapshot(self) -> dict[str, Any]:
        evidence = snapshot_evidence(
            self.cache.snapshot(), max_age_s=self.max_source_age_s
        )
        sources: dict[str, Any] = {}
        for name in ("camera", "lidar", "localization", "semantic_map", "control"):
            source = evidence[name]
            value = dict(source.get("value", {}))
            value.pop("thumbnail_data_url", None)
            sources[name] = {
                "present": bool(source.get("present", False)),
                "valid": bool(source.get("valid", False)),
                "fresh": bool(source.get("fresh", False)),
                "age_s": source.get("age_s"),
                "received_at_s": source.get("received_at_s"),
                "source_timestamp_s": source.get("source_timestamp_s"),
                "error": str(source.get("error", "")),
                "value": value,
            }
        camera_value = sources["camera"]["value"]
        semantic_value = sources["semantic_map"]["value"]
        detections = camera_value.get("detections", [])
        tracks = semantic_value.get("tracks", [])
        uncertain = str(
            camera_value.get("uncertain_track_id")
            or semantic_value.get("uncertain_track_id")
            or ""
        )
        payload: dict[str, Any] = {
            "schema": STATIONARY_SNAPSHOT_SCHEMA,
            "mission_id": self.mission_id,
            "mission": self.mission,
            "version": self._tick,
            "observed_at_s": evidence["observed_at_s"],
            "sources": sources,
            "occupancy": semantic_value.get(
                "occupancy",
                sources["lidar"]["value"].get("raw_scan_occupancy_preview", {}),
            ),
            "detections": detections if isinstance(detections, list) else [],
            "semantic_tracks": tracks if isinstance(tracks, list) else [],
            "uncertain_track_id": uncertain,
            "active_intent_revision": (
                None if self._active_intent is None else self._active_intent.revision
            ),
            "safety": {
                "motion_authority": False,
                "physical_execution_enabled": False,
                "stationary": True,
            },
        }
        payload["snapshot_id"] = canonical_digest(payload)
        return payload

    def _measure_sensor_updates(self) -> None:
        sources = self._latest_world["sources"]
        versions = {
            "camera": sources["camera"]["value"].get("frame_id"),
            "lidar": sources["lidar"]["value"].get("scan_id"),
            "localization": sources["localization"]["value"].get("scan_id"),
            "semantic_map": sources["semantic_map"]["value"].get("revision"),
        }
        changed = False
        for name, version in versions.items():
            if version is not None and version != self._last_versions.get(name):
                changed = True
                if name == "camera":
                    self._camera_updates += 1
                elif name == "lidar":
                    self._lidar_updates += 1
                elif name == "semantic_map":
                    self._map_updates += 1
        if changed:
            self._sensor_updates += 1
            if self._future is not None:
                self._sensor_updates_while_inference += 1
        self._last_versions = versions

    @staticmethod
    def _sensor_terminal_reason(snapshot: Mapping[str, Any]) -> str:
        sources = snapshot.get("sources", {})
        for name in ("lidar", "localization", "camera", "semantic_map"):
            source = sources.get(name, {}) if isinstance(sources, Mapping) else {}
            if not isinstance(source, Mapping) or not bool(source.get("fresh", False)):
                return f"{name}_stale"
        localization = sources["localization"].get("value", {})
        state = (
            str(localization.get("state", "")).lower()
            if isinstance(localization, Mapping)
            else ""
        )
        if state not in {"valid", "degraded"}:
            return "localization_lost"
        return ""

    @staticmethod
    def _control_terminal_reason(snapshot: Mapping[str, Any]) -> str:
        sources = snapshot.get("sources", {})
        control = sources.get("control", {}) if isinstance(sources, Mapping) else {}
        if not isinstance(control, Mapping) or not bool(control.get("fresh", False)):
            return ""
        value = control.get("value", {})
        if not isinstance(value, Mapping):
            return ""
        state = str(value.get("state", "")).upper()
        if bool(value.get("estop_latched", False)) or state in {
            "ESTOP",
            "ESTOPPED",
            "LATCHED",
        }:
            return "estop_latched"
        if bool(value.get("stop_active", False)) or state in {
            "STOP",
            "STOPPED",
            "CANCEL",
            "CANCELLED",
        }:
            return "stop_requested"
        return ""

    def _projection(self) -> dict[str, Any]:
        result = self._terminal_result() if self._terminal else {}
        return {
            "schema": STATIONARY_RESULT_SCHEMA,
            "mission_id": self.mission_id,
            "status": self._terminal_state or "running",
            "terminal": self._terminal,
            "terminal_reason": self._terminal_reason,
            "progress": min(0.99, len(self._revisions) / 4.0)
            if not self._terminal
            else 1.0,
            "world_snapshot": json.loads(json.dumps(self._latest_world)),
            "decision_snapshots": json.loads(json.dumps(self._decision_snapshots)),
            "active_intent": (
                None
                if self._active_intent is None
                else self._active_intent.to_json_dict()
            ),
            "intent_revisions": json.loads(json.dumps(self._revisions)),
            "inference": {
                "in_flight": self._future is not None,
                "call": self._provider_calls_started if self._future is not None else None,
                "snapshot_id": (
                    ""
                    if self._future_snapshot is None
                    else self._future_snapshot["snapshot_id"]
                ),
                "provider_calls_started": self._provider_calls_started,
                "provider_calls_completed": self._provider_calls_completed,
                "provider_rejections": self._provider_rejections,
                "sensor_updates_during_calls": self._sensor_updates_while_inference,
            },
            "metrics": self._metrics(),
            "events": json.loads(json.dumps(self._events)),
            "motion_authority": False,
            "physical_execution_enabled": False,
            "result": result,
        }

    def _metrics(self) -> dict[str, Any]:
        tracks = self._latest_world.get("semantic_tracks", [])
        identities = [
            track
            for track in tracks
            if isinstance(track, Mapping)
            and track.get("kind") == "face"
            and track.get("label") != "unknown"
        ]
        unknown_faces = [
            track
            for track in tracks
            if isinstance(track, Mapping)
            and track.get("kind") == "face"
            and track.get("label") == "unknown"
        ]
        return {
            "sensor_updates": self._sensor_updates,
            "camera_updates": self._camera_updates,
            "lidar_updates": self._lidar_updates,
            "semantic_map_updates": self._map_updates,
            "sensor_updates_while_llm_in_flight": self._sensor_updates_while_inference,
            "intent_revision_count": len(self._revisions),
            "enrolled_face_track_count": len(identities),
            "unknown_face_track_count": len(unknown_faces),
            "stable_track_ids": sorted(
                str(track.get("track_id", ""))
                for track in tracks
                if isinstance(track, Mapping) and track.get("track_id")
            ),
            "stationary": True,
            "motion_authority": False,
            "physical_execution_enabled": False,
        }

    def _terminal_result(self) -> dict[str, Any]:
        return {
            "schema": STATIONARY_RESULT_SCHEMA,
            "mission_id": self.mission_id,
            "status": self._terminal_state,
            "terminal_reason": self._terminal_reason,
            "final_snapshot": json.loads(json.dumps(self._latest_world)),
            "decision_snapshots": json.loads(json.dumps(self._decision_snapshots)),
            "intent_revisions": json.loads(json.dumps(self._revisions)),
            "detections": self._latest_world.get("detections", []),
            "semantic_tracks": self._latest_world.get("semantic_tracks", []),
            "metrics": self._metrics(),
            "motion_authority": False,
            "physical_execution_enabled": False,
        }

    def _finish(self, state: str, reason: str) -> None:
        if self._terminal:
            return
        self._terminal = True
        self._terminal_state = str(state)
        self._terminal_reason = str(reason)
        self._append_event(
            "terminal",
            f"Stationary perception terminated deterministically: {reason}.",
        )
        self._emit_checkpoint("terminal", self._projection())

    def _append_event(self, kind: str, message: str) -> None:
        self._events.append(
            {
                "sequence": len(self._events) + 1,
                "event_type": str(kind),
                "message": str(message),
                "tick": self._tick,
                "elapsed_s": max(0.0, time.monotonic() - self._started_monotonic),
            }
        )

    def _emit_checkpoint(self, kind: str, payload: Mapping[str, Any]) -> None:
        if self._checkpoint is None:
            return
        try:
            self._checkpoint(kind, payload)
        except Exception as exc:
            if not self._terminal:
                self._terminal = True
                self._terminal_state = "failed"
                self._terminal_reason = "persistence_checkpoint_failed"
                self._append_event(
                    "persistence_failure",
                    f"Mission persistence failed closed: {exc.__class__.__name__}.",
                )


class StationaryPerceptionController:
    """MissionService controller for live sensing with permanently absent motion."""

    def __init__(
        self,
        service: MissionService,
        provider: StationaryIntentProvider,
        cache: LiveStateCache,
        *,
        tick_s: float = 0.2,
        max_source_age_s: float = 1.5,
    ) -> None:
        if service.mode != "live" or service.live_execution_enabled:
            raise MissionValidationError(
                "stationary perception requires live mode with physical execution disabled"
            )
        self.service = service
        self.provider = provider
        self.cache = cache
        self.tick_s = float(tick_s)
        self.max_source_age_s = float(max_source_age_s)
        self._lock = threading.RLock()
        self._engines: dict[str, StationaryPerceptionEngine] = {}
        self._closed = False

    def submit(
        self,
        prompt: str,
        *,
        session_id: str,
        source: str = "web",
        mission_id: Optional[str] = None,
    ) -> dict[str, Any]:
        objective = str(prompt).strip()
        if not objective:
            raise MissionValidationError("stationary perception mission is required")
        with self._lock:
            self._ensure_open()
            identifier = str(mission_id or f"stationary-{uuid.uuid4().hex}")
            self.service.begin_prompt_mission(
                mission_id=identifier,
                session_id=session_id,
                prompt=objective,
                source=source,
            )
            proposal_body = {
                "schema": STATIONARY_PROPOSAL_SCHEMA,
                "mission_id": identifier,
                "prompt": objective,
                "source_sha": self.service.source_sha,
                "provider_id": self.provider.provider_id,
                "model_id": self.provider.model_id,
                "reasoning_effort": self.provider.reasoning_effort,
                "decision": "propose",
                "summary": (
                    "Run continuous live stationary lidar/camera perception while "
                    "the authenticated LLM revises leased observation intent."
                ),
                "segments": [],
                "limits": {
                    "motion_authority": False,
                    "physical_execution_enabled": False,
                    "lease_s": 90,
                    "max_provider_calls": 12,
                },
                "contract": {
                    "live_sensors": True,
                    "asynchronous": True,
                    "fixed_route": False,
                    "motion_authority": False,
                    "physical_execution_enabled": False,
                },
            }
            proposal = {
                **proposal_body,
                "proposal_digest": canonical_digest(proposal_body),
            }
            return self.service.record_stationary_perception_proposal(
                identifier, proposal
            )

    def approve(
        self,
        mission_id: str,
        *,
        supplied_approval: str,
        operator: str,
        authentication_source: str = "",
    ) -> dict[str, Any]:
        del authentication_source
        with self._lock:
            self._ensure_open()
            current = self.service.prompt_status(mission_id)
            proposal = current.get("proposal", {})
            if not isinstance(proposal, Mapping):
                raise MissionValidationError("stationary perception proposal is unavailable")
            digest = str(proposal.get("proposal_digest", ""))
            expected = f"APPROVE STATIONARY PERCEPTION {digest}"
            if str(supplied_approval).strip() != expected:
                raise MissionValidationError(
                    "stationary perception approval does not match the persisted proposal"
                )
            engine = StationaryPerceptionEngine(
                mission_id,
                str(current["prompt"]),
                self.provider,
                self.cache,
                tick_s=self.tick_s,
                max_source_age_s=self.max_source_age_s,
                checkpoint=lambda kind, payload: self._persist(
                    mission_id, kind, payload
                ),
            )
            engine.start()
            self.service.approve_stationary_perception_mission(
                mission_id,
                proposal_digest=digest,
                operator=operator,
            )
            self._engines[mission_id] = engine
            return self.service.prompt_status(mission_id)

    def status(self, mission_id: str) -> dict[str, Any]:
        return self.service.prompt_status(mission_id)

    def cancel(
        self, mission_id: str, *, reason: str = "operator cancelled mission"
    ) -> dict[str, Any]:
        with self._lock:
            status = str(self.service.prompt_status(mission_id)["status"])
            engine = self._engines.get(mission_id)
            if status == "running" and engine is not None:
                engine.cancel()
                return self.service.prompt_status(mission_id)
            return self.service.cancel_prompt_mission(mission_id, reason=reason)

    def service_snapshot(self) -> dict[str, Any]:
        return {
            "api_version": "mission_api.v2",
            "mode": self.service.mode,
            "source_sha": self.service.source_sha,
            "deployed_sha": self.service.deployed_sha,
            "planning_enabled": True,
            "stationary_perception_enabled": True,
            "live_execution_enabled": False,
            "physical_execution_enabled": False,
            "motion_authority": False,
            "direct_ros_commands_allowed": False,
            "credentials_accepted_over_service": False,
            "provider_id": self.provider.provider_id,
            "model_id": self.provider.model_id,
            "reasoning_effort": self.provider.reasoning_effort,
            "capabilities": self.service.capabilities(),
        }

    def close(self, *, timeout_s: float = 5.0) -> None:
        del timeout_s
        with self._lock:
            self._closed = True
            engines = tuple(self._engines.values())
            self._engines.clear()
        for engine in engines:
            engine.close()

    def _persist(
        self, mission_id: str, kind: str, projection: Mapping[str, Any]
    ) -> None:
        if kind == "terminal":
            result = projection.get("result", {})
            if not isinstance(result, Mapping):
                raise MissionValidationError(
                    "stationary terminal result is unavailable"
                )
            self.service.finish_stationary_perception_mission(
                mission_id,
                status=str(projection.get("status", "failed")),
                reason=str(projection.get("terminal_reason", "")),
                result=result,
            )
            return
        self.service.record_stationary_perception_checkpoint(
            mission_id,
            kind=kind,
            checkpoint={
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
            },
        )

    def _ensure_open(self) -> None:
        if self._closed:
            raise MissionValidationError("stationary perception controller is closed")
