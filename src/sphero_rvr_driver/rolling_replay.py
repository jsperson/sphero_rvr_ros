"""Concurrent no-authority replay for rolling LLM navigation intents.

The simulator, perception model, and provider worker are deliberately separate:
the simulator keeps updating while a provider call is in flight.  The model
selects one finite leased navigation/observation intent at a time; deterministic
code validates and atomically swaps it, and freshness or safety can stop without
waiting for another model response.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Any, Callable, Mapping, Optional, Protocol

from .mission_api import MissionValidationError
from .perception_navigation import LocalizationEstimate, LocalizationState, Pose2D
from .prompt_drive import ALLOWED_REASONING_EFFORTS, DEFAULT_CODEX_MODEL_ID


WORLD_SNAPSHOT_SCHEMA = "sphero_rvr.world_snapshot.v1"
ROLLING_INTENT_SCHEMA = "sphero_rvr.rolling_intent.v1"
ROLLING_RESULT_SCHEMA = "sphero_rvr.rolling_replay_result.v1"
ROLLING_PROPOSAL_SCHEMA = "sphero_rvr.rolling_replay_proposal.v1"

_DIRECTIONS = {"forward", "left", "right", "hold"}
_CORRIDORS = {"center", "left", "right", "hold"}
_VIEWPOINTS = {"forward", "alternate_left", "alternate_right", "hold"}


def canonical_digest(payload: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        dict(payload), sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class RollingIntent:
    revision: int
    snapshot_id: str
    goal_direction: str
    steering: float
    speed_limit_mps: float
    safe_corridor: str
    lease_s: float
    observation_focus: str
    viewpoint: str
    rationale: str
    issued_at_s: float
    expires_at_s: float
    provider_id: str
    model_id: str
    remaining_distance_budget_m: float
    remaining_observation_budget: int

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
    ) -> "RollingIntent":
        if raw.get("snapshot_id") != snapshot.get("snapshot_id"):
            raise MissionValidationError(
                "rolling intent is not bound to the exact world snapshot"
            )
        direction = str(raw.get("goal_direction", "")).strip()
        corridor = str(raw.get("safe_corridor", "")).strip()
        viewpoint = str(raw.get("viewpoint", "")).strip()
        if direction not in _DIRECTIONS:
            raise MissionValidationError("rolling intent goal direction is invalid")
        if corridor not in _CORRIDORS:
            raise MissionValidationError("rolling intent safe corridor is invalid")
        if viewpoint not in _VIEWPOINTS:
            raise MissionValidationError("rolling intent viewpoint is invalid")
        steering = _finite(raw.get("steering"), "rolling intent steering")
        speed = _finite(raw.get("speed_limit_mps"), "rolling intent speed limit")
        lease = _finite(raw.get("lease_s"), "rolling intent lease")
        if not -1.0 <= steering <= 1.0:
            raise MissionValidationError("rolling intent steering exceeds [-1, 1]")
        if not 0.0 <= speed <= 0.18:
            raise MissionValidationError("rolling intent speed exceeds replay limit")
        if not 8.0 <= lease <= 90.0:
            raise MissionValidationError("rolling intent lease must be 8 to 90 seconds")
        rationale = str(raw.get("rationale", "")).strip()
        focus = str(raw.get("observation_focus", "")).strip()
        if not rationale or len(rationale) > 600:
            raise MissionValidationError("rolling intent rationale is required and bounded")
        if not focus or len(focus) > 120:
            raise MissionValidationError(
                "rolling intent observation focus is required and bounded"
            )

        obstacles = snapshot.get("obstacles", {})
        if isinstance(obstacles, Mapping) and obstacles.get("ahead", False):
            left = float(obstacles.get("left_clearance_m", 0.0))
            right = float(obstacles.get("right_clearance_m", 0.0))
            if left >= right and not (steering >= 0.2 and corridor == "left"):
                raise MissionValidationError(
                    "obstacle snapshot requires steering into the clear left corridor"
                )
            if right > left and not (steering <= -0.2 and corridor == "right"):
                raise MissionValidationError(
                    "obstacle snapshot requires steering into the clear right corridor"
                )

        camera = snapshot.get("camera", {})
        uncertain_track = (
            camera.get("uncertain_track_id", "")
            if isinstance(camera, Mapping)
            else ""
        )
        if uncertain_track and (
            focus != uncertain_track or viewpoint != "alternate_left"
        ):
            raise MissionValidationError(
                "uncertain visual evidence requires the alternate-left track viewpoint"
            )

        budgets = snapshot.get("budgets", {})
        if not isinstance(budgets, Mapping):
            budgets = {}
        return cls(
            revision=revision,
            snapshot_id=str(snapshot["snapshot_id"]),
            goal_direction=direction,
            steering=steering,
            speed_limit_mps=speed,
            safe_corridor=corridor,
            lease_s=lease,
            observation_focus=focus,
            viewpoint=viewpoint,
            rationale=rationale,
            issued_at_s=issued_at_s,
            expires_at_s=issued_at_s + lease,
            provider_id=provider_id,
            model_id=model_id,
            remaining_distance_budget_m=max(
                0.0, float(budgets.get("remaining_distance_m", 0.0))
            ),
            remaining_observation_budget=max(
                0, int(budgets.get("remaining_observations", 0))
            ),
        )

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "schema": ROLLING_INTENT_SCHEMA,
            "revision": self.revision,
            "snapshot_id": self.snapshot_id,
            "goal_direction": self.goal_direction,
            "steering": self.steering,
            "speed_limit_mps": self.speed_limit_mps,
            "safe_corridor": self.safe_corridor,
            "lease_s": self.lease_s,
            "issued_at_s": self.issued_at_s,
            "expires_at_s": self.expires_at_s,
            "observation_focus": self.observation_focus,
            "viewpoint": self.viewpoint,
            "rationale": self.rationale,
            "provider_id": self.provider_id,
            "model_id": self.model_id,
            "budgets": {
                "remaining_distance_m": self.remaining_distance_budget_m,
                "remaining_observations": self.remaining_observation_budget,
            },
        }


class RollingIntentProvider(Protocol):
    provider_id: str
    model_id: str
    reasoning_effort: str

    def revise(
        self, mission: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]: ...


class ScriptedRollingIntentProvider:
    """Delayed deterministic provider for concurrency and safety tests only."""

    provider_id = "scripted-test-provider"
    model_id = "deterministic-not-live"
    reasoning_effort = "fixture"

    def __init__(self, *, delay_s: float = 0.12) -> None:
        self.delay_s = float(delay_s)

    def revise(
        self, mission: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        del mission
        time.sleep(max(0.0, self.delay_s))
        obstacles = snapshot["obstacles"]
        camera = snapshot["camera"]
        uncertain = str(camera.get("uncertain_track_id", ""))
        if obstacles["ahead"]:
            steering, corridor = 0.62, "left"
            rationale = "The forward obstacle makes the wider left corridor safer."
        else:
            steering, corridor = 0.0, "center"
            rationale = "Fresh localization and the center corridor support continued progress."
        return {
            "snapshot_id": snapshot["snapshot_id"],
            "goal_direction": "left" if steering > 0.0 else "forward",
            "steering": steering,
            "speed_limit_mps": 0.12,
            "safe_corridor": corridor,
            "lease_s": 30.0,
            "observation_focus": uncertain or "forward_search",
            "viewpoint": "alternate_left" if uncertain else "forward",
            "rationale": (
                "Inspect the uncertain shoe track from the requested alternate-left viewpoint "
                "while preserving safe rolling motion."
                if uncertain
                else rationale
            ),
        }


class CodexOAuthRollingIntentProvider:
    """Real ChatGPT-OAuth Codex boundary for one structured rolling revision."""

    provider_id = "openai-codex-oauth"

    def __init__(
        self,
        *,
        model: Optional[str] = None,
        reasoning_effort: str = "low",
        codex_command: str = "codex",
        timeout_s: float = 90.0,
    ) -> None:
        if reasoning_effort not in ALLOWED_REASONING_EFFORTS:
            raise MissionValidationError(
                f"unsupported reasoning effort: {reasoning_effort}"
            )
        self.model_id = model or os.environ.get(
            "OPENAI_MODEL", DEFAULT_CODEX_MODEL_ID
        )
        self.reasoning_effort = reasoning_effort
        self.codex_command = codex_command
        self.timeout_s = float(timeout_s)
        self._oauth_checked = False
        self._oauth_lock = threading.Lock()

    def revise(
        self, mission: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        executable = shutil.which(self.codex_command)
        if executable is None:
            raise MissionValidationError(
                "Codex CLI is not installed; rolling replay requires the real OAuth provider"
            )
        env = dict(os.environ)
        env.pop("OPENAI_API_KEY", None)
        env.pop("CODEX_API_KEY", None)
        self._require_chatgpt_oauth(executable, env)
        with tempfile.TemporaryDirectory(prefix="rvr-rolling-replay-") as directory:
            root = Path(directory)
            schema_path = root / "intent-schema.json"
            output_path = root / "intent.json"
            schema_path.write_text(
                json.dumps(_rolling_intent_output_schema(), sort_keys=True),
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
            prompt = _rolling_provider_prompt(mission, snapshot)
            try:
                completed = subprocess.run(
                    command,
                    input=prompt,
                    text=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    timeout=self.timeout_s,
                    env=env,
                    cwd=root,
                    check=False,
                )
            except subprocess.TimeoutExpired as exc:
                raise MissionValidationError(
                    "Codex OAuth rolling-intent call timed out"
                ) from exc
            if completed.returncode != 0:
                detail = str(completed.stderr).strip().splitlines()
                suffix = f": {detail[-1]}" if detail else ""
                raise MissionValidationError(
                    f"Codex OAuth rolling-intent call failed with exit code "
                    f"{completed.returncode}{suffix}"
                )
            try:
                payload = json.loads(output_path.read_text(encoding="utf-8"))
            except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise MissionValidationError(
                    "Codex OAuth rolling-intent call returned malformed output"
                ) from exc
        if not isinstance(payload, Mapping):
            raise MissionValidationError(
                "Codex OAuth rolling-intent output must be an object"
            )
        return dict(payload)

    def _require_chatgpt_oauth(
        self, executable: str, env: Mapping[str, str]
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
                    env=dict(env),
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


def _rolling_intent_output_schema() -> dict[str, Any]:
    return {
        "$schema": "http://json-schema.org/draft-07/schema#",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "snapshot_id": {"type": "string"},
            "goal_direction": {
                "type": "string",
                "enum": sorted(_DIRECTIONS),
            },
            "steering": {"type": "number", "minimum": -1.0, "maximum": 1.0},
            "speed_limit_mps": {
                "type": "number",
                "minimum": 0.0,
                "maximum": 0.18,
            },
            "safe_corridor": {
                "type": "string",
                "enum": sorted(_CORRIDORS),
            },
            "lease_s": {"type": "number", "minimum": 90.0, "maximum": 90.0},
            "observation_focus": {
                "type": "string",
                "minLength": 1,
                "maxLength": 120,
            },
            "viewpoint": {
                "type": "string",
                "enum": sorted(_VIEWPOINTS),
            },
            "rationale": {
                "type": "string",
                "minLength": 1,
                "maxLength": 600,
            },
        },
        "required": [
            "snapshot_id",
            "goal_direction",
            "steering",
            "speed_limit_mps",
            "safe_corridor",
            "lease_s",
            "observation_focus",
            "viewpoint",
            "rationale",
        ],
    }


def _rolling_provider_prompt(
    mission: str, snapshot: Mapping[str, Any]
) -> str:
    request = {
        "role": "You are the navigation-decision driver for a replay rover.",
        "operator_mission": mission,
        "world_snapshot": dict(snapshot),
        "rules": [
            "Return exactly one finite rolling intent bound to world_snapshot.snapshot_id.",
            "Do not emit a route, primitives, motor commands, ROS, files, credentials, or claims of execution.",
            "Use lease_s 90 so the current intent remains finite through one bounded OAuth call and a validation retry.",
            "Keep safe rolling motion when localization is fresh; do not stop merely to think or observe.",
            "When obstacles.ahead is true, you MUST set goal_direction left, steering at least 0.2, and safe_corridor left because this replay snapshot identifies left as the larger candidate clearance.",
            "When camera.uncertain_track_id is non-empty, set observation_focus to that exact ID and viewpoint to alternate_left while retaining safe navigation.",
            "Ground the concise rationale only in IDs and facts present in the snapshot.",
        ],
    }
    return json.dumps(request, sort_keys=True, separators=(",", ":"), allow_nan=False)


class RollingReplayEngine:
    """One continuously ticking replay with a separate asynchronous LLM worker."""

    def __init__(
        self,
        mission_id: str,
        mission: str,
        provider: RollingIntentProvider,
        *,
        tick_s: float = 0.1,
        checkpoint: Optional[Callable[[str, Mapping[str, Any]], None]] = None,
    ) -> None:
        self.mission_id = str(mission_id)
        self.mission = str(mission)
        self.provider = provider
        self.tick_s = float(tick_s)
        if self.tick_s <= 0.0:
            raise ValueError("rolling replay tick must be positive")
        self._checkpoint = checkpoint
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="rolling-intent-provider"
        )
        self._future: Optional[Future[Mapping[str, Any]]] = None
        self._future_snapshot: Optional[dict[str, Any]] = None
        self._future_call = 0
        self._future_started_tick = 0
        self._started_monotonic = time.monotonic()
        self._tick = 0
        self._x_m = 0.35
        self._y_m = 1.45
        self._yaw_rad = 0.0
        self._distance_m = 0.0
        self._path: list[dict[str, float]] = [
            {"x_m": self._x_m, "y_m": self._y_m}
        ]
        self._events: list[dict[str, Any]] = []
        self._decision_snapshots: list[dict[str, Any]] = []
        self._revisions: list[dict[str, Any]] = []
        self._active_intent: Optional[RollingIntent] = None
        self._command_speed_mps = 0.0
        self._command_samples: list[dict[str, Any]] = []
        self._provider_calls_started = 0
        self._provider_calls_completed = 0
        self._provider_rejections = 0
        self._motion_updates_while_inference = 0
        self._perception_updates = 0
        self._mapping_updates = 0
        self._compatible_swaps = 0
        self._zero_motion_gaps = 0
        self._obstacle_present = False
        self._obstacle_x_m = 1.55
        self._uncertain_visible = False
        self._obstacle_steering_changed = False
        self._uncertain_focus_changed = False
        self._localization_frozen_at_s: Optional[float] = None
        self._localization_state = LocalizationState.VALID
        self._collision_state = "CLEAR"
        self._stop_active = False
        self._estop_latched = False
        self._terminal = False
        self._terminal_state = ""
        self._terminal_reason = ""
        self._latest_world = self._build_world_snapshot()
        self._append_event(
            "replay_ready",
            "Continuous replay perception initialized with no motion authority.",
        )

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                raise RuntimeError("rolling replay already started")
            self._thread = threading.Thread(
                target=self._run,
                name=f"rolling-replay-{self.mission_id}",
                daemon=True,
            )
            self._thread.start()

    def cancel(self) -> None:
        with self._lock:
            if not self._terminal:
                self._finish("cancelled", "operator_cancelled")

    def collision_stop(self) -> None:
        with self._lock:
            if not self._terminal:
                self._collision_state = "BLOCKED"
                self._finish("blocked", "collision_veto")

    def stop(self) -> None:
        with self._lock:
            if not self._terminal:
                self._stop_active = True
                self._finish("stopped", "stop_requested")

    def estop(self) -> None:
        with self._lock:
            if not self._terminal:
                self._estop_latched = True
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
                checkpoint: Optional[tuple[str, dict[str, Any]]] = None
                with self._lock:
                    if self._terminal:
                        break
                    self._tick_once()
                    checkpoint = self._collect_provider_result()
                    if self._terminal:
                        pass
                    elif (
                        self._active_intent is not None
                        and self._elapsed_s() >= self._active_intent.expires_at_s
                    ):
                        self._finish("blocked", "intent_lease_expired")
                    elif self._localization_state is LocalizationState.STALE:
                        self._finish("blocked", "localization_stale")
                    elif self._future is None and len(self._revisions) < 4:
                        self._dispatch_provider_call()
                    elif len(self._revisions) >= 4:
                        fourth_tick = int(self._revisions[3]["applied_tick"])
                        if self._tick - fourth_tick >= 8:
                            self._localization_state = LocalizationState.STALE
                            self._localization_frozen_at_s = self._elapsed_s() - 1.0
                            self._command_speed_mps = 0.0
                            self._append_event(
                                "intent_expired",
                                "Fresh localization was lost; the leased intent was invalidated and zero applied immediately.",
                            )
                            self._finish("blocked", "localization_stale")
                if checkpoint is not None:
                    self._emit_checkpoint(*checkpoint)
        finally:
            self._stop.set()

    def _tick_once(self) -> None:
        self._tick += 1
        elapsed = self._elapsed_s()
        moving = (
            self._active_intent is not None
            and self._localization_state is LocalizationState.VALID
            and elapsed < self._active_intent.expires_at_s
        )
        if moving:
            speed = min(self._active_intent.speed_limit_mps, 0.02)
            self._command_speed_mps = speed
            yaw_rate = self._active_intent.steering * 0.32
            self._yaw_rad += yaw_rate * self.tick_s
            step = speed * self.tick_s
            self._x_m += step * math.cos(self._yaw_rad)
            self._y_m += step * math.sin(self._yaw_rad)
            self._distance_m += step
            if self._future is not None:
                self._motion_updates_while_inference += 1
        else:
            self._command_speed_mps = 0.0
        self._path.append({"x_m": self._x_m, "y_m": self._y_m})
        if len(self._path) > 1200:
            self._path = self._path[-1200:]

        self._perception_updates += 1
        self._mapping_updates += 1
        if (
            self._future is not None
            and self._future_call == 2
            and self._tick - self._future_started_tick >= 3
            and not self._obstacle_present
        ):
            self._obstacle_present = True
            self._obstacle_x_m = self._x_m + 1.5
            self._append_event(
                "obstacle_detected",
                "lidar-obstacle-01 entered the forward corridor while the LLM call remained in flight.",
            )
            self._emit_checkpoint("world_event", self._projection())
        self._latest_world = self._build_world_snapshot()
        self._command_samples.append(
            {
                "tick": self._tick,
                "elapsed_s": elapsed,
                "speed_mps": self._command_speed_mps,
                "steering": (
                    0.0 if self._active_intent is None else self._active_intent.steering
                ),
                "inference_in_flight": self._future is not None,
            }
        )
        if len(self._command_samples) > 500:
            self._command_samples = self._command_samples[-500:]

    def _dispatch_provider_call(self) -> None:
        snapshot = json.loads(json.dumps(self._latest_world))
        self._provider_calls_started += 1
        call = self._provider_calls_started
        self._future_call = call
        self._future_started_tick = self._tick
        self._future_snapshot = snapshot
        self._decision_snapshots.append(snapshot)
        self._future = self._pool.submit(self.provider.revise, self.mission, snapshot)
        self._append_event(
            "llm_call_started",
            f"Real/fake provider call {call} started from snapshot {snapshot['snapshot_id'][:12]}.",
        )
        self._emit_checkpoint("llm_call_started", self._projection())

    def _collect_provider_result(
        self,
    ) -> Optional[tuple[str, dict[str, Any]]]:
        if self._future is None or not self._future.done():
            return None
        future = self._future
        snapshot = self._future_snapshot
        call = self._future_call
        self._future = None
        self._future_snapshot = None
        self._provider_calls_completed += 1
        assert snapshot is not None
        try:
            raw = future.result()
            intent = RollingIntent.validated(
                raw,
                revision=len(self._revisions) + 1,
                snapshot=snapshot,
                issued_at_s=self._elapsed_s(),
                provider_id=self.provider.provider_id,
                model_id=self.provider.model_id,
            )
        except Exception as exc:
            self._provider_rejections += 1
            self._append_event(
                "llm_response_rejected",
                f"Provider call {call} was rejected by deterministic validation: {exc}",
            )
            if self._provider_calls_started >= 8:
                self._finish("failed", "provider_validation_exhausted")
            return ("llm_response_rejected", self._projection())

        previous = self._active_intent
        if (
            previous is not None
            and previous.speed_limit_mps > 0.0
            and intent.speed_limit_mps > 0.0
        ):
            self._compatible_swaps += 1
            if self._command_speed_mps <= 0.0:
                self._zero_motion_gaps += 1
        self._active_intent = intent
        revision = intent.to_json_dict()
        revision["applied_tick"] = self._tick
        revision["movement_updates_during_call"] = max(
            0, self._tick - self._future_started_tick
        )
        self._revisions.append(revision)
        self._append_event(
            "intent_revision",
            f"Revision {intent.revision} atomically applied: steering "
            f"{intent.steering:+.2f}, {intent.safe_corridor} corridor, focus "
            f"{intent.observation_focus}.",
        )
        obstacles = snapshot["obstacles"]
        if obstacles["ahead"] and abs(intent.steering) >= 0.2:
            self._obstacle_steering_changed = True
            self._append_event(
                "obstacle_steering_change",
                "Revision changed steering before collision using lidar-obstacle-01.",
            )
        uncertain = str(snapshot["camera"].get("uncertain_track_id", ""))
        if uncertain and intent.observation_focus == uncertain:
            self._uncertain_focus_changed = True
            self._append_event(
                "observation_focus_changed",
                f"Uncertain evidence {uncertain} selected alternate-left viewpoint.",
            )
        if len(self._revisions) == 3 and not self._uncertain_visible:
            self._uncertain_visible = True
            self._append_event(
                "uncertain_visual_evidence",
                "camera-frame evidence created uncertain shoe track shoe-uncertain-01.",
            )
        self._latest_world = self._build_world_snapshot()
        return ("intent_revision", self._projection())

    def _build_world_snapshot(self) -> dict[str, Any]:
        elapsed = self._elapsed_s()
        stamp_s = (
            elapsed
            if self._localization_frozen_at_s is None
            else self._localization_frozen_at_s
        )
        pose = Pose2D(
            x_m=self._x_m,
            y_m=self._y_m,
            yaw_rad=self._yaw_rad,
            stamp_s=max(0.0, stamp_s),
            frame_id="map",
        )
        localization = LocalizationEstimate(
            state=self._localization_state,
            source="lidar_replay_slam",
            pose=pose,
            quality=0.96 if self._localization_state is LocalizationState.VALID else 0.0,
            covariance_xy_m2=0.004,
            covariance_yaw_rad2=0.003,
            odom_translation_disagreement_m=0.012,
            odom_heading_disagreement_rad=0.018,
        )
        obstacle_distance = max(0.0, self._obstacle_x_m - self._x_m)
        obstacle_ahead = bool(
            self._obstacle_present
            and abs(self._yaw_rad) < 0.35
            and obstacle_distance < 1.5
        )
        observations = self._perception_updates
        objects = [
            {
                "track_id": "shoe-stable-01",
                "kind": "object",
                "label": "shoe",
                "confidence": min(0.97, 0.72 + observations * 0.002),
                "x_m": 0.88,
                "y_m": 1.92,
                "uncertainty_m": max(0.05, 0.22 - observations * 0.001),
                "observation_count": observations,
                "evidence_ids": [f"camera-frame-{max(1, self._tick):04d}"],
            },
            {
                "track_id": "face-unknown-01",
                "kind": "face",
                "label": "unknown",
                "confidence": min(0.90, 0.61 + observations * 0.0015),
                "x_m": 1.18,
                "y_m": 2.28,
                "uncertainty_m": max(0.08, 0.30 - observations * 0.001),
                "observation_count": observations,
                "evidence_ids": [f"face-crop-{max(1, self._tick):04d}"],
            },
        ]
        detections = [
            {
                "detection_id": f"shoe-detection-{max(1, self._tick):04d}",
                "track_id": "shoe-stable-01",
                "label": "shoe",
                "confidence": objects[0]["confidence"],
            },
            {
                "detection_id": f"face-detection-{max(1, self._tick):04d}",
                "track_id": "face-unknown-01",
                "label": "unknown_face",
                "confidence": objects[1]["confidence"],
            },
        ]
        uncertain_track = ""
        if self._uncertain_visible:
            uncertain_track = "shoe-uncertain-01"
            objects.append(
                {
                    "track_id": uncertain_track,
                    "kind": "object",
                    "label": "possible_shoe",
                    "confidence": min(0.68, 0.43 + max(0, observations - 6) * 0.004),
                    "x_m": self._x_m + 0.45,
                    "y_m": self._y_m + 0.52,
                    "uncertainty_m": 0.34,
                    "observation_count": max(1, observations - 6),
                    "evidence_ids": [f"uncertain-crop-{max(1, self._tick):04d}"],
                }
            )
            detections.append(
                {
                    "detection_id": f"uncertain-detection-{max(1, self._tick):04d}",
                    "track_id": uncertain_track,
                    "label": "possible_shoe",
                    "confidence": objects[-1]["confidence"],
                }
            )
        payload: dict[str, Any] = {
            "schema": WORLD_SNAPSHOT_SCHEMA,
            "mission_id": self.mission_id,
            "tick": self._tick,
            "elapsed_s": elapsed,
            "mission": self.mission,
            "budgets": {
                "remaining_distance_m": max(0.0, 2.0 - self._distance_m),
                "remaining_runtime_s": max(0.0, 180.0 - elapsed),
                "remaining_observations": max(0, 1200 - observations),
                "remaining_provider_calls": max(0, 8 - self._provider_calls_started),
            },
            "localization": {
                **localization.to_json_dict(),
                "fresh": self._localization_state is LocalizationState.VALID,
                "age_s": max(0.0, elapsed - stamp_s),
            },
            "obstacles": {
                "ahead": obstacle_ahead,
                "nearest_id": "lidar-obstacle-01" if self._obstacle_present else "",
                "forward_clearance_m": obstacle_distance if self._obstacle_present else 2.4,
                "left_clearance_m": 1.35,
                "right_clearance_m": 0.48,
                "collision": self._collision_state != "CLEAR",
            },
            "progress": {
                "distance_m": self._distance_m,
                "path_points": len(self._path),
                "moving": self._command_speed_mps > 0.0,
                "speed_mps": self._command_speed_mps,
            },
            "camera": {
                "frame_id": f"camera-frame-{max(1, self._tick):04d}",
                "stamp_s": elapsed,
                "detections": detections,
                "uncertain_track_id": uncertain_track,
            },
            "semantic_map": {
                "revision": self._mapping_updates,
                "tracks": objects,
                "object_track_count": sum(
                    1 for item in objects if item["kind"] == "object"
                ),
                "face_track_count": sum(
                    1 for item in objects if item["kind"] == "face"
                ),
            },
            "safety": {
                "collision_state": self._collision_state,
                "stop_active": self._stop_active,
                "estop_latched": self._estop_latched,
                "motion_authority": False,
                "physical_execution_enabled": False,
            },
            "active_intent_revision": (
                None if self._active_intent is None else self._active_intent.revision
            ),
            "source_evidence": {
                "stage1_replay": "artifacts/perception_navigation_stage1/turn_attempt_5_replay.json",
                "localization_contract": "sphero_rvr.perception_navigation_result.v1",
            },
        }
        payload["snapshot_id"] = canonical_digest(payload)
        return payload

    def _projection(self) -> dict[str, Any]:
        result = self._terminal_result() if self._terminal else {}
        tracks = self._latest_world["semantic_map"]["tracks"]
        return {
            "schema": ROLLING_RESULT_SCHEMA,
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
                "call": self._future_call if self._future is not None else None,
                "snapshot_id": (
                    ""
                    if self._future_snapshot is None
                    else self._future_snapshot["snapshot_id"]
                ),
                "movement_updates_during_calls": self._motion_updates_while_inference,
                "provider_calls_started": self._provider_calls_started,
                "provider_calls_completed": self._provider_calls_completed,
                "provider_rejections": self._provider_rejections,
            },
            "metrics": self._metrics(),
            "events": json.loads(json.dumps(self._events)),
            "map": {
                "available": True,
                "fixture_only": True,
                "frame": "map",
                "bounds": {
                    "origin": {"x_m": 0.0, "y_m": 0.0},
                    "width_m": 3.2,
                    "height_m": 3.0,
                },
                "rover": {
                    "x_m": self._x_m,
                    "y_m": self._y_m,
                    "yaw_deg": math.degrees(self._yaw_rad),
                },
                "goal_region": {"x_m": 2.8, "y_m": 1.45, "radius_m": 0.25},
                "proposed_route": [
                    {"x_m": self._x_m, "y_m": self._y_m},
                    {
                        "x_m": self._x_m + 0.45 * math.cos(self._yaw_rad),
                        "y_m": self._y_m + 0.45 * math.sin(self._yaw_rad),
                    },
                ],
                "traveled_path": list(self._path),
                "obstacles": (
                    [
                        {
                            "obstacle_id": "lidar-obstacle-01",
                            "x_m": self._obstacle_x_m,
                            "y_m": 1.36,
                            "width_m": 0.24,
                            "height_m": 0.24,
                            "label": "introduced obstacle",
                        }
                    ]
                    if self._obstacle_present
                    else []
                ),
                "objects": [
                    {
                        "object_id": item["track_id"],
                        "label": item["label"],
                        "x_m": item["x_m"],
                        "y_m": item["y_m"],
                        "confidence": item["confidence"],
                        "evidence_ref": item["evidence_ids"][0],
                    }
                    for item in tracks
                ],
            },
            "result": result,
            "motion_authority": False,
            "physical_execution_enabled": False,
        }

    def _metrics(self) -> dict[str, Any]:
        return {
            "simulation_ticks": self._tick,
            "perception_updates": self._perception_updates,
            "semantic_map_updates": self._mapping_updates,
            "path_points": len(self._path),
            "intent_revision_count": len(self._revisions),
            "compatible_intent_swaps": self._compatible_swaps,
            "artificial_zero_motion_gaps": self._zero_motion_gaps,
            "motion_updates_while_llm_in_flight": self._motion_updates_while_inference,
            "obstacle_steering_changed": self._obstacle_steering_changed,
            "uncertain_evidence_changed_focus": self._uncertain_focus_changed,
            "continuous_object_tracking": self._perception_updates > 1,
            "continuous_face_tracking": self._perception_updates > 1,
            "stale_localization_stopped": (
                self._terminal_reason == "localization_stale"
                and self._command_speed_mps == 0.0
            ),
        }

    def _terminal_result(self) -> dict[str, Any]:
        return {
            "schema": ROLLING_RESULT_SCHEMA,
            "mission_id": self.mission_id,
            "status": self._terminal_state,
            "terminal_reason": self._terminal_reason,
            "metrics": self._metrics(),
            "final_snapshot": json.loads(json.dumps(self._latest_world)),
            "intent_revisions": json.loads(json.dumps(self._revisions)),
            "decision_snapshots": json.loads(json.dumps(self._decision_snapshots)),
            "path": list(self._path),
            "detections": self._latest_world["camera"]["detections"],
            "semantic_tracks": self._latest_world["semantic_map"]["tracks"],
            "command_samples": list(self._command_samples),
            "motion_authority": False,
            "physical_execution_enabled": False,
        }

    def _finish(self, state: str, reason: str) -> None:
        self._command_speed_mps = 0.0
        self._terminal = True
        self._terminal_state = str(state)
        self._terminal_reason = str(reason)
        self._append_event(
            "terminal",
            f"Replay stopped deterministically: {reason}; simulated speed is zero.",
        )
        self._latest_world = self._build_world_snapshot()
        self._emit_checkpoint("terminal", self._projection())

    def _append_event(self, kind: str, message: str) -> None:
        self._events.append(
            {
                "sequence": len(self._events) + 1,
                "event_type": str(kind),
                "message": str(message),
                "tick": self._tick,
                "elapsed_s": self._elapsed_s(),
            }
        )

    def _emit_checkpoint(self, kind: str, payload: Mapping[str, Any]) -> None:
        if self._checkpoint is None:
            return
        try:
            self._checkpoint(kind, payload)
        except Exception as exc:
            with self._lock:
                if not self._terminal:
                    self._terminal = True
                    self._terminal_state = "failed"
                    self._terminal_reason = "persistence_checkpoint_failed"
                    self._command_speed_mps = 0.0
                    self._append_event(
                        "persistence_failure",
                        f"Mission persistence failed closed: {exc.__class__.__name__}.",
                    )

    def _elapsed_s(self) -> float:
        return max(0.0, time.monotonic() - self._started_monotonic)


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise MissionValidationError(f"{name} must be finite") from exc
    if not math.isfinite(number):
        raise MissionValidationError(f"{name} must be finite")
    return number
