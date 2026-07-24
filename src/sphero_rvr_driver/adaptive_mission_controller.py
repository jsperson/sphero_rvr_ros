"""Adaptive mission snapshot-to-intent closed-loop controller.

The model receives typed world evidence and selects one bounded intent.  It has
no ROS, serial, topic, velocity, or motor surface.  A deterministic executor
owns primitive completion and an independent supervisor owns the final
requested-versus-supervised movement decision.
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
from .prompt_drive import ALLOWED_REASONING_EFFORTS, DEFAULT_CODEX_MODEL_ID


ADAPTIVE_MISSION_WORLD_SCHEMA = "sphero_rvr.adaptive_mission_world_snapshot.v1"
ADAPTIVE_MISSION_INTENT_SCHEMA = "sphero_rvr.adaptive_mission_intent.v1"
ADAPTIVE_MISSION_PROPOSAL_SCHEMA = "sphero_rvr.adaptive_mission_proposal.v1"
ADAPTIVE_MISSION_RESULT_SCHEMA = "sphero_rvr.adaptive_mission_result.v1"
ADAPTIVE_MISSION_SAFETY_POLICY = "lidar_collision_stop.v1"

_ACTIONS = {"move_distance", "turn_angle", "observe", "stop"}
_CLEAR_COLLISION_STATES = {"CLEAR", "SLOW"}


def canonical_digest(payload: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        dict(payload), sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class AdaptiveMissionLimits:
    mission_lease_s: float = 15.0 * 60.0
    max_translation_per_intent_m: float = 0.25
    max_rotation_per_intent_deg: float = 45.0
    max_intent_timeout_s: float = 5.0
    max_intent_lease_s: float = 5.0
    linear_speed_mps: float = 0.10
    angular_speed_rad_s: float = 0.4

    def __post_init__(self) -> None:
        values = {
            name: float(getattr(self, name))
            for name in (
                "mission_lease_s",
                "max_translation_per_intent_m",
                "max_rotation_per_intent_deg",
                "max_intent_timeout_s",
                "max_intent_lease_s",
                "linear_speed_mps",
                "angular_speed_rad_s",
            )
        }
        if any(not math.isfinite(value) or value <= 0.0 for value in values.values()):
            raise ValueError("adaptive mission limits must be positive and finite")
        ceilings = {
            "mission_lease_s": 900.0,
            "max_translation_per_intent_m": 0.25,
            "max_rotation_per_intent_deg": 45.0,
            "max_intent_timeout_s": 5.0,
            "max_intent_lease_s": 5.0,
            "linear_speed_mps": 0.10,
            "angular_speed_rad_s": 0.4,
        }
        for name, ceiling in ceilings.items():
            if values[name] > ceiling:
                raise ValueError(f"{name} exceeds the adaptive mission authority ceiling")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "mission_lease_s": self.mission_lease_s,
            "max_translation_per_intent_m": self.max_translation_per_intent_m,
            "max_rotation_per_intent_deg": self.max_rotation_per_intent_deg,
            "max_intent_timeout_s": self.max_intent_timeout_s,
            "max_intent_lease_s": self.max_intent_lease_s,
            "linear_speed_mps": self.linear_speed_mps,
            "angular_speed_rad_s": self.angular_speed_rad_s,
            "max_cumulative_translation_m": None,
            "max_cumulative_rotation_deg": None,
            "max_intent_count": None,
        }


def make_world_snapshot(payload: Mapping[str, Any]) -> dict[str, Any]:
    body = json.loads(json.dumps(dict(payload), allow_nan=False))
    body["schema"] = ADAPTIVE_MISSION_WORLD_SCHEMA
    body.pop("snapshot_id", None)
    body["snapshot_id"] = canonical_digest(body)
    return body


def validate_world_snapshot(
    snapshot: Mapping[str, Any],
    *,
    mission_id: str,
    require_motion: bool = False,
) -> None:
    if str(snapshot.get("schema", "")) != ADAPTIVE_MISSION_WORLD_SCHEMA:
        raise MissionValidationError("adaptive mission world snapshot schema is invalid")
    if str(snapshot.get("mission_id", "")) != str(mission_id):
        raise MissionValidationError("adaptive mission world snapshot mission identity changed")
    supplied = str(snapshot.get("snapshot_id", ""))
    digest_payload = dict(snapshot)
    digest_payload.pop("snapshot_id", None)
    if supplied != canonical_digest(digest_payload):
        raise MissionValidationError("adaptive mission world snapshot digest is invalid")
    evidence = snapshot.get("evidence")
    safety = snapshot.get("safety")
    execution = snapshot.get("execution")
    observations = snapshot.get("observations")
    if not isinstance(evidence, Mapping) or not isinstance(safety, Mapping):
        raise MissionValidationError("adaptive mission world snapshot lacks typed evidence")
    if not isinstance(execution, Mapping):
        raise MissionValidationError("adaptive mission world snapshot lacks executor evidence")
    if not isinstance(observations, Mapping):
        raise MissionValidationError("adaptive mission world snapshot lacks typed observations")
    for name in (
        "camera_detections",
        "semantic_tracks",
        "recognized_objects",
        "recognized_faces",
        "unknown_faces",
    ):
        if name in observations and not isinstance(observations.get(name), list):
            raise MissionValidationError(
                f"adaptive mission world snapshot {name} must be a list"
            )
    if "perception" in observations and not isinstance(
        observations.get("perception"), Mapping
    ):
        raise MissionValidationError(
            "adaptive mission world snapshot perception status must be an object"
        )
    for name in ("scan_fresh", "transform_fresh", "odometry_fresh"):
        if evidence.get(name) is not True:
            raise MissionValidationError(f"stale evidence: {name}")
    if safety.get("stop_active") is True:
        raise MissionValidationError("STOP is active")
    if safety.get("estop_latched") is True:
        raise MissionValidationError("ESTOP is latched")
    collision_state = str(safety.get("collision_state", "")).upper()
    if collision_state not in _CLEAR_COLLISION_STATES:
        raise MissionValidationError(f"collision supervisor is {collision_state or 'UNKNOWN'}")
    if require_motion and execution.get("motion_permitted") is not True:
        raise MissionValidationError("motion is not permitted by the executor snapshot")


@dataclass(frozen=True)
class AdaptiveMissionIntent:
    revision: int
    snapshot_id: str
    action: str
    distance_m: float
    angle_deg: float
    observation_focus: str
    rationale: str
    interpreted_objective: str
    lease_s: float
    timeout_s: float
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
        limits: AdaptiveMissionLimits,
    ) -> "AdaptiveMissionIntent":
        if str(raw.get("snapshot_id", "")) != str(snapshot.get("snapshot_id", "")):
            raise MissionValidationError(
                "adaptive mission intent is not bound to the exact world snapshot"
            )
        action = str(raw.get("action", "")).strip()
        if action not in _ACTIONS:
            raise MissionValidationError("adaptive mission intent action is unsupported")
        distance = _finite(raw.get("distance_m"), "adaptive mission intent distance")
        angle = _finite(raw.get("angle_deg"), "adaptive mission intent angle")
        lease = _finite(raw.get("lease_s"), "adaptive mission intent lease")
        timeout = _finite(raw.get("timeout_s"), "adaptive mission intent timeout")
        if abs(distance) > limits.max_translation_per_intent_m:
            raise MissionValidationError("adaptive mission intent translation exceeds 0.25 m")
        if abs(angle) > limits.max_rotation_per_intent_deg:
            raise MissionValidationError("adaptive mission intent rotation exceeds 45 degrees")
        if not 0.0 < lease <= limits.max_intent_lease_s:
            raise MissionValidationError("adaptive mission intent lease exceeds 5 seconds")
        if not 0.0 < timeout <= limits.max_intent_timeout_s:
            raise MissionValidationError("adaptive mission intent timeout exceeds 5 seconds")
        if action == "move_distance":
            if distance == 0.0 or angle != 0.0:
                raise MissionValidationError(
                    "move_distance requires nonzero distance_m and zero angle_deg"
                )
        elif action == "turn_angle":
            if angle == 0.0 or distance != 0.0:
                raise MissionValidationError(
                    "turn_angle requires nonzero angle_deg and zero distance_m"
                )
        elif distance != 0.0 or angle != 0.0:
            raise MissionValidationError(
                f"{action} requires zero distance_m and angle_deg"
            )
        if action in {"move_distance", "turn_angle"}:
            execution = snapshot.get("execution", {})
            if not isinstance(execution, Mapping) or execution.get("motion_permitted") is not True:
                raise MissionValidationError(
                    "motion intent rejected because the snapshot permits observation only"
                )
        rationale = str(raw.get("rationale", "")).strip()
        objective = str(raw.get("interpreted_objective", "")).strip()
        focus = str(raw.get("observation_focus", "")).strip()
        if not rationale or len(rationale) > 800:
            raise MissionValidationError("adaptive mission intent rationale is required and bounded")
        if not objective or len(objective) > 500:
            raise MissionValidationError(
                "Adaptive mission interpreted objective is required and bounded"
            )
        if not focus or len(focus) > 160:
            raise MissionValidationError(
                "Adaptive mission observation focus is required and bounded"
            )
        return cls(
            revision=int(revision),
            snapshot_id=str(snapshot["snapshot_id"]),
            action=action,
            distance_m=distance,
            angle_deg=angle,
            observation_focus=focus,
            rationale=rationale,
            interpreted_objective=objective,
            lease_s=lease,
            timeout_s=timeout,
            issued_at_s=float(issued_at_s),
            expires_at_s=float(issued_at_s) + lease,
            provider_id=str(provider_id),
            model_id=str(model_id),
        )

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "schema": ADAPTIVE_MISSION_INTENT_SCHEMA,
            "revision": self.revision,
            "snapshot_id": self.snapshot_id,
            "action": self.action,
            "distance_m": self.distance_m,
            "angle_deg": self.angle_deg,
            "observation_focus": self.observation_focus,
            "rationale": self.rationale,
            "interpreted_objective": self.interpreted_objective,
            "lease_s": self.lease_s,
            "timeout_s": self.timeout_s,
            "issued_at_s": self.issued_at_s,
            "expires_at_s": self.expires_at_s,
            "provider_id": self.provider_id,
            "model_id": self.model_id,
        }


class AdaptiveMissionIntentProvider(Protocol):
    provider_id: str
    model_id: str
    reasoning_effort: str

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]: ...


class CodexOAuthAdaptiveMissionIntentProvider:
    """Real ChatGPT-OAuth planner for one snapshot-bound adaptive mission intent."""

    provider_id = "openai-codex-oauth"

    def __init__(
        self,
        *,
        model: Optional[str] = None,
        reasoning_effort: str = "low",
        codex_command: str = "codex",
        timeout_s: float = 120.0,
    ) -> None:
        if reasoning_effort not in ALLOWED_REASONING_EFFORTS:
            raise MissionValidationError(
                f"unsupported reasoning effort: {reasoning_effort}"
            )
        self.model_id = model or os.environ.get(
            "OPENAI_MODEL", DEFAULT_CODEX_MODEL_ID
        )
        self.reasoning_effort = reasoning_effort
        self.codex_command = str(codex_command)
        self.timeout_s = float(timeout_s)
        self._oauth_checked = False
        self._oauth_lock = threading.Lock()

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        executable = shutil.which(self.codex_command)
        if executable is None:
            raise MissionValidationError(
                "Codex CLI is not installed; Adaptive mission requires the real OAuth provider"
            )
        env = dict(os.environ)
        env.pop("OPENAI_API_KEY", None)
        env.pop("CODEX_API_KEY", None)
        self._require_chatgpt_oauth(executable, env)
        with tempfile.TemporaryDirectory(prefix="rvr-adaptive-mission-") as directory:
            root = Path(directory)
            schema_path = root / "intent-schema.json"
            output_path = root / "intent.json"
            schema_path.write_text(
                json.dumps(_adaptive_mission_output_schema(), sort_keys=True),
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
                    input=_adaptive_mission_provider_prompt(prompt, snapshot),
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
                    "Codex OAuth adaptive mission intent call timed out"
                ) from exc
            if completed.returncode != 0:
                detail = str(completed.stderr).strip().splitlines()
                suffix = f": {detail[-1]}" if detail else ""
                raise MissionValidationError(
                    "Codex OAuth adaptive mission intent call failed with exit code "
                    f"{completed.returncode}{suffix}"
                )
            try:
                payload = json.loads(output_path.read_text(encoding="utf-8"))
            except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise MissionValidationError(
                    "Codex OAuth adaptive mission intent call returned malformed output"
                ) from exc
        if not isinstance(payload, Mapping):
            raise MissionValidationError(
                "Codex OAuth adaptive mission intent output must be an object"
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
                raise MissionValidationError(
                    "Codex OAuth status check timed out"
                ) from exc
            if (
                status.returncode != 0
                or "logged in using chatgpt" not in str(status.stdout).lower()
            ):
                raise MissionValidationError(
                    "Codex CLI is not authenticated with ChatGPT OAuth; "
                    "run `codex login --device-auth`"
                )
            self._oauth_checked = True


def _adaptive_mission_output_schema() -> dict[str, Any]:
    return {
        "$schema": "http://json-schema.org/draft-07/schema#",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "snapshot_id": {"type": "string"},
            "action": {"type": "string", "enum": sorted(_ACTIONS)},
            "distance_m": {"type": "number", "minimum": -0.25, "maximum": 0.25},
            "angle_deg": {"type": "number", "minimum": -45.0, "maximum": 45.0},
            "observation_focus": {
                "type": "string",
                "minLength": 1,
                "maxLength": 160,
            },
            "rationale": {"type": "string", "minLength": 1, "maxLength": 800},
            "interpreted_objective": {
                "type": "string",
                "minLength": 1,
                "maxLength": 500,
            },
            "lease_s": {"type": "number", "minimum": 5.0, "maximum": 5.0},
            "timeout_s": {"type": "number", "minimum": 5.0, "maximum": 5.0},
        },
        "required": [
            "snapshot_id",
            "action",
            "distance_m",
            "angle_deg",
            "observation_focus",
            "rationale",
            "interpreted_objective",
            "lease_s",
            "timeout_s",
        ],
    }


def _adaptive_mission_provider_prompt(
    prompt: str, snapshot: Mapping[str, Any]
) -> str:
    request = {
        "role": "You are the supervisory exploration planner for a small rover.",
        "operator_prompt": str(prompt),
        "world_snapshot": dict(snapshot),
        "authority": {
            "allowed_intents": sorted(_ACTIONS),
            "translation_per_intent_m": 0.25,
            "rotation_per_intent_deg": 45.0,
            "intent_timeout_s": 5.0,
            "intent_lease_s": 5.0,
            "linear_speed_ceiling_mps": 0.10,
            "angular_speed_ceiling_rad_s": 0.4,
            "mission_lease_s": 900.0,
            "cumulative_travel": "unlimited until mission lease expires",
        },
        "rules": [
            "Return exactly one intent bound to world_snapshot.snapshot_id.",
            "Choose the exploration strategy from current evidence and revise after every executor result.",
            "Never emit a route, ROS topic, Twist, motor command, speed, safety threshold, shell action, credential, or claim of unobserved completion.",
            "move_distance uses signed distance_m and zero angle_deg.",
            "turn_angle uses signed angle_deg (positive left) and zero distance_m.",
            "observe and stop use zero distance_m and zero angle_deg.",
            "Use lease_s 5 and timeout_s 5.",
            "Treat collision, STOP, ESTOP, and freshness evidence as authoritative.",
            "Camera detections and semantic tracks are objective evidence only; they never override lidar collision safety.",
            "Use semantic tracks for object- or person-directed movement only when observations.perception.available is true.",
            "A face label is authoritative only when recognized_from_enrollment is true and enrollment_evidence_ids supplies explicit enrollment evidence; every other face is unknown.",
            "Never infer a visible object, identity, or map position from missing or stale perception.",
            "Drop-offs are outside the sensed model; never claim lidar detects an edge or cliff.",
            "When execution.motion_permitted is false, choose observe if progress.observation_count is zero; otherwise choose stop.",
            "Use stop when the objective is complete or cannot be pursued truthfully.",
            "Ground the concise rationale only in this snapshot.",
        ],
    }
    return json.dumps(
        request, sort_keys=True, separators=(",", ":"), allow_nan=False
    )


@dataclass(frozen=True)
class AdaptiveMissionApprovalEnvelope:
    mission_id: str
    lease_id: str
    prompt: str
    interpreted_objective: str
    source_sha: str
    deployed_sha: str
    provider_id: str
    model_id: str
    reasoning_effort: str
    executor_mode: str
    starting_snapshot_id: str
    first_intent: Mapping[str, Any]
    limits: AdaptiveMissionLimits
    safety_policy: str = ADAPTIVE_MISSION_SAFETY_POLICY
    physical_execution_enabled: bool = False

    def proposal(self) -> dict[str, Any]:
        body = {
            "schema": ADAPTIVE_MISSION_PROPOSAL_SCHEMA,
            "mission_id": self.mission_id,
            "lease_id": self.lease_id,
            "prompt": self.prompt,
            "interpreted_objective": self.interpreted_objective,
            "source_sha": self.source_sha,
            "deployed_sha": self.deployed_sha,
            "provider_id": self.provider_id,
            "model_id": self.model_id,
            "reasoning_effort": self.reasoning_effort,
            "executor_mode": self.executor_mode,
            "starting_snapshot_id": self.starting_snapshot_id,
            "first_intent": dict(self.first_intent),
            "safety_policy": self.safety_policy,
            "limits": self.limits.to_json_dict(),
            "segments": [],
            "decision": "propose",
            "summary": (
                "Approve one 15-minute adaptive mission lease. The LLM selects one "
                "bounded intent from each fresh snapshot; deterministic validation, "
                "the executor, and collision supervision remain authoritative."
            ),
            "contract": {
                "fixed_route": False,
                "replanning_after_every_intent": True,
                "one_authenticated_approval": True,
                "per_intent_approval": False,
                "motion_authority": False,
                "physical_execution_enabled": bool(
                    self.physical_execution_enabled
                ),
                "drop_off_detection": False,
            },
        }
        return {**body, "proposal_digest": canonical_digest(body)}


@dataclass(frozen=True)
class MovementDecision:
    outcome: str
    reason: str
    requested_linear_mps: float
    requested_angular_rad_s: float
    supervised_linear_mps: float
    supervised_angular_rad_s: float
    collision_state: str

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "outcome": self.outcome,
            "reason": self.reason,
            "requested": {
                "linear_mps": self.requested_linear_mps,
                "angular_rad_s": self.requested_angular_rad_s,
            },
            "supervised": {
                "linear_mps": self.supervised_linear_mps,
                "angular_rad_s": self.supervised_angular_rad_s,
            },
            "collision_state": self.collision_state,
            "motor_topic_publisher": "lidar_collision_stop_supervisor",
            "command_path": [
                "adaptive_mission_intent_executor",
                "/cmd_vel",
                "lidar_collision_stop_supervisor",
                "/cmd_vel_motor",
            ],
        }


@dataclass(frozen=True)
class IntentExecutionResult:
    outcome: str
    reason: str
    snapshot: Mapping[str, Any]
    movement: MovementDecision
    duration_s: float

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "outcome": self.outcome,
            "reason": self.reason,
            "duration_s": self.duration_s,
            "movement": self.movement.to_json_dict(),
            "updated_snapshot_id": self.snapshot.get("snapshot_id", ""),
        }


class AdaptiveMissionExecutor(Protocol):
    mode: str

    def snapshot(self, mission_id: str) -> Mapping[str, Any]: ...

    def execute(
        self, intent: AdaptiveMissionIntent, cancellation: threading.Event
    ) -> IntentExecutionResult: ...


class ReplayCollisionSupervisor:
    """Deterministic collision boundary used behind the executor protocol."""

    def __init__(self, *, collision_on_intent: Optional[int] = None) -> None:
        self.collision_on_intent = collision_on_intent
        self.calls = 0

    def supervise(
        self,
        intent: AdaptiveMissionIntent,
        snapshot: Mapping[str, Any],
        limits: AdaptiveMissionLimits,
    ) -> MovementDecision:
        self.calls += 1
        evidence = snapshot.get("evidence", {})
        if not isinstance(evidence, Mapping) or any(
            evidence.get(name) is not True
            for name in ("scan_fresh", "transform_fresh", "odometry_fresh")
        ):
            return MovementDecision(
                "stale",
                "stale_evidence",
                0.0,
                0.0,
                0.0,
                0.0,
                "SENSOR_STALE",
            )
        safety = snapshot.get("safety", {})
        collision_state = (
            str(safety.get("collision_state", "UNKNOWN")).upper()
            if isinstance(safety, Mapping)
            else "UNKNOWN"
        )
        if (
            self.collision_on_intent is not None
            and self.calls == self.collision_on_intent
        ):
            collision_state = "BLOCKED"
        requested_linear = (
            math.copysign(limits.linear_speed_mps, intent.distance_m)
            if intent.action == "move_distance"
            else 0.0
        )
        requested_angular = (
            math.copysign(limits.angular_speed_rad_s, intent.angle_deg)
            if intent.action == "turn_angle"
            else 0.0
        )
        if collision_state not in _CLEAR_COLLISION_STATES:
            return MovementDecision(
                "blocked",
                "collision_veto",
                requested_linear,
                requested_angular,
                0.0,
                0.0,
                collision_state,
            )
        scale = 0.5 if collision_state == "SLOW" and requested_linear > 0.0 else 1.0
        return MovementDecision(
            "allowed",
            "clear" if scale == 1.0 else "supervisor_slowed",
            requested_linear,
            requested_angular,
            requested_linear * scale,
            requested_angular,
            collision_state,
        )


class ReplayAdaptiveMissionExecutor:
    """Fast replay executor implementing the future physical adapter boundary."""

    mode = "replay-simulation"

    def __init__(
        self,
        *,
        limits: Optional[AdaptiveMissionLimits] = None,
        supervisor: Optional[ReplayCollisionSupervisor] = None,
        motion_permitted: bool = True,
        stale_after_intents: Optional[int] = None,
    ) -> None:
        self.limits = limits or AdaptiveMissionLimits()
        self.supervisor = supervisor or ReplayCollisionSupervisor()
        self.motion_permitted = bool(motion_permitted)
        self.stale_after_intents = stale_after_intents
        self.x_m = 0.4
        self.y_m = 1.4
        self.yaw_rad = 0.0
        self.intent_count = 0
        self.observation_count = 0
        self.cumulative_translation_m = 0.0
        self.cumulative_rotation_deg = 0.0
        self.path = [{"x_m": self.x_m, "y_m": self.y_m}]
        self._mission_id = ""
        self._last_execution: Optional[dict[str, Any]] = None

    def snapshot(self, mission_id: str) -> Mapping[str, Any]:
        self._mission_id = str(mission_id)
        stale = (
            self.stale_after_intents is not None
            and self.intent_count >= self.stale_after_intents
        )
        return make_world_snapshot(
            {
                "mission_id": self._mission_id,
                "version": self.intent_count + 1,
                "observed_at_s": time.time(),
                "pose": {
                    "frame": "map",
                    "x_m": self.x_m,
                    "y_m": self.y_m,
                    "yaw_deg": math.degrees(self.yaw_rad),
                },
                "evidence": {
                    "scan_fresh": not stale,
                    "transform_fresh": not stale,
                    "odometry_fresh": not stale,
                    "localization_fresh": not stale,
                    "scan_age_s": 0.02 if not stale else 0.31,
                    "odometry_age_s": 0.02 if not stale else 0.31,
                    "drop_off_detection_available": False,
                },
                "safety": {
                    "collision_state": "CLEAR",
                    "stop_active": False,
                    "estop_latched": False,
                    "supervisor": ADAPTIVE_MISSION_SAFETY_POLICY,
                },
                "execution": {
                    "mode": self.mode,
                    "motion_permitted": self.motion_permitted,
                    "physical_execution_enabled": False,
                    "motion_authority": False,
                    "motor_topic_publisher": "lidar_collision_stop_supervisor",
                },
                "progress": {
                    "intent_count": self.intent_count,
                    "observation_count": self.observation_count,
                    "cumulative_translation_m": self.cumulative_translation_m,
                    "cumulative_rotation_deg": self.cumulative_rotation_deg,
                    "cumulative_limits": "unlimited within approved mission lease",
                },
                "observations": {
                    "forward_clearance_m": 1.8,
                    "left_clearance_m": 1.2,
                    "right_clearance_m": 0.9,
                    "camera_detections": [],
                    "semantic_tracks": [],
                    "recognized_objects": [],
                    "recognized_faces": [],
                    "unknown_faces": [],
                    "perception": {
                        "available": False,
                        "camera_fresh": False,
                        "semantic_map_fresh": False,
                        "localization_fresh": not stale,
                        "localization_state": (
                            "valid" if not stale else "stale"
                        ),
                        "camera_frame_id": None,
                        "semantic_map_revision": None,
                        "uncertain_track_id": "",
                        "identity_policy": (
                            "face labels are authoritative only with explicit "
                            "enrollment evidence"
                        ),
                    },
                    "coverage_note": (
                        (
                            "Observation completed after supervised replay movement."
                            if self.cumulative_translation_m > 0.0
                            or self.cumulative_rotation_deg > 0.0
                            else "No motion occurred; safety evidence observed."
                        )
                        if self.observation_count
                        else "Starting snapshot; no observation intent completed yet."
                    ),
                },
                "last_execution": self._last_execution,
            }
        )

    def execute(
        self, intent: AdaptiveMissionIntent, cancellation: threading.Event
    ) -> IntentExecutionResult:
        before = self.snapshot(self._mission_id)
        if cancellation.is_set():
            movement = MovementDecision(
                "cancelled", "operator_cancelled", 0.0, 0.0, 0.0, 0.0, "CLEAR"
            )
            return IntentExecutionResult(
                "cancelled", "operator_cancelled", before, movement, 0.0
            )
        movement = self.supervisor.supervise(intent, before, self.limits)
        if movement.outcome != "allowed":
            self._last_execution = {
                "intent": intent.to_json_dict(),
                "movement": movement.to_json_dict(),
            }
            return IntentExecutionResult(
                movement.outcome,
                movement.reason,
                self.snapshot(self._mission_id),
                movement,
                0.0,
            )
        duration = 0.0
        if intent.action == "move_distance":
            speed = abs(movement.supervised_linear_mps)
            duration = math.inf if speed == 0.0 else abs(intent.distance_m) / speed
        elif intent.action == "turn_angle":
            speed = abs(movement.supervised_angular_rad_s)
            duration = (
                math.inf
                if speed == 0.0
                else abs(math.radians(intent.angle_deg)) / speed
            )
        if duration > intent.timeout_s:
            timed_out = MovementDecision(
                "timeout",
                "intent_timeout",
                movement.requested_linear_mps,
                movement.requested_angular_rad_s,
                0.0,
                0.0,
                movement.collision_state,
            )
            self._last_execution = {
                "intent": intent.to_json_dict(),
                "movement": timed_out.to_json_dict(),
            }
            return IntentExecutionResult(
                "timeout",
                "intent_timeout",
                self.snapshot(self._mission_id),
                timed_out,
                intent.timeout_s,
            )
        if intent.action == "move_distance":
            self.x_m += intent.distance_m * math.cos(self.yaw_rad)
            self.y_m += intent.distance_m * math.sin(self.yaw_rad)
            self.cumulative_translation_m += abs(intent.distance_m)
            self.path.append({"x_m": self.x_m, "y_m": self.y_m})
        elif intent.action == "turn_angle":
            self.yaw_rad += math.radians(intent.angle_deg)
            self.cumulative_rotation_deg += abs(intent.angle_deg)
        elif intent.action == "observe":
            self.observation_count += 1
        self.intent_count += 1
        self._last_execution = {
            "intent": intent.to_json_dict(),
            "movement": movement.to_json_dict(),
            "duration_s": duration,
        }
        return IntentExecutionResult(
            "completed",
            "intent_completed",
            self.snapshot(self._mission_id),
            movement,
            duration,
        )

    def map_projection(self) -> dict[str, Any]:
        return {
            "available": True,
            "fixture_only": True,
            "source": "adaptive-mission-replay-executor",
            "frame": "map",
            "bounds": {
                "origin": {"x_m": 0.0, "y_m": 0.0},
                "width_m": 3.2,
                "height_m": 3.0,
            },
            "rover": {
                "x_m": self.x_m,
                "y_m": self.y_m,
                "yaw_deg": math.degrees(self.yaw_rad),
            },
            "goal_region": None,
            "proposed_route": [],
            "traveled_path": list(self.path),
            "obstacles": [],
            "objects": [],
            "localization": {"state": "valid", "fresh": True, "quality": 0.98},
        }


class AdaptiveMissionController:
    """Execute one approved, repeatedly replanned adaptive mission lease."""

    def __init__(
        self,
        *,
        mission_id: str,
        prompt: str,
        proposal_digest: str,
        operator: str,
        authenticated: bool,
        authentication_source: str,
        approved_at_s: float,
        first_snapshot: Mapping[str, Any],
        first_intent: AdaptiveMissionIntent,
        provider: AdaptiveMissionIntentProvider,
        executor: AdaptiveMissionExecutor,
        limits: Optional[AdaptiveMissionLimits] = None,
        checkpoint: Optional[Callable[[str, Mapping[str, Any]], None]] = None,
        owns_executor: bool = True,
        now: Callable[[], float] = time.time,
    ) -> None:
        if (
            not str(operator).strip()
            or not authenticated
            or not str(authentication_source).strip()
        ):
            raise MissionValidationError(
                "Adaptive mission requires an authenticated approval operator and source"
            )
        self.mission_id = str(mission_id)
        self.prompt = str(prompt)
        self.proposal_digest = str(proposal_digest)
        self.operator = str(operator)
        self.authentication_source = str(authentication_source)
        self.approved_at_s = float(approved_at_s)
        self.provider = provider
        self.executor = executor
        self.limits = limits or AdaptiveMissionLimits()
        self._checkpoint = checkpoint
        self._owns_executor = bool(owns_executor)
        self._now = now
        self._mission_expires_at_s = self.approved_at_s + self.limits.mission_lease_s
        self._lock = threading.RLock()
        self._cancellation = threading.Event()
        self._shutdown = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._provider_pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="adaptive-mission-provider"
        )
        self._provider_future: Optional[Future[Mapping[str, Any]]] = None
        self._terminal = False
        self._status = "approved"
        self._terminal_reason = ""
        self._world = json.loads(json.dumps(dict(first_snapshot)))
        self._active_intent: Optional[AdaptiveMissionIntent] = first_intent
        self._revisions: list[dict[str, Any]] = []
        self._snapshots = [json.loads(json.dumps(self._world))]
        self._events: list[dict[str, Any]] = []
        self._provider_calls_started = 1
        self._provider_calls_completed = 1
        self._inference_in_flight = False
        self._execution_in_flight = False
        self._requested_terminal: Optional[tuple[str, str]] = None
        self._append_event(
            "approval_bound",
            f"Authenticated operator {self.operator} approved lease through "
            f"{self._mission_expires_at_s:.3f}; no per-intent approval is required.",
        )

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                raise RuntimeError("Adaptive mission controller already started")
            self._status = "running"
            self._thread = threading.Thread(
                target=self._run,
                name=f"adaptive-mission-{self.mission_id}",
                daemon=True,
            )
            self._thread.start()

    def cancel(self) -> None:
        self._request_terminal("cancelled", "operator_cancelled")

    def stop(self) -> None:
        self._request_terminal("stopped", "stop_requested")

    def estop(self) -> None:
        self._request_terminal("estopped", "estop_latched")

    def collision_stop(self) -> None:
        self._request_terminal("blocked", "collision_veto")

    def close(self, *, timeout_s: float = 10.0) -> None:
        self._shutdown.set()
        self._cancellation.set()
        cancel_executor = getattr(self.executor, "cancel", None)
        if callable(cancel_executor):
            cancel_executor()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, float(timeout_s)))
        if thread is not None and thread.is_alive():
            raise RuntimeError(
                "Adaptive mission controller shutdown did not prove executor cleanup"
            )
        self._provider_pool.shutdown(wait=False, cancel_futures=True)
        close_executor = getattr(self.executor, "close", None)
        if self._owns_executor and callable(close_executor):
            close_executor()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._projection()

    def _run(self) -> None:
        while not self._shutdown.is_set():
            checkpoint: Optional[tuple[str, dict[str, Any]]] = None
            with self._lock:
                if self._terminal:
                    return
                if self._cancellation.is_set():
                    self._finish_requested_terminal()
                    return
                if self._now() >= self._mission_expires_at_s:
                    self._finish("timeout", "mission_lease_expired")
                    return
                intent = self._active_intent
                if intent is None:
                    self._finish("failed", "missing_intent")
                    return
                try:
                    validate_world_snapshot(
                        self._world,
                        mission_id=self.mission_id,
                        require_motion=intent.action in {"move_distance", "turn_angle"},
                    )
                except MissionValidationError as exc:
                    self._finish("blocked", f"stale_or_unsafe_evidence: {exc}")
                    return
                if self._now() > intent.expires_at_s:
                    self._finish("timeout", "intent_lease_expired")
                    return
                revision = intent.to_json_dict()
                self._append_event(
                    "intent_started",
                    f"Revision {intent.revision}: {intent.action} — {intent.rationale}",
                )

            with self._lock:
                self._execution_in_flight = True
            execution = self.executor.execute(intent, self._cancellation)

            with self._lock:
                self._execution_in_flight = False
                if self._terminal:
                    return
                revision["execution"] = execution.to_json_dict()
                self._revisions.append(revision)
                self._world = json.loads(json.dumps(dict(execution.snapshot)))
                self._snapshots.append(json.loads(json.dumps(self._world)))
                self._append_event(
                    "intent_result",
                    f"Revision {intent.revision} {execution.outcome}: "
                    f"requested {execution.movement.requested_linear_mps:+.2f} m/s, "
                    f"{execution.movement.requested_angular_rad_s:+.2f} rad/s; "
                    f"supervised {execution.movement.supervised_linear_mps:+.2f} m/s, "
                    f"{execution.movement.supervised_angular_rad_s:+.2f} rad/s.",
                )
                if execution.outcome != "completed":
                    if (
                        self._requested_terminal is not None
                        and "cleanup_uncertain" not in execution.reason
                    ):
                        requested_status, _ = self._requested_terminal
                        self._finish(requested_status, execution.reason)
                        return
                    terminal_status = {
                        "blocked": "blocked",
                        "stale": "blocked",
                        "cancelled": "cancelled",
                        "timeout": "timeout",
                    }.get(execution.outcome, "failed")
                    if "cleanup_uncertain" in execution.reason:
                        terminal_status = "recovery_required"
                    self._finish(terminal_status, execution.reason)
                    return
                checkpoint = ("intent_result", self._projection())
                if intent.action == "stop":
                    self._finish("complete", "planner_stop")
                    return
                if self._now() >= self._mission_expires_at_s:
                    self._finish("timeout", "mission_lease_expired")
                    return
                try:
                    validate_world_snapshot(
                        self._world,
                        mission_id=self.mission_id,
                        require_motion=False,
                    )
                except MissionValidationError as exc:
                    self._finish("blocked", f"stale_or_unsafe_evidence: {exc}")
                    return
                self._provider_calls_started += 1
                self._inference_in_flight = True
                self._append_event(
                    "llm_revision_started",
                    f"Provider call {self._provider_calls_started} received updated "
                    f"snapshot {str(self._world.get('snapshot_id', ''))[:12]}.",
                )
            if checkpoint is not None:
                self._emit_checkpoint(*checkpoint)
            try:
                future = self._provider_pool.submit(
                    self.provider.choose, self.prompt, self._world
                )
                self._provider_future = future
                while not future.done():
                    if self._shutdown.wait(0.02):
                        future.cancel()
                        return
                    with self._lock:
                        if self._terminal:
                            future.cancel()
                            return
                        if self._cancellation.is_set():
                            self._finish_requested_terminal()
                            future.cancel()
                            return
                        if self._now() >= self._mission_expires_at_s:
                            self._finish("timeout", "mission_lease_expired")
                            future.cancel()
                            return
                raw = future.result()
                self._provider_future = None
                issued_at = self._now()
                next_intent = AdaptiveMissionIntent.validated(
                    raw,
                    revision=len(self._revisions) + 1,
                    snapshot=self._world,
                    issued_at_s=issued_at,
                    provider_id=self.provider.provider_id,
                    model_id=self.provider.model_id,
                    limits=self.limits,
                )
            except Exception as exc:
                with self._lock:
                    self._provider_future = None
                    self._inference_in_flight = False
                    if not self._terminal:
                        self._finish(
                            "failed",
                            f"provider_failure: {exc.__class__.__name__}: {exc}",
                        )
                return
            with self._lock:
                self._provider_calls_completed += 1
                self._inference_in_flight = False
                if self._terminal or self._cancellation.is_set():
                    if not self._terminal:
                        self._finish_requested_terminal()
                    return
                if self._now() >= self._mission_expires_at_s:
                    self._finish("timeout", "mission_lease_expired")
                    return
                self._active_intent = next_intent
                self._append_event(
                    "llm_revision",
                    f"LLM chose revision {next_intent.revision}: "
                    f"{next_intent.action} — {next_intent.rationale}",
                )
                checkpoint = ("llm_revision", self._projection())
            self._emit_checkpoint(*checkpoint)

    def _finish(self, status: str, reason: str) -> None:
        if self._terminal:
            return
        self._terminal = True
        self._status = str(status)
        self._terminal_reason = str(reason)
        self._inference_in_flight = False
        self._execution_in_flight = False
        self._append_event(
            "terminal",
            f"Adaptive mission loop ended {status}: {reason}; automatic resumption is disabled.",
        )
        self._emit_checkpoint("terminal", self._projection())

    def _request_terminal(self, status: str, reason: str) -> None:
        self._cancellation.set()
        with self._lock:
            if self._terminal:
                return
            self._requested_terminal = (str(status), str(reason))
            if not self._execution_in_flight:
                self._finish_requested_terminal()

    def _finish_requested_terminal(self) -> None:
        status, reason = self._requested_terminal or (
            "cancelled",
            "operator_cancelled",
        )
        self._finish(status, reason)

    def _append_event(self, kind: str, message: str) -> None:
        self._events.append(
            {
                "sequence": len(self._events) + 1,
                "event_type": str(kind),
                "message": str(message),
                "at_s": self._now(),
            }
        )

    def _emit_checkpoint(self, kind: str, projection: Mapping[str, Any]) -> None:
        if self._checkpoint is None:
            return
        try:
            self._checkpoint(str(kind), projection)
        except Exception:
            if str(kind) != "terminal":
                with self._lock:
                    if not self._terminal:
                        self._terminal = True
                        self._status = "failed"
                        self._terminal_reason = "persistence_checkpoint_failed"

    def _projection(self) -> dict[str, Any]:
        progress = self._world.get("progress", {})
        result = self._terminal_result() if self._terminal else {}
        map_projection = getattr(self.executor, "map_projection", None)
        map_payload = (
            map_projection()
            if callable(map_projection)
            else {
                "available": False,
                "fixture_only": False,
                "unavailable_reason": "executor supplied no spatial projection",
            }
        )
        return {
            "schema": ADAPTIVE_MISSION_RESULT_SCHEMA,
            "mission_id": self.mission_id,
            "status": self._status,
            "terminal": self._terminal,
            "terminal_reason": self._terminal_reason,
            "progress": 1.0
            if self._terminal
            else min(0.99, len(self._revisions) / 10.0),
            "world_snapshot": json.loads(json.dumps(self._world)),
            "world_snapshots": json.loads(json.dumps(self._snapshots)),
            "active_intent": (
                None
                if self._active_intent is None
                else self._active_intent.to_json_dict()
            ),
            "intent_revisions": json.loads(json.dumps(self._revisions)),
            "inference": {
                "in_flight": self._inference_in_flight,
                "call": (
                    self._provider_calls_started
                    if self._inference_in_flight
                    else None
                ),
                "provider_calls_started": self._provider_calls_started,
                "provider_calls_completed": self._provider_calls_completed,
            },
            "metrics": {
                "intent_revision_count": len(self._revisions),
                "completed_intents": sum(
                    1
                    for item in self._revisions
                    if item.get("execution", {}).get("outcome") == "completed"
                ),
                "provider_calls_started": self._provider_calls_started,
                "provider_calls_completed": self._provider_calls_completed,
                "cumulative_translation_m": (
                    progress.get("cumulative_translation_m", 0.0)
                    if isinstance(progress, Mapping)
                    else 0.0
                ),
                "cumulative_rotation_deg": (
                    progress.get("cumulative_rotation_deg", 0.0)
                    if isinstance(progress, Mapping)
                    else 0.0
                ),
                "cumulative_travel_limit": "none within mission lease",
            },
            "mission_lease": {
                "approved_at_s": self.approved_at_s,
                "expires_at_s": self._mission_expires_at_s,
                "remaining_s": max(0.0, self._mission_expires_at_s - self._now()),
                "duration_s": self.limits.mission_lease_s,
                "proposal_digest": self.proposal_digest,
                "operator": self.operator,
                "authenticated": True,
                "authentication_source": self.authentication_source,
            },
            "events": json.loads(json.dumps(self._events)),
            "map": map_payload,
            "result": result,
            "motion_authority": False,
            "physical_execution_enabled": bool(
                getattr(self.executor, "execution_enabled", False)
            ),
        }

    def _terminal_result(self) -> dict[str, Any]:
        return {
            "schema": ADAPTIVE_MISSION_RESULT_SCHEMA,
            "mission_id": self.mission_id,
            "status": self._status,
            "terminal_reason": self._terminal_reason,
            "final_snapshot": json.loads(json.dumps(self._world)),
            "world_snapshots": json.loads(json.dumps(self._snapshots)),
            "intent_revisions": json.loads(json.dumps(self._revisions)),
            "provider": {
                "provider_id": self.provider.provider_id,
                "model_id": self.provider.model_id,
                "reasoning_effort": self.provider.reasoning_effort,
                "calls_started": self._provider_calls_started,
                "calls_completed": self._provider_calls_completed,
            },
            "approval": {
                "operator": self.operator,
                "authenticated": True,
                "authentication_source": self.authentication_source,
                "proposal_digest": self.proposal_digest,
                "approved_at_s": self.approved_at_s,
                "expires_at_s": self._mission_expires_at_s,
            },
            "limits": self.limits.to_json_dict(),
            "safety_policy": ADAPTIVE_MISSION_SAFETY_POLICY,
            "auto_resume": False,
            "drop_off_detection_available": False,
            "motor_topic_publisher": "lidar_collision_stop_supervisor",
            "motion_authority": False,
            "physical_execution_enabled": bool(
                getattr(self.executor, "execution_enabled", False)
            ),
        }


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise MissionValidationError(f"{name} must be finite") from exc
    if not math.isfinite(number):
        raise MissionValidationError(f"{name} must be finite")
    return number
