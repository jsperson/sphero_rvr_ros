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
import logging
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Any, Callable, Mapping, Optional, Protocol
import uuid

from .codex_app_server import (
    CodexAppServerClient,
    codex_oauth_environment,
    resolve_codex_executable,
)
from .mission_api import MissionValidationError
from .prompt_drive import ALLOWED_REASONING_EFFORTS, DEFAULT_CODEX_MODEL_ID


ADAPTIVE_MISSION_WORLD_SCHEMA = "sphero_rvr.adaptive_mission_world_snapshot.v1"
ADAPTIVE_MISSION_INTENT_SCHEMA = "sphero_rvr.adaptive_mission_intent.v1"
ADAPTIVE_MISSION_PROPOSAL_SCHEMA = "sphero_rvr.adaptive_mission_proposal.v1"
ADAPTIVE_MISSION_RESULT_SCHEMA = "sphero_rvr.adaptive_mission_result.v1"
ADAPTIVE_MISSION_SAFETY_REJECTION_SCHEMA = (
    "sphero_rvr.adaptive_mission_safety_rejection.v1"
)
ADAPTIVE_MISSION_SAFETY_POLICY = "lidar_collision_stop.v1"
DEFAULT_SAFETY_REJECTION_RETRY_BUDGET = 2

_ACTIONS = {"move_distance", "turn_angle", "observe", "stop"}
_OBJECTIVE_STATUSES = {
    "in_progress",
    "needs_observation",
    "complete",
    "blocked",
}
_CLEAR_COLLISION_STATES = {"CLEAR", "SLOW"}
_LOG = logging.getLogger(__name__)


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


class RecoverableSafetyRejection(MissionValidationError):
    """A proposal rejected before motion by one numeric safety boundary."""

    def __init__(
        self,
        message: str,
        *,
        rejected_request: Mapping[str, Any],
        violated_condition: str,
        limit_name: str,
        limit_value: float,
        limit_unit: str,
        rejected_snapshot_id: str,
    ) -> None:
        super().__init__(message)
        self.rejected_request = json.loads(
            json.dumps(dict(rejected_request), allow_nan=False)
        )
        self.violated_condition = str(violated_condition)
        self.limit_name = str(limit_name)
        self.limit_value = float(limit_value)
        self.limit_unit = str(limit_unit)
        self.rejected_snapshot_id = str(rejected_snapshot_id)

    def feedback(self, *, current_snapshot_id: str) -> dict[str, Any]:
        return {
            "schema": ADAPTIVE_MISSION_SAFETY_REJECTION_SCHEMA,
            "rejected_request": json.loads(
                json.dumps(self.rejected_request)
            ),
            "violated_condition": self.violated_condition,
            "applicable_numeric_limit": {
                "name": self.limit_name,
                "value": self.limit_value,
                "unit": self.limit_unit,
            },
            "rejected_snapshot_id": self.rejected_snapshot_id,
            "current_snapshot_id": str(current_snapshot_id),
            "motion_executed": False,
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
    require_execution_safety: bool = True,
    allow_supervised_collision_escape: bool = False,
    allow_collision_stopped_observation: bool = False,
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
    perception = observations.get("perception", {})
    camera_image = (
        perception.get("camera_image", {})
        if isinstance(perception, Mapping)
        else {}
    )
    if camera_image and not isinstance(camera_image, Mapping):
        raise MissionValidationError(
            "adaptive mission camera image attachment must be an object"
        )
    if (
        isinstance(camera_image, Mapping)
        and camera_image.get("available") is True
        and (
            str(camera_image.get("frame_id", ""))
            != str(perception.get("camera_frame_id", ""))
            or str(camera_image.get("mime_type", "")) != "image/jpeg"
            or not Path(str(camera_image.get("path", ""))).is_absolute()
            or not isinstance(camera_image.get("byte_count"), int)
            or isinstance(camera_image.get("byte_count"), bool)
            or not 1 <= int(camera_image.get("byte_count", 0)) <= 512_000
            or len(str(camera_image.get("sha256", ""))) != 64
        )
    ):
        raise MissionValidationError(
            "adaptive mission camera image attachment metadata is invalid"
        )
    if not require_execution_safety:
        if require_motion:
            raise MissionValidationError(
                "observation-only snapshot validation cannot authorize motion"
            )
        if execution.get("motion_permitted") is not False:
            raise MissionValidationError(
                "observation-only snapshot unexpectedly permits motion"
            )
        receipts = evidence.get("source_receipts")
        if not isinstance(receipts, Mapping):
            raise MissionValidationError(
                "observation-only snapshot lacks typed source receipts"
            )
        for name in ("lidar", "localization"):
            receipt = receipts.get(name)
            if (
                not isinstance(receipt, Mapping)
                or receipt.get("fresh") is not True
                or receipt.get("valid") is not True
            ):
                raise MissionValidationError(
                    f"stale observation evidence: {name}"
                )
        return
    for name in ("scan_fresh", "transform_fresh", "odometry_fresh"):
        if evidence.get(name) is not True:
            raise MissionValidationError(f"stale evidence: {name}")
    collision_state = str(safety.get("collision_state", "")).upper()
    control_state = str(safety.get("control_state", "")).upper()
    collision_escape = bool(
        allow_supervised_collision_escape
        and collision_state in {"STOP", "STOPPED", "BLOCKED"}
        and control_state not in {
            "STOP",
            "STOPPED",
            "ESTOP",
            "ESTOPPED",
            "LATCHED",
        }
    )
    collision_observation = bool(
        allow_collision_stopped_observation
        and not require_motion
        and collision_state in {"STOP", "STOPPED", "BLOCKED"}
        and control_state not in {
            "ESTOP",
            "ESTOPPED",
            "LATCHED",
        }
    )
    if (
        safety.get("stop_active") is True
        and not collision_escape
        and not collision_observation
    ):
        raise MissionValidationError("STOP is active")
    if safety.get("estop_latched") is True:
        raise MissionValidationError("ESTOP is latched")
    if (
        collision_state not in _CLEAR_COLLISION_STATES
        and not collision_escape
        and not collision_observation
    ):
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
    objective_status: str
    lease_s: float
    timeout_s: float
    issued_at_s: float
    expires_at_s: float
    provider_id: str
    model_id: str
    supervised_collision_escape: bool = False

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
        supervised_collision_escape: bool = False,
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
            raise RecoverableSafetyRejection(
                "adaptive mission intent translation exceeds 0.25 m",
                rejected_request=raw,
                violated_condition="translation_per_intent_limit_exceeded",
                limit_name="max_translation_per_intent_m",
                limit_value=limits.max_translation_per_intent_m,
                limit_unit="m",
                rejected_snapshot_id=str(snapshot.get("snapshot_id", "")),
            )
        if abs(angle) > limits.max_rotation_per_intent_deg:
            raise RecoverableSafetyRejection(
                "adaptive mission intent rotation exceeds 45 degrees",
                rejected_request=raw,
                violated_condition="rotation_per_intent_limit_exceeded",
                limit_name="max_rotation_per_intent_deg",
                limit_value=limits.max_rotation_per_intent_deg,
                limit_unit="deg",
                rejected_snapshot_id=str(snapshot.get("snapshot_id", "")),
            )
        if lease <= 0.0:
            raise MissionValidationError(
                "adaptive mission intent lease must be positive"
            )
        if lease > limits.max_intent_lease_s:
            raise RecoverableSafetyRejection(
                "adaptive mission intent lease exceeds 5 seconds",
                rejected_request=raw,
                violated_condition="intent_lease_limit_exceeded",
                limit_name="max_intent_lease_s",
                limit_value=limits.max_intent_lease_s,
                limit_unit="s",
                rejected_snapshot_id=str(snapshot.get("snapshot_id", "")),
            )
        if timeout <= 0.0:
            raise MissionValidationError(
                "adaptive mission intent timeout must be positive"
            )
        if timeout > limits.max_intent_timeout_s:
            raise RecoverableSafetyRejection(
                "adaptive mission intent timeout exceeds 5 seconds",
                rejected_request=raw,
                violated_condition="intent_timeout_limit_exceeded",
                limit_name="max_intent_timeout_s",
                limit_value=limits.max_intent_timeout_s,
                limit_unit="s",
                rejected_snapshot_id=str(snapshot.get("snapshot_id", "")),
            )
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
        if action == "move_distance":
            _validate_snapshot_translation_clearance(
                snapshot, distance, rejected_request=raw
            )
        rationale = str(raw.get("rationale", "")).strip()
        objective = str(raw.get("interpreted_objective", "")).strip()
        objective_status = str(raw.get("objective_status", "")).strip()
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
        if objective_status not in _OBJECTIVE_STATUSES:
            raise MissionValidationError(
                "adaptive mission objective status is unsupported"
            )
        if action in {"move_distance", "turn_angle"} and objective_status != "in_progress":
            raise MissionValidationError(
                "motion requires an in_progress objective status"
            )
        if action == "observe" and objective_status not in {
            "in_progress",
            "needs_observation",
        }:
            raise MissionValidationError(
                "observe requires an in_progress or needs_observation objective status"
            )
        if action == "stop" and objective_status not in {"complete", "blocked"}:
            raise MissionValidationError(
                "stop requires a complete or blocked objective status"
            )
        collision_escape = bool(supervised_collision_escape)
        if collision_escape and not (
            action == "move_distance" and distance < 0.0
        ):
            raise MissionValidationError(
                "supervised collision escape is restricted to reverse motion"
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
            objective_status=objective_status,
            lease_s=lease,
            timeout_s=timeout,
            issued_at_s=float(issued_at_s),
            expires_at_s=float(issued_at_s) + lease,
            provider_id=str(provider_id),
            model_id=str(model_id),
            supervised_collision_escape=collision_escape,
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
            "objective_status": self.objective_status,
            "lease_s": self.lease_s,
            "timeout_s": self.timeout_s,
            "issued_at_s": self.issued_at_s,
            "expires_at_s": self.expires_at_s,
            "provider_id": self.provider_id,
            "model_id": self.model_id,
            "supervised_collision_escape": self.supervised_collision_escape,
        }


class AdaptiveMissionIntentProvider(Protocol):
    provider_id: str
    model_id: str
    reasoning_effort: str

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]: ...

    def choose_after_safety_rejection(
        self,
        prompt: str,
        snapshot: Mapping[str, Any],
        rejection: Mapping[str, Any],
    ) -> Mapping[str, Any]: ...


class CodexOAuthAdaptiveMissionIntentProvider:
    """ChatGPT-OAuth planner with isolated turns on one supervised app-server."""

    provider_id = "openai-codex-oauth"

    def __init__(
        self,
        *,
        model: Optional[str] = None,
        reasoning_effort: str = "low",
        codex_command: str = "codex",
        timeout_s: float = 120.0,
        limits: Optional[AdaptiveMissionLimits] = None,
        integration: str = "app-server",
        compact_input: bool = True,
        latency_logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        if reasoning_effort not in ALLOWED_REASONING_EFFORTS:
            raise MissionValidationError(
                f"unsupported reasoning effort: {reasoning_effort}"
            )
        if integration not in {"app-server", "exec"}:
            raise MissionValidationError(
                f"unsupported Codex OAuth integration: {integration}"
            )
        self.model_id = model or os.environ.get(
            "OPENAI_MODEL", DEFAULT_CODEX_MODEL_ID
        )
        self.reasoning_effort = reasoning_effort
        self.codex_command = str(codex_command)
        self.timeout_s = float(timeout_s)
        self.limits = limits or AdaptiveMissionLimits()
        self.integration = str(integration)
        self.compact_input = bool(compact_input)
        self._latency_logger = latency_logger or _LOG.info
        self._oauth_checked = False
        self._oauth_lock = threading.Lock()
        self._client = (
            CodexAppServerClient(codex_command=self.codex_command)
            if self.integration == "app-server"
            else None
        )
        self._latency_lock = threading.Lock()
        self._latency_history: list[dict[str, Any]] = []
        self._cycle_sequence = 0

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        return self._choose(
            prompt, snapshot, safety_rejection=None
        )

    def choose_after_safety_rejection(
        self,
        prompt: str,
        snapshot: Mapping[str, Any],
        rejection: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        return self._choose(
            prompt,
            snapshot,
            safety_rejection=rejection,
        )

    def _choose(
        self,
        prompt: str,
        snapshot: Mapping[str, Any],
        *,
        safety_rejection: Optional[Mapping[str, Any]],
    ) -> Mapping[str, Any]:
        total_started = time.perf_counter()
        preparation_started = total_started
        decision_id = uuid.uuid4().hex
        metric: dict[str, Any] = {
            "schema": "sphero_rvr.adaptive_planning_latency.v1",
            "decision_id": decision_id,
            "isolated_model_thread": True,
            "snapshot_id": str(snapshot.get("snapshot_id", "")),
            "safety_recovery": safety_rejection is not None,
            "safety_rejection_condition": (
                str(safety_rejection.get("violated_condition", ""))
                if isinstance(safety_rejection, Mapping)
                else ""
            ),
            "model_id": self.model_id,
            "reasoning_effort": self.reasoning_effort,
            "integration": self.integration,
            "compact_input": self.compact_input,
            "prompt_image_preparation_ms": 0.0,
            "oauth_client_startup_ms": 0.0,
            "inference_ms": 0.0,
            "validation_ms": 0.0,
            "total_ms": 0.0,
            "image_attached": False,
            "image_reason": "not_relevant",
            "input_characters": 0,
            "server_restart_count": 0,
            "success": False,
            "error_type": "",
        }
        payload: Any = None
        try:
            with tempfile.TemporaryDirectory(
                prefix="rvr-adaptive-mission-"
            ) as directory:
                root = Path(directory)
                decision_snapshot = (
                    _decision_evidence_snapshot(snapshot)
                    if self.compact_input
                    else json.loads(json.dumps(dict(snapshot)))
                )
                image_required, image_reason = _visual_reasoning_relevance(
                    prompt, snapshot
                )
                metric["image_reason"] = image_reason
                attach_available_image = image_required or not self.compact_input
                camera_path = (
                    _verified_camera_attachment(snapshot, root)
                    if attach_available_image
                    else None
                )
                if image_required and camera_path is None:
                    raise MissionValidationError(
                        "visual planning decision lacks a fresh digest-bound camera image"
                    )
                metric["image_attached"] = camera_path is not None
                decision_observations = decision_snapshot.get(
                    "observations", {}
                )
                if isinstance(decision_observations, dict):
                    decision_perception = decision_observations.get(
                        "perception", {}
                    )
                    if isinstance(decision_perception, dict):
                        decision_image = decision_perception.get(
                            "camera_image", {}
                        )
                        if isinstance(decision_image, dict):
                            decision_image["attached"] = (
                                camera_path is not None
                            )
                            decision_image["attachment_reason"] = image_reason
                provider_prompt = _adaptive_mission_provider_prompt(
                    prompt,
                    decision_snapshot,
                    limits=self.limits,
                    safety_rejection=safety_rejection,
                )
                metric["input_characters"] = len(provider_prompt)
                metric["prompt_image_preparation_ms"] = (
                    time.perf_counter() - preparation_started
                ) * 1000.0
                if self.integration == "app-server":
                    inference_started = time.perf_counter()
                    assert self._client is not None
                    output, startup_ms, restart_count = self._client.run_turn(
                        prompt=provider_prompt,
                        model=self.model_id,
                        effort=self.reasoning_effort,
                        output_schema=_adaptive_mission_output_schema(),
                        cwd=str(root),
                        image_path=(
                            str(camera_path)
                            if camera_path is not None
                            else None
                        ),
                        timeout_s=self.timeout_s,
                    )
                    inference_elapsed_ms = (
                        time.perf_counter() - inference_started
                    ) * 1000.0
                    metric["oauth_client_startup_ms"] = startup_ms
                    metric["inference_ms"] = max(
                        0.0, inference_elapsed_ms - startup_ms
                    )
                    metric["server_restart_count"] = restart_count
                else:
                    output, startup_ms, inference_ms = self._choose_ephemeral(
                        provider_prompt=provider_prompt,
                        root=root,
                        camera_path=camera_path,
                    )
                    metric["oauth_client_startup_ms"] = startup_ms
                    metric["inference_ms"] = inference_ms
                validation_started = time.perf_counter()
                try:
                    payload = json.loads(output)
                except json.JSONDecodeError as exc:
                    raise MissionValidationError(
                        "Codex OAuth adaptive mission intent call returned malformed output"
                    ) from exc
                if not isinstance(payload, Mapping):
                    raise MissionValidationError(
                        "Codex OAuth adaptive mission intent output must be an object"
                    )
                metric["validation_ms"] = (
                    time.perf_counter() - validation_started
                ) * 1000.0
            metric["success"] = True
            return dict(payload)
        except Exception as exc:
            metric["error_type"] = exc.__class__.__name__
            raise
        finally:
            metric["total_ms"] = (
                time.perf_counter() - total_started
            ) * 1000.0
            self._record_latency(metric)

    def _choose_ephemeral(
        self,
        *,
        provider_prompt: str,
        root: Path,
        camera_path: Optional[Path],
    ) -> tuple[str, float, float]:
        executable = resolve_codex_executable(self.codex_command)
        if executable is None:
            raise MissionValidationError(
                "Codex CLI is not installed; Adaptive mission requires the real OAuth provider"
            )
        env = codex_oauth_environment()
        startup_started = time.perf_counter()
        self._require_chatgpt_oauth(executable, env)
        startup_ms = (time.perf_counter() - startup_started) * 1000.0
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
        ]
        if camera_path is not None:
            command.extend(["--image", str(camera_path)])
        command.append("-")
        inference_started = time.perf_counter()
        try:
            completed = subprocess.run(
                command,
                input=provider_prompt,
                text=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=self.timeout_s,
                env=env,
                cwd=root,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise MissionValidationError(
                "Codex OAuth adaptive mission intent call timed out"
            ) from exc
        inference_ms = (time.perf_counter() - inference_started) * 1000.0
        if completed.returncode != 0:
            raise MissionValidationError(
                "Codex OAuth adaptive mission intent call failed with exit code "
                f"{completed.returncode}; inspect Pi Codex logs"
            )
        try:
            output = output_path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError) as exc:
            raise MissionValidationError(
                "Codex OAuth adaptive mission intent call returned malformed output"
            ) from exc
        return output, startup_ms, inference_ms

    def latency_history(self) -> list[dict[str, Any]]:
        with self._latency_lock:
            return json.loads(json.dumps(self._latency_history))

    def add_validation_latency(
        self,
        snapshot_id: str,
        *,
        validation_ms: float,
        valid: bool,
    ) -> None:
        """Add deterministic rover-schema validation to the latest cycle."""

        with self._latency_lock:
            for metric in reversed(self._latency_history):
                if metric["snapshot_id"] == str(snapshot_id):
                    metric["validation_ms"] = round(
                        float(metric["validation_ms"]) + float(validation_ms), 3
                    )
                    metric["total_ms"] = round(
                        float(metric["total_ms"]) + float(validation_ms), 3
                    )
                    if not valid:
                        metric["success"] = False
                        metric["error_type"] = "MissionValidationError"
                    self._latency_logger(
                        "adaptive_planning_cycle_validated "
                        + json.dumps(
                            metric,
                            sort_keys=True,
                            separators=(",", ":"),
                        )
                    )
                    break

    def cancel(self) -> None:
        if self._client is not None:
            self._client.cancel()

    def close(self) -> None:
        if self._client is not None:
            self._client.close()

    def _record_latency(self, metric: Mapping[str, Any]) -> None:
        sanitized = dict(metric)
        with self._latency_lock:
            self._cycle_sequence += 1
            sanitized["cycle"] = self._cycle_sequence
            for name in (
                "prompt_image_preparation_ms",
                "oauth_client_startup_ms",
                "inference_ms",
                "validation_ms",
                "total_ms",
            ):
                sanitized[name] = round(float(sanitized[name]), 3)
            self._latency_history.append(sanitized)
            del self._latency_history[:-256]
        self._latency_logger(
            "adaptive_planning_cycle "
            + json.dumps(
                sanitized,
                sort_keys=True,
                separators=(",", ":"),
            )
        )

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


def choose_validated_adaptive_intent(
    provider: AdaptiveMissionIntentProvider,
    prompt: str,
    snapshot: Mapping[str, Any],
    *,
    revision: int,
    issued_at_s: Optional[float],
    limits: AdaptiveMissionLimits,
    supervised_collision_escape: Optional[bool] = None,
    safety_rejection: Optional[Mapping[str, Any]] = None,
    issue_clock: Callable[[], float] = time.time,
) -> tuple[dict[str, Any], AdaptiveMissionIntent]:
    """Run provider inference and deterministic rover intent validation.

    Passing ``issued_at_s=None`` starts the finite intent lease after provider
    inference returns, so inference latency cannot consume motion authority.
    """

    raw = dict(
        _choose_provider_intent(
            provider,
            prompt,
            snapshot,
            safety_rejection=safety_rejection,
        )
    )
    effective_issued_at_s = (
        float(issue_clock()) if issued_at_s is None else float(issued_at_s)
    )
    collision_escape = (
        bool(
            _is_collision_replan_snapshot(snapshot)
            and str(raw.get("action", "")).strip() == "move_distance"
            and _finite(
                raw.get("distance_m"),
                "adaptive mission collision escape distance",
            )
            < 0.0
        )
        if supervised_collision_escape is None
        else bool(supervised_collision_escape)
    )
    validation_started = time.perf_counter()
    valid = False
    try:
        intent = AdaptiveMissionIntent.validated(
            raw,
            revision=revision,
            snapshot=snapshot,
            issued_at_s=effective_issued_at_s,
            provider_id=provider.provider_id,
            model_id=provider.model_id,
            limits=limits,
            supervised_collision_escape=collision_escape,
        )
        valid = True
        return raw, intent
    finally:
        record = getattr(provider, "add_validation_latency", None)
        if callable(record):
            record(
                str(snapshot.get("snapshot_id", "")),
                validation_ms=(
                    time.perf_counter() - validation_started
                )
                * 1000.0,
                valid=valid,
            )


def _choose_provider_intent(
    provider: AdaptiveMissionIntentProvider,
    prompt: str,
    snapshot: Mapping[str, Any],
    *,
    safety_rejection: Optional[Mapping[str, Any]],
) -> Mapping[str, Any]:
    if safety_rejection is None:
        return provider.choose(prompt, snapshot)
    recovery_choose = getattr(
        provider, "choose_after_safety_rejection", None
    )
    if not callable(recovery_choose):
        raise MissionValidationError(
            "adaptive mission provider cannot start an isolated safety-recovery decision"
        )
    return recovery_choose(prompt, snapshot, safety_rejection)


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
            "objective_status": {
                "type": "string",
                "enum": sorted(_OBJECTIVE_STATUSES),
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
            "objective_status",
            "lease_s",
            "timeout_s",
        ],
    }


def _adaptive_mission_provider_prompt(
    prompt: str,
    snapshot: Mapping[str, Any],
    *,
    limits: Optional[AdaptiveMissionLimits] = None,
    safety_rejection: Optional[Mapping[str, Any]] = None,
) -> str:
    authority_limits = limits or AdaptiveMissionLimits()
    observations = snapshot.get("observations", {})
    motion_clearance = (
        observations.get("motion_clearance", {})
        if isinstance(observations, Mapping)
        else {}
    )
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
            "mission_lease_s": authority_limits.mission_lease_s,
            "cumulative_travel": "unlimited until mission lease expires",
            "translation_clearance": (
                dict(motion_clearance)
                if isinstance(motion_clearance, Mapping)
                else {}
            ),
        },
        "rules": [
            "Return exactly one intent bound to world_snapshot.snapshot_id.",
            "Choose the exploration strategy from current evidence and revise after every executor result.",
            "Never emit a route, ROS topic, Twist, motor command, speed, safety threshold, shell action, credential, or claim of unobserved completion.",
            "move_distance uses signed distance_m and zero angle_deg.",
            "turn_angle uses signed angle_deg (positive left) and zero distance_m.",
            "observe and stop use zero distance_m and zero angle_deg.",
            "Set objective_status to in_progress for motion, in_progress or needs_observation for observe, and complete or blocked for stop.",
            "Use lease_s 5 and timeout_s 5.",
            "Treat collision, STOP, ESTOP, and freshness evidence as authoritative.",
            "Reason explicitly from world_snapshot.last_execution.navigation_outcome when present; a recoverable settled motion error or obstacle does not itself mean the objective is complete.",
            "After collision_veto or stall, consider observe, a turn only when collision safety is CLEAR or SLOW, or a reverse no greater than typed reverse_usable_m; the controller independently decides whether any reverse may be submitted as a supervised collision escape.",
            "Treat stall as commanded motion not occurring as expected: do not immediately repeat the same forward move or compensate with higher speed or motor authority; use fresh evidence to back away, change the approach angle, observe, retry after a maneuver, or stop blocked.",
            "After any recoverable navigation outcome, reverse motion is additionally limited to 0.15 m.",
            "Before move_distance, require the signed distance magnitude to be no greater than authority.translation_clearance.forward_usable_m for forward motion or reverse_usable_m for reverse motion; if that value is missing or insufficient, choose turn_angle, observe, or stop.",
            "Camera detections and semantic tracks are objective evidence only; they never override lidar collision safety.",
            "When observations.perception.camera_image.attached is true, an exact frame- and SHA-bound camera image is attached as advisory evidence; use it to assess whether a stalled path shows a likely fixed obstacle, a likely minor surface feature, or remains visually indeterminate.",
            "State that visual assessment in the rationale after a stall. Never use the image to override lidar, STOP, ESTOP, clearance, freshness, or deterministic motion limits.",
            "Use semantic tracks for object- or person-directed movement only when observations.perception.available is true.",
            "A face label is authoritative only when recognized_from_enrollment is true and enrollment_evidence_ids supplies explicit enrollment evidence; every other face is unknown.",
            "Never infer a visible object, identity, or map position from missing or stale perception.",
            "Drop-offs are outside the sensed model; never claim lidar detects an edge or cliff.",
            "When execution.motion_permitted is false, choose observe if progress.observation_count is zero; otherwise choose stop.",
            "Use stop with objective_status complete only when current evidence demonstrates the objective is complete. Use stop with objective_status blocked only when no validated observation or maneuver can pursue it truthfully.",
            "Ground the concise rationale only in this snapshot.",
        ],
    }
    if safety_rejection is not None:
        rejection = json.loads(
            json.dumps(dict(safety_rejection), allow_nan=False)
        )
        if (
            rejection.get("schema")
            != ADAPTIVE_MISSION_SAFETY_REJECTION_SCHEMA
            or rejection.get("motion_executed") is not False
            or str(rejection.get("current_snapshot_id", ""))
            != str(snapshot.get("snapshot_id", ""))
            or not isinstance(
                rejection.get("applicable_numeric_limit"), Mapping
            )
        ):
            raise MissionValidationError(
                "adaptive mission safety rejection feedback is invalid"
            )
        request["safety_rejection"] = rejection
        suggestion_prefixes = (
            "After collision_veto or stall, consider ",
            "Treat stall as commanded motion not occurring as expected:",
            "Before move_distance, require the signed distance magnitude ",
        )
        request["rules"] = [
            rule
            for rule in request["rules"]
            if not rule.startswith(suggestion_prefixes)
        ]
        request["rules"].append(
            "Treat safety_rejection as authoritative and independently choose "
            "one intent through the unchanged schema without receiving any "
            "candidate, suggested, clamped, or preselected replacement."
        )
        request["recovery_instruction"] = (
            "Independently propose one new intent through the normal schema. "
            "The rejection reports only the binding safety condition; it does "
            "not select, suggest, or enumerate a replacement action."
        )
    return json.dumps(
        request, sort_keys=True, separators=(",", ":"), allow_nan=False
    )


def _decision_evidence_snapshot(
    snapshot: Mapping[str, Any],
) -> dict[str, Any]:
    """Project a world snapshot to typed fields that can affect one decision."""

    def selected(
        value: Any,
        keys: tuple[str, ...],
    ) -> dict[str, Any]:
        mapping = value if isinstance(value, Mapping) else {}
        return {
            key: json.loads(json.dumps(mapping.get(key)))
            for key in keys
            if key in mapping
        }

    observations = snapshot.get("observations", {})
    observations = observations if isinstance(observations, Mapping) else {}
    perception = observations.get("perception", {})
    perception = perception if isinstance(perception, Mapping) else {}
    camera_image = perception.get("camera_image", {})
    camera_image = camera_image if isinstance(camera_image, Mapping) else {}
    image_metadata = selected(
        camera_image,
        (
            "available",
            "frame_id",
            "mime_type",
            "sha256",
            "byte_count",
        ),
    )
    compact_perception = selected(
        perception,
        (
            "available",
            "camera_frame_id",
            "camera_fresh",
            "localization_fresh",
            "localization_state",
            "semantic_map_fresh",
            "semantic_map_revision",
            "uncertain_track_id",
        ),
    )
    compact_perception["camera_image"] = image_metadata
    compact_observations = selected(
        observations,
        (
            "forward_clearance_m",
            "left_clearance_m",
            "right_clearance_m",
            "motion_clearance",
            "camera_detections",
            "semantic_tracks",
            "recognized_faces",
            "unknown_faces",
            "coverage_note",
        ),
    )
    # Camera freshness and detections are deliberately retained even when no
    # image pixels are relevant to the current decision.
    compact_observations.setdefault("camera_detections", [])
    compact_observations["perception"] = compact_perception

    evidence = snapshot.get("evidence", {})
    evidence = evidence if isinstance(evidence, Mapping) else {}
    compact_evidence = selected(
        evidence,
        (
            "drop_off_detection_available",
            "localization_fresh",
            "odometry_age_s",
            "odometry_fresh",
            "scan_age_s",
            "scan_fresh",
            "transform_fresh",
            "transform_reason",
        ),
    )
    receipts = evidence.get("source_receipts", {})
    receipts = receipts if isinstance(receipts, Mapping) else {}
    compact_evidence["source_receipts"] = {
        str(name): selected(receipt, ("fresh", "valid"))
        for name, receipt in receipts.items()
        if isinstance(receipt, Mapping)
    }

    last_execution = snapshot.get("last_execution", {})
    last_execution = (
        last_execution if isinstance(last_execution, Mapping) else {}
    )
    last_intent = selected(
        last_execution.get("intent"),
        (
            "action",
            "angle_deg",
            "distance_m",
            "interpreted_objective",
            "objective_status",
            "observation_focus",
            "rationale",
        ),
    )
    last_movement = selected(
        last_execution.get("movement"),
        (
            "collision_state",
            "outcome",
            "reason",
            "requested",
            "supervised",
        ),
    )
    last_route = selected(
        last_execution.get("route_terminal"),
        (
            "status",
            "terminal_reason",
            "terminal_settled",
            "measured_angle_deg",
            "measured_distance_m",
            "route_displacement_m",
            "route_heading_change_deg",
        ),
    )
    compact_last_execution = {
        "intent": last_intent,
        "movement": last_movement,
        "navigation_outcome": selected(
            last_execution.get("navigation_outcome"),
            (
                "angle_error_deg",
                "classification",
                "collision_state",
                "distance_error_m",
                "fresh_evidence_required",
                "measured_angle_deg",
                "measured_distance_m",
                "reason",
                "recoverable",
                "requested_angle_deg",
                "requested_distance_m",
                "terminal_settled",
            ),
        ),
        "route_terminal": last_route,
    }

    compact = {
        key: json.loads(json.dumps(snapshot.get(key)))
        for key in (
            "schema",
            "snapshot_id",
            "mission_id",
            "version",
            "observed_at_s",
            "safety",
            "pose",
            "progress",
        )
        if key in snapshot
    }
    compact["evidence"] = compact_evidence
    compact["execution"] = selected(
        snapshot.get("execution"),
        (
            "approval_bound",
            "mode",
            "motion_authority",
            "motion_permitted",
            "physical_execution_enabled",
        ),
    )
    compact["last_execution"] = compact_last_execution
    compact["observations"] = compact_observations
    return compact


def _visual_reasoning_relevance(
    prompt: str,
    snapshot: Mapping[str, Any],
) -> tuple[bool, str]:
    outcome = _navigation_outcome(snapshot)
    reason = str(outcome.get("reason", "")).strip().lower()
    if outcome.get("recoverable") is True and reason in {
        "stall",
        "collision_veto",
    }:
        return True, f"recovery:{reason}"

    objective = str(prompt).casefold()
    observations = snapshot.get("observations", {})
    observations = observations if isinstance(observations, Mapping) else {}
    labels: set[str] = set()
    for name in (
        "camera_detections",
        "semantic_tracks",
        "recognized_objects",
        "recognized_faces",
        "unknown_faces",
    ):
        values = observations.get(name, [])
        if not isinstance(values, list):
            continue
        for item in values:
            if not isinstance(item, Mapping):
                continue
            for key in ("label", "kind", "name", "class_name"):
                value = str(item.get(key, "")).strip().casefold()
                if len(value) >= 3:
                    labels.add(value)
    words = {
        "".join(character for character in token if character.isalnum())
        for token in objective.replace("_", " ").replace("-", " ").split()
    }
    if any(
        label_words
        and label_words.issubset(words)
        for label in labels
        for label_words in (
            {
                "".join(
                    character
                    for character in token
                    if character.isalnum()
                )
                for token in label.replace("_", " ")
                .replace("-", " ")
                .split()
            },
        )
    ):
        return True, "objective:detected_label"
    object_terms = {
        "approach",
        "camera",
        "face",
        "find",
        "follow",
        "identify",
        "locate",
        "object",
        "person",
        "shoe",
        "track",
        "visual",
        "image",
        "pixels",
    }
    if words & object_terms:
        return True, "objective:object_directed"
    return False, "not_relevant"


def _verified_camera_attachment(
    snapshot: Mapping[str, Any],
    root: Path,
) -> Optional[Path]:
    observations = snapshot.get("observations", {})
    perception = (
        observations.get("perception", {})
        if isinstance(observations, Mapping)
        else {}
    )
    image = (
        perception.get("camera_image", {})
        if isinstance(perception, Mapping)
        else {}
    )
    if not isinstance(image, Mapping) or image.get("available") is not True:
        outcome = _navigation_outcome(snapshot)
        if (
            outcome.get("recoverable") is True
            and str(outcome.get("reason", "")) == "stall"
            and snapshot.get("execution", {}).get("mode")
            == "physical-supervised-live-route"
        ):
            raise MissionValidationError(
                "recoverable physical stall lacks a fresh camera image attachment"
            )
        return None
    frame_id = str(image.get("frame_id", ""))
    camera_frame_id = str(perception.get("camera_frame_id", ""))
    source = Path(str(image.get("path", "")))
    expected_sha = str(image.get("sha256", "")).lower()
    expected_size = image.get("byte_count")
    if (
        frame_id != camera_frame_id
        or not frame_id
        or not source.is_absolute()
        or str(image.get("mime_type", "")) != "image/jpeg"
        or not isinstance(expected_size, int)
        or isinstance(expected_size, bool)
        or expected_size < 1
        or expected_size > 512_000
        or len(expected_sha) != 64
        or any(character not in "0123456789abcdef" for character in expected_sha)
    ):
        raise MissionValidationError(
            "adaptive mission camera image attachment metadata is invalid"
        )
    try:
        payload = source.read_bytes()
    except OSError as exc:
        raise MissionValidationError(
            "adaptive mission camera image attachment is unavailable"
        ) from exc
    if (
        len(payload) != expected_size
        or hashlib.sha256(payload).hexdigest() != expected_sha
    ):
        raise MissionValidationError(
            "adaptive mission camera image attachment digest is invalid"
        )
    copied = root / "camera-observation.jpg"
    copied.write_bytes(payload)
    copied.chmod(0o600)
    return copied


def _validate_snapshot_translation_clearance(
    snapshot: Mapping[str, Any],
    distance_m: float,
    *,
    rejected_request: Mapping[str, Any],
) -> None:
    execution = snapshot.get("execution", {})
    observations = snapshot.get("observations", {})
    physical = bool(
        isinstance(execution, Mapping)
        and execution.get("mode") == "physical-supervised-live-route"
    )
    clearance = (
        observations.get("motion_clearance")
        if isinstance(observations, Mapping)
        else None
    )
    if not isinstance(clearance, Mapping):
        if physical:
            raise MissionValidationError(
                "physical move intent lacks typed translation clearance"
            )
        return
    direction = "forward" if distance_m > 0.0 else "reverse"
    usable = _finite(
        clearance.get(f"{direction}_usable_m"),
        f"adaptive mission {direction} usable translation",
    )
    if usable < 0.0:
        raise MissionValidationError(
            f"adaptive mission {direction} usable translation is negative"
        )
    if abs(distance_m) > usable + 1e-9:
        raise RecoverableSafetyRejection(
            "adaptive mission intent translation exceeds snapshot usable "
            f"{direction} clearance",
            rejected_request=rejected_request,
            violated_condition=(
                f"translation_exceeds_{direction}_usable_clearance"
            ),
            limit_name=f"{direction}_usable_m",
            limit_value=usable,
            limit_unit="m",
            rejected_snapshot_id=str(snapshot.get("snapshot_id", "")),
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
                f"Approve one {_duration_label(self.limits.mission_lease_s)} "
                "adaptive mission lease. Approval activates "
                "the supervised sensor and motion graph; the LLM is called only "
                "after fresh camera, lidar, localization, and safety evidence arrive. "
                "Deterministic validation, the executor, and collision supervision "
                "remain authoritative."
            ),
            "contract": {
                "fixed_route": False,
                "replanning_after_every_intent": True,
                "one_authenticated_approval": True,
                "per_intent_approval": False,
                "approval_activates_supervised_graph": True,
                "telemetry_managed_for_lease": True,
                "telemetry_stops_when_lease_ends": True,
                "authenticated_objective_updates_within_lease": True,
                "first_intent_requires_fresh_post_approval_evidence": True,
                "recoverable_safety_rejection": {
                    "retry_budget": (
                        DEFAULT_SAFETY_REJECTION_RETRY_BUDGET
                    ),
                    "requires_fresh_snapshot": True,
                    "requires_isolated_model_thread": True,
                    "replacement_uses_normal_intent_schema": True,
                    "alternative_actions_supplied": False,
                    "motion_on_rejected_action": False,
                },
                "bounded_exploration_recovery": {
                    "trigger": "settled_collision_stall_or_modest_target_error",
                    "maximum_attempts": 2,
                    "maximum_reverse_m": 0.15,
                    "maximum_turn_deg": 45.0,
                    "requires_typed_rear_clearance": True,
                    "supervised_motion_only": True,
                    "fresh_evidence_before_llm_replan": True,
                    "llm_selects_problem_solving_action": True,
                    "deterministic_validation_remains_authoritative": True,
                },
                "motion_authority": False,
                "physical_execution_enabled": bool(
                    self.physical_execution_enabled
                ),
                "drop_off_detection": False,
            },
        }
        return {**body, "proposal_digest": canonical_digest(body)}


def _duration_label(duration_s: float) -> str:
    seconds = float(duration_s)
    if seconds >= 60.0 and seconds % 60.0 == 0.0:
        minutes = seconds / 60.0
        value = int(minutes) if minutes.is_integer() else minutes
        return f"{value}-minute"
    value = int(seconds) if seconds.is_integer() else seconds
    return f"{value}-second"


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

    def refresh_after_safety_rejection(
        self,
        previous: Mapping[str, Any],
        cancellation: threading.Event,
        *,
        timeout_s: float,
    ) -> Mapping[str, Any]: ...


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
                    "motion_clearance": {
                        "translation_reserve_m": 0.40,
                        "forward_usable_m": 1.40,
                        "reverse_usable_m": 0.80,
                    },
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
            recoverable = movement.reason in {"collision_veto", "stall"}
            self._last_execution = {
                "intent": intent.to_json_dict(),
                "movement": movement.to_json_dict(),
                "navigation_outcome": {
                    "classification": (
                        "recoverable_settled"
                        if recoverable
                        else "terminal"
                    ),
                    "reason": movement.reason,
                    "recoverable": recoverable,
                    "terminal_settled": True,
                    "fresh_evidence_required": recoverable,
                    "requested_distance_m": intent.distance_m,
                    "requested_angle_deg": intent.angle_deg,
                    "measured_distance_m": 0.0,
                    "measured_angle_deg": 0.0,
                },
            }
            return IntentExecutionResult(
                "replan" if recoverable else movement.outcome,
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
            "navigation_outcome": {
                "classification": "completed",
                "reason": "intent_completed",
                "recoverable": False,
                "terminal_settled": True,
                "fresh_evidence_required": False,
                "requested_distance_m": intent.distance_m,
                "requested_angle_deg": intent.angle_deg,
                "measured_distance_m": abs(intent.distance_m),
                "measured_angle_deg": abs(intent.angle_deg),
            },
        }
        return IntentExecutionResult(
            "completed",
            "intent_completed",
            self.snapshot(self._mission_id),
            movement,
            duration,
        )

    def refresh_after_safety_rejection(
        self,
        previous: Mapping[str, Any],
        cancellation: threading.Event,
        *,
        timeout_s: float,
    ) -> Mapping[str, Any]:
        del timeout_s
        if cancellation.is_set():
            raise MissionValidationError(
                "operator cancelled while refreshing safety-rejection evidence"
            )
        refreshed = dict(self.snapshot(self._mission_id))
        if str(refreshed.get("snapshot_id", "")) == str(
            previous.get("snapshot_id", "")
        ):
            time.sleep(0.001)
            refreshed = dict(self.snapshot(self._mission_id))
        return refreshed

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
        keep_active_after_planner_stop: bool = False,
        enable_exploration_recovery: bool = False,
        max_exploration_recoveries: int = 2,
        recovery_reverse_m: float = 0.15,
        max_safety_rejection_retries: int = (
            DEFAULT_SAFETY_REJECTION_RETRY_BUDGET
        ),
        initial_safety_rejections: tuple[Mapping[str, Any], ...] = (),
        activation_event_message: str = "",
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
        self._keep_active_after_planner_stop = bool(
            keep_active_after_planner_stop
        )
        self._enable_exploration_recovery = bool(
            enable_exploration_recovery
        )
        self._max_exploration_recoveries = int(
            max_exploration_recoveries
        )
        self._recovery_reverse_m = float(recovery_reverse_m)
        self._max_safety_rejection_retries = int(
            max_safety_rejection_retries
        )
        if self._max_exploration_recoveries < 1:
            raise MissionValidationError(
                "exploration recovery count must be positive"
            )
        if (
            not math.isfinite(self._recovery_reverse_m)
            or not 0.0 < self._recovery_reverse_m <= 0.15
        ):
            raise MissionValidationError(
                "exploration recovery reverse distance must be within 0.15 m"
            )
        if not 1 <= self._max_safety_rejection_retries <= 3:
            raise MissionValidationError(
                "safety rejection retry budget must be between 1 and 3"
            )
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
        self._objective_revision = 0
        self._idle_objective_revision: Optional[int] = None
        self._objective_condition = threading.Condition(self._lock)
        self._requested_terminal: Optional[tuple[str, str]] = None
        self._recovery_attempts = 0
        self._safety_rejections = [
            json.loads(json.dumps(dict(item)))
            for item in initial_safety_rejections
        ]
        self._append_event(
            "approval_bound",
            f"Authenticated operator {self.operator} approved lease through "
            f"{self._mission_expires_at_s:.3f}; no per-intent approval is required.",
        )
        if str(activation_event_message).strip():
            self._append_event(
                "physical_session_activated",
                str(activation_event_message).strip(),
            )
        self._append_event(
            "snapshot",
            _snapshot_event_message(self._world),
        )
        for rejection in self._safety_rejections:
            self._append_safety_rejection_event(
                rejection, retry_number=None
            )
        self._append_event(
            "objective_interpreted",
            first_intent.interpreted_objective,
        )
        self._append_event(
            "llm_revision",
            f"LLM chose revision {first_intent.revision}: "
            f"{first_intent.action} [{first_intent.objective_status}] — "
            f"{first_intent.rationale}",
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

    def update_objective(
        self, prompt: str, *, operator: str
    ) -> dict[str, Any]:
        objective = str(prompt).strip()
        principal = str(operator).strip()
        if not objective or len(objective) > 500:
            raise MissionValidationError(
                "updated Adaptive mission objective must contain 1 to 500 characters"
            )
        with self._lock:
            if self._terminal:
                raise MissionValidationError(
                    "a terminal Adaptive mission lease cannot accept a new objective"
                )
            if principal != self.operator:
                raise MissionValidationError(
                    "only the operator who approved the lease may update its objective"
                )
            if objective == self.prompt:
                return self._projection()
            previous = self.prompt
            self.prompt = objective
            self._objective_revision += 1
            self._objective_condition.notify_all()
            self._append_event(
                "objective_updated",
                f"Authenticated operator {principal} updated the objective from "
                f"{previous!r} to {objective!r}; the original lease expiry and "
                "safety limits remain unchanged.",
            )
            checkpoint = self._projection()
        self._emit_checkpoint("objective_updated", checkpoint)
        return checkpoint

    def close(self, *, timeout_s: float = 10.0) -> None:
        self._shutdown.set()
        self._cancellation.set()
        cancel_provider = getattr(self.provider, "cancel", None)
        if callable(cancel_provider):
            cancel_provider()
        with self._objective_condition:
            self._objective_condition.notify_all()
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
            replan_after_outcome = False
            pending_safety_rejection: Optional[dict[str, Any]] = None
            consecutive_safety_rejections = 0
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
                    if not self._keep_active_after_planner_stop:
                        self._finish("failed", "missing_intent")
                        return
                    idle_revision = self._idle_objective_revision
                else:
                    idle_revision = None
            if intent is None:
                if idle_revision is None:
                    with self._lock:
                        self._finish(
                            "failed", "missing_idle_objective_revision"
                        )
                    return
                next_intent = self._wait_for_updated_objective(
                    idle_revision
                )
                if next_intent is None:
                    return
                with self._lock:
                    if self._terminal or self._cancellation.is_set():
                        if not self._terminal:
                            self._finish_requested_terminal()
                        return
                    self._active_intent = next_intent
                    self._idle_objective_revision = None
                    self._append_event(
                        "llm_revision",
                        f"LLM chose revision {next_intent.revision}: "
                        f"{next_intent.action} "
                        f"[{next_intent.objective_status}] — "
                        f"{next_intent.rationale}",
                    )
                    checkpoint = ("llm_revision", self._projection())
                self._emit_checkpoint(*checkpoint)
                continue
            with self._lock:
                try:
                    validate_world_snapshot(
                        self._world,
                        mission_id=self.mission_id,
                        require_motion=intent.action in {"move_distance", "turn_angle"},
                        allow_supervised_collision_escape=(
                            _is_supervised_collision_escape_intent(intent)
                        ),
                        allow_collision_stopped_observation=(
                            intent.action in {"observe", "stop"}
                            and _is_collision_replan_snapshot(self._world)
                        ),
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
                self._append_event(
                    "snapshot",
                    _snapshot_event_message(self._world),
                )
                if execution.outcome != "completed":
                    if (
                        self._requested_terminal is not None
                        and "cleanup_uncertain" not in execution.reason
                    ):
                        requested_status, _ = self._requested_terminal
                        self._finish(requested_status, execution.reason)
                        return
                    collision_rejection = (
                        self._collision_safety_rejection(
                            intent, execution
                        )
                    )
                    if collision_rejection is not None:
                        try:
                            refreshed = (
                                self._fresh_snapshot_after_safety_rejection(
                                    self._world
                                )
                            )
                        except Exception as refresh_exc:
                            self._finish(
                                "blocked",
                                "safety_rejection_evidence_unavailable: "
                                f"{refresh_exc.__class__.__name__}: "
                                f"{refresh_exc}",
                            )
                            return
                        self._world = json.loads(
                            json.dumps(refreshed)
                        )
                        self._snapshots.append(
                            json.loads(json.dumps(self._world))
                        )
                        collision_rejection[
                            "current_snapshot_id"
                        ] = str(self._world.get("snapshot_id", ""))
                        consecutive_safety_rejections = 1
                        pending_safety_rejection = collision_rejection
                        self._safety_rejections.append(
                            json.loads(
                                json.dumps(collision_rejection)
                            )
                        )
                        self._append_safety_rejection_event(
                            collision_rejection,
                            retry_number=1,
                        )
                        self._append_event(
                            "snapshot",
                            _snapshot_event_message(self._world),
                        )
                        replan_after_outcome = True
                        checkpoint = (
                            "safety_rejection",
                            self._projection(),
                        )
                    elif self._can_replan_after_outcome(
                        intent, execution
                    ):
                        self._recovery_attempts += 1
                        replan_after_outcome = True
                        self._append_event(
                            "outcome_replan",
                            "The settled supervised outcome "
                            f"{execution.reason} is bounded and recoverable; "
                            "fresh typed evidence will be sent to the LLM for "
                            "the next validated navigation decision.",
                        )
                        checkpoint = (
                            "outcome_replan",
                            self._projection(),
                        )
                    else:
                        if execution.outcome == "replan":
                            self._append_event(
                                "recovery_unavailable",
                                "The bounded LLM problem-solving limit was "
                                "reached or this objective is not eligible; "
                                "automatic motion remains stopped.",
                            )
                        terminal_status = {
                            "blocked": "blocked",
                            "stale": "blocked",
                            "cancelled": "cancelled",
                            "timeout": "timeout",
                        }.get(execution.outcome, "failed")
                        if execution.reason == "collision_veto":
                            terminal_status = "blocked"
                        if "cleanup_uncertain" in execution.reason:
                            terminal_status = "recovery_required"
                        self._finish(
                            terminal_status, execution.reason
                        )
                        return
                if not replan_after_outcome:
                    checkpoint = ("intent_result", self._projection())
                if intent.action == "stop":
                    if not self._keep_active_after_planner_stop:
                        self._finish(
                            (
                                "complete"
                                if intent.objective_status == "complete"
                                else "blocked"
                            ),
                            (
                                "planner_stop"
                                if intent.objective_status == "complete"
                                else "planner_objective_blocked"
                            ),
                        )
                        return
                    self._active_intent = None
                    self._idle_objective_revision = (
                        self._objective_revision
                    )
                    objective_complete = (
                        intent.objective_status == "complete"
                    )
                    self._append_event(
                        (
                            "objective_complete"
                            if objective_complete
                            else "objective_blocked"
                        ),
                        (
                            "The model found the current objective complete. "
                            if objective_complete
                            else "The model found no currently validated way "
                            "to continue the objective. "
                        )
                        + "The authenticated physical session remains safely idle "
                        "with telemetry on until the original lease expires, is "
                        "cancelled, or receives another objective.",
                    )
                    checkpoint = (
                        (
                            "objective_complete"
                            if objective_complete
                            else "objective_blocked"
                        ),
                        self._projection(),
                    )
                if self._now() >= self._mission_expires_at_s:
                    self._finish("timeout", "mission_lease_expired")
                    return
                try:
                    validate_world_snapshot(
                        self._world,
                        mission_id=self.mission_id,
                        require_motion=False,
                        allow_collision_stopped_observation=(
                            replan_after_outcome
                            and _is_collision_replan_snapshot(
                                self._world
                            )
                        ),
                    )
                except MissionValidationError as exc:
                    self._finish(
                        "blocked",
                        f"stale_or_unsafe_evidence: {exc}",
                    )
                    return
            if checkpoint is not None:
                self._emit_checkpoint(*checkpoint)
            if intent.action == "stop":
                continue
            next_intent = self._choose_next_intent(
                safety_rejection=pending_safety_rejection,
                consecutive_safety_rejections=(
                    consecutive_safety_rejections
                ),
            )
            if next_intent is None:
                return
            with self._lock:
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
                    f"{next_intent.action} "
                    f"[{next_intent.objective_status}] — "
                    f"{next_intent.rationale}",
                )
                checkpoint = ("llm_revision", self._projection())
            self._emit_checkpoint(*checkpoint)

    def _can_replan_after_outcome(
        self,
        failed_intent: AdaptiveMissionIntent,
        execution: IntentExecutionResult,
    ) -> bool:
        if (
            not self._enable_exploration_recovery
            or execution.outcome != "replan"
            or self._recovery_attempts
            >= self._max_exploration_recoveries
        ):
            return False
        if not _is_exploration_objective(
            self.prompt,
            failed_intent.interpreted_objective,
        ):
            return False
        outcome = _navigation_outcome(self._world)
        return bool(
            outcome.get("recoverable") is True
            and outcome.get("terminal_settled") is True
            and str(outcome.get("reason", ""))
            in {"target_error", "stall", "collision_veto"}
        )

    def _collision_safety_rejection(
        self,
        intent: AdaptiveMissionIntent,
        execution: IntentExecutionResult,
    ) -> Optional[dict[str, Any]]:
        outcome = _navigation_outcome(self._world)
        measured_distance = outcome.get("measured_distance_m")
        measured_angle = outcome.get("measured_angle_deg")
        try:
            zero_measured_motion = (
                float(measured_distance) == 0.0
                and float(measured_angle) == 0.0
            )
        except (TypeError, ValueError):
            zero_measured_motion = False
        if not (
            execution.outcome == "replan"
            and execution.reason == "collision_veto"
            and outcome.get("recoverable") is True
            and outcome.get("terminal_settled") is True
            and execution.movement.supervised_linear_mps == 0.0
            and execution.movement.supervised_angular_rad_s == 0.0
            and zero_measured_motion
            and str(self._world.get("snapshot_id", ""))
            != intent.snapshot_id
        ):
            return None
        rejection = RecoverableSafetyRejection(
            "collision supervisor vetoed the proposed action",
            rejected_request=intent.to_json_dict(),
            violated_condition="collision_supervisor_veto",
            limit_name="maximum_executed_motion_m",
            limit_value=0.0,
            limit_unit="m",
            rejected_snapshot_id=intent.snapshot_id,
        )
        return rejection.feedback(
            current_snapshot_id=str(
                self._world.get("snapshot_id", "")
            )
        )

    def _fresh_snapshot_after_safety_rejection(
        self, previous: Mapping[str, Any]
    ) -> dict[str, Any]:
        refresh = getattr(
            self.executor, "refresh_after_safety_rejection", None
        )
        if not callable(refresh):
            raise MissionValidationError(
                "executor cannot prove fresh safety-rejection evidence"
            )
        refreshed = dict(
            refresh(
                previous,
                self._cancellation,
                timeout_s=min(
                    self.limits.max_intent_timeout_s,
                    max(
                        0.0,
                        self._mission_expires_at_s - self._now(),
                    ),
                ),
            )
        )
        if str(refreshed.get("snapshot_id", "")) == str(
            previous.get("snapshot_id", "")
        ):
            raise MissionValidationError(
                "safety-rejection evidence did not advance to a new snapshot"
            )
        validate_world_snapshot(
            refreshed,
            mission_id=self.mission_id,
            require_motion=False,
            allow_collision_stopped_observation=True,
        )
        return refreshed

    def _append_safety_rejection_event(
        self,
        rejection: Mapping[str, Any],
        *,
        retry_number: Optional[int],
    ) -> None:
        limit = rejection.get("applicable_numeric_limit", {})
        limit = limit if isinstance(limit, Mapping) else {}
        retry = (
            ""
            if retry_number is None
            else (
                f"; isolated retry {retry_number}/"
                f"{self._max_safety_rejection_retries}"
            )
        )
        self._append_event(
            "safety_rejection",
            "No motion executed; rejected condition "
            f"{str(rejection.get('violated_condition', 'unknown'))} at "
            f"snapshot {str(rejection.get('current_snapshot_id', ''))[:12]} "
            f"with {str(limit.get('name', 'limit'))}="
            f"{limit.get('value')} {str(limit.get('unit', '')).strip()}"
            f"{retry}.",
        )

    def _wait_for_updated_objective(
        self, idle_revision: int
    ) -> Optional[AdaptiveMissionIntent]:
        while not self._shutdown.is_set():
            with self._objective_condition:
                if self._terminal:
                    return None
                if self._cancellation.is_set():
                    self._finish_requested_terminal()
                    return None
                remaining_s = (
                    self._mission_expires_at_s - self._now()
                )
                if remaining_s <= 0.0:
                    self._finish(
                        "timeout", "mission_lease_expired"
                    )
                    return None
                if self._objective_revision > idle_revision:
                    break
                self._objective_condition.wait(
                    timeout=min(0.1, remaining_s)
                )
        try:
            refreshed = dict(self.executor.snapshot(self.mission_id))
            validate_world_snapshot(
                refreshed,
                mission_id=self.mission_id,
                require_motion=False,
            )
        except Exception as exc:
            with self._lock:
                self._finish(
                    "blocked",
                    f"updated_objective_evidence_unavailable: {exc}",
                )
            return None
        with self._lock:
            self._world = json.loads(json.dumps(refreshed))
            self._snapshots.append(json.loads(json.dumps(self._world)))
            self._append_event(
                "snapshot",
                _snapshot_event_message(self._world),
            )
            checkpoint = self._projection()
        self._emit_checkpoint(
            "objective_update_replan", checkpoint
        )
        return self._choose_next_intent()

    def _choose_next_intent(
        self,
        *,
        safety_rejection: Optional[Mapping[str, Any]] = None,
        consecutive_safety_rejections: int = 0,
    ) -> Optional[AdaptiveMissionIntent]:
        pending_rejection = (
            json.loads(json.dumps(dict(safety_rejection)))
            if safety_rejection is not None
            else None
        )
        rejection_count = int(consecutive_safety_rejections)
        while not self._shutdown.is_set():
            with self._lock:
                if self._terminal:
                    return None
                if self._cancellation.is_set():
                    self._finish_requested_terminal()
                    return None
                if self._now() >= self._mission_expires_at_s:
                    self._finish("timeout", "mission_lease_expired")
                    return None
                self._provider_calls_started += 1
                self._inference_in_flight = True
                provider_prompt = self.prompt
                objective_revision = self._objective_revision
                provider_snapshot = json.loads(
                    json.dumps(self._world)
                )
                intent_revision = len(self._revisions) + 1
                self._append_event(
                    (
                        "safety_recovery_decision_started"
                        if pending_rejection is not None
                        else "llm_revision_started"
                    ),
                    f"Provider call {self._provider_calls_started} started an "
                    "isolated decision from snapshot "
                    f"{str(provider_snapshot.get('snapshot_id', ''))[:12]}"
                    + (
                        " after a typed safety rejection."
                        if pending_rejection is not None
                        else "."
                    ),
                )
            try:
                future = self._provider_pool.submit(
                    _choose_provider_intent,
                    self.provider,
                    provider_prompt,
                    provider_snapshot,
                    safety_rejection=pending_rejection,
                )
                self._provider_future = future
                while not future.done():
                    if self._shutdown.wait(0.02):
                        future.cancel()
                        return None
                    with self._lock:
                        if self._terminal:
                            future.cancel()
                            return None
                        if self._cancellation.is_set():
                            self._finish_requested_terminal()
                            future.cancel()
                            return None
                        if self._now() >= self._mission_expires_at_s:
                            self._finish(
                                "timeout", "mission_lease_expired"
                            )
                            future.cancel()
                            return None
                raw = dict(future.result())
                self._provider_future = None
                validation_started = time.perf_counter()
                validation_valid = False
                try:
                    next_intent = AdaptiveMissionIntent.validated(
                        raw,
                        revision=intent_revision,
                        snapshot=provider_snapshot,
                        issued_at_s=self._now(),
                        provider_id=self.provider.provider_id,
                        model_id=self.provider.model_id,
                        limits=self.limits,
                        supervised_collision_escape=bool(
                            _is_collision_replan_snapshot(provider_snapshot)
                            and str(raw.get("action", "")).strip()
                            == "move_distance"
                            and _finite(
                                raw.get("distance_m"),
                                "adaptive mission collision escape distance",
                            )
                            < 0.0
                        ),
                    )
                    validation_valid = True
                finally:
                    record_latency = getattr(
                        self.provider, "add_validation_latency", None
                    )
                    if callable(record_latency):
                        record_latency(
                            str(provider_snapshot.get("snapshot_id", "")),
                            validation_ms=(
                                time.perf_counter() - validation_started
                            )
                            * 1000.0,
                            valid=validation_valid,
                        )
                if (
                    _navigation_outcome(provider_snapshot).get(
                        "recoverable"
                    )
                    is True
                    and next_intent.action == "move_distance"
                    and next_intent.distance_m < 0.0
                    and abs(next_intent.distance_m)
                    > self._recovery_reverse_m + 1e-9
                ):
                    raise RecoverableSafetyRejection(
                        "problem-solving reverse exceeds 0.15 m",
                        rejected_request=raw,
                        violated_condition=(
                            "problem_solving_reverse_limit_exceeded"
                        ),
                        limit_name="max_problem_solving_reverse_m",
                        limit_value=self._recovery_reverse_m,
                        limit_unit="m",
                        rejected_snapshot_id=str(
                            provider_snapshot.get("snapshot_id", "")
                        ),
                    )
            except RecoverableSafetyRejection as exc:
                with self._lock:
                    self._provider_future = None
                    self._provider_calls_completed += 1
                    self._inference_in_flight = False
                rejection_count += 1
                try:
                    refreshed = self._fresh_snapshot_after_safety_rejection(
                        provider_snapshot
                    )
                except Exception as refresh_exc:
                    with self._lock:
                        if not self._terminal:
                            self._finish(
                                "blocked",
                                "safety_rejection_evidence_unavailable: "
                                f"{refresh_exc.__class__.__name__}: "
                                f"{refresh_exc}",
                            )
                    return None
                pending_rejection = exc.feedback(
                    current_snapshot_id=str(
                        refreshed.get("snapshot_id", "")
                    )
                )
                with self._lock:
                    self._world = json.loads(json.dumps(refreshed))
                    self._snapshots.append(
                        json.loads(json.dumps(self._world))
                    )
                    self._safety_rejections.append(
                        json.loads(json.dumps(pending_rejection))
                    )
                    self._append_safety_rejection_event(
                        pending_rejection,
                        retry_number=rejection_count,
                    )
                    self._append_event(
                        "snapshot",
                        _snapshot_event_message(self._world),
                    )
                    checkpoint = self._projection()
                self._emit_checkpoint(
                    "safety_rejection", checkpoint
                )
                if (
                    rejection_count
                    > self._max_safety_rejection_retries
                ):
                    with self._lock:
                        self._finish(
                            "blocked",
                            "safety_rejection_retry_budget_exhausted",
                        )
                    return None
                continue
            except Exception as exc:
                with self._lock:
                    self._provider_future = None
                    self._inference_in_flight = False
                    if not self._terminal:
                        self._finish(
                            "failed",
                            f"provider_failure: {exc.__class__.__name__}: {exc}",
                        )
                return None
            with self._lock:
                self._provider_calls_completed += 1
                self._inference_in_flight = False
                objective_changed = (
                    objective_revision != self._objective_revision
                )
                if not objective_changed:
                    if pending_rejection is not None:
                        self._append_event(
                            "safety_recovery_decision_validated",
                            "The isolated replacement decision passed the "
                            "normal deterministic validator; the consecutive "
                            "safety-rejection counter reset.",
                        )
                    return next_intent
                self._append_event(
                    "llm_revision_discarded",
                    "An in-flight model response for the previous objective "
                    "was discarded; fresh evidence will be reacquired before "
                    "replanning without changing the lease.",
                )
                pending_rejection = None
                rejection_count = 0
            try:
                refreshed = dict(self.executor.snapshot(self.mission_id))
                validate_world_snapshot(
                    refreshed,
                    mission_id=self.mission_id,
                    require_motion=False,
                )
            except Exception as exc:
                with self._lock:
                    self._finish(
                        "blocked",
                        f"updated_objective_evidence_unavailable: {exc}",
                    )
                return None
            with self._lock:
                self._world = json.loads(json.dumps(refreshed))
                self._snapshots.append(
                    json.loads(json.dumps(self._world))
                )
                self._append_event(
                    "snapshot",
                    _snapshot_event_message(self._world),
                )
                checkpoint = self._projection()
            self._emit_checkpoint(
                "objective_update_replan", checkpoint
            )
        return None

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
        cancel_provider = getattr(self.provider, "cancel", None)
        if callable(cancel_provider):
            cancel_provider()
        with self._lock:
            if self._terminal:
                return
            self._requested_terminal = (str(status), str(reason))
            self._objective_condition.notify_all()
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
        latency_reader = getattr(self.provider, "latency_history", None)
        planning_cycles = (
            latency_reader() if callable(latency_reader) else []
        )
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
            "lease_waiting_for_objective": bool(
                self._keep_active_after_planner_stop
                and self._active_intent is None
                and not self._terminal
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
                "planning_cycles": planning_cycles,
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
                "safety_rejection_count": len(
                    self._safety_rejections
                ),
            },
            "safety_recovery": {
                "retry_budget": self._max_safety_rejection_retries,
                "rejections": json.loads(
                    json.dumps(self._safety_rejections)
                ),
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
            "current_objective": self.prompt,
            "events": json.loads(json.dumps(self._events)),
            "map": map_payload,
            "result": result,
            "motion_authority": False,
            "physical_execution_enabled": bool(
                getattr(self.executor, "execution_enabled", False)
            ),
        }

    def _terminal_result(self) -> dict[str, Any]:
        latency_reader = getattr(self.provider, "latency_history", None)
        return {
            "schema": ADAPTIVE_MISSION_RESULT_SCHEMA,
            "mission_id": self.mission_id,
            "status": self._status,
            "terminal_reason": self._terminal_reason,
            "final_snapshot": json.loads(json.dumps(self._world)),
            "world_snapshots": json.loads(json.dumps(self._snapshots)),
            "intent_revisions": json.loads(json.dumps(self._revisions)),
            "events": json.loads(json.dumps(self._events)),
            "provider": {
                "provider_id": self.provider.provider_id,
                "model_id": self.provider.model_id,
                "reasoning_effort": self.provider.reasoning_effort,
                "calls_started": self._provider_calls_started,
                "calls_completed": self._provider_calls_completed,
                "planning_cycles": (
                    latency_reader() if callable(latency_reader) else []
                ),
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
            "safety_recovery": {
                "retry_budget": self._max_safety_rejection_retries,
                "rejections": json.loads(
                    json.dumps(self._safety_rejections)
                ),
            },
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


def _is_supervised_collision_escape_intent(
    intent: AdaptiveMissionIntent,
) -> bool:
    return bool(
        intent.supervised_collision_escape
        and intent.action == "move_distance"
        and intent.distance_m < 0.0
    )


def _navigation_outcome(
    snapshot: Mapping[str, Any],
) -> Mapping[str, Any]:
    last_execution = snapshot.get("last_execution")
    if not isinstance(last_execution, Mapping):
        return {}
    outcome = last_execution.get("navigation_outcome")
    return outcome if isinstance(outcome, Mapping) else {}


def _is_collision_replan_snapshot(snapshot: Mapping[str, Any]) -> bool:
    outcome = _navigation_outcome(snapshot)
    return bool(
        outcome.get("recoverable") is True
        and outcome.get("terminal_settled") is True
        and str(outcome.get("reason", "")) == "collision_veto"
    )


def _is_exploration_objective(*values: str) -> bool:
    text = " ".join(str(value).lower() for value in values)
    tokens = {
        token.strip(".,:;!?()[]{}")
        for token in text.split()
    }
    return bool(
        "map" in tokens
        or any(
            marker in text
            for marker in ("explor", "mapping", "survey")
        )
    )


def _snapshot_event_message(snapshot: Mapping[str, Any]) -> str:
    evidence = snapshot.get("evidence", {})
    receipts = (
        evidence.get("source_receipts", {})
        if isinstance(evidence, Mapping)
        else {}
    )
    observations = snapshot.get("observations", {})
    perception = (
        observations.get("perception", {})
        if isinstance(observations, Mapping)
        else {}
    )
    identifiers = []
    if isinstance(receipts, Mapping):
        for name in ("camera", "lidar", "localization"):
            record = receipts.get(name, {})
            if not isinstance(record, Mapping) or not record:
                identifiers.append(f"{name}=unavailable")
                continue
            state = (
                "fresh"
                if record.get("fresh") is True
                else "stale"
            )
            identifiers.append(
                f"{name}={state}"
                f"@{record.get('received_at_s')}"
            )
    frame = (
        perception.get("camera_frame_id")
        if isinstance(perception, Mapping)
        else None
    )
    suffix = ", ".join(identifiers) if identifiers else "typed replay evidence"
    if frame:
        suffix += f", frame={frame}"
    return (
        f"Snapshot v{snapshot.get('version', '?')} "
        f"{str(snapshot.get('snapshot_id', ''))[:12]}: {suffix}."
    )
