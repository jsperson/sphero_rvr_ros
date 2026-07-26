from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import threading
import time
from typing import Any, Mapping
import urllib.error
import urllib.request

import pytest

from sphero_rvr_driver.codex_app_server import (
    CodexAppServerClient,
    codex_oauth_environment,
    resolve_codex_executable,
)
from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_service import MissionService
from sphero_rvr_driver.mission_web import (
    MissionWebError,
    AdaptiveMissionAdapter,
    make_server,
)
from sphero_rvr_driver.adaptive_mission_controller import (
    AdaptiveMissionController,
    CodexOAuthAdaptiveMissionIntentProvider,
    ReplayCollisionSupervisor,
    ReplayAdaptiveMissionExecutor,
    AdaptiveMissionApprovalEnvelope,
    AdaptiveMissionIntent,
    AdaptiveMissionLimits,
    MovementDecision,
    RecoverableSafetyRejection,
    _adaptive_mission_provider_prompt,
    _adaptive_mission_output_schema,
    _decision_evidence_snapshot,
    _visual_reasoning_relevance,
    _verified_camera_attachment,
    make_world_snapshot,
    validate_world_snapshot,
)


PROMPT = "Explore the room, revise from each observation, then stop safely."


def _raw(snapshot: Mapping[str, Any], action: str, value: float = 0.0) -> dict[str, Any]:
    return {
        "snapshot_id": snapshot["snapshot_id"],
        "action": action,
        "distance_m": value if action == "move_distance" else 0.0,
        "angle_deg": value if action == "turn_angle" else 0.0,
        "observation_focus": "fresh local lidar corridor",
        "rationale": f"Choose {action} from the updated typed snapshot.",
        "interpreted_objective": (
            "Explore reachable local free space and stop when the planned sample is complete."
        ),
        "objective_status": "complete" if action == "stop" else "in_progress",
        "lease_s": 5.0,
        "timeout_s": 5.0,
    }


def test_codex_oauth_environment_excludes_credentials(monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "must-not-propagate")
    monkeypatch.setenv("UNRELATED_OAUTH_TOKEN", "must-not-propagate")
    monkeypatch.setenv("HOME", "/tmp/codex-oauth-home")
    monkeypatch.setenv("PATH", "/usr/bin")

    environment = codex_oauth_environment()

    assert environment["HOME"] == "/tmp/codex-oauth-home"
    assert environment["PATH"] == "/usr/bin"
    assert "OPENAI_API_KEY" not in environment
    assert "UNRELATED_OAUTH_TOKEN" not in environment


def test_codex_resolution_falls_back_to_user_local_bin(
    monkeypatch, tmp_path: Path
) -> None:
    executable = tmp_path / ".local" / "bin" / "codex"
    executable.parent.mkdir(parents=True)
    executable.write_text("#!/bin/sh\n", encoding="utf-8")
    executable.chmod(0o755)
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("PATH", "/usr/bin")

    assert resolve_codex_executable("codex") == str(executable)


def test_app_server_every_decision_starts_new_ephemeral_thread(
    monkeypatch,
) -> None:
    client = CodexAppServerClient()
    starts: list[tuple[str, Mapping[str, Any]]] = []
    messages: list[Mapping[str, Any]] = []
    thread_number = 0

    def request(method, params, *, deadline):
        nonlocal thread_number
        del deadline
        starts.append((str(method), dict(params)))
        if method == "thread/start":
            thread_number += 1
            return {"thread": {"id": f"thread-{thread_number}"}}
        thread_id = str(params["threadId"])
        turn_id = f"turn-{thread_number}"
        messages.extend(
            [
                {
                    "method": "item/completed",
                    "params": {
                        "threadId": thread_id,
                        "turnId": turn_id,
                        "item": {
                            "type": "agentMessage",
                            "text": "{}",
                        },
                    },
                },
                {
                    "method": "turn/completed",
                    "params": {
                        "threadId": thread_id,
                        "turn": {
                            "id": turn_id,
                            "status": "completed",
                        },
                    },
                },
            ]
        )
        return {"turn": {"id": turn_id}}

    monkeypatch.setattr(client, "_request_unlocked", request)
    monkeypatch.setattr(
        client,
        "_next_message_unlocked",
        lambda _deadline: messages.pop(0),
    )

    for _ in range(2):
        assert client._run_turn_unlocked(
            prompt="{}",
            model="test-model",
            effort="low",
            output_schema={"type": "object"},
            cwd="/tmp",
            image_path=None,
            deadline=time.monotonic() + 1.0,
        ) == "{}"

    thread_starts = [
        params for method, params in starts if method == "thread/start"
    ]
    turn_starts = [
        params for method, params in starts if method == "turn/start"
    ]
    assert len(thread_starts) == 2
    assert all(params["ephemeral"] is True for params in thread_starts)
    assert [params["threadId"] for params in turn_starts] == [
        "thread-1",
        "thread-2",
    ]


class SequenceProvider:
    provider_id = "injected-adaptive-mission-provider"
    model_id = "deterministic-test-model"
    reasoning_effort = "fixture"

    def __init__(
        self,
        actions: list[tuple[str, float]],
        *,
        delay_after_first_s: float = 0.0,
        fail_after_calls: int | None = None,
    ) -> None:
        self.actions = list(actions)
        self.delay_after_first_s = float(delay_after_first_s)
        self.fail_after_calls = fail_after_calls
        self.calls = 0
        self.entered_delayed_call = threading.Event()
        self.safety_rejections: list[Mapping[str, Any]] = []
        self.snapshots: list[str] = []
        self.snapshot_payloads: list[Mapping[str, Any]] = []
        self.prompts: list[str] = []

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        assert prompt
        self.prompts.append(str(prompt))
        self.snapshots.append(str(snapshot["snapshot_id"]))
        self.snapshot_payloads.append(
            json.loads(json.dumps(dict(snapshot)))
        )
        self.calls += 1
        if self.fail_after_calls is not None and self.calls > self.fail_after_calls:
            raise ConnectionError("injected provider network failure")
        if self.calls > 1 and self.delay_after_first_s:
            self.entered_delayed_call.set()
            time.sleep(self.delay_after_first_s)
        if self.calls > len(self.actions):
            raise AssertionError("provider was called after scripted stop")
        action, value = self.actions[self.calls - 1]
        return _raw(snapshot, action, value)

    def choose_after_safety_rejection(
        self,
        prompt: str,
        snapshot: Mapping[str, Any],
        rejection: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        self.safety_rejections.append(
            json.loads(json.dumps(dict(rejection)))
        )
        return self.choose(prompt, snapshot)


class RecognitionDrivenProvider:
    provider_id = "injected-recognition-provider"
    model_id = "deterministic-recognition-model"
    reasoning_effort = "fixture"

    def __init__(self) -> None:
        self.snapshots: list[Mapping[str, Any]] = []

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        assert "shoe" in prompt.lower()
        self.snapshots.append(snapshot)
        observations = snapshot["observations"]
        progress = snapshot["progress"]
        if observations["perception"]["available"] is not True:
            return _raw(snapshot, "observe")
        target = next(
            (
                item
                for item in observations["recognized_objects"]
                if "shoe" in str(item.get("label", "")).lower()
            ),
            None,
        )
        if target is None:
            return _raw(snapshot, "stop")
        if progress["intent_count"] == 1:
            raw = _raw(snapshot, "turn_angle", 25.0)
        elif progress["intent_count"] == 2:
            raw = _raw(snapshot, "move_distance", 0.10)
        else:
            raw = _raw(snapshot, "stop")
        raw["rationale"] = (
            f"Revise from fresh localized semantic track {target['track_id']}."
        )
        raw["interpreted_objective"] = (
            "Use fresh semantic evidence to approach the shoe through clear floor."
        )
        return raw


class SemanticReplayAdaptiveMissionExecutor(ReplayAdaptiveMissionExecutor):
    def snapshot(self, mission_id: str) -> Mapping[str, Any]:
        snapshot = dict(super().snapshot(mission_id))
        snapshot.pop("schema", None)
        snapshot.pop("snapshot_id", None)
        observations = dict(snapshot["observations"])
        if self.intent_count >= 1:
            target = {
                "track_id": "object-shoe-1",
                "kind": "object",
                "label": "possible_shoe",
                "confidence": 0.82,
                "x_m": self.x_m + 0.6,
                "y_m": self.y_m + 0.3,
                "uncertainty_m": 0.18,
                "recognized_from_enrollment": False,
                "enrollment_evidence_ids": [],
            }
            observations.update(
                {
                    "camera_detections": [dict(target)],
                    "semantic_tracks": [dict(target)],
                    "recognized_objects": [dict(target)],
                    "perception": {
                        "available": True,
                        "camera_fresh": True,
                        "semantic_map_fresh": True,
                        "localization_fresh": True,
                        "localization_state": "valid",
                        "camera_frame_id": f"semantic-frame-{self.intent_count}",
                        "semantic_map_revision": self.intent_count,
                        "uncertain_track_id": "",
                        "identity_policy": (
                            "face labels are authoritative only with explicit "
                            "enrollment evidence"
                        ),
                    },
                }
            )
        snapshot["observations"] = observations
        return make_world_snapshot(snapshot)


class RecoveryReplayAdaptiveMissionExecutor(ReplayAdaptiveMissionExecutor):
    def snapshot(self, mission_id: str) -> Mapping[str, Any]:
        snapshot = dict(super().snapshot(mission_id))
        snapshot.pop("schema", None)
        snapshot.pop("snapshot_id", None)
        observations = dict(snapshot["observations"])
        observations["motion_clearance"] = {
            "translation_reserve_m": 0.40,
            "forward_usable_m": 1.0,
            "reverse_usable_m": 0.30,
        }
        snapshot["observations"] = observations
        return make_world_snapshot(snapshot)


class StallOnceSupervisor(ReplayCollisionSupervisor):
    def supervise(
        self,
        intent: AdaptiveMissionIntent,
        snapshot: Mapping[str, Any],
        limits: AdaptiveMissionLimits,
    ) -> MovementDecision:
        if self.calls == 0:
            self.calls += 1
            return MovementDecision(
                "failed",
                "stall",
                limits.linear_speed_mps,
                0.0,
                0.0,
                0.0,
                "CLEAR",
            )
        return super().supervise(intent, snapshot, limits)


class StallOnCallsSupervisor(ReplayCollisionSupervisor):
    def __init__(self, stall_calls: set[int]) -> None:
        super().__init__()
        self.stall_calls = set(stall_calls)

    def supervise(
        self,
        intent: AdaptiveMissionIntent,
        snapshot: Mapping[str, Any],
        limits: AdaptiveMissionLimits,
    ) -> MovementDecision:
        next_call = self.calls + 1
        if next_call in self.stall_calls:
            self.calls = next_call
            return MovementDecision(
                "failed",
                "stall",
                (
                    limits.linear_speed_mps
                    if intent.action == "move_distance"
                    else 0.0
                ),
                (
                    limits.angular_speed_rad_s
                    if intent.action == "turn_angle"
                    else 0.0
                ),
                0.0,
                0.0,
                "CLEAR",
            )
        return super().supervise(intent, snapshot, limits)


def _wait_terminal(adapter: AdaptiveMissionAdapter, timeout_s: float = 3.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = dict(adapter.snapshot())
        if snapshot["mission"]["terminal"]:
            return snapshot
        time.sleep(0.005)
    raise AssertionError("adaptive mission did not reach a terminal state")


def _wait_controller_terminal(
    controller: AdaptiveMissionController,
    timeout_s: float = 3.0,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = controller.snapshot()
        if snapshot["terminal"]:
            return snapshot
        time.sleep(0.005)
    raise AssertionError("adaptive mission controller did not become terminal")


def _run_stall_sequence(
    *,
    actions: list[tuple[str, float]],
    stall_calls: set[int],
    prompt: str,
) -> tuple[dict[str, Any], SequenceProvider]:
    provider = SequenceProvider(actions)
    executor = RecoveryReplayAdaptiveMissionExecutor(
        supervisor=StallOnCallsSupervisor(stall_calls)
    )
    mission_id = "navigation-stall-sequence"
    first_snapshot = executor.snapshot(mission_id)
    first_raw = provider.choose(prompt, first_snapshot)
    approved_at = time.time()
    first_intent = AdaptiveMissionIntent.validated(
        first_raw,
        revision=1,
        snapshot=first_snapshot,
        issued_at_s=approved_at,
        provider_id=provider.provider_id,
        model_id=provider.model_id,
        limits=executor.limits,
    )
    controller = AdaptiveMissionController(
        mission_id=mission_id,
        prompt=prompt,
        proposal_digest="c" * 64,
        operator="operator@example.com",
        authenticated=True,
        authentication_source="tailscale-serve",
        approved_at_s=approved_at,
        first_snapshot=first_snapshot,
        first_intent=first_intent,
        provider=provider,
        executor=executor,
        enable_navigation_outcome_recovery=True,
    )
    try:
        controller.start()
        terminal = _wait_controller_terminal(controller)
    finally:
        controller.close()
    return terminal, provider


def test_adaptive_mission_provider_prompt_exposes_semantics_without_granting_safety_authority() -> None:
    snapshot = ReplayAdaptiveMissionExecutor().snapshot("semantic-prompt")
    request = json.loads(
        _adaptive_mission_provider_prompt(
            "Find the shoe, approach only through clear floor, then stop.",
            snapshot,
        )
    )

    assert request["operator_prompt"].startswith("Find the shoe")
    assert request["world_snapshot"]["observations"]["perception"][
        "available"
    ] is False
    rules = "\n".join(request["rules"])
    assert "objective evidence only" in rules
    assert "never override lidar collision safety" in rules
    assert "forward_usable_m" in rules
    assert "observations.perception.available is true" in rules
    assert "explicit enrollment evidence" in rules
    assert "missing or stale perception" in rules
    assert "do not immediately repeat the same forward move" in rules
    assert "compensate with higher speed or motor authority" in rules
    assert "retry after a maneuver" in rules
    assert "likely fixed obstacle" in rules
    assert "likely minor surface feature" in rules
    assert request["authority"]["allowed_intents"] == [
        "move_distance",
        "observe",
        "stop",
        "turn_angle",
    ]
    assert request["authority"]["translation_clearance"] == {
        "translation_reserve_m": 0.40,
        "forward_usable_m": 1.40,
        "reverse_usable_m": 0.80,
    }
    configured = json.loads(
        _adaptive_mission_provider_prompt(
            "Observe safely.",
            snapshot,
            limits=AdaptiveMissionLimits(mission_lease_s=120.0),
        )
    )
    assert configured["authority"]["mission_lease_s"] == 120.0


def test_recovery_prompt_supplies_only_typed_limiting_condition() -> None:
    snapshot = ReplayAdaptiveMissionExecutor().snapshot(
        "typed-recovery-prompt"
    )
    rejected = _raw(snapshot, "move_distance", 0.25)
    rejection = RecoverableSafetyRejection(
        "clearance exceeded",
        rejected_request=rejected,
        violated_condition=(
            "translation_exceeds_forward_usable_clearance"
        ),
        limit_name="forward_usable_m",
        limit_value=0.2225,
        limit_unit="m",
        rejected_snapshot_id=str(snapshot["snapshot_id"]),
    ).feedback(current_snapshot_id=str(snapshot["snapshot_id"]))

    request = json.loads(
        _adaptive_mission_provider_prompt(
            PROMPT,
            snapshot,
            safety_rejection=rejection,
        )
    )

    assert request["safety_rejection"] == rejection
    assert set(request["safety_rejection"]) == {
        "schema",
        "rejected_request",
        "violated_condition",
        "applicable_numeric_limit",
        "rejected_snapshot_id",
        "current_snapshot_id",
        "motion_executed",
    }
    assert "alternatives" not in request["safety_rejection"]
    assert "candidate" not in json.dumps(
        request["safety_rejection"]
    ).lower()
    recovery_rules = "\n".join(request["rules"])
    assert "consider observe" not in recovery_rules
    assert "choose turn_angle" not in recovery_rules
    assert "use fresh evidence to back away" not in recovery_rules


def test_settled_navigation_outcome_uses_normal_schema_without_alternatives() -> None:
    objective = "Travel to the inspection point and return to the dock."
    base = dict(
        RecoveryReplayAdaptiveMissionExecutor().snapshot(
            "navigation-outcome-prompt"
        )
    )
    base.pop("schema", None)
    base.pop("snapshot_id", None)
    base["last_execution"] = {
        "intent": {
            "action": "move_distance",
            "distance_m": 0.10,
            "angle_deg": 0.0,
        },
        "movement": {
            "outcome": "allowed",
            "reason": "collision_supervised",
            "collision_state": "CLEAR",
        },
        "navigation_outcome": {
            "classification": "recoverable_settled",
            "reason": "target_error",
            "recoverable": True,
            "terminal_settled": True,
            "fresh_evidence_required": True,
            "requested_distance_m": 0.10,
            "measured_distance_m": 0.07,
            "residual_distance_m": 0.03,
            "measurement_uncertainty": {
                "odometry_precision": "imprecise",
            },
            "encoder_evidence": {
                "left_encoder_delta_counts": 302,
                "right_encoder_delta_counts": 299,
            },
            "collision_state": "CLEAR",
        },
        "route_terminal": {
            "status": "failed",
            "terminal_reason": "target_error",
            "terminal_settled": True,
            "measured_distance_m": 0.07,
            "left_encoder_delta_counts": 302,
            "right_encoder_delta_counts": 299,
        },
    }
    snapshot = make_world_snapshot(base)

    request = json.loads(
        _adaptive_mission_provider_prompt(objective, snapshot)
    )

    assert request["operator_prompt"] == objective
    assert request["world_snapshot"]["last_execution"][
        "navigation_outcome"
    ]["residual_distance_m"] == pytest.approx(0.03)
    assert "outcome_judgment_instruction" in request
    encoded = json.dumps(request).lower()
    assert "no candidate" in encoded
    assert "consider observe" not in encoded
    assert "change the approach angle" not in encoded
    assert '"alternatives"' not in encoded
    output_schema = _adaptive_mission_output_schema(
        str(snapshot["snapshot_id"])
    )
    assert output_schema["properties"]["action"][
        "enum"
    ] == ["move_distance", "observe", "stop", "turn_angle"]
    assert output_schema["properties"]["snapshot_id"]["enum"] == [
        snapshot["snapshot_id"]
    ]


def test_output_schema_requires_and_binds_exact_current_snapshot() -> None:
    current_snapshot_id = "fresh-snapshot-id"
    historical_snapshot_id = "prior-snapshot-id"

    output_schema = _adaptive_mission_output_schema(current_snapshot_id)

    assert output_schema["properties"]["snapshot_id"] == {
        "type": "string",
        "enum": [current_snapshot_id],
    }
    assert historical_snapshot_id not in output_schema["properties"][
        "snapshot_id"
    ]["enum"]
    assert output_schema["properties"]["action"]["enum"] == [
        "move_distance",
        "observe",
        "stop",
        "turn_angle",
    ]
    with pytest.raises(
        MissionValidationError,
        match="requires an exact snapshot identity",
    ):
        _adaptive_mission_output_schema("")


def test_normal_success_prompt_has_no_recovery_specific_feedback() -> None:
    snapshot = ReplayAdaptiveMissionExecutor().snapshot(
        "normal-success-prompt"
    )
    request = json.loads(
        _adaptive_mission_provider_prompt(
            "Patrol the available corridor.",
            snapshot,
        )
    )

    assert "outcome_judgment_instruction" not in request
    assert "safety_rejection" not in request


def test_oauth_provider_copies_only_digest_bound_camera_attachment(
    tmp_path,
) -> None:
    payload = b"\xff\xd8oauth-camera-observation\xff\xd9"
    source = tmp_path / "sensor-frame.jpg"
    source.write_bytes(payload)
    base = dict(
        ReplayAdaptiveMissionExecutor().snapshot(
            "camera-oauth-attachment"
        )
    )
    base.pop("schema", None)
    base.pop("snapshot_id", None)
    observations = dict(base["observations"])
    perception = dict(observations["perception"])
    perception["camera_frame_id"] = "live-camera-00000007"
    perception["camera_image"] = {
        "available": True,
        "frame_id": "live-camera-00000007",
        "path": str(source),
        "mime_type": "image/jpeg",
        "sha256": hashlib.sha256(payload).hexdigest(),
        "byte_count": len(payload),
    }
    observations["perception"] = perception
    base["observations"] = observations
    snapshot = make_world_snapshot(base)
    provider_root = tmp_path / "provider"
    provider_root.mkdir()

    copied = _verified_camera_attachment(snapshot, provider_root)

    assert copied == provider_root / "camera-observation.jpg"
    assert copied.read_bytes() == payload
    assert copied.stat().st_mode & 0o777 == 0o600
    source.write_bytes(b"mutated")
    with pytest.raises(
        MissionValidationError,
        match="attachment digest is invalid",
    ):
        _verified_camera_attachment(snapshot, provider_root)


def test_real_oauth_provider_passes_verified_camera_to_codex(
    tmp_path,
    monkeypatch,
) -> None:
    payload = b"\xff\xd8multimodal-oauth-frame\xff\xd9"
    source = tmp_path / "live-frame.jpg"
    source.write_bytes(payload)
    base = dict(
        ReplayAdaptiveMissionExecutor().snapshot(
            "multimodal-oauth-mission"
        )
    )
    base.pop("schema", None)
    base.pop("snapshot_id", None)
    observations = dict(base["observations"])
    perception = dict(observations["perception"])
    perception["camera_frame_id"] = "live-camera-00000008"
    perception["camera_image"] = {
        "available": True,
        "frame_id": "live-camera-00000008",
        "path": str(source),
        "mime_type": "image/jpeg",
        "sha256": hashlib.sha256(payload).hexdigest(),
        "byte_count": len(payload),
    }
    observations["perception"] = perception
    base["observations"] = observations
    snapshot = make_world_snapshot(base)
    codex_calls = []

    def fake_run(command, **kwargs):
        if command[1:3] == ["login", "status"]:
            return subprocess.CompletedProcess(
                command,
                0,
                stdout="Logged in using ChatGPT\n",
                stderr="",
            )
        codex_calls.append(list(command))
        image_index = command.index("--image") + 1
        assert Path(command[image_index]).read_bytes() == payload
        schema_index = command.index("--output-schema") + 1
        output_schema = json.loads(
            Path(command[schema_index]).read_text(encoding="utf-8")
        )
        assert output_schema["properties"]["snapshot_id"]["enum"] == [
            snapshot["snapshot_id"]
        ]
        output_index = command.index("--output-last-message") + 1
        Path(command[output_index]).write_text(
            json.dumps(_raw(snapshot, "observe")),
            encoding="utf-8",
        )
        return subprocess.CompletedProcess(
            command,
            0,
            stdout="",
            stderr="",
        )

    monkeypatch.setattr(
        "sphero_rvr_driver.adaptive_mission_controller.shutil.which",
        lambda _: "/usr/bin/codex",
    )
    monkeypatch.setattr(
        "sphero_rvr_driver.adaptive_mission_controller.subprocess.run",
        fake_run,
    )
    provider = CodexOAuthAdaptiveMissionIntentProvider(
        codex_command="codex",
        timeout_s=5.0,
        integration="exec",
    )

    chosen = provider.choose("Explore from visual evidence.", snapshot)

    assert chosen["action"] == "observe"
    assert len(codex_calls) == 1
    assert "--image" in codex_calls[0]


def test_app_server_provider_reuses_client_but_sends_compact_isolated_evidence(
    tmp_path,
    monkeypatch,
) -> None:
    payload = b"\xff\xd8persistent-oauth-frame\xff\xd9"
    source = tmp_path / "live-frame.jpg"
    source.write_bytes(payload)
    base = dict(
        ReplayAdaptiveMissionExecutor().snapshot("persistent-app-server")
    )
    base.pop("schema", None)
    base.pop("snapshot_id", None)
    observations = dict(base["observations"])
    observations["camera_detections"] = [
        {"label": "shoe", "confidence": 0.91}
    ]
    perception = dict(observations["perception"])
    perception.update(
        {
            "camera_fresh": True,
            "camera_frame_id": "live-camera-00000009",
            "camera_image": {
                "available": True,
                "frame_id": "live-camera-00000009",
                "path": str(source),
                "mime_type": "image/jpeg",
                "sha256": hashlib.sha256(payload).hexdigest(),
                "byte_count": len(payload),
            },
        }
    )
    observations["perception"] = perception
    base["observations"] = observations
    snapshot = make_world_snapshot(base)

    class FakeAppServer:
        instances = 0

        def __init__(self, **_kwargs) -> None:
            FakeAppServer.instances += 1
            self.calls = []

        def run_turn(self, **kwargs):
            self.calls.append(kwargs)
            if kwargs["image_path"] is not None:
                assert Path(kwargs["image_path"]).read_bytes() == payload
            request = json.loads(kwargs["prompt"])
            assert "path" not in json.dumps(request["world_snapshot"])
            return json.dumps(_raw(snapshot, "observe")), 3.0, 0

        def cancel(self) -> None:
            return None

        def close(self) -> None:
            return None

    monkeypatch.setattr(
        "sphero_rvr_driver.adaptive_mission_controller.CodexAppServerClient",
        FakeAppServer,
    )
    latency_records: list[str] = []
    provider = CodexOAuthAdaptiveMissionIntentProvider(
        timeout_s=5.0,
        latency_logger=latency_records.append,
    )

    provider.choose("Explore mapped free space.", snapshot)
    provider.choose("Find the detected shoe.", snapshot)
    rejection = RecoverableSafetyRejection(
        "clearance exceeded",
        rejected_request=_raw(snapshot, "move_distance", 0.25),
        violated_condition=(
            "translation_exceeds_forward_usable_clearance"
        ),
        limit_name="forward_usable_m",
        limit_value=0.2225,
        limit_unit="m",
        rejected_snapshot_id=str(snapshot["snapshot_id"]),
    ).feedback(current_snapshot_id=str(snapshot["snapshot_id"]))
    provider.choose_after_safety_rejection(
        "Explore mapped free space.", snapshot, rejection
    )

    assert FakeAppServer.instances == 1
    assert provider._client is not None
    assert [call["image_path"] is not None for call in provider._client.calls] == [
        False,
        True,
        False,
    ]
    assert all(
        call["output_schema"]["properties"]["snapshot_id"]["enum"]
        == [snapshot["snapshot_id"]]
        for call in provider._client.calls
    )
    assert all(
        call["output_schema"]["properties"]["action"]["enum"]
        == ["move_distance", "observe", "stop", "turn_angle"]
        for call in provider._client.calls
    )
    prompts = [
        json.loads(call["prompt"])
        for call in provider._client.calls
    ]
    assert "safety_rejection" not in prompts[0]
    assert "safety_rejection" not in prompts[1]
    assert prompts[2]["safety_rejection"] == rejection
    metrics = provider.latency_history()
    assert len({item["decision_id"] for item in metrics}) == 3
    assert [item["isolated_model_thread"] for item in metrics] == [
        True,
        True,
        True,
    ]
    assert [item["safety_recovery"] for item in metrics] == [
        False,
        False,
        True,
    ]
    compact = _decision_evidence_snapshot(snapshot)
    assert compact["observations"]["camera_detections"] == [
        {"label": "shoe", "confidence": 0.91}
    ]
    assert compact["observations"]["perception"]["camera_fresh"] is True
    assert "path" not in compact["observations"]["perception"]["camera_image"]
    assert "recognized_objects" not in compact["observations"]
    assert all(
        "received_at_s" not in receipt
        for receipt in compact["evidence"]["source_receipts"].values()
    )
    assert (
        "executed_segments"
        not in compact["last_execution"]["route_terminal"]
    )
    assert _visual_reasoning_relevance(
        "Find the detected shoe.", snapshot
    ) == (True, "objective:detected_label")
    assert _visual_reasoning_relevance(
        "Choose one safe action.", snapshot
    ) == (False, "not_relevant")
    assert len(provider.latency_history()) == 3
    assert len(latency_records) == 3
    assert all(
        record.startswith("adaptive_planning_cycle {")
        and "OPENAI_API_KEY" not in record
        and str(source) not in record
        for record in latency_records
    )


def test_isolated_provider_decision_uses_call_specific_deadline(
    monkeypatch,
) -> None:
    snapshot = ReplayAdaptiveMissionExecutor().snapshot(
        "provider-decision-deadline"
    )

    class FakeAppServer:
        def __init__(self, **_kwargs) -> None:
            self.calls = []

        def run_turn(self, **kwargs):
            self.calls.append(kwargs)
            return json.dumps(_raw(snapshot, "stop")), 0.0, 0

        def cancel(self) -> None:
            return None

        def close(self) -> None:
            return None

    monkeypatch.setattr(
        "sphero_rvr_driver.adaptive_mission_controller.CodexAppServerClient",
        FakeAppServer,
    )
    provider = CodexOAuthAdaptiveMissionIntentProvider(timeout_s=120.0)

    provider.choose_for_decision(
        "Stop safely.",
        snapshot,
        decision_id="lease-bounded-decision",
        safety_rejection=None,
        decision_timeout_s=0.25,
    )

    assert provider._client is not None
    assert provider._client.calls[0]["timeout_s"] == pytest.approx(0.25)
    assert provider.latency_history()[0]["decision_timeout_s"] == pytest.approx(
        0.25
    )


def test_adaptive_mission_rejects_malformed_semantic_observation_shape() -> None:
    snapshot = dict(ReplayAdaptiveMissionExecutor().snapshot("malformed-semantics"))
    snapshot.pop("schema", None)
    snapshot.pop("snapshot_id", None)
    observations = dict(snapshot["observations"])
    observations["recognized_objects"] = {"track_id": "not-a-list"}
    snapshot["observations"] = observations
    malformed = make_world_snapshot(snapshot)

    with pytest.raises(
        MissionValidationError,
        match="recognized_objects must be a list",
    ):
        validate_world_snapshot(
            malformed,
            mission_id="malformed-semantics",
        )


def test_collision_escape_never_overrides_operator_stop_or_estop() -> None:
    base = dict(
        ReplayAdaptiveMissionExecutor().snapshot(
            "collision-escape-safety"
        )
    )
    base.pop("schema", None)
    base.pop("snapshot_id", None)
    base["safety"] = {
        **dict(base["safety"]),
        "collision_state": "STOPPED",
        "stop_active": True,
        "control_state": "STOP",
    }
    stopped = make_world_snapshot(base)

    with pytest.raises(MissionValidationError, match="STOP is active"):
        validate_world_snapshot(
            stopped,
            mission_id="collision-escape-safety",
            require_motion=True,
            allow_supervised_collision_escape=True,
        )

    base["safety"] = {
        **dict(base["safety"]),
        "control_state": "READY",
        "estop_latched": True,
    }
    estopped = make_world_snapshot(base)
    with pytest.raises(MissionValidationError, match="ESTOP is latched"):
        validate_world_snapshot(
            estopped,
            mission_id="collision-escape-safety",
            require_motion=True,
            allow_supervised_collision_escape=True,
        )


@pytest.mark.parametrize(
    ("action", "value"),
    (
        ("move_distance", 0.25),
        ("turn_angle", -45.0),
        ("observe", 0.0),
        ("stop", 0.0),
    ),
)
def test_adaptive_mission_accepts_all_supported_snapshot_bound_intents(
    action: str, value: float
) -> None:
    limits = AdaptiveMissionLimits()
    executor = ReplayAdaptiveMissionExecutor(limits=limits)
    snapshot = executor.snapshot("intent-validation")

    intent = AdaptiveMissionIntent.validated(
        _raw(snapshot, action, value),
        revision=1,
        snapshot=snapshot,
        issued_at_s=10.0,
        provider_id="test",
        model_id="test",
        limits=limits,
    )

    assert intent.action == action
    assert intent.snapshot_id == snapshot["snapshot_id"]
    assert intent.lease_s == 5.0
    assert intent.timeout_s == 5.0


@pytest.mark.parametrize(
    ("action", "objective_status", "message"),
    (
        ("stop", "in_progress", "stop requires"),
        ("move_distance", "complete", "motion requires"),
        ("observe", "complete", "observe requires"),
        ("turn_angle", "", "objective status is unsupported"),
    ),
)
def test_adaptive_mission_rejects_unreasonable_action_objective_status_pair(
    action: str,
    objective_status: str,
    message: str,
) -> None:
    limits = AdaptiveMissionLimits()
    snapshot = ReplayAdaptiveMissionExecutor(limits=limits).snapshot(
        "objective-status"
    )
    value = 0.10 if action == "move_distance" else 10.0
    raw = _raw(snapshot, action, value)
    raw["objective_status"] = objective_status

    with pytest.raises(MissionValidationError, match=message):
        AdaptiveMissionIntent.validated(
            raw,
            revision=1,
            snapshot=snapshot,
            issued_at_s=10.0,
            provider_id="test",
            model_id="test",
            limits=limits,
        )


@pytest.mark.parametrize(
    ("action", "value", "message"),
    (
        ("move_distance", 0.251, "translation exceeds 0.25"),
        ("turn_angle", 45.1, "rotation exceeds 45"),
    ),
)
def test_adaptive_mission_rejects_intent_above_per_intent_envelope(
    action: str, value: float, message: str
) -> None:
    limits = AdaptiveMissionLimits()
    snapshot = ReplayAdaptiveMissionExecutor(limits=limits).snapshot("bounded")

    with pytest.raises(MissionValidationError, match=message):
        AdaptiveMissionIntent.validated(
            _raw(snapshot, action, value),
            revision=1,
            snapshot=snapshot,
            issued_at_s=10.0,
            provider_id="test",
            model_id="test",
            limits=limits,
        )


def test_adaptive_mission_replans_repeatedly_without_a_cumulative_travel_cap(tmp_path) -> None:
    provider = SequenceProvider(
        [("move_distance", 0.25)] * 6
        + [("turn_angle", 45.0), ("observe", 0.0), ("stop", 0.0)]
    )
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "replanning.sqlite3",
        source_sha="adaptive-mission-source",
        deployed_sha="adaptive-mission-source",
        allow_loopback_test_approval=True,
    )
    try:
        proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
        assert proposed["mission"]["state"] == "PROPOSED"
        assert proposed["proposal"]["segments"] == []
        assert proposed["proposal"]["interpreted_objective"]
        assert proposed["proposal"]["first_intent"]["action"] == "move_distance"
        assert proposed["proposal"]["limits"]["max_cumulative_translation_m"] is None
        assert proposed["proposal"]["limits"]["mission_lease_s"] == 900.0
        assert proposed["proposal"]["contract"]["per_intent_approval"] is False

        running = adapter.approve(proposed["approval"]["required_phrase"])
        assert running["mission"]["state"] in {"RUNNING", "COMPLETE"}
        terminal = _wait_terminal(adapter)
    finally:
        adapter.close()

    revisions = terminal["mission"]["result"]["intent_revisions"]
    assert terminal["mission"]["state"] == "COMPLETE"
    assert terminal["mission"]["terminal_reason"] == "planner_stop"
    assert provider.calls == 9
    assert [item["action"] for item in revisions] == [
        "move_distance",
        "move_distance",
        "move_distance",
        "move_distance",
        "move_distance",
        "move_distance",
        "turn_angle",
        "observe",
        "stop",
    ]
    assert revisions[-1]["revision"] == 9
    assert terminal["mission"]["result"]["final_snapshot"]["progress"][
        "cumulative_translation_m"
    ] == pytest.approx(1.5)
    assert all(
        abs(item["distance_m"]) <= 0.25 and abs(item["angle_deg"]) <= 45.0
        for item in revisions
    )
    assert terminal["mission"]["result"]["approval"]["authenticated"] is True
    assert (
        terminal["mission"]["result"]["approval"]["authentication_source"]
        == "explicit-loopback-test-mode"
    )
    assert terminal["mission"]["result"]["limits"]["linear_speed_mps"] == 0.10
    assert terminal["mission"]["result"]["limits"]["angular_speed_rad_s"] == 0.4
    snapshot_messages = [
        event["message"]
        for event in terminal["mission"]["result"]["events"]
        if event["event_type"] == "snapshot"
    ]
    assert snapshot_messages
    assert all(
        "camera=unavailable" in message
        and "lidar=unavailable" in message
        and "localization=unavailable" in message
        for message in snapshot_messages
    )


def test_model_stop_distinguishes_completed_from_blocked_objective(
    tmp_path,
) -> None:
    class BlockedStopProvider(SequenceProvider):
        def choose(
            self, prompt: str, snapshot: Mapping[str, Any]
        ) -> Mapping[str, Any]:
            raw = dict(super().choose(prompt, snapshot))
            raw["objective_status"] = "blocked"
            raw["rationale"] = (
                "Fresh evidence offers no validated observation or maneuver."
            )
            return raw

    provider = BlockedStopProvider([("stop", 0.0)])
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "blocked-stop.sqlite3",
        source_sha="adaptive-mission-blocked-stop",
        allow_loopback_test_approval=True,
    )
    try:
        proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
        adapter.approve(proposed["approval"]["required_phrase"])
        terminal = _wait_terminal(adapter)
    finally:
        adapter.close()

    assert terminal["mission"]["state"] == "BLOCKED"
    assert terminal["mission"]["terminal_reason"] == (
        "planner_objective_blocked"
    )
    revision = terminal["mission"]["result"]["intent_revisions"][0]
    assert revision["objective_status"] == "blocked"


def test_adaptive_mission_replans_movement_from_fresh_recognition_evidence(tmp_path) -> None:
    provider = RecognitionDrivenProvider()
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "recognition-driven.sqlite3",
        source_sha="semantic-adaptive-mission-source",
        allow_loopback_test_approval=True,
        executor_factory=SemanticReplayAdaptiveMissionExecutor,
    )
    try:
        proposed = adapter.propose(
            "Find the shoe, approach it only through clear floor, then stop.",
            "adaptive_mission_explore",
        )
        assert proposed["proposal"]["first_intent"]["action"] == "observe"
        adapter.approve(proposed["approval"]["required_phrase"])
        terminal = _wait_terminal(adapter)
    finally:
        adapter.close()

    result = terminal["mission"]["result"]
    assert result["status"] == "complete"
    assert [item["action"] for item in result["intent_revisions"]] == [
        "observe",
        "turn_angle",
        "move_distance",
        "stop",
    ]
    assert len(provider.snapshots) == 4
    assert provider.snapshots[0]["observations"]["perception"][
        "available"
    ] is False
    assert all(
        snapshot["observations"]["recognized_objects"][0]["track_id"]
        == "object-shoe-1"
        for snapshot in provider.snapshots[1:]
    )
    assert result["final_snapshot"]["progress"][
        "cumulative_translation_m"
    ] == pytest.approx(0.10)


def test_collision_supervisor_veto_recovers_with_typed_zero_motion_rejection(
    tmp_path,
) -> None:
    provider = SequenceProvider([("move_distance", 0.25), ("stop", 0.0)])
    supervisor = ReplayCollisionSupervisor(collision_on_intent=1)
    executor = ReplayAdaptiveMissionExecutor(supervisor=supervisor)
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "collision.sqlite3",
        source_sha="adaptive-mission-collision",
        allow_loopback_test_approval=True,
        executor_factory=lambda: executor,
    )
    try:
        proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
        adapter.approve(proposed["approval"]["required_phrase"])
        terminal = _wait_terminal(adapter)
    finally:
        adapter.close()

    revision = terminal["mission"]["result"]["intent_revisions"][0]
    movement = revision["execution"]["movement"]
    assert terminal["mission"]["state"] == "COMPLETE"
    assert terminal["mission"]["terminal_reason"] == "planner_stop"
    assert provider.calls == 2
    assert movement["requested"]["linear_mps"] == pytest.approx(0.10)
    assert movement["supervised"]["linear_mps"] == 0.0
    assert movement["motor_topic_publisher"] == "lidar_collision_stop_supervisor"
    assert len(provider.safety_rejections) == 1
    rejection = provider.safety_rejections[0]
    assert set(rejection) == {
        "schema",
        "rejected_request",
        "violated_condition",
        "applicable_numeric_limit",
        "rejected_snapshot_id",
        "current_snapshot_id",
        "motion_executed",
    }
    assert rejection["violated_condition"] == "collision_supervisor_veto"
    assert rejection["motion_executed"] is False
    assert rejection["applicable_numeric_limit"] == {
        "name": "maximum_executed_motion_m",
        "value": 0.0,
        "unit": "m",
    }
    assert "alternatives" not in rejection
    assert provider.snapshots[0] != provider.snapshots[1]
    assert rejection["current_snapshot_id"] == provider.snapshots[1]


@pytest.mark.parametrize(
    "supervisor",
    [
        ReplayCollisionSupervisor(collision_on_intent=1),
        StallOnceSupervisor(),
    ],
    ids=["collision", "stall"],
)
def test_exploration_recovery_reverses_turns_and_replans_after_failure(
    supervisor,
) -> None:
    provider = SequenceProvider(
        [
            ("move_distance", 0.20),
            ("move_distance", -0.15),
            ("turn_angle", 45.0),
            ("stop", 0.0),
        ]
    )
    executor = RecoveryReplayAdaptiveMissionExecutor(
        supervisor=supervisor
    )
    mission_id = f"recovery-{type(supervisor).__name__}"
    first_snapshot = executor.snapshot(mission_id)
    first_raw = provider.choose(PROMPT, first_snapshot)
    approved_at = time.time()
    first_intent = AdaptiveMissionIntent.validated(
        first_raw,
        revision=1,
        snapshot=first_snapshot,
        issued_at_s=approved_at,
        provider_id=provider.provider_id,
        model_id=provider.model_id,
        limits=executor.limits,
    )
    controller = AdaptiveMissionController(
        mission_id=mission_id,
        prompt="Explore and map the room",
        proposal_digest="a" * 64,
        operator="operator@example.com",
        authenticated=True,
        authentication_source="tailscale-serve",
        approved_at_s=approved_at,
        first_snapshot=first_snapshot,
        first_intent=first_intent,
        provider=provider,
        executor=executor,
        enable_navigation_outcome_recovery=True,
    )
    try:
        controller.start()
        terminal = _wait_controller_terminal(controller)
    finally:
        controller.close()

    revisions = terminal["result"]["intent_revisions"]
    assert terminal["status"] == "complete"
    assert [revision["action"] for revision in revisions] == [
        "move_distance",
        "move_distance",
        "turn_angle",
        "stop",
    ]
    assert revisions[1]["distance_m"] == pytest.approx(-0.15)
    assert revisions[1]["provider_id"] == provider.provider_id
    assert revisions[1]["supervised_collision_escape"] is (
        isinstance(supervisor, ReplayCollisionSupervisor)
        and not isinstance(supervisor, StallOnceSupervisor)
    )
    assert revisions[2]["angle_deg"] == pytest.approx(45.0)
    assert provider.calls == 4
    event_types = [event["event_type"] for event in terminal["events"]]
    assert (
        "safety_rejection"
        if isinstance(supervisor, ReplayCollisionSupervisor)
        and not isinstance(supervisor, StallOnceSupervisor)
        else "outcome_judgment_submitted"
    ) in event_types


def test_non_exploration_stall_gets_fresh_isolated_normal_decision() -> None:
    objective = "Travel to the charging dock and stop there."
    terminal, provider = _run_stall_sequence(
        actions=[
            ("move_distance", 0.10),
            ("turn_angle", 30.0),
            ("stop", 0.0),
        ],
        stall_calls={1},
        prompt=objective,
    )

    assert terminal["status"] == "complete"
    assert provider.prompts == [objective, objective, objective]
    assert len(set(provider.snapshots)) == 3
    recovered_snapshot = provider.snapshot_payloads[1]
    outcome = recovered_snapshot["last_execution"]["navigation_outcome"]
    assert outcome["reason"] == "stall"
    assert outcome["recoverable"] is True
    assert provider.safety_rejections == []
    recovery = terminal["result"]["navigation_outcome_recovery"]
    assert recovery["judgments"][0]["reason"] == "stall"
    decision = next(
        item
        for item in terminal["result"]["isolated_decisions"]
        if item["kind"] == "navigation_outcome_judgment"
    )
    assert decision["snapshot_id"] == provider.snapshots[1]
    assert decision["status"] == "validated"


def test_completed_turn_resets_consecutive_stall_recovery_debt() -> None:
    terminal, provider = _run_stall_sequence(
        actions=[
            ("move_distance", 0.10),
            ("turn_angle", 20.0),
            ("move_distance", 0.08),
            ("turn_angle", -20.0),
            ("stop", 0.0),
        ],
        stall_calls={1, 3},
        prompt="Inspect both aisle ends and return to the start.",
    )

    assert terminal["status"] == "complete"
    assert provider.calls == 5
    recovery = terminal["result"]["navigation_outcome_recovery"]
    assert len(recovery["judgments"]) == 2
    assert len(recovery["resets"]) == 2
    assert all(
        reset["previous_consecutive_stall_recoveries"] == 1
        and reset["new_consecutive_stall_recoveries"] == 0
        for reset in recovery["resets"]
    )
    assert recovery["terminal_exhaustion_reason"] == ""


def test_repeated_consecutive_stalls_exhaust_only_stall_budget() -> None:
    terminal, provider = _run_stall_sequence(
        actions=[
            ("move_distance", 0.10),
            ("move_distance", 0.08),
            ("move_distance", 0.06),
        ],
        stall_calls={1, 2, 3},
        prompt="Approach the inspection marker.",
    )

    assert terminal["status"] == "blocked"
    assert terminal["terminal_reason"] == (
        "consecutive_stall_recovery_budget_exhausted"
    )
    assert provider.calls == 3
    recovery = terminal["result"]["navigation_outcome_recovery"]
    assert recovery["consecutive_stall_recoveries"] == 2
    assert len(recovery["judgments"]) == 2
    assert recovery["terminal_exhaustion_reason"] == (
        "consecutive_stall_recovery_budget_exhausted"
    )
    assert terminal["result"]["intent_revisions"][-1]["execution"][
        "movement"
    ]["supervised"]["linear_mps"] == 0.0


def test_llm_problem_solving_reverse_is_capped_at_fifteen_centimeters() -> None:
    provider = SequenceProvider(
        [
            ("move_distance", 0.20),
            ("move_distance", -0.20),
            ("move_distance", -0.20),
        ]
    )
    executor = RecoveryReplayAdaptiveMissionExecutor(
        supervisor=ReplayCollisionSupervisor(collision_on_intent=1)
    )
    mission_id = "recovery-reverse-cap"
    first_snapshot = executor.snapshot(mission_id)
    first_raw = provider.choose(PROMPT, first_snapshot)
    approved_at = time.time()
    first_intent = AdaptiveMissionIntent.validated(
        first_raw,
        revision=1,
        snapshot=first_snapshot,
        issued_at_s=approved_at,
        provider_id=provider.provider_id,
        model_id=provider.model_id,
        limits=executor.limits,
    )
    controller = AdaptiveMissionController(
        mission_id=mission_id,
        prompt="Explore and map the room",
        proposal_digest="b" * 64,
        operator="operator@example.com",
        authenticated=True,
        authentication_source="tailscale-serve",
        approved_at_s=approved_at,
        first_snapshot=first_snapshot,
        first_intent=first_intent,
        provider=provider,
        executor=executor,
        enable_navigation_outcome_recovery=True,
    )
    try:
        controller.start()
        terminal = _wait_controller_terminal(controller)
    finally:
        controller.close()

    assert terminal["status"] == "blocked"
    assert terminal["terminal_reason"] == (
        "safety_rejection_retry_budget_exhausted"
    )
    assert len(terminal["result"]["intent_revisions"]) == 1
    assert provider.calls == 3
    assert len(provider.safety_rejections) == 2
    assert all(
        rejection["motion_executed"] is False
        for rejection in provider.safety_rejections
    )
    assert len(
        terminal["result"]["safety_recovery"]["rejections"]
    ) == 3


def test_stale_updated_snapshot_stops_before_another_provider_call(tmp_path) -> None:
    provider = SequenceProvider([("move_distance", 0.10), ("stop", 0.0)])
    executor = ReplayAdaptiveMissionExecutor(stale_after_intents=1)
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "stale.sqlite3",
        source_sha="adaptive-mission-stale",
        allow_loopback_test_approval=True,
        executor_factory=lambda: executor,
    )
    try:
        proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
        adapter.approve(proposed["approval"]["required_phrase"])
        terminal = _wait_terminal(adapter)
    finally:
        adapter.close()

    assert terminal["mission"]["state"] == "BLOCKED"
    assert "stale_or_unsafe_evidence" in terminal["mission"]["terminal_reason"]
    assert "scan_fresh" in terminal["mission"]["terminal_reason"]
    assert provider.calls == 1
    assert terminal["mission"]["result"]["final_snapshot"]["evidence"][
        "scan_fresh"
    ] is False


def test_provider_network_failure_is_terminal_and_never_restarts(tmp_path) -> None:
    provider = SequenceProvider(
        [("observe", 0.0)],
        fail_after_calls=1,
    )
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "provider.sqlite3",
        source_sha="adaptive-mission-provider-failure",
        allow_loopback_test_approval=True,
    )
    try:
        proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
        adapter.approve(proposed["approval"]["required_phrase"])
        terminal = _wait_terminal(adapter)
        later = dict(adapter.snapshot())
    finally:
        adapter.close()

    assert terminal["mission"]["state"] == "FAILED"
    assert "provider_failure: ConnectionError" in terminal["mission"]["terminal_reason"]
    assert later["mission"]["state"] == "FAILED"
    assert provider.calls == 2
    assert provider.safety_rejections == []
    assert terminal["mission"]["result"]["auto_resume"] is False


class TimeoutProvider(SequenceProvider):
    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        payload = dict(super().choose(prompt, snapshot))
        payload["timeout_s"] = 0.1
        return payload


def test_executor_timeout_stops_loop_with_supervised_zero(tmp_path) -> None:
    provider = TimeoutProvider([("move_distance", 0.25)])
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "timeout.sqlite3",
        source_sha="adaptive-mission-timeout",
        allow_loopback_test_approval=True,
    )
    try:
        proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
        adapter.approve(proposed["approval"]["required_phrase"])
        terminal = _wait_terminal(adapter)
    finally:
        adapter.close()

    movement = terminal["mission"]["result"]["intent_revisions"][0]["execution"][
        "movement"
    ]
    assert terminal["mission"]["state"] == "TIMEOUT"
    assert terminal["mission"]["terminal_reason"] == "intent_timeout"
    assert movement["requested"]["linear_mps"] == pytest.approx(0.10)
    assert movement["supervised"]["linear_mps"] == 0.0


def test_mission_lease_expiry_stops_while_provider_is_still_in_flight(
    tmp_path,
) -> None:
    provider = SequenceProvider(
        [("observe", 0.0), ("stop", 0.0)],
        delay_after_first_s=0.4,
    )
    limits = AdaptiveMissionLimits(mission_lease_s=0.08)
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "mission-lease.sqlite3",
        source_sha="adaptive-mission-mission-lease",
        allow_loopback_test_approval=True,
        limits=limits,
        executor_factory=lambda: ReplayAdaptiveMissionExecutor(limits=limits),
    )
    try:
        proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
        started = time.monotonic()
        adapter.approve(proposed["approval"]["required_phrase"])
        terminal = _wait_terminal(adapter)
        elapsed = time.monotonic() - started
    finally:
        adapter.close()

    assert elapsed < 0.25
    assert terminal["mission"]["state"] == "TIMEOUT"
    assert terminal["mission"]["terminal_reason"] == "mission_lease_expired"
    assert terminal["mission"]["result"]["auto_resume"] is False


@pytest.mark.parametrize(
    ("method", "state", "reason"),
    (
        ("stop", "STOPPED", "stop_requested"),
        ("estop", "ESTOPPED", "estop_latched"),
    ),
)
def test_stop_and_estop_end_adaptive_mission_during_provider_call(
    tmp_path, method: str, state: str, reason: str
) -> None:
    provider = SequenceProvider(
        [("observe", 0.0), ("stop", 0.0)],
        delay_after_first_s=0.4,
    )
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / f"{method}.sqlite3",
        source_sha=f"adaptive-mission-{method}",
        allow_loopback_test_approval=True,
    )
    try:
        proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
        adapter.approve(proposed["approval"]["required_phrase"])
        assert provider.entered_delayed_call.wait(timeout=1.0)
        assert adapter._controller is not None
        getattr(adapter._controller, method)()
        terminal = dict(adapter.snapshot())
    finally:
        adapter.close()

    assert terminal["mission"]["state"] == state
    assert terminal["mission"]["terminal_reason"] == reason
    assert terminal["mission"]["result"]["auto_resume"] is False


def test_cancellation_during_provider_call_is_immediate_and_terminal(tmp_path) -> None:
    provider = SequenceProvider(
        [("observe", 0.0), ("stop", 0.0)],
        delay_after_first_s=0.4,
    )
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "cancel.sqlite3",
        source_sha="adaptive-mission-cancel",
        allow_loopback_test_approval=True,
    )
    try:
        proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
        adapter.approve(proposed["approval"]["required_phrase"])
        assert provider.entered_delayed_call.wait(timeout=1.0)
        started = time.monotonic()
        cancelled = adapter.cancel()
        elapsed = time.monotonic() - started
    finally:
        adapter.close()

    assert elapsed < 0.2
    assert cancelled["mission"]["state"] == "CANCELLED"
    assert cancelled["mission"]["terminal_reason"] == "operator_cancelled"
    assert cancelled["mission"]["result"]["auto_resume"] is False


def test_restart_marks_approved_adaptive_mission_recovery_required(tmp_path) -> None:
    database = tmp_path / "restart.sqlite3"
    limits = AdaptiveMissionLimits()
    executor = ReplayAdaptiveMissionExecutor(limits=limits)
    mission_id = "adaptive-mission-restart"
    snapshot = executor.snapshot(mission_id)
    raw = _raw(snapshot, "observe")
    intent = AdaptiveMissionIntent.validated(
        raw,
        revision=1,
        snapshot=snapshot,
        issued_at_s=10.0,
        provider_id="test",
        model_id="test",
        limits=limits,
    )
    proposal = AdaptiveMissionApprovalEnvelope(
        mission_id=mission_id,
        lease_id="restart-lease",
        prompt=PROMPT,
        interpreted_objective=intent.interpreted_objective,
        source_sha="restart-sha",
        deployed_sha="restart-sha",
        provider_id="test",
        model_id="test",
        reasoning_effort="fixture",
        executor_mode=executor.mode,
        starting_snapshot_id=str(snapshot["snapshot_id"]),
        first_intent=raw,
        limits=limits,
    ).proposal()
    first = MissionService(
        database,
        source_sha="restart-sha",
        deployed_sha="restart-sha",
        mode="replay",
        live_execution_enabled=False,
    )
    first.begin_prompt_mission(
        mission_id=mission_id,
        session_id="restart-session",
        prompt=PROMPT,
        source="web",
    )
    first.record_rolling_replay_proposal(mission_id, proposal)
    first.approve_rolling_replay_mission(
        mission_id,
        proposal_digest=proposal["proposal_digest"],
        operator="authenticated-test-operator",
    )
    first.close()

    restarted = MissionService(
        database,
        source_sha="restart-sha",
        deployed_sha="restart-sha",
        mode="replay",
        live_execution_enabled=False,
    )
    try:
        recovered = restarted.prompt_status(mission_id)
    finally:
        restarted.close()

    assert recovered["status"] == "recovery_required"
    assert recovered["recovery_required"] is True
    assert recovered["auto_resume"] is False
    assert "execution is not resumed" in recovered["terminal_reason"]


def test_browser_bundle_exposes_adaptive_mission_objective_lease_and_supervision() -> None:
    from sphero_rvr_driver.mission_web import build_mission_web_bundle

    html = build_mission_web_bundle()["index_html"]

    assert "ADAPTIVE MISSION CLOSED LOOP" in html
    assert "Mission log" in html
    assert "Instruction received" in html
    assert "Objective interpreted" in html
    assert "First intent" in html
    assert "leaseDurationLabel(snapshot)" in html
    assert "$('approve').textContent = 'Approve';" in html
    assert "$('cancel').textContent = 'Cancel';" in html
    assert "requested" in html
    assert "supervised" in html
    assert "Revision ${revision.revision}" in html
    assert "const loopEvents = Array.isArray(rolling.events)" in html
    assert (
        "Disabled: physical execution and fresh supervised safety readiness "
        "are required before approval."
    ) in html
    assert "Fresh world snapshot &amp; detections" not in html


def _post_json(
    url: str, payload: Mapping[str, Any], headers: Mapping[str, str]
) -> dict[str, Any]:
    request = urllib.request.Request(
        url,
        data=json.dumps(dict(payload)).encode(),
        headers=dict(headers),
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=2.0) as response:
        return json.loads(response.read())


def test_adaptive_mission_approval_requires_server_authenticated_identity(tmp_path) -> None:
    provider = SequenceProvider([("stop", 0.0)])
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "secure-approval.sqlite3",
        source_sha="adaptive-mission-secure-approval",
        operator="untrusted-loopback-label",
    )
    proposed = adapter.propose(PROMPT, "adaptive_mission_explore")
    with pytest.raises(MissionWebError, match="server-authenticated Tailscale"):
        adapter.approve(proposed["approval"]["required_phrase"])

    server = make_server(port=0, adapter=adapter)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    spoofed_headers = {
        "Content-Type": "application/json",
        "Tailscale-User-Login": "spoofed@example.com",
    }
    try:
        with pytest.raises(urllib.error.HTTPError) as error:
            _post_json(
                f"{base}/api/web/mission/approve",
                {"approval_phrase": proposed["approval"]["required_phrase"]},
                spoofed_headers,
            )
        assert error.value.code == 400
        assert "server-authenticated Tailscale" in json.loads(
            error.value.read()
        )["error"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
        adapter.close()
    assert not thread.is_alive()


def test_adaptive_mission_tailscale_identity_is_bound_to_single_lease_approval(
    tmp_path,
) -> None:
    provider = SequenceProvider(
        [("move_distance", 0.10), ("observe", 0.0), ("stop", 0.0)]
    )
    adapter = AdaptiveMissionAdapter(
        provider,
        database=tmp_path / "tailscale-approval.sqlite3",
        source_sha="adaptive-mission-auth-source",
        deployed_sha="adaptive-mission-deployed",
        operator="untrusted-fallback",
    )
    allowed_origin = "https://sphero-pi-2.example.ts.net"
    server = make_server(
        port=0,
        adapter=adapter,
        allowed_origin=allowed_origin,
        require_tailscale_identity=True,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    headers = {
        "Content-Type": "application/json",
        "Origin": allowed_origin,
        "Sec-Fetch-Site": "same-origin",
        "Tailscale-User-Login": "scott@example.com",
    }
    try:
        proposed = _post_json(
            f"{base}/api/web/mission/propose",
            {"prompt": PROMPT, "scenario": "adaptive_mission_explore"},
            headers,
        )
        assert proposed["approval"]["authenticated_operator"] == ""
        assert proposed["approval"]["authentication_source"] == "tailscale-serve"
        _post_json(
            f"{base}/api/web/mission/approve",
            {"approval_phrase": proposed["approval"]["required_phrase"]},
            headers,
        )
        terminal = _wait_terminal(adapter)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
        adapter.close()

    assert not thread.is_alive()
    approval = terminal["mission"]["result"]["approval"]
    assert approval["operator"] == "scott@example.com"
    assert approval["authenticated"] is True
    assert approval["authentication_source"] == "tailscale-serve"
    assert approval["proposal_digest"] == proposed["proposal"]["proposal_digest"]
    assert terminal["mission"]["result"]["limits"]["mission_lease_s"] == 900.0
    assert (
        terminal["mission"]["result"]["safety_policy"]
        == proposed["proposal"]["safety_policy"]
    )
    assert provider.calls == 3
