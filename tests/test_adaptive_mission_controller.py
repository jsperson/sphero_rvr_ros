from __future__ import annotations

import json
import threading
import time
from typing import Any, Mapping
import urllib.error
import urllib.request

import pytest

from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_service import MissionService
from sphero_rvr_driver.mission_web import (
    MissionWebError,
    AdaptiveMissionAdapter,
    make_server,
)
from sphero_rvr_driver.adaptive_mission_controller import (
    AdaptiveMissionController,
    ReplayCollisionSupervisor,
    ReplayAdaptiveMissionExecutor,
    AdaptiveMissionApprovalEnvelope,
    AdaptiveMissionIntent,
    AdaptiveMissionLimits,
    MovementDecision,
    _adaptive_mission_provider_prompt,
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

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        assert prompt
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


def test_collision_supervisor_vetoes_llm_and_records_zero_supervised_motion(
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
    assert terminal["mission"]["state"] == "BLOCKED"
    assert terminal["mission"]["terminal_reason"] == "collision_veto"
    assert provider.calls == 1
    assert movement["requested"]["linear_mps"] == pytest.approx(0.10)
    assert movement["supervised"]["linear_mps"] == 0.0
    assert movement["motor_topic_publisher"] == "lidar_collision_stop_supervisor"


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
        enable_exploration_recovery=True,
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
    assert "outcome_replan" in event_types


def test_llm_problem_solving_reverse_is_capped_at_fifteen_centimeters() -> None:
    provider = SequenceProvider(
        [("move_distance", 0.20), ("move_distance", -0.20)]
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
        enable_exploration_recovery=True,
    )
    try:
        controller.start()
        terminal = _wait_controller_terminal(controller)
    finally:
        controller.close()

    assert terminal["status"] == "failed"
    assert "problem-solving reverse exceeds 0.15 m" in (
        terminal["terminal_reason"]
    )
    assert len(terminal["result"]["intent_revisions"]) == 1


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
