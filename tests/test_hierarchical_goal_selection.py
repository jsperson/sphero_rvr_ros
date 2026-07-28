from __future__ import annotations

from dataclasses import replace
import hashlib
import json
from pathlib import Path
import threading
import time

import pytest

from sphero_rvr_driver.hierarchical_exploration import (
    FrontierCandidate,
    detect_frontiers,
    load_slam_toolbox_map,
)
from sphero_rvr_driver.hierarchical_phase3_replay_validation import (
    evaluate_phase3_fixture,
    main as phase3_replay_main,
)
from sphero_rvr_driver.hierarchical_goal_selection import (
    ALLOWED_ACTIONS,
    AsyncSemanticGoalController,
    CodexOAuthSemanticGoalProvider,
    DeterministicGoalResolver,
    NextBestViewPlan,
    ScriptedSemanticGoalProvider,
    SemanticEventKind,
    SemanticGoalDecision,
    SemanticReplanEvent,
    SemanticTrack,
    build_semantic_world_snapshot,
    generate_next_best_views,
    revalidate_resolved_goal,
    semantic_goal_output_schema,
    semantic_goal_provider_output_schema,
    semantic_goal_prompt,
)
from sphero_rvr_driver.mission_api import MissionValidationError


REPO_ROOT = Path(__file__).resolve().parents[1]
RECORDED_MAP = (
    REPO_ROOT
    / "artifacts"
    / "phase1_recorded_slam_map"
    / "phase1_recorded_slam_map.yaml"
)
PHASE3_FIXTURE = (
    REPO_ROOT
    / "artifacts"
    / "phase3_semantic_goal_replay"
    / "fixture.json"
)
PHASE3_OAUTH_SMOKE = (
    REPO_ROOT
    / "artifacts"
    / "phase3_semantic_goal_replay"
    / "oauth_smoke.json"
)


def _grid():
    return load_slam_toolbox_map(RECORDED_MAP, map_id="rvr-room-20260626")


def _track() -> SemanticTrack:
    return SemanticTrack(
        track_id="shoe-track-01",
        signature="shoe-track-signature-01",
        class_name="shoe",
        x_m=0.30,
        y_m=0.94,
        position_method="floor_projection",
        position_sigma_m=0.15,
        last_seen_s=100.0,
        evidence_ids=("camera-frame-01", "localization-result-01"),
        stable_observations=3,
    )


def _frontiers(count: int = 4) -> tuple[FrontierCandidate, ...]:
    detected = detect_frontiers(
        _grid(),
        robot_x_m=-0.028,
        robot_y_m=0.941,
    )
    return tuple(
        replace(item, path_distance_m=2.5)
        for item in detected[2 : 2 + count]
    )


def _nbv(track: SemanticTrack | None = None) -> NextBestViewPlan:
    track = track or _track()
    plan = generate_next_best_views(
        _grid(),
        track,
        robot_x_m=-0.028,
        robot_y_m=0.941,
    )
    assert plan.candidates
    return replace(
        plan,
        candidates=tuple(
            replace(item, route_length_m=2.5)
            for item in plan.candidates
        ),
    )


def _snapshot(
    *,
    event_generation: int = 0,
    objective_revision: int = 0,
    frontiers: tuple[FrontierCandidate, ...] | None = None,
    track: SemanticTrack | None = None,
) -> dict:
    selected_track = track or _track()
    return build_semantic_world_snapshot(
        mission_id="phase3-canonical",
        objective=(
            "Explore this room, map shoes, inspect uncertain findings, "
            "then return or stop safely."
        ),
        objective_revision=objective_revision,
        event_generation=event_generation,
        requested_object_classes=("shoe", "person"),
        map_id="rvr-room-20260626",
        map_revision=_grid().revision,
        robot_x_m=-0.028,
        robot_y_m=0.941,
        robot_yaw_rad=0.0,
        localization_timestamp_s=100.0,
        now_s=100.1,
        frontiers=frontiers or _frontiers(),
        tracks=(selected_track,),
        next_best_views=(_nbv(selected_track),),
        origin_x_m=2.472,
        origin_y_m=0.941,
        coverage_fraction=0.42,
    )


def _decision(
    snapshot: dict,
    action: str,
    arguments: dict,
    *,
    generation: int | None = None,
    rationale: str | None = None,
) -> dict:
    if rationale is None:
        referenced = (
            next(iter(arguments.values()))
            if arguments
            else action
        )
        if isinstance(referenced, list):
            referenced = referenced[0]
        rationale = f"Select {referenced} from bounded snapshot evidence."
    return {
        "schema": "sphero_rvr.semantic_goal.v1",
        "mission_id": snapshot["mission_id"],
        "snapshot_id": snapshot["snapshot_id"],
        "decision_generation": int(
            generation
            if generation is not None
            else snapshot.get("decision_generation", 1)
        ),
        "event_generation": snapshot["event_generation"],
        "action": action,
        "arguments": arguments,
        "rationale": rationale,
    }


def _frontier_decision(index: int):
    def choose(snapshot: dict) -> dict:
        target = snapshot["frontiers"][index]["signature"]
        return _decision(
            snapshot,
            "go_to_frontier",
            {"frontier_id": target},
        )

    return choose


def _inspect_decision(snapshot: dict) -> dict:
    track_id = snapshot["tracks"][0]["track_id"]
    return _decision(snapshot, "inspect", {"track_id": track_id})


def _return_decision(snapshot: dict) -> dict:
    return _decision(
        snapshot,
        "return_to_start",
        {},
        rationale="Return to the server-owned mission origin.",
    )


def _wait_for_provider(
    provider: ScriptedSemanticGoalProvider,
    calls: int,
    *,
    completed: bool = True,
) -> None:
    deadline = time.monotonic() + 1.0
    while (
        (
            provider.completed_calls if completed else provider.calls
        )
        < calls
        and time.monotonic() < deadline
    ):
        time.sleep(0.001)
    observed = provider.completed_calls if completed else provider.calls
    assert observed >= calls
    if completed:
        # ``completed_calls`` increments immediately before ``choose`` returns.
        # Yield once so ConcurrentFuture can publish its done/result state.
        time.sleep(0.001)


def test_structured_schema_exposes_only_semantic_ids_and_actions() -> None:
    schema = semantic_goal_output_schema()
    rendered = json.dumps(schema, sort_keys=True)

    assert set(schema["properties"]["action"]["enum"]) == ALLOWED_ACTIONS
    assert '"x_m"' not in rendered
    assert '"y_m"' not in rendered
    assert '"route"' not in rendered
    assert '"speed"' not in rendered
    assert '"velocity"' not in rendered
    assert schema["additionalProperties"] is False
    assert all(
        branch["additionalProperties"] is False
        for branch in schema["properties"]["arguments"]["anyOf"]
    )


def test_real_oauth_adapter_uses_toolless_semantic_output_schema() -> None:
    snapshot = _snapshot()
    expected = _frontier_decision(1)(snapshot)

    class FakeClient:
        def __init__(self) -> None:
            self.call = None
            self.cancelled = False
            self.closed = False

        def run_turn(self, **kwargs):
            self.call = kwargs
            return json.dumps(expected), 3.5, 0

        def cancel(self) -> None:
            self.cancelled = True

        def close(self) -> None:
            self.closed = True

    client = FakeClient()
    provider = CodexOAuthSemanticGoalProvider(
        model="gpt-test",
        reasoning_effort="low",
        client=client,
    )
    result = provider.choose(
        semantic_goal_prompt(snapshot["objective"], snapshot),
        snapshot,
    )
    provider.close()

    assert result == expected
    assert client.call["model"] == "gpt-test"
    assert client.call["image_path"] is None
    assert client.call["output_schema"] == semantic_goal_provider_output_schema(
        snapshot
    )
    assert client.call["cwd"] != str(REPO_ROOT)
    assert provider.latency_history()[0]["success"] is True
    assert client.closed is True


def test_world_snapshot_is_bounded_digest_bound_and_has_no_authority() -> None:
    snapshot = _snapshot()
    prompt = json.loads(semantic_goal_prompt(snapshot["objective"], snapshot))
    digest_input = dict(snapshot)
    snapshot_id = digest_input.pop("snapshot_id")
    expected_digest = hashlib.sha256(
        json.dumps(
            digest_input,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()

    assert snapshot["schema"] == "sphero_rvr.semantic_world_snapshot.v1"
    assert len(snapshot["snapshot_id"]) == 64
    assert snapshot_id == expected_digest
    assert len(snapshot["frontiers"]) <= 16
    assert len(snapshot["tracks"]) <= 16
    assert snapshot["authority"] == {
        "motion_authority": False,
        "physical_execution_enabled": False,
        "live_sensors": False,
        "serial_access": False,
    }
    assert prompt["world_snapshot"]["snapshot_id"] == snapshot["snapshot_id"]
    assert all(
        "approach_pose" not in frontier
        for frontier in prompt["world_snapshot"]["frontiers"]
    )
    assert all(
        field not in candidate
        for candidates in prompt["world_snapshot"]["next_best_views"].values()
        for candidate in candidates
        for field in ("x_m", "y_m", "yaw_rad")
    )
    assert all(
        text not in prompt["rules"][-2]
        for text in ("/cmd_vel", "/cmd_vel_motor")
    )


def test_live_wfd_frontier_oversubscription_is_deterministically_bounded() -> None:
    base = _frontiers()
    oversubscribed = tuple(
        replace(
            base[index % len(base)],
            signature=f"live-frontier-{index:02d}",
        )
        for index in range(19)
    )

    snapshot = _snapshot(frontiers=oversubscribed)

    assert len(snapshot["frontiers"]) == 16
    assert [
        frontier["signature"] for frontier in snapshot["frontiers"]
    ] == [frontier.signature for frontier in oversubscribed[:16]]


def test_next_best_view_is_stable_reachable_and_records_rejections() -> None:
    first = _nbv()
    second = _nbv()

    assert [item.viewpoint_id for item in first.candidates] == [
        item.viewpoint_id for item in second.candidates
    ]
    assert first.rejected
    assert all(item.clearance_m >= 0.15 for item in first.candidates)
    assert all(item.route_length_m >= 0.0 for item in first.candidates)
    assert all(item.expected_uncertainty_reduction > 0.0 for item in first.candidates)
    assert {item.reason for item in first.rejected} <= {
        "out_of_map",
        "occupied_or_unknown",
        "unreachable",
        "insufficient_clearance",
        "occluded",
    }


@pytest.mark.parametrize(
    ("action", "arguments", "rationale"),
    [
        (
            "go_to_frontier",
            lambda snap: {"frontier_id": snap["frontiers"][0]["signature"]},
            lambda snap: f"Use {snap['frontiers'][0]['signature']}.",
        ),
        (
            "inspect",
            lambda snap: {"track_id": snap["tracks"][0]["track_id"]},
            lambda snap: f"Inspect {snap['tracks'][0]['track_id']}.",
        ),
        (
            "search_region",
            lambda snap: {
                "region_id": "recorded_room",
                "target_classes": ["shoe"],
            },
            lambda snap: "Search recorded_room for shoe evidence.",
        ),
        ("return_to_start", lambda snap: {}, lambda snap: "Return to origin."),
        ("wait", lambda snap: {}, lambda snap: "Wait for fresh evidence."),
        (
            "finish",
            lambda snap: {
                "outcome": "partial",
                "evidence_ids": [snap["evidence_ids"][0]],
            },
            lambda snap: f"Finish with {snap['evidence_ids'][0]}.",
        ),
    ],
)
def test_all_goal_actions_validate_without_model_geometry(
    action: str,
    arguments,
    rationale,
) -> None:
    snapshot = _snapshot()
    raw = _decision(
        snapshot,
        action,
        arguments(snapshot),
        rationale=rationale(snapshot),
    )

    validated = SemanticGoalDecision.validated(
        raw,
        snapshot=snapshot,
        expected_generation=1,
        provider_id="test-provider",
        model_id="test-model",
    )

    assert validated.action == action
    assert validated.arguments == arguments(snapshot)


def test_goal_response_rejects_geometry_extra_fields_and_stale_bindings() -> None:
    snapshot = _snapshot()
    target = snapshot["frontiers"][0]["signature"]
    raw = _decision(
        snapshot,
        "go_to_frontier",
        {"frontier_id": target},
    )

    geometry = dict(raw)
    geometry["arguments"] = {"frontier_id": target, "x_m": 1.0}
    with pytest.raises(MissionValidationError, match="arguments are invalid"):
        SemanticGoalDecision.validated(
            geometry,
            snapshot=snapshot,
            expected_generation=1,
            provider_id="p",
            model_id="m",
        )

    stale = dict(raw)
    stale["snapshot_id"] = "stale-snapshot"
    with pytest.raises(MissionValidationError, match="snapshot binding"):
        SemanticGoalDecision.validated(
            stale,
            snapshot=snapshot,
            expected_generation=1,
            provider_id="p",
            model_id="m",
        )

    duplicate_classes = _decision(
        snapshot,
        "search_region",
        {
            "region_id": snapshot["frontiers"][0]["region_id"],
            "target_classes": ["shoe", "shoe"],
        },
    )
    with pytest.raises(MissionValidationError, match="target classes"):
        SemanticGoalDecision.validated(
            duplicate_classes,
            snapshot=snapshot,
            expected_generation=1,
            provider_id="p",
            model_id="m",
        )

    unapproved_class = _decision(
        snapshot,
        "search_region",
        {
            "region_id": snapshot["frontiers"][0]["region_id"],
            "target_classes": ["hazard"],
        },
    )
    with pytest.raises(MissionValidationError, match="approved objective"):
        SemanticGoalDecision.validated(
            unapproved_class,
            snapshot=snapshot,
            expected_generation=1,
            provider_id="p",
            model_id="m",
        )


def test_resolver_uses_server_geometry_and_revalidation_rejects_invalidation() -> None:
    snapshot = _snapshot()
    target = snapshot["frontiers"][0]["signature"]
    decision = SemanticGoalDecision.validated(
        _decision(snapshot, "go_to_frontier", {"frontier_id": target}),
        snapshot=snapshot,
        expected_generation=1,
        provider_id="p",
        model_id="m",
    )
    goal = DeterministicGoalResolver().resolve(
        decision, snapshot, ready_at_s=12.691
    )
    revision_only = json.loads(json.dumps(snapshot))
    revision_only["map"]["map_revision"] = "new-revision-same-signature"

    assert goal.x_m == snapshot["frontiers"][0]["approach_pose"]["x_m"]
    assert goal.y_m == snapshot["frontiers"][0]["approach_pose"]["y_m"]
    assert revalidate_resolved_goal(
        goal,
        captured_snapshot=snapshot,
        current_snapshot=revision_only,
    ).accepted is True

    invalidated = json.loads(json.dumps(snapshot))
    invalidated["frontiers"] = invalidated["frontiers"][1:]
    result = revalidate_resolved_goal(
        goal,
        captured_snapshot=snapshot,
        current_snapshot=invalidated,
    )
    assert result.accepted is False
    assert result.reasons == ("frontier_signature_invalidated",)

    stale_motion = json.loads(json.dumps(snapshot))
    stale_motion["safety"]["motion_evidence_fresh"] = False
    result = revalidate_resolved_goal(
        goal,
        captured_snapshot=snapshot,
        current_snapshot=stale_motion,
    )
    assert result.accepted is False
    assert "motion_evidence_stale" in result.reasons


def test_long_leg_async_prefetch_hands_off_without_zero_under_recorded_p95() -> None:
    snapshot = _snapshot()
    first_target = snapshot["frontiers"][0]["signature"]
    provider = ScriptedSemanticGoalProvider([_frontier_decision(1)])
    controller = AsyncSemanticGoalController(
        provider, modeled_provider_latency_s=12.691
    )
    try:
        controller.start(
            _decision(
                snapshot,
                "go_to_frontier",
                {"frontier_id": first_target},
            ),
            snapshot,
        )
        started = controller.tick(
            snapshot,
            now_s=11.309,
            remaining_distance_m=1.369,
            eta_s=13.691,
        )
        _wait_for_provider(provider, 1)
        captured = provider.captured_snapshots[0]
        captured_digest_input = dict(captured)
        captured_id = captured_digest_input.pop("snapshot_id")
        assert captured_id == hashlib.sha256(
            json.dumps(
                captured_digest_input,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        ).hexdigest()
        assert captured["decision_generation"] == 2
        assert captured["active_goal"] == {
            "generation": 1,
            "action": "go_to_frontier",
            "target_id": first_target,
            "remaining_distance_m": 1.369,
            "eta_s": 13.691,
            "follower_state": "navigating",
            "prefetch_state": "requested",
        }
        ready = controller.tick(
            snapshot,
            now_s=24.01,
            remaining_distance_m=0.10,
            eta_s=1.0,
        )
        handed_off = controller.tick(
            snapshot,
            now_s=25.0,
            remaining_distance_m=0.0,
            eta_s=30.0,
        )

        assert started.provider_in_flight is True
        assert ready.prefetched_generation == 2
        assert ready.handoff.queued_generations == (2,)
        assert handed_off.handoff.state == "navigating"
        assert handed_off.handoff.controller_session == 1
        assert handed_off.handoff.command.zero_required is False
        assert handed_off.handoff.events[-1]["kind"] == "atomic_handoff"
        assert handed_off.handoff.events[-1]["deliberate_zero"] is False
    finally:
        controller.close()


def test_short_hop_under_recorded_p95_honestly_waits_then_resumes() -> None:
    frontiers = tuple(
        replace(item, path_distance_m=0.50) for item in _frontiers(2)
    )
    snapshot = _snapshot(frontiers=frontiers)
    provider = ScriptedSemanticGoalProvider([_frontier_decision(1)])
    controller = AsyncSemanticGoalController(
        provider, modeled_provider_latency_s=12.691
    )
    try:
        controller.start(
            _decision(
                snapshot,
                "go_to_frontier",
                {"frontier_id": snapshot["frontiers"][0]["signature"]},
            ),
            snapshot,
        )
        controller.tick(
            snapshot,
            now_s=0.0,
            remaining_distance_m=0.50,
            eta_s=5.0,
        )
        _wait_for_provider(provider, 1)
        waiting = controller.tick(
            snapshot,
            now_s=5.0,
            remaining_distance_m=0.0,
            eta_s=0.0,
        )
        resumed = controller.tick(
            snapshot,
            now_s=12.70,
            remaining_distance_m=0.50,
            eta_s=30.0,
        )

        assert waiting.handoff.state == "wait_planning"
        assert waiting.handoff.command.zero_required is True
        assert waiting.handoff.command.reason == "wait_planning"
        assert resumed.handoff.state == "navigating"
        assert resumed.handoff.controller_session == 2
        assert resumed.handoff.events[-1]["kind"] == "planning_resume"
    finally:
        controller.close()


def test_stable_new_detection_preempts_and_invalidates_inflight_snapshot() -> None:
    snapshot0 = _snapshot(event_generation=0)
    snapshot1 = _snapshot(event_generation=1)
    release = threading.Event()
    provider = ScriptedSemanticGoalProvider(
        [_frontier_decision(1), _inspect_decision],
        release_event=release,
    )
    controller = AsyncSemanticGoalController(
        provider, modeled_provider_latency_s=12.691
    )
    try:
        controller.start(
            _decision(
                snapshot0,
                "go_to_frontier",
                {"frontier_id": snapshot0["frontiers"][0]["signature"]},
            ),
            snapshot0,
        )
        controller.tick(
            snapshot0,
            now_s=0.0,
            remaining_distance_m=2.5,
            eta_s=10.0,
        )
        _wait_for_provider(provider, 1, completed=False)
        unstable = controller.handle_event(
            SemanticReplanEvent(
                "detection-unstable",
                SemanticEventKind.NEW_DETECTION,
                observed_at_s=0.1,
                target_id="shoe-track-01",
                confidence=0.9,
                stable_observations=1,
            ),
            snapshot0,
            now_s=0.1,
        )
        preempted = controller.handle_event(
            SemanticReplanEvent(
                "detection-stable",
                SemanticEventKind.NEW_DETECTION,
                observed_at_s=2.2,
                target_id="shoe-track-01",
                confidence=0.9,
                stable_observations=3,
            ),
            snapshot1,
            now_s=2.2,
        )
        release.set()
        _wait_for_provider(provider, 1)
        discarded = controller.tick(
            snapshot1,
            now_s=12.70,
            remaining_distance_m=2.4,
            eta_s=0.0,
        )
        _wait_for_provider(provider, 2)
        resumed = controller.tick(
            snapshot1,
            now_s=25.40,
            remaining_distance_m=2.5,
            eta_s=30.0,
        )

        assert unstable.events[-1]["kind"] == "semantic_event_coalesced"
        assert preempted.handoff.state == "wait_planning"
        assert preempted.handoff.command.reason == "semantic_replan"
        assert preempted.events[-1]["kind"] == "event_triggered_replan"
        assert any(
            event["kind"] == "prefetch_discarded"
            and "event_invalidated:new_detection" in event["reason"]
            for event in discarded.events
        )
        assert resumed.handoff.state == "navigating"
        assert resumed.handoff.events[-1]["kind"] == "planning_resume"
    finally:
        release.set()
        controller.close()


def test_supervisor_veto_is_immediate_while_provider_is_blocked() -> None:
    snapshot = _snapshot()
    release = threading.Event()
    provider = ScriptedSemanticGoalProvider(
        [_frontier_decision(1)],
        release_event=release,
    )
    controller = AsyncSemanticGoalController(provider)
    try:
        controller.start(
            _decision(
                snapshot,
                "go_to_frontier",
                {"frontier_id": snapshot["frontiers"][0]["signature"]},
            ),
            snapshot,
        )
        controller.tick(
            snapshot,
            now_s=0.0,
            remaining_distance_m=2.5,
            eta_s=10.0,
        )
        _wait_for_provider(provider, 1, completed=False)
        vetoed = controller.tick(
            snapshot,
            now_s=0.01,
            remaining_distance_m=2.49,
            eta_s=9.9,
            collision_state="BLOCKED",
        )

        assert vetoed.provider_in_flight is True
        assert vetoed.handoff.state == "terminal_safety"
        assert vetoed.handoff.command.zero_required is True
        assert vetoed.handoff.command.reason == "collision_veto"
        assert vetoed.events[-1]["kind"] == "safety_veto_during_provider"
    finally:
        release.set()
        controller.close()


def test_real_provider_result_is_not_artificially_delayed_to_p95() -> None:
    snapshot = _snapshot()
    provider = ScriptedSemanticGoalProvider([_frontier_decision(1)])
    controller = AsyncSemanticGoalController(provider)
    try:
        controller.start(
            _decision(
                snapshot,
                "go_to_frontier",
                {"frontier_id": snapshot["frontiers"][0]["signature"]},
            ),
            snapshot,
        )
        controller.tick(
            snapshot,
            now_s=0.0,
            remaining_distance_m=2.5,
            eta_s=10.0,
        )
        _wait_for_provider(provider, 1)
        collected = controller.tick(
            snapshot,
            now_s=0.01,
            remaining_distance_m=2.49,
            eta_s=9.9,
        )

        assert collected.prefetched_generation == 2
        assert any(
            event["kind"] == "prefetch_revalidated"
            for event in collected.events
        )
    finally:
        controller.close()


def test_invalidated_frontier_event_preempts_and_replans_from_fresh_snapshot() -> None:
    original = _frontiers(4)
    snapshot0 = _snapshot(event_generation=0, frontiers=original)
    snapshot1 = _snapshot(event_generation=1, frontiers=original[1:])
    provider = ScriptedSemanticGoalProvider([_inspect_decision])
    controller = AsyncSemanticGoalController(
        provider, modeled_provider_latency_s=12.691
    )
    try:
        controller.start(
            _decision(
                snapshot0,
                "go_to_frontier",
                {"frontier_id": snapshot0["frontiers"][0]["signature"]},
            ),
            snapshot0,
        )
        preempted = controller.handle_event(
            SemanticReplanEvent(
                "frontier-invalidated",
                SemanticEventKind.INVALID_TARGET,
                observed_at_s=1.0,
                target_id=snapshot0["frontiers"][0]["signature"],
            ),
            snapshot1,
            now_s=1.0,
        )
        _wait_for_provider(provider, 1)
        resumed = controller.tick(
            snapshot1,
            now_s=13.70,
            remaining_distance_m=2.5,
            eta_s=30.0,
        )

        assert preempted.handoff.state == "wait_planning"
        assert preempted.handoff.command.reason == "semantic_replan"
        assert any(
            event.get("event_kind") == "invalid_target"
            for event in preempted.events
        )
        assert preempted.events[-1]["kind"] == "prefetch_started"
        assert resumed.handoff.state == "navigating"
        assert resumed.handoff.events[-1]["kind"] == "planning_resume"
    finally:
        controller.close()


def test_canonical_long_leg_replay_reduces_model_decisions_tenfold() -> None:
    snapshot = _snapshot()
    provider = ScriptedSemanticGoalProvider(
        [_frontier_decision(1), _inspect_decision, _return_decision]
    )
    controller = AsyncSemanticGoalController(
        provider, modeled_provider_latency_s=12.691
    )
    try:
        controller.start(
            _decision(
                snapshot,
                "go_to_frontier",
                {"frontier_id": snapshot["frontiers"][0]["signature"]},
            ),
            snapshot,
        )
        for leg in range(3):
            start = 25.0 * leg
            controller.tick(
                snapshot,
                now_s=start + 11.309,
                remaining_distance_m=1.369,
                eta_s=13.691,
            )
            _wait_for_provider(provider, leg + 1)
            controller.tick(
                snapshot,
                now_s=start + 24.01,
                remaining_distance_m=0.10,
                eta_s=1.0,
            )
            handoff = controller.tick(
                snapshot,
                now_s=start + 25.0,
                remaining_distance_m=0.0,
                eta_s=30.0,
            )
            assert handoff.handoff.command.zero_required is False
            assert handoff.handoff.events[-1]["kind"] == "atomic_handoff"
        completed = controller.tick(
            snapshot,
            now_s=100.0,
            remaining_distance_m=0.0,
            eta_s=30.0,
        )
        evidence = controller.evidence()

        assert completed.handoff.state == "wait_planning"
        assert evidence["distance_m"] == pytest.approx(10.0)
        assert evidence["motion_goal_decisions"] == 4
        assert evidence["decision_reduction_ratio"] == pytest.approx(10.0)
        assert evidence["handoff"]["controller_session"] == 1
        assert sum(
            event["kind"] == "atomic_handoff"
            for event in evidence["handoff"]["trace"]
        ) == 3
        assert evidence["authority"]["motion_authority"] is False
        assert evidence["carryovers"] == {
            "phase2_accuracy_is_physical_certification": False,
            "pi_no_motion_wfd_before_physical": True,
            "pi_command_ownership_before_physical": True,
            "dropoff_sensing_available": False,
        }
    finally:
        controller.close()


def test_committed_phase3_replay_fixture_passes_every_acceptance_gate() -> None:
    fixture = json.loads(PHASE3_FIXTURE.read_text())
    report = evaluate_phase3_fixture(fixture, root=REPO_ROOT)

    assert report["passed"] is True
    assert all(report["checks"].values())
    assert report["metrics"]["provider_p95_s"] == pytest.approx(12.691)
    assert report["metrics"]["prefetch_threshold_s"] == pytest.approx(13.691)
    assert report["metrics"]["canonical_distance_m"] == pytest.approx(10.0)
    assert report["metrics"]["motion_goal_decisions"] == 4
    assert report["metrics"]["decision_reduction_ratio"] == pytest.approx(10.0)
    assert report["metrics"]["atomic_handoffs"] == 3
    assert report["short_hop"]["arrival_state"] == "wait_planning"
    assert report["safety_veto"]["provider_in_flight"] is True


def test_committed_oauth_smoke_is_semantic_only_and_snapshot_bound() -> None:
    evidence = json.loads(PHASE3_OAUTH_SMOKE.read_text())
    snapshot = _snapshot()
    # The historical OAuth decision records its exact captured snapshot ID,
    # while reconstructing one NBV clearance through libm differs by one ULP
    # between Darwin/arm64 and Linux/aarch64. Validate the committed binding
    # itself rather than pretending that a cross-platform reconstruction is
    # byte-identical to the captured evidence.
    assert evidence["decision"]["snapshot_id"] == (
        "9cf648fb7272c93951ce388126a4cb834fcef435cae822004588ca9aaff548a2"
    )
    snapshot["snapshot_id"] = evidence["decision"]["snapshot_id"]
    decision = SemanticGoalDecision.validated(
        evidence["decision"],
        snapshot=snapshot,
        expected_generation=1,
        provider_id=evidence["provider"]["provider_id"],
        model_id=evidence["provider"]["model_id"],
    )
    stale_snapshot = json.loads(json.dumps(snapshot))
    stale_snapshot["snapshot_id"] = "0" * 64
    with pytest.raises(MissionValidationError, match="snapshot binding"):
        SemanticGoalDecision.validated(
            evidence["decision"],
            snapshot=stale_snapshot,
            expected_generation=1,
            provider_id=evidence["provider"]["provider_id"],
            model_id=evidence["provider"]["model_id"],
        )
    rendered = json.dumps(evidence["decision"], sort_keys=True)

    assert decision.action == "inspect"
    assert evidence["provider"]["tools_enabled"] is False
    assert evidence["provider"]["success"] is True
    assert evidence["interpretation"]["statistical_latency_profile_updated"] is False
    assert evidence["authority"]["motion_authority"] is False
    assert all(
        token not in rendered
        for token in ('"x_m"', '"y_m"', '"route"', '"speed"', '"velocity"', '"ros"')
    )


def test_phase3_docs_artifacts_and_console_command_are_packaged() -> None:
    setup_text = (REPO_ROOT / "setup.py").read_text()

    assert '"docs/hierarchical_exploration_phase3.md"' in setup_text
    assert '"artifacts/phase3_semantic_goal_replay/fixture.json"' in setup_text
    assert '"artifacts/phase3_semantic_goal_replay/oauth_smoke.json"' in setup_text
    assert (
        "rvr_hierarchical_phase3_replay_validate = "
        "sphero_rvr_driver.hierarchical_phase3_replay_validation:main"
    ) in setup_text


def test_phase3_replay_cli_writes_structured_evidence(tmp_path: Path) -> None:
    output = tmp_path / "phase3_evaluation.json"

    assert phase3_replay_main([str(PHASE3_FIXTURE), "--output", str(output)]) == 0
    report = json.loads(output.read_text())
    assert report["passed"] is True
    assert report["scope"]["physical_execution"] is False
