"""Executable Phase 3 replay evidence over the real Phase 1 map fixture."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Optional, Sequence

from .hierarchical_exploration import detect_frontiers, load_slam_toolbox_map
from .hierarchical_goal_selection import (
    AsyncSemanticGoalController,
    NextBestViewPlan,
    ScriptedSemanticGoalProvider,
    SemanticEventKind,
    SemanticReplanEvent,
    SemanticTrack,
    build_semantic_world_snapshot,
    generate_next_best_views,
    semantic_goal_output_schema,
)
from .mission_api import MissionValidationError


def _decision(
    snapshot: Mapping[str, Any],
    action: str,
    arguments: Mapping[str, Any],
    rationale: str,
) -> dict[str, Any]:
    return {
        "schema": "sphero_rvr.semantic_goal.v1",
        "mission_id": snapshot["mission_id"],
        "snapshot_id": snapshot["snapshot_id"],
        "decision_generation": int(snapshot.get("decision_generation", 1)),
        "event_generation": int(snapshot["event_generation"]),
        "action": action,
        "arguments": dict(arguments),
        "rationale": rationale,
    }


def _frontier_choice(index: int):
    def choose(snapshot: Mapping[str, Any]) -> Mapping[str, Any]:
        target = str(snapshot["frontiers"][index]["signature"])
        return _decision(
            snapshot,
            "go_to_frontier",
            {"frontier_id": target},
            f"Select recorded frontier {target}.",
        )

    return choose


def _inspect_choice(snapshot: Mapping[str, Any]) -> Mapping[str, Any]:
    track_id = str(snapshot["tracks"][0]["track_id"])
    return _decision(
        snapshot,
        "inspect",
        {"track_id": track_id},
        f"Inspect stable uncertain track {track_id}.",
    )


def _return_choice(snapshot: Mapping[str, Any]) -> Mapping[str, Any]:
    return _decision(
        snapshot,
        "return_to_start",
        {},
        "Return to the server-owned mission origin.",
    )


def _wait_for_provider(
    provider: ScriptedSemanticGoalProvider,
    count: int,
    *,
    completed: bool = True,
) -> None:
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        observed = provider.completed_calls if completed else provider.calls
        if observed >= count:
            if completed:
                # The provider counter advances immediately before the worker
                # returns; allow its Future to publish the result deterministically.
                time.sleep(0.001)
            return
        time.sleep(0.001)
    raise RuntimeError("scripted semantic provider did not reach expected state")


def _snapshot_components(
    fixture: Mapping[str, Any], root: Path
) -> tuple[Any, tuple[Any, ...], SemanticTrack, NextBestViewPlan]:
    grid = load_slam_toolbox_map(
        root / str(fixture["recorded_map"]),
        map_id=str(fixture["recorded_map_id"]),
    )
    robot = fixture["robot_pose"]
    detected = detect_frontiers(
        grid,
        robot_x_m=float(robot["x_m"]),
        robot_y_m=float(robot["y_m"]),
    )
    indexes = fixture["canonical_replay"]["frontier_indexes"]
    route_length = float(
        fixture["canonical_replay"]["route_length_m_per_goal"]
    )
    frontiers = tuple(
        replace(detected[int(index)], path_distance_m=route_length)
        for index in indexes
    )
    track_value = fixture["semantic_track"]
    track = SemanticTrack(
        track_id=str(track_value["track_id"]),
        signature=str(track_value["signature"]),
        class_name=str(track_value["class_name"]),
        x_m=float(track_value["x_m"]),
        y_m=float(track_value["y_m"]),
        position_method=str(track_value["position_method"]),
        position_sigma_m=float(track_value["position_sigma_m"]),
        last_seen_s=100.0,
        evidence_ids=tuple(str(item) for item in track_value["evidence_ids"]),
        stable_observations=3,
    )
    generated = generate_next_best_views(
        grid,
        track,
        robot_x_m=float(robot["x_m"]),
        robot_y_m=float(robot["y_m"]),
    )
    nbv = replace(
        generated,
        candidates=tuple(
            replace(item, route_length_m=route_length)
            for item in generated.candidates
        ),
    )
    return grid, frontiers, track, nbv


def _snapshot(
    fixture: Mapping[str, Any],
    grid: Any,
    frontiers: tuple[Any, ...],
    track: SemanticTrack,
    nbv: NextBestViewPlan,
    *,
    event_generation: int = 0,
) -> dict[str, Any]:
    robot = fixture["robot_pose"]
    origin = fixture["mission_origin"]
    return build_semantic_world_snapshot(
        mission_id="phase3-canonical-replay",
        objective=str(fixture["canonical_replay"]["objective"]),
        objective_revision=0,
        event_generation=event_generation,
        requested_object_classes=tuple(
            str(item)
            for item in fixture["canonical_replay"][
                "requested_object_classes"
            ]
        ),
        map_id=grid.map_id,
        map_revision=grid.revision,
        robot_x_m=float(robot["x_m"]),
        robot_y_m=float(robot["y_m"]),
        robot_yaw_rad=float(robot["yaw_rad"]),
        localization_timestamp_s=100.0,
        now_s=100.1,
        frontiers=frontiers,
        tracks=(track,),
        next_best_views=(nbv,),
        origin_x_m=float(origin["x_m"]),
        origin_y_m=float(origin["y_m"]),
        coverage_fraction=0.42,
    )


def _canonical_evidence(
    fixture: Mapping[str, Any], snapshot: Mapping[str, Any]
) -> dict[str, Any]:
    latency = fixture["provider_latency_profile"]
    provider = ScriptedSemanticGoalProvider(
        [_frontier_choice(1), _inspect_choice, _return_choice]
    )
    controller = AsyncSemanticGoalController(
        provider,
        provider_p95_s=float(latency["p95_s"]),
        prefetch_margin_s=float(latency["prefetch_margin_s"]),
        modeled_provider_latency_s=float(latency["p95_s"]),
    )
    try:
        first = str(snapshot["frontiers"][0]["signature"])
        controller.start(
            _decision(
                snapshot,
                "go_to_frontier",
                {"frontier_id": first},
                f"Start with recorded frontier {first}.",
            ),
            snapshot,
        )
        duration = float(
            fixture["canonical_replay"]["leg_duration_s_at_0_10_mps"]
        )
        route_length = float(
            fixture["canonical_replay"]["route_length_m_per_goal"]
        )
        threshold = controller.prefetch_threshold_s
        dispatch_offset = duration - threshold
        for leg in range(3):
            start = duration * leg
            controller.tick(
                snapshot,
                now_s=start + dispatch_offset,
                remaining_distance_m=threshold * 0.10,
                eta_s=threshold,
            )
            _wait_for_provider(provider, leg + 1)
            controller.tick(
                snapshot,
                now_s=start + duration - 0.99,
                remaining_distance_m=0.10,
                eta_s=1.0,
            )
            handoff = controller.tick(
                snapshot,
                now_s=start + duration,
                remaining_distance_m=0.0,
                eta_s=duration + 1.0,
            )
            if (
                handoff.handoff.command.zero_required
                or not handoff.handoff.events
                or handoff.handoff.events[-1]["kind"] != "atomic_handoff"
            ):
                raise RuntimeError("canonical long-leg replay did not hand off")
        controller.tick(
            snapshot,
            now_s=4.0 * duration,
            remaining_distance_m=0.0,
            eta_s=duration + 1.0,
        )
        evidence = controller.evidence()
        evidence["route_length_m_per_goal"] = route_length
        return evidence
    finally:
        controller.close()


def _short_hop_evidence(
    fixture: Mapping[str, Any],
    snapshot: Mapping[str, Any],
) -> dict[str, Any]:
    latency = fixture["provider_latency_profile"]
    route = float(fixture["short_hop_replay"]["route_length_m"])
    short = json.loads(json.dumps(dict(snapshot)))
    for frontier in short["frontiers"]:
        frontier["path_distance_m"] = route
    provider = ScriptedSemanticGoalProvider([_frontier_choice(1)])
    controller = AsyncSemanticGoalController(
        provider,
        provider_p95_s=float(latency["p95_s"]),
        prefetch_margin_s=float(latency["prefetch_margin_s"]),
        modeled_provider_latency_s=float(latency["p95_s"]),
    )
    try:
        first = str(short["frontiers"][0]["signature"])
        controller.start(
            _decision(
                short,
                "go_to_frontier",
                {"frontier_id": first},
                f"Start short hop at {first}.",
            ),
            short,
        )
        controller.tick(
            short,
            now_s=0.0,
            remaining_distance_m=route,
            eta_s=float(fixture["short_hop_replay"]["arrival_s"]),
        )
        _wait_for_provider(provider, 1)
        arrival = controller.tick(
            short,
            now_s=float(fixture["short_hop_replay"]["arrival_s"]),
            remaining_distance_m=0.0,
            eta_s=0.0,
        )
        resumed = controller.tick(
            short,
            now_s=float(
                fixture["short_hop_replay"]["modeled_provider_ready_s"]
            )
            + 0.01,
            remaining_distance_m=route,
            eta_s=30.0,
        )
        return {
            "arrival_state": arrival.handoff.state,
            "arrival_zero_required": arrival.handoff.command.zero_required,
            "arrival_reason": arrival.handoff.command.reason,
            "resume_state": resumed.handoff.state,
            "resume_controller_session": resumed.handoff.controller_session,
        }
    finally:
        controller.close()


def _safety_evidence(
    fixture: Mapping[str, Any],
    snapshot: Mapping[str, Any],
) -> dict[str, Any]:
    release = threading.Event()
    provider = ScriptedSemanticGoalProvider(
        [_frontier_choice(1)], release_event=release
    )
    controller = AsyncSemanticGoalController(provider)
    try:
        first = str(snapshot["frontiers"][0]["signature"])
        controller.start(
            _decision(
                snapshot,
                "go_to_frontier",
                {"frontier_id": first},
                f"Start safety replay at {first}.",
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
        veto = controller.tick(
            snapshot,
            now_s=0.01,
            remaining_distance_m=2.49,
            eta_s=9.9,
            collision_state="BLOCKED",
        )
        return {
            "provider_in_flight": veto.provider_in_flight,
            "state": veto.handoff.state,
            "zero_required": veto.handoff.command.zero_required,
            "reason": veto.handoff.command.reason,
            "event_kinds": [item["kind"] for item in veto.events],
        }
    finally:
        release.set()
        controller.close()


def _event_evidence(
    fixture: Mapping[str, Any],
    snapshot0: Mapping[str, Any],
    snapshot1: Mapping[str, Any],
) -> dict[str, Any]:
    release = threading.Event()
    provider = ScriptedSemanticGoalProvider(
        [_frontier_choice(1), _inspect_choice], release_event=release
    )
    controller = AsyncSemanticGoalController(
        provider,
        modeled_provider_latency_s=float(
            fixture["provider_latency_profile"]["p95_s"]
        ),
    )
    try:
        first = str(snapshot0["frontiers"][0]["signature"])
        controller.start(
            _decision(
                snapshot0,
                "go_to_frontier",
                {"frontier_id": first},
                f"Start event replay at {first}.",
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
        preempted = controller.handle_event(
            SemanticReplanEvent(
                event_id="stable-new-shoe",
                kind=SemanticEventKind.NEW_DETECTION,
                observed_at_s=2.2,
                target_id="shoe-track-01",
                confidence=0.90,
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
        return {
            "preempt_state": preempted.handoff.state,
            "preempt_reason": preempted.handoff.command.reason,
            "event_kinds": [item["kind"] for item in preempted.events],
            "discard_reasons": [
                item.get("reason", "")
                for item in discarded.events
                if item["kind"] == "prefetch_discarded"
            ],
            "resume_state": resumed.handoff.state,
            "resume_event_kinds": [
                item["kind"] for item in resumed.handoff.events
            ],
        }
    finally:
        release.set()
        controller.close()


def evaluate_phase3_fixture(
    fixture: Mapping[str, Any], *, root: Path
) -> dict[str, Any]:
    if fixture.get("schema") != "sphero_rvr.phase3_replay_fixture.v1":
        raise MissionValidationError("Phase 3 replay fixture schema is unsupported")
    grid, frontiers, track, nbv = _snapshot_components(fixture, root)
    snapshot0 = _snapshot(fixture, grid, frontiers, track, nbv)
    snapshot1 = _snapshot(
        fixture, grid, frontiers, track, nbv, event_generation=1
    )
    canonical = _canonical_evidence(fixture, snapshot0)
    short_hop = _short_hop_evidence(fixture, snapshot0)
    event = _event_evidence(fixture, snapshot0, snapshot1)
    safety = _safety_evidence(fixture, snapshot0)
    handoff_kinds = [item["kind"] for item in canonical["handoff"]["trace"]]
    schema_rendered = json.dumps(semantic_goal_output_schema(), sort_keys=True)
    checks = {
        "semantic_schema_has_no_control_geometry": all(
            token not in schema_rendered
            for token in ('"x_m"', '"y_m"', '"route"', '"speed"', '"velocity"')
        ),
        "canonical_atomic_handoffs": handoff_kinds.count("atomic_handoff")
        >= int(fixture["canonical_replay"]["minimum_atomic_handoffs"]),
        "canonical_one_controller_session": canonical["handoff"][
            "controller_session"
        ]
        == 1,
        "decision_reduction_at_least_tenfold": canonical[
            "decision_reduction_ratio"
        ]
        + 1e-9
        >= float(
            fixture["canonical_replay"]["minimum_decision_reduction_ratio"]
        ),
        "short_hop_honest_wait": short_hop["arrival_state"]
        == fixture["short_hop_replay"]["expected_arrival_state"]
        and short_hop["arrival_zero_required"] is True,
        "stable_detection_preempts_and_replans": event["preempt_reason"]
        == "semantic_replan"
        and "event_triggered_replan" in event["event_kinds"]
        and event["resume_state"] == "navigating",
        "old_inflight_result_is_discarded": any(
            "event_invalidated:new_detection" in reason
            for reason in event["discard_reasons"]
        ),
        "supervisor_veto_during_provider": safety["provider_in_flight"] is True
        and safety["zero_required"] is True
        and safety["reason"] == "collision_veto",
        "authority_false": canonical["authority"] == fixture["authority"],
        "carryovers_retained": canonical["carryovers"] == fixture["carryovers"],
    }
    return {
        "schema": "sphero_rvr.phase3_replay_evaluation.v1",
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            "provider_p95_s": canonical["provider_latency_p95_s"],
            "prefetch_threshold_s": canonical["prefetch_threshold_s"],
            "canonical_distance_m": canonical["distance_m"],
            "motion_goal_decisions": canonical["motion_goal_decisions"],
            "decision_reduction_ratio": canonical["decision_reduction_ratio"],
            "atomic_handoffs": handoff_kinds.count("atomic_handoff"),
            "controller_sessions": canonical["handoff"]["controller_session"],
        },
        "canonical": canonical,
        "short_hop": short_hop,
        "event_replan": event,
        "safety_veto": safety,
        "scope": {
            "recorded_map_replay": True,
            "scripted_provider_for_deterministic_acceptance": True,
            "real_oauth_adapter_covered_separately": True,
            "physical_accuracy_claim": False,
            "physical_execution": False,
        },
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run Milestone 6 Phase 3 semantic-goal replay evidence"
    )
    parser.add_argument("fixture", type=Path)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    fixture_path = args.fixture.resolve()
    root = Path.cwd().resolve()
    fixture = json.loads(fixture_path.read_text())
    report = evaluate_phase3_fixture(fixture, root=root)
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
        print(args.output)
    return 0 if report["passed"] else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
