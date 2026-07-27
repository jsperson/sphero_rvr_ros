"""Durable real-provider evidence for Milestone 6 Phase 4.

The replay aligns measured provider wall time with the unscaled 0.10 m/s
motion timeline.  Traversal before the prefetch threshold is fast-forwarded;
the planning window itself advances one replay second per wall second.  No ROS
node, serial transport, motor topic, or physical command authority is present.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import statistics
import threading
import time
import uuid
from typing import Any, Mapping, Optional, Sequence

from .hierarchical_exploration import detect_frontiers, load_slam_toolbox_map
from .hierarchical_goal_selection import (
    AsyncSemanticGoalController,
    CodexOAuthSemanticGoalProvider,
    DeterministicGoalResolver,
    NextBestViewPlan,
    SemanticGoalDecision,
    SemanticGoalProvider,
    SemanticTrack,
    build_semantic_world_snapshot,
    generate_next_best_views,
    revalidate_resolved_goal,
    semantic_goal_prompt,
)
from .mission_api import MissionValidationError
from .mission_service import MissionService


PHASE4_PROPOSAL_SCHEMA = "sphero_rvr.hierarchical_phase4_proposal.v1"
PHASE4_RESULT_SCHEMA = "sphero_rvr.hierarchical_phase4_replay.v1"
PHASE4_CHECKPOINT_SCHEMA = "sphero_rvr.hierarchical_phase4_checkpoint.v1"


def _canonical_digest(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _percentile(samples: Sequence[float], percentile: float) -> float:
    ordered = sorted(float(item) for item in samples)
    if not ordered:
        return 0.0
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(percentile)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


class RecordingSemanticGoalProvider:
    """Thread-safe recorder around the real or injected semantic provider."""

    def __init__(self, provider: SemanticGoalProvider) -> None:
        self.provider = provider
        self.provider_id = provider.provider_id
        self.model_id = provider.model_id
        self.reasoning_effort = str(
            getattr(provider, "reasoning_effort", "recorded")
        )
        self._lock = threading.Lock()
        self._calls: list[dict[str, Any]] = []

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        started = time.perf_counter()
        wall_started_s = time.time()
        captured = json.loads(json.dumps(dict(snapshot)))
        output: Optional[dict[str, Any]] = None
        error = ""
        try:
            raw = self.provider.choose(prompt, snapshot)
            output = json.loads(json.dumps(dict(raw)))
            return raw
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            elapsed_s = time.perf_counter() - started
            provider_metric: Mapping[str, Any] = {}
            latency_history = getattr(self.provider, "latency_history", None)
            if callable(latency_history):
                history = latency_history()
                if history:
                    provider_metric = history[-1]
            record = {
                "call_index": 0,
                "snapshot": captured,
                "snapshot_id": str(captured.get("snapshot_id", "")),
                "decision_generation": int(
                    captured.get("decision_generation", 0)
                ),
                "wall_started_s": wall_started_s,
                "wall_finished_s": time.time(),
                "latency_s": elapsed_s,
                "success": output is not None,
                "error": error,
                "output": output,
                "provider_metric": json.loads(
                    json.dumps(dict(provider_metric))
                ),
            }
            with self._lock:
                record["call_index"] = len(self._calls) + 1
                self._calls.append(record)

    def calls(self) -> list[dict[str, Any]]:
        with self._lock:
            return json.loads(json.dumps(self._calls))

    def cancel(self) -> None:
        cancel = getattr(self.provider, "cancel", None)
        if callable(cancel):
            cancel()

    def close(self) -> None:
        close = getattr(self.provider, "close", None)
        if callable(close):
            close()


def _load_components(
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
    route_length = float(
        fixture["canonical_replay"]["route_length_m_per_goal"]
    )
    frontiers = tuple(
        replace(detected[int(index)], path_distance_m=route_length)
        for index in fixture["canonical_replay"]["frontier_indexes"]
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
        evidence_ids=tuple(
            str(item) for item in track_value["evidence_ids"]
        ),
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


def _build_snapshot(
    *,
    fixture: Mapping[str, Any],
    mission_id: str,
    grid: Any,
    frontiers: Sequence[Any],
    tracks: Sequence[SemanticTrack],
    nbv: NextBestViewPlan,
    decision_generation: int,
    coverage_fraction: float,
    minimum_real_calls: int,
) -> dict[str, Any]:
    robot = fixture["robot_pose"]
    origin = fixture["mission_origin"]
    active_nbv = tuple(
        nbv for track in tracks if track.track_id == nbv.track_id
    )
    snapshot = build_semantic_world_snapshot(
        mission_id=mission_id,
        objective=str(fixture["canonical_replay"]["objective"]),
        objective_revision=0,
        decision_generation=decision_generation,
        event_generation=0,
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
        frontiers=tuple(frontiers),
        tracks=tuple(tracks),
        next_best_views=active_nbv,
        origin_x_m=float(origin["x_m"]),
        origin_y_m=float(origin["y_m"]),
        coverage_fraction=coverage_fraction,
    )
    actions = {"return_to_start"}
    if frontiers:
        actions.update({"go_to_frontier", "search_region"})
    if tracks and active_nbv and active_nbv[0].candidates:
        actions.add("inspect")
    snapshot["provider_action_allowlist"] = sorted(actions)
    snapshot["evaluation"] = {
        "purpose": "Phase 4 real-provider multi-decision replay evidence",
        "minimum_real_provider_calls": int(minimum_real_calls),
        "current_call_generation": int(decision_generation),
        "finish_allowed": False,
        "wait_allowed": False,
        "motion_authority": False,
        "physical_execution_enabled": False,
    }
    snapshot.pop("snapshot_id", None)
    snapshot["snapshot_id"] = _canonical_digest(snapshot)
    return snapshot


def _validated_resolved_call(
    call: Mapping[str, Any],
    *,
    provider: RecordingSemanticGoalProvider,
    ready_at_s: float,
) -> tuple[SemanticGoalDecision, Any]:
    snapshot = call["snapshot"]
    output = call.get("output")
    if not isinstance(output, Mapping):
        raise MissionValidationError("provider call has no semantic output")
    generation = int(snapshot["decision_generation"])
    decision = SemanticGoalDecision.validated(
        output,
        snapshot=snapshot,
        expected_generation=generation,
        provider_id=provider.provider_id,
        model_id=provider.model_id,
    )
    resolved = DeterministicGoalResolver().resolve(
        decision, snapshot, ready_at_s=ready_at_s
    )
    validation = revalidate_resolved_goal(
        resolved,
        captured_snapshot=snapshot,
        current_snapshot=snapshot,
    )
    if not validation.accepted:
        raise MissionValidationError(
            "recorded provider goal failed replay revalidation: "
            + ",".join(validation.reasons)
        )
    if resolved.kind != "motion":
        raise MissionValidationError(
            "Phase 4 multi-decision evidence requires a motion semantic goal"
        )
    return decision, resolved


def _decision_evidence(
    call: Mapping[str, Any],
    decision: SemanticGoalDecision,
    resolved: Any,
    *,
    dispatch_at_s: float,
    handoff_at_s: float,
    handoff_kind: str,
    wait_started_at_s: Optional[float],
) -> dict[str, Any]:
    latency_s = float(call["latency_s"])
    ready_at_s = dispatch_at_s + latency_s
    pause_s = (
        0.0
        if wait_started_at_s is None
        else max(0.0, handoff_at_s - wait_started_at_s)
    )
    return {
        "call_index": int(call["call_index"]),
        "decision_generation": decision.decision_generation,
        "snapshot_id": decision.snapshot_id,
        "action": decision.action,
        "arguments": dict(decision.arguments),
        "rationale": decision.rationale,
        "resolved_target_id": resolved.target_id,
        "resolved_target_signature": resolved.target_signature,
        "resolved_path": {
            "frame_id": "map",
            "x_m": resolved.x_m,
            "y_m": resolved.y_m,
            "yaw_rad": resolved.yaw_rad,
            "route_length_m": resolved.route_length_m,
            "minimum_clearance_m": resolved.minimum_clearance_m,
            "server_owned": True,
        },
        "timing": {
            "dispatch_at_s": dispatch_at_s,
            "provider_ready_at_s": ready_at_s,
            "handoff_at_s": handoff_at_s,
            "latency_s": latency_s,
            "handoff_kind": handoff_kind,
            "wait_planning_started_at_s": wait_started_at_s,
            "motor_zero_interval_s": pause_s,
        },
        "provider_metric": dict(call.get("provider_metric", {})),
    }


def _map_projection(
    grid: Any,
    snapshot: Mapping[str, Any],
    decisions: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    localization = snapshot["localization"]
    origin = {
        "x_m": float(grid.origin_x_m),
        "y_m": float(grid.origin_y_m),
    }
    route = [
        {
            "x_m": float(item["resolved_path"]["x_m"]),
            "y_m": float(item["resolved_path"]["y_m"]),
        }
        for item in decisions
        if item["resolved_path"]["x_m"] is not None
        and item["resolved_path"]["y_m"] is not None
    ]
    occupancy = []
    stride = max(1, int(max(grid.width, grid.height) / 80))
    for y in range(0, grid.height, stride):
        for x in range(0, grid.width, stride):
            if grid.value((x, y)) == 100:
                x_m, y_m = grid.cell_to_world((x, y))
                occupancy.append(
                    {
                        "x_m": x_m,
                        "y_m": y_m,
                        "width_m": grid.resolution_m * stride,
                        "height_m": grid.resolution_m * stride,
                        "label": "occupancy",
                    }
                )
    return {
        "available": True,
        "fixture_only": True,
        "source": grid.source,
        "frame": grid.frame_id,
        "bounds": {
            "origin": origin,
            "width_m": grid.width * grid.resolution_m,
            "height_m": grid.height * grid.resolution_m,
        },
        "rover": {
            "x_m": float(localization["x_m"]),
            "y_m": float(localization["y_m"]),
            "yaw_deg": math.degrees(float(localization["yaw_rad"])),
        },
        "goal_region": (
            None
            if not route
            else {**route[-1], "radius_m": 0.12}
        ),
        "proposed_route": route,
        "traveled_path": [
            {
                "x_m": float(localization["x_m"]),
                "y_m": float(localization["y_m"]),
            },
            *route,
        ],
        "obstacles": occupancy,
        "objects": [
            {
                "object_id": str(track["track_id"]),
                "label": str(track["class_name"]),
                "x_m": float(track["x_m"]),
                "y_m": float(track["y_m"]),
                "uncertainty_m": float(track["position_sigma_m"]),
            }
            for track in snapshot.get("tracks", ())
            if "x_m" in track and "y_m" in track
        ],
        "frontiers": [
            {
                "frontier_id": str(frontier["signature"]),
                "x_m": float(frontier["approach_pose"]["x_m"]),
                "y_m": float(frontier["approach_pose"]["y_m"]),
                "information_gain_m": float(
                    frontier["information_gain_m"]
                ),
            }
            for frontier in snapshot.get("frontiers", ())
        ],
        "localization": {
            "state": "replay",
            "fresh": True,
            "quality": float(localization["quality"]),
        },
    }


def _persist_checkpoint(
    service: MissionService,
    mission_id: str,
    *,
    kind: str,
    snapshot: Mapping[str, Any],
    decisions: Sequence[Mapping[str, Any]],
    controller: AsyncSemanticGoalController,
    provider: RecordingSemanticGoalProvider,
    phase4_events: Sequence[Mapping[str, Any]],
) -> None:
    service.record_rolling_replay_checkpoint(
        mission_id,
        kind=kind,
        checkpoint={
            "schema": PHASE4_CHECKPOINT_SCHEMA,
            "mission_id": mission_id,
            "status": "running",
            "world_snapshot": dict(snapshot),
            "decisions": list(decisions),
            "provider_calls": provider.calls(),
            "controller": _json_safe(controller.evidence()),
            "phase4_events": list(phase4_events),
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
    )


def run_phase4_replay(
    fixture: Mapping[str, Any],
    *,
    root: Path,
    provider: SemanticGoalProvider,
    database: str | Path,
    source_sha: str,
    mission_id: Optional[str] = None,
    session_id: str = "hierarchical-phase4-replay",
    minimum_real_calls: int = 4,
    poll_s: float = 0.02,
    provider_p95_s: Optional[float] = None,
    prefetch_margin_s: Optional[float] = None,
) -> dict[str, Any]:
    """Run and persist one wall-clock-aligned no-authority Phase 4 replay."""

    if fixture.get("schema") != "sphero_rvr.phase3_replay_fixture.v1":
        raise MissionValidationError("Phase 4 requires the Phase 3 replay fixture")
    if minimum_real_calls < 4:
        raise MissionValidationError(
            "Phase 4 requires at least four consecutive real provider calls"
        )
    if poll_s <= 0.0:
        raise MissionValidationError("Phase 4 poll interval must be positive")
    grid, original_frontiers, track, nbv = _load_components(fixture, root)
    remaining_frontiers = list(original_frontiers)
    remaining_tracks = [track]
    coverage = 0.42
    mission = mission_id or f"hierarchical-phase4-{uuid.uuid4().hex}"
    latency_profile = fixture["provider_latency_profile"]
    p95_s = float(
        latency_profile["p95_s"]
        if provider_p95_s is None
        else provider_p95_s
    )
    margin_s = float(
        latency_profile["prefetch_margin_s"]
        if prefetch_margin_s is None
        else prefetch_margin_s
    )
    threshold_s = p95_s + margin_s
    speed_mps = 0.10
    route_length_m = float(
        fixture["canonical_replay"]["route_length_m_per_goal"]
    )
    leg_duration_s = route_length_m / speed_mps
    if threshold_s <= 0.0 or threshold_s >= leg_duration_s:
        raise MissionValidationError(
            "Phase 4 long-leg duration must exceed the prefetch threshold"
        )

    recorder = RecordingSemanticGoalProvider(provider)
    service = MissionService(
        database,
        source_sha=str(source_sha),
        deployed_sha=str(source_sha),
        mode="replay",
        live_execution_enabled=False,
    )
    controller = AsyncSemanticGoalController(
        recorder,
        provider_p95_s=p95_s,
        prefetch_margin_s=margin_s,
        modeled_provider_latency_s=None,
    )
    decisions: list[dict[str, Any]] = []
    phase4_events: list[dict[str, Any]] = []
    snapshots: list[dict[str, Any]] = []
    wait_intervals: list[dict[str, Any]] = []
    started_wall_s = time.time()
    replay_now_s = 0.0
    mission_persisted = False
    try:
        service.begin_prompt_mission(
            mission_id=mission,
            session_id=session_id,
            prompt=str(fixture["canonical_replay"]["objective"]),
            source="cli",
        )
        mission_persisted = True
        proposal_body = {
            "schema": PHASE4_PROPOSAL_SCHEMA,
            "mission_id": mission,
            "prompt": str(fixture["canonical_replay"]["objective"]),
            "source_sha": str(source_sha),
            "provider_id": recorder.provider_id,
            "model_id": recorder.model_id,
            "reasoning_effort": recorder.reasoning_effort,
            "decision": "propose",
            "summary": (
                "Run at least four consecutive semantic-goal provider calls "
                "against the recorded map with wall-clock-aligned prefetch."
            ),
            "segments": [],
            "contract": {
                "recorded_map_replay": True,
                "provider_wall_clock": True,
                "minimum_real_provider_calls": minimum_real_calls,
                "motion_authority": False,
                "physical_execution_enabled": False,
            },
            "limits": {
                "linear_speed_mps": speed_mps,
                "prefetch_threshold_s": threshold_s,
                "physical_execution": False,
            },
        }
        proposal = {
            **proposal_body,
            "proposal_digest": _canonical_digest(proposal_body),
        }
        service.record_rolling_replay_proposal(mission, proposal)
        service.approve_rolling_replay_mission(
            mission,
            proposal_digest=proposal["proposal_digest"],
            operator="phase4-replay-runner",
        )

        snapshot = _build_snapshot(
            fixture=fixture,
            mission_id=mission,
            grid=grid,
            frontiers=remaining_frontiers,
            tracks=remaining_tracks,
            nbv=nbv,
            decision_generation=1,
            coverage_fraction=coverage,
            minimum_real_calls=minimum_real_calls,
        )
        snapshots.append(snapshot)
        initial_dispatch_wall = time.perf_counter()
        raw = recorder.choose(
            semantic_goal_prompt(snapshot["objective"], snapshot),
            snapshot,
        )
        del initial_dispatch_wall
        call = recorder.calls()[-1]
        decision, resolved = _validated_resolved_call(
            call, provider=recorder, ready_at_s=float(call["latency_s"])
        )
        initial = controller.start(
            raw, snapshot, now_s=float(call["latency_s"])
        )
        replay_now_s = float(call["latency_s"])
        decisions.append(
            _decision_evidence(
                call,
                decision,
                resolved,
                dispatch_at_s=0.0,
                handoff_at_s=replay_now_s,
                handoff_kind="initial_goal_started",
                wait_started_at_s=None,
            )
        )
        phase4_events.extend(dict(item) for item in initial.events)
        _consume_target(
            resolved,
            remaining_frontiers=remaining_frontiers,
            remaining_tracks=remaining_tracks,
        )
        _persist_checkpoint(
            service,
            mission,
            kind="phase4_initial_goal",
            snapshot=snapshot,
            decisions=decisions,
            controller=controller,
            provider=recorder,
            phase4_events=phase4_events,
        )

        while len(decisions) < minimum_real_calls:
            coverage = min(0.95, coverage + 0.08)
            next_generation = len(recorder.calls()) + 1
            snapshot = _build_snapshot(
                fixture=fixture,
                mission_id=mission,
                grid=grid,
                frontiers=remaining_frontiers,
                tracks=remaining_tracks,
                nbv=nbv,
                decision_generation=next_generation,
                coverage_fraction=coverage,
                minimum_real_calls=minimum_real_calls,
            )
            snapshots.append(snapshot)
            leg_start_s = replay_now_s
            dispatch_at_s = leg_start_s + leg_duration_s - threshold_s
            expected_call_count = len(recorder.calls()) + 1
            dispatched = controller.tick(
                snapshot,
                now_s=dispatch_at_s,
                remaining_distance_m=threshold_s * speed_mps,
                eta_s=threshold_s,
            )
            phase4_events.extend(dict(item) for item in dispatched.events)
            provider_window_started = time.monotonic()
            wait_started_at_s: Optional[float] = None
            handoff_at_s: Optional[float] = None
            handoff_kind = ""
            while handoff_at_s is None:
                elapsed_s = time.monotonic() - provider_window_started
                now_s = dispatch_at_s + elapsed_s
                remaining_s = max(0.0, threshold_s - elapsed_s)
                step = controller.tick(
                    snapshot,
                    now_s=now_s,
                    remaining_distance_m=remaining_s * speed_mps,
                    eta_s=(
                        remaining_s
                        if remaining_s > 0.0
                        else leg_duration_s + 1.0
                    ),
                )
                phase4_events.extend(dict(item) for item in step.events)
                discarded = [
                    item
                    for item in step.events
                    if item.get("kind") == "prefetch_discarded"
                ]
                if discarded:
                    raise MissionValidationError(
                        "Phase 4 real provider decision was rejected: "
                        + str(discarded[-1].get("reason", "unknown"))
                    )
                if (
                    step.handoff.state == "wait_planning"
                    and wait_started_at_s is None
                ):
                    wait_started_at_s = now_s
                handoff_events = [
                    event
                    for event in step.handoff.events
                    if event.get("kind")
                    in {"atomic_handoff", "planning_resume"}
                ]
                if handoff_events:
                    handoff_at_s = now_s
                    handoff_kind = str(handoff_events[-1]["kind"])
                    phase4_events.extend(
                        dict(item) for item in step.handoff.events
                    )
                    break
                if elapsed_s > max(
                    180.0, float(getattr(provider, "timeout_s", 120.0)) + 5.0
                ):
                    raise MissionValidationError(
                        "Phase 4 provider did not produce a handoff before timeout"
                    )
                time.sleep(poll_s)
            calls = recorder.calls()
            if len(calls) < expected_call_count:
                raise MissionValidationError(
                    "Phase 4 handoff occurred without a recorded provider call"
                )
            call = calls[expected_call_count - 1]
            decision, resolved = _validated_resolved_call(
                call,
                provider=recorder,
                ready_at_s=dispatch_at_s + float(call["latency_s"]),
            )
            decisions.append(
                _decision_evidence(
                    call,
                    decision,
                    resolved,
                    dispatch_at_s=dispatch_at_s,
                    handoff_at_s=handoff_at_s,
                    handoff_kind=handoff_kind,
                    wait_started_at_s=wait_started_at_s,
                )
            )
            if wait_started_at_s is not None:
                wait_intervals.append(
                    {
                        "decision_generation": decision.decision_generation,
                        "start_at_s": wait_started_at_s,
                        "end_at_s": handoff_at_s,
                        "duration_s": max(
                            0.0, handoff_at_s - wait_started_at_s
                        ),
                        "reason": "wait_planning",
                        "motor_zero": True,
                    }
                )
            replay_now_s = handoff_at_s
            _consume_target(
                resolved,
                remaining_frontiers=remaining_frontiers,
                remaining_tracks=remaining_tracks,
            )
            _persist_checkpoint(
                service,
                mission,
                kind="phase4_handoff",
                snapshot=snapshot,
                decisions=decisions,
                controller=controller,
                provider=recorder,
                phase4_events=phase4_events,
            )

        final_arrival_s = replay_now_s + leg_duration_s
        terminal_step = controller.tick(
            snapshot,
            now_s=final_arrival_s,
            remaining_distance_m=0.0,
            eta_s=threshold_s + 1.0,
        )
        phase4_events.extend(dict(item) for item in terminal_step.handoff.events)
        controller_evidence = controller.evidence()
        calls = recorder.calls()
        latencies = [float(item["latency_s"]) for item in calls]
        short_hop_s = float(fixture["short_hop_replay"]["arrival_s"])
        short_hops = [
            {
                "call_index": int(item["call_index"]),
                "latency_s": float(item["latency_s"]),
                "arrival_s": short_hop_s,
                "state_at_arrival": (
                    "wait_planning"
                    if float(item["latency_s"]) > short_hop_s
                    else "prefetch_ready"
                ),
                "motor_zero_interval_s": max(
                    0.0, float(item["latency_s"]) - short_hop_s
                ),
            }
            for item in calls
        ]
        handoffs = [
            item["timing"]["handoff_kind"]
            for item in decisions[1:]
        ]
        distance_m = route_length_m * len(decisions)
        decisions_per_m = len(decisions) / distance_m
        result = {
            "schema": PHASE4_RESULT_SCHEMA,
            "mission_id": mission,
            "status": "complete",
            "terminal_reason": "phase4_real_provider_replay_complete",
            "source_sha": str(source_sha),
            "provider": {
                "provider_id": recorder.provider_id,
                "model_id": recorder.model_id,
                "reasoning_effort": recorder.reasoning_effort,
                "calls_started": len(calls),
                "calls_completed": sum(
                    1 for item in calls if item["success"]
                ),
                "calls": calls,
                "latency_distribution_s": {
                    "count": len(latencies),
                    "min": min(latencies),
                    "p50": statistics.median(latencies),
                    "p95": _percentile(latencies, 0.95),
                    "max": max(latencies),
                    "baseline_p95": p95_s,
                },
            },
            "decisions": decisions,
            "snapshots": snapshots,
            "events": phase4_events,
            "metrics": {
                "prefetch_threshold_s": threshold_s,
                "long_leg_duration_s": leg_duration_s,
                "long_leg_atomic_handoffs": handoffs.count(
                    "atomic_handoff"
                ),
                "long_leg_wait_planning_handoffs": handoffs.count(
                    "planning_resume"
                ),
                "controller_sessions": controller_evidence["handoff"][
                    "controller_session"
                ],
                "motor_zero_intervals": wait_intervals,
                "distance_m": distance_m,
                "decisions_per_m": decisions_per_m,
                "legacy_025m_decisions_per_m": 4.0,
                "decision_reduction_ratio": 4.0 / decisions_per_m,
                "coverage_start": 0.42,
                "coverage_end": coverage,
                "coverage_delta": coverage - 0.42,
            },
            "short_hop_characterization": {
                "route_length_m": float(
                    fixture["short_hop_replay"]["route_length_m"]
                ),
                "arrival_s": short_hop_s,
                "samples": short_hops,
                "wait_planning_count": sum(
                    1
                    for item in short_hops
                    if item["state_at_arrival"] == "wait_planning"
                ),
                "interpretation": (
                    "Counterfactual replay using each measured real-provider "
                    "latency; no short-hop continuity is claimed."
                ),
            },
            "controller": controller_evidence,
            "map": _map_projection(grid, snapshot, decisions),
            "scope": {
                "recorded_map_replay": True,
                "real_provider_wall_latency": recorder.provider_id
                == "openai-codex-oauth-semantic-goal",
                "accelerated_before_prefetch_window": True,
                "prefetch_window_wall_clock_ratio": 1.0,
                "motor_zero_intervals_are_replay_derived": True,
                "physical_accuracy_claim": False,
                "motion_authority": False,
                "physical_execution_enabled": False,
                "live_sensors": False,
                "serial_access": False,
            },
            "carryovers": {
                "phase2_accuracy_is_physical_certification": False,
                "pi_no_motion_wfd_before_physical": True,
                "pi_command_ownership_before_physical": True,
                "dropoff_sensing_available": False,
            },
            "wall_run": {
                "started_at_s": started_wall_s,
                "finished_at_s": time.time(),
            },
            "motion_authority": False,
            "physical_execution_enabled": False,
        }
        service.finish_rolling_replay_mission(
            mission,
            status="complete",
            reason=result["terminal_reason"],
            result=result,
        )
        return result
    except Exception as exc:
        try:
            status = (
                service.prompt_status(mission)["status"]
                if mission_persisted
                else ""
            )
            if status == "running":
                service.finish_rolling_replay_mission(
                    mission,
                    status="failed",
                    reason=str(exc),
                    result={
                        "schema": PHASE4_RESULT_SCHEMA,
                        "mission_id": mission,
                        "status": "failed",
                        "terminal_reason": str(exc),
                        "provider_calls": recorder.calls(),
                        "motion_authority": False,
                        "physical_execution_enabled": False,
                    },
                )
        finally:
            raise
    finally:
        controller.close()
        service.close()


def _consume_target(
    resolved: Any,
    *,
    remaining_frontiers: list[Any],
    remaining_tracks: list[SemanticTrack],
) -> None:
    if resolved.decision.action in {"go_to_frontier", "search_region"}:
        remaining_frontiers[:] = [
            item
            for item in remaining_frontiers
            if item.signature != resolved.target_signature
        ]
    elif resolved.decision.action == "inspect":
        remaining_tracks[:] = [
            item
            for item in remaining_tracks
            if item.track_id != resolved.target_id
        ]


def load_phase4_mission(
    database: str | Path,
    mission_id: str,
    *,
    source_sha: str,
) -> dict[str, Any]:
    """Reopen one persisted Phase 4 mission by its exact mission ID."""

    service = MissionService(
        database,
        source_sha=str(source_sha),
        deployed_sha=str(source_sha),
        mode="replay",
        live_execution_enabled=False,
    )
    try:
        mission = service.prompt_status(mission_id)
    finally:
        service.close()
    proposal = mission.get("proposal", {})
    if not isinstance(proposal, Mapping) or proposal.get(
        "schema"
    ) != PHASE4_PROPOSAL_SCHEMA:
        raise MissionValidationError(
            "persisted mission is not a Phase 4 hierarchical replay"
        )
    return mission


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run Milestone 6 Phase 4 real-provider replay evidence without "
            "ROS, serial access, or physical authority"
        )
    )
    parser.add_argument("fixture", type=Path)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mission-id")
    parser.add_argument("--source-sha", required=True)
    parser.add_argument("--model", default=None)
    parser.add_argument("--reasoning-effort", default="low")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    fixture_path = args.fixture.resolve()
    root = Path.cwd().resolve()
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    report = run_phase4_replay(
        fixture,
        root=root,
        provider=CodexOAuthSemanticGoalProvider(
            model=args.model,
            reasoning_effort=args.reasoning_effort,
        ),
        database=args.database,
        source_sha=args.source_sha,
        mission_id=args.mission_id,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "mission_id": report["mission_id"],
                "output": str(args.output),
                "database": str(args.database),
                "provider_calls": report["provider"]["calls_completed"],
                "latency_distribution_s": report["provider"][
                    "latency_distribution_s"
                ],
                "atomic_handoffs": report["metrics"][
                    "long_leg_atomic_handoffs"
                ],
                "wait_planning_handoffs": report["metrics"][
                    "long_leg_wait_planning_handoffs"
                ],
                "physical_execution_enabled": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
