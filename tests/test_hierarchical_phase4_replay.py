from __future__ import annotations

import json
from pathlib import Path
import time

import pytest

from sphero_rvr_driver.hierarchical_goal_selection import (
    semantic_goal_provider_output_schema,
)
from sphero_rvr_driver.hierarchical_phase4_replay import (
    PHASE4_RESULT_SCHEMA,
    load_phase4_mission,
    run_phase4_replay,
)
from sphero_rvr_driver.mission_web import (
    Phase4ReplayEvidenceAdapter,
    build_mission_web_bundle,
)


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = (
    ROOT / "artifacts" / "phase3_semantic_goal_replay" / "fixture.json"
)


class DelayedSemanticProvider:
    provider_id = "test-real-wall-provider"
    model_id = "test-model"
    reasoning_effort = "low"

    def __init__(self, delays: list[float]) -> None:
        self.delays = list(delays)
        self.calls = 0

    def choose(self, prompt: str, snapshot: dict) -> dict:
        del prompt
        index = self.calls
        self.calls += 1
        time.sleep(self.delays[index])
        generation = int(snapshot["decision_generation"])
        if index == 0:
            action = "go_to_frontier"
            arguments = {
                "frontier_id": snapshot["frontiers"][0]["signature"]
            }
            target = arguments["frontier_id"]
        elif index == 1:
            action = "inspect"
            arguments = {"track_id": snapshot["tracks"][0]["track_id"]}
            target = arguments["track_id"]
        elif index == 2:
            action = "go_to_frontier"
            arguments = {
                "frontier_id": snapshot["frontiers"][0]["signature"]
            }
            target = arguments["frontier_id"]
        else:
            action = "return_to_start"
            arguments = {}
            target = "mission_origin"
        return {
            "schema": "sphero_rvr.semantic_goal.v1",
            "mission_id": snapshot["mission_id"],
            "snapshot_id": snapshot["snapshot_id"],
            "decision_generation": generation,
            "event_generation": snapshot["event_generation"],
            "action": action,
            "arguments": arguments,
            "rationale": f"Select {target} for recorded replay evidence.",
        }


def test_provider_schema_can_be_narrowed_by_server_owned_replay_policy() -> None:
    snapshot = {
        "mission_id": "phase4-test",
        "snapshot_id": "a" * 64,
        "decision_generation": 3,
        "event_generation": 0,
        "provider_action_allowlist": [
            "inspect",
            "go_to_frontier",
            "inspect",
        ],
    }

    schema = semantic_goal_provider_output_schema(snapshot)

    assert schema["properties"]["action"]["enum"] == [
        "go_to_frontier",
        "inspect",
    ]


def test_real_wall_timeline_persists_and_reopens_pause_evidence(
    tmp_path: Path,
) -> None:
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    database = tmp_path / "phase4.sqlite3"
    provider = DelayedSemanticProvider([0.001, 0.001, 0.035, 0.001])

    report = run_phase4_replay(
        fixture,
        root=ROOT,
        provider=provider,
        database=database,
        source_sha="phase4-test-sha",
        mission_id="phase4-persisted-test",
        poll_s=0.001,
        provider_p95_s=0.015,
        prefetch_margin_s=0.005,
    )
    reopened = load_phase4_mission(
        database,
        report["mission_id"],
        source_sha="phase4-test-sha",
    )
    adapter = Phase4ReplayEvidenceAdapter(
        database=database,
        mission_id=report["mission_id"],
        source_sha="phase4-view-test",
    )
    try:
        browser = adapter.snapshot()
    finally:
        adapter.close()

    assert report["schema"] == PHASE4_RESULT_SCHEMA
    assert report["provider"]["calls_completed"] == 4
    assert report["metrics"]["long_leg_atomic_handoffs"] == 2
    assert report["metrics"]["long_leg_wait_planning_handoffs"] == 1
    assert report["metrics"]["controller_sessions"] == 2
    assert report["metrics"]["motor_zero_intervals"][0]["duration_s"] > 0.0
    assert report["metrics"]["decision_reduction_ratio"] == pytest.approx(10.0)
    assert report["short_hop_characterization"]["wait_planning_count"] == 0
    assert report["scope"]["motion_authority"] is False
    assert report["scope"]["physical_execution_enabled"] is False
    assert reopened["status"] == "complete"
    assert reopened["result"]["schema"] == PHASE4_RESULT_SCHEMA
    assert reopened["result"]["mission_id"] == "phase4-persisted-test"
    assert browser["adapter"]["hierarchical_phase4"] is True
    assert browser["adapter"]["read_only"] is True
    assert browser["mission"]["mission_id"] == "phase4-persisted-test"
    assert browser["rolling"]["phase4"]["decisions"]
    assert browser["map"]["frontiers"]
    assert any(
        event["kind"] == "phase4_handoff" for event in reopened["events"]
    )


def test_measured_latency_characterizes_short_hop_without_claiming_motion(
    tmp_path: Path,
) -> None:
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    fixture["short_hop_replay"]["arrival_s"] = 0.002

    report = run_phase4_replay(
        fixture,
        root=ROOT,
        provider=DelayedSemanticProvider([0.004, 0.004, 0.004, 0.004]),
        database=tmp_path / "short.sqlite3",
        source_sha="phase4-short-test",
        mission_id="phase4-short-test",
        poll_s=0.001,
        provider_p95_s=0.01,
        prefetch_margin_s=0.001,
    )

    short = report["short_hop_characterization"]
    assert short["wait_planning_count"] == 4
    assert all(
        sample["state_at_arrival"] == "wait_planning"
        and sample["motor_zero_interval_s"] > 0.0
        for sample in short["samples"]
    )
    assert report["scope"]["motor_zero_intervals_are_replay_derived"] is True


def test_browser_bundle_exposes_phase4_evidence_without_mutation_controls() -> None:
    html = build_mission_web_bundle()["index_html"]

    assert "Phase 4 latency, handoff &amp; pause evidence" in html
    assert "PHASE 4 REAL-PROVIDER EVIDENCE — READ ONLY" in html
    assert "snapshot.adapter.read_only" in html
    assert "shortHop.wait_planning_count" in html
    assert "map.frontiers || []" in html
