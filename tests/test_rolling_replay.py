from __future__ import annotations

import time

import pytest

from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_web import (
    RollingReplayMissionAdapter,
    build_mission_web_bundle,
)
from sphero_rvr_driver.rolling_replay import (
    RollingIntent,
    RollingReplayEngine,
    ScriptedRollingIntentProvider,
)


MISSION = (
    "Explore this room, map shoes and people, inspect uncertain findings from "
    "another viewpoint, then stop safely."
)


def _wait_terminal(subject, timeout_s: float = 4.0) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = subject.snapshot()
        terminal = (
            snapshot["terminal"]
            if "terminal" in snapshot
            else snapshot["mission"]["terminal"]
        )
        if terminal:
            return snapshot
        time.sleep(0.01)
    raise AssertionError("rolling replay did not reach its deterministic terminal")


def test_rolling_intent_rejects_obstacle_response_that_keeps_center_course() -> None:
    snapshot = {
        "snapshot_id": "snapshot-1",
        "obstacles": {
            "ahead": True,
            "left_clearance_m": 1.2,
            "right_clearance_m": 0.4,
        },
        "camera": {"uncertain_track_id": ""},
        "budgets": {
            "remaining_distance_m": 1.5,
            "remaining_observations": 10,
        },
    }
    with pytest.raises(MissionValidationError, match="clear left corridor"):
        RollingIntent.validated(
            {
                "snapshot_id": "snapshot-1",
                "goal_direction": "forward",
                "steering": 0.0,
                "speed_limit_mps": 0.1,
                "safe_corridor": "center",
                "lease_s": 30.0,
                "observation_focus": "forward",
                "viewpoint": "forward",
                "rationale": "Continue straight.",
            },
            revision=1,
            snapshot=snapshot,
            issued_at_s=1.0,
            provider_id="test",
            model_id="test",
        )


def test_replay_ticks_motion_and_perception_during_delayed_provider_calls() -> None:
    engine = RollingReplayEngine(
        "rolling-test",
        MISSION,
        ScriptedRollingIntentProvider(delay_s=0.07),
        tick_s=0.01,
    )
    try:
        engine.start()
        terminal = _wait_terminal(engine)
    finally:
        engine.close()

    metrics = terminal["metrics"]
    revisions = terminal["intent_revisions"]
    assert terminal["status"] == "blocked"
    assert terminal["terminal_reason"] == "localization_stale"
    assert len(revisions) == 4
    assert metrics["motion_updates_while_llm_in_flight"] >= 3
    assert metrics["perception_updates"] > len(revisions)
    assert metrics["semantic_map_updates"] == metrics["perception_updates"]
    assert metrics["compatible_intent_swaps"] >= 2
    assert metrics["artificial_zero_motion_gaps"] == 0
    assert metrics["obstacle_steering_changed"] is True
    assert metrics["uncertain_evidence_changed_focus"] is True
    assert metrics["continuous_object_tracking"] is True
    assert metrics["continuous_face_tracking"] is True
    assert metrics["stale_localization_stopped"] is True
    assert revisions[2]["steering"] > 0.2
    assert revisions[2]["safe_corridor"] == "left"
    assert revisions[3]["observation_focus"] == "shoe-uncertain-01"
    assert revisions[3]["viewpoint"] == "alternate_left"
    assert terminal["result"]["motion_authority"] is False
    assert terminal["result"]["physical_execution_enabled"] is False


@pytest.mark.parametrize(
    ("method", "status", "reason"),
    (
        ("collision_stop", "blocked", "collision_veto"),
        ("stop", "stopped", "stop_requested"),
        ("estop", "estopped", "estop_latched"),
    ),
)
def test_independent_safety_stops_do_not_wait_for_in_flight_provider(
    method: str, status: str, reason: str
) -> None:
    engine = RollingReplayEngine(
        f"safety-{method}",
        MISSION,
        ScriptedRollingIntentProvider(delay_s=1.0),
        tick_s=0.01,
    )
    try:
        engine.start()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if engine.snapshot()["inference"]["in_flight"]:
                break
            time.sleep(0.005)
        else:
            raise AssertionError("provider call did not enter in-flight state")
        started = time.monotonic()
        getattr(engine, method)()
        terminal = engine.snapshot()
        elapsed = time.monotonic() - started
    finally:
        engine.close()

    assert elapsed < 0.2
    assert terminal["status"] == status
    assert terminal["terminal_reason"] == reason
    assert terminal["inference"]["provider_calls_completed"] == 0
    assert terminal["world_snapshot"]["progress"]["speed_mps"] == 0.0


def test_web_adapter_persists_replay_and_exposes_terminal_evidence(tmp_path) -> None:
    adapter = RollingReplayMissionAdapter(
        ScriptedRollingIntentProvider(delay_s=0.05),
        database=tmp_path / "rolling.sqlite3",
        source_sha="stage-b-test-sha",
        tick_s=0.01,
    )
    try:
        proposed = adapter.propose(MISSION, "rolling_llm_replay")
        assert proposed["mission"]["state"] == "PROPOSED"
        assert proposed["proposal"]["segments"] == []
        assert proposed["adapter"]["mission_service_persistence"] is True
        assert proposed["adapter"]["live_execution_enabled"] is False
        running = adapter.approve(proposed["approval"]["required_phrase"])
        assert running["mission"]["state"] == "RUNNING"

        terminal = _wait_terminal(adapter)
        persisted = adapter._service.prompt_status(  # evidence seam under test
            terminal["mission"]["mission_id"]
        )
    finally:
        adapter.close()

    assert terminal["mission"]["state"] == "BLOCKED"
    assert terminal["mission"]["terminal_reason"] == "localization_stale"
    assert terminal["mission"]["result"]["metrics"]["intent_revision_count"] == 4
    assert terminal["artifacts"][0]["href"].endswith("terminal-result")
    assert persisted["status"] == "blocked"
    assert persisted["result"]["motion_authority"] is False
    assert any(
        event["kind"] == "intent_revision" for event in persisted["events"]
    )
    assert any(event["kind"] == "terminal" for event in persisted["events"])


def test_rolling_replay_proposal_can_cancel_without_starting_provider(tmp_path) -> None:
    adapter = RollingReplayMissionAdapter(
        ScriptedRollingIntentProvider(),
        database=tmp_path / "cancel.sqlite3",
        source_sha="stage-b-cancel-sha",
    )
    try:
        adapter.propose(MISSION, "rolling_llm_replay")
        cancelled = adapter.cancel()
    finally:
        adapter.close()

    assert cancelled["mission"]["state"] == "CANCELLED"
    assert cancelled["rolling"]["inference"]["provider_calls_started"] == 0


def test_browser_bundle_renders_stage_b_evidence_panels() -> None:
    html = build_mission_web_bundle()["index_html"]

    assert "Current finite leased intent" in html
    assert "Asynchronous LLM loop" in html
    assert "Fresh world snapshot & detections" in html
    assert "Motion ticks during LLM" in html
    assert "ROLLING LLM REPLAY — NO MOTION AUTHORITY" in html
