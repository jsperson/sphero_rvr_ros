from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request

import pytest

from sphero_rvr_driver.mission_web import (
    WEB_API_VERSION,
    MissionWebError,
    MockReplayMissionAdapter,
    WebMissionState,
    build_mission_web_bundle,
    handle_mission_web_request,
    make_server,
)


PROMPT = "Move forward 20 centimeters, turn left 45 degrees, then move forward 15 centimeters."


def _proposal(adapter: MockReplayMissionAdapter, scenario: str = "success") -> dict:
    return dict(adapter.propose(PROMPT, scenario))


def _approve(adapter: MockReplayMissionAdapter, snapshot: dict) -> dict:
    return dict(adapter.approve(snapshot["approval"]["required_phrase"]))


def test_mock_adapter_exposes_typed_contract_without_live_authority_or_credentials() -> None:
    snapshot = MockReplayMissionAdapter(source_sha="test-web-sha").snapshot()

    assert snapshot["web_api_version"] == WEB_API_VERSION
    assert snapshot["mission_api_version"] == "mission_api.v2"
    assert snapshot["prompt_drive_api_version"] == "prompt_drive.v1"
    assert snapshot["adapter"] == {
        "mode": "mock/replay",
        "fixture_only": True,
        "live_execution_enabled": False,
        "direct_ros_commands_allowed": False,
        "credentials_accepted": False,
        "future_live_boundary": "Pi-hosted authenticated mission service",
    }
    assert snapshot["mission"]["state"] == WebMissionState.READY.value
    assert snapshot["safety"]["independent_robot_safety"] is True
    assert snapshot["safety"]["browser_is_sole_safety_mechanism"] is False
    assert snapshot["map"]["fixture_only"] is True
    assert snapshot["map"]["proposed_route"]
    assert snapshot["map"]["obstacles"]
    assert {item["label"] for item in snapshot["map"]["objects"]} == {"shoe"}


def test_proposal_reuses_prompt_drive_validation_and_requires_exact_digest_approval() -> None:
    adapter = MockReplayMissionAdapter(source_sha="test-web-sha")
    proposed = _proposal(adapter)

    assert proposed["mission"]["state"] == "PROPOSED"
    assert proposed["proposal"]["api_version"] == "prompt_drive.v1"
    assert proposed["proposal"]["prompt"] == PROMPT
    assert proposed["proposal"]["provider_id"] == "mock-replay"
    assert proposed["proposal"]["model_id"] == "fixture-model-not-live"
    assert proposed["proposal"]["source_sha"] == "test-web-sha"
    assert [segment["tool_id"] for segment in proposed["proposal"]["segments"]] == [
        "move_distance",
        "turn_angle",
        "move_distance",
    ]
    assert proposed["approval"]["required_phrase"].startswith("APPROVE ")
    assert proposed["approval"]["proposal_digest"] in proposed["approval"]["required_phrase"]

    with pytest.raises(MissionWebError, match="does not match"):
        adapter.approve("APPROVE wrong")

    running = _approve(adapter, proposed)
    assert running["mission"]["state"] == "RUNNING"
    assert running["approval"]["approved"] is True
    assert running["adapter"]["live_execution_enabled"] is False


def test_success_scenario_advances_progress_map_path_events_and_terminal_result() -> None:
    adapter = MockReplayMissionAdapter()
    running = _approve(adapter, _proposal(adapter))
    assert running["map"]["traveled_path"] == []

    first = adapter.advance()
    second = adapter.advance()
    complete = adapter.advance()

    assert first["mission"]["progress"] == pytest.approx(0.34)
    assert second["mission"]["progress"] == pytest.approx(0.68)
    assert complete["mission"] == {
        "state": "COMPLETE",
        "progress": 1.0,
        "terminal": True,
        "terminal_reason": "target_reached",
    }
    assert len(complete["map"]["traveled_path"]) == len(complete["map"]["proposed_route"])
    assert [event["event_type"] for event in complete["events"]][-2:] == ["progress", "complete"]


@pytest.mark.parametrize(
    ("scenario", "state", "reason", "safety_key", "safety_value"),
    (
        ("cancellation", "CANCELLED", "cancelled", None, None),
        ("stop", "STOPPED", "stop_requested", "stop_active", True),
        ("estop", "ESTOPPED", "estop_latched", "estop_latched", True),
        ("collision_blocked", "BLOCKED", "collision_veto", "collision_state", "BLOCKED"),
        ("stale_telemetry", "BLOCKED", "stale_telemetry", "telemetry_fresh", False),
    ),
)
def test_required_terminal_scenarios_fail_truthfully(
    scenario: str,
    state: str,
    reason: str,
    safety_key: str | None,
    safety_value: object,
) -> None:
    adapter = MockReplayMissionAdapter()
    _approve(adapter, _proposal(adapter, scenario))

    result = adapter.advance()

    assert result["mission"]["state"] == state
    assert result["mission"]["terminal"] is True
    assert result["mission"]["terminal_reason"] == reason
    if safety_key:
        assert result["safety"][safety_key] == safety_value


def test_rejection_has_no_motion_no_approval_and_cannot_start() -> None:
    adapter = MockReplayMissionAdapter()
    rejected = _proposal(adapter, "rejection")

    assert rejected["mission"]["state"] == "REJECTED"
    assert rejected["mission"]["terminal_reason"] == "model_rejected"
    assert rejected["proposal"]["decision"] == "reject"
    assert rejected["proposal"]["segments"] == []
    assert rejected["approval"]["required"] is False
    assert rejected["approval"]["required_phrase"] == ""
    with pytest.raises(MissionWebError, match="executable proposal"):
        adapter.approve("")


def test_explicit_browser_cancel_latches_proposed_or_running_simulation() -> None:
    proposed_adapter = MockReplayMissionAdapter()
    _proposal(proposed_adapter)
    proposed_cancel = proposed_adapter.cancel()
    assert proposed_cancel["mission"]["state"] == "CANCELLED"

    running_adapter = MockReplayMissionAdapter()
    _approve(running_adapter, _proposal(running_adapter))
    running_cancel = running_adapter.cancel()
    assert running_cancel["mission"]["state"] == "CANCELLED"
    assert running_cancel["events"][-1]["event_type"] == "cancelled"


def test_router_exposes_only_bounded_mock_routes_and_rejects_direct_command_paths() -> None:
    adapter = MockReplayMissionAdapter()

    root = handle_mission_web_request("GET", "/", "", adapter)
    scenarios = handle_mission_web_request("GET", "/api/web/scenarios", "", adapter)
    favicon = handle_mission_web_request("GET", "/favicon.ico", "", adapter)
    proposed = handle_mission_web_request(
        "POST",
        "/api/web/mission/propose",
        json.dumps({"prompt": PROMPT, "scenario": "success"}),
        adapter,
    )

    assert root.status == 200
    assert root.content_type == "text/html; charset=utf-8"
    assert "MOCK / REPLAY — NO LIVE EXECUTION" in root.body
    assert len(json.loads(scenarios.body)["scenarios"]) == 7
    assert (favicon.status, favicon.content_type, favicon.body) == (204, "image/x-icon", "")
    assert json.loads(proposed.body)["mission"]["state"] == "PROPOSED"

    for path in ("/api/motor", "/api/ros", "/api/write", "/cmd_vel", "/cmd_vel_motor"):
        with pytest.raises(MissionWebError, match="not exposed"):
            handle_mission_web_request("POST", path, "{}", adapter)
    with pytest.raises(MissionWebError, match="GET route is not exposed"):
        handle_mission_web_request("GET", "/api/mission/start", "", adapter)


def test_static_bundle_is_responsive_accessible_and_has_no_browser_persistence_or_live_adapter() -> None:
    bundle = build_mission_web_bundle(app_name="RVR Test Console")
    page = bundle["index_html"]

    assert bundle["manifest"]["display"] == "standalone"
    assert "RVR Test Console" in page
    assert "@media (max-width:760px)" in page
    assert "grid-template-columns:minmax(0,1fr)" in page
    assert ".map-frame { min-height:0; }" in page
    assert ".segment { flex-direction:column" in page
    assert 'data-testid="mission-prompt"' in page
    assert 'data-testid="scenario"' in page
    assert 'data-testid="approve"' in page
    assert 'aria-label="Fixture room map showing rover, route, path, obstacles, and objects"' in page
    for token in ("Mission prompt", "LLM proposal", "Room map", "Mission status", "Event history", "Authority boundary"):
        assert token in page
    for forbidden in ("localStorage", "sessionStorage", "OPENAI_API_KEY", "CODEX_API_KEY", "WebSocket("):
        assert forbidden not in page


def test_http_wrapper_serves_complete_mock_flow_with_security_headers() -> None:
    server = make_server(port=0, adapter=MockReplayMissionAdapter(source_sha="http-test"))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urllib.request.urlopen(f"{base}/", timeout=2.0) as response:
            assert response.status == 200
            assert response.headers["X-Content-Type-Options"] == "nosniff"
            assert b"RVR Mission Console" in response.read()

        request = urllib.request.Request(
            f"{base}/api/web/mission/propose",
            data=json.dumps({"prompt": PROMPT, "scenario": "success"}).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=2.0) as response:
            proposed = json.loads(response.read())
        assert proposed["mission"]["state"] == "PROPOSED"

        forbidden = urllib.request.Request(f"{base}/api/motor", data=b"{}", method="POST")
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(forbidden, timeout=2.0)
        assert error.value.code == 400
        assert "not exposed" in json.loads(error.value.read())["error"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
    assert not thread.is_alive()
