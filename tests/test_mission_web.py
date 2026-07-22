from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request

import pytest

from sphero_rvr_driver.mission_web import (
    WEB_API_VERSION,
    LiveMissionWebAdapter,
    MissionWebError,
    MockReplayMissionAdapter,
    WebMissionState,
    build_mission_web_bundle,
    handle_mission_web_request,
    make_server,
)
from sphero_rvr_driver.mission_api import MissionValidationError


PROMPT = "Move forward 20 centimeters, turn left 45 degrees, then move forward 15 centimeters."


class FakeLiveMissionClient:
    def __init__(self, *, execution_enabled: bool = False) -> None:
        self.proposal = _proposal(MockReplayMissionAdapter(source_sha="live-source"))["proposal"]
        self.mission = None
        self.submissions = []
        self.execution_enabled = execution_enabled
        self.approvals = []

    def service_snapshot(self):
        return {
            "api_version": "mission_api.v2",
            "mode": "live",
            "source_sha": "live-source",
            "deployed_sha": "live-deployed",
            "planning_enabled": True,
            "live_execution_enabled": self.execution_enabled,
            "capabilities": {
                "query_status_telemetry@1.0": {
                    "evidence": {
                        "safety": {
                            "collision_state": "UNKNOWN",
                            "stop_active": False,
                            "estop_latched": False,
                            "stop_state": "UNKNOWN",
                            "estop_state": "UNKNOWN",
                        },
                        "odom": {"fresh": False, "value": {"x_m": 0.2, "y_m": 0.3, "heading_deg": 4.0}},
                        "collision": {"fresh": False, "value": {}},
                        "route_progress": {"fresh": False, "value": {}},
                        "semantic_map": {"fresh": False, "valid": False, "value": {}},
                    }
                }
            },
        }

    def latest_prompt_status(self, session_id):
        del session_id
        return self.mission

    def submit_prompt(self, prompt, *, session_id, source, mission_id=None):
        del mission_id
        self.submissions.append((prompt, session_id, source))
        self.mission = self._snapshot("planning", proposal={})
        return self.mission

    def prompt_status(self, mission_id):
        assert mission_id == "live-mission"
        if self.mission is not None and self.mission.get("status") == "planning":
            self.mission = self._snapshot("proposed", proposal=self.proposal)
        return self.mission

    def approve_prompt(self, mission_id, *, approval_phrase, operator):
        if not self.execution_enabled:
            raise MissionValidationError("live execution is disabled by reviewed service configuration")
        self.approvals.append((mission_id, approval_phrase, operator))
        self.mission = self._snapshot("approved", proposal=self.proposal)
        self.mission["approval"] = {"approved": True, "operator": operator}
        return self.mission

    def cancel_prompt(self, mission_id, *, reason):
        del mission_id, reason
        self.mission = self._snapshot("cancelled", proposal=self.proposal)
        return self.mission

    @staticmethod
    def _snapshot(status, *, proposal):
        return {
            "mission_id": "live-mission",
            "session_id": "web-session",
            "status": status,
            "proposal": proposal,
            "approval": {},
            "result": {},
            "terminal_reason": "cancelled" if status == "cancelled" else "",
            "events": [
                {
                    "event_id": 1,
                    "kind": "planning" if status == "planning" else status,
                    "payload": {"status": status},
                }
            ],
        }


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


def test_live_adapter_does_not_render_missing_stop_estop_as_ready() -> None:
    snapshot = LiveMissionWebAdapter(FakeLiveMissionClient()).snapshot()

    assert snapshot["safety"]["stop_state"] == "UNKNOWN"
    assert snapshot["safety"]["estop_state"] == "UNKNOWN"
    assert snapshot["approval"]["enabled"] is False


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
        "result": {
            "status": "complete",
            "terminal_reason": "target_reached",
            "evidence_mode": "mock/replay",
        },
    }
    assert complete["artifacts"] == [
        {
            "artifact_id": "terminal-result",
            "label": "Terminal result (JSON)",
            "href": "/api/web/artifacts/terminal-result",
            "media_type": "application/json",
            "fixture_only": True,
        }
    ]
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

    with pytest.raises(MissionWebError, match="terminal mission evidence is not available"):
        handle_mission_web_request("GET", "/api/web/artifacts/terminal-result", "", adapter)
    approved = handle_mission_web_request(
        "POST",
        "/api/web/mission/approve",
        json.dumps({"approval_phrase": json.loads(proposed.body)["approval"]["required_phrase"]}),
        adapter,
    )
    assert json.loads(approved.body)["mission"]["state"] == "RUNNING"
    adapter.advance()
    adapter.advance()
    adapter.advance()
    artifact = handle_mission_web_request(
        "GET", "/api/web/artifacts/terminal-result", "", adapter
    )
    artifact_payload = json.loads(artifact.body)
    assert artifact_payload["state"] == "COMPLETE"
    assert artifact_payload["result"]["terminal_reason"] == "target_reached"

    for path in ("/api/motor", "/api/ros", "/api/write", "/cmd_vel", "/cmd_vel_motor"):
        with pytest.raises(MissionWebError, match="not exposed"):
            handle_mission_web_request("POST", path, "{}", adapter)
    with pytest.raises(MissionWebError, match="GET route is not exposed"):
        handle_mission_web_request("GET", "/api/mission/start", "", adapter)


def test_static_bundle_is_responsive_accessible_and_has_no_browser_persistence() -> None:
    bundle = build_mission_web_bundle(app_name="RVR Test Console")
    page = bundle["index_html"]

    assert bundle["manifest"]["display"] == "standalone"
    assert "RVR Test Console" in page
    assert "@media (max-width:760px)" in page
    assert "grid-template-columns:minmax(0,1fr)" in page
    assert "[hidden] { display:none !important; }" in page
    assert ".map-frame { min-height:0; }" in page
    assert ".segment { flex-direction:column" in page
    assert 'data-testid="mission-prompt"' in page
    assert 'data-testid="scenario"' in page
    assert 'data-testid="approve"' in page
    assert 'aria-label="Fixture room map showing rover, route, path, obstacles, and objects"' in page
    assert "Authoritative live room map unavailable" in page
    assert "The browser uses the Pi-local mission-service boundary" in page
    assert "if (map.available === false)" in page
    for token in (
        "Mission prompt",
        "LLM proposal",
        "Room map",
        "Mission status",
        "Event history",
        "Terminal evidence",
        "Authority boundary",
    ):
        assert token in page
    assert "artifact.href" in page
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


def test_live_adapter_uses_only_service_client_and_shows_truthful_proposal_only_state() -> None:
    client = FakeLiveMissionClient()
    adapter = LiveMissionWebAdapter(client, session_id="web-session", operator="scott@example.com")

    ready = adapter.snapshot()
    assert ready["adapter"] == {
        "mode": "live/proposal-only",
        "fixture_only": False,
        "live_execution_enabled": False,
        "direct_ros_commands_allowed": False,
        "credentials_accepted": False,
        "service_source_sha": "live-source",
        "service_deployed_sha": "live-deployed",
        "boundary": "Pi-local MissionService Unix socket",
    }
    assert ready["mission"]["state"] == "READY"
    assert ready["map"]["available"] is False
    assert ready["map"]["fixture_only"] is False
    assert ready["safety"]["telemetry_fresh"] is False

    planning = adapter.propose(PROMPT, "live")
    assert planning["mission"]["state"] == "PLANNING"
    assert client.submissions == [(PROMPT, "web-session", "web")]
    proposed = adapter.snapshot()
    assert proposed["mission"]["state"] == "PROPOSED"
    assert proposed["proposal"]["provider_id"] == "mock-replay"
    assert proposed["approval"]["required"] is True
    assert proposed["approval"]["enabled"] is False
    assert proposed["approval"]["required_phrase"] == ""
    assert proposed["approval"]["method"] == "authenticated_one_click"
    assert proposed["approval"]["server_digest_bound"] is True
    with pytest.raises(MissionWebError, match="explicit confirmation"):
        adapter.approve("")
    with pytest.raises(MissionWebError, match="execution is disabled"):
        adapter.approve("", confirm_current_proposal=True)
    assert adapter.cancel()["mission"]["state"] == "CANCELLED"


def test_live_terminal_result_is_authoritative_and_downloadable() -> None:
    client = FakeLiveMissionClient()
    client.mission = client._snapshot("complete", proposal=client.proposal)
    client.mission["result"] = {
        "status": "complete",
        "terminal_reason": "target_reached",
        "final_heading_deg": 44.8,
        "left_track_distance_m": 0.101,
        "right_track_distance_m": 0.099,
    }
    client.mission["terminal_reason"] = "target_reached"
    adapter = LiveMissionWebAdapter(client, session_id="web-session", operator="scott")

    snapshot = adapter.snapshot()
    assert snapshot["mission"]["result"]["final_heading_deg"] == pytest.approx(44.8)
    assert snapshot["artifacts"][0]["fixture_only"] is False
    response = handle_mission_web_request(
        "GET", "/api/web/artifacts/terminal-result", "", adapter
    )
    payload = json.loads(response.body)
    assert payload["mission_id"] == "live-mission"
    assert payload["result"]["left_track_distance_m"] == pytest.approx(0.101)


def test_live_partial_result_is_not_linked_before_terminal_state() -> None:
    client = FakeLiveMissionClient()
    client.mission = client._snapshot("running", proposal=client.proposal)
    client.mission["result"] = {"measured_distance_m": 0.02}
    adapter = LiveMissionWebAdapter(client, session_id="web-session", operator="scott")

    snapshot = adapter.snapshot()
    assert snapshot["mission"]["result"] == {"measured_distance_m": 0.02}
    assert snapshot["artifacts"] == []
    with pytest.raises(MissionWebError, match="terminal mission evidence is not available"):
        handle_mission_web_request(
            "GET", "/api/web/artifacts/terminal-result", "", adapter
        )


def test_live_http_requires_exact_same_origin_for_state_changes() -> None:
    adapter = LiveMissionWebAdapter(FakeLiveMissionClient(), session_id="web-session", operator="scott")
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
    try:
        payload = json.dumps({"prompt": PROMPT, "scenario": "live"}).encode()
        missing_origin = urllib.request.Request(
            f"{base}/api/web/mission/propose",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with pytest.raises(urllib.error.HTTPError) as missing_error:
            urllib.request.urlopen(missing_origin, timeout=2.0)
        assert missing_error.value.code == 403

        missing_identity = urllib.request.Request(
            f"{base}/api/web/mission/propose",
            data=payload,
            headers={
                "Content-Type": "application/json",
                "Origin": allowed_origin,
                "Sec-Fetch-Site": "same-origin",
            },
            method="POST",
        )
        with pytest.raises(urllib.error.HTTPError) as identity_error:
            urllib.request.urlopen(missing_identity, timeout=2.0)
        assert identity_error.value.code == 401

        authorized = urllib.request.Request(
            f"{base}/api/web/mission/propose",
            data=payload,
            headers={
                "Content-Type": "application/json",
                "Origin": allowed_origin,
                "Sec-Fetch-Site": "same-origin",
                "Tailscale-User-Login": "scott@example.com",
            },
            method="POST",
        )
        with urllib.request.urlopen(authorized, timeout=2.0) as response:
            planning = json.loads(response.read())
        assert planning["mission"]["state"] == "PLANNING"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
    assert not thread.is_alive()


def test_tailscale_identity_header_becomes_server_owned_approval_identity() -> None:
    client = FakeLiveMissionClient(execution_enabled=True)
    adapter = LiveMissionWebAdapter(client, session_id="web-session", operator="untrusted-fallback")
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
        propose = urllib.request.Request(
            f"{base}/api/web/mission/propose",
            data=json.dumps({"prompt": PROMPT, "scenario": "live"}).encode(),
            headers=headers,
            method="POST",
        )
        with urllib.request.urlopen(propose, timeout=2.0):
            pass
        proposed = adapter.snapshot()
        approve = urllib.request.Request(
            f"{base}/api/web/mission/approve",
            data=json.dumps({"confirm_current_proposal": True}).encode(),
            headers=headers,
            method="POST",
        )
        with urllib.request.urlopen(approve, timeout=2.0):
            pass
        assert client.approvals[0][2] == "scott@example.com"
        assert client.approvals[0][1].startswith("APPROVE ")
        assert proposed["approval"]["proposal_digest"] in client.approvals[0][1]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
