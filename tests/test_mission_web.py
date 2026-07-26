from __future__ import annotations

import json
from pathlib import Path
import threading
import urllib.error
import urllib.request
from types import SimpleNamespace
from typing import Mapping, Optional

import pytest

from sphero_rvr_driver.mission_web import (
    WEB_API_VERSION,
    LiveMissionWebAdapter,
    MissionWebError,
    MockReplayMissionAdapter,
    TELEMETRY_CONTROL_PATH,
    TELEMETRY_UNIT,
    SystemdTelemetryControl,
    WebMissionState,
    build_mission_web_bundle,
    handle_mission_web_request,
    make_server,
    _connect_live_web_adapter,
    _stationary_projection,
)
from sphero_rvr_driver.mission_api import MissionValidationError


PROMPT = "Move forward 20 centimeters, turn left 45 degrees, then move forward 15 centimeters."


class FakeLiveMissionClient:
    def __init__(
        self,
        *,
        execution_enabled: bool = False,
        stationary_perception_enabled: bool = False,
    ) -> None:
        self.proposal = _proposal(MockReplayMissionAdapter(source_sha="live-source"))["proposal"]
        self.mission = None
        self.submissions = []
        self.execution_enabled = execution_enabled
        self.stationary_perception_enabled = stationary_perception_enabled
        self.approvals = []

    def service_snapshot(self):
        return {
            "api_version": "mission_api.v2",
            "mode": "live",
            "source_sha": "live-source",
            "deployed_sha": "live-deployed",
            "planning_enabled": True,
            "live_execution_enabled": self.execution_enabled,
            "stationary_perception_enabled": self.stationary_perception_enabled,
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


class FakeAdaptiveMissionLiveMissionClient(FakeLiveMissionClient):
    def __init__(
        self,
        *,
        planning_ready: bool = True,
        approval_activation_enabled: bool = False,
    ) -> None:
        super().__init__(
            execution_enabled=not approval_activation_enabled
        )
        self.planning_ready = planning_ready
        self.approval_activation_enabled = approval_activation_enabled
        self.lease_requests: list[Optional[float]] = []
        self.objective_updates: list[tuple[str, str, str]] = []
        digest = "d" * 64
        self.proposal = {
            "schema": "sphero_rvr.adaptive_mission_proposal.v1",
            "mission_id": "live-mission",
            "prompt": "Explore the room",
            "interpreted_objective": "Explore reachable free space.",
            "proposal_digest": digest,
            "segments": [],
            "first_intent": {
                "action": "move_distance",
                "distance_m": 0.10,
                "angle_deg": 0.0,
                "rationale": "Probe the fresh clear corridor.",
            },
            "limits": {
                "mission_lease_s": 900.0,
                "max_translation_per_intent_m": 0.25,
                "max_rotation_per_intent_deg": 45.0,
                "linear_speed_mps": 0.10,
                "angular_speed_rad_s": 0.4,
            },
        }

    def submit_prompt(
        self,
        prompt,
        *,
        session_id,
        source,
        mission_id=None,
        mission_lease_s=None,
        operator="",
        authentication_source="",
    ):
        del mission_id
        if (
            self.mission is not None
            and self.mission.get("status") == "running"
        ):
            self.objective_updates.append(
                (prompt, operator, authentication_source)
            )
            self.mission["prompt"] = prompt
            return self.mission
        self.lease_requests.append(mission_lease_s)
        if mission_lease_s is not None:
            self.proposal["limits"]["mission_lease_s"] = float(
                mission_lease_s
            )
        self.submissions.append((prompt, session_id, source))
        self.mission = self._snapshot("planning", proposal={})
        return self.mission

    def service_snapshot(self):
        snapshot = super().service_snapshot()
        snapshot.update(
            {
                "mode": "live/adaptive-mission",
                "adaptive_mission_enabled": True,
                "approval_activation_enabled": (
                    self.approval_activation_enabled
                ),
                "physical_session": {
                    "activation_capable": (
                        self.approval_activation_enabled
                    ),
                    "active": self.execution_enabled,
                    "transitioning": False,
                },
                "provider_id": "openai-codex-oauth",
                "model_id": "gpt-5.6-sol",
                "reasoning_effort": "high",
                "adaptive_mission_readiness": {
                    "ready": self.planning_ready,
                    "reasons": [] if self.planning_ready else ["lidar"],
                    "planning_ready": self.planning_ready,
                    "planning_reasons": (
                        [] if self.planning_ready else ["lidar"]
                    ),
                    "execution_ready": True,
                    "execution_reasons": [],
                    "execution_enabled": True,
                    "motion_authority": False,
                },
            }
        )
        evidence = snapshot["capabilities"]["query_status_telemetry@1.0"][
            "evidence"
        ]
        evidence["odom"]["fresh"] = True
        evidence["collision"]["fresh"] = True
        evidence["safety"].update(
            {
                "collision_state": "CLEAR",
                "stop_state": "READY",
                "estop_state": "CLEAR",
            }
        )
        return snapshot

    def approve_prompt(
        self,
        mission_id,
        *,
        approval_phrase,
        operator,
        authentication_source="",
    ):
        self.approvals.append(
            (
                mission_id,
                approval_phrase,
                operator,
                authentication_source,
            )
        )
        self.mission = self._snapshot("running", proposal=self.proposal)
        self.mission["approval"] = {
            "approved": True,
            "authenticated": True,
            "operator": operator,
            "authentication_source": authentication_source,
        }
        return self.mission


class FakeTelemetryControl:
    def __init__(self) -> None:
        self.active = False
        self.requests: list[tuple[bool, Mapping[str, object]]] = []

    def status(self):
        return {
            "available": True,
            "active": self.active,
            "transitioning": False,
            "state": "active" if self.active else "inactive",
            "detail": "active / running" if self.active else "inactive / dead",
            "unit": TELEMETRY_UNIT,
        }

    def set_active(self, active, *, authority_snapshot):
        self.requests.append((active, authority_snapshot))
        self.active = active
        return self.status()


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


def test_live_adapter_renders_fresh_lidar_navigation_without_fabricating_map_layers() -> None:
    client = FakeLiveMissionClient()
    service = client.service_snapshot()
    evidence = service["capabilities"]["query_status_telemetry@1.0"]["evidence"]
    evidence["localization"] = {
        "fresh": True,
        "valid": True,
        "value": {
            "schema": "sphero_rvr.perception_navigation_result.v1",
            "outcome": "running",
            "localization": {
                "state": "valid",
                "source": "lidar_scan_match",
                "authoritative": True,
                "quality": 0.91,
                "odom_translation_disagreement_m": 0.02,
                "odom_heading_disagreement_rad": 0.03,
                "pose": {
                    "stamp_s": 2.0,
                    "frame_id": "map",
                    "x_m": -0.2,
                    "y_m": 0.1,
                    "yaw_rad": 0.0,
                    "heading_deg": 0.0,
                },
            },
            "goal": {
                "frame_id": "map",
                "x_m": 0.4,
                "y_m": 0.1,
                "radius_m": 0.08,
            },
            "next_horizon": {
                "kind": "translate",
                "distance_m": 0.15,
            },
            "path": [
                {"x_m": -0.3, "y_m": 0.1},
                {"x_m": -0.2, "y_m": 0.1},
            ],
        },
    }
    client.service_snapshot = lambda: service

    snapshot = LiveMissionWebAdapter(client).snapshot()
    live_map = snapshot["map"]

    assert live_map["available"] is True
    assert live_map["navigation_available"] is True
    assert live_map["occupancy_available"] is False
    assert live_map["semantic_objects_available"] is False
    assert live_map["unavailable_layers"] == ["occupancy", "semantic_objects"]
    assert live_map["fixture_only"] is False
    assert live_map["rover"] == {"x_m": -0.2, "y_m": 0.1, "yaw_deg": 0.0}
    assert live_map["goal_region"]["x_m"] == pytest.approx(0.4)
    assert live_map["proposed_route"][-1]["x_m"] == pytest.approx(-0.05)
    assert live_map["traveled_path"][0]["x_m"] == pytest.approx(-0.3)
    assert live_map["localization"]["quality"] == pytest.approx(0.91)
    assert live_map["localization"]["fresh"] is True
    assert live_map["objects"] == []


def test_live_semantic_map_preserves_localization_freshness_for_sensor_ui() -> None:
    client = FakeLiveMissionClient()
    service = client.service_snapshot()
    evidence = service["capabilities"]["query_status_telemetry@1.0"]["evidence"]
    evidence["localization"] = {
        "fresh": True,
        "valid": True,
        "value": {
            "state": "valid",
            "quality": 0.8,
            "source": "slam_toolbox_stationary",
        },
    }
    evidence["semantic_map"] = {
        "fresh": True,
        "valid": True,
        "value": {
            "map": {
                "bounds": {
                    "origin": {"x_m": -1.0, "y_m": -1.0},
                    "width_m": 2.0,
                    "height_m": 2.0,
                },
                "rover": {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0},
                "proposed_route": [],
                "traveled_path": [],
                "obstacles": [],
                "objects": [],
            }
        },
    }
    client.service_snapshot = lambda: service

    live_map = LiveMissionWebAdapter(client).snapshot()["map"]

    assert live_map["available"] is True
    assert live_map["localization"] == {
        "state": "valid",
        "quality": 0.8,
        "source": "slam_toolbox_stationary",
        "fresh": True,
    }


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


def test_stationary_sensor_route_controls_only_no_motion_stationary_perception_service() -> None:
    adapter = LiveMissionWebAdapter(
        FakeLiveMissionClient(stationary_perception_enabled=True)
    )
    telemetry_control = FakeTelemetryControl()

    ready = json.loads(
        handle_mission_web_request(
            "GET",
            "/api/web/state",
            "",
            adapter,
            telemetry_control,
        ).body
    )
    assert ready["telemetry_control"] == {
        "active": False,
        "available": True,
        "detail": "inactive / dead",
        "start_permitted": True,
        "state": "inactive",
        "transitioning": False,
        "unit": TELEMETRY_UNIT,
    }
    assert ready["adapter"]["motion_authority"] is False
    assert ready["adapter"]["physical_execution_enabled"] is False

    started = json.loads(
        handle_mission_web_request(
            "POST",
            TELEMETRY_CONTROL_PATH,
            json.dumps({"active": True}),
            adapter,
            telemetry_control,
        ).body
    )
    assert started["telemetry_control"]["active"] is True
    assert telemetry_control.requests[0][0] is True
    assert telemetry_control.requests[0][1]["adapter"]["live_execution_enabled"] is False

    stopped = json.loads(
        handle_mission_web_request(
            "POST",
            TELEMETRY_CONTROL_PATH,
            json.dumps({"active": False}),
            adapter,
            telemetry_control,
        ).body
    )
    assert stopped["telemetry_control"]["active"] is False
    assert [request[0] for request in telemetry_control.requests] == [True, False]


def test_active_adaptive_lease_owns_telemetry_until_terminal_shutdown() -> None:
    adapter = LiveMissionWebAdapter(
        FakeAdaptiveMissionLiveMissionClient(
            approval_activation_enabled=False
        )
    )
    telemetry_control = FakeTelemetryControl()

    snapshot = json.loads(
        handle_mission_web_request(
            "GET",
            "/api/web/state",
            "",
            adapter,
            telemetry_control,
        ).body
    )

    assert snapshot["adapter"]["physical_session"]["active"] is True
    assert snapshot["telemetry_control"]["active"] is True
    assert snapshot["telemetry_control"]["managed_by_lease"] is True
    assert snapshot["telemetry_control"]["unit"] == (
        "rvr-adaptive-mission.service"
    )
    assert snapshot["telemetry_control"]["start_permitted"] is False
    assert "shuts down automatically when the lease ends" in (
        snapshot["telemetry_control"]["detail"]
    )

    with pytest.raises(MissionWebError, match="managed by the active mission lease"):
        handle_mission_web_request(
            "POST",
            TELEMETRY_CONTROL_PATH,
            json.dumps({"active": False}),
            adapter,
            telemetry_control,
        )
    assert telemetry_control.requests == []


def test_stationary_sensor_route_rejects_unconfigured_and_non_boolean_requests() -> None:
    adapter = LiveMissionWebAdapter(
        FakeLiveMissionClient(stationary_perception_enabled=True)
    )
    with pytest.raises(MissionWebError, match="not available"):
        handle_mission_web_request(
            "POST",
            TELEMETRY_CONTROL_PATH,
            json.dumps({"active": True}),
            adapter,
        )
    with pytest.raises(MissionWebError, match="boolean active"):
        handle_mission_web_request(
            "POST",
            TELEMETRY_CONTROL_PATH,
            json.dumps({"active": "true"}),
            adapter,
            FakeTelemetryControl(),
        )


def test_systemd_telemetry_control_is_fixed_and_fails_closed_on_authority(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        if command[:2] == ["ps", "-eo"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if command[0] == "fuser":
            return SimpleNamespace(returncode=1, stdout="", stderr="")
        if "show" in command:
            return SimpleNamespace(
                returncode=0,
                stdout="LoadState=loaded\nActiveState=active\nSubState=running\n",
                stderr="",
            )
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr("sphero_rvr_driver.mission_web.subprocess.run", fake_run)
    with pytest.raises(MissionWebError, match="only the fixed telemetry"):
        SystemdTelemetryControl("rvr-mission-service.service")

    control = SystemdTelemetryControl()
    unsafe_snapshot = LiveMissionWebAdapter(
        FakeLiveMissionClient(execution_enabled=True)
    ).snapshot()
    with pytest.raises(MissionWebError, match="motion authority are disabled"):
        control.set_active(True, authority_snapshot=unsafe_snapshot)
    assert calls == []

    safe_snapshot = LiveMissionWebAdapter(
        FakeLiveMissionClient(stationary_perception_enabled=True)
    ).snapshot()
    status = control.set_active(True, authority_snapshot=safe_snapshot)
    assert status["active"] is True
    assert calls[0][0] == ["ps", "-eo", "pid=,args="]
    assert calls[1][0] == ["fuser", "/dev/ttyAMA0"]
    assert calls[2][0] == [
        "systemctl",
        "--user",
        "start",
        TELEMETRY_UNIT,
    ]
    assert calls[3][0][-1] == TELEMETRY_UNIT
    assert calls[3][1]["timeout"] == pytest.approx(3.0)


def test_systemd_telemetry_control_refuses_detected_motion_process(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def fake_run(command, **kwargs):
        del kwargs
        if command[:2] == ["ps", "-eo"]:
            return SimpleNamespace(
                returncode=0,
                stdout="123 ros2 run sphero_rvr_driver live_route_runner\n",
                stderr="",
            )
        raise AssertionError(f"unexpected command after motion conflict: {command}")

    monkeypatch.setattr("sphero_rvr_driver.mission_web.subprocess.run", fake_run)
    safe_snapshot = LiveMissionWebAdapter(
        FakeLiveMissionClient(stationary_perception_enabled=True)
    ).snapshot()
    with pytest.raises(MissionWebError, match="motion process is present"):
        SystemdTelemetryControl().set_active(
            True,
            authority_snapshot=safe_snapshot,
        )


def test_systemd_telemetry_stop_requires_clean_unit_descendants_and_lidar_handle(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        if command[:3] == ["systemctl", "--user", "stop"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if command[:3] == ["systemctl", "--user", "reset-failed"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if command[:3] == ["systemctl", "--user", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout=(
                    "LoadState=loaded\n"
                    "ActiveState=inactive\n"
                    "SubState=dead\n"
                    "Result=success\n"
                    "MainPID=0\n"
                    "ControlPID=0\n"
                ),
                stderr="",
            )
        if command[:2] == ["ps", "-eo"]:
            return SimpleNamespace(
                returncode=0,
                stdout="123 /usr/bin/live_mission_service\n",
                stderr="",
            )
        if command == ["fuser", "/dev/rplidar"]:
            return SimpleNamespace(returncode=1, stdout="", stderr="")
        raise AssertionError(f"unexpected shutdown command: {command}")

    monkeypatch.setattr("sphero_rvr_driver.mission_web.subprocess.run", fake_run)
    safe_snapshot = LiveMissionWebAdapter(
        FakeLiveMissionClient(stationary_perception_enabled=True)
    ).snapshot()

    status = SystemdTelemetryControl().set_active(
        False,
        authority_snapshot=safe_snapshot,
    )

    assert status["state"] == "inactive"
    assert status["verified_stopped"] is True
    assert calls[0][0] == [
        "systemctl",
        "--user",
        "stop",
        "rvr-adaptive-mission.service",
        TELEMETRY_UNIT,
    ]
    assert calls[1][0] == [
        "systemctl",
        "--user",
        "reset-failed",
        TELEMETRY_UNIT,
    ]
    assert [call[0] for call in calls].count(["fuser", "/dev/rplidar"]) == 1
    assert any(call[0][:2] == ["ps", "-eo"] for call in calls)


def test_systemd_telemetry_stop_retains_degraded_state_when_unit_does_not_stop_cleanly(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def fake_run(command, **kwargs):
        del kwargs
        if command[:3] == ["systemctl", "--user", "stop"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if command[:3] == ["systemctl", "--user", "reset-failed"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if command[:3] == ["systemctl", "--user", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout=(
                    "LoadState=loaded\n"
                    "ActiveState=failed\n"
                    "SubState=failed\n"
                    "Result=exit-code\n"
                    "MainPID=0\n"
                    "ControlPID=0\n"
                ),
                stderr="",
            )
        raise AssertionError(f"unexpected degraded shutdown command: {command}")

    monkeypatch.setattr("sphero_rvr_driver.mission_web.subprocess.run", fake_run)
    control = SystemdTelemetryControl()
    safe_snapshot = LiveMissionWebAdapter(
        FakeLiveMissionClient(stationary_perception_enabled=True)
    ).snapshot()

    with pytest.raises(MissionWebError, match="did not reach"):
        control.set_active(False, authority_snapshot=safe_snapshot)

    status = control.status()
    assert status["state"] == "degraded"
    assert status["verified_stopped"] is False
    assert "Shutdown degraded" in status["detail"]


def test_telemetry_unit_explicitly_stops_motor_before_sigint_shutdown() -> None:
    unit = (
        Path(__file__).resolve().parents[1]
        / "systemd"
        / "user"
        / "rvr-telemetry.service"
    ).read_text()

    assert "ros2 service call /stop_motor std_srvs/srv/Empty" in unit
    assert "KillMode=mixed" in unit
    assert "KillSignal=SIGINT" in unit
    assert "TimeoutStopSec=25" in unit


def test_static_bundle_is_responsive_accessible_and_has_no_browser_persistence() -> None:
    bundle = build_mission_web_bundle(app_name="RVR Test Console")
    page = bundle["index_html"]

    assert bundle["manifest"]["display"] == "standalone"
    assert "RVR Test Console" in page
    assert "@media (max-width:1100px)" in page
    assert "grid-template-columns:minmax(0,1fr)" in page
    assert "[hidden] { display:none !important; }" in page
    assert ".map-frame { min-height:0; aspect-ratio:4/3; }" in page
    assert ".workspace { display:grid; grid-template-columns:minmax(0,2.35fr)" in page
    assert ".ops-sidebar { position:sticky" in page
    assert ".camera-frame { aspect-ratio:4/3; }" in page
    assert "object-fit:contain" in page
    assert ".segment { flex-direction:column" in page
    assert 'data-testid="mission-prompt"' in page
    assert 'data-testid="scenario"' in page
    assert 'data-testid="approve"' in page
    assert 'data-testid="lease-duration-minutes"' in page
    assert page.index('data-testid="lease-duration-minutes"') < page.index(
        'data-testid="approve"'
    )
    assert "Lease minutes (max ${leaseMinutesText(maximum)})" in page
    assert "Duration changed: generate a new proposal" in page
    assert "Objective updated. The active lease" in page
    assert 'aria-label="Fixture room map showing rover, route, path, obstacles, and objects"' in page
    assert "Authoritative live room map unavailable" in page
    assert "The browser uses the Pi-local mission-service boundary" in page
    assert "No code or hash entry is required" in page
    assert "let hydratedMissionId = null;" in page
    assert "if (!promptDirty) $('mission-prompt').value = proposal.prompt || '';" in page
    assert "$('mission-prompt').addEventListener('input'" in page
    assert 'id="safety-corridor"' in page
    assert 'id="safety-trajectory"' in page
    assert "trajectory_min_clearance_m" in page
    assert "trajectory_horizon_s" in page
    assert "left_clearance_m" in page
    assert "right_clearance_m" in page
    assert "forward_corridor_clearance_m" in page
    assert "if (map.available === false)" in page
    assert "Unavailable layers:" in page
    assert "function shouldContinuouslyPoll(snapshot)" in page
    assert "Boolean(snapshot.adapter.stationary_perception)" in page
    assert "if (!livePoll) stopTimer();" in page
    assert "$('request-error').textContent = '';" in page
    assert "const origin = bounds.origin" in page
    assert "STALE LOCALIZATION" in page
    assert "STALE CAMERA EVIDENCE" in page
    assert "CAMERA INTERRUPTED" in page
    assert "empty.hidden = hasPixels || state === 'interrupted';" in page
    assert 'id="telemetry-toggle"' in page
    assert "Turn telemetry on" in page
    assert "Turn telemetry off" in page
    assert "Telemetry lease-managed" in page
    assert "leaseDurationLabel(snapshot)" in page
    assert "const sensorDataFresh = Boolean(" in page
    assert "snapshot.map.localization.fresh" in page
    assert "['failed','degraded'].includes" in page
    assert "/api/web/telemetry" in page
    assert 'id="request-status" role="status" aria-live="polite"' in page
    assert 'id="approval-state" role="status" aria-live="polite"' in page
    assert "Generating and persisting the proposal" in page
    assert "Starting stationary snapshots and the leased LLM observation-intent loop" in page
    assert "Connection interrupted:" in page
    assert "Retrying automatically" in page
    assert "motor, camera, and sensor processes verified stopped" in page
    assert "Frame pixels not supplied" in page
    assert "detection-box" in page
    assert 'id="safety-authority"' in page
    assert "physicalAdaptiveMission ? 'PHYSICAL LOCKED'" in page
    assert page.index('aria-label="Safety state"') < page.index('id="map-heading"')
    assert page.index('id="map-heading"') < page.index('id="camera-heading"')
    assert page.index('id="camera-heading"') < page.index('id="mission-heading"')
    assert page.index('id="status-heading"') < page.index('id="mission-heading"')
    assert page.index('id="mission-heading"') < page.index('id="mission-log-heading"')
    assert 'id="rolling-intent-panel"' not in page
    assert 'id="rolling-loop-panel"' not in page
    assert 'id="proposal-heading"' not in page
    for token in (
        "Mission prompt",
        "Live spatial map",
        "Camera",
        "Mission status",
        "Mission log",
        "Terminal evidence",
        "Authority boundary",
    ):
        assert token in page
    assert "Event history" not in page
    assert ">Explore and map the room</textarea>" in page
    assert "English instruction → fresh perception → supervised movement" in page
    assert "artifact.href" in page
    for forbidden in ("localStorage", "sessionStorage", "OPENAI_API_KEY", "CODEX_API_KEY", "WebSocket("):
        assert forbidden not in page


def test_live_camera_preview_preserves_truthful_frame_freshness_and_overlay_metadata() -> None:
    client = FakeLiveMissionClient()
    service = client.service_snapshot()
    evidence = service["capabilities"]["query_status_telemetry@1.0"]["evidence"]
    evidence["camera"] = {
        "present": True,
        "valid": True,
        "fresh": False,
        "age_s": 2.4,
        "received_at_s": 1_772_000_001.0,
        "source_timestamp_s": 1_772_000_000.5,
        "error": "",
        "value": {
            "frame_id": "live-camera-00000042",
            "stamp_s": 1_772_000_000.5,
            "width": 800,
            "height": 600,
            "thumbnail_data_url": "data:image/jpeg;base64,ZmFrZQ==",
            "uncertain_track_id": "track-uncertain",
            "detections": [
                {
                    "detection_id": "detection-42",
                    "track_id": "track-uncertain",
                    "label": "possible_shoe",
                    "confidence": 0.58,
                    "bbox": {"x": 100, "y": 120, "width": 200, "height": 160},
                }
            ],
            "tracks": [{"track_id": "track-uncertain", "uncertainty_m": 0.31}],
        },
    }
    client.service_snapshot = lambda: service

    snapshot = LiveMissionWebAdapter(client).snapshot()
    camera = snapshot["camera_preview"]

    assert camera["available"] is True
    assert camera["fresh"] is False
    assert camera["state"] == "stale"
    assert camera["age_s"] == pytest.approx(2.4)
    assert camera["frame_id"] == "live-camera-00000042"
    assert camera["width"] == 800
    assert camera["height"] == 600
    assert camera["uncertain_track_id"] == "track-uncertain"
    assert camera["detections"][0]["bbox"]["width"] == 200


def test_http_wrapper_serves_complete_mock_flow_with_security_headers() -> None:
    server = make_server(port=0, adapter=MockReplayMissionAdapter(source_sha="http-test"))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urllib.request.urlopen(f"{base}/", timeout=2.0) as response:
            assert response.status == 200
            assert response.headers["X-Content-Type-Options"] == "nosniff"
            assert "img-src 'self' data:" in response.headers["Content-Security-Policy"]
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
        "approval_activation_enabled": False,
        "physical_session": {},
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


def test_live_web_startup_retries_a_stale_socket_until_owner_accepts() -> None:
    client = FakeLiveMissionClient()
    original = client.service_snapshot
    attempts = 0

    def starting_service():
        nonlocal attempts
        attempts += 1
        if attempts < 3:
            raise ConnectionRefusedError("stale socket")
        return original()

    client.service_snapshot = starting_service
    adapter = _connect_live_web_adapter(
        client,
        session_id="web-session",
        operator="scott@example.com",
        timeout_s=1.0,
    )

    assert attempts == 3
    assert adapter.snapshot()["mission"]["state"] == "READY"


def test_live_adaptive_mission_web_renders_loop_and_binds_tailscale_approval() -> None:
    client = FakeAdaptiveMissionLiveMissionClient()
    client.mission = client._snapshot("proposed", proposal=client.proposal)
    client.mission["result"] = {
        "schema": "sphero_rvr.adaptive_mission_result.v1",
        "mission_id": "live-mission",
        "status": "running",
        "world_snapshot": {
            "snapshot_id": "snapshot-2",
            "version": 2,
            "evidence": {
                "scan_fresh": True,
                "transform_fresh": True,
                "odometry_fresh": True,
            },
            "safety": {"collision_state": "CLEAR"},
        },
        "world_snapshots": [
            {"snapshot_id": "snapshot-1"},
            {"snapshot_id": "snapshot-2"},
        ],
        "active_intent": {
            "revision": 2,
            "snapshot_id": "snapshot-2",
            "action": "turn_angle",
            "distance_m": 0.0,
            "angle_deg": 30.0,
            "lease_s": 5.0,
            "timeout_s": 5.0,
            "observation_focus": "left corridor",
            "rationale": "The new snapshot favors a left look.",
        },
        "intent_revisions": [
            {
                "revision": 1,
                "action": "move_distance",
                "distance_m": 0.10,
                "angle_deg": 0.0,
                "rationale": "Probe forward.",
                "execution": {
                    "outcome": "completed",
                    "movement": {
                        "requested": {
                            "linear_mps": 0.10,
                            "angular_rad_s": 0.0,
                        },
                        "supervised": {
                            "linear_mps": 0.04,
                            "angular_rad_s": 0.0,
                        },
                    },
                },
            }
        ],
        "inference": {
            "in_flight": False,
            "provider_calls_started": 2,
            "provider_calls_completed": 2,
        },
        "metrics": {
            "intent_revision_count": 1,
            "completed_intents": 1,
        },
        "mission_lease": {
            "duration_s": 900.0,
            "remaining_s": 850.0,
        },
        "motion_authority": False,
    }
    adapter = LiveMissionWebAdapter(
        client,
        session_id="web-session",
        operator="untrusted-fallback",
    )

    proposed = adapter.snapshot()
    assert proposed["adapter"]["mode"] == "live/adaptive-mission"
    assert proposed["adapter"]["physical_execution_enabled"] is True
    assert proposed["adapter"]["motion_authority"] is False
    assert proposed["approval"]["enabled"] is True
    assert proposed["rolling"]["active_intent"]["action"] == "turn_angle"
    assert len(proposed["rolling"]["world_snapshots"]) == 2
    movement = proposed["rolling"]["intent_revisions"][0]["execution"][
        "movement"
    ]
    assert movement["requested"]["linear_mps"] == 0.10
    assert movement["supervised"]["linear_mps"] == 0.04

    with pytest.raises(MissionWebError, match="authenticated Tailscale"):
        adapter.approve("", confirm_current_proposal=True)
    adapter.set_request_identity("scott@example.com", authenticated=True)
    approved = adapter.approve("", confirm_current_proposal=True)

    assert approved["mission"]["state"] == "RUNNING"
    assert client.approvals == [
        (
            "live-mission",
            f"APPROVE ADAPTIVE MISSION {client.proposal['proposal_digest']}",
            "scott@example.com",
            "tailscale-serve",
        )
    ]


def test_live_adaptive_mission_web_never_submits_predictably_stale_planning() -> None:
    client = FakeAdaptiveMissionLiveMissionClient(planning_ready=False)
    adapter = LiveMissionWebAdapter(
        client,
        session_id="web-session",
        operator="scott@example.com",
    )

    snapshot = adapter.snapshot()
    assert snapshot["planning"] == {
        "enabled": True,
        "ready": False,
        "reasons": ["lidar"],
    }
    with pytest.raises(
        MissionWebError,
        match=(
            "planning is locked until camera, lidar, and localization "
            "evidence are fresh: lidar"
        ),
    ):
        adapter.propose("Explore and map the room", "live")
    assert client.submissions == []

    page = str(build_mission_web_bundle()["index_html"])
    assert "Planning locked: waiting for fresh camera, lidar, and localization" in page
    assert "planning.ready !== true" in page


def test_live_adaptive_mission_approval_activation_stages_without_stale_model_call() -> None:
    client = FakeAdaptiveMissionLiveMissionClient(
        planning_ready=False,
        approval_activation_enabled=True,
    )
    adapter = LiveMissionWebAdapter(
        client,
        session_id="web-session",
        operator="untrusted-fallback",
    )

    ready = adapter.snapshot()
    assert ready["adapter"]["live_execution_enabled"] is False
    assert ready["adapter"]["approval_activation_enabled"] is True
    assert ready["approval"]["enabled"] is False

    planning = adapter.propose("Explore the room", "live")
    assert planning["mission"]["state"] == "PLANNING"
    assert client.submissions == [
        ("Explore the room", "web-session", "web")
    ]
    proposed = adapter.snapshot()
    assert proposed["mission"]["state"] == "PROPOSED"
    assert proposed["approval"]["enabled"] is True

    with pytest.raises(
        MissionWebError, match="authenticated Tailscale"
    ):
        adapter.approve("", confirm_current_proposal=True)
    adapter.set_request_identity(
        "scott@example.com", authenticated=True
    )
    approved = adapter.approve(
        "", confirm_current_proposal=True
    )
    assert approved["mission"]["state"] == "RUNNING"
    assert client.approvals[-1][2:] == (
        "scott@example.com",
        "tailscale-serve",
    )

    page = str(build_mission_web_bundle()["index_html"])
    assert "approval starts the supervised graph" in page
    assert "Activating the supervised graph and waiting for fresh" in page


def test_live_adaptive_lease_duration_and_objective_update_use_one_active_lease() -> None:
    client = FakeAdaptiveMissionLiveMissionClient(
        planning_ready=False,
        approval_activation_enabled=True,
    )
    adapter = LiveMissionWebAdapter(
        client,
        session_id="web-session",
        operator="untrusted-fallback",
    )
    adapter.set_request_identity(
        "scott@example.com", authenticated=True
    )

    planning = adapter.propose(
        "Explore the room",
        "live",
        mission_lease_s=120.0,
    )
    assert planning["mission"]["state"] == "PLANNING"
    assert client.lease_requests == [120.0]
    proposed = adapter.snapshot()
    assert proposed["proposal"]["limits"]["mission_lease_s"] == 120.0
    adapter.approve("", confirm_current_proposal=True)

    updated = adapter.propose(
        "Inspect the open area on the left",
        "live",
    )

    assert updated["mission"]["state"] == "RUNNING"
    assert updated["mission"]["mission_id"] == "live-mission"
    assert client.objective_updates == [
        (
            "Inspect the open area on the left",
            "scott@example.com",
            "tailscale-serve",
        )
    ]


def test_web_router_rejects_non_numeric_lease_duration() -> None:
    adapter = MockReplayMissionAdapter()

    with pytest.raises(
        MissionWebError, match="mission_lease_s must be a JSON number"
    ):
        handle_mission_web_request(
            "POST",
            "/api/web/mission/propose",
            json.dumps(
                {
                    "prompt": PROMPT,
                    "scenario": "success",
                    "mission_lease_s": "15",
                }
            ),
            adapter,
        )


def test_adaptive_mission_terminal_projection_preserves_truthful_loop_metrics() -> None:
    result = {
        "provider": {
            "calls_started": 4,
            "calls_completed": 4,
        },
        "final_snapshot": {
            "snapshot_id": "terminal-snapshot",
            "progress": {
                "cumulative_translation_m": 0.11733939294669324,
                "cumulative_rotation_deg": 26.132662180601223,
            },
        },
        "intent_revisions": [
            {
                "revision": revision,
                "execution": {"outcome": "completed"},
            }
            for revision in range(1, 5)
        ],
    }

    projection = _stationary_projection(result)

    assert projection["world_snapshot"]["snapshot_id"] == "terminal-snapshot"
    assert projection["inference"] == {
        "call": None,
        "in_flight": False,
        "provider_calls_started": 4,
        "provider_calls_completed": 4,
    }
    assert projection["metrics"] == {
        "intent_revision_count": 4,
        "completed_intents": 4,
        "cumulative_translation_m": pytest.approx(0.11733939294669324),
        "cumulative_rotation_deg": pytest.approx(26.132662180601223),
    }


def test_locked_restart_preserves_adaptive_mission_terminal_browser_projection() -> None:
    client = FakeLiveMissionClient(execution_enabled=False)
    client.mission = client._snapshot("complete", proposal={})
    client.mission["result"] = {
        "schema": "sphero_rvr.adaptive_mission_result.v1",
        "status": "complete",
        "terminal_reason": "planner_stop",
        "provider": {
            "calls_started": 4,
            "calls_completed": 4,
        },
        "final_snapshot": {
            "snapshot_id": "terminal-snapshot",
            "progress": {
                "cumulative_translation_m": 0.11733939294669324,
                "cumulative_rotation_deg": 26.132662180601223,
            },
        },
        "intent_revisions": [
            {
                "revision": revision,
                "execution": {"outcome": "completed"},
            }
            for revision in range(1, 5)
        ],
    }
    client.mission["terminal_reason"] = "planner_stop"

    snapshot = LiveMissionWebAdapter(
        client,
        session_id="web-session",
        operator="scott",
    ).snapshot()

    assert snapshot["adapter"]["adaptive_mission"] is True
    assert snapshot["adapter"]["rolling_replay"] is True
    assert snapshot["adapter"]["live_execution_enabled"] is False
    assert snapshot["adapter"]["physical_execution_enabled"] is False
    assert snapshot["approval"]["enabled"] is False
    assert snapshot["mission"]["state"] == "COMPLETE"
    assert snapshot["mission"]["terminal_reason"] == "planner_stop"
    assert snapshot["rolling"]["metrics"] == {
        "intent_revision_count": 4,
        "completed_intents": 4,
        "cumulative_translation_m": pytest.approx(0.11733939294669324),
        "cumulative_rotation_deg": pytest.approx(26.132662180601223),
    }
    assert snapshot["rolling"]["inference"] == {
        "call": None,
        "in_flight": False,
        "provider_calls_started": 4,
        "provider_calls_completed": 4,
    }


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
