from __future__ import annotations

import json

import pytest

from sphero_rvr_driver.mission_api import build_canonical_shoe_mapping_plan
from sphero_rvr_driver.mission_controls import MissionControlSession, MissionExecutionMode, MissionPrincipal
from sphero_rvr_driver.mission_observability import (
    ReadOnlyRouteError,
    build_mock_observability_snapshot,
    build_static_pwa_bundle,
    handle_observability_request,
    iter_event_stream_frames,
)


def _operator() -> MissionPrincipal:
    return MissionPrincipal("operator:scott", permissions=("mission:start", "mission:pause"))


def _mission_session() -> MissionControlSession:
    session = MissionControlSession(build_canonical_shoe_mapping_plan(goal_id="obs-001"))
    session.start(_operator(), mode=MissionExecutionMode.REPLAY)
    return session


def test_observability_snapshot_serializes_canonical_contract_read_only_status_surfaces() -> None:
    snapshot = build_mock_observability_snapshot(_mission_session().snapshot())
    payload = snapshot.to_json_dict()

    assert payload["api_version"] == "mission_api.v2"
    assert payload["mission"]["mission_id"] == "obs-001"
    assert payload["mission"]["state"] == "RUNNING"
    assert payload["read_only"] is True
    assert payload["allowed_methods"] == ["GET"]
    assert payload["write_endpoints"] == []
    assert payload["robot_health"]["summary"] == "mock/replay"
    assert payload["sensor_freshness"]["lidar_scan"]["fresh"] is True
    assert payload["battery"]["percentage"] == pytest.approx(0.76)
    assert payload["diagnostics"][0]["level"] == "OK"
    assert payload["pose"]["frame"] == "map"
    assert payload["map_preview"]["occupancy_map_ref"].endswith("shoe_room_map.yaml")
    assert payload["camera_preview"]["stream_available"] is False
    assert payload["semantic_markers"][0]["label"] == "shoe"
    assert payload["mission_events"] == ["start_requested", "validated"]
    assert set(payload["artifact_links"]) == {"occupancy_map", "semantic_map", "shoe_detections"}


def test_event_stream_frames_are_json_sse_for_mock_telemetry_progression() -> None:
    session = _mission_session()
    running = build_mock_observability_snapshot(session.snapshot())
    paused = build_mock_observability_snapshot(session.pause(_operator()))

    frames = list(iter_event_stream_frames((running, paused)))

    assert len(frames) == 2
    assert frames[0].startswith("event: mission_observability\n")
    first_data = json.loads(frames[0].split("data: ", 1)[1])
    second_data = json.loads(frames[1].split("data: ", 1)[1])
    assert first_data["mission"]["state"] == "RUNNING"
    assert second_data["mission"]["state"] == "PAUSED"
    assert second_data["sensor_freshness"]["mission_state"]["fresh"] is True


def test_static_pwa_bundle_is_responsive_read_only_and_contains_observability_hooks() -> None:
    bundle = build_static_pwa_bundle(app_name="RVR Mission Watch")

    assert bundle.manifest["display"] == "standalone"
    assert bundle.manifest["name"] == "RVR Mission Watch"
    assert "viewport" in bundle.index_html
    assert "@media (max-width: 720px)" in bundle.index_html
    for token in [
        "Robot health",
        "Sensor freshness",
        "Battery & diagnostics",
        "Pose / map preview",
        "Camera preview",
        "Semantic markers",
        "Mission events",
        "Final artifacts",
    ]:
        assert token in bundle.index_html
    forbidden_write_tokens = ("Start mission", "Cancel mission", "cmd_vel", "motor command", "POST")
    assert not any(token in bundle.index_html for token in forbidden_write_tokens)


def test_observability_request_handler_exposes_only_get_routes_and_rejects_write_attempts() -> None:
    snapshot = build_mock_observability_snapshot(_mission_session().snapshot())

    status, content_type, body = handle_observability_request("GET", "/api/observability", snapshot)
    assert status == 200
    assert content_type == "application/json"
    assert json.loads(body)["read_only"] is True

    html_status, html_type, html_body = handle_observability_request("GET", "/", snapshot)
    assert html_status == 200
    assert html_type == "text/html; charset=utf-8"
    assert "RVR Mission Observability" in html_body

    with pytest.raises(ReadOnlyRouteError, match="read-only observability surface"):
        handle_observability_request("POST", "/api/mission/start", snapshot)

    with pytest.raises(ReadOnlyRouteError, match="not exposed"):
        handle_observability_request("GET", "/api/mission/cancel", snapshot)
