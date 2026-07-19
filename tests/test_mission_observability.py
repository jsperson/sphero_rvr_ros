from __future__ import annotations

import json

import pytest

from sphero_rvr_driver.mission_api import (
    CapabilitySet,
    MissionEventKind,
    MissionStateMachine,
    build_canonical_shoe_mapping_request,
    validate_mission_request,
)
from sphero_rvr_driver.mission_observability import (
    ReadOnlyRouteError,
    build_mock_observability_snapshot,
    build_static_pwa_bundle,
    handle_observability_request,
    iter_event_stream_frames,
)


def _mission_machine() -> MissionStateMachine:
    command = validate_mission_request(build_canonical_shoe_mapping_request(mission_id="obs-001"), CapabilitySet.all_enabled())
    machine = MissionStateMachine(command)
    machine.apply(MissionEventKind.START_REQUESTED)
    machine.apply(MissionEventKind.VALIDATED)
    return machine


def test_observability_snapshot_serializes_vs07_contract_read_only_status_surfaces() -> None:
    snapshot = build_mock_observability_snapshot(_mission_machine().snapshot())
    payload = snapshot.to_json_dict()

    assert payload["api_version"] == "mission_api.v1"
    assert payload["mission"]["mission_id"] == "obs-001"
    assert payload["mission"]["state"] == "MAPPING"
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
    machine = _mission_machine()
    mapping = build_mock_observability_snapshot(machine.snapshot())
    exploring = build_mock_observability_snapshot(machine.apply(MissionEventKind.MAPPING_STARTED))

    frames = list(iter_event_stream_frames((mapping, exploring)))

    assert len(frames) == 2
    assert frames[0].startswith("event: mission_observability\n")
    first_data = json.loads(frames[0].split("data: ", 1)[1])
    second_data = json.loads(frames[1].split("data: ", 1)[1])
    assert first_data["mission"]["state"] == "MAPPING"
    assert second_data["mission"]["state"] == "EXPLORING"
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
    snapshot = build_mock_observability_snapshot(_mission_machine().snapshot())

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
