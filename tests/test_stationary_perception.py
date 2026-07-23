from __future__ import annotations

from pathlib import Path
import threading
import time

import pytest

from sphero_rvr_driver.live_mission_service import LiveStateCache
from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_service import MissionService
from sphero_rvr_driver.mission_web import build_mission_web_bundle
from sphero_rvr_driver.stationary_perception import (
    ScriptedStationaryIntentProvider,
    StationaryObservationIntent,
    StationaryPerceptionController,
    StationaryPerceptionEngine,
)
from sphero_rvr_driver.stationary_perception_node import (
    SemanticTrackStore,
    scan_occupancy,
    select_trackable_detections,
    stationary_localization,
)


MISSION = (
    "Observe the room while stationary, track shoes and faces, inspect uncertain "
    "evidence, and never move."
)
REPO_ROOT = Path(__file__).resolve().parents[1]


def _update_live_sources(cache: LiveStateCache, version: int) -> None:
    now = time.time()
    cache.update(
        "lidar",
        {
            "scan_id": f"scan-{version}",
            "stamp_s": now,
            "occupancy": {
                "occupied_points": [{"x_m": 1.0, "y_m": 0.0}],
                "valid_range_count": 100,
            },
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
        received_at_s=now,
        source_timestamp_s=now,
    )
    cache.update(
        "localization",
        {
            "scan_id": f"scan-{version}",
            "stamp_s": now,
            "state": "valid",
            "source": "lidar_stationary_scan_registration",
            "authoritative": True,
            "quality": 0.92,
            "pose": {
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_rad": 0.0,
                "stamp_s": now,
                "frame_id": "stationary_map",
            },
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
        received_at_s=now,
        source_timestamp_s=now,
    )
    cache.update(
        "camera",
        {
            "frame_id": f"frame-{version}",
            "stamp_s": now,
            "detections": [
                {
                    "detection_id": f"shoe-{version}",
                    "track_id": "object-0001",
                    "label": "possible_shoe",
                    "status": "review",
                }
            ],
            "uncertain_track_id": "object-0001",
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
        received_at_s=now,
        source_timestamp_s=now,
    )
    cache.update(
        "semantic_map",
        {
            "revision": version,
            "stamp_s": now,
            "uncertain_track_id": "object-0001",
            "tracks": [
                {
                    "track_id": "object-0001",
                    "kind": "object",
                    "label": "possible_shoe",
                    "confidence": 0.58,
                },
                {
                    "track_id": "face-0001",
                    "kind": "face",
                    "label": "enrolled-person",
                    "recognized_from_enrollment": True,
                    "enrollment_evidence_ids": ["enrollment-sha256:abc"],
                },
                {
                    "track_id": "face-0002",
                    "kind": "face",
                    "label": "unknown",
                    "recognized_from_enrollment": False,
                    "enrollment_evidence_ids": [],
                },
            ],
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
        received_at_s=now,
        source_timestamp_s=now,
    )


def _wait_for(
    subject: StationaryPerceptionEngine, predicate, *, timeout_s: float = 3.0
) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = subject.snapshot()
        if predicate(snapshot):
            return snapshot
        time.sleep(0.01)
    raise AssertionError("stationary perception condition was not reached")


def test_stationary_intent_is_exactly_bound_and_cannot_express_motion() -> None:
    snapshot = {
        "snapshot_id": "snapshot-1",
        "uncertain_track_id": "object-0001",
    }
    intent = StationaryObservationIntent.validated(
        {
            "snapshot_id": "snapshot-1",
            "action": "inspect",
            "observation_focus": "object-0001",
            "viewpoint_recommendation": "wider",
            "search_targets": ["shoe"],
            "lease_s": 90,
            "rationale": "The review-status object track needs a wider view.",
        },
        revision=1,
        snapshot=snapshot,
        issued_at_s=10.0,
        provider_id="test",
        model_id="test",
    )
    serialized = intent.to_json_dict()
    assert serialized["motion_authority"] is False
    assert serialized["physical_execution_enabled"] is False
    assert not {"steering", "speed_limit_mps", "route", "segments"} & set(serialized)

    with pytest.raises(MissionValidationError, match="exact live world snapshot"):
        StationaryObservationIntent.validated(
            {
                **serialized,
                "snapshot_id": "different",
            },
            revision=2,
            snapshot=snapshot,
            issued_at_s=11.0,
            provider_id="test",
            model_id="test",
        )


def test_live_scan_builds_occupancy_and_stationary_registration() -> None:
    ranges = [1.0, 1.1, float("inf"), 0.8, 1.4]
    occupancy = scan_occupancy(
        ranges,
        angle_min=-0.2,
        angle_increment=0.1,
        range_min=0.05,
        range_max=4.0,
    )
    localization, baseline = stationary_localization(
        ranges, None, stamp_s=100.0
    )
    recovered, _ = stationary_localization(
        [1.01, 1.09, float("inf"), 0.81, 1.39],
        baseline,
        stamp_s=101.0,
    )

    assert occupancy["valid_range_count"] == 4
    assert occupancy["occupied_points"]
    assert localization["source"] == "lidar_stationary_scan_registration"
    assert localization["pose"]["x_m"] == 0.0
    assert recovered["state"] == "valid"
    assert recovered["quality"] > 0.7
    assert recovered["motion_authority"] is False
    assert recovered["physical_execution_enabled"] is False


def test_moved_object_keeps_one_track_and_unknown_face_stays_unknown() -> None:
    tracks = SemanticTrackStore()
    first = tracks.update(
        [
            {
                "kind": "object",
                "label": "shoe",
                "center_x": 100.0,
                "center_y": 300.0,
                "confidence": 0.8,
                "x_m": 1.0,
                "y_m": 0.1,
                "uncertainty_m": 0.18,
                "evidence_id": "frame-1-shoe",
            },
            {
                "kind": "face",
                "label": "unknown",
                "center_x": 500.0,
                "center_y": 150.0,
                "confidence": 0.3,
                "x_m": 1.5,
                "y_m": -0.2,
                "uncertainty_m": 0.18,
                "evidence_id": "frame-1-face",
                "recognized_from_enrollment": False,
                "enrollment_evidence_ids": [],
            },
        ],
        frame_width=800,
        observed_at_s=1.0,
    )
    second = tracks.update(
        [
            {
                "kind": "object",
                "label": "shoe",
                "center_x": 360.0,
                "center_y": 310.0,
                "confidence": 0.84,
                "x_m": 1.3,
                "y_m": 0.4,
                "uncertainty_m": 0.18,
                "evidence_id": "frame-2-shoe",
            },
            {
                "kind": "face",
                "label": "invented-name",
                "center_x": 530.0,
                "center_y": 150.0,
                "confidence": 0.9,
                "x_m": 1.5,
                "y_m": -0.1,
                "uncertainty_m": 0.18,
                "evidence_id": "frame-2-face",
                "recognized_from_enrollment": False,
                "enrollment_evidence_ids": [],
            },
        ],
        frame_width=800,
        observed_at_s=1.5,
    )

    assert len([track for track in second if track["kind"] == "object"]) == 1
    assert first[0]["track_id"] == second[0]["track_id"]
    face = next(track for track in second if track["kind"] == "face")
    assert face["label"] == "unknown"
    assert face["recognized_from_enrollment"] is False


def test_motion_cue_excludes_rejected_background_object_duplicates() -> None:
    detections = [
        {
            "kind": "object",
            "label": "possible_shoe",
            "confidence": 0.4,
            "status": "rejected",
            "evidence_id": "background-wide",
        },
        {
            "kind": "object",
            "label": "possible_shoe",
            "confidence": 0.44,
            "status": "rejected",
            "evidence_id": "background-edge",
        },
        {
            "kind": "object",
            "label": "moving_object",
            "confidence": 0.62,
            "status": "review",
            "evidence_id": "shoe-motion",
        },
        {
            "kind": "face",
            "label": "unknown",
            "confidence": 0.2,
            "status": "unknown",
            "evidence_id": "face",
        },
    ]

    selected = select_trackable_detections(detections)

    assert [item["evidence_id"] for item in selected] == ["shoe-motion", "face"]


def test_rejected_object_candidates_collapse_to_strongest_track_cue() -> None:
    selected = select_trackable_detections(
        [
            {
                "kind": "object",
                "label": "possible_shoe",
                "confidence": 0.4,
                "status": "rejected",
                "evidence_id": "wide",
            },
            {
                "kind": "object",
                "label": "possible_shoe",
                "confidence": 0.44,
                "status": "rejected",
                "evidence_id": "strongest",
            },
        ]
    )

    assert [item["evidence_id"] for item in selected] == ["strongest"]


def test_sensor_processing_continues_during_llm_calls_and_stale_lidar_stops() -> None:
    cache = LiveStateCache()
    _update_live_sources(cache, 1)
    engine = StationaryPerceptionEngine(
        "stationary-test",
        MISSION,
        ScriptedStationaryIntentProvider(delay_s=0.06),
        cache,
        tick_s=0.01,
        max_source_age_s=0.12,
    )
    feeding = threading.Event()
    feeding.set()

    def feed() -> None:
        version = 2
        while feeding.wait(0.0):
            _update_live_sources(cache, version)
            version += 1
            time.sleep(0.012)

    thread = threading.Thread(target=feed, daemon=True)
    thread.start()
    try:
        engine.start()
        running = _wait_for(
            engine, lambda item: item["metrics"]["intent_revision_count"] >= 3
        )
        feeding.clear()
        thread.join(timeout=1.0)
        terminal = _wait_for(engine, lambda item: item["terminal"])
    finally:
        feeding.clear()
        thread.join(timeout=1.0)
        engine.close()

    assert running["metrics"]["sensor_updates_while_llm_in_flight"] >= 3
    assert running["metrics"]["camera_updates"] >= 3
    assert running["metrics"]["semantic_map_updates"] >= 3
    assert terminal["status"] == "blocked"
    assert terminal["terminal_reason"] == "lidar_stale"
    assert terminal["result"]["motion_authority"] is False
    assert terminal["result"]["physical_execution_enabled"] is False
    assert terminal["result"]["metrics"]["enrolled_face_track_count"] == 1
    assert terminal["result"]["metrics"]["unknown_face_track_count"] == 1


def test_controller_persists_stationary_proposal_and_never_grants_execution(
    tmp_path,
) -> None:
    cache = LiveStateCache()
    _update_live_sources(cache, 1)
    service = MissionService(
        tmp_path / "stationary.sqlite3",
        source_sha="stage-c-source",
        deployed_sha="stage-c-source",
        mode="live",
        live_execution_enabled=False,
    )
    controller = StationaryPerceptionController(
        service,
        ScriptedStationaryIntentProvider(delay_s=1.0),
        cache,
        tick_s=0.02,
        max_source_age_s=1.0,
    )
    try:
        proposed = controller.submit(MISSION, session_id="stage-c-browser")
        proposal = proposed["proposal"]
        assert proposed["status"] == "proposed"
        assert proposal["segments"] == []
        phrase = (
            "APPROVE STATIONARY PERCEPTION "
            f"{proposal['proposal_digest']}"
        )
        running = controller.approve(
            proposed["mission_id"],
            supplied_approval=phrase,
            operator="test-operator",
        )
        assert running["status"] == "running"
        assert running["approval"]["motion_authority"] is False
        service.record_stationary_perception_checkpoint(
            proposed["mission_id"],
            kind="bounded_checkpoint",
            checkpoint={
                "schema": "sphero_rvr.stationary_perception_result.v1",
                "mission_id": proposed["mission_id"],
                "status": "running",
                "terminal": False,
                "terminal_reason": "",
                "progress": 0.25,
                "world_snapshot": {
                    "snapshot_id": "snapshot-bounded",
                    "camera_payload": "x" * 20_000,
                },
                "decision_snapshots": [{"payload": "y" * 20_000}],
                "active_intent": None,
                "inference": {"in_flight": True, "call": 1},
                "metrics": {"sensor_updates": 12},
                "motion_authority": False,
                "physical_execution_enabled": False,
            },
        )
        compact = [
            event
            for event in service.prompt_status(proposed["mission_id"])["events"]
            if event["kind"] == "bounded_checkpoint"
        ][0]["payload"]
        assert compact["snapshot_id"] == "snapshot-bounded"
        assert compact["motion_authority"] is False
        assert compact["physical_execution_enabled"] is False
        assert "world_snapshot" not in compact
        assert "decision_snapshots" not in compact
        cancelled = controller.cancel(proposed["mission_id"])
        assert cancelled["status"] == "cancelled"
        assert cancelled["result"]["motion_authority"] is False
        snapshot = controller.service_snapshot()
        assert snapshot["stationary_perception_enabled"] is True
        assert snapshot["live_execution_enabled"] is False
        assert snapshot["physical_execution_enabled"] is False
    finally:
        controller.close()
        service.close()


def test_stationary_launch_and_browser_have_no_motion_surface() -> None:
    launch_text = (REPO_ROOT / "launch/stationary_perception.launch.py").read_text()
    node_text = (
        REPO_ROOT
        / "src/sphero_rvr_driver/stationary_perception_node.py"
    ).read_text()
    html = build_mission_web_bundle()["index_html"]

    assert "lidar.launch.py" in launch_text
    assert "camera.launch.py" in launch_text
    assert 'get_package_share_directory("slam_toolbox")' in launch_text
    assert '"online_async_launch.py"' in launch_text
    assert '"autostart": "true"' in launch_text
    assert '"odom"' in launch_text
    assert '"base_link"' in launch_text
    assert "from nav_msgs.msg import OccupancyGrid" in node_text
    assert 'lookup_transform(\n                        "map",\n                        "base_link"' in node_text
    assert "cv2.absdiff(previous_gray, gray)" in node_text
    assert '"label": "moving_object"' in node_text
    assert '"rvr.launch.py"' not in launch_text
    assert "from geometry_msgs" not in node_text
    assert "create_publisher(Twist" not in node_text
    assert '"/cmd_vel"' not in node_text
    assert "serial.Serial(" not in node_text
    assert "import sphero_sdk" not in node_text
    assert "LIVE STATIONARY PERCEPTION — NO MOTION AUTHORITY" in html
    assert "Sensor updates during LLM" in html
    assert "Latest live stationary camera evidence" in html
