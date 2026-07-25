from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any, Mapping, Optional

import pytest

from sphero_rvr_driver.live_mission_service import LiveStateCache
from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.prompt_drive_ros import _supervision_sample
from sphero_rvr_driver.adaptive_mission_controller import AdaptiveMissionIntent, AdaptiveMissionLimits
from sphero_rvr_driver.adaptive_mission_physical import PhysicalAdaptiveMissionExecutor


SHA = "adaptive-mission-reviewed-sha"
REPO_ROOT = Path(__file__).resolve().parents[1]


def _cache(now: float = 100.0) -> LiveStateCache:
    cache = LiveStateCache()
    cache.update(
        "camera",
        {"frame_id": "live-camera-1", "detections": []},
        received_at_s=now,
    )
    cache.update(
        "lidar",
        {"scan_id": "live-scan-1", "sample_count": 720},
        received_at_s=now,
    )
    cache.update(
        "localization",
        {"state": "valid", "stamp_s": now},
        received_at_s=now,
    )
    cache.update(
        "odom",
        {
            "frame_id": "map",
            "x_m": 1.0,
            "y_m": 2.0,
            "heading_deg": 10.0,
        },
        received_at_s=now,
    )
    cache.update(
        "collision",
        {
            "state": "CLEAR",
            "scan_healthy": True,
            "scan_age_s": 0.02,
            "tf_available": True,
            "tf_reason": "ok",
            "front_clearance_m": 1.5,
            "forward_corridor_clearance_m": 1.5,
            "left_clearance_m": 1.0,
            "right_clearance_m": 0.8,
        },
        received_at_s=now,
    )
    cache.update(
        "control",
        {"state": "READY", "stop_active": False, "estop_latched": False},
        received_at_s=now,
    )
    return cache


def _refresh_perception(
    cache: LiveStateCache, *, received_at_s: float = 100.1
) -> None:
    cache.update(
        "camera",
        {"frame_id": "live-camera-2", "detections": []},
        received_at_s=received_at_s,
    )
    cache.update(
        "lidar",
        {"scan_id": "live-scan-2", "sample_count": 720},
        received_at_s=received_at_s,
    )
    cache.update(
        "localization",
        {"state": "valid", "stamp_s": received_at_s},
        received_at_s=received_at_s,
    )


def _intent(
    executor: PhysicalAdaptiveMissionExecutor,
    action: str,
    value: float,
    *,
    revision: int = 1,
) -> AdaptiveMissionIntent:
    snapshot = executor.snapshot("physical-adaptive-mission")
    raw = {
        "snapshot_id": snapshot["snapshot_id"],
        "action": action,
        "distance_m": value if action == "move_distance" else 0.0,
        "angle_deg": value if action == "turn_angle" else 0.0,
        "observation_focus": "authoritative live lidar",
        "rationale": "Fresh evidence supports one bounded intent.",
        "interpreted_objective": "Explore from fresh physical evidence.",
        "lease_s": 5.0,
        "timeout_s": 5.0,
    }
    return AdaptiveMissionIntent.validated(
        raw,
        revision=revision,
        snapshot=snapshot,
        issued_at_s=100.0,
        provider_id="test-provider",
        model_id="test-model",
        limits=AdaptiveMissionLimits(),
    )


class FakeRouteTransport:
    def __init__(
        self,
        result: Optional[Mapping[str, Any]] = None,
        *,
        after_execute: Optional[Any] = None,
    ) -> None:
        self.result = None if result is None else dict(result)
        self.after_execute = after_execute
        self.requests = []
        self.cancelled = False

    def execute(self, request):
        self.requests.append(request)
        if self.after_execute is not None:
            self.after_execute()
        if self.result is not None:
            return dict(self.result)
        return {
            "route_id": request.route_id,
            "status": "complete",
            "terminal_reason": "complete",
            "source_sha": request.source_sha,
            "terminal_settled": True,
            "measured_distance_m": abs(
                float(request.segments[0].arguments.get("distance_m", 0.0))
            ),
            "measured_angle_deg": abs(
                float(request.segments[0].arguments.get("angle_deg", 0.0))
            ),
            "executed_segments": [
                {
                    "correlation_id": request.segments[0].correlation_id,
                    "status": "complete",
                    "terminal_distance_error_m": 0.0,
                    "terminal_angle_error_deg": 0.0,
                }
            ],
            "supervision": {
                "samples": 2,
                "collision_state": "CLEAR",
                "requested": {
                    "linear_mps": 0.10,
                    "angular_rad_s": 0.0,
                },
                "supervised": {
                    "linear_mps": 0.05,
                    "angular_rad_s": 0.0,
                },
            },
        }

    def cancel(self) -> bool:
        self.cancelled = True
        return True


def test_physical_adaptive_mission_is_observation_only_when_gate_is_disabled() -> None:
    executor = PhysicalAdaptiveMissionExecutor(
        _cache(),
        source_sha=SHA,
        deployed_sha=SHA,
        execution_enabled=False,
        now=lambda: 100.0,
    )
    try:
        executor.reset("disabled-adaptive-mission")
        snapshot = executor.snapshot("disabled-adaptive-mission")
    finally:
        executor.close()

    assert snapshot["execution"]["motion_permitted"] is False
    assert snapshot["execution"]["motion_authority"] is False
    assert snapshot["evidence"]["scan_fresh"] is True
    assert snapshot["evidence"]["transform_fresh"] is True
    assert snapshot["evidence"]["odometry_fresh"] is True
    assert snapshot["evidence"]["drop_off_detection_available"] is False


def test_physical_adaptive_mission_fuses_only_fresh_localized_semantic_evidence() -> None:
    cache = _cache()
    cache.update(
        "camera",
        {
            "frame_id": "live-camera-42",
            "detections": [
                {
                    "detection_id": "shoe-detection",
                    "kind": "object",
                    "label": "possible_shoe",
                    "confidence": 0.76,
                    "status": "review",
                }
            ],
        },
        received_at_s=100.0,
    )
    cache.update(
        "localization",
        {
            "state": "valid",
            "stamp_s": 100.0,
            "pose": {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.1},
        },
        received_at_s=100.0,
    )
    cache.update(
        "semantic_map",
        {
            "revision": 17,
            "uncertain_track_id": "object-0001",
            "tracks": [
                {
                    "track_id": "object-0001",
                    "kind": "object",
                    "label": "possible_shoe",
                    "confidence": 0.76,
                    "x_m": 1.7,
                    "y_m": 2.4,
                    "uncertainty_m": 0.18,
                    "last_seen_s": 100.0,
                    "evidence_ids": ["camera:42"],
                },
                {
                    "track_id": "face-0001",
                    "kind": "face",
                    "label": "Scott",
                    "confidence": 0.91,
                    "x_m": 2.0,
                    "y_m": 2.2,
                    "last_seen_s": 100.0,
                    "recognized_from_enrollment": True,
                    "enrollment_evidence_ids": ["enrollment-sha256:abc"],
                },
                {
                    "track_id": "face-0002",
                    "kind": "face",
                    "label": "invented-name",
                    "confidence": 0.88,
                    "x_m": 2.2,
                    "y_m": 2.1,
                    "last_seen_s": 100.0,
                    "recognized_from_enrollment": True,
                    "enrollment_evidence_ids": [],
                },
                {
                    "track_id": "object-old",
                    "kind": "object",
                    "label": "old_shoe",
                    "confidence": 0.95,
                    "x_m": 4.0,
                    "y_m": 4.0,
                    "last_seen_s": 98.0,
                },
            ],
        },
        received_at_s=100.0,
    )
    executor = PhysicalAdaptiveMissionExecutor(
        cache,
        source_sha=SHA,
        deployed_sha=SHA,
        execution_enabled=False,
        now=lambda: 100.0,
    )
    try:
        snapshot = executor.snapshot("semantic-adaptive-mission")
    finally:
        executor.close()

    observations = snapshot["observations"]
    assert observations["perception"] == {
        "available": True,
        "camera_fresh": True,
        "semantic_map_fresh": True,
        "localization_fresh": True,
        "localization_state": "valid",
        "camera_frame_id": "live-camera-42",
        "semantic_map_revision": 17,
        "uncertain_track_id": "object-0001",
        "identity_policy": (
            "face labels are authoritative only with explicit enrollment evidence"
        ),
    }
    assert observations["recognized_objects"][0]["track_id"] == "object-0001"
    assert len(observations["recognized_objects"]) == 1
    assert observations["recognized_faces"][0]["label"] == "Scott"
    assert observations["recognized_faces"][0][
        "enrollment_evidence_ids"
    ] == ["enrollment-sha256:abc"]
    assert observations["unknown_faces"][0]["track_id"] == "face-0002"
    assert observations["unknown_faces"][0]["label"] == "unknown"
    assert observations["camera_detections"][0]["track_id"] == "shoe-detection"


def test_physical_adaptive_mission_withholds_stale_semantic_tracks_from_llm() -> None:
    cache = _cache()
    cache.update(
        "camera",
        {"frame_id": "fresh-frame", "detections": []},
        received_at_s=100.0,
    )
    cache.update(
        "localization",
        {"state": "valid", "stamp_s": 100.0},
        received_at_s=100.0,
    )
    cache.update(
        "semantic_map",
        {
            "revision": 3,
            "tracks": [
                {
                    "track_id": "object-stale",
                    "kind": "object",
                    "label": "shoe",
                    "last_seen_s": 98.8,
                }
            ],
        },
        received_at_s=98.8,
    )
    executor = PhysicalAdaptiveMissionExecutor(
        cache,
        source_sha=SHA,
        deployed_sha=SHA,
        execution_enabled=False,
        now=lambda: 100.0,
    )
    try:
        observations = executor.snapshot("stale-semantic-adaptive-mission")[
            "observations"
        ]
    finally:
        executor.close()

    assert observations["perception"]["available"] is False
    assert observations["perception"]["semantic_map_fresh"] is False
    assert observations["semantic_tracks"] == []
    assert observations["recognized_objects"] == []
    assert observations["recognized_faces"] == []


def test_adaptive_mission_perception_launch_fuses_sensors_without_bypassing_supervisor() -> None:
    launch_text = (
        REPO_ROOT / "launch/adaptive_mission_perception.launch.py"
    ).read_text()
    mapping_text = (REPO_ROOT / "launch/mapping.launch.py").read_text()
    perception_text = (
        REPO_ROOT
        / "src/sphero_rvr_driver/stationary_perception_node.py"
    ).read_text()
    adaptive_unit = (
        REPO_ROOT / "systemd/user/rvr-adaptive-mission.service"
    ).read_text()
    setup_text = (REPO_ROOT / "setup.py").read_text()

    assert '"start_rvr",\n                default_value="false"' in launch_text
    assert '"start_collision_stop": "true"' in launch_text
    assert '"start_lidar": "true"' in launch_text
    assert '"lidar_serial_port": lidar_serial_port' in launch_text
    assert '"start_camera": "true"' in launch_text
    assert '"start_slam": "true"' in launch_text
    assert '"slam_autostart": "true"' in launch_text
    assert '"stationary_session": False' in launch_text
    assert 'executable="stationary_perception"' in launch_text
    assert "Adaptive mission semantic perception exited" in launch_text
    assert '"start_live_route_runner": start_live_route_runner' in mapping_text
    assert '"serial_port": lidar_serial_port' in mapping_text
    assert 'default_value="/dev/rplidar"' in mapping_text
    assert '"online_async_launch.py"' in mapping_text
    assert '"autostart": "true"' in mapping_text
    assert '"slam_autostart",\n            default_value="false"' in mapping_text
    assert 'serial_port:="${RVR_SERIAL_PORT:-/dev/ttyAMA0}"' in adaptive_unit
    assert (
        'lidar_serial_port:="${RVR_LIDAR_SERIAL_PORT:-/dev/rplidar}"'
        in adaptive_unit
    )
    assert '"camera_info_url": camera_info_url' in mapping_text
    assert '"last_seen_s": self.last_seen_s' in perception_text
    assert "localization = self._localization_from_tf(" in perception_text
    assert "slam_toolbox_moving" in perception_text
    assert '"launch/adaptive_mission_perception.launch.py"' in setup_text


@pytest.mark.parametrize(
    ("source", "deployed", "reviewed"),
    (
        ("a", "b", "a"),
        ("a", "a", "b"),
        ("a", "a", ""),
    ),
)
def test_physical_adaptive_mission_gate_requires_exact_reviewed_deployment(
    source: str, deployed: str, reviewed: str
) -> None:
    with pytest.raises(MissionValidationError, match="identical source"):
        PhysicalAdaptiveMissionExecutor(
            _cache(),
            source_sha=source,
            deployed_sha=deployed,
            reviewed_sha=reviewed,
            execution_enabled=True,
            transport=FakeRouteTransport(),
            now=lambda: 100.0,
        )


def test_physical_adaptive_mission_submits_exactly_one_bounded_supervised_route() -> None:
    cache = _cache()
    transport = FakeRouteTransport(
        after_execute=lambda: _refresh_perception(cache)
    )
    executor = PhysicalAdaptiveMissionExecutor(
        cache,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
        now=lambda: 100.0,
    )
    try:
        executor.reset("physical-adaptive-mission")
        executor.bind_approval(
            proposal_digest="a" * 64,
            approval_id="operator:" + "a" * 64,
            operator="scott@example.com",
        )
        result = executor.execute(
            _intent(executor, "move_distance", 0.25),
            threading.Event(),
        )
    finally:
        executor.close()

    assert result.outcome == "completed"
    assert len(transport.requests) == 1
    request = transport.requests[0]
    assert len(request.segments) == 1
    assert request.max_runtime_s == 5.0
    assert request.max_travel_m == 0.25
    assert request.segments[0].arguments["speed_mps"] == 0.10
    assert result.movement.requested_linear_mps == 0.10
    assert result.movement.supervised_linear_mps == 0.05
    assert result.movement.to_json_dict()["motor_topic_publisher"] == (
        "lidar_collision_stop_supervisor"
    )
    assert result.snapshot["progress"]["cumulative_translation_m"] == 0.25
    receipts = result.snapshot["evidence"]["source_receipts"]
    assert receipts["camera"]["received_at_s"] == 100.1
    assert receipts["lidar"]["received_at_s"] == 100.1
    assert receipts["localization"]["received_at_s"] == 100.1


def test_physical_adaptive_mission_rejects_move_beyond_typed_usable_clearance() -> None:
    cache = _cache()
    cache.update(
        "collision",
        {
            "state": "CLEAR",
            "scan_healthy": True,
            "scan_age_s": 0.02,
            "tf_available": True,
            "tf_reason": "ok",
            "front_clearance_m": 0.5945,
            "forward_corridor_clearance_m": 0.4165,
            "left_clearance_m": 0.6158,
            "right_clearance_m": 0.4602,
        },
        received_at_s=100.0,
    )
    executor = PhysicalAdaptiveMissionExecutor(
        cache,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=FakeRouteTransport(),
        now=lambda: 100.0,
    )
    try:
        executor.reset("physical-adaptive-mission")
        snapshot = executor.snapshot("physical-adaptive-mission")
        clearance = snapshot["observations"]["motion_clearance"]
        raw = {
            "snapshot_id": snapshot["snapshot_id"],
            "action": "move_distance",
            "distance_m": 0.25,
            "angle_deg": 0.0,
            "observation_focus": "Choose a safer direction.",
            "rationale": "Forward clearance is narrow.",
            "interpreted_objective": "Explore safely.",
            "lease_s": 5.0,
            "timeout_s": 5.0,
        }
        with pytest.raises(
            MissionValidationError,
            match="exceeds snapshot usable forward clearance",
        ):
            AdaptiveMissionIntent.validated(
                raw,
                revision=1,
                snapshot=snapshot,
                issued_at_s=100.0,
                provider_id="test-provider",
                model_id="test-model",
                limits=AdaptiveMissionLimits(),
            )
    finally:
        executor.close()

    assert clearance["translation_reserve_m"] == 0.40
    assert clearance["forward_usable_m"] == pytest.approx(0.0165)
    assert clearance["reverse_usable_m"] is None


def test_physical_adaptive_mission_accepts_bounded_settled_distance_overshoot() -> None:
    cache = _cache()
    transport = FakeRouteTransport(
        {
            "route_id": (
                "physical-adaptive-mission:adaptive-mission-intent:1"
            ),
            "status": "complete",
            "terminal_reason": "complete",
            "source_sha": SHA,
            "terminal_settled": True,
            "measured_distance_m": 0.273,
            "measured_angle_deg": 0.0,
            "executed_segments": [
                {
                    "correlation_id": (
                        "physical-adaptive-mission:"
                        "adaptive-mission-intent:1"
                    ),
                    "status": "complete",
                    "terminal_distance_error_m": 0.023,
                    "terminal_angle_error_deg": None,
                }
            ],
            "supervision": {
                "samples": 78,
                "collision_state": "CLEAR",
                "requested": {
                    "linear_mps": 0.10,
                    "angular_rad_s": 0.0,
                },
                "supervised": {
                    "linear_mps": 0.10,
                    "angular_rad_s": -0.015,
                },
            },
        },
        after_execute=lambda: _refresh_perception(cache),
    )
    executor = PhysicalAdaptiveMissionExecutor(
        cache,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
        now=lambda: 100.0,
    )
    try:
        executor.reset("physical-adaptive-mission")
        executor.bind_approval(
            proposal_digest="6" * 64,
            approval_id="operator:" + "6" * 64,
            operator="scott@example.com",
        )
        result = executor.execute(
            _intent(executor, "move_distance", 0.25),
            threading.Event(),
        )
    finally:
        executor.close()

    assert result.outcome == "completed"
    assert result.reason == "complete"
    assert result.movement.supervised_linear_mps == 0.10
    assert result.snapshot["progress"]["cumulative_translation_m"] == 0.273


def test_physical_adaptive_mission_rejects_terminal_overshoot_beyond_tolerance() -> None:
    transport = FakeRouteTransport(
        {
            "route_id": (
                "physical-adaptive-mission:adaptive-mission-intent:1"
            ),
            "status": "complete",
            "terminal_reason": "complete",
            "source_sha": SHA,
            "terminal_settled": True,
            "measured_distance_m": 0.281,
            "measured_angle_deg": 0.0,
            "executed_segments": [
                {
                    "correlation_id": (
                        "physical-adaptive-mission:"
                        "adaptive-mission-intent:1"
                    ),
                    "status": "complete",
                    "terminal_distance_error_m": 0.031,
                    "terminal_angle_error_deg": None,
                }
            ],
            "supervision": {
                "samples": 78,
                "collision_state": "CLEAR",
                "requested": {
                    "linear_mps": 0.10,
                    "angular_rad_s": 0.0,
                },
                "supervised": {
                    "linear_mps": 0.10,
                    "angular_rad_s": 0.0,
                },
            },
        }
    )
    executor = PhysicalAdaptiveMissionExecutor(
        _cache(),
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
        now=lambda: 100.0,
    )
    try:
        executor.reset("physical-adaptive-mission")
        executor.bind_approval(
            proposal_digest="8" * 64,
            approval_id="operator:" + "8" * 64,
            operator="scott@example.com",
        )
        result = executor.execute(
            _intent(executor, "move_distance", 0.25),
            threading.Event(),
        )
    finally:
        executor.close()

    assert result.outcome == "failed"
    assert result.reason == "terminal_evidence_incomplete"
    assert result.movement.supervised_linear_mps == 0.0


def test_physical_adaptive_mission_blocks_replanning_without_updated_perception() -> None:
    transport = FakeRouteTransport()
    executor = PhysicalAdaptiveMissionExecutor(
        _cache(),
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
        now=lambda: 100.0,
    )
    executor.reset("physical-adaptive-mission")
    executor.bind_approval(
        proposal_digest="7" * 64,
        approval_id="operator:" + "7" * 64,
        operator="scott@example.com",
    )
    snapshot = executor.snapshot("physical-adaptive-mission")
    raw = {
        "snapshot_id": snapshot["snapshot_id"],
        "action": "move_distance",
        "distance_m": 0.10,
        "angle_deg": 0.0,
        "observation_focus": "new camera, lidar, and localization receipts",
        "rationale": "Require updated perception before another model call.",
        "interpreted_objective": "Explore only from updated physical evidence.",
        "lease_s": 0.05,
        "timeout_s": 0.05,
    }
    intent = AdaptiveMissionIntent.validated(
        raw,
        revision=1,
        snapshot=snapshot,
        issued_at_s=100.0,
        provider_id="test-provider",
        model_id="test-model",
        limits=AdaptiveMissionLimits(),
    )
    try:
        result = executor.execute(intent, threading.Event())
    finally:
        executor.close()

    assert result.outcome == "blocked"
    assert result.reason == "updated_perception_timeout"
    assert len(transport.requests) == 1
    assert result.movement.supervised_linear_mps == 0.05


def test_physical_adaptive_mission_rejects_uncorrelated_terminal_evidence() -> None:
    transport = FakeRouteTransport(
        {
            "route_id": "wrong-route",
            "status": "complete",
            "source_sha": SHA,
            "terminal_settled": True,
            "executed_segments": [],
            "supervision": {
                "samples": 1,
                "requested": {"linear_mps": 0.1, "angular_rad_s": 0.0},
                "supervised": {"linear_mps": 0.1, "angular_rad_s": 0.0},
            },
        }
    )
    executor = PhysicalAdaptiveMissionExecutor(
        _cache(),
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
        now=lambda: 100.0,
    )
    try:
        executor.reset("physical-adaptive-mission")
        executor.bind_approval(
            proposal_digest="b" * 64,
            approval_id="operator:" + "b" * 64,
            operator="scott@example.com",
        )
        result = executor.execute(
            _intent(executor, "move_distance", 0.10),
            threading.Event(),
        )
    finally:
        executor.close()

    assert result.outcome == "failed"
    assert result.reason == "terminal_correlation_mismatch"
    assert result.movement.supervised_linear_mps == 0.0


def test_physical_adaptive_mission_collision_terminal_vetoes_llm_motion() -> None:
    transport = FakeRouteTransport(
        {
            "route_id": "physical-adaptive-mission:adaptive-mission-intent:1",
            "status": "blocked",
            "terminal_reason": "collision_veto",
            "source_sha": SHA,
            "terminal_settled": True,
            "executed_segments": [],
            "supervision": {
                "samples": 2,
                "collision_state": "BLOCKED",
                "requested": {
                    "linear_mps": 0.10,
                    "angular_rad_s": 0.0,
                },
                "supervised": {
                    "linear_mps": 0.0,
                    "angular_rad_s": 0.0,
                },
            },
        }
    )
    executor = PhysicalAdaptiveMissionExecutor(
        _cache(),
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
        now=lambda: 100.0,
    )
    try:
        executor.reset("physical-adaptive-mission")
        executor.bind_approval(
            proposal_digest="e" * 64,
            approval_id="operator:" + "e" * 64,
            operator="scott@example.com",
        )
        result = executor.execute(
            _intent(executor, "move_distance", 0.10),
            threading.Event(),
        )
    finally:
        executor.close()

    assert result.outcome == "blocked"
    assert result.reason == "collision_veto"
    assert result.movement.requested_linear_mps == 0.10
    assert result.movement.supervised_linear_mps == 0.0


def test_physical_adaptive_mission_rechecks_stale_evidence_at_submission() -> None:
    cache = _cache()
    transport = FakeRouteTransport()
    executor = PhysicalAdaptiveMissionExecutor(
        cache,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
        now=lambda: 100.0,
    )
    executor.reset("physical-adaptive-mission")
    executor.bind_approval(
        proposal_digest="9" * 64,
        approval_id="operator:" + "9" * 64,
        operator="scott@example.com",
    )
    intent = _intent(executor, "move_distance", 0.10)
    # The intent was valid against its bound snapshot; time advances before
    # submission, so the executor must veto rather than publish a route.
    executor._now = lambda: 102.0
    try:
        result = executor.execute(intent, threading.Event())
    finally:
        executor.close()

    assert result.outcome == "blocked"
    assert "stale_or_unsafe_evidence" in result.reason
    assert transport.requests == []


class BlockingTransport(FakeRouteTransport):
    def __init__(self) -> None:
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def execute(self, request):
        self.requests.append(request)
        self.entered.set()
        self.release.wait(timeout=2.0)
        return {
            "route_id": request.route_id,
            "status": "cancelled",
            "terminal_reason": "cancelled",
            "source_sha": request.source_sha,
            "terminal_settled": True,
            "executed_segments": [],
            "supervision": {
                "samples": 1,
                "requested": {"linear_mps": 0.1, "angular_rad_s": 0.0},
                "supervised": {"linear_mps": 0.0, "angular_rad_s": 0.0},
            },
        }

    def cancel(self) -> bool:
        self.cancelled = True
        self.release.set()
        return True


def test_physical_adaptive_mission_cancellation_reaches_transport_and_returns_zero() -> None:
    transport = BlockingTransport()
    executor = PhysicalAdaptiveMissionExecutor(
        _cache(),
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
        now=lambda: 100.0,
    )
    cancellation = threading.Event()
    executor.reset("physical-adaptive-mission")
    executor.bind_approval(
        proposal_digest="c" * 64,
        approval_id="operator:" + "c" * 64,
        operator="scott@example.com",
    )
    intent = _intent(executor, "move_distance", 0.10)
    holder = {}

    thread = threading.Thread(
        target=lambda: holder.setdefault(
            "result", executor.execute(intent, cancellation)
        )
    )
    thread.start()
    assert transport.entered.wait(timeout=1.0)
    cancellation.set()
    thread.join(timeout=1.0)
    executor.close()

    assert not thread.is_alive()
    assert transport.cancelled is True
    assert holder["result"].outcome == "cancelled"
    assert holder["result"].movement.supervised_linear_mps == 0.0


def test_collision_state_supervision_parser_is_typed_and_finite() -> None:
    parsed = _supervision_sample(
        "SLOW reason=front_slow requested=(0.100,0.000) "
        "output=(0.040,0.000)"
    )

    assert parsed == {
        "collision_state": "SLOW",
        "requested": {"linear_mps": 0.1, "angular_rad_s": 0.0},
        "supervised": {"linear_mps": 0.04, "angular_rad_s": 0.0},
    }
    assert _supervision_sample("CLEAR output=(nan,0)") is None


class UnconfirmedCancellationTransport(BlockingTransport):
    def cancel(self) -> bool:
        self.cancelled = True
        return False


def test_physical_adaptive_mission_timeout_reports_uncertain_cleanup_until_settled() -> None:
    transport = UnconfirmedCancellationTransport()
    executor = PhysicalAdaptiveMissionExecutor(
        _cache(),
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
        cleanup_timeout_s=0.05,
        now=lambda: 100.0,
    )
    executor.reset("physical-adaptive-mission")
    executor.bind_approval(
        proposal_digest="f" * 64,
        approval_id="operator:" + "f" * 64,
        operator="scott@example.com",
    )
    snapshot = executor.snapshot("physical-adaptive-mission")
    raw = {
        "snapshot_id": snapshot["snapshot_id"],
        "action": "move_distance",
        "distance_m": 0.10,
        "angle_deg": 0.0,
        "observation_focus": "authoritative live lidar",
        "rationale": "Exercise bounded timeout cleanup.",
        "interpreted_objective": "Stop if route cleanup cannot be proven.",
        "lease_s": 0.05,
        "timeout_s": 0.05,
    }
    intent = AdaptiveMissionIntent.validated(
        raw,
        revision=1,
        snapshot=snapshot,
        issued_at_s=100.0,
        provider_id="test-provider",
        model_id="test-model",
        limits=AdaptiveMissionLimits(),
    )
    result = executor.execute(intent, threading.Event())
    transport.release.set()
    executor.close()

    assert transport.cancelled is True
    assert result.outcome == "failed"
    assert result.reason == "cleanup_uncertain"
    assert result.movement.supervised_linear_mps == 0.0
