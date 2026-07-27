from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path

import pytest

from sphero_rvr_driver.camera_lidar_localization import (
    CameraCalibration,
    ImageAnchor,
    LocalizationConfig,
    MapPose,
    SensorMount,
    evaluate_fixture,
    localize_floor_object,
    localize_plane_object,
    main,
    _anchor_from_mapping,
    _pose_from_mapping,
    _scan_from_mapping,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_PATH = (
    REPO_ROOT
    / "artifacts"
    / "phase2_camera_lidar_localization"
    / "recorded_calibration_fixture.json"
)


@pytest.fixture()
def fixture() -> dict:
    return json.loads(FIXTURE_PATH.read_text())


def _inputs(fixture: dict, case_name: str = "recorded_plane_target") -> tuple:
    calibration_data = fixture["calibration"]
    case = fixture["cases"][case_name]
    calibration = CameraCalibration.from_mapping(calibration_data["camera"])
    camera = SensorMount.from_mapping(calibration_data["base_to_camera"])
    lidar = SensorMount.from_mapping(calibration_data["base_to_lidar"])
    anchor = _anchor_from_mapping(case["anchor"])
    pose = _pose_from_mapping(case["pose"])
    return calibration, camera, lidar, anchor, pose, case


def test_fixture_is_recorded_offline_evidence_with_measured_calibration(fixture: dict) -> None:
    assert fixture["schema"] == "phase2_camera_lidar_localization_fixture.v1"
    assert fixture["authority"] == {
        "ros_publishers": False,
        "live_sensors": False,
        "motor_authority": False,
    }
    assert fixture["provenance"]["camera_info_sha256"] == (
        "f5c0de153eeb773ce4940d78b4956cd6a12e22de722803af7499038824761310"
    )
    assert fixture["calibration"]["base_to_lidar"]["x"] == pytest.approx(0.0045)
    assert fixture["calibration"]["base_to_lidar"]["y"] == pytest.approx(-0.011)
    assert fixture["calibration"]["base_to_lidar"]["yaw"] == pytest.approx(3.1239668018215028)
    case = fixture["cases"]["recorded_plane_target"]
    delta_ms = abs(case["anchor"]["timestamp_ns"] - case["scan"]["timestamp_ns"]) / 1_000_000
    assert delta_ms == pytest.approx(35.038348)
    assert delta_ms <= fixture["acceptance_gates"]["max_pair_delta_ms"]


def test_recorded_plane_target_associates_one_cluster_and_meets_error_gate(fixture: dict) -> None:
    calibration, camera, lidar, anchor, pose, case = _inputs(fixture)
    result = localize_plane_object(
        anchor=anchor,
        scan=_scan_from_mapping(case["scan"]),
        calibration=calibration,
        camera=camera,
        lidar=lidar,
        pose=pose,
    )

    assert result.method == "lidar_range"
    assert result.reason == "single_contiguous_lidar_cluster"
    assert result.cluster_indices == (1, 2, 3, 4)
    assert result.range_m == pytest.approx(0.44224999845027924)
    assert result.point is not None
    expected = case["expected_base_point"]
    error_m = math.hypot(result.point.x - expected["x"], result.point.y - expected["y"])
    model = fixture["acceptance_gates"]["recorded_point_error_model"]
    error_bound = min(model["maximum_m"], model["base_m"] + model["per_range_m"] * result.range_m)
    assert error_m <= error_bound
    assert result.calibration_id == "rvr_camera_lidar_20260717:f5c0de153eeb"
    assert result.map_revision == "recorded_pair_local_base_frame"
    assert set(result.source_timestamps_ns) == {"image", "lidar", "pose"}
    assert result.uncertainty.position_sigma_m is not None
    assert "synchronization" in result.uncertainty.sources


def test_timestamp_mismatch_returns_bounded_bearing_without_point(fixture: dict) -> None:
    calibration, camera, lidar, anchor, pose, case = _inputs(fixture)
    stale_scan = replace(
        _scan_from_mapping(case["scan"]),
        timestamp_ns=anchor.timestamp_ns + LocalizationConfig().max_sensor_delta_ns + 1,
    )

    result = localize_plane_object(
        anchor=anchor,
        scan=stale_scan,
        calibration=calibration,
        camera=camera,
        lidar=lidar,
        pose=pose,
    )

    assert result.method == "bearing_only"
    assert result.point is None
    assert result.reason == "camera_lidar_timestamp_delta_exceeded"
    assert result.bearing.half_angle_rad > 0.0
    assert result.uncertainty.position_sigma_m is None


def test_two_eligible_lidar_clusters_are_rejected_as_ambiguous(fixture: dict) -> None:
    calibration, camera, lidar, anchor, pose, case = _inputs(fixture, "ambiguous_association")
    result = localize_plane_object(
        anchor=anchor,
        scan=_scan_from_mapping(case["scan"]),
        calibration=calibration,
        camera=camera,
        lidar=lidar,
        pose=pose,
    )

    assert result.method == "bearing_only"
    assert result.point is None
    assert result.reason == "ambiguous_lidar_clusters"
    assert result.cluster_indices == ()


def test_calibrated_floor_projection_recovers_known_geometry(fixture: dict) -> None:
    calibration, camera, _, anchor, pose, case = _inputs(fixture, "calibrated_floor_geometry")
    result = localize_floor_object(
        anchor=anchor,
        calibration=calibration,
        camera=camera,
        pose=pose,
    )

    assert result.method == "floor_projection"
    assert result.point is not None
    expected = case["expected_map_point"]
    error_m = math.hypot(result.point.x - expected["x"], result.point.y - expected["y"])
    assert error_m <= fixture["acceptance_gates"]["max_floor_geometry_error_m"]
    assert result.uncertainty.position_sigma_m is not None
    assert set(result.uncertainty.sources) == {"pixel", "camera_extrinsic", "floor_plane", "robot_pose"}


def test_floor_anchor_is_derived_from_detection_bottom_center() -> None:
    anchor = ImageAnchor.from_bbox_bottom_center(
        x=320.0,
        y=400.0,
        width=40.0,
        height=60.0,
        timestamp_ns=123,
        evidence_ids=("frame-123",),
    )

    assert anchor.u == pytest.approx(339.5)
    assert anchor.v == pytest.approx(459.5)
    assert anchor.evidence_ids == ("frame-123",)


@pytest.mark.parametrize(
    ("anchor_change", "pose_change", "reason"),
    [
        ({"u": -1.0}, {}, "anchor_outside_calibrated_image"),
        ({"v": 299.0}, {}, "anchor_above_floor_projection_region"),
        ({}, {"timestamp_ns": 1784305472412875070}, "localization_stale"),
    ],
)
def test_invalid_floor_projection_is_bearing_only(
    fixture: dict,
    anchor_change: dict,
    pose_change: dict,
    reason: str,
) -> None:
    calibration, camera, _, anchor, pose, _ = _inputs(fixture, "calibrated_floor_geometry")
    changed_anchor = replace(anchor, **anchor_change)
    changed_pose = replace(pose, **pose_change)

    result = localize_floor_object(
        anchor=changed_anchor,
        calibration=calibration,
        camera=camera,
        pose=changed_pose,
    )

    assert result.method == "bearing_only"
    assert result.point is None
    assert result.reason == reason


def test_result_serialization_preserves_method_evidence_and_no_fallback_point(fixture: dict) -> None:
    report = evaluate_fixture(fixture)
    ambiguous = report["results"]["ambiguous_association"]

    assert report["passed"] is True
    assert ambiguous["method"] == "bearing_only"
    assert ambiguous["point"] is None
    assert ambiguous["calibration_id"]
    assert ambiguous["map_revision"]
    assert ambiguous["source_timestamps_ns"]
    assert ambiguous["evidence_ids"]
    assert ambiguous["uncertainty"]["bearing_sigma_rad"] > 0.0


def test_quantitative_report_is_honest_about_phase1_carryover(fixture: dict) -> None:
    report = evaluate_fixture(fixture)

    assert all(report["checks"].values())
    assert report["metrics"]["recorded_pair_delta_ms"] == pytest.approx(35.038348)
    assert report["metrics"]["recorded_point_error_m"] <= report["metrics"]["recorded_point_error_bound_m"]
    assert report["metrics"]["floor_geometry_error_m"] <= 0.05
    assert report["scope"] == {
        "recorded_offline_only": True,
        "proves_async_llm_prefetch_continuity": False,
        "physical_validation_complete": False,
    }


def test_cli_writes_a_passing_report(fixture: dict, tmp_path: Path) -> None:
    output = tmp_path / "evaluation.json"

    assert main([str(FIXTURE_PATH), "--output", str(output)]) == 0
    written = json.loads(output.read_text())
    assert written["passed"] is True
    assert written["checks"]["ambiguous_association_rejected"] is True
