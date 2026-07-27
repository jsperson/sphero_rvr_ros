from __future__ import annotations

import copy
import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

import pytest

from sphero_rvr_driver.m7_surveyed_localization import (
    CLEANUP_AUDIT_SCHEMA,
    MAX_POSE_AGE_NS,
    RANGE_BANDS,
    REPORT_SCHEMA,
    SAFE_AUTHORITY,
    audit_stationary_cleanup,
    build_capture_plan,
    build_sample_from_snapshot,
    build_session_template,
    evaluate_session,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_SHA = "a" * 40
ZERO_SHA256 = hashlib.sha256(b"").hexdigest()
CALIBRATION_ID = "m7-physical-camera:test"
MAP_REVISION = "m7-survey-grid:test"
TIMESTAMP_NS = 2_000_000_000


def _calibration() -> dict:
    return {
        "camera": {
            "width": 800,
            "height": 600,
            "k": [500.0, 0.0, 400.0, 0.0, 500.0, 300.0, 0.0, 0.0, 1.0],
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
            "distortion_model": "plumb_bob",
            "calibration_id": CALIBRATION_ID,
        },
        "base_to_camera": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.12,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "base_to_lidar": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.19,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "camera_info_sha256": ZERO_SHA256,
        "extrinsics_id": "surveyed-test-extrinsics",
    }


def _pose(timestamp_ns: int = TIMESTAMP_NS) -> dict:
    return {
        "timestamp_ns": timestamp_ns,
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
        "map_revision": MAP_REVISION,
        "position_sigma_m": 0.005,
        "yaw_sigma_rad": math.radians(0.25),
    }


def _anchor_for_point(x: float, y: float, *, floor: bool, evidence_id: str) -> dict:
    u = 400.0 - 500.0 * y / x
    v = 300.0 + 500.0 * 0.12 / x if floor else 300.0
    return {
        "u": u,
        "v": v,
        "timestamp_ns": TIMESTAMP_NS,
        "evidence_ids": [evidence_id],
    }


def _scan_for_point(x: float, y: float, *, evidence_id: str) -> dict:
    count = 720
    angle_min = -math.pi
    increment = 2.0 * math.pi / count
    angle = math.atan2(y, x)
    center = round((angle - angle_min) / increment)
    range_m = math.hypot(x, y)
    returns = [
        [center - 1, range_m],
        [center, range_m],
        [center + 1, range_m],
    ]
    return {
        "timestamp_ns": TIMESTAMP_NS + 20_000_000,
        "angle_min_rad": angle_min,
        "angle_increment_rad": increment,
        "sample_count": count,
        "range_min_m": 0.15,
        "range_max_m": 16.0,
        "returns": returns,
        "evidence_id": evidence_id,
    }


def _survey(x: float, y: float) -> dict:
    return {
        "technique": "steel tape from surveyed base-link origin and lateral square",
        "surveyor": "test operator",
        "surveyed_at_utc": "2026-07-27T00:00:00Z",
        "target_map_point": {"x": x, "y": y},
        "surveyed_range_m": math.hypot(x, y),
        "position_uncertainty_m": 0.005,
    }


def _point_sample(method: str, band: str, ordinal: int, range_m: float) -> dict:
    y = (-0.015, 0.0, 0.015)[ordinal]
    x = math.sqrt(range_m * range_m - y * y)
    sample_id = f"{method}-{band}-{ordinal + 1}"
    value = {
        "sample_id": sample_id,
        "target_id": f"target-{sample_id}",
        "expected_method": method,
        "range_band": band,
        "survey": _survey(x, y),
        "anchor": _anchor_for_point(
            x,
            y,
            floor=method == "floor_projection",
            evidence_id=f"image-{sample_id}",
        ),
        "pose": _pose(),
    }
    if method == "lidar_range":
        value["scan"] = _scan_for_point(x, y, evidence_id=f"scan-{sample_id}")
    return value


def _ambiguous_sample() -> dict:
    x, y = 0.4, 0.0
    scan = _scan_for_point(x, y, evidence_id="scan-ambiguous")
    center = 360
    scan["returns"] = [
        [center - 3, 0.4],
        [center - 2, 0.4],
        [center + 2, 0.7],
        [center + 3, 0.7],
    ]
    return {
        "sample_id": "ambiguity-control",
        "target_id": "target-ambiguity-control",
        "expected_method": "bearing_only",
        "range_band": "near",
        "survey": _survey(x, y),
        "anchor": _anchor_for_point(x, y, floor=False, evidence_id="image-ambiguous"),
        "scan": scan,
        "pose": _pose(),
    }


def _complete_session() -> dict:
    session = build_session_template(source_sha=SOURCE_SHA)
    session["provenance"] = {
        "source_sha": SOURCE_SHA,
        "source_host": "sphero-pi-2",
        "collected_at_utc": "2026-07-27T00:00:00Z",
        "ros_distro": "jazzy",
        "python_version": "3.12.3",
        "operator": "test operator",
        "environment": "level indoor survey grid; RVR power unavailable",
    }
    session["calibration"] = _calibration()
    session["map"] = {
        "map_revision": MAP_REVISION,
        "frame": "map",
        "origin_definition": "base_link center at (0,0), +x forward, +y left",
        "survey_method": "steel tape and lateral square",
        "survey_layout_sha256": ZERO_SHA256,
    }
    session["artifacts"] = [
        {"kind": kind, "path": f"/evidence/{kind}", "sha256": ZERO_SHA256}
        for kind in ("session_rosbag", "survey_layout", "camera_info", "cleanup_log")
    ]
    representative_ranges = {"near": 0.40, "mid": 0.70, "far": 1.00}
    session["samples"] = [
        _point_sample(method, band.name, ordinal, representative_ranges[band.name])
        for method in ("lidar_range", "floor_projection")
        for band in RANGE_BANDS
        for ordinal in range(3)
    ]
    session["samples"].append(_ambiguous_sample())
    cleanup = {
        "completed": True,
        "camera_stopped": True,
        "lidar_stopped": True,
        "rosbag_stopped": True,
        "prohibited_nodes_absent": True,
        "rover_serial_owner_absent": True,
    }
    session["cleanup"] = cleanup
    session["cleanup_audit"] = {
        "schema": CLEANUP_AUDIT_SCHEMA,
        "source_sha": SOURCE_SHA,
        "passed": True,
        "checks": {
            "exact_source_sha": True,
            "source_checkout_clean": True,
            "stationary_sensor_and_motion_processes_absent": True,
            "prohibited_ros_nodes_absent": True,
            "motion_topic_publishers_absent": True,
            "sensor_and_rover_devices_ownerless": True,
        },
        "cleanup": dict(cleanup),
    }
    return session


def test_complete_survey_recomputes_all_methods_and_passes() -> None:
    report = evaluate_session(_complete_session())

    assert report["schema"] == REPORT_SCHEMA
    assert report["passed"] is True
    assert len(report["samples"]) == 19
    assert all(report["coverage"].values())
    assert report["checks"]["ambiguous_association_rejected"] is True
    for method in ("lidar_range", "floor_projection"):
        for band in ("near", "mid", "far"):
            distribution = report["distributions"][method][band]
            assert distribution["sample_count"] == 3
            assert distribution["distinct_target_count"] == 3
            assert distribution["distinct_surveyed_configuration_count"] == 3
            assert distribution["recommendation_widens_provisional_bound"] is False


def test_ambiguity_control_is_bearing_only_without_point() -> None:
    report = evaluate_session(_complete_session())
    ambiguity = next(row for row in report["samples"] if row["sample_id"] == "ambiguity-control")

    assert ambiguity["result"]["method"] == "bearing_only"
    assert ambiguity["result"]["point"] is None
    assert ambiguity["result"]["reason"] == "ambiguous_lidar_clusters"


def test_three_distinct_surveyed_configurations_are_required_for_every_method_band() -> None:
    session = _complete_session()
    session["samples"] = [
        sample
        for sample in session["samples"]
        if sample["sample_id"] != "floor_projection-far-3"
    ]

    report = evaluate_session(session)

    assert report["passed"] is False
    assert report["coverage"]["floor_projection:far"] is False
    assert (
        report["checks"]["every_method_band_has_three_distinct_surveyed_configurations"]
        is False
    )


def test_one_fixed_target_may_be_reused_across_distinct_surveyed_configurations() -> None:
    session = _complete_session()
    for sample in session["samples"]:
        if sample["expected_method"] in ("lidar_range", "floor_projection"):
            sample["target_id"] = "fixed-checkerboard"

    report = evaluate_session(session)

    assert report["passed"] is True
    for method in ("lidar_range", "floor_projection"):
        for band in ("near", "mid", "far"):
            distribution = report["distributions"][method][band]
            assert distribution["distinct_target_count"] == 1
            assert distribution["distinct_surveyed_configuration_count"] == 3


def test_nonzero_surveyed_base_pose_localizes_into_the_fixed_map() -> None:
    session = _complete_session()
    sample = session["samples"][0]
    relative_x, relative_y = 0.4, -0.05
    sample["target_id"] = "fixed-checkerboard"
    sample["survey"] = _survey(0.4, 0.0)
    sample["survey"]["surveyed_range_m"] = math.hypot(relative_x, relative_y)
    sample["anchor"] = _anchor_for_point(
        relative_x,
        relative_y,
        floor=False,
        evidence_id="image-nonzero-pose",
    )
    sample["scan"] = _scan_for_point(
        relative_x,
        relative_y,
        evidence_id="scan-nonzero-pose",
    )
    sample["pose"] = _pose()
    sample["pose"]["y"] = 0.05

    report = evaluate_session(session)

    assert report["passed"] is True
    row = next(item for item in report["samples"] if item["sample_id"] == sample["sample_id"])
    assert row["result"]["point"]["x"] == pytest.approx(0.4, abs=0.001)
    assert row["result"]["point"]["y"] == pytest.approx(0.0, abs=0.001)
    assert row["relative_target_point"] == pytest.approx(
        {"x": relative_x, "y": relative_y}
    )


def test_out_of_band_position_is_rejected() -> None:
    session = _complete_session()
    sample = session["samples"][0]
    sample["survey"]["target_map_point"] = {"x": 0.2, "y": 0.0}
    sample["survey"]["surveyed_range_m"] = 0.2

    report = evaluate_session(session)

    assert report["passed"] is False
    assert "outside the reviewed" in report["error"]


def test_pose_age_gate_cannot_be_relaxed_by_manifest() -> None:
    session = _complete_session()
    session["samples"][0]["pose"]["timestamp_ns"] = TIMESTAMP_NS - MAX_POSE_AGE_NS - 1

    report = evaluate_session(session)

    assert report["passed"] is False
    assert "pose age exceeded 150 ms" in report["error"]


def test_manifest_cannot_widen_provisional_tolerances() -> None:
    session = _complete_session()
    session["gates"]["max_floor_error_m"] = 0.5

    report = evaluate_session(session)

    assert report["passed"] is False
    assert "preserve the reviewed M6 limits" in report["error"]


def test_result_like_extra_fields_cannot_inject_geometry() -> None:
    baseline = evaluate_session(_complete_session())
    session = _complete_session()
    session["samples"][0]["result"] = {
        "method": "lidar_range",
        "point": {"x": 999.0, "y": 999.0, "frame": "map"},
    }

    report = evaluate_session(session)

    assert report["passed"] is True
    assert report["samples"][0]["result"] == baseline["samples"][0]["result"]


def test_template_is_no_motion_and_deliberately_fails_until_completed() -> None:
    template = build_session_template(source_sha=SOURCE_SHA)

    assert template["authority"] == SAFE_AUTHORITY
    assert template["samples"] == []
    assert evaluate_session(template)["passed"] is False


def test_capture_plan_contains_only_stationary_sensor_topics() -> None:
    plan = build_capture_plan(run_id="m7-phase2-test")
    rendered = " ".join(plan["launch_command"] + plan["capture_command"]).lower()

    assert plan["authority"] == SAFE_AUTHORITY
    assert plan["topics"] == [
        "/scan",
        "/camera_node/image_raw",
        "/camera_node/camera_info",
        "/tf",
        "/tf_static",
    ]
    assert "m7_stationary_localization.launch.py" in rendered
    for prohibited in (
        "rvr_node",
        "live_route_runner",
        "collision_stop",
        "cmd_vel",
        "motion_authority",
    ):
        assert prohibited not in rendered


def test_snapshot_sample_builder_binds_survey_without_authority() -> None:
    snapshot = {
        "schema": "sphero_rvr.m7_phase2_stationary_snapshot.v1",
        "read_only": True,
        "publishers_created": False,
        "motor_authority": False,
        "image": {
            "timestamp_ns": TIMESTAMP_NS,
            "width": 800,
            "height": 600,
            "evidence_id": "image-live",
        },
        "scan": _scan_for_point(0.4, 0.0, evidence_id="scan-live"),
    }

    sample = build_sample_from_snapshot(
        snapshot,
        sample_id="live-near-1",
        target_id="physical-target-1",
        expected_method="lidar_range",
        target_x=0.4,
        target_y=0.0,
        anchor_u=400.0,
        anchor_v=300.0,
        map_revision=MAP_REVISION,
        surveyor="operator",
        survey_technique="steel tape and square",
        surveyed_at_utc="2026-07-27T00:00:00Z",
        survey_uncertainty_m=0.005,
    )

    assert sample["range_band"] == "near"
    assert sample["anchor"]["evidence_ids"] == ["image-live"]
    assert sample["scan"]["evidence_id"] == "scan-live"
    assert "authority" not in sample


def test_snapshot_sample_builder_binds_nonzero_surveyed_base_pose() -> None:
    snapshot = {
        "schema": "sphero_rvr.m7_phase2_stationary_snapshot.v1",
        "read_only": True,
        "publishers_created": False,
        "motor_authority": False,
        "image": {
            "timestamp_ns": TIMESTAMP_NS,
            "width": 800,
            "height": 600,
            "evidence_id": "image-live",
        },
        "scan": _scan_for_point(0.4, -0.05, evidence_id="scan-live"),
    }

    sample = build_sample_from_snapshot(
        snapshot,
        sample_id="live-near-offset",
        target_id="fixed-checkerboard",
        expected_method="lidar_range",
        target_x=0.4,
        target_y=0.0,
        anchor_u=462.5,
        anchor_v=300.0,
        map_revision=MAP_REVISION,
        surveyor="operator",
        survey_technique="fixed target plus surveyed powered-down rover pose",
        surveyed_at_utc="2026-07-27T00:00:00Z",
        survey_uncertainty_m=0.005,
        pose_x=0.0,
        pose_y=0.05,
        pose_yaw=0.0,
    )

    assert sample["pose"]["x"] == 0.0
    assert sample["pose"]["y"] == 0.05
    assert sample["pose"]["yaw"] == 0.0
    assert sample["survey"]["surveyed_range_m"] == pytest.approx(math.hypot(0.4, 0.05))


def test_snapshot_sample_builder_rejects_out_of_image_anchor() -> None:
    snapshot = {
        "schema": "sphero_rvr.m7_phase2_stationary_snapshot.v1",
        "read_only": True,
        "publishers_created": False,
        "motor_authority": False,
        "image": {
            "timestamp_ns": TIMESTAMP_NS,
            "width": 800,
            "height": 600,
            "evidence_id": "image-live",
        },
        "scan": _scan_for_point(0.4, 0.0, evidence_id="scan-live"),
    }
    try:
        build_sample_from_snapshot(
            snapshot,
            sample_id="live-near-1",
            target_id="physical-target-1",
            expected_method="floor_projection",
            target_x=0.4,
            target_y=0.0,
            anchor_u=900.0,
            anchor_v=300.0,
            map_revision=MAP_REVISION,
            surveyor="operator",
            survey_technique="steel tape and square",
            surveyed_at_utc="2026-07-27T00:00:00Z",
            survey_uncertainty_m=0.005,
        )
    except ValueError as exc:
        assert "anchor_u is outside" in str(exc)
    else:
        raise AssertionError("out-of-image anchor was accepted")


def test_stationary_launch_is_default_off_and_has_no_motion_surface() -> None:
    launch_text = (
        REPO_ROOT / "launch" / "m7_stationary_localization.launch.py"
    ).read_text()
    module_text = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "m7_surveyed_localization.py"
    ).read_text()

    assert 'default_value="false"' in launch_text
    assert "IfCondition(enabled)" in launch_text
    assert "lidar.launch.py" in launch_text
    assert "camera.launch.py" in launch_text
    assert '"survey_base_x"' in launch_text
    assert '"survey_base_y"' in launch_text
    assert '"survey_base_yaw"' in launch_text
    for prohibited in (
        'executable="rvr_node"',
        'executable="live_route_runner"',
        'executable="lidar_collision_stop_supervisor"',
        "/cmd_vel",
        "physical_execution_enabled",
    ):
        assert prohibited not in launch_text
    assert "create_subscription" in module_text
    assert "create_publisher" not in module_text


def test_authority_or_cleanup_claim_cannot_be_omitted() -> None:
    for field in ("motors_available", "rover_driver_started"):
        session = _complete_session()
        del session["authority"][field]
        assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["cleanup"]["camera_stopped"] = False
    assert evaluate_session(session)["passed"] is False


@dataclass
class _Completed:
    returncode: int
    stdout: str = ""
    stderr: str = ""


def _audit_runner(
    argv: list[str],
    *,
    capture_output: bool,
    text: bool,
    check: bool,
    timeout: float,
) -> _Completed:
    del capture_output, text, check, timeout
    command = tuple(argv)
    if command[-2:] == ("rev-parse", "HEAD"):
        return _Completed(0, f"{SOURCE_SHA}\n")
    if command[-2:] == ("status", "--porcelain"):
        return _Completed(0)
    if command[:3] == ("ps", "-eo", "pid=,args="):
        return _Completed(0, "1 /sbin/init\n")
    if command[1:3] == ("node", "list"):
        return _Completed(0)
    if command[1:3] == ("topic", "list"):
        return _Completed(0, "/rosout\n")
    if "topic" in command and "info" in command:
        return _Completed(1, stderr="Unknown topic")
    if command[0] == "fuser":
        return _Completed(1)
    raise AssertionError(f"unexpected audit command: {command}")


def test_generated_cleanup_audit_proves_process_graph_and_device_shutdown() -> None:
    report = audit_stationary_cleanup(
        source_sha=SOURCE_SHA,
        source_repo=Path("/source"),
        runner=_audit_runner,
    )

    assert report["passed"] is True
    assert all(report["checks"].values())
    assert report["cleanup"] == {
        "completed": True,
        "camera_stopped": True,
        "lidar_stopped": True,
        "rosbag_stopped": True,
        "prohibited_nodes_absent": True,
        "rover_serial_owner_absent": True,
    }
    assert report["findings"]["motion_topic_publishers"] == {
        "/cmd_vel": 0,
        "/cmd_vel_motor": 0,
        "/nav2_cmd_vel_request": 0,
    }


def test_cleanup_audit_fails_closed_when_a_motion_publisher_cannot_be_counted() -> None:
    def runner(
        argv: list[str],
        *,
        capture_output: bool,
        text: bool,
        check: bool,
        timeout: float,
    ) -> _Completed:
        result = _audit_runner(
            argv,
            capture_output=capture_output,
            text=text,
            check=check,
            timeout=timeout,
        )
        if argv[-1] == "/cmd_vel" and "info" in argv:
            return _Completed(0, "Type: geometry_msgs/msg/Twist\n")
        return result

    report = audit_stationary_cleanup(
        source_sha=SOURCE_SHA,
        source_repo=Path("/source"),
        runner=runner,
    )

    assert report["passed"] is False
    assert report["checks"]["motion_topic_publishers_absent"] is False
    assert report["cleanup"]["completed"] is False


def test_session_rejects_hand_entered_cleanup_without_generated_audit() -> None:
    session = _complete_session()
    session["cleanup_audit"] = {}

    report = evaluate_session(session)

    assert report["passed"] is False
    assert "cleanup_audit schema is invalid" in report["error"]


def test_sample_copy_does_not_mutate_shared_fixture() -> None:
    first = _complete_session()
    second = copy.deepcopy(first)
    second["samples"][0]["anchor"]["u"] += 20.0

    assert first["samples"][0]["anchor"]["u"] != second["samples"][0]["anchor"]["u"]
