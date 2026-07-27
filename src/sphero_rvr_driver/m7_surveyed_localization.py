"""Milestone 7.2 surveyed, stationary localization evidence.

The evaluator is deliberately ROS-free and recomputes every localization result
from compact recorded evidence.  Its companion capture plan contains only the
camera, lidar, static survey transform, and rosbag topics.  Neither surface can
start the RVR driver, publish velocity, or grant motion authority.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import time
import statistics
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

from .camera_lidar_localization import (
    CameraCalibration,
    LocalizationConfig,
    LocalizationResult,
    SensorMount,
    _anchor_from_mapping,
    _pose_from_mapping,
    _scan_from_mapping,
    localize_floor_object,
    localize_plane_object,
)


SESSION_SCHEMA = "sphero_rvr.m7_phase2_surveyed_localization_session.v1"
REPORT_SCHEMA = "sphero_rvr.m7_phase2_surveyed_localization_report.v1"
CAPTURE_PLAN_SCHEMA = "sphero_rvr.m7_phase2_stationary_capture_plan.v1"
SNAPSHOT_SCHEMA = "sphero_rvr.m7_phase2_stationary_snapshot.v1"
SHA_RE = re.compile(r"^[0-9a-f]{40}$")

METHODS = ("lidar_range", "floor_projection")
REQUIRED_UNCERTAINTY_SOURCES = {
    "lidar_range": {
        "pixel",
        "lidar_range",
        "camera_lidar_extrinsic",
        "synchronization",
        "robot_pose",
    },
    "floor_projection": {
        "pixel",
        "camera_extrinsic",
        "floor_plane",
        "robot_pose",
    },
}


@dataclass(frozen=True)
class RangeBand:
    name: str
    minimum_m: float
    maximum_m: float
    maximum_inclusive: bool = False

    def contains(self, value: float) -> bool:
        upper_ok = value <= self.maximum_m if self.maximum_inclusive else value < self.maximum_m
        return self.minimum_m <= value and upper_ok


RANGE_BANDS = (
    RangeBand("near", 0.30, 0.55),
    RangeBand("mid", 0.55, 0.85),
    RangeBand("far", 0.85, 1.20, maximum_inclusive=True),
)

MAX_SENSOR_DELTA_NS = 100_000_000
MAX_POSE_AGE_NS = 150_000_000
MIN_TARGETS_PER_METHOD_BAND = 3
MAX_FLOOR_ERROR_M = 0.05
LIDAR_ERROR_BASE_M = 0.03
LIDAR_ERROR_PER_RANGE_M = 0.04
LIDAR_ERROR_MAXIMUM_M = 0.08

SAFE_AUTHORITY = {
    "stationary": True,
    "rover_driver_started": False,
    "rover_serial_transport_started": False,
    "cmd_vel_publishers_present": False,
    "physical_execution_enabled": False,
    "motion_authority": False,
    "motors_available": False,
}
REQUIRED_CLEANUP = {
    "completed": True,
    "camera_stopped": True,
    "lidar_stopped": True,
    "rosbag_stopped": True,
    "prohibited_nodes_absent": True,
    "rover_serial_owner_absent": True,
}
SAFE_CAPTURE_TOPICS = (
    "/scan",
    "/camera_node/image_raw",
    "/camera_node/camera_info",
    "/tf",
    "/tf_static",
)
PROHIBITED_CAPTURE_TERMS = (
    "rvr_node",
    "live_route_runner",
    "collision_stop",
    "cmd_vel",
    "physical_execution",
    "motion_authority",
)


class SurveyValidationError(ValueError):
    """Raised when evidence is malformed or violates the M7.2 contract."""


def _message_timestamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _rgb_pixels(message: Any) -> bytes:
    encoding = str(message.encoding).lower()
    if encoding not in {"rgb8", "bgr8", "8uc3"}:
        raise SurveyValidationError(f"unsupported image encoding {message.encoding!r}")
    raw = bytes(message.data)
    pixels = bytearray()
    for y in range(int(message.height)):
        row = raw[y * int(message.step) : y * int(message.step) + int(message.width) * 3]
        if encoding == "bgr8":
            for x in range(0, len(row), 3):
                pixels.extend((row[x + 2], row[x + 1], row[x]))
        else:
            pixels.extend(row)
    return bytes(pixels)


def _write_ppm(path: Path, *, width: int, height: int, pixels: bytes) -> None:
    expected = width * height * 3
    if len(pixels) != expected:
        raise SurveyValidationError(
            f"image payload has {len(pixels)} bytes; expected {expected}"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as handle:
        handle.write(f"P6\n{width} {height}\n255\n".encode())
        handle.write(pixels)


def capture_stationary_snapshot(
    *,
    output: Path,
    image_output: Path,
    timeout_s: float = 10.0,
    image_topic: str = "/camera_node/image_raw",
    camera_info_topic: str = "/camera_node/camera_info",
    scan_topic: str = "/scan",
) -> dict[str, Any]:
    """Capture one timestamp-paired image/scan without publishers or motion."""

    _require(timeout_s > 0.0, "timeout_s must be positive")
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image, LaserScan
    except ImportError as exc:  # pragma: no cover - requires the Pi ROS runtime
        raise SurveyValidationError(
            "snapshot capture requires sourced ROS 2 rclpy and sensor_msgs"
        ) from exc

    images: list[Any] = []
    scans: list[Any] = []
    camera_infos: list[Any] = []
    rclpy.init(args=None)
    node = rclpy.create_node("rvr_m7_stationary_snapshot")

    def retain(target: list[Any], message: Any) -> None:
        target.append(message)
        del target[:-20]

    subscriptions = [
        node.create_subscription(
            Image,
            image_topic,
            lambda message: retain(images, message),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            CameraInfo,
            camera_info_topic,
            lambda message: retain(camera_infos, message),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            LaserScan,
            scan_topic,
            lambda message: retain(scans, message),
            qos_profile_sensor_data,
        ),
    ]
    pair: Optional[tuple[Any, Any]] = None
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))
            if not images or not scans or not camera_infos:
                continue
            candidates = [
                (
                    abs(_message_timestamp_ns(image) - _message_timestamp_ns(scan)),
                    image,
                    scan,
                )
                for image in images
                for scan in scans
            ]
            delta_ns, image, scan = min(candidates, key=lambda item: item[0])
            if delta_ns <= MAX_SENSOR_DELTA_NS:
                pair = (image, scan)
                break
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if pair is None:
        raise SurveyValidationError(
            "no image/scan pair within 100 ms arrived before the snapshot timeout"
        )

    image, scan = pair
    camera_info = camera_infos[-1]
    image_ns = _message_timestamp_ns(image)
    scan_ns = _message_timestamp_ns(scan)
    pixels = _rgb_pixels(image)
    _write_ppm(
        image_output,
        width=int(image.width),
        height=int(image.height),
        pixels=pixels,
    )
    image_sha256 = hashlib.sha256(image_output.read_bytes()).hexdigest()
    camera_payload = {
        "width": int(camera_info.width),
        "height": int(camera_info.height),
        "k": [float(value) for value in camera_info.k],
        "d": [float(value) for value in camera_info.d],
        "distortion_model": str(camera_info.distortion_model),
    }
    calibration_digest = _canonical_sha256(camera_payload)
    camera_payload["calibration_id"] = f"m7_camera_info:{calibration_digest[:16]}"
    finite_returns = [
        [index, float(value)]
        for index, value in enumerate(scan.ranges)
        if math.isfinite(float(value))
        and float(scan.range_min) <= float(value) <= float(scan.range_max)
    ]
    snapshot_id = f"snapshot-{image_ns}-{scan_ns}"
    snapshot = {
        "schema": SNAPSHOT_SCHEMA,
        "snapshot_id": snapshot_id,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "read_only": True,
        "publishers_created": False,
        "motor_authority": False,
        "topics": {
            "image": image_topic,
            "camera_info": camera_info_topic,
            "scan": scan_topic,
        },
        "image": {
            "timestamp_ns": image_ns,
            "width": int(image.width),
            "height": int(image.height),
            "encoding": str(image.encoding),
            "evidence_id": f"image-{image_ns}",
            "ppm_path": str(image_output),
            "ppm_sha256": image_sha256,
        },
        "camera": camera_payload,
        "scan": {
            "timestamp_ns": scan_ns,
            "angle_min_rad": float(scan.angle_min),
            "angle_increment_rad": float(scan.angle_increment),
            "sample_count": len(scan.ranges),
            "range_min_m": float(scan.range_min),
            "range_max_m": float(scan.range_max),
            "returns": finite_returns,
            "evidence_id": f"scan-{scan_ns}",
        },
        "sensor_delta_ns": abs(image_ns - scan_ns),
    }
    _write_or_print(snapshot, output)
    return snapshot


def build_sample_from_snapshot(
    snapshot: Mapping[str, Any],
    *,
    sample_id: str,
    target_id: str,
    expected_method: str,
    target_x: float,
    target_y: float,
    anchor_u: float,
    anchor_v: float,
    map_revision: str,
    surveyor: str,
    survey_technique: str,
    surveyed_at_utc: str,
    survey_uncertainty_m: float,
) -> dict[str, Any]:
    """Bind operator-reviewed survey geometry and image anchor to a snapshot."""

    _require(snapshot.get("schema") == SNAPSHOT_SCHEMA, "snapshot schema is invalid")
    _require(snapshot.get("read_only") is True, "snapshot must be read-only")
    _require(snapshot.get("publishers_created") is False, "snapshot must not create publishers")
    _require(snapshot.get("motor_authority") is False, "snapshot must not have motor authority")
    _require(expected_method in {*METHODS, "bearing_only"}, "unsupported expected method")
    for label, value in (
        ("sample_id", sample_id),
        ("target_id", target_id),
        ("map_revision", map_revision),
        ("surveyor", surveyor),
        ("survey_technique", survey_technique),
        ("surveyed_at_utc", surveyed_at_utc),
    ):
        _require(bool(value.strip()), f"{label} is required")
    image = snapshot["image"]
    _require(0.0 <= anchor_u < float(image["width"]), "anchor_u is outside the image")
    _require(0.0 <= anchor_v < float(image["height"]), "anchor_v is outside the image")
    surveyed_range_m = math.hypot(target_x, target_y)
    band = _range_band(surveyed_range_m)
    value = {
        "sample_id": sample_id,
        "target_id": target_id,
        "expected_method": expected_method,
        "range_band": band.name,
        "survey": {
            "technique": survey_technique,
            "surveyor": surveyor,
            "surveyed_at_utc": surveyed_at_utc,
            "target_map_point": {"x": float(target_x), "y": float(target_y)},
            "surveyed_range_m": surveyed_range_m,
            "position_uncertainty_m": float(survey_uncertainty_m),
        },
        "anchor": {
            "u": float(anchor_u),
            "v": float(anchor_v),
            "timestamp_ns": int(image["timestamp_ns"]),
            "evidence_ids": [str(image["evidence_id"])],
        },
        "pose": {
            "timestamp_ns": int(image["timestamp_ns"]),
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "map_revision": map_revision,
            "position_sigma_m": float(survey_uncertainty_m),
            "yaw_sigma_rad": math.radians(0.5),
        },
    }
    if expected_method in {"lidar_range", "bearing_only"}:
        value["scan"] = dict(snapshot["scan"])
    return value


def _canonical_sha256(value: Mapping[str, Any]) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise SurveyValidationError(message)


def _require_finite(value: Any, label: str) -> float:
    number = float(value)
    _require(math.isfinite(number), f"{label} must be finite")
    return number


def _percentile(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _range_band(value: float) -> RangeBand:
    for band in RANGE_BANDS:
        if band.contains(value):
            return band
    raise SurveyValidationError(
        f"surveyed range {value:.6f} m is outside the reviewed 0.30-1.20 m range"
    )


def _lidar_bound(range_m: float) -> float:
    return min(
        LIDAR_ERROR_MAXIMUM_M,
        LIDAR_ERROR_BASE_M + LIDAR_ERROR_PER_RANGE_M * range_m,
    )


def _error_bound(method: str, range_m: float) -> float:
    return _lidar_bound(range_m) if method == "lidar_range" else MAX_FLOOR_ERROR_M


def _verify_contract(session: Mapping[str, Any]) -> None:
    _require(session.get("schema") == SESSION_SCHEMA, f"schema must be {SESSION_SCHEMA}")
    provenance = session.get("provenance")
    _require(isinstance(provenance, Mapping), "provenance is required")
    _require(SHA_RE.fullmatch(str(provenance.get("source_sha", ""))) is not None, "exact 40-hex source_sha is required")
    for field in (
        "source_host",
        "collected_at_utc",
        "ros_distro",
        "python_version",
        "operator",
        "environment",
    ):
        _require(bool(str(provenance.get(field, "")).strip()), f"provenance.{field} is required")

    _require(session.get("authority") == SAFE_AUTHORITY, "authority must match the stationary no-motion contract")
    _require(session.get("cleanup") == REQUIRED_CLEANUP, "cleanup must prove all sensor and no-motion checks")

    declared_bands = session.get("range_bands")
    _require(
        declared_bands == [asdict(band) for band in RANGE_BANDS],
        "range_bands must match the reviewed M7.2 bands",
    )
    gates = session.get("gates")
    expected_gates = {
        "max_sensor_delta_ns": MAX_SENSOR_DELTA_NS,
        "max_pose_age_ns": MAX_POSE_AGE_NS,
        "minimum_distinct_targets_per_method_band": MIN_TARGETS_PER_METHOD_BAND,
        "lidar_error_model": {
            "base_m": LIDAR_ERROR_BASE_M,
            "per_range_m": LIDAR_ERROR_PER_RANGE_M,
            "maximum_m": LIDAR_ERROR_MAXIMUM_M,
        },
        "max_floor_error_m": MAX_FLOOR_ERROR_M,
        "reject_ambiguous": True,
        "bearing_only_never_a_point": True,
        "tolerance_policy": "physical evidence may tighten; widening requires separate review",
    }
    _require(gates == expected_gates, "gates must preserve the reviewed M6 limits")

    calibration = session.get("calibration")
    _require(isinstance(calibration, Mapping), "calibration is required")
    camera = CameraCalibration.from_mapping(calibration["camera"])
    _require(bool(camera.calibration_id), "calibration identity is required")
    for field in ("camera_info_sha256", "extrinsics_id"):
        value = str(calibration.get(field, ""))
        _require(bool(value), f"calibration.{field} is required")
    _require(
        re.fullmatch(r"[0-9a-f]{64}", str(calibration["camera_info_sha256"])) is not None,
        "camera_info_sha256 must be 64 lowercase hex characters",
    )

    map_identity = session.get("map")
    _require(isinstance(map_identity, Mapping), "map identity is required")
    for field in ("map_revision", "frame", "origin_definition", "survey_method", "survey_layout_sha256"):
        _require(bool(str(map_identity.get(field, "")).strip()), f"map.{field} is required")
    _require(map_identity["frame"] == "map", "survey frame must be map")
    _require(
        re.fullmatch(r"[0-9a-f]{64}", str(map_identity["survey_layout_sha256"])) is not None,
        "survey_layout_sha256 must be 64 lowercase hex characters",
    )

    artifacts = session.get("artifacts")
    _require(isinstance(artifacts, list), "artifacts inventory is required")
    kinds = set()
    for artifact in artifacts:
        _require(isinstance(artifact, Mapping), "each artifact must be an object")
        kinds.add(str(artifact.get("kind", "")))
        _require(bool(str(artifact.get("path", "")).strip()), "artifact path is required")
        _require(
            re.fullmatch(r"[0-9a-f]{64}", str(artifact.get("sha256", ""))) is not None,
            "artifact sha256 must be 64 lowercase hex characters",
        )
    _require(
        {"session_rosbag", "survey_layout", "camera_info", "cleanup_log"} <= kinds,
        "artifact inventory must include session_rosbag, survey_layout, camera_info, and cleanup_log",
    )


def _localize_sample(
    sample: Mapping[str, Any],
    *,
    calibration: CameraCalibration,
    camera: SensorMount,
    lidar: SensorMount,
) -> LocalizationResult:
    method = str(sample.get("expected_method", ""))
    anchor = _anchor_from_mapping(sample["anchor"])
    pose = _pose_from_mapping(sample["pose"])
    if method == "lidar_range":
        _require("scan" in sample, f"{sample.get('sample_id')}: lidar_range requires scan")
        return localize_plane_object(
            anchor=anchor,
            scan=_scan_from_mapping(sample["scan"]),
            calibration=calibration,
            camera=camera,
            lidar=lidar,
            pose=pose,
            config=LocalizationConfig(),
        )
    if method == "floor_projection":
        return localize_floor_object(
            anchor=anchor,
            calibration=calibration,
            camera=camera,
            pose=pose,
            config=LocalizationConfig(),
        )
    if method == "bearing_only":
        _require("scan" in sample, f"{sample.get('sample_id')}: ambiguity control requires scan")
        return localize_plane_object(
            anchor=anchor,
            scan=_scan_from_mapping(sample["scan"]),
            calibration=calibration,
            camera=camera,
            lidar=lidar,
            pose=pose,
            config=LocalizationConfig(),
        )
    raise SurveyValidationError(f"{sample.get('sample_id')}: unsupported expected_method {method!r}")


def _evaluate_sample(
    sample: Mapping[str, Any],
    *,
    calibration: CameraCalibration,
    camera: SensorMount,
    lidar: SensorMount,
    map_revision: str,
) -> dict[str, Any]:
    sample_id = str(sample.get("sample_id", ""))
    target_id = str(sample.get("target_id", ""))
    _require(sample_id and target_id, "every sample requires sample_id and target_id")
    survey = sample.get("survey")
    _require(isinstance(survey, Mapping), f"{sample_id}: survey is required")
    for field in ("technique", "surveyor", "surveyed_at_utc"):
        _require(bool(str(survey.get(field, "")).strip()), f"{sample_id}: survey.{field} is required")
    target = survey.get("target_map_point")
    _require(isinstance(target, Mapping), f"{sample_id}: target_map_point is required")
    target_x = _require_finite(target.get("x"), f"{sample_id}: target x")
    target_y = _require_finite(target.get("y"), f"{sample_id}: target y")
    surveyed_range = _require_finite(survey.get("surveyed_range_m"), f"{sample_id}: surveyed range")
    survey_uncertainty = _require_finite(
        survey.get("position_uncertainty_m"),
        f"{sample_id}: survey uncertainty",
    )
    _require(0.0 < survey_uncertainty <= 0.02, f"{sample_id}: survey uncertainty must be >0 and <=0.02 m")
    band = _range_band(surveyed_range)
    _require(sample.get("range_band") == band.name, f"{sample_id}: declared range_band disagrees with surveyed range")

    pose = _pose_from_mapping(sample["pose"])
    _require(pose.map_revision == map_revision, f"{sample_id}: pose map revision does not match session")
    geometric_range = math.hypot(target_x - pose.x, target_y - pose.y)
    _require(
        abs(geometric_range - surveyed_range) <= survey_uncertainty,
        f"{sample_id}: surveyed range disagrees with surveyed map geometry",
    )

    result = _localize_sample(sample, calibration=calibration, camera=camera, lidar=lidar)
    image_ns = int(result.source_timestamps_ns["image"])
    pose_age_ns = abs(image_ns - int(result.source_timestamps_ns["pose"]))
    sensor_delta_ns: Optional[int] = None
    if "lidar" in result.source_timestamps_ns:
        sensor_delta_ns = abs(image_ns - int(result.source_timestamps_ns["lidar"]))

    _require(result.calibration_id == calibration.calibration_id, f"{sample_id}: calibration identity changed")
    _require(result.map_revision == map_revision, f"{sample_id}: result map revision changed")
    _require(pose_age_ns <= MAX_POSE_AGE_NS, f"{sample_id}: pose age exceeded 150 ms")
    _require(bool(result.evidence_ids), f"{sample_id}: evidence IDs are required")

    expected_method = str(sample["expected_method"])
    if expected_method == "bearing_only":
        _require(result.method == "bearing_only", f"{sample_id}: ambiguity control did not fall back")
        _require(result.point is None, f"{sample_id}: bearing-only result contained a point")
        _require(result.reason == "ambiguous_lidar_clusters", f"{sample_id}: wrong ambiguity reason")
        _require(sensor_delta_ns is not None and sensor_delta_ns <= MAX_SENSOR_DELTA_NS, f"{sample_id}: ambiguity pair exceeded 100 ms")
        return {
            "sample_id": sample_id,
            "target_id": target_id,
            "expected_method": expected_method,
            "range_band": band.name,
            "surveyed_range_m": surveyed_range,
            "sensor_delta_ms": sensor_delta_ns / 1_000_000.0,
            "pose_age_ms": pose_age_ns / 1_000_000.0,
            "error_m": None,
            "error_bound_m": None,
            "passed": True,
            "result": result.to_json_dict(),
        }

    _require(expected_method in METHODS, f"{sample_id}: point sample has unsupported method")
    _require(result.method == expected_method, f"{sample_id}: expected {expected_method}, got {result.method}")
    _require(result.point is not None, f"{sample_id}: point-producing method returned no point")
    if expected_method == "lidar_range":
        _require(sensor_delta_ns is not None and sensor_delta_ns <= MAX_SENSOR_DELTA_NS, f"{sample_id}: camera/lidar delta exceeded 100 ms")
    _require(
        set(result.uncertainty.sources) == REQUIRED_UNCERTAINTY_SOURCES[expected_method],
        f"{sample_id}: uncertainty sources are incomplete",
    )
    _require(result.uncertainty.position_sigma_m is not None, f"{sample_id}: position uncertainty is required")
    error_m = math.hypot(result.point.x - target_x, result.point.y - target_y)
    bound_m = _error_bound(expected_method, surveyed_range)
    return {
        "sample_id": sample_id,
        "target_id": target_id,
        "expected_method": expected_method,
        "range_band": band.name,
        "surveyed_range_m": surveyed_range,
        "sensor_delta_ms": None if sensor_delta_ns is None else sensor_delta_ns / 1_000_000.0,
        "pose_age_ms": pose_age_ns / 1_000_000.0,
        "error_m": error_m,
        "error_bound_m": bound_m,
        "survey_uncertainty_m": survey_uncertainty,
        "passed": error_m <= bound_m,
        "result": result.to_json_dict(),
    }


def _distribution(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    errors = [float(row["error_m"]) for row in rows]
    uncertainties = [float(row["survey_uncertainty_m"]) for row in rows]
    provisional_cap = min(float(row["error_bound_m"]) for row in rows)
    evidence_candidate = max(max(errors), _percentile(errors, 0.95) + max(uncertainties))
    recommended = min(provisional_cap, math.ceil(evidence_candidate * 1000.0) / 1000.0)
    return {
        "sample_count": len(rows),
        "distinct_target_count": len({str(row["target_id"]) for row in rows}),
        "error_m": {
            "minimum": min(errors),
            "median": statistics.median(errors),
            "p95": _percentile(errors, 0.95),
            "maximum": max(errors),
            "rmse": math.sqrt(statistics.mean(value * value for value in errors)),
        },
        "provisional_cap_m": provisional_cap,
        "recommended_reviewed_tolerance_m": recommended,
        "recommendation_widens_provisional_bound": recommended > provisional_cap,
    }


def evaluate_session(session: Mapping[str, Any]) -> dict[str, Any]:
    """Recompute and validate one stationary physical-survey session."""

    try:
        _verify_contract(session)
        calibration_mapping = session["calibration"]
        calibration = CameraCalibration.from_mapping(calibration_mapping["camera"])
        camera = SensorMount.from_mapping(calibration_mapping["base_to_camera"])
        lidar = SensorMount.from_mapping(calibration_mapping["base_to_lidar"])
        samples = session.get("samples")
        _require(isinstance(samples, list) and samples, "samples are required")
        sample_ids = [str(sample.get("sample_id", "")) for sample in samples]
        _require(len(sample_ids) == len(set(sample_ids)), "sample_id values must be unique")

        rows = [
            _evaluate_sample(
                sample,
                calibration=calibration,
                camera=camera,
                lidar=lidar,
                map_revision=str(session["map"]["map_revision"]),
            )
            for sample in samples
        ]
        distributions: dict[str, dict[str, Any]] = {}
        coverage_checks: dict[str, bool] = {}
        for method in METHODS:
            distributions[method] = {}
            for band in RANGE_BANDS:
                selected = [
                    row
                    for row in rows
                    if row["expected_method"] == method and row["range_band"] == band.name
                ]
                key = f"{method}:{band.name}"
                coverage_checks[key] = (
                    len({str(row["target_id"]) for row in selected})
                    >= MIN_TARGETS_PER_METHOD_BAND
                )
                if selected:
                    distributions[method][band.name] = _distribution(selected)

        ambiguity_rows = [row for row in rows if row["expected_method"] == "bearing_only"]
        checks = {
            "all_sample_gates_passed": all(bool(row["passed"]) for row in rows),
            "every_method_band_has_three_distinct_targets": all(coverage_checks.values()),
            "ambiguous_association_rejected": bool(ambiguity_rows)
            and all(
                row["result"]["method"] == "bearing_only"
                and row["result"]["point"] is None
                and row["result"]["reason"] == "ambiguous_lidar_clusters"
                for row in ambiguity_rows
            ),
            "bearing_only_never_contains_point": all(
                row["result"]["point"] is None
                for row in rows
                if row["result"]["method"] == "bearing_only"
            ),
            "stationary_no_motion_authority": session["authority"] == SAFE_AUTHORITY,
            "cleanup_complete": session["cleanup"] == REQUIRED_CLEANUP,
            "no_tolerance_widening": all(
                not band["recommendation_widens_provisional_bound"]
                for method in distributions.values()
                for band in method.values()
            ),
        }
        return {
            "schema": REPORT_SCHEMA,
            "session_sha256": _canonical_sha256(session),
            "source_sha": session["provenance"]["source_sha"],
            "passed": all(checks.values()),
            "checks": checks,
            "coverage": coverage_checks,
            "distributions": distributions,
            "samples": rows,
            "scope": {
                "surveyed_physical_localization": True,
                "stationary_sensors_only": True,
                "rover_driver_started": False,
                "motor_authority": False,
                "moving_perception_validated": False,
                "physical_navigation_approved": False,
            },
        }
    except (KeyError, TypeError, ValueError) as exc:
        return {
            "schema": REPORT_SCHEMA,
            "session_sha256": _canonical_sha256(session),
            "passed": False,
            "checks": {"manifest_valid": False},
            "error": str(exc),
            "scope": {
                "surveyed_physical_localization": False,
                "stationary_sensors_only": True,
                "motor_authority": False,
            },
        }


def build_capture_plan(*, run_id: str, output_root: str = "/home/jsperson/rvr_runs") -> dict[str, Any]:
    """Return inspectable commands for a separately approved stationary session."""

    _require(bool(run_id.strip()), "run_id is required")
    launch_command = [
        "ros2",
        "launch",
        "sphero_rvr_driver",
        "m7_stationary_localization.launch.py",
        "survey_session_enabled:=true",
    ]
    capture_command = [
        "ros2",
        "run",
        "sphero_rvr_driver",
        "rvr_rosbag_capture",
        "--execute",
        "--until-interrupted",
        "--hardware-active",
        "--output-root",
        output_root,
        "--run-id",
        run_id,
    ]
    for topic in SAFE_CAPTURE_TOPICS:
        capture_command.extend(("--topic", topic))
    rendered = " ".join(launch_command + capture_command).lower()
    _require(
        not any(term in rendered for term in PROHIBITED_CAPTURE_TERMS),
        "capture plan unexpectedly contains a motor-capable surface",
    )
    return {
        "schema": CAPTURE_PLAN_SCHEMA,
        "run_id": run_id,
        "source_checkout_required": "clean exact candidate SHA",
        "operator_actions_required": [
            "keep the rover stationary with RVR power and motors unavailable",
            "survey and label every target map coordinate/range before capture",
            "inspect the ROS graph and serial owners before recording",
            "stop rosbag, camera, and lidar; then record cleanup",
        ],
        "launch_command": launch_command,
        "capture_command": capture_command,
        "topics": list(SAFE_CAPTURE_TOPICS),
        "authority": dict(SAFE_AUTHORITY),
    }


def build_session_template(*, source_sha: str) -> dict[str, Any]:
    """Build a deliberately incomplete operator template."""

    _require(SHA_RE.fullmatch(source_sha) is not None, "source_sha must be exact 40-hex")
    return {
        "schema": SESSION_SCHEMA,
        "description": "M7.2 surveyed physical localization; stationary sensors only",
        "provenance": {
            "source_sha": source_sha,
            "source_host": "",
            "collected_at_utc": "",
            "ros_distro": "",
            "python_version": "",
            "operator": "",
            "environment": "",
        },
        "authority": dict(SAFE_AUTHORITY),
        "range_bands": [asdict(band) for band in RANGE_BANDS],
        "gates": {
            "max_sensor_delta_ns": MAX_SENSOR_DELTA_NS,
            "max_pose_age_ns": MAX_POSE_AGE_NS,
            "minimum_distinct_targets_per_method_band": MIN_TARGETS_PER_METHOD_BAND,
            "lidar_error_model": {
                "base_m": LIDAR_ERROR_BASE_M,
                "per_range_m": LIDAR_ERROR_PER_RANGE_M,
                "maximum_m": LIDAR_ERROR_MAXIMUM_M,
            },
            "max_floor_error_m": MAX_FLOOR_ERROR_M,
            "reject_ambiguous": True,
            "bearing_only_never_a_point": True,
            "tolerance_policy": "physical evidence may tighten; widening requires separate review",
        },
        "calibration": {
            "camera": {},
            "base_to_camera": {},
            "base_to_lidar": {},
            "camera_info_sha256": "",
            "extrinsics_id": "",
        },
        "map": {
            "map_revision": "",
            "frame": "map",
            "origin_definition": "",
            "survey_method": "",
            "survey_layout_sha256": "",
        },
        "artifacts": [],
        "samples": [],
        "cleanup": {key: False for key in REQUIRED_CLEANUP},
    }


def _write_or_print(value: Mapping[str, Any], output: Optional[Path]) -> None:
    rendered = json.dumps(value, indent=2, sort_keys=True) + "\n"
    if output is None:
        print(rendered, end="")
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered)
    print(output)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="M7.2 stationary surveyed-localization evidence")
    subparsers = parser.add_subparsers(dest="command", required=True)
    plan = subparsers.add_parser("plan", help="emit a no-motion stationary sensor capture plan")
    plan.add_argument("--run-id", required=True)
    plan.add_argument("--output-root", default="/home/jsperson/rvr_runs")
    plan.add_argument("--output", type=Path)
    template = subparsers.add_parser("template", help="emit an incomplete physical-session template")
    template.add_argument("--source-sha", required=True)
    template.add_argument("--output", type=Path)
    snapshot = subparsers.add_parser(
        "snapshot",
        help="capture one read-only timestamp-paired camera/lidar snapshot",
    )
    snapshot.add_argument("--output", type=Path, required=True)
    snapshot.add_argument("--image-output", type=Path, required=True)
    snapshot.add_argument("--timeout", type=float, default=10.0)
    snapshot.add_argument("--image-topic", default="/camera_node/image_raw")
    snapshot.add_argument("--camera-info-topic", default="/camera_node/camera_info")
    snapshot.add_argument("--scan-topic", default="/scan")
    sample = subparsers.add_parser(
        "sample",
        help="bind surveyed geometry and a reviewed image anchor to a snapshot",
    )
    sample.add_argument("snapshot", type=Path)
    sample.add_argument("--sample-id", required=True)
    sample.add_argument("--target-id", required=True)
    sample.add_argument(
        "--expected-method",
        required=True,
        choices=(*METHODS, "bearing_only"),
    )
    sample.add_argument("--target-x", type=float, required=True)
    sample.add_argument("--target-y", type=float, required=True)
    sample.add_argument("--anchor-u", type=float, required=True)
    sample.add_argument("--anchor-v", type=float, required=True)
    sample.add_argument("--map-revision", required=True)
    sample.add_argument("--surveyor", required=True)
    sample.add_argument("--survey-technique", required=True)
    sample.add_argument("--surveyed-at-utc", required=True)
    sample.add_argument("--survey-uncertainty-m", type=float, required=True)
    sample.add_argument("--output", type=Path)
    evaluate = subparsers.add_parser("evaluate", help="recompute and validate a completed session")
    evaluate.add_argument("session", type=Path)
    evaluate.add_argument("--output", type=Path)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args.command == "plan":
        value = build_capture_plan(run_id=args.run_id, output_root=args.output_root)
        _write_or_print(value, args.output)
        return 0
    if args.command == "template":
        value = build_session_template(source_sha=args.source_sha)
        _write_or_print(value, args.output)
        return 0
    if args.command == "snapshot":
        try:
            capture_stationary_snapshot(
                output=args.output,
                image_output=args.image_output,
                timeout_s=args.timeout,
                image_topic=args.image_topic,
                camera_info_topic=args.camera_info_topic,
                scan_topic=args.scan_topic,
            )
        except SurveyValidationError as exc:
            print(f"ERROR: {exc}")
            return 2
        return 0
    if args.command == "sample":
        snapshot = json.loads(args.snapshot.read_text())
        try:
            value = build_sample_from_snapshot(
                snapshot,
                sample_id=args.sample_id,
                target_id=args.target_id,
                expected_method=args.expected_method,
                target_x=args.target_x,
                target_y=args.target_y,
                anchor_u=args.anchor_u,
                anchor_v=args.anchor_v,
                map_revision=args.map_revision,
                surveyor=args.surveyor,
                survey_technique=args.survey_technique,
                surveyed_at_utc=args.surveyed_at_utc,
                survey_uncertainty_m=args.survey_uncertainty_m,
            )
        except SurveyValidationError as exc:
            print(f"ERROR: {exc}")
            return 2
        _write_or_print(value, args.output)
        return 0
    session = json.loads(args.session.read_text())
    report = evaluate_session(session)
    _write_or_print(report, args.output)
    return 0 if report["passed"] else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
