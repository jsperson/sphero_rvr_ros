"""ROS-free recorded-evidence camera-to-map localization.

This module deliberately has no subscriptions, publishers, launch integration, or
motor authority.  It associates a reviewed image anchor with a timestamp-paired
lidar cluster, projects a floor contact point, or returns a bounded bearing-only
fallback.  The fallback never contains a point estimate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence


def _angle_delta(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def _rotate_xyz(
    vector: tuple[float, float, float],
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    x, y, z = vector
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    y, z = y * cr - z * sr, y * sr + z * cr
    x, z = x * cp + z * sp, -x * sp + z * cp
    return (x * cy - y * sy, x * sy + y * cy, z)


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    k: tuple[float, ...]
    d: tuple[float, ...]
    distortion_model: str
    calibration_id: str

    def __post_init__(self) -> None:
        if len(self.k) != 9 or self.k[0] <= 0.0 or self.k[4] <= 0.0:
            raise ValueError("camera calibration requires a valid 3x3 K matrix")
        if self.width <= 0 or self.height <= 0 or not self.calibration_id:
            raise ValueError("camera calibration dimensions and identity are required")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "CameraCalibration":
        return cls(
            width=int(value["width"]),
            height=int(value["height"]),
            k=tuple(float(item) for item in value["k"]),
            d=tuple(float(item) for item in value.get("d", ())),
            distortion_model=str(value["distortion_model"]),
            calibration_id=str(value["calibration_id"]),
        )

    def optical_ray(self, u: float, v: float) -> tuple[float, float, float]:
        distorted_x = (u - self.k[2]) / self.k[0]
        distorted_y = (v - self.k[5]) / self.k[4]
        if self.distortion_model != "plumb_bob" or len(self.d) < 5:
            return (distorted_x, distorted_y, 1.0)
        k1, k2, p1, p2, k3 = self.d[:5]
        x, y = distorted_x, distorted_y
        for _ in range(8):
            radius2 = x * x + y * y
            radial = 1.0 + k1 * radius2 + k2 * radius2**2 + k3 * radius2**3
            if abs(radial) < 1e-9:
                raise ValueError("camera distortion inversion is singular")
            tangential_x = 2.0 * p1 * x * y + p2 * (radius2 + 2.0 * x * x)
            tangential_y = p1 * (radius2 + 2.0 * y * y) + 2.0 * p2 * x * y
            x = (distorted_x - tangential_x) / radial
            y = (distorted_y - tangential_y) / radial
        return (x, y, 1.0)


@dataclass(frozen=True)
class SensorMount:
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "SensorMount":
        return cls(**{name: float(value.get(name, 0.0)) for name in ("x", "y", "z", "roll", "pitch", "yaw")})


@dataclass(frozen=True)
class MapPose:
    timestamp_ns: int
    x: float
    y: float
    yaw: float
    map_revision: str
    position_sigma_m: float = 0.02
    yaw_sigma_rad: float = math.radians(1.0)


@dataclass(frozen=True)
class ImageAnchor:
    u: float
    v: float
    timestamp_ns: int
    evidence_ids: tuple[str, ...]

    @classmethod
    def from_bbox_bottom_center(
        cls,
        *,
        x: float,
        y: float,
        width: float,
        height: float,
        timestamp_ns: int,
        evidence_ids: tuple[str, ...],
    ) -> "ImageAnchor":
        if width <= 0.0 or height <= 0.0:
            raise ValueError("detection bounding box dimensions must be positive")
        return cls(
            u=x + (width - 1.0) / 2.0,
            v=y + height - 0.5,
            timestamp_ns=timestamp_ns,
            evidence_ids=evidence_ids,
        )


@dataclass(frozen=True)
class ScanReturn:
    index: int
    range_m: float


@dataclass(frozen=True)
class LidarScan:
    timestamp_ns: int
    angle_min_rad: float
    angle_increment_rad: float
    sample_count: int
    range_min_m: float
    range_max_m: float
    returns: tuple[ScanReturn, ...]
    evidence_id: str


@dataclass(frozen=True)
class LocalizationConfig:
    max_sensor_delta_ns: int = 100_000_000
    max_pose_age_ns: int = 150_000_000
    angular_gate_rad: float = math.radians(2.0)
    min_cluster_points: int = 2
    cluster_range_gap_m: float = 0.12
    pixel_sigma_px: float = 2.0
    lidar_range_sigma_m: float = 0.01
    extrinsic_position_sigma_m: float = 0.005
    extrinsic_yaw_sigma_rad: float = math.radians(0.5)
    synchronization_sigma_m: float = 0.01
    floor_height_sigma_m: float = 0.005
    floor_min_v_margin_px: float = 8.0


@dataclass(frozen=True)
class MapPoint:
    x: float
    y: float
    frame: str = "map"


@dataclass(frozen=True)
class BearingCone:
    center_rad: float
    half_angle_rad: float


@dataclass(frozen=True)
class Uncertainty:
    position_sigma_m: Optional[float]
    bearing_sigma_rad: float
    range_sigma_m: Optional[float]
    sources: tuple[str, ...]


@dataclass(frozen=True)
class LocalizationResult:
    method: str
    point: Optional[MapPoint]
    bearing: BearingCone
    uncertainty: Uncertainty
    source_timestamps_ns: Mapping[str, int]
    calibration_id: str
    map_revision: str
    evidence_ids: tuple[str, ...]
    reason: str
    range_m: Optional[float] = None
    cluster_indices: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        if self.method not in {"lidar_range", "floor_projection", "bearing_only"}:
            raise ValueError(f"unsupported localization method: {self.method}")
        if self.method == "bearing_only" and self.point is not None:
            raise ValueError("bearing-only localization cannot contain a point")

    def to_json_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["evidence_ids"] = list(self.evidence_ids)
        data["cluster_indices"] = list(self.cluster_indices)
        data["uncertainty"]["sources"] = list(self.uncertainty.sources)
        return data


def _camera_bearing_in_base(anchor: ImageAnchor, calibration: CameraCalibration, camera: SensorMount) -> float:
    ray = calibration.optical_ray(anchor.u, anchor.v)
    ray_link = (ray[2], -ray[0], -ray[1])
    ray_base = _rotate_xyz(ray_link, camera.roll, camera.pitch, camera.yaw)
    return math.atan2(ray_base[1], ray_base[0])


def _bearing_fallback(
    *,
    anchor: ImageAnchor,
    calibration: CameraCalibration,
    camera: SensorMount,
    pose: MapPose,
    config: LocalizationConfig,
    reason: str,
    timestamps: Mapping[str, int],
    extra_evidence: tuple[str, ...] = (),
) -> LocalizationResult:
    camera_bearing = _camera_bearing_in_base(anchor, calibration, camera)
    pixel_sigma_rad = config.pixel_sigma_px / calibration.k[0]
    bearing_sigma = math.hypot(pixel_sigma_rad, config.extrinsic_yaw_sigma_rad, pose.yaw_sigma_rad)
    return LocalizationResult(
        method="bearing_only",
        point=None,
        bearing=BearingCone(
            center_rad=pose.yaw + camera_bearing,
            half_angle_rad=config.angular_gate_rad + 2.0 * bearing_sigma,
        ),
        uncertainty=Uncertainty(
            position_sigma_m=None,
            bearing_sigma_rad=bearing_sigma,
            range_sigma_m=None,
            sources=("pixel", "camera_extrinsic", "robot_pose"),
        ),
        source_timestamps_ns=dict(timestamps),
        calibration_id=calibration.calibration_id,
        map_revision=pose.map_revision,
        evidence_ids=tuple(dict.fromkeys(anchor.evidence_ids + extra_evidence)),
        reason=reason,
    )


def _candidate_clusters(
    scan: LidarScan,
    *,
    desired_camera_bearing: float,
    camera: SensorMount,
    lidar: SensorMount,
    config: LocalizationConfig,
) -> list[list[tuple[ScanReturn, float, float]]]:
    candidates: list[tuple[ScanReturn, float, float]] = []
    for item in sorted(scan.returns, key=lambda value: value.index):
        if item.index < 0 or item.index >= scan.sample_count:
            continue
        if not scan.range_min_m <= item.range_m <= scan.range_max_m:
            continue
        angle = scan.angle_min_rad + item.index * scan.angle_increment_rad
        laser_x = item.range_m * math.cos(angle)
        laser_y = item.range_m * math.sin(angle)
        rotated_x, rotated_y = _rotate_xy(laser_x, laser_y, lidar.yaw)
        base_x = lidar.x + rotated_x
        base_y = lidar.y + rotated_y
        camera_bearing = math.atan2(base_y - camera.y, base_x - camera.x)
        if abs(_angle_delta(camera_bearing, desired_camera_bearing)) <= config.angular_gate_rad:
            candidates.append((item, base_x, base_y))

    clusters: list[list[tuple[ScanReturn, float, float]]] = []
    for candidate in candidates:
        if not clusters:
            clusters.append([candidate])
            continue
        previous = clusters[-1][-1]
        adjacent = candidate[0].index == previous[0].index + 1
        range_contiguous = abs(candidate[0].range_m - previous[0].range_m) <= config.cluster_range_gap_m
        if adjacent and range_contiguous:
            clusters[-1].append(candidate)
        else:
            clusters.append([candidate])

    if len(clusters) > 1:
        first = clusters[0]
        last = clusters[-1]
        wraps = first[0][0].index == 0 and last[-1][0].index == scan.sample_count - 1
        ranges_join = abs(first[0][0].range_m - last[-1][0].range_m) <= config.cluster_range_gap_m
        if wraps and ranges_join:
            clusters = [last + first] + clusters[1:-1]
    return [cluster for cluster in clusters if len(cluster) >= config.min_cluster_points]


def localize_plane_object(
    *,
    anchor: ImageAnchor,
    scan: LidarScan,
    calibration: CameraCalibration,
    camera: SensorMount,
    lidar: SensorMount,
    pose: MapPose,
    config: LocalizationConfig = LocalizationConfig(),
) -> LocalizationResult:
    timestamps = {"image": anchor.timestamp_ns, "lidar": scan.timestamp_ns, "pose": pose.timestamp_ns}
    sensor_delta_ns = abs(anchor.timestamp_ns - scan.timestamp_ns)
    pose_age_ns = abs(anchor.timestamp_ns - pose.timestamp_ns)
    if sensor_delta_ns > config.max_sensor_delta_ns:
        return _bearing_fallback(
            anchor=anchor,
            calibration=calibration,
            camera=camera,
            pose=pose,
            config=config,
            reason="camera_lidar_timestamp_delta_exceeded",
            timestamps=timestamps,
            extra_evidence=(scan.evidence_id,),
        )
    if pose_age_ns > config.max_pose_age_ns:
        return _bearing_fallback(
            anchor=anchor,
            calibration=calibration,
            camera=camera,
            pose=pose,
            config=config,
            reason="localization_stale",
            timestamps=timestamps,
            extra_evidence=(scan.evidence_id,),
        )

    desired_bearing = _camera_bearing_in_base(anchor, calibration, camera)
    clusters = _candidate_clusters(
        scan,
        desired_camera_bearing=desired_bearing,
        camera=camera,
        lidar=lidar,
        config=config,
    )
    if len(clusters) != 1:
        reason = "no_lidar_cluster" if not clusters else "ambiguous_lidar_clusters"
        return _bearing_fallback(
            anchor=anchor,
            calibration=calibration,
            camera=camera,
            pose=pose,
            config=config,
            reason=reason,
            timestamps=timestamps,
            extra_evidence=(scan.evidence_id,),
        )

    cluster = clusters[0]
    base_x = statistics.median(value[1] for value in cluster)
    base_y = statistics.median(value[2] for value in cluster)
    map_dx, map_dy = _rotate_xy(base_x, base_y, pose.yaw)
    point = MapPoint(x=pose.x + map_dx, y=pose.y + map_dy)
    range_m = statistics.median(value[0].range_m for value in cluster)
    pixel_sigma_rad = config.pixel_sigma_px / calibration.k[0]
    bearing_sigma = math.hypot(pixel_sigma_rad, config.extrinsic_yaw_sigma_rad, pose.yaw_sigma_rad)
    position_sigma = math.sqrt(
        pose.position_sigma_m**2
        + config.lidar_range_sigma_m**2
        + config.extrinsic_position_sigma_m**2
        + config.synchronization_sigma_m**2
        + (range_m * bearing_sigma) ** 2
    )
    return LocalizationResult(
        method="lidar_range",
        point=point,
        bearing=BearingCone(center_rad=pose.yaw + desired_bearing, half_angle_rad=config.angular_gate_rad),
        uncertainty=Uncertainty(
            position_sigma_m=position_sigma,
            bearing_sigma_rad=bearing_sigma,
            range_sigma_m=config.lidar_range_sigma_m,
            sources=("pixel", "lidar_range", "camera_lidar_extrinsic", "synchronization", "robot_pose"),
        ),
        source_timestamps_ns=timestamps,
        calibration_id=calibration.calibration_id,
        map_revision=pose.map_revision,
        evidence_ids=tuple(dict.fromkeys(anchor.evidence_ids + (scan.evidence_id,))),
        reason="single_contiguous_lidar_cluster",
        range_m=range_m,
        cluster_indices=tuple(value[0].index for value in cluster),
    )


def _floor_point_in_base(
    anchor: ImageAnchor,
    calibration: CameraCalibration,
    camera: SensorMount,
    *,
    camera_height_offset_m: float = 0.0,
) -> tuple[float, float]:
    optical = calibration.optical_ray(anchor.u, anchor.v)
    camera_link = (optical[2], -optical[0], -optical[1])
    ray = _rotate_xyz(camera_link, camera.roll, camera.pitch, camera.yaw)
    height = camera.z + camera_height_offset_m
    if ray[2] >= -1e-9 or height <= 0.0:
        raise ValueError("floor anchor ray does not intersect the floor in front of the camera")
    scale = -height / ray[2]
    return (camera.x + ray[0] * scale, camera.y + ray[1] * scale)


def localize_floor_object(
    *,
    anchor: ImageAnchor,
    calibration: CameraCalibration,
    camera: SensorMount,
    pose: MapPose,
    config: LocalizationConfig = LocalizationConfig(),
) -> LocalizationResult:
    timestamps = {"image": anchor.timestamp_ns, "pose": pose.timestamp_ns}
    if abs(anchor.timestamp_ns - pose.timestamp_ns) > config.max_pose_age_ns:
        return _bearing_fallback(
            anchor=anchor,
            calibration=calibration,
            camera=camera,
            pose=pose,
            config=config,
            reason="localization_stale",
            timestamps=timestamps,
        )
    if not 0.0 <= anchor.u < calibration.width or not 0.0 <= anchor.v < calibration.height:
        return _bearing_fallback(
            anchor=anchor,
            calibration=calibration,
            camera=camera,
            pose=pose,
            config=config,
            reason="anchor_outside_calibrated_image",
            timestamps=timestamps,
        )
    if anchor.v <= calibration.k[5] + config.floor_min_v_margin_px:
        return _bearing_fallback(
            anchor=anchor,
            calibration=calibration,
            camera=camera,
            pose=pose,
            config=config,
            reason="anchor_above_floor_projection_region",
            timestamps=timestamps,
        )
    try:
        base_x, base_y = _floor_point_in_base(anchor, calibration, camera)
    except ValueError:
        return _bearing_fallback(
            anchor=anchor,
            calibration=calibration,
            camera=camera,
            pose=pose,
            config=config,
            reason="floor_intersection_invalid",
            timestamps=timestamps,
        )

    map_dx, map_dy = _rotate_xy(base_x, base_y, pose.yaw)
    point = MapPoint(x=pose.x + map_dx, y=pose.y + map_dy)
    perturbations = [
        _floor_point_in_base(
            ImageAnchor(anchor.u + du, anchor.v + dv, anchor.timestamp_ns, anchor.evidence_ids),
            calibration,
            camera,
            camera_height_offset_m=dh,
        )
        for du, dv, dh in (
            (config.pixel_sigma_px, 0.0, 0.0),
            (-config.pixel_sigma_px, 0.0, 0.0),
            (0.0, config.pixel_sigma_px, 0.0),
            (0.0, -config.pixel_sigma_px, 0.0),
            (0.0, 0.0, config.floor_height_sigma_m),
            (0.0, 0.0, -config.floor_height_sigma_m),
        )
    ]
    projection_sigma = max(math.hypot(x - base_x, y - base_y) for x, y in perturbations)
    position_sigma = math.sqrt(
        pose.position_sigma_m**2 + config.extrinsic_position_sigma_m**2 + projection_sigma**2
    )
    bearing = _camera_bearing_in_base(anchor, calibration, camera)
    bearing_sigma = math.hypot(
        config.pixel_sigma_px / calibration.k[0],
        config.extrinsic_yaw_sigma_rad,
        pose.yaw_sigma_rad,
    )
    return LocalizationResult(
        method="floor_projection",
        point=point,
        bearing=BearingCone(center_rad=pose.yaw + bearing, half_angle_rad=2.0 * bearing_sigma),
        uncertainty=Uncertainty(
            position_sigma_m=position_sigma,
            bearing_sigma_rad=bearing_sigma,
            range_sigma_m=None,
            sources=("pixel", "camera_extrinsic", "floor_plane", "robot_pose"),
        ),
        source_timestamps_ns=timestamps,
        calibration_id=calibration.calibration_id,
        map_revision=pose.map_revision,
        evidence_ids=anchor.evidence_ids,
        reason="calibrated_floor_intersection",
        range_m=math.hypot(base_x - camera.x, base_y - camera.y),
    )


def _pose_from_mapping(value: Mapping[str, Any]) -> MapPose:
    return MapPose(
        timestamp_ns=int(value["timestamp_ns"]),
        x=float(value["x"]),
        y=float(value["y"]),
        yaw=float(value["yaw"]),
        map_revision=str(value["map_revision"]),
        position_sigma_m=float(value.get("position_sigma_m", 0.02)),
        yaw_sigma_rad=float(value.get("yaw_sigma_rad", math.radians(1.0))),
    )


def _anchor_from_mapping(value: Mapping[str, Any]) -> ImageAnchor:
    return ImageAnchor(
        u=float(value["u"]),
        v=float(value["v"]),
        timestamp_ns=int(value["timestamp_ns"]),
        evidence_ids=tuple(str(item) for item in value["evidence_ids"]),
    )


def _scan_from_mapping(value: Mapping[str, Any]) -> LidarScan:
    return LidarScan(
        timestamp_ns=int(value["timestamp_ns"]),
        angle_min_rad=float(value["angle_min_rad"]),
        angle_increment_rad=float(value["angle_increment_rad"]),
        sample_count=int(value["sample_count"]),
        range_min_m=float(value["range_min_m"]),
        range_max_m=float(value["range_max_m"]),
        returns=tuple(ScanReturn(index=int(item[0]), range_m=float(item[1])) for item in value["returns"]),
        evidence_id=str(value["evidence_id"]),
    )


def evaluate_fixture(fixture: Mapping[str, Any]) -> dict[str, Any]:
    calibration = CameraCalibration.from_mapping(fixture["calibration"]["camera"])
    camera = SensorMount.from_mapping(fixture["calibration"]["base_to_camera"])
    lidar = SensorMount.from_mapping(fixture["calibration"]["base_to_lidar"])
    config = LocalizationConfig()

    plane_case = fixture["cases"]["recorded_plane_target"]
    plane = localize_plane_object(
        anchor=_anchor_from_mapping(plane_case["anchor"]),
        scan=_scan_from_mapping(plane_case["scan"]),
        calibration=calibration,
        camera=camera,
        lidar=lidar,
        pose=_pose_from_mapping(plane_case["pose"]),
        config=config,
    )
    expected_plane = plane_case["expected_base_point"]
    plane_error = (
        math.hypot(plane.point.x - float(expected_plane["x"]), plane.point.y - float(expected_plane["y"]))
        if plane.point is not None
        else math.inf
    )

    floor_case = fixture["cases"]["calibrated_floor_geometry"]
    floor = localize_floor_object(
        anchor=_anchor_from_mapping(floor_case["anchor"]),
        calibration=calibration,
        camera=camera,
        pose=_pose_from_mapping(floor_case["pose"]),
        config=config,
    )
    expected_floor = floor_case["expected_map_point"]
    floor_error = (
        math.hypot(floor.point.x - float(expected_floor["x"]), floor.point.y - float(expected_floor["y"]))
        if floor.point is not None
        else math.inf
    )

    ambiguous_case = fixture["cases"]["ambiguous_association"]
    ambiguous = localize_plane_object(
        anchor=_anchor_from_mapping(ambiguous_case["anchor"]),
        scan=_scan_from_mapping(ambiguous_case["scan"]),
        calibration=calibration,
        camera=camera,
        lidar=lidar,
        pose=_pose_from_mapping(ambiguous_case["pose"]),
        config=config,
    )

    gates = fixture["acceptance_gates"]
    range_model = gates["recorded_point_error_model"]
    recorded_error_bound = min(
        float(range_model["maximum_m"]),
        float(range_model["base_m"]) + float(range_model["per_range_m"]) * float(plane.range_m or 0.0),
    )
    checks = {
        "recorded_pair_within_100ms": abs(
            plane.source_timestamps_ns["image"] - plane.source_timestamps_ns["lidar"]
        )
        <= int(float(gates["max_pair_delta_ms"]) * 1_000_000),
        "recorded_lidar_point_error": plane.method == "lidar_range"
        and plane_error <= recorded_error_bound,
        "calibrated_floor_error": floor.method == "floor_projection"
        and floor_error <= float(gates["max_floor_geometry_error_m"]),
        "ambiguous_association_rejected": ambiguous.method == "bearing_only"
        and ambiguous.point is None
        and ambiguous.reason == "ambiguous_lidar_clusters",
        "offline_no_authority": fixture["authority"]
        == {"ros_publishers": False, "live_sensors": False, "motor_authority": False},
    }
    canonical_fixture = json.dumps(fixture, sort_keys=True, separators=(",", ":")).encode()
    return {
        "schema": "phase2_camera_lidar_localization_evaluation.v1",
        "fixture_sha256": hashlib.sha256(canonical_fixture).hexdigest(),
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            "recorded_pair_delta_ms": abs(
                plane.source_timestamps_ns["image"] - plane.source_timestamps_ns["lidar"]
            )
            / 1_000_000.0,
            "recorded_point_error_m": plane_error,
            "recorded_point_error_bound_m": recorded_error_bound,
            "floor_geometry_error_m": floor_error,
        },
        "results": {
            "recorded_plane_target": plane.to_json_dict(),
            "calibrated_floor_geometry": floor.to_json_dict(),
            "ambiguous_association": ambiguous.to_json_dict(),
        },
        "scope": {
            "recorded_offline_only": True,
            "proves_async_llm_prefetch_continuity": False,
            "physical_validation_complete": False,
        },
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate recorded/offline camera-to-map localization evidence")
    parser.add_argument("fixture", type=Path)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    report = evaluate_fixture(json.loads(args.fixture.read_text()))
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
        print(args.output)
    else:
        print(rendered, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
