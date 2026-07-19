"""Project replay shoe detections into map-frame semantic observations.

The module stays ROS-import-free so the projection math, timestamp boundaries, and
tracking/deduplication rules can be tested on development hosts. Runtime ROS nodes
can adapt CameraInfo and TF/pose messages into the small dataclasses here.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterable, Optional, Sequence

from sphero_rvr_driver.camera_calibration import require_configured_camera_info
from sphero_rvr_driver.shoe_detector import ShoeDetection


@dataclass(frozen=True)
class ProjectionLimits:
    DEFAULT = (
        "ground-plane assumption: each detection is projected from the image-footpoint ray to z=0 in base_link/map",
        "occlusion: partially hidden shoes can move the apparent contact point away from the true object footprint",
        "pose drift: map-frame coordinates inherit SLAM/odometry drift and timestamp alignment error",
        "calibration error: CameraInfo intrinsics and base_link->camera optical TF errors directly move projected points",
        "inaccessible regions: detections outside camera view or beyond reliable ground-plane intersection are absent, not free space",
    )


@dataclass(frozen=True)
class CameraMount:
    """Static base_link -> camera_link transform in meters/radians."""

    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    camera_frame: str = "camera_link"
    optical_frame: str = "camera_optical_frame"

    @classmethod
    def measured_default(cls) -> "CameraMount":
        return cls(x=0.0587375, y=-0.0301625, z=0.114300)


@dataclass(frozen=True)
class Pose2D:
    timestamp_ns: int
    x: float
    y: float
    yaw: float

    def distance_xy(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class EvidenceReference:
    frame_id: str
    path: str
    timestamp_ns: int
    source: str = ""

    def to_json_dict(self) -> dict[str, Any]:
        data = asdict(self)
        return {key: value for key, value in data.items() if value != ""}


@dataclass(frozen=True)
class FrameDetections:
    frame_id: str
    timestamp_ns: int
    detections: tuple[ShoeDetection, ...]
    evidence: EvidenceReference


@dataclass(frozen=True)
class ProjectedObservation:
    observation_id: str
    track_id: str
    label: str
    status: str
    confidence: float
    frame: str
    position: Pose2D
    evidence: EvidenceReference
    uncertainty_limits: tuple[str, ...] = ProjectionLimits.DEFAULT
    detector_reasons: tuple[str, ...] = ()

    def with_ids(self, *, observation_id: str, track_id: str) -> "ProjectedObservation":
        return replace(self, observation_id=observation_id, track_id=track_id)

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "observation_id": self.observation_id,
            "track_id": self.track_id,
            "label": self.label,
            "status": self.status,
            "confidence": round(self.confidence, 3),
            "frame": self.frame,
            "position": {
                "x": round(self.position.x, 4),
                "y": round(self.position.y, 4),
                "yaw": round(self.position.yaw, 6),
                "timestamp_ns": self.position.timestamp_ns,
            },
            "evidence": self.evidence.to_json_dict(),
            "uncertainty_limits": list(self.uncertainty_limits),
            "detector_reasons": list(self.detector_reasons),
        }


class PoseHistory:
    """Timestamped map-frame robot poses with strict interpolation boundaries."""

    def __init__(self, poses: Iterable[Pose2D]) -> None:
        self._poses = tuple(sorted(poses, key=lambda pose: pose.timestamp_ns))
        if not self._poses:
            raise ValueError("PoseHistory requires at least one pose")
        timestamps = [pose.timestamp_ns for pose in self._poses]
        if len(set(timestamps)) != len(timestamps):
            raise ValueError("PoseHistory timestamps must be unique")

    def lookup(self, timestamp_ns: int) -> Pose2D:
        first = self._poses[0]
        last = self._poses[-1]
        if timestamp_ns < first.timestamp_ns:
            raise ValueError(f"timestamp {timestamp_ns} is before first pose {first.timestamp_ns}")
        if timestamp_ns > last.timestamp_ns:
            raise ValueError(f"timestamp {timestamp_ns} is after last pose {last.timestamp_ns}")
        for pose in self._poses:
            if pose.timestamp_ns == timestamp_ns:
                return pose
        for before, after in zip(self._poses, self._poses[1:]):
            if before.timestamp_ns <= timestamp_ns <= after.timestamp_ns:
                span = after.timestamp_ns - before.timestamp_ns
                ratio = (timestamp_ns - before.timestamp_ns) / span
                return Pose2D(
                    timestamp_ns=timestamp_ns,
                    x=before.x + (after.x - before.x) * ratio,
                    y=before.y + (after.y - before.y) * ratio,
                    yaw=_interpolate_angle(before.yaw, after.yaw, ratio),
                )
        raise ValueError(f"timestamp {timestamp_ns} is not covered by pose history")


@dataclass
class _Track:
    track_id: str
    observations: list[ProjectedObservation] = field(default_factory=list)

    @property
    def representative(self) -> Pose2D:
        total_weight = sum(max(obs.confidence, 1e-6) for obs in self.observations)
        x = sum(obs.position.x * max(obs.confidence, 1e-6) for obs in self.observations) / total_weight
        y = sum(obs.position.y * max(obs.confidence, 1e-6) for obs in self.observations) / total_weight
        latest = max(self.observations, key=lambda obs: obs.position.timestamp_ns)
        return Pose2D(timestamp_ns=latest.position.timestamp_ns, x=x, y=y, yaw=latest.position.yaw)

    @property
    def confidence(self) -> float:
        return max(obs.confidence for obs in self.observations)

    @property
    def status(self) -> str:
        if any(obs.status == "accepted" for obs in self.observations):
            return "accepted"
        if any(obs.status == "review" for obs in self.observations):
            return "review"
        return "rejected"

    def to_json_dict(self) -> dict[str, Any]:
        rep = self.representative
        return {
            "track_id": self.track_id,
            "label": self.observations[0].label,
            "status": self.status,
            "confidence": round(self.confidence, 3),
            "frame": "map",
            "position": {"x": round(rep.x, 4), "y": round(rep.y, 4), "timestamp_ns": rep.timestamp_ns},
            "observation_count": len(self.observations),
            "evidence": [obs.evidence.to_json_dict() for obs in self.observations],
        }


class ShoeObservationTracker:
    def __init__(self, *, dedup_radius_m: float = 0.25) -> None:
        if dedup_radius_m <= 0:
            raise ValueError("dedup_radius_m must be positive")
        self.dedup_radius_m = dedup_radius_m
        self._tracks: list[_Track] = []
        self._next_track_id = 1
        self._next_observation_id = 1

    def add(self, observation: ProjectedObservation) -> ProjectedObservation:
        track = self._nearest_track(observation)
        if track is None:
            track = _Track(track_id=f"shoe_track_{self._next_track_id:04d}")
            self._next_track_id += 1
            self._tracks.append(track)
        tracked = observation.with_ids(
            observation_id=f"shoe_obs_{self._next_observation_id:04d}",
            track_id=track.track_id,
        )
        self._next_observation_id += 1
        track.observations.append(tracked)
        return tracked

    def add_many(self, observations: Iterable[ProjectedObservation]) -> tuple[ProjectedObservation, ...]:
        return tuple(self.add(observation) for observation in observations)

    def _nearest_track(self, observation: ProjectedObservation) -> Optional[_Track]:
        nearest: Optional[_Track] = None
        nearest_distance = self.dedup_radius_m
        for track in self._tracks:
            distance = observation.position.distance_xy(track.representative)
            if distance <= nearest_distance:
                nearest = track
                nearest_distance = distance
        return nearest

    def to_report(self) -> dict[str, Any]:
        observations = [obs for track in self._tracks for obs in track.observations]
        return {
            "source_schema": "vs04_shoe_detector_evaluation",
            "frame": "map",
            "dedup_radius_m": self.dedup_radius_m,
            "track_count": len(self._tracks),
            "observation_count": len(observations),
            "uncertainty_limits": list(ProjectionLimits.DEFAULT),
            "tracks": [track.to_json_dict() for track in self._tracks],
            "observations": [obs.to_json_dict() for obs in observations],
        }


def project_detection_to_map(
    detection: ShoeDetection,
    *,
    camera_info: object,
    camera_mount: CameraMount,
    pose: Pose2D,
    evidence: EvidenceReference,
) -> ProjectedObservation:
    require_configured_camera_info(camera_info, context="map-frame shoe projection")
    k_value = getattr(camera_info, "k", None)
    if k_value is None:
        k_value = getattr(camera_info, "K")
    k = [float(item) for item in k_value]
    fx, fy = k[0], k[4]
    cx, cy = k[2], k[5]
    if fx <= 0 or fy <= 0:
        raise ValueError("camera focal lengths must be positive for map projection")

    u = detection.bbox.x + (detection.bbox.width - 1) / 2.0
    v = detection.bbox.y + detection.bbox.height - 0.5
    ray_optical = ((u - cx) / fx, (v - cy) / fy, 1.0)
    ray_link = _optical_ray_to_camera_link(ray_optical)
    ray_base = _rotate_xyz(ray_link, camera_mount.roll, camera_mount.pitch, camera_mount.yaw)
    if ray_base[2] >= -1e-9:
        raise ValueError("detection ray does not intersect the ground plane in front of the camera")
    scale = -camera_mount.z / ray_base[2]
    base_x = camera_mount.x + ray_base[0] * scale
    base_y = camera_mount.y + ray_base[1] * scale
    map_x, map_y = _base_to_map(base_x, base_y, pose)
    return ProjectedObservation(
        observation_id="",
        track_id="",
        label=detection.label,
        status=detection.status,
        confidence=detection.confidence,
        frame="map",
        position=Pose2D(timestamp_ns=pose.timestamp_ns, x=map_x, y=map_y, yaw=pose.yaw),
        evidence=evidence,
        detector_reasons=detection.reasons,
    )


def detections_from_evaluation_report(
    report_path: Path,
    *,
    evidence_dir: Path,
    include_statuses: set[str] | frozenset[str] = frozenset({"accepted", "review"}),
) -> tuple[FrameDetections, ...]:
    report = json.loads(report_path.read_text())
    frames: list[FrameDetections] = []
    for frame in report.get("frames", []):
        frame_id = str(frame["frame_id"])
        timestamp_ns = _timestamp_from_frame_id(frame_id)
        detections = tuple(
            ShoeDetection.from_mapping(value)
            for value in frame.get("detections", [])
            if str(value.get("status", "")) in include_statuses
        )
        evidence_path = evidence_dir / f"{frame_id}_evidence.png"
        frames.append(
            FrameDetections(
                frame_id=frame_id,
                timestamp_ns=timestamp_ns,
                detections=detections,
                evidence=EvidenceReference(
                    frame_id=frame_id,
                    path=str(evidence_path),
                    timestamp_ns=timestamp_ns,
                    source=str(frame.get("source", "")),
                ),
            )
        )
    return tuple(frames)


def write_observation_report(report: dict[str, Any], output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return output_path


def _interpolate_angle(start: float, end: float, ratio: float) -> float:
    delta = math.atan2(math.sin(end - start), math.cos(end - start))
    return start + delta * ratio


def _optical_ray_to_camera_link(ray: tuple[float, float, float]) -> tuple[float, float, float]:
    x_optical, y_optical, z_optical = ray
    return (z_optical, -x_optical, -y_optical)


def _rotate_xyz(vector: tuple[float, float, float], roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    x, y, z = vector
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    y, z = y * cr - z * sr, y * sr + z * cr
    x, z = x * cp + z * sp, -x * sp + z * cp
    x, y = x * cy - y * sy, x * sy + y * cy
    return (x, y, z)


def _base_to_map(x: float, y: float, pose: Pose2D) -> tuple[float, float]:
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return (pose.x + cos_yaw * x - sin_yaw * y, pose.y + sin_yaw * x + cos_yaw * y)


def _timestamp_from_frame_id(frame_id: str) -> int:
    match = re.search(r"_(\d{12,})$", frame_id)
    if not match:
        raise ValueError(f"frame_id {frame_id!r} does not end with a nanosecond timestamp")
    return int(match.group(1))


def _parse_pose(value: str) -> Pose2D:
    parts = value.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("pose must be timestamp_ns,x,y,yaw")
    try:
        return Pose2D(timestamp_ns=int(parts[0]), x=float(parts[1]), y=float(parts[2]), yaw=float(parts[3]))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("pose must be timestamp_ns,x,y,yaw") from exc


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Project VS04 shoe detections into map-frame observations")
    parser.add_argument("evaluation_json", type=Path, help="VS04 shoe_detector_evaluation.json")
    parser.add_argument("--camera-info-json", type=Path, required=True, help="Measured CameraInfo JSON with width, height, k/K, and distortion_model")
    parser.add_argument("--pose", action="append", type=_parse_pose, required=True, help="timestamp_ns,x,y,yaw; repeat for interpolation")
    parser.add_argument("--evidence-dir", type=Path, required=True, help="Directory containing *_evidence.png files")
    parser.add_argument("--dedup-radius-m", type=float, default=0.25)
    parser.add_argument("--output", type=Path, default=Path("artifacts/shoe_map_observations.json"))
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    camera_info = _camera_info_from_json(args.camera_info_json)
    pose_history = PoseHistory(args.pose)
    tracker = ShoeObservationTracker(dedup_radius_m=args.dedup_radius_m)
    for frame in detections_from_evaluation_report(args.evaluation_json, evidence_dir=args.evidence_dir):
        pose = pose_history.lookup(frame.timestamp_ns)
        tracker.add_many(
            project_detection_to_map(
                detection,
                camera_info=camera_info,
                camera_mount=CameraMount.measured_default(),
                pose=pose,
                evidence=frame.evidence,
            )
            for detection in frame.detections
        )
    output_path = write_observation_report(tracker.to_report(), args.output)
    print(output_path)
    print(json.dumps({"observation_count": tracker.to_report()["observation_count"], "track_count": tracker.to_report()["track_count"]}, sort_keys=True))
    return 0


def _camera_info_from_json(path: Path) -> SimpleNamespace:
    data = json.loads(path.read_text())
    k = data.get("k", data.get("K"))
    return SimpleNamespace(
        width=int(data.get("width", 0)),
        height=int(data.get("height", 0)),
        k=k if k is not None else [],
        d=data.get("d", data.get("D", [])),
        distortion_model=str(data.get("distortion_model", "")),
    )


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
