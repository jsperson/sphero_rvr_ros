"""ROS 2 live lidar/camera semantic-perception adapter.

The node subscribes to sensors and publishes JSON evidence consumed by the
mission-service cache.  It has no Twist type, rover SDK import, serial access,
route publisher, or physical execution surface.  Stationary perception supplies a static
odom transform and uses the default stationary session; Adaptive mission supplies real
wheel odometry and explicitly selects a moving session.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Optional, Sequence

from .perception_navigation import LocalizationEstimate, LocalizationState, Pose2D
from .shoe_detector import DetectorThresholds, detect_shoes_in_rgb


@dataclass
class _Track:
    track_id: str
    kind: str
    label: str
    center_x: float
    center_y: float
    confidence: float
    x_m: float
    y_m: float
    uncertainty_m: float
    last_seen_s: float
    observation_count: int = 1
    evidence_ids: list[str] = field(default_factory=list)
    recognized_from_enrollment: bool = False
    enrollment_evidence_ids: list[str] = field(default_factory=list)

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "track_id": self.track_id,
            "kind": self.kind,
            "label": self.label,
            "confidence": round(self.confidence, 4),
            "x_m": round(self.x_m, 4),
            "y_m": round(self.y_m, 4),
            "uncertainty_m": round(self.uncertainty_m, 4),
            "last_seen_s": self.last_seen_s,
            "observation_count": self.observation_count,
            "evidence_ids": list(self.evidence_ids[-12:]),
            "recognized_from_enrollment": self.recognized_from_enrollment,
            "enrollment_evidence_ids": list(self.enrollment_evidence_ids),
        }


class SemanticTrackStore:
    """Small nearest-image-plane tracker that preserves IDs while objects move."""

    def __init__(self, *, maximum_age_s: float = 30.0) -> None:
        self.maximum_age_s = float(maximum_age_s)
        self._tracks: list[_Track] = []
        self._next_by_kind = {"object": 1, "face": 1}

    def update(
        self,
        detections: Sequence[Mapping[str, Any]],
        *,
        frame_width: int,
        observed_at_s: float,
    ) -> list[dict[str, Any]]:
        unmatched = set(range(len(self._tracks)))
        for detection in detections:
            kind = str(detection["kind"])
            center_x = float(detection["center_x"])
            center_y = float(detection["center_y"])
            candidates: list[tuple[float, int]] = []
            for index in unmatched:
                track = self._tracks[index]
                if track.kind != kind or observed_at_s - track.last_seen_s > self.maximum_age_s:
                    continue
                distance = math.hypot(
                    center_x - track.center_x, center_y - track.center_y
                )
                if distance <= max(80.0, float(frame_width) * 0.48):
                    candidates.append((distance, index))
            if candidates:
                _distance, index = min(candidates)
                unmatched.remove(index)
                self._apply(self._tracks[index], detection, observed_at_s)
            else:
                self._tracks.append(self._new_track(detection, observed_at_s))
        self._tracks = [
            track
            for track in self._tracks
            if observed_at_s - track.last_seen_s <= self.maximum_age_s
        ]
        return [track.to_json_dict() for track in self._tracks]

    def _new_track(
        self, detection: Mapping[str, Any], observed_at_s: float
    ) -> _Track:
        kind = str(detection["kind"])
        sequence = self._next_by_kind[kind]
        self._next_by_kind[kind] += 1
        return _Track(
            track_id=f"{kind}-{sequence:04d}",
            kind=kind,
            label=str(detection["label"]),
            center_x=float(detection["center_x"]),
            center_y=float(detection["center_y"]),
            confidence=float(detection["confidence"]),
            x_m=float(detection["x_m"]),
            y_m=float(detection["y_m"]),
            uncertainty_m=float(detection["uncertainty_m"]),
            last_seen_s=float(observed_at_s),
            evidence_ids=[str(detection["evidence_id"])],
            recognized_from_enrollment=bool(
                detection.get("recognized_from_enrollment", False)
            ),
            enrollment_evidence_ids=[
                str(item)
                for item in detection.get("enrollment_evidence_ids", [])
            ],
        )

    @staticmethod
    def _apply(
        track: _Track, detection: Mapping[str, Any], observed_at_s: float
    ) -> None:
        alpha = 0.55
        track.center_x = alpha * float(detection["center_x"]) + (1.0 - alpha) * track.center_x
        track.center_y = alpha * float(detection["center_y"]) + (1.0 - alpha) * track.center_y
        track.x_m = alpha * float(detection["x_m"]) + (1.0 - alpha) * track.x_m
        track.y_m = alpha * float(detection["y_m"]) + (1.0 - alpha) * track.y_m
        track.confidence = max(track.confidence * 0.92, float(detection["confidence"]))
        track.uncertainty_m = float(detection["uncertainty_m"])
        track.last_seen_s = float(observed_at_s)
        track.observation_count += 1
        evidence_id = str(detection["evidence_id"])
        if evidence_id not in track.evidence_ids:
            track.evidence_ids.append(evidence_id)
        if bool(detection.get("recognized_from_enrollment", False)):
            track.label = str(detection["label"])
            track.recognized_from_enrollment = True
            track.enrollment_evidence_ids = [
                str(item)
                for item in detection.get("enrollment_evidence_ids", [])
            ]
        elif track.kind == "face" and not track.recognized_from_enrollment:
            track.label = "unknown"


def select_trackable_detections(
    detections: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Keep truthful face evidence and one authoritative uncertain object cue.

    The dependency-light shoe heuristic can emit several rejected background
    components in one frame. Those components are useful visible evidence but
    must not each create a semantic object track. A live motion cue is more
    specific, so it exclusively represents the moving object for that frame.
    Otherwise all review/accepted objects survive; if every object is rejected,
    only the strongest uncertain candidate is allowed into tracking.
    """

    objects = [
        dict(detection)
        for detection in detections
        if str(detection.get("kind", "")) == "object"
    ]
    non_objects = [
        dict(detection)
        for detection in detections
        if str(detection.get("kind", "")) != "object"
    ]
    moving = [
        detection
        for detection in objects
        if str(detection.get("label", "")) == "moving_object"
    ]
    if moving:
        selected = [
            max(moving, key=lambda detection: float(detection.get("confidence", 0.0)))
        ]
    else:
        selected = [
            detection
            for detection in objects
            if str(detection.get("status", "")) != "rejected"
        ]
        if not selected and objects:
            selected = [
                max(
                    objects,
                    key=lambda detection: float(detection.get("confidence", 0.0)),
                )
            ]
    return [*selected, *non_objects]


class ExplicitFaceEnrollment:
    """LBPH recognizer trained solely from explicitly named local image folders."""

    def __init__(
        self,
        root: Path,
        *,
        cascade: Any,
        threshold: float = 44.0,
    ) -> None:
        self.root = Path(root).expanduser()
        self.threshold = float(threshold)
        self._cascade = cascade
        self._recognizer: Any = None
        self._labels: dict[int, str] = {}
        self._evidence: dict[str, list[str]] = {}
        self._load()

    @property
    def identity_count(self) -> int:
        return len(self._labels)

    def recognize(self, gray_face: Any) -> tuple[str, float, bool, list[str]]:
        if self._recognizer is None:
            return ("unknown", 0.0, False, [])
        import cv2

        normalized = cv2.resize(gray_face, (128, 128))
        label_index, distance = self._recognizer.predict(normalized)
        identity = self._labels.get(int(label_index), "")
        if not identity or not math.isfinite(float(distance)) or float(distance) > self.threshold:
            return ("unknown", max(0.0, 1.0 - float(distance) / 100.0), False, [])
        confidence = max(0.0, min(1.0, 1.0 - float(distance) / 100.0))
        return (
            identity,
            confidence,
            True,
            list(self._evidence.get(identity, [])),
        )

    def _load(self) -> None:
        import cv2
        import numpy as np

        if not self.root.is_dir():
            return
        faces: list[Any] = []
        labels: list[int] = []
        for identity_dir in sorted(path for path in self.root.iterdir() if path.is_dir()):
            identity = identity_dir.name.strip()
            if not identity or identity.lower() == "unknown":
                continue
            identity_index = len(self._labels)
            identity_evidence: list[str] = []
            for image_path in sorted(identity_dir.iterdir()):
                if image_path.suffix.lower() not in {".jpg", ".jpeg", ".png", ".pgm"}:
                    continue
                data = image_path.read_bytes()
                image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
                if image is None:
                    continue
                detected = self._cascade.detectMultiScale(
                    image, scaleFactor=1.1, minNeighbors=5, minSize=(48, 48)
                )
                if len(detected) != 1:
                    continue
                x, y, width, height = detected[0]
                faces.append(cv2.resize(image[y : y + height, x : x + width], (128, 128)))
                labels.append(identity_index)
                identity_evidence.append(
                    f"enrollment-sha256:{hashlib.sha256(data).hexdigest()}"
                )
            if identity_evidence:
                self._labels[identity_index] = identity
                self._evidence[identity] = identity_evidence
        if not faces:
            self._labels.clear()
            self._evidence.clear()
            return
        recognizer = cv2.face.LBPHFaceRecognizer_create()
        recognizer.train(faces, np.asarray(labels, dtype=np.int32))
        self._recognizer = recognizer


def scan_occupancy(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    maximum_points: int = 140,
) -> dict[str, Any]:
    """Convert a live scan to bounded local-frame occupied points."""

    valid: list[tuple[float, float, float]] = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        valid.append(
            (distance * math.cos(angle), distance * math.sin(angle), distance)
        )
    stride = max(1, math.ceil(len(valid) / max(1, maximum_points)))
    points = [
        {"x_m": round(x_m, 4), "y_m": round(y_m, 4)}
        for x_m, y_m, _distance in valid[::stride]
    ][:maximum_points]
    return {
        "frame_id": "stationary_map",
        "occupied_points": points,
        "valid_range_count": len(valid),
        "nearest_range_m": (
            None if not valid else round(min(distance for _x, _y, distance in valid), 4)
        ),
        "coverage_ratio": len(valid) / max(1, len(ranges)),
    }


def stationary_localization(
    ranges: Sequence[float],
    baseline: Optional[Sequence[float]],
    *,
    stamp_s: float,
) -> tuple[dict[str, Any], tuple[float, ...]]:
    """Register a scan to the stationary-session baseline and report its quality."""

    normalized = tuple(float(value) for value in ranges)
    reference = tuple(normalized if baseline is None else baseline)
    differences = sorted(
        abs(current - original)
        for current, original in zip(normalized, reference)
        if math.isfinite(current)
        and math.isfinite(original)
        and current > 0.02
        and original > 0.02
    )
    if differences:
        clipped = differences[: max(1, int(len(differences) * 0.8))]
        residual = sum(clipped) / len(clipped)
        overlap = len(differences) / max(1, min(len(normalized), len(reference)))
    else:
        residual = float("inf")
        overlap = 0.0
    quality = max(0.0, min(1.0, overlap * (1.0 - min(residual, 0.4) / 0.4)))
    state = (
        LocalizationState.VALID
        if quality >= 0.55
        else LocalizationState.DEGRADED
        if quality >= 0.30
        else LocalizationState.LOST
    )
    estimate = LocalizationEstimate(
        state=state,
        source="lidar_stationary_scan_registration",
        pose=(
            None
            if state is LocalizationState.LOST
            else Pose2D(
                x_m=0.0,
                y_m=0.0,
                yaw_rad=0.0,
                stamp_s=float(stamp_s),
                frame_id="stationary_map",
            )
        ),
        quality=quality,
        covariance_xy_m2=None if not math.isfinite(residual) else max(0.0004, residual**2),
        covariance_yaw_rad2=None if not math.isfinite(residual) else max(0.0004, residual**2),
        detail=(
            "Pose is fixed at the origin of this physically stationary session; "
            "quality is live scan-to-baseline consistency, not mobile odometry."
        ),
    ).to_json_dict()
    estimate.update(
        {
            "registration_residual_m": (
                None if not math.isfinite(residual) else round(residual, 5)
            ),
            "baseline_overlap": round(overlap, 5),
            "stationary_session": True,
            "motion_authority": False,
            "physical_execution_enabled": False,
        }
    )
    return estimate, reference


def _stamp_s(message: Any) -> float:
    try:
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
        return value if value > 0.0 else time.time()
    except (AttributeError, TypeError, ValueError):
        return time.time()


def main(args=None):
    import cv2
    import numpy as np
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    class StationaryPerceptionNode(Node):
        def __init__(self):
            super().__init__("stationary_perception")
            self._sensor_callbacks = ReentrantCallbackGroup()
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("map_topic", "/map")
            self.declare_parameter("image_topic", "/camera_node/image_raw")
            self.declare_parameter("camera_info_topic", "/camera_node/camera_info")
            self.declare_parameter("camera_status_topic", "/mission_api/v2/camera/status")
            self.declare_parameter("lidar_status_topic", "/mission_api/v2/lidar/status")
            self.declare_parameter(
                "localization_status_topic", "/mission_api/v2/localization/status"
            )
            self.declare_parameter("semantic_map_status_topic", "/mission_api/v2/map/status")
            self.declare_parameter(
                "enrollment_dir", "~/.local/share/sphero_rvr/face-enrollment"
            )
            self.declare_parameter(
                "evidence_dir", "~/.local/state/sphero_rvr/stationary-evidence"
            )
            self.declare_parameter("stationary_session", True)
            self.declare_parameter("camera_process_period_s", 0.3)
            self.declare_parameter("face_match_threshold", 44.0)
            self._stationary_session = bool(
                self.get_parameter("stationary_session").value
            )
            self._lock = threading.RLock()
            self._latest_scan: Optional[dict[str, Any]] = None
            self._latest_occupancy: dict[str, Any] = {}
            self._latest_map_bounds = {
                "origin": {"x_m": -3.0, "y_m": -3.0},
                "width_m": 6.0,
                "height_m": 6.0,
            }
            self._latest_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0}
            self._latest_tracks: list[dict[str, Any]] = []
            self._latest_uncertain_track = ""
            self._camera_matrix: Optional[tuple[float, ...]] = None
            self._previous_gray: Optional[Any] = None
            self._last_camera_process_s = 0.0
            self._camera_processing = False
            self._scan_sequence = 0
            self._frame_sequence = 0
            self._map_revision = 0
            self._tracks = SemanticTrackStore()
            self._evidence_dir = Path(
                str(self.get_parameter("evidence_dir").value)
            ).expanduser()
            self._evidence_dir.mkdir(parents=True, exist_ok=True)
            cascade_path = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
            self._face_cascade = cv2.CascadeClassifier(cascade_path)
            if self._face_cascade.empty():
                raise RuntimeError(f"face cascade unavailable: {cascade_path}")
            self._enrollment = ExplicitFaceEnrollment(
                Path(str(self.get_parameter("enrollment_dir").value)),
                cascade=self._face_cascade,
                threshold=float(self.get_parameter("face_match_threshold").value),
            )
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(
                self._tf_buffer, self, spin_thread=False
            )
            self._camera_pub = self.create_publisher(
                String, str(self.get_parameter("camera_status_topic").value), 10
            )
            self._lidar_pub = self.create_publisher(
                String, str(self.get_parameter("lidar_status_topic").value), 10
            )
            self._localization_pub = self.create_publisher(
                String, str(self.get_parameter("localization_status_topic").value), 10
            )
            self._map_pub = self.create_publisher(
                String, str(self.get_parameter("semantic_map_status_topic").value), 10
            )
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self._on_scan,
                qos_profile_sensor_data,
                callback_group=self._sensor_callbacks,
            )
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("map_topic").value),
                self._on_map,
                qos_profile_sensor_data,
                callback_group=self._sensor_callbacks,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("image_topic").value),
                self._on_image,
                qos_profile_sensor_data,
                callback_group=self._sensor_callbacks,
            )
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self._on_camera_info,
                qos_profile_sensor_data,
                callback_group=self._sensor_callbacks,
            )
            self.get_logger().info(
                (
                    "stationary perception ready: no motion authority; "
                    if self._stationary_session
                    else "moving semantic perception ready: no motion authority; "
                )
                + f"{self._enrollment.identity_count} explicitly enrolled identities"
            )

        def _on_camera_info(self, message: Any) -> None:
            values = tuple(float(value) for value in message.k)
            if len(values) == 9 and values[0] > 0.0 and values[4] > 0.0:
                with self._lock:
                    self._camera_matrix = values

        def _localization_from_tf(
            self,
            *,
            stamp_s: float,
            map_id: str,
            quality: float = 0.8,
            resolution_m: float = 0.05,
        ) -> dict[str, Any]:
            source = (
                "slam_toolbox_stationary"
                if self._stationary_session
                else "slam_toolbox_moving"
            )
            try:
                transform = self._tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                yaw = math.atan2(
                    2.0
                    * (
                        float(rotation.w) * float(rotation.z)
                        + float(rotation.x) * float(rotation.y)
                    ),
                    1.0
                    - 2.0
                    * (
                        float(rotation.y) ** 2
                        + float(rotation.z) ** 2
                    ),
                )
                self._latest_pose = {
                    "x_m": float(translation.x),
                    "y_m": float(translation.y),
                    "yaw_deg": math.degrees(yaw),
                }
                localization = LocalizationEstimate(
                    state=LocalizationState.VALID,
                    source=source,
                    pose=Pose2D(
                        x_m=float(translation.x),
                        y_m=float(translation.y),
                        yaw_rad=yaw,
                        stamp_s=stamp_s,
                        frame_id="map",
                    ),
                    quality=max(0.0, min(1.0, float(quality))),
                    covariance_xy_m2=float(resolution_m) ** 2,
                    covariance_yaw_rad2=0.0025,
                    detail=(
                        (
                            "Live slam_toolbox map->base_link pose with a "
                            "truthful static odom->base_link transform for "
                            "this immobile Stationary perception session."
                        )
                        if self._stationary_session
                        else (
                            "Live slam_toolbox map->base_link pose driven by "
                            "the rover's authoritative moving odometry."
                        )
                    ),
                ).to_json_dict()
            except TransformException as exc:
                localization = LocalizationEstimate(
                    state=LocalizationState.LOST,
                    source=source,
                    pose=None,
                    quality=0.0,
                    covariance_xy_m2=None,
                    covariance_yaw_rad2=None,
                    detail=(
                        "slam_toolbox transform unavailable: "
                        f"{exc.__class__.__name__}"
                    ),
                ).to_json_dict()
            localization.update(
                {
                    "map_id": str(map_id),
                    "stamp_s": stamp_s,
                    "stationary_session": self._stationary_session,
                    "motion_authority": False,
                    "physical_execution_enabled": False,
                }
            )
            return localization

        def _on_scan(self, message: Any) -> None:
            with self._lock:
                self._scan_sequence += 1
                stamp_s = _stamp_s(message)
                occupancy = scan_occupancy(
                    message.ranges,
                    angle_min=float(message.angle_min),
                    angle_increment=float(message.angle_increment),
                    range_min=float(message.range_min),
                    range_max=float(message.range_max),
                )
                scan_id = f"live-scan-{self._scan_sequence:08d}"
                lidar = {
                    "schema": "sphero_rvr.live_lidar.v1",
                    "scan_id": scan_id,
                    "stamp_s": stamp_s,
                    "sample_count": len(message.ranges),
                    "angle_min_rad": float(message.angle_min),
                    "angle_increment_rad": float(message.angle_increment),
                    "range_min_m": float(message.range_min),
                    "range_max_m": float(message.range_max),
                    "raw_scan_occupancy_preview": occupancy,
                    "occupancy_source": "slam_toolbox:/map",
                    "motion_authority": False,
                    "physical_execution_enabled": False,
                }
                self._latest_scan = {
                    "ranges": tuple(float(value) for value in message.ranges),
                    "angle_min": float(message.angle_min),
                    "angle_increment": float(message.angle_increment),
                    "range_min": float(message.range_min),
                    "range_max": float(message.range_max),
                }
                self._publish(self._lidar_pub, lidar)

        def _on_map(self, message: Any) -> None:
            with self._lock:
                self._map_revision += 1
                width = int(message.info.width)
                height = int(message.info.height)
                resolution = float(message.info.resolution)
                if width <= 0 or height <= 0 or resolution <= 0.0:
                    return
                origin = message.info.origin
                orientation = origin.orientation
                origin_yaw = math.atan2(
                    2.0
                    * (
                        float(orientation.w) * float(orientation.z)
                        + float(orientation.x) * float(orientation.y)
                    ),
                    1.0
                    - 2.0
                    * (
                        float(orientation.y) ** 2
                        + float(orientation.z) ** 2
                    ),
                )
                occupied_indices = [
                    index
                    for index, value in enumerate(message.data)
                    if int(value) >= 50
                ]
                stride = max(1, math.ceil(len(occupied_indices) / 180))
                points: list[dict[str, float]] = []
                cosine = math.cos(origin_yaw)
                sine = math.sin(origin_yaw)
                for index in occupied_indices[::stride][:180]:
                    column = index % width
                    row = index // width
                    local_x = (column + 0.5) * resolution
                    local_y = (row + 0.5) * resolution
                    points.append(
                        {
                            "x_m": round(
                                float(origin.position.x)
                                + local_x * cosine
                                - local_y * sine,
                                4,
                            ),
                            "y_m": round(
                                float(origin.position.y)
                                + local_x * sine
                                + local_y * cosine,
                                4,
                            ),
                        }
                    )
                stamp_s = _stamp_s(message)
                map_id = f"slam-map-{stamp_s:.6f}"
                self._latest_occupancy = {
                    "map_id": map_id,
                    "stamp_s": stamp_s,
                    "frame_id": str(message.header.frame_id or "map"),
                    "resolution_m": resolution,
                    "width_cells": width,
                    "height_cells": height,
                    "occupied_cell_count": len(occupied_indices),
                    "occupied_points": points,
                    "source": "slam_toolbox",
                }
                self._latest_map_bounds = {
                    "origin": {
                        "x_m": float(origin.position.x),
                        "y_m": float(origin.position.y),
                    },
                    "width_m": width * resolution,
                    "height_m": height * resolution,
                }
                known_cells = sum(
                    1 for value in message.data if int(value) >= 0
                )
                known_ratio = known_cells / max(1, len(message.data))
                localization = self._localization_from_tf(
                    stamp_s=stamp_s,
                    map_id=map_id,
                    quality=min(1.0, 0.65 + known_ratio),
                    resolution_m=resolution,
                )
                self._publish(self._localization_pub, localization)
                self._publish(
                    self._map_pub,
                    self._semantic_map(
                        self._latest_tracks,
                        uncertain_track=self._latest_uncertain_track,
                        stamp_s=stamp_s,
                    ),
                )

        def _on_image(self, message: Any) -> None:
            observed_at_s = time.time()
            period = float(self.get_parameter("camera_process_period_s").value)
            with self._lock:
                if (
                    self._camera_processing
                    or observed_at_s - self._last_camera_process_s < period
                ):
                    return
                self._camera_processing = True
                self._last_camera_process_s = observed_at_s
                self._frame_sequence += 1
                frame_id = f"live-camera-{self._frame_sequence:08d}"
            try:
                frame_rgb = self._decode_rgb(message)
            except ValueError as exc:
                self.get_logger().warning(f"camera frame rejected: {exc}")
                with self._lock:
                    self._camera_processing = False
                return
            try:
                stamp_s = _stamp_s(message)
                with self._lock:
                    map_id = str(
                        self._latest_occupancy.get("map_id", "")
                    )
                    resolution_m = float(
                        self._latest_occupancy.get("resolution_m", 0.05)
                    )
                    localization = self._localization_from_tf(
                        stamp_s=stamp_s,
                        map_id=map_id,
                        resolution_m=resolution_m,
                    )
                self._publish(self._localization_pub, localization)
                gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
                detections = self._detections(frame_rgb, gray, frame_id)
                with self._lock:
                    self._map_revision += 1
                    tracks = self._tracks.update(
                        detections,
                        frame_width=int(message.width),
                        observed_at_s=observed_at_s,
                    )
                    self._latest_tracks = list(tracks)
                for detection in detections:
                    matched = min(
                        (
                            track
                            for track in tracks
                            if track["kind"] == detection["kind"]
                        ),
                        key=lambda track: math.hypot(
                            float(track["x_m"]) - float(detection["x_m"]),
                            float(track["y_m"]) - float(detection["y_m"]),
                        ),
                        default=None,
                    )
                    detection["track_id"] = (
                        "" if matched is None else str(matched["track_id"])
                    )
                    detection.pop("center_x", None)
                    detection.pop("center_y", None)
                    detection.pop("x_m", None)
                    detection.pop("y_m", None)
                    detection.pop("uncertainty_m", None)
                uncertain_track = next(
                    (
                        str(item["track_id"])
                        for item in detections
                        if item.get("status") == "review" and item.get("track_id")
                    ),
                    "",
                )
                with self._lock:
                    self._latest_uncertain_track = uncertain_track
                annotated = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                for detection in detections:
                    box = detection["bbox"]
                    color = (0, 180, 255) if detection["label"] == "unknown" else (90, 220, 90)
                    cv2.rectangle(
                        annotated,
                        (int(box["x"]), int(box["y"])),
                        (
                            int(box["x"] + box["width"]),
                            int(box["y"] + box["height"]),
                        ),
                        color,
                        2,
                    )
                    cv2.putText(
                        annotated,
                        f"{detection['track_id']} {detection['label']}",
                        (int(box["x"]), max(18, int(box["y"]) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        1,
                        cv2.LINE_AA,
                    )
                thumbnail = cv2.resize(annotated, (320, 240))
                encoded_ok, encoded = cv2.imencode(
                    ".jpg", thumbnail, [int(cv2.IMWRITE_JPEG_QUALITY), 72]
                )
                thumbnail_data_url = (
                    ""
                    if not encoded_ok
                    else "data:image/jpeg;base64,"
                    + base64.b64encode(encoded.tobytes()).decode("ascii")
                )
                camera = {
                    "schema": "sphero_rvr.live_camera_perception.v1",
                    "frame_id": frame_id,
                    "stamp_s": stamp_s,
                    "width": int(message.width),
                    "height": int(message.height),
                    "encoding": "rgb8",
                    "calibrated": self._camera_matrix is not None,
                    "enrollment_identity_count": self._enrollment.identity_count,
                    "detections": detections,
                    "tracks": tracks,
                    "uncertain_track_id": uncertain_track,
                    "thumbnail_data_url": thumbnail_data_url,
                    "motion_authority": False,
                    "physical_execution_enabled": False,
                }
                semantic_map = self._semantic_map(
                    tracks, uncertain_track=uncertain_track, stamp_s=stamp_s
                )
                self._publish(self._camera_pub, camera)
                self._publish(self._map_pub, semantic_map)
            finally:
                with self._lock:
                    self._camera_processing = False

        def _decode_rgb(self, message: Any) -> Any:
            width = int(message.width)
            height = int(message.height)
            step = int(message.step)
            if width <= 0 or height <= 0 or step < width * 3:
                raise ValueError("invalid image dimensions or row step")
            raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
            if raw.size != height * step:
                raise ValueError("image byte count does not match padded row step")
            packed = raw.reshape(height, step)[:, : width * 3].reshape(height, width, 3)
            encoding = str(message.encoding).lower()
            if encoding == "rgb8":
                return packed.copy()
            if encoding in {"bgr8", "bgr888"}:
                return cv2.cvtColor(packed, cv2.COLOR_BGR2RGB)
            raise ValueError(f"unsupported camera encoding: {message.encoding}")

        def _detections(
            self, frame_rgb: Any, gray: Any, frame_id: str
        ) -> list[dict[str, Any]]:
            height, width = gray.shape[:2]
            result: list[dict[str, Any]] = []
            previous_gray = self._previous_gray
            self._previous_gray = gray.copy()
            shoes = detect_shoes_in_rgb(
                width,
                height,
                frame_rgb.tobytes(),
                DetectorThresholds(accept=0.70, review=0.45),
                stride=8,
            )
            for index, shoe in enumerate(shoes, start=1):
                evidence_id = f"{frame_id}-shoe-{index:02d}"
                position = self._position_for_pixel(
                    shoe.bbox.x + shoe.bbox.width / 2.0
                )
                if position is None:
                    continue
                result.append(
                    {
                        "detection_id": evidence_id,
                        "evidence_id": evidence_id,
                        "kind": "object",
                        "label": "shoe" if shoe.status == "accepted" else "possible_shoe",
                        "confidence": shoe.confidence,
                        "status": shoe.status,
                        "bbox": {
                            "x": shoe.bbox.x,
                            "y": shoe.bbox.y,
                            "width": shoe.bbox.width,
                            "height": shoe.bbox.height,
                        },
                        "center_x": shoe.bbox.x + shoe.bbox.width / 2.0,
                        "center_y": shoe.bbox.y + shoe.bbox.height / 2.0,
                        **position,
                        "recognized_from_enrollment": False,
                        "enrollment_evidence_ids": [],
                    }
                )
            if previous_gray is not None and previous_gray.shape == gray.shape:
                difference = cv2.absdiff(previous_gray, gray)
                floor_top = int(height * 0.52)
                _threshold, motion_mask = cv2.threshold(
                    difference[floor_top:, :], 24, 255, cv2.THRESH_BINARY
                )
                motion_mask = cv2.morphologyEx(
                    motion_mask,
                    cv2.MORPH_OPEN,
                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
                )
                motion_mask = cv2.morphologyEx(
                    motion_mask,
                    cv2.MORPH_CLOSE,
                    cv2.getStructuringElement(cv2.MORPH_RECT, (17, 9)),
                )
                contours, _hierarchy = cv2.findContours(
                    motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                candidates: list[tuple[float, tuple[int, int, int, int]]] = []
                frame_area = float(width * height)
                for contour in contours:
                    x, local_y, box_width, box_height = cv2.boundingRect(contour)
                    y = local_y + floor_top
                    box_area = float(box_width * box_height)
                    aspect = box_width / max(1.0, float(box_height))
                    if (
                        frame_area * 0.002 <= box_area <= frame_area * 0.16
                        and 0.65 <= aspect <= 6.5
                        and y + box_height / 2.0 >= height * 0.58
                    ):
                        candidates.append(
                            (
                                float(cv2.contourArea(contour)),
                                (x, y, box_width, box_height),
                            )
                        )
                if candidates:
                    _area, (x, y, box_width, box_height) = max(candidates)
                    evidence_id = f"{frame_id}-moving-object-01"
                    position = self._position_for_pixel(x + box_width / 2.0)
                    if position is not None:
                        result.append(
                            {
                                "detection_id": evidence_id,
                                "evidence_id": evidence_id,
                                "kind": "object",
                                "label": "moving_object",
                                "confidence": 0.62,
                                "status": "review",
                                "bbox": {
                                    "x": x,
                                    "y": y,
                                    "width": box_width,
                                    "height": box_height,
                                },
                                "center_x": x + box_width / 2.0,
                                "center_y": y + box_height / 2.0,
                                **position,
                                "recognized_from_enrollment": False,
                                "enrollment_evidence_ids": [],
                            }
                        )
            faces = self._face_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(48, 48)
            )
            for index, (x, y, box_width, box_height) in enumerate(faces, start=1):
                evidence_id = f"{frame_id}-face-{index:02d}"
                identity, confidence, recognized, enrollment_ids = self._enrollment.recognize(
                    gray[y : y + box_height, x : x + box_width]
                )
                position = self._position_for_pixel(x + box_width / 2.0)
                if position is None:
                    continue
                result.append(
                    {
                        "detection_id": evidence_id,
                        "evidence_id": evidence_id,
                        "kind": "face",
                        "label": identity if recognized else "unknown",
                        "confidence": confidence,
                        "status": "accepted" if recognized else "unknown",
                        "bbox": {
                            "x": int(x),
                            "y": int(y),
                            "width": int(box_width),
                            "height": int(box_height),
                        },
                        "center_x": x + box_width / 2.0,
                        "center_y": y + box_height / 2.0,
                        **position,
                        "recognized_from_enrollment": recognized,
                        "enrollment_evidence_ids": enrollment_ids,
                    }
                )
            return select_trackable_detections(result)

        def _position_for_pixel(
            self, pixel_x: float
        ) -> Optional[dict[str, float]]:
            scan = self._latest_scan
            matrix = self._camera_matrix
            if scan is None or matrix is None:
                return None
            fx = float(matrix[0])
            cx = float(matrix[2])
            bearing = math.atan2(cx - float(pixel_x), fx)
            index = round((bearing - scan["angle_min"]) / scan["angle_increment"])
            samples = [
                scan["ranges"][candidate]
                for candidate in range(max(0, index - 2), min(len(scan["ranges"]), index + 3))
                if math.isfinite(scan["ranges"][candidate])
                and scan["range_min"] <= scan["ranges"][candidate] <= scan["range_max"]
            ]
            if not samples:
                return None
            distance = min(samples)
            rover_x = float(self._latest_pose["x_m"])
            rover_y = float(self._latest_pose["y_m"])
            rover_yaw = math.radians(float(self._latest_pose["yaw_deg"]))
            map_bearing = rover_yaw + bearing
            return {
                "x_m": round(rover_x + distance * math.cos(map_bearing), 4),
                "y_m": round(rover_y + distance * math.sin(map_bearing), 4),
                "uncertainty_m": 0.18,
            }

        def _semantic_map(
            self,
            tracks: Sequence[Mapping[str, Any]],
            *,
            uncertain_track: str,
            stamp_s: float,
        ) -> dict[str, Any]:
            occupancy_points = self._latest_occupancy.get("occupied_points", [])
            obstacles = [
                {
                    "obstacle_id": f"occupancy-{index:03d}",
                    "x_m": float(point["x_m"]),
                    "y_m": float(point["y_m"]),
                    "width_m": 0.04,
                    "height_m": 0.04,
                    "label": "live lidar occupancy",
                }
                for index, point in enumerate(occupancy_points)
            ]
            objects = [
                {
                    "object_id": str(track["track_id"]),
                    "label": str(track["label"]),
                    "x_m": float(track["x_m"]),
                    "y_m": float(track["y_m"]),
                    "confidence": float(track["confidence"]),
                    "evidence_ref": (
                        track.get("evidence_ids", [""])[-1]
                        if track.get("evidence_ids")
                        else ""
                    ),
                }
                for track in tracks
            ]
            return {
                "schema": "sphero_rvr.live_semantic_map.v1",
                "revision": self._map_revision,
                "stamp_s": stamp_s,
                "tracks": list(tracks),
                "uncertain_track_id": uncertain_track,
                "occupancy": dict(self._latest_occupancy),
                "map": {
                    "frame": "map",
                    "bounds": dict(self._latest_map_bounds),
                    "rover": dict(self._latest_pose),
                    "proposed_route": [
                        {
                            "x_m": float(self._latest_pose["x_m"]),
                            "y_m": float(self._latest_pose["y_m"]),
                        }
                    ],
                    "traveled_path": [
                        {
                            "x_m": float(self._latest_pose["x_m"]),
                            "y_m": float(self._latest_pose["y_m"]),
                        }
                    ],
                    "obstacles": obstacles,
                    "objects": objects,
                    "occupancy_available": bool(occupancy_points),
                    "occupancy_source": "slam_toolbox",
                    "stationary": self._stationary_session,
                },
                "motion_authority": False,
                "physical_execution_enabled": False,
            }

        @staticmethod
        def _publish(publisher: Any, payload: Mapping[str, Any]) -> None:
            message = String()
            message.data = json.dumps(
                dict(payload), sort_keys=True, separators=(",", ":"), allow_nan=False
            )
            publisher.publish(message)

    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = StationaryPerceptionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
