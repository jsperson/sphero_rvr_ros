"""Read-only lidar evidence for independent motion validation.

The live motion controller remains odometry based. This module captures
stationary LaserScan sets and compares a fixed planar target before and after a
run. A wall fit is deliberately used instead of one beam: a single range can
silently switch to a different object when the rover translates or turns.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence


CAPTURE_SCHEMA = "sphero_rvr.lidar_stationary_capture.v1"
COMPARISON_SCHEMA = "sphero_rvr.lidar_motion_comparison.v1"


class LidarMotionValidationError(ValueError):
    """Raised when lidar evidence is malformed or too weak to use."""


def _finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise LidarMotionValidationError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise LidarMotionValidationError(f"{name} must be finite")
    return result


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


@dataclass(frozen=True)
class LaserScanSnapshot:
    frame_id: str
    stamp_s: float
    receipt_time_s: float
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    ranges_m: tuple[Optional[float], ...]

    @classmethod
    def from_json_dict(cls, payload: Mapping[str, Any]) -> "LaserScanSnapshot":
        frame_id = str(payload.get("frame_id", "")).strip()
        if not frame_id:
            raise LidarMotionValidationError("scan frame_id is required")
        angle_increment = _finite_float(
            payload.get("angle_increment_rad"), "angle_increment_rad"
        )
        if angle_increment <= 0.0:
            raise LidarMotionValidationError("angle_increment_rad must be positive")
        range_min = _finite_float(payload.get("range_min_m"), "range_min_m")
        range_max = _finite_float(payload.get("range_max_m"), "range_max_m")
        if range_min < 0.0 or range_max <= range_min:
            raise LidarMotionValidationError("scan range bounds are invalid")
        raw_ranges = payload.get("ranges_m")
        if not isinstance(raw_ranges, list) or len(raw_ranges) < 3:
            raise LidarMotionValidationError("ranges_m must contain at least three values")
        ranges: list[Optional[float]] = []
        for raw in raw_ranges:
            if raw is None:
                ranges.append(None)
                continue
            try:
                value = float(raw)
            except (TypeError, ValueError):
                ranges.append(None)
                continue
            ranges.append(value if math.isfinite(value) else None)
        return cls(
            frame_id=frame_id,
            stamp_s=_finite_float(payload.get("stamp_s"), "stamp_s"),
            receipt_time_s=_finite_float(
                payload.get("receipt_time_s"), "receipt_time_s"
            ),
            angle_min_rad=_finite_float(
                payload.get("angle_min_rad"), "angle_min_rad"
            ),
            angle_increment_rad=angle_increment,
            range_min_m=range_min,
            range_max_m=range_max,
            ranges_m=tuple(ranges),
        )

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "stamp_s": self.stamp_s,
            "receipt_time_s": self.receipt_time_s,
            "angle_min_rad": self.angle_min_rad,
            "angle_increment_rad": self.angle_increment_rad,
            "range_min_m": self.range_min_m,
            "range_max_m": self.range_max_m,
            "ranges_m": list(self.ranges_m),
        }

    def points(
        self, *, sector_min_rad: float, sector_max_rad: float
    ) -> tuple[tuple[float, float], ...]:
        if sector_max_rad <= sector_min_rad:
            raise LidarMotionValidationError("sector maximum must exceed minimum")
        points: list[tuple[float, float]] = []
        for index, distance in enumerate(self.ranges_m):
            if distance is None:
                continue
            if distance < self.range_min_m or distance > self.range_max_m:
                continue
            angle = self.angle_min_rad + index * self.angle_increment_rad
            if sector_min_rad <= angle <= sector_max_rad:
                points.append((distance * math.cos(angle), distance * math.sin(angle)))
        return tuple(points)


@dataclass(frozen=True)
class WallFit:
    distance_m: float
    normal_angle_rad: float
    inlier_count: int
    candidate_count: int
    inlier_fraction: float
    residual_rms_m: float

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "distance_m": self.distance_m,
            "normal_angle_deg": math.degrees(self.normal_angle_rad),
            "inlier_count": self.inlier_count,
            "candidate_count": self.candidate_count,
            "inlier_fraction": self.inlier_fraction,
            "residual_rms_m": self.residual_rms_m,
        }


def _sample_points(
    points: Sequence[tuple[float, float]], limit: int = 36
) -> tuple[tuple[float, float], ...]:
    if len(points) <= limit:
        return tuple(points)
    step = (len(points) - 1) / float(limit - 1)
    return tuple(points[round(index * step)] for index in range(limit))


def _line_from_pair(
    first: tuple[float, float], second: tuple[float, float]
) -> Optional[tuple[float, float, float]]:
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    length = math.hypot(dx, dy)
    if length < 0.10:
        return None
    nx, ny = -dy / length, dx / length
    distance = nx * first[0] + ny * first[1]
    if distance < 0.0:
        nx, ny, distance = -nx, -ny, -distance
    return nx, ny, distance


def _tls_line(
    points: Sequence[tuple[float, float]]
) -> tuple[float, float, float]:
    mean_x = statistics.fmean(point[0] for point in points)
    mean_y = statistics.fmean(point[1] for point in points)
    covariance_xx = statistics.fmean(
        (point[0] - mean_x) ** 2 for point in points
    )
    covariance_yy = statistics.fmean(
        (point[1] - mean_y) ** 2 for point in points
    )
    covariance_xy = statistics.fmean(
        (point[0] - mean_x) * (point[1] - mean_y) for point in points
    )
    tangent_angle = 0.5 * math.atan2(
        2.0 * covariance_xy, covariance_xx - covariance_yy
    )
    normal_angle = tangent_angle + math.pi / 2.0
    nx, ny = math.cos(normal_angle), math.sin(normal_angle)
    distance = nx * mean_x + ny * mean_y
    if distance < 0.0:
        nx, ny, distance = -nx, -ny, -distance
    return nx, ny, distance


def fit_fixed_wall(
    snapshot: LaserScanSnapshot,
    *,
    sector_min_deg: float = -80.0,
    sector_max_deg: float = 80.0,
    inlier_tolerance_m: float = 0.025,
    min_inliers: int = 18,
    min_inlier_fraction: float = 0.35,
) -> WallFit:
    """Fit the dominant planar target in one scan using deterministic RANSAC."""

    tolerance = _finite_float(inlier_tolerance_m, "inlier_tolerance_m")
    if tolerance <= 0.0:
        raise LidarMotionValidationError("inlier_tolerance_m must be positive")
    if (
        isinstance(min_inliers, bool)
        or int(min_inliers) != min_inliers
        or min_inliers < 2
    ):
        raise LidarMotionValidationError("min_inliers must be an integer of at least two")
    fraction = _finite_float(min_inlier_fraction, "min_inlier_fraction")
    if not 0.0 < fraction <= 1.0:
        raise LidarMotionValidationError(
            "min_inlier_fraction must be greater than zero and at most one"
        )
    points = snapshot.points(
        sector_min_rad=math.radians(
            _finite_float(sector_min_deg, "sector_min_deg")
        ),
        sector_max_rad=math.radians(
            _finite_float(sector_max_deg, "sector_max_deg")
        ),
    )
    if len(points) < min_inliers:
        raise LidarMotionValidationError(
            f"only {len(points)} valid target-sector points; need {min_inliers}"
        )

    candidates = _sample_points(points)
    best_inliers: tuple[tuple[float, float], ...] = ()
    best_residual = math.inf
    for first_index, first in enumerate(candidates):
        for second in candidates[first_index + 1 :]:
            line = _line_from_pair(first, second)
            if line is None:
                continue
            nx, ny, distance = line
            inliers = tuple(
                point
                for point in points
                if abs(nx * point[0] + ny * point[1] - distance) <= tolerance
            )
            if len(inliers) < len(best_inliers):
                continue
            residual = (
                statistics.fmean(
                    abs(nx * point[0] + ny * point[1] - distance)
                    for point in inliers
                )
                if inliers
                else math.inf
            )
            if len(inliers) > len(best_inliers) or residual < best_residual:
                best_inliers = inliers
                best_residual = residual

    if len(best_inliers) < min_inliers:
        raise LidarMotionValidationError(
            f"dominant wall has {len(best_inliers)} inliers; need {min_inliers}"
        )
    inlier_fraction = len(best_inliers) / len(points)
    if inlier_fraction < fraction:
        raise LidarMotionValidationError(
            f"dominant wall inlier fraction {inlier_fraction:.3f} is below "
            f"{fraction:.3f}"
        )

    nx, ny, distance = _tls_line(best_inliers)
    refined = tuple(
        point
        for point in points
        if abs(nx * point[0] + ny * point[1] - distance) <= tolerance
    )
    if len(refined) >= min_inliers:
        nx, ny, distance = _tls_line(refined)
        best_inliers = refined
    residual_rms = math.sqrt(
        statistics.fmean(
            (nx * point[0] + ny * point[1] - distance) ** 2
            for point in best_inliers
        )
    )
    return WallFit(
        distance_m=distance,
        normal_angle_rad=_normalize_angle(math.atan2(ny, nx)),
        inlier_count=len(best_inliers),
        candidate_count=len(points),
        inlier_fraction=len(best_inliers) / len(points),
        residual_rms_m=residual_rms,
    )


def _median_angle(values: Sequence[float]) -> float:
    reference = values[0]
    unwrapped = [
        reference + _normalize_angle(value - reference) for value in values
    ]
    return _normalize_angle(statistics.median(unwrapped))


def _median_absolute_deviation(values: Sequence[float]) -> float:
    center = statistics.median(values)
    return statistics.median(abs(value - center) for value in values)


def _aggregate_capture(
    payload: Mapping[str, Any],
    *,
    sector_min_deg: float,
    sector_max_deg: float,
    inlier_tolerance_m: float,
    min_inliers: int,
    min_inlier_fraction: float,
) -> dict[str, Any]:
    if payload.get("schema") != CAPTURE_SCHEMA:
        raise LidarMotionValidationError(
            f"capture schema must be {CAPTURE_SCHEMA!r}"
        )
    raw_scans = payload.get("scans")
    if not isinstance(raw_scans, list) or not raw_scans:
        raise LidarMotionValidationError("capture must contain stationary scans")
    scans = [LaserScanSnapshot.from_json_dict(item) for item in raw_scans]
    frames = {scan.frame_id for scan in scans}
    if len(frames) != 1:
        raise LidarMotionValidationError("capture scan frames must be identical")
    fits = [
        fit_fixed_wall(
            scan,
            sector_min_deg=sector_min_deg,
            sector_max_deg=sector_max_deg,
            inlier_tolerance_m=inlier_tolerance_m,
            min_inliers=min_inliers,
            min_inlier_fraction=min_inlier_fraction,
        )
        for scan in scans
    ]
    distances = [fit.distance_m for fit in fits]
    angles = [fit.normal_angle_rad for fit in fits]
    median_angle = _median_angle(angles)
    angle_deviations = [
        abs(_normalize_angle(angle - median_angle)) for angle in angles
    ]
    return {
        "frame_id": scans[0].frame_id,
        "scan_count": len(scans),
        "wall_distance_m": statistics.median(distances),
        "wall_distance_mad_m": _median_absolute_deviation(distances),
        "wall_normal_angle_deg": math.degrees(median_angle),
        "wall_normal_angle_mad_deg": math.degrees(
            statistics.median(angle_deviations)
        ),
        "median_inlier_count": statistics.median(
            fit.inlier_count for fit in fits
        ),
        "median_inlier_fraction": statistics.median(
            fit.inlier_fraction for fit in fits
        ),
        "median_residual_rms_m": statistics.median(
            fit.residual_rms_m for fit in fits
        ),
        "first_stamp_s": min(scan.stamp_s for scan in scans),
        "last_stamp_s": max(scan.stamp_s for scan in scans),
    }


def compare_stationary_captures(
    before: Mapping[str, Any],
    after: Mapping[str, Any],
    *,
    sector_min_deg: float = -80.0,
    sector_max_deg: float = 80.0,
    inlier_tolerance_m: float = 0.025,
    min_inliers: int = 18,
    min_inlier_fraction: float = 0.35,
) -> dict[str, Any]:
    """Compare the same fixed planar target before and after rover motion."""

    options = {
        "sector_min_deg": sector_min_deg,
        "sector_max_deg": sector_max_deg,
        "inlier_tolerance_m": inlier_tolerance_m,
        "min_inliers": min_inliers,
        "min_inlier_fraction": min_inlier_fraction,
    }
    before_estimate = _aggregate_capture(before, **options)
    after_estimate = _aggregate_capture(after, **options)
    if before_estimate["frame_id"] != after_estimate["frame_id"]:
        raise LidarMotionValidationError(
            "before and after captures must use the same lidar frame"
        )
    before_angle = math.radians(before_estimate["wall_normal_angle_deg"])
    after_angle = math.radians(after_estimate["wall_normal_angle_deg"])
    return {
        "schema": COMPARISON_SCHEMA,
        "measurement_role": "independent_validation_only",
        "motion_authority": False,
        "safety_authority": False,
        "target_assumption": "the same fixed planar target dominates both sectors",
        "before": before_estimate,
        "after": after_estimate,
        "lidar_wall_normal_displacement_m": (
            before_estimate["wall_distance_m"]
            - after_estimate["wall_distance_m"]
        ),
        "lidar_heading_change_deg": math.degrees(
            _normalize_angle(before_angle - after_angle)
        ),
        "limitations": [
            "one wall measures only translation normal to that wall",
            "moving objects or a different dominant wall invalidate the comparison",
            "use tape or full-scan localization for independent two-dimensional translation",
        ],
        "fit_options": options,
    }


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        payload = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise LidarMotionValidationError(f"cannot read {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise LidarMotionValidationError(f"{path} must contain a JSON object")
    return payload


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")
        temporary = Path(handle.name)
    temporary.replace(path)


def capture_stationary_scans(
    *,
    topic: str,
    scan_count: int,
    timeout_s: float,
) -> dict[str, Any]:
    """Capture LaserScan messages without publishing or opening motor devices."""

    if scan_count < 3:
        raise LidarMotionValidationError("scan_count must be at least three")
    if timeout_s <= 0.0:
        raise LidarMotionValidationError("timeout_s must be positive")
    try:
        import rclpy
        from sensor_msgs.msg import LaserScan
    except ImportError as exc:  # pragma: no cover - exercised only in ROS runtime
        raise LidarMotionValidationError(
            "capture requires a sourced ROS 2 environment with rclpy and sensor_msgs"
        ) from exc

    rclpy.init(args=None)
    node = rclpy.create_node("rvr_lidar_motion_capture")
    snapshots: list[LaserScanSnapshot] = []

    def on_scan(message: Any) -> None:
        if len(snapshots) >= scan_count:
            return
        ranges = tuple(
            float(value) if math.isfinite(float(value)) else None
            for value in message.ranges
        )
        snapshots.append(
            LaserScanSnapshot(
                frame_id=str(message.header.frame_id),
                stamp_s=(
                    float(message.header.stamp.sec)
                    + float(message.header.stamp.nanosec) / 1_000_000_000.0
                ),
                receipt_time_s=time.time(),
                angle_min_rad=float(message.angle_min),
                angle_increment_rad=float(message.angle_increment),
                range_min_m=float(message.range_min),
                range_max_m=float(message.range_max),
                ranges_m=ranges,
            )
        )

    subscription = node.create_subscription(LaserScan, topic, on_scan, 10)
    deadline = time.monotonic() + timeout_s
    try:
        while len(snapshots) < scan_count and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if len(snapshots) != scan_count:
        raise LidarMotionValidationError(
            f"received {len(snapshots)} of {scan_count} scans before timeout"
        )
    frames = {snapshot.frame_id for snapshot in snapshots}
    if len(frames) != 1 or not next(iter(frames)).strip():
        raise LidarMotionValidationError(
            "captured scans must share one nonempty frame"
        )
    return {
        "schema": CAPTURE_SCHEMA,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "topic": topic,
        "read_only": True,
        "scan_count": len(snapshots),
        "scans": [snapshot.to_json_dict() for snapshot in snapshots],
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture and compare fixed-wall lidar evidence for rover motion."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture = subparsers.add_parser(
        "capture", help="capture a stationary read-only LaserScan set"
    )
    capture.add_argument("output", type=Path)
    capture.add_argument("--topic", default="/scan")
    capture.add_argument("--scan-count", type=int, default=20)
    capture.add_argument("--timeout", type=float, default=8.0)

    compare = subparsers.add_parser(
        "compare", help="fit the same fixed wall before and after motion"
    )
    compare.add_argument("before", type=Path)
    compare.add_argument("after", type=Path)
    compare.add_argument("--output", type=Path)
    compare.add_argument("--sector-min-deg", type=float, default=-80.0)
    compare.add_argument("--sector-max-deg", type=float, default=80.0)
    compare.add_argument("--inlier-tolerance-m", type=float, default=0.025)
    compare.add_argument("--min-inliers", type=int, default=18)
    compare.add_argument("--min-inlier-fraction", type=float, default=0.35)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "capture":
            payload = capture_stationary_scans(
                topic=args.topic,
                scan_count=args.scan_count,
                timeout_s=args.timeout,
            )
            _write_json(args.output, payload)
        else:
            payload = compare_stationary_captures(
                _load_json(args.before),
                _load_json(args.after),
                sector_min_deg=args.sector_min_deg,
                sector_max_deg=args.sector_max_deg,
                inlier_tolerance_m=args.inlier_tolerance_m,
                min_inliers=args.min_inliers,
                min_inlier_fraction=args.min_inlier_fraction,
            )
            if args.output is not None:
                _write_json(args.output, payload)
        print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))
        return 0
    except LidarMotionValidationError as exc:
        print(f"lidar motion validation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
