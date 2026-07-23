from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from sphero_rvr_driver.lidar_motion_validation import (
    CAPTURE_SCHEMA,
    COMPARISON_SCHEMA,
    LaserScanSnapshot,
    LidarMotionValidationError,
    compare_stationary_captures,
    fit_fixed_wall,
    main,
)


def _wall_scan(
    *,
    distance_m: float,
    normal_angle_deg: float,
    stamp_s: float,
    outliers: bool = False,
) -> LaserScanSnapshot:
    angle_min = math.radians(-90.0)
    increment = math.radians(1.0)
    normal = math.radians(normal_angle_deg)
    ranges: list[float | None] = []
    for index in range(181):
        angle = angle_min + index * increment
        denominator = math.cos(angle - normal)
        if denominator <= 0.10:
            ranges.append(None)
            continue
        distance = distance_m / denominator
        tangent = distance * math.sin(angle - normal)
        ranges.append(distance if abs(tangent) <= 2.0 else None)
    if outliers:
        for index in range(11, 170, 19):
            ranges[index] = 0.35 + (index % 7) * 0.04
    return LaserScanSnapshot(
        frame_id="laser",
        stamp_s=stamp_s,
        receipt_time_s=stamp_s + 0.01,
        angle_min_rad=angle_min,
        angle_increment_rad=increment,
        range_min_m=0.15,
        range_max_m=16.0,
        ranges_m=tuple(ranges),
    )


def _capture(
    *, distance_m: float, normal_angle_deg: float, start_stamp_s: float
) -> dict[str, object]:
    scans = [
        _wall_scan(
            distance_m=distance_m + offset,
            normal_angle_deg=normal_angle_deg + offset * 5.0,
            stamp_s=start_stamp_s + index * 0.1,
            outliers=True,
        ).to_json_dict()
        for index, offset in enumerate((-0.001, 0.0, 0.001, -0.0005, 0.0005))
    ]
    return {
        "schema": CAPTURE_SCHEMA,
        "captured_at": "2026-07-23T00:00:00+00:00",
        "topic": "/scan",
        "read_only": True,
        "scan_count": len(scans),
        "scans": scans,
    }


def test_fixed_wall_fit_is_robust_to_sparse_range_outliers() -> None:
    fit = fit_fixed_wall(
        _wall_scan(
            distance_m=1.25,
            normal_angle_deg=-17.0,
            stamp_s=1.0,
            outliers=True,
        )
    )

    assert fit.distance_m == pytest.approx(1.25, abs=0.002)
    assert math.degrees(fit.normal_angle_rad) == pytest.approx(-17.0, abs=0.2)
    assert fit.inlier_count > 100
    assert fit.inlier_fraction > 0.8
    assert fit.residual_rms_m < 0.002


def test_stationary_capture_comparison_reports_forward_wall_normal_motion() -> None:
    result = compare_stationary_captures(
        _capture(distance_m=1.30, normal_angle_deg=0.0, start_stamp_s=10.0),
        _capture(distance_m=0.80, normal_angle_deg=0.0, start_stamp_s=20.0),
    )

    assert result["schema"] == COMPARISON_SCHEMA
    assert result["measurement_role"] == "independent_validation_only"
    assert result["motion_authority"] is False
    assert result["safety_authority"] is False
    assert result["lidar_wall_normal_displacement_m"] == pytest.approx(
        0.50, abs=0.003
    )
    assert result["lidar_heading_change_deg"] == pytest.approx(0.0, abs=0.2)
    assert result["before"]["scan_count"] == 5
    assert result["before"]["wall_distance_mad_m"] < 0.002


def test_stationary_capture_comparison_reports_left_turn_from_fixed_wall() -> None:
    result = compare_stationary_captures(
        _capture(distance_m=1.10, normal_angle_deg=12.0, start_stamp_s=10.0),
        _capture(distance_m=1.10, normal_angle_deg=-33.0, start_stamp_s=20.0),
    )

    assert result["lidar_heading_change_deg"] == pytest.approx(45.0, abs=0.2)
    assert result["lidar_wall_normal_displacement_m"] == pytest.approx(
        0.0, abs=0.003
    )


def test_capture_rejects_weak_or_malformed_evidence() -> None:
    scan = _wall_scan(distance_m=1.0, normal_angle_deg=0.0, stamp_s=1.0)
    weak = LaserScanSnapshot(
        frame_id=scan.frame_id,
        stamp_s=scan.stamp_s,
        receipt_time_s=scan.receipt_time_s,
        angle_min_rad=scan.angle_min_rad,
        angle_increment_rad=scan.angle_increment_rad,
        range_min_m=scan.range_min_m,
        range_max_m=scan.range_max_m,
        ranges_m=tuple(
            value if index in {80, 90, 100} else None
            for index, value in enumerate(scan.ranges_m)
        ),
    )
    with pytest.raises(LidarMotionValidationError, match="valid target-sector"):
        fit_fixed_wall(weak)

    malformed = _capture(
        distance_m=1.0, normal_angle_deg=0.0, start_stamp_s=1.0
    )
    malformed["schema"] = "wrong"
    with pytest.raises(LidarMotionValidationError, match="capture schema"):
        compare_stationary_captures(malformed, malformed)

    with pytest.raises(LidarMotionValidationError, match="min_inliers"):
        fit_fixed_wall(scan, min_inliers=1)
    with pytest.raises(LidarMotionValidationError, match="min_inlier_fraction"):
        fit_fixed_wall(scan, min_inlier_fraction=1.1)


def test_compare_cli_is_ros_independent_and_writes_strict_json(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    before = tmp_path / "before.json"
    after = tmp_path / "after.json"
    output = tmp_path / "comparison.json"
    before.write_text(
        json.dumps(
            _capture(distance_m=1.2, normal_angle_deg=5.0, start_stamp_s=1.0)
        )
    )
    after.write_text(
        json.dumps(
            _capture(distance_m=0.9, normal_angle_deg=-10.0, start_stamp_s=2.0)
        )
    )

    assert main(["compare", str(before), str(after), "--output", str(output)]) == 0
    result = json.loads(output.read_text())
    assert result["lidar_wall_normal_displacement_m"] == pytest.approx(
        0.3, abs=0.003
    )
    assert result["lidar_heading_change_deg"] == pytest.approx(15.0, abs=0.2)
    assert json.loads(capsys.readouterr().out)["schema"] == COMPARISON_SCHEMA
