"""Pi-only, no-motion evidence tooling for Milestone 7 Phase 1."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
from pathlib import Path
import re
import resource
import statistics
import sys
import time
from typing import Any, Optional, Sequence

from .hierarchical_exploration import (
    FrontierDetectionConfig,
    detect_frontiers,
    load_slam_toolbox_map,
)
from .mission_api import MissionValidationError


AUDIT_SCHEMA = "sphero_rvr.m7_phase1_wfd_audit.v1"
SOURCE_SHA_PATTERN = re.compile(r"^[0-9a-f]{40}$")
DEFAULT_ROBOT_X_M = -0.028
DEFAULT_ROBOT_Y_M = 0.941
DEFAULT_REPETITIONS = 50


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Benchmark deterministic WFD from a recorded map without ROS, "
            "sensors, serial transport, or motion authority."
        )
    )
    parser.add_argument("map", type=Path)
    parser.add_argument("--source-sha", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    parser.add_argument("--robot-x-m", type=float, default=DEFAULT_ROBOT_X_M)
    parser.add_argument("--robot-y-m", type=float, default=DEFAULT_ROBOT_Y_M)
    return parser


def validate_source_sha(value: str) -> str:
    supplied = str(value).strip()
    if not SOURCE_SHA_PATTERN.fullmatch(supplied):
        raise MissionValidationError(
            "Milestone 7 Phase 1 source SHA must be exactly 40 lowercase hex characters"
        )
    return supplied


def _image_path(map_yaml: Path) -> Path:
    image_value: Optional[str] = None
    for raw_line in map_yaml.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key.strip() == "image":
            image_value = value.strip().strip("\"'")
            break
    if not image_value:
        raise MissionValidationError("slam_toolbox map YAML has no image path")
    return (map_yaml.parent / image_value).resolve()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _percentile(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("percentile requires at least one value")
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(percentile)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _max_rss_kib() -> int:
    value = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    # Linux reports KiB; macOS reports bytes.
    return value if platform.system() != "Darwin" else math.ceil(value / 1024)


def benchmark_wfd(
    map_yaml: str | Path,
    *,
    source_sha: str,
    repetitions: int = DEFAULT_REPETITIONS,
    robot_x_m: float = DEFAULT_ROBOT_X_M,
    robot_y_m: float = DEFAULT_ROBOT_Y_M,
) -> dict[str, Any]:
    """Run deterministic recorded-map WFD and return machine-readable evidence."""

    source = validate_source_sha(source_sha)
    yaml_path = Path(map_yaml).resolve()
    if not yaml_path.is_file():
        raise MissionValidationError("recorded map YAML does not exist")
    if int(repetitions) <= 0:
        raise MissionValidationError("WFD repetitions must be positive")
    if not math.isfinite(float(robot_x_m)) or not math.isfinite(
        float(robot_y_m)
    ):
        raise MissionValidationError("WFD robot pose must be finite")

    image_path = _image_path(yaml_path)
    if not image_path.is_file():
        raise MissionValidationError("recorded map image does not exist")
    grid = load_slam_toolbox_map(
        yaml_path,
        map_id="rvr-room-20260626",
    )
    config = FrontierDetectionConfig(
        minimum_frontier_cells=3,
        minimum_clearance_m=0.10,
    )
    durations_ms: list[float] = []
    signature_runs: list[tuple[str, ...]] = []
    candidates = ()
    for _ in range(int(repetitions)):
        started_ns = time.perf_counter_ns()
        candidates = detect_frontiers(
            grid,
            robot_x_m=float(robot_x_m),
            robot_y_m=float(robot_y_m),
            config=config,
        )
        durations_ms.append((time.perf_counter_ns() - started_ns) / 1_000_000.0)
        signature_runs.append(tuple(item.signature for item in candidates))

    first_signatures = signature_runs[0]
    deterministic = all(
        signatures == first_signatures for signatures in signature_runs
    )
    passed = deterministic and len(first_signatures) == 13
    result = {
        "schema": AUDIT_SCHEMA,
        "source_sha": source,
        "recorded_at_utc": time.strftime(
            "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
        ),
        "host": {
            "hostname": platform.node(),
            "architecture": platform.machine(),
            "platform": platform.platform(),
            "python": platform.python_version(),
        },
        "input": {
            "map_yaml": str(yaml_path),
            "map_image": str(image_path),
            "map_yaml_sha256": _sha256(yaml_path),
            "map_image_sha256": _sha256(image_path),
            "map_revision": grid.revision,
            "dimensions": [grid.width, grid.height],
            "resolution_m": grid.resolution_m,
            "robot_pose": {
                "x_m": float(robot_x_m),
                "y_m": float(robot_y_m),
            },
        },
        "parameters": {
            "algorithm": "project-owned deterministic WFD",
            "repetitions": int(repetitions),
            "minimum_frontier_cells": config.minimum_frontier_cells,
            "minimum_clearance_m": config.minimum_clearance_m,
            "connectivity": config.connectivity,
        },
        "result": {
            "passed": passed,
            "deterministic": deterministic,
            "frontier_count": len(first_signatures),
            "frontier_signatures": list(first_signatures),
            "duration_ms": {
                "minimum": min(durations_ms),
                "mean": statistics.fmean(durations_ms),
                "p50": _percentile(durations_ms, 0.50),
                "p95": _percentile(durations_ms, 0.95),
                "maximum": max(durations_ms),
                "samples": durations_ms,
            },
            "maximum_rss_kib": _max_rss_kib(),
        },
        "authority": {
            "recorded_data_only": True,
            "ros_started": False,
            "live_sensors_started": False,
            "serial_transport_started": False,
            "driver_started": False,
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
    }
    return result


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = benchmark_wfd(
            args.map,
            source_sha=args.source_sha,
            repetitions=args.repetitions,
            robot_x_m=args.robot_x_m,
            robot_y_m=args.robot_y_m,
        )
        payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(payload, encoding="utf-8")
        sys.stdout.write(payload)
        return 0 if report["result"]["passed"] else 1
    except Exception as exc:
        failure = {
            "schema": AUDIT_SCHEMA,
            "passed": False,
            "error": f"{type(exc).__name__}: {exc}",
            "authority": {
                "ros_started": False,
                "live_sensors_started": False,
                "serial_transport_started": False,
                "driver_started": False,
                "motion_authority": False,
                "physical_execution_enabled": False,
            },
        }
        sys.stdout.write(json.dumps(failure, indent=2, sort_keys=True) + "\n")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
