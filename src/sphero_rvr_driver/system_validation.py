"""System-level validation helpers for ROS CI and no-motion corpora.

This module stays ROS-free so development hosts can validate the gates that the
real ROS/colcon CI job will exercise on GitHub and on the Pi.  It intentionally
models bag manifests, fake route/collision replay, and latency gates without
launching hardware or owning serial devices.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

from .collision_stop import CollisionState, CollisionStopConfig, ScanInput, Transform2D
from .live_route_runner import (
    LiveRouteConfig,
    LiveRouteRequest,
    LiveRouteState,
    RouteSegmentRequest,
    run_route_replay,
)
from .mission_api import ToolResultStatus
from .odometry import MotionPrimitiveConfig, OdomMotionState

REQUIRED_SYSTEM_CI_JOBS = (
    "python-unit-system-gates",
    "ros2-colcon-system-gates",
)

REQUIRED_CURRENT_SHA_TOPICS = (
    "/scan",
    "/odom",
    "/tf",
    "/tf_static",
    "/camera_node/image_raw",
    "/camera_node/camera_info",
    "/diagnostics",
    "/collision_stop/state",
    "/mission_api/v2/live_route/request",
    "/mission_api/v2/live_route/status",
)

REQUIRED_FRAME_ROOTS = ("odom", "base_link")
REQUIRED_CLEANUP_KEYS = ("processes_terminated", "serial_owners_after")


class CorpusManifestError(ValueError):
    """Raised when a no-motion corpus manifest is incomplete or unsafe."""


@dataclass(frozen=True)
class Rosbag2MetadataSummary:
    duration_seconds: float
    topic_counts: dict[str, int]
    topic_rates_hz: dict[str, float]
    starting_time_unix_ns: Optional[int] = None


@dataclass(frozen=True)
class CurrentShaCorpusManifest:
    schema_version: str
    run_id: str
    git_sha: str
    bag_path: str
    no_motion: bool
    environment: dict[str, Any]
    topic_counts: dict[str, int]
    topic_rates_hz: dict[str, float]
    frame_graph: dict[str, list[str]]
    clock_basis: str
    cleanup: dict[str, Any]
    metadata: dict[str, Any]

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


_TOPIC_NAME_RE = re.compile(r"^\s*name:\s*(?P<name>/\S+)\s*$")
_MESSAGE_COUNT_RE = re.compile(r"^\s*message_count:\s*(?P<count>\d+)\s*$")
_NANOSECONDS_RE = re.compile(r"^\s*nanoseconds:\s*(?P<value>\d+)\s*$")
_STARTING_NS_RE = re.compile(r"^\s*nanoseconds_since_epoch:\s*(?P<value>\d+)\s*$")


def parse_rosbag2_metadata(text: str) -> Rosbag2MetadataSummary:
    """Extract topic counts and rough rates from rosbag2 metadata.yaml text.

    The project avoids a PyYAML dependency in the ROS-free test environment, so
    this parser only reads the stable fields emitted by rosbag2 that the corpus
    gate needs: duration nanoseconds, starting timestamp, topic name, and message
    count.
    """

    topic_counts: dict[str, int] = {}
    current_topic: Optional[str] = None
    duration_ns: Optional[int] = None
    starting_time_ns: Optional[int] = None

    for line in text.splitlines():
        start_match = _STARTING_NS_RE.match(line)
        if start_match:
            starting_time_ns = int(start_match.group("value"))
            continue
        duration_match = _NANOSECONDS_RE.match(line)
        if duration_match and duration_ns is None:
            duration_ns = int(duration_match.group("value"))
            continue
        name_match = _TOPIC_NAME_RE.match(line)
        if name_match:
            current_topic = name_match.group("name")
            continue
        count_match = _MESSAGE_COUNT_RE.match(line)
        if count_match and current_topic is not None:
            topic_counts[current_topic] = int(count_match.group("count"))
            current_topic = None

    duration_seconds = (duration_ns or 0) / 1_000_000_000.0
    topic_rates_hz = {
        topic: (count / duration_seconds if duration_seconds > 0.0 else 0.0)
        for topic, count in topic_counts.items()
    }
    return Rosbag2MetadataSummary(duration_seconds, topic_counts, topic_rates_hz, starting_time_ns)


def _metadata_text_for_bag(bag_path: Path) -> str:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise CorpusManifestError(f"missing rosbag2 metadata: {metadata_path}")
    return metadata_path.read_text(errors="replace")


def build_current_sha_corpus_manifest(
    *,
    run_id: str,
    bag_path: Path | str,
    git_sha: str,
    environment: Mapping[str, Any],
    frame_graph: Mapping[str, Sequence[str]],
    cleanup: Mapping[str, Any],
    clock_basis: str = "system_utc_and_ros_header_stamps",
) -> CurrentShaCorpusManifest:
    bag = Path(bag_path).expanduser()
    metadata_text = _metadata_text_for_bag(bag)
    parsed = parse_rosbag2_metadata(metadata_text)
    return CurrentShaCorpusManifest(
        schema_version="rvr-current-sha-corpus.v1",
        run_id=run_id,
        git_sha=git_sha,
        bag_path=str(bag),
        no_motion=not bool(environment.get("hardware_motion", False)),
        environment=dict(environment),
        topic_counts=parsed.topic_counts,
        topic_rates_hz=parsed.topic_rates_hz,
        frame_graph={str(parent): [str(child) for child in children] for parent, children in frame_graph.items()},
        clock_basis=clock_basis,
        cleanup=dict(cleanup),
        metadata={
            "duration_seconds": parsed.duration_seconds,
            "starting_time_unix_ns": parsed.starting_time_unix_ns,
            "source": "rosbag2 metadata.yaml",
        },
    )


def _manifest_dict(manifest: CurrentShaCorpusManifest | Mapping[str, Any]) -> dict[str, Any]:
    if isinstance(manifest, CurrentShaCorpusManifest):
        return manifest.to_json_dict()
    return dict(manifest)


def validate_current_sha_corpus_manifest(
    manifest: CurrentShaCorpusManifest | Mapping[str, Any],
    *,
    expected_sha: str,
) -> dict[str, Any]:
    data = _manifest_dict(manifest)
    if data.get("schema_version") != "rvr-current-sha-corpus.v1":
        raise CorpusManifestError("unsupported corpus manifest schema_version")
    if data.get("git_sha") != expected_sha:
        raise CorpusManifestError(f"manifest SHA {data.get('git_sha')!r} does not match expected {expected_sha!r}")
    if not data.get("no_motion"):
        raise CorpusManifestError("current-SHA corpus must be captured under a no-motion gate")

    topic_counts = dict(data.get("topic_counts") or {})
    for topic in REQUIRED_CURRENT_SHA_TOPICS:
        if int(topic_counts.get(topic, 0)) <= 0:
            raise CorpusManifestError(f"missing required topic in corpus: {topic}")

    frame_graph = data.get("frame_graph") or {}
    for frame in REQUIRED_FRAME_ROOTS:
        if frame not in frame_graph:
            raise CorpusManifestError(f"missing required frame graph root: {frame}")

    cleanup = data.get("cleanup") or {}
    for key in REQUIRED_CLEANUP_KEYS:
        if key not in cleanup:
            raise CorpusManifestError(f"missing cleanup evidence: {key}")
    if cleanup.get("processes_terminated") is not True:
        raise CorpusManifestError("cleanup must record deterministic process termination")
    if cleanup.get("serial_owners_after") not in ([], (), None):
        raise CorpusManifestError("no-motion cleanup must leave serial devices unowned")

    if not data.get("clock_basis"):
        raise CorpusManifestError("clock_basis is required")
    return data


def _full_scan(stamp: float, *, range_m: float = 1.5, transform: Transform2D | None = None) -> ScanInput:
    return ScanInput(
        ranges=tuple([range_m] * 360),
        angle_min=-math.pi,
        angle_increment=math.tau / 360.0,
        range_min=0.05,
        range_max=6.0,
        stamp=stamp,
        received_at=stamp,
        frame_id="laser" if transform is not None else "base_link",
        transform_to_base=transform,
    )


def _state(
    stamp: float,
    x: float,
    y: float,
    yaw: float,
    *,
    scan: ScanInput | None = None,
    collision: str = CollisionState.CLEAR.value,
    stop: bool = False,
    estop: bool = False,
    cancel: bool = False,
    collision_received_at: float | None = None,
) -> LiveRouteState:
    return LiveRouteState(
        stamp=stamp,
        odom=OdomMotionState(stamp=stamp, x_m=x, y_m=y, yaw_rad=yaw),
        scan=scan or _full_scan(stamp),
        collision_state=collision,
        collision_received_at=stamp if collision_received_at is None else collision_received_at,
        stop=stop,
        estop=estop,
        cancel=cancel,
    )


def _one_move_request(route_id: str = "one-move") -> LiveRouteRequest:
    return LiveRouteRequest(
        route_id=route_id,
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(RouteSegmentRequest("move", "move_distance", {"distance_m": 0.2, "speed_mps": 0.1, "timeout_s": 5.0}),),
    )


def _turn_request(route_id: str = "turn") -> LiveRouteRequest:
    return LiveRouteRequest(
        route_id=route_id,
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(RouteSegmentRequest("turn", "turn_angle", {"angle_deg": -90.0, "angular_speed_deg_s": 45.0, "timeout_s": 6.0}),),
    )


def _manifest_summary(manifest: Any) -> dict[str, Any]:
    return {
        "status": manifest.status.value if hasattr(manifest.status, "value") else str(manifest.status),
        "terminal_reason": manifest.terminal_reason,
        "executed_segments": len(manifest.executed_segments),
        "measured_distance_m": manifest.measured_distance_m,
        "measured_angle_deg": manifest.measured_angle_deg,
    }


def validate_fake_route_execution_corpus(config: LiveRouteConfig | None = None) -> dict[str, dict[str, Any]]:
    """Run deterministic ROS-free route/collision scenarios used by CI.

    These cases cover the system semantics the unit suite historically missed:
    truthful terminal status, route execution, STOP/ESTOP/cancel, stale collision
    data, collision veto, localization/odom input, and wrong-direction failures.
    """

    cfg = config or LiveRouteConfig(collision_state_max_age_s=0.30)
    odom_failure_cfg = LiveRouteConfig(
        odom=MotionPrimitiveConfig(startup_grace_s=0.5, stall_timeout_s=0.4),
        scan=cfg.scan,
        collision_state_max_age_s=cfg.collision_state_max_age_s,
    )
    request = _one_move_request()
    cases = {
        "complete_route": run_route_replay(request, (_state(1.0, 0.0, 0.0, 0.0), _state(3.1, 0.2, 0.0, 0.0)), cfg),
        "collision_veto": run_route_replay(
            request,
            (_state(1.0, 0.0, 0.0, 0.0), _state(1.1, 0.0, 0.0, 0.0, collision=CollisionState.STOPPED.value)),
            cfg,
        ),
        "stale_collision": run_route_replay(
            request,
            (_state(1.0, 0.0, 0.0, 0.0, collision_received_at=1.0), _state(1.4, 0.0, 0.0, 0.0, collision_received_at=1.0)),
            cfg,
        ),
        "estop": run_route_replay(request, (_state(1.0, 0.0, 0.0, 0.0), _state(1.1, 0.0, 0.0, 0.0, estop=True)), cfg),
        "stop": run_route_replay(request, (_state(1.0, 0.0, 0.0, 0.0), _state(1.1, 0.0, 0.0, 0.0, stop=True)), cfg),
        "cancel": run_route_replay(request, (_state(1.0, 0.0, 0.0, 0.0), _state(1.1, 0.0, 0.0, 0.0, cancel=True)), cfg),
        "wrong_direction": run_route_replay(
            _turn_request("wrong-way"),
            (_state(1.0, 0.0, 0.0, 0.0), _state(2.0, 0.0, 0.0, math.radians(80.0))),
            odom_failure_cfg,
        ),
    }
    summaries = {name: _manifest_summary(manifest) for name, manifest in cases.items()}

    expected = {
        "complete_route": (ToolResultStatus.COMPLETE.value, "complete"),
        "collision_veto": (ToolResultStatus.BLOCKED.value, "collision_veto"),
        "stale_collision": (ToolResultStatus.BLOCKED.value, "stale_collision_state"),
        "estop": (ToolResultStatus.ESTOPPED.value, "estopped"),
        "stop": (ToolResultStatus.STOPPED.value, "stopped"),
        "cancel": (ToolResultStatus.CANCELLED.value, "cancelled"),
        "wrong_direction": (ToolResultStatus.FAILED.value, "wrong_direction"),
    }
    for name, (status, terminal_reason) in expected.items():
        summary = summaries[name]
        if summary["status"] != status or summary["terminal_reason"] != terminal_reason:
            raise AssertionError(f"{name} produced false terminal result: {summary}")
    return summaries


def validate_emergency_dispatch_latency(samples_seconds: Sequence[float], *, max_p95_seconds: float = 0.010) -> dict[str, float]:
    if not samples_seconds:
        raise AssertionError("emergency dispatch latency requires at least one sample")
    samples = sorted(float(sample) for sample in samples_seconds)
    if any((not math.isfinite(sample) or sample < 0.0) for sample in samples):
        raise AssertionError("emergency dispatch latency samples must be finite and non-negative")
    index = max(0, math.ceil(0.95 * len(samples)) - 1)
    p95 = samples[index]
    if p95 > max_p95_seconds:
        raise AssertionError(
            f"emergency dispatch latency p95 {p95:.6f}s exceeds gate {max_p95_seconds:.6f}s"
        )
    return {"samples": float(len(samples)), "p95_seconds": p95, "max_p95_seconds": float(max_p95_seconds)}


def assert_ci_workflow_covers_system_gates(workflow_text: str) -> tuple[str, ...]:
    missing: list[str] = []
    for job in REQUIRED_SYSTEM_CI_JOBS:
        if job not in workflow_text:
            missing.append(job)
    for token in (
        "colcon build",
        "ros2 pkg executables sphero_rvr_driver",
        "ros2 launch sphero_rvr_driver supervised_rvr.launch.py --show-args",
        "python -m sphero_rvr_driver.system_validation",
        "pytest tests/test_system_validation.py",
    ):
        if token not in workflow_text:
            missing.append(token)
    if missing:
        raise AssertionError("CI workflow is missing system gate(s): " + ", ".join(missing))
    return REQUIRED_SYSTEM_CI_JOBS


def _check_repo(repo_root: Path) -> int:
    workflow = repo_root / ".github" / "workflows" / "ci.yml"
    assert_ci_workflow_covers_system_gates(workflow.read_text())
    validate_fake_route_execution_corpus()
    validate_emergency_dispatch_latency([0.001, 0.002, 0.003], max_p95_seconds=0.010)
    print("system validation gates passed")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run ROS-free system CI gates for the RVR package.")
    parser.add_argument("--repo-root", default=Path.cwd(), type=Path)
    args = parser.parse_args(argv)
    try:
        return _check_repo(args.repo_root)
    except (AssertionError, CorpusManifestError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
