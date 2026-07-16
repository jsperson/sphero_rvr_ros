"""Safe rosbag2 capture/replay helpers for RVR foundation data.

The helpers in this module deliberately build rosbag-only commands. They never
launch the RVR driver, lidar, camera, SLAM, teleop, or any process that can own
motors. CLI entry points default to dry-run so an operator can inspect the exact
command before explicitly executing it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import socket
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from shlex import join as shell_join
from typing import Any, Iterable, Mapping, Optional, Sequence, Union

PathLike = Union[Path, str]

DEFAULT_CAPTURE_TOPICS = (
    "/scan",
    "/camera/image_raw",
    "/camera/camera_info",
    "/odom",
    "/tf",
    "/tf_static",
    "/diagnostics",
)
DEFAULT_REPLAY_TOPICS = DEFAULT_CAPTURE_TOPICS
UNSAFE_TOPIC_KEYWORDS = (
    "cmd_vel",
    "motor",
    "motors",
    "raw_motor",
    "raw_motors",
    "drive_velocity",
    "velocity_cmd",
    "teleop",
)
SAFETY_WARNING = (
    "WARNING: rosbag capture/replay is data-only. Sensor/driver processes must "
    "be started separately under their own approval gate; this command will not "
    "launch RVR, lidar, camera, mapping, /cmd_vel, or motor-capable processes."
)


class UnsafeTopicError(ValueError):
    """Raised when a requested capture/replay topic can affect motion."""


@dataclass(frozen=True)
class RosbagPlan:
    mode: str
    run_id: str
    bag_path: Path
    topics: tuple[str, ...]
    command: list[str]
    unsafe_topics: tuple[str, ...] = ()


@dataclass
class RunManifest:
    run_id: str
    timestamp_utc: str
    mode: str
    command: list[str]
    topics: list[str]
    bag: dict[str, Any]
    git: dict[str, Any] = field(default_factory=dict)
    host: str = ""
    os: str = ""
    ros_distro: str = ""
    hardware_active: bool = False
    operator_notes: str = ""
    artifacts: list[dict[str, Any]] = field(default_factory=list)
    safety: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.run_id:
            raise ValueError("run_id is required")
        if not self.timestamp_utc:
            raise ValueError("timestamp_utc is required")
        if self.mode not in {"capture", "replay", "inspect"}:
            raise ValueError("mode must be capture, replay, or inspect")
        if not self.command:
            raise ValueError("command is required")
        if not self.bag.get("path"):
            raise ValueError("bag.path is required")


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def default_run_id() -> str:
    return datetime.now(timezone.utc).strftime("rvr-%Y%m%dT%H%M%SZ")


def normalize_topics(topics: Iterable[str]) -> tuple[str, ...]:
    normalized: list[str] = []
    seen: set[str] = set()
    for raw in topics:
        topic = raw.strip()
        if not topic:
            continue
        if not topic.startswith("/"):
            topic = f"/{topic}"
        if topic not in seen:
            normalized.append(topic)
            seen.add(topic)
    if not normalized:
        raise ValueError("at least one topic is required")
    return tuple(normalized)


def unsafe_topics(topics: Iterable[str]) -> tuple[str, ...]:
    matches: list[str] = []
    for topic in normalize_topics(topics):
        lowered = topic.lower()
        if any(keyword in lowered for keyword in UNSAFE_TOPIC_KEYWORDS):
            matches.append(topic)
    return tuple(matches)


def validate_safe_topics(topics: Iterable[str], *, allow_unsafe_topics: bool = False) -> tuple[str, ...]:
    normalized = normalize_topics(topics)
    unsafe = unsafe_topics(normalized)
    if unsafe and not allow_unsafe_topics:
        raise UnsafeTopicError(
            "Unsafe rosbag topic(s) rejected by default: "
            + ", ".join(unsafe)
            + ". Use --allow-unsafe-topics only for developer-only replay analysis, never live robot operation."
        )
    return normalized


def _topic_list(base: Optional[Sequence[str]], extra: Optional[Sequence[str]]) -> tuple[str, ...]:
    topics = list(base if base is not None else DEFAULT_CAPTURE_TOPICS)
    topics.extend(extra or [])
    return normalize_topics(topics)


def build_capture_plan(
    *,
    output_root: PathLike,
    run_id: Optional[str] = None,
    topics: Optional[Sequence[str]] = None,
    extra_topics: Optional[Sequence[str]] = None,
    allow_unsafe_topics: bool = False,
) -> RosbagPlan:
    capture_run_id = run_id or default_run_id()
    selected_topics = _topic_list(topics, extra_topics)
    selected_topics = validate_safe_topics(selected_topics, allow_unsafe_topics=allow_unsafe_topics)
    unsafe = unsafe_topics(selected_topics)
    bag_path = Path(output_root).expanduser() / capture_run_id / "rosbag"
    command = ["ros2", "bag", "record", "-o", str(bag_path), *selected_topics]
    return RosbagPlan("capture", capture_run_id, bag_path, selected_topics, command, unsafe)


def build_replay_plan(
    *,
    bag_path: PathLike,
    run_id: Optional[str] = None,
    topics: Optional[Sequence[str]] = None,
    extra_topics: Optional[Sequence[str]] = None,
    allow_unsafe_topics: bool = False,
) -> RosbagPlan:
    selected = list(topics if topics is not None else DEFAULT_REPLAY_TOPICS)
    selected.extend(extra_topics or [])
    selected_topics = validate_safe_topics(selected, allow_unsafe_topics=allow_unsafe_topics)
    unsafe = unsafe_topics(selected_topics)
    command = ["ros2", "bag", "play", str(Path(bag_path).expanduser()), "--topics", *selected_topics]
    return RosbagPlan("replay", run_id or default_run_id(), Path(bag_path).expanduser(), selected_topics, command, unsafe)


def build_inspect_plan(*, bag_path: PathLike, run_id: Optional[str] = None) -> RosbagPlan:
    bag = Path(bag_path).expanduser()
    return RosbagPlan("inspect", run_id or default_run_id(), bag, (), ["ros2", "bag", "info", str(bag)])


def read_bag_metadata_summary(bag_path: Path) -> str:
    candidates = [bag_path / "metadata.yaml", bag_path.with_suffix("") / "metadata.yaml"]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.read_text(errors="replace")[:4000]
    return "metadata.yaml not found; run `ros2 bag info <bag>` after capture/replay host has rosbag2 available."


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact_inventory(paths: Iterable[PathLike]) -> list[dict[str, Any]]:
    inventory: list[dict[str, Any]] = []
    for raw_path in paths:
        path = Path(raw_path).expanduser()
        if not path.exists():
            inventory.append({"path": str(path), "exists": False})
            continue
        if path.is_dir():
            files = sorted(child for child in path.rglob("*") if child.is_file())
            inventory.append(
                {
                    "path": str(path),
                    "exists": True,
                    "type": "directory",
                    "file_count": len(files),
                    "files": [
                        {"path": str(child), "size_bytes": child.stat().st_size, "sha256": sha256_file(child)}
                        for child in files[:200]
                    ],
                }
            )
        else:
            inventory.append(
                {
                    "path": str(path),
                    "exists": True,
                    "type": "file",
                    "size_bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    return inventory


def gather_git_info(repo_root: Optional[Path] = None) -> dict[str, Any]:
    cwd = repo_root or Path.cwd()

    def git(*args: str) -> str:
        completed = subprocess.run(["git", *args], cwd=cwd, text=True, capture_output=True, check=False)
        return completed.stdout.strip() if completed.returncode == 0 else ""

    sha = git("rev-parse", "HEAD")
    if not sha:
        return {"available": False}
    porcelain = git("status", "--porcelain")
    return {
        "available": True,
        "sha": sha,
        "branch": git("branch", "--show-current"),
        "clean": porcelain == "",
        "status_porcelain": porcelain,
    }


def manifest_from_plan(
    plan: RosbagPlan,
    *,
    mode: Optional[str] = None,
    operator_notes: str = "",
    hardware_active: bool = False,
    related_artifacts: Iterable[PathLike] = (),
    ros_distro: Optional[str] = None,
    git_info: Optional[Mapping[str, Any]] = None,
) -> RunManifest:
    return RunManifest(
        run_id=plan.run_id,
        timestamp_utc=utc_timestamp(),
        mode=mode or plan.mode,
        command=plan.command,
        topics=list(plan.topics),
        bag={"path": str(plan.bag_path), "metadata_summary": read_bag_metadata_summary(plan.bag_path)},
        git=dict(git_info) if git_info is not None else gather_git_info(),
        host=socket.gethostname(),
        os=f"{platform.system()} {platform.release()} ({platform.platform()})",
        ros_distro=ros_distro if ros_distro is not None else os.environ.get("ROS_DISTRO", "unknown"),
        hardware_active=hardware_active,
        operator_notes=operator_notes,
        artifacts=artifact_inventory(related_artifacts),
        safety={
            "dry_run_by_default": True,
            "launches_driver_or_sensors": False,
            "unsafe_topics": list(plan.unsafe_topics),
            "warning": SAFETY_WARNING,
        },
    )


def write_manifest(manifest: RunManifest, path: PathLike) -> Path:
    manifest_path = Path(path).expanduser()
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(asdict(manifest), indent=2, sort_keys=True) + "\n")
    return manifest_path


def _split_csv(values: Optional[Sequence[str]]) -> list[str]:
    if not values:
        return []
    split: list[str] = []
    for value in values:
        split.extend(part for part in value.split(",") if part.strip())
    return split


def _print_plan(plan: RosbagPlan, *, execute: bool, manifest_path: Optional[Path] = None) -> None:
    print(SAFETY_WARNING)
    print(f"mode: {plan.mode}")
    print(f"destination: {plan.bag_path}")
    if plan.topics:
        print("topics:")
        for topic in plan.topics:
            print(f"  - {topic}")
    if plan.unsafe_topics:
        print("UNSAFE TOPICS OVERRIDDEN:")
        for topic in plan.unsafe_topics:
            print(f"  - {topic}")
    print("command:")
    print(f"  {shell_join(plan.command)}")
    if manifest_path is not None:
        print(f"manifest: {manifest_path}")
    if not execute:
        print("DRY RUN: no rosbag process was started. Re-run with --execute to run exactly this command.")


def _run_command(command: list[str], runner: Optional[Any]) -> int:
    if runner is not None:
        result = runner.run(command)
        return int(result.returncode if hasattr(result, "returncode") else result)
    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


def _capture_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safe dry-run-first rosbag2 capture for RVR lidar/camera/odom/TF diagnostics data.")
    parser.add_argument("--execute", action="store_true", help="actually run ros2 bag record after printing the safety plan")
    parser.add_argument("--output-root", default="~/rvr_runs", help="root directory for per-run capture folders")
    parser.add_argument("--run-id", default=None, help="stable run identifier; defaults to UTC rvr-YYYYmmddTHHMMSSZ")
    parser.add_argument("--topic", action="append", help="replace default topic set; repeat or comma-separate")
    parser.add_argument("--extra-topic", action="append", help="add topic(s) to the default/replaced topic set; repeat or comma-separate")
    parser.add_argument("--allow-unsafe-topics", action="store_true", help="developer-only escape hatch for analysis; permits /cmd_vel or motor-like topics")
    parser.add_argument("--notes", default="", help="operator notes stored in run_manifest.json")
    parser.add_argument("--hardware-active", action="store_true", help="record that hardware/sensors were active outside this script")
    parser.add_argument("--artifact", action="append", default=[], help="related map/log/artifact path to inventory in the manifest")
    return parser


def capture_main(argv: Optional[Sequence[str]] = None, *, runner: Optional[Any] = None) -> int:
    args = _capture_parser().parse_args(argv)
    output_root = Path(args.output_root).expanduser()
    topics = _split_csv(args.topic) if args.topic else None
    extra_topics = _split_csv(args.extra_topic)
    try:
        plan = build_capture_plan(
            output_root=output_root,
            run_id=args.run_id,
            topics=topics,
            extra_topics=extra_topics,
            allow_unsafe_topics=args.allow_unsafe_topics,
        )
    except UnsafeTopicError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    manifest_path = plan.bag_path.parent / "run_manifest.json"
    manifest = manifest_from_plan(
        plan,
        mode="capture",
        operator_notes=args.notes,
        hardware_active=args.hardware_active,
        related_artifacts=args.artifact,
    )
    _print_plan(plan, execute=args.execute, manifest_path=manifest_path)
    if not args.execute:
        return 0
    plan.bag_path.parent.mkdir(parents=True, exist_ok=True)
    write_manifest(manifest, manifest_path)
    rc = _run_command(plan.command, runner)
    if rc != 0 and plan.bag_path.parent.exists():
        shutil.rmtree(plan.bag_path.parent)
    return rc


def _replay_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safe dry-run-first rosbag2 replay using a non-motor topic allowlist.")
    parser.add_argument("bag_path", help="rosbag directory to replay")
    parser.add_argument("--execute", action="store_true", help="actually run ros2 bag play after printing the safety plan")
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--topic", action="append", help="replace default replay topic allowlist; repeat or comma-separate")
    parser.add_argument("--extra-topic", action="append", help="add replay topic(s); unsafe topics are rejected unless overridden")
    parser.add_argument("--allow-unsafe-topics", action="store_true", help="developer-only analysis escape hatch; disabled by default")
    parser.add_argument("--manifest", help="optional path to write replay manifest JSON")
    parser.add_argument("--notes", default="")
    parser.add_argument("--artifact", action="append", default=[])
    return parser


def replay_main(argv: Optional[Sequence[str]] = None, *, runner: Optional[Any] = None) -> int:
    args = _replay_parser().parse_args(argv)
    topics = _split_csv(args.topic) if args.topic else None
    extra_topics = _split_csv(args.extra_topic)
    try:
        plan = build_replay_plan(
            bag_path=args.bag_path,
            run_id=args.run_id,
            topics=topics,
            extra_topics=extra_topics,
            allow_unsafe_topics=args.allow_unsafe_topics,
        )
    except UnsafeTopicError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    manifest_path = Path(args.manifest).expanduser() if args.manifest else None
    _print_plan(plan, execute=args.execute, manifest_path=manifest_path)
    if manifest_path is not None:
        write_manifest(
            manifest_from_plan(plan, mode="replay", operator_notes=args.notes, related_artifacts=args.artifact),
            manifest_path,
        )
    if not args.execute:
        return 0
    return _run_command(plan.command, runner)


def _inspect_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect a rosbag without hardware using ros2 bag info.")
    parser.add_argument("bag_path")
    parser.add_argument("--execute", action="store_true", help="run ros2 bag info; default only prints the command")
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--manifest", help="optional path to write inspect manifest JSON")
    return parser


def inspect_main(argv: Optional[Sequence[str]] = None, *, runner: Optional[Any] = None) -> int:
    args = _inspect_parser().parse_args(argv)
    plan = build_inspect_plan(bag_path=args.bag_path, run_id=args.run_id)
    manifest_path = Path(args.manifest).expanduser() if args.manifest else None
    _print_plan(plan, execute=args.execute, manifest_path=manifest_path)
    if manifest_path is not None:
        write_manifest(manifest_from_plan(plan, mode="inspect"), manifest_path)
    if not args.execute:
        return 0
    return _run_command(plan.command, runner)


if __name__ == "__main__":
    raise SystemExit(capture_main())
