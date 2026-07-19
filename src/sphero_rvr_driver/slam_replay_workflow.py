"""Replay-first SLAM Toolbox planning helpers.

This module builds dry-run plans for no-hardware SLAM replay validation.  It is
intentionally command-construction only: callers can inspect the exact ROS
commands before choosing to execute them on a ROS host.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from shlex import join as shell_join
from typing import Mapping, Sequence

from sphero_rvr_driver.rosbag_workflow import build_replay_plan
from sphero_rvr_driver.tui_launch import sanitize_map_name

REPLAY_MAPPING_TOPICS = (
    "/scan",
    "/tf_static",
    "/camera_node/image_raw",
    "/camera_node/camera_info",
)
LOCALIZATION_REQUIRED_TOPICS = ("/scan", "/odom", "/tf", "/tf_static")
MOTOR_TOPIC_KEYWORDS = ("cmd_vel", "cmd_vel_motor", "motor", "teleop", "raw_motor")


@dataclass(frozen=True)
class ReplaySlamPlan:
    bag_path: str
    map_stem: str
    map_yaml: str
    map_pgm: str
    mapping_launch_command: tuple[str, ...]
    replay_command: tuple[str, ...]
    map_save_command: tuple[str, ...]
    map_reload_commands: tuple[tuple[str, ...], ...]
    localization_supported: bool
    localization_limits: tuple[str, ...]

    def command_lines(self) -> list[str]:
        lines = [
            shell_join(self.mapping_launch_command),
            shell_join(self.replay_command),
            shell_join(self.map_save_command),
        ]
        lines.extend(shell_join(command) for command in self.map_reload_commands)
        return lines


def normalize_topic_counts(topic_counts: Mapping[str, int]) -> dict[str, int]:
    normalized: dict[str, int] = {}
    for raw_topic, raw_count in topic_counts.items():
        topic = raw_topic.strip()
        if not topic:
            continue
        if not topic.startswith("/"):
            topic = f"/{topic}"
        normalized[topic] = int(raw_count)
    return normalized


def localization_limits(topic_counts: Mapping[str, int]) -> tuple[str, ...]:
    counts = normalize_topic_counts(topic_counts)
    limits = [
        f"{topic} has {counts.get(topic, 0)} messages"
        for topic in LOCALIZATION_REQUIRED_TOPICS
        if counts.get(topic, 0) <= 0
    ]
    if limits:
        limits.append(
            "Replay can validate lidar/static-TF map-save and map-reload surfaces, "
            "but not odometry-backed localization outputs."
        )
    return tuple(limits)


def build_replay_slam_plan(
    *,
    bag_path: str | Path,
    map_name: str,
    topic_counts: Mapping[str, int],
    map_dir: str | Path,
) -> ReplaySlamPlan:
    map_stem = sanitize_map_name(map_name)
    map_base = Path(map_dir).expanduser() / map_stem
    replay_plan = build_replay_plan(bag_path=bag_path, topics=REPLAY_MAPPING_TOPICS)
    limits = localization_limits(topic_counts)
    mapping_launch = (
        "ros2",
        "launch",
        "sphero_rvr_driver",
        "mapping.launch.py",
        "start_rvr:=false",
        "start_lidar:=false",
        "start_camera:=false",
        "start_slam:=true",
        "use_sim_time:=true",
    )
    map_save = ("ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(map_base))
    map_yaml = f"{map_base}.yaml"
    map_reload = (
        (
            "ros2",
            "run",
            "nav2_map_server",
            "map_server",
            "--ros-args",
            "-p",
            f"yaml_filename:={map_yaml}",
            "-p",
            "use_sim_time:=true",
        ),
        ("ros2", "lifecycle", "set", "/map_server", "configure"),
        ("ros2", "lifecycle", "set", "/map_server", "activate"),
        ("ros2", "topic", "echo", "--once", "/map"),
    )
    return ReplaySlamPlan(
        bag_path=str(Path(bag_path).expanduser()),
        map_stem=map_stem,
        map_yaml=map_yaml,
        map_pgm=f"{map_base}.pgm",
        mapping_launch_command=mapping_launch,
        replay_command=tuple(replay_plan.command),
        map_save_command=map_save,
        map_reload_commands=map_reload,
        localization_supported=not limits,
        localization_limits=limits,
    )


def assert_no_motor_commands(plan: ReplaySlamPlan) -> None:
    for line in plan.command_lines():
        lowered = line.lower()
        if any(keyword in lowered for keyword in MOTOR_TOPIC_KEYWORDS):
            # Allow the explicit safety-disabled launch arg. Anything else is a bug.
            lowered_without_safe_arg = lowered.replace("start_rvr:=false", "")
            if any(keyword in lowered_without_safe_arg for keyword in MOTOR_TOPIC_KEYWORDS):
                raise ValueError(f"motor-capable command surfaced in replay plan: {line}")


def parse_topic_count(value: str) -> tuple[str, int]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("topic counts must look like /topic=count")
    topic, count_text = value.split("=", 1)
    try:
        count = int(count_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("topic count must be an integer") from exc
    if count < 0:
        raise argparse.ArgumentTypeError("topic count must be non-negative")
    return topic, count


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Print a no-hardware SLAM replay/map plan.")
    parser.add_argument("bag_path")
    parser.add_argument("--map-name", default="vs02_replay_map")
    parser.add_argument("--map-dir", default="~/maps")
    parser.add_argument(
        "--topic-count",
        action="append",
        type=parse_topic_count,
        default=[],
        metavar="/topic=count",
        help="Bag topic count from ros2 bag info; repeat for /scan, /odom, /tf, /tf_static.",
    )
    parser.add_argument("--json", action="store_true", help="Print the structured plan as JSON.")
    args = parser.parse_args(argv)

    topic_counts = dict(args.topic_count)
    plan = build_replay_slam_plan(
        bag_path=args.bag_path,
        map_name=args.map_name,
        topic_counts=topic_counts,
        map_dir=args.map_dir,
    )
    assert_no_motor_commands(plan)
    if args.json:
        print(json.dumps(asdict(plan), indent=2))
    else:
        print("DRY RUN: no ROS process was started")
        print("Replay-first SLAM/map command plan:")
        for line in plan.command_lines():
            print(f"  {line}")
        if plan.localization_supported:
            print("Localization prerequisites: supported by supplied topic counts.")
        else:
            print("Localization limits:")
            for limit in plan.localization_limits:
                print(f"  - {limit}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
