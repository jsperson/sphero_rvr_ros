from __future__ import annotations

import json
from pathlib import Path

import pytest

from sphero_rvr_driver.rosbag_workflow import (
    DEFAULT_CAPTURE_TOPICS,
    DEFAULT_REPLAY_TOPICS,
    RunManifest,
    UnsafeTopicError,
    build_capture_plan,
    build_replay_plan,
    capture_main,
    manifest_from_plan,
    replay_main,
    write_manifest,
)


def test_capture_plan_defaults_to_safe_rosbag_record_topics(tmp_path):
    plan = build_capture_plan(output_root=tmp_path, run_id="run-001")

    assert plan.bag_path == tmp_path / "run-001" / "rosbag"
    assert plan.topics == DEFAULT_CAPTURE_TOPICS
    assert plan.command == ["ros2", "bag", "record", "-o", str(plan.bag_path), *DEFAULT_CAPTURE_TOPICS]
    assert "/cmd_vel" not in plan.command


def test_capture_plan_allows_documented_topic_overrides_and_additions(tmp_path):
    plan = build_capture_plan(
        output_root=tmp_path,
        run_id="mission-a",
        topics=["/scan"],
        extra_topics=["/mission/events"],
    )

    assert plan.topics == ("/scan", "/mission/events")
    assert plan.command[-2:] == ["/scan", "/mission/events"]


@pytest.mark.parametrize("topic", ["/cmd_vel", "cmd_vel", "/rvr/raw_motors", "/motor/control", "/drive_velocity"])
def test_capture_rejects_motion_or_motor_topics(topic, tmp_path):
    with pytest.raises(UnsafeTopicError):
        build_capture_plan(output_root=tmp_path, run_id="bad", extra_topics=[topic])


def test_capture_cli_dry_run_prints_exact_command_and_never_runs_ros(tmp_path, capsys):
    rc = capture_main(["--run-id", "dry", "--output-root", str(tmp_path)])

    output = capsys.readouterr().out
    assert rc == 0
    assert "DRY RUN: no rosbag process was started" in output
    assert "ros2 bag record -o" in output
    assert str(tmp_path / "dry" / "rosbag") in output
    assert "/scan" in output and "/diagnostics" in output
    assert "Sensor/driver processes must be started separately" in output
    command_line = next(line.strip() for line in output.splitlines() if line.strip().startswith("ros2 bag record"))
    assert "/cmd_vel" not in command_line


def test_replay_plan_defaults_to_allowlisted_non_motor_topics(tmp_path):
    bag = tmp_path / "run" / "rosbag"
    plan = build_replay_plan(bag_path=bag)

    assert plan.bag_path == bag
    assert plan.topics == DEFAULT_REPLAY_TOPICS
    assert plan.command == ["ros2", "bag", "play", str(bag), "--topics", *DEFAULT_REPLAY_TOPICS]
    assert "/cmd_vel" not in plan.command


@pytest.mark.parametrize("topic", ["/cmd_vel", "/rvr/raw_motors", "/motor/control"])
def test_replay_rejects_unsafe_topic_additions_by_default(topic, tmp_path):
    with pytest.raises(UnsafeTopicError):
        build_replay_plan(bag_path=tmp_path / "bag", extra_topics=[topic])


def test_replay_developer_override_requires_explicit_flag_and_records_unsafe_topics(tmp_path):
    plan = build_replay_plan(
        bag_path=tmp_path / "bag",
        topics=["/scan"],
        extra_topics=["/cmd_vel"],
        allow_unsafe_topics=True,
    )

    assert plan.topics == ("/scan", "/cmd_vel")
    assert plan.unsafe_topics == ("/cmd_vel",)


def test_replay_cli_reports_unsafe_topic_rejection_without_traceback(tmp_path, capsys):
    rc = replay_main([str(tmp_path / "bag"), "--extra-topic", "/cmd_vel"])

    captured = capsys.readouterr()
    assert rc == 2
    assert "Unsafe rosbag topic(s) rejected by default: /cmd_vel" in captured.err
    assert "Traceback" not in captured.err


def test_replay_cli_dry_run_excludes_motor_topics(tmp_path, capsys):
    rc = replay_main([str(tmp_path / "bag")])

    output = capsys.readouterr().out
    assert rc == 0
    assert "DRY RUN: no rosbag process was started" in output
    assert "ros2 bag play" in output
    assert "--topics" in output
    assert "/scan" in output
    command_line = next(line.strip() for line in output.splitlines() if line.strip().startswith("ros2 bag play"))
    assert "/cmd_vel" not in command_line


def test_manifest_serializes_run_context_and_artifact_inventory(tmp_path):
    bag = tmp_path / "run-001" / "rosbag"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  duration: 1s\n")
    map_path = tmp_path / "maps" / "room.yaml"
    map_path.parent.mkdir()
    map_path.write_text("image: room.pgm\n")
    plan = build_capture_plan(output_root=tmp_path, run_id="run-001")

    manifest = manifest_from_plan(
        plan,
        mode="capture",
        operator_notes="stationary lidar sample",
        hardware_active=True,
        related_artifacts=[map_path],
        ros_distro="jazzy",
        git_info={"sha": "abc123", "branch": "feat/bag", "clean": True},
    )
    manifest_path = write_manifest(manifest, tmp_path / "run-001" / "run_manifest.json")

    data = json.loads(manifest_path.read_text())
    assert data["run_id"] == "run-001"
    assert data["mode"] == "capture"
    assert data["ros_distro"] == "jazzy"
    assert data["git"] == {"sha": "abc123", "branch": "feat/bag", "clean": True}
    assert data["hardware_active"] is True
    assert data["operator_notes"] == "stationary lidar sample"
    assert data["bag"]["path"] == str(bag)
    assert "duration: 1s" in data["bag"]["metadata_summary"]
    assert data["artifacts"][0]["path"] == str(map_path)
    assert data["artifacts"][0]["sha256"]


def test_manifest_requires_structured_run_identity(tmp_path):
    with pytest.raises(ValueError):
        RunManifest(run_id="", timestamp_utc="", mode="capture", command=[], topics=[], bag={})


def test_capture_failure_cleans_up_empty_run_directory(tmp_path):
    class FailingRunner:
        def run(self, command):
            return 7

    rc = capture_main(["--execute", "--run-id", "fails", "--output-root", str(tmp_path)], runner=FailingRunner())

    assert rc == 7
    assert not (tmp_path / "fails").exists()
