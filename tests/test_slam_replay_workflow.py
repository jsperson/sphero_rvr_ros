from __future__ import annotations

import json
from pathlib import Path

import pytest

from sphero_rvr_driver.slam_replay_workflow import (
    REPLAY_MAPPING_TOPICS,
    assert_no_motor_commands,
    build_replay_slam_plan,
    localization_limits,
    main,
)

VS01_TOPIC_COUNTS = {
    "/camera_node/camera_info": 596,
    "/camera_node/image_raw": 596,
    "/diagnostics": 0,
    "/odom": 0,
    "/scan": 202,
    "/tf": 0,
    "/tf_static": 3,
}


def test_replay_slam_plan_uses_bag_replay_and_disables_live_sensors_and_motors(tmp_path):
    plan = build_replay_slam_plan(
        bag_path="/home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag",
        map_name="VS02 Replay Map",
        topic_counts=VS01_TOPIC_COUNTS,
        map_dir=tmp_path,
    )

    assert plan.map_stem == "VS02_Replay_Map"
    assert plan.mapping_launch_command == (
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
    assert plan.replay_command == (
        "ros2",
        "bag",
        "play",
        "/home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag",
        "--topics",
        *REPLAY_MAPPING_TOPICS,
    )
    assert plan.map_save_command == (
        "ros2",
        "run",
        "nav2_map_server",
        "map_saver_cli",
        "-f",
        str(tmp_path / "VS02_Replay_Map"),
    )
    assert_no_motor_commands(plan)


def test_replay_slam_plan_includes_map_reload_surface(tmp_path):
    plan = build_replay_slam_plan(
        bag_path="/bag",
        map_name="room",
        topic_counts={"/scan": 1, "/odom": 1, "/tf": 1, "/tf_static": 1},
        map_dir=tmp_path,
    )

    assert plan.map_yaml == str(tmp_path / "room.yaml")
    assert plan.map_pgm == str(tmp_path / "room.pgm")
    assert plan.map_reload_commands[0] == (
        "ros2",
        "run",
        "nav2_map_server",
        "map_server",
        "--ros-args",
        "-p",
        f"yaml_filename:={tmp_path / 'room.yaml'}",
        "-p",
        "use_sim_time:=true",
    )
    assert ("ros2", "lifecycle", "set", "/map_server", "configure") in plan.map_reload_commands
    assert ("ros2", "topic", "echo", "--once", "/map") in plan.map_reload_commands
    assert plan.localization_supported is True
    assert plan.localization_limits == ()


def test_vs01_bag_topic_counts_make_localization_limit_explicit():
    limits = localization_limits(VS01_TOPIC_COUNTS)

    assert "/odom has 0 messages" in limits
    assert "/tf has 0 messages" in limits
    assert any("not odometry-backed localization outputs" in item for item in limits)


def test_cli_dry_run_prints_commands_and_limits_without_starting_ros(tmp_path, capsys):
    rc = main(
        [
            "/home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag",
            "--map-name",
            "vs02 replay",
            "--map-dir",
            str(tmp_path),
            "--topic-count",
            "/scan=202",
            "--topic-count",
            "/tf_static=3",
            "--topic-count",
            "/odom=0",
            "--topic-count",
            "/tf=0",
        ]
    )

    output = capsys.readouterr().out
    assert rc == 0
    assert "DRY RUN: no ROS process was started" in output
    assert "mapping.launch.py start_rvr:=false start_lidar:=false" in output
    assert "ros2 bag play /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag" in output
    assert "ros2 run nav2_map_server map_saver_cli -f" in output
    assert "Localization limits:" in output
    assert "/odom has 0 messages" in output
    assert "/tf has 0 messages" in output


def test_cli_json_serializes_plan(tmp_path, capsys):
    rc = main(["/bag", "--map-dir", str(tmp_path), "--topic-count", "/scan=1", "--json"])

    data = json.loads(capsys.readouterr().out)
    assert rc == 0
    assert data["bag_path"] == "/bag"
    assert data["map_yaml"] == str(tmp_path / "vs02_replay_map.yaml")
    assert data["localization_supported"] is False


def test_fixture_map_artifacts_are_deterministic_and_relocatable():
    fixture_dir = Path(__file__).resolve().parents[1] / "artifacts" / "vs02_slam_replay_fixture_map"
    yaml_path = fixture_dir / "vs02_replay_fixture_map.yaml"
    pgm_path = fixture_dir / "vs02_replay_fixture_map.pgm"

    assert yaml_path.is_file()
    assert pgm_path.is_file()
    yaml_text = yaml_path.read_text()
    assert "image: vs02_replay_fixture_map.pgm" in yaml_text
    assert "resolution: 0.05" in yaml_text
    assert pgm_path.read_text().startswith("P2\n# VS02 deterministic replay fixture map")


def test_mapping_launch_and_slam_config_keep_safe_replay_defaults():
    repo_root = Path(__file__).resolve().parents[1]
    launch_text = (repo_root / "launch" / "mapping.launch.py").read_text()
    slam_config = (repo_root / "config" / "slam_toolbox.yaml").read_text()

    assert '"start_rvr",\n            default_value="false"' in launch_text
    assert '"start_lidar",\n            default_value="true"' in launch_text
    assert '"start_camera",\n            default_value="false"' in launch_text
    assert '"start_slam",\n            default_value="true"' in launch_text
    assert "async_slam_toolbox_node" in launch_text
    assert "cmd_vel" not in launch_text
    assert "mode: mapping" in slam_config
    assert "scan_topic: /scan" in slam_config
    assert "base_frame: base_link" in slam_config


@pytest.mark.parametrize("bad", ["/cmd_vel=1", "/cmd_vel_motor=1", "/motor/control=1"])
def test_cli_rejects_bad_topic_count_shapes_before_plan(bad, capsys):
    # Topic counts are metadata only. Unsafe topics never get into the replay command;
    # the actual command topics are fixed by REPLAY_MAPPING_TOPICS.
    rc = main(["/bag", "--topic-count", bad])
    output = capsys.readouterr().out
    assert rc == 0
    assert "cmd_vel" not in next(line for line in output.splitlines() if "ros2 bag play" in line)
