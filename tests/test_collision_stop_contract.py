import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from sphero_rvr_driver.collision_stop_node import _transform2d_from_transform_stamped

REPO_ROOT = Path(__file__).resolve().parents[1]


def _keyword(call: ast.Call, name: str):
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    raise AssertionError(f"missing keyword {name}")


def _node_calls(path: str) -> list[ast.Call]:
    module = ast.parse((REPO_ROOT / path).read_text())
    return [node for node in ast.walk(module) if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "Node"]


def test_collision_stop_config_and_node_are_installed_package_surfaces():
    setup_text = (REPO_ROOT / "setup.py").read_text()

    assert "config/collision_stop.yaml" in setup_text
    assert "launch/supervised_rvr.launch.py" in setup_text
    assert "lidar_collision_stop_supervisor = sphero_rvr_driver.collision_stop_node:main" in setup_text
    assert (REPO_ROOT / "config" / "collision_stop.yaml").is_file()
    assert (REPO_ROOT / "launch" / "supervised_rvr.launch.py").is_file()


def test_supervised_launch_remaps_driver_away_from_public_cmd_vel_and_private_services():
    source = (REPO_ROOT / "launch" / "supervised_rvr.launch.py").read_text()

    assert "cmd_vel_motor" in source
    assert "rvr_driver/stop" in source
    assert "rvr_driver/estop" in source
    assert "rvr_driver/clear_estop" in source
    assert "lidar_collision_stop_supervisor" in source
    assert "collision_stop.yaml" in source


def test_mapping_motor_capable_launch_uses_supervised_graph_by_default():
    source = (REPO_ROOT / "launch" / "mapping.launch.py").read_text()

    assert "start_collision_stop" in source
    assert "allow_unsupervised_rvr" in source
    assert "supervised_rvr.launch.py" in source
    assert "MOTOR-CAPABLE supervised" in source


def test_rvr_direct_launch_warns_it_is_unsupervised_low_level_only():
    source = (REPO_ROOT / "launch" / "rvr.launch.py").read_text()

    assert "UNSUPERVISED" in source
    assert "supervised_rvr.launch.py" in source


def test_rosbag_docs_explicitly_reject_motor_bound_supervisor_topic():
    docs = (REPO_ROOT / "docs" / "rosbag_capture_replay.md").read_text()
    readme = (REPO_ROOT / "README.md").read_text()

    assert "/cmd_vel_motor" in docs
    assert "reject" in docs.lower() or "forbid" in docs.lower()
    assert "/cmd_vel -> lidar_collision_stop_supervisor -> /cmd_vel_motor" in readme


def test_tui_status_model_includes_collision_stop_gate_for_arming():
    source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "tui_ros.py").read_text()
    tui_source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "tui.py").read_text()

    assert "collision_stop_state" in source
    assert "collision_stop_fresh" in source
    assert "collision_stop_allows_motion" in tui_source
    assert "collision stop" in tui_source.lower()


def test_collision_stop_node_uses_real_tf_lookup_and_truthful_diagnostics():
    source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "collision_stop_node.py").read_text()

    assert "Buffer()" in source
    assert "TransformListener" in source
    assert "lookup_transform" in source
    assert "transform_to_base" in source
    assert "not_checked_ros_free_sector_mode" not in source
    assert "tf_available" in source
    assert "tf_reason" in source
    assert "base_frame" in source
    assert "tf_timeout_s" in source


def test_ros_transform_helper_extracts_planar_yaw_for_core_evaluation():
    half_yaw = math.pi / 4.0
    stamped = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.10, y=-0.20),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half_yaw), w=math.cos(half_yaw)),
        )
    )

    result = _transform2d_from_transform_stamped(stamped)

    assert result.x == pytest.approx(0.10)
    assert result.y == pytest.approx(-0.20)
    assert result.yaw == pytest.approx(math.pi / 2.0)
