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


def _register_process_exit_handlers(path: str) -> list[dict[str, object]]:
    module = ast.parse((REPO_ROOT / path).read_text())
    handlers = []
    for node in ast.walk(module):
        if not isinstance(node, ast.Call) or getattr(node.func, "id", None) != "RegisterEventHandler":
            continue
        if not node.args or not isinstance(node.args[0], ast.Call):
            continue
        process_exit = node.args[0]
        if getattr(process_exit.func, "id", None) != "OnProcessExit":
            continue
        target = _keyword(process_exit, "target_action")
        condition = None
        for keyword in node.keywords:
            if keyword.arg == "condition" and isinstance(keyword.value, ast.Call):
                if getattr(keyword.value.func, "id", None) == "IfCondition" and keyword.value.args:
                    condition_arg = keyword.value.args[0]
                    if isinstance(condition_arg, ast.Name):
                        condition = condition_arg.id
        reasons = []
        for descendant in ast.walk(process_exit):
            if isinstance(descendant, ast.Call) and getattr(descendant.func, "id", None) == "Shutdown":
                reason = _keyword(descendant, "reason")
                if isinstance(reason, ast.Constant):
                    reasons.append(reason.value)
        handlers.append(
            {
                "target": target.id if isinstance(target, ast.Name) else None,
                "condition": condition,
                "reasons": reasons,
            }
        )
    return handlers


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
    assert "condition=IfCondition(start_supervisor)" in source
    # The three lines that used to sit here pinned a launch-argument override of
    # the slow corridor ("Startup-only forward slow-corridor", 57e26be). That
    # override SHADOWED collision_stop.yaml -- ROS 2 launch lets the later
    # parameters entry win -- so the YAML's +/-35 was dead for 23 days while the
    # node ran the launch default's +/-45. The override was removed on
    # 2026-08-25; asserting its presence would now pin the defect.
    # The replacement obligation is structural and lives in
    # tests/test_launch_does_not_shadow_config.py: the supervisor's parameters
    # come from the YAML and nothing may be layered on top.
    assert "parameters=[str(collision_stop_config)]" in source


def test_mapping_motor_capable_launch_uses_supervised_graph_by_default():
    source = (REPO_ROOT / "launch" / "mapping.launch.py").read_text()

    assert "start_collision_stop" in source
    assert "allow_unsupervised_rvr" in source
    assert "supervised_rvr.launch.py" in source
    assert "MOTOR-CAPABLE supervised" in source
    assert "start_rvr requires start_collision_stop" in source


def test_rvr_direct_launch_warns_it_is_unsupervised_low_level_only():
    source = (REPO_ROOT / "launch" / "rvr.launch.py").read_text()

    assert "UNSUPERVISED" in source
    assert "supervised_rvr.launch.py" in source
    assert "allow_unsupervised_rvr" in source
    assert "condition=IfCondition(allow_unsupervised_rvr)" in source


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
    assert "nearest_front_slow_m" in source
    assert "front_slow_min_angle_deg" in source
    assert "front_slow_max_angle_deg" in source


def test_collision_stop_node_exposes_all_side_sector_boundaries_as_ros_parameters():
    source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "collision_stop_node.py").read_text()
    config = (REPO_ROOT / "config" / "collision_stop.yaml").read_text()

    for name in (
        "left_spin_min_angle_deg",
        "left_spin_max_angle_deg",
        "right_spin_min_angle_deg",
        "right_spin_max_angle_deg",
        "trajectory_clearance_margin_m",
        "max_scan_stamp_age_s",
    ):
        assert f'"{name}": defaults.{name}' in source
        assert f'{name}=float(self.get_parameter("{name}").value)' in source
        assert f"{name}:" in config


def test_collision_stop_public_services_do_not_spin_nested_executor():
    source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "collision_stop_node.py").read_text()

    assert "spin_until_future_complete" not in source
    assert "call_async" in source
    assert "add_done_callback" in source


def test_live_nodes_tolerate_launch_shutdown_without_rclpy_shutdown_crash():
    collision_source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "collision_stop_node.py").read_text()
    rvr_source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "rvr_node.py").read_text()

    for source in (collision_source, rvr_source):
        assert "ExternalShutdownException" in source
        assert "try_shutdown" in source
    assert "_context_ok" in rvr_source
    assert "self.context.ok()" in rvr_source


def test_lidar_launch_shutdowns_graph_when_required_lidar_process_exits():
    handlers = _register_process_exit_handlers("launch/lidar.launch.py")

    assert {
        "target": "rplidar_node",
        "condition": None,
        "reasons": ["rplidar_node exited; shutting down lidar launch"],
    } in handlers


def test_supervised_launch_shutdowns_motor_graph_when_safety_or_driver_process_exits():
    handlers = _register_process_exit_handlers("launch/supervised_rvr.launch.py")

    assert {
        "target": "collision_stop_node",
        "condition": "start_supervisor",
        "reasons": ["lidar_collision_stop_supervisor exited; shutting down motor-capable launch"],
    } in handlers
    assert {
        "target": "rvr_node",
        "condition": None,
        "reasons": ["sphero_rvr_driver exited; shutting down supervised launch"],
    } in handlers


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
