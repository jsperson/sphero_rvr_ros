import ast
import math
from pathlib import Path

from sphero_rvr_driver.collision_stop import CollisionStopConfig, ScanInput, Transform2D, TwistCommand
from sphero_rvr_driver.range_motion import MotionDirection, MotionMode, RangeMotionTelemetry, StopReason
from sphero_rvr_driver.range_motion_node import (
    _goal_from_json,
    _goal_from_parameters,
    _scan_sector_angular_candidates,
    _scan_sector_candidates,
    _telemetry_to_json,
)

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_range_motion_goal_json_exposes_parameterized_target_clearance_without_12_inch_cap():
    goal = _goal_from_json(
        '{"direction":"forward","mode":"approach","target_clearance_m":0.1016,'
        '"max_measured_displacement_m":1.2,"timeout_s":5.0}'
    )

    assert goal.direction is MotionDirection.FORWARD
    assert goal.mode is MotionMode.APPROACH
    assert goal.target_clearance_m == 0.1016
    assert goal.max_measured_displacement_m == 1.2
    assert goal.timeout_s == 5.0


def test_range_motion_start_service_uses_parameterized_goal_values():
    goal = _goal_from_parameters(
        {
            "service_goal_direction": "backward",
            "service_goal_mode": "retreat",
            "service_goal_target_clearance_m": 0.1016,
            "service_goal_max_measured_displacement_m": 0.75,
            "service_goal_timeout_s": 4.0,
        }
    )

    assert goal.direction is MotionDirection.BACKWARD
    assert goal.mode is MotionMode.RETREAT
    assert goal.target_clearance_m == 0.1016
    assert goal.max_measured_displacement_m == 0.75
    assert goal.timeout_s == 4.0


def test_range_motion_scan_candidates_keep_surface_cluster_not_only_nearest_point():
    ranges = [2.0] * 181
    angle_min = -math.pi / 2.0
    angle_increment = math.pi / 180.0
    for index in range(len(ranges)):
        deg = math.degrees(angle_min + index * angle_increment)
        if -4 <= deg <= 4:
            ranges[index] = 0.50 + 0.001 * deg
    ranges[90] = 0.18  # isolated nearer speckle must not define the tracked surface by itself
    scan = ScanInput(
        ranges=tuple(ranges),
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.05,
        range_max=6.0,
        stamp=1.0,
        received_at=1.0,
        frame_id="laser",
        transform_to_base=Transform2D(),
    )

    candidates = _scan_sector_candidates(scan, CollisionStopConfig(min_valid_ranges=1, min_valid_fraction=0.0), "front")

    assert 0.18 in candidates
    assert len([value for value in candidates if 0.49 <= value <= 0.51]) >= 7

    angular_candidates = _scan_sector_angular_candidates(
        scan, CollisionStopConfig(min_valid_ranges=1, min_valid_fraction=0.0), "front"
    )
    assert any(0.49 <= candidate.range_m <= 0.51 and abs(candidate.angle_rad) < 0.08 for candidate in angular_candidates)


def test_range_motion_status_json_reports_measured_not_theoretical_motion():
    payload = _telemetry_to_json(
        RangeMotionTelemetry(
            command=TwistCommand(0.05, 0.0),
            requested_velocity_mps=0.05,
            forwarded_velocity_mps=0.05,
            lidar_range_rate_mps=-0.04,
            odom_velocity_mps=0.03,
            measured_displacement_m=0.12,
            confidence=0.8,
            stop_reason=StopReason.RUNNING,
            target_clearance_m=0.1016,
            current_clearance_m=0.20,
        )
    )

    assert '"lidar_range_rate_mps": -0.04' in payload
    assert '"measured_displacement_m": 0.12' in payload
    assert "velocity * duration" not in payload


def test_range_motion_ros_node_is_installed_and_publishes_to_supervisor_input_not_motor_topic():
    setup_text = (REPO_ROOT / "setup.py").read_text()
    launch_text = (REPO_ROOT / "launch" / "supervised_rvr.launch.py").read_text()
    config_text = (REPO_ROOT / "config" / "range_motion.yaml").read_text()
    node_source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "range_motion_node.py").read_text()

    assert "range_motion_controller = sphero_rvr_driver.range_motion_node:main" in setup_text
    assert "config/range_motion.yaml" in setup_text
    assert "range_motion_controller" in launch_text
    assert "range_motion.yaml" in launch_text
    assert "cmd_vel_topic: /cmd_vel" in config_text
    assert "fail_on_missing_tf: true" in config_text
    assert "/cmd_vel_motor" not in config_text
    assert "motor_cmd_topic" not in node_source
    assert "RangeMotionController" in node_source
    assert "TransformListener" in node_source
    assert "range_motion/start" in node_source
    assert "_on_start_service" in node_source


def test_range_motion_node_keeps_ros_imports_lazy_for_no_ros_unit_tests():
    module = ast.parse((REPO_ROOT / "src" / "sphero_rvr_driver" / "range_motion_node.py").read_text())
    top_level_imports = [node for node in module.body if isinstance(node, (ast.Import, ast.ImportFrom))]
    imported = {alias.name for node in top_level_imports for alias in node.names}

    assert "rclpy" not in imported
    assert "geometry_msgs.msg" not in imported
    assert "sensor_msgs.msg" not in imported
