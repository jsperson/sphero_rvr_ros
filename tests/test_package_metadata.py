from __future__ import annotations

import ast
import importlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

tomllib = importlib.import_module("tomllib" if sys.version_info >= (3, 11) else "tomli")


REPO_ROOT = Path(__file__).resolve().parents[1]


EXPECTED_DATA_FILES = {
    "share/sphero_rvr_driver/launch": {
        "launch/rvr.launch.py",
        "launch/supervised_rvr.launch.py",
        "launch/lidar.launch.py",
        "launch/mapping.launch.py",
        "launch/camera.launch.py",
        "launch/mission_service.launch.py",
        "launch/stationary_perception.launch.py",
        "launch/m7_stationary_localization.launch.py",
    },
    "share/sphero_rvr_driver/config": {
        "config/rvr.yaml",
        "config/collision_stop.yaml",
        "config/lidar.yaml",
        "config/slam_toolbox.yaml",
        "config/camera.yaml",
        "config/mission_service.yaml",
        "config/mission-stack.env.example",
        "config/stationary_slam_toolbox.yaml",
        "config/hierarchical_slam_toolbox.yaml",
    },
    "share/sphero_rvr_driver/scripts": {
        "scripts/install-rvr-pi",
        "scripts/rvr-camera-node",
        "scripts/rvr-console",
        "scripts/rvr-shoe-detector-eval",
        "scripts/rvr-slam-replay-plan",
        "scripts/rvr_motion_calibration.py",
        "scripts/rvr_drivetrain_bench.py",
        "scripts/analyze_ground_calibration.py",
        "scripts/aggregate_ground_calibration.py",
        "scripts/install-rvr-mission-stack-services",
    },
    "share/sphero_rvr_driver/docs": {
        "docs/architecture_map.md",
        "docs/mapping.md",
        "docs/motion_calibration.md",
        "docs/rvr_drivetrain_bench.md",
        "docs/rosbag_capture_replay.md",
        "docs/camera_lidar_calibration.md",
        "docs/lidar_collision_stop_supervisor.md",
        "docs/range_motion_controller.md",
        "docs/mission_api.md",
        "docs/mission_api.md",
        "docs/mission_controls.md",
        "docs/mission_language.md",
        "docs/mission_planner.md",
        "docs/rvr_mcp_server.md",
        "docs/mission_observability.md",
        "docs/mission_web.md",
        "docs/mission_service.md",
        "docs/pi_mission_stack.md",
        "docs/lidar_motion_validation.md",
        "docs/perception_navigation.md",
        "docs/rolling_replay.md",
        "docs/stationary_perception.md",
        "docs/adaptive_mission_authority.md",
        "docs/semantic_map_artifacts.md",
        "docs/supervised_coordinator.md",
        "docs/vertical_slice_capability_matrix.md",
        "docs/shoe_detector_replay.md",
        "docs/shoe_map_projection.md",
        "docs/slam_replay.md",
        "docs/hierarchical_exploration_phase4.md",
        "docs/hierarchical_exploration_milestone7.md",
        "docs/hierarchical_exploration_milestone7_phase1.md",
        "docs/hierarchical_exploration_milestone7_phase2.md",
        "docs/hierarchical_exploration_milestone7_phase3.md",
    },
    "share/sphero_rvr_driver/docs/udev": {
        "docs/udev/99-rplidar.rules",
    },
    "share/sphero_rvr_driver/artifacts/phase2_camera_lidar_localization": {
        "artifacts/phase2_camera_lidar_localization/README.md",
        "artifacts/phase2_camera_lidar_localization/recorded_calibration_fixture.json",
    },
    "share/sphero_rvr_driver/artifacts/phase4_real_provider_replay": {
        "artifacts/phase4_real_provider_replay/README.md",
        "artifacts/phase4_real_provider_replay/report.json",
        "artifacts/phase4_real_provider_replay/evidence.sqlite3",
    },
    "share/sphero_rvr_driver/artifacts/m7_phase1_pi_no_motion": {
        "artifacts/m7_phase1_pi_no_motion/README.md",
        "artifacts/m7_phase1_pi_no_motion/environment.json",
        "artifacts/m7_phase1_pi_no_motion/graph.json",
        "artifacts/m7_phase1_pi_no_motion/wfd.json",
    },
    "share/sphero_rvr_driver/systemd/user": {
        "systemd/user/rvr-mission-service.service",
        "systemd/user/rvr-mission-web.service",
        "systemd/user/rvr-telemetry.service",
    },
}


def _setup_call() -> ast.Call:
    module = ast.parse((REPO_ROOT / "setup.py").read_text())
    for node in module.body:
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
            if getattr(node.value.func, "id", None) == "setup":
                return node.value
    raise AssertionError("setup.py does not call setup()")


def _setup_keyword(name: str) -> ast.AST:
    for keyword in _setup_call().keywords:
        if keyword.arg == name:
            return keyword.value
    raise AssertionError(f"setup.py missing {name!r} keyword")


def test_installed_package_data_lists_required_launch_config_docs_and_scripts() -> None:
    data_files = dict(ast.literal_eval(_setup_keyword("data_files")))

    for install_dir, expected_paths in EXPECTED_DATA_FILES.items():
        assert install_dir in data_files
        assert set(data_files[install_dir]) >= expected_paths
        for relative_path in expected_paths:
            assert (REPO_ROOT / relative_path).is_file(), relative_path


def test_installed_helper_scripts_are_world_readable_and_executable() -> None:
    for relative_path in EXPECTED_DATA_FILES["share/sphero_rvr_driver/scripts"]:
        mode = (REPO_ROOT / relative_path).stat().st_mode & 0o777
        assert mode & 0o555 == 0o555, f"{relative_path}: expected world read/execute, got {mode:04o}"
        assert mode & 0o002 == 0, f"{relative_path}: must not be world-writable, got {mode:04o}"


def test_package_xml_declares_runtime_dependencies_for_launches() -> None:
    root = ET.parse(REPO_ROOT / "package.xml").getroot()
    exec_depends = {element.text for element in root.findall("exec_depend")}

    assert {
        "ament_index_python",
        "camera_ros",
        "launch",
        "launch_ros",
        "lifecycle_msgs",
        "rplidar_ros",
        "nav2_map_server",
        "slam_toolbox",
        "tf2_msgs",
        "tf2_ros",
    } <= exec_depends


def test_dev_dependencies_support_python_39_metadata_tests() -> None:
    extras_require = ast.literal_eval(_setup_keyword("extras_require"))

    assert "tomli>=2; python_version < '3.11'" in extras_require["dev"]


def test_camera_helpers_pin_compatible_source_builds_and_require_real_sensor() -> None:
    installer = (REPO_ROOT / "scripts/install-rvr-pi").read_text()
    camera_wrapper = (REPO_ROOT / "scripts/rvr-camera-node").read_text()
    workspace_repos = (REPO_ROOT / "workspace.repos").read_text()

    assert "RPI_LIBCAMERA_REF" in installer
    assert "06c385619acb10bbfb33f52f3abeb8f8c095f42b" in installer
    assert "git checkout --detach" in installer
    assert "git cat-file -e" in installer
    assert "build-essential" in installer
    assert "libyaml-dev" in installer
    assert "--buildtype=release" in installer
    assert 'PKG_CONFIG_PATH="$RPI_LIBCAMERA_ROOT/lib/aarch64-linux-gnu/pkgconfig' in installer
    assert "packages+=(camera_ros)" in installer
    assert '"ros-${ROS_DISTRO}-camera-ros"' not in installer
    assert '"ros-${ROS_DISTRO}-libcamera"' not in installer
    assert "grep -E '^[0-9]+:.*imx708'" in installer
    assert "Available cameras" not in installer
    assert "camera_ros:" in workspace_repos
    assert "d267b0295d3e7d49d1b884b187a395cf655f2fad" in workspace_repos
    assert 'RVR_ROS_WS="${RVR_ROS_WS:-$HOME/ros2_ws}"' in camera_wrapper


def test_readme_camera_commands_are_location_independent() -> None:
    readme = (REPO_ROOT / "README.md").read_text()

    assert '"$(ros2 pkg prefix sphero_rvr_driver)/share/sphero_rvr_driver/scripts/rvr-camera-node"' in readme
    assert "Equivalent manual build commands" not in readme


def test_pyproject_stays_ament_python_safe() -> None:
    pyproject = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text())

    assert pyproject["build-system"]["build-backend"] == "setuptools.build_meta"
    assert "project" not in pyproject
    assert pyproject["tool"]["pytest"]["ini_options"]["testpaths"] == ["tests"]


def test_readme_documents_installed_lidar_mapping_package_data() -> None:
    readme = (REPO_ROOT / "README.md").read_text()

    for token in [
        "docs/mapping.md",
        "docs/motion_calibration.md",
        "docs/rosbag_capture_replay.md",
        "docs/camera_lidar_calibration.md",
        "docs/lidar_collision_stop_supervisor.md",
        "docs/mission_observability.md",
        "docs/mission_web.md",
        "docs/pi_mission_stack.md",
        "docs/lidar_motion_validation.md",
        "docs/mission_api.md",
        "docs/mission_controls.md",
        "docs/mission_language.md",
        "docs/mission_planner.md",
        "gpt-5.6",
        "docs/mission_api.md",
        "docs/rvr_mcp_server.md",
        "docs/semantic_map_artifacts.md",
        "docs/supervised_coordinator.md",
        "docs/adaptive_mission_authority.md",
        "docs/slam_replay.md",
        "docs/shoe_detector_replay.md",
        "docs/shoe_map_projection.md",
        "docs/udev/99-rplidar.rules",
        "ros2 launch sphero_rvr_driver lidar.launch.py --show-args",
        "ros2 launch sphero_rvr_driver mapping.launch.py --show-args",
        "ros2 launch sphero_rvr_driver camera.launch.py --show-args",
        "launch: `rvr.launch.py`, `supervised_rvr.launch.py`, `lidar.launch.py`, `mapping.launch.py`, `camera.launch.py`",
        "config: `rvr.yaml`, `collision_stop.yaml`, `lidar.yaml`, `slam_toolbox.yaml`, `camera.yaml`",
        "helper scripts: `install-rvr-pi`, `rvr-camera-node`, `rvr-console`, `rvr-slam-replay-plan`, `rvr-shoe-detector-eval`, `rvr_motion_calibration.py`",
        "console commands include `rvr_shoe_detector_eval`, `rvr_shoe_map_project`, `rvr_semantic_map_artifacts`, `rvr_mcp_server`, `rvr_mission_web`, and `rvr_lidar_motion_validation`",
        "LLM planner over allowlisted `mission_api.v2` rover tools",
    ]:
        assert token in readme


def test_adaptive_mission_contract_grants_broad_route_authority_without_motor_authority() -> None:
    contract = (REPO_ROOT / "docs" / "adaptive_mission_authority.md").read_text()

    for token in [
        "broad movement authority",
        "without another approval for every step",
        "without a fixed cumulative-distance",
        "This is broad route authority, not raw motor authority.",
        "0.10 m/s",
        "0.4 rad/s",
        "0.25 s",
        "0.50 s",
        "0.30 s",
        "continuous, level driving surface",
        "does not have cliff or drop-off sensing",
        "exactly one `/cmd_vel_motor` publisher",
        "cannot be disabled in the operator launch",
        "No step in this document enables Adaptive mission",
    ]:
        assert token in contract


def test_mission_planner_docs_and_config_distinguish_rover_planner_from_kanban_agents() -> None:
    readme = (REPO_ROOT / "README.md").read_text()
    docs = (REPO_ROOT / "docs" / "mission_planner.md").read_text()
    config = (REPO_ROOT / "config" / "mission_planner.yaml").read_text()

    for text in (readme, docs, config):
        assert "gpt-5.6" in text
        assert "OpenRouter" in text
        assert "Kanban" in text
    assert "provider: openai" in config
    assert "model_id: gpt-5.6" in config
    assert "supports_image_input: true" in config
    assert "supports_image_input: false" in config
    assert "OPENAI_API_KEY" in docs
    assert "OPENAI_API_KEY" not in config
    assert "raw camera" in docs
    assert "Mission API" in docs


def test_rosbag_console_scripts_are_installed() -> None:
    entry_points = ast.literal_eval(_setup_keyword("entry_points"))
    console_scripts = set(entry_points["console_scripts"])

    assert {
        "rvr_rosbag_capture = sphero_rvr_driver.rosbag_workflow:capture_main",
        "rvr_rosbag_replay = sphero_rvr_driver.rosbag_workflow:replay_main",
        "rvr_rosbag_inspect = sphero_rvr_driver.rosbag_workflow:inspect_main",
        "rvr_slam_replay_plan = sphero_rvr_driver.slam_replay_workflow:main",
        "rvr_shoe_detector_eval = sphero_rvr_driver.shoe_detector:main",
        "rvr_shoe_map_project = sphero_rvr_driver.shoe_map_projection:main",
        "rvr_semantic_map_artifacts = sphero_rvr_driver.semantic_map_artifacts:main",
        "rvr_mcp_server = sphero_rvr_driver.rvr_mcp_server:main",
        "rvr_mission_web = sphero_rvr_driver.mission_web:main",
        "live_mission_service = sphero_rvr_driver.live_mission_service_node:main",
        "rvr_lidar_motion_validation = sphero_rvr_driver.lidar_motion_validation:main",
        "rvr_perception_navigation_replay = sphero_rvr_driver.perception_navigation:main",
        "rvr_camera_lidar_localization_replay = sphero_rvr_driver.camera_lidar_localization:main",
        "rvr_hierarchical_m7_phase1_audit = sphero_rvr_driver.hierarchical_m7_phase1_audit:main",
        "rvr_m7_surveyed_localization = sphero_rvr_driver.m7_surveyed_localization:main",
    } <= console_scripts

    lidar_validation = (
        REPO_ROOT / "docs" / "lidar_motion_validation.md"
    ).read_text()
    assert (
        "ros2 run sphero_rvr_driver rvr_lidar_motion_validation"
        in lidar_validation
    )


def test_lidar_collision_stop_design_links_current_ros_contract() -> None:
    design = (REPO_ROOT / "docs" / "lidar_collision_stop_supervisor.md").read_text()
    readme = (REPO_ROOT / "README.md").read_text()
    tui_plan = (REPO_ROOT / "docs" / "rvr_control_interface_plan.md").read_text()

    for token in [
        "ordinary command sources -> /cmd_vel -> lidar_collision_stop_supervisor -> /cmd_vel_motor -> sphero_rvr_driver",
        "cmd_vel:=cmd_vel_motor",
        "/stop",
        "/estop",
        "/clear_estop",
        "LaserScan",
        "base_link -> laser",
        "max_scan_age_s",
        "SENSOR_STALE",
        "ESTOPPED",
        "rosbag replay",
        "src/sphero_rvr_driver/rvr_node.py",
        "src/sphero_rvr_core/driver.py",
        "launch/lidar.launch.py",
        "launch/mapping.launch.py",
        "config/rvr.yaml",
    ]:
        assert token in design

    assert "docs/lidar_collision_stop_supervisor.md" in readme
    assert "lidar_collision_stop_supervisor.md" in tui_plan


def test_supervised_coordinator_design_documents_safe_mission_contract() -> None:
    design = (REPO_ROOT / "docs" / "supervised_coordinator.md").read_text()

    for token in [
        "SupervisedCoordinator",
        "DeterministicSegmentSelector",
        "range_motion -> /cmd_vel -> collision_stop -> /cmd_vel_motor",
        "never publishes directly to `/cmd_vel_motor`",
        "STOP",
        "ESTOP",
        "collision_stop",
        "shutdown",
        "Mission API",
        "read-only UI",
        "FAILED_CLOSED",
        "no random bump-and-turn",
    ]:
        assert token in design


def test_mission_controls_design_documents_auth_gate_and_independent_safety() -> None:
    design = (REPO_ROOT / "docs" / "mission_controls.md").read_text()
    readme = (REPO_ROOT / "README.md").read_text()

    for token in [
        "MissionControlSession",
        "mission_api.v2",
        "authenticated",
        "mission:start",
        "mission:cancel",
        "mission:pause",
        "PhysicalStartApproval",
        "motor-capable mission start",
        "replay",
        "audit_log",
        "robot-side STOP/ESTOP/collision supervisor remains independent",
        "not a generic ROS bridge",
    ]:
        assert token in design
    assert "docs/mission_controls.md" in readme


def test_mission_api_design_documents_registry_runtime_and_extension_boundary() -> None:
    design = (REPO_ROOT / "docs" / "mission_api.md").read_text()
    readme = (REPO_ROOT / "README.md").read_text()

    for token in [
        "mission_api.v2",
        "Human goal -> planner -> typed tool invocation -> deterministic runtime/adapters",
        "MissionGoal",
        "ToolDefinition",
        "ToolInvocation",
        "ToolResult",
        "MissionPlan",
        "approval classes",
        "fail-closed",
        "move_to_clearance",
        "detect_objects",
        "object_class",
        "No arbitrary ROS topics",
        "Extension guide",
    ]:
        assert token in design
    assert "docs/mission_api.md" in readme
