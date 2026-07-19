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
    },
    "share/sphero_rvr_driver/config": {
        "config/rvr.yaml",
        "config/collision_stop.yaml",
        "config/lidar.yaml",
        "config/slam_toolbox.yaml",
        "config/camera.yaml",
    },
    "share/sphero_rvr_driver/scripts": {
        "scripts/install-rvr-pi",
        "scripts/rvr-camera-node",
        "scripts/rvr-console",
        "scripts/rvr-shoe-detector-eval",
        "scripts/rvr-slam-replay-plan",
        "scripts/rvr_motion_calibration.py",
    },
    "share/sphero_rvr_driver/docs": {
        "docs/mapping.md",
        "docs/motion_calibration.md",
        "docs/rosbag_capture_replay.md",
        "docs/camera_lidar_calibration.md",
        "docs/lidar_collision_stop_supervisor.md",
        "docs/range_motion_controller.md",
        "docs/mission_api.md",
        "docs/mission_observability.md",
        "docs/supervised_coordinator.md",
        "docs/vertical_slice_capability_matrix.md",
        "docs/shoe_detector_replay.md",
        "docs/shoe_map_projection.md",
        "docs/slam_replay.md",
    },
    "share/sphero_rvr_driver/docs/udev": {
        "docs/udev/99-rplidar.rules",
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
        "rplidar_ros",
        "nav2_map_server",
        "slam_toolbox",
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
        "docs/supervised_coordinator.md",
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
        "console commands include `rvr_shoe_detector_eval` and `rvr_shoe_map_project`",
    ]:
        assert token in readme


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
    } <= console_scripts


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
