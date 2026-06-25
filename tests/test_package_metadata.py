from __future__ import annotations

import ast
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


EXPECTED_DATA_FILES = {
    "share/sphero_rvr_driver/launch": {
        "launch/rvr.launch.py",
        "launch/lidar.launch.py",
        "launch/mapping.launch.py",
    },
    "share/sphero_rvr_driver/config": {
        "config/rvr.yaml",
        "config/lidar.yaml",
        "config/slam_toolbox.yaml",
    },
    "share/sphero_rvr_driver/scripts": {
        "scripts/rvr-console",
        "scripts/rvr_motion_calibration.py",
    },
    "share/sphero_rvr_driver/docs": {
        "docs/mapping.md",
        "docs/motion_calibration.md",
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


def test_package_xml_declares_runtime_dependencies_for_launches() -> None:
    root = ET.parse(REPO_ROOT / "package.xml").getroot()
    exec_depends = {element.text for element in root.findall("exec_depend")}

    assert {
        "ament_index_python",
        "launch",
        "launch_ros",
        "rplidar_ros",
        "slam_toolbox",
        "tf2_ros",
    } <= exec_depends


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
        "docs/udev/99-rplidar.rules",
        "ros2 launch sphero_rvr_driver lidar.launch.py --show-args",
        "ros2 launch sphero_rvr_driver mapping.launch.py --show-args",
        "launch: `rvr.launch.py`, `lidar.launch.py`, `mapping.launch.py`",
        "config: `rvr.yaml`, `lidar.yaml`, `slam_toolbox.yaml`",
        "helper scripts: `rvr-console`, `rvr_motion_calibration.py`",
    ]:
        assert token in readme
