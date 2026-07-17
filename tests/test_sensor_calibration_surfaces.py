from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from sphero_rvr_driver.camera_calibration import (
    camera_info_is_configured,
    require_configured_camera_info,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


def _setup_data_files() -> dict[str, list[str]]:
    module = ast.parse((REPO_ROOT / "setup.py").read_text())
    for node in module.body:
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
            if getattr(node.value.func, "id", None) == "setup":
                for keyword in node.value.keywords:
                    if keyword.arg == "data_files":
                        return dict(ast.literal_eval(keyword.value))
    raise AssertionError("setup.py data_files not found")


def test_camera_launch_exposes_calibration_url_and_static_tf_without_starting_rvr() -> None:
    launch_text = (REPO_ROOT / "launch" / "camera.launch.py").read_text()

    for token in [
        '"camera_info_url"',
        "rvr_pi_camera3_800x600.yaml",
        "0.0587375",
        "-0.0301625",
        "0.114300",
        '"camera_x"',
        '"camera_y"',
        '"camera_z"',
        '"camera_roll"',
        '"camera_pitch"',
        '"camera_yaw"',
        '"camera_frame_id"',
        '"camera_optical_frame_id"',
        'package="camera_ros"',
        'executable="camera_node"',
        'package="tf2_ros"',
        'executable="static_transform_publisher"',
        'name="base_to_camera_static_tf"',
        'name="camera_to_optical_static_tf"',
        '"--roll", "-1.57079632679"',
        '"--yaw", "-1.57079632679"',
    ]:
        assert token in launch_text

    assert 'package="sphero_rvr_driver"' not in launch_text
    assert "cmd_vel" not in launch_text


def test_lidar_launch_names_all_measured_mount_transform_inputs() -> None:
    launch_text = (REPO_ROOT / "launch" / "lidar.launch.py").read_text()

    for token in [
        '"laser_x"',
        '"laser_y"',
        '"laser_z"',
        '"laser_roll"',
        '"laser_pitch"',
        '"laser_yaw"',
        "-0.0074295",
        "-0.009525",
        "0.190500",
        "3.1239668018215028",
        '"--roll", laser_roll',
        '"--pitch", laser_pitch',
        '"--yaw", laser_yaw',
    ]:
        assert token in launch_text


def test_mapping_launch_can_include_camera_without_starting_it_by_default() -> None:
    launch_text = (REPO_ROOT / "launch" / "mapping.launch.py").read_text()

    for token in [
        "camera.launch.py",
        '"start_camera"',
        'default_value="false"',
        "condition=IfCondition(start_camera)",
        "camera_info_url",
        "camera_x",
        "camera_yaw",
    ]:
        assert token in launch_text


def test_camera_config_and_calibration_runbook_are_packaged() -> None:
    data_files = _setup_data_files()

    assert "launch/camera.launch.py" in data_files["share/sphero_rvr_driver/launch"]
    assert "config/camera.yaml" in data_files["share/sphero_rvr_driver/config"]
    assert "docs/camera_lidar_calibration.md" in data_files["share/sphero_rvr_driver/docs"]

    camera_config = (REPO_ROOT / "config" / "camera.yaml").read_text()
    assert "camera_info_url:" in camera_config
    assert "rvr_pi_camera3_800x600.yaml" in camera_config
    assert "width: 800" in camera_config
    assert "height: 600" in camera_config
    assert "format: BGR888" in camera_config
    assert "camera_frame_id: camera_link" in camera_config

    runbook = (REPO_ROOT / "docs" / "camera_lidar_calibration.md").read_text()
    for token in [
        "checkerboard",
        "square size",
        "camera_calibration cameracalibrator",
        "/camera_node/camera_info",
        "K must not be all zeros",
        "distortion_model",
        "reprojection error",
        "base_link -> laser",
        "base_link -> camera_link",
        "camera_link -> camera_optical_frame",
        "tread/contact footprint",
        "persistence after restart",
    ]:
        assert token in runbook


def test_camera_info_validation_rejects_empty_intrinsics_for_semantic_localization() -> None:
    empty = SimpleNamespace(width=0, height=0, k=[0.0] * 9, d=[], distortion_model="")

    assert camera_info_is_configured(empty) is False
    with pytest.raises(ValueError, match="camera intrinsics are unconfigured"):
        require_configured_camera_info(empty, context="semantic localization")


def test_camera_info_validation_accepts_nonzero_intrinsics() -> None:
    configured = SimpleNamespace(
        width=800,
        height=600,
        k=[500.0, 0.0, 400.0, 0.0, 500.0, 300.0, 0.0, 0.0, 1.0],
        d=[0.1, -0.01, 0.0, 0.0, 0.0],
        distortion_model="plumb_bob",
    )

    assert camera_info_is_configured(configured) is True
    assert require_configured_camera_info(configured, context="semantic localization") is configured
