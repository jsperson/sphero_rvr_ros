from __future__ import annotations

import json
from pathlib import Path
import subprocess

import pytest

from sphero_rvr_driver.hierarchical_m7_phase1_audit import (
    AUDIT_SCHEMA,
    benchmark_wfd,
    main,
    validate_source_sha,
)
from sphero_rvr_driver.hierarchical_nav2_replay_validation import (
    _prohibited_processes,
    _serial_device_owners,
    _validated_source_sha,
)
from sphero_rvr_driver.mission_api import MissionValidationError


REPO_ROOT = Path(__file__).resolve().parents[1]
RECORDED_MAP = (
    REPO_ROOT
    / "artifacts"
    / "phase1_recorded_slam_map"
    / "phase1_recorded_slam_map.yaml"
)
SOURCE_SHA = "1" * 40


def test_wfd_audit_records_exact_source_checksums_determinism_and_no_authority() -> None:
    report = benchmark_wfd(
        RECORDED_MAP,
        source_sha=SOURCE_SHA,
        repetitions=5,
    )

    assert report["schema"] == AUDIT_SCHEMA
    assert report["source_sha"] == SOURCE_SHA
    assert report["result"]["passed"] is True
    assert report["result"]["deterministic"] is True
    assert report["result"]["frontier_count"] == 13
    assert len(report["result"]["frontier_signatures"]) == 13
    assert len(report["result"]["duration_ms"]["samples"]) == 5
    assert report["result"]["maximum_rss_kib"] > 0
    assert report["input"]["map_image_sha256"] == (
        "c05ae6ec457d5ce389a4278e055fcf8cce1a77752de2a05244c0f43ae55e1d76"
    )
    assert report["authority"] == {
        "recorded_data_only": True,
        "ros_started": False,
        "live_sensors_started": False,
        "serial_transport_started": False,
        "driver_started": False,
        "motion_authority": False,
        "physical_execution_enabled": False,
    }


def test_wfd_audit_cli_writes_machine_readable_report(tmp_path: Path) -> None:
    output = tmp_path / "wfd.json"
    result = main(
        [
            str(RECORDED_MAP),
            "--source-sha",
            SOURCE_SHA,
            "--repetitions",
            "2",
            "--output",
            str(output),
        ]
    )

    assert result == 0
    saved = json.loads(output.read_text())
    assert saved["result"]["passed"] is True
    assert saved["source_sha"] == SOURCE_SHA


@pytest.mark.parametrize("value", ["", "abc", "A" * 40, "0" * 39])
def test_audit_source_sha_is_exact_and_lowercase(value: str) -> None:
    with pytest.raises((MissionValidationError, ValueError)):
        validate_source_sha(value)
    with pytest.raises(ValueError):
        _validated_source_sha(value)


def test_process_and_serial_owner_checks_fail_closed(tmp_path: Path) -> None:
    process_text = """
      11 python3 /opt/ros/jazzy/bin/ros2 run sphero_rvr_driver rvr_node
      12 python3 /opt/ros/jazzy/bin/ros2 run sphero_rvr_driver live_route_runner
      13 python3 /opt/ros/jazzy/bin/ros2 run rplidar_ros rplidar_composition
      14 live_mission_service -p stationary_perception_enabled:=false
    """
    prohibited = _prohibited_processes(process_text)
    assert len(prohibited) == 2
    assert "rvr_node" in prohibited[0]
    assert "rplidar" in prohibited[1]

    device = tmp_path / "ttyAMA0"

    def fake_runner(*_args, **_kwargs):
        return subprocess.CompletedProcess([], 0, stdout="123 456\n", stderr="")

    device.write_text("")
    assert _serial_device_owners(
        paths=(str(device),),
        runner=fake_runner,
    ) == {str(device): [123, 456]}


def test_milestone7_phase1_graph_audit_contract_is_no_motion() -> None:
    source = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_nav2_replay_validation.py"
    ).read_text()

    assert 'choices=("audit", "handoff", "veto")' in source
    assert '"nonzero_motor_samples": node.nonzero_motor_samples' in source
    assert '"hardware_sink_present": False' in source
    assert '"simulation_sink": "/loopback_simulator"' in source
    assert "node.action.send_goal_async(goal)" in source
    audit_block = source.split('if args.mode == "audit":', 1)[1].split(
        "goal = NavigateThroughPoses.Goal()", 1
    )[0]
    assert "send_goal" not in audit_block
