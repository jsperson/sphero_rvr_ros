import json
from pathlib import Path

import pytest

from sphero_rvr_driver.drive_trace import (
    BoundedDriveTrace,
    DriveTraceMetrics,
    TRACE_SAMPLE_SCHEMA,
    TRACE_SUMMARY_SCHEMA,
)
from sphero_rvr_driver.live_route_runner_node import _drive_trace_mission_id


REPO_ROOT = Path(__file__).resolve().parents[1]
SHA = "a" * 40


def test_drive_trace_metrics_quantify_reversals_and_progress() -> None:
    metrics = DriveTraceMetrics()
    for angular in (0.35, 0.0, 0.35, -0.35, 0.0, -0.35, 0.35):
        metrics.record_command("nav2_request", 0.0, angular)
    metrics.record_command("supervisor_request", 0.10, 0.0)
    metrics.record_command("motor_output", 0.05, 0.0)
    metrics.record_odom(1.0, 2.0, 0.0)
    metrics.record_odom(1.03, 2.04, 0.4)
    metrics.record_state("collision", {"state": "CLEAR"})
    metrics.record_state("hierarchical_controller", {"state": "navigating"})

    summary = metrics.summary()
    assert summary["schema"] == TRACE_SUMMARY_SCHEMA
    assert summary["command_streams"]["nav2_request"] == {
        "samples": 7,
        "nonzero_samples": 5,
        "angular_sign_reversals": 2,
        "max_abs_linear_mps": 0.0,
        "max_abs_angular_rad_s": 0.35,
    }
    assert summary["max_displacement_m"] == pytest.approx(0.05)
    assert summary["odom_samples"] == 2
    assert summary["collision_samples"] == 1
    assert summary["controller_state_samples"] == 1


def test_drive_trace_binds_mission_id_from_exact_proposal(tmp_path: Path) -> None:
    proposal = tmp_path / "proposal.json"
    proposal.write_text(json.dumps({"mission_id": "m7-canonical-trace"}))
    assert _drive_trace_mission_id(proposal) == "m7-canonical-trace"
    proposal.write_text("{}")
    with pytest.raises(ValueError, match="no mission ID"):
        _drive_trace_mission_id(proposal)


def test_drive_trace_is_private_rotating_and_retention_bounded(
    tmp_path: Path,
) -> None:
    trace = BoundedDriveTrace(
        tmp_path,
        mission_id="physical-jitter-test",
        source_sha=SHA,
        max_segment_bytes=4096,
        retained_files=2,
    )
    for index in range(80):
        sample = trace.metrics.record_state(
            "hierarchical_adapter",
            {"index": index, "detail": "x" * 120},
        )
        trace.record(
            sample,
            recorded_at_s=100.0 + index,
            monotonic_s=50.0 + index,
        )
    trace.close()

    files = sorted(tmp_path.glob("drive-trace-*.jsonl"))
    assert len(files) == 2
    assert all(path.stat().st_mode & 0o077 == 0 for path in files)
    assert all(path.stat().st_size <= 4600 for path in files)
    last = json.loads(trace.path.read_text().splitlines()[-1])
    assert last["schema"] == TRACE_SAMPLE_SCHEMA
    assert last["kind"] == "trace_summary"
    assert last["summary"]["controller_state_samples"] == 80


@pytest.mark.parametrize(
    ("stream", "linear", "angular"),
    [
        ("unknown", 0.0, 0.0),
        ("nav2_request", float("nan"), 0.0),
        ("motor_output", 0.0, float("inf")),
    ],
)
def test_drive_trace_rejects_untyped_or_nonfinite_commands(
    stream: str, linear: float, angular: float
) -> None:
    with pytest.raises(ValueError):
        DriveTraceMetrics().record_command(stream, linear, angular)


def test_physical_nav2_uses_rotation_shim_and_angular_progress() -> None:
    nav2 = (
        REPO_ROOT / "config" / "hierarchical_nav2_physical.yaml"
    ).read_text()
    route = (REPO_ROOT / "config" / "live_route_runner.yaml").read_text()
    launch = (
        REPO_ROOT
        / "launch"
        / "hierarchical_exploration_physical.launch.py"
    ).read_text()
    node = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "live_route_runner_node.py"
    ).read_text()

    assert 'plugin: "nav2_controller::PoseProgressChecker"' in nav2
    assert "required_movement_radius: 0.02" in nav2
    assert "required_movement_angle: 0.10" in nav2
    assert "movement_time_allowance: 15.0" in nav2
    assert (
        'plugin: "nav2_rotation_shim_controller::RotationShimController"'
        in nav2
    )
    assert 'primary_controller: "dwb_core::DWBLocalPlanner"' in nav2
    assert "rotate_to_heading_angular_vel: 0.35" in nav2
    assert "rotate_to_heading_once: true" in nav2
    assert "max_vel_x: 0.10" in nav2
    assert "max_vel_theta: 0.4" in nav2
    assert "hierarchical_max_linear_mps: 0.10" in route
    assert "hierarchical_max_angular_rad_s: 0.4" in route
    assert '"drive_trace_enabled": True' in launch
    assert '"drive_trace_proposal_file": proposal_file' in launch
    assert "MOTOR_TOPIC" in node
    assert "self._on_motor_output" in node
    assert "BoundedDriveTrace" in node


def test_live_camera_drops_backlog_and_ui_separates_source_from_freshness() -> None:
    perception = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "stationary_perception_node.py"
    ).read_text()
    web = (
        REPO_ROOT / "src" / "sphero_rvr_driver" / "mission_web.py"
    ).read_text()

    image_subscription = perception.split(
        "self.create_subscription(\n                Image,", 1
    )[1].split("callback_group=self._sensor_callbacks", 1)[0]
    assert "QoSProfile(" in image_subscription
    assert "ReliabilityPolicy.BEST_EFFORT" in image_subscription
    assert "HistoryPolicy.KEEP_LAST" in image_subscription
    assert "depth=1" in image_subscription
    assert "const sourceActive = physicalSession || telemetryActive" in web
    assert "'Source'," in web
    assert "'Evidence'," in web
