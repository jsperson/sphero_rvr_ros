import hashlib
import json
from pathlib import Path

import pytest

from sphero_rvr_driver.drive_trace_analysis import (
    ANALYSIS_SCHEMA,
    CONTEXT_SCHEMA,
    DriveTraceAnalysisError,
    analyze_trace,
)


MISSION = "m8-phase0-synthetic"
SOURCE_SHA = "a" * 40
SAMPLE_SCHEMA = "sphero_rvr.physical_drive_trace_sample.v1"


def _event(kind: str, **values):
    return {
        "schema": SAMPLE_SCHEMA,
        "mission_id": MISSION,
        "source_sha": SOURCE_SHA,
        "kind": kind,
        **values,
    }


def _write_trace(path: Path) -> str:
    events = [
        _event("trace_started"),
        _event(
            "state",
            stream="collision",
            recorded_at_s=0.9,
            value={"state": "CLEAR"},
        ),
        _event(
            "twist",
            stream="motor_output",
            recorded_at_s=0.9,
            linear_x=0.0,
            angular_z=0.0,
        ),
        _event(
            "twist",
            stream="supervisor_request",
            recorded_at_s=0.9,
            linear_x=0.0,
            angular_z=0.0,
        ),
        _event("odom", recorded_at_s=0.9, x_m=0.0, y_m=0.0, yaw_rad=0.0),
        _event(
            "twist",
            stream="nav2_request",
            recorded_at_s=1.0,
            linear_x=0.0,
            angular_z=-0.4,
        ),
        _event(
            "state",
            stream="hierarchical_controller",
            recorded_at_s=1.0,
            value={"mission_id": MISSION, "state": "navigating"},
        ),
        _event(
            "twist",
            stream="supervisor_request",
            recorded_at_s=1.0,
            linear_x=0.0,
            angular_z=-0.35,
        ),
        _event(
            "twist",
            stream="motor_output",
            recorded_at_s=1.0,
            linear_x=0.0,
            angular_z=-0.35,
        ),
        _event("odom", recorded_at_s=1.2, x_m=0.0, y_m=0.0, yaw_rad=-0.2),
        _event(
            "twist",
            stream="nav2_request",
            recorded_at_s=2.0,
            linear_x=0.04,
            angular_z=0.0,
        ),
        _event(
            "twist",
            stream="supervisor_request",
            recorded_at_s=2.0,
            linear_x=0.10,
            angular_z=0.0,
        ),
        _event(
            "twist",
            stream="motor_output",
            recorded_at_s=2.0,
            linear_x=0.10,
            angular_z=0.0,
        ),
        _event("odom", recorded_at_s=2.2, x_m=0.02, y_m=0.0, yaw_rad=-0.2),
        _event(
            "twist",
            stream="nav2_request",
            recorded_at_s=3.0,
            linear_x=0.0,
            angular_z=0.0,
        ),
        _event(
            "twist",
            stream="supervisor_request",
            recorded_at_s=3.0,
            linear_x=0.0,
            angular_z=0.0,
        ),
        _event(
            "twist",
            stream="motor_output",
            recorded_at_s=3.0,
            linear_x=0.0,
            angular_z=0.0,
        ),
        _event("odom", recorded_at_s=3.25, x_m=0.10, y_m=0.0, yaw_rad=-0.2),
        _event(
            "state",
            stream="hierarchical_controller",
            recorded_at_s=4.0,
            value={
                "mission_id": MISSION,
                "state": "complete",
                "reason": "finish",
            },
        ),
        _event(
            "trace_summary",
            summary={"schema": "sphero_rvr.physical_drive_trace_summary.v1"},
        ),
    ]
    raw = "".join(json.dumps(event, sort_keys=True) + "\n" for event in events).encode()
    path.write_bytes(raw)
    return hashlib.sha256(raw).hexdigest()


def _context(trace_sha256: str):
    return {
        "schema": CONTEXT_SCHEMA,
        "mission_id": MISSION,
        "source_sha": SOURCE_SHA,
        "trace_sha256": trace_sha256,
        "goal_dispatch": {
            "event_sha256": "b" * 64,
            "localization": {
                "frame_id": "map",
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_rad": 0.0,
            },
            "target": {"frame_id": "map", "x_m": -1.0, "y_m": 0.0},
        },
        "semantic_completion": {
            "event_sha256": "c" * 64,
            "action": "finish",
            "outcome": "partial",
        },
    }


def test_analysis_time_aligns_commands_odometry_and_goal_context(tmp_path: Path) -> None:
    trace = tmp_path / "trace.jsonl"
    digest = _write_trace(trace)
    report = analyze_trace(trace, context=_context(digest))

    assert report["schema"] == ANALYSIS_SCHEMA
    assert report["trace"]["sha256"] == digest
    assert report["active_navigation_interval"]["duration_s"] == pytest.approx(3.0)
    motor = report["active_navigation_command_streams"]["motor_output"]
    assert motor["categories"]["duration_s"]["pure_rotation"] == pytest.approx(0.5)
    assert motor["categories"]["duration_s"]["forward"] == pytest.approx(0.5)
    assert motor["jitter"]["angular_breakaway_floor_duration_s"] == pytest.approx(0.5)
    assert motor["jitter"]["motion_start_count"] == 2
    assert report["mission_interval"]["duration_s"] == pytest.approx(3.0)
    assert report["mission_command_streams"]["motor_output"]["categories"] == (
        motor["categories"]
    )
    assert len(report["motor_forward_windows"]) == 1
    assert report["motor_forward_windows"][0][
        "angular_z_max_abs_rad_s"
    ] == pytest.approx(0.0)
    assert report["motor_forward_windows"][0]["odometry"]["net_displacement_m"] == pytest.approx(0.10)
    geometry = report["goal_geometry_and_completion"]
    assert geometry["target_was_behind"] is True
    assert geometry["relative_bearing_deg"] == pytest.approx(180.0)


def test_analysis_treats_expired_commands_as_zero(tmp_path: Path) -> None:
    trace = tmp_path / "trace.jsonl"
    digest = _write_trace(trace)
    report = analyze_trace(trace, context=_context(digest))
    nav2 = report["active_navigation_command_streams"]["nav2_request"][
        "categories"
    ]["duration_s"]
    assert nav2["pure_rotation"] == pytest.approx(0.5)
    assert nav2["forward"] == pytest.approx(0.5)
    assert nav2["zero"] == pytest.approx(2.0)


def test_analysis_rejects_context_not_bound_to_raw_trace(tmp_path: Path) -> None:
    trace = tmp_path / "trace.jsonl"
    digest = _write_trace(trace)
    context = _context(digest)
    context["trace_sha256"] = "0" * 64
    with pytest.raises(DriveTraceAnalysisError, match="trace_sha256 mismatch"):
        analyze_trace(trace, context=context)


def test_analysis_rejects_mixed_trace_provenance(tmp_path: Path) -> None:
    trace = tmp_path / "trace.jsonl"
    _write_trace(trace)
    events = [json.loads(line) for line in trace.read_text().splitlines()]
    events[1]["source_sha"] = "f" * 40
    trace.write_text("".join(json.dumps(event) + "\n" for event in events))
    with pytest.raises(DriveTraceAnalysisError, match="exactly one source SHA"):
        analyze_trace(trace)


def test_committed_phase0_report_is_bound_to_context_and_private_trace() -> None:
    root = Path(__file__).parents[1]
    artifact = root / "artifacts" / "m8_phase0_drive_trace_analysis"
    context = json.loads((artifact / "context.json").read_text(encoding="utf-8"))
    report = json.loads((artifact / "report.json").read_text(encoding="utf-8"))
    canonical = hashlib.sha256(
        json.dumps(
            context, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
    ).hexdigest()

    assert report["schema"] == ANALYSIS_SCHEMA
    assert report["mission_id"] == context["mission_id"]
    assert report["source_sha"] == context["source_sha"]
    assert report["trace"]["sha256"] == context["trace_sha256"]
    assert (
        report["goal_geometry_and_completion"]["context_canonical_sha256"]
        == canonical
    )
