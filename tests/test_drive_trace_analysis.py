import hashlib
import json
from pathlib import Path

import pytest

from sphero_rvr_driver.drive_trace_analysis import (
    ANALYSIS_SCHEMA,
    CONTEXT_SCHEMA,
    DriveTraceAnalysisError,
    analyze_trace,
    phase1_motion_evidence_routing,
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


def _write_trace(
    path: Path,
    *,
    terminal_state: str = "complete",
    terminal_reason: str = "finish",
) -> str:
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
                "state": terminal_state,
                "reason": terminal_reason,
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


def _timeout_context(trace_sha256: str):
    context = _context(trace_sha256)
    context.pop("semantic_completion")
    context["mission_terminal"] = {
        "event_sha256": "d" * 64,
        "status": "timeout",
        "controller_state": "recovery_required",
        "reason": "mission_lease_expired",
    }
    return context


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
    assert report["phase1_motion_evidence"]["outcome"] == "geometry_ineligible"
    assert report["phase1_motion_evidence"]["routes_to_phase0b"] is False


def test_analysis_accepts_ordered_rotation_segments_and_safe_lease_terminal(
    tmp_path: Path,
) -> None:
    combined = tmp_path / "combined.jsonl"
    digest = _write_trace(
        combined,
        terminal_state="recovery_required",
        terminal_reason="mission_lease_expired",
    )
    raw = combined.read_bytes()
    split = raw.find(b"\n", len(raw) // 2) + 1
    previous = tmp_path / "trace.previous.jsonl"
    current = tmp_path / "trace.jsonl"
    previous.write_bytes(raw[:split])
    current.write_bytes(raw[split:])

    report = analyze_trace(
        [previous, current], context=_timeout_context(digest)
    )

    assert report["trace"]["sha256"] == digest
    assert report["trace"]["segment_count"] == 2
    assert report["active_navigation_interval"]["terminal_state"] == (
        "recovery_required"
    )
    assert report["active_navigation_interval"]["terminal_reason"] == (
        "mission_lease_expired"
    )
    assert report["mission_interval"]["terminal_state"] == (
        "recovery_required"
    )
    geometry = report["goal_geometry_and_completion"]
    assert geometry["semantic_completion"] is None
    assert geometry["mission_terminal"]["status"] == "timeout"


def test_analysis_rejects_non_lease_recovery_terminal(tmp_path: Path) -> None:
    trace = tmp_path / "trace.jsonl"
    digest = _write_trace(
        trace,
        terminal_state="recovery_required",
        terminal_reason="controller_failure",
    )

    with pytest.raises(
        DriveTraceAnalysisError, match="accepted controller terminal"
    ):
        analyze_trace(trace, context=_timeout_context(digest))


def test_analysis_rejects_reversed_rotation_segments(tmp_path: Path) -> None:
    combined = tmp_path / "combined.jsonl"
    digest = _write_trace(combined)
    raw = combined.read_bytes()
    split = raw.find(b"\n", len(raw) // 2) + 1
    previous = tmp_path / "trace.previous.jsonl"
    current = tmp_path / "trace.jsonl"
    previous.write_bytes(raw[:split])
    current.write_bytes(raw[split:])

    with pytest.raises(DriveTraceAnalysisError, match="segment order"):
        analyze_trace([current, previous], context=_context(digest))


@pytest.mark.parametrize(
    ("bearing_deg", "duration_s", "window_odom_m", "mission_odom_m", "outcome"),
    [
        (46.0, 1.1, 0.06, 0.60, "geometry_ineligible"),
        (0.0, 0.9, 0.06, 0.60, "forward_command_inconclusive"),
        (0.0, 1.1, 0.04, 0.60, "phase0b_breakaway_required"),
        (0.0, 1.1, 0.06, 0.49, "mission_distance_incomplete"),
        (0.0, 1.1, 0.06, 0.60, "motion_evidence_pass"),
    ],
)
def test_phase1_motion_evidence_routes_only_a_real_stall_to_phase0b(
    bearing_deg: float,
    duration_s: float,
    window_odom_m: float,
    mission_odom_m: float,
    outcome: str,
) -> None:
    result = phase1_motion_evidence_routing(
        goal_geometry={"relative_bearing_deg": bearing_deg},
        active_navigation_odometry={"net_displacement_m": mission_odom_m},
        motor_forward_windows=[
            {
                "window": 1,
                "duration_s": duration_s,
                "linear_x_time_weighted_mean_mps": 0.10,
                "angular_z_max_abs_rad_s": 0.0,
                "odometry": {"net_displacement_m": window_odom_m},
            }
        ],
    )

    assert result["outcome"] == outcome
    assert result["routes_to_phase0b"] is (
        outcome == "phase0b_breakaway_required"
    )


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
