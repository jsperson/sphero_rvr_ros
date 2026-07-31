"""Deterministic, ROS-free analysis of synchronized physical drive traces."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence


ANALYSIS_SCHEMA = "sphero_rvr.m8_phase0_drive_trace_analysis.v1"
CONTEXT_SCHEMA = "sphero_rvr.m8_phase0_drive_trace_context.v1"
TRACE_SAMPLE_SCHEMA = "sphero_rvr.physical_drive_trace_sample.v1"
_COMMAND_STREAMS = ("nav2_request", "supervisor_request", "motor_output")
PHASE1_MAX_INITIAL_BEARING_DEG = 45.0
PHASE1_MIN_FORWARD_WINDOW_S = 1.0
PHASE1_MIN_FORWARD_MEAN_MPS = 0.09
PHASE1_MAX_FORWARD_ANGULAR_RAD_S = 0.05
PHASE1_MIN_WINDOW_ODOM_M = 0.05
PHASE1_MIN_MISSION_ODOM_M = 0.50


class DriveTraceAnalysisError(ValueError):
    """Raised when trace provenance or event structure is not reviewable."""


def _finite(value: Any, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise DriveTraceAnalysisError(f"{name} must be finite") from exc
    if not math.isfinite(parsed):
        raise DriveTraceAnalysisError(f"{name} must be finite")
    return parsed


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical_sha256(value: Mapping[str, Any]) -> str:
    return _sha256(
        json.dumps(
            dict(value), sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DriveTraceAnalysisError(f"cannot read JSON context: {path}") from exc
    if not isinstance(value, Mapping):
        raise DriveTraceAnalysisError("analysis context must be a JSON object")
    return value


def _trace_paths(
    path: str | Path | Sequence[str | Path],
) -> list[Path]:
    if isinstance(path, (str, Path)):
        return [Path(path)]
    paths = [Path(item) for item in path]
    if not paths:
        raise DriveTraceAnalysisError("at least one drive trace segment is required")
    return paths


def load_trace(
    path: str | Path | Sequence[str | Path],
) -> tuple[list[dict[str, Any]], str, int]:
    """Load ordered private JSONL segments and return events and combined digest."""

    paths = _trace_paths(path)
    chunks: list[bytes] = []
    for trace_path in paths:
        try:
            chunks.append(trace_path.read_bytes())
        except OSError as exc:
            raise DriveTraceAnalysisError(
                f"cannot read drive trace: {trace_path}"
            ) from exc
    raw = b"".join(chunks)
    events: list[dict[str, Any]] = []
    for line_number, line in enumerate(raw.splitlines(), start=1):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError as exc:
            raise DriveTraceAnalysisError(
                f"drive trace line {line_number} is not JSON"
            ) from exc
        if not isinstance(event, dict):
            raise DriveTraceAnalysisError(
                f"drive trace line {line_number} is not an object"
            )
        events.append(event)
    if not events:
        raise DriveTraceAnalysisError("drive trace is empty")
    return events, _sha256(raw), len(raw)


def _validate_provenance(
    events: Sequence[Mapping[str, Any]],
) -> tuple[str, str]:
    missions = {str(event.get("mission_id", "")).strip() for event in events}
    sources = {str(event.get("source_sha", "")).strip() for event in events}
    if len(missions) != 1 or "" in missions:
        raise DriveTraceAnalysisError("trace must bind exactly one mission ID")
    if len(sources) != 1 or "" in sources:
        raise DriveTraceAnalysisError("trace must bind exactly one source SHA")
    if any(
        event.get("schema") != TRACE_SAMPLE_SCHEMA
        for event in events
    ):
        raise DriveTraceAnalysisError("trace contains an unexpected sample schema")
    if sum(event.get("kind") == "trace_started" for event in events) != 1:
        raise DriveTraceAnalysisError("trace must contain one start record")
    if sum(event.get("kind") == "trace_summary" for event in events) != 1:
        raise DriveTraceAnalysisError("trace must contain one terminal summary")
    if (
        events[0].get("kind") != "trace_started"
        or events[-1].get("kind") != "trace_summary"
    ):
        raise DriveTraceAnalysisError(
            "drive trace segment order must begin with trace_started and end "
            "with trace_summary"
        )
    return next(iter(missions)), next(iter(sources))


def _timed(events: Iterable[Mapping[str, Any]]) -> list[dict[str, Any]]:
    timed_events: list[dict[str, Any]] = []
    for raw in events:
        if raw.get("recorded_at_s") is None:
            continue
        event = dict(raw)
        event["recorded_at_s"] = _finite(
            event["recorded_at_s"], "event receipt time"
        )
        timed_events.append(event)
    return sorted(timed_events, key=lambda item: item["recorded_at_s"])


def _command_events(
    events: Sequence[Mapping[str, Any]], stream: str
) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    for event in events:
        if event.get("kind") != "twist" or event.get("stream") != stream:
            continue
        samples.append(
            {
                "time_s": _finite(event.get("recorded_at_s"), "command time"),
                "linear_x": _finite(event.get("linear_x"), "linear command"),
                "angular_z": _finite(event.get("angular_z"), "angular command"),
            }
        )
    return sorted(samples, key=lambda item: item["time_s"])


def _state_events(
    events: Sequence[Mapping[str, Any]], stream: str
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for event in events:
        if event.get("kind") != "state" or event.get("stream") != stream:
            continue
        samples.append(
            {
                "time_s": _finite(event.get("recorded_at_s"), "state time"),
                "value": event.get("value"),
            }
        )
    return sorted(samples, key=lambda item: item["time_s"])


def _value_at(
    samples: Sequence[Mapping[str, Any]],
    at_s: float,
    *,
    max_hold_s: float,
) -> Optional[Mapping[str, Any]]:
    latest: Optional[Mapping[str, Any]] = None
    for sample in samples:
        if float(sample["time_s"]) > at_s:
            break
        latest = sample
    if latest is None or at_s - float(latest["time_s"]) > max_hold_s:
        return None
    return latest


def _segments(
    samples: Sequence[Mapping[str, Any]],
    *,
    start_s: float,
    end_s: float,
    max_hold_s: float,
    zero_value: Mapping[str, Any],
) -> list[dict[str, Any]]:
    if not start_s < end_s:
        raise DriveTraceAnalysisError("analysis interval must be positive")
    breakpoints = {start_s, end_s}
    for sample in samples:
        time_s = float(sample["time_s"])
        if start_s < time_s < end_s:
            breakpoints.add(time_s)
        expiry = time_s + max_hold_s
        if start_s < expiry < end_s:
            breakpoints.add(expiry)
    ordered = sorted(breakpoints)
    output: list[dict[str, Any]] = []
    for left, right in zip(ordered, ordered[1:]):
        midpoint = (left + right) / 2.0
        sample = _value_at(samples, midpoint, max_hold_s=max_hold_s)
        value = dict(zero_value if sample is None else sample)
        value.pop("time_s", None)
        segment = {"start_s": left, "end_s": right, **value}
        if output and all(
            output[-1].get(key) == segment.get(key)
            for key in set(output[-1]) | set(segment)
            if key not in {"start_s", "end_s"}
        ):
            output[-1]["end_s"] = right
        else:
            output.append(segment)
    return output


def _command_category(linear_x: float, angular_z: float) -> str:
    if linear_x > 0.001:
        return "forward"
    if linear_x < -0.001:
        return "reverse"
    if abs(angular_z) > 0.01:
        return "pure_rotation"
    return "zero"


def _command_segments(
    samples: Sequence[Mapping[str, Any]],
    *,
    start_s: float,
    end_s: float,
    max_hold_s: float,
) -> list[dict[str, Any]]:
    segments = _segments(
        samples,
        start_s=start_s,
        end_s=end_s,
        max_hold_s=max_hold_s,
        zero_value={"linear_x": 0.0, "angular_z": 0.0},
    )
    for segment in segments:
        segment["category"] = _command_category(
            float(segment["linear_x"]), float(segment["angular_z"])
        )
    return segments


def _duration(segment: Mapping[str, Any]) -> float:
    return float(segment["end_s"]) - float(segment["start_s"])


def _category_metrics(
    segments: Sequence[Mapping[str, Any]], duration_s: float
) -> dict[str, Any]:
    durations = {
        name: sum(
            _duration(segment)
            for segment in segments
            if segment["category"] == name
        )
        for name in ("forward", "reverse", "pure_rotation", "zero")
    }
    return {
        "duration_s": durations,
        "fraction": {
            name: value / duration_s for name, value in durations.items()
        },
    }


def _jitter_metrics(
    samples: Sequence[Mapping[str, Any]],
    segments: Sequence[Mapping[str, Any]],
    *,
    start_s: float,
    end_s: float,
) -> dict[str, Any]:
    duration_s = end_s - start_s
    angular_nonzero_s = sum(
        _duration(segment)
        for segment in segments
        if abs(float(segment["angular_z"])) > 0.01
    )
    floor_s = sum(
        _duration(segment)
        for segment in segments
        if math.isclose(
            abs(float(segment["angular_z"])), 0.35, rel_tol=0.0, abs_tol=1e-9
        )
    )
    motion_starts = 0
    previous_moving = False
    for segment in segments:
        moving = segment["category"] != "zero"
        if moving and not previous_moving:
            motion_starts += 1
        previous_moving = moving

    angular_changes: list[tuple[float, float]] = []
    initial = _value_at(samples, start_s, max_hold_s=0.50)
    previous = 0.0 if initial is None else float(initial["angular_z"])
    angular_changes.append((start_s, previous))
    for sample in samples:
        time_s = float(sample["time_s"])
        if not start_s < time_s <= end_s:
            continue
        angular = float(sample["angular_z"])
        if not math.isclose(angular, previous, rel_tol=0.0, abs_tol=1e-12):
            angular_changes.append((time_s, angular))
            previous = angular
    total_variation = sum(
        abs(current[1] - previous_item[1])
        for previous_item, current in zip(angular_changes, angular_changes[1:])
    )
    accelerations: list[tuple[float, float]] = []
    for previous_item, current in zip(angular_changes, angular_changes[1:]):
        elapsed = current[0] - previous_item[0]
        if elapsed > 0.0:
            accelerations.append((current[0], (current[1] - previous_item[1]) / elapsed))
    jerks: list[float] = []
    for previous_item, current in zip(accelerations, accelerations[1:]):
        elapsed = current[0] - previous_item[0]
        if elapsed > 0.0:
            jerks.append((current[1] - previous_item[1]) / elapsed)

    last_sign = 0
    sign_reversals = 0
    for _, angular in angular_changes:
        sign = 1 if angular > 0.01 else -1 if angular < -0.01 else 0
        if sign:
            if last_sign and sign != last_sign:
                sign_reversals += 1
            last_sign = sign
    return {
        "angular_nonzero_duration_s": angular_nonzero_s,
        "angular_nonzero_duty": angular_nonzero_s / duration_s,
        "angular_breakaway_floor_duration_s": floor_s,
        "angular_breakaway_floor_duty": floor_s / duration_s,
        "angular_total_variation_rad_s": total_variation,
        "angular_change_count": max(0, len(angular_changes) - 1),
        "angular_sign_reversals": sign_reversals,
        "motion_start_count": motion_starts,
        "discrete_angular_jerk": {
            "definition": (
                "finite difference of angular-command acceleration across "
                "consecutive value changes; diagnostic, sample-timing-sensitive"
            ),
            "sample_count": len(jerks),
            "rms_rad_s3": (
                math.sqrt(sum(value * value for value in jerks) / len(jerks))
                if jerks
                else 0.0
            ),
            "max_abs_rad_s3": max((abs(value) for value in jerks), default=0.0),
        },
    }


def _odom_events(events: Sequence[Mapping[str, Any]]) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    for event in events:
        if event.get("kind") != "odom":
            continue
        samples.append(
            {
                "time_s": _finite(event.get("recorded_at_s"), "odometry time"),
                "x_m": _finite(event.get("x_m"), "odometry x"),
                "y_m": _finite(event.get("y_m"), "odometry y"),
                "yaw_rad": _finite(event.get("yaw_rad"), "odometry yaw"),
            }
        )
    return sorted(samples, key=lambda item: item["time_s"])


def _pose_before(
    samples: Sequence[Mapping[str, float]], at_s: float
) -> Mapping[str, float]:
    candidates = [sample for sample in samples if float(sample["time_s"]) <= at_s]
    if candidates:
        return candidates[-1]
    if samples:
        return samples[0]
    raise DriveTraceAnalysisError("trace contains no odometry")


def _pose_after(
    samples: Sequence[Mapping[str, float]], at_s: float
) -> Mapping[str, float]:
    for sample in samples:
        if float(sample["time_s"]) >= at_s:
            return sample
    if samples:
        return samples[-1]
    raise DriveTraceAnalysisError("trace contains no odometry")


def _odom_metrics(
    samples: Sequence[Mapping[str, float]], start_s: float, end_s: float
) -> dict[str, Any]:
    start = _pose_before(samples, start_s)
    end = _pose_after(samples, end_s)
    included = [
        sample
        for sample in samples
        if float(start["time_s"]) <= float(sample["time_s"]) <= float(end["time_s"])
    ]
    if not included or included[0] is not start:
        included.insert(0, start)
    if included[-1] is not end:
        included.append(end)
    path_distance = 0.0
    signed_yaw = 0.0
    absolute_yaw = 0.0
    for previous, current in zip(included, included[1:]):
        path_distance += math.hypot(
            float(current["x_m"]) - float(previous["x_m"]),
            float(current["y_m"]) - float(previous["y_m"]),
        )
        delta = _normalize_angle(
            float(current["yaw_rad"]) - float(previous["yaw_rad"])
        )
        signed_yaw += delta
        absolute_yaw += abs(delta)
    net_displacement = math.hypot(
        float(end["x_m"]) - float(start["x_m"]),
        float(end["y_m"]) - float(start["y_m"]),
    )
    return {
        "start_sample_time_s": float(start["time_s"]),
        "end_sample_time_s": float(end["time_s"]),
        "net_displacement_m": net_displacement,
        "path_distance_m": path_distance,
        "signed_yaw_change_rad": signed_yaw,
        "absolute_yaw_change_rad": absolute_yaw,
        "absolute_yaw_per_net_m_rad_m": (
            absolute_yaw / net_displacement if net_displacement > 1e-9 else None
        ),
    }


def _collision_segments(
    events: Sequence[Mapping[str, Any]], start_s: float, end_s: float
) -> list[dict[str, Any]]:
    raw = _state_events(events, "collision")
    samples = []
    for sample in raw:
        value = sample.get("value")
        state = value.get("state") if isinstance(value, Mapping) else None
        samples.append({"time_s": sample["time_s"], "state": str(state or "UNKNOWN")})
    return _segments(
        samples,
        start_s=start_s,
        end_s=end_s,
        max_hold_s=0.30,
        zero_value={"state": "UNKNOWN"},
    )


def _overlap(left: Mapping[str, Any], right: Mapping[str, Any]) -> float:
    return max(
        0.0,
        min(float(left["end_s"]), float(right["end_s"]))
        - max(float(left["start_s"]), float(right["start_s"])),
    )


def _forward_windows(
    command_segments: Sequence[Mapping[str, Any]],
    collision_segments: Sequence[Mapping[str, Any]],
    odom: Sequence[Mapping[str, float]],
    *,
    response_lag_s: float,
) -> list[dict[str, Any]]:
    groups: list[list[Mapping[str, Any]]] = []
    for segment in command_segments:
        if segment["category"] != "forward":
            continue
        if groups and math.isclose(
            float(groups[-1][-1]["end_s"]),
            float(segment["start_s"]),
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            groups[-1].append(segment)
        else:
            groups.append([segment])
    windows: list[dict[str, Any]] = []
    for index, group in enumerate(groups, start=1):
        start_s = float(group[0]["start_s"])
        end_s = float(group[-1]["end_s"])
        duration_s = end_s - start_s
        collision_duration: dict[str, float] = {}
        for collision in collision_segments:
            amount = sum(_overlap(segment, collision) for segment in group)
            if amount > 0.0:
                state = str(collision["state"])
                collision_duration[state] = collision_duration.get(state, 0.0) + amount
        windows.append(
            {
                "window": index,
                "start_s": start_s,
                "end_s": end_s,
                "duration_s": duration_s,
                "linear_x_min_mps": min(float(item["linear_x"]) for item in group),
                "linear_x_max_mps": max(float(item["linear_x"]) for item in group),
                "linear_x_time_weighted_mean_mps": sum(
                    float(item["linear_x"]) * _duration(item) for item in group
                )
                / duration_s,
                "angular_z_time_weighted_mean_rad_s": sum(
                    float(item["angular_z"]) * _duration(item) for item in group
                )
                / duration_s,
                "angular_z_max_abs_rad_s": max(
                    abs(float(item["angular_z"])) for item in group
                ),
                "assumed_floor_duration_s": sum(
                    _duration(item)
                    for item in group
                    if float(item["linear_x"]) >= 0.099
                ),
                "collision_state_duration_s": collision_duration,
                "odom_response_lag_s": response_lag_s,
                "odometry": _odom_metrics(odom, start_s, end_s + response_lag_s),
            }
        )
    return windows


def _accepted_controller_terminal(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    state = str(value.get("state", ""))
    if state == "complete":
        return True
    return bool(
        state == "recovery_required"
        and value.get("reason") == "mission_lease_expired"
    )


def _find_interval(
    events: Sequence[Mapping[str, Any]], mission_id: str
) -> dict[str, Any]:
    nav2 = _command_events(events, "nav2_request")
    nonzero = [
        sample
        for sample in nav2
        if abs(sample["linear_x"]) > 0.001 or abs(sample["angular_z"]) > 0.01
    ]
    if not nonzero:
        raise DriveTraceAnalysisError("trace contains no active Nav2 command")
    start_s = nonzero[0]["time_s"]
    terminals = [
        sample
        for sample in _state_events(events, "hierarchical_controller")
        if sample["time_s"] > start_s
        and isinstance(sample.get("value"), Mapping)
        and sample["value"].get("mission_id") == mission_id
        and _accepted_controller_terminal(sample["value"])
    ]
    if not terminals:
        raise DriveTraceAnalysisError(
            "trace has no accepted controller terminal after motion"
        )
    terminal = terminals[0]
    end_s = float(terminal["time_s"])
    return {
        "start_s": start_s,
        "end_s": end_s,
        "duration_s": end_s - start_s,
        "terminal_state": str(terminal["value"].get("state", "")),
        "terminal_reason": str(terminal["value"].get("reason", "")),
    }


def _find_mission_interval(
    events: Sequence[Mapping[str, Any]], mission_id: str
) -> dict[str, Any]:
    controller = [
        sample
        for sample in _state_events(events, "hierarchical_controller")
        if isinstance(sample.get("value"), Mapping)
        and sample["value"].get("mission_id") == mission_id
    ]
    if not controller:
        raise DriveTraceAnalysisError("trace has no mission-bound controller state")
    terminals = [
        sample
        for sample in controller
        if _accepted_controller_terminal(sample["value"])
    ]
    if not terminals:
        raise DriveTraceAnalysisError(
            "trace has no accepted mission-bound terminal"
        )
    start_s = float(controller[0]["time_s"])
    terminal = terminals[0]
    end_s = float(terminal["time_s"])
    return {
        "start_s": start_s,
        "end_s": end_s,
        "duration_s": end_s - start_s,
        "terminal_state": str(terminal["value"].get("state", "")),
        "terminal_reason": str(terminal["value"].get("reason", "")),
    }


def _goal_context(
    context: Optional[Mapping[str, Any]],
    *,
    mission_id: str,
    source_sha: str,
    trace_sha256: str,
) -> Optional[dict[str, Any]]:
    if context is None:
        return None
    if context.get("schema") != CONTEXT_SCHEMA:
        raise DriveTraceAnalysisError("drive trace context schema is invalid")
    for field, expected in (
        ("mission_id", mission_id),
        ("source_sha", source_sha),
        ("trace_sha256", trace_sha256),
    ):
        if str(context.get(field, "")).strip() != expected:
            raise DriveTraceAnalysisError(f"drive trace context {field} mismatch")
    dispatch = context.get("goal_dispatch")
    completion = context.get("semantic_completion")
    mission_terminal = context.get("mission_terminal")
    if not isinstance(dispatch, Mapping):
        raise DriveTraceAnalysisError("drive trace context is incomplete")
    if isinstance(completion, Mapping) == isinstance(mission_terminal, Mapping):
        raise DriveTraceAnalysisError(
            "drive trace context requires exactly one semantic completion or "
            "mission terminal"
        )
    localization = dispatch.get("localization")
    target = dispatch.get("target")
    if not isinstance(localization, Mapping) or not isinstance(target, Mapping):
        raise DriveTraceAnalysisError("goal-dispatch geometry is incomplete")
    terminal_context = (
        completion if isinstance(completion, Mapping) else mission_terminal
    )
    assert isinstance(terminal_context, Mapping)
    terminal_name = (
        "semantic completion"
        if isinstance(completion, Mapping)
        else "mission terminal"
    )
    for name, value in (
        ("goal dispatch", dispatch.get("event_sha256")),
        (terminal_name, terminal_context.get("event_sha256")),
    ):
        digest = str(value or "")
        if len(digest) != 64 or any(
            character not in "0123456789abcdef" for character in digest
        ):
            raise DriveTraceAnalysisError(f"{name} event digest is invalid")
    if isinstance(mission_terminal, Mapping) and (
        mission_terminal.get("status") != "timeout"
        or mission_terminal.get("controller_state") != "recovery_required"
        or mission_terminal.get("reason") != "mission_lease_expired"
    ):
        raise DriveTraceAnalysisError(
            "drive trace mission terminal is not a safe lease expiry"
        )
    dx = _finite(target.get("x_m"), "target x") - _finite(
        localization.get("x_m"), "localization x"
    )
    dy = _finite(target.get("y_m"), "target y") - _finite(
        localization.get("y_m"), "localization y"
    )
    relative = _normalize_angle(
        math.atan2(dy, dx) - _finite(localization.get("yaw_rad"), "localization yaw")
    )
    return {
        "context_canonical_sha256": _canonical_sha256(context),
        "goal_dispatch_event_sha256": str(dispatch.get("event_sha256", "")),
        "localization": dict(localization),
        "target": dict(target),
        "relative_bearing_rad": relative,
        "relative_bearing_deg": math.degrees(relative),
        "target_was_behind": abs(relative) > math.pi / 2.0,
        "semantic_completion": (
            dict(completion) if isinstance(completion, Mapping) else None
        ),
        "mission_terminal": (
            dict(mission_terminal)
            if isinstance(mission_terminal, Mapping)
            else None
        ),
    }


def phase1_motion_evidence_routing(
    *,
    goal_geometry: Optional[Mapping[str, Any]],
    active_navigation_odometry: Mapping[str, Any],
    motor_forward_windows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Route Phase-1 motion evidence without mistaking geometry for breakaway.

    This is an after-run evidence classifier, not a runtime safety or motion
    gate.  Phase 0B is eligible only when the dispatched goal was initially
    forward and the motor actually received a sustained, nearly straight
    command that failed to translate.
    """

    thresholds = {
        "max_initial_absolute_bearing_deg": PHASE1_MAX_INITIAL_BEARING_DEG,
        "min_forward_window_s": PHASE1_MIN_FORWARD_WINDOW_S,
        "min_forward_mean_mps": PHASE1_MIN_FORWARD_MEAN_MPS,
        "max_forward_angular_rad_s": PHASE1_MAX_FORWARD_ANGULAR_RAD_S,
        "min_window_net_odometry_m": PHASE1_MIN_WINDOW_ODOM_M,
        "min_mission_net_odometry_m": PHASE1_MIN_MISSION_ODOM_M,
    }

    def result(
        outcome: str,
        reason: str,
        *,
        qualifying_windows: Sequence[int] = (),
        translating_windows: Sequence[int] = (),
    ) -> dict[str, Any]:
        return {
            "outcome": outcome,
            "reason": reason,
            "routes_to_phase0b": outcome == "phase0b_breakaway_required",
            "qualifying_forward_windows": list(qualifying_windows),
            "translating_forward_windows": list(translating_windows),
            "thresholds": thresholds,
        }

    if goal_geometry is None:
        return result(
            "not_evaluable_without_goal_geometry",
            "SHA-bound dispatch geometry is required before assigning a motor verdict",
        )
    bearing_deg = abs(
        _finite(goal_geometry.get("relative_bearing_deg"), "relative goal bearing")
    )
    if bearing_deg > PHASE1_MAX_INITIAL_BEARING_DEG:
        return result(
            "geometry_ineligible",
            "the initial target bearing was not a genuinely forward validation leg",
        )

    qualifying: list[Mapping[str, Any]] = []
    for window in motor_forward_windows:
        if (
            _finite(window.get("duration_s"), "forward-window duration")
            >= PHASE1_MIN_FORWARD_WINDOW_S
            and _finite(
                window.get("linear_x_time_weighted_mean_mps"),
                "forward-window mean linear command",
            )
            >= PHASE1_MIN_FORWARD_MEAN_MPS
            and _finite(
                window.get("angular_z_max_abs_rad_s"),
                "forward-window maximum angular command",
            )
            <= PHASE1_MAX_FORWARD_ANGULAR_RAD_S
        ):
            qualifying.append(window)
    qualifying_ids = [int(item["window"]) for item in qualifying]
    if not qualifying:
        return result(
            "forward_command_inconclusive",
            "no sustained nearly straight motor-output window tested forward breakaway",
        )

    translating = [
        window
        for window in qualifying
        if isinstance(window.get("odometry"), Mapping)
        and _finite(
            window["odometry"].get("net_displacement_m"),
            "forward-window net odometry",
        )
        >= PHASE1_MIN_WINDOW_ODOM_M
    ]
    translating_ids = [int(item["window"]) for item in translating]
    if not translating:
        return result(
            "phase0b_breakaway_required",
            "a sustained nearly straight motor command failed to translate",
            qualifying_windows=qualifying_ids,
        )

    mission_odom = _finite(
        active_navigation_odometry.get("net_displacement_m"),
        "active-navigation net odometry",
    )
    if mission_odom < PHASE1_MIN_MISSION_ODOM_M:
        return result(
            "mission_distance_incomplete",
            "forward breakaway was demonstrated but the mission distance gate was not met",
            qualifying_windows=qualifying_ids,
            translating_windows=translating_ids,
        )
    return result(
        "motion_evidence_pass",
        "the geometry, sustained forward command, and translation gates were met",
        qualifying_windows=qualifying_ids,
        translating_windows=translating_ids,
    )


def analyze_trace(
    path: str | Path | Sequence[str | Path],
    *,
    context: Optional[Mapping[str, Any]] = None,
    response_lag_s: float = 0.25,
) -> dict[str, Any]:
    """Produce a deterministic compact Phase-0 analysis from one raw trace."""

    if not 0.0 <= response_lag_s <= 1.0:
        raise DriveTraceAnalysisError("odometry response lag must be in [0, 1] seconds")
    events, trace_digest, size_bytes = load_trace(path)
    mission_id, source_sha = _validate_provenance(events)
    timed = _timed(events)
    interval = _find_interval(timed, mission_id)
    mission_interval = _find_mission_interval(timed, mission_id)
    start_s = interval["start_s"]
    end_s = interval["end_s"]
    duration_s = interval["duration_s"]
    collision = _collision_segments(timed, start_s, end_s)
    odom = _odom_events(timed)
    streams: dict[str, Any] = {}
    mission_streams: dict[str, Any] = {}
    motor_segments: list[dict[str, Any]] = []
    for stream in _COMMAND_STREAMS:
        samples = _command_events(timed, stream)
        if not samples:
            raise DriveTraceAnalysisError(f"trace has no {stream} commands")
        segments = _command_segments(
            samples,
            start_s=start_s,
            end_s=end_s,
            max_hold_s=0.50,
        )
        streams[stream] = {
            "sample_count": len(samples),
            "categories": _category_metrics(segments, duration_s),
            "jitter": _jitter_metrics(
                samples, segments, start_s=start_s, end_s=end_s
            ),
        }
        mission_segments = _command_segments(
            samples,
            start_s=mission_interval["start_s"],
            end_s=mission_interval["end_s"],
            max_hold_s=0.50,
        )
        mission_streams[stream] = {
            "categories": _category_metrics(
                mission_segments, mission_interval["duration_s"]
            ),
            "jitter": _jitter_metrics(
                samples,
                mission_segments,
                start_s=mission_interval["start_s"],
                end_s=mission_interval["end_s"],
            ),
        }
        if stream == "motor_output":
            motor_segments = segments
    overall_odom = _odom_metrics(odom, start_s, end_s + response_lag_s)
    goal = _goal_context(
        context,
        mission_id=mission_id,
        source_sha=source_sha,
        trace_sha256=trace_digest,
    )
    event_counts: dict[str, int] = {}
    for event in events:
        key = f"{event.get('kind', '')}:{event.get('stream', '')}".rstrip(":")
        event_counts[key] = event_counts.get(key, 0) + 1
    forward_windows = _forward_windows(
        motor_segments,
        collision,
        odom,
        response_lag_s=response_lag_s,
    )
    report = {
        "schema": ANALYSIS_SCHEMA,
        "mission_id": mission_id,
        "source_sha": source_sha,
        "trace": {
            "sha256": trace_digest,
            "size_bytes": size_bytes,
            "segment_count": len(_trace_paths(path)),
            "event_counts": dict(sorted(event_counts.items())),
        },
        "analysis_config": {
            "interval_rule": (
                "first nonzero nav2_request through first subsequent "
                "mission-bound hierarchical-controller complete or safe "
                "mission_lease_expired recovery terminal"
            ),
            "command_max_hold_s": 0.50,
            "collision_max_hold_s": 0.30,
            "linear_nonzero_threshold_mps": 0.001,
            "angular_nonzero_threshold_rad_s": 0.01,
            "angular_breakaway_floor_rad_s": 0.35,
            "odom_response_lag_s": response_lag_s,
        },
        "active_navigation_interval": interval,
        "active_navigation_command_streams": streams,
        "mission_interval": mission_interval,
        "mission_command_streams": mission_streams,
        "collision_state_duration_s": {
            state: sum(
                _duration(segment)
                for segment in collision
                if segment["state"] == state
            )
            for state in sorted({str(item["state"]) for item in collision})
        },
        "active_navigation_odometry": overall_odom,
        "motor_forward_windows": forward_windows,
        "goal_geometry_and_completion": goal,
        "phase1_motion_evidence": phase1_motion_evidence_routing(
            goal_geometry=goal,
            active_navigation_odometry=overall_odom,
            motor_forward_windows=forward_windows,
        ),
        "limitations": {
            "encoder_samples_available": False,
            "angular_jerk_is_sample_timing_sensitive": True,
            "yaw_per_metre_is_unstable_at_small_displacement": True,
            "breakaway_threshold_measured": False,
        },
    }
    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, nargs="+")
    parser.add_argument("--context", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        help="write the deterministic JSON report instead of printing it",
    )
    parser.add_argument("--odom-response-lag-s", type=float, default=0.25)
    args = parser.parse_args(argv)
    context = _load_json(args.context) if args.context else None
    report = analyze_trace(
        args.trace,
        context=context,
        response_lag_s=args.odom_response_lag_s,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
