"""Fail-closed Milestone 7 attended physical-validation evidence.

This module never grants motion authority.  Its ROS command is a bounded,
subscription-only observer for an already-approved supervised session.  The
ROS-free evaluator verifies the separately approved M7.3 collision and M7.4
moving-perception gates from compact evidence and generated graph/cleanup
audits.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from typing import Any, Callable, Mapping, Optional, Sequence

from .m7_surveyed_localization import (
    CLEANUP_AUDIT_SCHEMA,
    audit_stationary_cleanup,
)


SESSION_SCHEMA = "sphero_rvr.m7_phase3_attended_validation_session.v1"
REPORT_SCHEMA = "sphero_rvr.m7_phase3_attended_validation_report.v1"
OBSERVATION_SCHEMA = "sphero_rvr.m7_phase3_read_only_observation.v1"
GRAPH_AUDIT_SCHEMA = "sphero_rvr.m7_phase3_graph_audit.v1"
PLAN_SCHEMA = "sphero_rvr.m7_phase3_validation_plan.v1"
M7_3_EVIDENCE_SCHEMA = "sphero_rvr.m7_phase3_collision_evidence.v1"
ROS_GRAPH_DISCOVERY_SPIN_S = 3.0

SOURCE_SHA_RE = re.compile(r"^[0-9a-f]{40}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

EXPECTED_CAMERA_PITCH_RAD = -0.0523598775598299
MAX_CAMERA_PITCH_ERROR_RAD = math.radians(0.5)
MAX_CAMERA_PITCH_DRIFT_RAD = math.radians(0.5)
MAX_FAR_FLOOR_ERROR_M = 0.05
FAR_RANGE_MIN_M = 0.85
FAR_RANGE_MAX_M = 1.20

FIXED_SAFETY = {
    "stop_distance_m": 0.35,
    "slow_distance_m": 0.60,
    "release_distance_m": 0.45,
    "release_time_s": 0.50,
    "requested_cmd_timeout_s": 0.25,
    "maximum_veto_latency_s": 0.30,
    "maximum_stale_zero_latency_s": 0.35,
    "max_forward_mps": 0.10,
    "max_angular_rad_s": 0.40,
    "command_lease_max_s": 0.50,
    "max_scan_age_s": 0.30,
    "max_camera_age_s": 1.00,
    "max_localization_age_s": 0.30,
    "max_map_age_s": 1.00,
}

OBSERVED_TOPICS = {
    "collision_state": "/collision_stop/state",
    "collision_events": "/collision_stop/events",
    "requested_cmd": "/cmd_vel",
    "motor_cmd": "/cmd_vel_motor",
    "odom": "/odom",
    "scan": "/scan",
    "camera": "/mission_api/v2/camera/status",
    "lidar": "/mission_api/v2/lidar/status",
    "localization": "/mission_api/v2/localization/status",
    "semantic_map": "/mission_api/v2/map/status",
    "route_status": "/mission_api/v2/live_route/status",
    "tf": "/tf",
    "tf_static": "/tf_static",
}
M7_3_REQUIRED_OBSERVATION_TOPICS = {
    OBSERVED_TOPICS[name]
    for name in (
        "collision_state",
        "collision_events",
        "requested_cmd",
        "motor_cmd",
        "odom",
        "scan",
        "route_status",
    )
}
M7_4_REQUIRED_OBSERVATION_TOPICS = set(OBSERVED_TOPICS.values())
COLLISION_SAMPLE_TOPICS = {
    OBSERVED_TOPICS[name]
    for name in ("collision_state", "requested_cmd", "motor_cmd", "scan")
}
MOVING_SAMPLE_TOPICS = {
    OBSERVED_TOPICS[name]
    for name in (
        "collision_state",
        "requested_cmd",
        "motor_cmd",
        "odom",
        "scan",
        "camera",
        "lidar",
        "localization",
        "semantic_map",
        "tf",
        "tf_static",
    )
}

MOTOR_NODE_NAMES = {
    "/sphero_rvr_driver",
    "/lidar_collision_stop_supervisor",
    "/live_route_runner",
    "/range_motion_controller",
}

CLEANUP_MOTION_TOPICS = (
    "/cmd_vel",
    "/cmd_vel_motor",
    "/nav2_cmd_vel_request",
)
CLEANUP_DEVICES = (
    "/dev/rplidar",
    "/dev/ttyAMA0",
    "/dev/ttyS0",
    "/dev/serial0",
)
CLEANUP_PROCESS_TERMS = (
    "rplidar_composition",
    "camera_node",
    "slam_toolbox",
    "rvr_node",
    "live_route_runner",
    "lidar_collision_stop_supervisor",
    "ros2 bag record",
    "m7_phase3_read_only_observer",
)
CLEANUP_NODE_TERMS = (
    "rplidar",
    "camera_node",
    "slam_toolbox",
    "rvr_node",
    "live_route_runner",
    "collision_stop",
    "m7_phase3_read_only_observer",
)


class M7AttendedValidationError(ValueError):
    """Raised when attended-validation evidence is incomplete or unsafe."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise M7AttendedValidationError(message)


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise M7AttendedValidationError(f"{name} must be numeric") from exc
    _require(math.isfinite(result), f"{name} must be finite")
    return result


def _exact_sha(value: Any, name: str) -> str:
    result = str(value).strip().lower()
    _require(SOURCE_SHA_RE.fullmatch(result) is not None, f"{name} must be exact 40-hex")
    return result


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise M7AttendedValidationError(f"cannot read {path}: {exc}") from exc
    _require(isinstance(value, dict), f"{path} must contain a JSON object")
    return value


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as handle:
        json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")
        temporary = Path(handle.name)
    temporary.replace(path)


def build_plan(*, source_sha: str) -> dict[str, Any]:
    """Return the reviewed sequence without starting ROS or hardware."""

    source = _exact_sha(source_sha, "source_sha")
    return {
        "schema": PLAN_SCHEMA,
        "source_sha": source,
        "executes_commands": False,
        "motion_authority": False,
        "physical_execution_enabled": False,
        "warning": "WARNING: M7.3 and M7.4 can start the RVR motors",
        "room": {
            "attended": True,
            "level_bounded": True,
            "stairs_ledges_dropoffs_absent": True,
            "negative_obstacle_sensing_available": False,
        },
        "fixed_safety": dict(FIXED_SAFETY),
        "approval_boundaries": [
            {
                "gate": "m7.3",
                "required_before": "driver, serial transport, or motion graph startup",
                "approval_must_bind": [
                    "source_sha",
                    "deployed_sha",
                    "reviewed_sha",
                    "room",
                    "limits",
                    "operator",
                    "cleanup",
                ],
            },
            {
                "gate": "m7.4",
                "required_before": "moving-perception motion",
                "approval_must_bind": [
                    "source_sha",
                    "deployed_sha",
                    "reviewed_sha",
                    "room",
                    "limits",
                    "operator",
                    "m7_3_evidence_sha256",
                    "pitch_checks",
                    "cleanup",
                ],
            },
        ],
        "steps": [
            "run exact-SHA Mac and Pi bounded tests",
            "run Pi no-motion graph and device-owner preflight",
            "obtain independent review of the exact candidate",
            "obtain explicit M7.3 exact-SHA approval",
            "run attended collision slow/stop/manual-reset/no-contact trials",
            "prove restart enters recovery_required without resuming a route",
            "stop and complete generated cleanup audit",
            "review M7.3 evidence and obtain separate M7.4 approval",
            "re-verify camera pitch and far floor projection before handling",
            "run bounded attended moving-perception observation",
            "re-verify camera pitch and far floor projection after handling",
            "stop and complete generated cleanup audit",
        ],
        "observer": {
            "read_only": True,
            "topics": dict(OBSERVED_TOPICS),
            "publishers": [],
            "services": [],
            "serial_access": False,
        },
        "out_of_scope": [
            "M7.5 physical hierarchical binding",
            "canonical physical mission",
            "final M7.6 execution approval",
            "unattended motion",
            "stairs, ledges, drop-offs, and negative obstacles",
        ],
    }


def build_session_template(*, source_sha: str) -> dict[str, Any]:
    source = _exact_sha(source_sha, "source_sha")
    return {
        "schema": SESSION_SCHEMA,
        "description": "M7.3 collision plus M7.4 moving-perception evidence",
        "provenance": {
            "source_sha": source,
            "deployed_sha": source,
            "environment": {},
        },
        "fixed_safety": dict(FIXED_SAFETY),
        "authority": {
            "canonical_mission_approved": False,
            "m7_5_binding_approved": False,
            "approvals": [],
        },
        "m7_3_collision": {
            "graph_audit": {},
            "observation": {},
            "trials": {},
            "physical_contact_observed": None,
            "artifacts": [],
            "cleanup_audit": {},
        },
        "m7_4_moving_perception": {
            "graph_audit": {},
            "observation": {},
            "pitch_checks": {},
            "samples": [],
            "replan_events": [],
            "stale_veto": {},
            "artifacts": [],
            "cleanup_audit": {},
        },
    }


def m7_3_evidence_sha256(
    *,
    source_sha: str,
    environment: Mapping[str, Any],
    approval: Mapping[str, Any],
    collision: Mapping[str, Any],
) -> str:
    """Bind the accepted M7.3 evidence that a later M7.4 approval reviewed."""

    return _canonical_sha256(
        {
            "schema": M7_3_EVIDENCE_SCHEMA,
            "source_sha": _exact_sha(source_sha, "source_sha"),
            "environment": environment,
            "fixed_safety": FIXED_SAFETY,
            "approval": approval,
            "collision": collision,
        }
    )


def _validate_fixed_safety(value: Any) -> None:
    _require(isinstance(value, Mapping), "fixed_safety is required")
    _require(set(value) == set(FIXED_SAFETY), "fixed_safety keys are not exact")
    for name, expected in FIXED_SAFETY.items():
        actual = _finite(value[name], f"fixed_safety.{name}")
        _require(
            math.isclose(actual, expected, abs_tol=1e-12),
            f"fixed_safety.{name} cannot be changed by evidence",
        )


def _validate_approvals(
    authority: Any, *, source_sha: str, required_gates: set[str]
) -> dict[str, Mapping[str, Any]]:
    _require(isinstance(authority, Mapping), "authority is required")
    _require(
        authority.get("canonical_mission_approved") is False,
        "Phase 3 cannot approve the canonical mission",
    )
    _require(
        authority.get("m7_5_binding_approved") is False,
        "Phase 3 cannot approve M7.5 binding",
    )
    approvals = authority.get("approvals")
    _require(isinstance(approvals, list), "authority.approvals must be a list")
    by_gate: dict[str, Mapping[str, Any]] = {}
    approval_ids: set[str] = set()
    for approval in approvals:
        _require(isinstance(approval, Mapping), "approval must be an object")
        gate = str(approval.get("gate", ""))
        _require(gate in {"m7.3", "m7.4"}, "approval gate is unsupported")
        _require(gate not in by_gate, f"{gate} requires exactly one approval")
        for name in ("source_sha", "deployed_sha", "reviewed_sha"):
            _require(
                _exact_sha(approval.get(name), f"{gate}.{name}") == source_sha,
                f"{gate} approval does not bind the exact source",
            )
        approval_id = str(approval.get("approval_id", "")).strip()
        approved_by = str(approval.get("approved_by", "")).strip()
        approved_at = str(approval.get("approved_at_utc", "")).strip()
        _require(approval_id and approved_by and approved_at, f"{gate} approval identity is incomplete")
        _require(approval_id not in approval_ids, "M7.3 and M7.4 require separate approval IDs")
        approval_ids.add(approval_id)
        _require(
            approval.get("explicit_motor_warning_acknowledged") is True,
            f"{gate} must acknowledge the motor warning",
        )
        _require(
            approval.get("authority_owner") == "live_mission_service",
            f"{gate} approval must come from the existing authority owner",
        )
        for name in ("proposal_digest", "approval_event_sha256"):
            _require(
                SHA256_RE.fullmatch(str(approval.get(name, "")).strip().lower())
                is not None,
                f"{gate} {name} is invalid",
            )
        if gate == "m7.4":
            _require(
                SHA256_RE.fullmatch(
                    str(approval.get("m7_3_evidence_sha256", "")).strip().lower()
                )
                is not None,
                "m7.4 approval must bind the accepted M7.3 evidence",
            )
        room = approval.get("room")
        _require(isinstance(room, Mapping), f"{gate}.room is required")
        _require(room.get("attended") is True, f"{gate} must be attended")
        _require(room.get("level_bounded") is True, f"{gate} room must be level and bounded")
        _require(
            room.get("stairs_ledges_dropoffs_absent") is True,
            f"{gate} room cannot contain stairs, ledges, or drop-offs",
        )
        _require(
            room.get("negative_obstacle_sensing_available") is False,
            f"{gate} cannot claim unavailable drop-off sensing",
        )
        limits = approval.get("limits")
        _require(isinstance(limits, Mapping), f"{gate}.limits is required")
        for name in ("max_forward_mps", "max_angular_rad_s", "command_lease_max_s"):
            _require(
                math.isclose(
                    _finite(limits.get(name), f"{gate}.limits.{name}"),
                    FIXED_SAFETY[name],
                    abs_tol=1e-12,
                ),
                f"{gate} approval limits cannot widen {name}",
            )
        by_gate[gate] = approval
    _require(
        set(by_gate) == required_gates,
        "approval set does not match the requested sequential gate",
    )
    return by_gate


def _observation_sha256(value: Mapping[str, Any]) -> str:
    body = dict(value)
    body.pop("observation_sha256", None)
    return _canonical_sha256(body)


def _validate_observation(
    value: Any, *, source_sha: str, gate: str
) -> dict[str, str]:
    _require(isinstance(value, Mapping), f"{gate} observation is required")
    _require(
        value.get("schema") == OBSERVATION_SCHEMA,
        f"{gate} observation schema is invalid",
    )
    _require(
        _exact_sha(value.get("source_sha"), f"{gate}.observation.source_sha")
        == source_sha,
        f"{gate} observation source SHA mismatch",
    )
    _require(value.get("gate") == gate, f"{gate} observation gate is wrong")
    _require(value.get("read_only") is True, f"{gate} observation is not read-only")
    _require(
        value.get("motion_authority") is False,
        f"{gate} observation cannot claim motion authority",
    )
    _require(
        value.get("physical_execution_enabled") is False,
        f"{gate} observer cannot enable physical execution",
    )
    _require(
        value.get("topics") == OBSERVED_TOPICS,
        f"{gate} observation topic inventory is not exact",
    )
    _require(
        str(value.get("captured_at_utc", "")).strip(),
        f"{gate} observation capture time is required",
    )
    duration = _finite(value.get("duration_s"), f"{gate}.observation.duration_s")
    _require(
        0.5 <= duration <= 301.0,
        f"{gate} observation duration is outside the bounded capture window",
    )
    digest = str(value.get("observation_sha256", "")).strip().lower()
    _require(
        SHA256_RE.fullmatch(digest) is not None
        and digest == _observation_sha256(value),
        f"{gate} observation digest is invalid",
    )

    events = value.get("events")
    _require(isinstance(events, list) and events, f"{gate} observation events are required")
    event_topics_by_id: dict[str, str] = {}
    event_topics: set[str] = set()
    previous_elapsed = -math.inf
    for event in events:
        _require(isinstance(event, Mapping), f"{gate} observation event is invalid")
        event_id = str(event.get("event_id", "")).strip()
        _require(
            event_id and event_id not in event_topics_by_id,
            f"{gate} observation event IDs must be nonempty and unique",
        )
        topic = str(event.get("topic", "")).strip()
        _require(
            topic in OBSERVED_TOPICS.values(),
            f"{gate} observation contains an unsupported topic",
        )
        _finite(event.get("receipt_time_s"), f"{gate}.{event_id}.receipt_time_s")
        elapsed = _finite(event.get("elapsed_s"), f"{gate}.{event_id}.elapsed_s")
        _require(
            0.0 <= elapsed <= duration and elapsed >= previous_elapsed,
            f"{gate} observation event timing is invalid",
        )
        previous_elapsed = elapsed
        payload = event.get("payload")
        _require(
            str(event.get("payload_sha256", "")).strip().lower()
            == _canonical_sha256(payload),
            f"{gate} observation event payload digest is invalid",
        )
        event_topics_by_id[event_id] = topic
        event_topics.add(topic)
    required_topics = (
        M7_3_REQUIRED_OBSERVATION_TOPICS
        if gate == "m7.3"
        else M7_4_REQUIRED_OBSERVATION_TOPICS
    )
    _require(
        required_topics.issubset(event_topics),
        f"{gate} observation is missing required live topics",
    )
    return event_topics_by_id


def _require_event_ids(
    value: Any,
    *,
    label: str,
    observation_events: Mapping[str, str],
    required_topics: Optional[set[str]] = None,
) -> list[str]:
    _require(
        isinstance(value, list)
        and value
        and all(str(item).strip() for item in value),
        f"{label} must bind raw observer event IDs",
    )
    event_ids = [str(item).strip() for item in value]
    _require(
        set(event_ids).issubset(observation_events),
        f"{label} references an event outside the bound observation",
    )
    if required_topics:
        referenced_topics = {observation_events[event_id] for event_id in event_ids}
        _require(
            required_topics.issubset(referenced_topics),
            f"{label} does not bind every required source topic",
        )
    return event_ids


def _validate_artifacts(
    value: Any, *, label: str, observation_sha256: str
) -> None:
    _require(isinstance(value, list) and value, f"{label} artifacts are required")
    has_bag = False
    has_observation = False
    paths: set[str] = set()
    for item in value:
        _require(isinstance(item, Mapping), f"{label} artifact must be an object")
        path = str(item.get("path", "")).strip()
        digest = str(item.get("sha256", "")).strip().lower()
        _require(path and path not in paths, f"{label} artifact path is missing or duplicated")
        _require(SHA256_RE.fullmatch(digest) is not None, f"{label} artifact SHA-256 is invalid")
        _require(int(item.get("byte_count", 0)) > 0, f"{label} artifact byte_count must be positive")
        paths.add(path)
        has_bag = has_bag or path.endswith(".mcap")
        if path.endswith("observation.json"):
            has_observation = (
                str(item.get("canonical_sha256", "")).strip().lower()
                == observation_sha256
            )
    _require(has_bag, f"{label} requires a raw rosbag MCAP artifact")
    _require(
        has_observation,
        f"{label} observation artifact is not bound to the inline evidence",
    )


def _validate_graph_audit(
    value: Any, *, source_sha: str, gate: str
) -> None:
    _require(isinstance(value, Mapping), f"{gate} graph_audit is required")
    _require(value.get("schema") == GRAPH_AUDIT_SCHEMA, f"{gate} graph_audit schema is invalid")
    _require(value.get("passed") is True, f"{gate} graph_audit must pass")
    _require(value.get("stage") == "active", f"{gate} graph_audit must observe the active graph")
    _require(value.get("gate") == gate, f"{gate} graph_audit gate is wrong")
    _require(
        _exact_sha(value.get("source_sha"), f"{gate}.graph_audit.source_sha")
        == source_sha,
        f"{gate} graph_audit source SHA mismatch",
    )
    commands = value.get("commands")
    _require(isinstance(commands, Mapping), f"{gate} graph_audit commands are required")
    required_commands = {
        "git_head",
        "git_status",
        "ros_nodes",
        "topic_info:/cmd_vel",
        "topic_info:/cmd_vel_motor",
        "topic_info:/nav2_cmd_vel_request",
        "serial_owner",
    }
    _require(
        required_commands.issubset(commands),
        f"{gate} graph_audit command inventory is incomplete",
    )
    head = commands["git_head"]
    status = commands["git_status"]
    nodes = commands["ros_nodes"]
    cmd_vel = commands["topic_info:/cmd_vel"]
    motor = commands["topic_info:/cmd_vel_motor"]
    nav2 = commands["topic_info:/nav2_cmd_vel_request"]
    serial = commands["serial_owner"]
    for name, command in (
        ("git_head", head),
        ("git_status", status),
        ("ros_nodes", nodes),
        ("cmd_vel", cmd_vel),
        ("cmd_vel_motor", motor),
        ("nav2", nav2),
        ("serial_owner", serial),
    ):
        _require(isinstance(command, Mapping), f"{gate} {name} command is invalid")
    node_names = {
        line.strip()
        for line in str(nodes.get("stdout", "")).splitlines()
        if line.strip().startswith("/")
    }
    recomputed = {
        "exact_source_sha": head.get("returncode") == 0
        and str(head.get("stdout", "")).strip() == source_sha,
        "source_checkout_clean": status.get("returncode") == 0
        and not str(status.get("stdout", "")).strip(),
        "expected_nodes_present": {
            "/sphero_rvr_driver",
            "/lidar_collision_stop_supervisor",
            "/live_route_runner",
        }.issubset(node_names),
        "exclusive_cmd_vel_publisher": cmd_vel.get("returncode") == 0
        and _topic_count(str(cmd_vel.get("stdout", "")), "Publisher") == 1,
        "exclusive_motor_publisher": motor.get("returncode") == 0
        and _topic_count(str(motor.get("stdout", "")), "Publisher") == 1,
        "driver_is_motor_subscriber": motor.get("returncode") == 0
        and _topic_count(str(motor.get("stdout", "")), "Subscription") == 1,
        "correct_cmd_vel_owners": (
            _topic_endpoint_node_names(
                str(cmd_vel.get("stdout", "")), "Publisher"
            )
            == {"live_route_runner"}
            and _topic_endpoint_node_names(
                str(cmd_vel.get("stdout", "")), "Subscription"
            )
            == {"lidar_collision_stop_supervisor"}
        ),
        "correct_motor_owners": (
            _topic_endpoint_node_names(
                str(motor.get("stdout", "")), "Publisher"
            )
            == {"lidar_collision_stop_supervisor"}
            and _topic_endpoint_node_names(
                str(motor.get("stdout", "")), "Subscription"
            )
            == {"sphero_rvr_driver"}
        ),
        "nav2_private_publisher_absent_before_m7_5": (
            nav2.get("returncode") == 1
            or _topic_count(str(nav2.get("stdout", "")), "Publisher") == 0
        ),
        "serial_owner_present": serial.get("returncode") == 0
        and bool(
            (
                str(serial.get("stdout", ""))
                + str(serial.get("stderr", ""))
            ).strip()
        ),
    }
    checks = value.get("checks")
    _require(isinstance(checks, Mapping), f"{gate} graph_audit checks are required")
    for name, recomputed_value in recomputed.items():
        _require(
            recomputed_value is True and checks.get(name) is True,
            f"{gate} graph audit failed {name}",
        )


def _samples(
    value: Any,
    label: str,
    *,
    observation_events: Mapping[str, str],
    required_topics: set[str],
) -> list[Mapping[str, Any]]:
    _require(isinstance(value, list) and value, f"{label} samples are required")
    result: list[Mapping[str, Any]] = []
    previous = -math.inf
    for sample in value:
        _require(isinstance(sample, Mapping), f"{label} sample must be an object")
        stamp = _finite(sample.get("t_s"), f"{label}.t_s")
        _require(stamp >= previous, f"{label} samples must be ordered")
        previous = stamp
        for name in (
            "requested_linear_x",
            "requested_angular_z",
            "motor_linear_x",
            "motor_angular_z",
        ):
            _finite(sample.get(name), f"{label}.{name}")
        _require_event_ids(
            sample.get("evidence_event_ids"),
            label=f"{label} sample",
            observation_events=observation_events,
            required_topics=required_topics,
        )
        _require(sample.get("physical_contact") is False, f"{label} recorded physical contact")
        result.append(sample)
    return result


def _first_matching(
    samples: Sequence[Mapping[str, Any]],
    predicate: Callable[[Mapping[str, Any]], bool],
) -> Optional[Mapping[str, Any]]:
    return next((sample for sample in samples if predicate(sample)), None)


def _validate_collision(value: Any, *, source_sha: str) -> dict[str, Any]:
    _require(isinstance(value, Mapping), "m7_3_collision is required")
    _validate_graph_audit(value.get("graph_audit"), source_sha=source_sha, gate="m7.3")
    observation = value.get("observation")
    observation_events = _validate_observation(
        observation, source_sha=source_sha, gate="m7.3"
    )
    _validate_artifacts(
        value.get("artifacts"),
        label="m7.3",
        observation_sha256=str(observation["observation_sha256"]),
    )
    _require(
        value.get("physical_contact_observed") is False,
        "M7.3 requires an explicit no-contact observation",
    )
    trials = value.get("trials")
    _require(isinstance(trials, Mapping), "M7.3 trials are required")
    required_trials = {
        "slow",
        "collision_stop",
        "blocked_reset",
        "clear_reset",
        "stale_command",
        "operator_stop",
        "operator_estop",
        "restart_recovery",
    }
    _require(set(trials) == required_trials, "M7.3 trial set is not exact")

    slow = _samples(
        trials["slow"].get("samples"),
        "slow",
        observation_events=observation_events,
        required_topics=COLLISION_SAMPLE_TOPICS,
    )
    slow_sample = _first_matching(
        slow,
        lambda item: (
            str(item.get("state")) == "SLOW"
            and 0.0 < float(item["motor_linear_x"]) < float(item["requested_linear_x"])
            and FIXED_SAFETY["stop_distance_m"]
            < float(item.get("front_m"))
            < FIXED_SAFETY["slow_distance_m"]
        ),
    )
    _require(slow_sample is not None, "M7.3 did not prove physical SLOW scaling")

    stop = _samples(
        trials["collision_stop"].get("samples"),
        "collision_stop",
        observation_events=observation_events,
        required_topics=COLLISION_SAMPLE_TOPICS,
    )
    request = _first_matching(
        stop, lambda item: float(item["requested_linear_x"]) > 0.0
    )
    hazard = _first_matching(
        stop,
        lambda item: (
            request is not None
            and float(item["t_s"]) >= float(request["t_s"])
            and str(item.get("state")) == "STOPPED"
            and float(item.get("front_m")) <= FIXED_SAFETY["stop_distance_m"]
        ),
    )
    zero = _first_matching(
        stop,
        lambda item: (
            hazard is not None
            and float(item["t_s"]) >= float(hazard["t_s"])
            and str(item.get("state")) == "STOPPED"
            and abs(float(item["motor_linear_x"])) <= 1e-9
            and float(item.get("front_m")) <= FIXED_SAFETY["stop_distance_m"]
        ),
    )
    _require(
        request is not None and hazard is not None and zero is not None,
        "M7.3 did not prove collision STOP",
    )
    stop_latency = float(zero["t_s"]) - float(hazard["t_s"])
    _require(
        0.0 <= stop_latency <= FIXED_SAFETY["maximum_veto_latency_s"],
        "collision veto latency exceeded the fixed bound",
    )
    _require(
        trials["collision_stop"].get("provider_inference_in_flight") is True,
        "collision STOP must be observed while inference is in flight",
    )
    _require_event_ids(
        trials["collision_stop"].get("evidence_event_ids"),
        label="collision STOP",
        observation_events=observation_events,
        required_topics={
            OBSERVED_TOPICS["collision_events"],
            OBSERVED_TOPICS["route_status"],
        },
    )

    blocked_reset = trials["blocked_reset"]
    _require(blocked_reset.get("accepted") is False, "blocked reset must be rejected")
    _require_event_ids(
        blocked_reset.get("evidence_event_ids"),
        label="blocked reset",
        observation_events=observation_events,
        required_topics={
            OBSERVED_TOPICS["collision_state"],
            OBSERVED_TOPICS["collision_events"],
        },
    )
    _require(str(blocked_reset.get("state")) == "STOPPED", "blocked reset must remain STOPPED")
    _require(
        _finite(blocked_reset.get("front_m"), "blocked_reset.front_m")
        < FIXED_SAFETY["release_distance_m"],
        "blocked reset evidence is not inside the release threshold",
    )

    clear_reset = trials["clear_reset"]
    _require(clear_reset.get("accepted") is True, "clear manual reset must be accepted")
    _require_event_ids(
        clear_reset.get("evidence_event_ids"),
        label="clear reset",
        observation_events=observation_events,
        required_topics={
            OBSERVED_TOPICS["collision_state"],
            OBSERVED_TOPICS["collision_events"],
        },
    )
    _require(
        _finite(clear_reset.get("clear_duration_s"), "clear_reset.clear_duration_s")
        >= FIXED_SAFETY["release_time_s"],
        "clear reset did not preserve release hysteresis",
    )
    _require(
        _finite(clear_reset.get("front_m"), "clear_reset.front_m")
        >= FIXED_SAFETY["release_distance_m"],
        "clear reset did not exceed the release distance",
    )
    post_reset = _samples(
        clear_reset.get("samples_after_reset"),
        "clear_reset",
        observation_events=observation_events,
        required_topics=COLLISION_SAMPLE_TOPICS,
    )
    _require(
        all(
            abs(float(item["motor_linear_x"])) <= 1e-9
            and abs(float(item["motor_angular_z"])) <= 1e-9
            for item in post_reset
        ),
        "manual reset replayed an old command",
    )

    stale = trials["stale_command"]
    stale_latency = _finite(stale.get("zero_latency_s"), "stale_command.zero_latency_s")
    _require(
        0.0 <= stale_latency <= FIXED_SAFETY["maximum_stale_zero_latency_s"],
        "stale command zero exceeded the fixed bound",
    )
    _require(stale.get("motor_zero") is True, "stale command did not produce motor zero")
    _require_event_ids(
        stale.get("evidence_event_ids"),
        label="stale command",
        observation_events=observation_events,
        required_topics={
            OBSERVED_TOPICS["requested_cmd"],
            OBSERVED_TOPICS["motor_cmd"],
            OBSERVED_TOPICS["route_status"],
        },
    )
    _require(
        _finite(stale.get("driver_watchdog_s"), "stale_command.driver_watchdog_s")
        >= 0.5,
        "stale command evidence does not preserve the driver backstop",
    )
    _require(
        stale.get("provider_inference_in_flight") is True,
        "stale-command veto must be observed while inference is in flight",
    )

    service_latencies: dict[str, float] = {}
    for name in ("operator_stop", "operator_estop"):
        event = trials[name]
        latency = _finite(event.get("zero_latency_s"), f"{name}.zero_latency_s")
        _require(
            0.0 <= latency <= FIXED_SAFETY["maximum_veto_latency_s"],
            f"{name} zero latency exceeded the fixed bound",
        )
        _require(event.get("motor_zero") is True, f"{name} did not produce motor zero")
        _require_event_ids(
            event.get("evidence_event_ids"),
            label=name,
            observation_events=observation_events,
            required_topics={
                OBSERVED_TOPICS["collision_events"],
                OBSERVED_TOPICS["motor_cmd"],
                OBSERVED_TOPICS["route_status"],
            },
        )
        _require(
            event.get("provider_inference_in_flight") is True,
            f"{name} must be observed while inference is in flight",
        )
        service_latencies[name] = latency
    _require(
        trials["operator_estop"].get("latched_until_explicit_clear") is True,
        "ESTOP was not proven latched",
    )

    restart = trials["restart_recovery"]
    _require(
        restart.get("pre_restart_state") == "RUNNING",
        "restart evidence must begin from an active mission",
    )
    _require(
        restart.get("post_restart_state") == "recovery_required",
        "restart must enter recovery_required",
    )
    _require(
        restart.get("motor_zero") is True,
        "restart recovery did not preserve motor zero",
    )
    _require(
        restart.get("route_resumed") is False,
        "restart cannot resume the previous route",
    )
    _require_event_ids(
        restart.get("evidence_event_ids"),
        label="restart recovery",
        observation_events=observation_events,
        required_topics={
            OBSERVED_TOPICS["motor_cmd"],
            OBSERVED_TOPICS["route_status"],
        },
    )

    _validate_cleanup(value.get("cleanup_audit"), source_sha=source_sha, label="m7.3")
    return {
        "slow_min_motor_mps": min(float(item["motor_linear_x"]) for item in slow if float(item["motor_linear_x"]) > 0.0),
        "collision_zero_latency_s": stop_latency,
        "operator_zero_latency_s": service_latencies,
        "physical_contact_observed": False,
        "restart_state": "recovery_required",
    }


def _validate_pitch_check(value: Any, *, label: str) -> dict[str, float]:
    _require(isinstance(value, Mapping), f"{label} pitch check is required")
    _require(
        value.get("method") == "surveyed_floor_contact_sweep",
        f"{label} pitch check must use the surveyed floor contact sweep",
    )
    pitch = _finite(value.get("camera_pitch_rad"), f"{label}.camera_pitch_rad")
    error = _finite(value.get("far_floor_error_m"), f"{label}.far_floor_error_m")
    target_range = _finite(value.get("target_range_m"), f"{label}.target_range_m")
    _require(
        abs(pitch - EXPECTED_CAMERA_PITCH_RAD) <= MAX_CAMERA_PITCH_ERROR_RAD,
        f"{label} camera pitch is outside the reviewed -3 degree band",
    )
    _require(
        FAR_RANGE_MIN_M <= target_range <= FAR_RANGE_MAX_M,
        f"{label} pitch check must use the far range band",
    )
    _require(
        0.0 <= error <= MAX_FAR_FLOOR_ERROR_M,
        f"{label} far floor projection exceeded the unchanged 0.05 m bound",
    )
    _require(
        value.get("tolerance_widened") is False,
        f"{label} cannot widen the floor-projection tolerance",
    )
    artifact = str(value.get("artifact_sha256", "")).strip().lower()
    _require(SHA256_RE.fullmatch(artifact) is not None, f"{label} pitch artifact SHA-256 is invalid")
    return {"pitch_rad": pitch, "far_floor_error_m": error, "target_range_m": target_range}


def _validate_localized_detection(
    value: Any, *, observation_events: Mapping[str, str]
) -> None:
    _require(isinstance(value, Mapping), "localized detection must be an object")
    method = str(value.get("method", ""))
    _require(
        method in {"lidar_range", "floor_projection", "bearing_only"},
        "localized detection method is unsupported",
    )
    point = value.get("point")
    if method == "bearing_only":
        _require(point is None, "bearing-only moving evidence cannot contain a point")
    else:
        _require(isinstance(point, Mapping), "point-producing moving evidence requires a point")
        _finite(point.get("x"), "detection.point.x")
        _finite(point.get("y"), "detection.point.y")
        _require(point.get("frame") == "map", "mapped detection point must use map frame")
    uncertainty = value.get("uncertainty")
    _require(isinstance(uncertainty, Mapping), "mapped detection uncertainty is required")
    _finite(uncertainty.get("bearing_sigma_rad"), "detection.bearing_sigma_rad")
    if method != "bearing_only":
        _finite(uncertainty.get("position_sigma_m"), "detection.position_sigma_m")
    _require_event_ids(
        value.get("evidence_ids"),
        label="mapped detection",
        observation_events=observation_events,
        required_topics={
            OBSERVED_TOPICS["camera"],
            OBSERVED_TOPICS["scan"],
            OBSERVED_TOPICS["localization"],
        },
    )
    timestamps = value.get("source_timestamps_ns")
    _require(isinstance(timestamps, Mapping) and "image" in timestamps, "mapped detection source timestamps are required")
    _require(str(value.get("calibration_id", "")).strip(), "mapped detection calibration ID is required")
    _require(str(value.get("map_revision", "")).strip(), "mapped detection map revision is required")


def _validate_moving_perception(value: Any, *, source_sha: str) -> dict[str, Any]:
    _require(isinstance(value, Mapping), "m7_4_moving_perception is required")
    _validate_graph_audit(value.get("graph_audit"), source_sha=source_sha, gate="m7.4")
    observation = value.get("observation")
    observation_events = _validate_observation(
        observation, source_sha=source_sha, gate="m7.4"
    )
    _validate_artifacts(
        value.get("artifacts"),
        label="m7.4",
        observation_sha256=str(observation["observation_sha256"]),
    )
    pitch_checks = value.get("pitch_checks")
    _require(isinstance(pitch_checks, Mapping), "M7.4 pitch_checks are required")
    before = _validate_pitch_check(pitch_checks.get("before"), label="before")
    after = _validate_pitch_check(pitch_checks.get("after"), label="after")
    pitch_drift = abs(after["pitch_rad"] - before["pitch_rad"])
    _require(
        pitch_drift <= MAX_CAMERA_PITCH_DRIFT_RAD,
        "camera pitch drift exceeded the reviewed handling bound",
    )

    samples = _samples(
        value.get("samples"),
        "moving_perception",
        observation_events=observation_events,
        required_topics=MOVING_SAMPLE_TOPICS,
    )
    _require(len(samples) >= 5, "M7.4 requires at least five ordered moving samples")
    moving_indices = {
        index
        for index, sample in enumerate(samples)
        if abs(float(sample["motor_linear_x"])) > 1e-6
        or abs(float(sample["motor_angular_z"])) > 1e-6
    }
    _require(
        len(moving_indices) >= 2,
        "M7.4 requires live perception during physical motion",
    )
    revisions: set[str] = set()
    poses: list[tuple[float, float]] = []
    point_detection_count = 0
    track_ids_by_sample: list[set[str]] = []
    for index, sample in enumerate(samples):
        freshness = sample.get("freshness_s")
        _require(isinstance(freshness, Mapping), "moving sample freshness is required")
        limits = {
            "lidar": FIXED_SAFETY["max_scan_age_s"],
            "camera": FIXED_SAFETY["max_camera_age_s"],
            "localization": FIXED_SAFETY["max_localization_age_s"],
            "map": FIXED_SAFETY["max_map_age_s"],
        }
        if index in moving_indices:
            for name, limit in limits.items():
                age = _finite(freshness.get(name), f"moving freshness {name}")
                _require(0.0 <= age <= limit, f"motion occurred with stale {name} evidence")
        transforms = sample.get("transforms")
        _require(isinstance(transforms, Mapping), "moving sample transforms are required")
        for name in ("map_to_base_link", "base_link_to_camera", "base_link_to_lidar"):
            _require(transforms.get(name) is True, f"moving sample missing {name}")
        localization = sample.get("localization")
        _require(isinstance(localization, Mapping), "moving localization is required")
        _require(localization.get("state") == "valid", "moving localization must be valid")
        _require(
            str(localization.get("source", "")).startswith("slam"),
            "moving localization must remain lidar/SLAM authoritative",
        )
        pose = localization.get("pose")
        _require(isinstance(pose, Mapping), "moving localization pose is required")
        poses.append(
            (
                _finite(pose.get("x"), "moving pose.x"),
                _finite(pose.get("y"), "moving pose.y"),
            )
        )
        _require(sample.get("camera_calibrated") is True, "moving camera must be calibrated")
        revision = str(sample.get("map_revision", "")).strip()
        _require(revision, "moving sample map revision is required")
        revisions.add(revision)
        detections = sample.get("localized_detections")
        _require(isinstance(detections, list), "localized_detections must be a list")
        for detection in detections:
            _validate_localized_detection(
                detection,
                observation_events=observation_events,
            )
            if detection.get("method") != "bearing_only":
                point_detection_count += 1
        tracks = sample.get("tracks")
        _require(isinstance(tracks, list), "moving tracks must be a list")
        track_ids_by_sample.append(
            {
                str(track.get("track_id"))
                for track in tracks
                if isinstance(track, Mapping)
                and str(track.get("track_id", "")).strip()
                and int(track.get("observation_count", 0)) >= 2
            }
        )
    _require(len(revisions) >= 2, "M7.4 must observe changing map revisions")
    displacement = max(
        math.hypot(x - poses[0][0], y - poses[0][1]) for x, y in poses
    )
    _require(displacement >= 0.05, "M7.4 did not prove moving localization")
    _require(point_detection_count >= 1, "M7.4 requires at least one mapped point detection")

    initial_tracks = track_ids_by_sample[0]
    new_tracks: set[str] = set()
    for index, track_ids in enumerate(track_ids_by_sample[1:], start=1):
        if index in moving_indices:
            new_tracks.update(track_ids - initial_tracks)
    _require(new_tracks, "M7.4 did not produce a new stable track while moving")
    replan_events = value.get("replan_events")
    _require(isinstance(replan_events, list) and replan_events, "M7.4 replan event is required")
    matched_replan = any(
        isinstance(event, Mapping)
        and event.get("trigger") == "new_stable_detection"
        and str(event.get("track_id")) in new_tracks
        and event.get("replan_required") is True
        and event.get("contains_motion_geometry") is False
        and str(event.get("map_revision", "")).strip()
        and isinstance(event.get("evidence_event_ids"), list)
        and bool(event["evidence_event_ids"])
        for event in replan_events
    )
    _require(matched_replan, "M7.4 replan evidence is not bound to a new stable track")
    for event in replan_events:
        if (
            isinstance(event, Mapping)
            and event.get("trigger") == "new_stable_detection"
            and str(event.get("track_id")) in new_tracks
        ):
            _require_event_ids(
                event.get("evidence_event_ids"),
                label="M7.4 replan",
                observation_events=observation_events,
                required_topics={
                    OBSERVED_TOPICS["semantic_map"],
                    OBSERVED_TOPICS["route_status"],
                },
            )

    stale = value.get("stale_veto")
    _require(isinstance(stale, Mapping), "M7.4 stale_veto is required")
    _require(
        str(stale.get("source")) in {"lidar", "localization", "camera", "map"},
        "M7.4 stale veto source is unsupported",
    )
    stale_topic_key = (
        "semantic_map" if str(stale.get("source")) == "map" else str(stale.get("source"))
    )
    _require(stale.get("motor_zero") is True, "M7.4 stale evidence did not force motor zero")
    _require_event_ids(
        stale.get("evidence_event_ids"),
        label="M7.4 stale veto",
        observation_events=observation_events,
        required_topics={
            OBSERVED_TOPICS[stale_topic_key],
            OBSERVED_TOPICS["motor_cmd"],
            OBSERVED_TOPICS["route_status"],
        },
    )
    stale_latency = _finite(stale.get("zero_latency_s"), "stale_veto.zero_latency_s")
    _require(
        0.0 <= stale_latency <= FIXED_SAFETY["maximum_veto_latency_s"],
        "M7.4 stale-evidence zero exceeded the fixed bound",
    )
    _require(
        stale.get("provider_inference_in_flight") is True,
        "M7.4 stale veto must remain independent of inference",
    )

    _validate_cleanup(value.get("cleanup_audit"), source_sha=source_sha, label="m7.4")
    return {
        "sample_count": len(samples),
        "moving_sample_count": len(moving_indices),
        "maximum_observed_displacement_m": displacement,
        "mapped_point_detection_count": point_detection_count,
        "new_stable_track_ids": sorted(new_tracks),
        "pitch_drift_deg": math.degrees(pitch_drift),
        "stale_zero_latency_s": stale_latency,
    }


def _validate_cleanup(value: Any, *, source_sha: str, label: str) -> None:
    _require(isinstance(value, Mapping), f"{label} cleanup audit is required")
    _require(
        value.get("schema") == CLEANUP_AUDIT_SCHEMA,
        f"{label} cleanup audit schema is invalid",
    )
    _require(value.get("passed") is True, f"{label} cleanup audit must pass")
    _require(
        _exact_sha(value.get("source_sha"), f"{label}.cleanup.source_sha")
        == source_sha,
        f"{label} cleanup source SHA mismatch",
    )
    commands = value.get("commands")
    _require(isinstance(commands, Mapping), f"{label} cleanup commands are required")
    required_commands = {
        "git_head",
        "git_status",
        "processes",
        "ros_nodes",
        "ros_topics",
        *(f"topic_info:{topic}" for topic in CLEANUP_MOTION_TOPICS),
        *(f"device_owner:{device}" for device in CLEANUP_DEVICES),
    }
    _require(
        required_commands.issubset(commands),
        f"{label} cleanup command inventory is incomplete",
    )
    for name in required_commands:
        _require(
            isinstance(commands[name], Mapping),
            f"{label} cleanup command {name} is invalid",
        )

    head = commands["git_head"]
    status = commands["git_status"]
    processes = commands["processes"]
    nodes = commands["ros_nodes"]
    topics = commands["ros_topics"]
    process_lines = [
        line.strip()
        for line in str(processes.get("stdout", "")).splitlines()
        if line.strip()
    ]
    node_lines = [
        line.strip()
        for line in str(nodes.get("stdout", "")).splitlines()
        if line.strip()
    ]
    prohibited_processes = [
        line
        for line in process_lines
        if any(term in line.lower() for term in CLEANUP_PROCESS_TERMS)
    ]
    prohibited_nodes = [
        line
        for line in node_lines
        if any(term in line.lower() for term in CLEANUP_NODE_TERMS)
    ]
    motion_publishers: dict[str, Optional[int]] = {}
    for topic in CLEANUP_MOTION_TOPICS:
        command = commands[f"topic_info:{topic}"]
        if command.get("returncode") == 0:
            motion_publishers[topic] = _topic_count(
                str(command.get("stdout", "")), "Publisher"
            )
        elif command.get("returncode") == 1:
            motion_publishers[topic] = 0
        else:
            motion_publishers[topic] = None
    device_owners = {
        device: (
            (
                str(commands[f"device_owner:{device}"].get("stdout", ""))
                + str(commands[f"device_owner:{device}"].get("stderr", ""))
            ).strip()
            if commands[f"device_owner:{device}"].get("returncode") == 0
            else ""
        )
        for device in CLEANUP_DEVICES
    }
    device_inspection_valid = all(
        commands[f"device_owner:{device}"].get("returncode") in {0, 1}
        for device in CLEANUP_DEVICES
    )
    recomputed = {
        "exact_source_sha": head.get("returncode") == 0
        and str(head.get("stdout", "")).strip() == source_sha,
        "source_checkout_clean": status.get("returncode") == 0
        and not str(status.get("stdout", "")).strip(),
        "process_inspection_succeeded": processes.get("returncode") == 0,
        "stationary_sensor_and_motion_processes_absent": not prohibited_processes,
        "ros_node_inspection_succeeded": nodes.get("returncode") == 0,
        "prohibited_ros_nodes_absent": not prohibited_nodes,
        "ros_topic_inspection_succeeded": topics.get("returncode") == 0,
        "motion_topic_publishers_absent": all(
            count == 0 for count in motion_publishers.values()
        ),
        "device_owner_inspection_succeeded": device_inspection_valid,
        "sensor_and_rover_devices_ownerless": all(
            not owner for owner in device_owners.values()
        ),
    }
    checks = value.get("checks")
    _require(isinstance(checks, Mapping), f"{label} cleanup checks are required")
    for name, recomputed_value in recomputed.items():
        _require(
            recomputed_value is True and checks.get(name) is True,
            f"{label} cleanup failed {name}",
        )

    cleanup = value.get("cleanup")
    _require(isinstance(cleanup, Mapping), f"{label} cleanup state is required")
    recomputed_cleanup = {
        "camera_stopped": processes.get("returncode") == 0
        and not any("camera_node" in line.lower() for line in process_lines),
        "lidar_stopped": processes.get("returncode") == 0
        and not any("rplidar" in line.lower() for line in process_lines)
        and not device_owners["/dev/rplidar"],
        "rosbag_stopped": processes.get("returncode") == 0
        and not any("ros2 bag record" in line.lower() for line in process_lines),
        "prohibited_nodes_absent": nodes.get("returncode") == 0
        and not prohibited_nodes
        and all(count == 0 for count in motion_publishers.values()),
        "rover_serial_owner_absent": device_inspection_valid
        and all(
            not device_owners[device]
            for device in ("/dev/ttyAMA0", "/dev/ttyS0", "/dev/serial0")
        ),
        "completed": all(recomputed.values()),
    }
    _require(
        cleanup == recomputed_cleanup,
        f"{label} cleanup state is not derived from raw inspections",
    )


def evaluate_session(
    session: Mapping[str, Any], *, through_gate: str = "m7.4"
) -> dict[str, Any]:
    """Evaluate M7.3 alone or the complete sequential M7.3 + M7.4 session."""

    session_hash = _canonical_sha256(session)
    try:
        _require(
            through_gate in {"m7.3", "m7.4"},
            "through_gate must be m7.3 or m7.4",
        )
        _require(session.get("schema") == SESSION_SCHEMA, "session schema is invalid")
        provenance = session.get("provenance")
        _require(isinstance(provenance, Mapping), "provenance is required")
        source_sha = _exact_sha(provenance.get("source_sha"), "provenance.source_sha")
        _require(
            _exact_sha(provenance.get("deployed_sha"), "provenance.deployed_sha")
            == source_sha,
            "source and deployed SHAs must match",
        )
        environment = provenance.get("environment")
        _require(isinstance(environment, Mapping), "physical environment is required")
        for name in ("hostname", "platform", "ros_distro", "python_version", "operator"):
            _require(str(environment.get(name, "")).strip(), f"environment.{name} is required")
        _validate_fixed_safety(session.get("fixed_safety"))
        required_gates = (
            {"m7.3"} if through_gate == "m7.3" else {"m7.3", "m7.4"}
        )
        approvals = _validate_approvals(
            session.get("authority"),
            source_sha=source_sha,
            required_gates=required_gates,
        )
        collision_metrics = _validate_collision(
            session.get("m7_3_collision"), source_sha=source_sha
        )
        collision_evidence_sha = m7_3_evidence_sha256(
            source_sha=source_sha,
            environment=environment,
            approval=approvals["m7.3"],
            collision=session.get("m7_3_collision"),
        )
        perception_metrics = None
        if through_gate == "m7.4":
            _require(
                str(
                    approvals["m7.4"].get("m7_3_evidence_sha256", "")
                ).strip().lower()
                == collision_evidence_sha,
                "m7.4 approval does not bind the accepted M7.3 evidence",
            )
            perception_metrics = _validate_moving_perception(
                session.get("m7_4_moving_perception"), source_sha=source_sha
            )
        checks = {
            "exact_source_and_required_approvals": True,
            "unchanged_safety_limits": True,
            "m7_3_attended_collision_passed": True,
            "m7_3_no_contact_and_cleanup_passed": True,
            "canonical_mission_and_m7_5_remain_locked": True,
        }
        if through_gate == "m7.4":
            checks.update(
                {
                    "m7_4_approval_binds_m7_3_evidence": True,
                    "m7_4_moving_perception_passed": True,
                    "m7_4_pitch_reverified_before_and_after": True,
                    "m7_4_stale_veto_and_cleanup_passed": True,
                }
            )
        return {
            "schema": REPORT_SCHEMA,
            "session_sha256": session_hash,
            "source_sha": source_sha,
            "through_gate": through_gate,
            "passed": True,
            "checks": checks,
            "m7_3_evidence_sha256": collision_evidence_sha,
            "m7_3_metrics": collision_metrics,
            "m7_4_metrics": perception_metrics,
            "scope": {
                "m7_3_collision_gate": "passed",
                "m7_4_moving_perception_gate": (
                    "passed" if through_gate == "m7.4" else "not_proven"
                ),
                "m7_5_physical_binding_approved": False,
                "canonical_mission_approved": False,
                "drop_off_detection_available": False,
            },
        }
    except (M7AttendedValidationError, KeyError, TypeError, ValueError) as exc:
        return {
            "schema": REPORT_SCHEMA,
            "session_sha256": session_hash,
            "through_gate": through_gate,
            "passed": False,
            "error": str(exc),
            "checks": {},
            "scope": {
                "m7_3_collision_gate": "not_proven",
                "m7_4_moving_perception_gate": "not_proven",
                "m7_5_physical_binding_approved": False,
                "canonical_mission_approved": False,
                "drop_off_detection_available": False,
            },
        }


def _run_command(argv: Sequence[str], *, timeout_s: float = 8.0) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            list(argv),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        return {
            "argv": list(argv),
            "returncode": int(completed.returncode),
            "stdout": completed.stdout,
            "stderr": completed.stderr,
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "argv": list(argv),
            "returncode": 124 if isinstance(exc, subprocess.TimeoutExpired) else 127,
            "stdout": "",
            "stderr": str(exc),
        }


def _topic_count(output: str, kind: str) -> Optional[int]:
    match = re.search(rf"^{re.escape(kind)} count:\s*(\d+)\s*$", output, re.MULTILINE)
    return None if match is None else int(match.group(1))


def _topic_node_names(output: str) -> set[str]:
    return {
        match.group(1).strip().lstrip("/")
        for match in re.finditer(r"^Node name:\s*(\S+)\s*$", output, re.MULTILINE)
    }


def _topic_endpoint_node_names(output: str, kind: str) -> set[str]:
    _require(kind in {"Publisher", "Subscription"}, "topic endpoint kind is invalid")
    start = output.find(f"{kind} count:")
    if start < 0:
        return set()
    other = "Subscription" if kind == "Publisher" else "Publisher"
    end = output.find(f"{other} count:", start + 1)
    section = output[start:] if end < 0 else output[start:end]
    return _topic_node_names(section)


def generate_graph_audit(
    *,
    source_sha: str,
    source_repo: Path,
    gate: str,
    stage: str,
    command_runner: Callable[..., Mapping[str, Any]] = _run_command,
) -> dict[str, Any]:
    """Inspect an absent preflight graph or active supervised ownership graph."""

    source = _exact_sha(source_sha, "source_sha")
    _require(gate in {"m7.3", "m7.4"}, "graph audit gate is unsupported")
    _require(stage in {"preflight", "active"}, "graph audit stage is unsupported")
    commands: dict[str, Mapping[str, Any]] = {}

    def run(name: str, argv: Sequence[str]) -> Mapping[str, Any]:
        result = command_runner(list(argv), timeout_s=8.0)
        commands[name] = result
        return result

    head = run("git_head", ["git", "-C", str(source_repo), "rev-parse", "HEAD"])
    status = run("git_status", ["git", "-C", str(source_repo), "status", "--porcelain"])
    discovery_spin = str(ROS_GRAPH_DISCOVERY_SPIN_S)
    nodes = run(
        "ros_nodes",
        [
            "ros2",
            "node",
            "list",
            "--spin-time",
            discovery_spin,
            "--no-daemon",
        ],
    )
    topic_results = {
        topic: run(
            f"topic_info:{topic}",
            [
                "ros2",
                "topic",
                "info",
                "-v",
                "--spin-time",
                discovery_spin,
                "--no-daemon",
                topic,
            ],
        )
        for topic in ("/cmd_vel", "/cmd_vel_motor", "/nav2_cmd_vel_request")
    }
    serial = run("serial_owner", ["fuser", "/dev/ttyAMA0"])

    node_names = {
        line.strip()
        for line in str(nodes.get("stdout", "")).splitlines()
        if line.strip().startswith("/")
    }
    cmd_vel_publishers = _topic_count(
        str(topic_results["/cmd_vel"].get("stdout", "")), "Publisher"
    )
    motor_publishers = _topic_count(
        str(topic_results["/cmd_vel_motor"].get("stdout", "")), "Publisher"
    )
    motor_subscribers = _topic_count(
        str(topic_results["/cmd_vel_motor"].get("stdout", "")), "Subscription"
    )
    nav2_publishers = _topic_count(
        str(topic_results["/nav2_cmd_vel_request"].get("stdout", "")), "Publisher"
    )

    common = {
        "exact_source_sha": head.get("returncode") == 0
        and str(head.get("stdout", "")).strip() == source,
        "source_checkout_clean": status.get("returncode") == 0
        and not str(status.get("stdout", "")).strip(),
        "ros_node_inspection_succeeded": nodes.get("returncode") == 0,
        "serial_inspection_succeeded": serial.get("returncode") in {0, 1},
    }
    if stage == "preflight":
        checks = {
            **common,
            "motor_nodes_absent": not (node_names & MOTOR_NODE_NAMES),
            "motion_publishers_absent": all(
                count in {None, 0}
                for count in (
                    cmd_vel_publishers,
                    motor_publishers,
                    nav2_publishers,
                )
            ),
            "serial_owner_absent": serial.get("returncode") == 1
            and not str(serial.get("stdout", "")).strip(),
        }
    else:
        checks = {
            **common,
            "expected_nodes_present": {
                "/sphero_rvr_driver",
                "/lidar_collision_stop_supervisor",
                "/live_route_runner",
            }.issubset(node_names),
            "exclusive_cmd_vel_publisher": cmd_vel_publishers == 1,
            "exclusive_motor_publisher": motor_publishers == 1,
            "driver_is_motor_subscriber": motor_subscribers == 1,
            "correct_cmd_vel_owners": (
                _topic_endpoint_node_names(
                    str(topic_results["/cmd_vel"].get("stdout", "")),
                    "Publisher",
                )
                == {"live_route_runner"}
                and _topic_endpoint_node_names(
                    str(topic_results["/cmd_vel"].get("stdout", "")),
                    "Subscription",
                )
                == {"lidar_collision_stop_supervisor"}
            ),
            "correct_motor_owners": (
                _topic_endpoint_node_names(
                    str(topic_results["/cmd_vel_motor"].get("stdout", "")),
                    "Publisher",
                )
                == {"lidar_collision_stop_supervisor"}
                and _topic_endpoint_node_names(
                    str(topic_results["/cmd_vel_motor"].get("stdout", "")),
                    "Subscription",
                )
                == {"sphero_rvr_driver"}
            ),
            "nav2_private_publisher_absent_before_m7_5": nav2_publishers in {None, 0},
            "serial_owner_present": serial.get("returncode") == 0
            and bool(
                (
                    str(serial.get("stdout", ""))
                    + str(serial.get("stderr", ""))
                ).strip()
            ),
        }
    return {
        "schema": GRAPH_AUDIT_SCHEMA,
        "source_sha": source,
        "gate": gate,
        "stage": stage,
        "audited_at_utc": datetime.now(timezone.utc).isoformat(),
        "passed": all(checks.values()),
        "checks": checks,
        "findings": {
            "nodes": sorted(node_names),
            "publisher_counts": {
                "/cmd_vel": cmd_vel_publishers,
                "/cmd_vel_motor": motor_publishers,
                "/nav2_cmd_vel_request": nav2_publishers,
            },
            "motor_subscriber_count": motor_subscribers,
            "serial_owner": (
                str(serial.get("stdout", ""))
                + str(serial.get("stderr", ""))
            ).strip(),
        },
        "commands": commands,
        "motion_authority": False,
        "note": (
            "read-only audit; an active result observes authority but cannot grant it"
        ),
    }


def capture_ros_observation(
    *,
    source_sha: str,
    gate: str,
    duration_s: float,
) -> dict[str, Any]:
    """Capture bounded ROS topic evidence without publishers, services, or serial."""

    source = _exact_sha(source_sha, "source_sha")
    _require(gate in {"m7.3", "m7.4"}, "observation gate is unsupported")
    duration = _finite(duration_s, "duration_s")
    _require(0.5 <= duration <= 300.0, "duration_s must be between 0.5 and 300")
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:  # pragma: no cover - ROS runtime only
        raise M7AttendedValidationError(
            "observe requires a sourced ROS 2 Jazzy environment"
        ) from exc

    rclpy.init(args=None)
    node = rclpy.create_node("m7_phase3_read_only_observer")
    events: list[dict[str, Any]] = []
    started_wall = time.time()
    started_mono = time.monotonic()

    def append(topic_key: str, payload: Any) -> None:
        event_id = f"{gate}-event-{len(events) + 1:06d}"
        safe_payload = _json_safe(payload)
        events.append(
            {
                "event_id": event_id,
                "topic": OBSERVED_TOPICS[topic_key],
                "receipt_time_s": time.time(),
                "elapsed_s": time.monotonic() - started_mono,
                "payload": safe_payload,
                "payload_sha256": _canonical_sha256(safe_payload),
            }
        )

    def on_string(topic_key: str) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            raw = str(message.data)
            try:
                payload: Any = json.loads(raw)
            except json.JSONDecodeError:
                payload = raw
            append(topic_key, payload)

        return callback

    def on_twist(topic_key: str) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            append(
                topic_key,
                {
                    "linear_x": float(message.linear.x),
                    "angular_z": float(message.angular.z),
                },
            )

        return callback

    def on_odom(message: Any) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0
            * (
                float(orientation.w) * float(orientation.z)
                + float(orientation.x) * float(orientation.y)
            ),
            1.0
            - 2.0
            * (
                float(orientation.y) ** 2 + float(orientation.z) ** 2
            ),
        )
        append(
            "odom",
            {
                "frame_id": str(message.header.frame_id),
                "child_frame_id": str(message.child_frame_id),
                "x": float(position.x),
                "y": float(position.y),
                "yaw": yaw,
                "linear_x": float(message.twist.twist.linear.x),
                "angular_z": float(message.twist.twist.angular.z),
            },
        )

    def on_scan(message: Any) -> None:
        finite_ranges = [
            float(value)
            for value in message.ranges
            if math.isfinite(float(value))
            and float(message.range_min) <= float(value) <= float(message.range_max)
        ]
        append(
            "scan",
            {
                "frame_id": str(message.header.frame_id),
                "sample_count": len(message.ranges),
                "finite_count": len(finite_ranges),
                "minimum_range_m": min(finite_ranges) if finite_ranges else None,
                "angle_min": float(message.angle_min),
                "angle_increment": float(message.angle_increment),
            },
        )

    def on_tf(topic_key: str) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            append(
                topic_key,
                [
                    {
                        "parent": str(item.header.frame_id),
                        "child": str(item.child_frame_id),
                        "x": float(item.transform.translation.x),
                        "y": float(item.transform.translation.y),
                        "z": float(item.transform.translation.z),
                        "qx": float(item.transform.rotation.x),
                        "qy": float(item.transform.rotation.y),
                        "qz": float(item.transform.rotation.z),
                        "qw": float(item.transform.rotation.w),
                    }
                    for item in message.transforms
                ],
            )

        return callback

    subscriptions = []
    for key in (
        "collision_state",
        "collision_events",
        "camera",
        "lidar",
        "localization",
        "semantic_map",
        "route_status",
    ):
        subscriptions.append(
            node.create_subscription(String, OBSERVED_TOPICS[key], on_string(key), 50)
        )
    for key in ("requested_cmd", "motor_cmd"):
        subscriptions.append(
            node.create_subscription(Twist, OBSERVED_TOPICS[key], on_twist(key), 50)
        )
    subscriptions.append(
        node.create_subscription(Odometry, OBSERVED_TOPICS["odom"], on_odom, 50)
    )
    subscriptions.append(
        node.create_subscription(
            LaserScan,
            OBSERVED_TOPICS["scan"],
            on_scan,
            qos_profile_sensor_data,
        )
    )
    subscriptions.append(
        node.create_subscription(TFMessage, OBSERVED_TOPICS["tf"], on_tf("tf"), 50)
    )
    tf_static_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    subscriptions.append(
        node.create_subscription(
            TFMessage,
            OBSERVED_TOPICS["tf_static"],
            on_tf("tf_static"),
            tf_static_qos,
        )
    )
    deadline = started_mono + duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic()))
            )
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    observation = {
        "schema": OBSERVATION_SCHEMA,
        "source_sha": source,
        "gate": gate,
        "captured_at_utc": datetime.fromtimestamp(
            started_wall, tz=timezone.utc
        ).isoformat(),
        "duration_s": time.monotonic() - started_mono,
        "read_only": True,
        "motion_authority": False,
        "physical_execution_enabled": False,
        "topics": dict(OBSERVED_TOPICS),
        "events": events,
    }
    observation["observation_sha256"] = _observation_sha256(observation)
    return observation


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plan, observe, audit, and evaluate M7 Phase 3 evidence"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    plan = subparsers.add_parser("plan", help="write the non-executing Phase 3 plan")
    plan.add_argument("--source-sha", required=True)
    plan.add_argument("--output", type=Path)

    template = subparsers.add_parser("template", help="write a fail-closed session template")
    template.add_argument("--source-sha", required=True)
    template.add_argument("--output", type=Path)

    observe = subparsers.add_parser(
        "observe", help="capture bounded subscription-only ROS evidence"
    )
    observe.add_argument("--source-sha", required=True)
    observe.add_argument("--gate", choices=("m7.3", "m7.4"), required=True)
    observe.add_argument("--duration", type=float, default=30.0)
    observe.add_argument("--output", type=Path, required=True)

    audit = subparsers.add_parser("graph-audit", help="inspect graph ownership")
    audit.add_argument("--source-sha", required=True)
    audit.add_argument("--source-repo", type=Path, required=True)
    audit.add_argument("--gate", choices=("m7.3", "m7.4"), required=True)
    audit.add_argument("--stage", choices=("preflight", "active"), required=True)
    audit.add_argument("--output", type=Path)

    cleanup = subparsers.add_parser(
        "cleanup-audit",
        help="record fail-closed post-session process, graph, and device cleanup",
    )
    cleanup.add_argument("--source-sha", required=True)
    cleanup.add_argument("--source-repo", type=Path, required=True)
    cleanup.add_argument("--output", type=Path)

    evaluate = subparsers.add_parser(
        "evaluate", help="evaluate M7.3 alone or the complete sequential session"
    )
    evaluate.add_argument("session", type=Path)
    evaluate.add_argument("--through", choices=("m7.3", "m7.4"), default="m7.4")
    evaluate.add_argument("--output", type=Path)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "plan":
            result = build_plan(source_sha=args.source_sha)
        elif args.command == "template":
            result = build_session_template(source_sha=args.source_sha)
        elif args.command == "observe":
            result = capture_ros_observation(
                source_sha=args.source_sha,
                gate=args.gate,
                duration_s=args.duration,
            )
        elif args.command == "graph-audit":
            result = generate_graph_audit(
                source_sha=args.source_sha,
                source_repo=args.source_repo,
                gate=args.gate,
                stage=args.stage,
            )
        elif args.command == "cleanup-audit":
            result = audit_stationary_cleanup(
                source_sha=args.source_sha,
                source_repo=args.source_repo,
            )
        else:
            result = evaluate_session(
                _load_json(args.session), through_gate=args.through
            )
        if getattr(args, "output", None) is not None:
            _write_json(args.output, result)
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
        return 0 if result.get("passed", True) else 1
    except M7AttendedValidationError as exc:
        print(f"M7 attended validation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
