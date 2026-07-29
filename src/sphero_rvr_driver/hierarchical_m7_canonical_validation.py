"""Durable evaluator for the attended M7.6/M7.7 canonical physical run."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sqlite3
import subprocess
import time
from typing import Any, Callable, Mapping, Optional, Sequence

from .hierarchical_physical_binding import (
    APPROVAL_SCHEMA,
    NAV2_BATCH_SCHEMA,
    PHYSICAL_PROPOSAL_SCHEMA,
    PREFLIGHT_SCHEMA,
    HierarchicalPhysicalApproval,
    canonical_digest,
    resolve_goal_dispatch,
    validate_physical_proposal,
)
from .hierarchical_goal_selection import SemanticGoalDecision
from .mission_api import MissionValidationError


REPORT_SCHEMA = "sphero_rvr.hierarchical_m7_canonical_report.v1"
CLEANUP_SCHEMA = "sphero_rvr.hierarchical_m7_cleanup_capture.v1"
ACTIVE_GRAPH_SCHEMA = (
    "sphero_rvr.hierarchical_m7_active_graph_capture.v1"
)
DEFAULT_MISSION_DATABASE = (
    "~/.local/state/sphero_rvr/missions.sqlite3"
)
DEFAULT_BINDING_JOURNAL = (
    "~/.local/state/sphero_rvr/hierarchical-physical-evidence.sqlite3"
)
DEFAULT_SESSION_DIRECTORY = (
    "~/.local/state/sphero_rvr/hierarchical-session"
)
DEFAULT_EVIDENCE_DIRECTORY = (
    "~/.local/state/sphero_rvr/hierarchical-perception"
)
DEFAULT_SOURCE_REPOSITORY = (
    "/home/jsperson/ros2_ws/src/sphero_rvr_ros"
)


def _json(value: str) -> dict[str, Any]:
    parsed = json.loads(str(value))
    if not isinstance(parsed, Mapping):
        raise MissionValidationError("canonical evidence JSON must be an object")
    return dict(parsed)


def capture_active_graph_evidence(
    *,
    source_sha: str,
    source_repository: str | Path = DEFAULT_SOURCE_REPOSITORY,
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
    captured_at_s: Optional[float] = None,
) -> dict[str, Any]:
    """Capture the exact active command graph before semantic planning starts."""

    commands = {
        "git_head": [
            "git",
            "-C",
            str(Path(source_repository).expanduser()),
            "rev-parse",
            "HEAD",
        ],
        "git_status": [
            "git",
            "-C",
            str(Path(source_repository).expanduser()),
            "status",
            "--porcelain",
        ],
        "nodes": [
            "timeout",
            "8",
            "ros2",
            "node",
            "list",
            "--spin-time",
            "3.0",
            "--no-daemon",
        ],
        "cmd_vel": [
            "timeout",
            "8",
            "ros2",
            "topic",
            "info",
            "-v",
            "--spin-time",
            "3.0",
            "--no-daemon",
            "/cmd_vel",
        ],
        "cmd_vel_motor": [
            "timeout",
            "8",
            "ros2",
            "topic",
            "info",
            "-v",
            "--spin-time",
            "3.0",
            "--no-daemon",
            "/cmd_vel_motor",
        ],
        "nav2_private": [
            "timeout",
            "8",
            "ros2",
            "topic",
            "info",
            "-v",
            "--spin-time",
            "3.0",
            "--no-daemon",
            "/nav2_cmd_vel_request",
        ],
        "serial_owner": ["fuser", "/dev/ttyAMA0"],
    }
    observations: dict[str, Any] = {}
    for name, argv in commands.items():
        try:
            completed = runner(
                argv,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=12.0,
                check=False,
            )
            observations[name] = {
                "argv": list(argv),
                "returncode": int(completed.returncode),
                "stdout": str(completed.stdout),
                "stderr": str(completed.stderr),
            }
        except (OSError, subprocess.TimeoutExpired) as exc:
            observations[name] = {
                "argv": list(argv),
                "returncode": 124,
                "stdout": "",
                "stderr": f"{exc.__class__.__name__}: {exc}",
            }
    payload = {
        "schema": ACTIVE_GRAPH_SCHEMA,
        "source_sha": str(source_sha).strip(),
        "captured_at_s": float(
            time.time() if captured_at_s is None else captured_at_s
        ),
        "observations": observations,
        "motion_authority": False,
    }
    return {**payload, "capture_digest": canonical_digest(payload)}


def active_graph_checks(
    capture: Mapping[str, Any], *, source_sha: str
) -> dict[str, bool]:
    """Recompute active ownership only from raw command observations."""

    payload = dict(capture)
    supplied_digest = str(payload.pop("capture_digest", ""))
    observations = payload.get("observations", {})
    if not isinstance(observations, Mapping):
        observations = {}

    def observation(name: str) -> Mapping[str, Any]:
        value = observations.get(name, {})
        return value if isinstance(value, Mapping) else {}

    def returncode(name: str) -> int:
        try:
            return int(observation(name).get("returncode", 1))
        except (TypeError, ValueError):
            return 1

    def stdout(name: str) -> str:
        return str(observation(name).get("stdout", ""))

    exact_source = str(source_sha).strip()
    return {
        "capture_digest_valid": (
            supplied_digest == canonical_digest(payload)
        ),
        "capture_schema_valid": (
            payload.get("schema") == ACTIVE_GRAPH_SCHEMA
            and payload.get("motion_authority") is False
        ),
        "exact_source_sha": (
            payload.get("source_sha") == exact_source
            and returncode("git_head") == 0
            and stdout("git_head").strip() == exact_source
        ),
        "source_checkout_clean": (
            returncode("git_status") == 0
            and not stdout("git_status").strip()
        ),
    }


def validate_active_graph_evidence(
    capture: Mapping[str, Any], *, source_sha: str
) -> dict[str, Any]:
    checks = active_graph_checks(capture, source_sha=source_sha)
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise MissionValidationError(
            "canonical active graph audit failed: " + ",".join(failed)
        )
    return {**dict(capture), "recomputed_checks": checks}


def capture_cleanup_evidence(
    *,
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
    recorded_at_s: Optional[float] = None,
    session_directory: str | Path = DEFAULT_SESSION_DIRECTORY,
    evidence_directory: str | Path = DEFAULT_EVIDENCE_DIRECTORY,
) -> dict[str, Any]:
    """Capture raw cleanup observations; no pass/fail booleans are accepted."""

    commands = {
        "hierarchical_unit": [
            "systemctl",
            "--user",
            "show",
            "--property=ActiveState",
            "--property=SubState",
            "rvr-hierarchical-mission.service",
        ],
        "telemetry_unit": [
            "systemctl",
            "--user",
            "show",
            "--property=ActiveState",
            "--property=SubState",
            "rvr-telemetry.service",
        ],
        "nodes": [
            "timeout",
            "8",
            "ros2",
            "node",
            "list",
            "--spin-time",
            "3.0",
        ],
        "cmd_vel": [
            "timeout",
            "8",
            "ros2",
            "topic",
            "info",
            "-v",
            "/cmd_vel",
            "--spin-time",
            "3.0",
        ],
        "cmd_vel_motor": [
            "timeout",
            "8",
            "ros2",
            "topic",
            "info",
            "-v",
            "/cmd_vel_motor",
            "--spin-time",
            "3.0",
        ],
        "processes": ["ps", "-eo", "pid=,args="],
        "serial_owner": ["fuser", "/dev/ttyAMA0"],
    }
    observations: dict[str, Any] = {}
    for name, argv in commands.items():
        try:
            completed = runner(
                argv,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=12.0,
                check=False,
            )
            observations[name] = {
                "argv": list(argv),
                "returncode": int(completed.returncode),
                "stdout": str(completed.stdout),
                "stderr": str(completed.stderr),
            }
        except (OSError, subprocess.TimeoutExpired) as exc:
            observations[name] = {
                "argv": list(argv),
                "returncode": 124,
                "stdout": "",
                "stderr": f"{exc.__class__.__name__}: {exc}",
            }
    directory = Path(session_directory).expanduser()
    files = {
        name: (directory / name).exists()
        for name in (
            "session.env",
            "proposal.json",
            "approval.json",
            "active-graph.json",
        )
    }
    evidence_root = Path(evidence_directory).expanduser()
    evidence_files = []
    evidence_inspection_error = ""
    try:
        if evidence_root.exists():
            evidence_files = [
                {
                    "name": item.name,
                    "byte_count": int(item.stat().st_size),
                }
                for item in sorted(evidence_root.iterdir())
                if item.is_file()
            ]
    except OSError as exc:
        evidence_inspection_error = (
            f"{exc.__class__.__name__}: {exc}"
        )
    payload = {
        "schema": CLEANUP_SCHEMA,
        "recorded_at_s": float(
            time.time() if recorded_at_s is None else recorded_at_s
        ),
        "observations": observations,
        "activation_files_present": files,
        "evidence_storage": {
            "directory": str(evidence_root),
            "files": evidence_files,
            "inspection_error": evidence_inspection_error,
        },
    }
    return {**payload, "capture_digest": canonical_digest(payload)}


def _cleanup_checks(capture: Mapping[str, Any]) -> dict[str, bool]:
    payload = dict(capture)
    supplied = str(payload.pop("capture_digest", ""))
    digest_valid = supplied == canonical_digest(payload)
    observations = payload.get("observations", {})
    files = payload.get("activation_files_present", {})
    if not isinstance(observations, Mapping) or not isinstance(files, Mapping):
        return {
            "cleanup_capture_digest_valid": digest_valid,
            "hierarchical_unit_inactive": False,
            "telemetry_unit_inactive": False,
            "motion_nodes_absent": False,
            "cmd_vel_publishers_absent": False,
            "cmd_vel_motor_publishers_absent": False,
            "motion_processes_absent": False,
            "serial_owner_absent": False,
            "evidence_writers_absent": False,
            "camera_storage_bounded": False,
            "activation_files_consumed": False,
        }

    def observation(name: str) -> Mapping[str, Any]:
        value = observations.get(name, {})
        return value if isinstance(value, Mapping) else {}

    def stdout(name: str) -> str:
        return str(observation(name).get("stdout", ""))

    def stderr(name: str) -> str:
        return str(observation(name).get("stderr", ""))

    def returncode(name: str) -> int:
        try:
            return int(observation(name).get("returncode", 1))
        except (TypeError, ValueError):
            return 1

    def no_publishers(name: str) -> bool:
        output = stdout(name)
        error = stderr(name)
        code = returncode(name)
        return (
            code == 0 and "Publisher count: 0" in output
            or (
                code == 1
                and "Publisher count:" not in output
                and "Unknown topic" in f"{output}\n{error}"
            )
        )

    nodes = stdout("nodes")
    processes = stdout("processes")
    forbidden_nodes = (
        "rvr_driver",
        "live_route_runner",
        "collision_stop",
        "controller_server",
        "planner_server",
        "hierarchical_physical_authority",
        "hierarchical_mission_controller",
        "hierarchical_nav2_adapter",
        "rplidar",
        "camera",
        "stationary_perception",
        "semantic_perception",
        "slam_toolbox",
        "rosbag",
    )
    forbidden_processes = (
        "sphero_rvr_driver.rvr_node",
        " rvr_node",
        "live_route_runner",
        "collision_stop_node",
        "rplidar_node",
        "nav2_controller",
        "hierarchical_physical_authority",
        "hierarchical_mission_controller",
        "hierarchical_nav2_adapter",
        "camera_node",
        "stationary_perception",
        "semantic_perception",
        "slam_toolbox",
        "ros2 bag record",
        "rosbag2",
    )
    storage = payload.get("evidence_storage", {})
    storage_files = (
        storage.get("files", ())
        if isinstance(storage, Mapping)
        else ()
    )
    storage_valid = isinstance(storage_files, Sequence) and not isinstance(
        storage_files, (str, bytes)
    ) and not str(
        storage.get("inspection_error", "")
        if isinstance(storage, Mapping)
        else "missing"
    )
    if storage_valid:
        for item in storage_files:
            if not isinstance(item, Mapping):
                storage_valid = False
                break
            try:
                name = str(item["name"])
                byte_count = int(item["byte_count"])
            except (KeyError, TypeError, ValueError):
                storage_valid = False
                break
            if (
                not name.startswith("live-camera-")
                or not name.endswith(".jpg")
                or byte_count <= 0
                or byte_count > 512_000
            ):
                storage_valid = False
                break
    return {
        "cleanup_capture_digest_valid": (
            digest_valid and payload.get("schema") == CLEANUP_SCHEMA
        ),
        "hierarchical_unit_inactive": (
            returncode("hierarchical_unit") == 0
            and "ActiveState=inactive" in stdout("hierarchical_unit")
        ),
        "telemetry_unit_inactive": (
            returncode("telemetry_unit") == 0
            and "ActiveState=inactive" in stdout("telemetry_unit")
        ),
        "motion_nodes_absent": (
            returncode("nodes") == 0
            and not any(token in nodes for token in forbidden_nodes)
        ),
        "cmd_vel_publishers_absent": no_publishers("cmd_vel"),
        "cmd_vel_motor_publishers_absent": no_publishers(
            "cmd_vel_motor"
        ),
        "motion_processes_absent": (
            returncode("processes") == 0
            and not any(
                token in processes for token in forbidden_processes
            )
        ),
        "serial_owner_absent": (
            returncode("serial_owner") == 1
            and not stdout("serial_owner").strip()
            and not stderr("serial_owner").strip()
        ),
        "evidence_writers_absent": (
            returncode("processes") == 0
            and not any(
                token in processes
                for token in (
                    "camera_node",
                    "stationary_perception",
                    "semantic_perception",
                    "ros2 bag record",
                    "rosbag2",
                )
            )
        ),
        "camera_storage_bounded": (
            storage_valid and len(storage_files) <= 96
        ),
        "activation_files_consumed": (
            set(files)
            == {
                "session.env",
                "proposal.json",
                "approval.json",
                "active-graph.json",
            }
            and not any(bool(value) for value in files.values())
        ),
    }


def _binding_events(
    journal: str | Path, mission_id: str
) -> list[dict[str, Any]]:
    connection = sqlite3.connect(
        str(Path(journal).expanduser())
    )
    try:
        rows = connection.execute(
            """
            SELECT event_index,kind,recorded_at_s,payload_json,payload_sha256
            FROM hierarchical_binding_events
            WHERE mission_id=?
            ORDER BY event_index
            """,
            (str(mission_id),),
        ).fetchall()
    finally:
        connection.close()
    return [
        {
            "event_index": int(row[0]),
            "kind": str(row[1]),
            "recorded_at_s": float(row[2]),
            "payload": _json(str(row[3])),
            "payload_sha256": str(row[4]),
        }
        for row in rows
    ]


def _mission_record(
    database: str | Path, mission_id: str
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    connection = sqlite3.connect(
        str(Path(database).expanduser())
    )
    connection.row_factory = sqlite3.Row
    try:
        row = connection.execute(
            "SELECT * FROM prompt_missions WHERE mission_id=?",
            (str(mission_id),),
        ).fetchone()
        if row is None:
            raise MissionValidationError(
                f"unknown canonical mission: {mission_id}"
            )
        event_rows = connection.execute(
            """
            SELECT event_id,kind,payload_json,created_at_s,source_sha,deployed_sha
            FROM events WHERE mission_id=? ORDER BY event_id
            """,
            (str(mission_id),),
        ).fetchall()
    finally:
        connection.close()
    mission = dict(row)
    for name in (
        "proposal_json",
        "approval_json",
        "route_json",
        "result_json",
    ):
        mission[name] = _json(str(mission[name]))
    events = [
        {
            "event_id": int(item[0]),
            "kind": str(item[1]),
            "payload": _json(str(item[2])),
            "created_at_s": float(item[3]),
            "source_sha": str(item[4]),
            "deployed_sha": str(item[5]),
        }
        for item in event_rows
    ]
    return mission, events


def _wait_planning_intervals(
    service_events: Sequence[Mapping[str, Any]],
    *,
    terminal_at_s: Optional[float],
) -> tuple[list[dict[str, Any]], bool]:
    """Reconstruct every controller wait interval from durable checkpoints."""

    checkpoints: list[tuple[float, str, str, int]] = []
    valid = True
    for event in service_events:
        if event.get("kind") != "hierarchical_checkpoint":
            continue
        payload = event.get("payload", {})
        if (
            not isinstance(payload, Mapping)
            or payload.get("source") != "hierarchical_controller"
            or not isinstance(payload.get("value"), Mapping)
        ):
            continue
        value = payload["value"]
        try:
            observed_at_s = float(payload["received_at_s"])
            event_id = int(event["event_id"])
        except (KeyError, TypeError, ValueError):
            valid = False
            continue
        if not math.isfinite(observed_at_s):
            valid = False
            continue
        checkpoints.append(
            (
                observed_at_s,
                str(value.get("state", "")).lower(),
                str(value.get("reason", "")),
                event_id,
            )
        )
    if not checkpoints:
        return [], False
    if any(
        current[0] < previous[0]
        for previous, current in zip(checkpoints, checkpoints[1:])
    ):
        valid = False
    intervals: list[dict[str, Any]] = []
    active: Optional[dict[str, Any]] = None
    for observed_at_s, state, reason, event_id in checkpoints:
        if state == "wait_planning":
            if active is None:
                active = {
                    "started_at_s": observed_at_s,
                    "started_event_id": event_id,
                    "reasons": [],
                }
            if reason and reason not in active["reasons"]:
                active["reasons"].append(reason)
            continue
        if active is not None:
            intervals.append(
                {
                    **active,
                    "ended_at_s": observed_at_s,
                    "ended_event_id": event_id,
                    "duration_s": observed_at_s
                    - float(active["started_at_s"]),
                    "terminal_close": False,
                }
            )
            active = None
    if active is not None:
        try:
            terminal = float(terminal_at_s)
        except (TypeError, ValueError):
            valid = False
        else:
            if (
                not math.isfinite(terminal)
                or terminal < float(active["started_at_s"])
            ):
                valid = False
            else:
                intervals.append(
                    {
                        **active,
                        "ended_at_s": terminal,
                        "ended_event_id": None,
                        "duration_s": terminal
                        - float(active["started_at_s"]),
                        "terminal_close": True,
                    }
                )
    valid = valid and all(
        math.isfinite(float(item["duration_s"]))
        and float(item["duration_s"]) >= 0.0
        for item in intervals
    )
    return intervals, valid


def _camera_evidence_is_bounded(value: Mapping[str, Any]) -> bool:
    allowed_top_level = {
        "schema",
        "frame_id",
        "stamp_s",
        "width",
        "height",
        "calibrated",
        "uncertain_track_id",
        "detections",
        "image_attachment",
    }
    allowed_detection_fields = {
        "kind",
        "label",
        "confidence",
        "status",
        "track_id",
        "bbox",
        "position_method",
        "calibration_id",
        "map_revision",
        "localization_evidence_ids",
        "localization_reason",
        "source_timestamps_ns",
        "bearing",
    }
    if set(value) != allowed_top_level:
        return False
    try:
        stamp_s = float(value["stamp_s"])
        width = int(value["width"])
        height = int(value["height"])
    except (KeyError, TypeError, ValueError):
        return False
    detections = value.get("detections")
    attachment = value.get("image_attachment")
    if (
        value.get("schema")
        != "sphero_rvr.live_camera_perception.v1"
        or not str(value.get("frame_id", "")).strip()
        or value.get("calibrated") is not True
        or not math.isfinite(stamp_s)
        or stamp_s <= 0.0
        or width <= 0
        or height <= 0
        or not isinstance(detections, list)
        or len(detections) > 32
        or any(
            not isinstance(detection, Mapping)
            or not set(detection) <= allowed_detection_fields
            for detection in detections
        )
        or not isinstance(attachment, Mapping)
    ):
        return False
    if not attachment:
        return True
    if set(attachment) != {
        "schema",
        "frame_id",
        "path",
        "mime_type",
        "sha256",
        "byte_count",
    }:
        return False
    try:
        byte_count = int(attachment["byte_count"])
    except (KeyError, TypeError, ValueError):
        return False
    digest = str(attachment.get("sha256", ""))
    return (
        attachment.get("schema")
        == "sphero_rvr.camera_image_attachment.v1"
        and attachment.get("frame_id") == value.get("frame_id")
        and str(attachment.get("path", "")).startswith("/")
        and attachment.get("mime_type") == "image/jpeg"
        and len(digest) == 64
        and all(character in "0123456789abcdef" for character in digest)
        and 0 < byte_count <= 512_000
    )


def _semantic_snapshot_digest_valid(value: Mapping[str, Any]) -> bool:
    unsigned = dict(value)
    supplied = str(unsigned.pop("snapshot_id", ""))
    try:
        return (
            len(supplied) == 64
            and supplied == canonical_digest(unsigned)
        )
    except (TypeError, ValueError):
        return False


def evaluate_canonical_mission(
    *,
    mission_database: str | Path,
    binding_journal: str | Path,
    mission_id: str,
    cleanup_capture: Mapping[str, Any],
) -> dict[str, Any]:
    mission, service_events = _mission_record(
        mission_database, mission_id
    )
    binding_events = _binding_events(binding_journal, mission_id)
    proposal = mission["proposal_json"]
    approval = mission["approval_json"]
    result = mission["result_json"]
    proposal_unsigned = dict(proposal)
    proposal_digest = str(
        proposal_unsigned.pop("proposal_digest", "")
    )
    proposal_valid = False
    proposal_lease_s = float("nan")
    try:
        validated_proposal = validate_physical_proposal(
            proposal,
            authority={
                "mission_id": str(mission_id),
                "proposal_digest": proposal_digest,
                "mission_lease_s": approval.get("mission_lease_s"),
            },
            source_sha=str(mission["source_sha"]),
        )
        proposal_lease_s = float(
            validated_proposal["mission_lease_s"]
        )
        proposal_valid = True
    except (KeyError, TypeError, ValueError, MissionValidationError):
        proposal_valid = False
    approval_valid = False
    try:
        validated = HierarchicalPhysicalApproval.validated(
            approval,
            now_s=float(approval["approved_at_s"]) + 0.001,
            source_sha=str(mission["source_sha"]),
            deployed_sha=str(mission["deployed_sha"]),
            reviewed_sha=str(mission["source_sha"]),
        )
        approval_valid = (
            validated.mission_id == str(mission_id)
            and validated.proposal_digest == proposal_digest
            and validated.mission_lease_s == proposal_lease_s
        )
    except (KeyError, TypeError, ValueError, MissionValidationError):
        approval_valid = False

    binding_digests_valid = all(
        event["payload_sha256"] == canonical_digest(event["payload"])
        for event in binding_events
    )
    kinds = [event["kind"] for event in binding_events]
    dispatch_events = [
        event for event in binding_events if event["kind"] == "goal_dispatch"
    ]
    dispatches = [event["payload"] for event in dispatch_events]
    provider_events = [
        event["payload"]
        for event in binding_events
        if event["kind"] == "provider_call_completed"
        or (
            event["kind"] == "controller_event"
            and event["payload"].get("provider_elapsed_s")
            is not None
        )
    ]
    controller_events = [
        event["payload"]
        for event in binding_events
        if event["kind"] == "controller_event"
    ]
    world_snapshots = [
        event["payload"]
        for event in binding_events
        if event["kind"] == "world_snapshot"
    ]
    resolved_batch_events = [
        event
        for event in binding_events
        if event["kind"] == "resolved_goal_batch"
    ]
    resolved_goal_batches = [
        event["payload"] for event in resolved_batch_events
    ]
    nav2_paths = [
        event["payload"]
        for event in binding_events
        if event["kind"] == "nav2_path"
    ]
    decisions: list[dict[str, Any]] = []
    decision_bindings: list[
        tuple[dict[str, Any], Mapping[str, Any]]
    ] = []
    tracks: dict[str, dict[str, Any]] = {}
    coverage_samples: list[float] = []
    frontier_snapshots: list[dict[str, Any]] = []
    dispatch_snapshots_valid = bool(dispatches)

    def observe_snapshot(snapshot: Mapping[str, Any]) -> None:
        map_snapshot = snapshot.get("map", {})
        if isinstance(map_snapshot, Mapping):
            try:
                coverage = float(map_snapshot["coverage_fraction"])
            except (KeyError, TypeError, ValueError):
                coverage = float("nan")
            if math.isfinite(coverage):
                coverage_samples.append(coverage)
        frontiers = snapshot.get("frontiers", ())
        frontier_snapshots.append(
            {
                "snapshot_id": str(snapshot.get("snapshot_id", "")),
                "map_id": str(
                    map_snapshot.get("map_id", "")
                    if isinstance(map_snapshot, Mapping)
                    else ""
                ),
                "map_revision": str(
                    map_snapshot.get("map_revision", "")
                    if isinstance(map_snapshot, Mapping)
                    else ""
                ),
                "signatures": [
                    str(item.get("signature", ""))
                    for item in frontiers
                    if isinstance(item, Mapping)
                    and str(item.get("signature", ""))
                ],
            }
        )
        for track in snapshot.get("tracks", ()):
            if isinstance(track, Mapping):
                track_id = str(track.get("track_id", ""))
                if track_id:
                    tracks[track_id] = dict(track)

    for dispatch in dispatches:
        for goal in dispatch.get("goals", ()):
            if not isinstance(goal, Mapping):
                dispatch_snapshots_valid = False
                continue
            captured = goal.get("captured_snapshot", {})
            decision = goal.get("decision", {})
            if isinstance(decision, Mapping):
                recorded_decision = dict(decision)
                decisions.append(recorded_decision)
                if isinstance(captured, Mapping):
                    decision_bindings.append(
                        (recorded_decision, captured)
                    )
                else:
                    dispatch_snapshots_valid = False
            else:
                dispatch_snapshots_valid = False
            current = goal.get("current_snapshot", {})
            if (
                not isinstance(captured, Mapping)
                or not _semantic_snapshot_digest_valid(captured)
                or not isinstance(current, Mapping)
                or not _semantic_snapshot_digest_valid(current)
            ):
                dispatch_snapshots_valid = False
            if not world_snapshots:
                if isinstance(current, Mapping):
                    observe_snapshot(current)
    if world_snapshots:
        for evidence in world_snapshots:
            snapshot = evidence.get("snapshot", {})
            if isinstance(snapshot, Mapping):
                observe_snapshot(snapshot)
    provider_snapshots_by_id = {
        str(evidence["snapshot"].get("snapshot_id", "")): evidence[
            "snapshot"
        ]
        for evidence in world_snapshots
        if isinstance(evidence.get("snapshot"), Mapping)
        and str(evidence["snapshot"].get("snapshot_id", ""))
    }
    provider_event_snapshot_ids = [
        str(event.get("snapshot_id", "")).strip()
        for event in provider_events
    ]
    for event in controller_events:
        if event.get("kind") != "semantic_non_motion_goal_ready":
            continue
        decision = event.get("decision")
        if isinstance(decision, Mapping):
            recorded_decision = dict(decision)
            decisions.append(recorded_decision)
            provider_snapshot = provider_snapshots_by_id.get(
                str(recorded_decision.get("snapshot_id", ""))
            )
            if provider_snapshot is not None:
                decision_bindings.append(
                    (recorded_decision, provider_snapshot)
                )
    expected_argument_keys = {
        "go_to_frontier": {"frontier_id"},
        "inspect": {"track_id"},
        "search_region": {"region_id", "target_classes"},
        "return_to_start": set(),
        "wait": set(),
        "finish": {"outcome", "evidence_ids"},
    }
    semantic_only = all(
        str(decision.get("action", "")) in expected_argument_keys
        and isinstance(decision.get("arguments"), Mapping)
        and set(decision["arguments"])
        == expected_argument_keys[str(decision["action"])]
        and not any(
            name in decision
            for name in (
                "x",
                "y",
                "x_m",
                "y_m",
                "yaw",
                "yaw_rad",
                "pose",
                "route",
                "path",
                "speed",
                "velocity",
                "cmd_vel",
            )
        )
        and bool(str(decision.get("rationale", "")).strip())
        for decision in decisions
    )
    strict_semantic_bindings_valid = (
        bool(decisions)
        and len(decision_bindings) == len(decisions)
    )
    semantic_response_fields = {
        "schema",
        "mission_id",
        "snapshot_id",
        "decision_generation",
        "event_generation",
        "action",
        "arguments",
        "rationale",
    }
    semantic_evidence_fields = {"provider_id", "model_id"}
    for decision, snapshot in decision_bindings:
        if (
            not semantic_response_fields <= set(decision)
            or not set(decision)
            <= semantic_response_fields | semantic_evidence_fields
        ):
            strict_semantic_bindings_valid = False
            break
        response = {
            key: decision[key] for key in semantic_response_fields
        }
        try:
            SemanticGoalDecision.validated(
                response,
                snapshot=snapshot,
                expected_generation=int(
                    snapshot.get("decision_generation", 0)
                ),
                provider_id=str(
                    decision.get(
                        "provider_id",
                        "mission-service-semantic-provider",
                    )
                ),
                model_id=str(
                    decision.get(
                        "model_id",
                        "mission-service-semantic-model",
                    )
                ),
            )
        except (TypeError, ValueError, MissionValidationError):
            strict_semantic_bindings_valid = False
            break
    mapped_tracks_truthful = True
    for track in tracks.values():
        position = track.get("position", {})
        try:
            sigma_m = float(track["position_sigma_m"])
            x_m = float(position["x_m"])
            y_m = float(position["y_m"])
        except (KeyError, TypeError, ValueError):
            mapped_tracks_truthful = False
            break
        if (
            not isinstance(position, Mapping)
            or position.get("frame_id") != "map"
            or not all(
                math.isfinite(value) for value in (sigma_m, x_m, y_m)
            )
            or sigma_m < 0.0
            or not track.get("evidence_ids")
            or str(track.get("position_method", ""))
            not in {"lidar_range", "floor_projection"}
        ):
            mapped_tracks_truthful = False
            break
    world_evidence_valid = bool(world_snapshots) and all(
        evidence.get("schema")
        == "sphero_rvr.hierarchical_world_evidence.v1"
        and evidence.get("source_sha") == str(mission["source_sha"])
        and evidence.get("mission_id") == str(mission_id)
        and isinstance(evidence.get("snapshot"), Mapping)
        and bool(str(evidence.get("provider_snapshot_id", "")).strip())
        and evidence.get("provider_snapshot_id")
        == evidence["snapshot"].get("snapshot_id")
        and _semantic_snapshot_digest_valid(evidence["snapshot"])
        and isinstance(evidence.get("camera_evidence"), Mapping)
        and _camera_evidence_is_bounded(
            evidence["camera_evidence"]
        )
        for evidence in world_snapshots
    )
    dispatch_digests = {
        str(dispatch.get("dispatch_digest", ""))
        for dispatch in dispatches
        if len(str(dispatch.get("dispatch_digest", ""))) == 64
    }
    resolved_batches_valid = (
        bool(resolved_goal_batches)
        and len(resolved_goal_batches) == len(dispatches)
        and dispatch_snapshots_valid
    )
    resolution_authority = {
        "mission_id": str(mission_id),
        "source_sha": str(mission["source_sha"]),
        "approval_digest": str(approval.get("approval_digest", "")),
    }
    for dispatch_event, batch_event in zip(
        dispatch_events, resolved_batch_events
    ):
        batch = batch_event["payload"]
        unsigned = dict(batch)
        supplied = str(unsigned.pop("batch_digest", ""))
        poses = batch.get("poses", ())
        if (
            batch_event["event_index"]
            != dispatch_event["event_index"] + 1
            or supplied != canonical_digest(unsigned)
            or batch.get("schema") != NAV2_BATCH_SCHEMA
            or batch.get("mission_id") != str(mission_id)
            or batch.get("source_sha") != str(mission["source_sha"])
            or batch.get("approval_digest")
            != str(approval.get("approval_digest", ""))
            or not isinstance(poses, list)
            or not poses
        ):
            resolved_batches_valid = False
            break
        try:
            created_at_s = float(batch["created_at_s"])
            recomputed_batch = resolve_goal_dispatch(
                dispatch_event["payload"],
                authority=resolution_authority,
                now_s=created_at_s,
            ).to_json_dict()
        except (
            KeyError,
            TypeError,
            ValueError,
            MissionValidationError,
        ):
            resolved_batches_valid = False
            break
        if (
            not math.isfinite(created_at_s)
            or created_at_s != float(batch_event["recorded_at_s"])
            or recomputed_batch != batch
        ):
            resolved_batches_valid = False
            break
        for pose in poses:
            try:
                values = (
                    float(pose["x_m"]),
                    float(pose["y_m"]),
                    float(pose["yaw_rad"]),
                )
            except (KeyError, TypeError, ValueError):
                resolved_batches_valid = False
                break
            if (
                not isinstance(pose, Mapping)
                or not all(math.isfinite(value) for value in values)
                or not str(pose.get("target_id", "")).strip()
                or not str(
                    pose.get("target_signature", "")
                ).strip()
            ):
                resolved_batches_valid = False
                break
        if not resolved_batches_valid:
            break
    goal_batches_by_digest = {
        str(batch.get("batch_digest", "")): batch
        for batch in resolved_goal_batches
        if len(str(batch.get("batch_digest", ""))) == 64
    }
    dispatch_batch_bindings = {
        str(dispatch_event["payload"].get("dispatch_digest", "")): str(
            batch_event["payload"].get("batch_digest", "")
        )
        for dispatch_event, batch_event in zip(
            dispatch_events, resolved_batch_events
        )
    }
    nav2_paths_valid = bool(nav2_paths)
    for path in nav2_paths:
        unsigned = dict(path)
        supplied = str(unsigned.pop("path_digest", ""))
        supplied_content = str(unsigned.get("path_content_digest", ""))
        content = dict(unsigned)
        content.pop("path_content_digest", None)
        content.pop("recorded_at_s", None)
        poses = path.get("poses", ())
        try:
            original_count = int(path["original_pose_count"])
            sampled_count = int(path["sampled_pose_count"])
        except (KeyError, TypeError, ValueError):
            nav2_paths_valid = False
            break
        if (
            supplied != canonical_digest(unsigned)
            or supplied_content != canonical_digest(content)
            or path.get("schema")
            != "sphero_rvr.hierarchical_nav2_path_evidence.v1"
            or path.get("source_sha") != str(mission["source_sha"])
            or path.get("mission_id") != str(mission_id)
            or path.get("dispatch_digest") not in dispatch_digests
            or path.get("goal_batch_digest")
            not in goal_batches_by_digest
            or path.get("goal_batch_digest")
            != dispatch_batch_bindings.get(
                str(path.get("dispatch_digest", ""))
            )
            or path.get("frame_id") != "map"
            or not isinstance(poses, list)
            or not poses
            or sampled_count != len(poses)
            or original_count < sampled_count
        ):
            nav2_paths_valid = False
            break
        try:
            source_indices = [
                int(pose["source_index"]) for pose in poses
            ]
            if (
                not all(
                    math.isfinite(float(pose[name]))
                    for pose in poses
                    if isinstance(pose, Mapping)
                    for name in ("x_m", "y_m", "yaw_rad")
                )
                or any(not isinstance(pose, Mapping) for pose in poses)
                or source_indices != sorted(set(source_indices))
                or source_indices[0] != 0
                or source_indices[-1] != original_count - 1
            ):
                nav2_paths_valid = False
                break
        except (KeyError, TypeError, ValueError):
            nav2_paths_valid = False
            break
        batch = goal_batches_by_digest[
            str(path["goal_batch_digest"])
        ]
        endpoint = poses[-1]
        try:
            endpoint_matches_batch = any(
                math.hypot(
                    float(endpoint["x_m"]) - float(goal["x_m"]),
                    float(endpoint["y_m"]) - float(goal["y_m"]),
                )
                <= 0.02
                for goal in batch["poses"]
                if isinstance(goal, Mapping)
            )
        except (KeyError, TypeError, ValueError):
            endpoint_matches_batch = False
        if not endpoint_matches_batch:
            nav2_paths_valid = False
            break
    nav2_paths_valid = (
        nav2_paths_valid
        and {
            str(path.get("dispatch_digest", ""))
            for path in nav2_paths
        }
        == dispatch_digests
    )
    terminal_checkpoints = [
        event["payload"]
        for event in service_events
        if event["kind"] == "hierarchical_checkpoint"
        and isinstance(event["payload"].get("value"), Mapping)
        and str(event["payload"]["value"].get("state", "")).lower()
        in {"complete", "recovery_required"}
    ]
    checkpoint_states = [
        str(event["payload"]["value"].get("state", "")).lower()
        for event in service_events
        if event["kind"] == "hierarchical_checkpoint"
        and isinstance(event["payload"].get("value"), Mapping)
    ]
    active_graph_events = [
        event
        for event in service_events
        if event["kind"] == "hierarchical_checkpoint"
        and event["payload"].get("source") == "active_graph_audit"
        and isinstance(event["payload"].get("value"), Mapping)
    ]
    active_graph_capture: dict[str, Any] = {}
    active_graph_valid = False
    if len(active_graph_events) == 1:
        active_graph_event = active_graph_events[0]
        active_graph_capture = dict(
            active_graph_event["payload"]["value"]
        )
        try:
            validate_active_graph_evidence(
                active_graph_capture,
                source_sha=str(mission["source_sha"]),
            )
            graph_capture_at_s = float(
                active_graph_capture["captured_at_s"]
            )
            graph_persisted_at_s = float(
                active_graph_event["created_at_s"]
            )
            planning_times = [
                float(event["recorded_at_s"])
                for event in binding_events
                if event["kind"]
                in {
                    "provider_call_completed",
                    "controller_event",
                    "goal_dispatch",
                    "world_snapshot",
                    "resolved_goal_batch",
                    "nav2_path",
                }
            ]
            active_graph_valid = (
                math.isfinite(graph_capture_at_s)
                and math.isfinite(graph_persisted_at_s)
                and graph_capture_at_s <= graph_persisted_at_s
                and bool(planning_times)
                and graph_capture_at_s <= min(planning_times)
            )
        except (
            KeyError,
            TypeError,
            ValueError,
            MissionValidationError,
        ):
            active_graph_valid = False
    distinct_semantic_goals = {
        (
            str(decision.get("action", "")),
            json.dumps(
                decision.get("arguments", {}),
                sort_keys=True,
                separators=(",", ":"),
            ),
        )
        for decision in decisions
    }
    run_evidence = result.get("run_evidence", {})
    if not isinstance(run_evidence, Mapping):
        run_evidence = {}
    wait_planning_intervals, wait_planning_valid = (
        _wait_planning_intervals(
            service_events,
            terminal_at_s=run_evidence.get("ended_at_s"),
        )
    )
    required_sensor_limits = {
        "lidar": 0.50,
        "localization": 0.300,
    }
    required_sensor_maxima = run_evidence.get(
        "max_required_sensor_age_s", {}
    )
    required_sensor_source_maxima = run_evidence.get(
        "max_required_sensor_source_age_s", {}
    )
    required_sensor_freshness_valid = False
    if (
        isinstance(required_sensor_maxima, Mapping)
        and set(required_sensor_limits).issubset(required_sensor_maxima)
        and isinstance(required_sensor_source_maxima, Mapping)
        and set(required_sensor_limits).issubset(
            required_sensor_source_maxima
        )
    ):
        try:
            parsed_sensor_maxima = {
                source_name: float(required_sensor_maxima[source_name])
                for source_name in required_sensor_limits
            }
            parsed_sensor_source_maxima = {
                source_name: float(
                    required_sensor_source_maxima[source_name]
                )
                for source_name in required_sensor_limits
            }
        except (KeyError, TypeError, ValueError):
            parsed_sensor_maxima = {}
            parsed_sensor_source_maxima = {}
        required_sensor_freshness_valid = bool(
            parsed_sensor_maxima
            and parsed_sensor_source_maxima
        ) and all(
            math.isfinite(parsed_sensor_maxima[source_name])
            and math.isfinite(
                parsed_sensor_source_maxima[source_name]
            )
            and 0.0
            <= parsed_sensor_maxima[source_name]
            <= max_age_s
            and 0.0
            <= parsed_sensor_source_maxima[source_name]
            <= max_age_s
            for source_name, max_age_s in required_sensor_limits.items()
        )
    approval_events = [
        event["payload"]
        for event in service_events
        if event["kind"] == "hierarchical_m7_6_approval"
    ]
    no_contact_events = [
        event["payload"]
        for event in service_events
        if event["kind"]
        == "hierarchical_no_contact_observation"
    ]
    no_contact_observation: dict[str, Any] = {}
    no_contact_valid = False
    if len(no_contact_events) == 1:
        no_contact_observation = dict(no_contact_events[0])
        no_contact_unsigned = dict(no_contact_observation)
        no_contact_digest = str(
            no_contact_unsigned.pop("observation_digest", "")
        )
        try:
            observed_at_s = float(
                no_contact_unsigned["observed_at_s"]
            )
        except (KeyError, TypeError, ValueError):
            observed_at_s = float("nan")
        no_contact_valid = (
            no_contact_unsigned.get("schema")
            == "sphero_rvr.hierarchical_no_contact_observation.v1"
            and no_contact_unsigned.get("mission_id") == str(mission_id)
            and no_contact_unsigned.get("no_contact") is True
            and no_contact_unsigned.get("authentication_source")
            == "tailscale-serve"
            and str(no_contact_unsigned.get("operator", ""))
            == str(approval.get("operator", ""))
            and no_contact_unsigned.get("source_sha")
            == str(mission["source_sha"])
            and no_contact_unsigned.get("deployed_sha")
            == str(mission["deployed_sha"])
            and math.isfinite(observed_at_s)
            and observed_at_s
            >= float(approval.get("approved_at_s", float("inf")))
            and no_contact_digest
            == canonical_digest(no_contact_unsigned)
        )
    preflight: dict[str, Any] = {}
    preflight_valid = False
    if len(approval_events) == 1:
        approval_event = approval_events[0]
        raw_preflight = approval_event.get("preflight", {})
        if isinstance(raw_preflight, Mapping):
            preflight = dict(raw_preflight)
            preflight_unsigned = dict(preflight)
            preflight_digest = str(
                preflight_unsigned.pop("preflight_digest", "")
            )
            sources = preflight_unsigned.get("sources", {})
            expected_ages = {
                "lidar": 5.00,
                "camera": 5.00,
                "localization": 5.00,
                "semantic_map": 5.00,
            }
            source_checks = []
            if isinstance(sources, Mapping) and set(sources) == set(
                expected_ages
            ):
                for source_name, max_age_s in expected_ages.items():
                    source = sources.get(source_name, {})
                    if not isinstance(source, Mapping):
                        source_checks.append(False)
                        continue
                    try:
                        age_s = float(source["age_s"])
                        source_age_s = float(
                            source["source_age_s"]
                        )
                        recorded_max_age_s = float(
                            source["max_age_s"]
                        )
                        received_at_s = float(
                            source["received_at_s"]
                        )
                        observed_at_s = float(
                            preflight_unsigned["observed_at_s"]
                        )
                        source_timestamp_s = float(
                            source["source_timestamp_s"]
                        )
                    except (KeyError, TypeError, ValueError):
                        source_checks.append(False)
                        continue
                    value_digest = str(
                        source.get("value_digest", "")
                    )
                    summary = source.get("summary", {})
                    source_checks.append(
                        math.isfinite(age_s)
                        and 0.0 <= age_s <= max_age_s
                        and math.isfinite(source_age_s)
                        and 0.0 <= source_age_s <= max_age_s
                        and recorded_max_age_s == max_age_s
                        and math.isfinite(received_at_s)
                        and received_at_s <= observed_at_s
                        and math.isfinite(source_timestamp_s)
                        and source_timestamp_s <= observed_at_s
                        and len(value_digest) == 64
                        and all(
                            character in "0123456789abcdef"
                            for character in value_digest
                        )
                        and isinstance(summary, Mapping)
                        and bool(summary)
                    )
            preflight_valid = (
                preflight_unsigned.get("schema") == PREFLIGHT_SCHEMA
                and preflight_unsigned.get("motion_authority") is False
                and preflight_unsigned.get(
                    "physical_execution_enabled"
                )
                is False
                and preflight_digest
                == canonical_digest(preflight_unsigned)
                and str(
                    approval_event.get("preflight_digest", "")
                )
                == preflight_digest
                and math.isclose(
                    float(preflight_unsigned.get("observed_at_s", -1.0)),
                    float(approval.get("approved_at_s", -2.0)),
                    abs_tol=0.001,
                )
                and len(source_checks) == len(expected_ages)
                and all(source_checks)
            )
    cleanup_checks = _cleanup_checks(cleanup_capture)
    checks = {
        "proposal_schema_and_digest_valid": (
            proposal.get("schema") == PHYSICAL_PROPOSAL_SCHEMA
            and proposal_digest == canonical_digest(proposal_unsigned)
            and str(proposal.get("mission_id", "")) == str(mission_id)
            and str(proposal.get("source_sha", ""))
            == str(mission["source_sha"])
            and proposal_valid
        ),
        "approval_schema_and_all_evidence_valid": (
            approval.get("schema") == APPROVAL_SCHEMA
            and approval_valid
        ),
        "fresh_no_motion_sensor_preflight_bound": preflight_valid,
        "active_graph_ownership_verified_before_planning": (
            active_graph_valid
        ),
        "exact_source_deployed_sha": (
            str(mission["source_sha"])
            == str(mission["deployed_sha"])
            and len(str(mission["source_sha"])) == 40
        ),
        "authority_activated_once": kinds.count(
            "authority_activated"
        )
        == 1,
        "authority_relocked_after_activation": (
            "authority_relocked" in kinds
            and kinds.index("authority_relocked")
            > kinds.index("authority_activated")
        )
        if "authority_activated" in kinds
        else False,
        "binding_event_digests_valid": binding_digests_valid,
        "real_provider_completion_recorded": bool(provider_events)
        and len(provider_events) >= 2
        and all(
            event.get("real_provider") is True
            for event in provider_events
        )
        and all(
            isinstance(event.get("provider_elapsed_s"), (int, float))
            and float(event["provider_elapsed_s"]) >= 0.0
            for event in provider_events
        ),
        "semantic_goal_dispatch_recorded": bool(decisions),
        "provider_world_snapshots_recorded": (
            world_evidence_valid
            and len(world_snapshots) == len(provider_events)
            and len(set(provider_event_snapshot_ids))
            == len(provider_event_snapshot_ids)
            and set(provider_event_snapshot_ids)
            == set(provider_snapshots_by_id)
        ),
        "material_semantic_replanning_recorded": (
            len(distinct_semantic_goals) >= 2
        ),
        "wfd_frontier_history_reconstructable": (
            len(frontier_snapshots) >= 2
            and all(
                bool(snapshot["snapshot_id"])
                and bool(snapshot["map_id"])
                and bool(snapshot["map_revision"])
                for snapshot in frontier_snapshots
            )
            and any(
                bool(snapshot["signatures"])
                for snapshot in frontier_snapshots
            )
        ),
        "coverage_reconstructable": (
            len(coverage_samples) >= 2
            and all(
                0.0 < value <= 1.0
                for value in coverage_samples
            )
        ),
        "model_geometry_absent_and_rationales_present": (
            bool(decisions)
            and semantic_only
            and strict_semantic_bindings_valid
        ),
        "server_resolved_goal_batches_recorded": (
            resolved_batches_valid
        ),
        "nav2_planned_paths_recorded": nav2_paths_valid,
        "camera_detection_evidence_recorded": (
            world_evidence_valid
            and all(
                isinstance(
                    evidence["camera_evidence"].get(
                        "detections", ()
                    ),
                    list,
                )
                for evidence in world_snapshots
            )
        ),
        "mapped_tracks_evidence_bound": mapped_tracks_truthful,
        "handoffs_and_pauses_reconstructable": (
            wait_planning_valid
            and all(
                "kind" in event and "at_s" in event
                for event in controller_events
            )
            and any(
                str(event.get("kind", ""))
                in {
                    "atomic_handoff",
                    "planning_hold",
                    "planning_resume",
                    "prefetch_dispatched",
                    "prefetch_revalidated",
                }
                for event in controller_events
            )
        ),
        "physical_motion_observed_from_odom": (
            int(run_evidence.get("nonzero_odom_samples", 0)) > 0
            and float(run_evidence.get("max_displacement_m", 0.0))
            >= 0.02
        ),
        "localization_freshness_remained_within_gate": (
            int(
                run_evidence.get(
                    "localization_freshness_violations", 0
                )
            )
            == 0
            and float(
                run_evidence.get(
                    "max_localization_age_s", float("inf")
                )
            )
            <= 0.300
        ),
        "all_motion_critical_sensor_freshness_remained_within_gate": (
            int(
                run_evidence.get(
                    "required_sensor_freshness_violations", 0
                )
            )
            == 0
            and required_sensor_freshness_valid
        ),
        "terminal_controller_checkpoint_recorded": bool(
            terminal_checkpoints
        ),
        "mission_terminal_complete": mission["status"] == "complete",
        "authenticated_operator_no_contact_observation": (
            no_contact_valid
        ),
        "terminal_cleanup_claim_matches_capture": (
            result.get("cleanup_verified") is True
        ),
        **cleanup_checks,
    }
    passed = all(checks.values())
    evidence = {
        "mission_id": str(mission_id),
        "source_sha": str(mission["source_sha"]),
        "deployed_sha": str(mission["deployed_sha"]),
        "proposal_digest": proposal_digest,
        "approval_digest": str(approval.get("approval_digest", "")),
        "binding_event_count": len(binding_events),
        "service_event_count": len(service_events),
        "semantic_decisions": decisions,
        "coverage_samples": coverage_samples,
        "frontier_snapshots": frontier_snapshots,
        "mapped_tracks": list(tracks.values()),
        "provider_calls": provider_events,
        "world_snapshots": world_snapshots,
        "resolved_goal_batches": resolved_goal_batches,
        "nav2_paths": nav2_paths,
        "sensor_preflight": preflight,
        "active_graph_capture": active_graph_capture,
        "operator_no_contact_observation": no_contact_observation,
        "controller_events": controller_events,
        "wait_planning_intervals": wait_planning_intervals,
        "checkpoint_states": checkpoint_states,
        "run_evidence": dict(run_evidence),
        "goal_dispatches": dispatches,
        "binding_events": binding_events,
        "service_events": service_events,
        "terminal_result": result,
        "cleanup_capture": dict(cleanup_capture),
        "cleanup_capture_digest": str(
            cleanup_capture.get("capture_digest", "")
        ),
    }
    payload = {
        "schema": REPORT_SCHEMA,
        "mission_id": str(mission_id),
        "passed": passed,
        "checks": checks,
        "evidence": evidence,
    }
    return {**payload, "report_digest": canonical_digest(payload)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate durable M7.6/M7.7 canonical physical evidence."
    )
    parser.add_argument("--mission-id", required=True)
    parser.add_argument(
        "--mission-database", default=DEFAULT_MISSION_DATABASE
    )
    parser.add_argument(
        "--binding-journal", default=DEFAULT_BINDING_JOURNAL
    )
    parser.add_argument(
        "--cleanup-capture",
        help="Read an existing generated cleanup capture instead of capturing now.",
    )
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    if args.cleanup_capture:
        cleanup = _json(
            Path(args.cleanup_capture)
            .expanduser()
            .read_text(encoding="utf-8")
        )
    else:
        cleanup = capture_cleanup_evidence()
    report = evaluate_canonical_mission(
        mission_database=args.mission_database,
        binding_journal=args.binding_journal,
        mission_id=args.mission_id,
        cleanup_capture=cleanup,
    )
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    temporary.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.chmod(0o600)
    temporary.replace(output)
    print(
        json.dumps(
            {
                "mission_id": args.mission_id,
                "passed": report["passed"],
                "report_digest": report["report_digest"],
                "output": str(output),
            },
            sort_keys=True,
        )
    )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
