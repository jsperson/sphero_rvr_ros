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
    PHYSICAL_PROPOSAL_SCHEMA,
    PREFLIGHT_SCHEMA,
    HierarchicalPhysicalApproval,
    canonical_digest,
)
from .mission_api import MissionValidationError


REPORT_SCHEMA = "sphero_rvr.hierarchical_m7_canonical_report.v1"
CLEANUP_SCHEMA = "sphero_rvr.hierarchical_m7_cleanup_capture.v1"
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


def _json(value: str) -> dict[str, Any]:
    parsed = json.loads(str(value))
    if not isinstance(parsed, Mapping):
        raise MissionValidationError("canonical evidence JSON must be an object")
    return dict(parsed)


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
        for name in ("session.env", "proposal.json", "approval.json")
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
            == {"session.env", "proposal.json", "approval.json"}
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
        )
    except (KeyError, TypeError, ValueError, MissionValidationError):
        approval_valid = False

    binding_digests_valid = all(
        event["payload_sha256"] == canonical_digest(event["payload"])
        for event in binding_events
    )
    kinds = [event["kind"] for event in binding_events]
    dispatches = [
        event["payload"]
        for event in binding_events
        if event["kind"] == "goal_dispatch"
    ]
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
    decisions: list[dict[str, Any]] = []
    tracks: dict[str, dict[str, Any]] = {}
    coverage_samples: list[float] = []
    for dispatch in dispatches:
        for goal in dispatch.get("goals", ()):
            if not isinstance(goal, Mapping):
                continue
            decision = goal.get("decision", {})
            if isinstance(decision, Mapping):
                decisions.append(dict(decision))
            current = goal.get("current_snapshot", {})
            if isinstance(current, Mapping):
                map_snapshot = current.get("map", {})
                if isinstance(map_snapshot, Mapping):
                    try:
                        coverage = float(
                            map_snapshot["coverage_fraction"]
                        )
                    except (KeyError, TypeError, ValueError):
                        coverage = float("nan")
                    if math.isfinite(coverage):
                        coverage_samples.append(coverage)
                for track in current.get("tracks", ()):
                    if isinstance(track, Mapping):
                        track_id = str(track.get("track_id", ""))
                        if track_id:
                            tracks[track_id] = dict(track)
    semantic_only = all(
        set(decision.get("arguments", {}))
        <= {"frontier_id", "track_id", "view_id", "region_id"}
        and not any(
            name in decision
            for name in ("x_m", "y_m", "pose", "route", "velocity")
        )
        and bool(str(decision.get("rationale", "")).strip())
        for decision in decisions
    )
    mapped_tracks_truthful = all(
        bool(track.get("evidence_ids"))
        and str(track.get("position_method", ""))
        in {"lidar_range", "floor_projection"}
        for track in tracks.values()
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
                "lidar": 0.50,
                "camera": 1.00,
                "localization": 0.300,
                "semantic_map": 1.00,
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
                        recorded_max_age_s = float(
                            source["max_age_s"]
                        )
                        received_at_s = float(
                            source["received_at_s"]
                        )
                        observed_at_s = float(
                            preflight_unsigned["observed_at_s"]
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
                        and recorded_max_age_s == max_age_s
                        and math.isfinite(received_at_s)
                        and received_at_s <= observed_at_s
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
        ),
        "approval_schema_and_all_evidence_valid": (
            approval.get("schema") == APPROVAL_SCHEMA
            and approval_valid
        ),
        "fresh_no_motion_sensor_preflight_bound": preflight_valid,
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
        and any(
            event.get("real_provider") is True
            for event in provider_events
        )
        and all(
            isinstance(event.get("provider_elapsed_s"), (int, float))
            and float(event["provider_elapsed_s"]) >= 0.0
            for event in provider_events
        ),
        "semantic_goal_dispatch_recorded": bool(decisions),
        "material_semantic_replanning_recorded": (
            len(distinct_semantic_goals) >= 2
        ),
        "coverage_reconstructable": (
            len(coverage_samples) >= 2
            and all(
                0.0 < value <= 1.0
                for value in coverage_samples
            )
        ),
        "model_geometry_absent_and_rationales_present": (
            bool(decisions) and semantic_only
        ),
        "mapped_tracks_evidence_bound": mapped_tracks_truthful,
        "handoffs_and_pauses_reconstructable": (
            bool(checkpoint_states)
            and all(
                "kind" in event and "at_s" in event
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
        "mapped_tracks": list(tracks.values()),
        "provider_calls": provider_events,
        "sensor_preflight": preflight,
        "operator_no_contact_observation": no_contact_observation,
        "controller_events": controller_events,
        "checkpoint_states": checkpoint_states,
        "run_evidence": dict(run_evidence),
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
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
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
