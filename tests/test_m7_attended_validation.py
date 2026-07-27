from __future__ import annotations

import copy
import hashlib
import inspect
import json
import math
from pathlib import Path

import pytest

from sphero_rvr_driver.m7_attended_validation import (
    CLEANUP_AUDIT_SCHEMA,
    EXPECTED_CAMERA_PITCH_RAD,
    FIXED_SAFETY,
    GRAPH_AUDIT_SCHEMA,
    OBSERVATION_SCHEMA,
    OBSERVED_TOPICS,
    PLAN_SCHEMA,
    REPORT_SCHEMA,
    SESSION_SCHEMA,
    build_plan,
    build_session_template,
    capture_ros_observation,
    evaluate_session,
    generate_graph_audit,
    m7_3_evidence_sha256,
    main,
)


SOURCE_SHA = "a" * 40
ARTIFACT_SHA = "b" * 64


def _approval(gate: str, sequence: int) -> dict[str, object]:
    return {
        "gate": gate,
        "source_sha": SOURCE_SHA,
        "deployed_sha": SOURCE_SHA,
        "reviewed_sha": SOURCE_SHA,
        "approval_id": f"approval-{gate}-{sequence}",
        "approved_by": "jsperson",
        "approved_at_utc": f"2026-07-28T00:0{sequence}:00Z",
        "authority_owner": "live_mission_service",
        "proposal_digest": "e" * 64,
        "approval_event_sha256": "f" * 64,
        "explicit_motor_warning_acknowledged": True,
        "room": {
            "attended": True,
            "level_bounded": True,
            "stairs_ledges_dropoffs_absent": True,
            "negative_obstacle_sensing_available": False,
        },
        "limits": {
            "max_forward_mps": FIXED_SAFETY["max_forward_mps"],
            "max_angular_rad_s": FIXED_SAFETY["max_angular_rad_s"],
            "command_lease_max_s": FIXED_SAFETY["command_lease_max_s"],
        },
    }


def _graph(gate: str) -> dict[str, object]:
    commands = {
        "git_head": {
            "returncode": 0,
            "stdout": SOURCE_SHA + "\n",
            "stderr": "",
        },
        "git_status": {"returncode": 0, "stdout": "", "stderr": ""},
        "ros_nodes": {
            "returncode": 0,
            "stdout": (
                "/sphero_rvr_driver\n"
                "/lidar_collision_stop_supervisor\n"
                "/live_route_runner\n"
            ),
            "stderr": "",
        },
        "topic_info:/cmd_vel": {
            "returncode": 0,
            "stdout": (
                "Publisher count: 1\n"
                "Node name: live_route_runner\n"
                "Subscription count: 1\n"
                "Node name: lidar_collision_stop_supervisor\n"
            ),
            "stderr": "",
        },
        "topic_info:/cmd_vel_motor": {
            "returncode": 0,
            "stdout": (
                "Publisher count: 1\n"
                "Node name: lidar_collision_stop_supervisor\n"
                "Subscription count: 1\n"
                "Node name: sphero_rvr_driver\n"
            ),
            "stderr": "",
        },
        "topic_info:/nav2_cmd_vel_request": {
            "returncode": 1,
            "stdout": "",
            "stderr": "Unknown topic",
        },
        "serial_owner": {
            "returncode": 0,
            "stdout": " 1234",
            "stderr": "",
        },
    }
    return {
        "schema": GRAPH_AUDIT_SCHEMA,
        "source_sha": SOURCE_SHA,
        "gate": gate,
        "stage": "active",
        "passed": True,
        "checks": {
            "exact_source_sha": True,
            "source_checkout_clean": True,
            "expected_nodes_present": True,
            "exclusive_cmd_vel_publisher": True,
            "exclusive_motor_publisher": True,
            "driver_is_motor_subscriber": True,
            "correct_cmd_vel_owners": True,
            "correct_motor_owners": True,
            "nav2_private_publisher_absent_before_m7_5": True,
            "serial_owner_present": True,
        },
        "commands": commands,
    }


def _cleanup() -> dict[str, object]:
    commands: dict[str, object] = {
        "git_head": {
            "returncode": 0,
            "stdout": SOURCE_SHA + "\n",
            "stderr": "",
        },
        "git_status": {"returncode": 0, "stdout": "", "stderr": ""},
        "processes": {
            "returncode": 0,
            "stdout": "1 /sbin/init\n",
            "stderr": "",
        },
        "ros_nodes": {
            "returncode": 0,
            "stdout": "/live_mission_service\n",
            "stderr": "",
        },
        "ros_topics": {
            "returncode": 0,
            "stdout": "/rosout\n",
            "stderr": "",
        },
    }
    for topic in ("/cmd_vel", "/cmd_vel_motor", "/nav2_cmd_vel_request"):
        commands[f"topic_info:{topic}"] = {
            "returncode": 1,
            "stdout": "",
            "stderr": "Unknown topic",
        }
    for device in ("/dev/rplidar", "/dev/ttyAMA0", "/dev/ttyS0", "/dev/serial0"):
        commands[f"device_owner:{device}"] = {
            "returncode": 1,
            "stdout": "",
            "stderr": "",
        }
    return {
        "schema": CLEANUP_AUDIT_SCHEMA,
        "source_sha": SOURCE_SHA,
        "passed": True,
        "checks": {
            "exact_source_sha": True,
            "source_checkout_clean": True,
            "process_inspection_succeeded": True,
            "stationary_sensor_and_motion_processes_absent": True,
            "ros_node_inspection_succeeded": True,
            "prohibited_ros_nodes_absent": True,
            "ros_topic_inspection_succeeded": True,
            "motion_topic_publishers_absent": True,
            "device_owner_inspection_succeeded": True,
            "sensor_and_rover_devices_ownerless": True,
        },
        "cleanup": {
            "camera_stopped": True,
            "lidar_stopped": True,
            "rosbag_stopped": True,
            "prohibited_nodes_absent": True,
            "rover_serial_owner_absent": True,
            "completed": True,
        },
        "commands": commands,
    }


def _canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode()
    return hashlib.sha256(payload).hexdigest()


def _collect_evidence_ids(value: object) -> set[str]:
    if isinstance(value, dict):
        result: set[str] = set()
        for key, item in value.items():
            if key in {"evidence_event_ids", "evidence_ids"} and isinstance(
                item, list
            ):
                result.update(str(event_id) for event_id in item)
            else:
                result.update(_collect_evidence_ids(item))
        return result
    if isinstance(value, list):
        result = set()
        for item in value:
            result.update(_collect_evidence_ids(item))
        return result
    return set()


def _observation(gate: str, evidence_ids: set[str]) -> dict[str, object]:
    required_keys = (
        (
            "collision_state",
            "collision_events",
            "requested_cmd",
            "motor_cmd",
            "odom",
            "scan",
            "route_status",
        )
        if gate == "m7.3"
        else tuple(OBSERVED_TOPICS)
    )
    ordered_ids = sorted(evidence_ids)
    present_topic_keys = {
        event_id.rsplit("--", 1)[1]
        for event_id in ordered_ids
        if "--" in event_id and event_id.rsplit("--", 1)[1] in OBSERVED_TOPICS
    }
    for key in required_keys:
        if key not in present_topic_keys:
            ordered_ids.append(f"{gate}-required-{key}--{key}")
    events = []
    for index, event_id in enumerate(ordered_ids):
        topic_key = event_id.rsplit("--", 1)[1]
        payload = {"evidence_id": event_id}
        events.append(
            {
                "event_id": event_id,
                "topic": OBSERVED_TOPICS[topic_key],
                "receipt_time_s": 1_000.0 + index * 0.01,
                "elapsed_s": index * 0.01,
                "payload": payload,
                "payload_sha256": _canonical_sha256(payload),
            }
        )
    observation: dict[str, object] = {
        "schema": OBSERVATION_SCHEMA,
        "source_sha": SOURCE_SHA,
        "gate": gate,
        "captured_at_utc": "2026-07-28T00:00:00Z",
        "duration_s": 120.0,
        "read_only": True,
        "motion_authority": False,
        "physical_execution_enabled": False,
        "topics": dict(OBSERVED_TOPICS),
        "events": events,
    }
    observation["observation_sha256"] = _canonical_sha256(observation)
    return observation


def _event_ids(gate: str, label: str, *topic_keys: str) -> list[str]:
    return [f"{gate}-{label}--{topic_key}" for topic_key in topic_keys]


def _artifact(
    gate: str, *, observation_sha256: str
) -> list[dict[str, object]]:
    return [
        {
            "path": f"/home/jsperson/rvr_runs/{gate}/data_0.mcap",
            "sha256": ARTIFACT_SHA,
            "byte_count": 1024,
        },
        {
            "path": f"/home/jsperson/rvr_runs/{gate}/observation.json",
            "sha256": "c" * 64,
            "byte_count": 512,
            "canonical_sha256": observation_sha256,
        },
    ]


def _motion_sample(
    t_s: float,
    *,
    requested: float = 0.08,
    motor: float = 0.08,
    state: str = "CLEAR",
    front_m: float = 1.0,
) -> dict[str, object]:
    return {
        "t_s": t_s,
        "requested_linear_x": requested,
        "requested_angular_z": 0.0,
        "motor_linear_x": motor,
        "motor_angular_z": 0.0,
        "state": state,
        "front_m": front_m,
        "physical_contact": False,
        "evidence_event_ids": _event_ids(
            "m7.3",
            f"motion-{t_s:.2f}",
            "collision_state",
            "requested_cmd",
            "motor_cmd",
            "scan",
        ),
    }


def _collision() -> dict[str, object]:
    result = {
        "graph_audit": _graph("m7.3"),
        "physical_contact_observed": False,
        "trials": {
            "slow": {
                "samples": [
                    _motion_sample(0.0),
                    _motion_sample(0.1, motor=0.04, state="SLOW", front_m=0.48),
                ]
            },
            "collision_stop": {
                "provider_inference_in_flight": True,
                "evidence_event_ids": _event_ids(
                    "m7.3",
                    "collision-stop",
                    "collision_events",
                    "route_status",
                ),
                "samples": [
                    _motion_sample(1.0, front_m=0.40),
                    _motion_sample(
                        1.20,
                        state="STOPPED",
                        front_m=0.34,
                    ),
                    _motion_sample(
                        1.24,
                        motor=0.0,
                        state="STOPPED",
                        front_m=0.34,
                    ),
                ],
            },
            "blocked_reset": {
                "accepted": False,
                "state": "STOPPED",
                "front_m": 0.40,
                "evidence_event_ids": _event_ids(
                    "m7.3",
                    "reset-blocked",
                    "collision_state",
                    "collision_events",
                ),
            },
            "clear_reset": {
                "accepted": True,
                "clear_duration_s": 0.60,
                "front_m": 0.50,
                "evidence_event_ids": _event_ids(
                    "m7.3",
                    "reset-clear",
                    "collision_state",
                    "collision_events",
                ),
                "samples_after_reset": [
                    _motion_sample(2.0, requested=0.0, motor=0.0),
                    _motion_sample(2.1, requested=0.0, motor=0.0),
                ],
            },
            "stale_command": {
                "zero_latency_s": 0.28,
                "motor_zero": True,
                "driver_watchdog_s": 0.50,
                "provider_inference_in_flight": True,
                "evidence_event_ids": _event_ids(
                    "m7.3",
                    "stale-command",
                    "requested_cmd",
                    "motor_cmd",
                    "route_status",
                ),
            },
            "operator_stop": {
                "zero_latency_s": 0.04,
                "motor_zero": True,
                "provider_inference_in_flight": True,
                "evidence_event_ids": _event_ids(
                    "m7.3",
                    "operator-stop",
                    "collision_events",
                    "motor_cmd",
                    "route_status",
                ),
            },
            "operator_estop": {
                "zero_latency_s": 0.03,
                "motor_zero": True,
                "provider_inference_in_flight": True,
                "latched_until_explicit_clear": True,
                "evidence_event_ids": _event_ids(
                    "m7.3",
                    "operator-estop",
                    "collision_events",
                    "motor_cmd",
                    "route_status",
                ),
            },
            "restart_recovery": {
                "pre_restart_state": "RUNNING",
                "post_restart_state": "recovery_required",
                "motor_zero": True,
                "route_resumed": False,
                "evidence_event_ids": _event_ids(
                    "m7.3",
                    "restart-recovery",
                    "motor_cmd",
                    "route_status",
                ),
            },
        },
        "cleanup_audit": _cleanup(),
    }
    observation = _observation("m7.3", _collect_evidence_ids(result))
    result["observation"] = observation
    result["artifacts"] = _artifact(
        "m7.3",
        observation_sha256=str(observation["observation_sha256"]),
    )
    return result


def _pitch(pitch_rad: float, error_m: float) -> dict[str, object]:
    return {
        "method": "surveyed_floor_contact_sweep",
        "camera_pitch_rad": pitch_rad,
        "far_floor_error_m": error_m,
        "target_range_m": 0.95,
        "tolerance_widened": False,
        "artifact_sha256": "d" * 64,
    }


def _detection(method: str = "floor_projection") -> dict[str, object]:
    return {
        "method": method,
        "point": (
            None
            if method == "bearing_only"
            else {"x": 0.72, "y": 0.12, "frame": "map"}
        ),
        "uncertainty": {
            "position_sigma_m": None if method == "bearing_only" else 0.04,
            "bearing_sigma_rad": 0.02,
        },
        "evidence_ids": _event_ids(
            "m7.4",
            "mapped-detection",
            "camera",
            "scan",
            "localization",
        ),
        "source_timestamps_ns": {
            "image": 2_000_000_000,
            "lidar": 2_050_000_000,
            "pose": 2_010_000_000,
        },
        "calibration_id": "rvr-pi-camera3-800x600",
        "map_revision": "slam-map-12",
    }


def _perception_sample(
    index: int,
    *,
    motor: float,
    tracks: list[dict[str, object]],
    detections: list[dict[str, object]],
) -> dict[str, object]:
    return {
        "t_s": 10.0 + index * 0.2,
        "requested_linear_x": motor,
        "requested_angular_z": 0.0,
        "motor_linear_x": motor,
        "motor_angular_z": 0.0,
        "state": "CLEAR",
        "front_m": 1.2,
        "physical_contact": False,
        "evidence_event_ids": _event_ids(
            "m7.4",
            f"moving-sample-{index}",
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
        ),
        "freshness_s": {
            "lidar": 0.04,
            "camera": 0.20,
            "localization": 0.06,
            "map": 0.14,
        },
        "transforms": {
            "map_to_base_link": True,
            "base_link_to_camera": True,
            "base_link_to_lidar": True,
        },
        "localization": {
            "state": "valid",
            "source": "slam_toolbox_moving",
            "pose": {"x": index * 0.03, "y": 0.0, "yaw": 0.0},
        },
        "camera_calibrated": True,
        "map_revision": f"slam-map-{index}",
        "localized_detections": detections,
        "tracks": tracks,
    }


def _moving_perception() -> dict[str, object]:
    stable_track = {
        "track_id": "object-0001",
        "observation_count": 2,
        "label": "shoe",
    }
    samples = [
        _perception_sample(0, motor=0.0, tracks=[], detections=[]),
        _perception_sample(
            1,
            motor=0.08,
            tracks=[stable_track],
            detections=[_detection()],
        ),
        _perception_sample(
            2,
            motor=0.08,
            tracks=[stable_track],
            detections=[_detection()],
        ),
        _perception_sample(
            3,
            motor=0.06,
            tracks=[stable_track],
            detections=[_detection("bearing_only")],
        ),
        _perception_sample(
            4,
            motor=0.0,
            tracks=[stable_track],
            detections=[_detection()],
        ),
    ]
    result = {
        "graph_audit": _graph("m7.4"),
        "pitch_checks": {
            "before": _pitch(EXPECTED_CAMERA_PITCH_RAD, 0.040),
            "after": _pitch(EXPECTED_CAMERA_PITCH_RAD + math.radians(0.1), 0.043),
        },
        "samples": samples,
        "replan_events": [
            {
                "trigger": "new_stable_detection",
                "track_id": "object-0001",
                "map_revision": "slam-map-1",
                "replan_required": True,
                "contains_motion_geometry": False,
                "evidence_event_ids": _event_ids(
                    "m7.4",
                    "new-track-replan",
                    "semantic_map",
                    "route_status",
                ),
            }
        ],
        "stale_veto": {
            "source": "localization",
            "motor_zero": True,
            "zero_latency_s": 0.08,
            "provider_inference_in_flight": True,
            "evidence_event_ids": _event_ids(
                "m7.4",
                "stale-localization",
                "localization",
                "motor_cmd",
                "route_status",
            ),
        },
        "cleanup_audit": _cleanup(),
    }
    observation = _observation("m7.4", _collect_evidence_ids(result))
    result["observation"] = observation
    result["artifacts"] = _artifact(
        "m7.4",
        observation_sha256=str(observation["observation_sha256"]),
    )
    return result


def _complete_session() -> dict[str, object]:
    session = build_session_template(source_sha=SOURCE_SHA)
    session["provenance"]["environment"] = {
        "hostname": "sphero-pi-2",
        "platform": "Linux-aarch64",
        "ros_distro": "jazzy",
        "python_version": "3.12.3",
        "operator": "jsperson",
    }
    m7_3_approval = _approval("m7.3", 1)
    session["authority"]["approvals"] = [m7_3_approval]
    session["m7_3_collision"] = _collision()
    m7_4_approval = _approval("m7.4", 2)
    m7_4_approval["m7_3_evidence_sha256"] = m7_3_evidence_sha256(
        source_sha=SOURCE_SHA,
        environment=session["provenance"]["environment"],
        approval=m7_3_approval,
        collision=session["m7_3_collision"],
    )
    session["authority"]["approvals"].append(m7_4_approval)
    session["m7_4_moving_perception"] = _moving_perception()
    return session


def test_complete_sequential_attended_session_passes_without_unlocking_later_phases() -> None:
    report = evaluate_session(_complete_session())

    assert report["schema"] == REPORT_SCHEMA
    assert report["passed"] is True
    assert all(report["checks"].values())
    assert report["m7_3_metrics"]["collision_zero_latency_s"] == pytest.approx(0.04)
    assert report["m7_4_metrics"]["moving_sample_count"] == 3
    assert report["m7_4_metrics"]["mapped_point_detection_count"] == 3
    assert report["scope"]["m7_5_physical_binding_approved"] is False
    assert report["scope"]["canonical_mission_approved"] is False
    assert report["scope"]["drop_off_detection_available"] is False


def test_m7_3_can_pass_independently_before_m7_4_is_approved() -> None:
    session = _complete_session()
    session["authority"]["approvals"] = session["authority"]["approvals"][:1]
    session["m7_4_moving_perception"] = build_session_template(
        source_sha=SOURCE_SHA
    )["m7_4_moving_perception"]

    report = evaluate_session(session, through_gate="m7.3")

    assert report["passed"] is True
    assert report["m7_3_evidence_sha256"]
    assert report["m7_4_metrics"] is None
    assert report["scope"]["m7_3_collision_gate"] == "passed"
    assert report["scope"]["m7_4_moving_perception_gate"] == "not_proven"


def test_template_fails_closed_until_both_physical_gates_are_completed() -> None:
    template = build_session_template(source_sha=SOURCE_SHA)

    assert template["schema"] == SESSION_SCHEMA
    report = evaluate_session(template)
    assert report["passed"] is False
    assert report["scope"]["m7_3_collision_gate"] == "not_proven"
    assert report["scope"]["m7_4_moving_perception_gate"] == "not_proven"


def test_m7_3_and_m7_4_require_separate_exact_sha_approvals() -> None:
    for mutation in (
        "missing",
        "same_id",
        "wrong_sha",
        "room",
        "wrong_owner",
        "invalid_digest",
    ):
        session = _complete_session()
        approvals = session["authority"]["approvals"]
        if mutation == "missing":
            approvals.pop()
        elif mutation == "same_id":
            approvals[1]["approval_id"] = approvals[0]["approval_id"]
        elif mutation == "wrong_sha":
            approvals[1]["reviewed_sha"] = "f" * 40
        elif mutation == "room":
            approvals[1]["room"]["stairs_ledges_dropoffs_absent"] = False
        elif mutation == "wrong_owner":
            approvals[1]["authority_owner"] = "unreviewed_source"
        else:
            approvals[1]["approval_event_sha256"] = "not-a-digest"
        report = evaluate_session(session)
        assert report["passed"] is False, mutation

    session = _complete_session()
    session["authority"]["approvals"][1]["m7_3_evidence_sha256"] = "0" * 64
    report = evaluate_session(session)
    assert report["passed"] is False
    assert "accepted M7.3 evidence" in report["error"]


def test_manifest_cannot_change_fixed_safety_or_motion_limits() -> None:
    session = _complete_session()
    session["fixed_safety"]["stop_distance_m"] = 0.20
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["authority"]["approvals"][0]["limits"]["max_forward_mps"] = 0.11
    assert evaluate_session(session)["passed"] is False


def test_collision_gate_requires_scaling_bounded_stop_latency_and_no_contact() -> None:
    session = _complete_session()
    session["m7_3_collision"]["trials"]["slow"]["samples"][1]["motor_linear_x"] = 0.08
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["m7_3_collision"]["trials"]["collision_stop"]["samples"][2]["t_s"] = 1.51
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["m7_3_collision"]["physical_contact_observed"] = True
    assert evaluate_session(session)["passed"] is False


def test_collision_reset_never_replays_old_command_and_estop_must_latch() -> None:
    session = _complete_session()
    session["m7_3_collision"]["trials"]["clear_reset"]["samples_after_reset"][0][
        "motor_linear_x"
    ] = 0.02
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["m7_3_collision"]["trials"]["operator_estop"][
        "latched_until_explicit_clear"
    ] = False
    assert evaluate_session(session)["passed"] is False


def test_all_independent_veto_trials_must_run_while_inference_is_in_flight() -> None:
    for trial in (
        "collision_stop",
        "stale_command",
        "operator_stop",
        "operator_estop",
    ):
        session = _complete_session()
        session["m7_3_collision"]["trials"][trial][
            "provider_inference_in_flight"
        ] = False
        assert evaluate_session(session)["passed"] is False, trial

    session = _complete_session()
    session["m7_4_moving_perception"]["stale_veto"][
        "provider_inference_in_flight"
    ] = False
    assert evaluate_session(session)["passed"] is False


def test_restart_enters_recovery_required_without_resuming_old_route() -> None:
    session = _complete_session()
    restart = session["m7_3_collision"]["trials"]["restart_recovery"]
    restart["post_restart_state"] = "RUNNING"
    restart["route_resumed"] = True

    report = evaluate_session(session)

    assert report["passed"] is False
    assert "recovery_required" in report["error"]


def test_moving_perception_requires_fresh_sources_transforms_and_real_motion() -> None:
    session = _complete_session()
    session["m7_4_moving_perception"]["samples"][1]["freshness_s"]["lidar"] = 0.31
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["m7_4_moving_perception"]["samples"][1]["transforms"][
        "map_to_base_link"
    ] = False
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    for sample in session["m7_4_moving_perception"]["samples"]:
        sample["motor_linear_x"] = 0.0
    assert evaluate_session(session)["passed"] is False


def test_moving_replan_is_recomputed_from_new_stable_track_not_manifest_claim() -> None:
    session = _complete_session()
    session["m7_4_moving_perception"]["samples"][1]["tracks"][0][
        "observation_count"
    ] = 1
    session["m7_4_moving_perception"]["samples"][2]["tracks"][0][
        "observation_count"
    ] = 1
    session["m7_4_moving_perception"]["samples"][3]["tracks"][0][
        "observation_count"
    ] = 1
    session["m7_4_moving_perception"]["samples"][4]["tracks"][0][
        "observation_count"
    ] = 1
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["m7_4_moving_perception"]["replan_events"][0][
        "contains_motion_geometry"
    ] = True
    assert evaluate_session(session)["passed"] is False


def test_bearing_only_moving_detection_never_contains_a_point() -> None:
    session = _complete_session()
    bearing = session["m7_4_moving_perception"]["samples"][3][
        "localized_detections"
    ][0]
    bearing["point"] = {"x": 1.0, "y": 2.0, "frame": "map"}

    report = evaluate_session(session)
    assert report["passed"] is False
    assert "bearing-only" in report["error"]


def test_camera_pitch_is_reverified_before_and_after_without_widening() -> None:
    session = _complete_session()
    session["m7_4_moving_perception"]["pitch_checks"]["after"][
        "camera_pitch_rad"
    ] = EXPECTED_CAMERA_PITCH_RAD + math.radians(0.6)
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["m7_4_moving_perception"]["pitch_checks"]["after"][
        "far_floor_error_m"
    ] = 0.051
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["m7_4_moving_perception"]["pitch_checks"]["before"][
        "tolerance_widened"
    ] = True
    assert evaluate_session(session)["passed"] is False


def test_raw_bag_checksums_and_generated_cleanup_are_mandatory() -> None:
    session = _complete_session()
    session["m7_3_collision"]["artifacts"] = [
        {
            "path": "summary.json",
            "sha256": ARTIFACT_SHA,
            "byte_count": 1,
        }
    ]
    assert evaluate_session(session)["passed"] is False

    session = _complete_session()
    session["m7_4_moving_perception"]["cleanup_audit"]["checks"][
        "motion_topic_publishers_absent"
    ] = False
    assert evaluate_session(session)["passed"] is False


def test_compact_evidence_is_bound_to_hashed_read_only_observer_events() -> None:
    session = _complete_session()
    event = session["m7_3_collision"]["observation"]["events"][0]
    event["payload"] = {"tampered": True}
    report = evaluate_session(session)
    assert report["passed"] is False
    assert "observation digest" in report["error"]

    session = _complete_session()
    session["m7_4_moving_perception"]["samples"][1]["evidence_event_ids"] = [
        "not-in-observation"
    ]
    report = evaluate_session(session)
    assert report["passed"] is False
    assert "outside the bound observation" in report["error"]

    session = _complete_session()
    sample = session["m7_4_moving_perception"]["samples"][1]
    sample["evidence_event_ids"] = [
        event_id
        for event_id in sample["evidence_event_ids"]
        if event_id.endswith("--requested_cmd")
    ]
    report = evaluate_session(session)
    assert report["passed"] is False
    assert "every required source topic" in report["error"]

    session = _complete_session()
    session["m7_4_moving_perception"]["artifacts"][1][
        "canonical_sha256"
    ] = "0" * 64
    report = evaluate_session(session)
    assert report["passed"] is False
    assert "observation artifact" in report["error"]


def test_plan_is_nonexecuting_and_preserves_two_approval_boundaries() -> None:
    plan = build_plan(source_sha=SOURCE_SHA)

    assert plan["schema"] == PLAN_SCHEMA
    assert plan["executes_commands"] is False
    assert plan["motion_authority"] is False
    assert plan["physical_execution_enabled"] is False
    assert [item["gate"] for item in plan["approval_boundaries"]] == [
        "m7.3",
        "m7.4",
    ]
    assert plan["observer"]["publishers"] == []
    assert plan["observer"]["services"] == []
    assert plan["observer"]["topics"] == OBSERVED_TOPICS


def _audit_runner(active: bool):
    def run(argv: list[str], *, timeout_s: float) -> dict[str, object]:
        del timeout_s
        text = ""
        returncode = 0
        if argv[-2:] == ["rev-parse", "HEAD"]:
            text = SOURCE_SHA + "\n"
        elif argv[-2:] == ["status", "--porcelain"]:
            text = ""
        elif argv[:4] == ["ros2", "node", "list", "--no-daemon"]:
            if active:
                text = (
                    "/sphero_rvr_driver\n"
                    "/lidar_collision_stop_supervisor\n"
                    "/live_route_runner\n"
                )
            else:
                text = "/live_mission_service\n"
        elif argv[:4] == ["ros2", "topic", "info", "-v"]:
            topic = argv[-1]
            if not active:
                returncode = 1
                text = ""
            elif topic == "/cmd_vel":
                text = (
                    "Publisher count: 1\n"
                    "Node name: live_route_runner\n"
                    "Subscription count: 1\n"
                    "Node name: lidar_collision_stop_supervisor\n"
                )
            elif topic == "/cmd_vel_motor":
                text = (
                    "Publisher count: 1\n"
                    "Node name: lidar_collision_stop_supervisor\n"
                    "Subscription count: 1\n"
                    "Node name: sphero_rvr_driver\n"
                )
            else:
                returncode = 1
        elif argv[0] == "fuser":
            if active:
                text = " 1234"
            else:
                returncode = 1
        return {
            "argv": argv,
            "returncode": returncode,
            "stdout": text,
            "stderr": "",
        }

    return run


def test_graph_audit_distinguishes_no_motion_preflight_from_active_ownership(
    tmp_path: Path,
) -> None:
    preflight = generate_graph_audit(
        source_sha=SOURCE_SHA,
        source_repo=tmp_path,
        gate="m7.3",
        stage="preflight",
        command_runner=_audit_runner(False),
    )
    active = generate_graph_audit(
        source_sha=SOURCE_SHA,
        source_repo=tmp_path,
        gate="m7.4",
        stage="active",
        command_runner=_audit_runner(True),
    )

    assert preflight["passed"] is True
    assert preflight["checks"]["motor_nodes_absent"] is True
    assert active["passed"] is True
    assert active["checks"]["exclusive_cmd_vel_publisher"] is True
    assert active["checks"]["exclusive_motor_publisher"] is True
    assert active["checks"]["nav2_private_publisher_absent_before_m7_5"] is True
    assert active["motion_authority"] is False


def test_graph_audit_claims_are_recomputed_from_raw_command_output() -> None:
    session = _complete_session()
    command = session["m7_3_collision"]["graph_audit"]["commands"][
        "topic_info:/cmd_vel"
    ]
    command["stdout"] = str(command["stdout"]).replace(
        "Publisher count: 1", "Publisher count: 2"
    )

    report = evaluate_session(session)

    assert report["passed"] is False
    assert "exclusive_cmd_vel_publisher" in report["error"]

    session = _complete_session()
    command = session["m7_3_collision"]["graph_audit"]["commands"][
        "topic_info:/cmd_vel"
    ]
    command["stdout"] = str(command["stdout"]).replace(
        "Node name: live_route_runner", "Node name: rogue_publisher", 1
    )
    report = evaluate_session(session)
    assert report["passed"] is False
    assert "correct_cmd_vel_owners" in report["error"]


def test_cleanup_is_recomputed_from_raw_process_topic_and_device_inspections() -> None:
    session = _complete_session()
    device = session["m7_3_collision"]["cleanup_audit"]["commands"][
        "device_owner:/dev/ttyAMA0"
    ]
    device["returncode"] = 0
    device["stderr"] = " 4242"

    report = evaluate_session(session)

    assert report["passed"] is False
    assert "sensor_and_rover_devices_ownerless" in report["error"]


def test_observer_source_has_subscriptions_only_and_no_authority_surfaces() -> None:
    source = inspect.getsource(capture_ros_observation)

    assert "create_subscription" in source
    assert "create_publisher" not in source
    assert "create_service" not in source
    assert "create_client" not in source
    assert "serial.Serial" not in source
    assert '"/dev/' not in source
    assert "open(" not in source
    assert '"motion_authority": False' in source
    assert '"physical_execution_enabled": False' in source


def test_cli_writes_plan_template_and_passing_report(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    plan_path = tmp_path / "plan.json"
    template_path = tmp_path / "template.json"
    session_path = tmp_path / "session.json"
    report_path = tmp_path / "report.json"

    assert main(["plan", "--source-sha", SOURCE_SHA, "--output", str(plan_path)]) == 0
    assert main(
        ["template", "--source-sha", SOURCE_SHA, "--output", str(template_path)]
    ) == 0
    session_path.write_text(json.dumps(_complete_session()))
    assert main(["evaluate", str(session_path), "--output", str(report_path)]) == 0

    assert json.loads(plan_path.read_text())["executes_commands"] is False
    assert json.loads(template_path.read_text())["schema"] == SESSION_SCHEMA
    assert json.loads(report_path.read_text())["passed"] is True
    assert REPORT_SCHEMA in capsys.readouterr().out
