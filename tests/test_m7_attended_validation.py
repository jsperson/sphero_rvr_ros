from __future__ import annotations

import copy
import inspect
import json
import math
from pathlib import Path

import pytest

from sphero_rvr_driver.m7_attended_validation import (
    EXPECTED_CAMERA_PITCH_RAD,
    FIXED_SAFETY,
    GRAPH_AUDIT_SCHEMA,
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
    return {
        "source_sha": SOURCE_SHA,
        "passed": True,
        "checks": {
            "exact_source_sha": True,
            "source_checkout_clean": True,
            "stationary_sensor_and_motion_processes_absent": True,
            "prohibited_ros_nodes_absent": True,
            "motion_topic_publishers_absent": True,
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
    }


def _artifact(gate: str) -> list[dict[str, object]]:
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
        "evidence_event_ids": [f"event-{t_s:.2f}"],
    }


def _collision() -> dict[str, object]:
    return {
        "graph_audit": _graph("m7.3"),
        "physical_contact_observed": False,
        "artifacts": _artifact("m7.3"),
        "trials": {
            "slow": {
                "samples": [
                    _motion_sample(0.0),
                    _motion_sample(0.1, motor=0.04, state="SLOW", front_m=0.48),
                ]
            },
            "collision_stop": {
                "provider_inference_in_flight": True,
                "evidence_event_ids": ["provider-1", "collision-1"],
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
                "evidence_event_ids": ["reset-blocked-1"],
            },
            "clear_reset": {
                "accepted": True,
                "clear_duration_s": 0.60,
                "front_m": 0.50,
                "evidence_event_ids": ["reset-clear-1"],
                "samples_after_reset": [
                    _motion_sample(2.0, requested=0.0, motor=0.0),
                    _motion_sample(2.1, requested=0.0, motor=0.0),
                ],
            },
            "stale_command": {
                "zero_latency_s": 0.28,
                "motor_zero": True,
                "driver_watchdog_s": 0.50,
                "evidence_event_ids": ["stale-1", "motor-zero-1"],
            },
            "operator_stop": {
                "zero_latency_s": 0.04,
                "motor_zero": True,
                "provider_inference_in_flight": True,
                "evidence_event_ids": ["stop-service-1", "motor-zero-2"],
            },
            "operator_estop": {
                "zero_latency_s": 0.03,
                "motor_zero": True,
                "provider_inference_in_flight": True,
                "latched_until_explicit_clear": True,
                "evidence_event_ids": ["estop-service-1", "motor-zero-3"],
            },
        },
        "cleanup_audit": _cleanup(),
    }


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
        "evidence_ids": ["live-camera-00000012-shoe-01", "scan-0012"],
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
        "evidence_event_ids": [f"moving-event-{index}"],
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
    return {
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
                "evidence_event_ids": ["map-event-1", "track-event-1"],
            }
        ],
        "stale_veto": {
            "source": "localization",
            "motor_zero": True,
            "zero_latency_s": 0.08,
            "provider_inference_in_flight": True,
            "evidence_event_ids": ["stale-localization-1", "motor-zero-4"],
        },
        "artifacts": _artifact("m7.4"),
        "cleanup_audit": _cleanup(),
    }


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
    for trial in ("collision_stop", "operator_stop", "operator_estop"):
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
