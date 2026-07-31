from __future__ import annotations

import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.live_mission_service import (
    LiveRouteProgressExecutor,
    LiveStateCache,
    LiveStatusExecutor,
    SafetyGatedPromptRouteExecutor,
    live_executor_bindings,
    snapshot_evidence,
)
from sphero_rvr_driver.live_mission_service_node import (
    _collision_mapping,
    _control_mapping,
    _json_mapping,
    _localization_mapping,
    _odom_mapping,
    _validated_execution_gate,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


def _status_executor(cache: LiveStateCache, *, max_age_s: float = 1.0) -> LiveStatusExecutor:
    return LiveStatusExecutor(
        cache,
        source_sha="reviewed-source-sha",
        deployed_sha="deployed-package-sha",
        max_source_age_s=max_age_s,
    )


def test_live_cache_reports_missing_fresh_stale_and_invalid_sources_truthfully() -> None:
    cache = LiveStateCache()
    missing = snapshot_evidence(cache.snapshot(now_s=100.0), max_age_s=1.0)
    assert missing["odom"] == {
        "present": False,
        "valid": False,
        "fresh": False,
        "age_s": None,
        "received_at_s": None,
        "source_timestamp_s": None,
        "error": "source has not been observed",
        "value": {},
    }

    cache.update(
        "odom",
        {"x_m": 0.25, "y_m": -0.1, "heading_deg": 4.0},
        received_at_s=100.0,
        source_timestamp_s=99.9,
    )
    fresh = snapshot_evidence(cache.snapshot(now_s=100.5), max_age_s=1.0)["odom"]
    assert fresh["present"] is True
    assert fresh["valid"] is True
    assert fresh["fresh"] is True
    assert fresh["age_s"] == pytest.approx(0.5)

    stale = snapshot_evidence(cache.snapshot(now_s=102.0), max_age_s=1.0)["odom"]
    assert stale["fresh"] is False
    assert stale["age_s"] == pytest.approx(2.0)

    cache.mark_invalid("odom", "non-finite heading", received_at_s=103.0)
    invalid = snapshot_evidence(cache.snapshot(now_s=103.1), max_age_s=1.0)["odom"]
    assert invalid["present"] is True
    assert invalid["valid"] is False
    assert invalid["fresh"] is False
    assert invalid["error"] == "non-finite heading"


def test_live_cache_rejects_unknown_sources_nonfinite_times_and_payloads() -> None:
    cache = LiveStateCache()
    with pytest.raises(ValueError, match="unsupported live source"):
        cache.update("motors", {}, received_at_s=1.0)
    with pytest.raises(ValueError, match="receipt time must be finite"):
        cache.update("odom", {}, received_at_s=float("nan"))
    with pytest.raises(ValueError, match="non-finite"):
        cache.update("odom", {"x_m": float("inf")}, received_at_s=1.0)
    with pytest.raises(ValueError, match="positive and finite"):
        snapshot_evidence(cache.snapshot(now_s=1.0), max_age_s=0.0)


def test_read_only_status_surfaces_provenance_pose_route_safety_and_no_authority() -> None:
    cache = LiveStateCache()
    cache.update("odom", {"x_m": 1.0, "y_m": 2.0, "heading_deg": 12.5}, received_at_s=10.0)
    cache.update("collision", {"state": "CLEAR"}, received_at_s=10.0)
    cache.update(
        "route_progress",
        {"status": "running", "progress": 0.4, "completed_segments": 1},
        received_at_s=10.0,
    )
    executor = _status_executor(cache)

    evidence = executor.evidence(now_s=10.2)

    assert evidence["source_sha"] == "reviewed-source-sha"
    assert evidence["deployed_sha"] == "deployed-package-sha"
    assert evidence["status_state"] == "NO_MOTION_MONITORING"
    assert evidence["odom"]["value"]["heading_deg"] == 12.5
    assert evidence["route_progress"]["value"]["progress"] == 0.4
    assert evidence["safety"]["collision_state"] == "CLEAR"
    assert evidence["safety"]["stop_state"] == "READY"
    assert evidence["safety"]["estop_state"] == "CLEAR"
    assert evidence["motion_authority"] is False
    assert evidence["route_submission_enabled"] is False
    assert evidence["safety"]["browser_is_safety_authority"] is False


def test_missing_or_stale_stop_estop_evidence_is_unknown_not_ready() -> None:
    cache = LiveStateCache()
    missing = _status_executor(cache).evidence(now_s=10.0)["safety"]
    assert missing["stop_state"] == "UNKNOWN"
    assert missing["estop_state"] == "UNKNOWN"

    cache.update("control", {"state": "READY"}, received_at_s=1.0)
    stale = _status_executor(cache).evidence(now_s=10.0)["safety"]
    assert stale["stop_state"] == "UNKNOWN"
    assert stale["estop_state"] == "UNKNOWN"


def test_control_source_reports_stop_and_estop_truthfully() -> None:
    cache = LiveStateCache()
    cache.update("control", {"stop_active": True}, received_at_s=10.0)
    stopped = _status_executor(cache).evidence(now_s=10.1)
    assert stopped["status_state"] == "STOPPED"
    assert stopped["safety"]["stop_state"] == "ACTIVE"

    cache.update("control", {"estop_latched": True}, received_at_s=11.0)
    estopped = _status_executor(cache).evidence(now_s=11.1)
    assert estopped["status_state"] == "ESTOPPED"
    assert estopped["safety"]["estop_state"] == "LATCHED"


@pytest.mark.parametrize(
    ("collision_state", "route_status", "expected"),
    [
        ("CLEAR", "cancelled", "CANCELLED"),
        ("STOPPED", "running", "STOPPED"),
        ("CLEAR", "stopped", "STOPPED"),
        ("ESTOPPED", "running", "ESTOPPED"),
        ("CLEAR", "estopped", "ESTOPPED"),
    ],
)
def test_status_executor_reports_independent_terminal_safety_state(
    collision_state: str,
    route_status: str,
    expected: str,
) -> None:
    cache = LiveStateCache()
    cache.update("collision", {"state": collision_state}, received_at_s=1.0)
    cache.update("route_progress", {"status": route_status}, received_at_s=1.0)
    assert _status_executor(cache).evidence(now_s=1.0)["status_state"] == expected


def test_route_bindings_are_present_but_unhealthy_and_cannot_gain_authority() -> None:
    cache = LiveStateCache()
    status = _status_executor(cache)
    route = LiveRouteProgressExecutor(status)
    bindings = live_executor_bindings(status, route, heartbeat_at_s=10.0)

    assert set(bindings) == {"query_status_telemetry", "move_distance", "turn_angle"}
    assert bindings["query_status_telemetry"].executor.healthy is True
    for tool_id in ("move_distance", "turn_angle"):
        binding = bindings[tool_id]
        assert binding.executor.healthy is False
        assert binding.evidence["motion_authority"] is False
        assert binding.evidence["route_submission_enabled"] is False


class _RecordingPromptRouteExecutor:
    def __init__(self) -> None:
        self.requests = []
        self.cancelled = False

    def execute(self, request):
        self.requests.append(request)
        return {"status": "complete"}

    def cancel(self):
        self.cancelled = True
        return True


def test_safety_gated_prompt_executor_requires_fresh_clear_authority() -> None:
    cache = LiveStateCache()
    status = _status_executor(cache)
    delegate = _RecordingPromptRouteExecutor()
    executor = SafetyGatedPromptRouteExecutor(status, delegate)

    with pytest.raises(MissionValidationError, match="fresh authoritative odom"):
        executor.assert_ready()

    now = time.time()
    cache.update("odom", {"x_m": 0.0, "y_m": 0.0}, received_at_s=now)
    cache.update("collision", {"state": "STOPPED"}, received_at_s=now)
    with pytest.raises(MissionValidationError, match="collision state CLEAR"):
        executor.assert_ready()

    cache.update("collision", {"state": "CLEAR"}, received_at_s=now)
    evidence = executor.assert_ready()
    assert evidence["safety"]["stop_state"] == "READY"
    assert executor.execute("route") == {"status": "complete"}
    assert delegate.requests == ["route"]
    assert executor.cancel() is True
    assert delegate.cancelled is True


def test_live_execution_gate_requires_planner_and_exact_deployed_sha() -> None:
    assert _validated_execution_gate(
        enabled=False,
        reviewed_sha="",
        source_sha="source",
        deployed_sha="deployed",
        planning_enabled=True,
    ) is False
    assert _validated_execution_gate(
        enabled=True,
        reviewed_sha="deployed",
        source_sha="deployed",
        deployed_sha="deployed",
        planning_enabled=True,
    ) is True
    with pytest.raises(ValueError, match="requires the prompt planner"):
        _validated_execution_gate(
            enabled=True,
            reviewed_sha="deployed",
            source_sha="deployed",
            deployed_sha="deployed",
            planning_enabled=False,
        )
    with pytest.raises(ValueError, match="exactly match"):
        _validated_execution_gate(
            enabled=True,
            reviewed_sha="other",
            source_sha="deployed",
            deployed_sha="deployed",
            planning_enabled=True,
        )
    with pytest.raises(ValueError, match="source and deployed"):
        _validated_execution_gate(
            enabled=True,
            reviewed_sha="deployed",
            source_sha="source",
            deployed_sha="deployed",
            planning_enabled=True,
        )


def test_ros_payload_parsers_accept_canonical_collision_text_and_reject_malformed_json() -> None:
    assert _collision_mapping(
        "CLEAR reason=idle scan_age=0.1 front=1.27 front_slow=1.26 "
        "front_slow_min_angle_deg=-35 front_slow_max_angle_deg=35 "
        "stop_distance_m=0.35 slow_distance_m=0.6 "
        "rear=1.4 left=0.31 right=1.2 trajectory_clearance_margin_m=0.02 "
        "trajectory_horizon_s=0.75 trajectory_min_clearance_m=0.01 "
        "trajectory_collision_time_s=None"
    ) == {
            "state": "CLEAR",
        "raw": (
            "CLEAR reason=idle scan_age=0.1 front=1.27 front_slow=1.26 "
            "front_slow_min_angle_deg=-35 front_slow_max_angle_deg=35 "
            "stop_distance_m=0.35 slow_distance_m=0.6 "
            "rear=1.4 left=0.31 right=1.2 trajectory_clearance_margin_m=0.02 "
            "trajectory_horizon_s=0.75 trajectory_min_clearance_m=0.01 "
            "trajectory_collision_time_s=None"
            ),
            "reason": "idle",
            "scan_age_s": 0.1,
            "front_clearance_m": 1.27,
        "forward_corridor_clearance_m": 1.26,
        "forward_corridor_min_angle_deg": -35.0,
        "forward_corridor_max_angle_deg": 35.0,
        "collision_stop_distance_m": 0.35,
        "collision_slow_distance_m": 0.6,
        "rear_clearance_m": 1.4,
        "left_clearance_m": 0.31,
        "right_clearance_m": 1.2,
        "trajectory_clearance_margin_m": 0.02,
        "trajectory_horizon_s": 0.75,
        "trajectory_min_clearance_m": 0.01,
    }
    assert _collision_mapping('{"state":"STOPPED"}') == {"state": "STOPPED"}
    assert _json_mapping('{"status":"running","progress":0.5}') == {
        "status": "running",
        "progress": 0.5,
    }
    with pytest.raises(ValueError, match="JSON object"):
        _json_mapping("not-json")
    with pytest.raises(ValueError, match="JSON object"):
        _json_mapping("[]")


def test_control_parser_accepts_canonical_text_and_typed_json_but_rejects_ambiguous_state() -> None:
    assert _control_mapping("STOP reason=operator") == {
        "state": "STOP",
        "raw": "STOP reason=operator",
    }
    assert _control_mapping('{"state":"estopped"}') == {"state": "ESTOPPED"}
    assert _control_mapping('{"stop_active":false,"estop_latched":false}') == {
        "stop_active": False,
        "estop_latched": False,
    }
    with pytest.raises(ValueError, match="unsupported state"):
        _control_mapping("unknown")
    with pytest.raises(ValueError, match="lacks an authoritative state"):
        _control_mapping("{}")


def test_localization_parser_enforces_lidar_authority_and_no_motion_navigation_status() -> None:
    localization = {
        "state": "valid",
        "source": "lidar_scan_match",
        "map_id": "live-stationary-map",
        "stationary_session": True,
        "motion_authority": False,
        "physical_execution_enabled": False,
        "quality": 0.9,
        "covariance_xy_m2": 0.0025,
        "covariance_yaw_rad2": 0.001,
        "odom_translation_disagreement_m": 0.01,
        "odom_heading_disagreement_rad": 0.02,
        "pose": {
            "stamp_s": 12.5,
            "frame_id": "map",
            "x_m": 0.2,
            "y_m": -0.1,
            "yaw_rad": 0.3,
        },
    }
    parsed = _localization_mapping(json.dumps(localization))
    assert parsed["authoritative"] is True
    assert parsed["pose"]["heading_deg"] == pytest.approx(math.degrees(0.3))
    assert parsed["odom_translation_disagreement_m"] == pytest.approx(0.01)
    assert parsed["map_id"] == "live-stationary-map"
    assert parsed["stationary_session"] is True
    assert parsed["motion_authority"] is False
    assert parsed["physical_execution_enabled"] is False

    without_session_provenance = dict(localization)
    for name in (
        "map_id",
        "stationary_session",
        "motion_authority",
        "physical_execution_enabled",
    ):
        without_session_provenance.pop(name)
    normalized_without_provenance = _localization_mapping(
        json.dumps(without_session_provenance)
    )
    assert "stationary_session" not in normalized_without_provenance
    assert "motion_authority" not in normalized_without_provenance

    navigation = {
        "schema": "sphero_rvr.perception_navigation_result.v1",
        "localization": localization,
        "goal": {
            "frame_id": "map",
            "x_m": 0.5,
            "y_m": 0.0,
            "radius_m": 0.1,
            "minimum_clearance_m": 0.2,
            "max_runtime_s": 20.0,
            "max_cumulative_translation_m": 1.0,
            "heading_min_rad": None,
            "heading_max_rad": None,
        },
        "motion_authority": False,
        "physical_execution_enabled": False,
    }
    parsed_navigation = _localization_mapping(json.dumps(navigation))
    assert parsed_navigation["localization"]["authoritative"] is True
    assert parsed_navigation["goal"]["radius_m"] == pytest.approx(0.1)

    navigation["physical_execution_enabled"] = True
    with pytest.raises(ValueError, match="physical execution disabled"):
        _localization_mapping(json.dumps(navigation))


def test_odom_parser_exposes_heading_and_rejects_nonfinite_measurements() -> None:
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=500_000_000), frame_id="odom"),
        child_frame_id="base_link",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.5, y=-0.25),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(math.pi / 8), w=math.cos(math.pi / 8)),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.1),
                angular=SimpleNamespace(z=0.2),
            )
        ),
    )
    parsed = _odom_mapping(msg)
    assert parsed is not None
    assert parsed["stamp_s"] == pytest.approx(12.5)
    assert parsed["heading_deg"] == pytest.approx(45.0)

    msg.pose.pose.position.x = float("nan")
    assert _odom_mapping(msg) is None


def test_live_owner_source_has_no_motion_ros_or_serial_authority() -> None:
    source = (REPO_ROOT / "src/sphero_rvr_driver/live_mission_service_node.py").read_text()
    assert 'create_publisher(\n                String' in source
    assert '"/cmd_vel"' not in source
    assert '"/cmd_vel_motor"' not in source
    assert "serial" not in source.lower()
    assert "LiveRouteRequest" not in source
    assert "create_client" not in source
    assert "publish_route" not in source


def test_mission_service_launch_is_default_off_and_contains_no_motor_process() -> None:
    source = (REPO_ROOT / "launch/mission_service.launch.py").read_text()
    assert 'default_value="false"' in source
    assert 'executable="live_mission_service"' in source
    assert 'executable="rvr_node"' not in source
    assert 'executable="live_route_runner"' not in source
    assert 'executable="lidar_collision_stop_supervisor"' not in source


def test_user_services_keep_approval_activation_exact_sha_bound_and_default_off() -> None:
    mission_unit = (REPO_ROOT / "systemd/user/rvr-mission-service.service").read_text()
    web_unit = (REPO_ROOT / "systemd/user/rvr-mission-web.service").read_text()
    adaptive_unit = (
        REPO_ROOT / "systemd/user/rvr-adaptive-mission.service"
    ).read_text()
    installer = (REPO_ROOT / "scripts/install-rvr-mission-stack-services").read_text()
    environment = (REPO_ROOT / "config/mission-stack.env.example").read_text()

    assert "live_mission_service" in mission_unit
    assert "After=default.target" not in mission_unit
    assert "rvr_mission_web --mode live --host 127.0.0.1" in web_unit
    assert "ExecStartPre=" in web_unit
    assert '[[ -S "$HOME/.local/state/sphero_rvr/mission-service.sock" ]]' in web_unit
    assert '--public-origin "$RVR_WEB_ORIGIN"' in web_unit
    assert "UMask=0077" in mission_unit and "UMask=0077" in web_unit
    for source in (mission_unit, web_unit):
        assert "rvr_node" not in source
        assert "live_route_runner" not in source
        assert "lidar_collision_stop_supervisor" not in source
        assert "/cmd_vel" not in source
        assert "/dev/tty" not in source
    assert "enable --now" in installer
    assert "systemctl --user enable --now" not in "\n".join(
        line for line in installer.splitlines() if not line.startswith('echo ')
    )
    assert "OPENAI_API_KEY" not in environment
    assert "CODEX_API_KEY" not in environment
    assert "RVR_ROS_WORKSPACE=replace-with-absolute-mission-stack-workspace" in environment
    assert "RVR_LIVE_EXECUTION_ENABLED=false" in environment
    assert "RVR_LIVE_EXECUTION_REVIEWED_SHA=" in environment
    assert "RVR_APPROVAL_ACTIVATION_ENABLED=false" in environment
    assert "RVR_APPROVAL_ACTIVATION_REVIEWED_SHA=" in environment
    assert "RVR_APPROVAL_ACTIVATION_TIMEOUT_S=30.0" in environment
    assert "RVR_ADAPTIVE_MISSION_LEASE_S=900.0" in environment
    assert "RVR_ADAPTIVE_MISSION_ENABLED=false" in environment
    assert "RVR_PLANNING_MAX_MOTION_CALLS=3" in environment
    assert "RVR_PLANNING_MAX_TRANSLATION_M=0.5" in environment
    assert "RVR_PLANNING_MAX_TRANSLATION_PER_CALL_M=0.5" in environment
    assert "RVR_PLANNING_MAX_RUNTIME_S=45.0" in environment
    assert '${RVR_LIVE_EXECUTION_ENABLED:-false}' in mission_unit
    assert '${RVR_LIVE_EXECUTION_REVIEWED_SHA:-disabled}' in mission_unit
    assert '${RVR_APPROVAL_ACTIVATION_ENABLED:-false}' in mission_unit
    assert '${RVR_APPROVAL_ACTIVATION_REVIEWED_SHA:-disabled}' in mission_unit
    assert '${RVR_ADAPTIVE_MISSION_LEASE_S:-900.0}' in mission_unit
    assert '${RVR_ADAPTIVE_MISSION_ENABLED:-false}' in mission_unit
    assert '${RVR_PLANNING_MAX_MOTION_CALLS:-3}' in mission_unit
    mission_config = (REPO_ROOT / "config/mission_service.yaml").read_text()
    assert "live_execution_enabled: false" in mission_config
    assert "adaptive_mission_enabled: false" in mission_config
    assert "approval_activation_enabled: false" in mission_config
    assert "approval_activation_reviewed_sha:" in mission_config
    assert "approval_activation_timeout_s: 30.0" in mission_config
    assert "adaptive_mission_lease_s: 900.0" in mission_config
    assert "planning_max_motion_calls: 3" in mission_config
    assert "planning_max_translation_per_call_m: 0.5" in mission_config
    assert 'source "$RVR_ROS_WORKSPACE/install/setup.bash"' in mission_unit
    assert 'source "$RVR_ROS_WORKSPACE/install/setup.bash"' in web_unit
    assert "replace-with-reviewed-source-sha" in environment
    assert (
        '"${RVR_APPROVAL_ACTIVATION_ENABLED:-false}" == "true"'
        in adaptive_unit
    )
    assert (
        '"$RVR_SOURCE_SHA" == "$RVR_APPROVAL_ACTIVATION_REVIEWED_SHA"'
        in adaptive_unit
    )
    assert "WantedBy=default.target" in adaptive_unit
    assert "systemctl" not in adaptive_unit
