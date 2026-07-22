from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from sphero_rvr_driver.live_mission_service import (
    LiveRouteProgressExecutor,
    LiveStateCache,
    LiveStatusExecutor,
    live_executor_bindings,
    snapshot_evidence,
)
from sphero_rvr_driver.live_mission_service_node import (
    _collision_mapping,
    _json_mapping,
    _odom_mapping,
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
    assert evidence["motion_authority"] is False
    assert evidence["route_submission_enabled"] is False
    assert evidence["safety"]["browser_is_safety_authority"] is False


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


def test_ros_payload_parsers_accept_canonical_collision_text_and_reject_malformed_json() -> None:
    assert _collision_mapping("CLEAR reason=idle scan_age=0.1") == {
        "state": "CLEAR",
        "raw": "CLEAR reason=idle scan_age=0.1",
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


def test_user_services_are_no_motion_loopback_only_and_not_self_enabling() -> None:
    mission_unit = (REPO_ROOT / "systemd/user/rvr-mission-service.service").read_text()
    web_unit = (REPO_ROOT / "systemd/user/rvr-mission-web.service").read_text()
    installer = (REPO_ROOT / "scripts/install-rvr-mission-stack-services").read_text()
    environment = (REPO_ROOT / "config/mission-stack.env.example").read_text()

    assert "live_mission_service" in mission_unit
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
    assert 'source "$RVR_ROS_WORKSPACE/install/setup.bash"' in mission_unit
    assert 'source "$RVR_ROS_WORKSPACE/install/setup.bash"' in web_unit
    assert "replace-with-reviewed-source-sha" in environment
