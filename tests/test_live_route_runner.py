from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from sphero_rvr_driver.collision_stop import CollisionState, CollisionStopConfig, ScanInput, Transform2D
from sphero_rvr_driver.live_route_runner import (
    LiveRouteConfig,
    LiveRouteRequest,
    LiveRouteRunner,
    LiveRouteState,
    RouteSegmentRequest,
    route_request_from_json,
    run_route_replay,
)
from sphero_rvr_driver.mission_api_v2 import ToolResultStatus
from sphero_rvr_driver.odometry import MotionPrimitiveConfig, OdomMotionState

REPO_ROOT = Path(__file__).resolve().parents[1]


def _full_scan(stamp: float, *, range_m: float = 1.5, transform: Transform2D | None = None) -> ScanInput:
    return ScanInput(
        ranges=tuple([range_m] * 360),
        angle_min=-math.pi,
        angle_increment=math.tau / 360.0,
        range_min=0.05,
        range_max=6.0,
        stamp=stamp,
        received_at=stamp,
        frame_id="laser" if transform is not None else "base_link",
        transform_to_base=transform,
    )


def _state(
    stamp: float,
    x: float,
    y: float,
    yaw: float,
    *,
    scan: ScanInput | None = None,
    collision: str = "CLEAR",
    stop: bool = False,
    estop: bool = False,
    cancel: bool = False,
    collision_received_at: float | None = None,
) -> LiveRouteState:
    return LiveRouteState(
        stamp=stamp,
        odom=OdomMotionState(stamp=stamp, x_m=x, y_m=y, yaw_rad=yaw),
        scan=scan or _full_scan(stamp),
        collision_state=collision,
        collision_received_at=stamp if collision_received_at is None else collision_received_at,
        stop=stop,
        estop=estop,
        cancel=cancel,
    )


def _route() -> LiveRouteRequest:
    return LiveRouteRequest(
        route_id="route-72in",
        max_runtime_s=60.0,
        max_travel_m=1.8288,
        source_sha="test-sha",
        segments=(
            RouteSegmentRequest("move-1", "move_distance", {"distance_m": 0.6096, "speed_mps": 0.10, "timeout_s": 10.0}),
            RouteSegmentRequest("turn-1", "turn_angle", {"angle_deg": 90.0, "angular_speed_deg_s": 45.0, "timeout_s": 6.0}),
            RouteSegmentRequest("move-2", "move_distance", {"distance_m": 0.6096, "speed_mps": 0.10, "timeout_s": 10.0}),
            RouteSegmentRequest("turn-2", "turn_angle", {"angle_deg": -90.0, "angular_speed_deg_s": 45.0, "timeout_s": 6.0}),
            RouteSegmentRequest("move-3", "move_distance", {"distance_m": 0.6096, "speed_mps": 0.10, "timeout_s": 10.0}),
        ),
    )


def test_live_route_request_parses_mission_api_v2_invocations_with_budgets() -> None:
    payload = json.dumps(
        {
            "plan": {
                "plan_id": "operator-approved-route",
                "goal": {"budgets": {"max_runtime_s": 30.0, "max_travel_m": 0.5}},
                "invocations": [
                    {
                        "correlation_id": "move-a",
                        "tool_id": "move_distance",
                        "tool_version": "1.0",
                        "arguments": {"distance_m": 0.25, "speed_mps": 0.1, "timeout_s": 5.0},
                    },
                    {
                        "correlation_id": "turn-a",
                        "tool_id": "turn_angle",
                        "tool_version": "1.0",
                        "arguments": {"angle_deg": -45.0, "angular_speed_deg_s": 30.0, "timeout_s": 4.0},
                    },
                ],
            }
        }
    )

    request = route_request_from_json(payload, source_sha="abc123")

    assert request.route_id == "operator-approved-route"
    assert request.max_runtime_s == 30.0
    assert request.max_travel_m == 0.5
    assert request.source_sha == "abc123"
    assert [segment.tool_id for segment in request.segments] == ["move_distance", "turn_angle"]


def test_dynamic_translation_cap_uses_tf_corrected_base_link_corridor() -> None:
    ranges = [2.0] * 181
    ranges[0] = 0.80
    scan = ScanInput(
        ranges=tuple(ranges),
        angle_min=-math.pi / 2.0,
        angle_increment=math.pi / 180.0,
        range_min=0.05,
        range_max=6.0,
        stamp=10.0,
        received_at=10.0,
        frame_id="laser",
        transform_to_base=Transform2D(yaw=math.pi / 2.0),
    )
    runner = LiveRouteRunner(
        LiveRouteConfig(
            scan=CollisionStopConfig(min_valid_ranges=1, min_valid_fraction=0.0, sector_unknown_policy="open"),
            clearance_margin_m=0.40,
        )
    )

    cap = runner.dynamic_translation_cap(_state(10.0, 0.0, 0.0, 0.0, scan=scan), direction=1.0)

    assert cap == pytest.approx(0.40)


def test_live_route_replay_completes_72_inches_with_three_translations_two_signed_turns_and_manifest() -> None:
    states = (
        _state(100.0, 0.0, 0.0, 0.0),
        _state(106.1, 0.6096, 0.0, 0.0),
        _state(108.1, 0.6096, 0.0, math.radians(90.0)),
        _state(114.2, 0.6096, 0.6096, math.radians(90.0)),
        _state(116.2, 0.6096, 0.6096, 0.0),
        _state(122.3, 1.2192, 0.6096, 0.0),
    )

    manifest = run_route_replay(_route(), states)

    assert manifest.status is ToolResultStatus.COMPLETE
    assert manifest.terminal_reason == "complete"
    assert manifest.measured_distance_m == pytest.approx(1.8288)
    assert len([s for s in manifest.executed_segments if s.tool_id == "move_distance"]) == 3
    assert len([s for s in manifest.executed_segments if s.tool_id == "turn_angle"]) == 2
    assert manifest.executed_segments[1].measured_angle_deg == pytest.approx(90.0)
    assert manifest.executed_segments[3].measured_angle_deg == pytest.approx(-90.0)
    payload = manifest.to_json_dict()
    assert payload["source_sha"] == "test-sha"
    assert "/cmd_vel_motor" not in str(payload)


@pytest.mark.parametrize(
    ("state_kwargs", "terminal", "status"),
    (
        ({"collision": CollisionState.STOPPED.value}, "collision_veto", ToolResultStatus.BLOCKED),
        ({"collision": CollisionState.SENSOR_STALE.value}, "collision_veto", ToolResultStatus.BLOCKED),
    ),
)
def test_live_route_runner_propagates_collision_supervisor_terminal_blocks(state_kwargs, terminal, status) -> None:
    states = (_state(1.0, 0.0, 0.0, 0.0), _state(1.1, 0.0, 0.0, 0.0, **state_kwargs))

    manifest = run_route_replay(_route(), states)

    assert manifest.terminal_reason == terminal
    assert manifest.status is status


def test_live_route_runner_fails_closed_before_start_when_collision_state_never_received() -> None:
    manifest = run_route_replay(_route(), (LiveRouteState(1.0, OdomMotionState(1.0, 0, 0, 0), _full_scan(1.0)),))

    assert manifest.terminal_reason == "missing_collision_state"
    assert manifest.status is ToolResultStatus.BLOCKED
    assert manifest.executed_segments == ()


def test_live_route_runner_fails_closed_when_collision_supervisor_goes_stale_after_clear() -> None:
    config = LiveRouteConfig(collision_state_max_age_s=0.30)

    manifest = run_route_replay(
        _route(),
        (
            _state(1.0, 0.0, 0.0, 0.0, collision_received_at=1.0),
            _state(1.4, 0.0, 0.0, 0.0, collision_received_at=1.0),
        ),
        config,
    )

    assert manifest.terminal_reason == "stale_collision_state"
    assert manifest.status is ToolResultStatus.BLOCKED


def test_live_route_runner_requires_new_route_request_after_collision_supervisor_restart() -> None:
    config = LiveRouteConfig(collision_state_max_age_s=0.30)
    runner = LiveRouteRunner(config)
    request = LiveRouteRequest(
        route_id="one-move",
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(RouteSegmentRequest("move", "move_distance", {"distance_m": 0.2, "speed_mps": 0.1, "timeout_s": 5.0}),),
    )

    first = runner.start(request, _state(1.0, 0.0, 0.0, 0.0, collision_received_at=0.0))
    recovered_without_request = runner.update(_state(1.1, 0.0, 0.0, 0.0, collision_received_at=1.1))

    assert first.linear_x == 0.0
    assert recovered_without_request.linear_x == 0.0
    assert runner.manifest().terminal_reason == "stale_collision_state"
    assert not runner.active

    runner.start(request, _state(2.0, 0.0, 0.0, 0.0, collision_received_at=2.0))
    restarted = runner.update(_state(2.1, 0.0, 0.0, 0.0, collision_received_at=2.1))

    assert restarted.linear_x > 0.0
    assert runner.active


@pytest.mark.parametrize("collision", ["", "BOGUS", CollisionState.STARTUP.value, CollisionState.SLOW.value])
def test_live_route_runner_treats_unknown_or_not_explicitly_clear_collision_state_as_missing(collision: str) -> None:
    manifest = run_route_replay(_route(), (_state(1.0, 0.0, 0.0, 0.0, collision=collision),))

    assert manifest.terminal_reason == "missing_collision_state"
    assert manifest.status is ToolResultStatus.BLOCKED


def test_live_route_runner_propagates_stop_estop_cancel_and_stale_data() -> None:
    request = LiveRouteRequest(
        route_id="one-move",
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(RouteSegmentRequest("move", "move_distance", {"distance_m": 0.2, "speed_mps": 0.1, "timeout_s": 5.0}),),
    )

    stopped = run_route_replay(request, (_state(1.0, 0.0, 0.0, 0.0), _state(1.1, 0.0, 0.0, 0.0, stop=True)))
    estopped = run_route_replay(request, (_state(1.0, 0.0, 0.0, 0.0), _state(1.1, 0.0, 0.0, 0.0, estop=True)))
    cancelled = run_route_replay(request, (_state(1.0, 0.0, 0.0, 0.0), _state(1.1, 0.0, 0.0, 0.0, cancel=True)))
    stale = run_route_replay(request, (LiveRouteState(1.0, OdomMotionState(0.0, 0, 0, 0), _full_scan(1.0), CollisionState.CLEAR.value, 1.0),))

    assert stopped.terminal_reason == "stopped"
    assert estopped.terminal_reason == "estopped"
    assert cancelled.terminal_reason == "cancelled"
    assert stale.terminal_reason == "stale_odom"


def test_live_route_runner_blocks_wrong_direction_progress_as_truthful_terminal_failure() -> None:
    request = LiveRouteRequest(
        route_id="wrong-way",
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(RouteSegmentRequest("turn", "turn_angle", {"angle_deg": -90.0, "angular_speed_deg_s": 45.0, "timeout_s": 6.0}),),
    )
    config = LiveRouteConfig(odom=MotionPrimitiveConfig(startup_grace_s=0.5, stall_timeout_s=0.4))

    manifest = run_route_replay(
        request,
        (_state(1.0, 0.0, 0.0, 0.0), _state(2.0, 0.0, 0.0, math.radians(80.0))),
        config,
    )

    assert manifest.terminal_reason == "wrong_direction"
    assert manifest.status is ToolResultStatus.FAILED


def test_live_route_runner_reports_no_progress_turn_as_stall_not_wrong_direction() -> None:
    request = LiveRouteRequest(
        route_id="turn-stall",
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(RouteSegmentRequest("turn", "turn_angle", {"angle_deg": -90.0, "angular_speed_deg_s": 45.0, "timeout_s": 6.0}),),
    )
    config = LiveRouteConfig(odom=MotionPrimitiveConfig(startup_grace_s=0.5, stall_timeout_s=0.4))

    manifest = run_route_replay(
        request,
        (_state(1.0, 0.0, 0.0, 0.0), _state(2.0, 0.0, 0.0, 0.0)),
        config,
    )

    assert manifest.terminal_reason == "stall"
    assert manifest.status is ToolResultStatus.FAILED


def test_live_route_runner_reports_translation_stall_distinct_from_wrong_direction_turn() -> None:
    request = LiveRouteRequest(
        route_id="stalled-translation",
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(RouteSegmentRequest("move", "move_distance", {"distance_m": 0.2, "speed_mps": 0.1, "timeout_s": 5.0}),),
    )
    config = LiveRouteConfig(odom=MotionPrimitiveConfig(startup_grace_s=0.5, stall_timeout_s=0.4))

    manifest = run_route_replay(
        request,
        (_state(1.0, 0.0, 0.0, 0.0), _state(2.0, 0.0, 0.0, 0.0)),
        config,
    )

    assert manifest.terminal_reason == "stall"
    assert manifest.status is ToolResultStatus.FAILED


def test_live_route_runner_blocks_after_partial_dynamic_cap_instead_of_completing_short_route() -> None:
    request = LiveRouteRequest(
        route_id="partial-cap",
        max_runtime_s=10.0,
        max_travel_m=0.6,
        segments=(RouteSegmentRequest("move", "move_distance", {"distance_m": 0.5, "speed_mps": 0.1, "timeout_s": 5.0}),),
    )
    config = LiveRouteConfig(clearance_margin_m=0.40)

    manifest = run_route_replay(
        request,
        (
            _state(1.0, 0.0, 0.0, 0.0, scan=_full_scan(1.0, range_m=0.65)),
            _state(3.5, 0.25, 0.0, 0.0, scan=_full_scan(3.5, range_m=0.65)),
        ),
        config,
    )

    assert manifest.terminal_reason == "unsafe_clearance"
    assert manifest.status is ToolResultStatus.BLOCKED
    assert manifest.measured_distance_m == pytest.approx(0.25)
    assert manifest.executed_segments[0].executed["distance_m"] == pytest.approx(0.25)


def test_segment_start_exception_terminally_deactivates_runner_with_manifest() -> None:
    request = LiveRouteRequest(
        route_id="too-close",
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(RouteSegmentRequest("move", "move_distance", {"distance_m": 0.2, "speed_mps": 0.1, "timeout_s": 5.0}),),
    )
    runner = LiveRouteRunner(LiveRouteConfig(clearance_margin_m=0.40, min_translation_cap_m=0.05))

    command = runner.start(request, _state(1.0, 0.0, 0.0, 0.0, scan=_full_scan(1.0, range_m=0.42)))

    assert command.linear_x == 0.0
    assert command.angular_z == 0.0
    assert not runner.active
    manifest = runner.manifest()
    assert manifest.status is ToolResultStatus.BLOCKED
    assert manifest.terminal_reason == "unsafe_clearance"
    assert manifest.executed_segments == ()


def test_live_route_node_is_installed_default_off_and_cannot_own_motor_or_serial_surfaces() -> None:
    setup_text = (REPO_ROOT / "setup.py").read_text()
    launch_text = (REPO_ROOT / "launch" / "supervised_rvr.launch.py").read_text()
    config_text = (REPO_ROOT / "config" / "live_route_runner.yaml").read_text()
    node_source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "live_route_runner_node.py").read_text()

    assert "live_route_runner = sphero_rvr_driver.live_route_runner_node:main" in setup_text
    assert "config/live_route_runner.yaml" in setup_text
    assert "start_live_route_runner" in launch_text
    assert 'default_value="false"' in launch_text
    assert "cmd_vel_topic: /cmd_vel" in config_text
    assert "collision_state_max_age_s: 0.30" in config_text
    assert "/cmd_vel_motor" not in config_text
    assert "Serial" not in node_source
    assert "cmd_vel_motor" not in node_source
    assert "create_publisher(Twist, self._supervisor_cmd_topic(), 10)" in node_source
    assert "cmd_vel_topic must remain /cmd_vel" in node_source
    assert "_normalize_exception_terminal" in node_source
    assert 'self._runner.abort("invalid_route"' not in node_source
    assert 'self._publish_zero_status("route_failed"' not in node_source
    assert "rejected non-finite command; publishing zero" in node_source
    assert "TransformListener" in node_source
    assert "live_route/cancel" in node_source
    assert "self._collision_state: Optional[str] = None" in node_source
    assert "self._collision_received_at: Optional[float] = None" in node_source
    assert "collision_received_at=self._collision_received_at" in node_source
    assert "CollisionState(str(state).upper()).value" in node_source
    assert setup_text.count("config/live_route_runner.yaml") == 1
