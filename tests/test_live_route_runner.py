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
    TrackEncoderState,
    route_request_from_json,
    run_route_replay,
)
from sphero_rvr_driver.live_route_runner_node import _collision_state_value, _encoder_state
from sphero_rvr_driver.mission_api import ToolResultStatus
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
    encoder_counts: tuple[int, int] | None = None,
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
        encoder_counts=(
            None
            if encoder_counts is None
            else TrackEncoderState(stamp, encoder_counts[0], encoder_counts[1])
        ),
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


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        ("CLEAR reason=tick scan_age=0.1", "CLEAR"),
        ("SLOW reason=clearance", "SLOW"),
        ('{"state":"STOPPED","reason":"obstacle"}', "STOPPED"),
        ('{"collision_state":"ESTOPPED"}', "ESTOPPED"),
        ("unknown reason=bad", None),
        ("", None),
        (None, None),
    ],
)
def test_live_route_node_parses_collision_supervisor_state_token(raw, expected) -> None:
    assert _collision_state_value(raw) == expected


def test_live_route_request_parses_canonical_mission_api_invocations_with_budgets() -> None:
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


def test_manifest_distinguishes_route_local_progress_from_absolute_pose_and_track_counts() -> None:
    request = LiveRouteRequest(
        route_id="measured-route",
        segments=(
            RouteSegmentRequest(
                "move-1",
                "move_distance",
                {"distance_m": 0.10, "speed_mps": 0.08, "timeout_s": 4.0},
            ),
        ),
        max_runtime_s=10.0,
        max_travel_m=0.10,
        source_sha="measurement-sha",
    )
    start_yaw = math.radians(10.0)
    final_yaw = math.radians(12.0)
    states = (
        _state(1.0, 0.40, -0.20, start_yaw, encoder_counts=(1_000, 2_000)),
        _state(
            2.0,
            0.40 + 0.10 * math.cos(start_yaw),
            -0.20 + 0.10 * math.sin(start_yaw),
            final_yaw,
            encoder_counts=(1_430, 2_450),
        ),
    )

    manifest = run_route_replay(request, states)
    payload = manifest.to_json_dict()

    assert manifest.status is ToolResultStatus.COMPLETE
    assert manifest.measured_distance_m == pytest.approx(0.10)
    assert payload["route_start_pose"]["x_m"] == pytest.approx(0.40)
    assert payload["route_final_pose"]["heading_deg"] == pytest.approx(12.0)
    assert payload["route_delta_x_m"] == pytest.approx(0.10 * math.cos(start_yaw))
    assert payload["route_delta_y_m"] == pytest.approx(0.10 * math.sin(start_yaw))
    assert payload["route_displacement_m"] == pytest.approx(0.10)
    assert payload["route_heading_change_deg"] == pytest.approx(2.0)
    assert payload["final_heading_deg"] == pytest.approx(12.0)
    assert payload["encoder_start_stamp"] == pytest.approx(1.0)
    assert payload["encoder_final_stamp"] == pytest.approx(2.500001)
    assert payload["left_encoder_delta_counts"] == 430
    assert payload["right_encoder_delta_counts"] == 450
    assert payload["left_track_distance_m"] == pytest.approx(430 / 4337.768)
    assert payload["right_track_distance_m"] == pytest.approx(450 / 4337.768)
    segment = payload["executed_segments"][0]
    assert segment["start_pose"]["heading_deg"] == pytest.approx(10.0)
    assert segment["final_pose"]["heading_deg"] == pytest.approx(12.0)
    assert segment["heading_change_deg"] == pytest.approx(2.0)
    assert segment["encoder_start_stamp"] == pytest.approx(1.0)
    assert segment["encoder_final_stamp"] == pytest.approx(2.500001)
    assert segment["left_encoder_delta_counts"] == 430
    assert segment["right_encoder_delta_counts"] == 450
    assert segment["terminal_settled"] is True
    assert segment["terminal_settle_duration_s"] == pytest.approx(0.500001)
    assert payload["terminal_settled"] is True


def test_manifest_marks_unchanged_or_stale_track_samples_unavailable() -> None:
    request = LiveRouteRequest(
        route_id="stale-track-evidence",
        segments=(
            RouteSegmentRequest(
                "move-1",
                "move_distance",
                {"distance_m": 0.10, "speed_mps": 0.08, "timeout_s": 4.0},
            ),
        ),
        max_runtime_s=10.0,
        max_travel_m=0.10,
    )
    states = (
        _state(1.0, 0.0, 0.0, 0.0, encoder_counts=(100, 100)),
        LiveRouteState(
            stamp=2.0,
            odom=OdomMotionState(2.0, 0.10, 0.0, 0.0),
            scan=_full_scan(2.0),
            collision_state="CLEAR",
            collision_received_at=2.0,
            encoder_counts=TrackEncoderState(1.0, 100, 100),
        ),
    )

    payload = run_route_replay(request, states).to_json_dict()

    assert payload["encoder_start_stamp"] is None
    assert payload["encoder_final_stamp"] is None
    assert payload["left_encoder_delta_counts"] is None
    assert payload["right_encoder_delta_counts"] is None
    assert payload["left_track_distance_m"] is None
    assert payload["right_track_distance_m"] is None


def test_encoder_state_parser_is_typed_and_rejects_malformed_or_nonfinite_input() -> None:
    valid = _encoder_state(
        json.dumps(
            {
                "schema": "sphero_rvr.encoder_counts.v1",
                "stamp": 12.5,
                "left_count": -2_147_483_640,
                "right_count": 42,
                "counts_per_meter": 4337.768,
            }
        )
    )
    assert valid == TrackEncoderState(12.5, -2_147_483_640, 42, 4337.768)
    assert _encoder_state("not-json") is None
    assert _encoder_state(json.dumps({"schema": "wrong"})) is None
    assert _encoder_state(
        json.dumps(
            {
                "schema": "sphero_rvr.encoder_counts.v1",
                "stamp": float("nan"),
                "left_count": 1,
                "right_count": 2,
                "counts_per_meter": 4337.768,
            }
        )
    ) is None
    assert _encoder_state(
        json.dumps(
            {
                "schema": "sphero_rvr.encoder_counts.v1",
                "stamp": 1.0,
                "left_count": True,
                "right_count": 2,
                "counts_per_meter": 4337.768,
            }
        )
    ) is None


@pytest.mark.parametrize(
    ("state_kwargs", "terminal", "status"),
    (
        ({"collision": CollisionState.STOPPED.value}, "collision_veto", ToolResultStatus.BLOCKED),
        ({"collision": CollisionState.SENSOR_STALE.value}, "collision_veto", ToolResultStatus.BLOCKED),
        ({"collision": CollisionState.ESTOPPED.value}, "collision_veto", ToolResultStatus.BLOCKED),
        ({"collision": CollisionState.DISABLED.value}, "collision_veto", ToolResultStatus.BLOCKED),
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


@pytest.mark.parametrize("collision", ["", "BOGUS", CollisionState.STARTUP.value])
def test_live_route_runner_treats_unknown_or_startup_collision_state_as_missing(collision: str) -> None:
    manifest = run_route_replay(_route(), (_state(1.0, 0.0, 0.0, 0.0, collision=collision),))

    assert manifest.terminal_reason == "missing_collision_state"
    assert manifest.status is ToolResultStatus.BLOCKED


def test_live_route_runner_accepts_fresh_supervisor_slow_state_as_safe_bounded_motion() -> None:
    runner = LiveRouteRunner()
    request = LiveRouteRequest(
        route_id="slow-bounded-move",
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(
            RouteSegmentRequest(
                "move",
                "move_distance",
                {"distance_m": 0.2, "speed_mps": 0.1, "timeout_s": 5.0},
            ),
        ),
    )

    initial = runner.start(
        request,
        _state(1.0, 0.0, 0.0, 0.0, collision=CollisionState.SLOW.value),
    )
    command = runner.update(
        _state(1.05, 0.0, 0.0, 0.0, collision=CollisionState.SLOW.value),
    )

    assert initial.linear_x == 0.0
    assert command.linear_x > 0.0
    assert runner.active
    assert runner.manifest().terminal_reason == "running"


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
        (
            _state(1.0, 0.0, 0.0, 0.0),
            _state(2.0, 0.0, 0.0, math.radians(80.0)),
            _state(2.6, 0.0, 0.0, math.radians(80.0)),
        ),
        config,
    )

    assert manifest.terminal_reason == "wrong_direction"
    assert manifest.status is ToolResultStatus.FAILED
    assert manifest.terminal_settled is True


def test_live_route_runner_waits_for_stationary_evidence_before_complete() -> None:
    request = LiveRouteRequest(
        route_id="settled-target",
        max_runtime_s=10.0,
        max_travel_m=0.1,
        segments=(
            RouteSegmentRequest(
                "move",
                "move_distance",
                {"distance_m": 0.1, "speed_mps": 0.08, "timeout_s": 5.0},
            ),
        ),
    )
    runner = LiveRouteRunner()
    runner.start(request, _state(1.0, 0.0, 0.0, 0.0, encoder_counts=(100, 100)))
    assert runner.update(_state(1.1, 0.0, 0.0, 0.0, encoder_counts=(100, 100))).linear_x > 0.0

    target_zero = runner.update(_state(2.0, 0.1, 0.0, 0.0, encoder_counts=(530, 540)))

    assert target_zero.linear_x == 0.0
    assert runner.active
    assert runner.manifest().executed_segments == ()

    runner.update(_state(2.2, 0.105, 0.0, 0.0, encoder_counts=(550, 560)))
    assert runner.active
    runner.update(_state(2.71, 0.105, 0.0, 0.0, encoder_counts=(550, 560)))

    manifest = runner.manifest()
    assert not runner.active
    assert manifest.status is ToolResultStatus.COMPLETE
    assert manifest.terminal_reason == "complete"
    assert manifest.terminal_settled is True
    assert manifest.route_final_pose["x_m"] == pytest.approx(0.105)
    assert manifest.executed_segments[0].terminal_settled is True
    assert manifest.executed_segments[0].terminal_settle_duration_s == pytest.approx(0.71)


def test_live_route_runner_fails_when_motion_never_settles() -> None:
    request = LiveRouteRequest(
        route_id="unsettled-target",
        max_runtime_s=10.0,
        max_travel_m=0.1,
        segments=(
            RouteSegmentRequest(
                "move",
                "move_distance",
                {"distance_m": 0.1, "speed_mps": 0.08, "timeout_s": 5.0},
            ),
        ),
    )
    runner = LiveRouteRunner(LiveRouteConfig(terminal_settle_time_s=0.3, terminal_settle_timeout_s=0.8))
    runner.start(request, _state(1.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(_state(1.1, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(_state(2.0, 0.1, 0.0, 0.0, encoder_counts=(430, 430)))
    runner.update(_state(2.4, 0.12, 0.0, 0.0, encoder_counts=(520, 520)))
    runner.update(_state(2.81, 0.14, 0.0, 0.0, encoder_counts=(610, 610)))

    manifest = runner.manifest()
    assert not runner.active
    assert manifest.status is ToolResultStatus.FAILED
    assert manifest.terminal_reason == "motion_not_settled"
    assert manifest.terminal_settled is False
    assert manifest.executed_segments[0].terminal_reason == "motion_not_settled"


def test_live_route_runner_fails_when_settled_target_error_exceeds_bound() -> None:
    request = LiveRouteRequest(
        route_id="overshot-target",
        max_runtime_s=10.0,
        max_travel_m=0.1,
        segments=(
            RouteSegmentRequest(
                "move",
                "move_distance",
                {"distance_m": 0.1, "speed_mps": 0.08, "timeout_s": 5.0},
            ),
        ),
    )
    runner = LiveRouteRunner()
    runner.start(request, _state(1.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(_state(1.1, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(_state(2.0, 0.17, 0.0, 0.0, encoder_counts=(730, 770)))
    runner.update(_state(2.51, 0.17, 0.0, 0.0, encoder_counts=(730, 770)))

    manifest = runner.manifest()
    assert manifest.status is ToolResultStatus.FAILED
    assert manifest.terminal_reason == "target_error"
    assert manifest.terminal_settled is True
    assert manifest.executed_segments[0].terminal_distance_error_m == pytest.approx(0.07)


def test_live_route_runner_stops_early_for_observed_adaptive_mission_coast_and_settles_in_bounds() -> None:
    request = LiveRouteRequest(
        route_id="adaptive-mission-duty-64-coast",
        max_runtime_s=5.0,
        max_travel_m=0.25,
        segments=(
            RouteSegmentRequest(
                "move",
                "move_distance",
                {"distance_m": 0.1, "speed_mps": 0.1, "timeout_s": 5.0},
            ),
        ),
    )
    runner = LiveRouteRunner()
    runner.start(request, _state(0.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(_state(0.1, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(_state(0.2, 0.0009, 0.0, 0.0, encoder_counts=(4, 4)))
    runner.update(_state(0.3, 0.0055, 0.0, 0.0, encoder_counts=(24, 24)))
    runner.update(_state(0.4, 0.0129, 0.0, 0.0, encoder_counts=(56, 56)))
    runner.update(_state(0.5, 0.0235, 0.0, 0.0, encoder_counts=(102, 102)))
    runner.update(_state(0.6, 0.0369, 0.0, 0.0, encoder_counts=(160, 160)))
    runner.update(_state(0.7, 0.0535, 0.0, 0.0, encoder_counts=(232, 232)))

    predictive_zero = runner.update(
        _state(0.8, 0.0747, 0.0, 0.0, encoder_counts=(324, 324))
    )
    assert predictive_zero.linear_x == 0.0
    assert runner.active

    # Project the observed post-zero coast from the attended duty-64 trace
    # onto the earlier release point, then provide the required stable window.
    runner.update(_state(0.9, 0.0959, 0.0, 0.0, encoder_counts=(416, 416)))
    runner.update(_state(1.0, 0.1088, 0.0, 0.0, encoder_counts=(472, 472)))
    runner.update(_state(1.1, 0.1134, 0.0, 0.0, encoder_counts=(492, 492)))
    runner.update(_state(1.7, 0.1134, 0.0, 0.0, encoder_counts=(492, 492)))

    manifest = runner.manifest()
    assert manifest.status is ToolResultStatus.COMPLETE
    assert manifest.terminal_reason == "complete"
    assert manifest.terminal_settled is True
    assert manifest.measured_distance_m == pytest.approx(0.1134)
    assert manifest.executed_segments[0].terminal_distance_error_m == pytest.approx(
        0.0134
    )


def test_live_route_runner_uses_observed_turn_rate_before_adaptive_mission_turn_coast() -> None:
    request = LiveRouteRequest(
        route_id="adaptive-mission-turn-45-rate",
        max_runtime_s=5.0,
        max_travel_m=0.25,
        segments=(
            RouteSegmentRequest(
                "turn",
                "turn_angle",
                {
                    "angle_deg": 45.0,
                    "angular_speed_deg_s": math.degrees(0.4),
                    "timeout_s": 5.0,
                },
            ),
        ),
    )
    runner = LiveRouteRunner()
    runner.start(request, _state(0.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))

    for stamp, yaw_deg, counts in (
        (0.1, 2.8, (-34, 21)),
        (0.2, 8.0, (-97, 61)),
        (0.3, 13.0, (-158, 99)),
        (0.4, 19.0, (-231, 145)),
        (0.5, 25.0, (-304, 191)),
    ):
        outside = runner.update(
            _state(
                stamp,
                0.0,
                0.0,
                math.radians(yaw_deg),
                encoder_counts=counts,
            )
        )
        assert outside.angular_z == pytest.approx(0.35)

    outside = runner.update(
        _state(
            0.6,
            0.0,
            0.0,
            math.radians(31.0),
            encoder_counts=(-377, 237),
        )
    )
    predictive_zero = runner.update(
        _state(
            0.7,
            0.0,
            0.0,
            math.radians(37.0),
            encoder_counts=(-450, 283),
        )
    )

    assert outside.angular_z == pytest.approx(0.35)
    assert predictive_zero.angular_z == 0.0
    assert runner.active

    # The reviewed breakaway command and 0.10 s turn horizon release near
    # 37 degrees. Project the measured stop response into the configured
    # terminal band, then provide the required stationary window.
    runner.update(
        _state(
            0.8,
            -0.002,
            -0.001,
            math.radians(43.0),
            encoder_counts=(-523, 329),
        )
    )
    runner.update(
        _state(
            0.9,
            -0.002,
            -0.001,
            math.radians(44.5),
            encoder_counts=(-541, 340),
        )
    )
    runner.update(
        _state(
            1.0,
            -0.002,
            -0.001,
            math.radians(44.5),
            encoder_counts=(-541, 340),
        )
    )
    runner.update(
        _state(
            1.6,
            -0.002,
            -0.001,
            math.radians(44.5),
            encoder_counts=(-541, 340),
        )
    )

    manifest = runner.manifest()
    assert manifest.status is ToolResultStatus.COMPLETE
    assert manifest.terminal_reason == "complete"
    assert manifest.terminal_settled is True
    assert manifest.measured_angle_deg == pytest.approx(44.5)
    assert manifest.executed_segments[0].executed["angular_speed_deg_s"] == pytest.approx(
        math.degrees(0.35)
    )
    assert manifest.executed_segments[0].executed[
        "requested_angular_speed_deg_s"
    ] == pytest.approx(math.degrees(0.4))
    assert manifest.executed_segments[0].terminal_angle_error_deg == pytest.approx(
        0.5
    )


def test_live_route_runner_corrects_stationary_turn_undershoot_within_same_intent() -> None:
    request = LiveRouteRequest(
        route_id="adaptive-mission-turn-correction",
        max_runtime_s=5.0,
        max_travel_m=0.25,
        segments=(
            RouteSegmentRequest(
                "turn",
                "turn_angle",
                {
                    "angle_deg": 45.0,
                    "angular_speed_deg_s": math.degrees(0.4),
                    "timeout_s": 5.0,
                },
            ),
        ),
    )
    # Exercise the correction mechanism with a deliberately tighter threshold
    # than the capability-oriented production default.
    runner = LiveRouteRunner(
        LiveRouteConfig(max_terminal_angle_error_rad=math.radians(5.0))
    )
    runner.start(request, _state(0.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(
        _state(0.1, 0.0, 0.0, math.radians(6.0), encoder_counts=(-70, 40))
    )
    predictive_zero = runner.update(
        _state(0.3, 0.0, 0.0, math.radians(34.0), encoder_counts=(-410, 255))
    )
    assert predictive_zero.angular_z == 0.0

    runner.update(
        _state(0.4, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )
    correction = runner.update(
        _state(1.0, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )
    assert correction.angular_z == pytest.approx(0.35)
    assert runner.active

    correction_zero = runner.update(
        _state(1.1, 0.0, 0.0, math.radians(44.0), encoder_counts=(-532, 331))
    )
    assert correction_zero.angular_z == 0.0
    runner.update(
        _state(1.2, 0.0, 0.0, math.radians(44.0), encoder_counts=(-532, 331))
    )
    runner.update(
        _state(1.8, 0.0, 0.0, math.radians(44.0), encoder_counts=(-532, 331))
    )

    manifest = runner.manifest()
    assert manifest.status is ToolResultStatus.COMPLETE
    assert manifest.terminal_reason == "complete"
    assert manifest.measured_angle_deg == pytest.approx(44.0)
    assert manifest.executed_segments[0].turn_correction_count == 1


def test_adaptive_mission_turn_capability_accepts_attended_settled_trace_within_ten_degrees() -> None:
    request = LiveRouteRequest(
        route_id="adaptive-mission-attended-turn-capability",
        max_runtime_s=5.0,
        max_travel_m=0.25,
        segments=(
            RouteSegmentRequest(
                "turn",
                "turn_angle",
                {
                    "angle_deg": 45.0,
                    "angular_speed_deg_s": math.degrees(0.35),
                    "timeout_s": 5.0,
                },
            ),
        ),
    )
    runner = LiveRouteRunner()
    runner.start(request, _state(0.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(
        _state(0.1, 0.0, 0.0, math.radians(6.0), encoder_counts=(-70, 40))
    )
    runner.update(
        _state(0.3, 0.0, 0.0, math.radians(34.0), encoder_counts=(-410, 255))
    )
    runner.update(
        _state(0.4, 0.0, 0.0, math.radians(34.352), encoder_counts=(-414, 258))
    )
    correction = runner.update(
        _state(1.0, 0.0, 0.0, math.radians(34.352), encoder_counts=(-414, 258))
    )
    assert correction.angular_z == pytest.approx(0.35)
    assert runner.update(
        _state(1.049, 0.0, 0.0, math.radians(34.352), encoder_counts=(-414, 258))
    ).angular_z == 0.0
    runner.update(
        _state(1.15, 0.0, 0.0, math.radians(35.353), encoder_counts=(-426, 266))
    )
    runner.update(
        _state(1.75, 0.0, 0.0, math.radians(35.353), encoder_counts=(-426, 266))
    )

    manifest = runner.manifest()
    assert manifest.status is ToolResultStatus.COMPLETE
    assert manifest.terminal_reason == "complete"
    assert manifest.measured_angle_deg == pytest.approx(35.353)
    assert manifest.executed_segments[0].terminal_angle_error_deg == pytest.approx(
        9.647
    )
    assert manifest.executed_segments[0].turn_correction_count == 1


def test_turn_correction_emits_one_command_then_zeros_before_next_odometry_sample() -> None:
    request = LiveRouteRequest(
        route_id="adaptive-mission-turn-timed-correction",
        max_runtime_s=5.0,
        max_travel_m=0.25,
        segments=(
            RouteSegmentRequest(
                "turn",
                "turn_angle",
                {
                    "angle_deg": 45.0,
                    "angular_speed_deg_s": math.degrees(0.35),
                    "timeout_s": 5.0,
                },
            ),
        ),
    )
    runner = LiveRouteRunner(
        LiveRouteConfig(max_terminal_angle_error_rad=math.radians(5.0))
    )
    runner.start(request, _state(0.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(
        _state(0.1, 0.0, 0.0, math.radians(6.0), encoder_counts=(-70, 40))
    )
    runner.update(
        _state(0.3, 0.0, 0.0, math.radians(34.0), encoder_counts=(-410, 255))
    )
    runner.update(
        _state(0.4, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )
    correction = runner.update(
        _state(1.0, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )
    assert correction.angular_z == pytest.approx(0.35)

    # The route timer runs faster than authoritative odometry. The correction
    # must publish zero on the very next control tick even when that tick lands
    # slightly before the nominal 50 ms period and odometry has not advanced.
    correction_zero = runner.update(
        _state(1.049, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )
    assert correction_zero.angular_z == 0.0
    assert runner.active

    runner.update(
        _state(1.15, 0.0, 0.0, math.radians(43.0), encoder_counts=(-520, 324))
    )
    runner.update(
        _state(1.75, 0.0, 0.0, math.radians(43.0), encoder_counts=(-520, 324))
    )
    assert runner.manifest().status is ToolResultStatus.COMPLETE
    assert runner.manifest().executed_segments[0].turn_correction_count == 1


def test_live_route_runner_allows_three_stationary_verified_turn_corrections() -> None:
    request = LiveRouteRequest(
        route_id="adaptive-mission-turn-three-corrections",
        max_runtime_s=5.0,
        max_travel_m=0.25,
        segments=(
            RouteSegmentRequest(
                "turn",
                "turn_angle",
                {
                    "angle_deg": 45.0,
                    "angular_speed_deg_s": math.degrees(0.35),
                    "timeout_s": 5.0,
                },
            ),
        ),
    )
    runner = LiveRouteRunner(
        LiveRouteConfig(max_terminal_angle_error_rad=math.radians(5.0))
    )
    runner.start(request, _state(0.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(
        _state(0.1, 0.0, 0.0, math.radians(6.0), encoder_counts=(-70, 40))
    )
    runner.update(
        _state(0.3, 0.0, 0.0, math.radians(34.0), encoder_counts=(-410, 255))
    )
    runner.update(
        _state(0.4, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )
    assert runner.update(
        _state(1.0, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    ).angular_z == pytest.approx(0.35)
    assert runner.update(
        _state(1.049, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    ).angular_z == 0.0

    runner.update(
        _state(1.15, 0.0, 0.0, math.radians(37.0), encoder_counts=(-447, 278))
    )
    assert runner.update(
        _state(1.75, 0.0, 0.0, math.radians(37.0), encoder_counts=(-447, 278))
    ).angular_z == pytest.approx(0.35)
    assert runner.update(
        _state(1.799, 0.0, 0.0, math.radians(37.0), encoder_counts=(-447, 278))
    ).angular_z == 0.0

    runner.update(
        _state(1.9, 0.0, 0.0, math.radians(39.0), encoder_counts=(-471, 293))
    )
    assert runner.update(
        _state(2.5, 0.0, 0.0, math.radians(39.0), encoder_counts=(-471, 293))
    ).angular_z == pytest.approx(0.35)
    assert runner.update(
        _state(2.549, 0.0, 0.0, math.radians(39.0), encoder_counts=(-471, 293))
    ).angular_z == 0.0

    runner.update(
        _state(2.65, 0.0, 0.0, math.radians(41.0), encoder_counts=(-495, 308))
    )
    runner.update(
        _state(3.25, 0.0, 0.0, math.radians(41.0), encoder_counts=(-495, 308))
    )
    manifest = runner.manifest()
    assert manifest.status is ToolResultStatus.COMPLETE
    assert manifest.measured_angle_deg == pytest.approx(41.0)
    assert manifest.executed_segments[0].turn_correction_count == 3


def test_live_route_runner_never_corrects_turn_when_budget_is_zero() -> None:
    request = LiveRouteRequest(
        route_id="adaptive-mission-turn-no-correction",
        max_runtime_s=5.0,
        max_travel_m=0.25,
        segments=(
            RouteSegmentRequest(
                "turn",
                "turn_angle",
                {
                    "angle_deg": 45.0,
                    "angular_speed_deg_s": math.degrees(0.35),
                    "timeout_s": 5.0,
                },
            ),
        ),
    )
    runner = LiveRouteRunner(
        LiveRouteConfig(
            max_terminal_angle_error_rad=math.radians(5.0),
            max_turn_corrections=0,
        )
    )
    runner.start(request, _state(0.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(
        _state(0.1, 0.0, 0.0, math.radians(6.0), encoder_counts=(-70, 40))
    )
    runner.update(
        _state(0.3, 0.0, 0.0, math.radians(34.0), encoder_counts=(-410, 255))
    )
    runner.update(
        _state(0.4, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )
    runner.update(
        _state(1.0, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )

    manifest = runner.manifest()
    assert not runner.active
    assert manifest.status is ToolResultStatus.FAILED
    assert manifest.terminal_reason == "target_error"
    assert manifest.executed_segments[0].turn_correction_count == 0


def test_collision_veto_terminates_active_turn_correction_without_resumption() -> None:
    request = LiveRouteRequest(
        route_id="adaptive-mission-turn-correction-collision",
        max_runtime_s=5.0,
        max_travel_m=0.25,
        segments=(
            RouteSegmentRequest(
                "turn",
                "turn_angle",
                {
                    "angle_deg": 45.0,
                    "angular_speed_deg_s": math.degrees(0.35),
                    "timeout_s": 5.0,
                },
            ),
        ),
    )
    runner = LiveRouteRunner(
        LiveRouteConfig(max_terminal_angle_error_rad=math.radians(5.0))
    )
    runner.start(request, _state(0.0, 0.0, 0.0, 0.0, encoder_counts=(0, 0)))
    runner.update(
        _state(0.1, 0.0, 0.0, math.radians(6.0), encoder_counts=(-70, 40))
    )
    runner.update(
        _state(0.3, 0.0, 0.0, math.radians(34.0), encoder_counts=(-410, 255))
    )
    runner.update(
        _state(0.4, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )
    correction = runner.update(
        _state(1.0, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )
    assert correction.angular_z == pytest.approx(0.35)

    vetoed = runner.update(
        _state(
            1.1,
            0.0,
            0.0,
            math.radians(36.0),
            collision="STOPPED",
            encoder_counts=(-435, 270),
        )
    )
    later_clear = runner.update(
        _state(1.2, 0.0, 0.0, math.radians(36.0), encoder_counts=(-435, 270))
    )

    assert vetoed.linear_x == vetoed.angular_z == 0.0
    assert later_clear.linear_x == later_clear.angular_z == 0.0
    assert not runner.active
    assert runner.manifest().terminal_reason == "collision_veto"


@pytest.mark.parametrize("value", [-1, 1.5, True])
def test_live_route_config_rejects_invalid_turn_correction_budget(value) -> None:
    with pytest.raises(ValueError, match="max_turn_corrections"):
        LiveRouteConfig(max_turn_corrections=value)


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
        (
            _state(1.0, 0.0, 0.0, 0.0),
            _state(2.0, 0.0, 0.0, 0.0),
            _state(2.6, 0.0, 0.0, 0.0),
        ),
        config,
    )

    assert manifest.terminal_reason == "stall"
    assert manifest.status is ToolResultStatus.FAILED
    assert manifest.terminal_settled is True
    assert (
        manifest.executed_segments[0].terminal_settle_duration_s
        >= config.terminal_settle_time_s
    )


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


def test_live_route_runner_rejects_non_finite_route_motion_inputs_without_execution() -> None:
    request = LiveRouteRequest(
        route_id="nan-route",
        max_runtime_s=10.0,
        max_travel_m=0.5,
        segments=(RouteSegmentRequest("move", "move_distance", {"distance_m": 0.2, "speed_mps": math.nan, "timeout_s": 5.0}),),
    )

    manifest = run_route_replay(request, (_state(1.0, 0.0, 0.0, 0.0),))

    assert manifest.status is ToolResultStatus.FAILED
    assert manifest.terminal_reason == "invalid_route"
    assert manifest.executed_segments == ()


def test_live_route_node_is_installed_default_off_and_cannot_own_motor_or_serial_surfaces() -> None:
    setup_text = (REPO_ROOT / "setup.py").read_text()
    launch_text = (REPO_ROOT / "launch" / "supervised_rvr.launch.py").read_text()
    config_text = (REPO_ROOT / "config" / "live_route_runner.yaml").read_text()
    node_source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "live_route_runner_node.py").read_text()
    driver_source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "rvr_node.py").read_text()

    assert "live_route_runner = sphero_rvr_driver.live_route_runner_node:main" in setup_text
    assert "config/live_route_runner.yaml" in setup_text
    assert "start_live_route_runner" in launch_text
    assert 'default_value="false"' in launch_text
    assert "cmd_vel_topic: /cmd_vel" in config_text
    assert "collision_state_max_age_s: 0.30" in config_text
    assert "encoder_counts_topic: /encoder_counts" in config_text
    assert "track_counts_per_meter: 4337.768" in config_text
    assert "terminal_settle_time_s: 0.50" in config_text
    assert "terminal_settle_timeout_s: 2.0" in config_text
    assert "max_terminal_distance_error_m: 0.03" in config_text
    assert "max_terminal_angle_error_rad: 0.17453292519943295" in config_text
    assert "target_stop_horizon_s: 0.25" in config_text
    assert "turn_target_stop_horizon_s: 0.10" in config_text
    assert "max_turn_speed_rad_s: 0.35" in config_text
    assert "max_turn_progress_rate_rad_s: 3.5" in config_text
    assert "control_period_s: 0.05" in config_text
    assert "max_turn_corrections: 3" in config_text
    assert "MAX_TURN_CORRECTION_CONTROL_PERIOD_S = 0.05" in node_source
    assert "min_progress_m: 0.005" in config_text
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
    assert "encoder_counts=self._latest_encoder_counts" in node_source
    assert 'create_publisher(String, "encoder_counts", 10)' in driver_source
    assert "sphero_rvr.encoder_counts.v1" in driver_source
    assert "_collision_state_value(getattr(msg, \"data\", None))" in node_source
    assert setup_text.count("config/live_route_runner.yaml") == 1
