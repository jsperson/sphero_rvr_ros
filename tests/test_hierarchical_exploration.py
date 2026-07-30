from __future__ import annotations

from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest

from sphero_rvr_driver.hierarchical_exploration import (
    ContinuousGoalFollowerReplay,
    FrontierDetectionConfig,
    FrontierGoal,
    FrontierRegistry,
    FrontierState,
    HierarchicalBridgeConfig,
    HierarchicalCommandBridge,
    HierarchicalReplayAdaptiveMissionExecutor,
    PRIVATE_NAV2_CMD_TOPIC,
    UPSTREAM_FRONTIER_REVISION,
    detect_frontiers,
    load_slam_toolbox_map,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
RECORDED_MAP_DIR = REPO_ROOT / "artifacts" / "phase1_recorded_slam_map"
RECORDED_MAP = RECORDED_MAP_DIR / "phase1_recorded_slam_map.yaml"


def recorded_grid():
    return load_slam_toolbox_map(
        RECORDED_MAP,
        map_id="rvr-room-20260626",
    )


def replay_goal(
    generation: int,
    *,
    route_length_m: float,
    ready_at_s: float,
    valid: bool = True,
) -> FrontierGoal:
    return FrontierGoal(
        generation=generation,
        frontier_signature=f"frontier-{generation}",
        map_id="recorded-room",
        map_revision="map-revision-1",
        x_m=float(generation),
        y_m=0.0,
        route_length_m=route_length_m,
        ready_at_s=ready_at_s,
        planning_snapshot_valid=valid,
    )


def test_recorded_slam_toolbox_map_provenance_and_trinary_load() -> None:
    manifest = json.loads((RECORDED_MAP_DIR / "manifest.json").read_text())
    image = RECORDED_MAP_DIR / "phase1_recorded_slam_map.pgm"
    grid = recorded_grid()

    assert hashlib.sha256(image.read_bytes()).hexdigest() == (
        manifest["source_pgm_sha256"]
    )
    assert (grid.width, grid.height) == (70, 71)
    assert grid.resolution_m == pytest.approx(0.05)
    assert {value: grid.cells.count(value) for value in (-1, 0, 100)} == {
        -1: 2759,
        0: 2034,
        100: 177,
    }
    assert manifest["authority"]["motion_authority"] is False
    assert manifest["authority"]["live_sensors_started"] is False


def test_wfd_is_deterministic_and_signatures_survive_map_revision_only() -> None:
    grid = recorded_grid()
    config = FrontierDetectionConfig(
        minimum_frontier_cells=3,
        minimum_clearance_m=0.10,
    )
    kwargs = {
        "robot_x_m": -0.028,
        "robot_y_m": 0.941,
        "config": config,
    }

    first = detect_frontiers(grid, **kwargs)
    second = detect_frontiers(grid, **kwargs)
    revision_only = detect_frontiers(
        replace(grid, revision="f" * 64),
        **kwargs,
    )

    assert len(first) == 13
    assert [item.signature for item in first] == [
        item.signature for item in second
    ]
    assert [item.signature for item in first] == [
        item.signature for item in revision_only
    ]
    assert all(item.approach_cells for item in first)
    assert all(item.clearance_m >= 0.10 for item in first)
    assert all(item.map_revision == grid.revision for item in first)


def test_frontier_filtering_invalidation_and_exhaustion_are_explicit() -> None:
    grid = recorded_grid()
    candidates = detect_frontiers(
        grid,
        robot_x_m=-0.028,
        robot_y_m=0.941,
        config=FrontierDetectionConfig(
            minimum_frontier_cells=20,
            minimum_clearance_m=0.30,
        ),
    )
    registry = FrontierRegistry()

    active = registry.update(candidates[:2])
    assert len(active) == 2
    registry.update(candidates[:1])
    assert registry.states[candidates[1].signature] is FrontierState.INVALIDATED
    registry.mark(candidates[0].signature, FrontierState.VISITED)
    assert registry.exhausted is True


def test_long_leg_prefetch_extends_one_controller_session_without_zero() -> None:
    follower = ContinuousGoalFollowerReplay(queue_depth=3)
    follower.start(
        [
            replay_goal(1, route_length_m=1.5, ready_at_s=0.0),
            replay_goal(2, route_length_m=0.8, ready_at_s=12.7),
            replay_goal(3, route_length_m=0.7, ready_at_s=12.7),
        ]
    )

    prefetched = follower.advance(now_s=13.0, remaining_distance_m=0.2)
    handed_off = follower.advance(now_s=15.0, remaining_distance_m=0.0)

    assert prefetched.queued_generations == (2, 3)
    assert [event["kind"] for event in prefetched.events] == [
        "path_extended",
        "path_extended",
    ]
    assert handed_off.state == "navigating"
    assert handed_off.active_generation == 2
    assert handed_off.controller_session == 1
    assert handed_off.controller_active is True
    assert handed_off.command.zero_required is False
    assert handed_off.events[-1]["kind"] == "atomic_handoff"
    assert handed_off.events[-1]["controller_exit"] is False
    assert handed_off.events[-1]["deliberate_zero"] is False


def test_short_hop_honestly_holds_wait_planning_then_starts_new_session() -> None:
    follower = ContinuousGoalFollowerReplay(queue_depth=2)
    follower.start(
        [
            replay_goal(1, route_length_m=0.5, ready_at_s=0.0),
            replay_goal(2, route_length_m=0.6, ready_at_s=12.7),
        ]
    )

    waiting = follower.advance(now_s=5.0, remaining_distance_m=0.0)
    resumed = follower.advance(now_s=12.7, remaining_distance_m=0.6)

    assert waiting.state == "wait_planning"
    assert waiting.controller_active is False
    assert waiting.command.zero_required is True
    assert waiting.command.reason == "wait_planning"
    assert resumed.state == "navigating"
    assert resumed.controller_session == 2
    assert resumed.command.zero_required is False
    assert resumed.events[-1]["kind"] == "planning_resume"


def test_invalid_prefetch_discards_and_supervisor_veto_is_immediate() -> None:
    follower = ContinuousGoalFollowerReplay(queue_depth=2)
    follower.start(
        [
            replay_goal(1, route_length_m=1.0, ready_at_s=0.0),
            replay_goal(2, route_length_m=1.0, ready_at_s=1.0, valid=False),
        ]
    )

    discarded = follower.advance(now_s=1.0, remaining_distance_m=0.5)
    stopped = follower.advance(
        now_s=1.1,
        remaining_distance_m=0.4,
        collision_state="BLOCKED",
    )

    assert discarded.events[-1]["kind"] == "prefetch_discarded"
    assert discarded.state == "navigating"
    assert stopped.state == "terminal_safety"
    assert stopped.controller_active is False
    assert stopped.command.zero_required is True
    assert stopped.command.reason == "collision_veto"


def test_stale_motion_evidence_holds_then_resumes_with_new_goal() -> None:
    follower = ContinuousGoalFollowerReplay(queue_depth=2)
    follower.start([replay_goal(1, route_length_m=1.0, ready_at_s=0.0)])

    held = follower.advance(
        now_s=0.5,
        remaining_distance_m=0.8,
        collision_state="BLOCKED",
        motion_evidence_fresh=False,
    )
    follower.submit_prefetch(
        replay_goal(2, route_length_m=0.7, ready_at_s=0.6)
    )
    resumed = follower.advance(
        now_s=0.6,
        remaining_distance_m=0.8,
        motion_evidence_fresh=True,
    )

    assert held.state == "wait_planning"
    assert held.controller_active is False
    assert held.command.zero_required is True
    assert held.command.reason == "motion_evidence_stale"
    assert held.events[-1]["kind"] == "motion_evidence_stale"
    assert resumed.state == "navigating"
    assert resumed.controller_session == 2
    assert resumed.controller_active is True
    assert resumed.command.zero_required is False
    assert resumed.events[-1]["kind"] == "planning_resume"
    assert resumed.events[-1]["generation"] == 2


def test_private_command_bridge_is_default_off_bounded_and_lease_limited() -> None:
    disabled = HierarchicalCommandBridge()
    disabled.accept(0.1, 0.4, received_at_s=1.0)
    assert disabled.evaluate(
        now_s=1.0,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="CLEAR",
    ).reason == "hierarchical_mode_disabled"

    enabled = HierarchicalCommandBridge(
        HierarchicalBridgeConfig(enabled=True)
    )
    enabled.accept(0.5, -1.0, received_at_s=2.0)
    bounded = enabled.evaluate(
        now_s=2.1,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="CLEAR",
    )
    stale = enabled.evaluate(
        now_s=2.26,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="CLEAR",
    )

    assert bounded.bridged_linear_mps == pytest.approx(0.10)
    assert bounded.bridged_angular_rad_s == pytest.approx(-0.4)
    assert stale.zero_required is True
    assert stale.reason == "nav2_command_stale"

    physical = HierarchicalCommandBridge(
        HierarchicalBridgeConfig(
            enabled=True,
            clear_breakaway_linear_mps=0.10,
            clear_breakaway_angular_rad_s=0.35,
            reverse_escape_linear_mps=0.07,
        )
    )
    physical.accept(0.014, 0.0, received_at_s=3.0)
    clear = physical.evaluate(
        now_s=3.1,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="CLEAR",
    )
    slow = physical.evaluate(
        now_s=3.1,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="SLOW",
    )
    assert clear.bridged_linear_mps == pytest.approx(0.10)
    assert clear.reason == "clear_breakaway_floor"
    assert slow.bridged_linear_mps == pytest.approx(-0.07)
    assert slow.bridged_angular_rad_s == 0.0
    assert slow.reason == "slow_reverse_escape"

    physical.accept(0.071, 0.109, received_at_s=3.2)
    pivot = physical.evaluate(
        now_s=3.3,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="SLOW",
    )
    assert pivot.bridged_linear_mps == 0.0
    assert pivot.bridged_angular_rad_s == pytest.approx(0.35)
    assert pivot.reason == "slow_pivot_breakaway"

    physical.accept(-0.04, 0.0, received_at_s=3.4)
    reverse = physical.evaluate(
        now_s=3.5,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="SLOW",
    )
    assert reverse.bridged_linear_mps == pytest.approx(-0.04)
    assert reverse.bridged_angular_rad_s == 0.0
    assert reverse.reason == "clear"

    physical.accept(0.10, 0.1, received_at_s=3.6)
    stopped_escape = physical.evaluate(
        now_s=3.7,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="STOPPED",
    )
    assert stopped_escape.bridged_linear_mps == pytest.approx(-0.07)
    assert stopped_escape.bridged_angular_rad_s == 0.0
    assert stopped_escape.reason == "stopped_reverse_escape_request"

    physical.accept(0.0, 0.4, received_at_s=3.8)
    blocked_turn_escape = physical.evaluate(
        now_s=3.9,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="SLOW",
        collision_reason="left_trajectory_blocked",
    )
    assert blocked_turn_escape.bridged_linear_mps == pytest.approx(-0.07)
    assert blocked_turn_escape.bridged_angular_rad_s == 0.0
    assert (
        blocked_turn_escape.reason
        == "blocked_trajectory_reverse_escape"
    )

    physical.accept(0.0, -0.036, received_at_s=4.0)
    turn = physical.evaluate(
        now_s=4.1,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="CLEAR",
    )
    assert turn.bridged_linear_mps == 0.0
    assert turn.bridged_angular_rad_s == pytest.approx(-0.35)
    assert turn.reason == "clear_angular_breakaway_floor"

    physical.accept(0.02, 0.036, received_at_s=5.0)
    mixed = physical.evaluate(
        now_s=5.1,
        goal_active=True,
        mission_lease_valid=True,
        motion_evidence_fresh=True,
        collision_state="CLEAR",
    )
    assert mixed.bridged_linear_mps == pytest.approx(0.10)
    assert mixed.bridged_angular_rad_s == pytest.approx(0.036)
    assert mixed.reason == "clear_breakaway_floor"


def test_hierarchical_executor_uses_existing_replay_seam_without_authority() -> None:
    executor = HierarchicalReplayAdaptiveMissionExecutor(queue_depth=3)
    active = executor.update_map(
        recorded_grid(),
        robot_x_m=-0.028,
        robot_y_m=0.941,
        config=FrontierDetectionConfig(minimum_clearance_m=0.10),
    )
    snapshot = executor.frontier_snapshot()
    projection = executor.map_projection()

    assert active
    assert snapshot["upstream_frontier_revision"] == UPSTREAM_FRONTIER_REVISION
    assert snapshot["motion_authority"] is False
    assert snapshot["physical_execution_enabled"] is False
    assert projection["fixture_only"] is False
    assert projection["nav2_handoff"]["private_command_topic"] == (
        PRIVATE_NAV2_CMD_TOPIC
    )


def test_nav2_replay_graph_is_default_off_and_has_exclusive_publishers() -> None:
    launch = (
        REPO_ROOT / "launch" / "hierarchical_exploration_replay.launch.py"
    ).read_text()
    nav2 = (REPO_ROOT / "config" / "hierarchical_nav2.yaml").read_text()
    navigation_tree = (
        REPO_ROOT / "config" / "hierarchical_navigate_through_poses.xml"
    ).read_text()
    route = (REPO_ROOT / "config" / "live_route_runner.yaml").read_text()
    collision = (
        REPO_ROOT / "src" / "sphero_rvr_driver" / "collision_stop_node.py"
    ).read_text()

    assert launch.count('default_value="false"') == 4
    assert 'remappings=[("cmd_vel", "/nav2_cmd_vel_request")]' in launch
    assert 'executable="live_route_runner"' in launch
    assert 'executable="lidar_collision_stop_supervisor"' in launch
    assert 'executable="rvr_nav2_loopback_sim"' in launch
    assert 'executable="behavior_server"' in launch
    assert 'remappings=[("cmd_vel", "/cmd_vel_motor")]' in launch
    assert "rvr_node" not in launch
    assert "serial" not in launch.lower()
    assert "nav2_smac_planner::SmacPlanner2D" in nav2
    assert "dwb_core::DWBLocalPlanner" in nav2
    assert "max_vel_x: 0.10" in nav2
    assert "max_vel_theta: 0.4" in nav2
    assert '<RateController hz="0.05">' in navigation_tree
    assert 'radius="0.10"' in navigation_tree
    assert "hierarchical_mode_enabled: false" in route
    assert 'self.create_publisher(Twist, motor_cmd_topic, 10)' in collision
