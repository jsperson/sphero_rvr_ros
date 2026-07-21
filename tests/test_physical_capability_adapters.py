from __future__ import annotations

import pytest

from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_api import (
    ApprovalGrant,
    CapabilityAvailability,
    CriterionKind,
    DeterministicMissionRuntime,
    MissionBudgets,
    MissionGoal,
    MissionRuntimeStatus,
    MissionPlan,
    SuccessCriterion,
    ToolInvocation,
    ToolResultStatus,
    _arguments_digest,
    build_default_registry,
    issue_approval_grant,
)
from sphero_rvr_driver.odometry import OdomMotionState
from sphero_rvr_driver.physical_capability_adapters import PhysicalCapabilityAdapters
from sphero_rvr_driver.range_motion import RangeMotionSample, StopReason
from sphero_rvr_driver.supervised_coordinator import SegmentStatus


def _grant(
    now_s: float = 100.0,
    *,
    mission_id: str = "physical-move",
    approval_id: str = "operator-approval-1",
    tool_id: str = "move_to_clearance",
    correlation_id: str = "move-1",
    arguments: dict | None = None,
) -> ApprovalGrant:
    if arguments is None:
        arguments = {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.5}
    return issue_approval_grant(
        approval_id=approval_id,
        approved_by="operator:scott",
        approved_at_s=now_s,
        expires_at_s=now_s + 60.0,
        approval_class="supervised_motion",
        mission_id=mission_id,
        issued_to="mission-runtime",
        tool_id=tool_id,
        correlation_id=correlation_id,
        arguments_digest=_arguments_digest(arguments),
        principal="operator:scott",
    )


def _move_plan(*, max_steps: int = 1) -> MissionPlan:
    return MissionPlan(
        goal=MissionGoal(
            goal_id="physical-move",
            objective="move until four inches from the object",
            success_criteria=(
                SuccessCriterion("physical-move-complete", "bounded range motion reaches requested clearance", CriterionKind.TOOL_COMPLETE, tool_id="move_to_clearance"),
            ),
            budgets=MissionBudgets(max_steps=max_steps, max_runtime_s=30.0, max_travel_m=0.5),
        ),
        invocations=(
            ToolInvocation(
                "move-1",
                "move_to_clearance",
                "1.0",
                {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.5},
                approval=_grant(),
            ),
        ),
    )


def _exploration_plan() -> MissionPlan:
    return MissionPlan(
        goal=MissionGoal(
            goal_id="physical-exploration",
            objective="run two bounded exploration segments",
            success_criteria=(
                SuccessCriterion("physical-exploration-complete", "supervised coordinator completes bounded segments", CriterionKind.TOOL_COMPLETE, tool_id="bounded_exploration_segment"),
            ),
            budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0, max_travel_m=1.0),
        ),
        invocations=(
            ToolInvocation(
                "explore-1",
                "bounded_exploration_segment",
                "1.0",
                {"max_segments": 2, "segment_timeout_s": 4.0, "max_travel_m": 0.6},
                approval=_grant(
                    mission_id="physical-exploration",
                    tool_id="bounded_exploration_segment",
                    correlation_id="explore-1",
                    arguments={"max_segments": 2, "segment_timeout_s": 4.0, "max_travel_m": 0.6},
                ),
            ),
        ),
    )


def _control_plan(action: str) -> MissionPlan:
    return MissionPlan(
        goal=MissionGoal(
            goal_id=f"physical-control-{action}",
            objective="propagate mission control action",
            success_criteria=("mission control action latches at runtime boundary",),
            budgets=MissionBudgets(max_steps=1, max_runtime_s=5.0),
        ),
        invocations=(ToolInvocation("control-1", "pause_cancel_stop_estop", "1.0", {"action": action}),),
    )


def _odom_route_plan() -> MissionPlan:
    return MissionPlan(
        goal=MissionGoal(
            goal_id="physical-odom-route",
            objective="execute typed odometry route primitives",
            success_criteria=(
                SuccessCriterion("physical-move-distance-complete", "move_distance uses measured odometry", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),
                SuccessCriterion("physical-turn-angle-complete", "turn_angle uses measured odometry", CriterionKind.TOOL_COMPLETE, tool_id="turn_angle"),
            ),
            budgets=MissionBudgets(max_steps=2, max_runtime_s=20.0, max_travel_m=0.5),
        ),
        invocations=(
            ToolInvocation(
                "move-distance-1",
                "move_distance",
                "1.0",
                {"distance_m": 0.4572, "speed_mps": 0.10, "timeout_s": 10.0},
                approval=_grant(
                    mission_id="physical-odom-route",
                    approval_id="operator-approval-move-distance",
                    tool_id="move_distance",
                    correlation_id="move-distance-1",
                    arguments={"distance_m": 0.4572, "speed_mps": 0.10, "timeout_s": 10.0},
                ),
            ),
            ToolInvocation(
                "turn-angle-1",
                "turn_angle",
                "1.0",
                {"angle_deg": -90.0, "angular_speed_deg_s": 45.0, "timeout_s": 6.0},
                approval=_grant(
                    mission_id="physical-odom-route",
                    approval_id="operator-approval-turn-angle",
                    tool_id="turn_angle",
                    correlation_id="turn-angle-1",
                    arguments={"angle_deg": -90.0, "angular_speed_deg_s": 45.0, "timeout_s": 6.0},
                ),
            ),
        ),
    )


def _sample(t: float, clearance: float, *, odom_x: float = 0.0, front: float = 1.0) -> RangeMotionSample:
    return RangeMotionSample(
        stamp=t,
        target_clearance_m=clearance,
        front_clearance_m=front,
        rear_clearance_m=1.0,
        left_clearance_m=1.0,
        right_clearance_m=1.0,
        odom_displacement_m=odom_x,
    )


def test_physical_move_to_clearance_uses_range_motion_controller_without_leaking_motor_surfaces() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(
            move_samples_by_correlation_id={
                "move-1": (
                    _sample(100.0, 0.50, odom_x=0.0),
                    _sample(100.2, 0.42, odom_x=0.08),
                    _sample(100.6, 0.22, odom_x=0.28),
                    _sample(101.0, 0.105, odom_x=0.395),
                )
            },
            move_sample_wall_times_by_correlation_id={"move-1": (100.0, 100.2, 100.6, 101.0)},
        ),
        now_s=100.0,
    )

    result = runtime.execute_plan(_move_plan())

    assert result.status is MissionRuntimeStatus.COMPLETE
    tool_result = result.results[0]
    assert tool_result.status is ToolResultStatus.COMPLETE
    assert tool_result.observation == {"target_clearance_m": pytest.approx(0.1016)}
    assert tool_result.provenance == {"adapter": "physical/supervised_control_fixture", "deterministic": True}
    payload = result.to_json_dict()
    assert "/cmd_vel" not in str(payload)
    assert "/cmd_vel_motor" not in str(payload)


def test_physical_move_to_clearance_fails_closed_when_range_motion_does_not_reach_target() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(
            move_samples_by_correlation_id={
                "move-1": (
                    _sample(100.0, 0.50, odom_x=0.0),
                    _sample(100.2, 0.49, odom_x=0.0),
                    _sample(101.0, 0.48, odom_x=0.0),
                    _sample(101.8, 0.47, odom_x=0.0),
                )
            },
            move_sample_wall_times_by_correlation_id={"move-1": (100.0, 100.2, 101.0, 101.8)},
        ),
        now_s=100.0,
    )

    result = runtime.execute_plan(_move_plan())

    assert result.status is MissionRuntimeStatus.FAILED
    assert result.results[0].status is ToolResultStatus.FAILED
    assert "range motion stopped: stall" in result.results[0].error["message"]


def test_physical_move_to_clearance_fails_closed_on_stale_samples_instead_of_rebasing_sample_time() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(
            move_samples_by_correlation_id={
                "move-1": (
                    _sample(100.0, 0.50, odom_x=0.0),
                    _sample(100.2, 0.105, odom_x=0.395),
                )
            },
            move_sample_wall_times_by_correlation_id={"move-1": (100.0, 100.6)},
        ),
        now_s=100.0,
    )

    result = runtime.execute_plan(_move_plan())

    assert result.status is MissionRuntimeStatus.BLOCKED
    assert result.results[0].status is ToolResultStatus.BLOCKED
    assert result.results[0].error["message"] == "range motion stopped: stale_sensor"


def test_physical_move_to_clearance_blocks_when_sample_wall_times_are_missing() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(
            move_samples_by_correlation_id={"move-1": (_sample(100.0, 0.50, odom_x=0.0),)}
        ),
        now_s=100.0,
    )

    result = runtime.execute_plan(_move_plan())

    assert result.status is MissionRuntimeStatus.BLOCKED
    assert result.results[0].status is ToolResultStatus.BLOCKED
    assert "wall-time receipt" in result.results[0].error["message"]


def test_physical_bounded_exploration_segment_uses_supervised_coordinator_and_measured_statuses() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(
            exploration_clearances_m=(0.80, 0.70, 0.60),
            exploration_segment_statuses_by_correlation_id={
                "explore-1": (
                    SegmentStatus(StopReason.TARGET_REACHED, measured_displacement_m=0.25, current_clearance_m=0.25),
                    SegmentStatus(StopReason.TARGET_REACHED, measured_displacement_m=0.20, current_clearance_m=0.25),
                )
            },
        ),
        now_s=100.0,
    )

    result = runtime.execute_plan(_exploration_plan())

    assert result.status is MissionRuntimeStatus.COMPLETE
    assert result.results[0].observation == {"completed_segments": 2}
    assert "/cmd_vel" not in str(result.to_json_dict())


def test_physical_odom_primitives_return_signed_measured_terminal_results_without_motor_surfaces() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(
            odom_states_by_correlation_id={
                "move-distance-1": (
                    OdomMotionState(stamp=100.0, x_m=0.0, y_m=0.0, yaw_rad=0.0),
                    OdomMotionState(stamp=104.8, x_m=0.46, y_m=0.0, yaw_rad=0.0),
                ),
                "turn-angle-1": (
                    OdomMotionState(stamp=105.0, x_m=0.46, y_m=0.0, yaw_rad=0.0),
                    OdomMotionState(stamp=107.0, x_m=0.46, y_m=0.0, yaw_rad=-1.56),
                ),
            },
            odom_state_wall_times_by_correlation_id={
                "move-distance-1": (100.0, 104.8),
                "turn-angle-1": (105.0, 107.0),
            },
        ),
        now_s=100.0,
    )

    route_plan = _odom_route_plan()
    with pytest.raises(MissionValidationError, match="exclusive resource conflict"):
        runtime.execute_plan(route_plan)

    move_result = runtime.execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="physical-odom-route",
                objective="execute measured move_distance primitive",
                success_criteria=(SuccessCriterion("physical-move-distance-complete", "move_distance uses measured odometry", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=20.0, max_travel_m=0.5),
            ),
            invocations=(route_plan.invocations[0],),
        )
    )
    turn_result = runtime.execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="physical-odom-route",
                objective="execute measured turn_angle primitive",
                success_criteria=(SuccessCriterion("physical-turn-angle-complete", "turn_angle uses measured odometry", CriterionKind.TOOL_COMPLETE, tool_id="turn_angle"),),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=20.0),
            ),
            invocations=(route_plan.invocations[1],),
        )
    )

    assert move_result.status is MissionRuntimeStatus.COMPLETE
    assert turn_result.status is MissionRuntimeStatus.COMPLETE
    assert move_result.results[0].observation["measured_distance_m"] == pytest.approx(0.46)
    assert turn_result.results[0].observation["measured_angle_deg"] == pytest.approx(-89.38, abs=0.01)
    assert "/cmd_vel" not in str(move_result.to_json_dict())
    assert "/cmd_vel" not in str(turn_result.to_json_dict())


@pytest.mark.parametrize(
    ("field", "status", "message"),
    (
        ("odom_stop_at_index_by_correlation_id", ToolResultStatus.STOPPED, "odom primitive stopped: stop"),
        ("odom_estop_at_index_by_correlation_id", ToolResultStatus.ESTOPPED, "odom primitive stopped: estop"),
        ("odom_cancel_at_index_by_correlation_id", ToolResultStatus.CANCELLED, "odom primitive stopped: cancelled"),
        ("odom_collision_veto_at_index_by_correlation_id", ToolResultStatus.BLOCKED, "odom primitive stopped: collision_veto"),
    ),
)
def test_physical_odom_primitive_propagates_authoritative_stops_during_execution(field, status, message) -> None:
    adapter_kwargs = {
        "odom_states_by_correlation_id": {
            "move-distance-1": (
                OdomMotionState(stamp=100.0, x_m=0.0, y_m=0.0, yaw_rad=0.0),
                OdomMotionState(stamp=100.5, x_m=0.01, y_m=0.0, yaw_rad=0.0),
            )
        },
        "odom_state_wall_times_by_correlation_id": {"move-distance-1": (100.0, 100.5)},
        field: {"move-distance-1": 1},
    }
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(**adapter_kwargs),
        now_s=100.0,
    )

    result = runtime.execute_plan(MissionPlan(goal=_odom_route_plan().goal, invocations=(_odom_route_plan().invocations[0],)))

    assert result.results[0].status is status
    assert result.results[0].error["message"] == message


def test_physical_bounded_exploration_segment_blocks_without_measured_statuses_instead_of_faking_success() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(exploration_clearances_m=(0.80, 0.70, 0.60)),
        now_s=100.0,
    )

    result = runtime.execute_plan(_exploration_plan())

    assert result.status is MissionRuntimeStatus.BLOCKED
    assert result.results[0].status is ToolResultStatus.BLOCKED
    assert result.results[0].error["message"] == "physical bounded_exploration_segment requires measured segment statuses"


def test_physical_bounded_exploration_segment_fails_closed_on_measured_segment_failure() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(
            exploration_clearances_m=(0.80, 0.70, 0.60),
            exploration_segment_statuses_by_correlation_id={
                "explore-1": (SegmentStatus(StopReason.STALE_SENSOR, measured_displacement_m=0.0, current_clearance_m=None),)
            },
        ),
        now_s=100.0,
    )

    result = runtime.execute_plan(_exploration_plan())

    assert result.status is MissionRuntimeStatus.FAILED
    assert result.results[0].status is ToolResultStatus.FAILED
    assert "stale_sensor" in result.results[0].error["message"]


def test_physical_bounded_exploration_segment_fails_closed_on_cleanup_uncertain_segment_status() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(
            exploration_clearances_m=(0.80, 0.70, 0.60),
            exploration_segment_statuses_by_correlation_id={
                "explore-1": (
                    SegmentStatus(StopReason.CLEANUP_UNCERTAIN, measured_displacement_m=0.05, current_clearance_m=0.70),
                )
            },
        ),
        now_s=100.0,
    )

    result = runtime.execute_plan(_exploration_plan())

    assert result.status is MissionRuntimeStatus.FAILED
    assert result.results[0].status is ToolResultStatus.FAILED
    assert "cleanup_uncertain" in result.results[0].error["message"]


def test_physical_adapter_blocks_motion_when_required_observations_are_missing_instead_of_faking_success() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(),
        now_s=100.0,
    )

    result = runtime.execute_plan(_move_plan())

    assert result.status is MissionRuntimeStatus.BLOCKED
    assert result.results[0].status is ToolResultStatus.BLOCKED
    assert result.results[0].error["message"] == "physical move_to_clearance requires range-motion samples"


def test_physical_adapter_propagates_stop_estop_and_cancel_without_running_motion() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
    stopped = DeterministicMissionRuntime(registry, PhysicalCapabilityAdapters(stop_before="move_to_clearance"), now_s=100.0)
    estopped = DeterministicMissionRuntime(registry, PhysicalCapabilityAdapters(estop_before="move_to_clearance"), now_s=100.0)
    control = DeterministicMissionRuntime(registry, PhysicalCapabilityAdapters(), now_s=100.0)

    stop_result = stopped.execute_plan(_move_plan())
    estop_result = estopped.execute_plan(_move_plan())
    cancel_result = control.execute_plan(_control_plan("cancel"))

    assert stop_result.status is MissionRuntimeStatus.STOPPED
    assert stop_result.results[0].status is ToolResultStatus.STOPPED
    assert estop_result.status is MissionRuntimeStatus.ESTOPPED
    assert estop_result.results[0].status is ToolResultStatus.ESTOPPED
    assert cancel_result.status is MissionRuntimeStatus.CANCELLED
    assert cancel_result.results[0].status is ToolResultStatus.CANCELLED


def test_physical_adapter_rejects_malformed_unavailable_and_over_budget_motion_before_adapter_execution() -> None:
    unavailable = DeterministicMissionRuntime(
        build_default_registry(
            detector_classes=("shoe",), availability={"move_to_clearance": CapabilityAvailability.UNAVAILABLE}
        ),
        PhysicalCapabilityAdapters(),
        now_s=100.0,
    )
    malformed_plan = MissionPlan(
        goal=_move_plan().goal,
        invocations=(
            ToolInvocation(
                "move-1",
                "move_to_clearance",
                "1.0",
                {"clearance_m": "/cmd_vel", "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.5},
                approval=_grant(),
            ),
        ),
    )
    over_budget_plan = MissionPlan(
        goal=MissionGoal(
            goal_id="physical-move-over-budget",
            objective="move too far",
            success_criteria=("budget rejects untrusted travel",),
            budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0, max_travel_m=0.25),
        ),
        invocations=(
            ToolInvocation(
                "move-1",
                "move_to_clearance",
                "1.0",
                {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.5},
                approval=_grant(),
            ),
        ),
    )

    with pytest.raises(MissionValidationError, match="unavailable"):
        unavailable.execute_plan(_move_plan())
    with pytest.raises(MissionValidationError, match="direct ROS surface"):
        DeterministicMissionRuntime(build_default_registry(), PhysicalCapabilityAdapters(), now_s=100.0).execute_plan(
            malformed_plan
        )
    with pytest.raises(MissionValidationError, match="max_travel_m budget"):
        DeterministicMissionRuntime(build_default_registry(), PhysicalCapabilityAdapters(), now_s=100.0).execute_plan(
            over_budget_plan
        )


def test_physical_adapter_rejects_non_control_capabilities_until_real_adapters_exist() -> None:
    runtime = DeterministicMissionRuntime(
        build_default_registry(detector_classes=("shoe",)),
        PhysicalCapabilityAdapters(),
        now_s=100.0,
    )
    with pytest.raises(MissionValidationError, match="capture_observation is not bound to a healthy adapter"):
        runtime.execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="no-fake-perception",
                    objective="capture a real observation",
                    success_criteria=("no fake physical perception",),
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0),
                ),
                invocations=(ToolInvocation("capture", "capture_observation", "1.0", {"sensor": "camera"}),),
            )
        )
