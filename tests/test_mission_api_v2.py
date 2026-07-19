from __future__ import annotations

import pytest

from sphero_rvr_driver.mission_api import MissionApiVersion, MissionValidationError
from sphero_rvr_driver.mission_api_v2 import (
    ApprovalGrant,
    CapabilityAvailability,
    DeterministicMissionRuntime,
    FakeCapabilityAdapters,
    MissionBudgets,
    MissionGoal,
    MissionPlan,
    MissionRuntimeStatus,
    ToolInvocation,
    ToolResultStatus,
    build_canonical_shoe_mapping_v2_plan,
    build_default_v2_registry,
)


def _grant(now_s: float = 100.0) -> ApprovalGrant:
    return ApprovalGrant(
        approval_id="operator-approval-1",
        approved_by="operator:scott",
        approved_at_s=now_s,
        expires_at_s=now_s + 60.0,
        approval_class="supervised_motion",
    )


def test_v2_mission_goal_and_registry_are_generic_not_shoe_contracts() -> None:
    registry = build_default_v2_registry(detector_classes=("shoe", "backpack"))
    goal = MissionGoal(
        goal_id="goal-backpack-map",
        objective="Map the room and locate backpacks.",
        constraints={"area": "test_room", "coverage": "observed_only"},
        success_criteria=("semantic artifact references are produced",),
        execution_mode="replay",
        budgets=MissionBudgets(max_steps=8, max_runtime_s=120.0, max_travel_m=2.0),
        requested_artifacts=("semantic_map", "mission_summary"),
    )

    assert goal.api_version is MissionApiVersion.V2
    assert goal.to_json_dict()["objective"] == "Map the room and locate backpacks."
    assert "shoe" not in goal.to_json_dict()["constraints"]
    detect = registry.require("detect_objects", "1.0")
    assert detect.availability is CapabilityAvailability.AVAILABLE
    assert detect.argument_schema["properties"]["object_class"]["enum"] == ["shoe", "backpack"]
    assert detect.safety_class == "perception"
    assert detect.resource_ownership == ("detector",)
    assert "ROS topic" not in " ".join(detect.effects)


def test_registry_and_runtime_fail_closed_for_untrusted_invalid_or_unsupported_invocations() -> None:
    registry = build_default_v2_registry(
        detector_classes=("shoe",),
        availability={"rotate_scan": CapabilityAvailability.UNAVAILABLE},
    )
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)
    goal = MissionGoal(
        goal_id="goal-invalid",
        objective="exercise validation",
        success_criteria=("reject bad planner JSON",),
        budgets=MissionBudgets(max_steps=3, max_runtime_s=30.0, max_travel_m=0.5),
    )

    with pytest.raises(MissionValidationError, match="unknown tool"):
        runtime.execute_plan(MissionPlan(goal=goal, invocations=(ToolInvocation("a", "fly", "1.0", {}),)))

    with pytest.raises(MissionValidationError, match="object_class"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(ToolInvocation("b", "detect_objects", "1.0", {"object_class": "cat"}),),
            )
        )

    with pytest.raises(MissionValidationError, match="direct ROS"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(ToolInvocation("c", "capture_observation", "1.0", {"topic": "/cmd_vel"}),),
            )
        )

    unsafe_goal = MissionGoal(
        goal_id="unsafe-goal",
        objective="publish to /cmd_vel through the rover",
        success_criteria=("unsafe request rejected",),
        budgets=MissionBudgets(max_steps=1, max_runtime_s=5.0),
    )
    with pytest.raises(MissionValidationError, match="direct ROS"):
        runtime.execute_plan(
            MissionPlan(
                goal=unsafe_goal,
                invocations=(ToolInvocation("goal-unsafe", "capture_observation", "1.0", {"sensor": "replay"}),),
            )
        )

    with pytest.raises(MissionValidationError, match="unavailable"):
        runtime.execute_plan(MissionPlan(goal=goal, invocations=(ToolInvocation("d", "rotate_scan", "1.0", {"angle_deg": 45}),)))

    with pytest.raises(MissionValidationError, match="approval is stale"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(
                    ToolInvocation(
                        "e",
                        "move_to_clearance",
                        "1.0",
                        {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                        approval=_grant(now_s=1.0),
                    ),
                ),
            )
        )

    delayed_runtime = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(duration_by_tool={"capture_observation": 3.0}),
        now_s=100.0,
    )
    expires_before_second_tool = ApprovalGrant(
        approval_id="short-approval",
        approved_by="operator:scott",
        approved_at_s=100.0,
        expires_at_s=102.0,
        approval_class="supervised_motion",
    )
    with pytest.raises(MissionValidationError, match="approval is stale"):
        delayed_runtime.execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="approval-expires-mid-plan",
                    objective="approval expires before motion starts",
                    success_criteria=("motion is not authorized",),
                    budgets=MissionBudgets(max_steps=2, max_runtime_s=90.0, max_travel_m=0.5),
                ),
                invocations=(
                    ToolInvocation("pre", "capture_observation", "1.0", {"sensor": "replay"}),
                    ToolInvocation(
                        "motion",
                        "move_to_clearance",
                        "1.0",
                        {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                        approval=expires_before_second_tool,
                    ),
                ),
            )
        )


def test_runtime_rejects_unbounded_motion_excess_budgets_and_raw_movement_bypass() -> None:
    registry = build_default_v2_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)
    goal = MissionGoal(
        goal_id="goal-motion",
        objective="move until four inches from the object",
        success_criteria=("bounded range motion completes",),
        budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0, max_travel_m=0.25),
    )

    with pytest.raises(MissionValidationError, match="max_steps"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(
                    ToolInvocation("a", "capture_observation", "1.0", {"sensor": "replay"}),
                    ToolInvocation("b", "capture_observation", "1.0", {"sensor": "replay"}),
                ),
            )
        )

    with pytest.raises(MissionValidationError, match="max_travel_m"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(
                    ToolInvocation(
                        "c",
                        "move_to_clearance",
                        "1.0",
                        {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.5},
                        approval=_grant(),
                    ),
                ),
            )
        )

    with pytest.raises(MissionValidationError, match="max_travel_m"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(
                    ToolInvocation(
                        "explore-too-far",
                        "bounded_exploration_segment",
                        "1.0",
                        {"max_segments": 1, "segment_timeout_s": 3.0, "max_travel_m": 0.5},
                        approval=_grant(),
                    ),
                ),
            )
        )

    with pytest.raises(MissionValidationError, match="max_travel_m must be finite"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(
                    ToolInvocation(
                        "bad-travel",
                        "move_to_clearance",
                        "1.0",
                        {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": "far"},
                        approval=_grant(),
                    ),
                ),
            )
        )

    with pytest.raises(MissionValidationError, match="arguments must be an object"):
        ToolInvocation("bad-args", "move_to_clearance", "1.0", ["not", "a", "mapping"])  # type: ignore[arg-type]

    with pytest.raises(MissionValidationError, match="unknown tool"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(ToolInvocation("d", "raw_drive", "1.0", {"linear_x": 0.1}),),
            )
        )

    result = runtime.execute_plan(
        MissionPlan(
            goal=goal,
            invocations=(
                ToolInvocation(
                    "e",
                    "move_to_clearance",
                    "1.0",
                    {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                    approval=_grant(),
                ),
            ),
        )
    )
    assert result.status is MissionRuntimeStatus.COMPLETE
    assert result.results[0].observation == {"target_clearance_m": pytest.approx(0.1016)}


def test_runtime_enforces_trusted_budget_ceilings_not_planner_selected_limits() -> None:
    registry = build_default_v2_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)

    def plan_with(budgets: MissionBudgets) -> MissionPlan:
        return MissionPlan(
            goal=MissionGoal(
                goal_id="planner-budget-ceiling",
                objective="planner cannot raise runtime ceilings",
                success_criteria=("trusted ceiling wins",),
                budgets=budgets,
            ),
            invocations=(ToolInvocation("obs", "capture_observation", "1.0", {"sensor": "replay"}),),
        )

    with pytest.raises(MissionValidationError, match="trusted max_steps ceiling"):
        runtime.execute_plan(plan_with(MissionBudgets(max_steps=9, max_runtime_s=120.0, max_travel_m=2.0)))
    with pytest.raises(MissionValidationError, match="trusted max_runtime_s ceiling"):
        runtime.execute_plan(plan_with(MissionBudgets(max_steps=8, max_runtime_s=121.0, max_travel_m=2.0)))
    with pytest.raises(MissionValidationError, match="trusted max_travel_m ceiling"):
        runtime.execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="travel-ceiling",
                    objective="planner cannot raise travel ceilings",
                    success_criteria=("trusted travel ceiling wins",),
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0, max_travel_m=2.01),
                ),
                invocations=(
                    ToolInvocation(
                        "move",
                        "move_to_clearance",
                        "1.0",
                        {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                        approval=_grant(),
                    ),
                ),
            )
        )


def test_runtime_enforces_tool_timeout_with_deterministic_cleanup_audit() -> None:
    registry = build_default_v2_registry(detector_classes=("shoe",))
    result = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(duration_by_tool={"capture_observation": 10.0}),
        now_s=100.0,
    ).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="tool-timeout",
                objective="tool timeout is separate from mission timeout",
                success_criteria=("tool timeout is audited",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0),
            ),
            invocations=(ToolInvocation("slow", "capture_observation", "1.0", {"sensor": "replay"}),),
        )
    )

    assert result.status is MissionRuntimeStatus.TIMEOUT
    assert result.results[0].status is ToolResultStatus.TIMEOUT
    assert result.results[0].completed_at_s - result.results[0].started_at_s == pytest.approx(5.0)
    assert "cancellation cleanup completed" in result.results[0].error["message"]
    assert result.audit[0]["status"] == "timeout"

    failed_overrun = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(duration_by_tool={"detect_objects": 20.0}, fail_tools={"detect_objects": "detector offline"}),
        now_s=100.0,
    ).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="failed-tool-timeout",
                objective="timeout is authoritative over failed adapter status",
                success_criteria=("tool timeout is audited",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0),
            ),
            invocations=(ToolInvocation("failed-slow", "detect_objects", "1.0", {"object_class": "shoe"}),),
        )
    )
    assert failed_overrun.status is MissionRuntimeStatus.TIMEOUT
    assert failed_overrun.results[0].status is ToolResultStatus.TIMEOUT
    assert failed_overrun.results[0].completed_at_s - failed_overrun.results[0].started_at_s == pytest.approx(10.0)


def test_runtime_rejects_adapter_results_outside_declared_schema_boundary() -> None:
    registry = build_default_v2_registry(detector_classes=("shoe",))

    result = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(observation_by_tool={"move_to_clearance": {"target_clearance_m": 0.1016, "command_path": ["/cmd_vel"]}}),
        now_s=100.0,
    ).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="schema-boundary",
                objective="adapter results cannot leak internals",
                success_criteria=("typed result boundary fails closed",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0, max_travel_m=0.5),
            ),
            invocations=(
                ToolInvocation(
                    "move",
                    "move_to_clearance",
                    "1.0",
                    {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                    approval=_grant(),
                ),
            ),
        )
    )
    assert result.status is MissionRuntimeStatus.FAILED
    assert result.results[0].status is ToolResultStatus.FAILED
    assert result.results[0].error["message"] == "adapter result failed boundary validation"

    undeclared = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(observation_by_tool={"move_to_clearance": {"target_clearance_m": 0.1016, "undeclared": True}}),
        now_s=100.0,
    ).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="schema-undeclared",
                objective="adapter results cannot add undeclared fields",
                success_criteria=("typed result schema fails closed",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0, max_travel_m=0.5),
            ),
            invocations=(
                ToolInvocation(
                    "move",
                    "move_to_clearance",
                    "1.0",
                    {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                    approval=_grant(),
                ),
            ),
        )
    )
    assert undeclared.status is MissionRuntimeStatus.FAILED
    assert "adapter result failed schema validation" in undeclared.results[0].error["message"]

    failed_error = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(fail_tools={"detect_objects": "failed after /cmd_vel"}),
        now_s=100.0,
    ).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="error-boundary",
                objective="adapter errors cannot leak internals",
                success_criteria=("typed error boundary fails closed",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0),
            ),
            invocations=(ToolInvocation("detect", "detect_objects", "1.0", {"object_class": "shoe"}),),
        )
    )
    assert failed_error.status is MissionRuntimeStatus.FAILED
    assert failed_error.results[0].error["message"] == "adapter result failed boundary validation"

    failed_provenance = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(provenance_by_tool={"capture_observation": {"topic": "/cmd_vel"}}),
        now_s=100.0,
    ).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="provenance-boundary",
                objective="adapter provenance cannot leak internals",
                success_criteria=("typed provenance boundary fails closed",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0),
            ),
            invocations=(ToolInvocation("obs", "capture_observation", "1.0", {"sensor": "replay"}),),
        )
    )
    assert failed_provenance.status is MissionRuntimeStatus.FAILED
    assert failed_provenance.results[0].error["message"] == "adapter result failed boundary validation"


def test_numeric_boundaries_and_dependencies_are_deterministic_validation_errors() -> None:
    registry = build_default_v2_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)

    for max_travel_m in ("far", float("nan"), float("inf")):
        with pytest.raises(MissionValidationError, match="max_travel_m must be finite"):
            runtime.execute_plan(
                MissionPlan(
                    goal=MissionGoal(
                        goal_id="numeric-boundary",
                        objective="reject malformed travel before arithmetic",
                        success_criteria=("validation error",),
                        budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0, max_travel_m=0.5),
                    ),
                    invocations=(
                        ToolInvocation(
                            "move",
                            "move_to_clearance",
                            "1.0",
                            {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": max_travel_m},
                            approval=_grant(),
                        ),
                    ),
                )
            )

    with pytest.raises(MissionValidationError, match="max_runtime_s"):
        MissionBudgets(max_steps=1, max_runtime_s="forever")  # type: ignore[arg-type]
    with pytest.raises(MissionValidationError, match="max_runtime_s"):
        MissionBudgets(max_steps=1, max_runtime_s="5")  # type: ignore[arg-type]
    with pytest.raises(MissionValidationError, match="max_travel_m"):
        MissionBudgets(max_steps=1, max_runtime_s=30.0, max_travel_m="0.5")  # type: ignore[arg-type]
    with pytest.raises(MissionValidationError, match="dependencies are not supported"):
        MissionPlan(
            goal=MissionGoal(
                goal_id="dependencies",
                objective="dependency semantics must not be ignored",
                success_criteria=("dependency rejection",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0),
            ),
            invocations=(ToolInvocation("obs", "capture_observation", "1.0", {"sensor": "replay"}),),
            dependencies=(("obs", "later"),),
        )


def test_three_heterogeneous_v2_plans_execute_with_fake_replay_adapters() -> None:
    registry = build_default_v2_registry(detector_classes=("shoe", "backpack"))
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)

    shoe = runtime.execute_plan(build_canonical_shoe_mapping_v2_plan(goal_id="shoe-fixture", approval=_grant()))
    assert shoe.status is MissionRuntimeStatus.COMPLETE
    assert [result.invocation.tool_id for result in shoe.results] == [
        "map_localize",
        "bounded_exploration_segment",
        "capture_observation",
        "detect_objects",
        "project_detections_to_map",
        "generate_semantic_artifacts",
    ]
    assert shoe.results[-1].artifact_refs["semantic_map"].endswith("semantic_map.json")
    assert shoe.audit[0]["api_version"] == "mission_api.v2"

    backpack_goal = MissionGoal(
        goal_id="backpack-goal",
        objective="Map visible backpacks using the installed detector plugin.",
        success_criteria=("backpack detections are projected into map frame",),
        budgets=MissionBudgets(max_steps=4, max_runtime_s=60.0, max_travel_m=1.0),
    )
    backpack = runtime.execute_plan(
        MissionPlan(
            goal=backpack_goal,
            invocations=(
                ToolInvocation("b1", "capture_observation", "1.0", {"sensor": "replay"}),
                ToolInvocation("b2", "detect_objects", "1.0", {"object_class": "backpack"}),
                ToolInvocation("b3", "project_detections_to_map", "1.0", {"target_frame": "map"}),
                ToolInvocation("b4", "generate_semantic_artifacts", "1.0", {"artifact_kinds": ["semantic_map", "mission_summary"]}),
            ),
        )
    )
    assert backpack.status is MissionRuntimeStatus.COMPLETE
    assert backpack.results[1].observation["detections_ref"].endswith("backpack_detections.json")
    assert backpack.results[-1].artifact_refs["mission_summary"].endswith("mission_summary.md")

    clearance_goal = MissionGoal(
        goal_id="clearance-goal",
        objective="Move until four inches from the object and report the observation.",
        success_criteria=("range motion stops at requested clearance", "observation captured"),
        budgets=MissionBudgets(max_steps=3, max_runtime_s=20.0, max_travel_m=0.25),
    )
    clearance = runtime.execute_plan(
        MissionPlan(
            goal=clearance_goal,
            invocations=(
                ToolInvocation(
                    "m1",
                    "move_to_clearance",
                    "1.0",
                    {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 4.0, "max_travel_m": 0.2},
                    approval=_grant(),
                ),
                ToolInvocation("m2", "capture_observation", "1.0", {"sensor": "replay"}),
                ToolInvocation("m3", "generate_semantic_artifacts", "1.0", {"artifact_kinds": ["mission_summary"]}),
            ),
        )
    )
    assert clearance.status is MissionRuntimeStatus.COMPLETE
    assert clearance.results[0].observation["target_clearance_m"] == pytest.approx(0.1016)


def test_runtime_cancellation_timeout_failure_blocked_stop_and_estop_are_deterministic_and_audited() -> None:
    registry = build_default_v2_registry(detector_classes=("shoe",))

    cancelled = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(cancel_after=1), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_v2_plan(goal_id="cancelled", approval=_grant())
    )
    assert cancelled.status is MissionRuntimeStatus.CANCELLED
    assert cancelled.results[-1].status is ToolResultStatus.CANCELLED
    assert cancelled.audit[-1]["status"] == "cancelled"

    direct_stop = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="direct-stop",
                objective="STOP immediately",
                success_criteria=("STOP is latched",),
                budgets=MissionBudgets(max_steps=2, max_runtime_s=5.0),
            ),
            invocations=(
                ToolInvocation("s1", "pause_cancel_stop_estop", "1.0", {"action": "stop"}),
                ToolInvocation("s2", "capture_observation", "1.0", {"sensor": "replay"}),
            ),
        )
    )
    assert direct_stop.status is MissionRuntimeStatus.STOPPED
    assert [result.invocation.tool_id for result in direct_stop.results] == ["pause_cancel_stop_estop"]

    direct_estop = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="direct-estop",
                objective="ESTOP immediately",
                success_criteria=("ESTOP is latched",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=5.0),
            ),
            invocations=(ToolInvocation("e1", "pause_cancel_stop_estop", "1.0", {"action": "estop"}),),
        )
    )
    assert direct_estop.status is MissionRuntimeStatus.ESTOPPED

    timeout = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(duration_by_tool={"capture_observation": 20.0}), now_s=100.0).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="timeout",
                objective="timeout",
                success_criteria=("timeout is audited",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=5.0),
            ),
            invocations=(ToolInvocation("t1", "capture_observation", "1.0", {"sensor": "replay"}),),
        )
    )
    assert timeout.status is MissionRuntimeStatus.TIMEOUT
    assert timeout.results[-1].status is ToolResultStatus.TIMEOUT

    tool_timeout = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(duration_by_tool={"capture_observation": 20.0}), now_s=100.0).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="tool-timeout",
                objective="tool timeout",
                success_criteria=("tool timeout is audited independently of mission runtime",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=100.0),
            ),
            invocations=(ToolInvocation("tt1", "capture_observation", "1.0", {"sensor": "replay"}),),
        )
    )
    assert tool_timeout.status is MissionRuntimeStatus.TIMEOUT
    assert tool_timeout.results[-1].status is ToolResultStatus.TIMEOUT
    assert tool_timeout.results[-1].completed_at_s - tool_timeout.results[-1].started_at_s == pytest.approx(5.0)

    insufficient_remaining_budget = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="remaining-budget-timeout",
                objective="do not start a tool that cannot finish within remaining budget",
                success_criteria=("second tool is not started",),
                budgets=MissionBudgets(max_steps=2, max_runtime_s=6.0, max_travel_m=0.5),
            ),
            invocations=(
                ToolInvocation("r1", "capture_observation", "1.0", {"sensor": "replay"}),
                ToolInvocation(
                    "r2",
                    "move_to_clearance",
                    "1.0",
                    {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 10.0, "max_travel_m": 0.2},
                    approval=_grant(),
                ),
            ),
        )
    )
    assert insufficient_remaining_budget.status is MissionRuntimeStatus.TIMEOUT
    assert insufficient_remaining_budget.results[-1].invocation.correlation_id == "r2"

    failed = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(fail_tools={"detect_objects": "detector offline"}), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_v2_plan(goal_id="failed", approval=_grant())
    )
    assert failed.status is MissionRuntimeStatus.FAILED
    assert failed.results[-1].error["message"] == "detector offline"

    blocked = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(block_tools={"map_localize": "map unavailable"}), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_v2_plan(goal_id="blocked", approval=_grant())
    )
    assert blocked.status is MissionRuntimeStatus.BLOCKED
    assert blocked.results[-1].error["message"] == "map unavailable"

    stopped = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(stop_before="capture_observation"), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_v2_plan(goal_id="stopped", approval=_grant())
    )
    assert stopped.status is MissionRuntimeStatus.STOPPED
    assert stopped.audit[-1]["status"] == "stopped"

    estopped = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(estop_before="capture_observation"), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_v2_plan(goal_id="estopped", approval=_grant())
    )
    assert estopped.status is MissionRuntimeStatus.ESTOPPED
    assert estopped.audit[-1]["status"] == "estopped"
