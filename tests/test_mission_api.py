from __future__ import annotations

import pytest

import sphero_rvr_driver.mission_api as mission_api_module
import sphero_rvr_driver.mission_controls as mission_controls_module
import sphero_rvr_driver.physical_capability_adapters as physical_adapters_module

from sphero_rvr_driver.mission_api import (
    ApprovalGrant,
    CapabilityAvailability,
    CapabilityRegistry,
    CriterionKind,
    DeterministicMissionRuntime,
    FakeCapabilityAdapters,
    MissionBudgets,
    MissionGoal,
    MissionPlan,
    MissionApiVersion,
    MissionRuntimeStatus,
    MissionValidationError,
    SuccessCriterion,
    ToolDefinition,
    ToolInvocation,
    ToolResultStatus,
    _arguments_digest,
    build_canonical_shoe_mapping_plan,
    build_default_registry,
    _issue_approval_grant,
)
from sphero_rvr_driver.physical_capability_adapters import PhysicalCapabilityAdapters


def _grant(
    now_s: float = 100.0,
    *,
    expires_at_s: float | None = None,
    mission_id: str = "goal-motion",
    approval_id: str = "operator-approval-1",
    tool_id: str = "move_to_clearance",
    correlation_id: str = "e",
    arguments: dict | None = None,
) -> ApprovalGrant:
    if arguments is None:
        arguments = {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2}
    return _issue_approval_grant(
        approval_id=approval_id,
        approved_by="operator:scott",
        approved_at_s=now_s,
        expires_at_s=now_s + 60.0 if expires_at_s is None else expires_at_s,
        approval_class="supervised_motion",
        mission_id=mission_id,
        issued_to="mission-runtime",
        tool_id=tool_id,
        correlation_id=correlation_id,
        arguments_digest=_arguments_digest(arguments),
        principal="operator:scott",
    )


def _shoe_grant(mission_id: str, approval_id: str = "shoe-motion-approval") -> ApprovalGrant:
    return _grant(
        mission_id=mission_id,
        approval_id=approval_id,
        tool_id="bounded_exploration_segment",
        correlation_id="shoe-2",
        arguments={"max_segments": 2, "segment_timeout_s": 8.0, "max_travel_m": 1.0},
    )


def test_runtime_trust_authority_has_no_public_caller_minting_helpers() -> None:
    assert not hasattr(mission_api_module, "issue_approval_grant")
    assert not hasattr(mission_api_module, "physical_adapter_authority")
    assert not hasattr(mission_controls_module, "issue_physical_start_approval")
    assert not hasattr(physical_adapters_module, "physical_adapter_authority")


def test_replay_approval_authority_cannot_authorize_physical_execution() -> None:
    arguments = {"distance_m": 0.1, "speed_mps": 0.05, "timeout_s": 3.0}
    approval = _issue_approval_grant(
        approval_id="replay-only",
        approved_by="replay-supervisor",
        approved_at_s=100.0,
        expires_at_s=120.0,
        approval_class="supervised_motion",
        mission_id="physical-with-replay-authority",
        tool_id="move_distance",
        correlation_id="move",
        arguments_digest=_arguments_digest(arguments),
        principal="replay-supervisor",
    )
    plan = MissionPlan(
        goal=MissionGoal(
            goal_id="physical-with-replay-authority",
            objective="replay authority cannot cross into physical execution",
            success_criteria=(
                SuccessCriterion("move", "move completes", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),
            ),
            execution_mode="physical",
            budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0, max_travel_m=0.5),
        ),
        invocations=(ToolInvocation("move", "move_distance", "1.0", arguments, approval=approval),),
    )

    with pytest.raises(MissionValidationError, match="approval execution mode binding mismatch"):
        DeterministicMissionRuntime(
            build_default_registry(detector_classes=("shoe",)),
            PhysicalCapabilityAdapters(),
            now_s=100.0,
        ).execute_plan(plan)


def test_v2_mission_goal_and_registry_are_generic_not_shoe_contracts() -> None:
    registry = build_default_registry(detector_classes=("shoe", "backpack"))
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
    registry = build_default_registry(
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

    duplicate = registry.require("capture_observation", "1.0")
    with pytest.raises(MissionValidationError, match="duplicate tool definition"):
        CapabilityRegistry((duplicate, duplicate))

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
                        approval=_grant(now_s=1.0, mission_id="goal-invalid"),
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
        mission_id="approval-expires-mid-plan",
        issued_to="mission-runtime",
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
    registry = build_default_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)
    goal = MissionGoal(
        goal_id="goal-motion",
        objective="move until four inches from the object",
        success_criteria=(SuccessCriterion("move-complete", "move_to_clearance completed", CriterionKind.TOOL_COMPLETE, tool_id="move_to_clearance"),),
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

    with pytest.raises(MissionValidationError, match="turn_angle.angle_deg must be non-zero"):
        runtime.execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="zero-turn",
                    objective="zero-degree turn should be rejected before adapter execution",
                    success_criteria=("validation error",),
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
                ),
                invocations=(
                    ToolInvocation(
                        "zero-turn",
                        "turn_angle",
                        "1.0",
                        {"angle_deg": 0.0, "angular_speed_deg_s": 45.0, "timeout_s": 5.0},
                        approval=_grant(),
                    ),
                ),
            )
        )

    move_distance = registry.require("move_distance", "1.0")
    turn_angle = registry.require("turn_angle", "1.0")
    assert move_distance.argument_schema["properties"]["distance_m"]["minimum"] == 0.01
    assert move_distance.approval_class == "supervised_motion"
    assert turn_angle.argument_schema["properties"]["angle_deg"]["maximum"] == 180.0
    assert turn_angle.resource_ownership == ("odom_motion",)

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


def test_physical_execution_mode_rejects_fake_replay_adapters_and_exposes_live_capability_state() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
    physical_goal = MissionGoal(
        goal_id="physical-mode-fake-adapter",
        objective="physical execution cannot use replay adapters",
        success_criteria=("adapter authority is physical",),
        execution_mode="physical",
        budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
    )
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)

    with pytest.raises(MissionValidationError, match="physical execution requires physical adapters"):
        runtime.execute_plan(
            MissionPlan(goal=physical_goal, invocations=(ToolInvocation("obs", "capture_observation", "1.0", {"sensor": "replay"}),))
        )

    spoofed_fake_runtime = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(execution_mode="physical"),
        now_s=100.0,
    )
    with pytest.raises(MissionValidationError, match="physical execution requires physical adapters"):
        spoofed_fake_runtime.execute_plan(
            MissionPlan(
                goal=physical_goal,
                invocations=(ToolInvocation("status", "query_status_telemetry", "1.0", {}),),
            )
        )

    unbound_physical_approval = ApprovalGrant(
        approval_id="browser-replay-token",
        approved_by="mcp-local-replay-supervisor",
        approved_at_s=100.0,
        expires_at_s=120.0,
        approval_class="supervised_motion",
    )
    with pytest.raises(MissionValidationError, match="mission and runtime identity binding"):
        DeterministicMissionRuntime(registry, PhysicalCapabilityAdapters(), now_s=100.0).execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="physical-unbound-approval",
                    objective="physical motion cannot use replay-minted authority",
                    success_criteria=("unbound approval rejected",),
                    execution_mode="physical",
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0, max_travel_m=0.5),
                ),
                invocations=(
                    ToolInvocation(
                        "move",
                        "move_distance",
                        "1.0",
                        {"distance_m": 0.1, "speed_mps": 0.05, "timeout_s": 3.0},
                        approval=unbound_physical_approval,
                    ),
                ),
            )
        )

    state = runtime.capability_state()
    capture = state["capture_observation@1.0"]
    assert capture["declared"] is True
    assert capture["bound"] is True
    assert capture["healthy"] is True
    assert capture["mode"] == "replay"
    assert capture["deployed_sha"]
    assert capture["evidence_level"] == "deterministic_replay"


def test_runtime_enforces_trusted_budget_ceilings_not_planner_selected_limits() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
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


def test_runtime_rejects_approval_replay_wrong_identity_and_wrong_mission_binding() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)
    goal = MissionGoal(
        goal_id="approval-binding",
        objective="approval envelope is bound to mission and caller",
        success_criteria=("bad approvals fail",),
        budgets=MissionBudgets(max_steps=2, max_runtime_s=30.0, max_travel_m=0.5),
    )
    wrong_mission = ApprovalGrant(
        approval_id="wrong-mission",
        approved_by="operator:scott",
        approved_at_s=100.0,
        expires_at_s=120.0,
        approval_class="supervised_motion",
        mission_id="other-mission",
        issued_to="mission-runtime",
    )
    with pytest.raises(MissionValidationError, match="mission binding"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(
                    ToolInvocation(
                        "move",
                        "move_to_clearance",
                        "1.0",
                        {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                        approval=wrong_mission,
                    ),
                ),
            )
        )

    unbound = ApprovalGrant(
        approval_id="unbound-approval",
        approved_by="operator:scott",
        approved_at_s=100.0,
        expires_at_s=120.0,
        approval_class="supervised_motion",
    )
    with pytest.raises(MissionValidationError, match="mission and runtime identity binding"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(
                    ToolInvocation(
                        "unbound-move",
                        "move_to_clearance",
                        "1.0",
                        {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                        approval=unbound,
                    ),
                ),
            )
        )

    replayed = _grant(
        mission_id="approval-binding",
        approval_id="replayed-approval",
        correlation_id="move-1",
        arguments={"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
    )
    with pytest.raises(MissionValidationError, match="approval replay"):
        runtime.execute_plan(
            MissionPlan(
                goal=goal,
                invocations=(
                    ToolInvocation(
                        "move-1",
                        "move_to_clearance",
                        "1.0",
                        {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                        approval=replayed,
                    ),
                    ToolInvocation(
                        "move-1",
                        "move_to_clearance",
                        "1.0",
                        {"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 3.0, "max_travel_m": 0.2},
                        approval=replayed,
                    ),
                ),
            )
        )


def test_required_artifacts_and_success_criteria_gate_v2_complete_status() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)
    result = runtime.execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="missing-artifacts",
                objective="cannot complete without requested artifacts",
                success_criteria=("mission_summary artifact is returned",),
                requested_artifacts=("mission_summary",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(ToolInvocation("obs", "capture_observation", "1.0", {"sensor": "replay"}),),
        )
    )

    assert result.status is MissionRuntimeStatus.FAILED
    assert result.results[-1].error["message"] == "mission completion missing requested artifacts: mission_summary"

    fake_success = runtime.execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="fake-success-criteria",
                objective="cannot complete by assuming criteria are satisfied",
                success_criteria=("the rover has found the secret purple unicorn",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(ToolInvocation("obs", "capture_observation", "1.0", {"sensor": "replay"}),),
        )
    )
    assert fake_success.status is MissionRuntimeStatus.FAILED
    assert "mission completion missing success criteria evidence" in fake_success.results[-1].error["message"]


def test_timeout_interrupts_synchronous_hung_adapter_without_late_side_effects(tmp_path) -> None:
    import time

    marker = tmp_path / "late-side-effect"
    cancelled: list[bool] = []

    class HungHandle:
        def wait(self, timeout_s):
            del timeout_s
            return None

        def cancel(self):
            cancelled.append(True)

        def cleanup(self, timeout_s):
            del timeout_s
            return True

        def wait_idle(self, timeout_s):
            del timeout_s
            return True

    class NonCooperativeHungAdapter:
        execution_mode = "replay"
        authority_kind = "replay"
        healthy = True
        evidence_level = "test_hung_adapter"
        deployed_sha = "test"

        def execute(self, invocation, definition, *, started_at_s, index):
            time.sleep(1.0)
            marker.write_text("adapter kept running after timeout")
            raise AssertionError("unreachable")

    class CooperativeHungAdapter(NonCooperativeHungAdapter):
        cooperative_execution = True

        def begin_execution(self, invocation, definition, *, started_at_s, index):
            del invocation, definition, started_at_s, index
            return HungHandle()

    registry = CapabilityRegistry(
        (
            ToolDefinition(
                "hung_tool",
                "1.0",
                {"type": "object", "properties": {}, "required": [], "additionalProperties": False},
                {"type": "object", "properties": {}, "required": [], "additionalProperties": False},
                timeout_s=0.05,
            ),
        )
    )
    with pytest.raises(MissionValidationError, match="cooperative cancel"):
        DeterministicMissionRuntime(registry, NonCooperativeHungAdapter(), now_s=100.0).execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="non-cooperative-hung-adapter",
                    objective="hung adapters must expose an interruptible handle",
                    success_criteria=("timeout is audited",),
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
                ),
                invocations=(ToolInvocation("hung", "hung_tool", "1.0", {}),),
            )
        )

    result = DeterministicMissionRuntime(registry, CooperativeHungAdapter(), now_s=100.0).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="hung-adapter-timeout",
                objective="hung adapters cannot keep running after timeout",
                success_criteria=("timeout is audited",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(ToolInvocation("hung", "hung_tool", "1.0", {}),),
        )
    )

    assert result.status is MissionRuntimeStatus.TIMEOUT
    time.sleep(1.2)
    assert cancelled == [True]
    assert not marker.exists()


def test_runtime_enforces_tool_timeout_with_deterministic_cleanup_audit() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
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
    registry = build_default_registry(detector_classes=("shoe",))

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
                    approval=_grant(mission_id="schema-boundary", correlation_id="move"),
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
                    approval=_grant(mission_id="schema-undeclared", correlation_id="move"),
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
    registry = build_default_registry(detector_classes=("shoe",))
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
    registry = build_default_registry(detector_classes=("shoe", "backpack"))
    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)

    shoe = runtime.execute_plan(build_canonical_shoe_mapping_plan(goal_id="shoe-fixture", approval=_shoe_grant("shoe-fixture")))
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
        success_criteria=(
            SuccessCriterion("detected", "backpack detections completed", CriterionKind.TOOL_COMPLETE, tool_id="detect_objects"),
            SuccessCriterion("projected", "detections projected into map", CriterionKind.TOOL_COMPLETE, tool_id="project_detections_to_map"),
        ),
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
        success_criteria=(
            SuccessCriterion("clearance", "move_to_clearance completed", CriterionKind.TOOL_COMPLETE, tool_id="move_to_clearance"),
            SuccessCriterion("observation", "capture_observation completed", CriterionKind.TOOL_COMPLETE, tool_id="capture_observation"),
            SuccessCriterion("summary", "mission summary artifact exists", CriterionKind.ARTIFACT_PRESENT, field="mission_summary"),
        ),
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
                    approval=_grant(
                        mission_id="clearance-goal",
                        approval_id="clearance-approval",
                        correlation_id="m1",
                        arguments={"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 4.0, "max_travel_m": 0.2},
                    ),
                ),
                ToolInvocation("m2", "capture_observation", "1.0", {"sensor": "replay"}),
                ToolInvocation("m3", "generate_semantic_artifacts", "1.0", {"artifact_kinds": ["mission_summary"]}),
            ),
        )
    )
    assert clearance.status is MissionRuntimeStatus.COMPLETE
    assert clearance.results[0].observation["target_clearance_m"] == pytest.approx(0.1016)


def test_runtime_cancellation_timeout_failure_blocked_stop_and_estop_are_deterministic_and_audited() -> None:
    registry = build_default_registry(detector_classes=("shoe",))

    cancelled = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(cancel_after=1), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_plan(goal_id="cancelled", approval=_shoe_grant("cancelled"))
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
                    approval=_grant(
                        mission_id="remaining-budget-timeout",
                        correlation_id="r2",
                        arguments={"clearance_m": 0.1016, "speed_mps": 0.05, "timeout_s": 10.0, "max_travel_m": 0.2},
                    ),
                ),
            ),
        )
    )
    assert insufficient_remaining_budget.status is MissionRuntimeStatus.TIMEOUT
    assert insufficient_remaining_budget.results[-1].invocation.correlation_id == "r2"

    failed = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(fail_tools={"detect_objects": "detector offline"}), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_plan(goal_id="failed", approval=_shoe_grant("failed"))
    )
    assert failed.status is MissionRuntimeStatus.FAILED
    assert failed.results[-1].error["message"] == "detector offline"

    blocked = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(block_tools={"map_localize": "map unavailable"}), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_plan(goal_id="blocked", approval=_shoe_grant("blocked"))
    )
    assert blocked.status is MissionRuntimeStatus.BLOCKED
    assert blocked.results[-1].error["message"] == "map unavailable"

    stopped = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(stop_before="capture_observation"), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_plan(goal_id="stopped", approval=_shoe_grant("stopped"))
    )
    assert stopped.status is MissionRuntimeStatus.STOPPED
    assert stopped.audit[-1]["status"] == "stopped"

    estopped = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(estop_before="capture_observation"), now_s=100.0).execute_plan(
        build_canonical_shoe_mapping_plan(goal_id="estopped", approval=_shoe_grant("estopped"))
    )
    assert estopped.status is MissionRuntimeStatus.ESTOPPED
    assert estopped.audit[-1]["status"] == "estopped"


def test_latest_runtime_trust_blockers_are_regressed() -> None:
    registry = build_default_registry(detector_classes=("shoe",))

    physical_goal = MissionGoal(
        goal_id="spoofed-physical",
        objective="public strings cannot mint physical adapter authority",
        success_criteria=(SuccessCriterion("status", "status completed", CriterionKind.TOOL_COMPLETE, tool_id="query_status_telemetry"),),
        execution_mode="physical",
        budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
    )
    with pytest.raises(MissionValidationError, match="physical execution requires physical adapters"):
        DeterministicMissionRuntime(
            registry,
            FakeCapabilityAdapters(execution_mode="physical", authority_kind="physical"),
            now_s=100.0,
        ).execute_plan(MissionPlan(goal=physical_goal, invocations=(ToolInvocation("status", "query_status_telemetry", "1.0", {}),)))

    spoofed_marker = FakeCapabilityAdapters(
        execution_mode="physical",
        authority_kind="physical",
        provenance_by_tool={"query_status_telemetry": {"adapter": "physical/spoofed", "deterministic": True}},
    )
    spoofed_marker.physical_authority = object()  # type: ignore[attr-defined]
    with pytest.raises(MissionValidationError, match="physical execution requires physical adapters"):
        DeterministicMissionRuntime(registry, spoofed_marker, now_s=100.0).execute_plan(
            MissionPlan(goal=physical_goal, invocations=(ToolInvocation("status", "query_status_telemetry", "1.0", {}),))
        )

    SpoofPhysicalAdapters = type(
        "PhysicalCapabilityAdapters",
        (),
        {
            "__module__": "sphero_rvr_driver.physical_capability_adapters",
            "cooperative_execution": True,
            "execution_mode": "physical",
            "authority_kind": "physical",
            "healthy": True,
            "evidence_level": "live_bounded_physical",
            "supported_tool_ids": ("query_status_telemetry",),
            "satisfied_preconditions": (),
            "begin_execution": lambda self, invocation, definition, *, started_at_s, index: FakeCapabilityAdapters(
                provenance_by_tool={"query_status_telemetry": {"adapter": "physical/spoofed", "deterministic": True}}
            ).begin_execution(invocation, definition, started_at_s=started_at_s, index=index),
        },
    )
    with pytest.raises(MissionValidationError, match="physical execution requires physical adapters"):
        DeterministicMissionRuntime(registry, SpoofPhysicalAdapters(), now_s=100.0).execute_plan(  # type: ignore[arg-type]
            MissionPlan(goal=physical_goal, invocations=(ToolInvocation("status", "query_status_telemetry", "1.0", {}),))
        )

    physical_state = DeterministicMissionRuntime(registry, PhysicalCapabilityAdapters(), now_s=100.0).capability_state()
    assert physical_state["map_localize@1.0"]["bound"] is False
    assert physical_state["map_localize@1.0"]["healthy"] is False
    assert physical_state["move_distance@1.0"]["bound"] is True

    physical_map_plan = MissionPlan(goal=physical_goal, invocations=(ToolInvocation("map", "map_localize", "1.0", {"mode": "live"}),))
    with pytest.raises(MissionValidationError, match="not bound to a healthy adapter"):
        DeterministicMissionRuntime(registry, PhysicalCapabilityAdapters(), now_s=100.0).execute_plan(physical_map_plan)

    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)
    fake_text_success = runtime.execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="telemetry-banana",
                objective="text token aliases cannot mint success",
                success_criteria=("telemetry banana",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(ToolInvocation("status", "query_status_telemetry", "1.0", {}),),
        )
    )
    assert fake_text_success.status is MissionRuntimeStatus.FAILED

    alias_only_text_success = runtime.execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="telemetry-alias",
                objective="legacy text criteria must come from runtime evidence, not aliases",
                success_criteria=("status telemetry state completes",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(ToolInvocation("status", "query_status_telemetry", "1.0", {}),),
        )
    )
    assert alias_only_text_success.status is MissionRuntimeStatus.FAILED

    nonexistent_artifact = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(
            observation_by_tool={"generate_semantic_artifacts": {"artifact_refs": {"mission_summary": "artifacts/missing/does-not-exist.md"}}},
            artifact_refs_by_tool={"generate_semantic_artifacts": {"mission_summary": "artifacts/missing/does-not-exist.md"}},
        ),
        now_s=100.0,
    ).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="nonexistent-artifact",
                objective="artifact refs must exist and be provenance-valid",
                success_criteria=(SuccessCriterion("mission-summary", "mission summary artifact exists", CriterionKind.ARTIFACT_PRESENT, field="mission_summary"),),
                requested_artifacts=("mission_summary",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(ToolInvocation("artifact", "generate_semantic_artifacts", "1.0", {"artifact_kinds": ["mission_summary"]}),),
        )
    )
    assert nonexistent_artifact.status is MissionRuntimeStatus.FAILED
    assert "artifact_refs failed provenance validation" in nonexistent_artifact.results[-1].error["message"]

    missing_precondition_args = {"distance_m": 0.1, "speed_mps": 0.05, "timeout_s": 3.0}
    missing_precondition_grant = ApprovalGrant(
        approval_id="missing-precondition-grant",
        approved_by="operator:scott",
        approved_at_s=100.0,
        expires_at_s=120.0,
        approval_class="supervised_motion",
        mission_id="missing-precondition",
        issued_to="mission-runtime",
        tool_id="move_distance",
        correlation_id="move",
        arguments_digest=_arguments_digest(missing_precondition_args),
        principal="operator:scott",
    )
    with pytest.raises(MissionValidationError, match="precondition not satisfied: collision stop clear"):
        DeterministicMissionRuntime(
            registry,
            PhysicalCapabilityAdapters(satisfied_preconditions=("fresh odometry",)),
            now_s=100.0,
        ).execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="missing-precondition",
                    objective="preconditions are enforced before physical execution",
                    success_criteria=(SuccessCriterion("move-complete", "move completed", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),),
                    execution_mode="physical",
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0, max_travel_m=0.5),
                ),
                invocations=(ToolInvocation("move", "move_distance", "1.0", missing_precondition_args, approval=missing_precondition_grant),),
            )
        )


@pytest.mark.parametrize(
    "wrong_type_ref",
    (
        "artifacts/vs06_semantic_map/mission_summary.md",
        "artifacts/vs02_slam_replay_fixture_map/manifest.json",
    ),
)
def test_artifact_kind_contract_rejects_wrong_extension_and_content_type(wrong_type_ref: str) -> None:
    registry = build_default_registry(detector_classes=("shoe",))
    result = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(
            observation_by_tool={"generate_semantic_artifacts": {"artifact_refs": {"semantic_map": wrong_type_ref}}},
            artifact_refs_by_tool={"generate_semantic_artifacts": {"semantic_map": wrong_type_ref}},
        ),
        now_s=100.0,
    ).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="wrong-artifact-kind",
                objective="artifact kinds require matching file types",
                success_criteria=(
                    SuccessCriterion(
                        "semantic-map",
                        "semantic map artifact exists",
                        CriterionKind.ARTIFACT_PRESENT,
                        field="semantic_map",
                    ),
                ),
                requested_artifacts=("semantic_map",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(
                ToolInvocation(
                    "artifact",
                    "generate_semantic_artifacts",
                    "1.0",
                    {"artifact_kinds": ["semantic_map"]},
                ),
            ),
        )
    )

    assert result.status is MissionRuntimeStatus.FAILED
    assert "artifact_refs failed provenance validation" in result.results[-1].error["message"]


def test_cumulative_ledger_and_physical_approval_envelope_survive_replans() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(),
        now_s=100.0,
        budget_ceilings=MissionBudgets(max_steps=8, max_runtime_s=120.0, max_travel_m=2.0),
    )

    def travel_plan(goal_id: str, correlation_id: str, approval_id: str, *, distance_m: float = 1.5) -> MissionPlan:
        args = {"distance_m": distance_m, "speed_mps": 0.05, "timeout_s": 3.0}
        return MissionPlan(
            goal=MissionGoal(
                goal_id=goal_id,
                objective="cumulative travel ledger survives replans",
                success_criteria=(SuccessCriterion("move-complete", "move completes", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=30.0, max_travel_m=distance_m),
            ),
            invocations=(
                ToolInvocation(
                    correlation_id,
                    "move_distance",
                    "1.0",
                    args,
                    approval=_grant(
                        mission_id=goal_id,
                        approval_id=approval_id,
                        tool_id="move_distance",
                        correlation_id=correlation_id,
                        arguments=args,
                    ),
                ),
            ),
        )

    assert runtime.execute_plan(travel_plan("ledger-one", "move-1", "grant-1")).status is MissionRuntimeStatus.COMPLETE
    with pytest.raises(MissionValidationError, match="cumulative max_travel_m"):
        runtime.execute_plan(travel_plan("ledger-two", "move-2", "grant-2"))
    with pytest.raises(MissionValidationError, match="approval replay"):
        runtime.execute_plan(travel_plan("ledger-one", "move-1b", "grant-1", distance_m=0.1))

    tool_call_runtime = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(),
        now_s=100.0,
        budget_ceilings=MissionBudgets(max_steps=8, max_runtime_s=120.0, max_tool_calls=1, max_observations=1),
    )

    def status_plan(goal_id: str, correlation_id: str) -> MissionPlan:
        return MissionPlan(
            goal=MissionGoal(
                goal_id=goal_id,
                objective="cumulative non-travel ledgers survive replans",
                success_criteria=(SuccessCriterion("status", "status completed", CriterionKind.TOOL_COMPLETE, tool_id="query_status_telemetry"),),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0, max_tool_calls=1, max_observations=1),
            ),
            invocations=(ToolInvocation(correlation_id, "query_status_telemetry", "1.0", {}),),
        )

    assert tool_call_runtime.execute_plan(status_plan("ledger-status-one", "status-1")).status is MissionRuntimeStatus.COMPLETE
    with pytest.raises(MissionValidationError, match="cumulative max_tool_calls"):
        tool_call_runtime.execute_plan(status_plan("ledger-status-two", "status-2"))

    args = {"distance_m": 0.1, "speed_mps": 0.05, "timeout_s": 3.0}
    physical_approval = ApprovalGrant(
        approval_id="bad-envelope",
        approved_by="operator:scott",
        approved_at_s=100.0,
        expires_at_s=120.0,
        approval_class="supervised_motion",
        mission_id="physical-envelope",
        issued_to="mission-runtime",
        tool_id="turn_angle",
        correlation_id="move",
        arguments_digest=_arguments_digest(args),
        principal="operator:scott",
    )
    with pytest.raises(MissionValidationError, match="approval tool binding mismatch"):
        DeterministicMissionRuntime(registry, PhysicalCapabilityAdapters(), now_s=100.0).execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="physical-envelope",
                    objective="physical approvals bind full execution envelope",
                    success_criteria=(SuccessCriterion("move-complete", "move completed", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),),
                    execution_mode="physical",
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0, max_travel_m=0.5),
                ),
                invocations=(ToolInvocation("move", "move_distance", "1.0", args, approval=physical_approval),),
            )
        )


def test_latest_review_resource_and_approval_blockers_are_regressed() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
    args = {"distance_m": 0.1, "speed_mps": 0.05, "timeout_s": 3.0}

    loose = ApprovalGrant(
        approval_id="loose",
        approved_by="operator:scott",
        approved_at_s=100.0,
        expires_at_s=200.0,
        approval_class="supervised_motion",
        mission_id="replay-loose",
        issued_to="mission-runtime",
    )
    with pytest.raises(MissionValidationError, match="approval tool binding mismatch"):
        DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0).execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="replay-loose",
                    objective="replay approvals bind the full execution envelope",
                    success_criteria=(SuccessCriterion("done", "move done", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),),
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0, max_travel_m=0.5),
                ),
                invocations=(ToolInvocation("any-corr", "move_distance", "1.0", args, approval=loose),),
            )
        )

    with pytest.raises(MissionValidationError, match="exclusive resource conflict"):
        DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0).execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="resource-conflict",
                    objective="exclusive resources cannot be double-booked inside one plan",
                    success_criteria=(SuccessCriterion("done", "telemetry done", CriterionKind.TOOL_COMPLETE, tool_id="query_status_telemetry"),),
                    budgets=MissionBudgets(max_steps=2, max_runtime_s=10.0),
                ),
                invocations=(
                    ToolInvocation("q1", "query_status_telemetry", "1.0", {}),
                    ToolInvocation("q2", "query_status_telemetry", "1.0", {}),
                ),
            )
        )

    empty_schema = {"type": "object", "properties": {}, "required": [], "additionalProperties": False}
    tool_id_result_schema = {
        "type": "object",
        "properties": {"tool_id": {"type": "string"}},
        "required": ["tool_id"],
        "additionalProperties": False,
    }
    custom_registry = CapabilityRegistry(
        (
            ToolDefinition("custom_a", "1.0", empty_schema, tool_id_result_schema, resource_ownership=("shared_runtime",)),
            ToolDefinition("custom_b", "1.0", empty_schema, tool_id_result_schema, resource_ownership=("shared_runtime",)),
        )
    )
    with pytest.raises(MissionValidationError, match="exclusive resource conflict"):
        DeterministicMissionRuntime(custom_registry, FakeCapabilityAdapters(), now_s=100.0).execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="cross-tool-resource-conflict",
                    objective="exclusive resources are enforced across tool definitions",
                    success_criteria=(
                        SuccessCriterion("a", "custom_a completed", CriterionKind.TOOL_COMPLETE, tool_id="custom_a"),
                        SuccessCriterion("b", "custom_b completed", CriterionKind.TOOL_COMPLETE, tool_id="custom_b"),
                    ),
                    budgets=MissionBudgets(max_steps=2, max_runtime_s=10.0),
                ),
                invocations=(ToolInvocation("a", "custom_a", "1.0", {}), ToolInvocation("b", "custom_b", "1.0", {})),
            )
        )


def test_replay_adapter_must_attest_declared_preconditions() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(satisfied_preconditions=None),
        now_s=100.0,
    )

    with pytest.raises(MissionValidationError, match="preconditions are not attested"):
        runtime.execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="replay-preconditions",
                    objective="replay adapters cannot silently bypass preconditions",
                    success_criteria=(SuccessCriterion("detect", "detect completed", CriterionKind.TOOL_COMPLETE, tool_id="detect_objects"),),
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
                ),
                invocations=(ToolInvocation("detect", "detect_objects", "1.0", {"object_class": "shoe"}),),
            )
        )


def test_rebased_review_terminal_text_artifact_and_unsigned_approval_blockers_are_regressed() -> None:
    registry = build_default_registry(detector_classes=("shoe",))

    unsigned_args = {"distance_m": 0.1, "speed_mps": 0.05, "timeout_s": 3.0}
    unsigned = ApprovalGrant(
        approval_id="caller-minted",
        approved_by="operator:scott",
        approved_at_s=100.0,
        expires_at_s=200.0,
        approval_class="supervised_motion",
        mission_id="unsigned-approval",
        issued_to="mission-runtime",
        tool_id="move_distance",
        correlation_id="move",
        arguments_digest=_arguments_digest(unsigned_args),
        principal="operator:scott",
    )
    with pytest.raises(MissionValidationError, match="approval signature"):
        DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0).execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="unsigned-approval",
                    objective="caller-created approvals cannot mint authority",
                    success_criteria=(SuccessCriterion("done", "move done", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),),
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0, max_travel_m=0.5),
                ),
                invocations=(ToolInvocation("move", "move_distance", "1.0", unsigned_args, approval=unsigned),),
            )
        )

    text_success = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="running-state-text",
                objective="legacy text criteria are not production completion evidence",
                success_criteria=("running state",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(ToolInvocation("status", "query_status_telemetry", "1.0", {}),),
        )
    )
    assert text_success.status is MissionRuntimeStatus.FAILED
    assert "typed success criteria" in text_success.results[-1].error["message"]

    runtime = DeterministicMissionRuntime(registry, FakeCapabilityAdapters(), now_s=100.0)
    stopped = runtime.execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="latched-stop",
                objective="STOP latches across execute_plan calls",
                success_criteria=(SuccessCriterion("stop", "stop completed", CriterionKind.TOOL_COMPLETE, tool_id="pause_cancel_stop_estop"),),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(ToolInvocation("stop", "pause_cancel_stop_estop", "1.0", {"action": "stop"}),),
        )
    )
    assert stopped.status is MissionRuntimeStatus.STOPPED
    with pytest.raises(MissionValidationError, match="terminal runtime state STOPPED"):
        runtime.execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="after-stop",
                    objective="no execution after stop",
                    success_criteria=(SuccessCriterion("status", "status completed", CriterionKind.TOOL_COMPLETE, tool_id="query_status_telemetry"),),
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
                ),
                invocations=(ToolInvocation("status", "query_status_telemetry", "1.0", {}),),
            )
        )

    weak_artifact = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(
            observation_by_tool={"generate_semantic_artifacts": {"artifact_refs": {"mission_summary": "artifacts/vs06_semantic_map/mission_summary.md"}}},
            artifact_refs_by_tool={"generate_semantic_artifacts": {"mission_summary": "artifacts/vs06_semantic_map/mission_summary.md"}},
            provenance_by_tool={"generate_semantic_artifacts": {"adapter": "fake/replay", "deterministic": True}},
        ),
        now_s=100.0,
    ).execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="weak-artifact-provenance",
                objective="artifact refs require hash and invocation provenance",
                success_criteria=(SuccessCriterion("mission-summary", "mission summary artifact exists", CriterionKind.ARTIFACT_PRESENT, field="mission_summary"),),
                requested_artifacts=("mission_summary",),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=10.0),
            ),
            invocations=(ToolInvocation("artifact", "generate_semantic_artifacts", "1.0", {"artifact_kinds": ["mission_summary"]}),),
        )
    )
    assert weak_artifact.status is MissionRuntimeStatus.FAILED
    assert "artifact_refs failed provenance validation" in weak_artifact.results[-1].error["message"]


def test_cumulative_runtime_ledger_uses_executed_time_not_repeated_plan_ceiling() -> None:
    empty_schema = {"type": "object", "properties": {}, "required": [], "additionalProperties": False}
    tool_id_result_schema = {
        "type": "object",
        "properties": {"tool_id": {"type": "string"}},
        "required": ["tool_id"],
        "additionalProperties": False,
    }
    registry = CapabilityRegistry((ToolDefinition("short_read", "1.0", empty_schema, tool_id_result_schema, timeout_s=0.25),))
    runtime = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(duration_by_tool={"short_read": 0.25}),
        now_s=100.0,
        budget_ceilings=MissionBudgets(max_steps=4, max_runtime_s=1.0),
    )

    def plan(correlation_id: str) -> MissionPlan:
        return MissionPlan(
            goal=MissionGoal(
                goal_id=f"runtime-ledger-{correlation_id}",
                objective="session runtime ledger decrements actual bounded execution time",
                success_criteria=(SuccessCriterion("done", "short_read completed", CriterionKind.TOOL_COMPLETE, tool_id="short_read"),),
                budgets=MissionBudgets(max_steps=1, max_runtime_s=1.0),
            ),
            invocations=(ToolInvocation(correlation_id, "short_read", "1.0", {}),),
        )

    assert runtime.execute_plan(plan("one")).status is MissionRuntimeStatus.COMPLETE
    assert runtime.execute_plan(plan("two")).status is MissionRuntimeStatus.COMPLETE


def test_cumulative_runtime_clock_expires_later_plan_approval() -> None:
    registry = build_default_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(duration_by_tool={"query_status_telemetry": 2.0}),
        now_s=100.0,
        budget_ceilings=MissionBudgets(max_steps=2, max_runtime_s=5.0, max_travel_m=1.0),
    )
    status_plan = MissionPlan(
        goal=MissionGoal(
            goal_id="advance-clock",
            objective="advance the mission session clock",
            success_criteria=(SuccessCriterion("status", "status completed", CriterionKind.TOOL_COMPLETE, tool_id="query_status_telemetry"),),
            budgets=MissionBudgets(max_steps=1, max_runtime_s=2.0),
        ),
        invocations=(ToolInvocation("status", "query_status_telemetry", "1.0", {}),),
    )
    assert runtime.execute_plan(status_plan).status is MissionRuntimeStatus.COMPLETE

    move_args = {"distance_m": 0.1, "speed_mps": 0.05, "timeout_s": 1.0}
    move_plan = MissionPlan(
        goal=MissionGoal(
            goal_id="approval-after-clock-advance",
            objective="reject approval expired on the cumulative mission clock",
            success_criteria=(SuccessCriterion("move", "move completed", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),),
            budgets=MissionBudgets(max_steps=1, max_runtime_s=1.0, max_travel_m=0.1),
        ),
        invocations=(
            ToolInvocation(
                "move",
                "move_distance",
                "1.0",
                move_args,
                approval=_grant(
                    now_s=100.0,
                    expires_at_s=101.5,
                    mission_id="approval-after-clock-advance",
                    tool_id="move_distance",
                    correlation_id="move",
                    arguments=move_args,
                ),
            ),
        ),
    )
    with pytest.raises(MissionValidationError, match="approval is stale or missing"):
        runtime.execute_plan(move_plan)


def test_trusted_monotonic_clock_expires_approval_when_adapter_underreports_time() -> None:
    clock = [0.0]

    class ClockAdvancingHandle:
        def __init__(self, result, advance_s):
            self.result = result
            self.advance_s = advance_s

        def wait(self, timeout_s):
            del timeout_s
            clock[0] += self.advance_s
            return self.result

        def cancel(self):
            return None

        def cleanup(self, timeout_s):
            del timeout_s
            return True

        def wait_idle(self, timeout_s):
            del timeout_s
            return True

    class UnderreportingAdapters(FakeCapabilityAdapters):
        def begin_execution(self, invocation, definition, *, started_at_s, index):
            result = self.execute(invocation, definition, started_at_s=started_at_s, index=index)
            return ClockAdvancingHandle(result, 2.0 if invocation.tool_id == "query_status_telemetry" else 0.0)

    registry = build_default_registry(detector_classes=("shoe",))
    runtime = DeterministicMissionRuntime(
        registry,
        UnderreportingAdapters(duration_by_tool={"query_status_telemetry": 0.0}),
        now_s=100.0,
        clock_s=lambda: clock[0],
    )
    move_args = {"distance_m": 0.1, "speed_mps": 0.05, "timeout_s": 1.0}
    approval = _grant(
        now_s=100.0,
        expires_at_s=101.5,
        mission_id="trusted-monotonic-clock",
        tool_id="move_distance",
        correlation_id="move",
        arguments=move_args,
    )

    with pytest.raises(MissionValidationError, match="approval is stale or missing"):
        runtime.execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="trusted-monotonic-clock",
                    objective="adapter timestamps cannot hold approval time still",
                    success_criteria=(SuccessCriterion("move", "move completed", CriterionKind.TOOL_COMPLETE, tool_id="move_distance"),),
                    budgets=MissionBudgets(max_steps=2, max_runtime_s=5.0, max_travel_m=0.1),
                ),
                invocations=(
                    ToolInvocation("status", "query_status_telemetry", "1.0", {}),
                    ToolInvocation("move", "move_distance", "1.0", move_args, approval=approval),
                ),
            )
        )


def test_timeout_ledger_preserves_prior_elapsed_runtime() -> None:
    empty_schema = {"type": "object", "properties": {}, "required": [], "additionalProperties": False}
    tool_id_result_schema = {
        "type": "object",
        "properties": {"tool_id": {"type": "string"}},
        "required": ["tool_id"],
        "additionalProperties": False,
    }
    registry = CapabilityRegistry(
        (
            ToolDefinition("first", "1.0", empty_schema, tool_id_result_schema, timeout_s=1.0),
            ToolDefinition("times_out", "1.0", empty_schema, tool_id_result_schema, timeout_s=1.0),
            ToolDefinition("later", "1.0", empty_schema, tool_id_result_schema, timeout_s=1.0),
        )
    )
    runtime = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(duration_by_tool={"first": 0.25, "times_out": 2.0, "later": 1.0}),
        budget_ceilings=MissionBudgets(max_steps=3, max_runtime_s=2.0),
    )
    timed_out = runtime.execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="timeout-ledger",
                objective="preserve elapsed time before a timeout",
                success_criteria=(SuccessCriterion("later", "later completed", CriterionKind.TOOL_COMPLETE, tool_id="later"),),
                budgets=MissionBudgets(max_steps=2, max_runtime_s=2.0),
            ),
            invocations=(ToolInvocation("first", "first", "1.0", {}), ToolInvocation("timeout", "times_out", "1.0", {})),
        )
    )
    assert timed_out.status is MissionRuntimeStatus.TIMEOUT

    later_plan = MissionPlan(
        goal=MissionGoal(
            goal_id="after-timeout",
            objective="cumulative runtime cannot be reclaimed after timeout",
            success_criteria=(SuccessCriterion("later", "later completed", CriterionKind.TOOL_COMPLETE, tool_id="later"),),
            budgets=MissionBudgets(max_steps=1, max_runtime_s=1.0),
        ),
        invocations=(ToolInvocation("later", "later", "1.0", {}),),
    )
    with pytest.raises(MissionValidationError, match="cumulative max_runtime_s"):
        runtime.execute_plan(later_plan)


def test_prestart_timeout_records_prior_execution_in_cumulative_ledger() -> None:
    empty_schema = {"type": "object", "properties": {}, "required": [], "additionalProperties": False}
    tool_id_result_schema = {
        "type": "object",
        "properties": {"tool_id": {"type": "string"}},
        "required": ["tool_id"],
        "additionalProperties": False,
    }
    registry = CapabilityRegistry(
        (
            ToolDefinition("first", "1.0", empty_schema, tool_id_result_schema, timeout_s=1.0),
            ToolDefinition("cannot_start", "1.0", empty_schema, tool_id_result_schema, timeout_s=1.0),
            ToolDefinition("later", "1.0", empty_schema, tool_id_result_schema, timeout_s=1.1),
        )
    )
    runtime = DeterministicMissionRuntime(
        registry,
        FakeCapabilityAdapters(duration_by_tool={"first": 1.0}),
        budget_ceilings=MissionBudgets(max_steps=3, max_runtime_s=2.0),
    )

    timed_out = runtime.execute_plan(
        MissionPlan(
            goal=MissionGoal(
                goal_id="prestart-timeout-ledger",
                objective="do not discard work completed before a tool cannot start",
                success_criteria=(SuccessCriterion("first", "first completed", CriterionKind.TOOL_COMPLETE, tool_id="first"),),
                budgets=MissionBudgets(max_steps=2, max_runtime_s=1.5),
            ),
            invocations=(ToolInvocation("first", "first", "1.0", {}), ToolInvocation("blocked", "cannot_start", "1.0", {})),
        )
    )
    assert timed_out.status is MissionRuntimeStatus.TIMEOUT
    assert [item.invocation.correlation_id for item in timed_out.results] == ["first", "blocked"]

    with pytest.raises(MissionValidationError, match="cumulative max_runtime_s"):
        runtime.execute_plan(
            MissionPlan(
                goal=MissionGoal(
                    goal_id="after-prestart-timeout",
                    objective="prior elapsed time remains charged",
                    success_criteria=(SuccessCriterion("later", "later completed", CriterionKind.TOOL_COMPLETE, tool_id="later"),),
                    budgets=MissionBudgets(max_steps=1, max_runtime_s=1.1),
                ),
                invocations=(ToolInvocation("later", "later", "1.0", {}),),
            )
        )


def test_architecture_has_one_canonical_mission_api_module_and_registry() -> None:
    from pathlib import Path

    import sphero_rvr_driver.mission_api as mission_api

    repo_root = Path(__file__).resolve().parents[1]
    removed_module = "mission_api" + "_v2"
    legacy_symbols = (
        "build_canonical_shoe_mapping_request",
        "validate_mission_request",
        "MissionStateMachine",
        "CapabilitySet",
    )

    assert not (repo_root / "src" / "sphero_rvr_driver" / f"{removed_module}.py").exists()
    assert hasattr(mission_api, "build_default_registry")
    assert not hasattr(mission_api, "build_default_v2_registry")
    assert not hasattr(mission_api.MissionApiVersion, "V1")
    assert mission_api.MissionApiVersion.V2.value == "mission_api.v2"
    for symbol in legacy_symbols:
        assert not hasattr(mission_api, symbol)

    default_registry_defs = 0
    for path in repo_root.rglob("*"):
        if (
            path.is_dir()
            or ".git" in path.parts
            or ".worktrees" in path.parts
            or ".pytest_cache" in path.parts
            or "build" in path.parts
            or "dist" in path.parts
            or path.suffix in {".pyc", ".png", ".jpg", ".jpeg", ".gif"}
        ):
            continue
        try:
            content = path.read_text()
        except UnicodeDecodeError:
            continue
        rel = path.relative_to(repo_root).as_posix()
        if rel == "tests/test_mission_api.py":
            content = content.replace('removed_module = "mission_api" + "_v2"', "")
            content = content.replace('"build_canonical_shoe_mapping_request",', "")
            content = content.replace('"validate_mission_request",', "")
            content = content.replace('"MissionStateMachine",', "")
            content = content.replace('"CapabilitySet",', "")
        assert removed_module not in content, rel
        for symbol in legacy_symbols:
            assert symbol not in content, f"{symbol} remains in {rel}"
        default_registry_defs += content.count("def " + "build_default_registry")
    assert default_registry_defs == 1
