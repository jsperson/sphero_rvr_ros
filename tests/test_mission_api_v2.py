from __future__ import annotations

import pytest

from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_api_v2 import (
    ApprovalState,
    CapabilityState,
    MissionBudgets,
    RemainingBudgets,
    ScriptedToolAdapter,
    ToolCall,
    ToolDefinition,
    ToolResult,
    ToolResultStatus,
    build_default_rover_tool_registry,
)
from sphero_rvr_driver.mission_controls import MissionExecutionMode


def test_mission_budgets_reject_strings_nonfinite_values_and_expansion() -> None:
    with pytest.raises(MissionValidationError, match="max_runtime_s"):
        MissionBudgets(max_runtime_s="30")  # type: ignore[arg-type]
    with pytest.raises(MissionValidationError, match="max_runtime_s"):
        MissionBudgets(max_runtime_s=float("inf"))
    with pytest.raises(MissionValidationError, match="max_tool_calls"):
        MissionBudgets(max_tool_calls=10_000)
    with pytest.raises(MissionValidationError, match="max_travel_m"):
        MissionBudgets(max_travel_m=99.0)


def test_registry_validates_allowlisted_tool_calls_capabilities_schema_and_budgets() -> None:
    registry = build_default_rover_tool_registry()
    remaining = RemainingBudgets(iterations=1, runtime_s=30.0, tool_calls=1, travel_m=1.0, segments=1)

    definition = registry.validate_tool_call(
        ToolCall("approach_clearance", {"target_clearance_m": 0.25}, call_id="ok"),
        capabilities=CapabilityState.all_enabled(),
        approval_state=ApprovalState.REPLAY_ONLY,
        execution_mode=MissionExecutionMode.REPLAY,
        remaining=remaining,
    )
    assert definition.name == "approach_clearance"

    with pytest.raises(MissionValidationError, match="unknown rover tool"):
        registry.validate_tool_call(
            ToolCall("dance", {"style": "unsafe"}, call_id="unknown"),
            capabilities=CapabilityState.all_enabled(),
            approval_state=ApprovalState.REPLAY_ONLY,
            execution_mode=MissionExecutionMode.REPLAY,
            remaining=remaining,
        )
    with pytest.raises(MissionValidationError, match="target_clearance_m"):
        registry.validate_tool_call(
            ToolCall("approach_clearance", {"target_clearance_m": "0.25"}, call_id="bad-schema"),
            capabilities=CapabilityState.all_enabled(),
            approval_state=ApprovalState.REPLAY_ONLY,
            execution_mode=MissionExecutionMode.REPLAY,
            remaining=remaining,
        )
    with pytest.raises(MissionValidationError, match="supervised_motion"):
        registry.validate_tool_call(
            ToolCall("approach_clearance", {"target_clearance_m": 0.25}, call_id="missing-capability"),
            capabilities=CapabilityState.all_enabled(supervised_motion=False),
            approval_state=ApprovalState.REPLAY_ONLY,
            execution_mode=MissionExecutionMode.REPLAY,
            remaining=remaining,
        )
    with pytest.raises(MissionValidationError, match="physical approval"):
        registry.validate_tool_call(
            ToolCall("approach_clearance", {"target_clearance_m": 0.25}, call_id="physical-with-replay-auth"),
            capabilities=CapabilityState.all_enabled(),
            approval_state=ApprovalState.REPLAY_ONLY,
            execution_mode=MissionExecutionMode.PHYSICAL,
            remaining=remaining,
        )
    with pytest.raises(MissionValidationError, match="travel budget exhausted"):
        registry.validate_tool_call(
            ToolCall("approach_clearance", {"target_clearance_m": 0.25}, call_id="travel-exhausted"),
            capabilities=CapabilityState.all_enabled(),
            approval_state=ApprovalState.REPLAY_ONLY,
            execution_mode=MissionExecutionMode.REPLAY,
            remaining=RemainingBudgets(iterations=1, runtime_s=30.0, tool_calls=1, travel_m=0.0, segments=1),
        )


def test_execute_tool_enforces_timeout_result_schema_and_ros_surface_boundary_for_all_statuses() -> None:
    definition = ToolDefinition(
        name="safe_capture",
        description="Safe replay capture.",
        input_schema={"type": "object", "properties": {"label": {"type": "string"}}, "required": ["label"], "additionalProperties": False},
        result_schema={"type": "object", "properties": {"label": {"type": "string"}}, "required": ["label"], "additionalProperties": False},
        timeout_s=0.5,
    )

    timeout_registry = build_default_rover_tool_registry(registry_version="timeout-test")
    timeout_registry.register(definition, ScriptedToolAdapter([ToolResult(ToolResultStatus.OK, observation={"label": "ok"}, duration_s=2.0)]))
    with pytest.raises(MissionValidationError, match="exceeded timeout"):
        timeout_registry.execute_tool(ToolCall("safe_capture", {"label": "ok"}), timeout_registry.definition("safe_capture"))

    extra_registry = build_default_rover_tool_registry(registry_version="schema-test")
    extra_registry.register(
        definition,
        ScriptedToolAdapter([ToolResult(ToolResultStatus.FAILED, observation={"label": "bad", "extra": "not allowed"})]),
    )
    with pytest.raises(MissionValidationError, match="unexpected fields"):
        extra_registry.execute_tool(ToolCall("safe_capture", {"label": "bad"}), extra_registry.definition("safe_capture"))

    with pytest.raises(MissionValidationError, match="direct ROS/motor/system surfaces"):
        ToolResult(ToolResultStatus.FAILED, observation={"label": "bad", "command_path": ["/cmd_vel"]})


def test_tool_definitions_reject_direct_surfaces_and_unknown_result_shapes() -> None:
    with pytest.raises(MissionValidationError, match="forbidden direct surface"):
        ToolDefinition(
            name="raw_motor",
            description="unsafe",
            input_schema={"type": "object", "properties": {}, "additionalProperties": False},
            result_schema={"type": "object", "properties": {}, "additionalProperties": False},
            timeout_s=1.0,
        )
    with pytest.raises(MissionValidationError, match="unsupported type"):
        ToolDefinition(
            name="bad_schema",
            description="bad schema",
            input_schema={"type": "object", "properties": {"x": {"type": "null"}}, "additionalProperties": False},
            result_schema={"type": "object", "properties": {}, "additionalProperties": False},
            timeout_s=1.0,
        )
