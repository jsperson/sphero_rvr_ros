"""Reviewed physical-control adapters for mission_api.v2 rover tools.

This module is still ROS-free for testability.  It maps the motor-capable v2
allowlist onto the existing deterministic rover controllers instead of returning
fake success.  The ROS node or hardware seam owns live sensor collection and
command publication; this adapter consumes bounded observations and returns only
schema-declared Mission API results.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from .mission_api import MissionValidationError
from .mission_api_v2 import ToolDefinition, ToolInvocation, ToolResult, ToolResultStatus
from .range_motion import (
    MotionDirection,
    MotionGoal,
    MotionMode,
    RangeMotionConfig,
    RangeMotionController,
    RangeMotionSample,
    RangeMotionTelemetry,
    StopReason,
)
from .supervised_coordinator import (
    CoordinatorConfig,
    CoordinatorPhase,
    DeterministicSegmentSelector,
    SegmentStatus,
    SupervisedCoordinator,
)

_PHYSICAL_PROVENANCE = {"adapter": "physical/supervised_control", "deterministic": True}


@dataclass(frozen=True)
class PhysicalCapabilityAdapters:
    """Execute reviewed physical-control v2 tools through deterministic controllers.

    Sensor samples and coarse exploration clearances are injected by the caller so
    tests and replay preflights can exercise the same adapter without a live ROS
    graph.  Missing physical observations block rather than silently falling back
    to replay/fake results.
    """

    move_samples_by_correlation_id: Mapping[str, Sequence[RangeMotionSample]] = field(default_factory=dict)
    move_sample_wall_times_by_correlation_id: Mapping[str, Sequence[float]] = field(default_factory=dict)
    exploration_clearances_m: Sequence[float] = field(default_factory=tuple)
    exploration_segment_statuses_by_correlation_id: Mapping[str, Sequence[SegmentStatus]] = field(default_factory=dict)
    stop_before: str = ""
    estop_before: str = ""

    def execute(self, invocation: ToolInvocation, definition: ToolDefinition, *, started_at_s: float, index: int) -> ToolResult:
        del index
        completed_at_s = started_at_s + min(0.25, float(definition.timeout_s))
        if self.stop_before == invocation.tool_id:
            return _terminal_result(
                invocation,
                ToolResultStatus.STOPPED,
                started_at_s,
                completed_at_s,
                "STOP propagated before physical tool start",
            )
        if self.estop_before == invocation.tool_id:
            return _terminal_result(
                invocation,
                ToolResultStatus.ESTOPPED,
                started_at_s,
                completed_at_s,
                "ESTOP propagated before physical tool start",
            )
        if invocation.tool_id == "move_to_clearance":
            return self._move_to_clearance(invocation, started_at_s=started_at_s)
        if invocation.tool_id == "bounded_exploration_segment":
            return self._bounded_exploration_segment(invocation, started_at_s=started_at_s)
        if invocation.tool_id == "pause_cancel_stop_estop":
            return self._pause_cancel_stop_estop(invocation, started_at_s=started_at_s, completed_at_s=completed_at_s)
        if invocation.tool_id == "query_status_telemetry":
            return ToolResult(
                invocation=invocation,
                status=ToolResultStatus.COMPLETE,
                started_at_s=started_at_s,
                completed_at_s=completed_at_s,
                observation={"state": "READY"},
                provenance=dict(_PHYSICAL_PROVENANCE),
            )
        raise MissionValidationError(f"{invocation.tool_id} requires a dedicated physical adapter")

    def _move_to_clearance(self, invocation: ToolInvocation, *, started_at_s: float) -> ToolResult:
        samples = tuple(self.move_samples_by_correlation_id.get(invocation.correlation_id, ()))
        if not samples:
            return _terminal_result(
                invocation,
                ToolResultStatus.BLOCKED,
                started_at_s,
                started_at_s,
                "physical move_to_clearance requires range-motion samples",
            )
        sample_wall_times = tuple(self.move_sample_wall_times_by_correlation_id.get(invocation.correlation_id, ()))
        if len(sample_wall_times) != len(samples):
            return _terminal_result(
                invocation,
                ToolResultStatus.BLOCKED,
                started_at_s,
                started_at_s,
                "physical move_to_clearance requires wall-time receipt for each range-motion sample",
            )
        goal = MotionGoal(
            direction=MotionDirection.FORWARD,
            mode=MotionMode.APPROACH,
            target_clearance_m=float(invocation.arguments["clearance_m"]),
            max_measured_displacement_m=float(invocation.arguments["max_travel_m"]),
            timeout_s=float(invocation.arguments["timeout_s"]),
        )
        controller = RangeMotionController(
            RangeMotionConfig(
                max_speed_mps=min(0.20, float(invocation.arguments["speed_mps"])),
            )
        )
        first_sample = samples[0]
        first_sample_wall_time_s = float(sample_wall_times[0])
        first_sample_age_s = first_sample_wall_time_s - float(first_sample.stamp)
        if first_sample_age_s > controller.config.max_sample_age_s:
            return _terminal_result(
                invocation,
                ToolResultStatus.BLOCKED,
                started_at_s,
                max(started_at_s, first_sample_wall_time_s),
                "range motion stopped: stale_sensor",
            )
        telemetry = controller.start(goal, samples[0])
        for sample, wall_time_s in zip(samples[1:], sample_wall_times[1:]):
            telemetry = controller.update(sample, now=float(wall_time_s))
            if telemetry.stop_reason is not StopReason.RUNNING:
                break
        completed_at_s = started_at_s + max(0.0, float(samples[-1].stamp) - float(samples[0].stamp))
        if telemetry.stop_reason is StopReason.TARGET_REACHED:
            return ToolResult(
                invocation=invocation,
                status=ToolResultStatus.COMPLETE,
                started_at_s=started_at_s,
                completed_at_s=completed_at_s,
                observation={"target_clearance_m": goal.target_clearance_m},
                provenance=dict(_PHYSICAL_PROVENANCE),
            )
        return _terminal_result(
            invocation,
            _status_for_range_motion_stop(telemetry.stop_reason),
            started_at_s,
            completed_at_s,
            f"range motion stopped: {telemetry.stop_reason.value}",
        )

    def _bounded_exploration_segment(self, invocation: ToolInvocation, *, started_at_s: float) -> ToolResult:
        if not self.exploration_clearances_m:
            return _terminal_result(
                invocation,
                ToolResultStatus.BLOCKED,
                started_at_s,
                started_at_s,
                "physical bounded_exploration_segment requires clearance observations",
            )
        segment_statuses = tuple(self.exploration_segment_statuses_by_correlation_id.get(invocation.correlation_id, ()))
        if not segment_statuses:
            return _terminal_result(
                invocation,
                ToolResultStatus.BLOCKED,
                started_at_s,
                started_at_s,
                "physical bounded_exploration_segment requires measured segment statuses",
            )
        config = CoordinatorConfig(
            mission_id=invocation.correlation_id,
            max_segments=int(invocation.arguments["max_segments"]),
            segment_target_clearance_m=0.25,
            max_segment_displacement_m=float(invocation.arguments["max_travel_m"]) / int(invocation.arguments["max_segments"]),
            segment_timeout_s=float(invocation.arguments["segment_timeout_s"]),
        )
        coordinator = SupervisedCoordinator(
            config=config,
            selector=DeterministicSegmentSelector(clearances_m=tuple(self.exploration_clearances_m), config=config),
        )
        telemetry = coordinator.start(now=started_at_s)
        status_index = 0
        while telemetry.phase is CoordinatorPhase.RUNNING_SEGMENT:
            command = coordinator.next_range_motion_goal()
            if command is None:
                return _terminal_result(
                    invocation,
                    ToolResultStatus.FAILED,
                    started_at_s,
                    started_at_s,
                    "supervised coordinator produced no range-motion command",
                )
            if status_index >= len(segment_statuses):
                return _terminal_result(
                    invocation,
                    ToolResultStatus.BLOCKED,
                    started_at_s,
                    started_at_s + float(telemetry.completed_segments),
                    "physical bounded_exploration_segment missing measured status for active segment",
                )
            segment_status = segment_statuses[status_index]
            status_index += 1
            if segment_status.measured_displacement_m > command.segment.max_displacement_m:
                segment_status = SegmentStatus(
                    stop_reason=StopReason.MAX_DISPLACEMENT,
                    measured_displacement_m=segment_status.measured_displacement_m,
                    current_clearance_m=segment_status.current_clearance_m,
                )
            telemetry = coordinator.handle_segment_status(
                segment_status,
                now=started_at_s + status_index,
            )
        completed_at_s = started_at_s + float(telemetry.completed_segments)
        if telemetry.phase is CoordinatorPhase.COMPLETE:
            return ToolResult(
                invocation=invocation,
                status=ToolResultStatus.COMPLETE,
                started_at_s=started_at_s,
                completed_at_s=completed_at_s,
                observation={"completed_segments": telemetry.completed_segments},
                provenance=dict(_PHYSICAL_PROVENANCE),
            )
        return _terminal_result(
            invocation,
            ToolResultStatus.FAILED,
            started_at_s,
            completed_at_s,
            _coordinator_stop_message(telemetry.stop_reason.value, telemetry.range_motion_stop_reason),
        )

    @staticmethod
    def _pause_cancel_stop_estop(invocation: ToolInvocation, *, started_at_s: float, completed_at_s: float) -> ToolResult:
        action = str(invocation.arguments["action"])
        status_by_action = {
            "cancel": ToolResultStatus.CANCELLED,
            "stop": ToolResultStatus.STOPPED,
            "estop": ToolResultStatus.ESTOPPED,
            "pause": ToolResultStatus.BLOCKED,
        }
        return ToolResult(
            invocation=invocation,
            status=status_by_action[action],
            started_at_s=started_at_s,
            completed_at_s=completed_at_s,
            observation={"latched_state": action.upper()},
            provenance=dict(_PHYSICAL_PROVENANCE),
        )


def _status_for_range_motion_stop(reason: StopReason) -> ToolResultStatus:
    if reason is StopReason.TIMEOUT:
        return ToolResultStatus.TIMEOUT
    if reason is StopReason.OPERATOR_STOP:
        return ToolResultStatus.STOPPED
    if reason is StopReason.ESTOP:
        return ToolResultStatus.ESTOPPED
    if reason in {StopReason.UNSAFE_CLEARANCE, StopReason.TARGET_LOST, StopReason.STALE_SENSOR}:
        return ToolResultStatus.BLOCKED
    return ToolResultStatus.FAILED


def _coordinator_stop_message(stop_reason: str, range_motion_stop_reason: StopReason | None) -> str:
    if range_motion_stop_reason is None:
        return f"supervised coordinator stopped: {stop_reason}"
    return f"supervised coordinator stopped: {stop_reason}/{range_motion_stop_reason.value}"


def _terminal_result(
    invocation: ToolInvocation,
    status: ToolResultStatus,
    started_at_s: float,
    completed_at_s: float,
    message: str,
) -> ToolResult:
    return ToolResult(
        invocation=invocation,
        status=status,
        started_at_s=started_at_s,
        completed_at_s=completed_at_s,
        error={"message": message},
        provenance=dict(_PHYSICAL_PROVENANCE),
    )
