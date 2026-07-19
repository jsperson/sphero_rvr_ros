"""ROS-free supervised mapping/navigation coordinator core.

This module owns mission-level sequencing only.  It deliberately does not know how
to publish motor commands; the only motion command it can emit is a range-motion
goal that must travel through::

    range_motion -> /cmd_vel -> collision_stop -> /cmd_vel_motor

STOP, ESTOP, collision-stop, and shutdown observations are independent inputs and
latch the coordinator failed-closed with deterministic cancellation required.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional, Sequence

from .range_motion import MotionDirection, MotionGoal, MotionMode, StopReason


class CoordinatorPhase(str, Enum):
    IDLE = "IDLE"
    RUNNING_SEGMENT = "RUNNING_SEGMENT"
    COMPLETE = "COMPLETE"
    FAILED_CLOSED = "FAILED_CLOSED"


class CoordinatorStopReason(str, Enum):
    NONE = "none"
    COMPLETE = "complete"
    NO_SEGMENT_AVAILABLE = "no_segment_available"
    RANGE_MOTION_FAILED = "range_motion_failed"
    STOP = "stop"
    ESTOP = "estop"
    COLLISION_STOP = "collision_stop"
    SHUTDOWN = "shutdown"


class MissionEventKind(str, Enum):
    STOP = "stop"
    ESTOP = "estop"
    COLLISION_STOP = "collision_stop"
    SHUTDOWN = "shutdown"


@dataclass(frozen=True)
class CoordinatorConfig:
    mission_id: str = "supervised_mapping"
    max_segments: int = 8
    segment_target_clearance_m: float = 0.25
    max_segment_displacement_m: float = 0.50
    segment_timeout_s: float = 8.0
    command_topic: str = "/range_motion/goal"

    def __post_init__(self) -> None:
        if self.max_segments <= 0:
            raise ValueError("max_segments must be positive")
        if self.segment_target_clearance_m <= 0.0:
            raise ValueError("segment_target_clearance_m must be positive")
        if self.max_segment_displacement_m <= 0.0:
            raise ValueError("max_segment_displacement_m must be positive")
        if self.segment_timeout_s <= 0.0:
            raise ValueError("segment_timeout_s must be positive")


@dataclass(frozen=True)
class SegmentPlan:
    index: int
    direction: str
    observed_clearance_m: float
    target_clearance_m: float
    max_displacement_m: float
    timeout_s: float

    def to_motion_goal(self) -> MotionGoal:
        direction = MotionDirection(self.direction)
        mode = MotionMode.APPROACH if direction is MotionDirection.FORWARD else MotionMode.RETREAT
        return MotionGoal(
            direction=direction,
            mode=mode,
            target_clearance_m=self.target_clearance_m,
            max_measured_displacement_m=self.max_displacement_m,
            timeout_s=self.timeout_s,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "direction": self.direction,
            "observed_clearance_m": self.observed_clearance_m,
            "target_clearance_m": self.target_clearance_m,
            "max_displacement_m": self.max_displacement_m,
            "timeout_s": self.timeout_s,
        }


@dataclass(frozen=True)
class RangeMotionCommand:
    goal: MotionGoal
    segment: SegmentPlan
    topic: str = "/range_motion/goal"
    channel: str = "range_motion"
    motor_topic: Optional[str] = None


@dataclass(frozen=True)
class SegmentStatus:
    stop_reason: StopReason
    measured_displacement_m: float
    current_clearance_m: Optional[float]


@dataclass(frozen=True)
class MissionEvent:
    kind: MissionEventKind
    detail: str = ""

    def __post_init__(self) -> None:
        if isinstance(self.kind, str):
            object.__setattr__(self, "kind", MissionEventKind(self.kind))


@dataclass(frozen=True)
class CoordinatorTelemetry:
    mission_id: str
    phase: CoordinatorPhase
    stop_reason: CoordinatorStopReason
    completed_segments: int
    total_measured_displacement_m: float
    active_segment: Optional[SegmentPlan] = None
    range_motion_stop_reason: Optional[StopReason] = None
    observable_safety_state: str = "clear"
    cancellation_required: bool = False
    last_event_detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "mission_id": self.mission_id,
            "phase": self.phase.value,
            "stop_reason": self.stop_reason.value,
            "completed_segments": self.completed_segments,
            "total_measured_displacement_m": self.total_measured_displacement_m,
            "active_segment": None if self.active_segment is None else self.active_segment.to_dict(),
            "range_motion_stop_reason": None
            if self.range_motion_stop_reason is None
            else self.range_motion_stop_reason.value,
            "observable_safety_state": self.observable_safety_state,
            "cancellation_required": self.cancellation_required,
            "last_event_detail": self.last_event_detail,
            "command_path": ["range_motion", "/cmd_vel", "collision_stop", "/cmd_vel_motor"],
            "cmd_vel_motor_publish_allowed": False,
            "safety": {
                "fail_closed": True,
                "stop_estop_collision_authoritative": True,
                "shutdown_cancels_active_segment": True,
            },
        }


@dataclass
class DeterministicSegmentSelector:
    """Pick bounded linear segments from a fixed, observable clearance snapshot.

    The selector is deterministic by construction: it walks the configured order
    once and never reuses a direction in the same mission.  That gives Mission API
    and a read-only UI a stable segment list instead of random bump-and-turn
    wandering.
    """

    clearances_m: Sequence[float]
    config: CoordinatorConfig = field(default_factory=CoordinatorConfig)
    directions: Sequence[str] = ("forward", "backward")
    _used: set[str] = field(default_factory=set, init=False, repr=False)
    _next_index: int = field(default=0, init=False, repr=False)

    def next_segment(self) -> Optional[SegmentPlan]:
        for direction, clearance in self._candidate_pairs():
            if direction in self._used:
                continue
            if clearance <= self.config.segment_target_clearance_m:
                continue
            self._used.add(direction)
            segment = SegmentPlan(
                index=self._next_index,
                direction=direction,
                observed_clearance_m=float(clearance),
                target_clearance_m=self.config.segment_target_clearance_m,
                max_displacement_m=min(
                    self.config.max_segment_displacement_m,
                    max(0.0, float(clearance) - self.config.segment_target_clearance_m),
                ),
                timeout_s=self.config.segment_timeout_s,
            )
            self._next_index += 1
            return segment
        return None

    def _candidate_pairs(self) -> tuple[tuple[str, float], ...]:
        return tuple(
            (direction, float(clearance))
            for direction, clearance in zip(self.directions, self.clearances_m)
            if direction in {MotionDirection.FORWARD.value, MotionDirection.BACKWARD.value}
        )


class SupervisedCoordinator:
    def __init__(self, *, config: CoordinatorConfig, selector: DeterministicSegmentSelector):
        self.config = config
        self.selector = selector
        self._phase = CoordinatorPhase.IDLE
        self._stop_reason = CoordinatorStopReason.NONE
        self._active_segment: Optional[SegmentPlan] = None
        self._pending_command: Optional[RangeMotionCommand] = None
        self._completed_segments = 0
        self._total_measured_displacement_m = 0.0
        self._range_motion_stop_reason: Optional[StopReason] = None
        self._observable_safety_state = "clear"
        self._cancellation_required = False
        self._last_event_detail = ""

    def start(self, *, now: float) -> CoordinatorTelemetry:
        if self._phase is CoordinatorPhase.FAILED_CLOSED:
            return self._telemetry()
        self._phase = CoordinatorPhase.IDLE
        self._stop_reason = CoordinatorStopReason.NONE
        return self._start_next_segment(now=now)

    def next_range_motion_goal(self) -> Optional[RangeMotionCommand]:
        if self._phase is not CoordinatorPhase.RUNNING_SEGMENT:
            return None
        return self._pending_command

    def handle_segment_status(self, status: SegmentStatus, *, now: float) -> CoordinatorTelemetry:
        if self._phase is CoordinatorPhase.FAILED_CLOSED:
            return self._telemetry()
        if self._phase is not CoordinatorPhase.RUNNING_SEGMENT or self._active_segment is None:
            return self._fail_closed(
                CoordinatorStopReason.RANGE_MOTION_FAILED,
                observable_safety_state="range_motion_unexpected_status",
                range_motion_stop_reason=status.stop_reason,
            )
        self._range_motion_stop_reason = status.stop_reason
        self._pending_command = None
        if status.stop_reason is not StopReason.TARGET_REACHED:
            return self._fail_closed(
                CoordinatorStopReason.RANGE_MOTION_FAILED,
                observable_safety_state=status.stop_reason.value,
                range_motion_stop_reason=status.stop_reason,
            )
        self._completed_segments += 1
        self._total_measured_displacement_m += max(0.0, float(status.measured_displacement_m))
        self._active_segment = None
        if self._completed_segments >= self.config.max_segments:
            self._phase = CoordinatorPhase.COMPLETE
            self._stop_reason = CoordinatorStopReason.COMPLETE
            return self._telemetry()
        return self._start_next_segment(now=now)

    def handle_event(self, event: MissionEvent, *, now: float) -> CoordinatorTelemetry:
        reason_by_kind = {
            MissionEventKind.STOP: CoordinatorStopReason.STOP,
            MissionEventKind.ESTOP: CoordinatorStopReason.ESTOP,
            MissionEventKind.COLLISION_STOP: CoordinatorStopReason.COLLISION_STOP,
            MissionEventKind.SHUTDOWN: CoordinatorStopReason.SHUTDOWN,
        }
        return self._fail_closed(
            reason_by_kind[event.kind],
            observable_safety_state=event.kind.value,
            detail=event.detail,
        )

    def _start_next_segment(self, *, now: float) -> CoordinatorTelemetry:
        del now  # time is currently telemetry caller context; sequencing is deterministic.
        segment = self.selector.next_segment()
        if segment is None:
            self._phase = CoordinatorPhase.COMPLETE if self._completed_segments > 0 else CoordinatorPhase.FAILED_CLOSED
            self._stop_reason = (
                CoordinatorStopReason.COMPLETE
                if self._completed_segments > 0
                else CoordinatorStopReason.NO_SEGMENT_AVAILABLE
            )
            self._cancellation_required = self._phase is CoordinatorPhase.FAILED_CLOSED
            return self._telemetry()
        self._active_segment = segment
        self._pending_command = RangeMotionCommand(
            goal=segment.to_motion_goal(),
            segment=segment,
            topic=self.config.command_topic,
        )
        self._phase = CoordinatorPhase.RUNNING_SEGMENT
        self._stop_reason = CoordinatorStopReason.NONE
        self._cancellation_required = False
        self._observable_safety_state = "clear"
        return self._telemetry()

    def _fail_closed(
        self,
        stop_reason: CoordinatorStopReason,
        *,
        observable_safety_state: str,
        range_motion_stop_reason: Optional[StopReason] = None,
        detail: str = "",
    ) -> CoordinatorTelemetry:
        self._phase = CoordinatorPhase.FAILED_CLOSED
        self._stop_reason = stop_reason
        self._pending_command = None
        self._cancellation_required = True
        self._observable_safety_state = observable_safety_state
        self._range_motion_stop_reason = range_motion_stop_reason
        self._last_event_detail = detail
        return self._telemetry()

    def _telemetry(self) -> CoordinatorTelemetry:
        return CoordinatorTelemetry(
            mission_id=self.config.mission_id,
            phase=self._phase,
            stop_reason=self._stop_reason,
            completed_segments=self._completed_segments,
            total_measured_displacement_m=self._total_measured_displacement_m,
            active_segment=self._active_segment,
            range_motion_stop_reason=self._range_motion_stop_reason,
            observable_safety_state=self._observable_safety_state,
            cancellation_required=self._cancellation_required,
            last_event_detail=self._last_event_detail,
        )
