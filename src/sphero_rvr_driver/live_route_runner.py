"""ROS-free live Mission API v2 route runner for odometry primitives.

The ROS node owns subscriptions and publications.  This module owns the typed
route contract, dynamic scan/TF clearance checks, MotionPrimitiveController seam,
and durable audit manifest assembly so it can be tested without ROS or hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
import json
import math
from typing import Any, Mapping, Optional, Sequence

from .collision_stop import CollisionState, CollisionStopConfig, ScanEvaluation, ScanInput, TwistCommand, evaluate_scan
from .mission_api import MissionValidationError
from .mission_api import ToolInvocation, ToolResultStatus
from .odometry import (
    MotionPrimitiveConfig,
    MotionPrimitiveController,
    MotionPrimitiveGoal,
    MotionPrimitiveKind,
    MotionPrimitiveStopReason,
    MotionPrimitiveTelemetry,
    OdomMotionState,
    encoder_delta,
    normalize_angle,
)

DEFAULT_TRACK_COUNTS_PER_METER = 4337.768


class RouteTerminalReason(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    COMPLETE = "complete"
    CANCELLED = "cancelled"
    STOPPED = "stopped"
    ESTOPPED = "estopped"
    COLLISION_VETO = "collision_veto"
    STALE_ODOM = "stale_odom"
    STALE_SCAN = "stale_scan"
    MISSING_ODOM = "missing_odom"
    MISSING_SCAN = "missing_scan"
    MISSING_COLLISION_STATE = "missing_collision_state"
    STALE_COLLISION_STATE = "stale_collision_state"
    UNSAFE_CLEARANCE = "unsafe_clearance"
    TIMEOUT = "timeout"
    STALL = "stall"
    WRONG_DIRECTION = "wrong_direction"
    MOTION_NOT_SETTLED = "motion_not_settled"
    TARGET_ERROR = "target_error"
    INVALID_ROUTE = "invalid_route"


@dataclass(frozen=True)
class RouteSegmentRequest:
    correlation_id: str
    tool_id: str
    arguments: Mapping[str, Any]
    tool_version: str = "1.0"

    def __post_init__(self) -> None:
        if not str(self.correlation_id).strip():
            raise MissionValidationError("route segment correlation_id is required")
        if self.tool_id not in {"move_distance", "turn_angle"}:
            raise MissionValidationError(f"unsupported live route segment: {self.tool_id}")
        if self.tool_version != "1.0":
            raise MissionValidationError(f"unsupported live route segment version: {self.tool_version}")
        if not isinstance(self.arguments, Mapping):
            raise MissionValidationError("route segment arguments must be an object")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "correlation_id": self.correlation_id,
            "tool_id": self.tool_id,
            "tool_version": self.tool_version,
            "arguments": dict(self.arguments),
        }


@dataclass(frozen=True)
class LiveRouteRequest:
    route_id: str
    segments: Sequence[RouteSegmentRequest]
    max_runtime_s: float
    max_travel_m: float
    source_sha: str = "unknown"
    approval_id: str = ""

    def __post_init__(self) -> None:
        if not str(self.route_id).strip():
            raise MissionValidationError("route_id is required")
        object.__setattr__(self, "segments", tuple(self.segments))
        if not self.segments:
            raise MissionValidationError("live route requires at least one segment")
        if not math.isfinite(float(self.max_runtime_s)) or float(self.max_runtime_s) <= 0.0:
            raise MissionValidationError("max_runtime_s must be positive and finite")
        if not math.isfinite(float(self.max_travel_m)) or float(self.max_travel_m) <= 0.0:
            raise MissionValidationError("max_travel_m must be positive and finite")
        travel = 0.0
        for segment in self.segments:
            if segment.tool_id == "move_distance":
                try:
                    distance_m = abs(float(segment.arguments["distance_m"]))
                except (KeyError, TypeError, ValueError) as exc:
                    raise MissionValidationError("move_distance.distance_m must be finite") from exc
                if not math.isfinite(distance_m) or distance_m <= 0.0:
                    raise MissionValidationError("move_distance.distance_m must be positive and finite")
                travel += distance_m
            elif segment.tool_id == "turn_angle":
                try:
                    angle_deg = float(segment.arguments["angle_deg"])
                except (KeyError, TypeError, ValueError) as exc:
                    raise MissionValidationError("turn_angle.angle_deg must be finite") from exc
                if not math.isfinite(angle_deg) or angle_deg == 0.0:
                    raise MissionValidationError("turn_angle.angle_deg must be non-zero and finite")
        if travel > float(self.max_travel_m) + 1e-9:
            raise MissionValidationError("route exceeds max_travel_m budget")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": "mission_api.v2",
            "route_id": self.route_id,
            "max_runtime_s": self.max_runtime_s,
            "max_travel_m": self.max_travel_m,
            "source_sha": self.source_sha,
            "approval_id": self.approval_id,
            "segments": [segment.to_json_dict() for segment in self.segments],
        }


@dataclass(frozen=True)
class LiveRouteConfig:
    odom: MotionPrimitiveConfig = field(default_factory=MotionPrimitiveConfig)
    scan: CollisionStopConfig = field(default_factory=CollisionStopConfig)
    clearance_margin_m: float = 0.40
    min_translation_cap_m: float = 0.01
    max_translation_segment_m: float = 0.75
    collision_state_max_age_s: float = 0.30
    track_counts_per_meter: float = DEFAULT_TRACK_COUNTS_PER_METER
    terminal_settle_time_s: float = 0.50
    terminal_settle_timeout_s: float = 2.0
    terminal_settle_distance_m: float = 0.005
    terminal_settle_angle_rad: float = math.radians(1.0)
    terminal_settle_encoder_counts: int = 8
    max_terminal_distance_error_m: float = 0.03
    max_terminal_angle_error_rad: float = math.radians(5.0)

    def __post_init__(self) -> None:
        for name in (
            "clearance_margin_m",
            "min_translation_cap_m",
            "max_translation_segment_m",
            "collision_state_max_age_s",
            "track_counts_per_meter",
            "terminal_settle_time_s",
            "terminal_settle_timeout_s",
            "terminal_settle_distance_m",
            "terminal_settle_angle_rad",
            "max_terminal_distance_error_m",
            "max_terminal_angle_error_rad",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.terminal_settle_timeout_s <= self.terminal_settle_time_s:
            raise ValueError("terminal_settle_timeout_s must exceed terminal_settle_time_s")
        if (
            isinstance(self.terminal_settle_encoder_counts, bool)
            or int(self.terminal_settle_encoder_counts) != self.terminal_settle_encoder_counts
            or int(self.terminal_settle_encoder_counts) < 0
        ):
            raise ValueError("terminal_settle_encoder_counts must be a non-negative integer")


@dataclass(frozen=True)
class TrackEncoderState:
    stamp: float
    left_count: int
    right_count: int
    counts_per_meter: float = DEFAULT_TRACK_COUNTS_PER_METER


@dataclass(frozen=True)
class LiveRouteState:
    stamp: float
    odom: Optional[OdomMotionState]
    scan: Optional[ScanInput]
    collision_state: Optional[str] = None
    collision_received_at: Optional[float] = None
    stop: bool = False
    estop: bool = False
    cancel: bool = False
    encoder_counts: Optional[TrackEncoderState] = None


@dataclass(frozen=True)
class ExecutedRouteSegment:
    correlation_id: str
    tool_id: str
    status: ToolResultStatus
    requested: Mapping[str, Any]
    executed: Mapping[str, Any]
    measured_distance_m: float = 0.0
    measured_angle_deg: float = 0.0
    terminal_reason: str = RouteTerminalReason.RUNNING.value
    collision_state: str = CollisionState.CLEAR.value
    start_pose: Optional[Mapping[str, float]] = None
    final_pose: Optional[Mapping[str, float]] = None
    heading_change_deg: Optional[float] = None
    encoder_start_stamp: Optional[float] = None
    encoder_final_stamp: Optional[float] = None
    left_encoder_delta_counts: Optional[int] = None
    right_encoder_delta_counts: Optional[int] = None
    left_track_distance_m: Optional[float] = None
    right_track_distance_m: Optional[float] = None
    terminal_settled: bool = False
    terminal_settle_duration_s: Optional[float] = None
    terminal_distance_error_m: Optional[float] = None
    terminal_angle_error_deg: Optional[float] = None

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "correlation_id": self.correlation_id,
            "tool_id": self.tool_id,
            "status": self.status.value,
            "requested": dict(self.requested),
            "executed": dict(self.executed),
            "measured_distance_m": self.measured_distance_m,
            "measured_angle_deg": self.measured_angle_deg,
            "terminal_reason": self.terminal_reason,
            "collision_state": self.collision_state,
            "start_pose": None if self.start_pose is None else dict(self.start_pose),
            "final_pose": None if self.final_pose is None else dict(self.final_pose),
            "heading_change_deg": self.heading_change_deg,
            "encoder_start_stamp": self.encoder_start_stamp,
            "encoder_final_stamp": self.encoder_final_stamp,
            "left_encoder_delta_counts": self.left_encoder_delta_counts,
            "right_encoder_delta_counts": self.right_encoder_delta_counts,
            "left_track_distance_m": self.left_track_distance_m,
            "right_track_distance_m": self.right_track_distance_m,
            "terminal_settled": self.terminal_settled,
            "terminal_settle_duration_s": self.terminal_settle_duration_s,
            "terminal_distance_error_m": self.terminal_distance_error_m,
            "terminal_angle_error_deg": self.terminal_angle_error_deg,
        }


@dataclass(frozen=True)
class LiveRouteManifest:
    route_id: str
    status: ToolResultStatus
    terminal_reason: str
    proposed_segments: Sequence[Mapping[str, Any]]
    executed_segments: Sequence[ExecutedRouteSegment]
    measured_distance_m: float
    measured_angle_deg: float
    collision_state: str
    source_sha: str
    route_start_pose: Optional[Mapping[str, float]] = None
    route_final_pose: Optional[Mapping[str, float]] = None
    route_delta_x_m: Optional[float] = None
    route_delta_y_m: Optional[float] = None
    route_displacement_m: Optional[float] = None
    route_heading_change_deg: Optional[float] = None
    final_heading_deg: Optional[float] = None
    encoder_start_stamp: Optional[float] = None
    encoder_final_stamp: Optional[float] = None
    left_encoder_delta_counts: Optional[int] = None
    right_encoder_delta_counts: Optional[int] = None
    left_track_distance_m: Optional[float] = None
    right_track_distance_m: Optional[float] = None
    terminal_settled: bool = False
    terminal_settle_duration_s: Optional[float] = None

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "api_version": "mission_api.v2",
            "route_id": self.route_id,
            "status": self.status.value,
            "terminal_reason": self.terminal_reason,
            "proposed_segments": [dict(item) for item in self.proposed_segments],
            "executed_segments": [segment.to_json_dict() for segment in self.executed_segments],
            "measured_distance_m": self.measured_distance_m,
            "measured_angle_deg": self.measured_angle_deg,
            "collision_state": self.collision_state,
            "source_sha": self.source_sha,
            "route_start_pose": None if self.route_start_pose is None else dict(self.route_start_pose),
            "route_final_pose": None if self.route_final_pose is None else dict(self.route_final_pose),
            "route_delta_x_m": self.route_delta_x_m,
            "route_delta_y_m": self.route_delta_y_m,
            "route_displacement_m": self.route_displacement_m,
            "route_heading_change_deg": self.route_heading_change_deg,
            "final_heading_deg": self.final_heading_deg,
            "encoder_start_stamp": self.encoder_start_stamp,
            "encoder_final_stamp": self.encoder_final_stamp,
            "left_encoder_delta_counts": self.left_encoder_delta_counts,
            "right_encoder_delta_counts": self.right_encoder_delta_counts,
            "left_track_distance_m": self.left_track_distance_m,
            "right_track_distance_m": self.right_track_distance_m,
            "terminal_settled": self.terminal_settled,
            "terminal_settle_duration_s": self.terminal_settle_duration_s,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_json_dict(), sort_keys=True)


@dataclass
class _ActiveSegment:
    request: RouteSegmentRequest
    goal: MotionPrimitiveGoal
    executed_arguments: Mapping[str, Any]
    controller: MotionPrimitiveController
    telemetry: MotionPrimitiveTelemetry
    start_odom: OdomMotionState
    start_encoder_counts: Optional[TrackEncoderState]


@dataclass
class _SettlingState:
    started_at: float
    stable_since: float
    reference_odom: OdomMotionState
    reference_encoder_counts: Optional[TrackEncoderState]


class LiveRouteRunner:
    """Step-wise live route runner that publishes only TwistCommand intents."""

    def __init__(self, config: Optional[LiveRouteConfig] = None):
        self.config = config or LiveRouteConfig()
        self._request: Optional[LiveRouteRequest] = None
        self._started_at: float = 0.0
        self._segment_index = 0
        self._active: Optional[_ActiveSegment] = None
        self._executed: list[ExecutedRouteSegment] = []
        self._terminal_reason = RouteTerminalReason.IDLE.value
        self._last_collision_state = "MISSING"
        self._route_start_odom: Optional[OdomMotionState] = None
        self._route_final_odom: Optional[OdomMotionState] = None
        self._route_start_encoder_counts: Optional[TrackEncoderState] = None
        self._route_final_encoder_counts: Optional[TrackEncoderState] = None
        self._settling: Optional[_SettlingState] = None

    @property
    def active(self) -> bool:
        return self._request is not None and self._terminal_reason == RouteTerminalReason.RUNNING.value

    def start(self, request: LiveRouteRequest, state: LiveRouteState) -> TwistCommand:
        self._request = request
        self._started_at = float(state.stamp)
        self._segment_index = 0
        self._active = None
        self._executed = []
        self._terminal_reason = RouteTerminalReason.RUNNING.value
        self._last_collision_state = str(state.collision_state)
        self._route_start_odom = state.odom
        self._route_final_odom = state.odom
        self._route_start_encoder_counts = state.encoder_counts
        self._route_final_encoder_counts = state.encoder_counts
        self._settling = None
        return self.update(state)

    def abort(self, reason: str, state: LiveRouteState) -> None:
        """Force a terminal manifest state after node-side validation failures."""
        self._finish(reason, state)

    def update(self, state: LiveRouteState) -> TwistCommand:
        if self._request is None or self._terminal_reason != RouteTerminalReason.RUNNING.value:
            return TwistCommand()
        terminal = self._safety_terminal(state)
        if terminal is not None:
            self._finish(terminal, state)
            return TwistCommand()
        if float(state.stamp) - self._started_at > self._request.max_runtime_s:
            self._finish(RouteTerminalReason.TIMEOUT.value, state)
            return TwistCommand()
        if self._active is None:
            if self._segment_index >= len(self._request.segments):
                self._finish(RouteTerminalReason.COMPLETE.value, state)
                return TwistCommand()
            try:
                self._active = self._start_segment(self._request.segments[self._segment_index], state)
            except MissionValidationError as exc:
                self._finish(_normalize_exception_terminal(exc), state)
                return TwistCommand()
            return self._active.telemetry.command
        assert state.odom is not None
        if self._settling is not None:
            return self._update_settling(state)
        telemetry = self._active.controller.update(
            state.odom,
            now=float(state.stamp),
            cancel=state.cancel,
            stop=state.stop,
            estop=state.estop,
            collision_veto=_collision_veto(state.collision_state),
        )
        self._active.telemetry = telemetry
        if telemetry.stop_reason is MotionPrimitiveStopReason.RUNNING:
            return telemetry.command
        if telemetry.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED:
            self._settling = _SettlingState(
                started_at=float(state.stamp),
                stable_since=float(state.stamp),
                reference_odom=state.odom,
                reference_encoder_counts=state.encoder_counts,
            )
            return TwistCommand()
        self._record_active(telemetry, state)
        terminal = self._terminal_for_active_stop(telemetry.stop_reason, state)
        self._finish(terminal, state)
        return TwistCommand()

    def manifest(self) -> LiveRouteManifest:
        request = self._require_request()
        status = self._status_for_terminal(self._terminal_reason)
        final_segment = self._executed[-1] if self._executed else None
        pose_measurement = _pose_measurement(self._route_start_odom, self._route_final_odom)
        track_measurement = _track_measurement(
            self._route_start_encoder_counts,
            self._route_final_encoder_counts,
            counts_per_meter=self.config.track_counts_per_meter,
            expected_start_stamp=None if self._route_start_odom is None else self._route_start_odom.stamp,
            expected_final_stamp=None if self._route_final_odom is None else self._route_final_odom.stamp,
            max_sample_age_s=self.config.odom.max_sample_age_s,
        )
        return LiveRouteManifest(
            route_id=request.route_id,
            status=status,
            terminal_reason=self._terminal_reason,
            proposed_segments=[segment.to_json_dict() for segment in request.segments],
            executed_segments=tuple(self._executed),
            measured_distance_m=sum(segment.measured_distance_m for segment in self._executed),
            measured_angle_deg=sum(segment.measured_angle_deg for segment in self._executed),
            collision_state=self._last_collision_state,
            source_sha=request.source_sha,
            terminal_settled=bool(final_segment and final_segment.terminal_settled),
            terminal_settle_duration_s=(
                None if final_segment is None else final_segment.terminal_settle_duration_s
            ),
            **pose_measurement,
            **track_measurement,
        )

    def _start_segment(self, segment: RouteSegmentRequest, state: LiveRouteState) -> _ActiveSegment:
        assert state.odom is not None
        if segment.tool_id == "move_distance":
            goal, executed_arguments = self._move_goal(segment, state)
        else:
            goal, executed_arguments = self._turn_goal(segment)
        controller = MotionPrimitiveController(self.config.odom)
        telemetry = controller.start(goal, state.odom)
        return _ActiveSegment(
            segment,
            goal,
            executed_arguments,
            controller,
            telemetry,
            state.odom,
            state.encoder_counts,
        )

    def _move_goal(self, segment: RouteSegmentRequest, state: LiveRouteState) -> tuple[MotionPrimitiveGoal, Mapping[str, Any]]:
        requested_distance = float(segment.arguments["distance_m"])
        speed = float(segment.arguments["speed_mps"])
        timeout_s = float(segment.arguments["timeout_s"])
        _require_positive_finite(speed, "move_distance.speed_mps")
        _require_positive_finite(timeout_s, "move_distance.timeout_s")
        cap = self.dynamic_translation_cap(state, direction=1.0 if requested_distance > 0.0 else -1.0)
        executed_distance = math.copysign(min(abs(requested_distance), cap, self.config.max_translation_segment_m), requested_distance)
        if abs(executed_distance) < self.config.min_translation_cap_m:
            raise MissionValidationError("unsafe_clearance: translation cap below minimum")
        return (
            MotionPrimitiveGoal.move_distance(distance_m=executed_distance, speed_mps=speed, timeout_s=timeout_s),
            {"distance_m": executed_distance, "speed_mps": speed, "timeout_s": timeout_s, "dynamic_cap_m": cap},
        )

    def _turn_goal(self, segment: RouteSegmentRequest) -> tuple[MotionPrimitiveGoal, Mapping[str, Any]]:
        angle_deg = float(segment.arguments["angle_deg"])
        angular_speed_deg_s = float(segment.arguments["angular_speed_deg_s"])
        timeout_s = float(segment.arguments["timeout_s"])
        _require_positive_finite(angular_speed_deg_s, "turn_angle.angular_speed_deg_s")
        _require_positive_finite(timeout_s, "turn_angle.timeout_s")
        return (
            MotionPrimitiveGoal.turn_angle(
                angle_rad=math.radians(angle_deg),
                angular_speed_rad_s=math.radians(angular_speed_deg_s),
                timeout_s=timeout_s,
            ),
            {"angle_deg": angle_deg, "angular_speed_deg_s": angular_speed_deg_s, "timeout_s": timeout_s},
        )

    def dynamic_translation_cap(self, state: LiveRouteState, *, direction: float) -> float:
        scan_eval = self._require_scan_eval(state)
        sector = "front" if direction >= 0.0 else "rear"
        clearance = scan_eval.nearest.get(sector)
        if clearance is None:
            raise MissionValidationError(f"unsafe_clearance: {sector} corridor unknown")
        return max(0.0, float(clearance) - self.config.clearance_margin_m)

    def _require_scan_eval(self, state: LiveRouteState) -> ScanEvaluation:
        if state.scan is None:
            raise MissionValidationError("missing_scan")
        scan_eval = evaluate_scan(state.scan, self.config.scan, now=float(state.stamp))
        if not scan_eval.healthy:
            reason = RouteTerminalReason.STALE_SCAN.value if scan_eval.reason == "stale_scan" else scan_eval.reason
            raise MissionValidationError(reason)
        return scan_eval

    def _safety_terminal(self, state: LiveRouteState) -> Optional[str]:
        self._last_collision_state = str(state.collision_state or "MISSING")
        if state.estop:
            return RouteTerminalReason.ESTOPPED.value
        if state.stop:
            return RouteTerminalReason.STOPPED.value
        if state.cancel:
            return RouteTerminalReason.CANCELLED.value
        collision_terminal = self._collision_terminal(state)
        if collision_terminal is not None:
            return collision_terminal
        if state.odom is None:
            return RouteTerminalReason.MISSING_ODOM.value
        if float(state.stamp) - float(state.odom.stamp) > self.config.odom.max_sample_age_s:
            return RouteTerminalReason.STALE_ODOM.value
        if state.scan is None:
            return RouteTerminalReason.MISSING_SCAN.value
        scan_eval = evaluate_scan(state.scan, self.config.scan, now=float(state.stamp))
        if not scan_eval.healthy:
            return RouteTerminalReason.STALE_SCAN.value if scan_eval.reason == "stale_scan" else RouteTerminalReason.UNSAFE_CLEARANCE.value
        if _collision_veto(state.collision_state):
            return RouteTerminalReason.COLLISION_VETO.value
        return None

    def _collision_terminal(self, state: LiveRouteState) -> Optional[str]:
        if state.collision_state is None or state.collision_received_at is None:
            return RouteTerminalReason.MISSING_COLLISION_STATE.value
        try:
            received_at = float(state.collision_received_at)
            collision_state = CollisionState(str(state.collision_state).upper())
        except (TypeError, ValueError):
            return RouteTerminalReason.MISSING_COLLISION_STATE.value
        age_s = float(state.stamp) - received_at
        if not math.isfinite(age_s) or age_s < -1e-9:
            return RouteTerminalReason.MISSING_COLLISION_STATE.value
        if age_s > self.config.collision_state_max_age_s + 1e-9:
            return RouteTerminalReason.STALE_COLLISION_STATE.value
        # SLOW is an authoritative, fresh supervisor state: the downstream
        # collision supervisor is actively bounding the command.  Treat only
        # its stop/fault states as vetoes so route progress remains observable
        # while that bounded command is applied.
        if collision_state not in {CollisionState.CLEAR, CollisionState.SLOW}:
            if collision_state is CollisionState.STARTUP:
                return RouteTerminalReason.MISSING_COLLISION_STATE.value
            return RouteTerminalReason.COLLISION_VETO.value
        return None

    def _record_active(
        self,
        telemetry: MotionPrimitiveTelemetry,
        state: LiveRouteState,
        *,
        status_override: Optional[ToolResultStatus] = None,
        terminal_reason_override: Optional[str] = None,
        terminal_settled: bool = False,
        terminal_settle_duration_s: Optional[float] = None,
    ) -> None:
        active = self._active
        if active is None:
            return
        measured_angle = math.degrees(telemetry.measured_angle_rad)
        status = status_override or (
            ToolResultStatus.COMPLETE
            if telemetry.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
            else self._status_for_terminal(_terminal_for_odom_stop(telemetry.stop_reason, kind=active.goal.kind))
        )
        pose_measurement = _segment_pose_measurement(active.start_odom, state.odom)
        track_measurement = _track_measurement(
            active.start_encoder_counts,
            state.encoder_counts,
            counts_per_meter=self.config.track_counts_per_meter,
            expected_start_stamp=active.start_odom.stamp,
            expected_final_stamp=None if state.odom is None else state.odom.stamp,
            max_sample_age_s=self.config.odom.max_sample_age_s,
        )
        self._executed.append(
            ExecutedRouteSegment(
                correlation_id=active.request.correlation_id,
                tool_id=active.request.tool_id,
                status=status,
                requested=dict(active.request.arguments),
                executed=dict(active.executed_arguments),
                measured_distance_m=telemetry.measured_distance_m,
                measured_angle_deg=measured_angle,
                terminal_reason=terminal_reason_override or telemetry.stop_reason.value,
                collision_state=str(state.collision_state),
                terminal_settled=terminal_settled,
                terminal_settle_duration_s=terminal_settle_duration_s,
                terminal_distance_error_m=(
                    abs(telemetry.measured_distance_m - abs(active.goal.target))
                    if active.goal.kind is MotionPrimitiveKind.MOVE_DISTANCE
                    else None
                ),
                terminal_angle_error_deg=(
                    math.degrees(abs(abs(telemetry.measured_angle_rad) - abs(active.goal.target)))
                    if active.goal.kind is MotionPrimitiveKind.TURN_ANGLE
                    else None
                ),
                **pose_measurement,
                **track_measurement,
            )
        )

    def _update_settling(self, state: LiveRouteState) -> TwistCommand:
        active = self._active
        settling = self._settling
        assert active is not None and settling is not None and state.odom is not None
        telemetry = active.controller.update(state.odom, now=float(state.stamp))
        active.telemetry = telemetry
        elapsed = float(state.stamp) - settling.started_at
        if elapsed > self.config.terminal_settle_timeout_s:
            self._record_active(
                telemetry,
                state,
                status_override=ToolResultStatus.FAILED,
                terminal_reason_override=RouteTerminalReason.MOTION_NOT_SETTLED.value,
                terminal_settle_duration_s=elapsed,
            )
            self._finish(RouteTerminalReason.MOTION_NOT_SETTLED.value, state)
            return TwistCommand()
        if self._settling_sample_moved(settling, state):
            settling.stable_since = float(state.stamp)
            settling.reference_odom = state.odom
            settling.reference_encoder_counts = state.encoder_counts
            return TwistCommand()
        if float(state.stamp) - settling.stable_since < self.config.terminal_settle_time_s:
            return TwistCommand()

        settled_duration = float(state.stamp) - settling.started_at
        if self._terminal_target_error_exceeded(active, telemetry):
            self._record_active(
                telemetry,
                state,
                status_override=ToolResultStatus.FAILED,
                terminal_reason_override=RouteTerminalReason.TARGET_ERROR.value,
                terminal_settled=True,
                terminal_settle_duration_s=settled_duration,
            )
            self._finish(RouteTerminalReason.TARGET_ERROR.value, state)
            return TwistCommand()
        self._record_active(
            telemetry,
            state,
            terminal_settled=True,
            terminal_settle_duration_s=settled_duration,
        )
        if active.request.tool_id == "move_distance" and self._active_is_partial_translation():
            self._finish(RouteTerminalReason.UNSAFE_CLEARANCE.value, state)
            return TwistCommand()
        self._segment_index += 1
        self._active = None
        self._settling = None
        return self.update(state)

    def _settling_sample_moved(self, settling: _SettlingState, state: LiveRouteState) -> bool:
        assert state.odom is not None
        dx = float(state.odom.x_m) - float(settling.reference_odom.x_m)
        dy = float(state.odom.y_m) - float(settling.reference_odom.y_m)
        if math.hypot(dx, dy) > self.config.terminal_settle_distance_m:
            return True
        angle_delta = abs(normalize_angle(float(state.odom.yaw_rad) - float(settling.reference_odom.yaw_rad)))
        if angle_delta > self.config.terminal_settle_angle_rad:
            return True
        previous_encoder = settling.reference_encoder_counts
        current_encoder = state.encoder_counts
        if previous_encoder is not None and current_encoder is not None:
            if (
                abs(encoder_delta(current_encoder.left_count, previous_encoder.left_count))
                > self.config.terminal_settle_encoder_counts
            ):
                return True
            if (
                abs(encoder_delta(current_encoder.right_count, previous_encoder.right_count))
                > self.config.terminal_settle_encoder_counts
            ):
                return True
        return False

    def _terminal_target_error_exceeded(
        self,
        active: _ActiveSegment,
        telemetry: MotionPrimitiveTelemetry,
    ) -> bool:
        if active.goal.kind is MotionPrimitiveKind.MOVE_DISTANCE:
            error = abs(telemetry.measured_distance_m - abs(active.goal.target))
            return error > self.config.max_terminal_distance_error_m
        error = abs(abs(telemetry.measured_angle_rad) - abs(active.goal.target))
        return error > self.config.max_terminal_angle_error_rad

    def _active_is_partial_translation(self) -> bool:
        active = self._active
        if active is None or active.request.tool_id != "move_distance":
            return False
        requested = abs(float(active.request.arguments["distance_m"]))
        executed = abs(float(active.executed_arguments["distance_m"]))
        return executed + self.config.odom.distance_tolerance_m < requested

    def _terminal_for_active_stop(self, reason: MotionPrimitiveStopReason, state: LiveRouteState) -> str:
        active = self._active
        if active is None:
            return RouteTerminalReason.STALL.value
        if reason is MotionPrimitiveStopReason.STALL and active.goal.kind is MotionPrimitiveKind.TURN_ANGLE:
            start = active.controller._require_state().start
            signed = 0.0 if state.odom is None else normalize_angle(float(state.odom.yaw_rad) - float(start.yaw_rad))
            directional = signed if active.goal.target >= 0.0 else -signed
            if directional < -self.config.odom.min_angle_progress_rad:
                return RouteTerminalReason.WRONG_DIRECTION.value
            return RouteTerminalReason.STALL.value
        return _terminal_for_odom_stop(reason, kind=active.goal.kind)

    def _finish(self, reason: str, state: LiveRouteState) -> None:
        self._terminal_reason = reason
        self._last_collision_state = str(state.collision_state or "MISSING")
        self._route_final_odom = state.odom
        self._route_final_encoder_counts = state.encoder_counts
        self._settling = None

    @staticmethod
    def _status_for_terminal(reason: str) -> ToolResultStatus:
        if reason == RouteTerminalReason.COMPLETE.value:
            return ToolResultStatus.COMPLETE
        if reason == RouteTerminalReason.TIMEOUT.value:
            return ToolResultStatus.TIMEOUT
        if reason == RouteTerminalReason.CANCELLED.value:
            return ToolResultStatus.CANCELLED
        if reason == RouteTerminalReason.STOPPED.value:
            return ToolResultStatus.STOPPED
        if reason == RouteTerminalReason.ESTOPPED.value:
            return ToolResultStatus.ESTOPPED
        if reason in {
            RouteTerminalReason.COLLISION_VETO.value,
            RouteTerminalReason.STALE_ODOM.value,
            RouteTerminalReason.STALE_SCAN.value,
            RouteTerminalReason.MISSING_ODOM.value,
            RouteTerminalReason.MISSING_SCAN.value,
            RouteTerminalReason.MISSING_COLLISION_STATE.value,
            RouteTerminalReason.STALE_COLLISION_STATE.value,
            RouteTerminalReason.UNSAFE_CLEARANCE.value,
        }:
            return ToolResultStatus.BLOCKED
        if reason in {
            RouteTerminalReason.MOTION_NOT_SETTLED.value,
            RouteTerminalReason.TARGET_ERROR.value,
        }:
            return ToolResultStatus.FAILED
        return ToolResultStatus.FAILED

    def _require_request(self) -> LiveRouteRequest:
        if self._request is None:
            raise RuntimeError("live route runner has not been started")
        return self._request


def _pose_dict(state: OdomMotionState) -> dict[str, float]:
    return {
        "stamp": float(state.stamp),
        "x_m": float(state.x_m),
        "y_m": float(state.y_m),
        "heading_deg": math.degrees(float(state.yaw_rad)),
    }


def _pose_measurement(
    start: Optional[OdomMotionState],
    final: Optional[OdomMotionState],
) -> dict[str, Any]:
    if start is None or final is None:
        return {
            "route_start_pose": None,
            "route_final_pose": None,
            "route_delta_x_m": None,
            "route_delta_y_m": None,
            "route_displacement_m": None,
            "route_heading_change_deg": None,
            "final_heading_deg": None,
        }
    delta_x = float(final.x_m) - float(start.x_m)
    delta_y = float(final.y_m) - float(start.y_m)
    return {
        "route_start_pose": _pose_dict(start),
        "route_final_pose": _pose_dict(final),
        "route_delta_x_m": delta_x,
        "route_delta_y_m": delta_y,
        "route_displacement_m": math.hypot(delta_x, delta_y),
        "route_heading_change_deg": math.degrees(
            normalize_angle(float(final.yaw_rad) - float(start.yaw_rad))
        ),
        "final_heading_deg": math.degrees(float(final.yaw_rad)),
    }


def _segment_pose_measurement(
    start: OdomMotionState,
    final: Optional[OdomMotionState],
) -> dict[str, Any]:
    if final is None:
        return {"start_pose": _pose_dict(start), "final_pose": None, "heading_change_deg": None}
    return {
        "start_pose": _pose_dict(start),
        "final_pose": _pose_dict(final),
        "heading_change_deg": math.degrees(
            normalize_angle(float(final.yaw_rad) - float(start.yaw_rad))
        ),
    }


def _track_measurement(
    start: Optional[TrackEncoderState],
    final: Optional[TrackEncoderState],
    *,
    counts_per_meter: float,
    expected_start_stamp: Optional[float],
    expected_final_stamp: Optional[float],
    max_sample_age_s: float,
) -> dict[str, Any]:
    empty = {
        "encoder_start_stamp": None,
        "encoder_final_stamp": None,
        "left_encoder_delta_counts": None,
        "right_encoder_delta_counts": None,
        "left_track_distance_m": None,
        "right_track_distance_m": None,
    }
    if start is None or final is None:
        return empty
    if final.stamp <= start.stamp:
        return empty
    if expected_start_stamp is None or expected_final_stamp is None:
        return empty
    if (
        abs(float(start.stamp) - float(expected_start_stamp)) > float(max_sample_age_s)
        or abs(float(final.stamp) - float(expected_final_stamp)) > float(max_sample_age_s)
    ):
        return empty
    if not (
        math.isclose(float(start.counts_per_meter), float(counts_per_meter), rel_tol=1e-9)
        and math.isclose(float(final.counts_per_meter), float(counts_per_meter), rel_tol=1e-9)
    ):
        return empty
    left_delta = encoder_delta(final.left_count, start.left_count)
    right_delta = encoder_delta(final.right_count, start.right_count)
    return {
        "encoder_start_stamp": float(start.stamp),
        "encoder_final_stamp": float(final.stamp),
        "left_encoder_delta_counts": left_delta,
        "right_encoder_delta_counts": right_delta,
        "left_track_distance_m": left_delta / float(counts_per_meter),
        "right_track_distance_m": right_delta / float(counts_per_meter),
    }


def route_request_from_json(payload: str, *, source_sha: str = "unknown") -> LiveRouteRequest:
    data = json.loads(payload or "{}")
    if not isinstance(data, Mapping):
        raise MissionValidationError("route request must be a JSON object")
    route_id = str(data.get("route_id") or data.get("plan_id") or "live-route")
    budgets = data.get("budgets", {}) if isinstance(data.get("budgets", {}), Mapping) else {}
    max_runtime_s = float(data.get("max_runtime_s", budgets.get("max_runtime_s", 30.0)))
    max_travel_m = float(data.get("max_travel_m", budgets.get("max_travel_m", 2.0)))
    raw_segments = data.get("segments")
    if raw_segments is None and isinstance(data.get("invocations"), Sequence):
        raw_segments = data["invocations"]
    if raw_segments is None and isinstance(data.get("plan"), Mapping):
        raw_segments = data["plan"].get("invocations")
        goal = data["plan"].get("goal", {}) if isinstance(data["plan"].get("goal", {}), Mapping) else {}
        plan_budgets = goal.get("budgets", {}) if isinstance(goal.get("budgets", {}), Mapping) else {}
        max_runtime_s = float(plan_budgets.get("max_runtime_s", max_runtime_s))
        max_travel_m = float(plan_budgets.get("max_travel_m", max_travel_m))
        route_id = str(data["plan"].get("plan_id") or route_id)
    if not isinstance(raw_segments, Sequence) or isinstance(raw_segments, (str, bytes)):
        raise MissionValidationError("route request requires segments/invocations array")
    segments = tuple(_segment_from_mapping(item) for item in raw_segments)
    return LiveRouteRequest(
        route_id=route_id,
        segments=segments,
        max_runtime_s=max_runtime_s,
        max_travel_m=max_travel_m,
        source_sha=str(data.get("source_sha") or source_sha),
        approval_id=str(data.get("approval_id", "")),
    )


def _segment_from_mapping(value: Any) -> RouteSegmentRequest:
    if isinstance(value, ToolInvocation):
        return RouteSegmentRequest(value.correlation_id, value.tool_id, value.arguments, value.tool_version)
    if not isinstance(value, Mapping):
        raise MissionValidationError("route segment must be an object")
    return RouteSegmentRequest(
        correlation_id=str(value.get("correlation_id") or value.get("id") or value.get("tool_id", "")),
        tool_id=str(value.get("tool_id", "")),
        tool_version=str(value.get("tool_version", "1.0")),
        arguments=value.get("arguments", {}),
    )


def run_route_replay(request: LiveRouteRequest, states: Sequence[LiveRouteState], config: Optional[LiveRouteConfig] = None) -> LiveRouteManifest:
    if not states:
        raise MissionValidationError("live route replay requires at least one state")
    runner = LiveRouteRunner(config)
    try:
        runner.start(request, states[0])
        for state in states[1:]:
            runner.update(state)
            _settle_replay_target(runner, state)
            if not runner.active:
                break
    except MissionValidationError as exc:
        if runner._request is None:
            runner._request = request
        runner.abort(_normalize_exception_terminal(exc), states[0])
    return runner.manifest()


def _settle_replay_target(runner: LiveRouteRunner, state: LiveRouteState) -> None:
    """Advance a replay's final stationary sample through the settle window.

    Real ROS execution must receive fresh samples for the complete settle window.
    Replay fixtures conventionally provide one state per reached target, so this
    helper repeats that target as a fresh stationary observation before the next
    fixture state is consumed.
    """

    if not runner.active or runner._settling is None or state.odom is None:
        return
    stamp = float(state.stamp) + runner.config.terminal_settle_time_s + 1e-6
    odom = replace(state.odom, stamp=stamp)
    scan = state.scan
    if scan is not None:
        scan = replace(scan, stamp=stamp, received_at=stamp)
    encoder_counts = state.encoder_counts
    if (
        encoder_counts is not None
        and abs(float(encoder_counts.stamp) - float(state.odom.stamp)) <= runner.config.odom.max_sample_age_s
    ):
        encoder_counts = replace(encoder_counts, stamp=stamp)
    runner.update(
        replace(
            state,
            stamp=stamp,
            odom=odom,
            scan=scan,
            collision_received_at=stamp,
            encoder_counts=encoder_counts,
        )
    )


def _normalize_exception_terminal(exc: Exception) -> str:
    message = str(exc)
    if "stale_scan" in message:
        return RouteTerminalReason.STALE_SCAN.value
    if "missing_scan" in message:
        return RouteTerminalReason.MISSING_SCAN.value
    if "unsafe_clearance" in message:
        return RouteTerminalReason.UNSAFE_CLEARANCE.value
    return RouteTerminalReason.INVALID_ROUTE.value


def _collision_veto(state: Optional[str]) -> bool:
    normalized = str(state or "").upper()
    return normalized in {
        CollisionState.STOPPED.value,
        CollisionState.SENSOR_STALE.value,
        CollisionState.ESTOPPED.value,
        CollisionState.DISABLED.value,
    }


def _require_positive_finite(value: float, name: str) -> None:
    if not math.isfinite(float(value)) or float(value) <= 0.0:
        raise MissionValidationError(f"{name} must be positive and finite")


def _terminal_for_odom_stop(reason: MotionPrimitiveStopReason, *, kind: MotionPrimitiveKind) -> str:
    if reason is MotionPrimitiveStopReason.TIMEOUT:
        return RouteTerminalReason.TIMEOUT.value
    if reason is MotionPrimitiveStopReason.STALE_ODOM:
        return RouteTerminalReason.STALE_ODOM.value
    if reason is MotionPrimitiveStopReason.CANCELLED:
        return RouteTerminalReason.CANCELLED.value
    if reason is MotionPrimitiveStopReason.STOP:
        return RouteTerminalReason.STOPPED.value
    if reason is MotionPrimitiveStopReason.ESTOP:
        return RouteTerminalReason.ESTOPPED.value
    if reason is MotionPrimitiveStopReason.COLLISION_VETO:
        return RouteTerminalReason.COLLISION_VETO.value
    if reason is MotionPrimitiveStopReason.STALL:
        return RouteTerminalReason.STALL.value
    return RouteTerminalReason.STALL.value
