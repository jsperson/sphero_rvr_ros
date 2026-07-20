"""ROS-free live Mission API v2 route runner for odometry primitives.

The ROS node owns subscriptions and publications.  This module owns the typed
route contract, dynamic scan/TF clearance checks, MotionPrimitiveController seam,
and durable audit manifest assembly so it can be tested without ROS or hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import json
import math
from typing import Any, Mapping, Optional, Sequence

from .collision_stop import CollisionState, CollisionStopConfig, ScanEvaluation, ScanInput, TwistCommand, evaluate_scan
from .mission_api import MissionValidationError
from .mission_api_v2 import ToolInvocation, ToolResultStatus
from .odometry import (
    MotionPrimitiveConfig,
    MotionPrimitiveController,
    MotionPrimitiveGoal,
    MotionPrimitiveKind,
    MotionPrimitiveStopReason,
    MotionPrimitiveTelemetry,
    OdomMotionState,
    normalize_angle,
)


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
    UNSAFE_CLEARANCE = "unsafe_clearance"
    TIMEOUT = "timeout"
    STALL = "stall"
    WRONG_DIRECTION = "wrong_direction"
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

    def __post_init__(self) -> None:
        for name in ("clearance_margin_m", "min_translation_cap_m", "max_translation_segment_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")


@dataclass(frozen=True)
class LiveRouteState:
    stamp: float
    odom: Optional[OdomMotionState]
    scan: Optional[ScanInput]
    collision_state: str = CollisionState.CLEAR.value
    stop: bool = False
    estop: bool = False
    cancel: bool = False


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
        self._last_collision_state = CollisionState.CLEAR.value

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
        return self.update(state)

    def abort(self, reason: str, state: LiveRouteState) -> None:
        """Force a terminal manifest state after node-side validation failures."""
        self._finish(reason, self._status_for_terminal(reason), state)

    def update(self, state: LiveRouteState) -> TwistCommand:
        if self._request is None or self._terminal_reason != RouteTerminalReason.RUNNING.value:
            return TwistCommand()
        terminal = self._safety_terminal(state)
        if terminal is not None:
            self._finish(terminal, self._status_for_terminal(terminal), state)
            return TwistCommand()
        if float(state.stamp) - self._started_at > self._request.max_runtime_s:
            self._finish(RouteTerminalReason.TIMEOUT.value, ToolResultStatus.TIMEOUT, state)
            return TwistCommand()
        if self._active is None:
            if self._segment_index >= len(self._request.segments):
                self._finish(RouteTerminalReason.COMPLETE.value, ToolResultStatus.COMPLETE, state)
                return TwistCommand()
            self._active = self._start_segment(self._request.segments[self._segment_index], state)
            return self._active.telemetry.command
        assert state.odom is not None
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
        self._record_active(telemetry, state)
        if telemetry.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED:
            if self._active and self._active.request.tool_id == "move_distance" and self._active_is_partial_translation():
                self._finish(RouteTerminalReason.UNSAFE_CLEARANCE.value, ToolResultStatus.BLOCKED, state)
                return TwistCommand()
            self._segment_index += 1
            self._active = None
            return self.update(state)
        terminal = self._terminal_for_active_stop(telemetry.stop_reason, state)
        self._finish(terminal, self._status_for_terminal(terminal), state)
        return TwistCommand()

    def manifest(self) -> LiveRouteManifest:
        request = self._require_request()
        status = ToolResultStatus.COMPLETE if self._terminal_reason == RouteTerminalReason.COMPLETE.value else self._status_for_terminal(self._terminal_reason)
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
        )

    def _start_segment(self, segment: RouteSegmentRequest, state: LiveRouteState) -> _ActiveSegment:
        assert state.odom is not None
        if segment.tool_id == "move_distance":
            goal, executed_arguments = self._move_goal(segment, state)
        else:
            goal, executed_arguments = self._turn_goal(segment)
        controller = MotionPrimitiveController(self.config.odom)
        telemetry = controller.start(goal, state.odom)
        return _ActiveSegment(segment, goal, executed_arguments, controller, telemetry)

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
        self._last_collision_state = str(state.collision_state)
        if state.estop:
            return RouteTerminalReason.ESTOPPED.value
        if state.stop:
            return RouteTerminalReason.STOPPED.value
        if state.cancel:
            return RouteTerminalReason.CANCELLED.value
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

    def _record_active(self, telemetry: MotionPrimitiveTelemetry, state: LiveRouteState) -> None:
        active = self._active
        if active is None:
            return
        measured_angle = math.degrees(telemetry.measured_angle_rad)
        status = (
            ToolResultStatus.COMPLETE
            if telemetry.stop_reason is MotionPrimitiveStopReason.TARGET_REACHED
            else self._status_for_terminal(_terminal_for_odom_stop(telemetry.stop_reason, kind=active.goal.kind))
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
                terminal_reason=telemetry.stop_reason.value,
                collision_state=str(state.collision_state),
            )
        )

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

    def _finish(self, reason: str, status: ToolResultStatus, state: LiveRouteState) -> None:
        del status
        self._terminal_reason = reason
        self._last_collision_state = str(state.collision_state)

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
            RouteTerminalReason.UNSAFE_CLEARANCE.value,
        }:
            return ToolResultStatus.BLOCKED
        return ToolResultStatus.FAILED

    def _require_request(self) -> LiveRouteRequest:
        if self._request is None:
            raise RuntimeError("live route runner has not been started")
        return self._request


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
            if not runner.active:
                break
    except MissionValidationError as exc:
        if runner._request is None:
            runner._request = request
        runner.abort(_normalize_exception_terminal(exc), states[0])
    return runner.manifest()


def _normalize_exception_terminal(exc: Exception) -> str:
    message = str(exc)
    if "stale_scan" in message:
        return RouteTerminalReason.STALE_SCAN.value
    if "missing_scan" in message:
        return RouteTerminalReason.MISSING_SCAN.value
    if "unsafe_clearance" in message:
        return RouteTerminalReason.UNSAFE_CLEARANCE.value
    return RouteTerminalReason.INVALID_ROUTE.value


def _collision_veto(state: str) -> bool:
    normalized = str(state).upper()
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
