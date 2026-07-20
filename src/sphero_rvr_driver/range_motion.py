"""ROS-free closed-loop lidar range motion controller.

The ROS wrapper owns subscriptions/services/actions.  This module is intentionally
pure Python so target tracking, range-rate estimation, and fail-closed motion
control can be tested without ROS, hardware, or a live graph.
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque, Optional, Sequence

from .collision_stop import TwistCommand


class MotionDirection(str, Enum):
    FORWARD = "forward"
    BACKWARD = "backward"

    @property
    def sign(self) -> float:
        return 1.0 if self is MotionDirection.FORWARD else -1.0


class MotionMode(str, Enum):
    APPROACH = "approach"
    RETREAT = "retreat"


class StopReason(str, Enum):
    RUNNING = "running"
    TARGET_REACHED = "target_reached"
    STALE_SENSOR = "stale_sensor"
    TARGET_LOST = "target_lost"
    TARGET_JUMP = "target_jump"
    UNSAFE_CLEARANCE = "unsafe_clearance"
    STALL = "stall"
    TIMEOUT = "timeout"
    MAX_DISPLACEMENT = "max_displacement"
    ODOM_DISAGREEMENT = "odom_disagreement"
    OPERATOR_STOP = "operator_stop"
    ESTOP = "estop"
    DRIVER_FAULT = "driver_fault"
    CLEANUP_UNCERTAIN = "cleanup_uncertain"


@dataclass(frozen=True)
class MotionGoal:
    direction: MotionDirection
    mode: MotionMode
    target_clearance_m: float
    max_measured_displacement_m: Optional[float] = None
    timeout_s: Optional[float] = None

    def __post_init__(self) -> None:
        if isinstance(self.direction, str):
            object.__setattr__(self, "direction", MotionDirection(self.direction))
        if isinstance(self.mode, str):
            object.__setattr__(self, "mode", MotionMode(self.mode))
        if not math.isfinite(self.target_clearance_m) or self.target_clearance_m <= 0.0:
            raise ValueError("target_clearance_m must be positive and finite")
        if self.max_measured_displacement_m is not None and self.max_measured_displacement_m <= 0.0:
            raise ValueError("max_measured_displacement_m must be positive when set")
        if self.timeout_s is not None and self.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive when set")


@dataclass(frozen=True)
class RangeMotionConfig:
    max_speed_mps: float = 0.20
    min_speed_mps: float = 0.04
    acceleration_mps2: float = 0.40
    slowdown_distance_m: float = 0.20
    target_tolerance_m: float = 0.01
    max_sample_age_s: float = 0.30
    rate_window_s: float = 0.75
    min_front_clearance_m: float = 0.05
    min_rear_clearance_m: float = 0.05
    min_side_clearance_m: float = 0.05
    max_range_jump_m: float = 0.35
    stall_timeout_s: float = 0.75
    min_progress_m: float = 0.015
    startup_grace_s: float = 0.0
    min_odom_progress_m: float = 0.005
    max_odom_lidar_disagreement_m: float = 0.12

    def __post_init__(self) -> None:
        for name in (
            "max_speed_mps",
            "min_speed_mps",
            "acceleration_mps2",
            "slowdown_distance_m",
            "target_tolerance_m",
            "max_sample_age_s",
            "rate_window_s",
            "stall_timeout_s",
            "min_progress_m",
            "min_odom_progress_m",
            "max_odom_lidar_disagreement_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if not math.isfinite(float(self.startup_grace_s)) or self.startup_grace_s < 0.0:
            raise ValueError("startup_grace_s must be finite and non-negative")
        if self.min_speed_mps > self.max_speed_mps:
            raise ValueError("min_speed_mps must not exceed max_speed_mps")


@dataclass(frozen=True)
class RangeMotionSample:
    stamp: float
    target_clearance_m: Optional[float]
    front_clearance_m: Optional[float]
    rear_clearance_m: Optional[float]
    left_clearance_m: Optional[float]
    right_clearance_m: Optional[float]
    odom_displacement_m: Optional[float] = None
    target_candidates: Sequence[AngularRangeCandidate] = ()


@dataclass(frozen=True)
class SurfaceTrack:
    clearance_m: Optional[float]
    confidence: float
    associated_count: int
    considered_count: int


@dataclass(frozen=True)
class AngularRangeCandidate:
    range_m: float
    angle_rad: float


@dataclass(frozen=True)
class TargetAssociationState:
    range_m: Optional[float]
    angle_rad: Optional[float]
    confidence: float
    associated_count: int = 0
    considered_count: int = 0


@dataclass(frozen=True)
class RangeMotionTelemetry:
    command: TwistCommand
    requested_velocity_mps: float
    forwarded_velocity_mps: float
    lidar_range_rate_mps: float
    odom_velocity_mps: Optional[float]
    measured_displacement_m: float
    confidence: float
    stop_reason: StopReason
    target_clearance_m: Optional[float]
    current_clearance_m: Optional[float]
    health: str = "ok"


@dataclass
class _ControllerState:
    goal: MotionGoal
    started_at: float
    start_clearance_m: float
    start_odom_m: Optional[float]
    last_sample: RangeMotionSample
    last_command_mps: float = 0.0
    last_progress_mark_m: float = 0.0
    last_progress_at: float = 0.0
    stopped_reason: Optional[StopReason] = None
    range_window: Deque[tuple[float, float]] = field(default_factory=deque)
    target_track: Optional[TargetAssociationState] = None


class RangeMotionController:
    """Closed-loop target-clearance controller driven by measured lidar progress."""

    def __init__(self, config: RangeMotionConfig):
        self.config = config
        self._state: Optional[_ControllerState] = None

    def start(self, goal: MotionGoal, first_sample: RangeMotionSample) -> RangeMotionTelemetry:
        clearance, confidence, target_track = self._initial_clearance(first_sample)
        if clearance is None:
            self._state = None
            return self._stop(StopReason.TARGET_LOST, first_sample, confidence=0.0, health="target_lost")
        state = _ControllerState(
            goal=goal,
            started_at=float(first_sample.stamp),
            start_clearance_m=clearance,
            start_odom_m=_finite_optional(first_sample.odom_displacement_m),
            last_sample=first_sample,
            last_progress_at=float(first_sample.stamp),
            target_track=target_track,
        )
        state.range_window.append((float(first_sample.stamp), clearance))
        self._state = state
        return self._telemetry(first_sample, 0.0, StopReason.RUNNING, confidence=confidence)

    def update(
        self,
        sample: RangeMotionSample,
        *,
        now: Optional[float] = None,
        operator_stop: bool = False,
        estop: bool = False,
        driver_fault: bool = False,
        cleanup_uncertain: bool = False,
    ) -> RangeMotionTelemetry:
        if self._state is None:
            return self._stop(StopReason.TARGET_LOST, sample, confidence=0.0, health="not_started")
        state = self._state
        if state.stopped_reason is not None:
            return self._stop(state.stopped_reason, sample, confidence=0.0, health=state.stopped_reason.value)

        sample_time = float(sample.stamp)
        wall_time = sample_time if now is None else float(now)
        if estop:
            return self._latch_stop(StopReason.ESTOP, sample, confidence=0.0)
        if operator_stop:
            return self._latch_stop(StopReason.OPERATOR_STOP, sample, confidence=0.0)
        if driver_fault:
            return self._latch_stop(StopReason.DRIVER_FAULT, sample, confidence=0.0)
        if cleanup_uncertain:
            return self._latch_stop(StopReason.CLEANUP_UNCERTAIN, sample, confidence=0.0)
        if wall_time - sample_time > self.config.max_sample_age_s:
            return self._latch_stop(StopReason.STALE_SENSOR, sample, confidence=0.0)
        if state.goal.timeout_s is not None and sample_time - state.started_at > state.goal.timeout_s:
            return self._latch_stop(StopReason.TIMEOUT, sample, confidence=0.0)

        clearance, confidence = self._associated_clearance(sample)
        if clearance is None:
            return self._latch_stop(StopReason.TARGET_LOST, sample, confidence=0.0)
        last_clearance = self._last_tracked_clearance()
        if last_clearance is not None and abs(clearance - last_clearance) > self.config.max_range_jump_m:
            return self._latch_stop(StopReason.TARGET_JUMP, sample, confidence=0.0)
        if self._unsafe_clearance(sample):
            return self._latch_stop(StopReason.UNSAFE_CLEARANCE, sample, confidence=0.0)

        state.range_window.append((sample_time, clearance))
        while state.range_window and sample_time - state.range_window[0][0] > self.config.rate_window_s:
            state.range_window.popleft()

        measured_displacement = self._measured_displacement(sample)
        if state.goal.max_measured_displacement_m is not None and measured_displacement >= state.goal.max_measured_displacement_m:
            return self._latch_stop(StopReason.MAX_DISPLACEMENT, sample, confidence=0.0)
        if self._odom_disagrees(sample):
            return self._latch_stop(StopReason.ODOM_DISAGREEMENT, sample, confidence=0.2)

        progress = self._progress_for_stall(sample, clearance)
        min_progress = self.config.min_odom_progress_m if self._has_odom_progress(sample) else self.config.min_progress_m
        if progress - state.last_progress_mark_m >= min_progress:
            state.last_progress_mark_m = progress
            state.last_progress_at = sample_time
        elif (
            sample_time - state.started_at >= self.config.startup_grace_s
            and sample_time - state.last_progress_at >= self.config.stall_timeout_s
        ):
            return self._latch_stop(StopReason.STALL, sample, confidence=0.2)

        if self._target_reached(clearance):
            return self._latch_stop(StopReason.TARGET_REACHED, sample, confidence=1.0)

        requested = self._requested_speed(sample, clearance)
        state.last_sample = sample
        state.last_command_mps = requested
        return self._telemetry(sample, requested, StopReason.RUNNING, confidence=confidence)

    def _requested_speed(self, sample: RangeMotionSample, clearance: float) -> float:
        state = self._require_state()
        remaining = self._remaining(clearance)
        slow_scale = min(1.0, max(0.0, remaining / self.config.slowdown_distance_m))
        desired_mag = max(self.config.min_speed_mps, self.config.max_speed_mps * slow_scale)
        dt = max(0.0, float(sample.stamp) - float(state.last_sample.stamp))
        max_delta = self.config.acceleration_mps2 * dt
        previous_mag = abs(state.last_command_mps)
        next_mag = min(desired_mag, previous_mag + max_delta, self.config.max_speed_mps)
        return state.goal.direction.sign * next_mag

    def _initial_clearance(
        self, sample: RangeMotionSample
    ) -> tuple[Optional[float], float, Optional[TargetAssociationState]]:
        if sample.target_candidates:
            track = track_target_cluster(
                previous=None,
                candidates=sample.target_candidates,
                range_gate_m=self.config.max_range_jump_m,
                angle_gate_rad=0.20,
            )
            return track.range_m, track.confidence, track
        return _finite_optional(sample.target_clearance_m), 1.0, None

    def _associated_clearance(self, sample: RangeMotionSample) -> tuple[Optional[float], float]:
        state = self._require_state()
        if sample.target_candidates:
            track = track_target_cluster(
                previous=state.target_track,
                candidates=sample.target_candidates,
                range_gate_m=self.config.max_range_jump_m,
                angle_gate_rad=0.20,
            )
            if track.range_m is not None:
                state.target_track = track
            return track.range_m, track.confidence
        return _finite_optional(sample.target_clearance_m), 1.0

    def _last_tracked_clearance(self) -> Optional[float]:
        state = self._require_state()
        if state.target_track is not None and state.target_track.range_m is not None:
            return state.target_track.range_m
        return _finite_optional(state.last_sample.target_clearance_m)

    def _target_reached(self, clearance: float) -> bool:
        return self._remaining(clearance) <= self.config.target_tolerance_m

    def _remaining(self, clearance: float) -> float:
        goal = self._require_state().goal
        if goal.mode is MotionMode.APPROACH:
            return clearance - goal.target_clearance_m
        return goal.target_clearance_m - clearance

    def _lidar_progress(self, clearance: float) -> float:
        state = self._require_state()
        if state.goal.mode is MotionMode.APPROACH:
            return max(0.0, state.start_clearance_m - clearance)
        return max(0.0, clearance - state.start_clearance_m)

    def _progress_for_stall(self, sample: RangeMotionSample, clearance: float) -> float:
        odom_progress = self._odom_progress(sample)
        if odom_progress is not None:
            return odom_progress
        return self._lidar_progress(clearance)

    def _has_odom_progress(self, sample: RangeMotionSample) -> bool:
        return self._odom_progress(sample) is not None

    def _odom_progress(self, sample: RangeMotionSample) -> Optional[float]:
        state = self._require_state()
        odom = _finite_optional(sample.odom_displacement_m)
        if odom is None or state.start_odom_m is None:
            return None
        return abs(odom - state.start_odom_m)

    def _measured_displacement(self, sample: RangeMotionSample) -> float:
        state = self._require_state()
        odom = _finite_optional(sample.odom_displacement_m)
        if odom is not None and state.start_odom_m is not None:
            return abs(odom - state.start_odom_m)
        clearance = _finite_optional(sample.target_clearance_m)
        if clearance is None:
            return 0.0
        return self._lidar_progress(clearance)

    def _odom_disagrees(self, sample: RangeMotionSample) -> bool:
        state = self._require_state()
        odom = _finite_optional(sample.odom_displacement_m)
        clearance = _finite_optional(sample.target_clearance_m)
        if odom is None or state.start_odom_m is None or clearance is None:
            return False
        odom_delta = abs(odom - state.start_odom_m)
        lidar_delta = self._lidar_progress(clearance)
        return abs(odom_delta - lidar_delta) > self.config.max_odom_lidar_disagreement_m

    def _unsafe_clearance(self, sample: RangeMotionSample) -> bool:
        return (
            _below(sample.front_clearance_m, self.config.min_front_clearance_m)
            or _below(sample.rear_clearance_m, self.config.min_rear_clearance_m)
            or _below(sample.left_clearance_m, self.config.min_side_clearance_m)
            or _below(sample.right_clearance_m, self.config.min_side_clearance_m)
        )

    def _latch_stop(self, reason: StopReason, sample: RangeMotionSample, *, confidence: float) -> RangeMotionTelemetry:
        state = self._require_state()
        state.stopped_reason = reason
        state.last_sample = sample
        state.last_command_mps = 0.0
        return self._telemetry(sample, 0.0, reason, confidence=confidence, health=reason.value)

    def _stop(self, reason: StopReason, sample: RangeMotionSample, *, confidence: float, health: str) -> RangeMotionTelemetry:
        return RangeMotionTelemetry(
            command=TwistCommand(),
            requested_velocity_mps=0.0,
            forwarded_velocity_mps=0.0,
            lidar_range_rate_mps=0.0,
            odom_velocity_mps=None,
            measured_displacement_m=0.0,
            confidence=confidence,
            stop_reason=reason,
            target_clearance_m=None,
            current_clearance_m=_finite_optional(sample.target_clearance_m),
            health=health,
        )

    def _telemetry(
        self,
        sample: RangeMotionSample,
        velocity_mps: float,
        reason: StopReason,
        *,
        confidence: float,
        health: str = "ok",
    ) -> RangeMotionTelemetry:
        state = self._require_state()
        return RangeMotionTelemetry(
            command=TwistCommand(linear_x=velocity_mps, angular_z=0.0),
            requested_velocity_mps=velocity_mps,
            forwarded_velocity_mps=velocity_mps if reason is StopReason.RUNNING else 0.0,
            lidar_range_rate_mps=_range_rate(tuple(state.range_window)),
            odom_velocity_mps=_odom_velocity(state.last_sample, sample),
            measured_displacement_m=self._measured_displacement(sample),
            confidence=confidence,
            stop_reason=reason,
            target_clearance_m=state.goal.target_clearance_m,
            current_clearance_m=self._telemetry_clearance(sample),
            health=health,
        )

    def _telemetry_clearance(self, sample: RangeMotionSample) -> Optional[float]:
        clearance = _finite_optional(sample.target_clearance_m)
        if clearance is not None:
            return clearance
        state = self._require_state()
        return None if state.target_track is None else state.target_track.range_m

    def _require_state(self) -> _ControllerState:
        if self._state is None:
            raise RuntimeError("range motion controller has not been started")
        return self._state


def track_stable_surface(
    *,
    previous_clearance_m: Optional[float],
    candidate_clearances_m: Sequence[float],
    association_gate_m: float,
) -> SurfaceTrack:
    valid = sorted(value for value in (_finite_optional(v) for v in candidate_clearances_m) if value is not None)
    if not valid:
        return SurfaceTrack(None, 0.0, 0, len(candidate_clearances_m))
    if previous_clearance_m is None:
        associated = _nearest_compact_cluster(valid, association_gate_m)
    else:
        associated = [value for value in valid if abs(value - previous_clearance_m) <= association_gate_m]
    if not associated:
        return SurfaceTrack(None, 0.0, 0, len(valid))
    clearance = float(statistics.median(associated))
    density = len(associated) / max(1, len(valid))
    spread = max(associated) - min(associated) if len(associated) > 1 else 0.0
    compactness = 1.0 - min(1.0, spread / max(association_gate_m, 1e-9))
    confidence = max(0.0, min(1.0, density + 0.4 * compactness))
    return SurfaceTrack(clearance, confidence, len(associated), len(valid))


def track_target_cluster(
    *,
    previous: Optional[TargetAssociationState],
    candidates: Sequence[AngularRangeCandidate],
    range_gate_m: float,
    angle_gate_rad: float,
) -> TargetAssociationState:
    valid = [
        AngularRangeCandidate(float(candidate.range_m), float(candidate.angle_rad))
        for candidate in candidates
        if math.isfinite(float(candidate.range_m)) and math.isfinite(float(candidate.angle_rad)) and float(candidate.range_m) > 0.0
    ]
    if not valid:
        return TargetAssociationState(None, None, 0.0, 0, len(candidates))
    if previous is not None and previous.range_m is not None and previous.angle_rad is not None:
        associated = [
            candidate
            for candidate in valid
            if abs(candidate.range_m - previous.range_m) <= range_gate_m
            and abs(_angle_delta(candidate.angle_rad, previous.angle_rad)) <= angle_gate_rad
        ]
    else:
        associated = _nearest_angular_cluster(valid, range_gate_m, angle_gate_rad)
    if not associated:
        return TargetAssociationState(None, None, 0.0, 0, len(valid))
    ranges = [candidate.range_m for candidate in associated]
    angles = [candidate.angle_rad for candidate in associated]
    range_m = float(statistics.median(ranges))
    angle_rad = math.atan2(
        sum(math.sin(angle) for angle in angles) / len(angles),
        sum(math.cos(angle) for angle in angles) / len(angles),
    )
    range_spread = max(ranges) - min(ranges) if len(ranges) > 1 else 0.0
    angle_spread = max(abs(_angle_delta(angle, angle_rad)) for angle in angles) if len(angles) > 1 else 0.0
    density = len(associated) / max(1, len(valid))
    compactness = 1.0 - min(1.0, max(range_spread / max(range_gate_m, 1e-9), angle_spread / max(angle_gate_rad, 1e-9)))
    confidence = max(0.0, min(1.0, density + 0.4 * compactness))
    return TargetAssociationState(range_m, angle_rad, confidence, len(associated), len(valid))


def _nearest_angular_cluster(
    valid: Sequence[AngularRangeCandidate], range_gate_m: float, angle_gate_rad: float
) -> list[AngularRangeCandidate]:
    ordered = sorted(valid, key=lambda candidate: candidate.range_m)
    best: list[AngularRangeCandidate] = []
    for seed in ordered:
        cluster = [
            candidate
            for candidate in ordered
            if abs(candidate.range_m - seed.range_m) <= range_gate_m
            and abs(_angle_delta(candidate.angle_rad, seed.angle_rad)) <= angle_gate_rad
        ]
        if len(cluster) > len(best):
            best = cluster
    return best or ([ordered[0]] if ordered else [])


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _nearest_compact_cluster(valid_clearances_m: Sequence[float], association_gate_m: float) -> list[float]:
    clusters: list[list[float]] = []
    for value in valid_clearances_m:
        if not clusters or value - clusters[-1][-1] > association_gate_m:
            clusters.append([value])
        else:
            clusters[-1].append(value)
    multi_sample_clusters = [cluster for cluster in clusters if len(cluster) > 1]
    if multi_sample_clusters:
        return multi_sample_clusters[0]
    return [valid_clearances_m[0]]


def _range_rate(window: Sequence[tuple[float, float]]) -> float:
    if len(window) < 2:
        return 0.0
    first_t, first_r = window[0]
    last_t, last_r = window[-1]
    dt = last_t - first_t
    if dt <= 0.0:
        return 0.0
    return (last_r - first_r) / dt


def _odom_velocity(previous: RangeMotionSample, current: RangeMotionSample) -> Optional[float]:
    previous_odom = _finite_optional(previous.odom_displacement_m)
    current_odom = _finite_optional(current.odom_displacement_m)
    if previous_odom is None or current_odom is None:
        return None
    dt = float(current.stamp) - float(previous.stamp)
    if dt <= 0.0:
        return None
    return (current_odom - previous_odom) / dt


def _finite_optional(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _below(value: Optional[float], threshold: float) -> bool:
    finite = _finite_optional(value)
    return finite is None or finite < threshold
