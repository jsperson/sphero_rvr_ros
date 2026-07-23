"""Differential-drive odometry helpers kept free of ROS imports for testing."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Optional, Tuple

from sphero_rvr_core.responses import EncoderCounts
from .collision_stop import TwistCommand

COVARIANCE_SIZE = 36
SIGNED_INT32_MODULUS = 2**32


@dataclass(frozen=True)
class DifferentialOdomConfig:
    counts_per_meter: float
    wheel_track_m: float
    frame_id: str = "odom"
    child_frame_id: str = "base_link"
    pose_xy_covariance: float = 0.05
    pose_yaw_covariance: float = 0.25
    twist_linear_covariance: float = 0.10
    twist_angular_covariance: float = 0.50
    encoder_count_modulus: Optional[int] = SIGNED_INT32_MODULUS
    source: str = "encoder_counts"
    quality_note: str = (
        "Open-loop differential encoder odometry; tune counts_per_meter and "
        "wheel_track_m on hardware before trusting SLAM quality."
    )

    def __post_init__(self) -> None:
        if self.counts_per_meter <= 0:
            raise ValueError("counts_per_meter must be positive")
        if self.wheel_track_m <= 0:
            raise ValueError("wheel_track_m must be positive")
        for name in (
            "pose_xy_covariance",
            "pose_yaw_covariance",
            "twist_linear_covariance",
            "twist_angular_covariance",
        ):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be non-negative")
        if self.encoder_count_modulus is not None and self.encoder_count_modulus <= 0:
            raise ValueError("encoder_count_modulus must be positive when set")


@dataclass(frozen=True)
class OdomSample:
    stamp: float
    x: float
    y: float
    yaw: float
    linear_mps: float
    angular_rad_s: float
    frame_id: str
    child_frame_id: str
    pose_covariance: Tuple[float, ...] = field(default_factory=lambda: (0.0,) * COVARIANCE_SIZE)
    twist_covariance: Tuple[float, ...] = field(default_factory=lambda: (0.0,) * COVARIANCE_SIZE)
    source: str = "encoder_counts"
    quality_note: str = ""


class MotionPrimitiveKind(str, Enum):
    MOVE_DISTANCE = "move_distance"
    TURN_ANGLE = "turn_angle"


class MotionPrimitiveStopReason(str, Enum):
    RUNNING = "running"
    TARGET_REACHED = "target_reached"
    STALE_ODOM = "stale_odom"
    STALL = "stall"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"
    STOP = "stop"
    ESTOP = "estop"
    COLLISION_VETO = "collision_veto"


@dataclass(frozen=True)
class OdomMotionState:
    stamp: float
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class MotionPrimitiveConfig:
    distance_tolerance_m: float = 0.01
    angle_tolerance_rad: float = math.radians(2.0)
    max_turn_speed_rad_s: float = 0.25
    heading_kp: float = 1.5
    max_heading_correction_rad_s: float = 0.6
    max_sample_age_s: float = 0.30
    stall_timeout_s: float = 0.75
    startup_grace_s: float = 1.50
    min_progress_m: float = 0.015
    min_angle_progress_rad: float = math.radians(2.0)

    def __post_init__(self) -> None:
        for name in (
            "distance_tolerance_m",
            "angle_tolerance_rad",
            "max_turn_speed_rad_s",
            "heading_kp",
            "max_heading_correction_rad_s",
            "max_sample_age_s",
            "stall_timeout_s",
            "startup_grace_s",
            "min_progress_m",
            "min_angle_progress_rad",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")


@dataclass(frozen=True)
class MotionPrimitiveGoal:
    kind: MotionPrimitiveKind
    target: float
    speed: float
    timeout_s: float

    def __post_init__(self) -> None:
        if isinstance(self.kind, str):
            object.__setattr__(self, "kind", MotionPrimitiveKind(self.kind))
        if not math.isfinite(float(self.target)) or float(self.target) == 0.0:
            raise ValueError("target must be non-zero and finite")
        if not math.isfinite(float(self.speed)) or float(self.speed) <= 0.0:
            raise ValueError("speed must be positive and finite")
        if not math.isfinite(float(self.timeout_s)) or float(self.timeout_s) <= 0.0:
            raise ValueError("timeout_s must be positive and finite")

    @staticmethod
    def move_distance(*, distance_m: float, speed_mps: float, timeout_s: float) -> "MotionPrimitiveGoal":
        return MotionPrimitiveGoal(MotionPrimitiveKind.MOVE_DISTANCE, float(distance_m), float(speed_mps), float(timeout_s))

    @staticmethod
    def turn_angle(*, angle_rad: float, angular_speed_rad_s: float, timeout_s: float) -> "MotionPrimitiveGoal":
        return MotionPrimitiveGoal(MotionPrimitiveKind.TURN_ANGLE, float(angle_rad), float(angular_speed_rad_s), float(timeout_s))


@dataclass(frozen=True)
class MotionPrimitiveTelemetry:
    kind: MotionPrimitiveKind
    command: TwistCommand
    measured_distance_m: float
    measured_angle_rad: float
    stop_reason: MotionPrimitiveStopReason
    health: str = "ok"


@dataclass
class _PrimitiveState:
    goal: MotionPrimitiveGoal
    started_at: float
    start: OdomMotionState
    last_state: OdomMotionState
    last_progress: float = 0.0
    last_progress_at: float = 0.0
    stopped_reason: Optional[MotionPrimitiveStopReason] = None


class MotionPrimitiveController:
    """ROS-free measured-odometry controller for typed route primitives."""

    def __init__(self, config: MotionPrimitiveConfig):
        self.config = config
        self._state: Optional[_PrimitiveState] = None

    def start(self, goal: MotionPrimitiveGoal, state: OdomMotionState) -> MotionPrimitiveTelemetry:
        self._state = _PrimitiveState(goal, float(state.stamp), state, state, last_progress_at=float(state.stamp))
        return self._telemetry(state, TwistCommand(), MotionPrimitiveStopReason.RUNNING)

    def update(
        self,
        state: OdomMotionState,
        *,
        now: Optional[float] = None,
        cancel: bool = False,
        stop: bool = False,
        estop: bool = False,
        collision_veto: bool = False,
    ) -> MotionPrimitiveTelemetry:
        current = self._require_state()
        if current.stopped_reason is not None:
            return self._telemetry(state, TwistCommand(), current.stopped_reason, health=current.stopped_reason.value)
        sample_time = float(state.stamp)
        wall_time = sample_time if now is None else float(now)
        if estop:
            return self._latch(MotionPrimitiveStopReason.ESTOP, state)
        if stop:
            return self._latch(MotionPrimitiveStopReason.STOP, state)
        if collision_veto:
            return self._latch(MotionPrimitiveStopReason.COLLISION_VETO, state)
        if cancel:
            return self._latch(MotionPrimitiveStopReason.CANCELLED, state)
        if wall_time - sample_time > self.config.max_sample_age_s:
            return self._latch(MotionPrimitiveStopReason.STALE_ODOM, state)
        if sample_time - current.started_at > current.goal.timeout_s:
            return self._latch(MotionPrimitiveStopReason.TIMEOUT, state)

        signed_progress = self._signed_progress(state)
        progress = max(0.0, signed_progress if current.goal.target >= 0.0 else -signed_progress)
        if self._reached(progress):
            return self._latch(MotionPrimitiveStopReason.TARGET_REACHED, state)
        min_progress = self.config.min_progress_m if current.goal.kind is MotionPrimitiveKind.MOVE_DISTANCE else self.config.min_angle_progress_rad
        if progress - current.last_progress >= min_progress:
            current.last_progress = progress
            current.last_progress_at = sample_time
        elif sample_time - current.started_at >= self.config.startup_grace_s and sample_time - current.last_progress_at >= self.config.stall_timeout_s:
            return self._latch(MotionPrimitiveStopReason.STALL, state)

        command = self._command(state)
        current.last_state = state
        return self._telemetry(state, command, MotionPrimitiveStopReason.RUNNING)

    def _command(self, state: OdomMotionState) -> TwistCommand:
        current = self._require_state()
        sign = 1.0 if current.goal.target >= 0.0 else -1.0
        if current.goal.kind is MotionPrimitiveKind.TURN_ANGLE:
            speed = min(current.goal.speed, self.config.max_turn_speed_rad_s)
            return TwistCommand(linear_x=0.0, angular_z=sign * speed)
        heading_error = normalize_angle(float(state.yaw_rad) - float(current.start.yaw_rad))
        correction = max(
            -self.config.max_heading_correction_rad_s,
            min(self.config.max_heading_correction_rad_s, -self.config.heading_kp * heading_error),
        )
        return TwistCommand(linear_x=sign * current.goal.speed, angular_z=correction)

    def _signed_progress(self, state: OdomMotionState) -> float:
        current = self._require_state()
        if current.goal.kind is MotionPrimitiveKind.TURN_ANGLE:
            return normalize_angle(float(state.yaw_rad) - float(current.start.yaw_rad))
        dx = float(state.x_m) - float(current.start.x_m)
        dy = float(state.y_m) - float(current.start.y_m)
        heading = float(current.start.yaw_rad)
        return dx * math.cos(heading) + dy * math.sin(heading)

    def _progress(self, state: OdomMotionState) -> float:
        current = self._require_state()
        signed = self._signed_progress(state)
        return max(0.0, signed if current.goal.target >= 0.0 else -signed)

    def _reached(self, progress: float) -> bool:
        current = self._require_state()
        tolerance = self.config.distance_tolerance_m if current.goal.kind is MotionPrimitiveKind.MOVE_DISTANCE else self.config.angle_tolerance_rad
        return abs(float(current.goal.target)) - progress <= tolerance

    def _latch(self, reason: MotionPrimitiveStopReason, state: OdomMotionState) -> MotionPrimitiveTelemetry:
        current = self._require_state()
        current.stopped_reason = reason
        current.last_state = state
        return self._telemetry(state, TwistCommand(), reason, health=reason.value)

    def _telemetry(
        self,
        state: OdomMotionState,
        command: TwistCommand,
        reason: MotionPrimitiveStopReason,
        *,
        health: str = "ok",
    ) -> MotionPrimitiveTelemetry:
        current = self._require_state()
        progress = self._progress(state)
        distance = progress if current.goal.kind is MotionPrimitiveKind.MOVE_DISTANCE else 0.0
        angle = (progress if current.goal.target >= 0.0 else -progress) if current.goal.kind is MotionPrimitiveKind.TURN_ANGLE else 0.0
        return MotionPrimitiveTelemetry(current.goal.kind, command, distance, angle, reason, health)

    def _require_state(self) -> _PrimitiveState:
        if self._state is None:
            raise RuntimeError("motion primitive controller has not been started")
        return self._state


def normalize_angle(angle: float) -> float:
    """Normalize radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def encoder_delta(current: int, previous: int, modulus: Optional[int] = SIGNED_INT32_MODULUS) -> int:
    """Return the shortest signed delta between successive encoder readings.

    RVR encoder counts are exposed as signed 32-bit integers. If a wheel count
    crosses the signed boundary between polls, subtracting the raw integers can
    produce a huge bogus jump. With a modulus, this unwraps the delta into the
    shortest equivalent step. Polls should still be frequent enough that a real
    movement cannot exceed half the modulus between samples, which is not a
    practical limitation for a small tracked robot.
    """
    raw = int(current) - int(previous)
    if modulus is None:
        return raw
    half = modulus // 2
    if raw > half:
        raw -= modulus
    elif raw < -half:
        raw += modulus
    return raw


def planar_pose_covariance(xy: float, yaw: float) -> Tuple[float, ...]:
    covariance = [0.0] * COVARIANCE_SIZE
    covariance[0] = float(xy)
    covariance[7] = float(xy)
    covariance[35] = float(yaw)
    return tuple(covariance)


def planar_twist_covariance(linear: float, angular: float) -> Tuple[float, ...]:
    covariance = [0.0] * COVARIANCE_SIZE
    covariance[0] = float(linear)
    covariance[35] = float(angular)
    return tuple(covariance)


class DifferentialOdomTracker:
    """Integrate encoder count deltas into a simple planar odometry estimate.

    This is intentionally conservative: it only uses typed encoder counts already
    exposed by the core driver. The estimate is useful for low-speed mapping
    experiments and slam_toolbox scan matching, not dead-reckoning wizardry.
    """

    def __init__(self, config: DifferentialOdomConfig):
        self.config = config
        self._previous_counts: Optional[EncoderCounts] = None
        self._previous_stamp: Optional[float] = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def reset(self) -> None:
        self._previous_counts = None
        self._previous_stamp = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def update(self, counts: EncoderCounts, stamp: float) -> Optional[OdomSample]:
        if self._previous_counts is None or self._previous_stamp is None:
            self._previous_counts = counts
            self._previous_stamp = stamp
            return None

        dt = stamp - self._previous_stamp
        if dt <= 0:
            self._previous_counts = counts
            self._previous_stamp = stamp
            return None

        left_delta = encoder_delta(
            counts.left,
            self._previous_counts.left,
            self.config.encoder_count_modulus,
        )
        right_delta = encoder_delta(
            counts.right,
            self._previous_counts.right,
            self.config.encoder_count_modulus,
        )
        left_m = left_delta / self.config.counts_per_meter
        right_m = right_delta / self.config.counts_per_meter
        distance = (left_m + right_m) / 2.0
        delta_yaw = (right_m - left_m) / self.config.wheel_track_m
        mid_yaw = self._yaw + (delta_yaw / 2.0)

        self._x += distance * math.cos(mid_yaw)
        self._y += distance * math.sin(mid_yaw)
        self._yaw = normalize_angle(self._yaw + delta_yaw)

        self._previous_counts = counts
        self._previous_stamp = stamp
        return OdomSample(
            stamp=stamp,
            x=self._x,
            y=self._y,
            yaw=self._yaw,
            linear_mps=distance / dt,
            angular_rad_s=delta_yaw / dt,
            frame_id=self.config.frame_id,
            child_frame_id=self.config.child_frame_id,
            pose_covariance=planar_pose_covariance(
                self.config.pose_xy_covariance,
                self.config.pose_yaw_covariance,
            ),
            twist_covariance=planar_twist_covariance(
                self.config.twist_linear_covariance,
                self.config.twist_angular_covariance,
            ),
            source=self.config.source,
            quality_note=self.config.quality_note,
        )
