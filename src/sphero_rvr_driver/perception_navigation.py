"""Replay-first perception-guided navigation contracts.

This module is deliberately ROS-free and has no physical execution adapter.  It
turns authoritative lidar-localization observations into bounded *requests* for
the next motion horizon.  A future ROS wrapper must route those requests through
the existing deterministic controller and collision supervisor; this module
cannot publish Twist or motor commands.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import json
import math
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence


REPLAY_SCHEMA = "sphero_rvr.perception_navigation_replay.v1"
RESULT_SCHEMA = "sphero_rvr.perception_navigation_result.v1"


class LocalizationState(str, Enum):
    VALID = "valid"
    DEGRADED = "degraded"
    STALE = "stale"
    LOST = "lost"


class HorizonKind(str, Enum):
    TRANSLATE = "translate"
    ROTATE = "rotate"


class NavigationOutcome(str, Enum):
    RUNNING = "running"
    REACHED = "reached"
    PARTIAL = "partial"
    BLOCKED = "blocked"
    LOCALIZATION_LOST = "localization_lost"
    PROGRESS_FAILED = "progress_failed"
    CANCELLED = "cancelled"
    STOPPED = "stopped"
    ESTOPPED = "estopped"


@dataclass(frozen=True)
class Pose2D:
    stamp_s: float
    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        for name in ("stamp_s", "x_m", "y_m", "yaw_rad"):
            _finite(getattr(self, name), name)
        if not str(self.frame_id).strip():
            raise ValueError("pose frame_id is required")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "Pose2D":
        return cls(
            stamp_s=_finite(value.get("stamp_s"), "pose stamp_s"),
            frame_id=str(value.get("frame_id", "")).strip(),
            x_m=_finite(value.get("x_m"), "pose x_m"),
            y_m=_finite(value.get("y_m"), "pose y_m"),
            yaw_rad=_finite(value.get("yaw_rad"), "pose yaw_rad"),
        )

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "stamp_s": self.stamp_s,
            "frame_id": self.frame_id,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "yaw_rad": self.yaw_rad,
            "heading_deg": math.degrees(self.yaw_rad),
        }


@dataclass(frozen=True)
class LocalizationEstimate:
    """Lidar-authoritative pose with odometry retained only as comparison."""

    state: LocalizationState
    pose: Optional[Pose2D]
    source: str
    quality: float
    covariance_xy_m2: Optional[float]
    covariance_yaw_rad2: Optional[float]
    odom_translation_disagreement_m: Optional[float] = None
    odom_heading_disagreement_rad: Optional[float] = None
    detail: str = ""

    def __post_init__(self) -> None:
        if isinstance(self.state, str):
            object.__setattr__(self, "state", LocalizationState(self.state))
        if not str(self.source).strip():
            raise ValueError("localization source is required")
        quality = _finite(self.quality, "localization quality")
        if not 0.0 <= quality <= 1.0:
            raise ValueError("localization quality must be between zero and one")
        for name in (
            "covariance_xy_m2",
            "covariance_yaw_rad2",
            "odom_translation_disagreement_m",
            "odom_heading_disagreement_rad",
        ):
            value = getattr(self, name)
            if value is not None and _finite(value, name) < 0.0:
                raise ValueError(f"{name} must be non-negative")
        if self.state in {LocalizationState.VALID, LocalizationState.DEGRADED} and self.pose is None:
            raise ValueError("available localization requires a pose")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "LocalizationEstimate":
        pose_value = value.get("pose")
        pose = Pose2D.from_mapping(pose_value) if isinstance(pose_value, Mapping) else None
        return cls(
            state=LocalizationState(str(value.get("state", "lost")).lower()),
            pose=pose,
            source=str(value.get("source", "")).strip(),
            quality=_finite(value.get("quality", 0.0), "localization quality"),
            covariance_xy_m2=_optional_finite(value.get("covariance_xy_m2")),
            covariance_yaw_rad2=_optional_finite(value.get("covariance_yaw_rad2")),
            odom_translation_disagreement_m=_optional_finite(
                value.get("odom_translation_disagreement_m")
            ),
            odom_heading_disagreement_rad=_optional_finite(
                value.get("odom_heading_disagreement_rad")
            ),
            detail=str(value.get("detail", "")),
        )

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "pose": None if self.pose is None else self.pose.to_json_dict(),
            "source": self.source,
            "authoritative": self.source.startswith("lidar") or self.source.startswith("slam"),
            "quality": self.quality,
            "covariance_xy_m2": self.covariance_xy_m2,
            "covariance_yaw_rad2": self.covariance_yaw_rad2,
            "odom_translation_disagreement_m": self.odom_translation_disagreement_m,
            "odom_heading_disagreement_rad": self.odom_heading_disagreement_rad,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class GoalRegion:
    frame_id: str
    x_m: float
    y_m: float
    radius_m: float
    minimum_clearance_m: float
    max_runtime_s: float
    max_cumulative_translation_m: float
    heading_min_rad: Optional[float] = None
    heading_max_rad: Optional[float] = None

    def __post_init__(self) -> None:
        if not str(self.frame_id).strip():
            raise ValueError("goal frame_id is required")
        for name in ("x_m", "y_m"):
            _finite(getattr(self, name), name)
        for name in (
            "radius_m",
            "minimum_clearance_m",
            "max_runtime_s",
            "max_cumulative_translation_m",
        ):
            if _finite(getattr(self, name), name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if (self.heading_min_rad is None) != (self.heading_max_rad is None):
            raise ValueError("goal heading range requires both minimum and maximum")
        if self.heading_min_rad is not None:
            _finite(self.heading_min_rad, "heading_min_rad")
            _finite(self.heading_max_rad, "heading_max_rad")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "GoalRegion":
        return cls(
            frame_id=str(value.get("frame_id", "")).strip(),
            x_m=_finite(value.get("x_m"), "goal x_m"),
            y_m=_finite(value.get("y_m"), "goal y_m"),
            radius_m=_finite(value.get("radius_m"), "goal radius_m"),
            minimum_clearance_m=_finite(
                value.get("minimum_clearance_m"), "goal minimum_clearance_m"
            ),
            max_runtime_s=_finite(value.get("max_runtime_s"), "goal max_runtime_s"),
            max_cumulative_translation_m=_finite(
                value.get("max_cumulative_translation_m"),
                "goal max_cumulative_translation_m",
            ),
            heading_min_rad=_optional_finite(
                value.get("heading_min_rad"), signed=True
            ),
            heading_max_rad=_optional_finite(
                value.get("heading_max_rad"), signed=True
            ),
        )

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "radius_m": self.radius_m,
            "minimum_clearance_m": self.minimum_clearance_m,
            "max_runtime_s": self.max_runtime_s,
            "max_cumulative_translation_m": self.max_cumulative_translation_m,
            "heading_min_rad": self.heading_min_rad,
            "heading_max_rad": self.heading_max_rad,
        }


@dataclass(frozen=True)
class NavigationConfig:
    max_localization_age_s: float = 0.35
    minimum_localization_quality: float = 0.60
    max_covariance_xy_m2: float = 0.04
    max_covariance_yaw_rad2: float = math.radians(12.0) ** 2
    max_odom_translation_disagreement_m: float = 0.12
    max_odom_heading_disagreement_rad: float = math.radians(15.0)
    max_translation_horizon_m: float = 0.15
    max_rotation_horizon_rad: float = math.radians(15.0)
    heading_deadband_rad: float = math.radians(5.0)
    horizon_timeout_s: float = 4.0
    minimum_progress_m: float = 0.01
    minimum_track_activity_m: float = 0.005
    max_track_asymmetry_fraction: float = 0.60
    max_alternate_horizons: int = 2

    def __post_init__(self) -> None:
        for name in (
            "max_localization_age_s",
            "minimum_localization_quality",
            "max_covariance_xy_m2",
            "max_covariance_yaw_rad2",
            "max_odom_translation_disagreement_m",
            "max_odom_heading_disagreement_rad",
            "max_translation_horizon_m",
            "max_rotation_horizon_rad",
            "heading_deadband_rad",
            "horizon_timeout_s",
            "minimum_progress_m",
            "minimum_track_activity_m",
            "max_track_asymmetry_fraction",
        ):
            if _finite(getattr(self, name), name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.minimum_localization_quality > 1.0:
            raise ValueError("minimum_localization_quality must not exceed one")
        if self.max_track_asymmetry_fraction > 1.0:
            raise ValueError("max_track_asymmetry_fraction must not exceed one")
        if self.max_alternate_horizons < 0:
            raise ValueError("max_alternate_horizons must be non-negative")


@dataclass(frozen=True)
class MotionHorizon:
    horizon_id: int
    kind: HorizonKind
    distance_m: float
    angle_rad: float
    timeout_s: float
    minimum_clearance_m: float
    reason: str

    def __post_init__(self) -> None:
        if isinstance(self.kind, str):
            object.__setattr__(self, "kind", HorizonKind(self.kind))
        if self.horizon_id <= 0:
            raise ValueError("horizon_id must be positive")
        for name in (
            "distance_m",
            "angle_rad",
            "timeout_s",
            "minimum_clearance_m",
        ):
            _finite(getattr(self, name), name)
        if self.distance_m < 0.0:
            raise ValueError("horizon distance_m must be non-negative")
        if self.timeout_s <= 0.0 or self.minimum_clearance_m <= 0.0:
            raise ValueError("horizon timeout and minimum clearance must be positive")
        if self.kind is HorizonKind.TRANSLATE and (
            self.distance_m <= 0.0 or self.angle_rad != 0.0
        ):
            raise ValueError("translation horizon requires distance and zero angle")
        if self.kind is HorizonKind.ROTATE and (
            self.distance_m != 0.0 or self.angle_rad == 0.0
        ):
            raise ValueError("rotation horizon requires angle and zero distance")
        if not str(self.reason).strip():
            raise ValueError("horizon reason is required")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "horizon_id": self.horizon_id,
            "kind": self.kind.value,
            "distance_m": self.distance_m,
            "angle_rad": self.angle_rad,
            "angle_deg": math.degrees(self.angle_rad),
            "timeout_s": self.timeout_s,
            "minimum_clearance_m": self.minimum_clearance_m,
            "reason": self.reason,
            "command_role": "bounded_navigation_request",
            "motor_command": False,
        }


@dataclass(frozen=True)
class HorizonObservation:
    localization: LocalizationEstimate
    now_s: float
    left_track_delta_m: Optional[float] = None
    right_track_delta_m: Optional[float] = None
    obstacle_blocked: bool = False
    left_clearance_m: Optional[float] = None
    right_clearance_m: Optional[float] = None
    cancel: bool = False
    stop: bool = False
    estop: bool = False

    def __post_init__(self) -> None:
        _finite(self.now_s, "observation now_s")
        for name in ("left_track_delta_m", "right_track_delta_m"):
            value = getattr(self, name)
            if value is not None:
                _finite(value, name)
        for name in ("left_clearance_m", "right_clearance_m"):
            value = getattr(self, name)
            if value is not None and _finite(value, name) < 0.0:
                raise ValueError(f"{name} must be non-negative")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "HorizonObservation":
        localization = value.get("localization")
        if not isinstance(localization, Mapping):
            raise ValueError("replay observation requires localization")
        return cls(
            localization=LocalizationEstimate.from_mapping(localization),
            now_s=_finite(value.get("now_s"), "observation now_s"),
            left_track_delta_m=_optional_finite(value.get("left_track_delta_m"), signed=True),
            right_track_delta_m=_optional_finite(value.get("right_track_delta_m"), signed=True),
            obstacle_blocked=bool(value.get("obstacle_blocked", False)),
            left_clearance_m=_optional_finite(value.get("left_clearance_m")),
            right_clearance_m=_optional_finite(value.get("right_clearance_m")),
            cancel=bool(value.get("cancel", False)),
            stop=bool(value.get("stop", False)),
            estop=bool(value.get("estop", False)),
        )


@dataclass(frozen=True)
class NavigationEvent:
    sequence: int
    kind: str
    detail: str
    pose: Optional[Pose2D]
    horizon: Optional[MotionHorizon] = None

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "kind": self.kind,
            "detail": self.detail,
            "pose": None if self.pose is None else self.pose.to_json_dict(),
            "horizon": None if self.horizon is None else self.horizon.to_json_dict(),
        }


@dataclass(frozen=True)
class NavigationDecision:
    outcome: NavigationOutcome
    terminal_reason: str
    localization: LocalizationEstimate
    goal: GoalRegion
    next_horizon: Optional[MotionHorizon]
    path: tuple[Pose2D, ...]
    events: tuple[NavigationEvent, ...]
    cumulative_translation_m: float
    zero_output_required: bool

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "schema": RESULT_SCHEMA,
            "outcome": self.outcome.value,
            "terminal_reason": self.terminal_reason,
            "localization": self.localization.to_json_dict(),
            "goal": self.goal.to_json_dict(),
            "next_horizon": (
                None if self.next_horizon is None else self.next_horizon.to_json_dict()
            ),
            "path": [pose.to_json_dict() for pose in self.path],
            "events": [event.to_json_dict() for event in self.events],
            "cumulative_translation_m": self.cumulative_translation_m,
            "zero_output_required": self.zero_output_required,
            "motion_authority": False,
            "physical_execution_enabled": False,
            "command_path": [
                "bounded_navigation_request",
                "deterministic_horizon_controller",
                "collision_stop",
                "rover",
            ],
        }


@dataclass
class PerceptionGuidedNavigator:
    config: NavigationConfig = field(default_factory=NavigationConfig)
    _goal: Optional[GoalRegion] = field(default=None, init=False)
    _started_at_s: Optional[float] = field(default=None, init=False)
    _last_localization: Optional[LocalizationEstimate] = field(default=None, init=False)
    _last_horizon: Optional[MotionHorizon] = field(default=None, init=False)
    _path: list[Pose2D] = field(default_factory=list, init=False)
    _events: list[NavigationEvent] = field(default_factory=list, init=False)
    _cumulative_translation_m: float = field(default=0.0, init=False)
    _alternate_horizons: int = field(default=0, init=False)
    _horizon_sequence: int = field(default=0, init=False)
    _terminal: Optional[NavigationDecision] = field(default=None, init=False)

    def start(
        self,
        goal: GoalRegion,
        localization: LocalizationEstimate,
        *,
        now_s: float,
    ) -> NavigationDecision:
        if self._goal is not None:
            raise RuntimeError("navigator has already started")
        self._goal = goal
        self._started_at_s = _finite(now_s, "navigation start time")
        problem = self._localization_problem(localization, now_s=now_s)
        if problem:
            return self._finish(
                NavigationOutcome.LOCALIZATION_LOST,
                problem,
                localization,
            )
        self._record_pose(localization)
        self._last_localization = localization
        return self._plan(localization, reason="initial_horizon")

    def observe(self, observation: HorizonObservation) -> NavigationDecision:
        if self._goal is None or self._started_at_s is None:
            raise RuntimeError("navigator has not started")
        if self._terminal is not None:
            return self._terminal
        localization = observation.localization
        if observation.estop:
            return self._finish(NavigationOutcome.ESTOPPED, "estop", localization)
        if observation.stop:
            return self._finish(NavigationOutcome.STOPPED, "operator_stop", localization)
        if observation.cancel:
            return self._finish(NavigationOutcome.CANCELLED, "cancelled", localization)
        if observation.now_s - self._started_at_s > self._goal.max_runtime_s:
            return self._finish(NavigationOutcome.PARTIAL, "runtime_budget_exhausted", localization)
        problem = self._localization_problem(localization, now_s=observation.now_s)
        if problem:
            return self._finish(
                NavigationOutcome.LOCALIZATION_LOST,
                problem,
                localization,
            )
        assert localization.pose is not None
        previous_pose = (
            None
            if self._last_localization is None
            else self._last_localization.pose
        )
        if previous_pose is not None:
            step = math.hypot(
                localization.pose.x_m - previous_pose.x_m,
                localization.pose.y_m - previous_pose.y_m,
            )
            self._cumulative_translation_m += step
        self._record_pose(localization)

        track_problem = self._track_problem(observation)
        if track_problem:
            return self._finish(
                NavigationOutcome.PROGRESS_FAILED,
                track_problem,
                localization,
            )
        if (
            self._last_horizon is not None
            and self._last_horizon.kind is HorizonKind.TRANSLATE
            and previous_pose is not None
        ):
            step = math.hypot(
                localization.pose.x_m - previous_pose.x_m,
                localization.pose.y_m - previous_pose.y_m,
            )
            if step < self.config.minimum_progress_m:
                return self._finish(
                    NavigationOutcome.PROGRESS_FAILED,
                    "localized_translation_progress_below_minimum",
                    localization,
                )
        if self._cumulative_translation_m > self._goal.max_cumulative_translation_m:
            return self._finish(
                NavigationOutcome.PARTIAL,
                "translation_budget_exhausted",
                localization,
            )

        self._last_localization = localization
        if observation.obstacle_blocked:
            alternative = self._alternate_horizon(observation)
            if alternative is None:
                return self._finish(
                    NavigationOutcome.BLOCKED,
                    "no_clear_alternate_horizon",
                    localization,
                )
            self._last_horizon = alternative
            self._append_event("alternate_horizon", alternative.reason, localization.pose, alternative)
            return self._decision(NavigationOutcome.RUNNING, "", localization, alternative)
        reason = "feedback_correction" if self._last_horizon is not None else "next_horizon"
        return self._plan(localization, reason=reason)

    def _plan(self, localization: LocalizationEstimate, *, reason: str) -> NavigationDecision:
        assert self._goal is not None
        assert localization.pose is not None
        pose = localization.pose
        distance = math.hypot(self._goal.x_m - pose.x_m, self._goal.y_m - pose.y_m)
        if distance <= self._goal.radius_m:
            if self._heading_satisfied(pose.yaw_rad):
                return self._finish(NavigationOutcome.REACHED, "goal_region_reached", localization)
            desired_heading = self._goal_heading()
            assert desired_heading is not None
            heading_error = normalize_angle(desired_heading - pose.yaw_rad)
        else:
            desired_heading = math.atan2(self._goal.y_m - pose.y_m, self._goal.x_m - pose.x_m)
            heading_error = normalize_angle(desired_heading - pose.yaw_rad)

        if abs(heading_error) > self.config.heading_deadband_rad:
            horizon = self._new_horizon(
                HorizonKind.ROTATE,
                distance_m=0.0,
                angle_rad=_clamp(
                    heading_error,
                    -self.config.max_rotation_horizon_rad,
                    self.config.max_rotation_horizon_rad,
                ),
                reason=reason,
            )
        else:
            remaining = max(0.0, distance - self._goal.radius_m)
            horizon = self._new_horizon(
                HorizonKind.TRANSLATE,
                distance_m=min(remaining, self.config.max_translation_horizon_m),
                angle_rad=0.0,
                reason=reason,
            )
        self._last_horizon = horizon
        self._append_event(
            "correction" if reason == "feedback_correction" else "horizon_planned",
            reason,
            pose,
            horizon,
        )
        return self._decision(NavigationOutcome.RUNNING, "", localization, horizon)

    def _alternate_horizon(
        self, observation: HorizonObservation
    ) -> Optional[MotionHorizon]:
        assert self._goal is not None
        if self._alternate_horizons >= self.config.max_alternate_horizons:
            return None
        choices = (
            (1.0, observation.left_clearance_m),
            (-1.0, observation.right_clearance_m),
        )
        viable = [
            (direction, float(clearance))
            for direction, clearance in choices
            if clearance is not None
            and math.isfinite(float(clearance))
            and float(clearance) >= self._goal.minimum_clearance_m
        ]
        if not viable:
            return None
        direction, clearance = max(viable, key=lambda item: item[1])
        self._alternate_horizons += 1
        return self._new_horizon(
            HorizonKind.ROTATE,
            distance_m=0.0,
            angle_rad=direction * self.config.max_rotation_horizon_rad,
            reason=f"obstacle_alternate_{'left' if direction > 0 else 'right'}_clearance_{clearance:.3f}m",
        )

    def _localization_problem(
        self, localization: LocalizationEstimate, *, now_s: float
    ) -> str:
        if localization.state in {LocalizationState.STALE, LocalizationState.LOST}:
            return f"localization_{localization.state.value}"
        if localization.pose is None:
            return "localization_pose_unavailable"
        if localization.pose.frame_id != (self._goal.frame_id if self._goal else localization.pose.frame_id):
            return "localization_frame_mismatch"
        if now_s - localization.pose.stamp_s > self.config.max_localization_age_s:
            return "localization_stale"
        if localization.quality < self.config.minimum_localization_quality:
            return "localization_quality_below_minimum"
        if (
            localization.covariance_xy_m2 is None
            or localization.covariance_xy_m2 > self.config.max_covariance_xy_m2
        ):
            return "localization_xy_covariance_unacceptable"
        if (
            localization.covariance_yaw_rad2 is None
            or localization.covariance_yaw_rad2 > self.config.max_covariance_yaw_rad2
        ):
            return "localization_yaw_covariance_unacceptable"
        if (
            localization.odom_translation_disagreement_m is not None
            and localization.odom_translation_disagreement_m
            > self.config.max_odom_translation_disagreement_m
        ):
            return "localization_odom_translation_disagreement"
        if (
            localization.odom_heading_disagreement_rad is not None
            and localization.odom_heading_disagreement_rad
            > self.config.max_odom_heading_disagreement_rad
        ):
            return "localization_odom_heading_disagreement"
        if not (
            localization.source.startswith("lidar")
            or localization.source.startswith("slam")
        ):
            return "localization_source_not_lidar_authoritative"
        return ""

    def _track_problem(self, observation: HorizonObservation) -> str:
        if self._last_horizon is None:
            return ""
        left = observation.left_track_delta_m
        right = observation.right_track_delta_m
        if left is None or right is None:
            return "per_track_evidence_unavailable"
        activity = max(abs(left), abs(right))
        if activity < self.config.minimum_track_activity_m:
            return "track_progress_below_minimum"
        asymmetry = abs(abs(left) - abs(right)) / activity
        if asymmetry > self.config.max_track_asymmetry_fraction:
            return "severe_tread_asymmetry"
        if self._last_horizon.kind is HorizonKind.TRANSLATE and left * right < 0.0:
            return "unexpected_opposed_tread_motion"
        if self._last_horizon.kind is HorizonKind.ROTATE and left * right >= 0.0:
            return "unexpected_same_direction_tread_motion"
        return ""

    def _record_pose(self, localization: LocalizationEstimate) -> None:
        if localization.pose is not None:
            self._path.append(localization.pose)

    def _new_horizon(
        self,
        kind: HorizonKind,
        *,
        distance_m: float,
        angle_rad: float,
        reason: str,
    ) -> MotionHorizon:
        assert self._goal is not None
        self._horizon_sequence += 1
        return MotionHorizon(
            horizon_id=self._horizon_sequence,
            kind=kind,
            distance_m=distance_m,
            angle_rad=angle_rad,
            timeout_s=self.config.horizon_timeout_s,
            minimum_clearance_m=self._goal.minimum_clearance_m,
            reason=reason,
        )

    def _heading_satisfied(self, yaw_rad: float) -> bool:
        if self._goal is None or self._goal.heading_min_rad is None:
            return True
        minimum = normalize_angle(self._goal.heading_min_rad)
        maximum = normalize_angle(self._goal.heading_max_rad)
        yaw = normalize_angle(yaw_rad)
        if minimum <= maximum:
            return minimum <= yaw <= maximum
        return yaw >= minimum or yaw <= maximum

    def _goal_heading(self) -> Optional[float]:
        assert self._goal is not None
        if self._goal.heading_min_rad is None:
            return None
        minimum = normalize_angle(self._goal.heading_min_rad)
        span = (normalize_angle(self._goal.heading_max_rad) - minimum) % (2.0 * math.pi)
        return normalize_angle(minimum + span / 2.0)

    def _append_event(
        self,
        kind: str,
        detail: str,
        pose: Optional[Pose2D],
        horizon: Optional[MotionHorizon],
    ) -> None:
        self._events.append(
            NavigationEvent(len(self._events) + 1, kind, detail, pose, horizon)
        )

    def _finish(
        self,
        outcome: NavigationOutcome,
        reason: str,
        localization: LocalizationEstimate,
    ) -> NavigationDecision:
        self._append_event("terminal", reason, localization.pose, None)
        decision = self._decision(outcome, reason, localization, None)
        self._terminal = decision
        return decision

    def _decision(
        self,
        outcome: NavigationOutcome,
        reason: str,
        localization: LocalizationEstimate,
        horizon: Optional[MotionHorizon],
    ) -> NavigationDecision:
        assert self._goal is not None
        return NavigationDecision(
            outcome=outcome,
            terminal_reason=reason,
            localization=localization,
            goal=self._goal,
            next_horizon=horizon,
            path=tuple(self._path),
            events=tuple(self._events),
            cumulative_translation_m=self._cumulative_translation_m,
            zero_output_required=outcome is not NavigationOutcome.RUNNING,
        )


def run_navigation_replay(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Run a replay corpus item without ROS, hardware, or command publication."""

    if payload.get("schema") != REPLAY_SCHEMA:
        raise ValueError(f"replay schema must be {REPLAY_SCHEMA!r}")
    goal_value = payload.get("goal")
    initial_value = payload.get("initial_localization")
    observations = payload.get("observations")
    if not isinstance(goal_value, Mapping) or not isinstance(initial_value, Mapping):
        raise ValueError("replay requires goal and initial_localization objects")
    if not isinstance(observations, list):
        raise ValueError("replay observations must be a list")
    navigator = PerceptionGuidedNavigator()
    initial = LocalizationEstimate.from_mapping(initial_value)
    initial_now = _finite(payload.get("started_at_s"), "replay started_at_s")
    decision = navigator.start(GoalRegion.from_mapping(goal_value), initial, now_s=initial_now)
    decisions = [decision.to_json_dict()]
    for item in observations:
        if decision.outcome is not NavigationOutcome.RUNNING:
            break
        if not isinstance(item, Mapping):
            raise ValueError("replay observation must be an object")
        decision = navigator.observe(HorizonObservation.from_mapping(item))
        decisions.append(decision.to_json_dict())
    return {
        "schema": RESULT_SCHEMA,
        "replay_id": str(payload.get("replay_id", "")).strip(),
        "source_evidence": dict(payload.get("source_evidence", {}))
        if isinstance(payload.get("source_evidence"), Mapping)
        else {},
        "decisions": decisions,
        "terminal": decision.to_json_dict(),
        "motion_authority": False,
        "physical_execution_enabled": False,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="Replay perception-guided navigation evidence without physical execution."
    )
    parser.add_argument("replay_json", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    payload = json.loads(args.replay_json.read_text())
    if not isinstance(payload, Mapping):
        raise ValueError("replay file must contain a JSON object")
    result = run_navigation_replay(payload)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        args.output.write_text(rendered)
    return 0


def normalize_angle(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _optional_finite(value: Any, *, signed: bool = False) -> Optional[float]:
    if value is None:
        return None
    result = _finite(value, "optional numeric value")
    if not signed and result < 0.0:
        raise ValueError("optional numeric value must be non-negative")
    return result


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


if __name__ == "__main__":
    raise SystemExit(main())
