"""Pure collision-stop arbitration for the lidar supervisor.

The ROS node wrapper lives in ``collision_stop_node.py``.  This module stays ROS-free
so safety behavior can be simulated on development hosts without hardware, ROS, or a
live graph.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Mapping, Optional, Sequence


class CollisionState(str, Enum):
    STARTUP = "STARTUP"
    CLEAR = "CLEAR"
    SLOW = "SLOW"
    STOPPED = "STOPPED"
    SENSOR_STALE = "SENSOR_STALE"
    ESTOPPED = "ESTOPPED"
    DISABLED = "DISABLED"


class ResetPolicy(str, Enum):
    MANUAL = "manual"
    AUTO_AFTER_CLEAR = "auto_after_clear"


@dataclass(frozen=True)
class Transform2D:
    """Planar transform from a scan frame into the configured base frame."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def is_finite(self) -> bool:
        return math.isfinite(self.x) and math.isfinite(self.y) and math.isfinite(self.yaw)

    def transform_point(self, x: float, y: float) -> tuple[float, float]:
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return (
            self.x + cos_yaw * x - sin_yaw * y,
            self.y + sin_yaw * x + cos_yaw * y,
        )


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class ScanInput:
    ranges: Sequence[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    stamp: Optional[float] = None
    received_at: Optional[float] = None
    frame_id: str = "laser"
    transform_to_base: Optional[Transform2D] = None
    transform_error: Optional[str] = None
    stamp_progress_error: Optional[str] = None


@dataclass(frozen=True)
class CollisionStopConfig:
    requested_cmd_timeout_s: float = 0.25
    max_scan_age_s: float = 0.30
    max_scan_stamp_age_s: float = 0.75
    startup_grace_s: float = 2.0
    min_valid_ranges: int = 12
    min_valid_fraction: float = 0.05
    min_range_m: float = 0.08
    max_range_m: float = 6.0
    sector_unknown_policy: str = "blocked"
    footprint_front_m: float = 0.22
    footprint_rear_m: float = 0.16
    footprint_left_m: float = 0.14
    footprint_right_m: float = 0.14
    payload_margin_m: float = 0.05
    front_stop_min_angle_deg: float = -30.0
    front_stop_max_angle_deg: float = 30.0
    front_slow_min_angle_deg: float = -45.0
    front_slow_max_angle_deg: float = 45.0
    rear_stop_angle_width_deg: float = 30.0
    left_spin_min_angle_deg: float = 45.0
    left_spin_max_angle_deg: float = 135.0
    right_spin_min_angle_deg: float = -135.0
    right_spin_max_angle_deg: float = -45.0
    stop_distance_m: float = 0.35
    slow_distance_m: float = 0.60
    reverse_stop_distance_m: float = 0.25
    trajectory_clearance_margin_m: float = 0.02
    measured_stop_time_s: float = 0.50
    braking_distance_margin_m: float = 0.02
    release_distance_m: float = 0.45
    release_time_s: float = 0.50
    min_forward_scale: float = 0.0
    max_forward_mps: float = 0.10
    max_angular_rad_s: float = 0.4
    reset_policy: ResetPolicy = ResetPolicy.MANUAL
    zero_publish_period_s: float = 0.10
    allow_disable: bool = False
    fail_on_missing_tf: bool = True
    base_frame: str = "base_link"
    laser_frame: str = "laser"
    tf_timeout_s: float = 0.05

    def __post_init__(self) -> None:
        if isinstance(self.reset_policy, str):
            object.__setattr__(self, "reset_policy", ResetPolicy(self.reset_policy))
        if self.max_scan_age_s <= 0:
            raise ValueError("max_scan_age_s must be positive")
        if self.max_scan_stamp_age_s <= 0:
            raise ValueError("max_scan_stamp_age_s must be positive")
        if self.max_scan_stamp_age_s < self.max_scan_age_s:
            raise ValueError("max_scan_stamp_age_s must be at least max_scan_age_s")
        if self.requested_cmd_timeout_s <= 0:
            raise ValueError("requested_cmd_timeout_s must be positive")
        if self.tf_timeout_s < 0:
            raise ValueError("tf_timeout_s must be non-negative")
        if self.slow_distance_m <= self.stop_distance_m:
            raise ValueError("slow_distance_m must be greater than stop_distance_m")
        if self.release_distance_m <= self.stop_distance_m:
            raise ValueError("release_distance_m must be greater than stop_distance_m")
        if self.measured_stop_time_s < 0.0:
            raise ValueError("measured_stop_time_s must be non-negative")
        if self.braking_distance_margin_m < 0.0:
            raise ValueError("braking_distance_margin_m must be non-negative")
        if (
            not math.isfinite(self.trajectory_clearance_margin_m)
            or self.trajectory_clearance_margin_m < 0.0
        ):
            raise ValueError("trajectory_clearance_margin_m must be finite and non-negative")
        for name, extent in (
            ("footprint_front_m", self.footprint_front_m),
            ("footprint_rear_m", self.footprint_rear_m),
            ("footprint_left_m", self.footprint_left_m),
            ("footprint_right_m", self.footprint_right_m),
        ):
            if not math.isfinite(extent) or extent <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not (0.0 <= self.min_forward_scale <= 1.0):
            raise ValueError("min_forward_scale must be between 0 and 1")


@dataclass(frozen=True)
class ScanHealth:
    healthy: bool
    reason: str
    age_s: Optional[float]
    valid_count: int
    considered_count: int
    frame_id: str
    base_frame: str = "base_link"
    tf_available: bool = False
    tf_reason: str = "not_checked"
    stamp_age_s: Optional[float] = None


@dataclass(frozen=True)
class ScanEvaluation:
    health: ScanHealth
    nearest: Mapping[str, Optional[float]]

    @property
    def healthy(self) -> bool:
        return self.health.healthy

    @property
    def reason(self) -> str:
        return self.health.reason


@dataclass(frozen=True)
class TrajectoryCheck:
    """Result of sweeping the configured footprint along a bounded command."""

    blocked: bool
    horizon_s: float
    minimum_clearance_m: Optional[float]
    collision_time_s: Optional[float] = None
    collision_point_x_m: Optional[float] = None
    collision_point_y_m: Optional[float] = None
    moving_away_point_count: int = 0


@dataclass(frozen=True)
class ArbitrationDecision:
    state: CollisionState
    previous_state: CollisionState
    reason: str
    output: TwistCommand
    requested: TwistCommand = TwistCommand()
    scan_health: ScanHealth = field(
        default_factory=lambda: ScanHealth(False, "missing_scan", None, 0, 0, "")
    )
    nearest: Mapping[str, Optional[float]] = field(default_factory=dict)
    trajectory: Optional[TrajectoryCheck] = None
    scale: float = 0.0
    reset_required: bool = False


@dataclass(frozen=True)
class ResetResult:
    accepted: bool
    reason: str
    decision: ArbitrationDecision


class ScanStampTracker:
    """Reject non-advancing source stamps without treating acquisition as transport delay."""

    def __init__(self) -> None:
        self._latest_stamp: Optional[float] = None

    def check(self, stamp: Optional[float]) -> Optional[str]:
        if stamp is None:
            return None
        try:
            current = float(stamp)
        except (TypeError, ValueError):
            return "non_finite_scan_stamp"
        if not math.isfinite(current):
            return "non_finite_scan_stamp"
        if self._latest_stamp is not None and current <= self._latest_stamp:
            return "non_advancing_scan_stamp"
        self._latest_stamp = current
        return None


def evaluate_scan(scan: ScanInput, config: CollisionStopConfig, *, now: float) -> ScanEvaluation:
    if scan is None:
        return _scan_eval(False, "missing_scan", None, 0, 0, "", {})
    ranges = tuple(scan.ranges or ())
    age, stamp_age = _scan_ages(scan, now)
    if age is not None and age > config.max_scan_age_s + 1e-9:
        return _scan_eval(
            False,
            "stale_scan",
            age,
            0,
            len(ranges),
            scan.frame_id,
            {},
            stamp_age=stamp_age,
        )
    if stamp_age is not None and stamp_age > config.max_scan_stamp_age_s + 1e-9:
        return _scan_eval(
            False,
            "stale_scan_stamp",
            age,
            0,
            len(ranges),
            scan.frame_id,
            {},
            stamp_age=stamp_age,
        )
    if scan.stamp_progress_error:
        return _scan_eval(
            False,
            str(scan.stamp_progress_error),
            age,
            0,
            len(ranges),
            scan.frame_id,
            {},
            stamp_age=stamp_age,
        )
    if not ranges:
        return _scan_eval(False, "empty_ranges", age, 0, 0, scan.frame_id, {}, stamp_age=stamp_age)
    if not math.isfinite(scan.angle_min):
        return _scan_eval(
            False,
            "angle_min",
            age,
            0,
            len(ranges),
            scan.frame_id,
            {},
            stamp_age=stamp_age,
        )
    if not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0.0:
        return _scan_eval(
            False,
            "angle_increment",
            age,
            0,
            len(ranges),
            scan.frame_id,
            {},
            stamp_age=stamp_age,
        )
    if not math.isfinite(scan.range_min) or not math.isfinite(scan.range_max) or scan.range_max <= scan.range_min:
        return _scan_eval(
            False,
            "range_limits",
            age,
            0,
            len(ranges),
            scan.frame_id,
            {},
            stamp_age=stamp_age,
        )

    transform, tf_available, tf_reason = _resolve_scan_transform(scan, config)
    if not tf_available and config.fail_on_missing_tf:
        return _scan_eval(
            False,
            tf_reason,
            age,
            0,
            len(ranges),
            scan.frame_id,
            {},
            base_frame=config.base_frame,
            tf_available=False,
            tf_reason=tf_reason,
            stamp_age=stamp_age,
        )

    sectors = _sector_samples(scan, config, transform)
    considered = sum(len(values) for values in sectors.values())
    valid_count = sum(1 for values in sectors.values() for value in values if value is not None)
    min_required = max(config.min_valid_ranges, math.ceil(considered * config.min_valid_fraction))
    nearest = {name: _nearest(values) for name, values in sectors.items()}
    if considered == 0:
        return _scan_eval(False, "sector_not_covered", age, valid_count, considered, scan.frame_id, nearest, base_frame=config.base_frame, tf_available=tf_available, tf_reason=tf_reason, stamp_age=stamp_age)
    if valid_count < min_required:
        return _scan_eval(False, "insufficient_valid_ranges", age, valid_count, considered, scan.frame_id, nearest, base_frame=config.base_frame, tf_available=tf_available, tf_reason=tf_reason, stamp_age=stamp_age)
    if config.sector_unknown_policy == "blocked":
        for name, values in sectors.items():
            sector_valid = sum(1 for value in values if value is not None)
            sector_required = max(1, math.ceil(len(values) * config.min_valid_fraction))
            if sector_valid < sector_required:
                return _scan_eval(False, f"{name}_unknown", age, valid_count, considered, scan.frame_id, nearest, base_frame=config.base_frame, tf_available=tf_available, tf_reason=tf_reason, stamp_age=stamp_age)
    return _scan_eval(True, "fresh", age, valid_count, considered, scan.frame_id, nearest, base_frame=config.base_frame, tf_available=tf_available, tf_reason=tf_reason, stamp_age=stamp_age)


def _scan_eval(
    healthy: bool,
    reason: str,
    age: Optional[float],
    valid_count: int,
    considered_count: int,
    frame_id: str,
    nearest: Mapping[str, Optional[float]],
    *,
    base_frame: str = "base_link",
    tf_available: bool = False,
    tf_reason: str = "not_checked",
    stamp_age: Optional[float] = None,
) -> ScanEvaluation:
    return ScanEvaluation(
        health=ScanHealth(
            healthy,
            reason,
            age,
            valid_count,
            considered_count,
            frame_id,
            base_frame,
            tf_available,
            tf_reason,
            stamp_age,
        ),
        nearest=nearest,
    )


def _scan_ages(scan: ScanInput, now: float) -> tuple[Optional[float], Optional[float]]:
    """Return receipt age and source-stamp age without conflating acquisition time.

    A LaserScan stamp commonly marks the beginning of a full-revolution
    acquisition, so its age at callback time is not transport staleness.  Receipt
    age detects a stopped stream; stamp age independently rejects frozen or
    replayed source evidence.
    """

    receipt_basis = scan.received_at if scan.received_at is not None else scan.stamp
    receipt_age = None if receipt_basis is None else _elapsed_age(now, receipt_basis)
    stamp_age = None if scan.stamp is None else _elapsed_age(now, scan.stamp)
    return receipt_age, stamp_age


def _elapsed_age(now: float, basis: float) -> float:
    try:
        age = float(now) - float(basis)
    except (TypeError, ValueError):
        return math.inf
    if not math.isfinite(age):
        return math.inf
    return max(0.0, age)


def _resolve_scan_transform(scan: ScanInput, config: CollisionStopConfig) -> tuple[Transform2D, bool, str]:
    frame_id = scan.frame_id or config.laser_frame
    if frame_id == config.base_frame:
        return Transform2D(), True, "identity_same_frame"
    if scan.transform_error:
        return Transform2D(), False, scan.transform_error
    transform = scan.transform_to_base
    if transform is None:
        return Transform2D(), False, "missing_tf"
    if not transform.is_finite():
        return Transform2D(), False, "malformed_tf"
    return transform, True, "ok"


def _range_is_number(value: float) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _sector_samples(scan: ScanInput, config: CollisionStopConfig, transform_to_base: Transform2D) -> dict[str, list[Optional[float]]]:
    sectors: dict[str, list[Optional[float]]] = {
        "front": [],
        "front_slow": [],
        "rear": [],
        "left": [],
        "right": [],
        # EVERY bearing, no angular window. The four directional sectors together
        # leave 60 deg uncovered -- (30,45), (135,150), (-150,-135), (-45,-30) -- and
        # the footprint's corners sit at +/-42.3 and +/-148.0 deg, i.e. the maximum
        # radius directions of a pivot are exactly the directions nothing reads. Any
        # check about sweeping the whole footprint has to use this, not the sectors.
        "any": [],
    }
    min_range = max(float(scan.range_min), config.min_range_m)
    max_range = min(float(scan.range_max), config.max_range_m)
    rear_width = abs(config.rear_stop_angle_width_deg)
    for index, raw_value in enumerate(scan.ranges):
        scan_angle = scan.angle_min + index * scan.angle_increment
        if _range_is_number(raw_value):
            point_x = float(raw_value) * math.cos(scan_angle)
            point_y = float(raw_value) * math.sin(scan_angle)
        else:
            point_x = 0.0
            point_y = 0.0
        base_x, base_y = transform_to_base.transform_point(point_x, point_y)
        angle_deg = _normalize_degrees(math.degrees(math.atan2(base_y, base_x)))
        value = _valid_range(raw_value, min_range=min_range, max_range=max_range)
        sectors["any"].append(value)
        if config.front_stop_min_angle_deg <= angle_deg <= config.front_stop_max_angle_deg:
            sectors["front"].append(value)
        if config.front_slow_min_angle_deg <= angle_deg <= config.front_slow_max_angle_deg:
            sectors["front_slow"].append(value)
        if angle_deg >= 180.0 - rear_width or angle_deg <= -180.0 + rear_width:
            sectors["rear"].append(value)
        if config.left_spin_min_angle_deg <= angle_deg <= config.left_spin_max_angle_deg:
            sectors["left"].append(value)
        if config.right_spin_min_angle_deg <= angle_deg <= config.right_spin_max_angle_deg:
            sectors["right"].append(value)
    return sectors


def _valid_range(value: float, *, min_range: float, max_range: float) -> Optional[float]:
    try:
        range_m = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(range_m):
        return None
    if range_m < min_range or range_m > max_range:
        return None
    return range_m


def _nearest(values: Sequence[Optional[float]]) -> Optional[float]:
    valid = [value for value in values if value is not None]
    return min(valid) if valid else None


def _normalize_degrees(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def evaluate_projected_trajectory(
    scan: ScanInput,
    config: CollisionStopConfig,
    command: TwistCommand,
) -> TrajectoryCheck:
    """Project a bounded command and test its swept rectangular footprint.

    The horizon covers the time before a stale command is suppressed plus the
    measured stopping interval.  A scan point blocks only when it enters the
    footprint expanded by the configured clearance margin at a sampled pose.
    This keeps turn safety tied to the actual trajectory instead of treating an
    entire side sector as occupied.
    """

    horizon_s = config.requested_cmd_timeout_s + config.measured_stop_time_s
    transform, tf_available, _ = _resolve_scan_transform(scan, config)
    if not tf_available:
        return TrajectoryCheck(True, horizon_s, None, collision_time_s=0.0)

    points = _valid_base_points(scan, config, transform)
    if not points:
        return TrajectoryCheck(True, horizon_s, None, collision_time_s=0.0)

    linear_x = float(command.linear_x)
    angular_z = float(command.angular_z)
    linear_steps = math.ceil(abs(linear_x) * horizon_s / 0.01)
    angular_steps = math.ceil(abs(angular_z) * horizon_s / math.radians(2.0))
    step_count = max(1, linear_steps, angular_steps)

    margin = config.trajectory_clearance_margin_m
    front = config.footprint_front_m + margin
    rear = config.footprint_rear_m + margin
    left = config.footprint_left_m + margin
    right = config.footprint_right_m + margin
    poses = tuple(
        _constant_twist_pose(
            linear_x,
            angular_z,
            horizon_s * step / step_count,
        )
        for step in range(step_count + 1)
    )
    minimum_clearance = math.inf
    moving_away_point_count = 0

    for point_x, point_y in points:
        initial_signed_clearance = _signed_footprint_clearance(
            point_x,
            point_y,
            front=front,
            rear=rear,
            left=left,
            right=right,
        )
        if (
            initial_signed_clearance <= 0.0
            and _trajectory_moves_away_from_overlapped_point(
                point_x,
                point_y,
                command=command,
                poses=poses,
                front=front,
                rear=rear,
                left=left,
                right=right,
            )
        ):
            moving_away_point_count += 1
            continue
        for step, pose in enumerate(poses):
            local_x, local_y = _point_in_pose_frame(point_x, point_y, pose)
            clearance = max(
                0.0,
                _signed_footprint_clearance(
                    local_x,
                    local_y,
                    front=front,
                    rear=rear,
                    left=left,
                    right=right,
                ),
            )
            minimum_clearance = min(minimum_clearance, clearance)
            if clearance <= 0.0:
                elapsed_s = horizon_s * step / step_count
                return TrajectoryCheck(
                    True,
                    horizon_s,
                    0.0,
                    collision_time_s=elapsed_s,
                    collision_point_x_m=point_x,
                    collision_point_y_m=point_y,
                    moving_away_point_count=moving_away_point_count,
                )

    if moving_away_point_count == len(points):
        return TrajectoryCheck(
            True,
            horizon_s,
            None,
            collision_time_s=0.0,
            moving_away_point_count=moving_away_point_count,
        )
    return TrajectoryCheck(
        False,
        horizon_s,
        minimum_clearance if math.isfinite(minimum_clearance) else None,
        moving_away_point_count=moving_away_point_count,
    )


def _trajectory_moves_away_from_overlapped_point(
    point_x: float,
    point_y: float,
    *,
    command: TwistCommand,
    poses: Sequence[tuple[float, float, float]],
    front: float,
    rear: float,
    left: float,
    right: float,
) -> bool:
    """Return true only when the complete sampled path moves away from a point."""

    if len(poses) < 2:
        return False
    previous_signed: Optional[float] = None
    signed_increased = False
    tolerance = 1e-9
    for pose in poses:
        local_x, local_y = _point_in_pose_frame(point_x, point_y, pose)
        signed = _signed_footprint_clearance(
            local_x,
            local_y,
            front=front,
            rear=rear,
            left=left,
            right=right,
        )
        if previous_signed is not None:
            if signed < previous_signed - tolerance:
                return False
            signed_increased = signed_increased or signed > previous_signed + tolerance
        previous_signed = signed
    # Relative radial separation from base_link changes at -linear_x * point_x;
    # angular velocity cancels because rotation alone preserves radius.  This
    # admits translation away from a trailing/front point even when the nearest
    # rectangular edge stays momentarily unchanged, while tangential motion
    # does not qualify unless signed footprint clearance itself increases.
    radial_separation_rate = -float(command.linear_x) * point_x
    return signed_increased or radial_separation_rate > tolerance


def _point_in_pose_frame(
    point_x: float,
    point_y: float,
    pose: tuple[float, float, float],
) -> tuple[float, float]:
    pose_x, pose_y, pose_yaw = pose
    dx = point_x - pose_x
    dy = point_y - pose_y
    cos_yaw = math.cos(pose_yaw)
    sin_yaw = math.sin(pose_yaw)
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
    )


def _signed_footprint_clearance(
    local_x: float,
    local_y: float,
    *,
    front: float,
    rear: float,
    left: float,
    right: float,
) -> float:
    outside_x = max(local_x - front, -rear - local_x, 0.0)
    outside_y = max(local_y - left, -right - local_y, 0.0)
    if outside_x > 0.0 or outside_y > 0.0:
        return math.hypot(outside_x, outside_y)
    penetration = min(
        front - local_x,
        local_x + rear,
        left - local_y,
        local_y + right,
    )
    return -penetration


def _valid_base_points(
    scan: ScanInput,
    config: CollisionStopConfig,
    transform_to_base: Transform2D,
) -> tuple[tuple[float, float], ...]:
    min_range = max(float(scan.range_min), config.min_range_m)
    max_range = min(float(scan.range_max), config.max_range_m)
    points: list[tuple[float, float]] = []
    for index, raw_value in enumerate(scan.ranges):
        value = _valid_range(raw_value, min_range=min_range, max_range=max_range)
        if value is None:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        point_x = value * math.cos(angle)
        point_y = value * math.sin(angle)
        points.append(transform_to_base.transform_point(point_x, point_y))
    return tuple(points)


def _constant_twist_pose(
    linear_x: float,
    angular_z: float,
    elapsed_s: float,
) -> tuple[float, float, float]:
    yaw = angular_z * elapsed_s
    if abs(angular_z) <= 1e-9:
        return linear_x * elapsed_s, 0.0, 0.0
    radius = linear_x / angular_z
    return radius * math.sin(yaw), radius * (1.0 - math.cos(yaw)), yaw


class CollisionStopSupervisor:
    def __init__(self, config: CollisionStopConfig, *, now: float = 0.0):
        self.config = config
        self.state = CollisionState.STARTUP
        self.previous_state = CollisionState.STARTUP
        self._started_at = now
        self._latest_scan: Optional[ScanInput] = None
        self._latest_eval: Optional[ScanEvaluation] = None
        self._latest_command: Optional[TwistCommand] = None
        self._latest_command_at: Optional[float] = None
        self._clear_since: Optional[float] = None
        self._latched_stop_reason = ""
        self._scan_stamp_tracker = ScanStampTracker()
        self._last_decision = self._decision(
            CollisionState.STARTUP,
            "startup",
            TwistCommand(),
            TwistCommand(),
            self._missing_scan_health(now),
            {},
            scale=0.0,
        )

    def update_scan(self, scan: ScanInput, *, now: float) -> ArbitrationDecision:
        progress_error = self._scan_stamp_tracker.check(scan.stamp)
        if progress_error is not None:
            scan = replace(scan, stamp_progress_error=progress_error)
        self._latest_scan = scan
        self._latest_eval = evaluate_scan(scan, self.config, now=now)
        return self._arbitrate(self._latest_command or TwistCommand(), now=now, reason="scan")

    def apply_command(self, command: TwistCommand, *, now: float) -> ArbitrationDecision:
        self._latest_command = command
        self._latest_command_at = now
        return self._arbitrate(command, now=now, reason="command")

    def tick(self, *, now: float) -> ArbitrationDecision:
        return self._arbitrate(self._latest_command or TwistCommand(), now=now, reason="tick")

    def stop(self, *, now: float) -> ArbitrationDecision:
        self._latest_command = None
        self._latest_command_at = None
        self._clear_since = None
        return self._decision_forced(CollisionState.STOPPED, "operator_stop", now)

    def estop(self, *, now: float) -> ArbitrationDecision:
        self._latest_command = None
        self._latest_command_at = None
        self._clear_since = None
        return self._decision_forced(CollisionState.ESTOPPED, "operator_estop", now)

    def clear_estop(self, *, now: float) -> ArbitrationDecision:
        self._latest_command = None
        self._latest_command_at = None
        if self.state is not CollisionState.ESTOPPED:
            return self._arbitrate(TwistCommand(), now=now, reason="clear_estop_noop")
        self.state = CollisionState.STARTUP
        return self._arbitrate(TwistCommand(), now=now, reason="clear_estop")

    def reset(self, *, now: float) -> ResetResult:
        if self.state is CollisionState.ESTOPPED:
            return ResetResult(False, "estop_active", self._last_decision)
        decision = self._arbitrate(TwistCommand(), now=now, reason="reset_check")
        if decision.state is CollisionState.CLEAR or (
            decision.state is CollisionState.STOPPED
            and decision.scan_health.healthy
            and self._release_elapsed(now)
        ):
            self._latest_command = None
            self._latest_command_at = None
            accepted = self._decision(CollisionState.CLEAR, "reset_accepted", TwistCommand(), TwistCommand(), decision.scan_health, decision.nearest)
            return ResetResult(True, "reset_accepted", accepted)
        return ResetResult(False, decision.reason, decision)

    def _decision_forced(self, state: CollisionState, reason: str, now: float) -> ArbitrationDecision:
        eval_result = self._current_scan_eval(now)
        return self._decision(state, reason, TwistCommand(), TwistCommand(), eval_result.health, eval_result.nearest, reset_required=state is CollisionState.STOPPED)

    def _arbitrate(self, command: TwistCommand, *, now: float, reason: str) -> ArbitrationDecision:
        if self.state is CollisionState.ESTOPPED:
            return self._decision(CollisionState.ESTOPPED, "estop_latched", TwistCommand(), command, self._current_scan_eval(now).health, self._current_scan_eval(now).nearest)

        eval_result = self._current_scan_eval(now)
        health = eval_result.health
        nearest = eval_result.nearest
        if not health.healthy:
            stale_state = CollisionState.STARTUP if now - self._started_at < self.config.startup_grace_s and health.reason == "missing_scan" else CollisionState.SENSOR_STALE
            self._latest_command = None
            self._latest_command_at = None
            return self._decision(stale_state, health.reason, TwistCommand(), command, health, nearest)

        if self._command_is_stale(now):
            command = TwistCommand()
            reason = "stale_command"

        if not _twist_is_finite(command):
            self._latest_command = None
            self._latest_command_at = None
            return self._decision(CollisionState.STOPPED, "non_finite_command", TwistCommand(), command, health, nearest, reset_required=True)

        bounded = self._bound(command)
        front = nearest.get("front")
        front_slow = nearest.get("front_slow")
        rear = nearest.get("rear")
        left = nearest.get("left")
        right = nearest.get("right")

        clear_for_release = self._release_clear(front, rear, bounded)
        if clear_for_release:
            if self._clear_since is None:
                self._clear_since = now
        else:
            self._clear_since = None

        front_stop_distance = self._front_stop_distance(bounded)
        rear_stop_distance = self._rear_stop_distance(bounded)
        trajectory_check: Optional[TrajectoryCheck] = None

        if self.state is CollisionState.STOPPED:
            if (
                self._latched_stop_reason == "front_stop"
                and bounded.linear_x < 0.0
            ):
                if _within(rear, rear_stop_distance):
                    return self._decision(
                        CollisionState.STOPPED,
                        "reverse_escape_rear_blocked",
                        TwistCommand(),
                        command,
                        health,
                        nearest,
                        reset_required=True,
                    )
                trajectory_check = evaluate_projected_trajectory(
                    self._latest_scan,
                    self.config,
                    bounded,
                )
                if trajectory_check.blocked:
                    return self._decision(
                        CollisionState.STOPPED,
                        "reverse_escape_trajectory_blocked",
                        TwistCommand(),
                        command,
                        health,
                        nearest,
                        trajectory=trajectory_check,
                        reset_required=True,
                    )
                return self._decision(
                    CollisionState.SLOW,
                    "front_stop_reverse_escape",
                    bounded,
                    command,
                    health,
                    nearest,
                    trajectory=trajectory_check,
                    scale=1.0,
                )
            if self.config.reset_policy is ResetPolicy.AUTO_AFTER_CLEAR and self._release_elapsed(now):
                return self._decision(CollisionState.CLEAR, "auto_released", TwistCommand(), command, health, nearest)
            # TURNING OUT of a front stop is allowed. A latched stop used to zero
            # rotation as well as translation, which left the rover facing an obstacle,
            # unable to drive forward and unable to turn toward the open floor beside
            # it -- observed 2026-08-09 stopped 0.28 m from an obstacle with 0.93 m
            # clear on its left, and it could only sit there until the mission gave up.
            # Forward stays zeroed; only rotation passes, and only when the swept path
            # of that rotation is itself clear, so this cannot turn the flank into
            # something.
            if bounded.angular_z != 0.0:
                # A pivot sweeps a CIRCLE of the footprint's corner radius, in every
                # direction. Both halves of that sentence were got wrong first time:
                #
                #  * the radius was max(front, rear) + margin = 0.180 m, measuring a
                #    rectangle along one axis. The corner is hypot(rear, lateral) =
                #    hypot(0.16, 0.10) = 0.189 m, 0.209 m with the margin -- so
                #    obstacles at 0.181-0.209 m were granted and are inside the sweep.
                #  * it consulted four sectors that leave 60 deg unread, and the
                #    corners sit at +/-42.3 and +/-148.0 deg, squarely in the gaps.
                #    An obstacle at 0.09 m spanning 137-149 deg was granted a pivot.
                #
                # Hence: the circumscribed corner radius, against every bearing. Fails
                # CLOSED -- no reading means no pivot.
                pivot_radius = math.hypot(
                    max(self.config.footprint_front_m, self.config.footprint_rear_m),
                    max(self.config.footprint_left_m, self.config.footprint_right_m),
                ) + self.config.payload_margin_m
                nearest_any = nearest.get("any")
                if nearest_any is not None and nearest_any > pivot_radius:
                    return self._decision(
                        CollisionState.SLOW,
                        "front_stop_turn_escape",
                        TwistCommand(0.0, bounded.angular_z),
                        command,
                        health,
                        nearest,
                        scale=1.0,
                    )
            return self._decision(CollisionState.STOPPED, "reset_required", TwistCommand(), command, health, nearest, reset_required=True)

        if bounded.linear_x > 0.0 and _within(front, front_stop_distance):
            self._clear_since = None
            return self._decision(CollisionState.STOPPED, "front_stop", TwistCommand(), command, health, nearest, reset_required=True)
        if bounded.linear_x < 0.0 and _within(rear, rear_stop_distance):
            output = TwistCommand(0.0, bounded.angular_z)
            return self._decision(CollisionState.SLOW, "rear_hold", output, command, health, nearest)
        if bounded.linear_x != 0.0 or bounded.angular_z != 0.0:
            trajectory_check = evaluate_projected_trajectory(
                self._latest_scan,
                self.config,
                bounded,
            )
            if trajectory_check.blocked:
                if bounded.angular_z > 0.0:
                    reason = "left_trajectory_blocked"
                elif bounded.angular_z < 0.0:
                    reason = "right_trajectory_blocked"
                elif bounded.linear_x > 0.0:
                    reason = "forward_trajectory_blocked"
                else:
                    reason = "reverse_trajectory_blocked"
                return self._decision(
                    CollisionState.SLOW,
                    reason,
                    TwistCommand(),
                    command,
                    health,
                    nearest,
                    trajectory=trajectory_check,
                )
        if bounded.linear_x > 0.0 and front_slow is not None and front_slow < self.config.slow_distance_m:
            scale = self._forward_scale(front_slow)
            output = TwistCommand(bounded.linear_x * scale, bounded.angular_z)
            state = CollisionState.SLOW if output.linear_x > 0.0 else CollisionState.STOPPED
            return self._decision(
                state,
                "front_slow",
                output,
                command,
                health,
                nearest,
                trajectory=trajectory_check,
                scale=scale,
                reset_required=state is CollisionState.STOPPED,
            )
        state = CollisionState.CLEAR if reason in {"command", "scan", "tick", "stale_command", "reset_check", "clear_estop", "clear_estop_noop"} else CollisionState.SLOW
        return self._decision(
            state,
            reason,
            bounded,
            command,
            health,
            nearest,
            trajectory=trajectory_check,
            scale=1.0,
        )

    def _current_scan_eval(self, now: float) -> ScanEvaluation:
        if self._latest_scan is None:
            return _scan_eval(False, "missing_scan", None, 0, 0, "", {})
        self._latest_eval = evaluate_scan(self._latest_scan, self.config, now=now)
        return self._latest_eval

    def _missing_scan_health(self, now: float) -> ScanHealth:
        return ScanHealth(False, "missing_scan", None, 0, 0, "")

    def _command_is_stale(self, now: float) -> bool:
        return self._latest_command_at is not None and now - self._latest_command_at > self.config.requested_cmd_timeout_s

    def _release_clear(self, front: Optional[float], rear: Optional[float], command: TwistCommand) -> bool:
        front_clear = front is not None and front > self.config.release_distance_m
        rear_clear = rear is not None and rear > self.config.reverse_stop_distance_m
        return front_clear and rear_clear

    def _release_elapsed(self, now: float) -> bool:
        return self._clear_since is not None and now - self._clear_since >= self.config.release_time_s

    def _forward_scale(self, distance: float) -> float:
        span = self.config.slow_distance_m - self.config.stop_distance_m
        raw = (distance - self.config.stop_distance_m) / span
        return max(self.config.min_forward_scale, min(1.0, raw))

    def _front_stop_distance(self, command: TwistCommand) -> float:
        dynamic = self.config.footprint_front_m + self.config.payload_margin_m + self._braking_distance(command.linear_x)
        return max(self.config.stop_distance_m, dynamic)

    def _rear_stop_distance(self, command: TwistCommand) -> float:
        dynamic = self.config.footprint_rear_m + self.config.payload_margin_m + self._braking_distance(command.linear_x)
        return max(self.config.reverse_stop_distance_m, dynamic)

    def _braking_distance(self, speed: float) -> float:
        return abs(float(speed)) * self.config.measured_stop_time_s + self.config.braking_distance_margin_m

    def _bound(self, command: TwistCommand) -> TwistCommand:
        linear = max(-self.config.max_forward_mps, min(self.config.max_forward_mps, float(command.linear_x)))
        angular = max(-self.config.max_angular_rad_s, min(self.config.max_angular_rad_s, float(command.angular_z)))
        return TwistCommand(linear, angular)

    def _decision(
        self,
        state: CollisionState,
        reason: str,
        output: TwistCommand,
        requested: TwistCommand,
        health: ScanHealth,
        nearest: Mapping[str, Optional[float]],
        *,
        trajectory: Optional[TrajectoryCheck] = None,
        scale: float = 0.0,
        reset_required: bool = False,
    ) -> ArbitrationDecision:
        previous = self.state
        self.previous_state = previous
        self.state = state
        if (
            state is CollisionState.STOPPED
            and reset_required
            and reason != "reset_required"
            and previous is not CollisionState.STOPPED
        ):
            self._latched_stop_reason = reason
        elif state is not CollisionState.STOPPED:
            self._latched_stop_reason = ""
        self._last_decision = ArbitrationDecision(
            state=state,
            previous_state=previous,
            reason=reason,
            output=output,
            requested=requested,
            scan_health=health,
            nearest=dict(nearest),
            trajectory=trajectory,
            scale=scale,
            reset_required=reset_required,
        )
        return self._last_decision


def _within(distance: Optional[float], threshold: float) -> bool:
    return distance is not None and distance <= threshold


def _twist_is_finite(command: TwistCommand) -> bool:
    try:
        return math.isfinite(float(command.linear_x)) and math.isfinite(float(command.angular_z))
    except (TypeError, ValueError):
        return False
