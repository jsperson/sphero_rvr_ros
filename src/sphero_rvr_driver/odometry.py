"""Differential-drive odometry helpers kept free of ROS imports for testing."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Optional, Tuple

from sphero_rvr_core.responses import EncoderCounts

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
