"""Map ROS Twist-like commands into core driver velocities."""

from dataclasses import dataclass

from sphero_rvr_core.safety import clamp
from sphero_rvr_core.state import VelocityCommand


@dataclass(frozen=True)
class TwistLike:
    linear_x: float
    angular_z: float


def map_twist_to_velocity(twist: TwistLike, max_linear_mps: float, max_angular_rad_s: float) -> VelocityCommand:
    return VelocityCommand(
        linear_mps=clamp(twist.linear_x, -max_linear_mps, max_linear_mps),
        angular_rad_s=clamp(twist.angular_z, -max_angular_rad_s, max_angular_rad_s),
    )
