"""Driver state models."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class VelocityCommand:
    linear_mps: float
    angular_rad_s: float


@dataclass(frozen=True)
class RVRState:
    connected: bool = False
    emergency_stopped: bool = False
    latest_velocity: Optional[VelocityCommand] = None
    fail_safe_active: bool = False
    fail_safe_reason: Optional[str] = None
    motor_transport_write_count: int = 0
    motion_transport_write_count: int = 0
    last_motor_command_id: Optional[int] = None
    last_motor_sequence_id: Optional[int] = None
    last_motor_payload_hex: Optional[str] = None
    last_motor_transport_write_epoch_s: Optional[float] = None
    last_motion_transport_write_epoch_s: Optional[float] = None
    motor_stall_triggered: bool = False
    motor_fault: bool = False
