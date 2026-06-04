"""Initial high-level command identifiers.

These IDs are internal placeholders for the first concurrency slice. The real
RVR packet IDs will be mapped here as hardware support lands.
"""

from dataclasses import dataclass
import struct

from .packet import Packet


@dataclass(frozen=True)
class RVRCommands:
    DEVICE: int = 1
    CONNECT: int = 1
    STOP: int = 2
    DRIVE_RC: int = 3
    EMERGENCY_STOP: int = 4
    CLEAR_EMERGENCY_STOP: int = 5

    def connect(self, sequence_id: int) -> Packet:
        return Packet(self.DEVICE, self.CONNECT, sequence_id)

    def stop(self, sequence_id: int) -> Packet:
        return Packet(self.DEVICE, self.STOP, sequence_id)

    def emergency_stop(self, sequence_id: int) -> Packet:
        return Packet(self.DEVICE, self.EMERGENCY_STOP, sequence_id)

    def clear_emergency_stop(self, sequence_id: int) -> Packet:
        return Packet(self.DEVICE, self.CLEAR_EMERGENCY_STOP, sequence_id)

    def drive_rc(self, sequence_id: int, linear_mps: float, angular_rad_s: float) -> Packet:
        payload = struct.pack(">ff", float(linear_mps), float(angular_rad_s))
        return Packet(self.DEVICE, self.DRIVE_RC, sequence_id, payload)
