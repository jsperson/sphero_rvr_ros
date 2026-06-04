"""Minimal packet model used by the dispatcher tests.

The wire format here is deliberately small and internal for the first slice.
The real RVR packet codec will replace/extend this module once the serial
concurrency substrate is proven.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Packet:
    device_id: int
    command_id: int
    sequence_id: int
    payload: bytes = b""

    def encode(self) -> bytes:
        if not 0 <= self.device_id <= 255:
            raise ValueError("device_id must fit in one byte")
        if not 0 <= self.command_id <= 255:
            raise ValueError("command_id must fit in one byte")
        if not 0 <= self.sequence_id <= 255:
            raise ValueError("sequence_id must fit in one byte")
        if len(self.payload) > 255:
            raise ValueError("payload too large for initial packet format")
        return bytes([0x8D, self.device_id, self.command_id, self.sequence_id, len(self.payload)]) + self.payload

    @classmethod
    def decode(cls, data: bytes) -> "Packet":
        if len(data) < 5:
            raise ValueError("packet too short")
        if data[0] != 0x8D:
            raise ValueError("invalid packet start byte")
        payload_len = data[4]
        expected_len = 5 + payload_len
        if len(data) != expected_len:
            raise ValueError(f"invalid packet length: expected {expected_len}, got {len(data)}")
        return cls(
            device_id=data[1],
            command_id=data[2],
            sequence_id=data[3],
            payload=data[5:],
        )
