"""Sphero RVR packet framing and parsing.

This module implements the RVR wire framing used by the direct serial protocol:
SOP/EOP delimiters, byte escaping, one's-complement checksum, dynamic
flag-driven headers, and request/response sequence matching fields.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

# Protocol delimiters and escape codes
SOP = 0x8D
EOP = 0xD8
ESC = 0xAB
ESC_SOP = 0x05
ESC_EOP = 0x50
ESC_ESC = 0x23

# Packet flags
FLAG_IS_RESPONSE = 0x01
FLAG_REQUEST_RESPONSE = 0x02
FLAG_REQUEST_ERROR_ONLY = 0x04
FLAG_IS_ACTIVITY = 0x08
FLAG_HAS_TARGET = 0x10
FLAG_HAS_SOURCE = 0x20

# Common device IDs
DID_SYSTEM_INFO = 0x11
DID_POWER = 0x13
DID_DRIVE = 0x16
DID_SENSOR = 0x18
DID_IO = 0x1A
DID_IR = 0x1C

# Common targets/sources
TARGET_BT = 0x01
TARGET_MCU = 0x02
SOURCE_HOST = 0x00


def checksum(data: bytes) -> int:
    """Return RVR checksum byte for packet content excluding checksum."""
    return (sum(data) & 0xFF) ^ 0xFF


def escape_buffer(data: bytes) -> bytes:
    """Escape protocol delimiter bytes inside packet content."""
    result = bytearray()
    for byte in data:
        if byte == SOP:
            result.extend([ESC, ESC_SOP])
        elif byte == EOP:
            result.extend([ESC, ESC_EOP])
        elif byte == ESC:
            result.extend([ESC, ESC_ESC])
        else:
            result.append(byte)
    return bytes(result)


def unescape_buffer(data: bytes) -> bytes:
    """Undo RVR byte escaping for packet content."""
    result = bytearray()
    index = 0
    while index < len(data):
        byte = data[index]
        if byte != ESC:
            result.append(byte)
            index += 1
            continue

        if index + 1 >= len(data):
            raise ValueError("Dangling escape byte at end of buffer")
        escaped = data[index + 1]
        if escaped == ESC_SOP:
            result.append(SOP)
        elif escaped == ESC_EOP:
            result.append(EOP)
        elif escaped == ESC_ESC:
            result.append(ESC)
        else:
            raise ValueError(f"Invalid escape sequence: {escaped:02x}")
        index += 2
    return bytes(result)


@dataclass(frozen=True)
class Packet:
    """Decoded or ready-to-encode RVR packet.

    `device_id`, `command_id`, and `sequence_id` intentionally keep the names
    used by the first driver skeleton tests while representing RVR DID/CID/SEQ.
    """

    device_id: int
    command_id: int
    sequence_id: int
    payload: bytes = b""
    target: Optional[int] = TARGET_BT
    source: Optional[int] = None
    flags: int = FLAG_HAS_TARGET | FLAG_IS_ACTIVITY
    error: Optional[int] = None

    @property
    def did(self) -> int:
        return self.device_id

    @property
    def cid(self) -> int:
        return self.command_id

    @property
    def seq(self) -> int:
        return self.sequence_id

    def encode(self) -> bytes:
        """Encode this packet as an RVR wire frame."""
        self._validate_byte("device_id", self.device_id)
        self._validate_byte("command_id", self.command_id)
        self._validate_byte("sequence_id", self.sequence_id)
        flags = self.flags

        header = bytearray([flags])
        if flags & FLAG_HAS_TARGET:
            if self.target is None:
                raise ValueError("target is required when FLAG_HAS_TARGET is set")
            self._validate_byte("target", self.target)
            header.append(self.target)
        if flags & FLAG_HAS_SOURCE:
            if self.source is None:
                raise ValueError("source is required when FLAG_HAS_SOURCE is set")
            self._validate_byte("source", self.source)
            header.append(self.source)

        header.extend([self.device_id, self.command_id, self.sequence_id])
        if flags & FLAG_IS_RESPONSE:
            header.append(0 if self.error is None else self.error & 0xFF)

        content = bytes(header) + self.payload
        content_with_checksum = content + bytes([checksum(content)])
        return bytes([SOP]) + escape_buffer(content_with_checksum) + bytes([EOP])

    @classmethod
    def decode(cls, data: bytes) -> "Packet":
        """Decode a complete SOP...EOP RVR wire frame."""
        if len(data) < 2:
            raise ValueError("Packet too short")
        if data[0] != SOP:
            raise ValueError(f"Invalid SOP: {data[0]:#x}")
        try:
            eop_index = data.index(EOP, 1)
        except ValueError as exc:
            raise ValueError("No EOP found") from exc

        content = unescape_buffer(data[1:eop_index])
        if len(content) < 5:
            raise ValueError(f"Packet content too short: {len(content)} bytes")
        if (sum(content) & 0xFF) != 0xFF:
            raise ValueError("Bad checksum")

        body = content[:-1]
        flags = body[0]
        index = 1
        target = None
        source = None

        if flags & FLAG_HAS_TARGET:
            if index >= len(body):
                raise ValueError("Packet too short for target field")
            target = body[index]
            index += 1
        if flags & FLAG_HAS_SOURCE:
            if index >= len(body):
                raise ValueError("Packet too short for source field")
            source = body[index]
            index += 1
        if index + 3 > len(body):
            raise ValueError("Packet too short for did/cid/seq fields")

        did = body[index]
        cid = body[index + 1]
        seq = body[index + 2]
        index += 3

        error = None
        if flags & FLAG_IS_RESPONSE:
            if index >= len(body):
                raise ValueError("Packet too short for response error field")
            error = body[index]
            index += 1

        return cls(
            device_id=did,
            command_id=cid,
            sequence_id=seq,
            payload=body[index:],
            target=target,
            source=source,
            flags=flags,
            error=error,
        )

    @staticmethod
    def _validate_byte(name: str, value: int) -> None:
        if not 0 <= value <= 0xFF:
            raise ValueError(f"{name} must fit in one byte")


def get_packet_header(packet: bytes) -> Tuple[int, int, int]:
    """Return `(did, cid, seq)` from a complete encoded packet."""
    parsed = Packet.decode(packet)
    return parsed.device_id, parsed.command_id, parsed.sequence_id
