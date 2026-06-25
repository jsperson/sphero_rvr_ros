"""Fake transport for deterministic concurrency tests."""

import asyncio
from typing import List

from .packet import FLAG_HAS_SOURCE, Packet


def unsolicited_packet(did: int, cid: int, source: int, payload: bytes = b"", seq: int = 0x7F) -> bytes:
    """Build an unsolicited device-to-host notification/event fixture."""
    return Packet(
        did,
        cid,
        seq,
        payload=payload,
        target=None,
        source=source,
        flags=FLAG_HAS_SOURCE,
    ).encode()


class FakeTransport:
    def __init__(self, auto_ack: bool = False):
        self.auto_ack = auto_ack
        self.is_open = False
        self.writes: List[bytes] = []
        self._reads: asyncio.Queue[bytes] = asyncio.Queue()
        self._write_event = asyncio.Event()

    async def open(self) -> None:
        self.is_open = True

    async def close(self) -> None:
        self.is_open = False

    async def write(self, data: bytes) -> None:
        if not self.is_open:
            raise RuntimeError("transport is closed")
        self.writes.append(data)
        self._write_event.set()
        if self.auto_ack:
            packet = Packet.decode(data)
            await self.inject_read(Packet(packet.device_id, packet.command_id, packet.sequence_id, b"ack").encode())

    async def read_packet(self) -> bytes:
        return await self._reads.get()

    async def inject_read(self, data: bytes) -> None:
        await self._reads.put(data)

    async def wait_for_write(self, timeout: float = 0.2) -> None:
        await asyncio.wait_for(self._write_event.wait(), timeout=timeout)
        self._write_event.clear()
