"""Request/response dispatcher with one serial reader loop."""

import asyncio
from typing import Dict, Optional, Tuple

from .packet import Packet
from .transport import Transport


class Dispatcher:
    def __init__(self, transport: Transport):
        self._transport = transport
        self._pending: Dict[Tuple[int, int, int], asyncio.Future[Packet]] = {}
        self._reader_task: Optional[asyncio.Task] = None
        self._write_lock = asyncio.Lock()
        self._started = False

    async def start(self) -> None:
        if self._started:
            return
        await self._transport.open()
        self._started = True
        self._reader_task = asyncio.create_task(self._read_loop())

    async def stop(self) -> None:
        if self._reader_task is not None:
            self._reader_task.cancel()
            try:
                await self._reader_task
            except asyncio.CancelledError:
                pass
            self._reader_task = None
        for future in list(self._pending.values()):
            if not future.done():
                future.cancel()
        self._pending.clear()
        await self._transport.close()
        self._started = False

    async def request(self, packet: Packet, timeout: float = 1.0) -> Packet:
        if not self._started:
            raise RuntimeError("dispatcher is not started")
        loop = asyncio.get_running_loop()
        key = self._key(packet)
        future: asyncio.Future[Packet] = loop.create_future()
        self._pending[key] = future
        try:
            async with self._write_lock:
                await self._transport.write(packet.encode())
            return await asyncio.wait_for(future, timeout=timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(f"timed out waiting for response to sequence {packet.sequence_id}")
        finally:
            self._pending.pop(key, None)

    async def send(self, packet: Packet) -> None:
        """Send a packet that does not require a response.

        Fire-and-forget commands still go through the dispatcher's single write
        lock so they serialize cleanly with request/response traffic, but they
        do not create a pending future that can hang waiting for an ACK the RVR
        firmware never promised to send.
        """
        if not self._started:
            raise RuntimeError("dispatcher is not started")
        async with self._write_lock:
            await self._transport.write(packet.encode())

    async def _read_loop(self) -> None:
        while True:
            raw = await self._transport.read_packet()
            packet = Packet.decode(raw)
            future = self._pending.get(self._key(packet))
            if future is not None and not future.done():
                future.set_result(packet)

    @staticmethod
    def _key(packet: Packet) -> Tuple[int, int, int]:
        return packet.device_id, packet.command_id, packet.sequence_id
