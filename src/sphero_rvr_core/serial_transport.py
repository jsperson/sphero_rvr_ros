"""Async wrapper around pyserial for packet-oriented transport.

This is intentionally thin. The packet framing still lives in `packet.py`; this
class just owns the blocking serial object and exposes async open/close/write/read.
"""

import asyncio
from typing import Optional

import serial

from .packet import EOP, SOP


class SerialTransport:
    def __init__(self, port: str = "/dev/ttyAMA0", baud_rate: int = 115200, read_timeout: float = 0.1):
        self.port = port
        self.baud_rate = baud_rate
        self.read_timeout = read_timeout
        self._serial: Optional[serial.Serial] = None

    async def open(self) -> None:
        if self._serial is not None and self._serial.is_open:
            return
        self._serial = await asyncio.to_thread(
            serial.Serial, self.port, self.baud_rate, timeout=self.read_timeout
        )

    async def close(self) -> None:
        if self._serial is not None:
            await asyncio.to_thread(self._serial.close)
            self._serial = None

    async def write(self, data: bytes) -> None:
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("serial transport is closed")
        await asyncio.to_thread(self._serial.write, data)
        await asyncio.to_thread(self._serial.flush)

    async def read_packet(self) -> bytes:
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("serial transport is closed")

        def _read_one_packet() -> bytes:
            in_frame = False
            frame = bytearray()

            while True:
                byte = self._serial.read(1)
                if byte == b"":
                    if in_frame:
                        raise TimeoutError("timed out reading complete packet")
                    continue

                value = byte[0]
                if not in_frame:
                    if value == SOP:
                        in_frame = True
                        frame.append(value)
                    continue

                frame.append(value)
                if value == EOP:
                    return bytes(frame)

        return await asyncio.to_thread(_read_one_packet)
