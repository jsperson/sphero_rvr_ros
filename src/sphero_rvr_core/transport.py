"""Transport abstractions for RVR serial I/O."""

from typing import Protocol


class Transport(Protocol):
    async def open(self) -> None:
        ...

    async def close(self) -> None:
        ...

    async def write(self, data: bytes) -> None:
        ...

    async def read_packet(self) -> bytes:
        ...
