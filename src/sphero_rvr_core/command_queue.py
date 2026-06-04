"""Priority command queue for single-writer hardware access."""

import asyncio
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Awaitable, Callable, Optional, TypeVar

T = TypeVar("T")


class CommandPriority(IntEnum):
    EMERGENCY = 0
    HIGH = 1
    NORMAL = 2
    LOW = 3


@dataclass(order=True)
class _QueuedCommand:
    priority: int
    sequence: int
    action: Callable[[], Awaitable[object]] = field(compare=False)
    future: asyncio.Future = field(compare=False)


class PriorityCommandQueue:
    def __init__(self):
        self._queue: asyncio.PriorityQueue[_QueuedCommand] = asyncio.PriorityQueue()
        self._task: Optional[asyncio.Task] = None
        self._sequence = 0
        self._stopped = asyncio.Event()

    async def start(self) -> None:
        if self._task is not None:
            return
        self._stopped.clear()
        self._task = asyncio.create_task(self._worker())

    async def stop(self) -> None:
        if self._task is None:
            return
        self._task.cancel()
        try:
            await self._task
        except asyncio.CancelledError:
            pass
        self._task = None
        while not self._queue.empty():
            queued = self._queue.get_nowait()
            if not queued.future.done():
                queued.future.cancel()

    async def submit(self, action: Callable[[], Awaitable[T]], priority: CommandPriority = CommandPriority.NORMAL) -> T:
        if self._task is None:
            raise RuntimeError("command queue is not started")
        loop = asyncio.get_running_loop()
        future: asyncio.Future = loop.create_future()
        self._sequence += 1
        await self._queue.put(_QueuedCommand(int(priority), self._sequence, action, future))
        return await future

    async def _worker(self) -> None:
        while True:
            queued = await self._queue.get()
            if queued.future.cancelled():
                continue
            try:
                result = await queued.action()
            except Exception as exc:
                if not queued.future.done():
                    queued.future.set_exception(exc)
            else:
                if not queued.future.done():
                    queued.future.set_result(result)
