"""High-level concurrency-safe RVR driver."""

import asyncio
from typing import Optional

from .command_queue import CommandPriority, PriorityCommandQueue
from .commands import RVRCommands
from .dispatcher import Dispatcher
from .safety import clamp_velocity, is_stale, now_seconds
from .state import RVRState, VelocityCommand
from .transport import Transport


class RVRDriver:
    def __init__(
        self,
        transport: Transport,
        control_period: float = 0.05,
        command_timeout: float = 0.5,
        max_linear_mps: float = 1.0,
        max_angular_rad_s: float = 3.0,
    ):
        self.commands = RVRCommands()
        self._dispatcher = Dispatcher(transport)
        self._queue = PriorityCommandQueue()
        self._control_period = control_period
        self._command_timeout = command_timeout
        self._max_linear_mps = max_linear_mps
        self._max_angular_rad_s = max_angular_rad_s
        self._desired_velocity: Optional[VelocityCommand] = None
        self._last_velocity_update: Optional[float] = None
        self._connected = False
        self._emergency_stopped = False
        self._control_task: Optional[asyncio.Task] = None
        self._sequence_id = 0

    async def connect(self) -> None:
        await self._dispatcher.start()
        await self._queue.start()
        await self._send(self.commands.connect, CommandPriority.HIGH)
        self._connected = True
        self._control_task = asyncio.create_task(self._control_loop())

    async def disconnect(self) -> None:
        if self._control_task is not None:
            self._control_task.cancel()
            try:
                await self._control_task
            except asyncio.CancelledError:
                pass
            self._control_task = None
        if self._connected and not self._emergency_stopped:
            try:
                await self.stop()
            except Exception:
                pass
        await self._queue.stop()
        await self._dispatcher.stop()
        self._connected = False

    async def set_velocity(self, linear_mps: float, angular_rad_s: float) -> None:
        self._desired_velocity = clamp_velocity(
            VelocityCommand(linear_mps, angular_rad_s),
            max_linear_mps=self._max_linear_mps,
            max_angular_rad_s=self._max_angular_rad_s,
        )
        self._last_velocity_update = now_seconds()

    async def stop(self) -> None:
        self._desired_velocity = None
        self._last_velocity_update = None
        await self._send(self.commands.stop, CommandPriority.HIGH)

    async def emergency_stop(self) -> None:
        self._emergency_stopped = True
        self._desired_velocity = None
        self._last_velocity_update = None
        await self._send(self.commands.emergency_stop, CommandPriority.EMERGENCY)

    async def clear_emergency_stop(self) -> None:
        await self._send(self.commands.clear_emergency_stop, CommandPriority.HIGH)
        self._emergency_stopped = False

    def get_state(self) -> RVRState:
        return RVRState(
            connected=self._connected,
            emergency_stopped=self._emergency_stopped,
            latest_velocity=self._desired_velocity,
        )

    async def _control_loop(self) -> None:
        stop_sent_for_stale = False
        while True:
            await asyncio.sleep(self._control_period)
            if self._emergency_stopped:
                continue
            if self._desired_velocity is None:
                continue
            if is_stale(self._last_velocity_update, self._command_timeout):
                if not stop_sent_for_stale:
                    await self.stop()
                    stop_sent_for_stale = True
                continue
            stop_sent_for_stale = False
            velocity = self._desired_velocity
            await self._send(
                lambda seq: self.commands.drive_rc(seq, velocity.linear_mps, velocity.angular_rad_s),
                CommandPriority.NORMAL,
            )

    async def _send(self, packet_factory, priority: CommandPriority):
        sequence_id = self._next_sequence_id()
        packet = packet_factory(sequence_id)
        return await self._queue.submit(lambda: self._dispatcher.request(packet), priority=priority)

    def _next_sequence_id(self) -> int:
        self._sequence_id = (self._sequence_id + 1) % 256
        return self._sequence_id
