"""Request/response dispatcher with one serial reader loop.

Besides ordinary request/response matching, the dispatcher owns local routing for
unsolicited RVR command packets (SDK "notifications"). Response futures are
matched only by the real correlation fields `(did, cid, seq)`; unmatched
command packets can be parsed, cached, and delivered to notification
subscribers keyed by `(did, cid, source)` with a source-agnostic fallback.
"""

import asyncio
import inspect
import logging
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Set, Tuple

from . import responses
from .packet import (
    DID_DRIVE,
    DID_POWER,
    DID_SENSOR,
    FLAG_IS_RESPONSE,
    Packet,
    TARGET_BT,
    TARGET_MCU,
)
from .transport import Transport

NotificationKey = Tuple[int, int, Optional[int]]
NotificationParser = Callable[[bytes], Any]
NotificationCallback = Callable[[Any], Any]

LOGGER = logging.getLogger(__name__)

# SDK notification/event packet IDs from docs/rvr_notification_events.md.  These
# are the emitted event CIDs, not the enable/disable command CIDs.
EVENT_WILL_SLEEP = (DID_POWER, 0x19, TARGET_BT)
EVENT_DID_SLEEP = (DID_POWER, 0x1A, TARGET_BT)
EVENT_BATTERY_VOLTAGE_STATE_CHANGE = (DID_POWER, 0x1C, TARGET_BT)
EVENT_MOTOR_STALL = (DID_DRIVE, 0x26, TARGET_MCU)
EVENT_MOTOR_FAULT = (DID_DRIVE, 0x28, TARGET_MCU)
EVENT_GYRO_MAX = (DID_SENSOR, 0x10, TARGET_MCU)
EVENT_IR_MESSAGE_RECEIVED = (DID_SENSOR, 0x2C, TARGET_MCU)
EVENT_COLOR_DETECTION = (DID_SENSOR, 0x36, TARGET_BT)
EVENT_STREAMING_SERVICE_DATA_BT = (DID_SENSOR, 0x3D, TARGET_BT)
EVENT_STREAMING_SERVICE_DATA_MCU = (DID_SENSOR, 0x3D, TARGET_MCU)
EVENT_MOTOR_THERMAL_PROTECTION_STATUS = (DID_SENSOR, 0x4D, TARGET_MCU)

EVENT_PARSERS: Dict[NotificationKey, NotificationParser] = {
    EVENT_WILL_SLEEP: responses.parse_will_sleep_event,
    EVENT_DID_SLEEP: responses.parse_did_sleep_event,
    EVENT_BATTERY_VOLTAGE_STATE_CHANGE: responses.parse_battery_voltage_state,
    EVENT_MOTOR_STALL: responses.parse_motor_stall_event,
    EVENT_MOTOR_FAULT: responses.parse_motor_fault_event,
    EVENT_GYRO_MAX: responses.parse_gyro_max_event,
    EVENT_IR_MESSAGE_RECEIVED: responses.parse_infrared_message_event,
    EVENT_COLOR_DETECTION: responses.parse_current_detected_color,
    EVENT_STREAMING_SERVICE_DATA_BT: responses.parse_streaming_service_data,
    EVENT_STREAMING_SERVICE_DATA_MCU: responses.parse_streaming_service_data,
    EVENT_MOTOR_THERMAL_PROTECTION_STATUS: responses.parse_thermal_protection_status,
}

# Streaming data is intentionally not globally cached; it can be high-rate and
# token-scoped. Subscribers still receive it.
CACHEABLE_EVENTS: Set[NotificationKey] = set(EVENT_PARSERS) - {
    EVENT_STREAMING_SERVICE_DATA_BT,
    EVENT_STREAMING_SERVICE_DATA_MCU,
}


@dataclass(frozen=True)
class Subscription:
    """Handle returned by :meth:`Dispatcher.subscribe`."""

    _unsubscribe: Callable[[], None]

    def unsubscribe(self) -> None:
        self._unsubscribe()


@dataclass(frozen=True)
class _Subscriber:
    callback: NotificationCallback
    parser: Optional[NotificationParser]


class Dispatcher:
    def __init__(self, transport: Transport):
        self._transport = transport
        self._pending: Dict[Tuple[int, int, int], asyncio.Future[Packet]] = {}
        self._subscribers: Dict[NotificationKey, Set[_Subscriber]] = {}
        self._event_cache: Dict[NotificationKey, Any] = {}
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
        self._subscribers.clear()
        self._event_cache.clear()
        await self._transport.close()
        self._started = False

    def subscribe(
        self,
        device_id: int,
        command_id: int,
        source: Optional[int],
        callback: NotificationCallback,
        *,
        parser: Optional[NotificationParser] = None,
    ) -> Subscription:
        """Subscribe to unsolicited notification packets.

        `source=None` registers a source-agnostic fallback for packets with the
        same DID/CID. If `parser` is omitted, built-in SDK event parsers are used
        when the DID/CID/source is known; otherwise callbacks receive the raw
        :class:`Packet`.
        """
        key = (device_id, command_id, source)
        subscriber = _Subscriber(callback=callback, parser=parser)
        self._subscribers.setdefault(key, set()).add(subscriber)

        def _unsubscribe() -> None:
            subscribers = self._subscribers.get(key)
            if subscribers is None:
                return
            subscribers.discard(subscriber)
            if not subscribers:
                self._subscribers.pop(key, None)

        return Subscription(_unsubscribe)

    def get_cached_event(self, device_id: int, command_id: int, source: Optional[int] = None) -> Any:
        """Return the latest cacheable notification value for a DID/CID/source.

        If `source` is omitted and there is exactly one cached source for the
        DID/CID, that value is returned. `None` means no cached value exists.
        """
        if source is not None:
            return self._event_cache.get((device_id, command_id, source))
        matches = [value for (did, cid, _source), value in self._event_cache.items() if did == device_id and cid == command_id]
        if len(matches) == 1:
            return matches[0]
        return None

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
            try:
                packet = Packet.decode(raw)
            except Exception:
                LOGGER.exception("dropping malformed RVR packet")
                continue

            future = self._pending.get(self._key(packet))
            if packet.flags & FLAG_IS_RESPONSE:
                if future is not None and not future.done():
                    future.set_result(packet)
                else:
                    LOGGER.debug(
                        "dropping unmatched RVR response did=0x%02x cid=0x%02x seq=%s source=%s",
                        packet.device_id,
                        packet.command_id,
                        packet.sequence_id,
                        packet.source,
                    )
                continue

            if self._route_notification(packet):
                continue

            LOGGER.debug(
                "dropping unmatched RVR packet did=0x%02x cid=0x%02x seq=%s source=%s flags=0x%02x",
                packet.device_id,
                packet.command_id,
                packet.sequence_id,
                packet.source,
                packet.flags,
            )

    def _route_notification(self, packet: Packet) -> bool:
        source_key = (packet.device_id, packet.command_id, packet.source)
        fallback_key = (packet.device_id, packet.command_id, None)
        subscribers = tuple(self._subscribers.get(source_key, ())) + tuple(self._subscribers.get(fallback_key, ()))
        parser = self._parser_for(source_key, fallback_key, subscribers)

        if parser is None and not subscribers:
            return False

        try:
            event = parser(packet.payload) if parser is not None else packet
        except Exception:
            LOGGER.exception(
                "dropping RVR notification with invalid payload did=0x%02x cid=0x%02x source=%s",
                packet.device_id,
                packet.command_id,
                packet.source,
            )
            return True

        if source_key in CACHEABLE_EVENTS:
            self._event_cache[source_key] = event

        for subscriber in subscribers:
            subscriber_event = event
            if subscriber.parser is not None and subscriber.parser is not parser:
                try:
                    subscriber_event = subscriber.parser(packet.payload)
                except Exception:
                    LOGGER.exception(
                        "dropping RVR notification for subscriber with invalid payload did=0x%02x cid=0x%02x source=%s",
                        packet.device_id,
                        packet.command_id,
                        packet.source,
                    )
                    continue
            self._schedule_callback(subscriber.callback, subscriber_event)
        return True

    @staticmethod
    def _parser_for(
        source_key: NotificationKey,
        fallback_key: NotificationKey,
        subscribers: Tuple[_Subscriber, ...],
    ) -> Optional[NotificationParser]:
        if source_key in EVENT_PARSERS:
            return EVENT_PARSERS[source_key]
        if fallback_key in EVENT_PARSERS:
            return EVENT_PARSERS[fallback_key]
        for subscriber in subscribers:
            if subscriber.parser is not None:
                return subscriber.parser
        return None

    @staticmethod
    def _schedule_callback(callback: NotificationCallback, event: Any) -> None:
        async def _invoke() -> None:
            try:
                result = callback(event)
                if inspect.isawaitable(result):
                    await result
            except Exception:
                LOGGER.exception("RVR notification subscriber failed")

        asyncio.create_task(_invoke())

    @staticmethod
    def _key(packet: Packet) -> Tuple[int, int, int]:
        return packet.device_id, packet.command_id, packet.sequence_id
