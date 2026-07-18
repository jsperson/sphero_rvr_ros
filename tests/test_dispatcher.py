import asyncio
import struct

import pytest

from sphero_rvr_core import responses
from sphero_rvr_core.dispatcher import (
    CACHEABLE_EVENTS,
    EVENT_BATTERY_VOLTAGE_STATE_CHANGE,
    EVENT_COLOR_DETECTION,
    EVENT_DID_SLEEP,
    EVENT_GYRO_MAX,
    EVENT_IR_MESSAGE_RECEIVED,
    EVENT_MOTOR_FAULT,
    EVENT_MOTOR_STALL,
    EVENT_MOTOR_THERMAL_PROTECTION_STATUS,
    EVENT_PARSERS,
    EVENT_STREAMING_SERVICE_DATA_BT,
    EVENT_STREAMING_SERVICE_DATA_MCU,
    EVENT_WILL_SLEEP,
    Dispatcher,
)
from sphero_rvr_core.fake_transport import FakeTransport, unsolicited_packet
from sphero_rvr_core.packet import DID_DRIVE, FLAG_HAS_SOURCE, FLAG_IS_RESPONSE, Packet, TARGET_MCU


class FlakyReadTransport(FakeTransport):
    def __init__(self, failures: list[Exception], *, auto_ack: bool = False):
        super().__init__(auto_ack=auto_ack)
        self.failures = list(failures)

    async def read_packet(self) -> bytes:
        if self.failures:
            raise self.failures.pop(0)
        return await super().read_packet()


@pytest.mark.asyncio
async def test_dispatcher_matches_response_by_sequence_id():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    await dispatcher.start()

    request = Packet(device_id=1, command_id=2, sequence_id=7, payload=b"go")
    pending = asyncio.create_task(dispatcher.request(request, timeout=0.2))
    await transport.wait_for_write()

    await transport.inject_read(Packet(
        device_id=1,
        command_id=2,
        sequence_id=7,
        payload=b"ok",
        flags=FLAG_IS_RESPONSE | FLAG_HAS_SOURCE,
        source=1,
        error=0,
    ).encode())

    response = await pending
    await dispatcher.stop()

    assert response.payload == b"ok"
    assert transport.writes == [request.encode()]


@pytest.mark.asyncio
async def test_dispatcher_times_out_missing_response_without_hanging():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    await dispatcher.start()

    with pytest.raises(TimeoutError):
        await dispatcher.request(Packet(device_id=1, command_id=2, sequence_id=8), timeout=0.01)

    await dispatcher.stop()


@pytest.mark.asyncio
async def test_dispatcher_send_writes_without_waiting_for_response():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    await dispatcher.start()

    packet = Packet(device_id=1, command_id=2, sequence_id=9, payload=b"go")
    await asyncio.wait_for(dispatcher.send(packet), timeout=0.05)

    await dispatcher.stop()
    assert transport.writes == [packet.encode()]


@pytest.mark.asyncio
async def test_unsolicited_event_without_pending_request_invokes_subscribers_and_cache():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    received = asyncio.Queue()
    dispatcher.subscribe(*EVENT_MOTOR_FAULT, received.put_nowait)
    await dispatcher.start()

    await transport.inject_read(unsolicited_packet(*EVENT_MOTOR_FAULT, payload=b"\x01"))

    event = await asyncio.wait_for(received.get(), timeout=0.2)
    cached = dispatcher.get_cached_event(*EVENT_MOTOR_FAULT)
    await dispatcher.stop()

    assert event == responses.MotorFaultEvent(is_fault=True)
    assert cached == event
    assert dispatcher.get_cached_event(*EVENT_MOTOR_FAULT) is None  # cache cleared on stop


@pytest.mark.asyncio
async def test_unsolicited_event_does_not_complete_pending_request_with_same_did_cid_sequence():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    received = asyncio.Queue()
    dispatcher.subscribe(DID_DRIVE, 0x26, TARGET_MCU, received.put_nowait)
    await dispatcher.start()

    request = Packet(device_id=DID_DRIVE, command_id=0x26, sequence_id=7, payload=b"request")
    pending = asyncio.create_task(dispatcher.request(request, timeout=0.3))
    await transport.wait_for_write()

    await transport.inject_read(unsolicited_packet(DID_DRIVE, 0x26, TARGET_MCU, payload=b"\x00\x01", seq=7))
    event = await asyncio.wait_for(received.get(), timeout=0.2)
    assert event == responses.MotorStallEvent(motor_index=0, is_triggered=True)
    assert not pending.done()

    await transport.inject_read(Packet(
        device_id=DID_DRIVE,
        command_id=0x26,
        sequence_id=7,
        payload=b"ok",
        flags=FLAG_IS_RESPONSE | FLAG_HAS_SOURCE,
        source=TARGET_MCU,
        error=0,
    ).encode())
    response = await pending
    await dispatcher.stop()

    assert response.payload == b"ok"


@pytest.mark.asyncio
async def test_unmatched_response_packet_is_not_routed_as_notification():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    received = asyncio.Queue()
    dispatcher.subscribe(*EVENT_MOTOR_FAULT, received.put_nowait)
    await dispatcher.start()

    await transport.inject_read(Packet(
        device_id=EVENT_MOTOR_FAULT[0],
        command_id=EVENT_MOTOR_FAULT[1],
        sequence_id=1,
        payload=b"\x01",
        flags=FLAG_IS_RESPONSE | FLAG_HAS_SOURCE,
        source=EVENT_MOTOR_FAULT[2],
        error=0,
    ).encode())

    with pytest.raises(asyncio.TimeoutError):
        await asyncio.wait_for(received.get(), timeout=0.02)
    await dispatcher.stop()


@pytest.mark.asyncio
async def test_pending_response_still_completes_when_event_arrives_before_response():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    received = asyncio.Queue()
    dispatcher.subscribe(*EVENT_COLOR_DETECTION, received.put_nowait)
    await dispatcher.start()

    request = Packet(device_id=1, command_id=2, sequence_id=3, payload=b"go")
    pending = asyncio.create_task(dispatcher.request(request, timeout=0.3))
    await transport.wait_for_write()

    await transport.inject_read(unsolicited_packet(*EVENT_COLOR_DETECTION, payload=bytes([10, 20, 30, 99, 0xFF])))
    await transport.inject_read(Packet(
        device_id=1,
        command_id=2,
        sequence_id=3,
        payload=b"ok",
        flags=FLAG_IS_RESPONSE | FLAG_HAS_SOURCE,
        source=1,
        error=0,
    ).encode())

    event = await asyncio.wait_for(received.get(), timeout=0.2)
    response = await pending
    cached = dispatcher.get_cached_event(*EVENT_COLOR_DETECTION)
    await dispatcher.stop()

    assert event == responses.DetectedColor(10, 20, 30, 99, 0xFF)
    assert cached == event
    assert response.payload == b"ok"


@pytest.mark.asyncio
async def test_multiple_subscribers_fallback_and_unsubscribe_receive_parsed_event():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    exact = asyncio.Queue()
    fallback = asyncio.Queue()
    removed = asyncio.Queue()
    dispatcher.subscribe(*EVENT_MOTOR_STALL, exact.put_nowait)
    dispatcher.subscribe(EVENT_MOTOR_STALL[0], EVENT_MOTOR_STALL[1], None, fallback.put_nowait)
    handle = dispatcher.subscribe(*EVENT_MOTOR_STALL, removed.put_nowait)
    handle.unsubscribe()
    await dispatcher.start()

    await transport.inject_read(unsolicited_packet(*EVENT_MOTOR_STALL, payload=b"\x01\x00"))

    expected = responses.MotorStallEvent(motor_index=1, is_triggered=False)
    assert await asyncio.wait_for(exact.get(), timeout=0.2) == expected
    assert await asyncio.wait_for(fallback.get(), timeout=0.2) == expected
    with pytest.raises(asyncio.TimeoutError):
        await asyncio.wait_for(removed.get(), timeout=0.02)
    await dispatcher.stop()


@pytest.mark.asyncio
async def test_callback_exception_is_logged_without_killing_read_loop(caplog):
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    received = asyncio.Queue()

    def bad_callback(_event):
        raise RuntimeError("boom")

    dispatcher.subscribe(*EVENT_GYRO_MAX, bad_callback)
    dispatcher.subscribe(*EVENT_GYRO_MAX, received.put_nowait)
    await dispatcher.start()

    await transport.inject_read(unsolicited_packet(*EVENT_GYRO_MAX, payload=b"\x07"))
    assert await asyncio.wait_for(received.get(), timeout=0.2) == responses.GyroMaxEvent(flags=7)
    await asyncio.sleep(0)
    await dispatcher.stop()

    assert "RVR notification subscriber failed" in caplog.text


@pytest.mark.asyncio
async def test_parser_error_is_logged_and_reader_keeps_running(caplog):
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    received = asyncio.Queue()
    dispatcher.subscribe(*EVENT_MOTOR_STALL, received.put_nowait)
    await dispatcher.start()

    await transport.inject_read(unsolicited_packet(*EVENT_MOTOR_STALL, payload=b"\x01"))
    await transport.inject_read(unsolicited_packet(*EVENT_MOTOR_STALL, payload=b"\x01\x01"))

    assert await asyncio.wait_for(received.get(), timeout=0.2) == responses.MotorStallEvent(1, True)
    await dispatcher.stop()

    assert "dropping RVR notification with invalid payload" in caplog.text


@pytest.mark.asyncio
async def test_read_timeout_is_logged_and_reader_recovers_for_next_packet(caplog):
    transport = FlakyReadTransport([TimeoutError("mid-frame timeout")])
    dispatcher = Dispatcher(transport, read_error_retry_limit=3, read_error_backoff=0)
    received = asyncio.Queue()
    dispatcher.subscribe(*EVENT_MOTOR_FAULT, received.put_nowait)
    await dispatcher.start()

    await transport.inject_read(unsolicited_packet(*EVENT_MOTOR_FAULT, payload=b"\x01"))

    assert await asyncio.wait_for(received.get(), timeout=0.2) == responses.MotorFaultEvent(True)
    await dispatcher.stop()
    assert "RVR transport read failed" in caplog.text


@pytest.mark.asyncio
async def test_read_exception_does_not_prevent_later_request_response(caplog):
    transport = FlakyReadTransport([RuntimeError("serial disconnect blip")])
    dispatcher = Dispatcher(transport, read_error_retry_limit=3, read_error_backoff=0)
    await dispatcher.start()

    request = Packet(device_id=1, command_id=2, sequence_id=9, payload=b"go")
    pending = asyncio.create_task(dispatcher.request(request, timeout=0.3))
    await transport.wait_for_write()
    await transport.inject_read(Packet(
        device_id=1,
        command_id=2,
        sequence_id=9,
        payload=b"ok",
        flags=FLAG_IS_RESPONSE | FLAG_HAS_SOURCE,
        source=1,
        error=0,
    ).encode())

    assert (await pending).payload == b"ok"
    await dispatcher.stop()
    assert "RVR transport read failed" in caplog.text


@pytest.mark.asyncio
async def test_persistent_read_failures_fault_pending_requests_without_hot_spin(caplog):
    transport = FlakyReadTransport([
        RuntimeError("disconnect 1"),
        RuntimeError("disconnect 2"),
    ])
    dispatcher = Dispatcher(transport, read_error_retry_limit=2, read_error_backoff=0.05)
    await dispatcher.start()

    request = Packet(device_id=1, command_id=2, sequence_id=10, payload=b"go")
    pending = asyncio.create_task(dispatcher.request(request, timeout=1.0))
    await transport.wait_for_write()

    with pytest.raises(RuntimeError, match="RVR transport reader stopped"):
        await pending
    with pytest.raises(RuntimeError, match="dispatcher reader failed"):
        await dispatcher.request(Packet(device_id=1, command_id=2, sequence_id=11), timeout=0.1)

    await dispatcher.stop()
    assert "RVR transport read failed persistently" in caplog.text


@pytest.mark.parametrize(
    ("event_key", "payload", "expected"),
    [
        (EVENT_WILL_SLEEP, b"", responses.SleepEvent("will_sleep")),
        (EVENT_DID_SLEEP, b"", responses.SleepEvent("did_sleep")),
        (EVENT_BATTERY_VOLTAGE_STATE_CHANGE, b"\x03", responses.BatteryVoltageState(3, "critical")),
        (EVENT_MOTOR_STALL, b"\x02\x01", responses.MotorStallEvent(2, True)),
        (EVENT_MOTOR_FAULT, b"\x01", responses.MotorFaultEvent(True)),
        (EVENT_GYRO_MAX, b"\x07", responses.GyroMaxEvent(7)),
        (EVENT_IR_MESSAGE_RECEIVED, b"\x2a", responses.InfraredMessageEvent(42)),
        (EVENT_COLOR_DETECTION, bytes([1, 2, 3, 4, 0xFF]), responses.DetectedColor(1, 2, 3, 4, 0xFF)),
        (EVENT_STREAMING_SERVICE_DATA_BT, b"\x09abc", responses.StreamingServiceData(9, b"abc")),
        (EVENT_STREAMING_SERVICE_DATA_MCU, b"\x0adef", responses.StreamingServiceData(10, b"def")),
        (
            EVENT_MOTOR_THERMAL_PROTECTION_STATUS,
            struct.pack(">fBfB", 44.0, 1, 45.0, 2),
            responses.ThermalProtectionStatus(44.0, 1, 45.0, 2),
        ),
    ],
)
def test_sdk_notification_event_matrix_has_deterministic_parsers_and_cache_policy(event_key, payload, expected):
    assert EVENT_PARSERS[event_key](payload) == expected
    if event_key in (EVENT_STREAMING_SERVICE_DATA_BT, EVENT_STREAMING_SERVICE_DATA_MCU):
        assert event_key not in CACHEABLE_EVENTS
    else:
        assert event_key in CACHEABLE_EVENTS
