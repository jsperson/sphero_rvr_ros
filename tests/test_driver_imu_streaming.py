"""Driver-level tests for IMU sensor streaming (Stage B)."""

import asyncio
import struct

import pytest

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import Packet
from sphero_rvr_core.responses import StreamingServiceData
from sphero_rvr_core import sensor_streaming as ss

TARGET_ST = 2  # secondary / ST processor carries the IMU sensors


def _pack32(value: float, minimum: float, maximum: float) -> bytes:
    raw = round((value - minimum) / (maximum - minimum) * 0xFFFFFFFF)
    return struct.pack(">I", raw)


def _imu_payload() -> bytes:
    quat = b"".join(_pack32(v, -1.0, 1.0) for v in (1.0, 0.0, 0.0, 0.0))
    accel = b"".join(_pack32(v, -16.0, 16.0) for v in (0.0, 0.0, 1.0))
    gyro = b"".join(_pack32(v, -2000.0, 2000.0) for v in (0.0, 0.0, 90.0))
    return quat + accel + gyro


@pytest.mark.asyncio
async def test_enable_imu_streaming_configures_and_starts_st_slot():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport)
    await asyncio.wait_for(driver.connect(), timeout=0.05)
    transport.writes.clear()

    await asyncio.wait_for(driver.enable_imu_streaming(100), timeout=0.2)

    packets = [Packet.decode(raw) for raw in transport.writes]
    cids = [p.command_id for p in packets]
    assert driver.commands.CID_CONFIGURE_STREAMING_SERVICE in cids
    assert driver.commands.CID_START_STREAMING_SERVICE in cids

    configure = next(
        p for p in packets if p.command_id == driver.commands.CID_CONFIGURE_STREAMING_SERVICE
    )
    assert configure.target == TARGET_ST
    assert configure.payload[0] == ss.IMU_SLOT_TOKEN
    assert configure.payload[1:] == ss.build_slot_configuration(ss.IMU_STREAM_SERVICES)

    start = next(
        p for p in packets if p.command_id == driver.commands.CID_START_STREAMING_SERVICE
    )
    assert start.target == TARGET_ST
    assert struct.unpack(">H", start.payload)[0] == 100

    await asyncio.wait_for(driver.disconnect(), timeout=0.2)


@pytest.mark.asyncio
async def test_enable_imu_streaming_clamps_below_firmware_minimum():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport)
    await asyncio.wait_for(driver.connect(), timeout=0.05)
    transport.writes.clear()

    await asyncio.wait_for(driver.enable_imu_streaming(5), timeout=0.2)

    start = next(
        Packet.decode(raw)
        for raw in transport.writes
        if Packet.decode(raw).command_id == driver.commands.CID_START_STREAMING_SERVICE
    )
    assert struct.unpack(">H", start.payload)[0] == 33  # firmware minimum

    await asyncio.wait_for(driver.disconnect(), timeout=0.2)


def test_streaming_handler_decodes_and_fires_imu_callback():
    driver = RVRDriver(transport=FakeTransport(auto_ack=False))
    samples = []
    driver.set_imu_callback(samples.append)

    driver._handle_streaming_data(StreamingServiceData(token=0x01, sensor_data=_imu_payload()))

    assert len(samples) == 1
    assert driver.get_imu_sample() is samples[0]
    assert samples[0].is_valid is True
    assert samples[0].orientation[3] == pytest.approx(1.0, abs=1e-6)


def test_streaming_handler_ignores_other_slot_tokens_and_short_payloads():
    driver = RVRDriver(transport=FakeTransport(auto_ack=False))
    samples = []
    driver.set_imu_callback(samples.append)

    # token id 2 is a different slot (Locator/Velocity), not the IMU slot.
    driver._handle_streaming_data(StreamingServiceData(token=0x02, sensor_data=_imu_payload()))
    # truncated payload for the IMU slot must not raise, just drop.
    driver._handle_streaming_data(StreamingServiceData(token=0x01, sensor_data=b"\x00\x00"))

    assert samples == []
    assert driver.get_imu_sample() is None
