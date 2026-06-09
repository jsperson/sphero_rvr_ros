import asyncio

import pytest

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import Packet


@pytest.mark.asyncio
async def test_stale_velocity_command_causes_stop_packet():
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=0.03)
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await asyncio.sleep(0.08)
    await driver.disconnect()

    command_ids = [Packet.decode(raw).command_id for raw in transport.writes]
    assert driver.commands.CID_RAW_MOTORS in command_ids
    assert driver.commands.STOP in command_ids


@pytest.mark.asyncio
async def test_emergency_stop_preempts_velocity_and_blocks_drive_until_cleared():
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=1.0)
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await asyncio.sleep(0.02)
    await driver.emergency_stop()
    await driver.set_velocity(linear_mps=0.3, angular_rad_s=0.0)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    packets = [Packet.decode(raw) for raw in transport.writes]
    estop_index = next(i for i, p in enumerate(packets) if p.command_id == driver.commands.EMERGENCY_STOP)
    assert all(p.command_id != driver.commands.CID_RAW_MOTORS for p in packets[estop_index + 1:])


@pytest.mark.asyncio
async def test_driver_caps_raw_motor_duty_for_velocity_control():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        max_linear_mps=1.0,
        max_angular_rad_s=1.0,
        max_raw_motor_duty=64,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=1.0, angular_rad_s=0.0)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    raw_motor_packets = [
        packet
        for packet in (Packet.decode(raw) for raw in transport.writes)
        if packet.command_id == driver.commands.CID_RAW_MOTORS
    ]
    assert raw_motor_packets
    assert raw_motor_packets[0].payload == bytes([1, 64, 1, 64])
