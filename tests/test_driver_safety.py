import asyncio

import pytest

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import Packet

RAW_OFF = bytes([0, 0, 0, 0])


@pytest.mark.asyncio
async def test_stale_velocity_command_causes_validated_raw_motor_off_packet():
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=0.03)
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await asyncio.sleep(0.08)
    await driver.disconnect()

    packets = [Packet.decode(raw) for raw in transport.writes]
    raw_motor_packets = [packet for packet in packets if packet.command_id == driver.commands.CID_RAW_MOTORS]
    assert any(packet.payload != RAW_OFF for packet in raw_motor_packets)
    assert any(packet.payload == RAW_OFF for packet in raw_motor_packets)


@pytest.mark.asyncio
async def test_emergency_stop_preempts_velocity_with_raw_motor_off_and_blocks_drive_until_cleared():
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
    estop_index = next(
        i
        for i, packet in enumerate(packets)
        if packet.command_id == driver.commands.CID_RAW_MOTORS and packet.payload == RAW_OFF
    )
    assert all(
        not (packet.command_id == driver.commands.CID_RAW_MOTORS and packet.payload != RAW_OFF)
        for packet in packets[estop_index + 1 :]
    )


@pytest.mark.asyncio
async def test_clear_emergency_stop_is_software_only_until_new_velocity_arrives():
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=1.0)
    await driver.connect()

    await driver.emergency_stop()
    before_clear_count = len(transport.writes)
    await driver.clear_emergency_stop()
    await asyncio.sleep(0.02)
    await driver.disconnect()

    # clear_estop should not send the old unvalidated fake 0xFD hardware command.
    assert len(transport.writes) == before_clear_count + 1  # disconnect raw-motor-off only
    assert Packet.decode(transport.writes[-1]).payload == RAW_OFF


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


@pytest.mark.asyncio
async def test_driver_scales_velocity_against_configured_limits_before_raw_motor_mapping():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        max_linear_mps=0.25,
        max_angular_rad_s=0.4,
        max_raw_motor_duty=64,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.25, angular_rad_s=0.0)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    raw_motor_packets = [
        packet
        for packet in (Packet.decode(raw) for raw in transport.writes)
        if packet.command_id == driver.commands.CID_RAW_MOTORS
    ]
    assert raw_motor_packets
    assert raw_motor_packets[0].payload == bytes([1, 64, 1, 64])


@pytest.mark.asyncio
async def test_driver_scales_configured_max_turn_to_tank_turn_duty_cap():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        max_linear_mps=0.25,
        max_angular_rad_s=0.4,
        max_raw_motor_duty=64,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.0, angular_rad_s=0.4)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    raw_motor_packets = [
        packet
        for packet in (Packet.decode(raw) for raw in transport.writes)
        if packet.command_id == driver.commands.CID_RAW_MOTORS
    ]
    assert raw_motor_packets
    assert raw_motor_packets[0].payload == bytes([1, 64, 2, 64])
