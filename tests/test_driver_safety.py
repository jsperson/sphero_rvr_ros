import asyncio
import struct

import pytest

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import Packet

RAW_OFF = bytes([0, 0, 0, 0])


def _packets(transport: FakeTransport) -> list[Packet]:
    return [Packet.decode(raw) for raw in transport.writes]


def _raw_motor_packets(transport: FakeTransport, driver: RVRDriver) -> list[Packet]:
    return [packet for packet in _packets(transport) if packet.command_id == driver.commands.CID_RAW_MOTORS]


def _rc_drive_packets(transport: FakeTransport, driver: RVRDriver) -> list[Packet]:
    return [packet for packet in _packets(transport) if packet.command_id == driver.commands.CID_DRIVE_RC_SI_UNITS]


def _tank_drive_packets(transport: FakeTransport, driver: RVRDriver) -> list[Packet]:
    return [packet for packet in _packets(transport) if packet.command_id == driver.commands.CID_DRIVE_TANK_NORMALIZED]


def _decode_rc_payload(packet: Packet) -> tuple[float, float, int]:
    yaw, linear, flags = struct.unpack(">ffB", packet.payload)
    return yaw, linear, flags


@pytest.mark.asyncio
async def test_stale_velocity_command_causes_validated_raw_motor_off_packet():
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=0.03)
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await asyncio.sleep(0.08)
    await driver.disconnect()

    assert _rc_drive_packets(transport, driver)
    raw_motor_packets = _raw_motor_packets(transport, driver)
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

    packets = _packets(transport)
    estop_index = next(
        i
        for i, packet in enumerate(packets)
        if packet.command_id == driver.commands.CID_RAW_MOTORS and packet.payload == RAW_OFF
    )
    assert all(
        not (packet.command_id == driver.commands.CID_DRIVE_RC_SI_UNITS)
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
async def test_driver_uses_native_rc_drive_for_velocity_control():
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

    rc_packets = _rc_drive_packets(transport, driver)
    assert rc_packets
    yaw, linear, flags = _decode_rc_payload(rc_packets[0])
    assert yaw == pytest.approx(0.0)
    assert linear == pytest.approx(1.0)
    assert flags == 1


@pytest.mark.asyncio
async def test_driver_clamps_velocity_before_native_rc_drive():
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

    await driver.set_velocity(linear_mps=1.0, angular_rad_s=2.0)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    rc_packets = _rc_drive_packets(transport, driver)
    assert rc_packets
    yaw, linear, flags = _decode_rc_payload(rc_packets[0])
    assert yaw == pytest.approx(0.4)
    assert linear == pytest.approx(0.25)
    assert flags == 1


@pytest.mark.asyncio
async def test_driver_uses_explicit_opposing_treads_for_near_pure_turning():
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

    tank_packets = _tank_drive_packets(transport, driver)
    assert tank_packets
    assert tank_packets[0].payload == bytes([0x81, 0x7F])
    assert not _rc_drive_packets(transport, driver)
    assert all(packet.payload == RAW_OFF for packet in _raw_motor_packets(transport, driver))


@pytest.mark.asyncio
async def test_mixed_drive_uses_native_rc_drive_commands():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        max_linear_mps=0.10,
        max_angular_rad_s=0.4,
        max_raw_motor_duty=160,
        max_linear_raw_motor_duty=64,
        max_angular_raw_motor_duty=255,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.10, angular_rad_s=0.4)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    rc_packets = _rc_drive_packets(transport, driver)
    assert rc_packets
    yaw, linear, flags = _decode_rc_payload(rc_packets[0])
    assert yaw == pytest.approx(0.4)
    assert linear == pytest.approx(0.10)
    assert flags == 1
