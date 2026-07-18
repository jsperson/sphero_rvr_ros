import asyncio
import struct

import pytest

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import Packet
from sphero_rvr_core.commands import RVRCommands

RAW_OFF = bytes([0, 0, 0, 0])


class FailingWriteTransport(FakeTransport):
    def __init__(self, failures: int, *, command_id: int, auto_ack: bool = False):
        super().__init__(auto_ack=auto_ack)
        self.failures = failures
        self.command_id = command_id

    async def write(self, data: bytes) -> None:
        packet = Packet.decode(data)
        if self.failures > 0 and packet.command_id == self.command_id:
            self.failures -= 1
            raise RuntimeError("injected write failure")
        await super().write(data)


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
async def test_driver_clamps_mixed_turn_velocity_before_tank_differential_arc():
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

    tank_packets = _tank_drive_packets(transport, driver)
    assert tank_packets
    assert tank_packets[0].payload == bytes([0, 127])
    assert not _rc_drive_packets(transport, driver)


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
async def test_mixed_turn_drive_uses_tank_differential_arc_instead_of_rc_straight():
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

    await driver.set_velocity(linear_mps=0.05, angular_rad_s=0.4)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    tank_packets = _tank_drive_packets(transport, driver)
    assert tank_packets
    assert tank_packets[0].payload == bytes([0, 127])
    assert not _rc_drive_packets(transport, driver)


@pytest.mark.asyncio
async def test_transient_control_send_fault_attempts_safe_stop_and_loop_survives(caplog):
    transport = FailingWriteTransport(failures=1, command_id=RVRCommands.CID_DRIVE_RC_SI_UNITS, auto_ack=True)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=1.0)
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await asyncio.sleep(0.04)
    await driver.set_velocity(linear_mps=0.1, angular_rad_s=0.0)
    await asyncio.sleep(0.04)
    await driver.disconnect()

    assert _raw_motor_packets(transport, driver)
    assert _rc_drive_packets(transport, driver)
    assert not driver.get_state().fail_safe_active
    assert "RVR control loop send failed; attempting safe stop" in caplog.text


@pytest.mark.asyncio
async def test_stale_stop_failure_enters_fail_safe_and_blocks_drive_until_recovered(caplog):
    transport = FailingWriteTransport(failures=1, command_id=RVRCommands.CID_RAW_MOTORS, auto_ack=True)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=0.02)
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await asyncio.sleep(0.08)

    state = driver.get_state()
    assert state.fail_safe_active
    assert state.latest_velocity is None
    with pytest.raises(RuntimeError, match="fail-safe fault active"):
        await driver.set_velocity(linear_mps=0.1, angular_rad_s=0.0)

    await driver.clear_fail_safe_fault()
    assert not driver.get_state().fail_safe_active
    await driver.set_velocity(linear_mps=0.1, angular_rad_s=0.0)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    assert _raw_motor_packets(transport, driver)
    assert _rc_drive_packets(transport, driver)
    assert "RVR fail-safe fault active" in caplog.text
