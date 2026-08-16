import asyncio
import math
import struct
import time
from contextlib import suppress

import pytest

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import Packet
from sphero_rvr_core.commands import RVRCommands
from sphero_rvr_core.safety import clamp

RAW_OFF = bytes([0, 0, 0, 0])
ESTOP_SOFTWARE_DISPATCH_BUDGET_S = 0.10


@pytest.mark.parametrize("value", (math.nan, math.inf, -math.inf))
def test_clamp_rejects_non_finite_values_instead_of_saturating_to_motion(value):
    with pytest.raises(ValueError, match="non-finite"):
        clamp(value, -0.1, 0.1)


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


class ReleaseControlledWriteTransport(FakeTransport):
    def __init__(self):
        super().__init__(auto_ack=False)
        self.write_started = asyncio.Event()
        self.release_write = asyncio.Event()

    async def write(self, data: bytes) -> None:
        self.write_started.set()
        await self.release_write.wait()
        await super().write(data)


def _packets(transport: FakeTransport) -> list[Packet]:
    return [Packet.decode(raw) for raw in transport.writes]


def _raw_motor_packets(transport: FakeTransport, driver: RVRDriver) -> list[Packet]:
    return [packet for packet in _packets(transport) if packet.command_id == driver.commands.CID_RAW_MOTORS]


def _rc_drive_packets(transport: FakeTransport, driver: RVRDriver) -> list[Packet]:
    return [packet for packet in _packets(transport) if packet.command_id == driver.commands.CID_DRIVE_RC_SI_UNITS]


def _tank_drive_packets(transport: FakeTransport, driver: RVRDriver) -> list[Packet]:
    return [packet for packet in _packets(transport) if packet.command_id == driver.commands.CID_DRIVE_TANK_NORMALIZED]


def _tank_si_drive_packets(transport: FakeTransport, driver: RVRDriver) -> list[Packet]:
    return [packet for packet in _packets(transport) if packet.command_id == driver.commands.CID_DRIVE_TANK_SI_UNITS]


def _motion_packets(transport: FakeTransport, driver: RVRDriver) -> list[Packet]:
    motion_cids = {
        driver.commands.CID_RAW_MOTORS,
        driver.commands.CID_DRIVE_WITH_HEADING,
        driver.commands.CID_DRIVE_TANK_SI_UNITS,
        driver.commands.CID_DRIVE_TANK_NORMALIZED,
        driver.commands.CID_DRIVE_RC_SI_UNITS,
        driver.commands.CID_DRIVE_RC_NORMALIZED,
        driver.commands.CID_DRIVE_TO_POSITION_SI,
    }
    return [packet for packet in _packets(transport) if packet.command_id in motion_cids]


async def _wait_until(predicate, *, timeout_s: float = 0.5) -> None:
    """Wait for an async driver outcome without assuming host scheduling speed."""

    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_s
    while not predicate():
        if loop.time() >= deadline:
            raise AssertionError(
                f"driver outcome was not observed within {timeout_s:.3f}s"
            )
        await asyncio.sleep(0.001)


async def _start_blocked_battery_request(driver: RVRDriver, transport: FakeTransport) -> asyncio.Task:
    battery_task = asyncio.create_task(driver.get_battery_percentage())
    await transport.wait_for_write()
    assert Packet.decode(transport.writes[-1]).command_id == driver.commands.CID_GET_BATTERY_PERCENTAGE
    return battery_task


async def _cleanup_driver(driver: RVRDriver, *tasks: asyncio.Task) -> None:
    for task in tasks:
        if not task.done():
            task.cancel()
    for task in tasks:
        with suppress(asyncio.CancelledError, TimeoutError):
            await task
    await asyncio.wait_for(driver.disconnect(), timeout=1.0)


def _decode_rc_payload(packet: Packet) -> tuple[float, float, int]:
    yaw, linear, flags = struct.unpack(">ffB", packet.payload)
    return yaw, linear, flags


@pytest.mark.asyncio
async def test_emergency_stop_bypasses_blocked_response_within_software_dispatch_budget():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport, control_period=10.0, command_timeout=10.0)
    await driver.connect()
    transport.writes.clear()
    transport._write_event.clear()
    battery_task = await _start_blocked_battery_request(driver, transport)

    started = time.perf_counter()
    await asyncio.wait_for(driver.emergency_stop(), timeout=ESTOP_SOFTWARE_DISPATCH_BUDGET_S)
    elapsed = time.perf_counter() - started

    try:
        assert elapsed <= ESTOP_SOFTWARE_DISPATCH_BUDGET_S
        assert _raw_motor_packets(transport, driver)[-1].payload == RAW_OFF
    finally:
        await _cleanup_driver(driver, battery_task)


@pytest.mark.asyncio
async def test_stop_invalidates_motion_queued_before_it_without_replaying_after_stop_frame():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport, control_period=10.0, command_timeout=10.0)
    await driver.connect()
    transport.writes.clear()
    transport._write_event.clear()
    battery_task = await _start_blocked_battery_request(driver, transport)

    stale_motion = asyncio.create_task(driver.drive_with_heading(speed=100, heading=90))
    await asyncio.sleep(0)
    await asyncio.wait_for(driver.stop(), timeout=ESTOP_SOFTWARE_DISPATCH_BUDGET_S)
    stop_index = len(_packets(transport)) - 1

    await asyncio.sleep(1.1)
    with suppress(TimeoutError):
        await stale_motion

    try:
        packets_after_stop = _packets(transport)[stop_index + 1 :]
        assert all(packet.command_id != driver.commands.CID_DRIVE_WITH_HEADING for packet in packets_after_stop)
    finally:
        await _cleanup_driver(driver, battery_task, stale_motion)


@pytest.mark.asyncio
async def test_stop_invalidates_queued_autonomous_ir_motion_before_it_can_dispatch():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport, control_period=10.0, command_timeout=10.0)
    await driver.connect()
    transport.writes.clear()
    transport._write_event.clear()
    battery_task = await _start_blocked_battery_request(driver, transport)

    stale_ir_motion = asyncio.create_task(driver.start_ir_following(far_code=4, near_code=5))
    await asyncio.sleep(0)
    await asyncio.wait_for(driver.stop(), timeout=ESTOP_SOFTWARE_DISPATCH_BUDGET_S)

    await asyncio.sleep(1.1)
    with suppress(TimeoutError):
        await stale_ir_motion

    try:
        packets = _packets(transport)
        assert all(packet.command_id != driver.commands.CID_START_IR_FOLLOWING for packet in packets)
    finally:
        await _cleanup_driver(driver, battery_task, stale_ir_motion)


@pytest.mark.asyncio
async def test_stop_allows_new_post_stop_motion_after_invalidating_old_queued_motion():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport, control_period=10.0, command_timeout=10.0)
    await driver.connect()
    transport.writes.clear()
    transport._write_event.clear()
    battery_task = await _start_blocked_battery_request(driver, transport)

    stale_motion = asyncio.create_task(driver.drive_with_heading(speed=100, heading=90))
    await asyncio.sleep(0)
    await asyncio.wait_for(driver.stop(), timeout=ESTOP_SOFTWARE_DISPATCH_BUDGET_S)
    fresh_motion = asyncio.create_task(driver.drive_with_heading(speed=40, heading=180))

    await asyncio.sleep(1.1)
    with suppress(TimeoutError):
        await stale_motion
    await asyncio.wait_for(fresh_motion, timeout=0.2)

    try:
        heading_packets = [
            packet for packet in _packets(transport) if packet.command_id == driver.commands.CID_DRIVE_WITH_HEADING
        ]
        assert len(heading_packets) == 1
        assert heading_packets[0].payload == struct.pack(">BHB", 40, 180, 0)
    finally:
        await _cleanup_driver(driver, battery_task, stale_motion, fresh_motion)


@pytest.mark.asyncio
async def test_stop_waits_for_in_progress_write_lock_without_interleaving_packet_bytes():
    transport = ReleaseControlledWriteTransport()
    driver = RVRDriver(transport=transport, control_period=10.0, command_timeout=10.0)
    await driver._dispatcher.start()

    request_packet = driver.commands.get_battery_percentage(1)
    request_task = asyncio.create_task(driver._dispatcher.request(request_packet, timeout=0.2))
    await transport.write_started.wait()

    stop_task = asyncio.create_task(driver.stop())
    await asyncio.sleep(0.02)
    assert transport.writes == []

    transport.release_write.set()
    await asyncio.wait_for(stop_task, timeout=ESTOP_SOFTWARE_DISPATCH_BUDGET_S)
    with suppress(TimeoutError):
        await request_task
    await driver._dispatcher.stop()

    packets = _packets(transport)
    assert packets[0].command_id == driver.commands.CID_GET_BATTERY_PERCENTAGE
    assert packets[1].command_id == driver.commands.CID_RAW_MOTORS
    assert packets[1].payload == RAW_OFF


@pytest.mark.asyncio
async def test_stop_fails_closed_if_write_contention_exceeds_dispatch_budget():
    transport = ReleaseControlledWriteTransport()
    driver = RVRDriver(transport=transport, control_period=10.0, command_timeout=10.0, safety_dispatch_timeout_s=0.02)
    await driver._dispatcher.start()

    request_packet = driver.commands.get_battery_percentage(1)
    request_task = asyncio.create_task(driver._dispatcher.request(request_packet, timeout=0.2))
    await transport.write_started.wait()

    started = time.perf_counter()
    with pytest.raises(TimeoutError, match="safety stop dispatch exceeded"):
        await driver.stop()
    elapsed = time.perf_counter() - started

    transport.release_write.set()
    with suppress(TimeoutError):
        await request_task
    await driver._dispatcher.stop()

    assert elapsed < ESTOP_SOFTWARE_DISPATCH_BUDGET_S
    assert driver.get_state().fail_safe_active


@pytest.mark.asyncio
async def test_stop_invalidates_motion_already_waiting_on_dispatcher_write_lock():
    transport = ReleaseControlledWriteTransport()
    driver = RVRDriver(transport=transport, control_period=10.0, command_timeout=10.0)
    await driver._dispatcher.start()
    await driver._queue.start()

    request_packet = driver.commands.get_battery_percentage(1)
    request_task = asyncio.create_task(driver._dispatcher.request(request_packet, timeout=0.2))
    await transport.write_started.wait()

    stale_motion = asyncio.create_task(driver.drive_with_heading(speed=100, heading=90))
    await asyncio.sleep(0.02)
    stop_task = asyncio.create_task(driver.stop())
    await asyncio.sleep(0)

    transport.release_write.set()
    await asyncio.wait_for(stop_task, timeout=ESTOP_SOFTWARE_DISPATCH_BUDGET_S)
    with suppress(TimeoutError):
        await request_task
    await asyncio.wait_for(stale_motion, timeout=0.2)
    await driver._queue.stop()
    await driver._dispatcher.stop()

    packets = _packets(transport)
    assert [packet.command_id for packet in packets] == [
        driver.commands.CID_GET_BATTERY_PERCENTAGE,
        driver.commands.CID_RAW_MOTORS,
    ]
    assert packets[1].payload == RAW_OFF


@pytest.mark.asyncio
async def test_estop_rejects_motor_capable_paths_until_clear_then_only_new_motion_runs():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport, control_period=10.0, command_timeout=10.0)
    await driver.connect()
    transport.writes.clear()

    await driver.emergency_stop()

    with pytest.raises(RuntimeError, match="emergency stop active"):
        await driver.set_velocity(linear_mps=0.1, angular_rad_s=0.0)
    with pytest.raises(RuntimeError, match="emergency stop active"):
        await driver.drive_with_heading(speed=20, heading=0)
    with pytest.raises(RuntimeError, match="emergency stop active"):
        await driver.raw_motors(1, 20, 1, 20)
    with pytest.raises(RuntimeError, match="emergency stop active"):
        await driver.drive_tank_normalized(-45, 45)
    with pytest.raises(RuntimeError, match="emergency stop active"):
        await driver.drive_to_position_si(yaw_angle=0.0, x=1.0, y=0.0, linear_speed=0.2)
    with pytest.raises(RuntimeError, match="emergency stop active"):
        await driver.start_ir_following(far_code=4, near_code=5)
    with pytest.raises(RuntimeError, match="emergency stop active"):
        await driver.start_ir_evading(far_code=6, near_code=7)

    await driver.clear_emergency_stop()
    await driver.drive_with_heading(speed=20, heading=0)

    await driver.disconnect()

    motion_packets = _motion_packets(transport, driver)
    assert [packet.command_id for packet in motion_packets] == [
        driver.commands.CID_RAW_MOTORS,
        driver.commands.CID_DRIVE_WITH_HEADING,
        driver.commands.CID_RAW_MOTORS,
    ]


@pytest.mark.asyncio
async def test_stale_velocity_command_causes_validated_raw_motor_off_packet():
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=0.03,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await _wait_until(
        lambda: any(
            packet.payload == RAW_OFF
            for packet in _raw_motor_packets(transport, driver)
        )
    )
    await asyncio.wait_for(driver.disconnect(), timeout=1.0)

    assert _tank_si_drive_packets(transport, driver)
    raw_motor_packets = _raw_motor_packets(transport, driver)
    assert any(packet.payload == RAW_OFF for packet in raw_motor_packets)


@pytest.mark.asyncio
async def test_zero_velocity_transition_uses_immediate_stop_once_without_tank_coast_command():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await asyncio.sleep(0.03)
    assert _tank_si_drive_packets(transport, driver)
    before_zero = len(transport.writes)

    await driver.set_velocity(linear_mps=0.0, angular_rad_s=0.0)
    after_first_zero = len(transport.writes)
    await driver.set_velocity(linear_mps=0.0, angular_rad_s=0.0)
    await asyncio.sleep(0.03)

    packets_after_motion = [Packet.decode(raw) for raw in transport.writes[before_zero:]]
    assert after_first_zero == before_zero + 1
    assert len(packets_after_motion) == 1
    assert packets_after_motion[0].command_id == driver.commands.CID_RAW_MOTORS
    assert packets_after_motion[0].payload == RAW_OFF
    assert driver.get_state().latest_velocity is None

    await driver.disconnect()


@pytest.mark.asyncio
async def test_driver_state_records_completed_motor_transport_writes_truthfully():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=10.0,
        command_timeout=10.0,
    )
    await driver.connect()

    await driver.raw_motors(2, 223, 1, 223)
    moving_state = driver.get_state()
    assert moving_state.motor_transport_write_count == 1
    assert moving_state.motion_transport_write_count == 1
    assert moving_state.last_motor_command_id == RVRCommands.CID_RAW_MOTORS
    assert moving_state.last_motor_payload_hex == "02df01df"
    assert moving_state.last_motor_transport_write_epoch_s is not None
    assert moving_state.last_motion_transport_write_epoch_s is not None

    await driver.stop()
    stopped_state = driver.get_state()
    assert stopped_state.motor_transport_write_count == 2
    assert stopped_state.motion_transport_write_count == 1
    assert stopped_state.last_motor_payload_hex == "00000000"
    assert (
        stopped_state.last_motion_transport_write_epoch_s
        == moving_state.last_motion_transport_write_epoch_s
    )

    await driver.disconnect()


@pytest.mark.asyncio
async def test_driver_state_does_not_count_failed_motor_transport_write():
    transport = FailingWriteTransport(
        failures=1,
        command_id=RVRCommands.CID_RAW_MOTORS,
        auto_ack=False,
    )
    driver = RVRDriver(
        transport=transport,
        control_period=10.0,
        command_timeout=10.0,
    )
    await driver.connect()

    with pytest.raises(RuntimeError, match="injected write failure"):
        await driver.raw_motors(2, 64, 1, 64)

    state = driver.get_state()
    assert state.motor_transport_write_count == 0
    assert state.motion_transport_write_count == 0
    assert state.last_motor_payload_hex is None

    await driver.disconnect()


@pytest.mark.asyncio
async def test_stop_invalidates_control_loop_velocity_cached_before_dispatch():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=1.0)
    await driver.connect()
    dispatch_entered = asyncio.Event()
    release_dispatch = asyncio.Event()
    original = driver._send_from_control_loop

    async def delayed_send(packet_factory, *, motion_generation):
        dispatch_entered.set()
        await release_dispatch.wait()
        await original(packet_factory, motion_generation=motion_generation)

    driver._send_from_control_loop = delayed_send
    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await asyncio.wait_for(dispatch_entered.wait(), timeout=0.2)
    await driver.set_velocity(linear_mps=0.0, angular_rad_s=0.0)
    stop_index = len(transport.writes) - 1
    release_dispatch.set()
    await asyncio.sleep(0.04)

    packets_after_stop = _packets(transport)[stop_index + 1 :]
    assert all(packet.command_id != driver.commands.CID_DRIVE_RC_SI_UNITS for packet in packets_after_stop)
    assert _packets(transport)[stop_index].command_id == driver.commands.CID_RAW_MOTORS
    assert _packets(transport)[stop_index].payload == RAW_OFF

    await driver.disconnect()


@pytest.mark.asyncio
async def test_emergency_stop_preempts_velocity_with_raw_motor_off_and_blocks_drive_until_cleared():
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=1.0)
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await asyncio.sleep(0.02)
    await driver.emergency_stop()
    with pytest.raises(RuntimeError, match="emergency stop active"):
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
async def test_driver_uses_native_tank_si_for_straight_velocity_control():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        max_linear_mps=0.05,
        max_angular_rad_s=0.4,
        max_raw_motor_duty=64,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
        wheel_track_m=0.2507,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.05, angular_rad_s=0.0)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    packets = _tank_si_drive_packets(transport, driver)
    assert packets
    left, right = struct.unpack(">ff", packets[0].payload)
    assert left == pytest.approx(0.05)
    assert right == pytest.approx(0.05)
    assert not _rc_drive_packets(transport, driver)


@pytest.mark.asyncio
async def test_native_tank_si_pivot_uses_calibrated_track_width():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        max_linear_mps=0.05,
        max_angular_rad_s=0.4,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
        wheel_track_m=0.2507,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.0, angular_rad_s=0.4)
    await asyncio.sleep(0.05)
    await driver.disconnect()

    duties = [struct.unpack(">bb", p.payload) for p in _tank_drive_packets(transport, driver)]
    assert duties
    # Closed-loop pivot with no odom feedback ramps the raw duty up from the
    # minimum; a positive angular command drives the left wheel back, right fwd.
    assert duties[0] == (-23, 23)
    assert all(left < 0 < right for left, right in duties)
    assert abs(duties[-1][1]) >= abs(duties[0][1])
    assert not _tank_si_drive_packets(transport, driver)


@pytest.mark.asyncio
async def test_raw_motor_velocity_control_honors_linear_calibration_cap():
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
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_RAW_MOTOR,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.08, angular_rad_s=0.0)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    moving = [packet for packet in _raw_motor_packets(transport, driver) if packet.payload != RAW_OFF]
    assert moving
    assert moving[0].payload == bytes([1, 51, 1, 51])
    state = driver.get_state()
    assert state.motion_transport_write_count == len(moving)
    assert state.motor_transport_write_count == len(_raw_motor_packets(transport, driver))
    assert state.last_motor_payload_hex == RAW_OFF.hex()
    assert not _rc_drive_packets(transport, driver)
    assert not _tank_drive_packets(transport, driver)


def test_driver_rejects_unknown_velocity_control_mode():
    with pytest.raises(ValueError, match="velocity_control_mode"):
        RVRDriver(FakeTransport(), velocity_control_mode="magic")


def test_driver_rejects_quarantined_native_rc_si_mode():
    with pytest.raises(ValueError, match="native_rc_si is quarantined"):
        RVRDriver(
            FakeTransport(),
            velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_RC_SI,
        )


@pytest.mark.asyncio
async def test_driver_clamps_twist_before_native_tank_si_differential_mapping():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        max_linear_mps=0.25,
        max_angular_rad_s=0.4,
        max_raw_motor_duty=64,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
        wheel_track_m=0.2507,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=1.0, angular_rad_s=2.0)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    tank_packets = _tank_si_drive_packets(transport, driver)
    assert tank_packets
    left, right = struct.unpack(">ff", tank_packets[0].payload)
    assert left == pytest.approx(0.25 - 0.4 * 0.2507 / 2.0)
    assert right == pytest.approx(0.25 + 0.4 * 0.2507 / 2.0)
    assert not _rc_drive_packets(transport, driver)


@pytest.mark.asyncio
async def test_driver_fails_non_finite_velocity_closed_to_zero():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport, control_period=0.01, command_timeout=1.0)
    await driver.connect()

    await driver.set_velocity(linear_mps=float("nan"), angular_rad_s=float("inf"))
    await asyncio.sleep(0.03)
    await driver.disconnect()

    for packet in _rc_drive_packets(transport, driver):
        _yaw, linear, _flags = _decode_rc_payload(packet)
        assert linear == pytest.approx(0.0)
    assert all(packet.payload == RAW_OFF for packet in _raw_motor_packets(transport, driver))


@pytest.mark.asyncio
async def test_driver_uses_opposing_tank_si_velocities_for_pure_turning():
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        max_linear_mps=0.25,
        max_angular_rad_s=0.4,
        max_raw_motor_duty=64,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
        wheel_track_m=0.2507,
    )
    await driver.connect()

    driver.set_measured_yaw_rate(5.0)
    await driver.set_velocity(linear_mps=0.0, angular_rad_s=0.4)
    await asyncio.sleep(0.05)
    await driver.disconnect()

    duties = [struct.unpack(">bb", p.payload) for p in _tank_drive_packets(transport, driver)]
    assert duties
    # A measured yaw rate well above target makes the error negative, so the
    # closed loop never ramps up and holds the minimum duty (opposing wheels).
    assert all(wheels == (-23, 23) for wheels in duties)
    assert not _tank_si_drive_packets(transport, driver)
    assert not _rc_drive_packets(transport, driver)
    assert all(packet.payload == RAW_OFF for packet in _raw_motor_packets(transport, driver))


@pytest.mark.asyncio
async def test_mixed_turn_drive_uses_tank_si_kinematics_instead_of_rc():
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
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
        wheel_track_m=0.2507,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.05, angular_rad_s=0.4)
    await asyncio.sleep(0.03)
    await driver.disconnect()

    tank_packets = _tank_si_drive_packets(transport, driver)
    assert tank_packets
    left, right = struct.unpack(">ff", tank_packets[0].payload)
    assert left == pytest.approx(0.05 - 0.4 * 0.2507 / 2.0)
    assert right == pytest.approx(0.05 + 0.4 * 0.2507 / 2.0)
    assert not _rc_drive_packets(transport, driver)


@pytest.mark.asyncio
async def test_transient_control_send_fault_attempts_safe_stop_and_loop_survives(caplog):
    transport = FailingWriteTransport(failures=1, command_id=RVRCommands.CID_DRIVE_TANK_SI_UNITS, auto_ack=True)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=1.0,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await _wait_until(
        lambda: "RVR control loop send failed; attempting safe stop"
        in caplog.text
        and bool(_raw_motor_packets(transport, driver))
    )
    await driver.set_velocity(linear_mps=0.1, angular_rad_s=0.0)
    await _wait_until(lambda: bool(_tank_si_drive_packets(transport, driver)))
    await driver.disconnect()

    assert _raw_motor_packets(transport, driver)
    assert _tank_si_drive_packets(transport, driver)
    assert not driver.get_state().fail_safe_active
    assert "RVR control loop send failed; attempting safe stop" in caplog.text


@pytest.mark.asyncio
async def test_stale_stop_transient_failure_retries_safe_stop_without_fail_safe(caplog):
    transport = FailingWriteTransport(failures=1, command_id=RVRCommands.CID_RAW_MOTORS, auto_ack=True)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=0.02,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await _wait_until(
        lambda: "RVR safe stop delivery failed; retrying" in caplog.text
        and bool(_raw_motor_packets(transport, driver))
    )
    await driver.set_velocity(linear_mps=0.1, angular_rad_s=0.0)
    await _wait_until(lambda: bool(_tank_si_drive_packets(transport, driver)))
    await driver.disconnect()

    state = driver.get_state()
    assert not state.fail_safe_active
    assert _raw_motor_packets(transport, driver)
    assert _tank_si_drive_packets(transport, driver)
    assert "RVR safe stop delivery failed; retrying" in caplog.text


@pytest.mark.asyncio
async def test_stale_stop_persistent_failure_enters_fail_safe_and_blocks_drive_until_recovered(caplog):
    transport = FailingWriteTransport(failures=2, command_id=RVRCommands.CID_RAW_MOTORS, auto_ack=True)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=0.02,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
    )
    await driver.connect()

    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)
    await _wait_until(lambda: driver.get_state().fail_safe_active)

    state = driver.get_state()
    assert state.fail_safe_active
    assert state.latest_velocity is None
    with pytest.raises(RuntimeError, match="fail-safe fault active"):
        await driver.set_velocity(linear_mps=0.1, angular_rad_s=0.0)

    await driver.clear_fail_safe_fault()
    assert not driver.get_state().fail_safe_active
    await driver.set_velocity(linear_mps=0.1, angular_rad_s=0.0)
    await _wait_until(lambda: bool(_tank_si_drive_packets(transport, driver)))
    await driver.disconnect()

    assert _raw_motor_packets(transport, driver)
    assert _tank_si_drive_packets(transport, driver)
    assert "RVR fail-safe fault active" in caplog.text
