import asyncio
import inspect
import struct

import pytest

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import FLAG_HAS_SOURCE, FLAG_IS_RESPONSE, Packet, TARGET_MCU
from sphero_rvr_core.responses import (
    BatteryThresholds,
    BatteryVoltageState,
    DetectedColor,
    EncoderCounts,
    FirmwareVersion,
    IRReadings,
    MagnetometerReadings,
    RGBCSensorValues,
    TemperatureReadings,
    ThermalProtectionStatus,
)


DRIVER_ASYNC_METHODS_WITH_CAPABILITY_COVERAGE = {
    "calibrate_magnetometer",
    "clear_emergency_stop",
    "connect",
    "disconnect",
    "drive_to_position_si",
    "drive_with_heading",
    "emergency_stop",
    "enable_color_detection",
    "enable_color_detection_notify",
    "enable_motor_fault_notify",
    "enable_motor_stall_notify",
    "get_ambient_light",
    "get_battery_percentage",
    "get_battery_thresholds",
    "get_battery_voltage",
    "get_battery_voltage_state",
    "get_board_revision",
    "get_core_uptime",
    "get_current_detected_color",
    "get_encoder_counts",
    "get_firmware_version",
    "get_ir_readings",
    "get_mac_address",
    "get_magnetometer",
    "get_motor_fault_state",
    "get_processor_name",
    "get_rgbc_sensor_values",
    "get_sku",
    "get_temperature",
    "get_thermal_protection_status",
    "raw_motors",
    "reset_locator",
    "reset_yaw",
    "send_ir_message",
    "set_all_leds",
    "set_led_group",
    "set_velocity",
    "start_ir_broadcast",
    "start_ir_evading",
    "start_ir_following",
    "stop",
    "stop_ir_broadcast",
    "stop_ir_evading",
    "stop_ir_following",
}


async def start_driver() -> tuple[RVRDriver, FakeTransport]:
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(transport=transport)
    await driver.connect()
    transport.writes.clear()
    transport._write_event.clear()
    return driver, transport


async def respond_to_next_write(transport: FakeTransport, payload: bytes = b"") -> Packet:
    await transport.wait_for_write()
    request = Packet.decode(transport.writes[-1])
    response = Packet(
        request.device_id,
        request.command_id,
        request.sequence_id,
        payload=payload,
        source=request.target if request.target is not None else TARGET_MCU,
        target=None,
        flags=FLAG_IS_RESPONSE | FLAG_HAS_SOURCE,
        error=0,
    )
    await transport.inject_read(response.encode())
    return request


def latest_packet(transport: FakeTransport) -> Packet:
    return Packet.decode(transport.writes[-1])


def assert_value_matches(actual, expected) -> None:
    if isinstance(actual, BatteryThresholds):
        assert actual.critical == pytest.approx(expected.critical)
        assert actual.low == pytest.approx(expected.low)
        assert actual.hysteresis == pytest.approx(expected.hysteresis)
        return
    if isinstance(actual, TemperatureReadings):
        assert actual.left_motor == pytest.approx(expected.left_motor)
        assert actual.right_motor == pytest.approx(expected.right_motor)
        assert actual.nordic_die == expected.nordic_die
        return
    if isinstance(actual, ThermalProtectionStatus):
        assert actual.left_temp == pytest.approx(expected.left_temp)
        assert actual.left_status == expected.left_status
        assert actual.right_temp == pytest.approx(expected.right_temp)
        assert actual.right_status == expected.right_status
        return
    if isinstance(actual, MagnetometerReadings):
        assert actual.x == pytest.approx(expected.x)
        assert actual.y == pytest.approx(expected.y)
        assert actual.z == pytest.approx(expected.z)
        return
    assert actual == expected


def test_driver_capability_coverage_stays_in_sync_with_public_async_methods():
    public_async_methods = {
        name
        for name, value in inspect.getmembers(RVRDriver, inspect.iscoroutinefunction)
        if not name.startswith("_")
    }

    assert public_async_methods == DRIVER_ASYNC_METHODS_WITH_CAPABILITY_COVERAGE


@pytest.mark.asyncio
async def test_driver_state_velocity_and_emergency_stop_capabilities():
    driver, transport = await start_driver()

    await driver.set_velocity(0.25, 0.4)
    state = driver.get_state()
    assert state.latest_velocity.linear_mps == 0.25
    assert state.latest_velocity.angular_rad_s == 0.4

    await driver.stop()
    assert latest_packet(transport).payload == bytes([0, 0, 0, 0])

    await driver.emergency_stop()
    assert driver.get_state().emergency_stopped is True
    assert latest_packet(transport).payload == bytes([0, 0, 0, 0])

    count_after_estop = len(transport.writes)
    await driver.clear_emergency_stop()
    assert driver.get_state().emergency_stopped is False
    assert len(transport.writes) == count_after_estop

    await driver.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("method_name", "args", "expected_command_id", "expected_payload", "responds"),
    [
        ("reset_yaw", (), RVRDriver(FakeTransport()).commands.CID_RESET_YAW, b"", False),
        ("reset_locator", (), RVRDriver(FakeTransport()).commands.CID_RESET_LOCATOR, b"", False),
        ("set_all_leds", (1, 2, 3), RVRDriver(FakeTransport()).commands.CID_SET_ALL_LEDS, None, False),
        ("set_led_group", ("brakelight_left", 10, 20, 30), RVRDriver(FakeTransport()).commands.CID_SET_ALL_LEDS, None, False),
        (
            "drive_with_heading",
            (64, 90, True),
            RVRDriver(FakeTransport()).commands.CID_DRIVE_WITH_HEADING,
            bytes([64, 0, 90, 1]),
            False,
        ),
        ("raw_motors", (1, 120, 2, 80), RVRDriver(FakeTransport()).commands.CID_RAW_MOTORS, bytes([1, 120, 2, 80]), False),
        (
            "drive_to_position_si",
            (90.0, 1.25, -0.5, 0.75, 2),
            RVRDriver(FakeTransport()).commands.CID_DRIVE_TO_POSITION_SI,
            struct.pack(">ffffB", 90.0, 1.25, -0.5, 0.75, 2),
            True,
        ),
        ("enable_color_detection", (False,), RVRDriver(FakeTransport()).commands.CID_ENABLE_COLOR_DETECTION, bytes([0]), True),
        (
            "enable_color_detection_notify",
            (False, 250, 7),
            RVRDriver(FakeTransport()).commands.CID_ENABLE_COLOR_DETECTION_NOTIFY,
            struct.pack(">BHB", 0, 250, 7),
            False,
        ),
        ("calibrate_magnetometer", (), RVRDriver(FakeTransport()).commands.CID_CALIBRATE_MAGNETOMETER, b"", False),
        ("enable_motor_stall_notify", (False,), RVRDriver(FakeTransport()).commands.CID_ENABLE_MOTOR_STALL_NOTIFY, bytes([0]), False),
        ("enable_motor_fault_notify", (True,), RVRDriver(FakeTransport()).commands.CID_ENABLE_MOTOR_FAULT_NOTIFY, bytes([1]), False),
        ("send_ir_message", (3, 32), RVRDriver(FakeTransport()).commands.CID_SEND_IR_MESSAGE, bytes([3, 32]), False),
        ("start_ir_broadcast", (1, 2), RVRDriver(FakeTransport()).commands.CID_START_IR_BROADCAST, bytes([1, 2]), False),
        ("stop_ir_broadcast", (), RVRDriver(FakeTransport()).commands.CID_STOP_IR_BROADCAST, b"", False),
        ("start_ir_following", (4, 5), RVRDriver(FakeTransport()).commands.CID_START_IR_FOLLOWING, bytes([4, 5]), False),
        ("stop_ir_following", (), RVRDriver(FakeTransport()).commands.CID_STOP_IR_FOLLOWING, b"", False),
        ("start_ir_evading", (6, 7), RVRDriver(FakeTransport()).commands.CID_START_IR_EVADING, bytes([6, 7]), False),
        ("stop_ir_evading", (), RVRDriver(FakeTransport()).commands.CID_STOP_IR_EVADING, b"", False),
    ],
)
async def test_driver_command_capabilities_send_expected_packets(method_name, args, expected_command_id, expected_payload, responds):
    driver, transport = await start_driver()

    task = asyncio.create_task(getattr(driver, method_name)(*args))
    if responds:
        await respond_to_next_write(transport)
    await asyncio.wait_for(task, timeout=0.1)

    packet = latest_packet(transport)
    assert packet.command_id == expected_command_id
    if expected_payload is not None:
        assert packet.payload == expected_payload

    await driver.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("method_name", "args", "payload", "expected"),
    [
        ("get_battery_percentage", (), bytes([88]), 88),
        ("get_battery_voltage", (), struct.pack(">f", 7.42), pytest.approx(7.42)),
        ("get_battery_voltage_state", (), bytes([3]), BatteryVoltageState(3, "critical")),
        ("get_battery_thresholds", (), struct.pack(">fff", 6.0, 6.8, 0.2), BatteryThresholds(6.0, 6.8, 0.2)),
        ("get_rgbc_sensor_values", (), struct.pack(">HHHH", 1, 2, 3, 4), RGBCSensorValues(1, 2, 3, 4)),
        ("get_ambient_light", (), struct.pack(">f", 123.5), pytest.approx(123.5)),
        ("get_current_detected_color", (), bytes([10, 20, 30, 99, 7]), DetectedColor(10, 20, 30, 99, 7)),
        (
            "get_temperature",
            (),
            bytes([4]) + struct.pack(">f", 41.5) + bytes([5]) + struct.pack(">f", 42.5),
            TemperatureReadings(left_motor=41.5, right_motor=42.5),
        ),
        (
            "get_thermal_protection_status",
            (),
            struct.pack(">fBfB", 44.0, 1, 45.0, 2),
            ThermalProtectionStatus(44.0, 1, 45.0, 2),
        ),
        ("get_encoder_counts", (), struct.pack(">ii", -5, 12), EncoderCounts(-5, 12)),
        ("get_magnetometer", (), struct.pack(">fff", 1.25, -2.5, 3.75), MagnetometerReadings(1.25, -2.5, 3.75)),
        ("get_motor_fault_state", (), bytes([1]), True),
        ("get_firmware_version", (TARGET_MCU,), struct.pack(">HHH", 1, 2, 3), FirmwareVersion(1, 2, 3)),
        ("get_mac_address", (), b"AA:BB\x00ignored", "AA:BB"),
        ("get_board_revision", (TARGET_MCU,), bytes([9]), 9),
        ("get_processor_name", (TARGET_MCU,), b"nRF52\x00ignored", "nRF52"),
        ("get_sku", (), b"RVR-123\x00ignored", "RVR-123"),
        ("get_core_uptime", (TARGET_MCU,), struct.pack(">Q", 123456789), 123456789),
        ("get_ir_readings", (), struct.pack(">I", 0x44332211), IRReadings(0x11, 0x22, 0x33, 0x44)),
    ],
)
async def test_driver_query_capabilities_parse_responses(method_name, args, payload, expected):
    driver, transport = await start_driver()

    task = asyncio.create_task(getattr(driver, method_name)(*args))
    await respond_to_next_write(transport, payload)

    assert_value_matches(await task, expected)

    await driver.disconnect()
