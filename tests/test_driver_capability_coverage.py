import asyncio
import inspect
import re
import struct
from pathlib import Path

import pytest

from sphero_rvr_core.commands import RVRCommands
from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import DID_DRIVE, DID_POWER, DID_SENSOR, FLAG_HAS_SOURCE, FLAG_IS_RESPONSE, Packet, TARGET_BT, TARGET_MCU
from sphero_rvr_core.responses import (
    ActiveColorPalette,
    BatteryThresholds,
    BatteryVoltageState,
    ColorIdentificationReport,
    DetectedColor,
    EncoderCounts,
    FirmwareVersion,
    GyroMaxEvent,
    InfraredMessageEvent,
    IRReadings,
    MagnetometerReadings,
    MotorFaultEvent,
    MotorStallEvent,
    RGBCSensorValues,
    SleepEvent,
    StreamingServiceData,
    TemperatureReadings,
    ThermalProtectionStatus,
)


CAPABILITY_MATRIX_PATH = Path(__file__).parents[1] / "docs" / "rvr_capability_matrix.md"
README_PATH = Path(__file__).parents[1] / "README.md"
STATUS_PATH = Path(__file__).parents[1] / "STATUS.md"
CAPABILITY_MATRIX_REQUIRED_COLUMNS = {
    "Domain",
    "SDK/API method",
    "DID",
    "CID",
    "Target",
    "Payload shape",
    "Response shape",
    "Mode",
    "Notification/event behavior",
    "Current repo support",
    "ROS exposure decision",
    "Test status",
    "Notes",
}
CAPABILITY_MATRIX_STATUS_TOKENS = {
    "core-implemented",
    "core-planned",
    "core-only",
    "ros-exposed",
    "ros-intentionally-omitted",
    "needs-protocol-research",
    "omitted-unsafe-or-admin",
}
CAPABILITY_MATRIX_TEST_STATUS_TOKENS = {
    "builder-test",
    "parser-test",
    "driver-test",
    "notification-test",
    "ros-exposure-test",
    "fake-transport-test",
    "documented-omission",
}


def public_async_driver_methods() -> set[str]:
    return {
        name
        for name, value in inspect.getmembers(RVRDriver, inspect.iscoroutinefunction)
        if not name.startswith("_")
    }


def public_command_builder_methods() -> set[str]:
    return {
        name
        for name, value in inspect.getmembers(RVRCommands, inspect.isfunction)
        if not name.startswith("_")
    }


def matrix_tables() -> list[tuple[list[str], list[dict[str, str]]]]:
    tables = []
    headers: list[str] | None = None
    rows: list[dict[str, str]] = []

    for raw_line in CAPABILITY_MATRIX_PATH.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line.startswith("|"):
            if headers and rows:
                tables.append((headers, rows))
            headers = None
            rows = []
            continue

        cells = [cell.strip() for cell in line.strip("|").split("|")]
        if all(cell and set(cell) <= {"-", ":"} for cell in cells):
            continue
        if headers is None:
            headers = cells
            rows = []
            continue
        if len(cells) == len(headers):
            rows.append(dict(zip(headers, cells)))

    if headers and rows:
        tables.append((headers, rows))
    return tables


def official_capability_rows() -> list[dict[str, str]]:
    for headers, rows in matrix_tables():
        if set(headers) == CAPABILITY_MATRIX_REQUIRED_COLUMNS:
            return rows
    raise AssertionError(f"Could not find official capability matrix table in {CAPABILITY_MATRIX_PATH}")


def current_repo_extra_rows() -> list[dict[str, str]]:
    for headers, rows in matrix_tables():
        if headers == ["Repo capability", "Current packet", "Current support", "Note"]:
            return rows
    raise AssertionError(f"Could not find current repo extras table in {CAPABILITY_MATRIX_PATH}")


def identifier_tokens(text: str) -> set[str]:
    tokens = set()
    for span in re.findall(r"`([^`]+)`", text):
        for piece in re.split(r"\s+or\s+|\s*/\s*|,\s*", span):
            match = re.match(r"([a-z_][a-z0-9_]*)\s*(?:\(|$)", piece.strip())
            if match:
                tokens.add(match.group(1))
    return tokens


def matrix_classified_identifiers() -> set[str]:
    identifiers = set()
    for row in official_capability_rows() + current_repo_extra_rows():
        identifiers.update(identifier_tokens(" ".join(row.values())))
    return identifiers


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


def test_capability_matrix_has_expected_shape_and_classification_tokens():
    rows = official_capability_rows()

    assert rows, "Capability matrix must contain at least one official API row"
    for row in rows:
        assert set(row) == CAPABILITY_MATRIX_REQUIRED_COLUMNS
        assert identifier_tokens(row["SDK/API method"]), f"Missing backticked SDK/API method: {row}"
        assert row["Current repo support"]
        assert row["ROS exposure decision"]
        assert row["Test status"]
        assert any(token in row["ROS exposure decision"] for token in CAPABILITY_MATRIX_STATUS_TOKENS), row


def test_capability_matrix_rows_have_explicit_test_state_tokens():
    rows = official_capability_rows()

    missing = [
        row["SDK/API method"]
        for row in rows
        if not any(token in row["Test status"] for token in CAPABILITY_MATRIX_TEST_STATUS_TOKENS)
    ]

    assert missing == [], (
        "Every capability matrix row must classify validation coverage using one "
        f"of {sorted(CAPABILITY_MATRIX_TEST_STATUS_TOKENS)}; missing {missing}"
    )


def test_capability_matrix_response_rows_declare_parser_or_omission_state():
    missing = [
        row["SDK/API method"]
        for row in official_capability_rows()
        if row["Response shape"].lower() not in {"none", "n/a"}
        and "parser-test" not in row["Test status"]
        and "documented-omission" not in row["Test status"]
    ]

    assert missing == [], (
        "Every capability with a response/event payload must declare parser-test "
        f"coverage or documented-omission; missing {missing}"
    )


def test_capability_matrix_ros_exposed_rows_declare_ros_exposure_tests():
    missing = [
        row["SDK/API method"]
        for row in official_capability_rows()
        if "ros-exposed" in row["ROS exposure decision"]
        and "ros-exposure-test" not in row["Test status"]
    ]

    assert missing == [], (
        "Every ros-exposed matrix row must declare ros-exposure-test coverage; "
        f"missing {missing}"
    )


def test_validation_checklist_documents_fake_ros_and_live_hardware_gates():
    docs = README_PATH.read_text(encoding="utf-8") + "\n" + STATUS_PATH.read_text(encoding="utf-8")

    for expected in [
        "API parity validation checklist",
        "Hardware-smoked on a Raspberry Pi 5",
        "pending Pi/ROS validation",
        "python3 scripts/run_pytest_bounded.py --timeout 90 -- -vv",
        "colcon build --symlink-install --packages-select sphero_rvr_driver",
        "WARNING: this can start the RVR motors",
        "ros2 topic pub --once /cmd_vel",
    ]:
        assert expected in docs


def test_driver_capability_coverage_stays_in_sync_with_public_async_methods():
    missing = public_async_driver_methods() - matrix_classified_identifiers()

    assert missing == set(), (
        "Every public async RVRDriver method must be explicitly classified in "
        f"{CAPABILITY_MATRIX_PATH}; missing {sorted(missing)}"
    )


def test_command_capability_coverage_stays_in_sync_with_public_builders():
    missing = public_command_builder_methods() - matrix_classified_identifiers()

    assert missing == set(), (
        "Every public RVRCommands builder must be explicitly classified in "
        f"{CAPABILITY_MATRIX_PATH}; missing {sorted(missing)}"
    )


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
            # Opposing treads: the exact packet shape the closed-loop pivot builds, and
            # the one diagnostics/pivot_duty_sweep.py sweeps by duty.
            "drive_tank_normalized",
            (-45, 45),
            RVRDriver(FakeTransport()).commands.CID_DRIVE_TANK_NORMALIZED,
            struct.pack(">bb", -45, 45),
            False,
        ),
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
        ("get_battery_voltage", (2,), struct.pack(">f", 7.84), pytest.approx(7.84)),
        ("get_battery_voltage_state", (), bytes([3]), BatteryVoltageState(3, "critical")),
        ("get_battery_thresholds", (), struct.pack(">fff", 6.0, 6.8, 0.2), BatteryThresholds(6.0, 6.8, 0.2)),
        ("get_current_sense_amplifier_current", (1,), struct.pack(">f", 1.25), pytest.approx(1.25)),
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
        ("echo", (bytes(range(16)), TARGET_MCU), bytes(range(16)), bytes(range(16))),
        ("get_firmware_version", (TARGET_MCU,), struct.pack(">HHH", 1, 2, 3), FirmwareVersion(1, 2, 3)),
        ("get_bootloader_version", (TARGET_MCU,), struct.pack(">HHH", 4, 5, 6), FirmwareVersion(4, 5, 6)),
        ("get_mac_address", (), b"AA:BB\x00ignored", "AA:BB"),
        ("get_stats_id", (), struct.pack(">H", 0x1234), 0x1234),
        ("get_board_revision", (TARGET_MCU,), bytes([9]), 9),
        ("get_processor_name", (TARGET_MCU,), b"nRF52\x00ignored", "nRF52"),
        ("get_sku", (), b"RVR-123\x00ignored", "RVR-123"),
        ("get_core_uptime", (TARGET_MCU,), struct.pack(">Q", 123456789), 123456789),
        ("get_bluetooth_advertising_name", (), b"RVR\x00ignored", "RVR"),
        ("get_ir_readings", (), struct.pack(">I", 0x44332211), IRReadings(0x11, 0x22, 0x33, 0x44)),
        ("get_active_color_palette", (), bytes(range(48)), ActiveColorPalette(bytes(range(48)))),
        (
            "get_color_identification_report",
            (10, 20, 30, 40),
            bytes(range(24)),
            None,
        ),
    ],
)
async def test_driver_query_capabilities_parse_responses(method_name, args, payload, expected):
    driver, transport = await start_driver()

    task = asyncio.create_task(getattr(driver, method_name)(*args))
    await respond_to_next_write(transport, payload)

    result = await task
    if method_name == "get_color_identification_report":
        assert isinstance(result, ColorIdentificationReport)
        assert result.index_confidence_bytes == bytes(range(24))
    else:
        assert_value_matches(result, expected)

    await driver.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("method_name", "args", "expected_command_id", "expected_payload", "responds"),
    [
        ("sleep", (), RVRCommands.CID_SLEEP, b"", False),
        ("enable_battery_voltage_state_change_notify", (False,), RVRCommands.CID_ENABLE_BATTERY_VOLTAGE_STATE_CHANGE_NOTIFY, bytes([0]), False),
        ("enable_gyro_max_notify", (True,), RVRCommands.CID_ENABLE_GYRO_MAX_NOTIFY, bytes([1]), False),
        ("set_locator_flags", (0x80,), RVRCommands.CID_SET_LOCATOR_FLAGS, bytes([0x80]), False),
        ("configure_streaming_service", (3, bytes(range(15)), TARGET_MCU), RVRCommands.CID_CONFIGURE_STREAMING_SERVICE, bytes([3]) + bytes(range(15)), False),
        ("start_streaming_service", (100, TARGET_MCU), RVRCommands.CID_START_STREAMING_SERVICE, struct.pack(">H", 100), False),
        ("stop_streaming_service", (TARGET_MCU,), RVRCommands.CID_STOP_STREAMING_SERVICE, b"", False),
        ("clear_streaming_service", (TARGET_MCU,), RVRCommands.CID_CLEAR_STREAMING_SERVICE, b"", False),
        ("enable_robot_infrared_message_notify", (True,), RVRCommands.CID_ENABLE_ROBOT_INFRARED_MESSAGE_NOTIFY, bytes([1]), False),
        ("enable_motor_thermal_protection_status_notify", (False,), RVRCommands.CID_ENABLE_MOTOR_THERMAL_PROTECTION_STATUS_NOTIFY, bytes([0]), False),
        ("set_active_color_palette", (bytes(range(48)),), RVRCommands.CID_SET_ACTIVE_COLOR_PALETTE, bytes(range(48)), False),
        ("load_color_palette", (2,), RVRCommands.CID_LOAD_COLOR_PALETTE, bytes([2]), False),
        ("save_color_palette", (3,), RVRCommands.CID_SAVE_COLOR_PALETTE, bytes([3]), False),
        ("release_led_requests", (), RVRCommands.CID_RELEASE_LED_REQUESTS, b"", False),
        ("set_all_leds", (0x00000007, bytes([10, 20, 30])), RVRCommands.CID_SET_ALL_LEDS, struct.pack(">I", 0x00000007) + bytes([10, 20, 30]), False),
    ],
)
async def test_driver_new_command_capabilities_send_expected_packets(method_name, args, expected_command_id, expected_payload, responds):
    driver, transport = await start_driver()

    task = asyncio.create_task(getattr(driver, method_name)(*args))
    if responds:
        await respond_to_next_write(transport)
    await asyncio.wait_for(task, timeout=0.1)

    packet = latest_packet(transport)
    assert packet.command_id == expected_command_id
    assert packet.payload == expected_payload

    await driver.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("method_name", "args", "did", "cid", "source", "payload", "expected"),
    [
        ("on_will_sleep_notify", (), DID_POWER, 0x19, TARGET_BT, b"", SleepEvent("will_sleep")),
        ("on_did_sleep_notify", (), DID_POWER, 0x1A, TARGET_BT, b"", SleepEvent("did_sleep")),
        ("on_battery_voltage_state_change_notify", (), DID_POWER, 0x1C, TARGET_BT, bytes([2]), BatteryVoltageState(2, "low")),
        ("on_motor_stall_notify", (), DID_DRIVE, 0x26, TARGET_MCU, bytes([1, 1]), MotorStallEvent(1, True)),
        ("on_motor_fault_notify", (), DID_DRIVE, 0x28, TARGET_MCU, bytes([1]), MotorFaultEvent(True)),
        ("on_gyro_max_notify", (), DID_SENSOR, 0x10, TARGET_MCU, bytes([0x03]), GyroMaxEvent(0x03)),
        (
            "on_robot_to_robot_infrared_message_received_notify",
            (),
            DID_SENSOR,
            0x2C,
            TARGET_MCU,
            bytes([9]),
            InfraredMessageEvent(9),
        ),
        ("on_color_detection_notify", (), DID_SENSOR, 0x36, TARGET_BT, bytes([10, 20, 30, 99, 7]), DetectedColor(10, 20, 30, 99, 7)),
        (
            "on_streaming_service_data_notify",
            (TARGET_MCU,),
            DID_SENSOR,
            0x3D,
            TARGET_MCU,
            bytes([4, 5, 6]),
            StreamingServiceData(4, bytes([5, 6])),
        ),
        (
            "on_motor_thermal_protection_status_notify",
            (),
            DID_SENSOR,
            0x4D,
            TARGET_MCU,
            struct.pack(">fBfB", 44.0, 1, 45.0, 2),
            ThermalProtectionStatus(44.0, 1, 45.0, 2),
        ),
    ],
)
async def test_driver_notification_callbacks_route_through_dispatcher(method_name, args, did, cid, source, payload, expected):
    driver, transport = await start_driver()
    seen = []

    subscription = getattr(driver, method_name)(seen.append, *args)
    event = Packet(did, cid, 0, payload=payload, source=source, flags=FLAG_HAS_SOURCE)
    await transport.inject_read(event.encode())
    await asyncio.sleep(0)
    await asyncio.sleep(0)

    assert seen == [expected]
    if method_name == "on_battery_voltage_state_change_notify":
        assert driver.get_cached_battery_voltage_state_change() == expected

    subscription.unsubscribe()
    await driver.disconnect()
