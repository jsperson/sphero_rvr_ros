import struct

import pytest

from sphero_rvr_core.commands import RVRCommands
from sphero_rvr_core.packet import (
    DID_DRIVE,
    DID_IO,
    DID_POWER,
    DID_SENSOR,
    DID_SYSTEM_INFO,
    FLAG_REQUEST_RESPONSE,
    TARGET_BT,
    TARGET_MCU,
)

DID_API_AND_SHELL = 0x10
DID_CONNECTION = 0x19


def assert_packet(packet, *, did, cid, target, payload=b"", request_response=False):
    assert packet.device_id == did
    assert packet.command_id == cid
    assert packet.target == target
    assert packet.payload == payload
    if request_response:
        assert packet.flags & FLAG_REQUEST_RESPONSE
    else:
        assert not packet.flags & FLAG_REQUEST_RESPONSE


@pytest.mark.parametrize(
    ("builder", "args", "did", "cid", "target", "payload", "request_response"),
    [
        ("echo", (bytes(range(16)), TARGET_MCU), DID_API_AND_SHELL, 0x00, TARGET_MCU, bytes(range(16)), True),
        ("get_bootloader_version", (TARGET_MCU,), DID_SYSTEM_INFO, 0x01, TARGET_MCU, b"", True),
        ("get_stats_id", (), DID_SYSTEM_INFO, 0x13, TARGET_BT, b"", True),
        ("sleep", (), DID_POWER, 0x01, TARGET_BT, b"", False),
        ("enable_battery_voltage_state_change_notify", (False,), DID_POWER, 0x1B, TARGET_BT, bytes([0]), False),
        ("get_current_sense_amplifier_current", (1,), DID_POWER, 0x27, TARGET_BT, bytes([1]), True),
        ("enable_gyro_max_notify", (True,), DID_SENSOR, 0x0F, TARGET_MCU, bytes([1]), False),
        ("set_locator_flags", (0x80,), DID_SENSOR, 0x17, TARGET_MCU, bytes([0x80]), False),
        (
            "configure_streaming_service",
            (3, bytes(range(15)), TARGET_MCU),
            DID_SENSOR,
            0x39,
            TARGET_MCU,
            bytes([3]) + bytes(range(15)),
            False,
        ),
        ("start_streaming_service", (100, TARGET_MCU), DID_SENSOR, 0x3A, TARGET_MCU, struct.pack(">H", 100), False),
        ("stop_streaming_service", (TARGET_MCU,), DID_SENSOR, 0x3B, TARGET_MCU, b"", False),
        ("clear_streaming_service", (TARGET_BT,), DID_SENSOR, 0x3C, TARGET_BT, b"", False),
        ("enable_robot_infrared_message_notify", (True,), DID_SENSOR, 0x3E, TARGET_MCU, bytes([1]), False),
        ("enable_motor_thermal_protection_status_notify", (False,), DID_SENSOR, 0x4C, TARGET_MCU, bytes([0]), False),
        ("get_bluetooth_advertising_name", (), DID_CONNECTION, 0x05, TARGET_BT, b"", True),
        ("get_active_color_palette", (), DID_IO, 0x44, TARGET_BT, b"", True),
        ("set_active_color_palette", (bytes(range(48)),), DID_IO, 0x45, TARGET_BT, bytes(range(48)), False),
        (
            "get_color_identification_report",
            (1, 2, 3, 4),
            DID_IO,
            0x46,
            TARGET_BT,
            bytes([1, 2, 3, 4]),
            True,
        ),
        ("load_color_palette", (2,), DID_IO, 0x47, TARGET_BT, bytes([2]), False),
        ("save_color_palette", (3,), DID_IO, 0x48, TARGET_BT, bytes([3]), False),
        ("release_led_requests", (), DID_IO, 0x4E, TARGET_BT, b"", False),
    ],
)
def test_missing_matrix_builders_emit_protocol_payloads(builder, args, did, cid, target, payload, request_response):
    packet = getattr(RVRCommands(), builder)(0x42, *args)

    assert packet.sequence_id == 0x42
    assert_packet(packet, did=did, cid=cid, target=target, payload=payload, request_response=request_response)


def test_battery_voltage_builder_accepts_matrix_reading_type_payload():
    packet = RVRCommands().get_battery_voltage(0x10, reading_type=2)

    assert_packet(
        packet,
        did=DID_POWER,
        cid=RVRCommands.CID_GET_BATTERY_VOLTAGE,
        target=TARGET_BT,
        payload=bytes([2]),
        request_response=True,
    )


def test_set_all_leds_accepts_official_mask_and_variable_brightness_vector():
    packet = RVRCommands().set_all_leds(0x11, led_group=0x00000007, led_brightness_values=bytes([10, 20, 30]))

    assert_packet(
        packet,
        did=DID_IO,
        cid=RVRCommands.CID_SET_ALL_LEDS,
        target=TARGET_BT,
        payload=struct.pack(">I", 0x00000007) + bytes([10, 20, 30]),
    )


@pytest.mark.parametrize(
    ("call", "match"),
    [
        (lambda c: c.echo(1, bytes(range(15))), "exactly 16 bytes"),
        (lambda c: c.configure_streaming_service(1, 0, bytes(range(16))), "1 to 15 bytes"),
        (lambda c: c.set_active_color_palette(1, bytes(range(47))), "exactly 48 bytes"),
        (lambda c: c.set_all_leds(1, led_group=1, led_brightness_values=b""), "1 to 32 bytes"),
        (lambda c: c.set_all_leds(1, led_group=1, led_brightness_values=bytes(range(33))), "1 to 32 bytes"),
    ],
)
def test_variable_length_protocol_builders_validate_payload_shapes(call, match):
    with pytest.raises(ValueError, match=match):
        call(RVRCommands())
