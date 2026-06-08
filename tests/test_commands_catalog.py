import pytest

from sphero_rvr_core.commands import LED_GROUPS, RVRCommands
from sphero_rvr_core.packet import (
    DID_DRIVE,
    DID_IO,
    DID_POWER,
    DID_SENSOR,
    DID_SYSTEM_INFO,
    FLAG_HAS_TARGET,
    FLAG_IS_ACTIVITY,
    FLAG_REQUEST_RESPONSE,
    TARGET_BT,
    TARGET_MCU,
)


def test_wake_builds_power_wake_packet_targeting_bt_core():
    packet = RVRCommands().wake(sequence_id=0x2A)

    assert packet.device_id == DID_POWER
    assert packet.command_id == RVRCommands.CID_WAKE
    assert packet.sequence_id == 0x2A
    assert packet.target == TARGET_BT
    assert packet.payload == b""


def test_drive_with_heading_packs_speed_heading_and_flags_for_mcu():
    packet = RVRCommands().drive_with_heading(
        sequence_id=0x11,
        speed=64,
        heading=90,
        flags=1,
    )

    assert packet.device_id == DID_DRIVE
    assert packet.command_id == RVRCommands.CID_DRIVE_WITH_HEADING
    assert packet.target == TARGET_MCU
    assert packet.payload == bytes([64, 0, 90, 1])


def test_stop_is_zero_speed_drive_with_heading():
    packet = RVRCommands().stop(sequence_id=0x12)

    assert packet.device_id == DID_DRIVE
    assert packet.command_id == RVRCommands.CID_DRIVE_WITH_HEADING
    assert packet.target == TARGET_MCU
    assert packet.payload == bytes([0, 0, 0, 0])


def test_raw_motors_packs_left_and_right_modes_and_speeds():
    packet = RVRCommands().raw_motors(
        sequence_id=0x13,
        left_mode=1,
        left_speed=120,
        right_mode=2,
        right_speed=80,
    )

    assert packet.device_id == DID_DRIVE
    assert packet.command_id == RVRCommands.CID_RAW_MOTORS
    assert packet.target == TARGET_MCU
    assert packet.payload == bytes([1, 120, 2, 80])


def test_set_all_leds_uses_full_30_channel_bitmap_and_rgb_repeated():
    packet = RVRCommands().set_all_leds(sequence_id=0x14, r=1, g=2, b=3)

    assert packet.device_id == DID_IO
    assert packet.command_id == RVRCommands.CID_SET_ALL_LEDS
    assert packet.target == TARGET_BT
    assert packet.payload[:4] == bytes([0x3F, 0xFF, 0xFF, 0xFF])
    assert packet.payload[4:] == bytes([1, 2, 3] * 10)


def test_set_led_group_sets_only_requested_group_channels():
    packet = RVRCommands().set_led_group(
        sequence_id=0x15,
        group_name="brakelight_left",
        r=10,
        g=20,
        b=30,
    )

    base_bit = LED_GROUPS["brakelight_left"] * 3
    expected_bitmap = (1 << base_bit) | (1 << (base_bit + 1)) | (1 << (base_bit + 2))
    assert packet.device_id == DID_IO
    assert packet.command_id == RVRCommands.CID_SET_ALL_LEDS
    assert packet.payload[:4] == expected_bitmap.to_bytes(4, "big")
    brightness = packet.payload[4:]
    assert len(brightness) == 30
    assert brightness[base_bit:base_bit + 3] == bytes([10, 20, 30])
    assert sum(brightness) == 60


def test_set_led_group_rejects_unknown_group_name():
    with pytest.raises(ValueError, match="Unknown LED group"):
        RVRCommands().set_led_group(0x16, "not_a_light", 1, 2, 3)


def test_query_commands_request_responses():
    commands = RVRCommands()

    battery = commands.get_battery_percentage(sequence_id=0x17)
    ambient = commands.get_ambient_light(sequence_id=0x18)
    mac = commands.get_mac_address(sequence_id=0x19)

    assert battery.device_id == DID_POWER
    assert battery.command_id == RVRCommands.CID_GET_BATTERY_PERCENTAGE
    assert battery.flags & FLAG_REQUEST_RESPONSE
    assert battery.flags & FLAG_HAS_TARGET
    assert battery.flags & FLAG_IS_ACTIVITY

    assert ambient.device_id == DID_SENSOR
    assert ambient.command_id == RVRCommands.CID_GET_AMBIENT_LIGHT
    assert ambient.flags & FLAG_REQUEST_RESPONSE

    assert mac.device_id == DID_SYSTEM_INFO
    assert mac.command_id == RVRCommands.CID_GET_MAC_ADDRESS
    assert mac.flags & FLAG_REQUEST_RESPONSE
