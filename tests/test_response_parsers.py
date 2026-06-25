import struct

import pytest

from sphero_rvr_core import responses


def test_parse_battery_and_voltage_payloads():
    assert responses.parse_battery_percentage(bytes([87])) == 87
    assert responses.parse_battery_voltage(struct.pack(">f", 7.42)) == pytest.approx(7.42)
    assert responses.parse_battery_voltage_state(bytes([2])).state_name == "low"
    thresholds = responses.parse_battery_thresholds(struct.pack(">fff", 6.0, 6.8, 0.2))
    assert thresholds.critical == pytest.approx(6.0)
    assert thresholds.low == pytest.approx(6.8)
    assert thresholds.hysteresis == pytest.approx(0.2)


def test_parse_color_and_light_payloads():
    assert responses.parse_rgbc_sensor_values(struct.pack(">HHHH", 1, 2, 3, 4)).red == 1
    assert responses.parse_ambient_light(struct.pack(">f", 123.5)) == pytest.approx(123.5)
    color = responses.parse_current_detected_color(bytes([10, 20, 30, 99, 7]))
    assert color.red == 10
    assert color.confidence == 99
    assert color.color_classification_id == 7


def test_parse_temperature_thermal_encoder_and_magnetometer_payloads():
    temps = responses.parse_temperature(bytes([4]) + struct.pack(">f", 41.5) + bytes([5]) + struct.pack(">f", 42.5))
    assert temps.left_motor == pytest.approx(41.5)
    assert temps.right_motor == pytest.approx(42.5)
    thermal = responses.parse_thermal_protection_status(struct.pack(">fBfB", 44.0, 1, 45.0, 2))
    assert thermal.left_status == 1
    assert thermal.right_status == 2
    encoders = responses.parse_encoder_counts(struct.pack(">ii", -123, 456))
    assert encoders.left == -123
    assert encoders.right == 456
    mag = responses.parse_magnetometer(struct.pack(">fff", 1.25, -2.5, 3.75))
    assert mag.y == pytest.approx(-2.5)


def test_parse_system_info_payloads():
    version = responses.parse_firmware_version(struct.pack(">HHH", 1, 2, 345))
    assert version.major == 1
    assert version.revision == 345
    assert responses.parse_echo(bytes(range(16))) == bytes(range(16))
    assert responses.parse_stats_id(struct.pack(">H", 0x1234)) == 0x1234
    assert responses.parse_null_terminated_ascii(b"RVR-123\x00ignored") == "RVR-123"
    assert responses.parse_board_revision(bytes([9])) == 9
    assert responses.parse_core_uptime(struct.pack(">Q", 123456789)) == 123456789


def test_parse_motor_fault_and_ir_readings():
    assert responses.parse_motor_fault_state(bytes([1])) is True
    readings = responses.parse_ir_readings(struct.pack(">I", 0x44332211))
    assert readings.front_left == 0x11
    assert readings.front_right == 0x22
    assert readings.back_right == 0x33
    assert readings.back_left == 0x44


def test_parse_new_query_payloads():
    assert responses.parse_current_sense_amplifier_current(struct.pack(">f", 1.25)) == pytest.approx(1.25)
    assert responses.parse_bluetooth_advertising_name(b"RVR-BT\x00ignored") == "RVR-BT"

    palette = responses.parse_active_color_palette(bytes(range(48)))
    assert palette.rgb_index_bytes == bytes(range(48))
    assert palette.rgb_triplets[0] == (0, 1, 2)
    assert palette.rgb_triplets[-1] == (45, 46, 47)

    report = responses.parse_color_identification_report(bytes(range(24)))
    assert len(report.index_confidence_bytes) == 24
    assert report.entries[0].color_index == 0
    assert report.entries[0].confidence == 1
    assert report.entries[-1].color_index == 22
    assert report.entries[-1].confidence == 23


def test_parsers_reject_short_payloads():
    with pytest.raises(ValueError, match="battery percentage"):
        responses.parse_battery_percentage(b"")
    with pytest.raises(ValueError, match="firmware version"):
        responses.parse_firmware_version(b"\x00")
    with pytest.raises(ValueError, match="echo"):
        responses.parse_echo(bytes(range(15)))
    with pytest.raises(ValueError, match="active color palette"):
        responses.parse_active_color_palette(bytes(range(47)))
    with pytest.raises(ValueError, match="color identification report"):
        responses.parse_color_identification_report(bytes(range(23)))
