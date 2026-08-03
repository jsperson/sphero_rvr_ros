"""ROS-free tests for the RVR sensor-streaming config/decode layer."""

import math
import struct

import pytest

from sphero_rvr_core import sensor_streaming as ss


def _pack32(value: float, minimum: float, maximum: float) -> bytes:
    """Encode a float as the RVR does: normalized big-endian uint32."""
    fraction = (value - minimum) / (maximum - minimum)
    raw = round(fraction * 0xFFFFFFFF)
    return struct.pack(">I", raw)


def test_build_slot_configuration_packs_id_and_data_size_in_order():
    config = ss.build_slot_configuration(ss.IMU_STREAM_SERVICES)
    # Quaternion 0x0000 sz2, Accelerometer 0x0002 sz2, Gyroscope 0x0004 sz2.
    # NOT padded: exactly 3 bytes/service (padding would parse as a phantom
    # service and the firmware rejects it with bad-data-value).
    assert config == bytes([0x00, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x04, 0x02])
    assert len(config) == 9


def test_build_slot_configuration_rejects_overlong_service_list():
    too_many = (ss.QUATERNION,) * 6  # 6 * 3 = 18 bytes > 15
    with pytest.raises(ValueError):
        ss.build_slot_configuration(too_many)


def test_byte_count_follows_data_size():
    assert ss.QUATERNION.byte_count == 4
    assert ss.StreamService(0, "s", ss.DATA_SIZE_8_BIT, (), ss.PROCESSOR_ST).byte_count == 1
    assert ss.StreamService(0, "s", ss.DATA_SIZE_16_BIT, (), ss.PROCESSOR_ST).byte_count == 2


def test_denormalize_endpoints_and_midpoint():
    service = ss.QUATERNION  # attrs in [-1, 1]
    data = _pack32(-1.0, -1.0, 1.0) + _pack32(0.0, -1.0, 1.0) + _pack32(1.0, -1.0, 1.0)
    data += _pack32(1.0, -1.0, 1.0)  # 4 attrs W,X,Y,Z
    packet = ss.decode_streaming_packet(0x01, data, (service,))
    q = packet.services["Quaternion"]
    assert q["W"] == pytest.approx(-1.0, abs=1e-6)
    assert q["X"] == pytest.approx(0.0, abs=1e-6)
    assert q["Y"] == pytest.approx(1.0, abs=1e-6)
    assert q["Z"] == pytest.approx(1.0, abs=1e-6)


def test_token_byte_status_and_id():
    data = bytes(16)  # Quaternion = 4 attrs * 4 bytes
    ok = ss.decode_streaming_packet(0x01, data, (ss.QUATERNION,))
    assert ok.token_id == 1 and ok.is_valid is True
    invalid = ss.decode_streaming_packet(0x11, data, (ss.QUATERNION,))
    assert invalid.token_id == 1 and invalid.is_valid is False


def test_decode_multi_service_slices_in_order():
    quat = b"".join(_pack32(v, -1.0, 1.0) for v in (1.0, 0.0, 0.0, 0.0))
    accel = b"".join(_pack32(v, -16.0, 16.0) for v in (0.0, 0.0, 1.0))
    gyro = b"".join(_pack32(v, -2000.0, 2000.0) for v in (0.0, 0.0, 90.0))
    packet = ss.decode_streaming_packet(0x01, quat + accel + gyro, ss.IMU_STREAM_SERVICES)
    assert packet.services["Quaternion"]["W"] == pytest.approx(1.0, abs=1e-6)
    assert packet.services["Accelerometer"]["Z"] == pytest.approx(1.0, abs=1e-4)
    assert packet.services["Gyroscope"]["Z"] == pytest.approx(90.0, abs=1e-3)


def test_decode_raises_on_short_payload():
    with pytest.raises(ValueError):
        ss.decode_streaming_packet(0x01, bytes(8), (ss.QUATERNION,))  # need 16


def test_imu_sample_converts_units_and_axes():
    # identity quaternion, +1g on Z (RVR frame), +90 deg/s yaw about RVR z.
    quat = b"".join(_pack32(v, -1.0, 1.0) for v in (1.0, 0.0, 0.0, 0.0))
    accel = b"".join(_pack32(v, -16.0, 16.0) for v in (0.0, 0.0, 1.0))
    gyro = b"".join(_pack32(v, -2000.0, 2000.0) for v in (0.0, 0.0, 90.0))
    packet = ss.decode_streaming_packet(0x01, quat + accel + gyro, ss.IMU_STREAM_SERVICES)
    sample = ss.imu_sample_from_packet(packet)
    assert sample is not None
    # orientation reordered RVR (W,X,Y,Z) -> ROS (x,y,z,w); w kept.
    assert sample.orientation[3] == pytest.approx(1.0, abs=1e-6)
    # gyro deg/s -> rad/s; Z negated (y-reflection): RVR +Z rate -> ROS -Z.
    assert sample.angular_velocity[2] == pytest.approx(-90.0 * math.pi / 180.0, abs=1e-3)
    # accel g -> m/s^2; Z is NOT negated (z-up reads +1g at rest); only Y flips.
    assert sample.linear_acceleration[2] == pytest.approx(9.80665, abs=1e-2)


def test_imu_sample_none_when_services_missing():
    data = bytes(16)
    packet = ss.decode_streaming_packet(0x01, data, (ss.QUATERNION,))  # no gyro/accel
    assert ss.imu_sample_from_packet(packet) is None
