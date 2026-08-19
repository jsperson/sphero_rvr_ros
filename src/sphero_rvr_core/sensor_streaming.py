"""Typed RVR sensor-streaming configuration and decode.

The RVR firmware exposes a generic streaming service: you configure a *slot*
(token) on a *processor* with a list of sensor services, start it at an interval,
and receive `StreamingServiceData` events (DID_SENSOR 0x3D) carrying packed,
normalized integers. This module turns that opaque byte stream into typed float
values, and in particular into an :class:`ImuSample` suitable for a ROS
``sensor_msgs/Imu`` message.

Protocol constants are transcribed from the Sphero public SDK
(``sphero_sdk/common/sensors/`` — ``sensor_streaming_control.py``,
``sensor_stream_service.py``, ``sensor_stream_slot.py``):

* Config packing (per slot): for each enabled service, ``uint16`` big-endian
  service id + ``uint8`` data-size value, concatenated. NOT padded — the firmware
  parses a packed list of 3-byte entries, so trailing zeros read as a phantom
  service and get rejected (hardware-confirmed: padding -> bad-data-value err 7).
* Data sizes: ``eight_bit=0``, ``sixteen_bit=1``, ``thirty_two_bit=2``;
  ``byte_count = 2 ** value`` (1/2/4).
* Streamed value denormalization: ``value = min + (raw / uint_max) * (max-min)``,
  where ``raw`` is a big-endian unsigned int of ``byte_count`` bytes.
* Streamed token byte: high nibble is a status flag (``0x0`` = OK, ``0x1`` =
  invalid data), low nibble is the slot token id.
* ST processor (target 2), slot token 1, all ``thirty_two_bit``:
  ``Quaternion`` id ``0x0000`` (W,X,Y,Z in [-1, 1]); ``IMU`` id ``0x0001``
  (Pitch [-180,180], Roll [-90,90], Yaw [-180,180] deg); ``Accelerometer`` id
  ``0x0002`` (X,Y,Z in [-16, 16] g); ``Gyroscope`` id ``0x0004`` (X,Y,Z in
  [-2000, 2000] deg/s).

The wire decode above is fully specified by the SDK. The *axis mapping* from the
RVR body frame to the ROS REP-103 frame (x-forward, y-left, z-up) in
:func:`imu_sample_from_packet` is the one piece that still wants a hardware check
— it is isolated and clearly marked so a sign/axis correction is a one-line fix.
"""

from dataclasses import dataclass
import math
import struct
from typing import Dict, Optional, Sequence, Tuple

# Processor (command "target") ids.
PROCESSOR_NORDIC = 1
PROCESSOR_ST = 2

# Fixed length of the configure_streaming_service configuration payload.
STREAMING_CONFIG_LENGTH = 15

# StreamingDataSizesEnum values; byte_count = 2 ** value.
DATA_SIZE_8_BIT = 0
DATA_SIZE_16_BIT = 1
DATA_SIZE_32_BIT = 2

_UINT_MAX_BY_BYTE_COUNT = {1: 0xFF, 2: 0xFFFF, 4: 0xFFFFFFFF}

# Unit conversions to ROS REP-103 units.
_DEG_TO_RAD = math.pi / 180.0
_STANDARD_GRAVITY = 9.80665  # m/s^2 per g


@dataclass(frozen=True)
class StreamAttribute:
    """One streamed value with its denormalization range."""

    name: str
    minimum: float
    maximum: float


@dataclass(frozen=True)
class StreamService:
    """A sensor streaming service (id, data size, ordered attributes, processor)."""

    service_id: int
    name: str
    data_size: int
    attributes: Tuple[StreamAttribute, ...]
    processor: int

    @property
    def byte_count(self) -> int:
        return 2 ** self.data_size


# --- The RVR streaming service table (ST processor, slot token 1) -------------

QUATERNION = StreamService(
    0x0000,
    "Quaternion",
    DATA_SIZE_32_BIT,
    (
        StreamAttribute("W", -1.0, 1.0),
        StreamAttribute("X", -1.0, 1.0),
        StreamAttribute("Y", -1.0, 1.0),
        StreamAttribute("Z", -1.0, 1.0),
    ),
    PROCESSOR_ST,
)

IMU_ATTITUDE = StreamService(
    0x0001,
    "IMU",
    DATA_SIZE_32_BIT,
    (
        StreamAttribute("Pitch", -180.0, 180.0),
        StreamAttribute("Roll", -90.0, 90.0),
        StreamAttribute("Yaw", -180.0, 180.0),
    ),
    PROCESSOR_ST,
)

ACCELEROMETER = StreamService(
    0x0002,
    "Accelerometer",
    DATA_SIZE_32_BIT,
    (
        StreamAttribute("X", -16.0, 16.0),
        StreamAttribute("Y", -16.0, 16.0),
        StreamAttribute("Z", -16.0, 16.0),
    ),
    PROCESSOR_ST,
)

GYROSCOPE = StreamService(
    0x0004,
    "Gyroscope",
    DATA_SIZE_32_BIT,
    (
        StreamAttribute("X", -2000.0, 2000.0),
        StreamAttribute("Y", -2000.0, 2000.0),
        StreamAttribute("Z", -2000.0, 2000.0),
    ),
    PROCESSOR_ST,
)

# Ordered set streamed for IMU fusion, all on ST processor, slot token 1. The
# order here MUST match the order used to build the configuration, because the
# firmware packs the streamed values in configuration order.
IMU_STREAM_SERVICES: Tuple[StreamService, ...] = (
    QUATERNION,
    ACCELEROMETER,
    GYROSCOPE,
)

# Slot token used for the IMU fusion set.
IMU_SLOT_TOKEN = 1


@dataclass(frozen=True)
class StreamingPacket:
    """A decoded streaming packet: per-service {attribute_name: float}."""

    token_id: int
    is_valid: bool
    services: Dict[str, Dict[str, float]]


@dataclass(frozen=True)
class ImuSample:
    """IMU reading in ROS REP-103 units/axes.

    orientation is (x, y, z, w) (ROS quaternion order); angular_velocity is
    (x, y, z) rad/s; linear_acceleration is (x, y, z) m/s^2.
    """

    orientation: Tuple[float, float, float, float]
    angular_velocity: Tuple[float, float, float]
    linear_acceleration: Tuple[float, float, float]
    is_valid: bool


def build_slot_configuration(services: Sequence[StreamService]) -> bytes:
    """Build the variable-length configuration payload for one slot.

    Each service contributes exactly ``uint16`` big-endian id + ``uint8`` data
    size (3 bytes). The payload is NOT zero-padded: the RVR firmware parses the
    config as a packed list of 3-byte service entries, so trailing zero bytes
    would be read as a phantom service (id 0x0000 = Quaternion at data-size 0)
    and rejected with a bad-data-value error. Max :data:`STREAMING_CONFIG_LENGTH`
    bytes (5 services).
    """
    data = bytearray()
    for service in services:
        data += struct.pack(">H", service.service_id & 0xFFFF)
        data.append(service.data_size & 0xFF)
    if len(data) > STREAMING_CONFIG_LENGTH:
        raise ValueError(
            "streaming configuration exceeds {} bytes ({} services)".format(
                STREAMING_CONFIG_LENGTH, len(services)
            )
        )
    return bytes(data)


def _denormalize(raw: int, raw_max: int, minimum: float, maximum: float) -> float:
    return minimum + (raw / raw_max) * (maximum - minimum)


def decode_streaming_packet(
    token_byte: int, sensor_data: bytes, services: Sequence[StreamService]
) -> StreamingPacket:
    """Decode one streaming packet into typed float values.

    ``services`` must be the same ordered list used to configure the slot.
    Raises ``ValueError`` if ``sensor_data`` is shorter than the services imply.
    """
    status = (token_byte & 0xF0) >> 4
    token_id = token_byte & 0x0F
    is_valid = status == 0

    decoded: Dict[str, Dict[str, float]] = {}
    index = 0
    for service in services:
        byte_count = service.byte_count
        raw_max = _UINT_MAX_BY_BYTE_COUNT[byte_count]
        attributes: Dict[str, float] = {}
        for attribute in service.attributes:
            end = index + byte_count
            if end > len(sensor_data):
                raise ValueError(
                    "streaming payload too short for {}.{}: need {} bytes at "
                    "offset {}, have {}".format(
                        service.name, attribute.name, byte_count, index, len(sensor_data)
                    )
                )
            raw = int.from_bytes(sensor_data[index:end], "big")
            attributes[attribute.name] = _denormalize(
                raw, raw_max, attribute.minimum, attribute.maximum
            )
            index = end
        decoded[service.name] = attributes
    return StreamingPacket(token_id=token_id, is_valid=is_valid, services=decoded)


def imu_sample_from_packet(packet: StreamingPacket) -> Optional[ImuSample]:
    """Extract an :class:`ImuSample` (ROS units/axes) from a decoded packet.

    Returns ``None`` if the packet lacks the Quaternion/Accelerometer/Gyroscope
    services. Requires the services decoded by :data:`IMU_STREAM_SERVICES`.

    Axis mapping: IDENTITY (only the quaternion is reordered from RVR (W,X,Y,Z)
    to ROS (x,y,z,w)). Verified on hardware 2026-08-03 against wheel odometry
    (the established ROS convention): at rest accel Z = +1g (z-up), and a
    wheel-CCW spin (ROS +yaw) gave a POSITIVE raw gyro Z — so angular_velocity.z
    is +raw. The RVR IMU frame is x-forward, y-left, z-up = ROS REP-103.

    NOTE: the earlier "fused yaw runs backwards" symptom was NOT an axis-sign
    bug — it was the EKF fusing the RVR *absolute-orientation quaternion*
    differentially, which chokes on quaternion wrapping during a fast multi-turn
    spin. The fix was to fuse the gyro yaw-RATE only (see config/ekf.yaml); the
    gyro sign is identity, confirmed by the vyaw-only EKF tracking wheel odom.
    """
    quat = packet.services.get("Quaternion")
    gyro = packet.services.get("Gyroscope")
    accel = packet.services.get("Accelerometer")
    if quat is None or gyro is None or accel is None:
        return None

    # Identity mapping; only reorder the quaternion (W,X,Y,Z) -> (x,y,z,w).
    orientation = (
        quat["X"],
        quat["Y"],
        quat["Z"],
        quat["W"],
    )
    # Gyroscope deg/s -> rad/s.
    angular_velocity = (
        gyro["X"] * _DEG_TO_RAD,
        gyro["Y"] * _DEG_TO_RAD,
        gyro["Z"] * _DEG_TO_RAD,
    )
    # Accelerometer g -> m/s^2.
    linear_acceleration = (
        accel["X"] * _STANDARD_GRAVITY,
        accel["Y"] * _STANDARD_GRAVITY,
        accel["Z"] * _STANDARD_GRAVITY,
    )
    return ImuSample(
        orientation=orientation,
        angular_velocity=angular_velocity,
        linear_acceleration=linear_acceleration,
        is_valid=packet.is_valid,
    )


def firmware_heading_deg(sample: "ImuSample") -> float:
    """The firmware's own yaw angle, in degrees [0, 360), from a streamed sample.

    This is the number `drive_with_heading` steers to -- the firmware's internal
    reference, NOT ROS yaw. The known convention caveat, stated rather than
    resolved here: the fused yaw ANGLE "runs backwards" relative to REP-103 (the
    EKF fuses only the yaw RATE for exactly that reason), so the sign relation
    between this heading and a ROS-frame turn direction is an ASSUMPTION until
    the bench card measures it (docs/bench_card_2026-08-19.md, item iii). The
    precise-turn primitive is un-runnable until that card passes, so the
    assumption cannot reach a motor before it is measured.
    """
    x, y, z, w = sample.orientation
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(yaw) % 360.0
