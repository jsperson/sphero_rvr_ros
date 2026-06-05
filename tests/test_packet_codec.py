import pytest

from sphero_rvr_core.packet import (
    EOP,
    ESC,
    ESC_EOP,
    ESC_ESC,
    ESC_SOP,
    FLAG_HAS_SOURCE,
    FLAG_HAS_TARGET,
    FLAG_IS_ACTIVITY,
    FLAG_IS_RESPONSE,
    FLAG_REQUEST_RESPONSE,
    SOP,
    Packet,
    checksum,
    escape_buffer,
    unescape_buffer,
)


def test_escape_buffer_escapes_protocol_delimiters():
    assert escape_buffer(bytes([SOP, EOP, ESC, 0x01])) == bytes([
        ESC, ESC_SOP,
        ESC, ESC_EOP,
        ESC, ESC_ESC,
        0x01,
    ])


def test_unescape_buffer_rejects_invalid_escape_sequence():
    with pytest.raises(ValueError, match="Invalid escape sequence"):
        unescape_buffer(bytes([ESC, 0x99]))


def test_packet_encode_builds_real_rvr_frame_with_checksum():
    packet = Packet(
        device_id=0x16,
        command_id=0x07,
        sequence_id=0x2A,
        target=0x02,
        payload=bytes([0x40, 0x00, 0x5A, 0x00]),
        flags=FLAG_HAS_TARGET | FLAG_IS_ACTIVITY | FLAG_REQUEST_RESPONSE,
    )

    encoded = packet.encode()
    content = unescape_buffer(encoded[1:-1])

    assert encoded[0] == SOP
    assert encoded[-1] == EOP
    assert content[:-1] == bytes([
        FLAG_HAS_TARGET | FLAG_IS_ACTIVITY | FLAG_REQUEST_RESPONSE,
        0x02,
        0x16,
        0x07,
        0x2A,
        0x40,
        0x00,
        0x5A,
        0x00,
    ])
    assert content[-1] == checksum(content[:-1])
    assert (sum(content) & 0xFF) == 0xFF


def test_packet_decode_parses_response_and_skips_error_byte():
    flags = FLAG_IS_RESPONSE | FLAG_HAS_SOURCE
    content = bytes([flags, 0x02, 0x13, 0x10, 0x33, 0x00, 87])
    raw = bytes([SOP]) + escape_buffer(content + bytes([checksum(content)])) + bytes([EOP])

    packet = Packet.decode(raw)

    assert packet.flags == flags
    assert packet.source == 0x02
    assert packet.device_id == 0x13
    assert packet.command_id == 0x10
    assert packet.sequence_id == 0x33
    assert packet.error == 0
    assert packet.payload == bytes([87])


def test_packet_decode_rejects_bad_checksum():
    raw = bytes([SOP, FLAG_IS_RESPONSE, 0x13, 0x10, 0x01, 0x00, EOP])

    with pytest.raises(ValueError, match="Bad checksum"):
        Packet.decode(raw)
