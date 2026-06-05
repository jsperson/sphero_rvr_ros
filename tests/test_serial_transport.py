import pytest

from sphero_rvr_core.packet import Packet
from sphero_rvr_core.serial_transport import SerialTransport


class _FakeSerial:
    def __init__(self, chunks):
        self._buffer = bytearray(b"".join(chunks))
        self.is_open = True
        self.closed = False
        self.written = []
        self.flushed = False

    def read(self, n):
        if not self._buffer:
            return b""
        data = self._buffer[:n]
        del self._buffer[:n]
        return bytes(data)

    def write(self, data):
        self.written.append(data)
        return len(data)

    def flush(self):
        self.flushed = True

    def close(self):
        self.closed = True
        self.is_open = False


@pytest.mark.asyncio
async def test_serial_transport_reads_until_real_rvr_eop_frame():
    packet = Packet(device_id=0x13, command_id=0x10, sequence_id=0x22, payload=b"abc").encode()
    transport = SerialTransport()
    transport._serial = _FakeSerial([b"noise", packet[:3], packet[3:]])

    assert await transport.read_packet() == packet


@pytest.mark.asyncio
async def test_serial_transport_times_out_if_stream_ends_mid_frame():
    packet = Packet(device_id=0x13, command_id=0x10, sequence_id=0x22, payload=b"abc").encode()
    transport = SerialTransport()
    transport._serial = _FakeSerial([packet[:-1]])

    with pytest.raises(TimeoutError, match="complete packet"):
        await transport.read_packet()
