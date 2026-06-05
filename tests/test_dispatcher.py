import asyncio

import pytest

from sphero_rvr_core.dispatcher import Dispatcher
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import FLAG_HAS_SOURCE, FLAG_IS_RESPONSE, Packet


@pytest.mark.asyncio
async def test_dispatcher_matches_response_by_sequence_id():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    await dispatcher.start()

    request = Packet(device_id=1, command_id=2, sequence_id=7, payload=b"go")
    pending = asyncio.create_task(dispatcher.request(request, timeout=0.2))
    await transport.wait_for_write()

    await transport.inject_read(Packet(
        device_id=1,
        command_id=2,
        sequence_id=7,
        payload=b"ok",
        flags=FLAG_IS_RESPONSE | FLAG_HAS_SOURCE,
        source=1,
        error=0,
    ).encode())

    response = await pending
    await dispatcher.stop()

    assert response.payload == b"ok"
    assert transport.writes == [request.encode()]


@pytest.mark.asyncio
async def test_dispatcher_times_out_missing_response_without_hanging():
    transport = FakeTransport()
    dispatcher = Dispatcher(transport)
    await dispatcher.start()

    with pytest.raises(TimeoutError):
        await dispatcher.request(Packet(device_id=1, command_id=2, sequence_id=8), timeout=0.01)

    await dispatcher.stop()
