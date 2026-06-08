import asyncio
import struct

import pytest

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import FLAG_HAS_SOURCE, FLAG_IS_RESPONSE, Packet
from sphero_rvr_core.responses import BatteryVoltageState, EncoderCounts, FirmwareVersion


async def respond_to_next_write(transport: FakeTransport, payload: bytes = b"") -> Packet:
    await transport.wait_for_write()
    request = Packet.decode(transport.writes[-1])
    response = Packet(
        request.device_id,
        request.command_id,
        request.sequence_id,
        payload=payload,
        source=request.target,
        target=None,
        flags=FLAG_IS_RESPONSE | FLAG_HAS_SOURCE,
        error=0,
    )
    await transport.inject_read(response.encode())
    return request


@pytest.mark.asyncio
async def test_driver_query_methods_parse_typed_responses():
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(transport=transport)
    await driver.connect()
    transport.auto_ack = False

    battery_task = asyncio.create_task(driver.get_battery_percentage())
    battery_request = await respond_to_next_write(transport, bytes([88]))
    assert await battery_task == 88
    assert battery_request.command_id == driver.commands.CID_GET_BATTERY_PERCENTAGE

    voltage_state_task = asyncio.create_task(driver.get_battery_voltage_state())
    await respond_to_next_write(transport, bytes([3]))
    voltage_state = await voltage_state_task
    assert isinstance(voltage_state, BatteryVoltageState)
    assert voltage_state.state_name == "critical"

    encoder_task = asyncio.create_task(driver.get_encoder_counts())
    await respond_to_next_write(transport, struct.pack(">ii", -5, 12))
    encoders = await encoder_task
    assert isinstance(encoders, EncoderCounts)
    assert encoders.left == -5
    assert encoders.right == 12

    version_task = asyncio.create_task(driver.get_firmware_version(target=0x02))
    await respond_to_next_write(transport, struct.pack(">HHH", 1, 2, 3))
    assert await version_task == FirmwareVersion(1, 2, 3)

    await driver.disconnect()


@pytest.mark.asyncio
async def test_driver_fire_and_forget_methods_use_command_catalog():
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(transport=transport)
    await driver.connect()

    await driver.set_all_leds(1, 2, 3)
    await driver.reset_yaw()
    await driver.send_ir_message(code=4, strength=32)

    packets = [Packet.decode(raw) for raw in transport.writes]
    command_ids = [packet.command_id for packet in packets]
    assert driver.commands.CID_SET_ALL_LEDS in command_ids
    assert driver.commands.CID_RESET_YAW in command_ids
    assert driver.commands.CID_SEND_IR_MESSAGE in command_ids

    await driver.disconnect()
