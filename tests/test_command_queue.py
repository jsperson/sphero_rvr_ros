import asyncio

import pytest

from sphero_rvr_core.command_queue import CommandPriority, PriorityCommandQueue


@pytest.mark.asyncio
async def test_commands_are_serialized_by_priority_then_sequence():
    events: list[str] = []
    queue = PriorityCommandQueue()
    await queue.start()

    async def record(name: str):
        events.append(f"start:{name}")
        await asyncio.sleep(0)
        events.append(f"end:{name}")
        return name

    low = asyncio.create_task(queue.submit(lambda: record("low"), priority=CommandPriority.LOW))
    await asyncio.sleep(0)
    emergency = asyncio.create_task(queue.submit(lambda: record("estop"), priority=CommandPriority.EMERGENCY))
    normal = asyncio.create_task(queue.submit(lambda: record("normal"), priority=CommandPriority.NORMAL))

    assert await low == "low"
    assert await emergency == "estop"
    assert await normal == "normal"
    await queue.stop()

    assert events == [
        "start:low",
        "end:low",
        "start:estop",
        "end:estop",
        "start:normal",
        "end:normal",
    ]
