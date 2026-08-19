"""D31: disconnect() must terminate whether or not its cancellation is delivered.

Python 3.9's asyncio.wait_for (this Mac; the Pi runs 3.12 where it was
reimplemented cancellation-safe) can CONSUME a cancellation that lands in the
same iteration its inner future resolves. The control loop round-trips through
exactly that primitive on every ack, so `disconnect()`'s old cancel-then-bare-
await hung when the race fired: 6/360 runs on an idle host, >60 s each
(diagnostics/disconnect_hang_probe.py). The race is 1.67% probabilistic; these
tests FORCE the post-race state instead of racing for it, which turns the hang
into a deterministic contract:

  * cancellation eaten, flag honored  -> disconnect returns within a period
  * cancellation AND flag both ignored -> disconnect raises within the derived
    bound -- loudly failed shutdown, never a silent hang (the un-grantable-by-
    construction check applied to our own new guarantee)
  * called twice -> the second call is a no-op

On the pre-fix driver the first test hangs its bound and fails: the must-flip.
"""

import asyncio
import time

import pytest

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport


def _driver() -> RVRDriver:
    return RVRDriver(
        transport=FakeTransport(auto_ack=True),
        control_period=0.01,
        command_timeout=0.03,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
    )


async def test_disconnect_survives_an_eaten_cancellation():
    """THE MUST-FLIP. The stub swallows CancelledError -- the exact state the
    3.9 wait_for race leaves the real loop in -- but honors the cooperative
    flag, as the real loop now does. Pre-fix (no flag, bare await) this
    disconnect never returns."""
    driver = _driver()
    await driver.connect()

    async def eaten_cancel_loop():
        # getattr, NOT attribute access: on the PRE-FIX driver the flag does
        # not exist, and an AttributeError would kill this stub instantly --
        # letting the old disconnect pass this test vacuously. The must-flip
        # must actually flip.
        while not getattr(driver, "_control_stopping", False):
            try:
                await asyncio.sleep(0.01)
            except asyncio.CancelledError:
                # the race, forced: cancellation delivered and lost
                pass

    real = driver._control_task
    real.cancel()
    try:
        await real
    except asyncio.CancelledError:
        pass
    driver._control_task = asyncio.get_running_loop().create_task(
        eaten_cancel_loop())
    # The stub must be RUNNING before disconnect cancels it: a task cancelled
    # before its first step completes instantly (never enters its body), which
    # makes this test pass vacuously against ANY disconnect. Caught live: the
    # first version of both these tests did exactly that.
    await asyncio.sleep(0.02)

    started = time.monotonic()
    try:
        await asyncio.wait_for(driver.disconnect(), timeout=5.0)
    except asyncio.TimeoutError:
        pytest.fail("disconnect hung on an eaten cancellation -- D31's exact "
                    "field shape, the state this fix exists for")
    assert time.monotonic() - started < 1.0, (
        "disconnect returned but took longer than the cooperative exit "
        "should ever need (one control period plus scheduling)")


async def test_disconnect_fails_loudly_when_the_task_ignores_everything():
    """A task that ignores BOTH the cancel and the flag is genuinely stuck, and
    the contract says shutdown FAILS LOUDLY at the derived bound rather than
    blocking forever: a leaked task with a RuntimeError naming it beats a
    silent hang every time (the register row's own words)."""
    driver = _driver()
    await driver.connect()

    give_up = asyncio.Event()

    async def immortal_loop():
        while not give_up.is_set():
            try:
                await asyncio.sleep(0.01)
            except asyncio.CancelledError:
                pass

    real = driver._control_task
    real.cancel()
    try:
        await real
    except asyncio.CancelledError:
        pass
    driver._control_task = asyncio.get_running_loop().create_task(
        immortal_loop())
    await asyncio.sleep(0.02)     # same vacuity trap as above: stub must RUN

    started = time.monotonic()
    with pytest.raises(RuntimeError, match="survived cancel"):
        await driver.disconnect()
    elapsed = time.monotonic() - started
    assert elapsed >= RVRDriver.DISCONNECT_JOIN_TIMEOUT_S - 0.1, (
        "the loud failure fired before the derived bound -- a live-but-slow "
        "loop would be misconvicted")
    assert elapsed < RVRDriver.DISCONNECT_JOIN_TIMEOUT_S + 2.0

    # A failed disconnect leaves the driver honestly half-torn (the leak is
    # real and named; queue + dispatcher deliberately not stopped past a
    # failure). Recovery semantics: release the stuck task and disconnect
    # AGAIN -- the retry must now succeed and finish the teardown, which is
    # also why the idempotency guard keys on _connected AND _control_task.
    give_up.set()
    await asyncio.sleep(0.03)
    await driver.disconnect()


async def test_disconnect_twice_is_a_no_op():
    """Teardown paths love calling disconnect twice; the second must return
    immediately -- no second cancel/await cycle, no RuntimeError against a
    finished task (ratified addition, 2026-08-19)."""
    driver = _driver()
    await driver.connect()
    await driver.disconnect()

    started = time.monotonic()
    await driver.disconnect()
    assert time.monotonic() - started < 0.1, (
        "the second disconnect did real work; it must be a no-op")


async def test_a_crashed_control_task_surfaces_through_disconnect():
    """Review amendment to ff03e02: the old bare `await` re-raised a control
    loop that died of a genuine bug; the first version of the fix retrieved
    and silently DROPPED it -- an error-visibility regression on the driver
    seam. A crash must stay loud through disconnect, and the retry after the
    crash surfaced must still complete the teardown."""
    driver = _driver()
    await driver.connect()

    async def dies_of_a_bug():
        await asyncio.sleep(0.01)
        raise ValueError("the control loop's own defect, not a cancellation")

    real = driver._control_task
    real.cancel()
    try:
        await real
    except asyncio.CancelledError:
        pass
    driver._control_task = asyncio.get_running_loop().create_task(
        dies_of_a_bug())
    await asyncio.sleep(0.03)     # let it crash before disconnect joins it

    with pytest.raises(ValueError, match="control loop's own defect"):
        await driver.disconnect()

    # the crash surfaced; the retry finishes the teardown cleanly
    await driver.disconnect()
