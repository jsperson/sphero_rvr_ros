"""The driver's pivot branch now maps the REQUESTED rate through the measured curve.

The behaviour these tests replace was a closed loop that read only the command's sign,
targeted a fixed 1.3 rad/s, and ramped a duty that was pinned at its own floor forever --
because the floor delivers 3.57 rad/s and the target was 1.3, so the error never changed
sign. It had no tests. These are the ones it should have had, written against what
replaced it.
"""

import asyncio

import pytest

from sphero_rvr_core import pivot_curve as pc
from sphero_rvr_core.commands import RVRCommands
from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import Packet

PIVOT_MIN = 28
PIVOT_MAX = 45


def _tank_pivot_duties(transport):
    """Signed right-tread duty of every drive_tank_normalized packet written."""
    duties = []
    for raw in transport.writes:
        packet = Packet.decode(raw)
        if packet.command_id == RVRCommands.CID_DRIVE_TANK_NORMALIZED:
            right = packet.payload[1]
            duties.append(right - 256 if right > 127 else right)
    return duties


async def _drive_pivot(rate, *, cycles=6, min_duty=PIVOT_MIN, max_duty=PIVOT_MAX):
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.001,
        command_timeout=10.0,
        pivot_min_duty=min_duty,
        pivot_max_duty=max_duty,
        max_angular_rad_s=99.0,  # do not let the arc clamp mask what the pivot path does
    )
    await driver.connect()
    transport.writes.clear()
    await driver.set_velocity(0.0, rate)
    for _ in range(cycles):
        await asyncio.sleep(0.005)
    duties = _tank_pivot_duties(transport)
    await driver.stop()
    await asyncio.wait_for(driver.disconnect(), timeout=1.0)
    return duties


@pytest.mark.asyncio
async def test_a_producible_rate_commands_the_duty_that_produces_it():
    duties = await _drive_pivot(4.034)  # measured at duty 32

    assert duties, "the pivot branch wrote no tank packets"
    expected = pc.plan_pivot(4.034, min_duty=PIVOT_MIN, max_duty=PIVOT_MAX).duty
    assert set(duties) == {expected}
    assert abs(expected) == 32


@pytest.mark.asyncio
async def test_the_duty_does_not_ramp_over_time_the_way_the_retired_loop_did():
    # THE REGRESSION THAT MATTERS. The old integrator added gain*error every cycle, so a
    # constant request produced a CHANGING duty until it pinned at the clamp. A curve
    # mapping is memoryless: the same request must give the same duty on every cycle.
    duties = await _drive_pivot(3.568, cycles=12)

    assert len(set(duties)) == 1, (
        f"duty varied across cycles for a constant request: {sorted(set(duties))}. "
        "That is integrator behaviour, and the integrator is supposed to be gone."
    )


@pytest.mark.asyncio
async def test_an_impossible_rate_is_raised_to_the_floor_not_scaled_into_the_dead_zone():
    # 0.4 rad/s is the supervisor's old clamp. No duty produces it. The drivetrain must
    # get the floor duty -- never a proportional fraction of it, which would land in the
    # measured dead zone and turn nothing at all.
    duties = await _drive_pivot(0.4)

    assert set(duties) == {PIVOT_MIN}


@pytest.mark.asyncio
@pytest.mark.parametrize("rate", [0.05, 0.4, 0.9, 1.3, 2.5])
async def test_no_sub_floor_request_ever_reaches_the_wheels_as_a_walk_band_duty(rate):
    duties = await _drive_pivot(rate)

    assert duties
    for duty in duties:
        assert abs(duty) >= PIVOT_MIN, (
            f"request {rate} rad/s produced duty {duty}, inside the measured dead zone "
            "or bimodal walk band"
        )


@pytest.mark.asyncio
async def test_the_sign_of_the_request_still_selects_the_direction():
    left = await _drive_pivot(3.5)
    right = await _drive_pivot(-3.5)

    assert left and right
    assert all(d > 0 for d in left)
    assert all(d < 0 for d in right)
    assert {abs(d) for d in left} == {abs(d) for d in right}


@pytest.mark.asyncio
async def test_a_faster_request_commands_a_higher_duty():
    # The old loop could not do this at all: it discarded the magnitude, so 3.0 and 5.5
    # were the same command. This is the whole point of the change.
    slow = await _drive_pivot(3.0)
    fast = await _drive_pivot(5.5)

    assert max(slow) < max(fast)


@pytest.mark.asyncio
async def test_the_deployed_band_bounds_what_the_wheels_can_be_given():
    duties = await _drive_pivot(99.0, min_duty=23, max_duty=32)

    assert set(duties) == {32}, "a huge request must cap at the configured ceiling"
