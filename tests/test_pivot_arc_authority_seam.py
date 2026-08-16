"""Two regimes, two measurements, two limits -- and neither may govern the other's path.

D45 in one sentence: a 0.4 rad/s clamp was enforced at the driver's door on a path whose
slowest producible rate is 3.55 rad/s. The clamp was real, the number was real, and it
governed a path the command was not on.

So this file asserts BOTH directions:

* no layer rewrites a pivot below what the drivetrain can execute (the D45-era test), and
* the pivot ceiling never leaks onto the ARC path, which run 4 did not measure and whose
  behaviour must not change at all.
"""

import asyncio
import math
import struct

import pytest

from sphero_rvr_core import pivot_curve as pc
from sphero_rvr_core.commands import RVRCommands
from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_core.packet import Packet
from sphero_rvr_core.safety import (
    VelocityCommand,
    clamp_velocity_for_path,
    is_pivot_command,
)

# The arc authority as the missions actually deploy it. Deliberately the old, small,
# UNMEASURED number: these tests exist to prove it is preserved exactly.
ARC_LIMIT = 0.4
PIVOT_MIN, PIVOT_MAX = 28, 45


# ======================================================================================
# The pure seam
# ======================================================================================


def _clamped(linear, angular, *, pivot_ceiling=None, arc_limit=ARC_LIMIT):
    ceiling = pc.maximum_clean_rate(PIVOT_MAX) if pivot_ceiling is None else pivot_ceiling
    return clamp_velocity_for_path(
        VelocityCommand(linear, angular),
        max_linear_mps=0.20,
        max_angular_rad_s=arc_limit,
        max_pivot_rate_rad_s=ceiling,
        is_pivot=is_pivot_command(linear, angular, pc.PIVOT_LINEAR_EPSILON_MPS),
    )


def test_a_pivot_is_not_rewritten_below_what_the_drivetrain_can_execute():
    # THE D45 TEST. 3.5 rad/s is producible (duty ~28). The 0.4 arc clamp must not touch
    # it: that clamp governs arcs, and this command is not an arc.
    assert _clamped(0.0, 3.5).angular_rad_s == pytest.approx(3.5)


@pytest.mark.parametrize("rate", [0.4, 0.9, 1.3, 2.9, 3.55, 5.8])
def test_no_producible_or_substitutable_pivot_rate_is_cut_down_by_the_arc_limit(rate):
    assert _clamped(0.0, rate).angular_rad_s == pytest.approx(rate)


def test_a_pivot_above_the_curves_ceiling_is_capped_by_the_PIVOT_authority():
    ceiling = pc.maximum_clean_rate(PIVOT_MAX)
    assert _clamped(0.0, 50.0).angular_rad_s == pytest.approx(ceiling)
    assert _clamped(0.0, -50.0).angular_rad_s == pytest.approx(-ceiling)


def test_the_pivot_ceiling_never_leaks_onto_the_arc_path():
    # An arc asking for 3.5 rad/s must still be cut to the arc limit. If the pivot ceiling
    # applied here it would command a tread differential far beyond max_linear_mps, on a
    # regime nobody has measured.
    assert _clamped(0.15, 3.5).angular_rad_s == pytest.approx(ARC_LIMIT)


def test_sub_minimum_arcs_behave_exactly_as_they_did_before():
    # No behaviour change on the unmeasured path: an arc under the arc limit passes
    # through untouched, exactly as clamp_velocity always did.
    assert _clamped(0.15, 0.2).angular_rad_s == pytest.approx(0.2)
    assert _clamped(0.15, -0.2).angular_rad_s == pytest.approx(-0.2)


def test_linear_is_clamped_the_same_way_on_both_paths():
    assert _clamped(9.0, 0.0).linear_mps == pytest.approx(0.20)
    assert _clamped(9.0, 3.5).linear_mps == pytest.approx(0.20)


def test_a_pure_stop_is_a_pivot_of_neither_kind_and_stays_zero():
    out = _clamped(0.0, 0.0)
    assert out.linear_mps == 0.0 and out.angular_rad_s == 0.0


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_requests_still_fail_closed_on_both_paths(bad):
    assert _clamped(0.0, bad).angular_rad_s == 0.0
    assert _clamped(bad, 3.5).linear_mps == 0.0


def test_the_pivot_epsilon_has_exactly_one_definition():
    # The control loop's branch condition and the clamp must agree about what "in place"
    # means. If they drift, the clamp governs a path the command is not on -- D45.
    #
    # The first version of this test was an `or` over two conditions, the second of which
    # was trivially true, so it passed while the control loop still carried its own
    # literal 0.005. A guard that cannot fail is not a guard; this one names the exact
    # line and forbids the literal.
    import inspect

    from sphero_rvr_core import driver as driver_module

    source = inspect.getsource(driver_module)

    assert "abs(velocity.linear_mps) < PIVOT_LINEAR_EPSILON_MPS" in source, (
        "the control loop must branch on the shared epsilon, not a literal of its own"
    )
    assert "< 0.005" not in source, (
        "a bare 0.005 in the driver is a second definition of 'in place'"
    )


@pytest.mark.parametrize(
    "linear, expected_pivot",
    [
        (0.0, True),
        (pc.PIVOT_LINEAR_EPSILON_MPS / 2, True),
        (pc.PIVOT_LINEAR_EPSILON_MPS, False),
        (pc.PIVOT_LINEAR_EPSILON_MPS * 2, False),
    ],
)
def test_the_clamp_and_the_control_loop_agree_on_the_epsilon_boundary(linear, expected_pivot):
    assert is_pivot_command(linear, 3.5, pc.PIVOT_LINEAR_EPSILON_MPS) is expected_pivot
    # And the clamp follows that classification: pivot side keeps the rate, arc side cuts.
    angular = _clamped(linear, 3.5).angular_rad_s
    assert (angular == pytest.approx(3.5)) is expected_pivot


# ======================================================================================
# The same seam, through the real driver
# ======================================================================================


def _tank_duties(writes):
    duties = []
    for raw in writes:
        packet = Packet.decode(raw)
        if packet.command_id == RVRCommands.CID_DRIVE_TANK_NORMALIZED:
            right = packet.payload[1]
            duties.append(right - 256 if right > 127 else right)
    return duties


def _tank_si_packets(writes):
    return [
        Packet.decode(raw)
        for raw in writes
        if Packet.decode(raw).command_id == RVRCommands.CID_DRIVE_TANK_SI_UNITS
    ]


async def _run(linear, angular, *, closed_loop_pivot=True, cycles=6):
    transport = FakeTransport(auto_ack=False)
    driver = RVRDriver(
        transport=transport,
        control_period=0.001,
        command_timeout=10.0,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
        max_linear_mps=0.20,
        max_angular_rad_s=ARC_LIMIT,
        pivot_min_duty=PIVOT_MIN,
        pivot_max_duty=PIVOT_MAX,
        closed_loop_pivot=closed_loop_pivot,
    )
    await driver.connect()
    transport.writes.clear()
    await driver.set_velocity(linear, angular)
    for _ in range(cycles):
        await asyncio.sleep(0.005)
    result = (list(transport.writes), driver._max_pivot_rate_rad_s)
    await driver.stop()
    await asyncio.wait_for(driver.disconnect(), timeout=1.0)
    return result


@pytest.mark.asyncio
async def test_the_driver_derives_its_pivot_ceiling_from_the_curve_not_the_arc_limit():
    _writes, ceiling = await _run(0.0, 3.5)
    assert ceiling == pytest.approx(pc.maximum_clean_rate(PIVOT_MAX))
    assert ceiling > ARC_LIMIT * 10


@pytest.mark.asyncio
async def test_a_fast_pivot_survives_the_arc_clamp_and_reaches_the_wheels():
    # End to end: 4.5 rad/s asked for, arc limit 0.4 in force, and the wheels must still
    # see the duty the curve says produces 4.5. Under the old single clamp this command
    # arrived at the pivot branch as 0.4.
    #
    # 4.5 and not 3.5 ON PURPOSE: at min_duty 28 the floor rate is 3.55, so a 3.5 request
    # is SUBSTITUTED to the floor duty -- which is the same duty a 0.4 request gets. The
    # first version of this test used 3.5 and therefore could not tell a working seam from
    # a broken one; a mutation that clamped every pivot to 0.4 survived it. The request
    # has to sit strictly inside the band for this assertion to mean anything.
    request = 4.5
    plan = pc.plan_pivot(request, min_duty=PIVOT_MIN, max_duty=PIVOT_MAX)
    assert plan.policy == "exact", "the probe rate must be inside the band, not substituted"
    assert abs(plan.duty) > PIVOT_MIN, "and strictly above the floor duty"

    writes, _ = await _run(0.0, request)
    duties = _tank_duties(writes)

    assert duties
    assert set(duties) == {plan.duty}


@pytest.mark.asyncio
async def test_an_arc_is_still_governed_by_the_unmeasured_arc_limit():
    # linear != 0, so this is an arc. It must be clamped to 0.4 rad/s and go out on the
    # tank-SI path -- never through the pivot branch.
    writes, _ = await _run(0.15, 3.5)

    assert not _tank_duties(writes), "an arc must not reach the in-place pivot branch"
    assert _tank_si_packets(writes), "an arc must go out on the tank-SI path"


def _tank_si_tread_speeds(packet):
    """(left, right) m/s from a drive_tank_si_units packet -- two big-endian float32."""
    return struct.unpack(">ff", packet.payload[:8])


@pytest.mark.asyncio
async def test_an_arc_reaching_the_mixer_carries_the_ARC_clamped_rate_not_the_request():
    # It is not enough that an arc goes out on the tank-SI path -- the SPEEDS on that path
    # must reflect the arc clamp. A seam that classified this as a pivot would hand the
    # mixer 3.5 rad/s and command a tread differential of 0.44 m/s against a 0.20 limit.
    writes, _ = await _run(0.15, 3.5)
    packets = _tank_si_packets(writes)
    assert packets

    left, right = _tank_si_tread_speeds(packets[-1])
    half_track = 0.2507 / 2.0
    assert (right - left) / 2.0 == pytest.approx(ARC_LIMIT * half_track, abs=1e-3)


@pytest.mark.asyncio
async def test_with_the_pivot_gate_off_a_pure_rotation_is_an_arc_and_keeps_arc_limits():
    # closed_loop_pivot=False means the pivot branch never runs, so the command falls
    # through to an arc path -- and the ARC authority is then the correct one. A clamp
    # that ignored the gate would hand 3.5 rad/s to the tank-SI mixer.
    writes, _ = await _run(0.0, 3.5, closed_loop_pivot=False)

    assert not _tank_duties(writes)
    packets = _tank_si_packets(writes)
    assert packets, "expected the tank-SI fall-through"

    # And the SPEEDS must be the arc-clamped ones. Classifying this as a pivot would let
    # 3.5 rad/s through to the mixer -- 0.44 m/s of tread differential against a 0.20
    # linear limit, on the unmeasured path.
    left, right = _tank_si_tread_speeds(packets[-1])
    half_track = 0.2507 / 2.0
    assert (right - left) / 2.0 == pytest.approx(ARC_LIMIT * half_track, abs=1e-3)
