"""The simulator must be faithful to the measurement, or it certifies nothing.

Every number here traces to `03_validation/breakaway_2026-08-16/`. If the sim and the
curve ever disagree, the sim is wrong -- it exists to reproduce a robot we measured, not
an idealised one we would prefer.
"""

import asyncio
import math
import struct

import pytest

from sphero_rvr_core import pivot_curve as pc
from sphero_rvr_core.chassis_sim import (
    BANNER,
    COUNTS_PER_METER,
    WHEEL_TRACK_M,
    ChassisState,
    CurveFaithfulChassis,
    SimTransport,
    WalkBandViolation,
)
from sphero_rvr_core.commands import RVRCommands
from sphero_rvr_core.packet import FLAG_IS_RESPONSE, Packet


# ======================================================================================
# Faithfulness to the measurement
# ======================================================================================


@pytest.mark.parametrize("duty, measured", [(23, 2.895), (28, 3.568), (32, 4.034), (45, 5.852)])
def test_the_sim_turns_at_the_rate_the_robot_turned(duty, measured):
    chassis = CurveFaithfulChassis()
    assert chassis.yaw_rate_for_duty(duty) == pytest.approx(measured, abs=0.06)


@pytest.mark.parametrize("duty", [1, 2, 5, 8, 10])
def test_the_dead_zone_is_EXACTLY_zero_not_merely_small(duty):
    # Duties 2..10 measured 0.000 mean AND peak, with 41 motor packets written per burst.
    # A sim that let the robot creep here would hide the defect it exists to expose.
    chassis = CurveFaithfulChassis()
    assert chassis.yaw_rate_for_duty(duty) == 0.0
    assert chassis.yaw_rate_for_duty(-duty) == 0.0


def test_zero_duty_is_zero():
    assert CurveFaithfulChassis().yaw_rate_for_duty(0) == 0.0


@pytest.mark.parametrize("duty", [11, 12, 16, 20, 22])
def test_the_walk_band_is_a_TRIPWIRE_not_a_model(duty):
    # Production never emits these: plan_pivot returns 0 or >= pivot_min_duty. Asserting
    # the invariant in closed loop is stronger than modelling a forbidden regime.
    chassis = CurveFaithfulChassis()
    with pytest.raises(WalkBandViolation) as excinfo:
        chassis.yaw_rate_for_duty(duty)
    assert "walk band" in str(excinfo.value)


@pytest.mark.parametrize("duty", [46, 60, 127])
def test_the_sim_refuses_to_extrapolate_above_the_measured_band(duty):
    with pytest.raises(WalkBandViolation) as excinfo:
        CurveFaithfulChassis().yaw_rate_for_duty(duty)
    assert "never measured" in str(excinfo.value)


def test_sign_follows_the_commanded_direction():
    chassis = CurveFaithfulChassis()
    assert chassis.yaw_rate_for_duty(28) > 0
    assert chassis.yaw_rate_for_duty(-28) < 0
    assert chassis.yaw_rate_for_duty(28) == -chassis.yaw_rate_for_duty(-28)


def test_the_banner_states_the_limit_of_what_this_proves():
    assert "ARCS ARE IDEAL" in BANNER
    assert "NOT a prediction" in BANNER
    assert "run_card_arc_rate_FUTURE" in BANNER


# ======================================================================================
# Integration
# ======================================================================================


def test_a_pivot_rotates_without_translating():
    chassis = CurveFaithfulChassis()
    chassis.apply_tank_normalized(-28, 28, 1.0)

    assert chassis.state.yaw == pytest.approx(pc.rate_for_duty(28), abs=1e-9)
    assert chassis.state.x == 0.0 and chassis.state.y == 0.0


def test_a_pivot_in_the_dead_zone_moves_nothing_at_all():
    chassis = CurveFaithfulChassis()
    chassis.apply_tank_normalized(-8, 8, 1.0)
    assert chassis.state.yaw == 0.0


def test_encoders_decode_back_to_the_yaw_that_was_injected():
    # The odom pipeline above this must be able to recover the motion, or the closed loop
    # is being fed nonsense.
    chassis = CurveFaithfulChassis()
    chassis.apply_tank_normalized(-28, 28, 0.5)
    left, right = chassis.encoder_counts

    recovered = ((right / COUNTS_PER_METER) - (left / COUNTS_PER_METER)) / WHEEL_TRACK_M
    assert recovered == pytest.approx(chassis.state.yaw, rel=1e-3)


def test_an_arc_translates_and_turns_by_ideal_kinematics():
    chassis = CurveFaithfulChassis()
    chassis.apply_tank_si(0.150, 0.250, 1.0)

    assert chassis.state.x == pytest.approx(0.200, abs=1e-6)
    assert chassis.state.yaw == pytest.approx((0.250 - 0.150) / WHEEL_TRACK_M, abs=1e-6)


def test_a_non_opposing_tank_normalized_command_is_refused():
    # The curve describes opposing treads only; anything else means the command builder
    # changed and the model's premise no longer holds.
    with pytest.raises(WalkBandViolation):
        CurveFaithfulChassis().apply_tank_normalized(10, 28, 0.1)


# ======================================================================================
# The transport
# ======================================================================================


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


@pytest.mark.asyncio
async def test_the_transport_integrates_between_motor_writes():
    clock = FakeClock()
    transport = SimTransport(clock)
    await transport.open()

    commands = RVRCommands()
    await transport.write(commands.drive_tank_normalized(1, -28, 28).encode())
    clock.t = 1.0
    await transport.write(commands.drive_tank_normalized(2, -28, 28).encode())

    assert transport.chassis.state.yaw == pytest.approx(pc.rate_for_duty(28), abs=1e-9)


@pytest.mark.asyncio
async def test_the_transport_answers_an_encoder_poll_with_the_simulated_counts():
    clock = FakeClock()
    transport = SimTransport(clock)
    await transport.open()
    transport.chassis.state.left_counts = -1234
    transport.chassis.state.right_counts = 5678

    commands = RVRCommands()
    await transport.write(commands.get_encoder_counts(7).encode())

    # WAIT_FOR, not a bare await: a mutation that stops answering encoder polls made this
    # test HANG rather than fail, and a hung mutation run looks like a passing one until
    # someone notices the clock. A test that can hang is a test that will hang in CI.
    raw = await asyncio.wait_for(transport.read_packet(), timeout=1.0)
    reply = Packet.decode(raw)

    assert struct.unpack(">ii", reply.payload) == (-1234, 5678)
    # FLAG_IS_RESPONSE is not decoration: the dispatcher matches pending requests ONLY on
    # packets carrying it. A reply without it is silently ignored and the caller times
    # out -- which is exactly what cost the first closed-loop bringup its odometry.
    assert reply.flags & FLAG_IS_RESPONSE, "a reply without FLAG_IS_RESPONSE satisfies nothing"


@pytest.mark.asyncio
async def test_battery_queries_are_answered_so_the_node_does_not_warn_every_second():
    clock = FakeClock()
    transport = SimTransport(clock)
    await transport.open()
    commands = RVRCommands()

    await transport.write(commands.get_battery_percentage(3).encode())
    reply = Packet.decode(await asyncio.wait_for(transport.read_packet(), timeout=1.0))
    assert reply.payload[0] == transport.battery_percentage
    assert reply.flags & FLAG_IS_RESPONSE


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "build",
    [
        lambda c, s: c.get_encoder_counts(s),
        lambda c, s: c.get_battery_percentage(s),
        lambda c, s: c.get_battery_voltage(s),
        lambda c, s: c.get_temperature(s),
        lambda c, s: c.get_ambient_light(s),
        lambda c, s: c.get_motor_fault_state(s),
        lambda c, s: c.get_thermal_protection_status(s),
    ],
)
async def test_every_query_the_node_polls_is_answered(build):
    """An unanswered poll is NOT a harmless silent chassis.

    Each one blocks for its full 1 s timeout while holding the command pipeline. Three of
    them dragged /odom from its configured 10 Hz to 0.4 Hz in the first closed-loop
    bringup -- destroying the odom-staleness fidelity this rig exists to reproduce. A real
    RVR answers these, so the sim must too, or the model is unfaithful on the exact axis
    the proof depends on.
    """
    clock = FakeClock()
    transport = SimTransport(clock)
    await transport.open()

    await transport.write(build(RVRCommands(), 9).encode())
    reply = Packet.decode(await asyncio.wait_for(transport.read_packet(), timeout=1.0))
    assert reply.flags & FLAG_IS_RESPONSE
    assert reply.payload, "an empty payload is not an answer"


@pytest.mark.asyncio
async def test_a_walk_band_duty_on_the_wire_fails_loudly():
    clock = FakeClock()
    transport = SimTransport(clock)
    await transport.open()
    commands = RVRCommands()

    await transport.write(commands.drive_tank_normalized(1, -16, 16).encode())
    clock.t = 0.1
    with pytest.raises(WalkBandViolation):
        await transport.write(commands.drive_tank_normalized(2, -16, 16).encode())


@pytest.mark.asyncio
async def test_a_stop_frame_is_allowed_but_a_raw_motor_drive_is_not():
    clock = FakeClock()
    transport = SimTransport(clock)
    await transport.open()
    commands = RVRCommands()

    await transport.write(commands.stop(1).encode())  # all-zero payload: fine

    clock.t = 0.1
    with pytest.raises(WalkBandViolation):
        await transport.write(
            commands.drive_rc(2, linear_mps=0.2, angular_rad_s=0.0).encode()
        )
