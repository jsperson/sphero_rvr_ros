"""A curve-faithful drivetrain simulator, for closing the loop with the chassis OFF.

    ROTATION IS CERTIFIED HERE. ARCS ARE NOT.

Read that twice before quoting any green result from this module. Rotation behaviour is
modelled from the 2026-08-16 measurement (`pivot_curve`, four runs on the operating
surface). **Arcs are modelled with IDEAL DIFFERENTIAL KINEMATICS because arc rates have
never been measured** -- see `docs/run_card_arc_rate_FUTURE.md`. A green run here proves
the in-place rotation defect is gone. **It is not a prediction that a field run will
succeed.**

WHERE THIS CUTS IN, and why it is one level deeper than it looks: this is a *transport*,
not a fake robot node. The REAL driver, the REAL `clamp_velocity_for_path`, the REAL
`plan_pivot`, and the REAL `rvr_node` odometry pipeline all run above it -- production
code from the serial bytes upward. The simulator's only job is the question the curve
actually answers: **given tank duties on the wire, what does this robot do?** Odom then
reaches the controller through `rvr_node`'s own polling at its own configured rate, so the
10 Hz-odom-versus-20 Hz-controller staleness that couples the limit cycle is reproduced
rather than simulated away.

THE WALK BAND IS A TRIPWIRE, NOT A MODEL. Duties 11..22 were measured bimodal -- clean
pivot or a one-tread arc that walks 16-22 cm. Production must never emit them
(`pivot_curve.plan_pivot` never returns a duty between 1 and `pivot_min_duty`). So this
module does not model that regime: it **raises**. Asserting an invariant in closed loop is
strictly stronger than modelling a state the system is forbidden to reach.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import List, Optional

from .commands import RVRCommands
from .fake_transport import FakeTransport
from .packet import FLAG_HAS_TARGET, FLAG_IS_ACTIVITY, FLAG_IS_RESPONSE, Packet
from .pivot_curve import (
    CURVE_VALID_DUTY_MAX,
    CURVE_VALID_DUTY_MIN,
    DEAD_ZONE_MAX_DUTY,
    WALK_BAND_MAX_DUTY,
    WALK_BAND_MIN_DUTY,
    rate_for_duty,
)

BANNER = (
    "CHASSIS SIM: rotation certified from the measured curve; ARCS ARE IDEAL "
    "KINEMATICS AND UNMEASURED (docs/run_card_arc_rate_FUTURE.md). A green run here is "
    "NOT a prediction that a field run succeeds."
)

#: Production odometry constants, so simulated encoders decode to the motion we injected.
COUNTS_PER_METER = 4337.768
WHEEL_TRACK_M = 0.2507

DID_DRIVE = 0x16
DID_SENSOR = 0x18
DID_POWER = 0x13
DID_SYSTEM_INFO = 0x11


class WalkBandViolation(AssertionError):
    """A duty landed in the measured bimodal band. Production must never emit one."""


@dataclass
class ChassisState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    left_counts: int = 0
    right_counts: int = 0
    #: Every (t, duty) pair the wire carried, for the invariant checks.
    duty_log: List[tuple] = field(default_factory=list)


#: First-order spin-up time constant for in-place rotation, FITTED FROM RECORDED DATA and
#: then frozen -- never tuned against a falsifier target, which would be the fabricated-
#: input failure mode in its most flattering costume.
#:
#: Fit source: breakaway run 1 (`03_validation/breakaway_2026-08-16/`), the ramp inside
#: each burst, solving v(t) = v_ss * (1 - exp(-t/tau)) per early sample:
#:     duty 12 -> median tau 0.267 s (n=7, range 0.162-0.312)
#:     duty 16 -> median tau 0.188 s (n=10, range 0.112-0.340)
#: Taken as a single constant at the midpoint.
#:
#: HONESTY LINE: tau is clearly DUTY-DEPENDENT -- it falls as duty rises -- and both fits
#: come from LOW duties, because those are the only bursts the breakaway sweep recorded
#: before its early stop. Using one constant across the whole band is an approximation, and
#: at production duties (28-45) the true tau is probably shorter than 0.22 s.
ROTATION_SPINUP_TAU_S = 0.22


class CurveFaithfulChassis:
    """Integrates robot motion from the commands actually written to the wire.

    Rotation is NOT applied instantly. The real drivetrain takes ~0.2 s to reach its
    commanded rate, and that lag matters more than it looks: RPP centres its acceleration
    window on the MEASURED speed from odom, so how the achieved rate arrives is part of
    the control loop, not a cosmetic detail. A model that snaps instantly to the curve
    value produces tidy odom and therefore tidy commands -- and cannot reproduce a
    dynamic instability, because it has no dynamics.
    """

    def __init__(self, state: Optional[ChassisState] = None):
        self.state = state or ChassisState()
        #: The rate the body is ACTUALLY turning at, which lags the commanded rate.
        self._achieved_yaw_rate = 0.0

    # -- the measured truth ------------------------------------------------------------

    def yaw_rate_for_duty(self, duty: int) -> float:
        """Body yaw rate for a signed tank duty, straight from the measurement.

        The dead zone is EXACTLY zero -- duties 2..10 produced 0.000 rad/s mean AND peak
        with 41 motor packets written per burst. Not a slow crawl: nothing.
        """
        magnitude = abs(duty)
        if magnitude == 0:
            return 0.0
        if magnitude <= DEAD_ZONE_MAX_DUTY:
            return 0.0
        if WALK_BAND_MIN_DUTY <= magnitude <= WALK_BAND_MAX_DUTY:
            raise WalkBandViolation(
                f"duty {duty} is inside the measured bimodal walk band "
                f"({WALK_BAND_MIN_DUTY}..{WALK_BAND_MAX_DUTY}). Production must never "
                "emit one: plan_pivot returns either 0 or a duty at/above pivot_min_duty. "
                "This is the invariant guard firing in closed loop, not a model gap."
            )
        if magnitude > CURVE_VALID_DUTY_MAX:
            raise WalkBandViolation(
                f"duty {duty} exceeds the measured band's top ({CURVE_VALID_DUTY_MAX}). "
                "The curve was never measured there and this simulator refuses to "
                "extrapolate -- that is the error class that started this whole episode."
            )
        return math.copysign(rate_for_duty(magnitude), duty)

    # -- integration -------------------------------------------------------------------

    def apply_tank_normalized(self, left: int, right: int, dt: float) -> None:
        """An in-place pivot: opposing treads. Rotation from the curve, no translation."""
        if left == 0 and right == 0:
            # A stop is a command to zero, not teleportation: the body coasts down with
            # the same time constant. Modelling stops as instant would remove the tail
            # that the field's odom plainly shows.
            rate = self._advance_rotation(0.0, dt)
            self.state.yaw += rate * dt
            self._advance_encoders(rate * dt * WHEEL_TRACK_M / 2.0, 0.0)
            return
        # Production only ever emits (-d, +d) here; a mismatch means someone changed the
        # command builder and the model's premise no longer holds.
        if left != -right:
            raise WalkBandViolation(
                f"tank_normalized({left}, {right}) is not an opposing-tread pivot; the "
                "curve only describes opposing treads"
            )
        target = self.yaw_rate_for_duty(right)
        rate = self._advance_rotation(target, dt)
        self.state.yaw += rate * dt
        self._advance_encoders(rate * dt * WHEEL_TRACK_M / 2.0, 0.0)
        self.state.duty_log.append((dt, right))

    def _advance_rotation(self, target: float, dt: float) -> float:
        """First-order approach to `target`, returning the rate applied over this step.

        The encoders therefore report the LAGGED rate, which is the channel RPP's
        acceleration clamp reads -- the coupling this whole rig exists to reproduce.
        """
        if dt <= 0.0:
            return self._achieved_yaw_rate
        alpha = 1.0 - math.exp(-dt / ROTATION_SPINUP_TAU_S)
        self._achieved_yaw_rate += (target - self._achieved_yaw_rate) * alpha
        return self._achieved_yaw_rate

    def apply_tank_si(self, left_mps: float, right_mps: float, dt: float) -> None:
        """An arc. IDEAL KINEMATICS -- unmeasured regime, see the module banner."""
        v = (left_mps + right_mps) / 2.0
        w = (right_mps - left_mps) / WHEEL_TRACK_M
        self.state.x += v * math.cos(self.state.yaw) * dt
        self.state.y += v * math.sin(self.state.yaw) * dt
        self.state.yaw += w * dt
        self._advance_encoders(w * dt * WHEEL_TRACK_M / 2.0, v * dt)

    def _advance_encoders(self, spin_m: float, travel_m: float) -> None:
        left = (-spin_m + travel_m) * COUNTS_PER_METER
        right = (spin_m + travel_m) * COUNTS_PER_METER
        self.state.left_counts += int(round(left))
        self.state.right_counts += int(round(right))

    @property
    def encoder_counts(self):
        return self.state.left_counts, self.state.right_counts


class SimTransport(FakeTransport):
    """FakeTransport that drives a CurveFaithfulChassis and answers encoder polls.

    Motion is integrated across the interval between consecutive motor writes, taken from
    an injected clock -- so the model advances on the driver's real cadence rather than on
    a rate this file invents.
    """

    def __init__(self, clock, chassis: Optional[CurveFaithfulChassis] = None):
        # auto_ack OFF: an ack without FLAG_IS_RESPONSE satisfies nothing, and motor
        # packets are fire-and-forget. Real replies come from _respond().
        super().__init__(auto_ack=False)
        self.clock = clock
        self.chassis = chassis or CurveFaithfulChassis()
        self._last_motor_t: Optional[float] = None
        self.battery_percentage = 78
        self.battery_voltage = 7.92

    async def write(self, data: bytes) -> None:
        packet = Packet.decode(data)
        now = self.clock()

        if packet.device_id == DID_DRIVE:
            dt = 0.0 if self._last_motor_t is None else max(0.0, now - self._last_motor_t)
            self._last_motor_t = now
            if packet.command_id == RVRCommands.CID_DRIVE_TANK_NORMALIZED and dt > 0:
                left, right = struct.unpack(">bb", packet.payload[:2])
                self.chassis.apply_tank_normalized(left, right, dt)
            elif packet.command_id == RVRCommands.CID_DRIVE_TANK_SI_UNITS and dt > 0:
                left, right = struct.unpack(">ff", packet.payload[:8])
                self.chassis.apply_tank_si(left, right, dt)
            elif packet.command_id == RVRCommands.CID_RAW_MOTORS:
                # Stop frames are all-zero; anything else means a path we did not expect.
                if packet.payload not in (b"\x00\x00\x00\x00",):
                    raise WalkBandViolation(
                        "raw-motor drive command reached the wire; the stock middle "
                        "should never take that path"
                    )

        await super().write(data)

        payload = self._response_payload(packet)
        if payload is not None:
            await self._respond(packet, payload)

    def _response_payload(self, packet: Packet) -> Optional[bytes]:
        """The payload a real RVR would answer this query with, or None for no answer.

        EVERY query `rvr_node` polls must be answered. Leaving one unanswered is not the
        harmless "silent chassis" I first assumed: each unanswered request blocks for its
        full 1 s timeout while holding the command pipeline, and three such polls dragged
        /odom from its configured 10 Hz down to 0.4 Hz -- destroying the very
        odom-staleness fidelity this rig exists to reproduce. A real RVR answers these, so
        the sim must too.
        """
        cid = packet.command_id
        if packet.device_id == DID_SENSOR and cid == RVRCommands.CID_GET_ENCODER_COUNTS:
            left, right = self.chassis.encoder_counts
            return struct.pack(">ii", left, right)
        if packet.device_id == DID_POWER:
            if cid == RVRCommands.CID_GET_BATTERY_PERCENTAGE:
                return bytes([self.battery_percentage])
            if cid == RVRCommands.CID_GET_BATTERY_VOLTAGE:
                return struct.pack(">f", self.battery_voltage)
            if cid == RVRCommands.CID_GET_BATTERY_VOLTAGE_STATE:
                return bytes([1])  # "ok"
        if packet.device_id == DID_SENSOR and cid == RVRCommands.CID_GET_AMBIENT_LIGHT:
            return struct.pack(">f", 120.0)
        if packet.device_id == DID_SENSOR and cid == RVRCommands.CID_GET_TEMPERATURE:
            # sensor 4 = left motor, sensor 5 = right motor; id byte then float32.
            return (
                bytes([4]) + struct.pack(">f", 30.0)
                + bytes([5]) + struct.pack(">f", 30.0)
            )
        if packet.device_id == DID_DRIVE and cid == RVRCommands.CID_GET_MOTOR_FAULT:
            return bytes([0])
        if packet.device_id == DID_SENSOR and cid == RVRCommands.CID_GET_THERMAL_PROTECTION:
            # left temp, left status, right temp, right status -- both nominal.
            return struct.pack(">f", 30.0) + bytes([0]) + struct.pack(">f", 30.0) + bytes([0])
        if packet.device_id == DID_SYSTEM_INFO:
            if cid == RVRCommands.CID_GET_MAIN_APP_VERSION:
                return struct.pack(">HHH", 1, 0, 0)
            if cid == RVRCommands.CID_GET_BOARD_REVISION:
                return bytes([1])
            if cid == RVRCommands.CID_GET_PROCESSOR_NAME:
                return b"SIMULATED\x00"
            if cid == RVRCommands.CID_GET_CORE_UPTIME:
                return struct.pack(">Q", 1000)
        return None

    async def _respond(self, request: Packet, payload: bytes) -> None:
        """Inject a properly flagged RESPONSE.

        FLAG_IS_RESPONSE is not decoration: the dispatcher matches pending requests only
        on packets carrying it, so a reply without it is silently ignored and the caller
        times out. That cost the first closed-loop bringup its odometry.
        """
        await self.inject_read(
            Packet(
                request.device_id,
                request.command_id,
                request.sequence_id,
                payload,
                target=None,
                source=None,
                flags=FLAG_IS_RESPONSE | FLAG_IS_ACTIVITY,
            ).encode()
        )
