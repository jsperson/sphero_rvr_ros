#!/usr/bin/env python3
"""Measure the lowest tank duty that actually rotates this robot. ATTENDED, ON THE PI.

SPEC: `docs/run_card_breakaway_2026-08-16.md`. MECHANISM: `docs/autopsy_phantom_freeze_2026-08-16.md`.

WHY A DUTY SWEEP AND NOT A RATE SWEEP
    For a pure in-place pivot (`|linear| < 0.005`, `|angular| > 0`) the driver's
    control loop takes a closed-loop pivot branch that uses the commanded angular
    rate ONLY for its sign, then ramps a duty toward a fixed internal target of
    1.3 rad/s (`driver.py`, the pivot branch above raw-motor). Commanded 0.4 and
    commanded 0.9 are the same command to that controller. Sweeping commanded rate
    would return one answer N times and "prove" a drivetrain that cannot turn.
    So this sweeps DUTY, on `drive_tank_normalized`, the command that path sends.

WHY ONE PROCESS
    `rvr_node` and the collision supervisor each cache `max_angular_rad_s` at
    __init__ with no parameter callback, so `ros2 param set` moves the parameter
    server while the live clamp keeps its cached value. Two authorities for one
    constant. This tool talks to the serial transport itself, with the ROS stack
    DOWN, and refuses to start if anything else holds the port.

WHY THE GYRO IS THE INSTRUMENT
    The production pivot loop regulates on WHEEL-encoder odometry. Encoders read
    wheels; this test asks whether the BODY turned. A grinding wheel that slips
    reports rotation that did not happen, and a stalled wheel reports zero while
    the chassis could still be creeping. The IMU gyro measures body rotation
    directly, so it is the primary instrument here; encoder counts are polled at
    the burst boundaries as a cross-check, and wheel-vs-body disagreement is
    itself a result (D32). The gyro path has NOT been exercised in production --
    `publish_imu` is false in every deployed config -- so this tool proves the
    instrument is alive before it commands a single duty, and says INSTRUMENT DEAD
    and refuses rather than fabricating a sweep from a stream that never arrived.

SAFETY: rotation in place only, attended, bounded bursts, refuses without --arm.
Read the preamble it prints. It is not a suggestion; the rover has been powered
down twice by sustained sub-moving-duty grinding.

Offline-testable: `tests/test_pivot_duty_sweep.py` drives every decision path
against a fake driver. Nothing in this module imports pyserial until --arm has
been accepted and the refusals have all passed.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.isdir(os.path.join(REPO_ROOT, "src")):
    sys.path.insert(0, os.path.join(REPO_ROOT, "src"))


# --------------------------------------------------------------------------------------
# THE SCALE CONVERSION -- one place, because two scales are what made the autopsy wrong.
# --------------------------------------------------------------------------------------
#
# The only moving-duty figure anywhere in this repo is a comment on the RAW-MOTOR
# branch, on a 0-255 magnitude scale: "angular duty <=128 does not move at all,
# 140-160 breaks away then bogs". In-place pivots do NOT use that branch; they use
# `drive_tank_normalized`, which clamps each tread to +/-127. Converting between the
# two is arithmetic; assuming the two firmware paths are EQUIVALENT IN TORQUE is an
# inference, and measuring it is the entire point of this tool. Do not let the
# conversion below be mistaken for the equivalence claim.

TANK_FULL_SCALE = 127  # drive_tank_normalized clamps each tread to +/-127
RAW_MOTOR_FULL_SCALE = 255  # raw-motor / drive_rc duty magnitude

DOCUMENTED_NO_MOVE_RAW255 = 128  # "<=128 does not move at all"
DOCUMENTED_BREAKAWAY_RAW255 = (140, 160)  # "140-160 breaks away then bogs"

# Every pivot ceiling this repo actually deploys, so the ladder is guaranteed to
# span all of them rather than whichever one a doc happened to quote.
#   rvr_node.RVRNodeConfig defaults      : min 23, max 32   (rvr.yaml sets neither)
#   config/lean_rvr_tank_si.yaml         : min 28, max 45   <- explore.launch.py default,
#                                                              i.e. what missions run
PRODUCTION_PIVOT_MIN_DUTY = 23
PRODUCTION_PIVOT_MAX_DUTY = 45

# CURVE MODE (--no-early-stop) exists because the ordinary ladder cannot map the
# production band: it stops one step past the knee, and 2026-08-16 measured that knee at
# tank 10-12 -- below EVERY deployed constant. So no ordinary run ever reaches 23/28/32/45,
# and the rate-vs-duty curve the config's MEASURE-FIRST markers need is unobtainable.
#
# Removing the early stop removes the thing that kept the rover out of the bog, so curve
# mode replaces it with a hard cap rather than leaving nothing there. The cap is the
# highest duty this repo actually deploys: there is no reason to grind above a duty no
# production path can command, and the documented bog begins not far above it. Raising
# this is a reviewed change, not a flag.
CURVE_MODE_MAX_DUTY = PRODUCTION_PIVOT_MAX_DUTY


def raw255_to_tank127(duty_raw255: float) -> int:
    """Convert a 0-255 raw-motor duty magnitude to the +/-127 tank scale."""
    return int(round(float(duty_raw255) * TANK_FULL_SCALE / RAW_MOTOR_FULL_SCALE))


def tank127_to_raw255(duty_tank127: float) -> int:
    """Convert a +/-127 tank duty magnitude to the 0-255 raw-motor scale."""
    return int(round(float(duty_tank127) * RAW_MOTOR_FULL_SCALE / TANK_FULL_SCALE))


# --------------------------------------------------------------------------------------
# Thresholds. Two are derived from the sensor at rest, not chosen for a room.
# --------------------------------------------------------------------------------------

# Absolute floor for "the body is rotating": 0.15 rad/s is ~8.6 deg/s, an unmistakable
# turn. The effective threshold is the larger of this and a multiple of the gyro's own
# measured at-rest noise, taken in the stationary baseline at the start of every run --
# so a noisier IMU raises its own bar instead of manufacturing motion.
MOVING_RAD_S_FLOOR = 0.15
MOVING_NOISE_MULTIPLE = 5.0

# Two adjacent duties that BOTH rotate must differ measurably, or something upstream
# is clamping and the sweep is invalid (run card section 3). Same construction.
RESPONSE_DELTA_RAD_S_FLOOR = 0.05
RESPONSE_DELTA_NOISE_MULTIPLE = 3.0

# A pivot that walks is an abort, per the run card's 5 cm rule; encoder-derived
# straight-line travel over a burst is the cheapest honest measure of it.
TRANSLATION_ABORT_M = 0.05

# Bounds on what an operator may ask for. Sustained sub-moving duty is the damaging
# case, so the burst is short and the cool-down is long.
MAX_BURST_S = 3.0
MIN_SETTLE_S = 2.0

STEADY_FRACTION = 0.5  # measure over the last half of the burst, after the spin-up

EXIT_OK = 0
EXIT_REFUSED = 2
EXIT_SWEEP_INVALID = 3
EXIT_ABORTED = 4


def default_ladder() -> List[int]:
    """Ascending tank duties: below production, through it, past the documented breakaway.

    Ends above `raw255_to_tank127(160)` so the sweep can climb clear of the region the
    raw-motor comment calls breakaway even if nothing has moved by then.
    """
    return [12, 16, 20, 23, 28, 32, 36, 40, 45, 50, 56, 62, 70, 76, 84, 92, 100]


class RefusalError(RuntimeError):
    """The tool declines to command a motor. Always printed with a reason."""


class AbortError(RuntimeError):
    """A sweep already under way must stop now; motors get stopped on the way out."""


SAFETY_PREAMBLE = """\
================================ PIVOT DUTY SWEEP ================================
THIS COMMANDS THE MOTORS. It deliberately drives duties in and above the range the
driver's own notes say grinds. Before --arm, all of the following must be true:

  [ ] Scott present, HAND ON THE POWER SWITCH.
  [ ] Rotation in place only -- this tool never commands linear motion.
  [ ] Open floor, more than 0.5 m clear all round (a tank drive rotating can walk).
  [ ] Battery >= 25% (duty behaviour is voltage-dependent; the level is recorded
      with the result, and the result is only valid near that level).
  [ ] ROS stack DOWN. `ros2 node list` empty. Nothing else holding the serial port.
  [ ] Lidar and explorer NOT running. Driver only.

ABORT (power switch) on: any translation over ~5 cm, grinding that does not resolve
into rotation within a burst, or any smell of hot motor.

Bursts are bounded and the tool stops the motors between every step. It stops one
step past the first duty that rotates, so it does not climb into the bog.
==================================================================================
"""


# --------------------------------------------------------------------------------------
# Refusals
# --------------------------------------------------------------------------------------


def scan_port_holders(
    port: str,
    proc_root: str = "/proc",
    self_pid: Optional[int] = None,
) -> Optional[List[Tuple[int, str]]]:
    """Return [(pid, fd_target)] of processes holding `port`, or None if unknowable.

    Reads /proc/<pid>/fd symlinks directly rather than shelling out to fuser/lsof,
    which are not installed everywhere and whose exit codes are easy to misread.
    None means "this host has no /proc" -- the caller must refuse rather than assume
    the port is free, because an unverified single-authority claim is the exact
    failure this measurement exists to avoid.
    """
    if not os.path.isdir(proc_root):
        return None
    self_pid = os.getpid() if self_pid is None else self_pid
    target = os.path.realpath(port)
    holders: List[Tuple[int, str]] = []
    for entry in sorted(os.listdir(proc_root)):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == self_pid:
            continue
        fd_dir = os.path.join(proc_root, entry, "fd")
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue  # gone, or not ours to read
        for fd in fds:
            try:
                link = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            if link == port or os.path.realpath(link) == target:
                holders.append((pid, link))
                break
    return holders


def process_name(pid: int, proc_root: str = "/proc") -> str:
    try:
        with open(os.path.join(proc_root, str(pid), "cmdline"), "rb") as handle:
            raw = handle.read()
    except OSError:
        return "?"
    parts = [piece.decode("utf-8", "replace") for piece in raw.split(b"\0") if piece]
    return " ".join(parts) if parts else "?"


@dataclass(frozen=True)
class SweepConfig:
    port: str = "/dev/ttyAMA0"
    baud_rate: int = 115200
    duties: Tuple[int, ...] = ()
    burst_s: float = 2.0
    settle_s: float = 3.0
    control_period: float = 0.05
    baseline_s: float = 2.0
    min_battery_pct: int = 25
    imu_interval_ms: int = 33
    imu_ready_timeout_s: float = 5.0
    imu_ready_samples: int = 10
    imu_stale_s: float = 0.75
    csv_path: str = ""
    # Curve mode: run EVERY rung instead of stopping one past the knee. Every abort
    # stays armed; only the early stop is removed. See CURVE_MODE_MAX_DUTY.
    no_early_stop: bool = False

    def validated(self) -> "SweepConfig":
        duties = tuple(self.duties) or tuple(default_ladder())
        if not duties:
            raise RefusalError("duty ladder is empty")
        if any(int(d) != d for d in duties):
            raise RefusalError("duties must be integers")
        if any(d < 1 or d > TANK_FULL_SCALE for d in duties):
            raise RefusalError(
                f"every duty must be within 1..{TANK_FULL_SCALE} "
                f"(drive_tank_normalized clamps there); got {list(duties)}"
            )
        if any(b <= a for a, b in zip(duties, duties[1:])):
            raise RefusalError(f"duties must ascend strictly; got {list(duties)}")
        if self.no_early_stop:
            # Curve mode maps the deployed band, so the knee-finding bounds do not apply:
            # the ladder is SUPPOSED to sit inside production, and forcing it up to the
            # documented breakaway region would march a rover that no longer stops at the
            # knee straight into the bog. One bound replaces both.
            if not self.duties:
                raise RefusalError(
                    "--no-early-stop requires an explicit --duties ladder. The default "
                    "ladder is a knee-finder that climbs to 100; running it with the "
                    "early stop removed would grind through every rung above breakaway."
                )
            if duties[-1] > CURVE_MODE_MAX_DUTY:
                raise RefusalError(
                    f"with --no-early-stop every duty must be <= {CURVE_MODE_MAX_DUTY} "
                    f"(the highest duty any deployed config commands); got {duties[-1]}. "
                    "Nothing removes the early stop AND climbs into the bog."
                )
        else:
            if duties[0] >= PRODUCTION_PIVOT_MIN_DUTY:
                raise RefusalError(
                    f"ladder must start BELOW the production pivot floor "
                    f"({PRODUCTION_PIVOT_MIN_DUTY}); got {duties[0]}"
                )
            breakaway_top = raw255_to_tank127(DOCUMENTED_BREAKAWAY_RAW255[1])
            if duties[-1] < breakaway_top:
                raise RefusalError(
                    f"ladder must reach at least {breakaway_top} "
                    f"(= raw-motor {DOCUMENTED_BREAKAWAY_RAW255[1]}/255, the top of the "
                    f"documented breakaway region); got {duties[-1]}"
                )
        if not 0.0 < self.burst_s <= MAX_BURST_S:
            raise RefusalError(f"burst-s must be in (0, {MAX_BURST_S}]; got {self.burst_s}")
        if self.settle_s < MIN_SETTLE_S:
            raise RefusalError(f"settle-s must be >= {MIN_SETTLE_S}; got {self.settle_s}")
        if self.control_period <= 0.0:
            raise RefusalError("control-period must be positive")
        return SweepConfig(**{**self.__dict__, "duties": duties})


# --------------------------------------------------------------------------------------
# The instrument
# --------------------------------------------------------------------------------------


@dataclass
class GyroSample:
    t: float
    yaw_rate: float
    valid: bool


class ImuMonitor:
    """Collects decoded IMU samples pushed by the driver's own streaming callback."""

    def __init__(self, now: Callable[[], float]):
        self._now = now
        self.samples: List[GyroSample] = []
        self.count = 0
        self.last_t: Optional[float] = None

    def on_sample(self, sample) -> None:
        yaw_rate = float(sample.angular_velocity[2])
        valid = bool(getattr(sample, "is_valid", True))
        self.last_t = self._now()
        self.count += 1
        self.samples.append(GyroSample(self.last_t, yaw_rate, valid))

    def latest(self) -> Optional[GyroSample]:
        return self.samples[-1] if self.samples else None

    def age(self) -> Optional[float]:
        return None if self.last_t is None else self._now() - self.last_t

    def between(self, start: float, end: float) -> List[GyroSample]:
        return [s for s in self.samples if start <= s.t <= end]


# --------------------------------------------------------------------------------------
# Results and the verdict
# --------------------------------------------------------------------------------------


@dataclass
class StepResult:
    index: int
    duty: int
    mean_abs_yaw_rate: float
    peak_abs_yaw_rate: float
    sample_count: int
    # The same gyro over the WHOLE burst, spin-up included. `mean_abs_yaw_rate` above is
    # the steady half and is the right figure for "did this duty turn the robot"; this one
    # is the only figure that may be compared against the encoders, which are read at the
    # burst boundaries and divided by the full dt. Comparing the steady half against a
    # whole-burst encoder rate manufactures a deficit the size of the spin-up -- that is
    # exactly what the 2026-08-16 run 1 reported as 0.212 rad/s of "slip" that was not there.
    full_burst_mean_abs_yaw_rate: Optional[float] = None
    encoder_yaw_rate: Optional[float] = None
    encoder_translation_m: Optional[float] = None
    motor_stall: bool = False
    motor_fault: bool = False
    # Motor packets the driver actually wrote to the transport during this burst. A
    # table of zeros means nothing turned OR nothing was ever sent; this is the field
    # that tells those apart, so the tool never convicts a drivetrain for a dead
    # command path.
    motor_writes: int = 0

    @property
    def duty_raw255_equiv(self) -> int:
        return tank127_to_raw255(self.duty)


@dataclass
class Verdict:
    status: str  # MOVING_DUTY_FOUND | SWEEP_INVALID | NO_ROTATION_IN_RANGE
    moving_duty: Optional[int]
    reason: str
    moving_threshold: float
    response_delta: float


def moving_threshold(noise_floor: float) -> float:
    return max(MOVING_RAD_S_FLOOR, MOVING_NOISE_MULTIPLE * noise_floor)


def response_delta(noise_floor: float) -> float:
    return max(RESPONSE_DELTA_RAD_S_FLOOR, RESPONSE_DELTA_NOISE_MULTIPLE * noise_floor)


def evaluate(steps: Sequence[StepResult], noise_floor: float) -> Verdict:
    """Turn the recorded steps into a verdict, including the validity check.

    The validity check is the run card's: the ONLY evidence a duty took effect is a
    measurably different achieved yaw rate. If every pair of adjacent rotating duties
    produced the same rate, something upstream is clamping and no number from this run
    may be trusted -- including the moving duty it appears to show.
    """
    threshold = moving_threshold(noise_floor)
    delta = response_delta(noise_floor)
    moving = [s for s in steps if s.mean_abs_yaw_rate >= threshold]

    if not moving:
        writes = sum(step.motor_writes for step in steps)
        if writes <= 0:
            tail = (
                "and NOT ONE motor packet reached the transport. This is a dead COMMAND "
                "PATH, not a dead drivetrain: the sweep says nothing about the hardware."
            )
        else:
            tail = (
                f"with {writes} motor packets written to the transport, so the commands "
                "did reach the wire. Either the drivetrain needs more than the tank "
                "scale can deliver, or something below the driver is refusing them."
            )
        return Verdict(
            status="NO_ROTATION_IN_RANGE",
            moving_duty=None,
            reason=(
                f"no duty up to {steps[-1].duty if steps else 0} produced sustained "
                f"rotation (>= {threshold:.3f} rad/s), " + tail
            ),
            moving_threshold=threshold,
            response_delta=delta,
        )

    candidate = min(s.duty for s in moving)
    pairs = [
        (a, b)
        for a, b in zip(steps, steps[1:])
        if a.mean_abs_yaw_rate >= threshold and b.mean_abs_yaw_rate >= threshold
    ]
    indistinguishable = [
        (a, b) for a, b in pairs if abs(b.mean_abs_yaw_rate - a.mean_abs_yaw_rate) < delta
    ]
    if pairs and len(indistinguishable) == len(pairs):
        return Verdict(
            status="SWEEP_INVALID",
            moving_duty=None,
            reason=(
                "every pair of adjacent rotating duties produced the same achieved rate "
                f"(within {delta:.3f} rad/s), so no duty change is shown to have taken "
                f"effect. Something upstream is clamping. The apparent moving duty "
                f"({candidate}) is NOT a measurement -- find the clamp first."
            ),
            moving_threshold=threshold,
            response_delta=delta,
        )
    if not pairs:
        return Verdict(
            status="SWEEP_INVALID",
            moving_duty=None,
            reason=(
                f"duty {candidate} rotated but no second rotating step was recorded, so "
                "the reading has no confirmation and the validity check could not run. "
                "Re-run with at least one step above it."
            ),
            moving_threshold=threshold,
            response_delta=delta,
        )
    return Verdict(
        status="MOVING_DUTY_FOUND",
        moving_duty=candidate,
        reason=(
            f"lowest duty with sustained rotation = {candidate} "
            f"(= {tank127_to_raw255(candidate)}/255 equivalent). "
            f"Adjacent rotating duties differ measurably, so the sweep is valid."
        ),
        moving_threshold=threshold,
        response_delta=delta,
    )


def verdict_for_production(verdict: Verdict) -> str:
    """What the number means for the deployed pivot ceilings, stated plainly."""
    if verdict.moving_duty is None:
        return "No moving duty measured; the ceilings cannot be re-derived from this run."
    lines = []
    for name, ceiling in (
        ("rvr_node defaults (pivot_max_duty 32)", 32),
        ("config/lean_rvr_tank_si.yaml, what explore.launch.py runs (pivot_max_duty 45)", 45),
    ):
        if verdict.moving_duty > ceiling:
            lines.append(
                f"  {name}: CEILING IS BELOW THE FLOOR -- {ceiling} < {verdict.moving_duty}. "
                "Every in-place pivot on this config was a no-op."
            )
        else:
            lines.append(
                f"  {name}: ceiling {ceiling} >= moving duty {verdict.moving_duty}; "
                "this ceiling could turn the robot, so the freeze episodes had another cause."
            )
    return "\n".join(lines)


# --------------------------------------------------------------------------------------
# The sweep
# --------------------------------------------------------------------------------------


@dataclass
class SweepReport:
    steps: List[StepResult] = field(default_factory=list)
    verdict: Optional[Verdict] = None
    noise_floor: float = 0.0
    battery_pct: Optional[int] = None
    rows: List[dict] = field(default_factory=list)
    aborted: Optional[str] = None


class SweepRunner:
    """Drives the ladder against a driver. Clock and sleep are injected for tests."""

    def __init__(
        self,
        driver,
        config: SweepConfig,
        *,
        now: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], "asyncio.Future"] = asyncio.sleep,
        out: Callable[[str], None] = print,
    ):
        self.driver = driver
        self.config = config
        self.now = now
        self.sleep = sleep
        self.out = out
        self.imu = ImuMonitor(now)
        self.report = SweepReport()
        self.abort_requested: Optional[str] = None
        self._t0 = now()

    def request_abort(self, reason: str) -> None:
        """Ask the sweep to wind itself down at the next loop check.

        Wired to SIGINT/SIGTERM. A bare KeyboardInterrupt would unwind through the
        awaits and leave the FIRMWARE HOLDING THE LAST DUTY -- the tank command latches,
        and this tool is the only thing sending it. So the signal sets a flag, the loop
        raises AbortError at its next check, and the ordinary abort path stops the
        motors and still writes the CSV for the steps already measured.
        """
        self.abort_requested = reason

    # -- plumbing -----------------------------------------------------------------

    def _rel(self) -> float:
        return self.now() - self._t0

    def _row(self, step_index: int, phase: str, duty: int, encoders=None) -> None:
        sample = self.imu.latest()
        self.report.rows.append(
            {
                "t_s": round(self._rel(), 4),
                "step_index": step_index,
                "phase": phase,
                "duty_tank127": duty,
                "duty_raw255_equiv": tank127_to_raw255(duty),
                "gyro_z_rad_s": "" if sample is None else round(sample.yaw_rate, 5),
                "imu_age_s": "" if self.imu.age() is None else round(self.imu.age(), 4),
                "encoder_left": "" if encoders is None else encoders[0],
                "encoder_right": "" if encoders is None else encoders[1],
            }
        )

    def _check_abort(self) -> None:
        if self.abort_requested:
            raise AbortError(self.abort_requested)

    def _check_instrument(self) -> None:
        age = self.imu.age()
        if age is None or age > self.config.imu_stale_s:
            raise AbortError(
                "INSTRUMENT DEAD mid-sweep: no IMU sample for "
                f"{'ever' if age is None else format(age, '.2f') + ' s'} "
                f"(limit {self.config.imu_stale_s} s). Refusing to record a rate from a "
                "stream that stopped."
            )

    async def _read_encoders(self) -> Optional[Tuple[int, int]]:
        try:
            counts = await self.driver.get_encoder_counts()
        except Exception:
            return None
        return (int(counts.left), int(counts.right))

    async def stop_motors(self, repeats: int = 3) -> None:
        for _ in range(repeats):
            try:
                await self.driver.drive_tank_normalized(0, 0)
            except Exception:
                pass

    # -- phases -------------------------------------------------------------------

    async def preflight(self) -> None:
        battery = None
        try:
            battery = int(await self.driver.get_battery_percentage())
        except Exception as exc:  # a refusal we cannot evaluate is still a refusal
            raise RefusalError(f"could not read battery percentage: {exc!r}") from exc
        self.report.battery_pct = battery
        if battery < self.config.min_battery_pct:
            raise RefusalError(
                f"battery {battery}% is below the {self.config.min_battery_pct}% floor; "
                "duty behaviour is voltage-dependent and a low-battery number is not the "
                "number we want."
            )
        self.out(f"battery: {battery}%")

        self.driver.set_imu_callback(self.imu.on_sample)
        await self.driver.enable_imu_streaming(self.config.imu_interval_ms)

        deadline = self.now() + self.config.imu_ready_timeout_s
        while self.imu.count < self.config.imu_ready_samples and self.now() < deadline:
            await self.sleep(self.config.control_period)
        if self.imu.count < self.config.imu_ready_samples:
            raise RefusalError(
                "INSTRUMENT DEAD: only "
                f"{self.imu.count} IMU samples in {self.config.imu_ready_timeout_s} s "
                f"(needed {self.config.imu_ready_samples}). The gyro is the primary "
                "instrument for this measurement and publish_imu is false in every "
                "deployed config, so a dead stream here is expected to be a real "
                "finding. NOT sweeping: a duty ladder with no instrument would produce "
                "a table of zeros indistinguishable from a drivetrain that never moved."
            )
        invalid = [s for s in self.imu.samples if not s.valid]
        if invalid:
            raise RefusalError(
                f"{len(invalid)} of {len(self.imu.samples)} baseline IMU samples report "
                "is_valid=False; the stream is arriving but the firmware says it is not "
                "trustworthy."
            )
        self.out(f"IMU stream alive: {self.imu.count} samples")

    async def baseline(self) -> float:
        """Measure the gyro's own noise while stationary. Nothing is commanded here."""
        start = self.now()
        end = start + self.config.baseline_s
        while self.now() < end:
            await self.sleep(self.config.control_period)
            self._row(-1, "baseline", 0)
        samples = self.imu.between(start, self.now())
        if not samples:
            raise AbortError("no IMU samples during the stationary baseline")
        noise = sum(abs(s.yaw_rate) for s in samples) / len(samples)
        self.report.noise_floor = noise
        self.out(
            f"gyro noise at rest: {noise:.4f} rad/s "
            f"-> rotation threshold {moving_threshold(noise):.3f} rad/s, "
            f"response delta {response_delta(noise):.3f} rad/s"
        )
        return noise

    async def run_step(self, index: int, duty: int) -> StepResult:
        encoders_before = await self._read_encoders()
        writes_before = self._driver_state().get("motor_transport_write_count", 0)
        burst_start = self.now()
        end = burst_start + self.config.burst_s
        while self.now() < end:
            await self.driver.drive_tank_normalized(-duty, duty)
            await self.sleep(self.config.control_period)
            self._check_abort()
            self._check_instrument()
            self._row(index, "burst", duty)
        burst_end = self.now()
        await self.stop_motors()
        encoders_after = await self._read_encoders()

        steady_from = burst_start + self.config.burst_s * STEADY_FRACTION
        steady = self.imu.between(steady_from, burst_end)
        rates = [abs(s.yaw_rate) for s in steady]
        # Whole-burst gyro, on the SAME window the encoders are differenced over, so the
        # wheel-vs-body comparison is between two measurements of the same interval.
        whole = [abs(s.yaw_rate) for s in self.imu.between(burst_start, burst_end)]
        state = self._driver_state()

        encoder_yaw_rate = None
        encoder_translation = None
        if encoders_before is not None and encoders_after is not None:
            dt = max(1e-6, burst_end - burst_start)
            encoder_yaw_rate, encoder_translation = encoder_pivot_estimate(
                encoders_before, encoders_after, dt
            )

        result = StepResult(
            index=index,
            duty=duty,
            mean_abs_yaw_rate=(sum(rates) / len(rates)) if rates else 0.0,
            peak_abs_yaw_rate=max(rates) if rates else 0.0,
            sample_count=len(rates),
            full_burst_mean_abs_yaw_rate=(sum(whole) / len(whole)) if whole else None,
            encoder_yaw_rate=encoder_yaw_rate,
            encoder_translation_m=encoder_translation,
            motor_stall=bool(state.get("motor_stall_triggered", False)),
            motor_fault=bool(state.get("motor_fault", False)),
            motor_writes=int(state.get("motor_transport_write_count", 0)) - int(writes_before),
        )
        # Recorded BEFORE the cool-down and before the abort checks: the burst is
        # measured, and an abort raised after it is a statement about safety, not a
        # reason to throw away a real reading. The CSV keeps the step that walked.
        self.report.steps.append(result)

        settle_end = self.now() + self.config.settle_s
        while self.now() < settle_end:
            await self.sleep(self.config.control_period)
            self._row(index, "settle", 0, encoders=encoders_after)
            self._check_abort()

        if result.motor_fault:
            raise AbortError(f"firmware reported a MOTOR FAULT at duty {duty}")
        if (
            encoder_translation is not None
            and abs(encoder_translation) > TRANSLATION_ABORT_M
        ):
            raise AbortError(
                f"duty {duty} translated {encoder_translation * 100:.1f} cm "
                f"(limit {TRANSLATION_ABORT_M * 100:.0f} cm). A pivot that walks is an "
                "abort, not a data point."
            )
        return result

    def _driver_state(self) -> dict:
        try:
            state = self.driver.get_state()
        except Exception:
            return {}
        return {
            "motor_stall_triggered": getattr(state, "motor_stall_triggered", False),
            "motor_fault": getattr(state, "motor_fault", False),
            "motor_transport_write_count": getattr(state, "motor_transport_write_count", 0) or 0,
        }

    async def run(self) -> SweepReport:
        await self.preflight()
        noise = await self.baseline()
        threshold = moving_threshold(noise)
        steps_after_knee = 0
        try:
            for index, duty in enumerate(self.config.duties):
                self.out(f"step {index}: duty {duty} (= {tank127_to_raw255(duty)}/255) ...")
                result = await self.run_step(index, duty)
                self.out(
                    f"  achieved {result.mean_abs_yaw_rate:.3f} rad/s mean "
                    f"({result.peak_abs_yaw_rate:.3f} peak, n={result.sample_count})"
                    + (
                        ""
                        if result.encoder_yaw_rate is None
                        else f"; encoders {result.encoder_yaw_rate:.3f} rad/s"
                    )
                    + (" [STALL]" if result.motor_stall else "")
                )
                if result.mean_abs_yaw_rate >= threshold:
                    steps_after_knee += 1
                    # The knee plus one confirming step, then stop -- climbing further
                    # buys nothing and the bog is where the motors get hurt. Curve mode
                    # keeps climbing on purpose, bounded by CURVE_MODE_MAX_DUTY instead;
                    # every abort below is still armed and still ends the run.
                    if steps_after_knee >= 2 and not self.config.no_early_stop:
                        self.out("rotation confirmed one step past the knee; stopping the ladder")
                        break
        except AbortError as exc:
            self.report.aborted = str(exc)
        finally:
            await self.stop_motors()

        self.report.verdict = evaluate(self.report.steps, noise)
        return self.report


def encoder_pivot_estimate(
    before: Tuple[int, int],
    after: Tuple[int, int],
    dt: float,
    counts_per_meter: float = 4337.768,
    wheel_track_m: float = 0.2507,
) -> Tuple[float, float]:
    """(yaw rate rad/s, straight-line travel m) from two encoder reads.

    Same kinematics and the same wrap-safe delta the production odometry uses, so a
    wheel-vs-body disagreement here is a disagreement with what the pivot loop
    regulates on -- not with a second opinion invented for this tool.
    """
    from sphero_rvr_driver.odometry import encoder_delta

    left_m = encoder_delta(after[0], before[0]) / counts_per_meter
    right_m = encoder_delta(after[1], before[1]) / counts_per_meter
    yaw = (right_m - left_m) / wheel_track_m
    return (yaw / dt, (left_m + right_m) / 2.0)


# --------------------------------------------------------------------------------------
# Artifact
# --------------------------------------------------------------------------------------

CSV_COLUMNS = [
    "t_s",
    "step_index",
    "phase",
    "duty_tank127",
    "duty_raw255_equiv",
    "gyro_z_rad_s",
    "imu_age_s",
    "encoder_left",
    "encoder_right",
]


def git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", REPO_ROOT, "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL,
            timeout=5,
        ).decode().strip()
    except Exception:
        return "unknown"


def render_csv(config: SweepConfig, report: SweepReport, stamp: str) -> str:
    """CSV with the run's whole configuration in the header. A duty number without the
    battery level, the ladder and the binary that produced it is not a measurement."""
    head = [
        f"# pivot_duty_sweep {stamp}",
        f"# git_sha={git_sha()}",
        f"# port={config.port} baud={config.baud_rate}",
        f"# duties={','.join(str(d) for d in config.duties)}",
        f"# burst_s={config.burst_s} settle_s={config.settle_s} "
        f"control_period={config.control_period} baseline_s={config.baseline_s}",
        f"# imu_interval_ms={config.imu_interval_ms} imu_stale_s={config.imu_stale_s}",
        f"# battery_pct={report.battery_pct}",
        f"# gyro_noise_rad_s={report.noise_floor:.5f}",
        f"# scale: tank +/-{TANK_FULL_SCALE} <-> raw-motor 0-{RAW_MOTOR_FULL_SCALE}; "
        f"documented raw no-move {DOCUMENTED_NO_MOVE_RAW255} "
        f"(= tank {raw255_to_tank127(DOCUMENTED_NO_MOVE_RAW255)}), "
        f"breakaway {DOCUMENTED_BREAKAWAY_RAW255[0]}-{DOCUMENTED_BREAKAWAY_RAW255[1]} "
        f"(= tank {raw255_to_tank127(DOCUMENTED_BREAKAWAY_RAW255[0])}-"
        f"{raw255_to_tank127(DOCUMENTED_BREAKAWAY_RAW255[1])})",
        f"# production pivot band tank {PRODUCTION_PIVOT_MIN_DUTY}-{PRODUCTION_PIVOT_MAX_DUTY}",
        f"# no_early_stop={int(config.no_early_stop)}"
        + (f" (curve mode, every rung run, cap {CURVE_MODE_MAX_DUTY})"
           if config.no_early_stop else " (knee-finder, stops one past the knee)"),
    ]
    if report.aborted:
        head.append(f"# ABORTED: {report.aborted}")
    lines = head + [",".join(CSV_COLUMNS)]
    for row in report.rows:
        lines.append(",".join(str(row[column]) for column in CSV_COLUMNS))
    lines.append("# step summary: duty,duty_raw255_equiv,mean_abs_yaw_rate,peak,"
                 "samples,full_burst_mean_abs_yaw_rate,encoder_yaw_rate,"
                 "encoder_translation_m,motor_stall,motor_writes")
    for step in report.steps:
        lines.append(
            "# %d,%d,%.4f,%.4f,%d,%s,%s,%s,%s,%d"
            % (
                step.duty,
                step.duty_raw255_equiv,
                step.mean_abs_yaw_rate,
                step.peak_abs_yaw_rate,
                step.sample_count,
                ""
                if step.full_burst_mean_abs_yaw_rate is None
                else f"{step.full_burst_mean_abs_yaw_rate:.4f}",
                "" if step.encoder_yaw_rate is None else f"{step.encoder_yaw_rate:.4f}",
                ""
                if step.encoder_translation_m is None
                else f"{step.encoder_translation_m:.4f}",
                int(step.motor_stall),
                step.motor_writes,
            )
        )
    if report.verdict is not None:
        lines.append(f"# verdict={report.verdict.status} moving_duty={report.verdict.moving_duty}")
        lines.append(f"# reason={report.verdict.reason}")
    return "\n".join(lines) + "\n"


def render_table(report: SweepReport) -> str:
    lines = [
        "",
        "duty  raw255  achieved rad/s (gyro)  peak    n   pkts  encoders rad/s  translation",
    ]
    for step in report.steps:
        lines.append(
            "%4d  %6d  %21.3f  %6.3f  %3d  %4d  %14s  %s"
            % (
                step.duty,
                step.duty_raw255_equiv,
                step.mean_abs_yaw_rate,
                step.peak_abs_yaw_rate,
                step.sample_count,
                step.motor_writes,
                "n/a" if step.encoder_yaw_rate is None else f"{step.encoder_yaw_rate:.3f}",
                "n/a"
                if step.encoder_translation_m is None
                else f"{step.encoder_translation_m * 100:.1f} cm",
            )
        )
    verdict = report.verdict
    if verdict is not None:
        lines += ["", f"VERDICT: {verdict.status}", f"  {verdict.reason}"]
        disagreement = wheel_body_disagreement(report.steps)
        if disagreement:
            lines.append("  wheel-vs-body: " + disagreement)
        lines.append(verdict_for_production(verdict))
    if report.aborted:
        lines += ["", f"ABORTED: {report.aborted}"]
    lines.append(f"BATTERY = {report.battery_pct}%")
    return "\n".join(lines)


def wheel_body_disagreement(steps: Sequence[StepResult], tolerance: float = 0.10) -> str:
    """D32's never-run measurement: does the wheel odometry the pivot loop regulates on
    agree with what the body actually did?

    Both sides must describe the SAME interval. The encoders are differenced across the
    whole burst, so the gyro figure here is the whole-burst mean -- never the steady-half
    mean the verdict uses. Steps recorded before that field existed are skipped rather
    than compared on the wrong window: silence beats a manufactured disagreement.
    """
    pairs = [
        (s.duty, s.encoder_yaw_rate, s.full_burst_mean_abs_yaw_rate)
        for s in steps
        if s.encoder_yaw_rate is not None and s.full_burst_mean_abs_yaw_rate is not None
    ]
    if not pairs:
        return ""
    worst = max(pairs, key=lambda p: abs(abs(p[1]) - p[2]))
    gap = abs(abs(worst[1]) - worst[2])
    if gap <= tolerance:
        return (
            f"agree within {gap:.3f} rad/s across {len(pairs)} steps "
            "(whole-burst window, both sides)"
        )
    return (
        f"DISAGREE by {gap:.3f} rad/s at duty {worst[0]} "
        f"(wheels {abs(worst[1]):.3f}, body {worst[2]:.3f}, whole-burst window). "
        "The pivot loop regulates on the wheels, so the wheels are what it believes. "
        "Candidates: tread slip, or an odometry scale error in wheel_track_m / "
        "counts_per_meter -- a gap that holds its RATIO across duties is a scale error, "
        "one that varies with duty is slip. This tool does not distinguish them."
    )


# --------------------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------------------


def parse_duties(text: str) -> Tuple[int, ...]:
    return tuple(int(piece) for piece in text.replace(" ", "").split(",") if piece)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Attended pivot duty sweep (Pi, ROS down).")
    parser.add_argument("--arm", action="store_true", help="required; without it nothing is commanded")
    parser.add_argument("--port", default=SweepConfig.port)
    parser.add_argument("--baud", type=int, default=SweepConfig.baud_rate)
    parser.add_argument("--duties", type=parse_duties, default=())
    parser.add_argument("--burst-s", type=float, default=SweepConfig.burst_s)
    parser.add_argument("--settle-s", type=float, default=SweepConfig.settle_s)
    parser.add_argument("--min-battery", type=int, default=SweepConfig.min_battery_pct)
    parser.add_argument("--imu-interval-ms", type=int, default=SweepConfig.imu_interval_ms)
    parser.add_argument("--csv", default="")
    parser.add_argument(
        "--no-early-stop",
        action="store_true",
        help=(
            "curve mode: run EVERY rung instead of stopping one past the knee, to map "
            f"rate-vs-duty across the production band. Requires --duties, caps every "
            f"rung at {CURVE_MODE_MAX_DUTY}, and leaves every abort armed."
        ),
    )
    return parser


def config_from_args(args) -> SweepConfig:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return SweepConfig(
        port=args.port,
        baud_rate=args.baud,
        duties=tuple(args.duties),
        burst_s=args.burst_s,
        settle_s=args.settle_s,
        min_battery_pct=args.min_battery,
        imu_interval_ms=args.imu_interval_ms,
        csv_path=args.csv or os.path.expanduser(f"~/breakaway_{stamp}.csv"),
        no_early_stop=args.no_early_stop,
    ).validated()


def install_abort_signals(runner: SweepRunner) -> List["signal.Signals"]:
    """Route Ctrl-C and SIGTERM into the sweep's own wind-down, not into an unwind.

    Returns the signals actually installed so they can be removed afterwards; if the
    handler stayed installed through teardown, a second Ctrl-C would only set a flag
    nobody reads and the process would feel unkillable.
    """
    loop = asyncio.get_running_loop()
    installed: List["signal.Signals"] = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, runner.request_abort, f"operator sent {sig.name}")
        except (NotImplementedError, RuntimeError, ValueError, AttributeError):
            continue
        installed.append(sig)
    return installed


def remove_abort_signals(installed: Sequence["signal.Signals"]) -> None:
    loop = asyncio.get_running_loop()
    for sig in installed:
        try:
            loop.remove_signal_handler(sig)
        except (NotImplementedError, RuntimeError, ValueError, AttributeError):
            pass


async def sweep_with_hardware(config: SweepConfig, out: Callable[[str], None] = print) -> int:
    from sphero_rvr_core.driver import RVRDriver
    from sphero_rvr_core.serial_transport import SerialTransport

    transport = SerialTransport(port=config.port, baud_rate=config.baud_rate)
    driver = RVRDriver(
        transport=transport,
        control_period=config.control_period,
        # The control loop is never given a desired velocity, so it never sends
        # anything: every packet in this run comes from the sweep itself. The pivot
        # controller is disabled as well, so there is exactly one authority for duty
        # even if some future edit hands the loop a velocity by accident.
        closed_loop_pivot=False,
    )
    try:
        await transport.open()
        await driver.connect()
    except Exception as exc:
        out(f"REFUSED: could not open {config.port}: {exc!r}")
        return EXIT_REFUSED

    runner = SweepRunner(driver, config, out=out)
    installed = install_abort_signals(runner)
    outcome: Optional[int] = None
    try:
        report = await runner.run()
    except RefusalError as exc:
        out(f"REFUSED: {exc}")
        await runner.stop_motors()
        outcome = EXIT_REFUSED
    except AbortError as exc:
        out(f"ABORTED: {exc}")
        await runner.stop_motors()
        outcome = EXIT_ABORTED
    finally:
        remove_abort_signals(installed)
        try:
            await asyncio.wait_for(driver.disable_imu_streaming(), timeout=2.0)
        except Exception:
            pass
        try:
            # disconnect() has a known intermittent hang (diagnostics/disconnect_hang_probe.py).
            # The motors are already stopped by SweepRunner; do not let teardown hold the
            # process hostage after the measurement is safely over.
            await asyncio.wait_for(driver.disconnect(), timeout=5.0)
        except Exception:
            out("WARNING: driver.disconnect() did not complete; motors were stopped first")
        try:
            await asyncio.wait_for(transport.close(), timeout=2.0)
        except Exception:
            pass

    if outcome is not None:
        return outcome

    out(render_table(report))
    with open(config.csv_path, "w", encoding="utf-8") as handle:
        handle.write(render_csv(config, report, time.strftime("%Y-%m-%d %H:%M:%S")))
    out(f"\nCSV -> {config.csv_path}")

    if report.aborted:
        return EXIT_ABORTED
    if report.verdict is not None and report.verdict.status == "SWEEP_INVALID":
        out("\nSWEEP INVALID")
        return EXIT_SWEEP_INVALID
    return EXIT_OK


def main(argv: Optional[Sequence[str]] = None, out: Callable[[str], None] = print) -> int:
    out(SAFETY_PREAMBLE)
    args = build_parser().parse_args(list(argv) if argv is not None else None)

    if not args.arm:
        out(
            "REFUSED: --arm not given. This tool commands the motors; it will not do so "
            "until an operator states, on the command line, that the checklist above is "
            "satisfied and someone is standing at the switch."
        )
        return EXIT_REFUSED

    try:
        config = config_from_args(args)
    except RefusalError as exc:
        out(f"REFUSED: {exc}")
        return EXIT_REFUSED

    if config.no_early_stop:
        # The preamble above is printed before the arguments are parsed, so it describes
        # the knee-finder. Say plainly that this run does NOT stop at the knee.
        out(
            "CURVE MODE (--no-early-stop): this run does NOT stop at the knee. It will "
            f"drive every rung of {','.join(str(d) for d in config.duties)}, including "
            "duties above breakaway, one bounded burst each. Aborts stay armed; the "
            "power switch is still the only abort software cannot perform."
        )

    holders = scan_port_holders(config.port)
    if holders is None:
        out(
            f"REFUSED: cannot verify who holds {config.port} on this host (no /proc). "
            "This tool must be the only authority on the serial link, and an unverified "
            "claim of exclusivity is exactly the failure this measurement exists to "
            "avoid. Run it on the Pi."
        )
        return EXIT_REFUSED
    if holders:
        out("REFUSED: the serial port is already held:")
        for pid, link in holders:
            out(f"  pid {pid} -> {link}  ({process_name(pid)})")
        out("  Stop the ROS stack first; `ros2 node list` must be empty.")
        return EXIT_REFUSED

    try:
        return asyncio.run(sweep_with_hardware(config, out=out))
    except RefusalError as exc:
        out(f"REFUSED: {exc}")
        return EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main())
