"""Bounded attended validation of the driver's native tank-SI velocity map.

Motion is default-off. Armed trials instantiate :class:`RVRDriver`, select its
native tank-SI mode, and refresh only ``RVRDriver.set_velocity``. Encoder reads
use the same onboard source and calibration as ROS odometry.
"""

from __future__ import annotations

import argparse
import asyncio
from contextlib import suppress
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
from typing import Sequence

from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.responses import EncoderCounts
from sphero_rvr_core.serial_transport import SerialTransport


DEFAULT_COUNTS_PER_METER = 4337.768
DEFAULT_WHEEL_TRACK_M = 0.2507
MAX_VALIDATION_SPEED_MPS = 0.05
REQUIRED_ACCEPTANCE_SPEED_MPS = 0.05


@dataclass(frozen=True)
class TimedEncoderCounts:
    monotonic_s: float
    left: int
    right: int


@dataclass(frozen=True)
class MappingTrialResult:
    commanded_mps: float
    measured_mps: float
    measured_left_mps: float
    measured_right_mps: float
    relative_error: float
    measurement_duration_s: float
    measurement_distance_m: float
    post_stop_travel_m: float
    speed_within_tolerance: bool
    stop_within_limit: bool
    start_counts: tuple[int, int]
    end_counts: tuple[int, int]
    stopped_counts: tuple[int, int]


def _signed_count_delta(before: int, after: int) -> int:
    """Return an int32 encoder delta, including a possible rollover."""

    delta = int(after) - int(before)
    if delta > 2**31:
        delta -= 2**32
    elif delta < -(2**31):
        delta += 2**32
    return delta


def analyze_trial(
    *,
    commanded_mps: float,
    start: TimedEncoderCounts,
    end: TimedEncoderCounts,
    stopped: TimedEncoderCounts,
    counts_per_meter: float,
    speed_tolerance_fraction: float,
    max_post_stop_travel_m: float,
) -> MappingTrialResult:
    """Convert encoder samples into commanded-versus-actual speed truth."""

    elapsed = end.monotonic_s - start.monotonic_s
    if elapsed <= 0.0:
        raise ValueError("encoder measurement duration must be positive")
    if counts_per_meter <= 0.0 or not math.isfinite(counts_per_meter):
        raise ValueError("counts_per_meter must be positive and finite")
    if commanded_mps <= 0.0 or not math.isfinite(commanded_mps):
        raise ValueError("commanded_mps must be positive and finite")

    left_delta = _signed_count_delta(start.left, end.left)
    right_delta = _signed_count_delta(start.right, end.right)
    left_mps = left_delta / counts_per_meter / elapsed
    right_mps = right_delta / counts_per_meter / elapsed
    measured_mps = (left_mps + right_mps) / 2.0
    relative_error = abs(measured_mps - commanded_mps) / commanded_mps

    post_left = abs(_signed_count_delta(end.left, stopped.left))
    post_right = abs(_signed_count_delta(end.right, stopped.right))
    post_stop_travel_m = (post_left + post_right) / (2.0 * counts_per_meter)

    return MappingTrialResult(
        commanded_mps=commanded_mps,
        measured_mps=measured_mps,
        measured_left_mps=left_mps,
        measured_right_mps=right_mps,
        relative_error=relative_error,
        measurement_duration_s=elapsed,
        measurement_distance_m=measured_mps * elapsed,
        post_stop_travel_m=post_stop_travel_m,
        speed_within_tolerance=relative_error <= speed_tolerance_fraction,
        stop_within_limit=post_stop_travel_m <= max_post_stop_travel_m,
        start_counts=(start.left, start.right),
        end_counts=(end.left, end.right),
        stopped_counts=(stopped.left, stopped.right),
    )


async def _read_counts(driver: RVRDriver) -> TimedEncoderCounts:
    counts: EncoderCounts = await asyncio.wait_for(
        driver.get_encoder_counts(), timeout=2.5
    )
    return TimedEncoderCounts(
        monotonic_s=asyncio.get_running_loop().time(),
        left=counts.left,
        right=counts.right,
    )


async def _keep_velocity_fresh(
    driver: RVRDriver, speed_mps: float, duration_s: float, refresh_s: float
) -> None:
    deadline = asyncio.get_running_loop().time() + duration_s
    while True:
        await driver.set_velocity(speed_mps, 0.0)
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0.0:
            return
        await asyncio.sleep(min(refresh_s, remaining))


async def _read_counts_while_commanding(
    driver: RVRDriver, speed_mps: float, refresh_s: float
) -> TimedEncoderCounts:
    """Keep the driver's lease fresh until one encoder response is complete."""

    read_task = asyncio.create_task(_read_counts(driver))
    try:
        while not read_task.done():
            await driver.set_velocity(speed_mps, 0.0)
            done, _pending = await asyncio.wait(
                {read_task}, timeout=refresh_s
            )
            if done:
                break
        return await read_task
    finally:
        if not read_task.done():
            read_task.cancel()
        with suppress(asyncio.CancelledError):
            await read_task


async def _run_trial(
    driver: RVRDriver,
    *,
    speed_mps: float,
    warmup_s: float,
    measurement_s: float,
    settle_s: float,
    refresh_s: float,
    counts_per_meter: float,
    speed_tolerance_fraction: float,
    max_post_stop_travel_m: float,
) -> MappingTrialResult:
    await driver.stop()
    await asyncio.sleep(settle_s)
    await _keep_velocity_fresh(driver, speed_mps, warmup_s, refresh_s)
    start = await _read_counts_while_commanding(driver, speed_mps, refresh_s)
    try:
        await _keep_velocity_fresh(driver, speed_mps, measurement_s, refresh_s)
        end = await _read_counts_while_commanding(driver, speed_mps, refresh_s)
    finally:
        try:
            await driver.set_velocity(0.0, 0.0)
        finally:
            await driver.stop()
    await asyncio.sleep(settle_s)
    stopped = await _read_counts(driver)
    return analyze_trial(
        commanded_mps=speed_mps,
        start=start,
        end=end,
        stopped=stopped,
        counts_per_meter=counts_per_meter,
        speed_tolerance_fraction=speed_tolerance_fraction,
        max_post_stop_travel_m=max_post_stop_travel_m,
    )


async def _run_hardware(args: argparse.Namespace) -> dict:
    transport = SerialTransport(
        port=args.port, baud_rate=args.baud, read_timeout=0.1
    )
    driver = RVRDriver(
        transport=transport,
        control_period=args.control_period,
        command_timeout=args.command_timeout,
        max_linear_mps=MAX_VALIDATION_SPEED_MPS,
        max_angular_rad_s=0.4,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
        wheel_track_m=args.wheel_track_m,
    )
    results: list[MappingTrialResult] = []
    try:
        await driver.connect()
        await driver.stop()
        battery = await asyncio.wait_for(
            driver.get_battery_percentage(), timeout=2.5
        )
        for speed in args.speeds:
            results.append(
                await _run_trial(
                    driver,
                    speed_mps=speed,
                    warmup_s=args.warmup,
                    measurement_s=args.measurement,
                    settle_s=args.settle,
                    refresh_s=min(
                        args.control_period, args.command_timeout / 2.0
                    ),
                    counts_per_meter=args.counts_per_meter,
                    speed_tolerance_fraction=args.speed_tolerance,
                    max_post_stop_travel_m=args.max_post_stop_travel,
                )
            )
    finally:
        try:
            try:
                await driver.set_velocity(0.0, 0.0)
            finally:
                await driver.stop()
        finally:
            await driver.disconnect()

    required = next(
        result
        for result in results
        if math.isclose(
            result.commanded_mps, REQUIRED_ACCEPTANCE_SPEED_MPS, abs_tol=1e-9
        )
    )
    all_stops_pass = all(result.stop_within_limit for result in results)
    floor_pass = required.speed_within_tolerance and all_stops_pass
    return {
        "schema": "sphero_rvr.tank_si_mapping_validation.v1",
        "surface": args.surface,
        "driver_path": "RVRDriver.set_velocity/native_tank_si",
        "battery_percentage": battery,
        "counts_per_meter": args.counts_per_meter,
        "wheel_track_m": args.wheel_track_m,
        "speed_tolerance_fraction": args.speed_tolerance,
        "max_post_stop_travel_m": args.max_post_stop_travel,
        "trials": [asdict(result) for result in results],
        "acceptance": (
            "PASS"
            if args.surface == "floor" and floor_pass
            else "FAIL"
            if args.surface == "floor"
            else "BENCH_ONLY_NOT_GROUND_SPEED_ACCEPTANCE"
        ),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--speeds", type=float, nargs="+", default=[0.03, 0.05]
    )
    parser.add_argument("--warmup", type=float, default=0.5)
    parser.add_argument("--measurement", type=float, default=2.0)
    parser.add_argument("--settle", type=float, default=0.75)
    parser.add_argument("--control-period", type=float, default=0.05)
    parser.add_argument("--command-timeout", type=float, default=0.30)
    parser.add_argument(
        "--counts-per-meter", type=float, default=DEFAULT_COUNTS_PER_METER
    )
    parser.add_argument(
        "--wheel-track-m", type=float, default=DEFAULT_WHEEL_TRACK_M
    )
    parser.add_argument("--speed-tolerance", type=float, default=0.20)
    parser.add_argument("--max-post-stop-travel", type=float, default=0.01)
    parser.add_argument(
        "--surface", choices=("wheels-up", "floor"), default="wheels-up"
    )
    parser.add_argument(
        "--armed",
        action="store_true",
        help="enable the bounded motor trials; omitted means no serial and no motion",
    )
    parser.add_argument(
        "--attended",
        action="store_true",
        help="confirm an operator is present with a hand at chassis power",
    )
    parser.add_argument(
        "--floor-area-clear",
        action="store_true",
        help="confirm a level bounded floor has no drop-offs or obstacles",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overall-timeout", type=float, default=30.0)
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if not args.speeds:
        parser.error("at least one speed is required")
    if any(
        not math.isfinite(speed)
        or speed <= 0.0
        or speed > MAX_VALIDATION_SPEED_MPS
        for speed in args.speeds
    ):
        parser.error("every speed must be > 0 and <= 0.05 m/s")
    if not any(
        math.isclose(speed, REQUIRED_ACCEPTANCE_SPEED_MPS, abs_tol=1e-9)
        for speed in args.speeds
    ):
        parser.error("the mandatory 0.05 m/s acceptance trial is required")
    for name, lower, upper in (
        ("warmup", 0.2, 1.0),
        ("measurement", 0.5, 3.0),
        ("settle", 0.25, 2.0),
        ("control_period", 0.02, 0.10),
        ("command_timeout", 0.20, 0.50),
        ("overall_timeout", 10.0, 60.0),
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or not lower <= value <= upper:
            parser.error(f"--{name.replace('_', '-')} must be in [{lower}, {upper}]")
    if not 0.0 < args.speed_tolerance <= 0.20:
        parser.error("--speed-tolerance must be > 0 and <= 0.20")
    if not 0.0 <= args.max_post_stop_travel <= 0.01:
        parser.error("--max-post-stop-travel must be in [0, 0.01] m")
    if args.armed and not args.attended:
        parser.error("--armed requires --attended")
    if args.armed and args.surface == "floor" and not args.floor_area_clear:
        parser.error("an armed floor run requires --floor-area-clear")


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)
    plan = {
        "motion": "ENABLED" if args.armed else "DISABLED",
        "surface": args.surface,
        "driver_path": "RVRDriver.set_velocity/native_tank_si",
        "speeds_mps": args.speeds,
        "max_linear_mps": MAX_VALIDATION_SPEED_MPS,
        "warmup_s": args.warmup,
        "measurement_s": args.measurement,
        "settle_s": args.settle,
    }
    print(json.dumps({"plan": plan}, sort_keys=True))
    if not args.armed:
        print("MOTION_SKIPPED: add the attended acknowledgements only at the rover.")
        return 0

    try:
        report = asyncio.run(
            asyncio.wait_for(_run_hardware(args), timeout=args.overall_timeout)
        )
    except KeyboardInterrupt:
        print("INTERRUPTED: stop/disconnect cleanup requested", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"VALIDATION_ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    encoded = json.dumps(report, indent=2, sort_keys=True)
    print(encoded)
    if args.output is not None:
        args.output.write_text(encoded + "\n", encoding="utf-8")
    if args.surface == "floor":
        return 0 if report["acceptance"] == "PASS" else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
