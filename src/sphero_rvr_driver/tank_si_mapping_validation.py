"""Bounded attended validation of the driver's velocity mappings.

Motion is default-off. Armed trials instantiate :class:`RVRDriver`, select its
configured velocity mode, and refresh only ``RVRDriver.set_velocity``. Encoder
reads use the same onboard source and calibration as ROS odometry.
"""

from __future__ import annotations

import argparse
import asyncio
from contextlib import suppress
import csv
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
MAX_TURN_RATE_RAD_S = 1.0
MAX_RAW_TURN_DUTY = 96
DEFAULT_TURN_RATES_RAD_S = [0.2, 0.4, 0.6, 0.8, 1.0]
TURN_PATH_TANK_SI = "tank_si"
TURN_PATH_RAW_DUTY = "raw_duty"
TURN_PATHS = (TURN_PATH_TANK_SI, TURN_PATH_RAW_DUTY)
TURN_MEASUREMENT_S = 0.5
MAX_TURN_PULSE_S = 0.75
MIN_SUSTAINED_YAW_RATE_RAD_S = 0.10
MAX_TRACK_ASYMMETRY_FRACTION = 0.25


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


@dataclass(frozen=True)
class TurnTrialResult:
    command_path: str
    commanded_angular_rad_s: float
    commanded_left_mps: float | None
    commanded_right_mps: float | None
    commanded_left_duty: int | None
    commanded_right_duty: int | None
    measured_angular_rad_s: float
    measured_left_mps: float
    measured_right_mps: float
    relative_error: float
    measurement_duration_s: float
    yaw_change_rad: float
    post_stop_yaw_rad: float
    post_stop_track_travel_m: float
    track_asymmetry_fraction: float
    counter_rotating: bool
    clean_pivot: bool
    opposing_track_motion: bool
    sustained: bool
    stalled: bool
    smooth: bool
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


def analyze_turn_trial(
    *,
    command_path: str = TURN_PATH_TANK_SI,
    commanded_angular_rad_s: float,
    start: TimedEncoderCounts,
    end: TimedEncoderCounts,
    stopped: TimedEncoderCounts,
    counts_per_meter: float,
    wheel_track_m: float,
    max_post_stop_travel_m: float,
) -> TurnTrialResult:
    """Convert opposing encoder travel into measured pure-turn yaw truth."""

    elapsed = end.monotonic_s - start.monotonic_s
    if elapsed <= 0.0:
        raise ValueError("encoder measurement duration must be positive")
    if counts_per_meter <= 0.0 or not math.isfinite(counts_per_meter):
        raise ValueError("counts_per_meter must be positive and finite")
    if wheel_track_m <= 0.0 or not math.isfinite(wheel_track_m):
        raise ValueError("wheel_track_m must be positive and finite")
    if commanded_angular_rad_s <= 0.0 or not math.isfinite(
        commanded_angular_rad_s
    ):
        raise ValueError("commanded_angular_rad_s must be positive and finite")
    if command_path not in TURN_PATHS:
        raise ValueError(f"unsupported turn command path: {command_path}")

    left_delta = _signed_count_delta(start.left, end.left)
    right_delta = _signed_count_delta(start.right, end.right)
    left_mps = left_delta / counts_per_meter / elapsed
    right_mps = right_delta / counts_per_meter / elapsed
    yaw_change_rad = (
        (right_delta - left_delta) / counts_per_meter / wheel_track_m
    )
    measured_angular_rad_s = yaw_change_rad / elapsed
    relative_error = (
        abs(measured_angular_rad_s - commanded_angular_rad_s)
        / commanded_angular_rad_s
    )

    post_left_delta = _signed_count_delta(end.left, stopped.left)
    post_right_delta = _signed_count_delta(end.right, stopped.right)
    post_stop_yaw_rad = (
        (post_right_delta - post_left_delta)
        / counts_per_meter
        / wheel_track_m
    )
    post_stop_track_travel_m = (
        abs(post_left_delta) + abs(post_right_delta)
    ) / (2.0 * counts_per_meter)

    largest_track_speed = max(abs(left_mps), abs(right_mps))
    track_asymmetry_fraction = (
        abs(abs(left_mps) - abs(right_mps)) / largest_track_speed
        if largest_track_speed > 0.0
        else 1.0
    )
    counter_rotating = left_mps < 0.0 < right_mps
    sustained = (
        measured_angular_rad_s >= MIN_SUSTAINED_YAW_RATE_RAD_S
        and yaw_change_rad > 0.0
    )
    clean_pivot = (
        sustained
        and counter_rotating
        and track_asymmetry_fraction <= MAX_TRACK_ASYMMETRY_FRACTION
    )
    half_track = wheel_track_m / 2.0
    duty = int(commanded_angular_rad_s / MAX_TURN_RATE_RAD_S * MAX_RAW_TURN_DUTY)
    is_tank_si = command_path == TURN_PATH_TANK_SI

    return TurnTrialResult(
        command_path=command_path,
        commanded_angular_rad_s=commanded_angular_rad_s,
        commanded_left_mps=(
            -commanded_angular_rad_s * half_track if is_tank_si else None
        ),
        commanded_right_mps=(
            commanded_angular_rad_s * half_track if is_tank_si else None
        ),
        commanded_left_duty=None if is_tank_si else -duty,
        commanded_right_duty=None if is_tank_si else duty,
        measured_angular_rad_s=measured_angular_rad_s,
        measured_left_mps=left_mps,
        measured_right_mps=right_mps,
        relative_error=relative_error,
        measurement_duration_s=elapsed,
        yaw_change_rad=yaw_change_rad,
        post_stop_yaw_rad=post_stop_yaw_rad,
        post_stop_track_travel_m=post_stop_track_travel_m,
        track_asymmetry_fraction=track_asymmetry_fraction,
        counter_rotating=counter_rotating,
        clean_pivot=clean_pivot,
        opposing_track_motion=counter_rotating,
        sustained=sustained,
        stalled=not sustained,
        smooth=clean_pivot,
        stop_within_limit=post_stop_track_travel_m <= max_post_stop_travel_m,
        start_counts=(start.left, start.right),
        end_counts=(end.left, end.right),
        stopped_counts=(stopped.left, stopped.right),
    )


def summarize_turn_trials(results: Sequence[TurnTrialResult]) -> dict:
    """Return the first measured breakaway and smooth pure-turn rates."""

    first_sustained = next((item for item in results if item.sustained), None)
    first_smooth = next((item for item in results if item.smooth), None)
    return {
        "first_sustained_angular_rad_s": (
            first_sustained.commanded_angular_rad_s
            if first_sustained is not None
            else None
        ),
        "first_smooth_angular_rad_s": (
            first_smooth.commanded_angular_rad_s
            if first_smooth is not None
            else None
        ),
        "stalled_angular_rates_rad_s": [
            item.commanded_angular_rad_s for item in results if item.stalled
        ],
    }


def summarize_turn_paths(results: Sequence[TurnTrialResult]) -> dict:
    """Report counter-rotation and clean-pivot outcomes for each driver path."""

    summaries = {}
    for path in TURN_PATHS:
        path_results = [item for item in results if item.command_path == path]
        first_counter_rotating = next(
            (item for item in path_results if item.counter_rotating), None
        )
        summary = summarize_turn_trials(path_results)
        summaries[path] = {
            **summary,
            "first_counter_rotating_angular_rad_s": (
                first_counter_rotating.commanded_angular_rad_s
                if first_counter_rotating is not None
                else None
            ),
            "actuates_clean_pivot": any(item.clean_pivot for item in path_results),
        }
    return {
        "path_summaries": summaries,
        "clean_pivot_paths": [
            path
            for path in TURN_PATHS
            if summaries[path]["actuates_clean_pivot"]
        ],
    }


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
    driver: RVRDriver,
    linear_mps: float,
    angular_rad_s: float,
    duration_s: float,
    refresh_s: float,
) -> None:
    deadline = asyncio.get_running_loop().time() + duration_s
    while True:
        await driver.set_velocity(linear_mps, angular_rad_s)
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0.0:
            return
        await asyncio.sleep(min(refresh_s, remaining))


async def _read_counts_while_commanding(
    driver: RVRDriver,
    linear_mps: float,
    angular_rad_s: float,
    refresh_s: float,
) -> TimedEncoderCounts:
    """Keep the driver's lease fresh until one encoder response is complete."""

    read_task = asyncio.create_task(_read_counts(driver))
    try:
        while not read_task.done():
            await driver.set_velocity(linear_mps, angular_rad_s)
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
    await _keep_velocity_fresh(driver, speed_mps, 0.0, warmup_s, refresh_s)
    start = await _read_counts_while_commanding(driver, speed_mps, 0.0, refresh_s)
    try:
        await _keep_velocity_fresh(
            driver, speed_mps, 0.0, measurement_s, refresh_s
        )
        end = await _read_counts_while_commanding(
            driver, speed_mps, 0.0, refresh_s
        )
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


async def _run_turn_trial(
    driver: RVRDriver,
    *,
    command_path: str,
    angular_rad_s: float,
    warmup_s: float,
    measurement_s: float,
    settle_s: float,
    refresh_s: float,
    counts_per_meter: float,
    wheel_track_m: float,
    max_post_stop_travel_m: float,
) -> TurnTrialResult:
    await driver.stop()
    await asyncio.sleep(settle_s)
    before = await _read_counts(driver)
    pulse_s = warmup_s + measurement_s
    started_s = asyncio.get_running_loop().time()
    try:
        await _keep_velocity_fresh(driver, 0.0, angular_rad_s, pulse_s, refresh_s)
    finally:
        try:
            await driver.set_velocity(0.0, 0.0)
        finally:
            await driver.stop()
    stopped_command_s = asyncio.get_running_loop().time()
    after = await _read_counts(driver)
    start = TimedEncoderCounts(started_s, before.left, before.right)
    end = TimedEncoderCounts(stopped_command_s, after.left, after.right)
    await asyncio.sleep(settle_s)
    stopped = await _read_counts(driver)
    return analyze_turn_trial(
        command_path=command_path,
        commanded_angular_rad_s=angular_rad_s,
        start=start,
        end=end,
        stopped=stopped,
        counts_per_meter=counts_per_meter,
        wheel_track_m=wheel_track_m,
        max_post_stop_travel_m=max_post_stop_travel_m,
    )


def _driver_for_path(args: argparse.Namespace, command_path: str) -> RVRDriver:
    transport = SerialTransport(
        port=args.port, baud_rate=args.baud, read_timeout=0.1
    )
    return RVRDriver(
        transport=transport,
        control_period=args.control_period,
        command_timeout=args.command_timeout,
        max_linear_mps=MAX_VALIDATION_SPEED_MPS,
        max_angular_rad_s=(
            MAX_TURN_RATE_RAD_S if args.mode == "turn" else 0.4
        ),
        max_raw_motor_duty=MAX_RAW_TURN_DUTY,
        max_linear_raw_motor_duty=0,
        max_angular_raw_motor_duty=MAX_RAW_TURN_DUTY,
        velocity_control_mode=(
            RVRDriver.VELOCITY_CONTROL_RAW_MOTOR
            if command_path == TURN_PATH_RAW_DUTY
            else RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI
        ),
        wheel_track_m=args.wheel_track_m,
    )


async def _run_turn_path(
    args: argparse.Namespace, command_path: str
) -> tuple[int, list[TurnTrialResult]]:
    driver = _driver_for_path(args, command_path)
    results = []
    try:
        await driver.connect()
        await driver.stop()
        battery = await asyncio.wait_for(
            driver.get_battery_percentage(), timeout=2.5
        )
        for angular_rate in args.angular_rates:
            results.append(
                await _run_turn_trial(
                    driver,
                    command_path=command_path,
                    angular_rad_s=angular_rate,
                    warmup_s=args.warmup,
                    measurement_s=TURN_MEASUREMENT_S,
                    settle_s=args.settle,
                    refresh_s=min(args.control_period, args.command_timeout / 2.0),
                    counts_per_meter=args.counts_per_meter,
                    wheel_track_m=args.wheel_track_m,
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
    return battery, results


async def _run_hardware(args: argparse.Namespace) -> dict:
    if args.mode == "turn":
        results: list[TurnTrialResult] = []
        batteries = {}
        for command_path in TURN_PATHS:
            battery, path_results = await _run_turn_path(args, command_path)
            batteries[command_path] = battery
            results.extend(path_results)

        turn_summary = summarize_turn_paths(results)
        all_stops_pass = all(result.stop_within_limit for result in results)
        return {
            "schema": "sphero_rvr.wheels_up_turn_actuation.v1",
            "mode": "turn",
            "surface": args.surface,
            "wheels_up_precheck": "PASS",
            "driver_paths": {
                TURN_PATH_TANK_SI: "RVRDriver.set_velocity/native_tank_si",
                TURN_PATH_RAW_DUTY: "RVRDriver.set_velocity/raw_motor",
            },
            "battery_percentage_by_path": batteries,
            "counts_per_meter": args.counts_per_meter,
            "wheel_track_m": args.wheel_track_m,
            "hard_angular_ceiling_rad_s": MAX_TURN_RATE_RAD_S,
            "hard_raw_turn_duty_ceiling": MAX_RAW_TURN_DUTY,
            "max_nonzero_pulse_s": MAX_TURN_PULSE_S,
            "min_sustained_yaw_rate_rad_s": MIN_SUSTAINED_YAW_RATE_RAD_S,
            "max_track_asymmetry_fraction": MAX_TRACK_ASYMMETRY_FRACTION,
            "max_post_stop_travel_m": args.max_post_stop_travel,
            "trials": [asdict(result) for result in results],
            **turn_summary,
            "stop_check": "PASS" if all_stops_pass else "FAIL",
        }

    driver = _driver_for_path(args, TURN_PATH_TANK_SI)
    results: list = []
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
    parser.add_argument(
        "--mode", choices=("linear", "turn"), default="linear"
    )
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--speeds", type=float, nargs="+", default=[0.03, 0.05]
    )
    parser.add_argument(
        "--angular-rates",
        type=float,
        nargs="+",
        default=DEFAULT_TURN_RATES_RAD_S,
    )
    parser.add_argument("--warmup", type=float)
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
    parser.add_argument(
        "--wheels-up-confirmed",
        action="store_true",
        help="confirm every wheel is securely clear of the ground",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--csv-output", type=Path)
    parser.add_argument("--overall-timeout", type=float, default=30.0)
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.warmup is None:
        args.warmup = 0.25 if args.mode == "turn" else 0.5
    if args.mode == "linear":
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
    else:
        if args.surface != "wheels-up":
            parser.error("turn actuation comparison is wheels-up only")
        if not args.angular_rates:
            parser.error("at least one angular rate is required")
        if any(
            not math.isfinite(rate)
            or rate <= 0.0
            or rate > MAX_TURN_RATE_RAD_S
            for rate in args.angular_rates
        ):
            parser.error("every angular rate must be > 0 and <= 1.0 rad/s")
        if args.angular_rates != sorted(set(args.angular_rates)):
            parser.error("angular rates must be unique and increasing")
        if args.warmup + TURN_MEASUREMENT_S > MAX_TURN_PULSE_S:
            parser.error("turn warmup plus measurement must be <= 0.75 s")
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
    if args.armed and args.mode == "turn" and not args.wheels_up_confirmed:
        parser.error("an armed turn run requires --wheels-up-confirmed")
    if args.armed and args.surface == "floor" and not args.floor_area_clear:
        parser.error("an armed floor run requires --floor-area-clear")


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)
    plan = {
        "motion": "ENABLED" if args.armed else "DISABLED",
        "mode": args.mode,
        "surface": args.surface,
        "driver_path": "RVRDriver.set_velocity/native_tank_si",
        "driver_paths": (
            {
                TURN_PATH_TANK_SI: "RVRDriver.set_velocity/native_tank_si",
                TURN_PATH_RAW_DUTY: "RVRDriver.set_velocity/raw_motor",
            }
            if args.mode == "turn"
            else None
        ),
        "wheels_up_precheck": (
            "PASS"
            if args.mode == "turn" and args.wheels_up_confirmed
            else "NOT_CONFIRMED"
            if args.mode == "turn"
            else None
        ),
        "speeds_mps": args.speeds if args.mode == "linear" else [],
        "angular_rates_rad_s": (
            args.angular_rates if args.mode == "turn" else []
        ),
        "max_linear_mps": MAX_VALIDATION_SPEED_MPS,
        "diagnostic_max_angular_rad_s": (
            MAX_TURN_RATE_RAD_S if args.mode == "turn" else 0.4
        ),
        "hard_raw_turn_duty_ceiling": (
            MAX_RAW_TURN_DUTY if args.mode == "turn" else None
        ),
        "max_nonzero_pulse_s": (
            MAX_TURN_PULSE_S if args.mode == "turn" else None
        ),
        "warmup_s": args.warmup,
        "measurement_s": TURN_MEASUREMENT_S if args.mode == "turn" else args.measurement,
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
    if args.csv_output is not None:
        trials = report["trials"]
        with args.csv_output.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(trials[0]))
            writer.writeheader()
            writer.writerows(trials)
    if args.mode == "turn":
        for trial in report["trials"]:
            print(
                "TURN_RESULT "
                f"path={trial['command_path']} "
                f"omega_rad_s={trial['commanded_angular_rad_s']:.3f} "
                f"left_mps={trial['measured_left_mps']:.6f} "
                f"right_mps={trial['measured_right_mps']:.6f} "
                f"counter_rotating={'yes' if trial['counter_rotating'] else 'no'} "
                f"clean_pivot={'yes' if trial['clean_pivot'] else 'no'}"
            )
        print(
            "TURN_SUMMARY "
            f"clean_pivot_paths={','.join(report['clean_pivot_paths']) or 'none'} "
            f"stop_check={report['stop_check']}"
        )
        return 0 if report["stop_check"] == "PASS" else 1
    if args.surface == "floor":
        return 0 if report["acceptance"] == "PASS" else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
