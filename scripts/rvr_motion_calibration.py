#!/usr/bin/env python3
"""Tiny, gated RVR motion/odometry calibration helper.

Default mode performs no-motion telemetry checks only. Pass --armed to allow a
very short low-speed velocity pulse, then physically measure the distance moved
and rerun/pass --actual-distance-m to compute odometry scale.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from sphero_rvr_core.driver import RVRDriver  # noqa: E402
from sphero_rvr_core.responses import EncoderCounts  # noqa: E402
from sphero_rvr_core.serial_transport import SerialTransport  # noqa: E402


def encoder_delta(before: EncoderCounts, after: EncoderCounts) -> tuple[int, int]:
    return after.left - before.left, after.right - before.right


def mean_abs_delta(left_delta: int, right_delta: int) -> float:
    return (abs(left_delta) + abs(right_delta)) / 2.0


def feet_to_meters(value: float) -> float:
    return value * 0.3048


async def read_counts(driver: RVRDriver, label: str) -> EncoderCounts:
    counts = await asyncio.wait_for(driver.get_encoder_counts(), timeout=2.5)
    print(f"{label}: left={counts.left} right={counts.right}")
    return counts


async def main_async() -> int:
    parser = argparse.ArgumentParser(
        description="Gated tiny-motion calibration for Sphero RVR /cmd_vel scale and encoder odometry."
    )
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--linear", type=float, default=0.02, help="requested linear m/s for the pulse")
    parser.add_argument("--duration", type=float, default=0.25, help="seconds to hold the velocity command")
    parser.add_argument("--max-linear", type=float, default=0.10, help="driver max_linear_mps used for mapping")
    parser.add_argument("--max-angular", type=float, default=0.4)
    parser.add_argument("--max-duty", type=int, default=32, help="driver raw motor duty cap for calibration")
    parser.add_argument(
        "--control-mode",
        choices=(RVRDriver.VELOCITY_CONTROL_RAW_MOTOR, RVRDriver.VELOCITY_CONTROL_NATIVE_RC_SI),
        default=RVRDriver.VELOCITY_CONTROL_RAW_MOTOR,
        help="packet backend; raw_motor is the measured ground-calibration path",
    )
    parser.add_argument("--command-timeout", type=float, default=0.25)
    parser.add_argument("--control-period", type=float, default=0.05)
    parser.add_argument("--settle", type=float, default=0.50)
    parser.add_argument("--actual-distance-m", type=float, help="measured physical travel in meters")
    parser.add_argument("--actual-distance-ft", type=float, help="measured physical travel in feet")
    parser.add_argument("--armed", action="store_true", help="ALLOW the short motion pulse")
    args = parser.parse_args()

    if args.actual_distance_m is not None and args.actual_distance_ft is not None:
        parser.error("pass only one of --actual-distance-m or --actual-distance-ft")
    actual_distance_m = args.actual_distance_m
    if args.actual_distance_ft is not None:
        actual_distance_m = feet_to_meters(args.actual_distance_ft)

    print("RVR motion calibration")
    print(f"port={args.port} baud={args.baud}")
    print(
        "pulse_config "
        f"linear={args.linear:.4f}m/s duration={args.duration:.3f}s "
        f"max_linear={args.max_linear:.3f}m/s max_duty={args.max_duty} "
        f"control_mode={args.control_mode} command_timeout={args.command_timeout:.3f}s"
    )
    print(f"nominal_distance_m={args.linear * args.duration:.4f}")
    if actual_distance_m is not None:
        print(f"actual_distance_m={actual_distance_m:.4f}")

    transport = SerialTransport(port=args.port, baud_rate=args.baud, read_timeout=0.1)
    driver = RVRDriver(
        transport=transport,
        control_period=args.control_period,
        command_timeout=args.command_timeout,
        max_linear_mps=args.max_linear,
        max_angular_rad_s=args.max_angular,
        max_raw_motor_duty=args.max_duty,
        max_linear_raw_motor_duty=args.max_duty,
        max_angular_raw_motor_duty=args.max_duty,
        velocity_control_mode=args.control_mode,
    )

    await driver.connect()
    try:
        battery = await asyncio.wait_for(driver.get_battery_percentage(), timeout=2.5)
        voltage_state = await asyncio.wait_for(driver.get_battery_voltage_state(), timeout=2.5)
        print(f"battery_percentage={battery}")
        print(f"battery_voltage_state={voltage_state.state_name}")

        await driver.stop()
        await driver.raw_motors(0, 0, 0, 0)
        await asyncio.sleep(args.settle)

        before = await read_counts(driver, "encoders_before")

        if not args.armed:
            print("MOTION_SKIPPED: pass --armed only when the RVR is physically ready.")
            print("Suggested first armed run:")
            print(
                "  python3 scripts/rvr_motion_calibration.py --armed "
                "--linear 0.02 --duration 0.25 --max-duty 32"
            )
            return 0

        print("ARMED: running tiny velocity pulse now")
        # Keep the velocity command fresh for the whole pulse. A single
        # set_velocity() can be missed or stale out near the drivetrain
        # deadband, which made early calibration pulses inconsistent.
        deadline = asyncio.get_running_loop().time() + args.duration
        refresh_period = max(0.02, min(args.control_period, args.command_timeout / 2.0))
        refresh_count = 0
        while asyncio.get_running_loop().time() < deadline:
            await driver.set_velocity(args.linear, 0.0)
            refresh_count += 1
            await asyncio.sleep(refresh_period)
        print(f"velocity_refresh_count={refresh_count}")
        await driver.set_velocity(0.0, 0.0)
        await driver.stop()
        await driver.raw_motors(0, 0, 0, 0)
        await asyncio.sleep(args.settle)

        after = await read_counts(driver, "encoders_after")
        left_delta, right_delta = encoder_delta(before, after)
        mean_delta = mean_abs_delta(left_delta, right_delta)
        print(f"encoder_delta_left={left_delta}")
        print(f"encoder_delta_right={right_delta}")
        print(f"encoder_delta_mean_abs={mean_delta:.3f}")

        if mean_delta == 0:
            print("RESULT=NO_ENCODER_MOVEMENT_DETECTED")
            print("Increase --duration slightly or --max-duty cautiously; keep the robot restrained/clear.")
            return 2

        if actual_distance_m is None:
            print("RESULT=MEASURE_DISTANCE_NEEDED")
            print("Measure the physical travel, then compute scale with one of:")
            print(
                f"  python3 scripts/rvr_motion_calibration.py --actual-distance-ft <feet> "
                f"# use encoder_delta_mean_abs={mean_delta:.3f}"
            )
            print(f"manual_counts_per_meter = {mean_delta:.3f} / actual_distance_m")
            return 0

        if actual_distance_m <= 0:
            raise ValueError("actual distance must be positive")
        counts_per_meter = mean_delta / actual_distance_m
        meters_per_count = actual_distance_m / mean_delta
        print("RESULT=CALIBRATION_SCALE")
        print(f"counts_per_meter={counts_per_meter:.3f}")
        print(f"meters_per_count={meters_per_count:.8f}")
        print("Suggested config/rvr.yaml update:")
        print(f"  odom_counts_per_meter: {counts_per_meter:.3f}")
        if abs(left_delta - right_delta) > max(5, mean_delta * 0.25):
            print("WARNING: left/right encoder deltas differ substantially; repeat and check straightness/track slip.")
        return 0
    finally:
        try:
            await driver.set_velocity(0.0, 0.0)
            await driver.stop()
            await driver.raw_motors(0, 0, 0, 0)
        except Exception:
            pass
        await driver.disconnect()


def main() -> int:
    return asyncio.run(main_async())


if __name__ == "__main__":
    raise SystemExit(main())
