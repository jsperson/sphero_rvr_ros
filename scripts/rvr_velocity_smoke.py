#!/usr/bin/env python3
"""Driver-level suspended velocity smoke test for Sphero RVR.

This validates the actual high-level path:

    RVRDriver.set_velocity()
      -> control loop
      -> RVRCommands.drive_rc()
      -> Dispatcher.send()
      -> SerialTransport
      -> RVR

By default this script performs only no-motion liveness checks. Pass --armed to
allow a short suspended movement pulse. Only run --armed when the RVR is safely
suspended or otherwise restrained.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from sphero_rvr_core.driver import RVRDriver  # noqa: E402
from sphero_rvr_core.serial_transport import SerialTransport  # noqa: E402


def delta(before, after):
    return after.left - before.left, after.right - before.right


async def main_async() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--linear", type=float, default=0.5, help="driver linear velocity input for suspended pulse")
    parser.add_argument("--angular", type=float, default=0.0, help="driver angular velocity input for suspended pulse")
    parser.add_argument("--duration", type=float, default=0.25, help="seconds to keep velocity command fresh")
    parser.add_argument("--control-period", type=float, default=0.05)
    parser.add_argument("--command-timeout", type=float, default=0.30)
    parser.add_argument("--max-linear", type=float, default=0.5)
    parser.add_argument("--max-angular", type=float, default=0.5)
    parser.add_argument("--settle", type=float, default=0.35)
    parser.add_argument("--armed", action="store_true", help="allow set_velocity() to start motors")
    args = parser.parse_args()

    transport = SerialTransport(port=args.port, baud_rate=115200, read_timeout=0.1)
    driver = RVRDriver(
        transport=transport,
        control_period=args.control_period,
        command_timeout=args.command_timeout,
        max_linear_mps=args.max_linear,
        max_angular_rad_s=args.max_angular,
    )

    print(f"opened driver on {args.port} @ 115200")
    print(
        "config "
        f"linear={args.linear:.3f} angular={args.angular:.3f} "
        f"duration={args.duration:.2f}s command_timeout={args.command_timeout:.2f}s "
        f"control_period={args.control_period:.2f}s max_linear={args.max_linear:.3f} max_angular={args.max_angular:.3f}"
    )

    await driver.connect()
    try:
        battery = await driver.get_battery_percentage()
        print(f"battery_percentage={battery}")
        try:
            voltage = await driver.get_battery_voltage()
            print(f"battery_voltage={voltage:.3f}V")
        except Exception as exc:
            print(f"battery_voltage=UNAVAILABLE ({exc})")

        await driver.set_all_leds(0, 32, 32)
        await driver.stop()
        await driver.raw_motors(0, 0, 0, 0)
        await asyncio.sleep(args.settle)

        before = await driver.get_encoder_counts()
        print(f"encoders_before={before}")

        if not args.armed:
            print("movement skipped; pass --armed only when RVR is safely suspended/restrained")
            return 0

        print("running ARMED driver velocity pulse")
        await driver.set_velocity(args.linear, args.angular)
        await asyncio.sleep(args.duration)

        # Do not refresh the command. Let the stale-command watchdog issue its stop.
        stale_wait = args.command_timeout + args.settle
        print(f"waiting {stale_wait:.2f}s for stale-command timeout stop")
        await asyncio.sleep(stale_wait)

        after_stale = await driver.get_encoder_counts()
        dl, dr = delta(before, after_stale)
        print(f"encoders_after_stale_timeout={after_stale}")
        print(f"encoder_delta_after_stale_timeout_left={dl} encoder_delta_after_stale_timeout_right={dr}")

        # Belt-and-suspenders explicit stop/zero after observing the stale timeout.
        await driver.stop()
        await driver.raw_motors(0, 0, 0, 0)
        await asyncio.sleep(args.settle)

        after_stop = await driver.get_encoder_counts()
        sdl, sdr = delta(after_stale, after_stop)
        print(f"encoders_after_explicit_stop={after_stop}")
        print(f"encoder_drift_after_explicit_stop_left={sdl} encoder_drift_after_explicit_stop_right={sdr}")

        if dl == 0 and dr == 0:
            print("RESULT=NO_MOVEMENT_DETECTED")
            return 2
        print("RESULT=DRIVER_VELOCITY_PATH_MOVED")
        return 0
    finally:
        try:
            await driver.raw_motors(0, 0, 0, 0)
            await driver.stop()
        except Exception:
            pass
        await driver.disconnect()


def main() -> int:
    return asyncio.run(main_async())


if __name__ == "__main__":
    raise SystemExit(main())
