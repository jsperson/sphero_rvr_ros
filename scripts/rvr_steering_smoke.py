#!/usr/bin/env python3
"""Suspended steering/yaw smoke test for Sphero RVR velocity mapping.

This exercises the actual RVRCommands.drive_rc(linear, angular) path rather
than direct raw-motor packets. Run only with the RVR suspended or safely
restrained.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from sphero_rvr_core import responses  # noqa: E402
from sphero_rvr_core.commands import RVRCommands  # noqa: E402
from sphero_rvr_core.packet import EOP, SOP, Packet  # noqa: E402


@dataclass(frozen=True)
class VelocityPulse:
    name: str
    linear_mps: float
    angular_rad_s: float


def read_frame(ser: serial.Serial, timeout: float = 1.0) -> bytes | None:
    deadline = time.monotonic() + timeout
    in_frame = False
    frame = bytearray()
    while time.monotonic() < deadline:
        byte = ser.read(1)
        if byte == b"":
            continue
        value = byte[0]
        if not in_frame:
            if value == SOP:
                in_frame = True
                frame.append(value)
            continue
        frame.append(value)
        if value == EOP:
            return bytes(frame)
    return None


def describe(packet: Packet) -> str:
    return (
        f"did=0x{packet.device_id:02x} cid=0x{packet.command_id:02x} "
        f"seq={packet.sequence_id} flags=0x{packet.flags:02x} err={packet.error} "
        f"payload={packet.payload.hex()}"
    )


def drain(ser: serial.Serial, seconds: float = 0.25) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = read_frame(ser, timeout=max(0.01, deadline - time.monotonic()))
        if raw is None:
            break
        try:
            Packet.decode(raw)
        except Exception:
            pass


def write_packet(ser: serial.Serial, name: str, packet: Packet, verbose: bool) -> None:
    if verbose:
        print(f"TX {name}: {describe(packet)} frame={packet.encode().hex()}")
    ser.write(packet.encode())
    ser.flush()


def request(ser: serial.Serial, name: str, packet: Packet, verbose: bool, timeout: float = 2.0) -> Packet:
    write_packet(ser, name, packet, verbose)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = read_frame(ser, timeout=max(0.05, deadline - time.monotonic()))
        if raw is None:
            break
        try:
            rx = Packet.decode(raw)
        except Exception as exc:
            print(f"RX decode error: {exc} raw={raw.hex()}")
            continue
        if verbose:
            print(f"RX: {describe(rx)}")
        if (rx.device_id, rx.command_id, rx.sequence_id) == (
            packet.device_id,
            packet.command_id,
            packet.sequence_id,
        ):
            if rx.error not in (None, 0):
                raise RuntimeError(f"{name} returned protocol error {rx.error}")
            return rx
    raise TimeoutError(f"Timed out waiting for {name} response")


def stop_all(ser: serial.Serial, commands: RVRCommands, next_seq, verbose: bool) -> None:
    # Use only validated stop/off commands here. A brake-style raw motor mode is
    # intentionally not used until confirmed against RVR firmware semantics.
    write_packet(ser, "raw_zero", commands.raw_motors(next_seq(), 0, 0, 0, 0), verbose)
    write_packet(ser, "drive_stop", commands.stop(next_seq()), verbose)


def read_encoders(ser: serial.Serial, commands: RVRCommands, next_seq, verbose: bool, name: str):
    packet = request(ser, name, commands.get_encoder_counts(next_seq()), verbose)
    return responses.parse_encoder_counts(packet.payload)


def wait_for_stable_encoders(
    ser: serial.Serial,
    commands: RVRCommands,
    next_seq,
    verbose: bool,
    settle: float,
    threshold: int = 25,
    attempts: int = 5,
) -> None:
    previous = read_encoders(ser, commands, next_seq, verbose, "stability_before")
    for _ in range(attempts):
        time.sleep(settle)
        current = read_encoders(ser, commands, next_seq, verbose, "stability_after")
        drift_left = current.left - previous.left
        drift_right = current.right - previous.right
        if abs(drift_left) <= threshold and abs(drift_right) <= threshold:
            return
        stop_all(ser, commands, next_seq, verbose)
        previous = current


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--angular", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=0.35)
    parser.add_argument("--settle", type=float, default=0.35)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument(
        "--only",
        choices=["angular_positive", "angular_negative", "straight_forward_reference"],
        help="run only one velocity pulse; useful when visual observation/reset is needed between tests",
    )
    args = parser.parse_args()

    commands = RVRCommands()
    seq = 1

    def next_seq() -> int:
        nonlocal seq
        value = seq
        seq = (seq + 1) % 256
        return value

    pulses = [
        VelocityPulse("angular_positive", 0.0, abs(args.angular)),
        VelocityPulse("angular_negative", 0.0, -abs(args.angular)),
        VelocityPulse("straight_forward_reference", min(abs(args.angular), 0.5), 0.0),
    ]

    if args.only:
        pulses = [pulse for pulse in pulses if pulse.name == args.only]

    with serial.Serial(args.port, 115200, timeout=0.1) as ser:
        print(f"opened {args.port} @ 115200")
        print(f"velocity pulses: angular=±{abs(args.angular):.3f} duration={args.duration:.2f}s settle={args.settle:.2f}s")
        drain(ser, 0.5)

        write_packet(ser, "wake", commands.wake(next_seq()), args.verbose)
        time.sleep(1.0)
        drain(ser, 0.5)

        battery = request(ser, "battery_percentage", commands.get_battery_percentage(next_seq()), args.verbose)
        print(f"battery_percentage={responses.parse_battery_percentage(battery.payload)}")
        try:
            voltage = request(ser, "battery_voltage", commands.get_battery_voltage(next_seq()), args.verbose)
            print(f"battery_voltage={responses.parse_battery_voltage(voltage.payload):.3f}V")
        except Exception as exc:
            print(f"battery_voltage=UNAVAILABLE ({exc})")

        write_packet(ser, "set_all_leds_steering_purple", commands.set_all_leds(next_seq(), 32, 0, 32), args.verbose)
        stop_all(ser, commands, next_seq, args.verbose)
        time.sleep(args.settle)
        drain(ser, 0.5)

        print("\nname,linear_mps,angular_rad_s,raw_payload,before_left,before_right,after_left,after_right,delta_left,delta_right")
        for pulse in pulses:
            packet = commands.drive_rc(next_seq(), pulse.linear_mps, pulse.angular_rad_s)
            before = read_encoders(ser, commands, next_seq, args.verbose, f"{pulse.name}_encoders_before")

            write_packet(ser, pulse.name, packet, args.verbose)
            time.sleep(args.duration)
            stop_all(ser, commands, next_seq, args.verbose)
            time.sleep(args.settle)

            after = read_encoders(ser, commands, next_seq, args.verbose, f"{pulse.name}_encoders_after")
            print(
                f"{pulse.name},{pulse.linear_mps:.3f},{pulse.angular_rad_s:.3f},{packet.payload.hex()},"
                f"{before.left},{before.right},{after.left},{after.right},{after.left - before.left},{after.right - before.right}"
            )
            drain(ser, 0.25)

        stop_all(ser, commands, next_seq, args.verbose)
        write_packet(ser, "set_all_leds_done_green", commands.set_all_leds(next_seq(), 0, 32, 0), args.verbose)
        time.sleep(0.25)
        drain(ser, 0.5)

    print("steering smoke complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
