#!/usr/bin/env python3
"""Suspended Sphero RVR raw-motor encoder mapping test.

Runs gated, short raw-motor pulses and records encoder deltas so we can map
left/right motor payloads and forward/reverse sign conventions before trusting
ROS /cmd_vel semantics.

Only run this with the RVR suspended or safely restrained.
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
class Pulse:
    name: str
    left_mode: int
    left_speed: int
    right_mode: int
    right_speed: int


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


def drain(ser: serial.Serial, seconds: float = 0.25) -> list[Packet]:
    seen: list[Packet] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = read_frame(ser, timeout=max(0.01, deadline - time.monotonic()))
        if raw is None:
            break
        try:
            seen.append(Packet.decode(raw))
        except Exception as exc:
            print(f"RX decode error: {exc} raw={raw.hex()}")
    return seen


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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--speed", type=int, default=128)
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--settle", type=float, default=0.35)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    speed = max(0, min(255, args.speed))
    commands = RVRCommands()
    seq = 1

    def next_seq() -> int:
        nonlocal seq
        value = seq
        seq = (seq + 1) % 256
        return value

    pulses = [
        Pulse("left_forward", 1, speed, 0, 0),
        Pulse("left_reverse", 2, speed, 0, 0),
        Pulse("right_forward", 0, 0, 1, speed),
        Pulse("right_reverse", 0, 0, 2, speed),
        Pulse("both_forward", 1, speed, 1, speed),
        Pulse("both_reverse", 2, speed, 2, speed),
    ]

    with serial.Serial(args.port, 115200, timeout=0.1) as ser:
        print(f"opened {args.port} @ 115200")
        print(f"mapping pulses: speed={speed} duration={args.duration:.2f}s settle={args.settle:.2f}s")
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

        # Bright blue while mapping; visible signal that movement tests are active.
        write_packet(ser, "set_all_leds_mapping_blue", commands.set_all_leds(next_seq(), 0, 0, 48), args.verbose)
        write_packet(ser, "pre_stop_drive", commands.stop(next_seq()), args.verbose)
        write_packet(ser, "pre_raw_zero", commands.raw_motors(next_seq(), 0, 0, 0, 0), args.verbose)
        time.sleep(args.settle)
        drain(ser, 0.5)

        print("\nname,left_mode,left_speed,right_mode,right_speed,before_left,before_right,after_left,after_right,delta_left,delta_right")
        for pulse in pulses:
            before_packet = request(ser, f"{pulse.name}_encoders_before", commands.get_encoder_counts(next_seq()), args.verbose)
            before = responses.parse_encoder_counts(before_packet.payload)

            write_packet(
                ser,
                pulse.name,
                commands.raw_motors(next_seq(), pulse.left_mode, pulse.left_speed, pulse.right_mode, pulse.right_speed),
                args.verbose,
            )
            time.sleep(args.duration)
            write_packet(ser, f"{pulse.name}_raw_zero", commands.raw_motors(next_seq(), 0, 0, 0, 0), args.verbose)
            write_packet(ser, f"{pulse.name}_stop", commands.stop(next_seq()), args.verbose)
            time.sleep(args.settle)

            after_packet = request(ser, f"{pulse.name}_encoders_after", commands.get_encoder_counts(next_seq()), args.verbose)
            after = responses.parse_encoder_counts(after_packet.payload)
            delta_left = after.left - before.left
            delta_right = after.right - before.right
            print(
                f"{pulse.name},{pulse.left_mode},{pulse.left_speed},{pulse.right_mode},{pulse.right_speed},"
                f"{before.left},{before.right},{after.left},{after.right},{delta_left},{delta_right}"
            )
            drain(ser, 0.25)

        write_packet(ser, "final_raw_zero", commands.raw_motors(next_seq(), 0, 0, 0, 0), args.verbose)
        write_packet(ser, "final_stop", commands.stop(next_seq()), args.verbose)
        write_packet(ser, "set_all_leds_done_green", commands.set_all_leds(next_seq(), 0, 32, 0), args.verbose)
        time.sleep(0.25)
        drain(ser, 0.5)

    print("encoder mapping complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
