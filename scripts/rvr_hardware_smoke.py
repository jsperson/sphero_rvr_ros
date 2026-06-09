#!/usr/bin/env python3
"""Low-risk Sphero RVR serial smoke test.

Default sequence has no intentional movement. Pass --move only when the RVR is
suspended or otherwise safely restrained; it runs a moderate raw-motor pulse,
then verifies movement using encoder deltas before sending multiple stop packets.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from sphero_rvr_core.commands import RVRCommands  # noqa: E402
from sphero_rvr_core.packet import EOP, SOP, Packet  # noqa: E402
from sphero_rvr_core import responses  # noqa: E402


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


def drain(ser: serial.Serial, seconds: float = 0.3) -> list[Packet]:
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


def write_packet(ser: serial.Serial, name: str, packet: Packet) -> None:
    encoded = packet.encode()
    print(f"TX {name}: {describe(packet)} frame={encoded.hex()}")
    ser.write(encoded)
    ser.flush()


def request(ser: serial.Serial, name: str, packet: Packet, timeout: float = 2.0) -> Packet:
    write_packet(ser, name, packet)
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
    parser.add_argument("--move", action="store_true", help="run a suspended motor pulse and verify encoder deltas")
    args = parser.parse_args()

    commands = RVRCommands()
    seq = 1

    def next_seq() -> int:
        nonlocal seq
        value = seq
        seq = (seq + 1) % 256
        return value

    with serial.Serial(args.port, 115200, timeout=0.1) as ser:
        print(f"opened {args.port} @ 115200")
        print("draining stale packets...")
        for rx in drain(ser, 0.5):
            print(f"RX stale: {describe(rx)}")

        # Wake/connect. Do not require a response; different firmware builds are
        # inconsistent here, and the battery query below proves liveness.
        write_packet(ser, "wake", commands.wake(next_seq()))
        time.sleep(1.0)
        for rx in drain(ser, 0.5):
            print(f"RX after wake: {describe(rx)}")

        battery = request(ser, "battery_percentage", commands.get_battery_percentage(next_seq()))
        print(f"battery_percentage={responses.parse_battery_percentage(battery.payload)}")

        try:
            voltage = request(ser, "battery_voltage", commands.get_battery_voltage(next_seq()))
            print(f"battery_voltage={responses.parse_battery_voltage(voltage.payload):.3f}V")
        except Exception as exc:
            print(f"battery_voltage=UNAVAILABLE ({exc})")

        # Visible non-motion command.
        write_packet(ser, "set_all_leds_soft_green", commands.set_all_leds(next_seq(), 0, 32, 0))
        time.sleep(0.25)
        for rx in drain(ser, 0.5):
            print(f"RX after led: {describe(rx)}")

        # Always send stops before optional movement.
        write_packet(ser, "stop_drive_with_heading", commands.stop(next_seq()))
        write_packet(ser, "raw_motors_zero", commands.raw_motors(next_seq(), 0, 0, 0, 0))
        time.sleep(0.25)
        for rx in drain(ser, 0.5):
            print(f"RX after pre-stop: {describe(rx)}")

        if args.move:
            encoder_before = request(ser, "encoders_before", commands.get_encoder_counts(next_seq()))
            before = responses.parse_encoder_counts(encoder_before.payload)
            print(f"encoders_before={before}")

            # The SDK examples warn that low duty cycles may not overcome
            # motor friction. This value is intended for suspended testing, not
            # floor teleop.
            print("running suspended motor pulse: raw motors forward speed=128 for 1.0s")
            write_packet(ser, "raw_motors_forward_suspended", commands.raw_motors(next_seq(), 1, 128, 1, 128))
            time.sleep(1.0)
            write_packet(ser, "raw_motors_zero_after_pulse", commands.raw_motors(next_seq(), 0, 0, 0, 0))
            write_packet(ser, "stop_after_pulse", commands.stop(next_seq()))
            time.sleep(0.5)
            encoder_after = request(ser, "encoders_after", commands.get_encoder_counts(next_seq()))
            after = responses.parse_encoder_counts(encoder_after.payload)
            print(f"encoders_after={after}")
            print(f"encoder_delta_left={after.left - before.left} encoder_delta_right={after.right - before.right}")
            for rx in drain(ser, 0.5):
                print(f"RX after move: {describe(rx)}")
        else:
            print("movement pulse skipped; pass --move to run it")

    print("smoke script complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
