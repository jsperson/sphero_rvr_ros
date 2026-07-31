#!/usr/bin/env python3
"""Attended, direct-serial Sphero RVR drivetrain characterization.

This diagnostic intentionally bypasses ROS, Nav2, the mission service, the
collision supervisor, and every mission/evidence authority path.  It sends
bounded raw-motor and native tank-SI packets directly to the RVR and measures
onboard encoder progress during each pulse.
"""

from __future__ import annotations

import argparse
import atexit
import csv
import json
import math
import signal
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from decimal import Decimal
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

import serial

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from sphero_rvr_core import responses  # noqa: E402
from sphero_rvr_core.commands import RVRCommands  # noqa: E402
from sphero_rvr_core.packet import EOP, SOP, Packet  # noqa: E402


HARD_MAX_PULSE_S = 0.75
HARD_MAX_RAW_DUTY = 96
HARD_MAX_SI_MPS = 0.30
DEFAULT_PULSE_S = 0.70
DEFAULT_SETTLE_S = 0.50
DEFAULT_SAMPLE_PERIOD_S = 0.10
DEFAULT_COUNTS_PER_METER = 4337.768
DEFAULT_TRACK_WIDTH_M = 0.2507
DEFAULT_MIN_FORWARD_M = 0.003
DEFAULT_MIN_TURN_RAD = 0.03
MIN_SUSTAINED_ACTIVE_INTERVALS = 3
MIN_SUSTAINED_ACTIVE_FRACTION = 0.60
MIN_SMOOTH_ACTIVE_FRACTION = 0.80
MAX_SMOOTH_RATE_CV = 0.45
FORWARD_MODE = 1
REVERSE_MODE = 2
OFF_MODE = 0


class BenchError(RuntimeError):
    """Base failure for the direct drivetrain bench."""


class ZeroDeliveryError(BenchError):
    """The script could not write its final zero commands."""


@dataclass(frozen=True)
class EncoderSample:
    elapsed_s: float
    left: int
    right: int


@dataclass(frozen=True)
class PulseClassification:
    moved: bool
    sustained: bool
    smooth: bool
    active_intervals: int
    interval_count: int
    active_fraction: float
    direction_consistency: float
    progress_rate_cv: Optional[float]


@dataclass(frozen=True)
class PulseRecord:
    mode: str
    representation: str
    axis: str
    commanded: float
    pulse_duration_s: float
    before_left: int
    before_right: int
    after_left: int
    after_right: int
    left_delta_counts: int
    right_delta_counts: int
    measured_displacement_m: float
    measured_yaw_rad: float
    moved_bool: bool
    sustained_bool: bool
    smooth_bool: bool
    active_intervals: int
    interval_count: int
    active_fraction: float
    direction_consistency: float
    progress_rate_cv: Optional[float]
    samples: tuple[EncoderSample, ...]


@dataclass(frozen=True)
class BenchConfig:
    mode: str
    port: str
    baud: int
    representations: tuple[str, ...]
    axes: tuple[str, ...]
    pulse_duration_s: float
    settle_s: float
    sample_period_s: float
    raw_max_duty: int
    raw_step: int
    si_max_mps: float
    si_step_mps: float
    counts_per_meter: float
    track_width_m: float
    min_forward_m: float
    min_turn_rad: float


def generate_sweep(stop: float, step: float) -> tuple[float, ...]:
    """Return an inclusive zero-upward sweep without binary-float drift."""

    stop_value = Decimal(str(stop))
    step_value = Decimal(str(step))
    if not stop_value.is_finite() or stop_value < 0:
        raise ValueError("sweep stop must be finite and nonnegative")
    if not step_value.is_finite() or step_value <= 0:
        raise ValueError("sweep step must be finite and positive")
    values = []
    current = Decimal("0")
    while current <= stop_value:
        values.append(float(current))
        current += step_value
    if Decimal(str(values[-1])) != stop_value:
        values.append(float(stop_value))
    return tuple(values)


def _axis_progress(
    axis: str,
    left_delta: int,
    right_delta: int,
    *,
    counts_per_meter: float,
    track_width_m: float,
) -> float:
    left_m = left_delta / counts_per_meter
    right_m = right_delta / counts_per_meter
    if axis == "forward":
        return (left_m + right_m) / 2.0
    if axis == "turn":
        return (right_m - left_m) / track_width_m
    raise ValueError(f"unsupported axis: {axis}")


def pulse_measurements(
    before_left: int,
    before_right: int,
    after_left: int,
    after_right: int,
    *,
    counts_per_meter: float,
    track_width_m: float,
) -> tuple[float, float]:
    left_m = (after_left - before_left) / counts_per_meter
    right_m = (after_right - before_right) / counts_per_meter
    return (left_m + right_m) / 2.0, (right_m - left_m) / track_width_m


def classify_pulse(
    axis: str,
    samples: Sequence[EncoderSample],
    *,
    counts_per_meter: float = DEFAULT_COUNTS_PER_METER,
    track_width_m: float = DEFAULT_TRACK_WIDTH_M,
    min_forward_m: float = DEFAULT_MIN_FORWARD_M,
    min_turn_rad: float = DEFAULT_MIN_TURN_RAD,
) -> PulseClassification:
    """Classify motion from encoder progress sampled while command is active."""

    if len(samples) < 2:
        return PulseClassification(False, False, False, 0, 0, 0.0, 0.0, None)
    if counts_per_meter <= 0 or track_width_m <= 0:
        raise ValueError("odometry calibration values must be positive")

    threshold = min_forward_m if axis == "forward" else min_turn_rad
    total_progress = _axis_progress(
        axis,
        samples[-1].left - samples[0].left,
        samples[-1].right - samples[0].right,
        counts_per_meter=counts_per_meter,
        track_width_m=track_width_m,
    )
    rates: list[float] = []
    directional_intervals = 0
    active_rates: list[float] = []
    interval_noise = threshold / max(4, len(samples) - 1)
    for previous, current in zip(samples, samples[1:]):
        elapsed = current.elapsed_s - previous.elapsed_s
        if elapsed <= 0 or not math.isfinite(elapsed):
            raise ValueError("encoder sample times must increase")
        progress = _axis_progress(
            axis,
            current.left - previous.left,
            current.right - previous.right,
            counts_per_meter=counts_per_meter,
            track_width_m=track_width_m,
        )
        rate = progress / elapsed
        rates.append(rate)
        if progress > interval_noise:
            directional_intervals += 1
            active_rates.append(rate)

    interval_count = len(rates)
    active_fraction = directional_intervals / interval_count
    direction_consistency = sum(rate >= 0.0 for rate in rates) / interval_count
    moved = total_progress >= threshold
    sustained = bool(
        moved
        and directional_intervals >= MIN_SUSTAINED_ACTIVE_INTERVALS
        and active_fraction >= MIN_SUSTAINED_ACTIVE_FRACTION
        and direction_consistency >= 0.80
    )
    rate_cv: Optional[float] = None
    if len(active_rates) >= 2:
        mean_rate = statistics.fmean(active_rates)
        if mean_rate > 0:
            rate_cv = statistics.pstdev(active_rates) / mean_rate
    smooth = bool(
        sustained
        and active_fraction >= MIN_SMOOTH_ACTIVE_FRACTION
        and direction_consistency == 1.0
        and rate_cv is not None
        and rate_cv <= MAX_SMOOTH_RATE_CV
    )
    return PulseClassification(
        moved=moved,
        sustained=sustained,
        smooth=smooth,
        active_intervals=directional_intervals,
        interval_count=interval_count,
        active_fraction=active_fraction,
        direction_consistency=direction_consistency,
        progress_rate_cv=rate_cv,
    )


def summarize_thresholds(records: Sequence[PulseRecord]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    groups = sorted({(item.representation, item.axis) for item in records})
    for representation, axis in groups:
        group = sorted(
            (
                item
                for item in records
                if item.representation == representation and item.axis == axis
            ),
            key=lambda item: item.commanded,
        )
        breakaway = next((item for item in group if item.sustained_bool), None)
        smooth = next(
            (
                item
                for item in group
                if item.smooth_bool
                and breakaway is not None
                and item.commanded > breakaway.commanded
            ),
            None,
        )
        summary[f"{representation}:{axis}"] = {
            "command_unit": (
                "duty_0_255" if representation == "raw_duty" else "track_mps"
            ),
            "breakaway_command": (
                breakaway.commanded if breakaway is not None else None
            ),
            "smooth_command_above_breakaway": (
                smooth.commanded if smooth is not None else None
            ),
            "tested_max": group[-1].commanded if group else None,
        }
    return summary


CSV_FIELDS = (
    "mode",
    "representation",
    "axis",
    "commanded",
    "pulse_duration_s",
    "before_left",
    "before_right",
    "after_left",
    "after_right",
    "left_delta_counts",
    "right_delta_counts",
    "measured_displacement_m",
    "measured_yaw_rad",
    "moved_bool",
    "sustained_bool",
    "smooth_bool",
    "active_intervals",
    "interval_count",
    "active_fraction",
    "direction_consistency",
    "progress_rate_cv",
)


def _record_json(record: PulseRecord) -> dict[str, Any]:
    value = asdict(record)
    value["samples"] = [asdict(sample) for sample in record.samples]
    return value


def write_outputs(
    prefix: Path,
    config: Mapping[str, Any],
    records: Sequence[PulseRecord],
) -> tuple[Path, Path]:
    """Write the raw data and current summary after every completed pulse."""

    prefix.parent.mkdir(parents=True, exist_ok=True)
    csv_path = prefix.with_suffix(".csv")
    json_path = prefix.with_suffix(".json")
    csv_temp = csv_path.with_name(csv_path.name + ".tmp")
    json_temp = json_path.with_name(json_path.name + ".tmp")
    with csv_temp.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for record in records:
            row = asdict(record)
            row.pop("samples")
            writer.writerow(row)
    csv_temp.replace(csv_path)
    report = {
        "schema": "sphero_rvr.drivetrain_bench.v1",
        "created_at_epoch_s": time.time(),
        "config": dict(config),
        "records": [_record_json(record) for record in records],
        "summary": summarize_thresholds(records),
    }
    json_temp.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    json_temp.replace(json_path)
    return csv_path, json_path


def validate_config(config: BenchConfig) -> None:
    if config.mode not in {"bench", "floor"}:
        raise ValueError("mode must be bench or floor")
    if not 0.0 < config.pulse_duration_s <= HARD_MAX_PULSE_S:
        raise ValueError(f"pulse duration must be <= {HARD_MAX_PULSE_S:.1f} s")
    if config.settle_s < 0.25:
        raise ValueError("settle time must be at least 0.25 s")
    if not 0.05 <= config.sample_period_s <= 0.25:
        raise ValueError("sample period must be between 0.05 and 0.25 s")
    if (
        config.pulse_duration_s
        < MIN_SUSTAINED_ACTIVE_INTERVALS * config.sample_period_s + 0.08
    ):
        raise ValueError(
            "pulse duration is too short for three measured progress intervals"
        )
    if not 0 <= config.raw_max_duty <= HARD_MAX_RAW_DUTY:
        raise ValueError(f"raw duty may not exceed {HARD_MAX_RAW_DUTY}")
    if not 1 <= config.raw_step <= 16:
        raise ValueError("raw duty step must be between 1 and 16")
    if not 0.0 <= config.si_max_mps <= HARD_MAX_SI_MPS:
        raise ValueError(f"SI velocity may not exceed {HARD_MAX_SI_MPS:.2f} m/s")
    if not 0.0 < config.si_step_mps <= 0.05:
        raise ValueError("SI velocity step must be in (0, 0.05] m/s")
    if config.counts_per_meter <= 0 or config.track_width_m <= 0:
        raise ValueError("odometry calibration values must be positive")
    if config.min_forward_m <= 0 or config.min_turn_rad <= 0:
        raise ValueError("movement thresholds must be positive")


def _read_frame(port: serial.Serial, timeout_s: float) -> Optional[bytes]:
    deadline = time.monotonic() + timeout_s
    in_frame = False
    frame = bytearray()
    while time.monotonic() < deadline:
        value = port.read(1)
        if value == b"":
            continue
        byte = value[0]
        if not in_frame:
            if byte == SOP:
                in_frame = True
                frame.append(byte)
            continue
        frame.append(byte)
        if byte == EOP:
            return bytes(frame)
    return None


class DirectRVR:
    """Minimal direct serial session; no ROS or mission-stack ownership."""

    def __init__(self, port: str, baud: int, *, verbose: bool = False):
        self.commands = RVRCommands()
        self.verbose = verbose
        self._sequence = 0
        self._port = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.04,
            write_timeout=0.20,
            exclusive=True,
        )

    def close(self) -> None:
        self._port.close()

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) % 256
        return self._sequence

    def _write(self, name: str, packet: Packet) -> None:
        encoded = packet.encode()
        if self.verbose:
            print(
                f"TX {name}: did=0x{packet.device_id:02x} "
                f"cid=0x{packet.command_id:02x} seq={packet.sequence_id} "
                f"payload={packet.payload.hex()}"
            )
        written = self._port.write(encoded)
        self._port.flush()
        if written != len(encoded):
            raise OSError(f"short serial write for {name}: {written}/{len(encoded)}")

    def _request(self, name: str, packet: Packet, timeout_s: float) -> Packet:
        self._write(name, packet)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = _read_frame(self._port, max(0.01, deadline - time.monotonic()))
            if raw is None:
                break
            try:
                response = Packet.decode(raw)
            except Exception:
                continue
            if (
                response.device_id,
                response.command_id,
                response.sequence_id,
            ) != (packet.device_id, packet.command_id, packet.sequence_id):
                continue
            if response.error not in (None, 0):
                raise BenchError(f"{name} returned protocol error {response.error}")
            return response
        raise TimeoutError(f"timed out waiting for {name}")

    def wake(self) -> None:
        self._write("wake", self.commands.wake(self._next_sequence()))

    def battery_percentage(self) -> int:
        packet = self._request(
            "battery_percentage",
            self.commands.get_battery_percentage(self._next_sequence()),
            2.0,
        )
        return responses.parse_battery_percentage(packet.payload)

    def encoders(self, timeout_s: float = 0.20) -> responses.EncoderCounts:
        packet = self._request(
            "encoder_counts",
            self.commands.get_encoder_counts(self._next_sequence()),
            timeout_s,
        )
        return responses.parse_encoder_counts(packet.payload)

    def command(self, representation: str, axis: str, magnitude: float) -> None:
        sequence = self._next_sequence()
        if representation == "raw_duty":
            duty = int(round(magnitude))
            if duty > HARD_MAX_RAW_DUTY:
                raise ValueError("raw duty exceeds hard ceiling")
            if duty == 0:
                packet = self.commands.raw_motors(sequence, 0, 0, 0, 0)
            elif axis == "forward":
                packet = self.commands.raw_motors(
                    sequence, FORWARD_MODE, duty, FORWARD_MODE, duty
                )
            else:
                packet = self.commands.raw_motors(
                    sequence, REVERSE_MODE, duty, FORWARD_MODE, duty
                )
        elif representation == "tank_si":
            if magnitude > HARD_MAX_SI_MPS:
                raise ValueError("SI velocity exceeds hard ceiling")
            left = magnitude if axis == "forward" else -magnitude
            packet = self.commands.drive_tank_si_units(sequence, left, magnitude)
        else:
            raise ValueError(f"unsupported representation: {representation}")
        self._write(f"{representation}_{axis}_{magnitude}", packet)

    def zero(self, *, attempts: int = 2) -> None:
        errors: list[BaseException] = []
        for _ in range(max(1, attempts)):
            try:
                self._write(
                    "raw_zero",
                    self.commands.raw_motors(
                        self._next_sequence(), OFF_MODE, 0, OFF_MODE, 0
                    ),
                )
                self._write(
                    "tank_si_zero",
                    self.commands.drive_tank_si_units(
                        self._next_sequence(), 0.0, 0.0
                    ),
                )
                return
            except BaseException as exc:
                errors.append(exc)
        raise ZeroDeliveryError(f"zero delivery failed: {errors[-1]}")


_ACTIVE_SESSION: Optional[DirectRVR] = None


def _emergency_zero() -> bool:
    if _ACTIVE_SESSION is None:
        return True
    try:
        _ACTIVE_SESSION.zero()
        return True
    except BaseException as exc:
        print(
            f"\nCRITICAL: ZERO DELIVERY FAILED ({exc}). CUT RVR POWER NOW.",
            file=sys.stderr,
            flush=True,
        )
        return False


def _signal_handler(signum: int, _frame: Any) -> None:
    print(f"\nSignal {signum} received; commanding zero.", file=sys.stderr)
    _emergency_zero()
    raise KeyboardInterrupt


atexit.register(_emergency_zero)


def _pulse_samples(
    session: DirectRVR,
    representation: str,
    axis: str,
    commanded: float,
    config: BenchConfig,
) -> tuple[responses.EncoderCounts, tuple[EncoderSample, ...]]:
    before = session.encoders()
    samples = [EncoderSample(0.0, before.left, before.right)]
    session.command(representation, axis, commanded)
    started = time.monotonic()
    deadline = started + config.pulse_duration_s
    sample_deadline = deadline - min(0.08, config.pulse_duration_s / 4.0)
    next_sample = started + config.sample_period_s
    try:
        while next_sample <= sample_deadline:
            remaining = sample_deadline - time.monotonic()
            if remaining <= 0:
                break
            wait_s = min(max(0.0, next_sample - time.monotonic()), remaining)
            if wait_s:
                time.sleep(wait_s)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            counts = session.encoders(timeout_s=min(0.18, remaining))
            samples.append(
                EncoderSample(time.monotonic() - started, counts.left, counts.right)
            )
            next_sample += config.sample_period_s
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    finally:
        session.zero()
    return before, tuple(samples)


def run_pulse(
    session: DirectRVR,
    representation: str,
    axis: str,
    commanded: float,
    config: BenchConfig,
) -> PulseRecord:
    session.zero()
    time.sleep(config.settle_s)
    before, samples = _pulse_samples(
        session, representation, axis, commanded, config
    )
    time.sleep(config.settle_s)
    after = session.encoders()
    displacement_m, yaw_rad = pulse_measurements(
        before.left,
        before.right,
        after.left,
        after.right,
        counts_per_meter=config.counts_per_meter,
        track_width_m=config.track_width_m,
    )
    classification = classify_pulse(
        axis,
        samples,
        counts_per_meter=config.counts_per_meter,
        track_width_m=config.track_width_m,
        min_forward_m=config.min_forward_m,
        min_turn_rad=config.min_turn_rad,
    )
    return PulseRecord(
        mode=config.mode,
        representation=representation,
        axis=axis,
        commanded=commanded,
        pulse_duration_s=config.pulse_duration_s,
        before_left=before.left,
        before_right=before.right,
        after_left=after.left,
        after_right=after.right,
        left_delta_counts=after.left - before.left,
        right_delta_counts=after.right - before.right,
        measured_displacement_m=displacement_m,
        measured_yaw_rad=yaw_rad,
        moved_bool=classification.moved,
        sustained_bool=classification.sustained,
        smooth_bool=classification.smooth,
        active_intervals=classification.active_intervals,
        interval_count=classification.interval_count,
        active_fraction=classification.active_fraction,
        direction_consistency=classification.direction_consistency,
        progress_rate_cv=classification.progress_rate_cv,
        samples=samples,
    )


def _config_from_args(args: argparse.Namespace) -> BenchConfig:
    mode = "floor" if args.floor else "bench"
    representations = (
        ("raw_duty", "tank_si")
        if args.representation == "both"
        else (args.representation,)
    )
    axes = ("forward", "turn") if args.axis == "both" else (args.axis,)
    config = BenchConfig(
        mode=mode,
        port=args.port,
        baud=args.baud,
        representations=representations,
        axes=axes,
        pulse_duration_s=args.pulse_duration,
        settle_s=args.settle,
        sample_period_s=args.sample_period,
        raw_max_duty=args.raw_max_duty,
        raw_step=args.raw_step,
        si_max_mps=args.si_max_mps,
        si_step_mps=args.si_step_mps,
        counts_per_meter=args.counts_per_meter,
        track_width_m=args.track_width_m,
        min_forward_m=args.min_forward_m,
        min_turn_rad=args.min_turn_rad,
    )
    validate_config(config)
    return config


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--bench", action="store_true", help="wheels-up unloaded mode (default)"
    )
    mode.add_argument("--floor", action="store_true", help="loaded floor mode")
    parser.add_argument(
        "--i-am-present",
        action="store_true",
        help="required acknowledgement that an operator and power cut are present",
    )
    parser.add_argument(
        "--floor-area-clear",
        action="store_true",
        help="required with --floor: level bounded room, clear area, no drop-offs",
    )
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--representation", choices=("raw_duty", "tank_si", "both"), default="both"
    )
    parser.add_argument("--axis", choices=("forward", "turn", "both"), default="both")
    parser.add_argument("--pulse-duration", type=float, default=DEFAULT_PULSE_S)
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE_S)
    parser.add_argument("--sample-period", type=float, default=DEFAULT_SAMPLE_PERIOD_S)
    parser.add_argument("--raw-max-duty", type=int, default=HARD_MAX_RAW_DUTY)
    parser.add_argument("--raw-step", type=int, default=4)
    parser.add_argument("--si-max-mps", type=float, default=HARD_MAX_SI_MPS)
    parser.add_argument("--si-step-mps", type=float, default=0.01)
    parser.add_argument("--counts-per-meter", type=float, default=DEFAULT_COUNTS_PER_METER)
    parser.add_argument("--track-width-m", type=float, default=DEFAULT_TRACK_WIDTH_M)
    parser.add_argument("--min-forward-m", type=float, default=DEFAULT_MIN_FORWARD_M)
    parser.add_argument("--min-turn-rad", type=float, default=DEFAULT_MIN_TURN_RAD)
    parser.add_argument("--output-prefix", type=Path)
    parser.add_argument("--verbose", action="store_true")
    return parser


def _print_banner(config: BenchConfig) -> None:
    print("=" * 72)
    print("ATTENDED DIRECT DRIVETRAIN BENCH — POWER CUT MUST BE REACHABLE")
    print("No collision supervisor or software mission safety layer is running.")
    if config.mode == "bench":
        print("MODE=BENCH: wheels must be fully off the ground and securely supported.")
    else:
        print("MODE=FLOOR: level bounded room, clear area, and NO DROP-OFFS.")
        print("Stay beside the rover with power reachable for every pulse.")
    print(
        f"Hard ceilings: pulse<={HARD_MAX_PULSE_S:.1f}s, "
        f"raw<={HARD_MAX_RAW_DUTY}/255, SI<={HARD_MAX_SI_MPS:.2f}m/s"
    )
    print("This diagnostic grants no speed or authority to the deployed stack.")
    print("=" * 72)


def _print_summary(records: Sequence[PulseRecord], mode: str) -> None:
    label = "loaded" if mode == "floor" else "unloaded"
    print(f"\n{label.upper()} THRESHOLDS")
    for key, value in summarize_thresholds(records).items():
        print(
            f"{key}: sustained_breakaway={value['breakaway_command']} "
            f"smooth_above_breakaway={value['smooth_command_above_breakaway']} "
            f"tested_max={value['tested_max']} unit={value['command_unit']}"
        )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        config = _config_from_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    if not args.i_am_present:
        parser.error("motion requires --i-am-present")
    if config.mode == "floor" and not args.floor_area_clear:
        parser.error("--floor also requires --floor-area-clear")

    prefix = args.output_prefix
    if prefix is None:
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        prefix = Path(f"rvr-drivetrain-bench-{config.mode}-{stamp}")
    if prefix.suffix:
        parser.error("--output-prefix must not include a file extension")
    if prefix.with_suffix(".csv").exists() or prefix.with_suffix(".json").exists():
        parser.error("output CSV/JSON already exists; choose a new --output-prefix")

    _print_banner(config)
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    records: list[PulseRecord] = []
    csv_path, json_path = write_outputs(prefix, asdict(config), records)

    global _ACTIVE_SESSION
    session: Optional[DirectRVR] = None
    zero_ok = True
    exit_code = 0
    try:
        session = DirectRVR(config.port, config.baud, verbose=args.verbose)
        _ACTIVE_SESSION = session
        session.wake()
        time.sleep(1.0)
        session.zero()
        print(f"battery_percentage={session.battery_percentage()}")

        for representation in config.representations:
            values: Iterable[float]
            if representation == "raw_duty":
                values = generate_sweep(config.raw_max_duty, config.raw_step)
            else:
                values = generate_sweep(config.si_max_mps, config.si_step_mps)
            for axis in config.axes:
                for commanded in values:
                    if config.mode == "floor" and commanded > 0:
                        input(
                            "FLOOR STEP: rover stopped. Reposition if needed, "
                            "verify the path/drop-off boundary, then press Enter. "
                        )
                    print(
                        f"pulse representation={representation} axis={axis} "
                        f"commanded={commanded:g}"
                    )
                    record = run_pulse(
                        session, representation, axis, commanded, config
                    )
                    records.append(record)
                    csv_path, json_path = write_outputs(
                        prefix, asdict(config), records
                    )
                    print(
                        f"  displacement={record.measured_displacement_m:.6f}m "
                        f"yaw={record.measured_yaw_rad:.6f}rad "
                        f"moved={record.moved_bool} sustained={record.sustained_bool} "
                        f"smooth={record.smooth_bool}"
                    )
        _print_summary(records, config.mode)
        print(f"CSV={csv_path}")
        print(f"JSON={json_path}")
    except KeyboardInterrupt:
        print("Bench interrupted; partial CSV/JSON retained.", file=sys.stderr)
        exit_code = 130
    except Exception as exc:
        print(f"BENCH FAILED: {exc}", file=sys.stderr)
        exit_code = 2
    finally:
        if session is not None:
            zero_ok = _emergency_zero()
            try:
                session.close()
            except Exception as exc:
                print(f"serial close failed: {exc}", file=sys.stderr)
            _ACTIVE_SESSION = None
        if not zero_ok:
            print("FINAL STATE UNKNOWN: CUT RVR POWER NOW.", file=sys.stderr)
            exit_code = 3
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
