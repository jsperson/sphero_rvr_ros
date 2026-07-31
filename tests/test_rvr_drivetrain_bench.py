import ast
import csv
import importlib.util
import json
from pathlib import Path
import struct
import sys

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "rvr_drivetrain_bench.py"
SPEC = importlib.util.spec_from_file_location("rvr_drivetrain_bench", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
bench = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = bench
SPEC.loader.exec_module(bench)


def _samples(values):
    return tuple(
        bench.EncoderSample(index * 0.1, left, right)
        for index, (left, right) in enumerate(values)
    )


def _record(commanded, *, sustained=False, smooth=False):
    return bench.PulseRecord(
        mode="bench",
        representation="raw_duty",
        axis="forward",
        commanded=commanded,
        pulse_duration_s=0.7,
        before_left=0,
        before_right=0,
        after_left=40,
        after_right=40,
        left_delta_counts=40,
        right_delta_counts=40,
        measured_displacement_m=0.04,
        measured_yaw_rad=0.0,
        moved_bool=sustained,
        sustained_bool=sustained,
        smooth_bool=smooth,
        active_intervals=4,
        interval_count=4,
        active_fraction=1.0,
        direction_consistency=1.0,
        progress_rate_cv=0.0,
        samples=_samples([(0, 0), (10, 10), (20, 20), (30, 30), (40, 40)]),
    )


def test_generate_sweep_is_zero_based_inclusive_and_float_stable():
    assert bench.generate_sweep(0.03, 0.01) == (0.0, 0.01, 0.02, 0.03)
    assert bench.generate_sweep(10, 4) == (0.0, 4.0, 8.0, 10.0)
    with pytest.raises(ValueError, match="positive"):
        bench.generate_sweep(1, 0)


def test_forward_classification_distinguishes_stall_twitch_and_smooth_motion():
    stalled = bench.classify_pulse(
        "forward", _samples([(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]),
        counts_per_meter=1000.0,
        min_forward_m=0.003,
    )
    twitch = bench.classify_pulse(
        "forward", _samples([(0, 0), (20, 20), (20, 20), (20, 20), (20, 20)]),
        counts_per_meter=1000.0,
        min_forward_m=0.003,
    )
    smooth = bench.classify_pulse(
        "forward", _samples([(0, 0), (10, 10), (20, 20), (30, 30), (40, 40)]),
        counts_per_meter=1000.0,
        min_forward_m=0.003,
    )

    assert (stalled.moved, stalled.sustained, stalled.smooth) == (
        False,
        False,
        False,
    )
    assert (twitch.moved, twitch.sustained, twitch.smooth) == (True, False, False)
    assert (smooth.moved, smooth.sustained, smooth.smooth) == (True, True, True)


def test_turn_classification_uses_opposed_track_progress():
    result = bench.classify_pulse(
        "turn",
        _samples([(0, 0), (-10, 10), (-20, 20), (-30, 30), (-40, 40)]),
        counts_per_meter=1000.0,
        track_width_m=0.25,
        min_turn_rad=0.03,
    )
    assert result.sustained is True
    assert result.smooth is True


def test_direct_packets_encode_forward_and_pure_turn_for_both_representations():
    session = object.__new__(bench.DirectRVR)
    session.commands = bench.RVRCommands()
    session._sequence = 0
    packets = []
    session._write = lambda _name, packet: packets.append(packet)

    session.command("raw_duty", "forward", 12)
    session.command("raw_duty", "turn", 12)
    session.command("tank_si", "forward", 0.1)
    session.command("tank_si", "turn", 0.1)

    assert packets[0].payload == bytes((1, 12, 1, 12))
    assert packets[1].payload == bytes((2, 12, 1, 12))
    assert struct.unpack(">ff", packets[2].payload) == pytest.approx((0.1, 0.1))
    assert struct.unpack(">ff", packets[3].payload) == pytest.approx((-0.1, 0.1))


def test_direct_zero_emits_raw_off_and_tank_si_zero():
    session = object.__new__(bench.DirectRVR)
    session.commands = bench.RVRCommands()
    session._sequence = 0
    packets = []
    session._write = lambda _name, packet: packets.append(packet)

    session.zero()

    assert packets[0].payload == bytes((0, 0, 0, 0))
    assert struct.unpack(">ff", packets[1].payload) == pytest.approx((0.0, 0.0))


def test_summary_reports_sustained_breakaway_and_next_smooth_command():
    summary = bench.summarize_thresholds(
        [
            _record(0),
            _record(4),
            _record(8, sustained=True, smooth=True),
            _record(12, sustained=True, smooth=True),
        ]
    )["raw_duty:forward"]
    assert summary["breakaway_command"] == 8
    assert summary["smooth_command_above_breakaway"] == 12
    assert summary["command_unit"] == "duty_0_255"


def test_output_writes_simple_csv_and_detailed_json(tmp_path):
    csv_path, json_path = bench.write_outputs(
        tmp_path / "bench-result", {"mode": "bench"}, [_record(8, sustained=True)]
    )

    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    report = json.loads(json_path.read_text(encoding="utf-8"))

    assert rows[0]["commanded"] == "8"
    assert rows[0]["moved_bool"] == "True"
    assert report["schema"] == "sphero_rvr.drivetrain_bench.v1"
    assert report["records"][0]["samples"][-1]["left"] == 40
    assert report["summary"]["raw_duty:forward"]["breakaway_command"] == 8


def test_hard_ceilings_reject_oversized_pulses_and_commands():
    base = dict(
        mode="bench",
        port="/dev/null",
        baud=115200,
        representations=("raw_duty",),
        axes=("forward",),
        pulse_duration_s=0.7,
        settle_s=0.5,
        sample_period_s=0.1,
        raw_max_duty=96,
        raw_step=4,
        si_max_mps=0.30,
        si_step_mps=0.01,
        counts_per_meter=4337.768,
        track_width_m=0.2507,
        min_forward_m=0.003,
        min_turn_rad=0.03,
    )
    with pytest.raises(ValueError, match="pulse duration"):
        bench.validate_config(bench.BenchConfig(**{**base, "pulse_duration_s": 0.76}))
    with pytest.raises(ValueError, match="raw duty"):
        bench.validate_config(bench.BenchConfig(**{**base, "raw_max_duty": 97}))
    with pytest.raises(ValueError, match="SI velocity"):
        bench.validate_config(bench.BenchConfig(**{**base, "si_max_mps": 0.31}))
    with pytest.raises(ValueError, match="three measured"):
        bench.validate_config(
            bench.BenchConfig(**{**base, "sample_period_s": 0.25})
        )


def test_motion_requires_presence_and_floor_requires_clear_area():
    with pytest.raises(SystemExit):
        bench.main(["--bench"])
    with pytest.raises(SystemExit):
        bench.main(["--floor", "--i-am-present"])


def test_script_imports_no_ros_or_mission_stack_modules():
    tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
    assert "rclpy" not in imported
    assert not any(name.startswith("sphero_rvr_driver") for name in imported)
