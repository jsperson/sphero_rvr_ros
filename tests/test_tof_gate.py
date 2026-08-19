"""D59's falsifier and the counter gate's shape, replayed from the recording we own.

The fixture is every /tof/state line from bag_bench_20260819_122918 -- the
sitting the one-sample gate stopped. The claims, both directions: the OLD
method (read the 5 s rate_hz field once) fails on real dip instants of this
recording, and the NEW method (frames-counter delta over >=10 s of sensor
clock) passes from EVERY possible starting instant of the same recording. A
gate whose verdict depends on when you happened to ask is a die; this file is
where that die is retired.
"""

import json
from pathlib import Path

from sphero_rvr_core.tof_gate import MIN_COUNTER_SPAN_S, tof_gate_verdict

BAND_HZ = 6.5  # mirrors scripts/bringup_gates.py TOF_RATE_MIN_HZ; drift-pinned below

ROWS = json.loads(
    (Path(__file__).resolve().parent / "fixtures" /
     "tof_state_bench_2026-08-19.json").read_text())["rows"]
SAMPLES = [(r["frames"], r["uptime_s"]) for r in ROWS]


def test_the_fixture_is_the_sitting_and_carries_the_dips():
    """187 lines, and the dips that stopped the sitting are IN here -- a fixture
    without the known-bad shape could not falsify anything."""
    assert len(ROWS) == 187
    dips = [r for r in ROWS if r["rate_hz"] < BAND_HZ]
    assert len(dips) == 16, "the recording's sub-band self-reports went missing"
    assert min(r["rate_hz"] for r in ROWS) < 6.0  # startup fill reaches 5.91


def test_the_one_sample_method_fails_on_real_instants_of_this_recording():
    """THE FALSIFIER: the old gate read rate_hz once. On 16 of 187 instants of
    the actual sitting it fails a healthy sensor -- including mid-recording
    dips (not just startup fill), which is exactly what happened live (6.30,
    then 6.43 on the re-probe)."""
    failing = [r for r in ROWS if r["rate_hz"] < BAND_HZ]
    assert failing, ("the old method passes everywhere -- this fixture can no "
                     "longer reproduce D59 and certifies nothing")
    mid = [r for r in failing if r["uptime_s"] > 30.0]
    assert mid, ("only startup instants fail -- a warmup allowance would have "
                 "fixed the old gate, and the counter method is over-engineering")


def test_the_counter_method_passes_from_every_starting_instant():
    """The fix's whole claim: no matter WHEN the probe starts sampling this
    recording -- including inside every dip that fooled the old gate -- the
    frames-counter verdict is PASS, with rate at or above the band."""
    for start in range(len(SAMPLES) - 1):
        ok, rate = tof_gate_verdict(SAMPLES[start:], BAND_HZ)
        if ok is None:
            # ran off the end of the recording before spanning 10 s -- only
            # legitimate for starts within the last window of the bag
            assert SAMPLES[-1][1] - SAMPLES[start][1] < MIN_COUNTER_SPAN_S
            continue
        assert ok, (f"counter gate failed starting at row {start} "
                    f"(uptime {SAMPLES[start][1]}): {rate} Hz")
        assert rate >= BAND_HZ


def test_a_restarted_producer_rebaselines_instead_of_counting_backwards():
    """frames going backwards is a new producer, not negative frames (the
    StallEventTracker rule). The verdict must re-baseline and answer from the
    new counter's own span."""
    samples = [(1000, 100.0), (1005, 101.0), (3, 1.0)] + [
        (3 + 7 * i, 1.0 + i) for i in range(1, 13)]
    ok, rate = tof_gate_verdict(samples, BAND_HZ)
    assert ok is True
    assert 6.5 <= rate <= 7.5


def test_not_enough_span_says_so_instead_of_guessing():
    ok, rate = tof_gate_verdict([(7, 1.0), (35, 5.0)], BAND_HZ)
    assert (ok, rate) == (None, None)


def test_a_genuinely_slow_sensor_still_fails():
    """The band did not move and a sick sensor cannot hide in the new
    arithmetic: 5.5 Hz sustained over the span fails."""
    samples = [(int(5.5 * i), float(i)) for i in range(0, 15)]
    ok, rate = tof_gate_verdict(samples, BAND_HZ)
    assert ok is False
    assert rate < BAND_HZ


def test_the_band_here_mirrors_the_probes():
    """Drift pin: this file's BAND_HZ is a mirror of the probe's
    TOF_RATE_MIN_HZ (whose derivation comment and 6.5 value are already pinned
    by test_launch_and_arm). CHANGE BOTH OR NEITHER."""
    probe_src = (Path(__file__).resolve().parents[1] / "scripts" /
                 "bringup_gates.py").read_text()
    assert f"TOF_RATE_MIN_HZ = {BAND_HZ}" in probe_src
    assert "tof_gate_verdict(samples, TOF_RATE_MIN_HZ)" in probe_src, (
        "the probe no longer gates through the counter verdict")
