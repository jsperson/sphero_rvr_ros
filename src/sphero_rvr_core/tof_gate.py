"""The ToF rate gate's arithmetic: counters-not-levels, applied to our own gate.

D59 (2026-08-19): the bringup probe used to read the sensor's `rate_hz` field
ONCE and compare it to the band. That field is a 5-second window, and 2-3
frames of ordinary jitter swing it 6.9 -> 6.3 — so a single sample against a
hard threshold was a flake machine: it stopped a bench sitting at 6.30/6.43 on
a sensor whose own frames counter never ran below 6.65 Hz in any 10 s span of
the same recording (tests/fixtures/tof_state_bench_2026-08-19.json — the free
falsifier). This morning's passing 6.83 was the same die rolled luckier.

The fix is the sensor's own FRAMES COUNTER across a span long enough that
jitter is noise: exact arithmetic, the owner's clock (`uptime_s`, no wall-clock
pairing — the align-recorder lesson), same band, same staleness derivation.
The band itself (6.5 Hz) lives in scripts/bringup_gates.py with its derivation
and does not move (PM ruling on D59's row). The deeper reframe — gating on max
gap, the quantity the 0.30 s staleness bound actually cares about — is parked
on D59 as future-if-it-recurs.
"""

from __future__ import annotations

#: DERIVED span: at ~7 Hz, +/-1 frame of counting jitter over 10 s is +/-0.1 Hz
#: — an order below the 5 s self-report window's measured +/-0.6 Hz swings, and
#: small against the band's working margin (the D59 recording's worst honest
#: 10 s span read 6.65 vs the 6.5 band).
MIN_COUNTER_SPAN_S = 10.0


def tof_gate_verdict(samples, min_hz, min_span_s=MIN_COUNTER_SPAN_S):
    """Gate verdict from chronological (frames, uptime_s) pairs.

    Returns ``(ok, rate)``: ``(None, None)`` while no pair yet spans
    ``min_span_s`` (keep sampling), else ``(rate >= min_hz, rate)`` for the
    first baseline-to-sample pair that does. A frames or uptime value that goes
    BACKWARDS is a restarted producer, not negative frames — re-baseline and
    keep counting (the marker's StallEventTracker rule, applied here).
    """
    base = None
    for frames, uptime_s in samples:
        if base is None or frames < base[0] or uptime_s <= base[1]:
            base = (int(frames), float(uptime_s))
            continue
        span = float(uptime_s) - base[1]
        if span >= min_span_s:
            rate = (int(frames) - base[0]) / span
            return (rate >= float(min_hz), round(rate, 2))
    return (None, None)
