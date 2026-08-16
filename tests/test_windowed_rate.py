"""`rate_hz` must report the CURRENT rate, not an average since boot.

The old field was `frames / (now - node_start)`. It cannot report the live rate, and
it reads below band for minutes after startup -- so a genuinely slow sensor and a
healthy one warming up are indistinguishable, while the run card gates on that
field. On 2026-08-15 it published 5.87 while `/tof/points` was genuinely running at
7.06 Hz.

Standards rule 6: a gate must measure the quantity it claims to.
"""

import pytest

from sphero_rvr_core.tof_frame import windowed_rate_hz


def _stamps(start, count, period):
    return [start + i * period for i in range(count)]


def test_a_steady_sensor_reads_its_actual_rate():
    period = 1.0 / 7.0
    stamps = _stamps(100.0, 35, period)
    now = stamps[-1]
    assert windowed_rate_hz(stamps, now, 5.0) == pytest.approx(7.0, rel=1e-6)


def test_the_startup_transient_that_made_the_old_field_lie():
    """THE REGRESSION THIS FIELD EXISTS FOR, as arithmetic.

    A sensor healthy at 7 Hz, 3 s after node start, with the node having spent 30 s
    coming up before the first frame. The cumulative average is dragged to ~1.9 Hz by
    the dead time and reads as a failing sensor; the windowed rate says 7.
    """
    node_start = 0.0
    first_frame = 30.0
    period = 1.0 / 7.0
    stamps = _stamps(first_frame, 22, period)      # ~3 s of frames
    now = stamps[-1]

    cumulative = len(stamps) / (now - node_start)
    windowed = windowed_rate_hz(stamps, now, 5.0)

    assert cumulative < 1.0, f"the old field would read {cumulative:.2f}"
    assert windowed == pytest.approx(7.0, rel=1e-6)
    assert windowed > 5.0 > cumulative, (
        "this is the whole defect: one number fails a 5 Hz band and the other passes, "
        "for the same healthy sensor"
    )


def test_a_sensor_that_STOPPED_reports_unknown_rather_than_its_old_rate():
    """The mirror failure, and the more dangerous one. A cumulative average keeps
    quoting a healthy-looking number long after frames stop arriving, because the
    numerator is frozen and the denominator grows slowly. The window empties."""
    node_start = 95.0
    stamps = _stamps(100.0, 40, 1.0 / 7.0)
    now = stamps[-1] + 30.0

    assert windowed_rate_hz(stamps, now, 5.0) is None, (
        "30 s after the last frame the window is empty and the only honest answer "
        "is that the rate is unknown"
    )
    # The cumulative form keeps publishing a finite, ordinary-looking number here --
    # its numerator is frozen and its denominator only creeps. The defect is not that
    # the value is high or low, it is that it EXISTS: a dead sensor goes on reporting
    # a rate, and nothing downstream can tell that from a slow one.
    cumulative = len(stamps) / (now - node_start)
    assert cumulative == pytest.approx(0.986, abs=0.01)


def test_a_genuinely_slow_sensor_is_reported_slow():
    stamps = _stamps(100.0, 6, 1.0)      # 1 Hz
    assert windowed_rate_hz(stamps, stamps[-1], 5.0) == pytest.approx(1.0, rel=1e-6)


def test_one_frame_in_the_window_is_UNKNOWN_not_a_manufactured_rate():
    """Rate is measured over INTERVALS. One stamp describes none, and dividing by the
    window length instead would invent a plausible low rate out of no evidence --
    exactly when the reader most needs to be told nothing is known."""
    assert windowed_rate_hz([100.0], 100.0, 5.0) is None
    assert windowed_rate_hz([], 100.0, 5.0) is None


def test_a_partly_filled_window_reports_the_real_rate_not_a_fraction_of_it():
    """0.4 s into a 5 s window, a 7 Hz sensor is at 7 Hz. Dividing the count by the
    WINDOW rather than by the observed span would report ~0.6 Hz and re-create the
    startup lie in a new place."""
    stamps = _stamps(100.0, 4, 1.0 / 7.0)
    assert windowed_rate_hz(stamps, stamps[-1], 5.0) == pytest.approx(7.0, rel=1e-6)


def test_stamps_outside_the_window_do_not_drag_the_answer():
    old = _stamps(0.0, 50, 1.0)                    # ancient 1 Hz history
    recent = _stamps(100.0, 35, 1.0 / 7.0)         # current 7 Hz
    now = recent[-1]
    assert windowed_rate_hz(old + recent, now, 5.0) == pytest.approx(7.0, rel=1e-6)


def test_degenerate_windows_are_refused_rather_than_divided_by():
    stamps = _stamps(100.0, 10, 0.1)
    assert windowed_rate_hz(stamps, stamps[-1], 0.0) is None
    assert windowed_rate_hz(stamps, stamps[-1], -1.0) is None


def test_it_does_not_mutate_the_callers_deque():
    stamps = _stamps(100.0, 20, 0.1)
    before = list(stamps)
    windowed_rate_hz(stamps, stamps[-1], 0.5)
    assert stamps == before, "pruning is the caller's business; this only reads"


def test_the_pure_core_needs_no_ros():
    """The proofs must bind on a machine with no rclpy, which is why this lives in
    core rather than in the node -- commit 1's lesson, applied up front."""
    import sphero_rvr_core.tof_frame as mod

    assert "rclpy" not in getattr(mod, "__dict__", {})
