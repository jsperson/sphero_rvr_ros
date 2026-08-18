"""The orphan tripwire's pure logic, held against the shape it was built to catch.

The tool itself was falsified and certified on the archived field bags the day it
was written: run 3c's log+bag yields exactly one unowned burst (the goal-3 orphan,
35 msgs, ending at RPP's self-abort) and run 3d yields none. These tests keep the
pure halves -- window parsing and burst clustering -- honest without needing mcap.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "diagnostics"))
try:
    from check_unowned_motion import goal_windows, unowned_bursts
finally:
    sys.path.pop(0)


LOG_3C_SHAPE = """
[bt_navigator-11] [INFO] [1787079123.412468410] [bt_navigator]: Begin navigating from current location (-0.12, -0.91) to (0.31, 0.11)
[controller_server-10] [INFO] [1787079123.517984221] [controller_server]: Received a goal, begin computing control effort.
[bt_navigator-11] [ERROR] [1787079123.539638515] [bt_navigator]: Goal failed
[controller_server-10] [ERROR] [1787079126.326674403] [controller_server]: RegulatedPurePursuitController detected collision ahead!
"""


def test_the_goal_3_log_shape_yields_a_tenth_of_a_second_window():
    """The orphan's signature in one number: the goal OWNED 0.13 s of wall clock,
    and the controller drove for 2.8. Everything past window+grace is unowned."""
    windows = goal_windows(LOG_3C_SHAPE)
    assert len(windows) == 1
    a, b = windows[0]
    assert b - a == pytest.approx(0.127, abs=0.01)


def test_the_orphan_burst_is_flagged_and_the_owned_tail_is_not():
    windows = [(100.0, 100.13)]
    cmd = [100.05, 100.5, 101.0] + [101.2 + 0.05 * i for i in range(30)]
    bursts = unowned_bursts(cmd, windows, grace_s=1.0)
    assert len(bursts) == 1
    first, last, n = bursts[0]
    assert first == pytest.approx(101.2)
    assert n == 30  # all thirty of the post-grace run; the owned three stay out


def test_commands_inside_the_window_plus_grace_are_owned():
    windows = [(100.0, 120.0)]
    cmd = [99.5, 100.0, 110.0, 120.9]
    assert unowned_bursts(cmd, windows, grace_s=1.0) == []


def test_an_unclosed_window_extends_to_the_end_of_the_recording():
    """A log truncated mid-goal (the launch died with the goal running) must not
    convert the whole tail of the flight into false unowned motion."""
    windows = goal_windows(
        "[x] [1787079123.4] [bt_navigator]: Begin navigating somewhere\n"
    )
    assert windows == [(1787079123.4, float("inf"))]
    assert unowned_bursts([1787080000.0], windows) == []


def test_two_goals_two_windows_no_bleed():
    text = (
        "[x] [100.0] [bt_navigator]: Begin navigating a\n"
        "[x] [110.0] [bt_navigator]: Goal succeeded\n"
        "[x] [200.0] [bt_navigator]: Begin navigating b\n"
        "[x] [210.0] [bt_navigator]: Goal failed\n"
    )
    assert goal_windows(text) == [(100.0, 110.0), (200.0, 210.0)]
    assert unowned_bursts([150.0], goal_windows(text), grace_s=1.0) == [
        (150.0, 150.0, 1)
    ]
