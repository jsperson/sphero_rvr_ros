"""Freeze-as-sensor: classification and the mark set. Pure, no ROS.

The freeze signature is this robot's ONLY sense of touch. Nothing else on it can
detect a contact with an obstacle below the 0.19 m lidar plane and inside the
camera's ~0.45 m blind zone: the lidar sees through it, the camera cannot focus that
close, the firmware's stall detector stays sub-threshold (measured 2026-08-10: zero
stall events during a contact Scott could hear from across the room), and position
odometry frozen at a stop looks identical to "commanded to stop".

What DOES distinguish it is the supervisor's output. If the supervisor zeroed the
motors, it is braking for something it can see. If the supervisor was letting us
drive and the robot still did not move, something is there that nothing can see.
"""

import math

import pytest

from sphero_rvr_core.decisive_control import (
    BackOffConfig,
    ProgressGuard,
)
from sphero_rvr_core.freeze_marks import FreezeMarkSet


CFG = BackOffConfig(stall_time_s=1.0, progress_epsilon_m=0.03, back_off_speed_mps=0.10)


def _stall(guard, output_moving, cycles=12, t0=0.0, step=0.1):
    """Hold the robot at one spot until the stall clock fires, and return THAT result.

    The classification is delivered once, on the transition out of "drive" — later
    cycles are back-off continuations and carry no verdict. Taking the last result
    instead of the transition is a real trap: it reads False on a genuine freeze,
    which is exactly how this test first failed.
    """
    for i in range(cycles):
        result = guard.step(0.0, 0.0, t0 + i * step, True, output_moving=output_moving)
        if result.action != "drive":
            return result
    raise AssertionError("the guard never left 'drive' — the stall never fired")


# --- classification ---------------------------------------------------------

def test_a_stall_while_the_supervisor_permits_motion_is_a_freeze():
    """REVERT-PROOF SCENARIO 1. The supervisor said go, the robot did not move:
    there is something there that no sensor can see. This is the case measured in
    the gap at 14:28:21 — output 0.14 m/s for 2.5 s, odometry frozen to the
    millimetre, every sector reading clear."""
    guard = ProgressGuard(CFG)
    result = _stall(guard, output_moving=True)
    assert result.action == "reverse", "the response is still to back straight out"
    assert result.freeze is True


def test_a_stall_while_the_supervisor_is_braking_is_NOT_a_freeze():
    """REVERT-PROOF SCENARIO 2. The supervisor zeroed the motors, so of course the
    robot did not move — that is the brake working, not a discovery. Without this
    pairing the freeze exemption would swallow every ordinary braked failure and the
    give-up counter would never fire again."""
    guard = ProgressGuard(CFG)
    result = _stall(guard, output_moving=False)
    assert result.action == "reverse"
    assert result.freeze is False


def test_classification_is_by_majority_not_by_the_final_instant():
    """An approach that flaps between SLOW and STOPPED must not classify on a coin
    toss. Mostly-permitted -> freeze."""
    guard = ProgressGuard(CFG)
    for i in range(12):
        r = guard.step(0.0, 0.0, i * 0.1, True, output_moving=(i % 4 != 0))
        if r.action != "drive":
            break
    assert r.freeze is True


def test_mostly_braked_is_not_a_freeze():
    guard = ProgressGuard(CFG)
    for i in range(12):
        r = guard.step(0.0, 0.0, i * 0.1, True, output_moving=(i % 4 == 0))
        if r.action != "drive":
            break
    assert r.freeze is False


def test_omitting_output_moving_never_classifies_a_freeze():
    """Back-compatibility: a caller that does not supply the supervisor's output
    cannot have its stalls silently reinterpreted."""
    guard = ProgressGuard(CFG)
    result = _stall(guard, output_moving=None)
    assert result.action == "reverse" and result.freeze is False


def test_real_progress_clears_the_freeze_window():
    """Moving again resets the tally, so a freeze cannot be inherited from an
    earlier, unrelated stall."""
    guard = ProgressGuard(CFG)
    for i in range(6):
        guard.step(0.0, 0.0, i * 0.1, True, output_moving=True)
    guard.step(0.5, 0.0, 0.7, True, output_moving=True)      # real progress
    for i in range(12):
        r = guard.step(0.5, 0.0, 0.8 + i * 0.1, True, output_moving=False)
        if r.action != "drive":
            break
    assert r.freeze is False


def test_a_freeze_that_exhausts_the_back_offs_still_reports_frozen():
    """The abort handed to the planner must still carry the classification, or the
    mission layer cannot tell discovery from a broken stack."""
    guard = ProgressGuard(BackOffConfig(stall_time_s=0.2, max_back_offs=0))
    for i in range(8):
        r = guard.step(0.0, 0.0, i * 0.1, True, output_moving=True)
        if r.action == "abort":
            break
    assert r.action == "abort" and r.freeze is True


# --- the mark set -----------------------------------------------------------

def test_a_mark_expires_after_its_ttl():
    """REVERT-PROOF SCENARIO 4. A moved chair must not haunt the map forever."""
    marks = FreezeMarkSet(ttl_s=300.0)
    marks.add(1.0, 2.0, now=0.0)
    assert len(marks.live(now=299.0)) == 1
    assert len(marks.live(now=301.0)) == 0


def test_refreezing_the_same_spot_refreshes_rather_than_stacks():
    """Three retries at one obstacle is one fact, not three."""
    marks = FreezeMarkSet(ttl_s=100.0, merge_radius_m=0.15)
    marks.add(1.0, 1.0, now=0.0)
    marks.add(1.05, 1.02, now=50.0)
    live = marks.live(now=60.0)
    assert len(live) == 1
    assert live[0].expires_at == pytest.approx(150.0), "the retry must extend the TTL"


def test_distinct_obstacles_are_kept_separately():
    marks = FreezeMarkSet(ttl_s=100.0, merge_radius_m=0.15)
    marks.add(0.0, 0.0, now=0.0)
    marks.add(2.0, 0.0, now=1.0)
    assert len(marks.live(now=2.0)) == 2


def test_report_list_is_plain_serializable_data():
    marks = FreezeMarkSet()
    marks.add(1.23456, -0.5, now=10.0)
    import json
    payload = marks.as_report_list(now=11.0)
    assert json.loads(json.dumps(payload)) == payload
    assert payload[0]["x"] == 1.235
