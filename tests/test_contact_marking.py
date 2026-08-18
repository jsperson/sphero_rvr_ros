"""Unit tests for the touch-response consumer's logic (D48's other half).

Every case here is a field failure or a named near-miss, not a coverage exercise.
"""

import math

import pytest

from sphero_rvr_core.contact_marking import (
    FOOTPRINT_FRONT_M,
    FOOTPRINT_REAR_M,
    ROBOT_RADIUS_M,
    StallEventTracker,
    contact_mark_centre,
    default_margin_m,
    disc_points,
)
from sphero_rvr_core.decisive_control import FreezeMarkSet


# --- the counter seam ---------------------------------------------------------------

def test_the_first_diagnostics_message_plants_nothing():
    """The count starts wherever the driver already is. On 2026-08-18 that was 4 by the
    end of the flight -- so a node coming up late would have planted four permanent
    marks at its own start pose, which is D43 reached through a different door."""
    tracker = StallEventTracker()
    assert tracker.observe(4) is None
    assert tracker.observe(4) is None


def test_an_increase_is_a_contact():
    tracker = StallEventTracker()
    tracker.observe(0)
    batch = tracker.observe(1)
    assert batch.contacts == 1 and batch.collapsed is False


def test_two_contacts_in_one_message_collapse_and_say_so():
    """Only one pose is available for a batch, so N contacts become one mark. The flag
    exists so the report can say '4 contacts, 3 marks' rather than implying they are
    the same number."""
    tracker = StallEventTracker()
    tracker.observe(0)
    batch = tracker.observe(3)
    assert batch.contacts == 3 and batch.collapsed is True


def test_a_driver_restart_is_not_negative_contacts():
    """The counter goes backwards when the driver restarts. Re-baseline; mark nothing.
    Treating a decrease as data would either crash or plant marks for contacts that
    happened somewhere this node cannot place."""
    tracker = StallEventTracker()
    tracker.observe(7)
    assert tracker.observe(0) is None, "a restart is a new counter, not new contacts"
    assert tracker.observe(1).contacts == 1, "and counting resumes normally after"


def test_the_field_sequence_yields_exactly_four_contacts():
    """The 08-18 retest: 0 -> 4 across the flight, one at a time, with the 1 Hz
    diagnostics repeating each value several times in between."""
    tracker = StallEventTracker()
    stream = [0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4]
    total = sum(b.contacts for b in map(tracker.observe, stream) if b)
    assert total == 4


# --- geometry: D37's leading edge, ported ------------------------------------------

def test_a_forward_contact_marks_ahead_and_a_reversing_contact_marks_behind():
    """D37. A contact while backing out means the obstacle is BEHIND; marking the front
    edge then plants a permanent lethal disc on clear floor ahead while leaving the
    real obstacle unmarked."""
    ahead = contact_mark_centre(0.0, 0.0, 0.0, reversing=False)
    behind = contact_mark_centre(0.0, 0.0, 0.0, reversing=True)
    assert ahead[0] > 0.0 and behind[0] < 0.0


def test_the_mark_is_placed_along_the_heading_not_the_axis():
    """Facing 90 deg, the mark goes to +y. A mark that ignores yaw marks the room's
    axes instead of the robot's approach."""
    x, y = contact_mark_centre(1.0, 2.0, math.pi / 2, reversing=False)
    assert x == pytest.approx(1.0, abs=1e-9)
    assert y == pytest.approx(2.0 + FOOTPRINT_FRONT_M + default_margin_m())


def test_the_disc_never_covers_the_robots_own_centre():
    """THE LOAD-BEARING GEOMETRIC INVARIANT. A robot whose own cell is at inscribed
    cost cannot plan, cannot recover, and cannot be rescued by any behaviour -- it is
    D43 in the field and `exploration-start-clearance` on the bench. The margin is
    derived from exactly this requirement, so the test states it directly."""
    for reversing in (False, True):
        cx, cy = contact_mark_centre(0.0, 0.0, 0.0, reversing=reversing)
        assert math.hypot(cx, cy) > ROBOT_RADIUS_M, (
            "the mark disc reaches the robot's centre; nothing can plan from here")


def test_the_disc_near_edge_clears_the_footprint_edge():
    """The stronger form the default margin actually buys: the disc's near edge sits at
    or beyond the body's own edge, so a mark is born entirely OUTSIDE the footprint."""
    cx, _ = contact_mark_centre(0.0, 0.0, 0.0, reversing=False)
    assert cx - ROBOT_RADIUS_M >= FOOTPRINT_FRONT_M - 1e-9
    cx_rev, _ = contact_mark_centre(0.0, 0.0, 0.0, reversing=True)
    assert abs(cx_rev) - ROBOT_RADIUS_M >= FOOTPRINT_REAR_M - 1e-9


def test_the_footprint_is_the_measured_body_not_the_padded_brake_distance():
    """`collision_stop.yaml` pads its front/rear numbers with braking and payload
    margin -- correct for a brake, wrong for describing where the body is. The bespoke
    controller's 0.11/0.16 were those padded values."""
    assert FOOTPRINT_FRONT_M == 0.0965 and FOOTPRINT_REAR_M == 0.1145


def test_a_shorter_margin_would_break_the_invariant():
    """Guard the derivation, not just its output: if someone 'tidies' the margin down
    to something that looks tidier, the centre-coverage invariant is what breaks."""
    cx, cy = contact_mark_centre(0.0, 0.0, 0.0, reversing=False, margin_m=0.01)
    assert math.hypot(cx, cy) < ROBOT_RADIUS_M, (
        "this margin must be UNSAFE -- if it is not, the invariant test above proves "
        "nothing and the derivation has drifted")


# --- the cloud ----------------------------------------------------------------------

def test_the_disc_is_filled_not_an_annulus():
    """At 0.05 m costmap resolution a single ring of radius 0.145 leaves interior cells
    untouched. Centre + half-radius + full-radius rings is what fills it."""
    points = disc_points(0.0, 0.0, 0.145, 12)
    assert len(points) == 1 + 12 * 2
    radii = sorted({round(math.hypot(px, py), 4) for px, py in points})
    assert radii == [0.0, 0.0725, 0.145]


def test_every_disc_point_is_within_the_radius():
    points = disc_points(3.0, -1.0, 0.145, 12)
    assert all(math.hypot(px - 3.0, py + 1.0) <= 0.145 + 1e-9 for px, py in points)


# --- the TTL decision ---------------------------------------------------------------

def test_an_infinite_ttl_keeps_marks_forever_and_the_mechanism_still_works():
    """v1's decision, tested rather than trusted: `now + inf` must not break pruning.

    The reason it is infinity: a finite TTL cannot expire a costmap mark (measured
    2026-08-10, cost 0 -> 100 and stayed), so it would only make the REPORT claim an
    expiry the world does not honour."""
    marks = FreezeMarkSet(ttl_s=float("inf"), merge_radius_m=0.15)
    marks.add(1.0, 1.0, now=0.0)
    assert len(marks.live(now=1e9)) == 1
    assert len(marks.as_report_list(now=1e9)) == 1


def test_a_finite_ttl_would_have_made_the_report_lie():
    """The counterfactual, kept as a test so the choice is legible: at 300 s the mark
    leaves the report while the costmap keeps enforcing it."""
    marks = FreezeMarkSet(ttl_s=300.0, merge_radius_m=0.15)
    marks.add(1.0, 1.0, now=0.0)
    assert marks.as_report_list(now=301.0) == [], (
        "this divergence is exactly why v1 sets the TTL to infinity")


def test_retrying_the_same_spot_is_one_fact_not_four():
    """08-18: four contacts with the same leg, ~12-14 s apart (the BT's retry period).
    Within the merge radius they are one learned fact."""
    marks = FreezeMarkSet(ttl_s=float("inf"), merge_radius_m=0.15)
    for i in range(4):
        marks.add(1.0 + 0.02 * i, 1.0, now=float(i * 13))
    assert len(marks.live(now=100.0)) == 1
