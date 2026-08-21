"""Unit tests for the touch-response consumer's logic (D48's other half).

Every case here is a field failure or a named near-miss, not a coverage exercise.
"""

import math

import pytest

from sphero_rvr_core.contact_marking import (
    COSTMAP_CIRCUMSCRIBED_RADIUS_M,
    COSTMAP_INSCRIBED_RADIUS_M,
    FOOTPRINT_FRONT_M,
    FOOTPRINT_REAR_M,
    HALF_WIDTH_LEFT_M,
    HALF_WIDTH_RIGHT_M,
    ROBOT_RADIUS_M,
    StallEventTracker,
    contact_mark_centre,
    default_margin_m,
    disc_points,
)
from sphero_rvr_core.freeze_marks import FreezeMarkSet


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


def test_the_footprint_is_the_measured_body_not_a_padded_number():
    """Pinned against the TAPE, not against a config field whose name sounds right.

    This test is here because the first version of this module took 0.1145 from
    `collision_stop.yaml`'s `footprint_rear_m` and called it measured -- it is the
    measured 0.0995 plus 0.015 m of margin. Padding is correct for a brake and wrong
    for saying where the body is, and a field name is not a provenance."""
    from tests.test_footprint_derivation import tape_in_base_link
    front, rear, left, right = tape_in_base_link()
    assert FOOTPRINT_FRONT_M == pytest.approx(front)
    assert FOOTPRINT_REAR_M == pytest.approx(rear)
    assert (HALF_WIDTH_LEFT_M, HALF_WIDTH_RIGHT_M) == pytest.approx((left, right))


def test_the_costmap_radii_are_not_the_configured_robot_radius():
    """M1/M2, 2026-08-18. `robot_radius: 0.145` is a REQUEST; nav2 pads it by
    `footprint_padding` and derives the inscribed radius -- and the polygon footprint
    clearing erases from -- out of the padded shape. Measured on the deployed binary:
    apothem 0.1519, vertex radii to 0.1591.

    This test exists because BOTH mark-geometry derivations this project produced were
    computed from 0.145, a number the costmap never sees. If someone 'tidies' these back
    to robot_radius, that whole class of error comes back."""
    assert COSTMAP_INSCRIBED_RADIUS_M != ROBOT_RADIUS_M
    assert COSTMAP_INSCRIBED_RADIUS_M > ROBOT_RADIUS_M, (
        "padding makes the deployed body BIGGER than configured, never smaller")
    assert COSTMAP_CIRCUMSCRIBED_RADIUS_M > COSTMAP_INSCRIBED_RADIUS_M


def test_the_declared_padding_explains_the_measured_radii():
    """Guard the DERIVATION, not just the numbers: the config now declares
    `footprint_padding`, and it must remain the value that accounts for the measured
    footprint. A padding edited without re-measuring would leave these constants
    describing a robot the costmap no longer builds."""
    import re
    from pathlib import Path
    cfg = (Path(__file__).resolve().parents[1] / "config" / "lean_nav2_stock.yaml").read_text()
    m = re.search(r"^\s*footprint_padding:\s*([0-9.]+)", cfg, re.M)
    assert m, "footprint_padding must stay DECLARED -- an invisible default is the defect"
    padding = float(m.group(1))
    # nav2 pads sign-wise per vertex, so the on-axis vertex moves out by exactly padding.
    assert ROBOT_RADIUS_M + padding == pytest.approx(0.1550, abs=5e-4), (
        "the smallest measured vertex radius (0.1550) is robot_radius + padding; if this "
        "no longer holds, re-run M1 rather than editing the constants")


def test_the_lateral_half_extents_are_not_the_longitudinal_ones():
    """The v2 strip spans laterally, and the number for that is 0.106 (right) -- not
    0.1145, which is a padded half-LENGTH. Guarding the distinction because the strip
    proposal reached for the longitudinal number first."""
    assert HALF_WIDTH_RIGHT_M != FOOTPRINT_REAR_M
    assert max(HALF_WIDTH_LEFT_M, HALF_WIDTH_RIGHT_M) == 0.106


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


# --- D57: stall classes -- translation marks, rotation must corroborate ---------------
#
# The 2026-08-19 flight's ground truth (Scott, eyewitness): two permanent marks
# planted in the door gap from pure-rotation stalls against floor grip, chassis
# touching nothing. The commanded twists below are the flight's own numbers.

from sphero_rvr_core.contact_marking import (  # noqa: E402
    STALL_IDLE,
    STALL_ROTATION,
    STALL_TRANSLATION,
    PIVOT_ZERO_EPSILON_RAD_S,
    classify_stall,
)
from sphero_rvr_core.pivot_curve import PIVOT_LINEAR_EPSILON_MPS  # noqa: E402
from sphero_rvr_core.refusal_promotion import RecentReturns  # noqa: E402


def test_the_flights_three_stalls_all_classify_as_rotation():
    """The false-positive class, by its own recorded commands: cmd_vel at each of
    the three firmware stalls (t+26/33/48) was vx 0.000 with wz 3.550/3.550/5.830."""
    for wz in (3.550, 3.550, 5.830):
        assert classify_stall(0.000, wz) == STALL_ROTATION


def test_a_driven_contact_classifies_as_translation_arcing_included():
    """The d45bd24 boot class (forward drive), reverse contacts (D37's rear-edge
    rule), and an RPP arc (translation WITH rotation) are all translation stalls:
    the rover was commanded through space, so a stall is strong contact evidence."""
    assert classify_stall(0.12, 0.0) == STALL_TRANSLATION
    assert classify_stall(-0.08, 0.0) == STALL_TRANSLATION
    assert classify_stall(0.10, 1.2) == STALL_TRANSLATION


def test_a_stall_with_no_commanded_motion_is_a_phantom():
    assert classify_stall(0.0, 0.0) == STALL_IDLE


def test_the_class_boundaries_are_the_drivers_own_not_new_tunables():
    """Derived-with-receipt (the design round's pin): the translation boundary IS
    pivot_curve's PIVOT_LINEAR_EPSILON_MPS -- the one shared definition of "in
    place" the driver's control loop routes on -- and the rotation/idle boundary
    is plan_pivot's zero test. Imported, not copied: these assertions hold the
    classifier to the same authority the drivetrain obeys, so the two can never
    disagree about what a command was."""
    assert classify_stall(PIVOT_LINEAR_EPSILON_MPS, 0.0) == STALL_TRANSLATION
    assert classify_stall(PIVOT_LINEAR_EPSILON_MPS * 0.99, 0.0) == STALL_IDLE
    assert classify_stall(0.0, PIVOT_ZERO_EPSILON_RAD_S * 2) == STALL_ROTATION
    assert classify_stall(0.0, PIVOT_ZERO_EPSILON_RAD_S) == STALL_IDLE
    # the receipt itself: plan_pivot's zero_epsilon default is this constant
    import inspect
    from sphero_rvr_core.pivot_curve import plan_pivot
    zero_default = inspect.signature(plan_pivot).parameters["zero_epsilon"].default
    assert zero_default == PIVOT_ZERO_EPSILON_RAD_S, (
        "plan_pivot's zero test drifted from the classifier's copy of it -- "
        "CHANGE BOTH OR NEITHER")


def test_rotation_paint_needs_a_fresh_return_inside_the_disc():
    """The corroboration rule, on the watcher's own rig-certified ring (one
    authority): a fresh return inside the would-be disc admits the mark; an
    empty ring, a stale return, or a return outside the disc refuses it."""
    disc_x, disc_y, disc_r = 1.0, 0.5, 0.145
    fresh = RecentReturns()
    fresh.add(t=100.0, x=disc_x + 0.05, y=disc_y)
    assert fresh.fresh_near(100.5, disc_x, disc_y, radius_m=disc_r)

    empty = RecentReturns()
    assert not empty.fresh_near(100.5, disc_x, disc_y, radius_m=disc_r)

    stale = RecentReturns()
    stale.add(t=10.0, x=disc_x, y=disc_y)
    assert not stale.fresh_near(100.5, disc_x, disc_y, radius_m=disc_r)

    outside = RecentReturns()
    outside.add(t=100.0, x=disc_x + disc_r + 0.10, y=disc_y)
    assert not outside.fresh_near(100.5, disc_x, disc_y, radius_m=disc_r)


def test_the_marker_wires_the_corroboration_feed_deliberately():
    """The QoS care-item, pinned at source: the marker subscribes /tof/points on
    the plain default profile -- the SAME profile the tof node publishes with and
    the refusal watcher already receives this stream on. The touch-port register
    carries one open QoS-silence row; this pin exists so the seam cannot grow a
    silent sibling by someone 'improving' one side's QoS alone."""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver" /
           "contact_marker_node.py").read_text()
    assert '"tof_points_topic", "/tof/points"' in src
    assert "self._on_tof, 10," in src, (
        "the marker's /tof/points subscription no longer uses the plain default "
        "profile (depth int => RELIABLE+VOLATILE); if that changed deliberately, "
        "re-verify against the tof node's publisher profile and update this pin")
    tof_src = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver" /
               "tof_node.py").read_text()
    assert 'create_publisher(PointCloud2, "~/points", 5)' in tof_src, (
        "the tof points publisher's profile changed -- re-verify the marker's "
        "and the watcher's subscriptions against it before updating this pin")
