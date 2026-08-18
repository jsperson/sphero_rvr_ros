"""The placement policy, held against the three contacts it actually lost.

2026-08-18, run 3d: contact_marker v1 detected 3/3 real contacts (an unstaged
horizontal table leg) and placed 0/3 marks -- its exact-stamp TF lookup found
map->odom 69/73/87 ms behind every contact stamp and refused. The fixture here IS
those three contacts, extracted from the flight bag with their real transform
neighbourhoods and an interpolated ground-truth pose (the answer tf2 would have
given once the data existed). Every test is that field failure or a named seam of
the fix, not a coverage exercise.

The rig-side falsifier (real tf2 under a lagging map->odom must reproduce the
ExtrapolationException before the fix's success means anything) lives in
`tests/test_placement_tf_lag_integration.py`, which needs rclpy and runs on the Pi.
"""

import json
import math
from pathlib import Path

import pytest

from sphero_rvr_core.contact_marking import (
    MEASURED_MAP_TF_GAP_MAX_S,
    MEASURED_MAP_TF_GAP_P99_S,
    ROBOT_RADIUS_M,
    STALENESS_BOUND_S,
    ContactPoseUnavailable,
    PoseDataLagsStamp,
    StallEventTracker,
    default_margin_m,
    resolve_contact_pose,
)

FIXTURE = json.loads(
    (Path(__file__).parent / "fixtures" / "contact_placement_3d_20260818.json")
    .read_text()
)


def _yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _compose_map_base(map_odom, odom_base):
    """map->base_link from the two recorded transforms, as tf2 would compose them."""
    myaw = _yaw_of(map_odom["q"])
    c, s = math.cos(myaw), math.sin(myaw)
    bx, by = odom_base["t"][0], odom_base["t"][1]
    return (
        map_odom["t"][0] + c * bx - s * by,
        map_odom["t"][1] + s * bx + c * by,
        myaw + _yaw_of(odom_base["q"]),
    )


def _lookups_for(contact):
    """exact/latest closures behaving exactly as tf2 did at that instant.

    `exact` raises the lag signal because the contact stamp is NEWER than the newest
    map->odom the buffer held -- which is true for all three fixture contacts and is
    the whole defect. `latest` answers with the newest transform and its stamp.
    """
    latest_mo = contact["latest_map_odom"]

    def exact():
        assert contact["stamp"] > latest_mo["stamp"]
        raise PoseDataLagsStamp(
            f"requested {contact['stamp']:.6f} but the latest data is at "
            f"{latest_mo['stamp']:.6f}"
        )

    def latest():
        pose = _compose_map_base(latest_mo, contact["nearest_odom_base"])
        return pose, latest_mo["stamp"]

    return exact, latest


# --- the field failure, pinned ------------------------------------------------------

def test_the_fixture_reproduces_the_field_failure_shape():
    """All three contacts arrived with the stamp AHEAD of the newest map->odom --
    the exact-stamp-only policy (v1's entire policy) loses every one of them. If this
    stops holding, the fixture no longer describes the defect and the suite must say
    so rather than quietly certifying against nothing."""
    for contact in FIXTURE["contacts"]:
        exact, _ = _lookups_for(contact)
        with pytest.raises(PoseDataLagsStamp):
            exact()


def test_the_recorded_staleness_is_ordinary_lag_not_an_outage():
    """69-87 ms: under the measured p99 gap, an order of magnitude under the bound.
    These were exactly the contacts the fallback exists for."""
    for contact in FIXTURE["contacts"]:
        staleness = contact["stamp"] - contact["latest_map_odom"]["stamp"]
        assert staleness < MEASURED_MAP_TF_GAP_P99_S < STALENESS_BOUND_S


def test_the_fix_places_all_three_field_contacts():
    for contact in FIXTURE["contacts"]:
        exact, latest = _lookups_for(contact)
        resolved = resolve_contact_pose(exact, latest, contact["stamp"])
        assert resolved.path == "fallback"
        assert 0.0 < resolved.staleness_s <= STALENESS_BOUND_S
        assert resolved.staleness_s == pytest.approx(
            contact["staleness_ms"] / 1000.0, abs=0.002
        )


def test_the_fallback_pose_matches_the_ground_truth_within_stall_creep():
    """The robot was STALLED at contact (that is what the detector detects), so the
    latest-available pose and the interpolated exact-stamp pose must agree to well
    under a centimetre -- and both sit far inside the disc's forgiveness."""
    for contact in FIXTURE["contacts"]:
        exact, latest = _lookups_for(contact)
        resolved = resolve_contact_pose(exact, latest, contact["stamp"])
        truth = contact["ground_truth_map_base"]
        error = math.hypot(resolved.x - truth["x"], resolved.y - truth["y"])
        assert error < 0.01
        assert error < default_margin_m()


# --- the policy's seams, both directions ---------------------------------------------

def _fresh_exact():
    return (1.0, 2.0, 0.5)


def _latest_at(pose, tf_stamp):
    return lambda: (pose, tf_stamp)


def test_the_exact_path_is_preferred_when_the_feed_is_caught_up():
    """The fallback must not replace the strictly-correct answer when it exists --
    looking up 'latest' for a MOVING robot answers a different question than asked."""
    resolved = resolve_contact_pose(
        _fresh_exact, _latest_at((9.0, 9.0, 9.0), 0.0), stamp_s=100.0
    )
    assert resolved.path == "exact"
    assert resolved.staleness_s == 0.0
    assert (resolved.x, resolved.y) == (1.0, 2.0)


def _lagging_exact():
    raise PoseDataLagsStamp("data not caught up")


def test_staleness_inside_the_bound_is_accepted_and_reported():
    resolved = resolve_contact_pose(
        _lagging_exact, _latest_at((1.0, 2.0, 0.5), 100.0 - 0.499), stamp_s=100.0
    )
    assert resolved.path == "fallback"
    assert resolved.staleness_s == pytest.approx(0.499)


def test_staleness_beyond_the_bound_still_refuses():
    """The honest refusal survives: beyond the bound is a transform OUTAGE, and a
    permanent lethal disc at an invented pose is worse than no mark."""
    with pytest.raises(ContactPoseUnavailable):
        resolve_contact_pose(
            _lagging_exact, _latest_at((1.0, 2.0, 0.5), 100.0 - 0.501), stamp_s=100.0
        )


def test_data_newer_than_the_stamp_is_lag_in_the_other_direction():
    """Between the failed exact lookup and the latest lookup a fresher transform can
    land, making 'latest' NEWER than the contact stamp. Same physics, same bound,
    signed staleness says which way."""
    resolved = resolve_contact_pose(
        _lagging_exact, _latest_at((1.0, 2.0, 0.5), 100.0 + 0.2), stamp_s=100.0
    )
    assert resolved.path == "fallback"
    assert resolved.staleness_s == pytest.approx(-0.2)
    with pytest.raises(ContactPoseUnavailable):
        resolve_contact_pose(
            _lagging_exact, _latest_at((1.0, 2.0, 0.5), 100.0 + 0.6), stamp_s=100.0
        )


def test_a_dead_transform_gets_no_fallback():
    """Only the lag signal opens the fallback. A missing frame or disconnected tree
    (any other exception) propagates untouched -- those poses are not late, they are
    untrustworthy, and 3d's honest-refusal behaviour must survive for them."""
    def dead_exact():
        raise RuntimeError('frame "map" does not exist')

    def must_not_be_called():
        raise AssertionError("fallback consulted for a non-lag failure")

    with pytest.raises(RuntimeError, match="does not exist"):
        resolve_contact_pose(dead_exact, must_not_be_called, stamp_s=100.0)


# --- the bound's provenance guard ----------------------------------------------------

def test_the_bound_still_covers_the_measured_feed():
    """STALENESS_BOUND_S is DERIVED from run 3d's measured map->odom cadence (p99
    103 ms, max gap 396 ms). If SLAM's cadence config changes and someone updates the
    measured constants, this fails BEFORE a mission does. And the safety argument is
    coupled to the SHIPPED disc geometry -- if the mark shape changes (the strip
    debate is dormant, not dead), the bound re-derives against the new shape."""
    assert STALENESS_BOUND_S > MEASURED_MAP_TF_GAP_P99_S
    assert STALENESS_BOUND_S > MEASURED_MAP_TF_GAP_MAX_S * 1.25
    # pathological-motion bound: worst regulated speed x bound stays inside the
    # disc's forgiveness (margin between footprint edge and disc centre).
    assert 0.2 * STALENESS_BOUND_S <= default_margin_m()
    assert default_margin_m() == pytest.approx(ROBOT_RADIUS_M)


# --- the counter seam replays the real flight ----------------------------------------

def test_the_recorded_diagnostics_timeline_yields_exactly_three_contacts():
    """The fixture's full 65-message counter timeline through the real tracker: the
    baseline is learned from the first message, and exactly the three increments
    become contacts -- the detection half was never the defect."""
    tracker = StallEventTracker()
    contacts = 0
    for row in FIXTURE["diagnostics_timeline"]:
        batch = tracker.observe(row["count"])
        if batch is not None:
            contacts += batch.contacts
    assert contacts == 3
