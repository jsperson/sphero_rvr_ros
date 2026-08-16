"""The hold that stops silence from being read as clearance (D39).

Every constant here is DERIVED, in the test, from the deployed config and the shipped
sensor geometry -- never from a number remembered out of an archive README. The README
that describes this band states it in `x`; the brake compares `range`; a test that
transcribed the README's figures would pin the wrong quantity and pass.
"""

import math
import re
from pathlib import Path

import pytest

from sphero_rvr_core.low_obstacle_brake import (
    BlindBandHold, forward_speed_scale, nearest_swept_point, transport_point,
)
from sphero_rvr_core.tof_frame import (
    TofConfig, blind_band_outer_range_m, blind_band_range_floor_m, zone_point,
)

CFG = Path(__file__).resolve().parents[1] / "config" / "collision_stop.yaml"


def _yaml_float(name):
    m = re.search(rf"^\s*{name}:\s*([0-9.]+)", CFG.read_text(), re.M)
    assert m, f"{name} is not in the deployed config"
    return float(m.group(1))


BAND = blind_band_outer_range_m(TofConfig())
#: How far the rover can close between two clouds the brake still calls fresh. The
#: brake's OWN freshness contract supplies both operands, so this cannot drift from the
#: staleness rule it is derived from.
CLOSURE = _yaml_float("max_forward_mps") * _yaml_float("low_obstacle_max_age_s")
STOP = _yaml_float("low_obstacle_stop_distance_m")
SLOW = _yaml_float("low_obstacle_slow_distance_m")
MIN_R = _yaml_float("low_obstacle_min_range_m")
MAX_R = _yaml_float("low_obstacle_max_range_m")
HALF_W = _yaml_float("low_obstacle_half_width_m")


def _hold():
    return BlindBandHold(BAND, CLOSURE, HALF_W, MIN_R, MAX_R)


def _fwd(hold, points, pose_delta=(0.0, 0.0, 0.0), v=0.14, w=0.0, looked=True):
    return hold.update(points, looked, v, w, pose_delta)


# --- the band itself ---------------------------------------------------------------

def test_band_outer_is_a_range_not_an_x():
    """The quantity, pinned. `blind_band_outer_range_m` must exceed every zone's x
    floor, because range >= x for any off-axis point -- if these were ever equal the
    function would have quietly become an x, which is the comparison mistake this
    whole constant exists to avoid."""
    x_max = max(zone_point(r, c, TofConfig().min_valid_mm, TofConfig())[0]
                for r in range(8) for c in range(8))
    assert BAND > x_max
    assert BAND == pytest.approx(0.1672, abs=5e-4)


def test_band_reproduces_the_archived_x_table():
    """The 2026-08-15 run-1 archive publishes the per-row x floors. Derivation and
    archive must agree, or one of them is describing a robot that does not exist.

    TOLERANCE IS THE ARCHIVE'S OWN PRECISION, not a chosen slack: the table is printed
    to three decimals, so it can only support agreement to +-1 mm however exact the
    geometry is. Seven rows agree to better than 0.4 mm; row 6 derives 0.15348 against
    a published 0.154 and is the one last-place rounding slip in the table. Demanding
    5e-4 here would fail on the archive's printing precision and read as a model
    error, which is a worse mistake than the 0.5 mm.
    """
    published = {0: 0.164, 1: 0.162, 2: 0.160, 3: 0.159,
                 4: 0.157, 5: 0.155, 6: 0.154, 7: 0.152}
    cfg = TofConfig()
    for row, x in published.items():
        assert zone_point(row, 3, cfg.min_valid_mm, cfg)[0] == pytest.approx(x, abs=1e-3)


def test_band_outer_takes_the_worst_zone_not_the_best():
    cfg = TofConfig()
    floors = [blind_band_range_floor_m(r, c, cfg) for r in range(8) for c in range(8)]
    assert BAND == max(floors)
    assert max(floors) - min(floors) > 0.010  # the spread is real, ~15 mm


# --- the field sequence this was built from ----------------------------------------

def test_run1_vanish_holds_instead_of_releasing():
    """RUN 1, 2026-08-15 17:25:44-49, the sequence that put the rangefinder into a
    table leg. Tracked to a hard stop at 0.181 m, then the returns left the band with
    neither the object nor the rover moving.

    THIS IS THE REVERT-PROOF. Against the pre-fix code the second frame yields None,
    `forward_speed_scale(None, ...)` returns 1.0, and the rover re-drives -- the field
    symptom exactly. The assertion is written against the SCALE, not an internal, so
    it indicts the behaviour rather than the implementation.
    """
    hold = _hold()
    leg = (0.181, 0.0)

    seen = _fwd(hold, [leg])
    assert seen.reason == "live"
    assert forward_speed_scale(seen.nearest_m, STOP, SLOW) == 0.0

    # The returns vanish. The rover has not moved -- odom was frozen through this.
    # THE SPEED ASSERTION COMES FIRST ON PURPOSE: against the pre-fix module this line
    # fails with 1.0 == 0.0, which is the field symptom itself (the rover accelerating
    # back into the leg). Leading with `.active` would kill the mutation on an internal
    # flag and prove only that a new field exists.
    vanished = _fwd(hold, [], pose_delta=(0.0, 0.0, 0.0))
    assert forward_speed_scale(vanished.nearest_m, STOP, SLOW) == 0.0
    assert vanished.nearest_m == pytest.approx(0.181)
    assert vanished.active
    assert vanished.reason == "vanished_in_band"

    # And the stray 0.201 m return that restored full commanded speed in the field
    # does not, because it is FURTHER than the belief and the nearer one governs.
    stray = _fwd(hold, [(0.201, 0.0)])
    assert forward_speed_scale(stray.nearest_m, STOP, SLOW) == 0.0


def test_the_prefix_behaviour_is_what_this_replaces():
    """Pins the defect itself: with no belief, an empty cloud IS full speed. If this
    ever stops being true the hold has become redundant and should be deleted rather
    than left as decoration."""
    assert nearest_swept_point([], 0.14, 0.0, HALF_W, MIN_R, MAX_R) is None
    assert forward_speed_scale(None, STOP, SLOW) == 1.0


def test_the_stop_distance_clears_the_structural_band():
    """The deployed config now CLAIMS this margin in prose, so it gets pinned here --
    a config comment is a claim, and an unchecked numeric claim in a comment is how
    'the footprint is honest' survived eleven months of being 11.5 mm wrong.

    The operand is the worst observed RANGE DECREASE under command-zero (0.027 m, run
    2's broad object), not travel and not creep: on a sloping object new nearer points
    enter the cloud while the rover is stationary, so the two quantities differ (the
    same run measured 9 mm of post-stop travel) and only the decrease bounds where the
    nearest return can end up.
    """
    worst_decrease_m = 0.027
    assert BAND + worst_decrease_m < STOP
    assert STOP - (BAND + worst_decrease_m) == pytest.approx(0.0058, abs=5e-4)


def test_the_hold_arms_further_out_than_the_stop_distance():
    """The two clauses are not redundant and must not be collapsed. The stop distance
    answers 'how close before we halt'; the hold answers 'how close before silence
    stops meaning clearance'. The second is necessarily the larger number, and if it
    ever falls below the stop distance the hold can only arm after the brake has
    already stopped -- useless for exactly the approach it exists to protect."""
    assert BAND + CLOSURE > STOP


def test_run1_last_reading_is_inside_the_arming_threshold():
    """The trigger has to actually cover the specimen it was built from -- and by a
    stated margin, so a small re-derivation of either operand does not silently
    uncover it."""
    assert 0.181 < BAND + CLOSURE
    assert (BAND + CLOSURE) - 0.181 > 0.030


# --- retirement: a look that could have seen, and did not ---------------------------

def test_reversing_out_retires_the_belief():
    """The belief is carried out to a range the sensor CAN report; the next look finds
    nothing there; only then is it retired."""
    hold = _hold()
    _fwd(hold, [(0.181, 0.0)])

    # Still inside the band after 20 mm of reverse: silence proves nothing.
    r1 = _fwd(hold, [], pose_delta=(-0.020, 0.0, 0.0))
    assert r1.active and r1.reason == "vanished_in_band"
    assert r1.nearest_m == pytest.approx(0.201)

    # Far enough out that the sensor would have reported it. Silence is now a fact.
    r2 = _fwd(hold, [], pose_delta=(-0.060, 0.0, 0.0))
    assert not r2.active
    assert r2.reason == "retired_sight_through"
    assert forward_speed_scale(r2.nearest_m, STOP, SLOW) == 1.0


def test_a_belief_still_visible_is_not_retired_it_is_tracked():
    """Reversing until the object is reportable and then SEEING it must refresh the
    belief, not clear it. Retirement is about absence, not about distance."""
    hold = _hold()
    _fwd(hold, [(0.181, 0.0)])
    out = _fwd(hold, [(0.241, 0.0)], pose_delta=(-0.060, 0.0, 0.0))
    assert out.reason == "live"
    assert out.nearest_m == pytest.approx(0.241)
    assert hold.belief_xy[0] == pytest.approx(0.241)


def test_pivot_lifts_the_clamp_without_discarding_the_belief():
    """STANDARDS RULE 5, ASKED OF THE ARBITER. If reverse were the only retirement,
    this clamp would be un-grantable by construction -- D40 proved the arbiter refuses
    reverse at exactly these poses, so the rover would be forward-frozen for the rest
    of the mission. Rotating out of the corridor must lift the clamp, and the object
    must still be believed afterwards."""
    hold = _hold()
    _fwd(hold, [(0.181, 0.0)])

    turned = hold.update([], True, 0.0, 0.6, (0.0, 0.0, math.radians(75)))
    assert turned.active                       # still believed
    assert turned.nearest_m is None            # but not clamping
    assert turned.reason.endswith("_off_path")
    assert hold.belief_xy is not None
    assert forward_speed_scale(turned.nearest_m, STOP, SLOW) == 1.0


def test_belief_inside_the_sensor_validity_floor_still_clamps():
    """The `min_range` floor rejects untrustworthy SENSOR READINGS; a belief is not a
    reading. Applying it to the belief would release the brake for the objects held
    CLOSEST -- this defect was written into the first draft of `_in_corridor` and is
    pinned here so it cannot come back."""
    hold = _hold()
    _fwd(hold, [(0.181, 0.0)])
    crept = _fwd(hold, [], pose_delta=(0.060, 0.0, 0.0))   # belief now 0.121 < MIN_R
    assert crept.nearest_m == pytest.approx(0.121)
    assert crept.nearest_m < MIN_R
    assert forward_speed_scale(crept.nearest_m, STOP, SLOW) == 0.0


# --- every uncertainty resolves to HOLD --------------------------------------------

def test_a_stale_cloud_does_not_retire_a_belief():
    """Staleness releasing the hold would be released-into-contact in a second
    costume. Nothing was looked at, so nothing was learned."""
    hold = _hold()
    _fwd(hold, [(0.181, 0.0)])
    # Reversed far enough that a LOOK would have retired it -- but there was no look.
    stale = _fwd(hold, [], looked=False, pose_delta=(-0.060, 0.0, 0.0))
    assert stale.active and stale.reason == "held_no_look"
    assert stale.nearest_m == pytest.approx(0.241)
    assert forward_speed_scale(stale.nearest_m, STOP, SLOW) < 1.0


def test_a_missing_pose_does_not_retire_a_belief():
    """A TF gap means the belief cannot be placed, and a belief that cannot be placed
    cannot be shown to be gone. Fail direction: hold."""
    hold = _hold()
    _fwd(hold, [(0.181, 0.0)])
    gap = _fwd(hold, [], pose_delta=None)
    assert gap.active and gap.reason == "held_no_pose"
    assert forward_speed_scale(gap.nearest_m, STOP, SLOW) == 0.0


def test_no_belief_no_clamp():
    hold = _hold()
    clear = _fwd(hold, [])
    assert not clear.active and clear.reason == "clear"
    assert forward_speed_scale(clear.nearest_m, STOP, SLOW) == 1.0


def test_an_object_that_vanishes_in_plain_sight_is_cleared_immediately():
    """The over-caution bound. An object seen at 0.45 m -- well outside the band --
    that disappears was genuinely looked at and genuinely is not there. Holding THAT
    would make every passing detection permanent and strangle the mission."""
    hold = _hold()
    seen = _fwd(hold, [(0.45, 0.0)])
    assert seen.reason == "live"
    gone = _fwd(hold, [])
    assert not gone.active and gone.reason == "retired_sight_through"
    assert forward_speed_scale(gone.nearest_m, STOP, SLOW) == 1.0


# --- the transform ------------------------------------------------------------------

def test_transport_is_the_inverse_of_the_robots_motion():
    assert transport_point((0.30, 0.0), (0.10, 0.0, 0.0))[0] == pytest.approx(0.20)
    assert transport_point((0.30, 0.0), (-0.10, 0.0, 0.0))[0] == pytest.approx(0.40)


def test_turning_left_puts_a_forward_object_on_the_right():
    """Sign check against a PHYSICAL fact, in the style `bearings.py` was given after
    the mirrored-clock error: rotate the robot left and the thing ahead of it must end
    up on its right."""
    x, y = transport_point((0.30, 0.0), (0.0, 0.0, math.radians(90)))
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(-0.30)
