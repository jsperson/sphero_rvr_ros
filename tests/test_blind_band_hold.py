"""The hold that stops silence from being read as clearance (D39).

Every constant here is DERIVED, in the test, from the deployed config and the shipped
sensor geometry -- never from a number remembered out of an archive README. The README
that describes this band states it in `x`; the brake compares `range`; a test that
transcribed the README's figures would pin the wrong quantity and pass.
"""

import json
import math
import re
from pathlib import Path

import pytest

from sphero_rvr_core.low_obstacle_brake import (
    BlindBandHold, forward_speed_scale, nearest_swept_point, swept_path_obstacle,
    transport_point,
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
MIN_SCALE = _yaml_float("low_obstacle_min_forward_scale")


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

def _fixture():
    path = Path(__file__).resolve().parent / "fixtures" / "run1_vanish_20260815.json"
    return json.loads(path.read_text())


def _pose_delta_series(odom):
    """Measured deltas between consecutive odom samples, in the PREVIOUS base_link
    frame -- the same arithmetic `collision_stop_node._pose_delta_since_last` does, so
    the replay feeds the hold what production would feed it."""
    def at(t):
        prev = None
        for p in odom:
            if p["t_rel_s"] <= t:
                prev = p
            else:
                break
        return prev
    return at


def test_run1_vanish_holds_instead_of_releasing():
    """THE REVERT-PROOF, replayed from run 1's OWN RECORDING.

    `tests/fixtures/run1_vanish_20260815.json` is extracted from
    `bag_20260815_172129_0.mcap` by `diagnostics/extract_bag_window.py`, pairing
    `/tof/obstacles`, `/collision_stop/state` and `/odom` from the same bag -- no clock
    fit, and the window located BY SIGNATURE (the frame where `cam_scale` leaves 0.00
    while `cam_nearest` jumps outward) rather than by a timestamp relayed through a
    recorder whose t=0 has been 53 s off before. The extractor found exactly the
    archived numbers: scale 0.00 -> 0.65, nearest 0.181 -> 0.251.

    What the recording shows, and what the fix must change:

        t_rel    points   live nearest   PRODUCTION did   this hold does
        -0.178      8        0.1815         scale 0.00      scale 0.00
        -0.031      8        0.2506      -> scale 0.65      scale 0.00   <- the release
         1.116      0          None      -> scale 1.00      scale 0.00   <- full speed
         1.258      1        0.2006         scale 1.00      scale 0.00   <- the stray

    At -0.031 the rover had not moved (odom identical across the preceding second) and
    neither had the leg. The object left the sensor's band and production read that as
    clearance.

    THE REPLAYED MOTION IS COUNTERFACTUAL AND THAT IS STATED, NOT HIDDEN: the odom in
    this window is the rover driving into the leg, which is motion the fix would have
    prevented. Feeding it back in transports the belief NEARER (0.1815 -> 0.1159), so
    the replay is strictly harder on the hold than reality would have been. It also
    means this test exercises the belief crossing INSIDE `low_obstacle_min_range_m`
    (0.14) on real data -- the case where applying the sensor's validity floor to a
    belief would silently release the clamp.
    """
    fx = _fixture()
    hold = _hold()
    pose_at = _pose_delta_series(fx["odom"])
    previous = None
    released = []

    for frame in fx["clouds"]:
        t = frame["t_rel_s"]
        if not (-0.6 <= t <= 1.4):
            continue
        current = pose_at(t)
        delta = None
        if previous is not None and current is not None:
            wx, wy = current["x"] - previous["x"], current["y"] - previous["y"]
            c, s = math.cos(previous["yaw"]), math.sin(previous["yaw"])
            dyaw = math.atan2(math.sin(current["yaw"] - previous["yaw"]),
                              math.cos(current["yaw"] - previous["yaw"]))
            delta = (c * wx + s * wy, -s * wx + c * wy, dyaw)
        previous = current

        result = hold.update(frame["points_xy"], True, 0.14, 0.0, delta)
        scale = forward_speed_scale(result.nearest_m, STOP, SLOW, MIN_SCALE)
        if scale > 0.0:
            released.append((t, scale, result.reason))

    # THE FIELD SYMPTOM, ASSERTED FIRST. Against the pre-fix module the frames at
    # -0.031 (0.65), 1.116 (1.00) and 1.258 (1.00) all appear here.
    assert released == [], f"the brake released on recorded data: {released}"


def test_the_recording_contains_the_release_it_is_supposed_to_indict():
    """PREMISE TRIPWIRE -- a revert-proof over a fixture that does not contain the
    defect would pass forever and prove nothing.

    Replays the same clouds through the STATELESS primitive the fix replaced, and
    requires that it does release. If a future re-extraction quietly grabs the wrong
    window, this fails rather than the test above silently going green.
    """
    fx = _fixture()
    scales = []
    for frame in fx["clouds"]:
        if not (-0.6 <= frame["t_rel_s"] <= 1.4):
            continue
        nearest = swept_path_obstacle(frame["points_xy"], 0.14, 0.0, HALF_W, MIN_R, MAX_R)
        scales.append(forward_speed_scale(nearest, STOP, SLOW, MIN_SCALE))

    assert 0.0 in scales, "the window must contain the tracked approach"
    assert max(scales) == 1.0, "the window must contain the full-speed release"
    assert scales.index(0.0) < scales.index(1.0), "the release must FOLLOW the hold"


def test_the_stray_return_does_not_govern_over_a_nearer_belief():
    """Run 1's stray 0.201 m return restored full commanded speed in the field. A
    farther sighting must never retire a nearer belief -- with the rover stationary it
    is evidence of partial visibility, not of the object receding."""
    hold = _hold()
    seen = _fwd(hold, [(0.181, 0.0)])
    assert seen.reason == "live"
    vanished = _fwd(hold, [], pose_delta=(0.0, 0.0, 0.0))
    assert forward_speed_scale(vanished.nearest_m, STOP, SLOW) == 0.0
    assert vanished.nearest_m == pytest.approx(0.181)
    assert vanished.active and vanished.reason == "vanished_in_band"

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
