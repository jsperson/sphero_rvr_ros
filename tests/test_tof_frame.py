"""Revert-proofs for the ToF frame core, replayed against RECORDED frames.

docs/tof_navigation_design.md §7. Every proof here runs against real sensor output
from 2026-08-13 rather than against a synthetic frame I designed to pass -- which
matters more than usual, because the design's central claim (background comparison
beats presence thresholding, 100% vs ~20%) is a claim ABOUT that data. If the claim
is wrong, these fail now, before any of it flies.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from fixtures.tof_recorded_frames import (  # noqa: E402
    BOX_074, CLEAR_FLOOR, RAIL_046, WALL_ONLY,
)
from sphero_rvr_core.tof_frame import (  # noqa: E402
    ObstacleDetector, TofConfig, expected_floor_m, floor_beyond_horizon,
    nearer_than_floor, plausible_for_zone, unexpected_returns, valid_mm,
    zone_angles, zone_point,
)


# ------------------------------------------------------------------ geometry facts

def test_axis_orientation_is_not_mirrored():
    """REVERT-PROOF 7. Column 0 is rover-LEFT and row 0 is UP -- both TESTED on
    hardware, not assumed.

    Pre-empting rather than repeating N1: the lidar's raw bearing 0 points BEHIND the
    robot, and a ladder rung once steered into a mirror image of open space because
    somebody assumed otherwise. A mirrored ToF mapping would put every obstacle on
    the wrong side, and the brake would swerve into what it was avoiding.
    """
    cfg = TofConfig()
    az_left, _ = zone_angles(3, 0, cfg)
    az_right, _ = zone_angles(3, 7, cfg)
    assert az_left > 0, "column 0 is not on the LEFT (positive azimuth, REP-103)"
    assert az_right < 0, "column 7 is not on the RIGHT"
    assert az_left == pytest.approx(-az_right, abs=1e-9), "azimuth is not symmetric"

    _, el_top = zone_angles(0, 3, cfg)
    _, el_bottom = zone_angles(7, 3, cfg)
    assert el_top > el_bottom, "row 0 is not UP"
    assert el_top - el_bottom == pytest.approx(math.radians(7 * cfg.zone_deg), abs=1e-9)


def test_the_fov_matches_the_measured_box_fill():
    """The 60 deg field was confirmed independently of the datasheet: a 25 cm box at
    0.30 m filled 6 of 8 columns. Six zones must therefore span 0.249 m at 0.30 m."""
    cfg = TofConfig()
    span = 2 * 0.30 * math.tan(math.radians(6 * cfg.zone_deg / 2.0))
    assert span == pytest.approx(0.249, abs=0.01), (
        f"6 columns span {span:.3f} m at 0.30 m; the measured box was 0.25 m wide")


# ------------------------------------------------------------------- validity rules

def test_sentinel_is_never_a_measurement():
    """REVERT-PROOF 8. 4000 mm means NO RETURN. Publishing it as a 4 m point would
    tell every consumer the floor is clear to 4 m in exactly the zones that saw
    nothing at all."""
    cfg = TofConfig()
    assert not valid_mm(4000, cfg)
    assert zone_point(3, 3, 4000, cfg) is None
    assert valid_mm(994, cfg)


def test_the_recorded_invalid_values_are_rejected_where_they_are_impossible():
    """The v1.2 invalid-value signature from the recorded baseline: 0 mm, and
    1824-3730 mm IN ROW 5, whose entire geometry tops out under a metre.

    Validity and plausibility are different questions, and the first version of this
    test conflated them: 1824 mm is nonsense in a downward row and perfectly ordinary
    in a row facing a wall, so a global bound would either miss the junk or reject
    real wall readings. The bound that matters is per-zone and geometric.
    """
    cfg = TofConfig()
    assert not valid_mm(0, cfg), "0 mm accepted as a reading anywhere"
    for junk in (1824, 2163, 2251, 2612, 2717, 3185, 3691, 3730):
        assert not plausible_for_zone(5, 3, junk, cfg), (
            f"{junk} mm accepted in a row whose floor lies under a metre away")
        assert plausible_for_zone(1, 3, junk, cfg) or junk > cfg.max_valid_mm, (
            f"{junk} mm rejected in a row that legitimately sees that far")


def test_the_sentinel_is_rejected_for_being_a_SENTINEL():
    """M5 caught this: the sentinel was passing only because 4000 happens to exceed
    max_valid_mm, so the constant that MEANS "no return" was never actually tested.
    Raise the plausible ceiling above it and the sentinel must still be refused.
    """
    loose = TofConfig(max_valid_mm=4500)
    assert not valid_mm(4000, loose), (
        "4000 mm was accepted once the range ceiling stopped hiding it -- the sentinel "
        "check is not doing its job")
    assert valid_mm(3900, loose), "a real 3.9 m reading was refused by the sentinel rule"


# ------------------------------------------------- the floor model and its horizon

def test_the_floor_model_matches_the_measured_floor():
    """The fitted geometry has to reproduce what the floor actually read: 0.26 m in
    row 7 and 0.39 m in row 6, measured in every segment of both sessions."""
    cfg = TofConfig()
    r7 = expected_floor_m(7, 3, cfg)
    r6 = expected_floor_m(6, 3, cfg)
    assert r7 is not None and r6 is not None, "the two floor-returning rows model as blind"
    assert r7 == pytest.approx(0.26, abs=0.06), f"row 7 models at {r7:.3f}, measured 0.26"
    assert r6 == pytest.approx(0.39, abs=0.08), f"row 6 models at {r6:.3f}, measured 0.39"


def test_the_floor_model_accounts_for_AZIMUTH():
    """M3 caught this: dropping cos(az) from the floor model left every test passing,
    because the 0.12 m margin absorbed an 11% error at the frame edge.

    It is still a bug. With the sensor reporting perpendicular distance, a floor point
    at the edge of the frame sits NEARER in z than one straight ahead -- so a
    row-constant model over-predicts the edge columns and calls real floor an
    obstacle. That is precisely what fired on the recorded clear-floor frames before
    the fix, and a margin that merely hides it would hide a real 3 cm obstacle too.
    """
    cfg = TofConfig()
    centre = expected_floor_m(6, 4, cfg)
    edge = expected_floor_m(6, 7, cfg)
    assert centre is not None and edge is not None
    assert edge < centre, (
        "the floor model gives the same distance across a row; with z-reporting the "
        "edge columns' floor is nearer and this ignores azimuth entirely")
    assert centre - edge == pytest.approx(centre * (1 - math.cos(math.radians(3.5 * cfg.zone_deg))),
                                          rel=0.2), "the azimuth term is not cos(az)"


def test_rows_past_the_horizon_are_known_to_be_blind():
    """Row 5's floor lies at ~0.79 m and returned NOTHING in every recorded segment.
    The model must know that -- it is what makes any return there informative."""
    cfg = TofConfig()
    assert floor_beyond_horizon(5, 3, cfg), "row 5 is not marked beyond the horizon"
    assert not floor_beyond_horizon(7, 3, cfg), "row 7 sees floor and must not be"
    assert not floor_beyond_horizon(2, 3, cfg), "a row above the horizon is not floor"


# ------------------------------------------------------ the proofs against the data

def test_floor_is_not_an_obstacle():
    """REVERT-PROOF 3. Replayed CLEAR-FLOOR frames must produce ZERO obstacles.

    This is the proof the first draft of the design FAILED: it claimed any return
    beyond the horizon was an obstacle, while the same data carried a 1.3-1.8%
    baseline of returns on empty floor -- a phantom every ~3 s. Adjacency plus N-of-M
    is what makes this pass, and mutating either one away brings the phantoms back.
    """
    det = ObstacleDetector(TofConfig())
    fired = []
    for i, frame in enumerate(CLEAR_FLOOR):
        result = det.update(frame)
        if result["obstacles"]:
            fired.append((i, result["obstacles"]))
    assert not fired, f"empty floor produced obstacles in frames {fired}"


def test_a_bare_wall_is_not_a_floor_obstacle():
    """The paired negative. A wall at 0.994 m is a real object, but it is not
    something the FLOOR rules should flag -- they exist for sub-lidar obstacles, and
    the lidar sees a wall perfectly well. Flagging it would fire the brake at every
    room boundary."""
    det = ObstacleDetector(TofConfig())
    for i, frame in enumerate(WALL_ONLY):
        assert not det.update(frame)["obstacles"], f"a bare wall flagged in frame {i}"


def test_the_recorded_rail_is_detected():
    """REVERT-PROOF 4. The 5 cm rail at 0.46 m -- the case the sensor was bought for.

    WHAT THIS ASSERTS, AND WHY IT IS NOT "most frames": the rail is detected in
    9 of these 40 consecutive frames (22.5%), not the ~45% the design's table quoted.
    That 45% was measured over a full 30 s segment; this particular 5 s window is
    less favourable, and the difference is exactly the kind of variance a threshold
    picked from a single summary statistic hides. The honest assertion is that the
    rail IS found, repeatedly, and that clear floor is not -- the CONTRAST, which is
    what a detector is for, rather than a rate this fixture cannot pin down.
    """
    cfg = TofConfig()
    det = ObstacleDetector(cfg)
    rail = sum(1 for frame in RAIL_046 if det.update(frame)["obstacles"])
    det_clear = ObstacleDetector(cfg)
    clear = sum(1 for frame in CLEAR_FLOOR if det_clear.update(frame)["obstacles"])
    assert rail > 0, "the recorded rail was never detected at all"
    assert clear == 0, f"clear floor fired {clear} times; the contrast is gone"
    assert rail >= 5, (
        f"the rail fired only {rail}/40 times -- too sparse to brake on, whatever the "
        "contrast")


def test_the_box_at_074_is_NOT_detected_and_that_is_the_honest_result():
    """REVERT-PROOF 5, INVERTED BY THE DATA -- and this is the important test here.

    The design claimed a 5 cm box at 0.74 m would be detected in ~100% of frames,
    against ~20% for presence-thresholding, and called it the central claim. Replaying
    the recorded frames through the rules the design actually permits gives **0 of 40**.

    The claim was not wrong about the data; it was wrong about the RULE. The 100%
    figure came from the box shortening its zone from 992 mm to 892 mm -- a comparison
    against the WALL behind it. That is a learned/persistent background, which the
    same design REJECTS for safety in 3.2(b) with good reason. Rule (i) compares
    against the FLOOR, and at 0.74 m in row 4 there is no floor to compare against;
    rule (ii) needs two adjacent zones, and a 5 cm object at that range spans barely
    one (measured: 4 of 40 frames).

    So the sensor's honest envelope for a 5 cm obstacle is the CLOSE band -- roughly
    0.3-0.5 m, where rule (ii)'s adjacency has something to work with -- and the
    0.74 m result belongs to a rule this design does not use. This test pins that
    limitation so nobody re-derives the optimistic version from the same recording.

    IF THIS TEST EVER FAILS because the box IS detected, that is good news and this
    file is where the reason must be written down.
    """
    det = ObstacleDetector(TofConfig())
    hit = sum(1 for frame in BOX_074 if det.update(frame)["obstacles"])
    assert hit == 0, (
        f"the box at 0.74 m was detected in {hit}/40 frames -- better than the rules "
        "as designed can explain. Find out which rule fired and why before treating "
        "it as capability")


def test_rule_two_alone_would_not_survive_the_baseline():
    """Why the adjacency requirement exists, stated as a test rather than a comment.

    Without adjacency, the beyond-horizon band fires on the recorded empty-floor
    baseline. This asserts the measured contrast the design rests on: adjacent-pair
    frames are rare on clear floor and common with the rail present.
    """
    cfg = TofConfig()
    clear = sum(1 for f in CLEAR_FLOOR if unexpected_returns(f, cfg))
    rail = sum(1 for f in RAIL_046 if unexpected_returns(f, cfg))
    assert rail > clear * 3, (
        f"adjacency does not separate rail ({rail}/{len(RAIL_046)}) from empty floor "
        f"({clear}/{len(CLEAR_FLOOR)}); the discriminator this design rests on is gone")


def test_rule_one_fires_on_a_real_floor_obstruction():
    """Rule (i) is the continuous one WHERE IT APPLIES -- and where it applies is the
    floor rows, not the whole frame.

    Synthetic rather than recorded, deliberately: no segment of the characterisation
    put an obstacle inside rows 6-7's floor band, so there is no recording to replay.
    Saying that out loud is better than dressing a synthetic frame as evidence. A
    5 cm obstacle standing at row 6's floor intersection shortens that zone by about
    0.18 m, comfortably past the 0.12 m margin derived from the measured residuals.
    """
    cfg = TofConfig()
    frame = list(CLEAR_FLOOR[0])
    expected = expected_floor_m(6, 3, cfg)
    for col in (3, 4):
        frame[6 * 8 + col] = int((expected - 0.18) * 1000)
    hits = nearer_than_floor(frame, cfg)
    assert (6, 3) in hits and (6, 4) in hits, (
        f"a 0.18 m shortening in the floor band was not flagged: {hits}")

    det = ObstacleDetector(cfg)
    assert det.update(frame)["obstacles"], "rule (i) did not reach the obstacle output"
    # ...and it fires on the FIRST frame, with no confirmation window. That is the
    # property that makes it usable for a brake, and the reason rule (ii) needs N-of-M
    # while this one does not.
