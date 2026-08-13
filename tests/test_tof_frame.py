"""Revert-proofs for the ToF frame core, replayed against RECORDED frames.

docs/tof_navigation_design.md §7. Every proof here runs against real sensor output
from 2026-08-13 rather than against a synthetic frame I designed to pass -- which
matters more than usual, because the design's central claim (background comparison
beats presence thresholding, 100% vs ~20%) is a claim ABOUT that data. If the claim
is wrong, these fail now, before any of it flies.
"""

import dataclasses
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from fixtures.tof_recorded_frames import (  # noqa: E402
    BOX_074, CLEAR_FLOOR, LEVEL_MOUNT_CFG, RAIL_046, TILT_BOX_050,
    TILT_BOX_050_OBJECT_M, TILT_BOX_050_WALL_M, TILT_CLEAR, TILT_FLOOR_MEDIANS,
    TILT_WALL_060, TILT_WALL_060_ROWS, TILT_WALL_060_SENSOR_M, WALL_ONLY,
)
from sphero_rvr_core.tof_frame import (  # noqa: E402
    _floor_reading_m,
    ObstacleDetector, TofConfig, expected_floor_m, floor_beyond_horizon,
    lidar_disagreement, nearer_than_floor, plausible_for_zone, rule_a_applies,
    rule_a_rows, rule_b_applies, scan_min_by_column, tall_enough_to_matter,
    unexpected_returns, valid_mm,
    zone_angles, zone_point, zone_ray,
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
    cfg = LEVEL_MOUNT_CFG
    az_left, _ = zone_angles(3, 0, cfg)
    az_right, _ = zone_angles(3, 7, cfg)
    assert az_left > 0, "column 0 is not on the LEFT (positive azimuth, REP-103)"
    assert az_right < 0, "column 7 is not on the RIGHT"
    assert az_left == pytest.approx(-az_right, abs=1e-9), "azimuth is not symmetric"

    _, el_top = zone_angles(0, 3, cfg)
    _, el_bottom = zone_angles(7, 3, cfg)
    assert el_top > el_bottom, "row 0 is not UP"
    assert el_top - el_bottom == pytest.approx(math.radians(7 * cfg.zone_deg_v), abs=1e-9)


def test_the_fov_matches_the_measured_box_fill():
    """The 60 deg field was confirmed independently of the datasheet: a 25 cm box at
    0.30 m filled 6 of 8 columns. Six zones must therefore span 0.249 m at 0.30 m."""
    cfg = LEVEL_MOUNT_CFG
    span = 2 * 0.30 * math.tan(math.radians(6 * cfg.zone_deg_h / 2.0))
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
    cfg = LEVEL_MOUNT_CFG
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
    cfg = LEVEL_MOUNT_CFG
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

    ANCHORED ON THE MUTATION, NOT ON A FORMULA. The first version asserted the
    difference equalled `centre * (1 - cos(az))`, which is only true if azimuth enters
    as a lone scale factor. It does not: an off-axis ray rotated by the mount pitch
    also lands at a DIFFERENT WORLD ELEVATION, and that term has the opposite sign.
    Measured through the model -- at the level mount the edge ray descends 0.4 deg more
    steeply and the two terms add (observed 46 mm vs 38 mm predicted); at the tilted
    mount it descends 1.7 deg LESS steeply and they partly cancel (14 mm vs 27 mm).
    A formula fitted at one mount would have failed at the other while the property it
    exists to protect held perfectly, so the assertion is the property instead.
    """
    for tag, cfg in (("level", LEVEL_MOUNT_CFG), ("tilted", TofConfig())):
        centre = expected_floor_m(6, 4, cfg)
        edge = expected_floor_m(6, 7, cfg)
        assert centre is not None and edge is not None
        assert edge < centre, (
            f"[{tag}] the floor model gives the same distance across a row; with "
            "z-reporting the edge columns' floor is nearer and this ignores azimuth")
        flat = dataclasses.replace(cfg, zone_deg_h=0.0)
        assert expected_floor_m(6, 4, flat) == pytest.approx(expected_floor_m(6, 7, flat)), (
            f"[{tag}] columns still differ with the azimuth spread set to zero, so the "
            "spread across a row is coming from something other than azimuth")


def test_rows_past_the_horizon_are_known_to_be_blind():
    """Row 5's floor lies at ~0.79 m and returned NOTHING in every recorded segment.
    The model must know that -- it is what makes any return there informative."""
    cfg = LEVEL_MOUNT_CFG
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
    det = ObstacleDetector(LEVEL_MOUNT_CFG)
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
    det = ObstacleDetector(LEVEL_MOUNT_CFG)
    for i, frame in enumerate(WALL_ONLY):
        assert not det.update(frame)["obstacles"], f"a bare wall flagged in frame {i}"


def test_the_recorded_rail_is_detected():
    """REVERT-PROOF 4. The 5 cm rail at 0.46 m -- the case the sensor was bought for.

    SUPERSEDED BY THE 9.x AMENDMENT, AND INVERTED. This proof used to assert the rail
    WAS detected. It was found by rule (ii) -- "any adjacent returns past the floor
    horizon" -- which the amendment removed, because the same premise fired on every
    wall (99.3% of frames at the tilted mount). With rule (ii) gone the rail at 0.46 m
    is a RULE B case, and rule B needs a lidar background these frames do not carry.

    So this now pins the COST of that trade rather than a capability: without a lidar,
    a 5 cm rail at 0.46 m is not detected by the floor rules alone. The old rule's fire
    count is asserted alongside, so the size of what was given up stays visible instead
    of quietly becoming zero.

    WHAT THE ORIGINAL ASSERTED, AND WHY IT WAS NOT "most frames": the rail is detected in
    9 of these 40 consecutive frames (22.5%), not the ~45% the design's table quoted.
    That 45% was measured over a full 30 s segment; this particular 5 s window is
    less favourable, and the difference is exactly the kind of variance a threshold
    picked from a single summary statistic hides. The honest assertion is that the
    rail IS found, repeatedly, and that clear floor is not -- the CONTRAST, which is
    what a detector is for, rather than a rate this fixture cannot pin down.
    """
    cfg = LEVEL_MOUNT_CFG
    det = ObstacleDetector(cfg)
    rail = sum(1 for frame in RAIL_046 if det.update(frame)["obstacles"])
    det_clear = ObstacleDetector(cfg)
    clear = sum(1 for frame in CLEAR_FLOOR if det_clear.update(frame)["obstacles"])
    assert clear == 0, f"clear floor fired {clear} times"
    assert rail == 0, (
        f"the rail fired {rail}/40 times with NO lidar background -- which the rules as "
        "amended cannot explain. Find out which rule fired before treating it as "
        "capability")
    # The old rule (ii) is what used to catch it, and it is still measurable here --
    # this is the capability that was traded away, quantified rather than asserted.
    old_rule_two = sum(1 for f in RAIL_046 if unexpected_returns(f, cfg))
    assert old_rule_two > 0, (
        "the removed rule (ii) does not fire on the rail either, so this fixture no "
        "longer documents the trade it was kept for")


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
    det = ObstacleDetector(LEVEL_MOUNT_CFG)
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
    cfg = LEVEL_MOUNT_CFG
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
    cfg = LEVEL_MOUNT_CFG
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


# ------------------------------------------- the tilted mount, pinned by TODAY's data

def _rms_mm(cfg, measurements):
    from sphero_rvr_core.tof_frame import _floor_reading_m
    errs = [(_floor_reading_m(r, c, cfg) - m) * 1000.0 for r, c, m in measurements]
    return math.sqrt(sum(e * e for e in errs) / len(errs)), errs


def test_the_vertical_zone_pitch_is_59_not_75():
    """REVERT-PROOF 15. The field of view is ASYMMETRIC and the data says so.

    Eight clear-floor medians from 4321 frames of the 2026-08-13 tilt session. The
    shipped geometry reproduces them at ~6 mm RMS; forcing the vertical zone pitch to
    equal the horizontal one gives ~31 mm with residuals that are ALL THE SAME SIGN.

    The sign test is the real assertion. A big RMS could be noise or a bad room; eight
    one-signed residuals cannot be either, and that is what distinguishes a wrong model
    from a noisy measurement. Anyone tempted to "simplify" this back to one zone angle
    has to explain that pattern away first.
    """
    good, _ = _rms_mm(TofConfig(), TILT_FLOOR_MEDIANS)
    bad, bad_errs = _rms_mm(dataclasses.replace(TofConfig(), zone_deg_v=7.5),
                            TILT_FLOOR_MEDIANS)
    assert good < 12.0, f"the shipped geometry misses the recorded floor by {good:.1f} mm RMS"
    assert bad > 3 * good, (
        f"a symmetric 7.5 deg field fits the recorded floor about as well ({bad:.1f} mm "
        f"vs {good:.1f} mm); the asymmetry this constant encodes is not visible in the data")
    assert all(e < 0 for e in bad_errs), (
        f"the symmetric model's residuals are not one-signed ({bad_errs}) -- without "
        "that pattern this is a noise argument, not a model-error argument")


def test_the_sensor_reports_along_its_own_BORESIGHT():
    """The projection convention, pinned against the wall that settled it.

    A flat wall is a plane, so its row-to-row gradient depends only on the pitch and the
    vertical zone pitch -- not on the mount height, which is the degenerate parameter in
    the floor fit. Rows 0-3 facing a wall at tape 0.60 m read 541/558/576/594 mm: the
    reading GROWS as the rows point further down, by exactly the ratio between the two
    projections.

    Projecting onto base_link x instead (what this module did until 2026-08-13) predicts
    a gradient of the WRONG SHAPE. At the old +4 deg mount the two agreed to a fraction
    of a percent, which is why the defect shipped.
    """
    cfg = TofConfig()
    d = [m for _, m in TILT_WALL_060_ROWS]
    assert d == sorted(d), "the recorded wall gradient is not monotonic; refit before trusting it"

    from sphero_rvr_core.tof_frame import _boresight_cosine, zone_ray
    # One free parameter -- the wall's distance from the sensor -- solved from row 0,
    # then the remaining three rows are PREDICTIONS with nothing left to tune.
    def predict(row, wall_m):
        ux = zone_ray(row, 3, cfg)[0]
        return wall_m / ux * _boresight_cosine(row, 3, cfg)
    wall_m = TILT_WALL_060_ROWS[0][1] / predict(0, 1.0)
    errs = [(predict(r, wall_m) - m) * 1000.0 for r, m in TILT_WALL_060_ROWS]
    assert max(abs(e) for e in errs) < 6.0, (
        f"the boresight projection mispredicts the recorded wall gradient by {errs} mm")
    assert 0.50 < wall_m < 0.62, (
        f"solved wall distance {wall_m:.3f} m is nowhere near the 0.60 m tape")


def test_the_mount_pitch_is_a_RIGID_rotation():
    """Pitch moves the frame; it must not reshape it.

    Adding the mount pitch to each zone's elevation -- the obvious shortcut, and what
    this module did until 2026-08-13 -- is not a rotation. It is exact on the centre
    column and wrong by 1.8 deg at the corner zone at the shipped -15.7 deg, which
    moves that zone's floor intersection by ~5%: the same size as the margin that is
    supposed to be absorbing sensor noise.

    NOT ANCHORED IN DATA, AND SAYING SO. The tilt session's edge columns cannot settle
    it: across nominally flat floor row 6 reads 247/267/282/271/267/261/252/243 mm, a
    ~35 mm spread that swamps the ~10 mm the two conventions differ by. So this asserts
    a geometric invariant instead -- a rigid rotation preserves the angle between any
    two rays -- which is weaker evidence than a measurement and is the honest strength
    available. If a future session gets clean edge columns, replace this with them.
    """
    def angle_between(cfg, a, b):
        u, v = zone_ray(*a, cfg), zone_ray(*b, cfg)
        return math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(u, v)))))

    pairs = [((0, 0), (7, 7)), ((0, 7), (7, 0)), ((3, 0), (3, 7)), ((0, 4), (7, 4))]
    base = TofConfig(mount_pitch_deg=0.0)
    for pitch in (-30.0, -15.7, -4.0, 4.0, 20.0):
        tilted = dataclasses.replace(TofConfig(), mount_pitch_deg=pitch)
        for a, b in pairs:
            assert angle_between(tilted, a, b) == pytest.approx(
                angle_between(base, a, b), abs=1e-9), (
                f"zones {a} and {b} are {math.degrees(angle_between(tilted, a, b)):.2f} deg "
                f"apart at pitch {pitch} but "
                f"{math.degrees(angle_between(base, a, b)):.2f} deg at pitch 0 -- the mount "
                "angle is being applied as a shear, not a rotation")

    assert zone_ray(7, 7, TofConfig()) == pytest.approx(
        zone_ray(7, 7, TofConfig()), abs=0), "ray is not deterministic"
    for row, col in ((0, 0), (3, 4), (7, 7)):
        n = math.sqrt(sum(x * x for x in zone_ray(row, col, TofConfig())))
        assert n == pytest.approx(1.0, abs=1e-9), f"zone ({row},{col}) ray is not a unit vector"


# ------------------------------- amendment 9.x: rule A's bound and rule B's background
#
# These replay TILTED-mount frames -- the shipped geometry. No lidar scan was recorded
# with them, so every background here is either a MEASURED distance or an explicitly
# stated hypothetical. Bench item J (design 9.8) is what removes that asterisk.

def test_row_two_does_not_brake_for_a_wall():
    """REVERT-PROOF 9. The defect that forced the amendment, replayed.

    At the tilted mount the shipped detector reported an obstacle in 99.3% of frames,
    rule (i) never fired, and the trigger was ROW 2 SEEING THE WALL. Row 2's floor moved
    to 1.16 m, so "beyond the floor horizon, therefore nothing should return here" --
    true of a row whose floor is 0.30 m away -- became false, and a wall obliges.

    Two independent reasons it is now zero, and the test asserts both because they fail
    separately: rule A has no authority in row 2 (its floor is far outside the stop
    distance), and rule B agrees with the lidar about where the wall is.
    """
    cfg = TofConfig()
    assert not any(rule_a_applies(2, c, cfg) for c in range(8)), (
        "row 2 holds rule A authority at the shipped stop distance; its floor is 1.16 m "
        "away and the comparison there cannot tell a wall from an obstacle")

    background = [TILT_WALL_060_SENSOR_M] * 8
    det = ObstacleDetector(cfg)
    fired = [(i, r["obstacles"]) for i, r in
             ((i, det.update(f, background)) for i, f in enumerate(TILT_WALL_060))
             if r["obstacles"]]
    assert not fired, f"a bare wall at the tilted mount still produces obstacles: {fired}"

    # ...and the OLD rule fires on these exact frames. Without this line the proof
    # passes against code that simply detects nothing at all.
    assert sum(1 for f in TILT_WALL_060 if unexpected_returns(f, cfg)) > 20, (
        "the removed rule (ii) does not fire on these wall frames, so this fixture no "
        "longer reproduces the defect it was chosen for")


def test_clear_floor_at_the_tilted_mount_is_not_an_obstacle():
    """REVERT-PROOF 3, re-run at the shipped geometry rather than the level one.

    With and without a lidar: the floor must never be an obstacle, and rule B must not
    invent one out of the floor either. The first version of rule B did exactly that --
    it compared every return against the lidar, and the floor is always nearer than the
    wall the lidar sees, so it fired in 38 of 40 clear-floor frames.
    """
    for background, tag in ((None, "no lidar"),
                            ([TILT_WALL_060_SENSOR_M] * 8, "lidar sees the wall")):
        det = ObstacleDetector(TofConfig())
        fired = [i for i, f in enumerate(TILT_CLEAR) if det.update(f, background)["obstacles"]]
        assert not fired, f"[{tag}] clear floor produced obstacles in frames {fired}"


def test_rule_a_row_set_follows_the_stop_distance():
    """REVERT-PROOF 10. Which rows may conclude from the floor is COMPUTED, never listed.

    A frozen row list is precisely how `floor_horizon_m` ended up enforcing an authority
    boundary nobody chose for it. Shortening the stop distance must shrink the set;
    lengthening it must grow it.
    """
    cfg = TofConfig()
    rows = {d: rule_a_rows(dataclasses.replace(cfg, stop_distance_m=d))
            for d in (0.25, 0.35, 0.45, 0.70, 1.20)}
    for a, b in ((0.25, 0.35), (0.35, 0.45), (0.45, 0.70), (0.70, 1.20)):
        assert set(rows[a]) < set(rows[b]), (
            f"stop distance {a} -> rows {rows[a]} and {b} -> rows {rows[b]}: the set is "
            "not growing with the distance, so it is not being computed from it")
    # Anchored on the two rows the amendment argues about by name: row 3's floor reads
    # 0.63 m and row 2's 1.16 m, so each is admitted only once the stop distance passes
    # its own floor -- which is the whole content of the bound.
    assert 3 not in rows[0.45] and 3 in rows[0.70], (
        f"row 3 (floor 0.63 m) is admitted at 0.45 m: {rows[0.45]}")
    assert 2 not in rows[0.70] and 2 in rows[1.20], (
        f"row 2 (floor 1.16 m) is admitted at 0.70 m: {rows[0.70]}")


def test_rule_b_needs_the_lidar_to_DISAGREE():
    """REVERT-PROOF 11. The 5 cm object at 0.50 m, in real recorded frames.

    The whole rule in one comparison. Same forty frames, same object, three different
    things the lidar might be saying:

        lidar sees the wall behind it (0.678 m)  -> DISAGREEMENT -> obstacle
        lidar sees the object itself (0.500 m)   -> AGREEMENT    -> not our class
        lidar sees open space                    -> DISAGREEMENT -> obstacle

    A detector using absolute proximity instead would fire identically in all three,
    which is the failure this rule exists to avoid: braking at every wall.

    OPTS IN EXPLICITLY. Rule B ships GATED OFF until bench item J pins its margin, so a
    proof of rule B's logic has to enable it and say so -- inheriting the shipped default
    would leave this quietly testing nothing the day the gate flipped either way.
    """
    cfg = dataclasses.replace(TofConfig(), rule_b_enable=True)

    def fires(background):
        det = ObstacleDetector(cfg)
        return sum(1 for f in TILT_BOX_050 if det.update(f, background)["obstacles"])

    disagree = fires([TILT_BOX_050_WALL_M] * 8)
    agree = fires([TILT_BOX_050_OBJECT_M] * 8)
    open_space = fires([float("inf")] * 8)

    assert disagree > 30, (
        f"the recorded object fired only {disagree}/40 when the lidar reported the wall "
        f"behind it ({TILT_BOX_050_WALL_M} m vs the object at {TILT_BOX_050_OBJECT_M} m)")
    assert agree == 0, (
        f"the object fired {agree}/40 while the lidar reported the SAME range -- rule B "
        "is thresholding on proximity, not on disagreement, and will brake at walls")
    assert open_space > 30, (
        f"a bearing the lidar reports as open produced only {open_space}/40 -- that is "
        "rule B's strongest case and it must not be its weakest")


def test_rule_a_alone_does_not_reach_the_recorded_object():
    """The layering argument of 9.4, as a measurement rather than a claim.

    The object sits in row 3, whose floor reads 0.63 m -- outside rule A's bound at the
    shipped stop distance. With no lidar the sensor sees it perfectly and the detector
    cannot say so. That is the band rule B exists to cover, and it is why the two rules
    are not redundant.
    """
    det = ObstacleDetector(TofConfig())
    assert sum(1 for f in TILT_BOX_050 if det.update(f, None)["obstacles"]) == 0, (
        "rule A reached an object outside its applicability bound; either the bound is "
        "not being enforced or 9.4's argument for the layering needs rewriting")


def test_missing_scan_drops_rule_b_zones():
    """REVERT-PROOF 12. No scan is UNAVAILABLE -- neither obstacle nor clearance.

    Both wrong answers are named in the assertion because they fail in opposite
    directions: treating a missing scan as OPEN SPACE brakes on everything, and treating
    it as AGREEMENT brakes on nothing while looking healthy.
    """
    cfg = dataclasses.replace(TofConfig(), rule_b_enable=True)   # see the note above
    det = ObstacleDetector(cfg)
    result = det.update(TILT_BOX_050[0], None)
    assert result["disagrees_this_frame"] == []
    assert result["confirmed_disagreement"] == []
    assert result["background"] == "unavailable", (
        "a missing scan is not reported on the state output, so a consumer cannot tell "
        "a degraded detector from a quiet one")

    # Run past the N-of-M window, or both answers are empty for the trivial reason that
    # nothing has been confirmed yet -- which would pass against a detector that treats
    # the two identically.
    def run(background):
        det = ObstacleDetector(cfg)
        return [det.update(f, background)["obstacles"] for f in TILT_BOX_050][-1]

    assert run(None) == [], "a missing scan produced obstacles"
    assert run([float("inf")] * 8), (
        "a scan reporting OPEN SPACE produced no obstacles over 40 frames, so this test "
        "cannot distinguish it from the missing-scan case it exists to separate")
    assert ObstacleDetector(cfg).update(TILT_BOX_050[0], [float("inf")] * 8)[
        "background"] == "ok"


def test_row_zero_stops_applying_above_the_lidar_plane():
    """REVERT-PROOF 13. Rule B is for what the lidar CANNOT see.

    Row 0 rises. Below ~0.59 m its ray is under the lidar plane and a disagreement means
    a sub-lidar object; beyond that the lidar can see whatever row 0 sees, so a
    disagreement means occlusion geometry and the rule must go quiet. Rows 1-7 descend
    from a mount 51 mm below the plane and never leave it.
    """
    cfg = TofConfig()
    assert rule_b_applies(0, 3, 0.40, cfg), "row 0 is excluded even close in"
    assert not rule_b_applies(0, 3, 1.50, cfg), (
        "row 0 still claims sub-lidar authority at 1.5 m, where its ray is well above "
        "the lidar's own scan plane")
    crossing = [r for r in [x / 100.0 for x in range(20, 150)]
                if not rule_b_applies(0, 3, r, cfg)]
    assert 0.50 < min(crossing) < 0.70, f"row 0 crosses the plane at {min(crossing):.2f} m"
    for row in range(1, 8):
        assert rule_b_applies(row, 3, 2.0, cfg), (
            f"row {row} descends from below the lidar plane and cannot rise above it")


def test_the_lidar_background_is_read_by_BEARING_not_by_index():
    """REVERT-PROOF 14. The N1 lesson, pre-empted for the second time in this file.

    The RPLIDAR's `base_link->laser` yaw is ~179 deg, so raw scan index order is very
    nearly the mirror image of robot bearings. `scan_min_by_column` takes bearings and
    believes them; feeding it the RAW bearings instead of the transformed ones must put
    the obstacle on the wrong side, which is exactly what a caller would then see and
    catch. An implementation that bucketed by index would show no difference at all --
    and would be silently mirrored forever.
    """
    cfg = TofConfig()
    left_bearing = math.radians(22.0)                 # rover-LEFT, REP-103 positive
    bearings = [left_bearing, -left_bearing]
    ranges = [0.40, 3.00]                             # something close on the LEFT only

    got = scan_min_by_column(bearings, ranges, cfg)
    near = [c for c, v in enumerate(got) if v < 1.0]
    assert near and max(near) < 4, (
        f"a return at +22 deg (rover-left) landed in columns {near}; column 0 is LEFT")

    mirrored = scan_min_by_column([-b for b in bearings], ranges, cfg)
    near_m = [c for c, v in enumerate(mirrored) if v < 1.0]
    assert near_m and min(near_m) > 3, (
        f"mirroring the bearings did not move the return to the right ({near_m}) -- the "
        "background is being bucketed by index, so the ~179 deg laser yaw would be "
        "silently baked in")

    empty = scan_min_by_column([], [], cfg)
    assert all(v == float("inf") for v in empty), (
        "a scan with no returns in a column must read as OPEN TO MAX RANGE (inf), which "
        "is rule B's strongest case -- not as zero, which would suppress it entirely")


def test_rule_a_authority_is_INDEPENDENT_of_the_floor_horizon():
    """REVERT-PROOF 10b, and it exists because proof 10's first version was INERT.

    A mutation deleting rule A's authority bound outright changed no test. The bound
    was real code doing nothing measurable: at the shipped 0.45 m stop distance the
    0.55 m visibility horizon was already stricter, so `expected_floor_m` returning
    None past the horizon was still making the authority decision -- the very confusion
    9.1 diagnosed, surviving inside the fix for it. (Same shape as the pivot controller
    wired below an unreachable branch, and the explorer's unstick that never fired.)

    Separating them means the two constants must be independently observable. Raise the
    stop distance ABOVE the horizon and rule A must gain rows; move the horizon with the
    stop distance fixed and it must gain none.
    """
    cfg = TofConfig()
    assert cfg.stop_distance_m < cfg.floor_horizon_m, (
        "the shipped constants no longer make this test meaningful; pick a stop "
        "distance either side of the horizon and re-derive what it should prove")

    wide = dataclasses.replace(cfg, stop_distance_m=0.70)
    assert set(rule_a_rows(wide)) > set(rule_a_rows(cfg)), (
        f"raising the stop distance past the {cfg.floor_horizon_m} m horizon gained no "
        f"rows ({rule_a_rows(wide)} vs {rule_a_rows(cfg)}) -- the horizon is still "
        "deciding who may conclude")

    frame = list(TILT_BOX_050[0])
    assert nearer_than_floor(frame, wide), (
        "with authority extended past the horizon the recorded object is still not "
        "flagged, so the bound is not what gates rule A")

    for horizon in (0.40, 0.55, 2.00):
        moved = dataclasses.replace(cfg, floor_horizon_m=horizon)
        assert rule_a_rows(moved) == rule_a_rows(cfg), (
            f"moving the VISIBILITY horizon to {horizon} changed rule A's authority "
            f"({rule_a_rows(moved)} vs {rule_a_rows(cfg)}); the two are still coupled")


# ------------------------------------------- Scott's traversable-terrain height gate

def test_a_mat_ridge_is_terrain_not_an_obstacle():
    """Scott, 2026-08-13, verbatim: "Anything smaller than say 0.7\" should be ignored."

    He named the artifact too -- the ridge of the mat his chair sits on, ~3/5 inch. A
    detector that brakes for that turns every threshold strip and floor mat in the house
    into a wall, and the rover drives over them without noticing.

    The gate is asserted in BOTH directions, because a gate that only ever passes is
    indistinguishable from no gate at all.
    """
    cfg = TofConfig()
    floor_mm = int(_floor_reading_m(5, 3, cfg) * 1000)

    # A 15 mm ridge: nearer than modelled floor, but terrain.
    for height_m, verdict in ((0.005, False), (0.012, False), (0.015, False),
                              (0.025, True), (0.050, True)):
        reading = None
        for mm in range(floor_mm, 100, -1):
            p = zone_point(5, 3, mm, cfg)
            if p and p[2] >= height_m:
                reading = mm
                break
        assert reading is not None, f"no reading in row 5 reaches {height_m} m"
        assert tall_enough_to_matter(5, 3, reading, cfg) is verdict, (
            f"a return {height_m*1000:.0f} mm off the floor (reading {reading} mm) was "
            f"judged {'terrain' if verdict else 'an obstacle'} -- the 18 mm gate is "
            "not where the spec puts it")


def test_the_height_gate_binds_on_BOTH_rules():
    """A ridge is nearer than modelled floor AND invisible to the lidar, so it satisfies
    rule A and rule B alike. Gating one would leave the other braking for the same mat.
    """
    cfg = TofConfig()
    frame = [4000] * 64
    # A 12 mm ridge across three adjacent columns of a rule-A row and a rule-B row.
    for row in (5, 3):
        for col in (2, 3, 4):
            mm = None
            for cand in range(int(_floor_reading_m(row, col, cfg) * 1000), 100, -1):
                p = zone_point(row, col, cand, cfg)
                if p and p[2] >= 0.012:
                    mm = cand
                    break
            frame[row * 8 + col] = mm

    assert not nearer_than_floor(frame, cfg), (
        "rule A flagged a 12 mm ridge; the height gate is not applied to the floor "
        "comparison")
    assert not lidar_disagreement(frame, cfg, [3.0] * 8), (
        "rule B flagged a 12 mm ridge against an open lidar background; the height gate "
        "is not applied to the lidar comparison, so a mat still brakes the rover")

    # ...and the SAME frame with a 60 mm object must still fire, or the test above is
    # only proving the detector is broken.
    for col in (2, 3, 4):
        for cand in range(int(_floor_reading_m(5, col, cfg) * 1000), 100, -1):
            p = zone_point(5, col, cand, cfg)
            if p and p[2] >= 0.060:
                frame[5 * 8 + col] = cand
                break
    assert nearer_than_floor(frame, cfg), (
        "a 60 mm obstacle was gated out as terrain -- the gate is set far too high")


def test_the_height_gate_is_NOT_redundant_with_the_floor_margin():
    """The gate must bind on its OWN terms, and this proof exists because it did not.

    Wired naively, deleting the 18 mm gate from either rule changed no test. The reason
    is that `floor_margin_m = 0.12` already refuses anything shorter than 27-76 mm
    depending on the row -- up to 4x Scott's spec. The margin was doing terrain
    classification by accident.

    That is the same confusion as `floor_horizon_m` making authority decisions: two
    different questions answered by one constant, agreeing today and diverging later.
    They diverge on a schedule we already know about -- the margin exists to cover
    FLOOR-MODEL UNCERTAINTY and design 3.3 requires it to be RE-DERIVED once the
    in-flight pitch envelope is measured. If that shrinks it, a stack relying on the
    margin for terrain would silently start braking for mats, and the symptom would be
    a rover that got MORE timid after the floor model got BETTER.

    So the assertion is made where the two constants come apart.
    """
    cfg = TofConfig()
    tight = dataclasses.replace(cfg, floor_margin_m=0.02)

    frame = [4000] * 64
    for col in (2, 3, 4):
        for cand in range(int(_floor_reading_m(5, col, cfg) * 1000), 100, -1):
            p = zone_point(5, col, cand, cfg)
            if p and p[2] >= 0.012:                  # a 12 mm mat ridge
                frame[5 * 8 + col] = cand
                break

    assert nearer_than_floor(frame, dataclasses.replace(tight, min_obstacle_height_m=0.0)), (
        "with the margin tightened and the gate disabled, a 12 mm ridge is STILL not "
        "flagged -- so this fixture cannot show the gate doing anything and the proof "
        "below would be vacuous")
    assert not nearer_than_floor(frame, tight), (
        "with the margin tightened to 0.02 m the 12 mm ridge is flagged as an obstacle: "
        "the 18 mm gate is not binding on its own, it was only ever hiding behind the "
        "margin, and improving the floor model would start braking for floor mats")

    assert not lidar_disagreement(frame, tight, [3.0] * 8), (
        "same ridge, same tightened margin, against an open lidar background: rule B "
        "flagged it, so the gate is not binding there either")
