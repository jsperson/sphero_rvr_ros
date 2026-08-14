"""The 8x8 ToF frame: geometry, validity, and what counts as an obstacle.

Design and evidence: docs/tof_navigation_design.md, and the 12,869 recorded frames in
the vault at 03_validation/sensor_2026-08-13_tof_characterisation/.

Pure core: no ROS, no I2C, no motion. A frame of 64 millimetre readings goes in; zone
geometry and obstacle conclusions come out. Everything here is replayable against the
recorded CSVs, which is the whole reason it is not inside the node.

THE ONE IDEA THIS MODULE EXISTS FOR: a marginal obstacle is intermittent if you ask
"did anything return" (~20% of frames, measured) and continuous if you ask "is this
zone NEARER THAN IT SHOULD BE" (100%, measured on a 5 cm box at 0.74 m). So detection
is comparison against an expected background, not a presence threshold.
"""

from dataclasses import dataclass
from typing import Optional
import math

ZONES = 8
N_ZONES = ZONES * ZONES


@dataclass(frozen=True)
class TofConfig:
    """Geometry and thresholds. Every value is derived from the robot, the deployed
    config or a measurement in the characterisation -- none is tuned for this room."""

    # --- geometry ---------------------------------------------------------------
    # THE FIELD OF VIEW IS ASYMMETRIC, and assuming otherwise was a real model error.
    #
    # Horizontal, 7.5 deg/zone = 60 deg total. Confirmed INDEPENDENTLY of the
    # datasheet: a 25 cm box at 0.30 m filled 6 of 8 columns, and 6/8 of 60 deg spans
    # 0.249 m at that range.
    zone_deg_h: float = 7.5
    # Vertical, 5.9 deg/zone = ~47 deg total. FITTED 2026-08-13 from the tilted-mount
    # floor rows: 5.9 reproduces eight clear-floor medians at 5.7 mm RMS, while forcing
    # 7.5 gives 31.4 mm with residuals that are systematically one-signed (-24 -14 -27
    # -25 -39 -35 -44 -33). A one-signed residual pattern is a model error, not noise,
    # which is what makes this a measurement rather than a preference.
    zone_deg_v: float = 5.9
    # Mount, JOINTLY FITTED 2026-08-13 (floor rows + the 0.60 m wall gradient).
    #
    # THESE ARE CALIBRATION CONSTANTS, NOT A DESCRIPTION OF THE MOUNT. They are the
    # numbers that make the model predict the readings; the fitted height sits ~30 mm
    # ABOVE Scott's tape measure. The likely mechanism is that each zone reports the
    # NEAREST returning part of its cone rather than its centre ray, which biases every
    # downward row short and a fit absorbs that bias into height and pitch. Quoted as
    # "the sensor is 13.9 cm up" they are wrong; used to predict a reading they are
    # right to +-6 mm. Re-mounting invalidates them even if the tape is unchanged.
    mount_height_m: float = 0.139
    mount_pitch_deg: float = -15.7      # positive = nose UP
    # FORWARD OFFSET OF THE SENSOR FROM base_link, and the reason this module now has a
    # single source of truth for position. It existed only as a node parameter feeding
    # the static TF; `zone_point` -- which builds the clouds PUBLISHED IN base_link --
    # never applied it, so every point was 0.10 m nearer to base_link than the thing it
    # represented, and every consumer of those points inherited the error.
    #
    # Found by bench item J on 2026-08-13, in its first twenty seconds and before the
    # wall was even staged: the ToF read a MEDIAN 0.10 m nearer than the lidar on the
    # same surfaces, CONSTANT from 0.72 m to 1.90 m. A constant offset across a 2.6x
    # range change is a missing translation; a sensor artifact would have scaled with
    # range. Data: 03_validation/escape_provocation_2026-08-13/J_probe2.csv.
    #
    # Unlike `mount_height_m` and `mount_pitch_deg` above, this is NOT a fitted
    # calibration value -- it is the same physical translation the static TF publishes,
    # and the two must move together. A remount changes both.
    mount_x_m: float = 0.10
    # The sensor reports distance along ITS OWN BORESIGHT, not along base_link x.
    # SETTLED 2026-08-13 by pre-registering three hypotheses and measuring at two wall
    # distances; see zone_angles_sensor for why the level mount could not have answered
    # it and why getting it backwards cost 30 mm at the tilted mount.
    reports_z: bool = True

    # --- validity ---------------------------------------------------------------
    # 4000 mm is the no-return sentinel. INFERRED from behaviour (it appears as that
    # exact value in 45 of 64 zones and sits above the rated 3.5 m range), NOT from
    # documentation -- confirm with DFRobot. If it turns out to be a real measurement,
    # this constant is the one line that changes.
    sentinel_mm: int = 4000
    # Physically impossible readings seen in the recorded baseline: 0, and 1824-3730
    # in rows whose entire geometry is under a metre. That is the invalid-value
    # signature the v1.2 firmware note warns about. Rejecting them is free.
    min_valid_mm: int = 60
    max_valid_mm: int = 3500

    # --- detection --------------------------------------------------------------
    # Rule (i): a zone reading nearer than its expected background by more than this.
    # Range noise is a few mm; the rest is floor-model uncertainty, and it MUST grow
    # to cover the measured in-flight pitch envelope before this drives a brake
    # (design note 3.3 -- a transient nose-down under braking moves every floor
    # intersection nearer, which is a feedback loop into phantom stops).
    # DERIVED, 2026-08-14, from 1058 empty-floor zone readings across the clear-floor
    # and wall-only segments: the residual (measured - model) runs -0.081 to +0.639 m,
    # p1 = -0.056, median +0.017. A margin of 0.08 fired on the -0.081 excursion, so
    # 0.12 clears the measured worst case with 40 mm to spare. It still catches what
    # matters: a 5 cm obstacle standing in row 6 shortens that zone by ~0.18 m, which
    # is 1.5x the margin.
    # THIS NUMBER IS NOT FINAL. It covers STATIC error only; the in-flight pitch
    # envelope (design note 3.3) has never been measured, and a transient nose-down
    # under braking moves every floor intersection nearer. Stage (ii) supplies that
    # envelope and this constant is re-derived before the brake ever consumes it.
    floor_margin_m: float = 0.12
    # Rule (ii): beyond the floor-visibility horizon nothing should return at all, so
    # a return is informative -- but the recorded baseline is 1.3-1.8% of frames, not
    # zero. Adjacency is what separates signal from that: >=2 ADJACENT zones in one
    # frame ran 45.2% with a rail present against 1.3-1.8% empty, a 25x separation
    # from a single frame with no memory.
    min_adjacent_zones: int = 2
    # ...then N of the last M frames. M=8, N=3 at 7.6 Hz: one false fire per ~50 min,
    # 78% detection per window, 1.05 s of window = 0.21 m at cruise. (F) -- recompute
    # after the v1.3 firmware retest rather than re-arguing.
    confirm_frames: int = 3
    window_frames: int = 8
    # Where the floor stops being visible: measured at 0.39 m yes, 0.79 m no.
    # THIS IS A VISIBILITY CONSTANT AND NOTHING ELSE. Until 2026-08-13 both detection
    # rules also used it to decide which rows were ALLOWED to conclude an obstacle,
    # which is a different question -- and at the level mount the two questions had the
    # same answer, so nothing said otherwise. Tilting the mount separated them and the
    # detector reported an obstacle in 99.3% of frames. Authority now belongs to
    # `stop_distance_m` below (design note 9.1-9.2).
    floor_horizon_m: float = 0.55

    # --- rule A: where the floor comparison is ALLOWED to conclude -----------------
    # A zone may use the floor comparison only where its floor lies INSIDE this
    # distance. The justification is the design's own rule that a brake may only use
    # models whose errors make it stop sooner: inside this bound, "an object" and "the
    # sub-lidar part of a wall" are both things we are about to stop for, so the
    # comparison cannot tell them apart AND DOES NOT NEED TO. Outside it the two
    # explanations demand opposite actions and rule A is not entitled to choose.
    # DERIVED 2026-08-13 IN THE TRUE FRAME, and it is a `base_link` GROUND RANGE --
    # compared against `floor_ground_range_m`, never against a sensor reading. It was
    # previously compared against `_floor_reading_m`, which is the shorter number, so
    # the bound admitted rows whose floor is further from the robot than it looks. The
    # error ran in the PERMISSIVE direction: it granted authority the geometry did not
    # support, to the one rule that cannot tell an object from a wall.
    #
    # THE OPERAND, NAMED. Rule A may conclude only where the two explanations of a
    # return -- an object, or the sub-lidar part of a wall -- demand the same action.
    # They do exactly where a WALL is already commanding a stop from another authority,
    # and that authority is the lidar supervisor's own `stop_distance_m: 0.30` in
    # config/collision_stop.yaml, which is a base_link distance (the scan is
    # transformed to base_link at collision_stop.py before any range comparison).
    #
    # THIS IS NOT THE RETRACTED COMPARISON OF DESIGN 11.2, and the difference is the
    # question being asked. 11.2's error was measuring rule A's REACH against 0.30 as
    # though 0.30 were the distance in which the rover must physically stop -- a
    # threshold standing in for a requirement. Here the question IS about a threshold:
    # "at what range does the other brake already act?" The physical stop requirement
    # (footprint + payload + braking) would be the wrong operand for that.
    #
    # THE ARITHMETIC. Rule A's reach per row, in base_link, at floor - floor_margin:
    #     row 4  0.397 m   > 0.30  -- EXCLUDED
    #     row 5  0.297 m  <= 0.30  -- admitted (3.3 mm inside)
    #     row 6  0.229 m           -- admitted
    #     row 7  0.185 m           -- admitted
    # The bound is expressed on the floor's ground range, so admitting {5,6,7} and
    # excluding 4 requires a value in [0.421, 0.5125) -- row 5's furthest floor and
    # row 4's nearest. 0.45 lies inside that window with 0.029 m below and 0.062 m
    # above, so the SHIPPED NUMBER DOES NOT MOVE; what changed is that it now has an
    # operand instead of a provenance of "provisional".
    #
    # Row 5 clears the bound by 3.3 mm and that is tolerable HERE for a reason that
    # does not generalise: exceeding it means braking early for a wall the lidar has
    # not yet stopped for, which is over-caution. Design 9.4's 9 mm was a DETECTION
    # claim, where the thin side is a miss. Thin margins are judged by which way they
    # fail, not by their width.
    #
    # Still PROVISIONAL in one respect: `floor_margin_m` below is static-only, and the
    # reach above is computed from it, so the in-flight pitch envelope (design 3.3)
    # can still move this. Which rows it admits stays a FUNCTION of this number and is
    # published on ~/state rather than hardcoded.
    stop_distance_m: float = 0.45

    # --- traversable terrain: what is NOT an obstacle ------------------------------
    # SCOTT'S REQUIREMENT, 2026-08-13, verbatim: "Anything smaller than say 0.7\" should
    # be ignored." A ridge that low is terrain the rover drives over, not an obstacle,
    # and a detector that brakes for it turns every floor mat and threshold strip into
    # a wall.
    #
    # 0.7 inch = 17.8 mm; 18 mm is the round number and the one that ships. WHICH NUMBER
    # WINS AND WHY: this is an OPERATOR SPEC, not a derivation from tread geometry. No
    # documented RVR climb capability exists in this repo or in Sphero's published
    # material, so there is nothing to derive from -- and a number invented from wheel
    # radius would be a guess wearing a derivation's clothes. If a real climb spec ever
    # surfaces and it is LOWER than 18 mm, the spec wins and this drops to it; if it is
    # higher, this stays at 18 mm, because being able to climb a thing is not a reason
    # to drive into it.
    #
    # Measured against the sensor rather than assumed: at the shipped geometry a return
    # 18 mm off the floor is ~15 mm of range difference in the near rows, which is
    # several times the ~3 mm range noise -- so this gate is resolvable, not wishful.
    # THE CAMERA COULD NEVER HAVE IMPLEMENTED THIS. A monocular floor-boundary reports
    # WHERE the floor stops, not HOW HIGH the thing stopping it is; height is exactly
    # the quantity it does not measure. One more reason the swap is a capability gain
    # and not only a reliability one.
    min_obstacle_height_m: float = 0.018

    # --- rule B: the lidar as a live background -----------------------------------
    # A ToF return this much nearer than the lidar's own range at the same bearing is
    # something the lidar cannot see -- which is the definition of the obstacle class
    # this sensor exists for.
    # PINNED 2026-08-14 BY BENCH ITEM J. Set FROM the measured distribution, which is
    # what design 9.8 required and what the previous 0.10 never was.
    #
    # THE MEASUREMENT. Bare wall at 0.70 m from the sensor face, rover stationary,
    # chassis off, /scan and ToF recorded together and compared PER COLUMN in
    # base_link through the recorded TF. 639 frames x 8 columns = 5112 column-samples
    # of bare wall, phantom-direction (ToF nearer than lidar) only:
    #
    #     p99 = 0.0293 m    p99.9 = 0.0352 m    MAX = 0.0396 m
    #
    # and the 5 cm object at 0.55 m base_link, in the two columns that see it:
    #
    #     min 0.1219 m   p1 0.2038 m   median 0.2182 m
    #
    # The two populations do not overlap: the wall never reached 0.040 m and the
    # object never fell below 0.122 m. Any margin in (0.040, 0.122) separates them.
    #
    # WHY 0.06 AND NOT THE OLD 0.10, which would also have measured zero false fires
    # here: the false-fire side is ours to measure, but the DETECTION side is not. A
    # return's disagreement is the gap between the object and whatever the lidar sees
    # behind it, and how close an object sits to its wall is the room's choice, not
    # ours. The tilt session's own object (TILT_BOX_050) sat 0.089 m in front of its
    # wall -- a real geometry that a 0.10 m margin MISSES and 0.06 catches with 48%
    # to spare. So the margin is set as low as the false-fire data allows, not as high
    # as the detection data would tolerate. 0.06 is 1.5x the measured worst case and
    # 2.0x p99.
    #
    # Capture: 03_validation/bench_J_2026-08-14/ (mcap, sha256 recorded there).
    # Analysis: diagnostics/tof_lidar_frame_check.py and the per-column script beside
    # the capture. Re-derive from the bag rather than trusting this comment.
    disagreement_margin_m: float = 0.06
    # RULE B IS LIVE, 2026-08-14, because bench item J pinned its margin against a
    # real synchronised capture. The gate lives on the DETECTOR rather than on the
    # consumer, because rule B decides what it publishes and a consumer-side gate
    # would leave the detector claiming obstacles nobody was allowed to act on -- two
    # components disagreeing about what is true, which is the seam class this project
    # keeps paying for.
    #
    # What J measured, and it is the number that decides this flag: ZERO phantom fires
    # in 5112 bare-wall column-samples at the pinned margin, against a detection that
    # holds 0.218 m of median disagreement on the object. Design 9.8's criterion was
    # "inside the margin in at least 99% of frames"; the measurement is 100%.
    #
    # WHAT J DID NOT SETTLE, so the next reader does not over-read this flag: one
    # wall, one object, one room, one afternoon. The false-fire rate is measured
    # against THIS background, and a room with a mirror, a glass door or a dark
    # low-reflectance surface is not in the sample. Rule B degrades toward silence
    # rather than toward phantoms (it can only ADD obstacles, and an absent or stale
    # scan removes them), so the failure direction is a missed obstacle, which the
    # lidar and the freeze machinery still cover.
    rule_b_enable: bool = True
    # Height of the lidar's scan plane. THE NODE OVERRIDES THIS FROM TF and this
    # default exists only so the pure core is runnable in a test -- geometry belongs to
    # TF, and a consumer that hardcodes a mounting height is the N1 defect wearing a
    # different hat. Rule B applies only where the ToF ray is BELOW this plane; above
    # it the lidar can see whatever the ToF sees, so disagreement means occlusion
    # geometry rather than a sub-lidar object.
    lidar_plane_m: float = 0.1905


def zone_angles(row: int, col: int, cfg: TofConfig) -> tuple:
    """(azimuth, elevation) of a zone centre in the ROBOT frame, radians.

    Azimuth: + is LEFT (REP-103). Elevation: + is UP.

    AXIS FACTS, TESTED on 2026-08-13 rather than assumed -- moving a box rover-left
    moved its returns to LOWER column indices, and standing an object up off the floor
    moved its returns to LOWER row indices:

        column 0 = rover-LEFT      row 0 = UP

    DO NOT generalise this from the lidar and do not generalise the lidar from this.
    The RPLIDAR's `base_link->laser` yaw is ~179 deg, so its raw bearing 0 points
    BEHIND the robot; a ladder rung once steered into a mirror image of open space
    because that was assumed away (N1). This sensor needs no such rotation. The two
    facts live next to each other on purpose.
    """
    az, el_s = zone_angles_sensor(row, col, cfg)
    return az, el_s + math.radians(cfg.mount_pitch_deg)


def zone_angles_sensor(row: int, col: int, cfg: TofConfig) -> tuple:
    """(azimuth, elevation) of a zone centre RELATIVE TO THE SENSOR BORESIGHT, radians.

    Both frames are needed and they are not interchangeable:

        WORLD elevation  -- where the ray GOES (what the floor intersection needs)
        SENSOR elevation -- what the reading is MEASURED ALONG (what `reports_z` needs)

    THE DEFECT THIS SPLIT FIXES, found 2026-08-13: every projection in this module used
    the WORLD elevation, i.e. it assumed the sensor reports distance along base_link x.
    It reports along its own boresight. At the level mount (pitch +4 deg) the two differ
    by a fraction of a percent and the error hid inside `floor_margin_m`; at -15.7 deg
    it is 4%, and the floor model under-predicted eight recorded clear-floor medians by
    19-42 mm -- ALL NEGATIVE, 29.7 mm RMS. With the boresight projection the SAME
    constants land at 5.7 mm RMS with mixed signs.

    So a mount angle that was chosen to see low obstacles also converted a harmless
    rounding error into a systematic one, and a level bench could never have shown it.
    """
    az = math.radians(-(col - (ZONES - 1) / 2.0) * cfg.zone_deg_h)
    el_s = math.radians(((ZONES - 1) / 2.0 - row) * cfg.zone_deg_v)
    return az, el_s


def zone_ray(row: int, col: int, cfg: TofConfig) -> tuple:
    """Unit vector of a zone's ray in the ROBOT frame, from the sensor origin.

    The mount pitch is applied as a REAL ROTATION rather than by adding it to the
    zone's elevation. Those agree only on the centre column: at the corner zone
    (az 26.25 deg, el_s -20.65 deg, pitch -15.7 deg) the shortcut is off by 1.8 deg of
    world elevation, which moves that zone's floor intersection by ~5%. At the old +4
    deg mount the same shortcut was wrong by 0.4 deg and nobody could have noticed.
    """
    az_s, el_s = zone_angles_sensor(row, col, cfg)
    ux, uy, uz = (math.cos(el_s) * math.cos(az_s),
                  math.cos(el_s) * math.sin(az_s),
                  math.sin(el_s))
    p = math.radians(cfg.mount_pitch_deg)          # + = nose UP
    cp, sp = math.cos(p), math.sin(p)
    return (cp * ux - sp * uz, uy, sp * ux + cp * uz)


def _boresight_cosine(row: int, col: int, cfg: TofConfig) -> float:
    """How much of a ray lies along the boresight -- the factor relating the RAY LENGTH
    to what the sensor REPORTS when `reports_z` is true."""
    az_s, el_s = zone_angles_sensor(row, col, cfg)
    return math.cos(el_s) * math.cos(az_s)


def _floor_ray_length(row: int, col: int, cfg: TofConfig) -> Optional[float]:
    """Ray length from the sensor to the floor, or None for a ray that never gets
    there (level or rising). Uses the ray's true world z-component, not an angle sum."""
    uz = zone_ray(row, col, cfg)[2]
    if uz >= -1e-9:
        return None
    return cfg.mount_height_m / -uz


def _floor_reading_m(row: int, col: int, cfg: TofConfig) -> Optional[float]:
    """What this zone would REPORT for flat floor -- ray length converted into the
    sensor's own reporting convention."""
    r = _floor_ray_length(row, col, cfg)
    if r is None:
        return None
    return r * _boresight_cosine(row, col, cfg) if cfg.reports_z else r


def valid_mm(value: int, cfg: TofConfig) -> bool:
    """A reading that means something ANYWHERE in the frame. Sentinels and impossible
    values are not measurements and must never be published as points."""
    return cfg.min_valid_mm <= int(value) < cfg.sentinel_mm and int(value) <= cfg.max_valid_mm


def plausible_for_zone(row: int, col: int, value: int, cfg: TofConfig) -> bool:
    """Validity is global; PLAUSIBILITY is per-zone, and conflating them was a bug.

    1824 mm is nonsense in a row that points at floor under a metre away and entirely
    ordinary in a row facing a wall. So the tight bound applies only where the
    geometry justifies it: a downward zone cannot honestly report more than its own
    floor intersection plus margin, and the recorded baseline junk (0, 1824, 2163,
    2251, 2612, 2717, 3185, 3691, 3730 in row 5) is exactly what that rejects.
    """
    if not valid_mm(value, cfg):
        return False
    floor = _floor_reading_m(row, col, cfg)
    if floor is None:
        return True
    return value / 1000.0 <= floor + cfg.floor_margin_m


def zone_point(row: int, col: int, value_mm: int, cfg: TofConfig) -> Optional[tuple]:
    """(x, y, z) of a zone's return in `base_link`, metres, or None if invalid.

    THE SINGLE SOURCE OF TRUTH FOR WHERE A RETURN IS. Every predicate below that needs
    a position asks this function, and none of them re-derives one from a scalar. That
    is a rule with a cost attached: the word "range" was used for two quantities -- a
    distance from the SENSOR (what the ToF reports) and a distance from `base_link`
    (what every consumer assumed) -- and the sensor sits `mount_x_m` forward of
    `base_link`. Three separate defects came out of that one ambiguity in a day.

    The mount offsets are applied HERE, once, so a caller cannot forget them:
    `mount_x_m` forward and `mount_height_m` up. A ray length is measured from the
    sensor; the point is expressed from `base_link`; nothing in between gets to hold a
    number whose frame you have to remember.

    `reports_z` decides the arithmetic: if the sensor reports PERPENDICULAR distance
    the reading is the x-component and the ray is scaled out to it; if it reports
    radial range the reading is the ray length. Getting this backwards misplaces every
    off-axis point, which is why it is a config switch with a measurement attached
    rather than an assumption baked into the maths.
    """
    if not valid_mm(value_mm, cfg):
        return None
    d = value_mm / 1000.0
    r = d / _boresight_cosine(row, col, cfg) if cfg.reports_z else d
    return _point_from_ray_length(row, col, r, cfg)


def _point_from_ray_length(row: int, col: int, r: float, cfg: TofConfig) -> tuple:
    """(x, y, z) in `base_link` for a ray of length `r` FROM THE SENSOR.

    THE ONLY PLACE A RAY LENGTH BECOMES A POSITION, and therefore the only place the
    mount offsets are applied. Two callers need this arithmetic -- a return's point and
    the floor's -- and the previous shape had each of them doing its own version, which
    is how one of them came to omit `mount_x_m` entirely while the other never had it
    to omit.
    """
    ux, uy, uz = zone_ray(row, col, cfg)
    return (cfg.mount_x_m + r * ux, r * uy, cfg.mount_height_m + r * uz)


def floor_ground_range_m(row: int, col: int, cfg: TofConfig) -> Optional[float]:
    """Ground range IN base_link at which this zone's ray meets the floor, or None for
    a ray that never gets there.

    The `base_link` counterpart of `_floor_reading_m`, and the two must never be
    substituted for each other -- substituting them is bug 2 of the frame batch.
    `_floor_reading_m` answers *"what would this zone REPORT for flat floor?"*: a
    sensor-frame reading, and the right operand only for comparison against another
    reading. This answers *"how far in front of the ROBOT is that floor?"*: the right
    operand for comparison against a distance the robot cares about, such as the stop
    distance. They differ by the mount offset AND by the boresight projection, and the
    gap is large enough to move rule A's row set.
    """
    r = _floor_ray_length(row, col, cfg)
    if r is None:
        return None
    x, y, _z = _point_from_ray_length(row, col, r, cfg)
    return math.hypot(x, y)


def expected_floor_m(row: int, col: int, cfg: TofConfig) -> Optional[float]:
    """What this zone would read if it saw nothing but flat floor, or None when the
    zone does not point at floor within the visible horizon.

    None has two causes and the caller must treat them differently: a zone pointing
    at or above the horizon never meets the floor at all, and a zone whose floor
    intersection lies beyond the visibility horizon meets it but gets nothing back
    (grazing incidence -- measured: floor at 0.39 m returns, floor at 0.79 m does not).
    `floor_beyond_horizon` below distinguishes them.
    """
    # AZIMUTH MATTERS when the sensor reports perpendicular distance: the reading
    # shrinks by cos(az) across the row, so the edge columns' floor legitimately reads
    # NEARER than the centre's. Modelling a row as one distance over-predicts by 11% at
    # column 7 (cos 26.25 deg) -- which made real floor look like an obstacle in the
    # recorded clear-floor frames, and is the bug `_boresight_cosine` exists to record.
    reading = _floor_reading_m(row, col, cfg)
    if reading is None:
        return None
    return None if reading > cfg.floor_horizon_m else reading


def floor_beyond_horizon(row: int, col: int, cfg: TofConfig) -> bool:
    """True when this zone points DOWN at floor we cannot see -- the band where any
    plausible return is informative, because nothing else in the room produces one."""
    reading = _floor_reading_m(row, col, cfg)
    return reading is not None and reading > cfg.floor_horizon_m


def nearer_than_floor(frame, cfg: TofConfig) -> list:
    """RULE (i). Zones reading meaningfully nearer than the floor they point at.

    This is the rule that turns a 20% signal into a 100% one: a 5 cm box at 0.74 m
    against a wall shortened its zone from 992 mm to 892 mm in EVERY frame, while
    "did anything return" caught it in about a fifth of them.
    """
    out = []
    for i, value in enumerate(frame):
        row, col = divmod(i, ZONES)
        if not valid_mm(value, cfg):
            continue
        # AUTHORITY FIRST, then the comparison. This zone may see floor perfectly well
        # and still not be entitled to conclude from it -- design 9.2.
        if not rule_a_applies(row, col, cfg):
            continue
        # Terrain, not an obstacle -- Scott's 18 mm gate, applied before the comparison
        # so that a mat ridge is never a brake in either rule.
        if not tall_enough_to_matter(row, col, value, cfg):
            continue
        # `_floor_reading_m`, NOT `expected_floor_m`. The latter caps at
        # `floor_horizon_m` and returns None past it -- which is an AUTHORITY decision
        # wearing a visibility constant's name, the exact confusion 9.1 diagnosed.
        # Calling it here left the new bound INERT: a mutation deleting
        # `rule_a_applies` altogether changed no test, because at a 0.45 m stop
        # distance the 0.55 m horizon was already stricter and hid it. Authority is the
        # line above; this line is only arithmetic.
        expected = _floor_reading_m(row, col, cfg)
        if expected is None:
            continue
        if value / 1000.0 < expected - cfg.floor_margin_m:
            out.append((row, col))
    return out


def rule_a_applies(row: int, col: int, cfg: TofConfig) -> bool:
    """Whether the floor comparison may CONCLUDE for this zone -- design note 9.2.

    Computed from the geometry and the stop distance every time, never a row list. A
    frozen row set is how `floor_horizon_m` came to be enforcing an authority boundary
    it was never chosen for.

    THE OPERAND IS A `base_link` GROUND RANGE, and it used to be a sensor READING. The
    bound asks "is this zone's floor inside the distance at which the ROBOT must stop?"
    -- both sides of that question belong to the robot, and `_floor_reading_m` belongs
    to the sensor. The reading is the shorter number, so the old comparison admitted
    rows whose floor is actually FURTHER from base_link than the stop distance, and
    admitted them to the one rule that cannot tell an object from a wall. The error was
    in the permissive direction: it granted authority the geometry did not support.
    """
    floor = floor_ground_range_m(row, col, cfg)
    return floor is not None and floor <= cfg.stop_distance_m


def rule_a_rows(cfg: TofConfig) -> list:
    """The rows rule A currently holds authority over, for ~/state. A recording that
    cannot say which rows were allowed to conclude cannot explain what fired."""
    return [r for r in range(ZONES) if any(rule_a_applies(r, c, cfg) for c in range(ZONES))]


def tall_enough_to_matter(row: int, col: int, value: int, cfg: TofConfig) -> bool:
    """Is this return HIGH ENOUGH off the floor to be an obstacle rather than terrain?

    Scott's 18 mm gate (see `min_obstacle_height_m`). The height comes from the same
    fitted geometry the floor model uses: a return at ray-length r sits at
    `mount_height + r * uz`, and uz is negative for every downward row, so a return
    nearer than the floor is correspondingly higher off it.

    APPLIED TO BOTH RULES, not just one. A 15 mm mat ridge produces a return that is
    genuinely nearer than modelled floor AND genuinely invisible to the lidar, so it
    satisfies rule A and rule B alike -- gating only one of them would leave the other
    braking for the same mat.

    A row that never meets the floor has no height above it to speak of and is passed
    through: those returns are adjudicated by the lidar comparison, which is the only
    thing that can judge them.
    """
    if _floor_ray_length(row, col, cfg) is None:
        return True
    point = zone_point(row, col, value, cfg)
    if point is None:
        return False
    return point[2] >= cfg.min_obstacle_height_m


def standing_above_floor(row: int, col: int, value: int, cfg: TofConfig) -> bool:
    """Is this return something standing ABOVE the ground, rather than the ground?

    The shared front half of both detection rules. A zone reading within
    `floor_margin_m` of its modelled floor is the floor; a zone reading NEARER than
    that has something in front of it. What the two rules then do with that fact is
    where they differ -- rule A concludes directly (inside its bound, where the answer
    is the same either way), rule B asks the lidar (outside it, where it is not).

    A row whose ray never reaches the floor has no floor to be confused with, so every
    plausible return there is standing by definition.

    THE MARGIN CUTS THE RIGHT WAY HERE. A return within the margin of modelled floor is
    within a couple of centimetres of the ground at any of these geometries, so calling
    it floor loses only objects thin enough that the freeze/escape machinery is the
    thing that handles them anyway -- and a floor-model error costs sensitivity rather
    than producing a phantom.
    """
    if not valid_mm(value, cfg):
        return False
    if not tall_enough_to_matter(row, col, value, cfg):
        return False
    floor = _floor_reading_m(row, col, cfg)
    if floor is None:
        return True
    return value / 1000.0 < floor - cfg.floor_margin_m


def rule_b_applies(point, cfg: TofConfig) -> bool:
    """Whether this return is BELOW the lidar's scan plane -- design 9.3.

    Above the plane the lidar sees what the ToF sees, so a disagreement there is
    occlusion geometry, not a sub-lidar object, and rule B must stay quiet. At the
    shipped geometry only row 0 rises, crossing at ~0.59 m.

    IT TAKES THE POINT, AND THE POINT ALREADY KNOWS ITS OWN HEIGHT. The previous
    version took a ground range and RECONSTRUCTED the height from the ray's slope --
    correct only while that range was measured from the sensor, which is exactly what
    stopped being true when `zone_point` started returning true `base_link`. The two
    frame errors had been cancelling: reconstruction and caller agreed because both
    omitted `mount_x_m`. Fixing the point alone would have left this formula treating a
    `base_link` ground range as a sensor-frame one and UNDER-estimating z by 22 mm at
    0.50 m -- growing with range, and in the PERMISSIVE direction, since a return that
    looks lower than it is looks more sub-lidar than it is. That is why the frame fix
    had to be one refactor and not three patches: `tall_enough_to_matter` already read
    `point[2]` and was already immune, and this is that pattern applied everywhere.
    """
    return point is not None and point[2] < cfg.lidar_plane_m


def column_span_rad(col: int, cfg: TofConfig) -> tuple:
    """(low, high) azimuth bounds of a column, radians, in the ROBOT frame."""
    half = math.radians(cfg.zone_deg_h) / 2.0
    az = math.radians(-(col - (ZONES - 1) / 2.0) * cfg.zone_deg_h)
    return az - half, az + half


def scan_min_by_column(bearings_rad, ranges_m, cfg: TofConfig) -> list:
    """Minimum lidar range within each ToF column's angular span.

    Bearings must ALREADY be in the robot frame -- the caller transforms them through
    TF. The RPLIDAR's `base_link->laser` yaw is ~179 deg, so raw scan indices are very
    nearly the mirror image of these bearings, and a rung once steered into exactly
    that mirror image (N1). This function refuses to guess: it takes bearings and
    believes them.

    MINIMUM, not mean or max, and the choice has a direction. A ToF column spans 7.5 deg
    against the lidar's ~1 deg, so a column straddling a doorway holds both near and far
    returns. Taking the nearest makes disagreement HARDER to claim, so rule B's error is
    a missed obstacle rather than a phantom brake at every corner -- deliberately the
    OPPOSITE bias from rule A, which operates where over-caution is free.

    A column with no return inside its span is `inf`, meaning OPEN TO MAX RANGE, which
    is rule B's strongest case. That is not the same as having no scan at all, which is
    the caller's job to signal by passing None (design 9.6).
    """
    out = []
    for col in range(ZONES):
        lo, hi = column_span_rad(col, cfg)
        best = math.inf
        for b, r in zip(bearings_rad, ranges_m):
            if lo <= b <= hi and math.isfinite(r) and r > 0.0 and r < best:
                best = r
        out.append(best)
    return out


def lidar_disagreement(frame, cfg: TofConfig, column_min) -> list:
    """RULE B, SINGLE FRAME. Zones the ToF sees NEARER than the lidar does.

    `column_min` is `scan_min_by_column`'s output, or None when there is no usable
    scan. None means UNAVAILABLE, and unavailable returns nothing -- not an obstacle
    and not a clearance. Rule B can only ADD obstacles, so its absence can only remove
    them; it cannot phantom-brake by being missing.
    """
    if column_min is None or not cfg.rule_b_enable:
        return []
    out = []
    for row in range(ZONES):
        hits = []
        for col in range(ZONES):
            value = frame[row * ZONES + col]
            if not plausible_for_zone(row, col, value, cfg):
                continue
            # THE FLOOR IS NOT AN OBSTACLE, and this line is why the rule works. The
            # first version of rule B compared every return against the lidar and
            # nothing else -- so on any recorded segment the FLOOR itself, which is
            # always nearer than the wall the lidar sees, fired in 38 of 40 frames on
            # clear floor. Only returns standing ABOVE the ground are candidates.
            if not standing_above_floor(row, col, value, cfg):
                continue
            point = zone_point(row, col, value, cfg)
            if point is None:
                continue
            if not rule_b_applies(point, cfg):
                continue
            # THE GROUND RANGE COMPARED AGAINST THE LIDAR IS THE POINT'S OWN, in
            # base_link, and so is `column_min` -- the node transforms the scan through
            # TF before it ever reaches `scan_min_by_column`. Both sides of this
            # inequality are therefore distances from the same origin. While
            # `zone_point` omitted `mount_x_m` they were not, and the ToF read 0.10 m
            # nearer than the lidar on identical surfaces, which is a phantom
            # disagreement of exactly the size rule B's margin was being asked to cover.
            ground = math.hypot(point[0], point[1])
            if ground < column_min[col] - cfg.disagreement_margin_m:
                hits.append(col)
        out.extend((row, col) for col in _adjacent_runs(hits, cfg))
    return out


def _adjacent_runs(hits, cfg: TofConfig) -> list:
    """Columns belonging to a run of at least `min_adjacent_zones` adjacent hits.

    Shared by rules A-beyond and B because the noise it rejects is a property of the
    SENSOR, not of either rule: the isolated-return baseline ran 1.3-1.8% of frames on
    clear floor against 45.2% adjacent-pair with a rail present.
    """
    out, run = [], []
    for c in list(hits) + [None]:
        if run and c is not None and c == run[-1] + 1:
            run.append(c)
            continue
        if len(run) >= cfg.min_adjacent_zones:
            out.extend(run)
        run = [] if c is None else [c]
    return out


def unexpected_returns(frame, cfg: TofConfig) -> list:
    """RULE (ii), SINGLE FRAME. Plausible returns in the band where the floor cannot
    be seen, keeping only groups of `min_adjacent_zones` ADJACENT columns in a row.

    Adjacency is the whole discriminator, and it is measured: ">=2 adjacent zones
    returning in one frame" ran 1.3% on clear floor, 1.8% facing a bare wall, and
    45.2% with a 5 cm rail at 0.46 m. Isolated single returns are the baseline and
    are dropped here -- including real ones, which is the price of a brake that does
    not fire every three seconds on empty floor.
    """
    out = []
    for row in range(ZONES):
        hits = [c for c in range(ZONES)
                if floor_beyond_horizon(row, c, cfg)
                and plausible_for_zone(row, c, frame[row * ZONES + c], cfg)]
        run = []
        for c in hits + [None]:
            if run and c is not None and c == run[-1] + 1:
                run.append(c)
                continue
            if len(run) >= cfg.min_adjacent_zones:
                out.extend((row, cc) for cc in run)
            run = [] if c is None else [c]
    return out


class ObstacleDetector:
    """Rules (i) and (ii) across frames, with rule (ii)'s N-of-M confirmation.

    Rule (i) needs no confirmation window -- it is continuous by construction, which
    is the point of comparing against a background. Rule (ii) does, because its
    single-frame evidence is a few percent contaminated even after adjacency.
    """

    def __init__(self, cfg: Optional[TofConfig] = None):
        self._cfg = cfg or TofConfig()
        self._history: list = []

    def retune(self, cfg: TofConfig) -> None:
        """Replace the config WITHOUT clearing the confirmation window.

        Exists for exactly one caller: the node latches the lidar's plane height from
        TF, which is not available at construction time. Public rather than a poke at
        `_cfg` from outside, because a private attribute assigned from another module is
        a contract nobody can see. The window survives on purpose -- the geometry has
        not changed, only our knowledge of one number in it, and dropping N-of-M history
        on a TF update would make confirmation depend on TF timing.
        """
        self._cfg = cfg

    def update(self, frame, column_min=None) -> dict:
        """One frame in; the obstacle conclusion out.

        `column_min` is `scan_min_by_column`'s output, or None when there is no usable
        scan -- in which case rule B is UNAVAILABLE and reports nothing. Note what the
        default does: a caller that forgets the lidar entirely gets rule A alone, which
        is the degraded behaviour and never the over-confident one.

        Returns every rule separately rather than a merged verdict: a consumer that
        wants only the certain one can have it, and a recording can say WHICH rule
        fired -- the alternative is a boolean nobody can debug afterwards.
        """
        cfg = self._cfg
        near = nearer_than_floor(frame, cfg)
        disagree = lidar_disagreement(frame, cfg, column_min)
        self._history.append(set(disagree))
        if len(self._history) > cfg.window_frames:
            self._history.pop(0)
        counts: dict = {}
        for seen in self._history:
            for z in seen:
                counts[z] = counts.get(z, 0) + 1
        confirmed = [z for z, n in counts.items() if n >= cfg.confirm_frames]
        return {
            "nearer_than_floor": near,                  # rule A, immediate
            "disagrees_this_frame": disagree,           # rule B, unconfirmed
            "confirmed_disagreement": sorted(confirmed),  # rule B, N-of-M
            # CONFIGURATION-descriptive, never capability-descriptive. "rule_a_only"
            # stays true whatever the constants do; a word like "short_range" would
            # quietly become a lie the moment a threshold moved. Report what IS and let
            # the reader judge what it means (D35).
            "rules": "rule_a+b" if cfg.rule_b_enable else "rule_a_only",
            "background": "unavailable" if column_min is None else "ok",
            "rule_a_rows": rule_a_rows(cfg),
            "obstacles": sorted(set(near) | set(confirmed)),
        }
