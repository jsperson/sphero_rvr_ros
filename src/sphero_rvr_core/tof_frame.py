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
    # detector reported an obstacle in 99.3% of frames. Separating the two is design
    # note 9.1-9.2, and lands with the rules that need it.
    floor_horizon_m: float = 0.55



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
    """(x, y, z) of a zone's return in the robot frame, metres, or None if invalid.

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
    ux, uy, uz = zone_ray(row, col, cfg)
    return (r * ux, r * uy, cfg.mount_height_m + r * uz)


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
        expected = expected_floor_m(row, col, cfg)
        if expected is None:
            continue
        if value / 1000.0 < expected - cfg.floor_margin_m:
            out.append((row, col))
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

    def update(self, frame) -> dict:
        """One frame in; the obstacle conclusion out.

        Returns both rules separately rather than a merged verdict: a consumer that
        wants only the certain one (i) can have it, and a recording can tell which
        rule fired -- the alternative is a boolean nobody can debug afterwards.
        """
        cfg = self._cfg
        near = nearer_than_floor(frame, cfg)
        adjacent = unexpected_returns(frame, cfg)
        self._history.append(set(adjacent))
        if len(self._history) > cfg.window_frames:
            self._history.pop(0)
        counts: dict = {}
        for seen in self._history:
            for z in seen:
                counts[z] = counts.get(z, 0) + 1
        confirmed = [z for z, n in counts.items() if n >= cfg.confirm_frames]
        return {
            "nearer_than_floor": near,          # rule (i), immediate
            "adjacent_this_frame": adjacent,    # rule (ii), unconfirmed
            "confirmed_unexpected": sorted(confirmed),   # rule (ii), N-of-M
            "obstacles": sorted(set(near) | set(confirmed)),
        }
