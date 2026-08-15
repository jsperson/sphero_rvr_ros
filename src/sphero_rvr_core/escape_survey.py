"""The stuck state's FIRST ACT: measure, and say what you measured.

Scott, 2026-08-15: *"The first thing when the rover gets stuck should be taking
measurements with the sensors and trying to figure a way out."* He said it long ago
-- "pivot more and use lidar to see where the open space is" is founding doctrine --
and the honest audit is that it was half-built: the stall ladder consumes ONE bearing
as a hint, and the give-up escape consumed nothing at all.

The field paid for the correction the same afternoon. Cornered with objects on three
sides, the rover SPUN 180 DEGREES to face out of the corner and then reversed straight
back into the obstacle it had just turned away from -- with 1.77 m of clear floor dead
ahead. It had solved the hard part and thrown the answer away, because the escape has
no notion of where it is pointing. It was not choosing badly among options; it had
exactly one option and never looked.

THIS MODULE IS STAGE ONE ONLY: it measures and it reports. It ranks nothing and it
commands nothing. Planning and execution land separately, deliberately, so that the
inert half can be checked against the recordings before anything acts on it.

VOUCHING IS THE IDEA THAT MAKES THE SURVEY HONEST. A direction nothing can see is not
a direction that is clear. Every bearing carries who vouches for it:

    TRAIL      the robot's own body swept it -- ALL HEIGHTS, the strongest evidence
    ToF        sub-lidar returns, within the measured envelope only
    LIDAR      the 0.19 m scan plane, and nothing above or below it
    UNVOUCHED  nothing can speak

On 2026-08-15 every freeze was "an obstacle no sensor on this robot can see" while the
ToF reported zero zones: the thing that pinned the rover lived in UNVOUCHED space. A
survey that reported that direction as merely "open" would be repeating the mistake
that produced the freeze.

The survey is logged as ONE unit on purpose. It is both the plan's input and the
register's evidence, so a future wedge autopsy starts from the robot's own words
instead of a reconstruction from a bag -- three autopsies so far have each cost hours
that the robot could have emitted in a single line.
"""

from dataclasses import dataclass, field
import math
from typing import Optional

from sphero_rvr_core.bearings import bearing_deg_to_clock

__all__ = [
    "VOUCH_TRAIL",
    "VOUCH_TOF",
    "VOUCH_LIDAR",
    "VOUCH_UNVOUCHED",
    "SurveyConfig",
    "DirectionReading",
    "Survey",
    "survey_from_scan",
    "format_survey",
]

# Vouching levels, strongest first. Ordered so `VOUCH_ORDER.index()` ranks them.
VOUCH_TRAIL = "trail"
VOUCH_TOF = "tof"
VOUCH_LIDAR = "lidar"
VOUCH_UNVOUCHED = "unvouched"
VOUCH_ORDER = (VOUCH_TRAIL, VOUCH_TOF, VOUCH_LIDAR, VOUCH_UNVOUCHED)


@dataclass(frozen=True)
class SurveyConfig:
    """Thresholds for reading a scan. Defaults mirror the deployed supervisor; a
    caller with the live config should pass its own rather than trust these."""

    open_m: float = 0.30
    """A clock position counts as open at or above this range."""

    footprint_front_m: float = 0.11
    footprint_rear_m: float = 0.16
    footprint_left_m: float = 0.10
    footprint_right_m: float = 0.10
    """Declared footprint. Returns INSIDE it are the 08-14b class: with points
    overlapping the footprint, the trajectory projection refuses everything unless the
    motion provably moves away, and every refusal is then correct by construction.
    A survey that did not count them would report a pose as open that no gate will
    ever let the robot leave."""


@dataclass(frozen=True)
class DirectionReading:
    clock: int
    bearing_deg: float
    range_m: Optional[float]
    vouched_by: str

    @property
    def is_open(self) -> bool:
        return self.range_m is not None and self.range_m >= DirectionReading._open_m

    _open_m: float = 0.30


@dataclass
class Survey:
    """One structured snapshot of a stuck pose. Input to the plan, evidence for the
    register, and content for the human help request -- one artifact, three consumers."""

    directions: dict = field(default_factory=dict)      # clock -> DirectionReading
    open_clocks: tuple = ()
    min_range_m: Optional[float] = None
    min_clock: Optional[int] = None
    min_bearing_deg: Optional[float] = None
    footprint_overlap_count: int = 0
    tof_zones: Optional[int] = None
    tof_available: bool = False
    trail_available: bool = False
    trail_length_m: Optional[float] = None
    cause: str = "unknown"
    ray_count: int = 0
    finite_ray_count: int = 0

    @property
    def unvouched_clocks(self) -> tuple:
        return tuple(sorted(c for c, d in self.directions.items()
                            if d.vouched_by == VOUCH_UNVOUCHED))

    @property
    def footprint_overlapped(self) -> bool:
        return self.footprint_overlap_count > 0


def _normalize_deg(deg: float) -> float:
    return (float(deg) + 180.0) % 360.0 - 180.0


def survey_from_scan(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    laser_yaw_deg,
    config: SurveyConfig = SurveyConfig(),
    tof_zones: Optional[int] = None,
    tof_available: bool = False,
    trail_available: bool = False,
    trail_length_m: Optional[float] = None,
    cause: str = "unknown",
) -> Survey:
    """Build a survey from one scan, in the BODY frame.

    `laser_yaw_deg` is the DEPLOYED base_link<-laser rotation and is required rather
    than defaulted: the data have only ever shown a bearing OFFSET, never a mounting
    fact, and a caller that forgets it would be surveying the laser frame -- which is
    the mirror-of-open-space defect that once steered recovery rungs the wrong way.
    """
    best: dict = {}
    overlap = 0
    finite = 0
    lo = max(float(range_min), 0.0)
    hi = float(range_max)

    for index, raw in enumerate(ranges):
        if raw is None:
            continue
        value = float(raw)
        if not math.isfinite(value) or value < lo or value > hi:
            continue
        finite += 1
        bearing = _normalize_deg(
            math.degrees(float(angle_min) + index * float(angle_increment))
            + float(laser_yaw_deg)
        )
        px = value * math.cos(math.radians(bearing))
        py = value * math.sin(math.radians(bearing))
        if (-config.footprint_rear_m <= px <= config.footprint_front_m
                and -config.footprint_right_m <= py <= config.footprint_left_m):
            overlap += 1
        clock = bearing_deg_to_clock(bearing)
        if clock not in best or value < best[clock][0]:
            best[clock] = (value, bearing)

    directions = {}
    for clock, (value, bearing) in best.items():
        # Stage one vouches from the lidar only. ToF and trail vouching arrive with
        # the plan stage, which is where they change a ranking; recording the level
        # here keeps the structure honest from the start rather than retrofitting it.
        directions[clock] = DirectionReading(
            clock=clock,
            bearing_deg=round(bearing, 1),
            range_m=value,
            vouched_by=VOUCH_LIDAR,
        )
    for clock in range(1, 13):
        if clock not in directions:
            directions[clock] = DirectionReading(
                clock=clock,
                bearing_deg=float(_normalize_deg(-(clock % 12) * 30.0)),
                range_m=None,
                vouched_by=VOUCH_UNVOUCHED,
            )

    open_clocks = tuple(sorted(
        c for c, d in directions.items()
        if d.range_m is not None and d.range_m >= config.open_m
    ))
    with_range = {c: d for c, d in directions.items() if d.range_m is not None}
    min_clock = min(with_range, key=lambda c: with_range[c].range_m) if with_range else None

    return Survey(
        directions=directions,
        open_clocks=open_clocks,
        min_range_m=with_range[min_clock].range_m if min_clock else None,
        min_clock=min_clock,
        min_bearing_deg=with_range[min_clock].bearing_deg if min_clock else None,
        footprint_overlap_count=overlap,
        tof_zones=tof_zones,
        tof_available=tof_available,
        trail_available=trail_available,
        trail_length_m=trail_length_m,
        cause=cause,
        ray_count=len(ranges),
        finite_ray_count=finite,
    )


def format_survey(survey: Survey) -> str:
    """One line, machine-greppable, human-readable. This is what a future autopsy
    reads instead of re-deriving the pose from a bag."""
    parts = [
        f"SURVEY cause={survey.cause}",
        f"open={len(survey.open_clocks)}/12",
        f"open_clocks={list(survey.open_clocks)}",
    ]
    if survey.min_range_m is not None:
        parts.append(
            f"min={survey.min_range_m:.3f}m@{survey.min_clock}oclock"
            f"(bearing={survey.min_bearing_deg:+.1f})"
        )
    parts.append(f"footprint_overlap={survey.footprint_overlap_count}")
    parts.append(
        f"tof={'zones=' + str(survey.tof_zones) if survey.tof_available else 'UNAVAILABLE'}"
    )
    if survey.trail_available:
        length = "?" if survey.trail_length_m is None else f"{survey.trail_length_m:.2f}m"
        parts.append(f"trail={length}")
    else:
        parts.append("trail=NONE")
    unvouched = survey.unvouched_clocks
    if unvouched:
        parts.append(f"unvouched={list(unvouched)}")
    parts.append(f"rays={survey.finite_ray_count}/{survey.ray_count}")
    per_clock = " ".join(
        f"{c}:{'---' if survey.directions[c].range_m is None else format(survey.directions[c].range_m, '.3f')}"
        for c in [12] + list(range(1, 12))
    )
    return " ".join(parts) + " | " + per_clock
