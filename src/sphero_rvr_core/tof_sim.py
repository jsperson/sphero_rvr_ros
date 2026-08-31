"""A world the ToF and the lidar both look at, so their disagreement is about
HEIGHT and nothing else.

WHY THIS EXISTS. The rig runs the real supervisor but feeds it only a simulated
lidar, so the entire low-obstacle brake -- rules A and B, hold-on-vanish, the
staleness bound -- has never been exercised by an automated test. Every
certification we hold, including the one that cleared the 10k-line deletion, was
blind to it.

THE ONE REQUIREMENT THAT SHAPES THIS MODULE: both sensors are raycast against the
SAME world, from one geometry. If they were driven from separate descriptions they
would disagree by construction rather than by design, and a test built on that
would prove only that two authors typed different numbers. The lidar sweeps a
horizontal plane at `lidar_plane_m`; the ToF looks down its pitched cone from
`mount_height_m`. An obstacle shorter than the lidar's plane is invisible to one
and visible to the other FOR A REASON THE GEOMETRY PRODUCES, which is the whole
point of the sub-lidar class.

WHAT IT REUSES AND DOES NOT RE-DERIVE. Every mount constant, ray direction and
reporting convention comes from `tof_frame` -- `zone_ray`, `_boresight_cosine`,
`TofConfig` -- which is certified to 3 mm against 12,869 recorded frames. The only
new geometry here is ray-vs-axis-aligned-box, which is the slab method and has no
robot-specific content. A second author of the mount maths is exactly how the
~179 deg lidar mount would have been got wrong.

WHAT THIS DOES NOT MODEL, and the limit is not a footnote:
  * It tests the BRAKE, not the SENSOR. Real ToF returns are noisy, drop out, and
    are contaminated a few percent even after adjacency -- which is precisely why
    rule B needs N-of-M confirmation. Here every reading is exact, so a test that
    passes on this sim says the DECISION LOGIC is right, never that the sensor
    will deliver the frames it needs.
  * It cannot show starvation. The ToF was measured running at 2.53 Hz under
    ordinary process load (2026-08-25, D71), below the 0.30 s staleness bound its
    own consumer depends on. Nothing here produces that; frames arrive whenever
    the caller asks.
  * No multipath, no ambient-light dropout, no surface reflectivity. A black
    5 cm box at 0.74 m is a different sensor problem from a white one and this
    module cannot tell them apart.
"""

from dataclasses import dataclass, field
from typing import Optional
import math

from sphero_rvr_core.tof_frame import (TofConfig, _boresight_cosine, zone_ray,
                                       ZONES)


@dataclass(frozen=True)
class Box:
    """An axis-aligned box in the ROBOT frame (base_link): x forward, y left, z up."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def __post_init__(self):
        for lo, hi, name in ((self.x_min, self.x_max, "x"),
                             (self.y_min, self.y_max, "y"),
                             (self.z_min, self.z_max, "z")):
            if not hi > lo:
                raise ValueError(f"box {name}_max must exceed {name}_min")


@dataclass
class World:
    """Flat floor at z=0 plus boxes. Both sensors see exactly this and nothing else."""

    boxes: list = field(default_factory=list)
    floor: bool = True


def _ray_box_t(origin, direction, box: Box) -> Optional[float]:
    """Slab method: distance along `direction` to the box, or None. Plain geometry,
    no robot content -- the only maths in this module that is not `tof_frame`'s."""
    t_lo, t_hi = 0.0, math.inf
    for o, d, lo, hi in ((origin[0], direction[0], box.x_min, box.x_max),
                         (origin[1], direction[1], box.y_min, box.y_max),
                         (origin[2], direction[2], box.z_min, box.z_max)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_lo, t_hi = max(t_lo, t1), min(t_hi, t2)
        if t_lo > t_hi:
            return None
    return t_lo if t_hi >= t_lo >= 0.0 else None


def _ray_floor_t(origin, direction) -> Optional[float]:
    """Distance to the z=0 plane, or None for a ray that never reaches it."""
    if direction[2] >= -1e-9:
        return None
    return (0.0 - origin[2]) / direction[2]


def _nearest_hit(origin, direction, world: World) -> Optional[float]:
    hits = [t for t in (_ray_box_t(origin, direction, b) for b in world.boxes)
            if t is not None]
    if world.floor:
        t = _ray_floor_t(origin, direction)
        if t is not None:
            hits.append(t)
    return min(hits) if hits else None


def simulate_tof_frame(world: World, cfg: Optional[TofConfig] = None) -> list:
    """The 64 millimetre readings this world would produce, row-major.

    Ray directions and the reporting convention come from `tof_frame`; a zone whose
    ray hits nothing gets the SENTINEL, which is what the real sensor reports and
    what `valid_mm` already knows to reject.
    """
    cfg = cfg or TofConfig()
    origin = (cfg.mount_x_m, 0.0, cfg.mount_height_m)
    frame = []
    for row in range(ZONES):
        for col in range(ZONES):
            u = zone_ray(row, col, cfg)
            t = _nearest_hit(origin, u, world)
            if t is None:
                frame.append(cfg.sentinel_mm)
                continue
            reported = t * _boresight_cosine(row, col, cfg) if cfg.reports_z else t
            mm = int(round(reported * 1000.0))
            frame.append(min(mm, cfg.sentinel_mm))
    return frame


def simulate_scan(world: World, bearings_rad, cfg: Optional[TofConfig] = None,
                  range_max_m: float = 6.0, laser_x_m: float = 0.0) -> list:
    """What a HORIZONTAL lidar at `lidar_plane_m` sees of the same world.

    The plane height is `TofConfig.lidar_plane_m` -- the same constant rule B uses to
    decide whether a point is below the lidar -- so the two sensors cannot drift apart
    in a test's favour. A bearing that hits nothing returns `range_max_m`, matching a
    scan over open floor: the floor itself is never hit by a horizontal ray, which is
    exactly why a low obstacle is invisible here.
    """
    cfg = cfg or TofConfig()
    origin = (laser_x_m, 0.0, cfg.lidar_plane_m)
    ranges = []
    for bearing in bearings_rad:
        u = (math.cos(bearing), math.sin(bearing), 0.0)
        t = _nearest_hit(origin, u, World(boxes=world.boxes, floor=False))
        ranges.append(range_max_m if t is None or t > range_max_m else t)
    return ranges
