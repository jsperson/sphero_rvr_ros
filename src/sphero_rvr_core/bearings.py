"""Bearing <-> clock-position conversion, in ONE place, with the convention stated.

WHY THIS MODULE EXISTS. On 2026-08-14 an analysis script bucketed bearings with
``((bearing_ccw + 15) % 360) // 30`` and labelled the result "o'clock". That is a
COUNTER-clockwise clock. Real clock faces run clockwise, so every label except 12
and 6 came out mirrored left-for-right, and a whole night's wedge analysis described
the open side as the wrong side. Production control code was never affected -- it
uses bearings -- but the written record was, and the next step was going to be a
direction-choosing primitive.

This project has been bitten by the same class in the control path before: escape
bearings once arrived in the LASER frame, so recovery rungs steered toward a MIRROR
of open space. A mirrored bearing is not a cosmetic error; it is the error that sends
a robot the wrong way.

THE CONVENTION, stated so it can be checked rather than assumed:

* Bearings are ROS REP-103 body frame: +x forward, **+y LEFT**, yaw CCW positive.
* Clock positions are read off a clock FACE, which runs CLOCKWISE: 12 ahead,
  3 to the RIGHT, 6 behind, 9 to the LEFT.
* Therefore clock number = ``(-bearing_deg / 30) mod 12`` -- note the MINUS. A
  bearing of -90 deg (physically right) is 3 o'clock, not 9.

Anything that prints an "o'clock" anywhere in this project routes through here.
"""

import math

__all__ = ["bearing_deg_to_clock", "clock_to_bearing_deg", "SECTOR_DEG"]

SECTOR_DEG = 30.0  # one clock position


def bearing_deg_to_clock(bearing_deg: float) -> int:
    """Clock position (1..12) for a body-frame bearing in degrees, +y LEFT.

    12 = straight ahead, 3 = right, 6 = behind, 9 = left. Each clock position is a
    30 deg sector centred on its bearing, so the boundaries sit at +/-15 deg.
    """
    clock = int(((-float(bearing_deg) + SECTOR_DEG / 2.0) % 360.0) // SECTOR_DEG)
    return 12 if clock == 0 else clock


def clock_to_bearing_deg(clock: int) -> float:
    """Centre bearing in degrees for a clock position, wrapped to (-180, 180]."""
    bearing = -(float(clock) % 12.0) * SECTOR_DEG
    return (bearing + 180.0) % 360.0 - 180.0


def bearing_of_point(x: float, y: float) -> float:
    """Body-frame bearing in degrees for a point, +y LEFT. Convenience so callers
    never hand-roll ``atan2`` with the arguments the wrong way round."""
    return math.degrees(math.atan2(float(y), float(x)))
