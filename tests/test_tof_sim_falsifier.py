"""THE ACCEPTANCE TEST FOR THE ToF SIM, and it is a falsifier before it is anything.

The sim exists for one class of obstacle: the one BELOW the lidar's plane. If it
cannot produce an obstacle the lidar misses and the ToF catches, it is simulating
nothing worth having, and every scenario built on it would be decoration.

So the first test is not "the sim runs" -- it is "the sim reproduces the
disagreement that motivates it, and reproduces it FOR THE RIGHT REASON." The
control matters as much as the case: raise the same box above the lidar plane and
the lidar must see it. Without that, a sim that simply never returns a lidar hit
would pass the headline test and be worthless.

Both sensors are raycast against ONE world. If they ever disagree for a reason
other than the obstacle's height, the sim is lying, and `test_the_two_sensors_
agree_when_the_obstacle_is_tall` is the assertion that would catch it.
"""

from __future__ import annotations

import math

import pytest

from sphero_rvr_core.tof_frame import (ObstacleDetector, TofConfig, expected_floor_m,
                                       scan_min_by_column)
from sphero_rvr_core.tof_sim import Box, World, simulate_scan, simulate_tof_frame

CFG = TofConfig()
# 60 deg of bearings at the rig's angular resolution, centred on the nose.
BEARINGS = [math.radians(d) for d in range(-45, 46)]
RANGE_MAX = 6.0

# A box 6 cm tall at half a metre: below the 0.1905 m lidar plane, inside the ToF's
# downward cone. The height is the ONLY thing that makes it invisible to one sensor.
LOW_BOX = Box(x_min=0.45, x_max=0.55, y_min=-0.12, y_max=0.12, z_min=0.0, z_max=0.06)
# The same box, the same place, tall enough to break the lidar's plane. The control.
TALL_BOX = Box(x_min=0.45, x_max=0.55, y_min=-0.12, y_max=0.12, z_min=0.0, z_max=0.40)


def _lidar_sees_something(world):
    ranges = simulate_scan(world, BEARINGS, CFG, range_max_m=RANGE_MAX)
    return min(ranges) < RANGE_MAX - 1e-6, min(ranges)


def _tof_obstacles(world, frames=6):
    """Run the REAL detector, with the real lidar background, for enough frames that
    rule B's N-of-M window can confirm. Nothing here reimplements a rule."""
    detector = ObstacleDetector(CFG)
    ranges = simulate_scan(world, BEARINGS, CFG, range_max_m=RANGE_MAX)
    column_min = scan_min_by_column(BEARINGS, ranges, CFG)
    verdict = {}
    for _ in range(frames):
        verdict = detector.update(simulate_tof_frame(world, CFG), column_min)
    return verdict


def test_the_empty_world_is_quiet():
    """Calibrate the NO with a YES: a detector that never reports anything would pass
    the headline test by accident. Open floor must produce no obstacle."""
    verdict = _tof_obstacles(World(boxes=[]))
    assert verdict["obstacles"] == [], (
        f"flat floor alone produced obstacles {verdict['obstacles']} -- the sim's floor "
        f"model disagrees with the detector's expected background")
    seen, nearest = _lidar_sees_something(World(boxes=[]))
    assert not seen, f"lidar found something at {nearest:.2f} m in an empty world"


def test_the_falsifier_the_lidar_misses_what_the_tof_catches():
    """THE ACCEPTANCE TEST. A 6 cm box at 0.5 m: lidar clear, ToF obstacle."""
    world = World(boxes=[LOW_BOX])

    seen, nearest = _lidar_sees_something(world)
    assert not seen, (
        f"the lidar SAW the low box at {nearest:.2f} m -- it sweeps a horizontal plane "
        f"at {CFG.lidar_plane_m} m and the box stops at {LOW_BOX.z_max} m, so either the "
        f"scan model or the box is wrong")

    verdict = _tof_obstacles(world)
    assert verdict["obstacles"], (
        "the ToF did not detect a 6 cm box at 0.5 m -- if the lidar cannot see it and "
        "the ToF cannot either, this sim reproduces nothing worth testing")


def test_the_two_sensors_agree_when_the_obstacle_is_tall():
    """THE CONTROL, and the assertion that catches a lying sim: same box, same place,
    40 cm tall. A sim whose lidar never returned a hit would pass the falsifier above
    and fail here.

    THE FIRST VERSION OF THIS TEST ASSERTED THE DETECTOR WOULD ALSO FIRE, AND THAT WAS
    MY WRONG EXPECTATION, NOT A DEFECT. It does not, and it should not: rule B reports
    what the lidar MISSES, and here the lidar sees the box at 0.45 m, so the two agree
    and there is nothing to report. The distinction the corrected test pins is the one
    that matters -- the ToF FRAME sees the box (raw readings ~0.35 m), while the
    DETECTOR deliberately stays quiet. "The ToF missed it" would have been a false
    description of a correct refusal.
    """
    world = World(boxes=[TALL_BOX])

    seen, nearest = _lidar_sees_something(world)
    assert seen, "the lidar missed a 40 cm box at 0.5 m -- the scan model is not raycasting"
    assert 0.4 < nearest < 0.6, f"lidar range {nearest:.2f} m is not the box's face at 0.45 m"

    # Both sensors are looking at the same world: the ToF's own readings must show it.
    frame = simulate_tof_frame(world, CFG)
    empty = simulate_tof_frame(World(boxes=[]), CFG)
    upper = [frame[r * 8 + c] for r in range(5) for c in range(1, 7)]
    upper_empty = [empty[r * 8 + c] for r in range(5) for c in range(1, 7)]
    assert min(upper) < 400, (
        f"the ToF frame does not show a 40 cm box at 0.5 m (nearest upper-row reading "
        f"{min(upper)} mm) -- the two sensors are not raycasting the same world")
    assert min(upper) < min(upper_empty), "the box did not change the ToF frame at all"

    # ... and the detector is RIGHT to say nothing about it.
    verdict = _tof_obstacles(world)
    assert verdict["obstacles"] == [], (
        f"rule B reported {verdict['obstacles']} for an obstacle the LIDAR can see at "
        f"{nearest:.2f} m -- it is a disagreement rule and there is no disagreement")


def test_the_sim_reproduces_the_certified_floor_model_to_the_millimetre():
    """CALIBRATE THE INSTRUMENT AGAINST THE OWNER'S OWN NUMBER before trusting it.

    `expected_floor_m` is `tof_frame`'s certified answer for what a zone reads over
    flat floor. The simulator must reproduce it exactly on the centre column, because
    it is supposed to be USING that geometry rather than a second author's version of
    it. A sim that merely looked plausible would drift here.
    """
    frame = simulate_tof_frame(World(boxes=[]), CFG)
    checked = 0
    for row in range(8):
        expected = expected_floor_m(row, 4, CFG)
        if expected is None:                      # ray never reaches the floor
            continue
        got_mm = frame[row * 8 + 4]
        assert got_mm == int(round(expected * 1000.0)), (
            f"row {row} centre: sim {got_mm} mm vs certified {expected * 1000:.1f} mm")
        checked += 1
    assert checked >= 3, f"only {checked} rows had a floor intersection to compare"


def test_height_is_the_only_difference_between_the_two_cases():
    """Pins the claim the whole sim rests on: the two boxes differ in z_max and in
    nothing else, so the lidar's change of verdict can only be about height."""
    assert (LOW_BOX.x_min, LOW_BOX.x_max) == (TALL_BOX.x_min, TALL_BOX.x_max)
    assert (LOW_BOX.y_min, LOW_BOX.y_max) == (TALL_BOX.y_min, TALL_BOX.y_max)
    assert LOW_BOX.z_min == TALL_BOX.z_min
    assert LOW_BOX.z_max < CFG.lidar_plane_m < TALL_BOX.z_max
