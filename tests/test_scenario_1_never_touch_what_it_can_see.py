"""SCENARIO 1 of Scott's nine, 2026-08-31.

  REQUIREMENT, his words: "It should never touch something on the floor it could
  have seen."

  BAR, PRE-REGISTERED before this ran, and written from his words rather than from
  current behaviour: with a 6 cm box squarely in the swept path, the commanded
  forward velocity must reach EXACTLY 0.000 while the box is still ahead of the
  footprint's front edge (`footprint_front_m` = 0.0965 m from base_link).
  Contacts = 0.

  PREDICTION FILED BEFORE EXECUTION: UNKNOWN. `06af706` proved DETECTION -- the
  lidar is blind to this box and rule B confirms it -- but whether the brake drives
  output to zero BEFORE the footprint arrives is a different claim that nothing had
  measured.

WHAT THIS COMPOSES, all production code, no rule re-implemented here:
  tof_sim.simulate_tof_frame / simulate_scan   the world both sensors look at
  tof_frame.ObstacleDetector                    rules A and B, N-of-M confirmation
  tof_frame.zone_point                          the same cloud tof_node publishes
  low_obstacle_brake.swept_path_obstacle        the arc the rover will actually drive
  low_obstacle_brake.forward_speed_scale        the brake's own scale
The only new code is the closed loop: step the rover, move the world under it,
ask again.

WHAT IT DOES NOT TEST, and the limit is the same one the sim carries: this is the
DECISION LOGIC end to end, not the ROS plumbing. Topic wiring, QoS, TF timing and
the 0.30 s staleness bound are untouched here, and the 2.53 Hz starvation of D71 is
invisible to it. A pass here is a statement about the arithmetic, and the on-robot
integration run remains owed.

RULE REACH, load-bearing for reading this result: rule A applies only to rows 5-7,
whose floor intersections sit at 0.219-0.330 m. A box first seen at 0.45 m is
detected by rule B, which needs a healthy lidar background to disagree with -- which
this scenario provides. A degraded scan here would test a refusal, not a brake.
"""

from __future__ import annotations

import math

import pytest
import yaml

from sphero_rvr_core.low_obstacle_brake import (BlindBandHold, forward_speed_scale,
                                                swept_path_obstacle)
from sphero_rvr_core.tof_frame import (ObstacleDetector, TofConfig, ZONES,
                                       blind_band_outer_range_m, scan_min_by_column,
                                       zone_point)
from sphero_rvr_core.tof_sim import Box, World, simulate_scan, simulate_tof_frame

import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEPLOYED = yaml.safe_load(
    open(os.path.join(REPO, "config", "collision_stop.yaml"))
)["lidar_collision_stop_supervisor"]["ros__parameters"]

CFG = TofConfig()
BEARINGS = [math.radians(d) for d in range(-45, 46)]
RANGE_MAX = 6.0
DT = 0.1

# Every threshold from the DEPLOYED config, never a literal retyped here: a bar
# tested against numbers this file chose would certify this file.
FOOTPRINT_FRONT = float(DEPLOYED["footprint_front_m"])
STOP_M = float(DEPLOYED["low_obstacle_stop_distance_m"])
SLOW_M = float(DEPLOYED["low_obstacle_slow_distance_m"])
MIN_SCALE = float(DEPLOYED["low_obstacle_min_forward_scale"])
HALF_W = float(DEPLOYED["low_obstacle_half_width_m"])
MIN_R = float(DEPLOYED["low_obstacle_min_range_m"])
MAX_R = float(DEPLOYED["low_obstacle_max_range_m"])
CRUISE = float(DEPLOYED["max_forward_mps"])

BOX_X = 0.45          # front face, metres ahead of base_link at t=0
BOX_H = 0.06          # below the 0.1905 m lidar plane by construction


def _approach(box_height_m=BOX_H, cruise=CRUISE, steps=200):
    """Drive at the box until the brake zeroes the command or we run out of steps.

    Returns (travelled_m, out_history, nearest_history). The world moves under the
    rover rather than the rover moving through a map: identical geometry, and it
    keeps every reading in the robot frame the sensors publish in.
    """
    detector = ObstacleDetector(CFG)
    # THE HOLD IS PART OF THE BRAKE AND LEAVING IT OUT INDICTS THE WRONG THING.
    # The first version of this loop omitted it and the rover drove straight over the
    # box -- which looked like a failed requirement and was actually a missing
    # mechanism. `BlindBandHold` exists for exactly this: D39, 2026-08-15, a table leg
    # tracked to 0.181 m that then LEFT the sensor's visibility band, after which
    # silence read as clearance and the rover drove 90 mm back into it. Constants
    # derived the way the node derives them, never retyped.
    hold = BlindBandHold(
        blind_band_outer_range_m(CFG),
        float(DEPLOYED["max_forward_mps"]) * float(DEPLOYED["low_obstacle_max_age_s"]),
        HALF_W, MIN_R, MAX_R,
    )
    travelled = 0.0
    outs, nearests = [], []
    for _ in range(steps):
        box = Box(x_min=BOX_X - travelled, x_max=BOX_X + 0.10 - travelled,
                  y_min=-0.12, y_max=0.12, z_min=0.0, z_max=box_height_m)
        world = World(boxes=[box])
        ranges = simulate_scan(world, BEARINGS, CFG, range_max_m=RANGE_MAX)
        column_min = scan_min_by_column(BEARINGS, ranges, CFG)
        frame = simulate_tof_frame(world, CFG)
        verdict = detector.update(frame, column_min)

        cloud = []
        for row, col in verdict["obstacles"]:
            point = zone_point(row, col, frame[row * ZONES + col], CFG)
            if point is not None:
                # The node subscribes with field_names=("x", "y"): the brake works in
                # the ground plane, so the cloud is projected exactly as it is on the
                # robot rather than handed a shape this test invented.
                cloud.append((point[0], point[1]))

        # Mirrors collision_stop_node._apply_low_obstacle_brake: the hold updates on
        # EVERY cycle and its nearest_m is what the brake acts on -- a live range, a
        # held belief, or None.
        result = hold.update(cloud, True, cruise, 0.0,
                             (outs[-1] * DT if outs else 0.0, 0.0, 0.0))
        nearest = result.nearest_m
        out = cruise * forward_speed_scale(nearest, STOP_M, SLOW_M, MIN_SCALE)
        outs.append(out)
        nearests.append(nearest)
        if out == 0.0:
            break
        travelled += out * DT
    return travelled, outs, nearests


def test_scenario_1_the_brake_stops_before_the_footprint_reaches_the_box():
    travelled, outs, nearests = _approach()

    assert outs[-1] == 0.0, (
        f"the brake never reached zero: last output {outs[-1]:.3f} m/s after "
        f"{travelled:.3f} m, nearest seen {min(n for n in nearests if n is not None):.3f} m")

    gap = (BOX_X - travelled) - FOOTPRINT_FRONT
    assert gap > 0.0, (
        f"CONTACT: the footprint front reached the box. travelled {travelled:.3f} m, "
        f"box face at {BOX_X - travelled:.3f} m, footprint front {FOOTPRINT_FRONT} m")
    # Reported, not asserted as a second bar -- Scott's requirement is "never touch",
    # and how much room was left over is scenario 6's question, not this one.
    print(f"\nSCENARIO 1: stopped with {gap * 100:.1f} cm between footprint and box "
          f"after {travelled:.3f} m of approach")


def test_scenario_1_control_a_box_it_could_not_have_seen_is_not_claimed():
    """His words are "something it COULD HAVE SEEN". A box outside the ToF's reach is
    outside the requirement, and a suite that stopped for it anyway would be scoring
    a different promise. Beyond `low_obstacle_max_range_m` the brake must not act."""
    detector = ObstacleDetector(CFG)
    far = Box(x_min=1.20, x_max=1.30, y_min=-0.12, y_max=0.12, z_min=0.0, z_max=BOX_H)
    world = World(boxes=[far])
    ranges = simulate_scan(world, BEARINGS, CFG, range_max_m=RANGE_MAX)
    column_min = scan_min_by_column(BEARINGS, ranges, CFG)
    frame = simulate_tof_frame(world, CFG)
    for _ in range(6):
        verdict = detector.update(frame, column_min)
    cloud = [(p[0], p[1]) for p in (zone_point(r, c, frame[r * ZONES + c], CFG)
                                    for r, c in verdict["obstacles"]) if p is not None]
    nearest = swept_path_obstacle(cloud, CRUISE, 0.0, HALF_W, MIN_R, MAX_R)
    assert nearest is None or nearest > MAX_R, (
        f"the brake considered a point at {nearest:.3f} m, beyond its own "
        f"{MAX_R} m window")
