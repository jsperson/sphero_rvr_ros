"""Freeze marks must land ON the obstacle, and cover the footprint we proved blocked.

Both axes drifted from the approved design (docs/design_d25_freeze.md 2b) and both
survived three reviews, because the design note lived in a scratchpad and an
uncommitted design cannot be diffed against an implementation:

  * PLACEMENT: the mark was stamped at the robot's CENTRE, so it sat
    footprint_front_m (0.11 m deployed) behind the thing it marked, along the
    approach heading. The costmap got a point where the robot was standing.
  * EXTENT: one point, where the design specifies a disc of radius ~robot_radius --
    "mark the footprint we proved is blocked, not a guess about the object".

Together those meant an approach from a slightly different angle reached the same
physical obstacle without crossing a mark, which is the contact-by-contact
face-walking observed against Scott's chair in gauntlet run 20260811_093818.

One test per file: `importorskip` skips the whole module.
"""

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not available on this host")


def _cloud_points(msg):
    import struct
    return [struct.unpack_from("<fff", bytes(msg.data), 12 * i)[:2]
            for i in range(msg.width)]


def test_marks_land_on_the_obstacle_and_cover_the_blocked_footprint():
    from sphero_rvr_driver.decisive_controller_node import DecisiveControllerNode

    rclpy.init(args=["--ros-args"])
    try:
        node = DecisiveControllerNode()
        try:
            # Run 093818's first contact: the rover froze heading roughly north-west
            # at the chair's wheel base.
            robot_x, robot_y, yaw = -0.91, -1.115, math.radians(150.0)

            fx, fy = node._freeze_mark_pose(robot_x, robot_y, yaw)
            forward = ((fx - robot_x) * math.cos(yaw)
                       + (fy - robot_y) * math.sin(yaw))
            assert forward > 0.05, (
                f"mark placed {forward:.3f} m along the heading — it is at the robot "
                "CENTRE, so it sits behind the obstacle it is meant to mark")
            assert abs(forward - node._footprint_front_m) < 1e-6

            # And the published geometry must be a disc, not a point.
            captured = []
            node._freeze_cloud_pub.publish = lambda m: captured.append(m)
            # Real monotonic time: the publisher filters the live set against
            # time.monotonic(), so a mark stamped at 0.0 is born expired and the
            # cloud comes out empty. (First version of this test did exactly that
            # and reported "the mark is a single point" when it was really zero.)
            node._record_freeze(fx, fy, time.monotonic())
            assert captured, "no freeze cloud was published"
            pts = _cloud_points(captured[-1])
            assert len(pts) > 1, "the mark is a single point, not the designed disc"

            radii = [math.hypot(px - fx, py - fy) for px, py in pts]
            assert max(radii) >= node._mark_radius_m - 1e-6, (
                f"disc extends only {max(radii):.3f} m, design says "
                f"{node._mark_radius_m:.2f} m")
            # Dense enough that a 0.05 m costmap cell cannot slip between samples.
            ring = sorted(r for r in radii if r > node._mark_radius_m - 1e-6)
            assert len(ring) >= 8, "disc rim too sparsely sampled to fill cells"
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
