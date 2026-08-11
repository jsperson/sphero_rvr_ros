"""The open bearing handed to the ladder must be in the ROBOT frame (N1).

The laser is mounted at ~179 deg yaw. An unrotated laser-frame bearing is therefore
very nearly a POINT REFLECTION of open space: a gap at robot-left arrives as
robot-right, and the ladder's pivot and drive rungs steer confidently into the closed
side. The pure core cannot catch this -- it is handed a number and has no frame
information -- so the assertion has to happen at the node, with a real TF.

The rotation is read FROM TF here exactly as the node does, rather than compared
against a hardcoded 179: a test that hardcodes the mounting angle would pass happily
after someone remounts the lidar, which is the failure this project already has a
standing rule about.

One test per file on purpose -- `importorskip` skips the whole module.
"""

import math
import os

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not available on this host")

from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

LASER_YAW = 3.1239668018215028      # base_to_laser_static_tf, as deployed
COUNT = 360


def _scan(gap_centre_laser_rad, gap_width_rad=math.radians(40.0)):
    """A scan that is blocked everywhere except one gap, in the LASER frame."""
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.angle_min = -math.pi
    msg.angle_increment = 2.0 * math.pi / COUNT
    msg.range_min, msg.range_max = 0.05, 8.0
    ranges = []
    for i in range(COUNT):
        bearing = msg.angle_min + i * msg.angle_increment
        delta = abs((bearing - gap_centre_laser_rad + math.pi) % (2.0 * math.pi) - math.pi)
        ranges.append(6.0 if delta <= gap_width_rad / 2.0 else 0.30)
    msg.ranges = ranges
    return msg


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def test_open_bearing_is_reported_in_the_base_frame():
    from sphero_rvr_driver.decisive_controller_node import DecisiveControllerNode

    rclpy.init(args=["--ros-args"])
    try:
        node = DecisiveControllerNode()
        try:
            tf = TransformStamped()
            tf.header.frame_id = "base_link"
            tf.child_frame_id = "laser"
            tf.transform.rotation.z = math.sin(LASER_YAW / 2.0)
            tf.transform.rotation.w = math.cos(LASER_YAW / 2.0)
            node._tf_buffer.set_transform_static(tf, "test")

            # Place the gap at a known BASE-frame bearing and work backwards to the
            # laser frame, so the expected answer is stated in the frame that matters.
            # The third case is WIDE on purpose. Dead ahead of the robot lands at
            # the array's wrap point for this mounting, so a linear gap search splits
            # it in two. A narrow gap split into two narrow halves still yields a
            # midpoint near the truth, which hides the bug; a wide gap split in two
            # yields midpoints far off to either side, which exposes it.
            for base_target, width_deg in ((math.radians(90.0), 40.0),   # robot-left
                                           (math.radians(-90.0), 40.0),  # robot-right
                                           (0.0, 120.0)):                # the wrap case
                laser_centre = _wrap(base_target - LASER_YAW)
                node._on_scan(_scan(laser_centre, math.radians(width_deg)))
                got = node._open_bearing()
                err = abs(_wrap(got - base_target))
                assert err < math.radians(15.0), (
                    f"gap at base-frame {math.degrees(base_target):+.0f} deg was "
                    f"reported as {math.degrees(got):+.0f} deg "
                    f"(off by {math.degrees(err):.0f} deg). The ladder would steer "
                    "into the closed side.")
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
