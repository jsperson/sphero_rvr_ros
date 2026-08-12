"""The PAIRING test for the open-bearing seam: absence is published, not inferred.

`_open_bearing()` used to answer 0.0 for three different facts -- "no scan yet",
"the scan is too old to trust" and "the way out is dead ahead" -- and every consumer
had to guess which one it had been handed. The ladder now decides its ESCAPE ORDER
from that value: a known gap outside the pivot tolerance opens with a pivot toward it,
anything else backs out along the entry path. A fabricated dead-ahead bearing would
therefore send the rover pivoting toward a direction nothing ever measured.

This is the project's recurring defect class (three mission-killers in two days from
one component inferring another's state), and the rule it produced is that the OWNER
of a fact publishes the fact, including its absence. So this test asserts both halves
at once -- the publisher answers None, and the consumer treats None as unknown --
because either half alone can drift silently past the other.

One test file per topic on purpose: `importorskip` skips the whole module, and the
pure-core half of the pairing must still run on a host with no ROS (it does, in
tests/test_stall_ladder.py::test_an_unknown_bearing_reverses_rather_than_guessing).
"""

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not available on this host")

from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from sphero_rvr_core.stall_ladder import (  # noqa: E402
    REVERSE_STRAIGHT, StallLadder,
)

LASER_YAW = 3.1239668018215028      # base_to_laser_static_tf, as deployed
COUNT = 360


def _scan_with_a_gap_at(base_bearing_rad):
    """Blocked everywhere except one wide gap at a known BASE-frame bearing."""
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.angle_min = -math.pi
    msg.angle_increment = 2.0 * math.pi / COUNT
    msg.range_min, msg.range_max = 0.05, 8.0
    centre = (base_bearing_rad - LASER_YAW + math.pi) % (2.0 * math.pi) - math.pi
    ranges = []
    for i in range(COUNT):
        bearing = msg.angle_min + i * msg.angle_increment
        delta = abs((bearing - centre + math.pi) % (2.0 * math.pi) - math.pi)
        ranges.append(6.0 if delta <= math.radians(20.0) else 0.30)
    msg.ranges = ranges
    return msg


def test_an_unplaceable_or_stale_bearing_is_published_as_absent():
    from sphero_rvr_driver.decisive_controller_node import DecisiveControllerNode

    rclpy.init(args=["--ros-args"])
    try:
        node = DecisiveControllerNode()
        try:
            # 1. Nothing has arrived yet: absent, not "dead ahead".
            assert node._open_bearing() is None, (
                "a controller that has never seen a scan reported a bearing")

            # 2. A scan with no TF to place it in the base frame: still absent. The
            #    node must not fall back to the laser frame, where this mounting makes
            #    an unrotated bearing very nearly a point reflection of open space.
            node._on_scan(_scan_with_a_gap_at(math.radians(-71.4)))
            assert node._open_bearing() is None, (
                "a bearing was reported for a scan that could not be placed")

            # 3. With TF, a real bearing.
            tf = TransformStamped()
            tf.header.frame_id = "base_link"
            tf.child_frame_id = "laser"
            tf.transform.rotation.z = math.sin(LASER_YAW / 2.0)
            tf.transform.rotation.w = math.cos(LASER_YAW / 2.0)
            node._tf_buffer.set_transform_static(tf, "test")
            node._on_scan(_scan_with_a_gap_at(math.radians(-71.4)))
            fresh = node._open_bearing()
            assert fresh is not None
            assert abs(fresh - math.radians(-71.4)) < math.radians(15.0)

            # 4. The same bearing, aged past the freshness bound: absent again.
            node._open_bearing_at = time.monotonic() - (
                node._open_bearing_max_age_s + 0.5)
            assert node._open_bearing() is None, (
                "a stale gap was still being offered to the ladder — an escape aimed "
                "at where the room used to be")
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_the_ladder_treats_absence_as_unknown_rather_than_dead_ahead():
    """The consumer's half, against the exact value the publisher emits."""
    ladder = StallLadder()
    result = None
    for i in range(40):
        result = ladder.step(x=0.0, y=0.0, yaw=0.0, now=i / 20.0,
                             commanding=True, output_moving=False,
                             open_bearing_rad=None)
        if result.action == "rung":
            break
    assert result.rung == REVERSE_STRAIGHT, (
        "an unknown bearing was treated as a real one and the rover pivoted toward a "
        "direction nothing measured")
