"""Node-level revert-proofs for the gentle turn-away (design note item 1).

The pure core is covered on any host by `test_avoidance_steering.py`. Three claims
can only be made at the node, with a real TF and real messages:

  1. lidar bearings reach the steering law in the BASE frame (the N1 trap: the laser
     is mounted at ~179 deg, so an unrotated bearing leans the rover TOWARD the
     obstacle);
  2. no camera geometry steers the robot at all -- the charter state (9ab4b88), asserted
     as a capability and always beside a lidar control so it cannot pass vacuously;
  3. a running ladder rung owns cmd_vel outright -- the steering offset cannot reach
     the wire while an escape is in progress.

rclpy is not installed on the workstation, so this module skips there and runs on
the Pi, same as `test_open_bearing_frame.py`.
"""

import math

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not available on this host")

from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
import sensor_msgs_py.point_cloud2 as pc2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

LASER_YAW = 3.1239668018215028      # base_to_laser_static_tf, as deployed
COUNT = 360


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _scan_with_obstacle_at(base_bearing_rad, range_m, clear_range=6.0):
    """A scan that is clear everywhere except one narrow return, placed at a known
    BASE-frame bearing and converted back into the LASER frame -- so the expected
    answer is stated in the frame that matters."""
    laser_bearing = _wrap(base_bearing_rad - LASER_YAW)
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.angle_min = -math.pi
    msg.angle_increment = 2.0 * math.pi / COUNT
    msg.range_min, msg.range_max = 0.05, 8.0
    ranges = []
    for i in range(COUNT):
        bearing = msg.angle_min + i * msg.angle_increment
        delta = abs(_wrap(bearing - laser_bearing))
        ranges.append(range_m if delta <= math.radians(3.0) else clear_range)
    msg.ranges = ranges
    return msg


def _node():
    from sphero_rvr_driver.decisive_controller_node import DecisiveControllerNode
    node = DecisiveControllerNode()
    tf = TransformStamped()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = "laser"
    tf.transform.rotation.z = math.sin(LASER_YAW / 2.0)
    tf.transform.rotation.w = math.cos(LASER_YAW / 2.0)
    node._tf_buffer.set_transform_static(tf, "test")
    return node


def test_lidar_blockers_reach_the_law_in_the_base_frame():
    """An obstacle at robot-LEFT must be reported at robot-left. Unrotated it would
    arrive at robot-right, and the rover would lean left -- into it."""
    rclpy.init(args=["--ros-args"])
    try:
        node = _node()
        try:
            for base_bearing in (math.radians(6.0), math.radians(-6.0)):
                node._on_scan(_scan_with_obstacle_at(base_bearing, 0.60))
                blocker = node._nearest_blocker(max_relevant_m=5.0)
                assert blocker is not None, "an obstacle in the corridor was missed"
                rng, bearing = blocker
                assert rng == pytest.approx(0.60, abs=0.02)
                assert abs(_wrap(bearing - base_bearing)) < math.radians(5.0), (
                    f"obstacle at base {math.degrees(base_bearing):+.0f} deg reported "
                    f"at {math.degrees(bearing):+.0f} deg -- the steering law would "
                    "lean the wrong way")
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_blocked_scan_still_yields_a_blocker():
    """The gap search returns early when nothing at all is open. Blockers must be
    extracted before that return: a scan with no gap is precisely a scan with
    something in the way, and it is the last moment steering could help."""
    rclpy.init(args=["--ros-args"])
    try:
        node = _node()
        try:
            node._on_scan(_scan_with_obstacle_at(0.0, 0.55, clear_range=0.55))
            assert node._nearest_blocker(max_relevant_m=5.0) is not None
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_the_camera_cannot_steer_the_robot_at_the_deployed_default():
    """THE CHARTER STATE, asserted as a capability rather than as wording (9ab4b88).

    This replaced a freshness test -- "a stale camera cloud stops steering the robot" --
    which asserted that a FRESH camera cloud steers it. That stopped being true when the
    camera charter of 2026-08-16 removed the monocular pipeline from navigation:
    `avoid_camera_enable` defaults False and `_nearest_blocker` returns before any camera
    geometry is consulted. The old test failed on the Pi from that day, and only on the
    Pi, because rclpy is absent on the workstation -- the tests that run nowhere but the
    robot were encoding a world that had been deliberately ended.

    THE LIDAR CONTROL IS THE POINT. Asserting "the camera does not steer" alone would
    pass just as happily if `_nearest_blocker` were broken outright, or if the cloud never
    reached the callback -- a green that means nothing. So the same node, in the same
    breath, must still steer for a LIDAR return: what is proven is that the camera
    specifically is inert, not that steering is."""
    rclpy.init(args=["--ros-args"])
    try:
        node = _node()
        try:
            assert node._avoid_camera_enable is False, (
                "the charter's default has changed; this test guards the disabled state")

            header = Header()
            header.frame_id = "base_link"
            # A point that WOULD steer if the camera had authority: comfortably inside
            # the engagement radius and outside the near band the old filter excluded.
            node._on_camera_cloud(pc2.create_cloud_xyz32(header, [(0.60, 0.05, 0.0)]))
            assert node._nearest_blocker(max_relevant_m=5.0) is None, (
                "a camera cloud reached the steering law -- the charter is breached")

            # ...and the node is not simply deaf.
            node._on_scan(_scan_with_obstacle_at(0.0, 0.55, clear_range=0.55))
            assert node._nearest_blocker(max_relevant_m=5.0) is not None, (
                "the lidar control failed, so the camera assertion above proves nothing")
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_no_camera_geometry_at_any_range_reaches_the_steering_law():
    """The charter is a statement about the SOURCE, not about a range window.

    What stood here tested the near-band filter: a point at 0.30 m, inside
    `camera_min_range_m`, must not steer, while a 0.65 m point in the same cloud still
    must. That test earned its place -- it replaced a clear-ray-at-1.8 m version that
    passed for the wrong reason (the 0.90 engagement radius excluded 1.8 m on its own, so
    the assertion held with the range filter deleted, and a mutation run caught it).

    But the filter it guarded is unreachable at the deployed config: `_nearest_blocker`
    returns on `avoid_camera_enable` before any range comparison happens. Keeping the old
    assertions would have meant re-enabling the camera to test a path the charter removed
    from navigation -- testing a world that was deliberately ended, which is the defect
    being fixed here rather than a way to fix it.

    So the claim is widened to the one that is actually true and actually load-bearing:
    NO camera geometry steers, at any range, near band or not. The lidar control is kept
    for the same reason as above -- without it this passes on a node that cannot steer at
    all."""
    rclpy.init(args=["--ros-args"])
    try:
        node = _node()
        try:
            header = Header()
            header.frame_id = "base_link"
            # Both halves of the old test's cloud: the distrusted near band AND the point
            # that used to be legitimate. Neither may reach the steering law now.
            node._on_camera_cloud(pc2.create_cloud_xyz32(
                header, [(0.30, 0.02, 0.0), (0.65, -0.05, 0.0)]))
            assert node._nearest_blocker(max_relevant_m=5.0) is None, (
                "camera geometry reached the steering law at some range")

            node._on_scan(_scan_with_obstacle_at(0.0, 0.55, clear_range=0.55))
            assert node._nearest_blocker(max_relevant_m=5.0) is not None, (
                "the lidar control failed, so the camera assertion above proves nothing")
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_running_rung_owns_cmd_vel():
    """While the ladder is working an escape, what reaches the wire is the rung's
    twist and nothing else. Steering must not be able to add a lean to an escape in
    progress -- two authors for one motion is the failure the ladder was built to
    end.

    Asserted structurally: the rung branch publishes `ladder_result` values and
    `continue`s before the normal command is ever built, and the steering offset is
    computed with `ladder.active` passed in (which the core zeroes on). Both halves
    are checked here because either one alone can be deleted by an edit that still
    looks reasonable.
    """
    import inspect
    from sphero_rvr_driver.decisive_controller_node import DecisiveControllerNode

    src = inspect.getsource(DecisiveControllerNode._execute)
    rung_branch = src.split('if ladder_result.action == "rung":', 1)
    assert len(rung_branch) == 2, "the rung branch has been renamed or removed"
    body = rung_branch[1].split("twist = Twist()", 1)[1].split("continue", 1)[0]
    assert "ladder_result.linear_x" in body and "ladder_result.angular_z" in body
    assert "_avoid_offset_rad" not in body, (
        "the steering offset must not reach a rung's twist")
    assert "ladder.active" in src, (
        "the steering law must be told when a rung is running")


def test_rung_transitions_are_logged_without_a_throttle():
    """The escalation lines are the ladder's testimony and must never be throttled.

    One throttled logger call used to serve both the ~30 `{rung}_running` lines a
    rung emits in its 3 s budget and the single `{rung}_failed->{next}` transition,
    so the transition almost always landed inside another line's 1 s shadow. Run
    114626 proves the damage: `reverse_arc_running`, `pivot_open_running` and
    `drive_open_running` all appear in its log while NOT ONE `_failed->` line does.
    The record shows rungs 2-4 executing with no trace of how they were entered --
    and it misled this session's own grading before the counts were checked.

    Asserted structurally on the source, because provoking a real escalation needs
    a whole stack; a logger call is not worth one.
    """
    import inspect
    from sphero_rvr_driver.decisive_controller_node import DecisiveControllerNode

    src = inspect.getsource(DecisiveControllerNode._execute)
    branch = src.split('if ladder_result.action == "rung":', 1)[1].split("continue", 1)[0]
    assert 'endswith("_running")' in branch, (
        "the rung logger no longer distinguishes repetition from transitions")
    throttled = [line for line in branch.splitlines()
                 if "throttle_duration_sec" in line]
    assert len(throttled) == 1, (
        "exactly one throttled call expected -- the `_running` repetition")
