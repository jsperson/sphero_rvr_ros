"""The falsifier gate for the placement fix, against REAL tf2. Pi-only (needs rclpy).

A rig that cannot reproduce the known-bad cannot certify the fix
(`falsifier-before-certifier`). The known-bad here is precise: contact_marker v1's
exact-stamp lookup shipped green through a rig whose map->odom was a STATIC
transform -- timeless to tf2, so ExtrapolationException was structurally impossible
-- and then lost 3/3 field contacts under real SLAM lag. So before the fixed policy's
success is admitted as evidence, this file makes real tf2, fed a map->odom that
trails wall clock like SLAM's does, throw the same ExtrapolationException at the
same shape of lookup (case A). Only then does it run the fixed policy on the same
buffer and require placement (case B).

In-process: one broadcaster thread standing in for sim_laggy_map_tf's output shape,
one buffer. No launch, no external nodes, nothing left on the graph afterwards
(`pi-test-nodes-outlive-pytest` is the reason for the try/finally shutdown).
"""

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import TransformStamped              # noqa: E402
from rclpy.duration import Duration                        # noqa: E402
from rclpy.time import Time                                # noqa: E402
from tf2_ros import Buffer, ExtrapolationException         # noqa: E402

from sphero_rvr_core.contact_marking import (               # noqa: E402
    PoseDataLagsStamp,
    resolve_contact_pose,
)

LAG_S = 0.090   # the run-3d contact staleness class (69-87 ms, rounded up)


def _tf(stamp_time, frame, child, x=0.0):
    msg = TransformStamped()
    msg.header.stamp = stamp_time.to_msg()
    msg.header.frame_id = frame
    msg.child_frame_id = child
    msg.transform.translation.x = x
    msg.transform.rotation.w = 1.0
    return msg


@pytest.fixture
def laggy_buffer():
    """A tf2 Buffer holding map->odom that trails `now` by LAG_S -- the field shape."""
    rclpy.init()
    try:
        buffer = Buffer()
        node = rclpy.create_node("placement_lag_test")
        clock_now = node.get_clock().now()
        # ~2 s of history at 20 Hz, newest sample LAG_S behind now; odom->base_link
        # fresh (the EKF runs ahead of SLAM in the field too).
        for i in range(40):
            t = clock_now - Duration(seconds=LAG_S + (39 - i) * 0.05)
            buffer.set_transform(_tf(t, "map", "odom"), "test")
        # odom->base_link fresh AND spanning: the latest-common-time composite lands
        # at map->odom's newest stamp, and tf2 must be able to interpolate odom->base
        # THERE, not just at `now` -- one lone sample would fail the fallback lookup
        # for a reason the field never has (the EKF publishes at 30+ Hz).
        buffer.set_transform(
            _tf(clock_now - Duration(seconds=0.5), "odom", "base_link", x=0.5), "test"
        )
        buffer.set_transform(_tf(clock_now, "odom", "base_link", x=0.5), "test")
        yield buffer, node, clock_now
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_case_a_real_tf2_reproduces_the_field_refusal(laggy_buffer):
    """THE FALSIFIER: v1's lookup shape (exact stamp, stamp newer than the feed)
    must fail against real tf2 under lag. If this passes vacuously -- if tf2 serves
    the lookup -- the laggy rig is not producing the field condition and nothing
    downstream certifies anything."""
    buffer, node, now = laggy_buffer
    with pytest.raises(ExtrapolationException):
        buffer.lookup_transform("map", "base_link", now)


def test_case_a2_a_static_map_odom_cannot_fail_this_way():
    """WHY THE OLD RIG WAS VACUOUS, demonstrated rather than asserted: the same
    lookup against a STATIC map->odom succeeds at any stamp, including one far in
    the future of everything the buffer holds."""
    rclpy.init()
    try:
        buffer = Buffer()
        node = rclpy.create_node("placement_static_test")
        now = node.get_clock().now()
        static = _tf(now - Duration(seconds=3600.0), "map", "odom")
        buffer.set_transform_static(static, "test")
        buffer.set_transform(_tf(now, "odom", "base_link", x=0.5), "test")
        transform = buffer.lookup_transform("map", "base_link", now)
        assert transform.transform.translation.x == pytest.approx(0.5)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_case_b_the_fixed_policy_places_under_the_same_lag(laggy_buffer):
    """The certifier, admitted only beside case A: the hybrid policy on the SAME
    buffer places via the fallback with the injected staleness, and the pose is the
    buffer's real answer."""
    buffer, node, now = laggy_buffer
    stamp_s = now.nanoseconds * 1e-9

    def exact():
        try:
            transform = buffer.lookup_transform("map", "base_link", now)
        except ExtrapolationException as exc:
            raise PoseDataLagsStamp(str(exc)) from exc
        t = transform.transform.translation
        return t.x, t.y, 0.0

    def latest():
        transform = buffer.lookup_transform("map", "base_link", Time())
        h = transform.header.stamp
        t = transform.transform.translation
        return (t.x, t.y, 0.0), h.sec + h.nanosec * 1e-9

    resolved = resolve_contact_pose(exact, latest, stamp_s)
    assert resolved.path == "fallback"
    assert resolved.staleness_s == pytest.approx(LAG_S, abs=0.02)
    assert resolved.x == pytest.approx(0.5, abs=1e-6)

