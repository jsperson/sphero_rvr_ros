"""D57's close criteria at NODE level, against the real ContactMarkerNode. Pi-only.

Both directions from the register row, because a fix that only removes marks is
half-certified: replaying the 2026-08-19 flight's own commanded twists and stall
deltas must plant ZERO marks (falsifier: the pre-D57 node planted 2 in the door
gap), while a translation stall -- the d45bd24 boot class -- must still plant its
mark, and a rotation stall WITH a fresh ToF return inside the disc is admitted.

In-process: the real node under a MultiThreadedExecutor, a static identity
map->base_link (the rover was stationary at the stalls; static TF is exactly
true here, unlike the placement-lag rig where it hid the defect), and this test
publishing cmd_vel / diagnostics / tof points as the drivers would. try/finally
shutdown per pi-test-nodes-outlive-pytest.
"""

import struct
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # noqa: E402
from geometry_msgs.msg import TransformStamped, Twist                        # noqa: E402
from rclpy.executors import MultiThreadedExecutor                            # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField                          # noqa: E402
from std_msgs.msg import Header                                              # noqa: E402
from tf2_ros import StaticTransformBroadcaster                               # noqa: E402

from sphero_rvr_core.contact_marking import (                                # noqa: E402
    FOOTPRINT_FRONT_M,
    FOOTPRINT_REAR_M,
    contact_mark_centre,
)
from sphero_rvr_driver.contact_marker_node import ContactMarkerNode          # noqa: E402


class Rig:
    """The marker plus this test's fake driver-side publishers."""

    def __init__(self):
        rclpy.init()
        self.marker = ContactMarkerNode()
        self.node = rclpy.create_node("d57_rig")
        static = StaticTransformBroadcaster(self.node)
        identity = TransformStamped()
        identity.header.stamp = self.node.get_clock().now().to_msg()
        identity.header.frame_id = "map"
        identity.child_frame_id = "base_link"
        identity.transform.rotation.w = 1.0
        static.sendTransform(identity)
        self.cmd = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.diag = self.node.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.tof = self.node.create_publisher(PointCloud2, "/tof/points", 10)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.marker)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        self._count = 0
        time.sleep(0.5)   # static TF + discovery settle
        self._publish_count()   # first observation is a BASELINE, never a batch
        time.sleep(0.3)

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.marker.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()
        self._thread.join(timeout=5.0)

    def command(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd.publish(msg)
        time.sleep(0.15)

    def _publish_count(self):
        status = DiagnosticStatus(name="sphero_rvr_driver")
        status.values.append(
            KeyValue(key="motor_stall_events", value=str(self._count)))
        msg = DiagnosticArray()
        msg.header = Header(stamp=self.node.get_clock().now().to_msg())
        msg.status.append(status)
        self.diag.publish(msg)

    def stall(self):
        """One more firmware stall on the counter, stamped now (exact TF path)."""
        self._count += 1
        self._publish_count()
        time.sleep(0.4)

    def tof_return_at(self, x, y):
        """One raw return at map==base_link (x, y), the corroboration evidence."""
        msg = PointCloud2()
        msg.header = Header(stamp=self.node.get_clock().now().to_msg(),
                            frame_id="base_link")
        msg.height, msg.width = 1, 1
        msg.fields = [
            PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
            for n, o in (("x", 0), ("y", 4), ("z", 8))
        ]
        msg.point_step, msg.row_step = 12, 12
        msg.is_dense = True
        msg.data = struct.pack("<fff", float(x), float(y), 0.05)
        self.tof.publish(msg)
        time.sleep(0.3)


@pytest.fixture
def rig():
    r = Rig()
    yield r
    r.close()


def test_the_flight_replay_plants_zero_marks_then_translation_still_plants(rig):
    """Direction one: the flight's three stalls (vx 0.000; wz 3.550, 3.550,
    5.830), no ToF corroboration -> zero marks, three withheld. The pre-D57 node
    plants on the first delta of each merged batch -- this is the falsifier the
    row demands. Direction two, same rig, same node: a forward-drive stall then
    plants a mark exactly as before -- the boot class must survive the fix."""
    for wz in (3.550, 3.550, 5.830):
        rig.command(0.000, wz)
        rig.stall()
    assert rig.marker.marks_placed == 0, (
        "the flight's floor-grip rotation stalls painted permanent marks again")
    assert rig.marker.rotation_stalls_unmarked == 3
    assert rig.marker.contacts_seen == 3, "the stalls themselves must stay visible"

    rig.command(0.12, 0.0)
    rig.stall()
    assert rig.marker.marks_placed == 1, (
        "a driven contact no longer plants -- the fix over-rotated and killed "
        "the d45bd24 boot class")
    assert rig.marker.report()["mark_placements"][-1]["stall_class"] == "translation"


def test_a_corroborated_rotation_stall_is_admitted(rig):
    """A rotation stall WITH a fresh return inside the would-be disc marks: the
    withholding is evidence policy, not a rotation ban -- a real object blocking
    a pivot that the ToF can see still becomes a permanent fact."""
    mx, my = contact_mark_centre(0.0, 0.0, 0.0, reversing=False,
                                 front_m=FOOTPRINT_FRONT_M,
                                 rear_m=FOOTPRINT_REAR_M)
    rig.tof_return_at(mx, my)
    rig.command(0.000, 3.550)
    rig.stall()
    assert rig.marker.marks_placed == 1, (
        "corroborated rotation paint was refused -- the gate is stricter than "
        "the ratified design")
    assert rig.marker.rotation_stalls_unmarked == 0
    assert rig.marker.report()["mark_placements"][-1]["stall_class"] == "rotation"
