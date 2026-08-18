"""Watches for the livelock and requests promotion. ZERO motion authority.

The D design's node half (decision 2026-08-18, rig-first; the brain is
`sphero_rvr_core.refusal_promotion` and every policy comment lives there). This
node only adapts published facts to it:

  /navigate_to_pose/_action/feedback   pose, recoveries, goal id -- the BT's own
                                       words about the goal it is running
  /plan                                the refused corridor's spine
  /{local,global}_costmap/costmap_raw + _updates
                                       tracked full+delta, because reading the
                                       latched frame alone is the trap
                                       `costmap-raw-is-latched-once` names

Its ONE output is a request: /contact_marks/promote (PointCloud2 of cluster
centroids, map frame). contact_marker remains the sole author of /contact_marks
and validates frame/cap/merge at plant time -- but does NOT re-derive the delta:
the watcher's snapshot is the evidence, and two components re-deriving one fact
is the seam class this project keeps paying for.

Suppressed firings are LOGGED with their reason. Silence is not discipline.

EVIDENCE FRESHNESS: every candidate centroid must be backed by a raw /tof/points
return inside the last EVIDENCE_FRESHNESS_S, or it is refused as stale -- the rig's
transient-short arm proved that an obstacle which leaves in silence strands its
paint in the local costmap (tof_layer clears only by successor-return raytrace; see
the tof-layer persistence item in the backlog and
memory/touch-port-v1-field-defects: the run-3d dead-band is the same disease seen
from the approach side), and an ungated watcher would promote that stale paint into
a permanent mark. Trust returns, not paint.
"""

from __future__ import annotations

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Path
from nav2_msgs.msg import Costmap, CostmapUpdate
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

from sphero_rvr_core.refusal_promotion import (
    Grid,
    FiringDiscipline,
    LivelockWindow,
    RecentReturns,
    blindness_delta_cells,
    cluster_and_cap,
    freshness_verdict,
)

RAW_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
)


class CostmapTracker:
    """Full frame on costmap_raw, deltas from costmap_raw_updates."""

    def __init__(self):
        self.grid: Grid | None = None

    def on_full(self, msg: Costmap) -> None:
        md = msg.metadata
        self.grid = Grid(
            data=list(msg.data),
            origin_x=md.origin.position.x, origin_y=md.origin.position.y,
            resolution=md.resolution, width=md.size_x, height=md.size_y,
        )

    def on_update(self, msg: CostmapUpdate) -> None:
        if self.grid is None:
            return
        self.grid.apply_update(msg.x, msg.y, msg.size_x, msg.size_y, list(msg.data))


class RefusalWatcher(Node):
    def __init__(self) -> None:
        super().__init__("refusal_watcher")
        self.declare_parameter("window_s", 12.0)
        self.declare_parameter("stall_displacement_m", 0.15)
        self.declare_parameter("min_recoveries", 2)
        self.window = LivelockWindow(
            window_s=float(self.get_parameter("window_s").value),
            stall_m=float(self.get_parameter("stall_displacement_m").value),
            min_recoveries=int(self.get_parameter("min_recoveries").value),
        )
        self.discipline = FiringDiscipline()
        self.local = CostmapTracker()
        self.global_ = CostmapTracker()
        self.plan_xy: list = []
        self.returns = RecentReturns()
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._last_suppression = ""

        self.create_subscription(
            Costmap, "/local_costmap/costmap_raw", self.local.on_full, RAW_QOS)
        self.create_subscription(
            CostmapUpdate, "/local_costmap/costmap_raw_updates",
            self.local.on_update, RAW_QOS)
        self.create_subscription(
            Costmap, "/global_costmap/costmap_raw", self.global_.on_full, RAW_QOS)
        self.create_subscription(
            CostmapUpdate, "/global_costmap/costmap_raw_updates",
            self.global_.on_update, RAW_QOS)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(PointCloud2, "/tof/points", self._on_tof, 10)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback", self._on_feedback, 10)
        self._pub = self.create_publisher(PointCloud2, "/contact_marks/promote", 10)
        self.get_logger().info(
            f"refusal_watcher up: window {self.window.window_s:.0f}s, stall bar "
            f"{self.window.stall_m} m, recoveries >= {self.window.min_recoveries}. "
            f"Output is a REQUEST; contact_marker owns the marks."
        )

    def _on_plan(self, msg: Path) -> None:
        self.plan_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.discipline.note_plan()

    def _on_tof(self, msg: PointCloud2) -> None:
        """Raw returns into the freshness ring, transformed to map. A return that
        cannot be placed (no TF) vouches for nothing and is dropped."""
        try:
            tf = self._buffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        now = self.get_clock().now().nanoseconds * 1e-9
        n = msg.width * msg.height
        for i in range(n):
            px, py, _ = struct.unpack_from("<fff", msg.data, i * msg.point_step)
            self.returns.add(now, t.x + c * px - s * py, t.y + s * px + c * py)
        self.returns.prune(now)

    def _on_feedback(self, msg) -> None:
        fb = msg.feedback
        goal_id = bytes(msg.goal_id.uuid).hex()
        x = fb.current_pose.pose.position.x
        y = fb.current_pose.pose.position.y
        t = self.get_clock().now().nanoseconds * 1e-9
        if not self.window.feed(t, x, y, int(fb.number_of_recoveries), goal_id):
            return
        if self.local.grid is None or self.global_.grid is None or not self.plan_xy:
            self._suppress("signature holds but a costmap or plan is missing")
            return
        cells = blindness_delta_cells(
            self.local.grid, self.global_.grid, self.plan_xy, (x, y))
        firing = cluster_and_cap(cells)
        if firing.suppressed_reason:
            self._suppress(firing.suppressed_reason)
            return
        fresh, stale = freshness_verdict(self.returns, t, firing.centroids)
        if stale:
            self.get_logger().info(
                f"freshness gate dropped {len(stale)} stale centroid(s) "
                f"{[(round(a, 2), round(b, 2)) for a, b in stale]} -- paint without "
                f"recent returns is a departed obstacle (or a cone swept away; "
                f"refusal self-corrects on the next sweep)")
        if not fresh:
            self._suppress("evidence stale: no recent ToF return backs any centroid")
            return
        firing.centroids[:] = fresh
        allowed, reason = self.discipline.allow(t, goal_id, firing.centroids)
        if not allowed:
            self._suppress(reason)
            return
        self._publish(firing.centroids)
        self.discipline.record(t, goal_id, firing.centroids)
        self.get_logger().warning(
            f"PROMOTION REQUESTED ({reason}): {len(firing.centroids)} centroid(s) "
            f"{[(round(cx, 3), round(cy, 3)) for cx, cy in firing.centroids]} -- "
            f"livelock signature sustained, delta cells lethal-local/free-global "
            f"on the refused corridor. The marks these become are "
            f"mission-permanent until revocation."
        )

    def _suppress(self, reason: str) -> None:
        if reason != self._last_suppression:
            self._last_suppression = reason
            self.get_logger().info(f"promotion suppressed: {reason}")

    def _publish(self, centroids: list) -> None:
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height, msg.width = 1, len(centroids)
        msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate("xyz")
        ]
        msg.is_bigendian, msg.point_step = False, 12
        msg.row_step = 12 * len(centroids)
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", cx, cy, 0.0) for cx, cy in centroids)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RefusalWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
