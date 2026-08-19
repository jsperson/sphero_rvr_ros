"""Publishes /contact_marks: places on the map where the robot HIT something.

D48's consumer, and the stock middle's entire touch response. The bespoke decisive
controller had this behaviour welded into it; the stock middle does not run that
controller, so on 2026-08-18 the robot drove into the same chair leg four times, each
time recovering and retrying into an obstacle no live sensor path can see. This node
is what makes the fourth attempt different from the first.

TTL GOVERNS PUBLICATION AND THE REPORT; THE COSTMAP KEEPS THE MARK REGARDLESS --
measured on the bench 2026-08-10, a free cell went cost 0 -> 100 on a mark and stayed
100 after publication stopped. v1 therefore sets the TTL to INFINITY rather than 300 s.
A finite TTL here would not expire anything; it would only make the mission report
claim a mark expired while the costmap still enforces it. Mission-permanent everywhere
or nowhere. The TTL parameter stays in `FreezeMarkSet` because it is the hook a real
revocation mechanism (Jazzy's `clear_around_pose`) will hang from in v2.

POSE AUTHORITY, stated because inferring it is how we lost three days: marks are placed
from TF `map` -> `base_link`, looked up at the stamp of the diagnostics message that
carried the contact. `map`, not `odom`, because these marks outlive odometry drift by
design. Every mark logs its authority and its lookup time. WHEN TF CANNOT ANSWER, the
contact is logged loudly and NO MARK IS PLANTED: a contact we cannot place is a fact
for the report, not a guess for the costmap.

WHAT THIS NODE DOES NOT DO: it does not brake, steer, or cancel anything. It publishes
facts. The costmap consumes them and the planner routes around them on the NEXT plan --
which is the whole mechanism, and also its latency. Stopping is the collision
supervisor's job and it keeps it.
"""

from __future__ import annotations

import math
import struct
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, ExtrapolationException, TransformListener

from sphero_rvr_core.contact_marking import (
    FOOTPRINT_FRONT_M,
    FOOTPRINT_REAR_M,
    ROBOT_RADIUS_M,
    STALL_IDLE,
    STALL_ROTATION,
    PoseDataLagsStamp,
    StallEventTracker,
    classify_stall,
    contact_mark_centre,
    default_margin_m,
    disc_points,
    resolve_contact_pose,
)
from sphero_rvr_core.decisive_control import FreezeMarkSet
from sphero_rvr_core.refusal_promotion import MAX_DISCS_PER_FIRING, RecentReturns

#: The height marks are presented at. These stand for an obstacle the LIDAR cannot see,
#: so they must arrive at a height the costmap's filter would have accepted from the
#: lidar -- present them at their true height (the floor) and the layer discards them.
MARK_Z_M = 0.15


def marks_qos() -> QoSProfile:
    """QoS for /contact_marks, and the honest provenance of the choice.

    RELIABLE + VOLATILE + depth 10 -- byte-identical to what the bespoke controller
    published freeze marks with. THE AUTHORITY IS THE FIELD: D43 is proof of delivery in
    the strongest possible form -- a robot buried by its own marks is a robot whose
    ObstacleLayer received every one of them through exactly this profile.

    (An earlier version of this docstring claimed nav2_costmap_2d ships no headers on
    this Pi to check against. That was false and never verified:
    `/opt/ros/jazzy/include/nav2_costmap_2d/` is fully populated, and
    `inflation_layer.hpp` carries `computeCost` inline. Two sessions reasoned from
    recollection about source that was readable the whole time. CHECK THE INCLUDE PATH
    BEFORE DECLARING A SOURCE UNREADABLE -- "we cannot check" is itself a claim, and it
    is the cheapest of all of them to test.)

    The compatibility reasoning agrees with the evidence (a RELIABLE publisher satisfies
    a BEST_EFFORT subscriber, so this is safe against either request), but the reasoning
    is the corroboration and the field is the authority. Deliberately NOT upgraded to
    TRANSIENT_LOCAL, tempting as late-joiner delivery is: that would be a change with no
    evidence behind it, and the 2 Hz republish already feeds late subscribers.
    """
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )


class ContactMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("contact_marker")

        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tof_points_topic", "/tof/points")
        self.declare_parameter("stall_counter_key", "motor_stall_events")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("mark_radius_m", ROBOT_RADIUS_M)
        self.declare_parameter("mark_ring_points", 12)
        self.declare_parameter("footprint_front_m", FOOTPRINT_FRONT_M)
        self.declare_parameter("footprint_rear_m", FOOTPRINT_REAR_M)
        self.declare_parameter("mark_margin_m", default_margin_m())
        self.declare_parameter("merge_radius_m", 0.15)
        self.declare_parameter("republish_hz", 2.0)
        #: Infinity by default -- see the module docstring. Overridable so v2 can turn
        #: it into a real bound once something can actually revoke a mark.
        self.declare_parameter("mark_ttl_s", float("inf"))

        self._global_frame = str(self.get_parameter("global_frame").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._radius = float(self.get_parameter("mark_radius_m").value)
        self._ring_points = int(self.get_parameter("mark_ring_points").value)
        self._front = float(self.get_parameter("footprint_front_m").value)
        self._rear = float(self.get_parameter("footprint_rear_m").value)
        self._margin = float(self.get_parameter("mark_margin_m").value)
        self._key = str(self.get_parameter("stall_counter_key").value)

        self._tracker = StallEventTracker()
        self._marks = FreezeMarkSet(
            ttl_s=float(self.get_parameter("mark_ttl_s").value),
            merge_radius_m=float(self.get_parameter("merge_radius_m").value),
        )
        #: Last COMMANDED twist. The command, never odometry -- D37's rule. Angular
        #: joined linear for D57: with only linear tracked, the 2026-08-19 flight's
        #: pure-rotation stalls (vx 0.000, wz 3.55) were indistinguishable from a
        #: forward contact and planted two false FRONT marks in the door gap.
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        #: D57's corroboration evidence: recent raw ToF returns in map frame, the
        #: same rig-certified ring the refusal watcher trusts (one authority --
        #: trust returns, not paint). Rotation-stall paint requires a fresh return
        #: inside the would-be mark disc; translation stalls never consult it.
        self._returns = RecentReturns()
        #: Counts for the mission report, including the failures. A node that silently
        #: drops contacts it could not place would make the report agree with itself
        #: and disagree with the world.
        self.contacts_seen = 0
        self.marks_placed = 0
        self.contacts_unplaceable = 0
        self.contacts_collapsed = 0
        #: D57: rotation stalls whose paint was withheld (no fresh ToF return in
        #: the would-be disc) and stalls with no commanded motion at all. Counted
        #: loudly -- the STALL stays fully visible on /diagnostics either way;
        #: only the permanent paint is withheld.
        self.rotation_stalls_unmarked = 0
        self.stalls_idle = 0
        #: One record per PLANTED mark, carrying placement provenance (path
        #: exact/fallback + signed staleness). In the report so an autopsy never has
        #: to reconstruct "was this mark exactly-placed?" from timestamps.
        self._mark_placements: list[dict] = []

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._pub = self.create_publisher(PointCloud2, "/contact_marks", marks_qos())
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self._on_cmd, 10
        )
        # D57 corroboration feed. QoS DELIBERATE, not defaulted-by-omission: the
        # tof node publishes ~/points with the default profile (RELIABLE +
        # VOLATILE, depth 5); a plain depth-10 subscription here is the SAME
        # profile the refusal watcher already receives this stream on. The
        # touch-port register carries one open QoS-silence row -- this comment
        # and tests/test_contact_marking's wiring pin exist so this seam never
        # grows a sibling.
        self.create_subscription(
            PointCloud2, str(self.get_parameter("tof_points_topic").value),
            self._on_tof, 10,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            self._on_diagnostics,
            10,
        )
        # Option D's request lane (decision 2026-08-18): refusal_watcher asks, this
        # node plants -- single authorship of /contact_marks is preserved, and the
        # validation here is FRAME and CAP only. Deliberately NOT a delta re-check:
        # the watcher's snapshot is the evidence, and two components re-deriving one
        # fact is the seam class this project keeps paying for.
        self.promotions_accepted = 0
        self.promotions_rejected = 0
        self.create_subscription(
            PointCloud2, "/contact_marks/promote", self._on_promote, 10
        )
        self.create_timer(
            1.0 / float(self.get_parameter("republish_hz").value), self._publish
        )
        self.get_logger().info(
            f"contact_marker up. Marks are MISSION-PERMANENT (ttl="
            f"{self.get_parameter('mark_ttl_s').value}); pose authority "
            f"{self._global_frame}->{self._robot_frame}; disc r={self._radius:.3f} m "
            f"at footprint edge + {self._margin:.3f} m."
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd_linear = float(msg.linear.x)
        self._last_cmd_angular = float(msg.angular.z)

    def _on_tof(self, msg: PointCloud2) -> None:
        """Raw returns into the corroboration ring, transformed to map -- the
        watcher's own recipe. A return that cannot be placed (no TF) vouches for
        nothing and is dropped."""
        try:
            transform = self._buffer.lookup_transform(
                self._global_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        now = time.monotonic()
        n = msg.width * msg.height
        for i in range(n):
            px, py, _ = struct.unpack_from("<fff", msg.data, i * msg.point_step)
            self._returns.add(now, t.x + c * px - s * py, t.y + s * px + c * py)
        self._returns.prune(now)

    @staticmethod
    def _pose_of(transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return t.x, t.y, yaw

    def _robot_pose(self, stamp):
        """The pose for a contact: exact stamp preferred, bounded-latest fallback.

        v1 stopped at the exact-stamp lookup and lost 3 of 3 field contacts on
        2026-08-18 -- SLAM's map->odom ran 69-87 ms behind the diagnostics stamp, tf2
        refused to extrapolate, and no mark was ever planted from a moving robot. The
        policy (and the measured bound) lives in
        `sphero_rvr_core.contact_marking.resolve_contact_pose`; this method only
        adapts tf2 to it. ONLY ExtrapolationException is translated into the fallback
        signal -- a missing frame or a disconnected tree still refuses outright,
        because those poses are not late, they are untrustworthy.
        """
        stamp_s = stamp.sec + stamp.nanosec * 1e-9

        def exact():
            try:
                transform = self._buffer.lookup_transform(
                    self._global_frame, self._robot_frame,
                    rclpy.time.Time.from_msg(stamp),
                )
            except ExtrapolationException as exc:
                raise PoseDataLagsStamp(str(exc)) from exc
            return self._pose_of(transform)

        def latest():
            transform = self._buffer.lookup_transform(
                self._global_frame, self._robot_frame, rclpy.time.Time()
            )
            h = transform.header.stamp
            return self._pose_of(transform), h.sec + h.nanosec * 1e-9

        return resolve_contact_pose(exact, latest, stamp_s)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        count = None
        for status in msg.status:
            for kv in status.values:
                if kv.key == self._key:
                    count = int(kv.value)
        if count is None:
            return
        batch = self._tracker.observe(count)
        if batch is None:
            return

        self.contacts_seen += batch.contacts
        if batch.collapsed:
            self.contacts_collapsed += batch.contacts - 1
            self.get_logger().warning(
                f"{batch.contacts} contacts arrived in ONE diagnostics message and "
                f"collapse to a single mark -- only one pose is available for them."
            )

        try:
            resolved = self._robot_pose(msg.header.stamp)
        except Exception as exc:
            self.contacts_unplaceable += 1
            self.get_logger().error(
                f"CONTACT AT AN UNPLACEABLE POSE -- {batch.contacts} contact(s) "
                f"detected, NO MARK PLANTED. TF {self._global_frame}->"
                f"{self._robot_frame} at stamp {msg.header.stamp.sec}."
                f"{msg.header.stamp.nanosec:09d} failed: {exc}. The contact is real "
                f"and is in the report; its location is not, so nothing goes on the "
                f"costmap. A permanent lethal disc at an invented pose is worse than "
                f"no mark."
            )
            return

        vx, wz = self._last_cmd_linear, self._last_cmd_angular
        stall_class = classify_stall(vx, wz)
        reversing = vx < 0.0
        mx, my = contact_mark_centre(
            resolved.x, resolved.y, resolved.yaw,
            reversing=reversing,
            front_m=self._front,
            rear_m=self._rear,
            margin_m=self._margin,
        )
        if stall_class == STALL_IDLE:
            self.stalls_idle += 1
            self.get_logger().error(
                f"STALL WITH NO COMMANDED MOTION (vx {vx:+.3f} m/s, wz {wz:+.3f} "
                f"rad/s) — a phantom by definition; NO MARK. If this repeats, the "
                f"command pairing is broken, not the room."
            )
            return
        if stall_class == STALL_ROTATION and not self._returns.fresh_near(
                time.monotonic(), mx, my, radius_m=self._radius):
            # D57: the 2026-08-19 flight planted two false marks in the door gap
            # from exactly this signature (pure rotation into floor grip, nothing
            # there -- Scott's eyewitness ground truth). Rotation paint must earn
            # corroboration; the stall itself stays fully visible on /diagnostics
            # and in this node's report.
            self.rotation_stalls_unmarked += 1
            self.get_logger().warning(
                f"ROTATION STALL, PAINT WITHHELD (vx {vx:+.3f} m/s, wz {wz:+.3f} "
                f"rad/s): no fresh ToF return inside the would-be disc at "
                f"({mx:.3f}, {my:.3f}). Floor grip wears contact's clothes; a "
                f"permanent lethal disc needs a sensor to vouch for it."
            )
            return
        self._marks.add(mx, my, time.monotonic())
        self.marks_placed += 1
        self._mark_placements.append({
            "x": round(mx, 4),
            "y": round(my, 4),
            "stamp": f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
            "path": resolved.path,
            "staleness_ms": round(resolved.staleness_s * 1000.0, 1),
            "stall_class": stall_class,
        })
        self.get_logger().warning(
            f"CONTACT MARKED at ({mx:.3f}, {my:.3f}) in {self._global_frame}. "
            f"Pose authority: TF {self._global_frame}->{self._robot_frame} at "
            f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
            f"path={resolved.path} staleness={resolved.staleness_s * 1000.0:+.0f} ms "
            f"-> ({resolved.x:.3f}, {resolved.y:.3f}, yaw "
            f"{math.degrees(resolved.yaw):.1f} deg). "
            f"Commanded (vx {vx:+.3f}, wz {wz:+.3f}) -> {stall_class} stall, "
            f"{'REAR' if reversing else 'FRONT'} edge"
            f"{'; ToF-corroborated' if stall_class == STALL_ROTATION else ''}. "
            f"This mark is permanent."
        )
        self._publish()

    def _on_promote(self, msg: PointCloud2) -> None:
        """Plant refusal-promotion marks. GOAL-IN-DELTA is accepted behaviour: if
        the goal itself sits on the promoted obstacle, the goal becomes unplannable
        and aborts honestly -- which is correct (better an honest abort than the
        livelock), and the goal tool's trinity gate makes it rare. An abort right
        after a promotion is not a defect. These marks are MISSION-PERMANENT until
        revocation, exactly like every other mark this node plants."""
        if msg.header.frame_id != self._global_frame:
            self.promotions_rejected += 1
            self.get_logger().error(
                f"promotion REJECTED: frame {msg.header.frame_id!r} is not "
                f"{self._global_frame!r} -- a mark in the wrong frame is an "
                f"invented pose"
            )
            return
        n = msg.width * msg.height
        if n == 0 or n > MAX_DISCS_PER_FIRING:
            self.promotions_rejected += 1
            self.get_logger().error(
                f"promotion REJECTED: {n} centroid(s) against the "
                f"{MAX_DISCS_PER_FIRING}-disc firing cap -- a request this size is "
                f"a wall or a defect, not an obstacle"
            )
            return
        for i in range(n):
            px, py, _ = struct.unpack_from("<fff", msg.data, i * msg.point_step)
            self._marks.add(px, py, time.monotonic())
            self.marks_placed += 1
            self.promotions_accepted += 1
            self._mark_placements.append({
                "x": round(px, 4),
                "y": round(py, 4),
                "stamp": f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
                "path": "refusal_promotion",
                "staleness_ms": None,
            })
            self.get_logger().warning(
                f"REFUSAL PROMOTION MARKED at ({px:.3f}, {py:.3f}) in "
                f"{self._global_frame}. Authority: refusal_watcher's livelock "
                f"snapshot (lethal-local/free-global on the refused corridor). "
                f"This mark is permanent."
            )
        self._publish()

    def _publish(self) -> None:
        live = self._marks.live(time.monotonic())
        points: list[tuple[float, float]] = []
        for mark in live:
            points.extend(disc_points(mark.x, mark.y, self._radius, self._ring_points))

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._global_frame
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(("x", "y", "z"))
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = b"".join(
            struct.pack("<fff", px, py, MARK_Z_M) for px, py in points
        )
        self._pub.publish(msg)

    def report(self) -> dict:
        """For the mission report. `contacts_seen` and `marks_placed` are reported
        SEPARATELY and are meant to differ -- collapsed batches and unplaceable poses
        both make marks fewer than contacts, and a report that showed only one number
        would hide exactly the cases worth knowing about."""
        return {
            "contacts_seen": self.contacts_seen,
            "marks_placed": self.marks_placed,
            "contacts_unplaceable": self.contacts_unplaceable,
            "contacts_collapsed": self.contacts_collapsed,
            "rotation_stalls_unmarked": self.rotation_stalls_unmarked,
            "stalls_idle": self.stalls_idle,
            "marks": self._marks.as_report_list(time.monotonic()),
            "mark_placements": list(self._mark_placements),
            "promotions_accepted": self.promotions_accepted,
            "promotions_rejected": self.promotions_rejected,
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ContactMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
