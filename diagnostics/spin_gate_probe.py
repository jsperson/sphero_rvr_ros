"""Does nav2's OWN collision gate refuse a Spin, and against what? No hardware.

D16: `behavior_server` runs an independent collision check (`simulate_ahead_time`,
lean_nav2.yaml) and has historically refused Spin when wedged ("Collision Ahead -
Exiting Spin", 2026-08-06). The supervisor-side pivot gates are measured closed
(D17-D19); this second gate was never measured at all. The sharp question: the gate
simulates the swept footprint against the LOCAL costmap, and decisive mode REMOVES
the local costmap -- so in the deployed configuration the gate may be reading
nothing, blocking on absence, or silently passing everything.

Three cases, each a separate observation:
  A  no local costmap published at all      <- the deployed decisive-mode reality
  B  synthetic local costmap, all free      <- gate reads it, should grant
  C  synthetic local costmap, obstacle ring
     inside the footprint's swept circle    <- gate reads it, should refuse

Run with the driver DOWN and the supervisor DOWN (nothing consumes /cmd_vel, so a
granted Spin moves nothing). Bring up TF + behavior_server first — NOTE: no static
odom->base_link; the PROBE broadcasts that one, rotating:

    # NON-IDENTITY map->odom (0.5 m): with an identity transform map and odom
    # coincide and a frame mismatch is invisible by construction — which is exactly
    # how this instrument missed the bug 4f3d1f2 introduced.
    ros2 run tf2_ros static_transform_publisher --x 0.5 --y 0 --z 0 \
        --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom &
    ros2 run nav2_behaviors behavior_server --ros-args \
        --params-file <lean_nav2.yaml>
    ros2 lifecycle set /behavior_server configure
    ros2 lifecycle set /behavior_server activate
    python3 spin_gate_probe.py [case] [topic_base]   # A, B, C or all (default
                                           # all); topic_base defaults to
                                           # local_costmap (D16 fix: global_costmap)

THE ROBOT MUST APPEAR TO ROTATE, or nothing is measured: Spin::isCollisionFree
(jazzy, spin.cpp:179) breaks out of its simulate-ahead loop while
`abs(relative_yaw) - abs(sim_position_change) <= 0`, and relative_yaw is the yaw
traveled SO FAR — zero at spin start. Against a static TF the collision check
never executes at all, every case degenerates to a time-allowance abort, and a
first version of this probe scored exactly that as the gate PASSING. So the probe
integrates the behaviour's own /cmd_vel into a live odom->base_link broadcast (a
fake robot that obeys), which is what lets the check run and the cases differ.

The probe publishes (for B/C) `local_costmap/costmap_raw` + `published_footprint`
(TRANSIENT_LOCAL — the CostmapSubscriber silently refuses a volatile publisher),
sends a Spin goal, and reports result, the server's own /rosout reason, and
whether rotation flowed. Case C carries the one hard verdict (a gate that grants a
spin through a visible obstacle ring is not gating); A and B are the
characterization numbers the D16 register row needs.

CASE A IS ONLY VALID ON A FRESH behavior_server: the latched costmap/footprint
from an earlier B/C run persists INSIDE the server's subscriber, so a repeat run's
case A silently measures stale data instead of absence (observed: a case-C ring
from the previous invocation turned run 2's case A into a "collision" refusal).
Restart the server between full runs when case A matters.

Measured 2026-08-10 (jazzy, deployed lean_nav2.yaml):
  A  ABORTED ~1.0 s in: "Current footprint not available." then "Collision Ahead"
     -- the checker treats MISSING DATA as collision, so in decisive mode (no
     local costmap exists) EVERY Spin recovery is refused shortly after starting,
     obstacle or not. The 2026-08-06 wedged-rover refusal likely was this.
  B  free costmap fed: full 1.57 rad spin SUCCEEDED (4.7 s at 0.4 rad/s).
  C  lethal ring fed: refused in 0.8 s, genuine collision reason. PASS.
"""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PolygonStamped, Twist
from nav2_msgs.action import Spin
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster

SIZE = 100          # cells
RES = 0.05          # m/cell -> 5 x 5 m local window centred on the robot
FREE = 0
LETHAL = 254        # nav2_msgs/Costmap uses the 0..255 cost scale, not 0..100


# The robot's TRUE position in the map frame. The bench publishes a NON-IDENTITY
# map->odom of exactly this offset, so map and odom no longer coincide and the frame
# path is actually exercised. With an identity transform (the original probe) a
# frame mismatch is invisible by construction -- which is why this instrument missed
# the bug that 4f3d1f2 introduced.
MAP_OFFSET_X = 0.5
MAP_OFFSET_Y = 0.0


def costmap_msg(stamp, obstacle_ring_m=None, free_island_m=None):
    """Costmap in the MAP frame, matching the real global costmap.

    Two shapes, both placed around the robot's TRUE map position:
      * obstacle_ring_m: a lethal annulus AT the robot -> a correct checker refuses.
      * free_island_m:   free only within this radius of the robot, lethal beyond
                         -> a correct checker grants; a checker reading the wrong
                         cell (off by the map->odom offset) lands outside the island
                         and refuses. That is what makes case B invert too.
    """
    m = Costmap()
    m.header.stamp = stamp
    m.header.frame_id = "map"
    m.metadata.resolution = RES
    m.metadata.size_x = SIZE
    m.metadata.size_y = SIZE
    # Centre the window on the robot's true map position.
    m.metadata.origin.position.x = MAP_OFFSET_X - SIZE * RES / 2.0
    m.metadata.origin.position.y = MAP_OFFSET_Y - SIZE * RES / 2.0
    data = bytearray(SIZE * SIZE)
    for cy in range(SIZE):
        for cx in range(SIZE):
            x = (cx + 0.5) * RES + m.metadata.origin.position.x
            y = (cy + 0.5) * RES + m.metadata.origin.position.y
            r = math.hypot(x - MAP_OFFSET_X, y - MAP_OFFSET_Y)
            if obstacle_ring_m is not None:
                lo, hi = obstacle_ring_m
                if lo <= r <= hi:
                    data[cy * SIZE + cx] = LETHAL
            elif free_island_m is not None and r > free_island_m:
                data[cy * SIZE + cx] = LETHAL
    m.data = bytes(data)
    return m


def footprint_msg(stamp, yaw):
    """The real footprint rectangle (front 0.11 / rear 0.16 / half-width 0.10),
    oriented at the fake robot's current yaw — published_footprint is the
    world-frame oriented footprint, and the checker un-rotates it using the pose,
    so an axis-aligned polygon under a rotated robot is a lie."""
    p = PolygonStamped()
    p.header.stamp = stamp
    p.header.frame_id = "map"
    c, s = math.cos(yaw), math.sin(yaw)
    for x, y in ((0.11, 0.10), (0.11, -0.10), (-0.16, -0.10), (-0.16, 0.10)):
        pt = Point32()
        pt.x = float(x * c - y * s) + MAP_OFFSET_X
        pt.y = float(x * s + y * c) + MAP_OFFSET_Y
        p.polygon.points.append(pt)
    return p


class Probe(Node):
    def __init__(self, topic_base="local_costmap"):
        super().__init__("spin_gate_probe")
        self.spin = ActionClient(self, Spin, "spin")
        # Costmap2D publishes costmap_raw latched; nav2's CostmapSubscriber
        # subscribes TRANSIENT_LOCAL and silently refuses a volatile publisher
        # ("offering incompatible QoS ... DURABILITY"), so a plain publisher here
        # feeds the gate NOTHING and every case degenerates to case A. Measured
        # before it was understood.
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        self.cm_pub = self.create_publisher(
            Costmap, f"{topic_base}/costmap_raw", latched)
        self.fp_pub = self.create_publisher(
            PolygonStamped, f"{topic_base}/published_footprint", latched)
        self.rotated = False
        self.wz = 0.0
        self.yaw = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        # The fake robot: integrate the behaviour's own commands into a live
        # odom->base_link broadcast, so relative_yaw grows and the collision
        # check actually executes (see the module docstring).
        self._tf = TransformBroadcaster(self)
        self._last_tick = time.monotonic()
        self.create_timer(0.02, self._animate)
        # The result status alone cannot say WHY a Spin ended (a collision refusal
        # and a time-allowance expiry are both ABORTED). The server says why on
        # /rosout; capture its lines per case.
        self.server_log = []
        self.create_subscription(Log, "/rosout", self._on_rosout, 50)

    def _on_cmd(self, m):
        self.wz = m.angular.z
        if abs(m.angular.z) > 1e-6:
            self.rotated = True

    def _animate(self):
        now = time.monotonic()
        self.yaw += self.wz * (now - self._last_tick)
        self._last_tick = now
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self._tf.sendTransform(t)

    def _on_rosout(self, m):
        if m.name == "behavior_server":
            self.server_log.append(m.msg)

    def _drain(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_case(self, label, publish_costmap, obstacle_ring, free_island=None):
        # rosout delivery lags the action result by a beat: without draining, each
        # case's terminal reason lands in the NEXT case's capture and every verdict
        # is attributed one case late (measured). Drain stragglers, then reset.
        self._drain(0.7)
        self.rotated = False
        self.server_log = []
        self.wz = 0.0
        self.yaw = 0.0
        if not self.spin.wait_for_server(timeout_sec=3.0):
            print(f"{label}: spin action server NOT AVAILABLE — is behavior_server "
                  "up and activated?")
            return None
        # Feed (or don't feed) the gate for a moment before asking.
        end_warm = time.monotonic() + 1.0
        while time.monotonic() < end_warm:
            if publish_costmap:
                now = self.get_clock().now().to_msg()
                self.cm_pub.publish(costmap_msg(now, obstacle_ring, free_island))
                self.fp_pub.publish(footprint_msg(now, self.yaw))
            rclpy.spin_once(self, timeout_sec=0.05)

        goal = Spin.Goal()
        goal.target_yaw = 1.57
        goal.time_allowance = Duration(seconds=8.0).to_msg()
        t0 = time.monotonic()
        send = self.spin.send_goal_async(goal)
        while not send.done() and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = send.result() if send.done() else None
        if handle is None or not handle.accepted:
            print(f"{label}: goal NOT ACCEPTED")
            return "rejected"
        result_future = handle.get_result_async()
        # Keep feeding the costmap while the behaviour runs; sample /cmd_vel.
        while not result_future.done() and time.monotonic() - t0 < 12.0:
            if publish_costmap:
                now = self.get_clock().now().to_msg()
                self.cm_pub.publish(costmap_msg(now, obstacle_ring, free_island))
                self.fp_pub.publish(footprint_msg(now, self.yaw))
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done():
            handle.cancel_goal_async()
            print(f"{label}: behaviour still running at 12 s — cancelled "
                  f"(rotation_cmds={'yes' if self.rotated else 'no'})")
            return "timeout"
        self._drain(0.7)   # let the terminal reason arrive before classifying
        status = result_future.result().status
        name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"status={status}")
        took = time.monotonic() - t0
        if any("Collision Ahead" in s for s in self.server_log):
            # "Collision Ahead" also covers checker DATA failures: a missing
            # footprint/costmap is caught and treated as a collision. Distinguish,
            # because "refused for an obstacle" and "refused because the gate is
            # blind" are opposite findings.
            why = ("no_data" if any("not available" in s for s in self.server_log)
                   else "collision")
        elif any("time allowance" in s for s in self.server_log):
            why = "time_allowance"
        else:
            why = "unstated"
        print(f"{label}: accepted, result={name} ({why}) after {took:.1f}s, "
              f"rotation reached /cmd_vel: {'yes' if self.rotated else 'no'}")
        for s in self.server_log:
            print(f"    server: {s}")
        return f"{name}/{why}"


def main():
    which = (sys.argv[1].upper() if len(sys.argv) > 1 else "ALL")
    # Optional second arg: topic base for the synthetic costmap/footprint
    # publishers (default local_costmap). E.g. `spin_gate_probe.py all
    # global_costmap` to feed a behavior_server whose collision source was
    # repointed at the global costmap (D16 fix).
    topic_base = sys.argv[2] if len(sys.argv) > 2 else "local_costmap"
    rclpy.init()
    n = Probe(topic_base)
    time.sleep(1.0)
    results = {}
    if which in ("A", "ALL"):
        results["A"] = n.run_case(
            "A (no local costmap at all — deployed decisive reality)", False, None)
    if which in ("B", "ALL"):
        # Free only NEAR the robot's true map position. A checker reading the wrong
        # cell (off by the map->odom offset) lands outside the island and refuses,
        # so this case inverts on a frame mismatch instead of passing regardless.
        results["B"] = n.run_case(
            # 0.30 m: comfortably covers the footprint at the TRUE position, but
            # SMALLER than the map->odom offset, so a checker reading the wrong cell
            # lands outside the island and refuses. Sized this way on purpose --
            # at 0.8 m the offset still landed inside and the case passed under both
            # frames, i.e. it discriminated nothing.
            "B (free island 0.30 m at TRUE pos, lethal beyond)", True, None,
            free_island=0.30)
    if which in ("C", "ALL"):
        results["C"] = n.run_case(
            "C (lethal ring 0.15-0.25 m at TRUE pos)", True, (0.15, 0.25))
    print()
    if "B" in results:
        okB = results["B"] == "SUCCEEDED/unstated"
        print(("PASS" if okB else "**FAIL**") +
              "  case B: gate GRANTS a spin on free ground at the robot's true map "
              f"position (got {results['B']})")
    if "C" in results:
        # The one hard verdict: a gate that grants a spin THROUGH a visible
        # obstacle ring is not gating. Only a COLLISION-reasoned refusal counts —
        # ABORTED on time allowance is the gate not firing at all (learned the
        # hard way: the first run scored exactly that as a PASS).
        # F6. ONLY a collision-reasoned refusal counts. "rejected" means the server
        # never even ran the behaviour -- it proves nothing about the gate, and
        # scoring it as a pass would let a dead behaviour_server look like a working
        # safety check.
        ok = results["C"] == "ABORTED/collision"
        print(("PASS" if ok else "**FAIL**") +
              "  case C: gate refuses (collision) a spin with an obstacle it can"
              f" see (got {results['C']})")
    n.destroy_node()
    rclpy.shutdown()


main()
