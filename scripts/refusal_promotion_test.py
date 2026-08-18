#!/usr/bin/env python3
"""The D rig: reproduce goal 4's livelock, then prove promotion flips it.

Run against a live sim_closed_loop stack (open rig map):

    python3 scripts/make_open_rig_map.py --out-dir /tmp
    ros2 launch sphero_rvr_driver sim_closed_loop.launch.py \
        map_yaml:=/tmp/open_rig_room.yaml
    python3 scripts/refusal_promotion_test.py --arm falsifier

ARMS, in the build's ratified order (falsifier BEFORE any watcher code exists):

  falsifier       Virtual sub-lidar obstacle at --obstacle-x, goal past it. The
                  watcher must be ABSENT (asserted). PRE-REGISTERED BAR: the
                  livelock reproduces -- goal NOT reached, terminal displacement
                  stalled, recoveries accumulating (constants below). A rig that
                  cannot produce goal 4's disease cannot certify the cure.
  certifier       Same scene, watcher PRESENT. BAR: marks appear with ZERO stall
                  events (in this rig any mark is therefore a promotion), the goal
                  SUCCEEDS, the track keeps clear of the obstacle after promotion,
                  and at most one firing's worth of marks is planted.
  transient-short Obstacle appears at approach, holds --hold-s (default 6 < T),
                  vanishes. BAR: ZERO marks ever; goal SUCCEEDS. The 5b cost made
                  falsifiable: a person's moment must not become permanent.
  transient-long  Obstacle holds until promotion or timeout. BAR: promotion fires.
                  The obstacle is then removed and the mark's permanence is LOGGED
                  as the named cost it is, not discovered later.

THE VIRTUAL OBSTACLE lives in this runner (the closed_loop_mark_test pattern: the
rig input is owned by the thing that measures it). It publishes /tof/points -- the
tof_layer's actual source -- as a base_link-frame cluster whenever the obstacle is
inside the sensor's measured envelope (0.17-0.60 m) and forward cone, at the
sensor's real rate class. The supervisor's /tof/obstacles brake feed is deliberately
NOT simulated: in the field (run 3c goal 4) RPP was the refusing party and the
brake stayed CLEAR; this rig reproduces that shape.
"""

import argparse
import math
import struct
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2, PointField
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

#: The sensor's own numbers: costmap envelope 0.17-0.60 m (lean_nav2_stock
#: tof_layer), ~45 deg total FOV (8x8 zones at ~7.5 deg/column), 6.5-7.6 Hz band.
TOF_MIN_M, TOF_MAX_M = 0.17, 0.60
TOF_HALF_CONE_RAD = math.radians(22.0)
TOF_RATE_HZ = 7.0
POINT_Z = 0.10                      # inside the layer's 0.02-0.20 height gate

#: PRE-REGISTERED BARS -- written before the watcher exists, mirrored from the
#: field: goal 4 livelocked with ~0.2 m of total shuffling and 10 recoveries in
#: 19 s; goal 2 (healthy) moved 1.46 m in 22 s. "Stalled" is the last
#: STALL_WINDOW_S of the run moving less than STALL_DISPLACEMENT_M.
STALL_WINDOW_S = 20.0
STALL_DISPLACEMENT_M = 0.15
MIN_RECOVERIES = 4
#: Post-promotion clearance: the track's closest approach to the obstacle centre
#: must exceed the costmap inscribed radius (0.1519) -- the promoted mark's whole
#: point is that the centre never enters the region the mark forbids.
CLEARANCE_M = 0.1519
#: One firing plants at most 3 merged mark discs (the consensus cap). More marks
#: than that on the wire means the discipline failed.
MAX_PROMOTED_POINTS = 3 * 25        # 3 discs x (1 centre + 2 rings x 12)

TL = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def say(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class Rig(Node):
    def __init__(self, args):
        super().__init__("refusal_promotion_test")
        self.args = args
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.track = []                       # (t, x, y) map-frame samples
        self.marks_width = 0
        self.marks_seen_nonzero = False
        self.stall_events = None
        self.obstacle_on = False
        self.obstacle_seen_at = None
        self.pub = self.create_publisher(PointCloud2, "/tof/points", 10)
        # marker publishes VOLATILE; subscribe compatibly (the QoS lesson, 3d).
        self.create_subscription(
            PointCloud2, "/contact_marks", self._on_marks,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.VOLATILE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=10))
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.create_timer(1.0 / TOF_RATE_HZ, self._tof_tick)
        self.create_timer(0.2, self._sample)

    # -- virtual obstacle ---------------------------------------------------------

    def robot_pose(self):
        try:
            tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _tof_tick(self):
        if not self.obstacle_on:
            return
        pose = self.robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        dx, dy = self.args.obstacle_x - rx, self.args.obstacle_y - ry
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - ryaw
        while bearing > math.pi:
            bearing -= 2 * math.pi
        while bearing < -math.pi:
            bearing += 2 * math.pi
        if not (TOF_MIN_M <= dist <= TOF_MAX_M and abs(bearing) <= TOF_HALF_CONE_RAD):
            return
        if self.obstacle_seen_at is None:
            self.obstacle_seen_at = time.monotonic()
            say(f"virtual obstacle ENTERED the sensor envelope at {dist:.2f} m")
        # a 5 cm-wide cluster in base_link, like a real return blob
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        bx, by = c * dx - s * dy, s * dx + c * dy
        pts = [(bx + ox, by + oy, POINT_Z)
               for ox in (-0.025, 0.0, 0.025) for oy in (-0.025, 0.0, 0.025)]
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=4 * i,
                                 datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        msg.is_bigendian, msg.point_step, msg.row_step = False, 12, 12 * len(pts)
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", *p) for p in pts)
        self.pub.publish(msg)

    # -- instruments ----------------------------------------------------------------

    def _on_marks(self, msg):
        self.marks_width = msg.width
        if msg.width > 0:
            self.marks_seen_nonzero = True

    def _on_diag(self, msg):
        for st in msg.status:
            for kv in st.values:
                if kv.key == "motor_stall_events":
                    self.stall_events = int(kv.value)

    def _sample(self):
        pose = self.robot_pose()
        if pose:
            self.track.append((time.monotonic(), pose[0], pose[1]))

    # -- derived readings -------------------------------------------------------------

    def displacement_over_last(self, window_s):
        if not self.track:
            return None
        t_end = self.track[-1][0]
        past = [p for p in self.track if p[0] <= t_end - window_s]
        if not past:
            return None
        a, b = past[-1], self.track[-1]
        return math.hypot(b[1] - a[1], b[2] - a[2])

    def closest_to_obstacle_after(self, t0):
        pts = [p for p in self.track if p[0] >= t0]
        if not pts:
            return None
        return min(math.hypot(x - self.args.obstacle_x, y - self.args.obstacle_y)
                   for _, x, y in pts)


def watcher_present(node):
    return any("refusal_watcher" in n for n in node.get_node_names())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--arm", required=True,
                    choices=("falsifier", "certifier",
                             "transient-short", "transient-long"))
    ap.add_argument("--obstacle-x", type=float, default=0.65)
    ap.add_argument("--obstacle-y", type=float, default=0.0)
    ap.add_argument("--goal-x", type=float, default=1.30)
    ap.add_argument("--goal-y", type=float, default=0.0)
    ap.add_argument("--hold-s", type=float, default=6.0,
                    help="transient-short: seconds the obstacle stays once seen")
    ap.add_argument("--timeout", type=float, default=90.0)
    args = ap.parse_args()

    rclpy.init()
    rig = Rig(args)
    spinner = threading.Thread(target=rclpy.spin, args=(rig,), daemon=True)
    spinner.start()
    time.sleep(2.0)

    present = watcher_present(rig)
    if args.arm == "falsifier" and present:
        say("INCONCLUSIVE: refusal_watcher is on the graph during the FALSIFIER "
            "arm -- the known-bad cannot be reproduced with the cure running.")
        return 3
    if args.arm in ("certifier", "transient-short", "transient-long") and not present:
        say(f"INCONCLUSIVE: {args.arm} needs refusal_watcher on the graph.")
        return 3

    baseline_stalls = rig.stall_events
    rig.obstacle_on = True

    nav = ActionClient(rig, NavigateToPose, "navigate_to_pose")
    if not nav.wait_for_server(timeout_sec=20.0):
        say("INCONCLUSIVE: no navigate_to_pose server")
        return 3
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = args.goal_x
    goal.pose.pose.position.y = args.goal_y
    goal.pose.pose.orientation.w = 1.0
    recoveries = [0]

    def fb(msg):
        recoveries[0] = msg.feedback.number_of_recoveries

    say(f"arm={args.arm} obstacle=({args.obstacle_x},{args.obstacle_y}) "
        f"goal=({args.goal_x},{args.goal_y}) -- sending")
    send = nav.send_goal_async(goal, feedback_callback=fb)
    t0 = time.monotonic()
    while not send.done() and time.monotonic() - t0 < 15:
        time.sleep(0.1)
    handle = send.result()
    if not handle or not handle.accepted:
        say("INCONCLUSIVE: goal not accepted")
        return 3
    result_future = handle.get_result_async()

    hold_deadline = None
    status = None
    while time.monotonic() - t0 < args.timeout:
        if args.arm == "transient-short" and rig.obstacle_seen_at and hold_deadline is None:
            hold_deadline = rig.obstacle_seen_at + args.hold_s
        if hold_deadline and time.monotonic() >= hold_deadline and rig.obstacle_on:
            rig.obstacle_on = False
            say(f"virtual obstacle REMOVED after {args.hold_s:.0f} s hold")
        if args.arm == "transient-long" and rig.marks_seen_nonzero and rig.obstacle_on:
            rig.obstacle_on = False
            say("promotion observed; obstacle REMOVED -- the mark now outlives it "
                "(mission-permanent until revocation: the named cost, on display)")
        if result_future.done():
            status = result_future.result().status
            break
        time.sleep(0.3)
    if status is None:
        say("goal did not terminate inside the timeout")
        cancel = handle.cancel_goal_async()
        t1 = time.monotonic()
        while not cancel.done() and time.monotonic() - t1 < 10:
            time.sleep(0.1)

    disp = rig.displacement_over_last(STALL_WINDOW_S)
    stalls_delta = ((rig.stall_events or 0) - (baseline_stalls or 0)
                    if rig.stall_events is not None else None)
    say(f"terminal: status={status} recoveries={recoveries[0]} "
        f"last-{STALL_WINDOW_S:.0f}s displacement="
        f"{disp if disp is None else round(disp, 3)} m marks_width={rig.marks_width} "
        f"stall_delta={stalls_delta}")

    if args.arm == "falsifier":
        livelocked = (status != 4 and disp is not None
                      and disp < STALL_DISPLACEMENT_M
                      and recoveries[0] >= MIN_RECOVERIES)
        say("LIVELOCK REPRODUCED -- the rig can now falsify the cure"
            if livelocked else
            "LIVELOCK NOT REPRODUCED -- fix the rig before building the watcher")
        return 0 if livelocked else 1

    if args.arm == "certifier":
        promoted = rig.marks_seen_nonzero and (stalls_delta in (0, None))
        clear = rig.closest_to_obstacle_after(t0)
        ok = (promoted and status == 4
              and clear is not None and clear >= CLEARANCE_M
              and rig.marks_width <= MAX_PROMOTED_POINTS)
        say(f"closest post-send approach to obstacle: "
            f"{clear if clear is None else round(clear, 3)} m (bar {CLEARANCE_M})")
        say("CERTIFIED: promotion -> replan -> goal, no contact, discipline held"
            if ok else "NOT CERTIFIED -- read the terminal line against the bars")
        return 0 if ok else 1

    if args.arm == "transient-short":
        ok = (not rig.marks_seen_nonzero) and status == 4
        say("TRANSIENT PASSED: no promotion, goal reached"
            if ok else "TRANSIENT FAILED: a moment became a mark, or the goal died")
        return 0 if ok else 1

    if args.arm == "transient-long":
        ok = rig.marks_seen_nonzero
        say("SUSTAINED PASSED: promotion fired on a held obstacle"
            if ok else "SUSTAINED FAILED: watcher never promoted a held blockage")
        return 0 if ok else 1
    return 3


def _run():
    """main() with an orderly rclpy shutdown -- a daemon spinner killed by
    sys.exit() dumps core in rcl ('terminate called without an active
    exception'), which eats the exit code the arms exist to produce."""
    try:
        return main()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        time.sleep(0.5)


if __name__ == "__main__":
    sys.exit(_run())
