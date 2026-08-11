"""Record what happened during a chassis run, so a failure is diagnosable afterwards.

Chassis runs are the scarce resource on this project, and one was wasted on
2026-08-09: the rover made contact with obstacles and the brake state was never
captured, so "did the brake fire?" could not be answered and the run had to be
written off. Anything that is not recorded may have to be repeated.

Records to a CSV, one row per sample:
  t, state, reason, front, rear, left, right, cam_cloud_age, pivot_veto,
  cmd_vx, cmd_wz, out_vx, out_wz, odom_x, odom_y, odom_yaw_deg

  * state/front/rear/... come from /collision_stop/state -- did the brake see it,
    and did it act?
  * reason is the supervisor's own word for WHICH gate acted. `state` is too coarse
    to diagnose with: several different paths all report SLOW, and one of them
    zeroes a commanded pivot outright. Recording state without reason is what left
    run 20260810_185048 undiagnosable.
  * cam_cloud_age / pivot_veto say whether the CAMERA layer was live and whether it
    refused a turn. Both camera gates fail OPEN on a stale cloud, so a run without
    this column cannot distinguish "the camera cleared it" from "the camera was not
    looking" (D22). The 2026-08-10 run could not answer that question at all.
  * cmd_* is what the controller asked for; out_* is what the supervisor allowed.
    The gap between them IS the brake's behaviour.
  * odom_* says whether the robot actually moved, which is what separates "stuck"
    from "commanded to stop"; odom_yaw_deg says which way it was FACING, without
    which no stall can be attributed to a direction (see docs/chassis_run_protocol.md).

Run it alongside a mission; Ctrl-C or let it time out.

Usage: python3 run_recorder.py [seconds] [outfile]     (default 600, ~/run_<ts>.csv)
"""

import csv
import math
import re
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
OUT = sys.argv[2] if len(sys.argv) > 2 else None

# Numeric-or-None fields scraped from /collision_stop/state.
# cam_cloud_age is D22's instrument: the camera pivot veto and the forward camera
# brake both fail OPEN on a stale cloud, and on 2026-08-10 the supervisor was
# measured starving its own cloud subscription under load while an independent
# observer received every message. Without this column a run recording cannot say
# whether the camera layer was actually live when it mattered.
NUM_FIELDS = ("front", "rear", "left", "right", "cam_cloud_age")
# Boolean words, matched separately (see _on_state).
BOOL_FIELDS = ("pivot_veto",)
FIELDS = NUM_FIELDS + BOOL_FIELDS


class Recorder(Node):
    def __init__(self, writer):
        super().__init__("run_recorder")
        self.w = writer
        self.state = ""
        # The supervisor's OWN word for why it decided what it decided. Run
        # 20260810_185048 died with 140 rows of state=SLOW, cmd_wz=-0.9 and
        # out_wz=0.0: the brake was zeroing every pivot the controller asked for,
        # and `state` alone cannot say which gate did it. Several distinct code
        # paths report SLOW, and only `reason` separates
        # "right_trajectory_blocked" from "rear_hold" from a stale command. That
        # run's diagnosis stalled on exactly this missing column while every other
        # field was present and healthy-looking.
        self.reason = ""
        self.near = {k: "" for k in FIELDS}
        self.cmd = (0.0, 0.0)
        self.out = (0.0, 0.0)
        self.odom = (0.0, 0.0)
        self.yaw = 0.0
        self.t0 = time.monotonic()
        self.create_subscription(String, "/collision_stop/state", self._on_state, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Twist, "/cmd_vel_motor", self._on_out, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_timer(0.1, self._sample)   # 10 Hz is plenty and keeps files small

    def _on_state(self, m):
        # A String SUBSCRIPTION always carries the whole payload -- the truncation
        # that hides the tail of this line ("tf_availab...") is a `ros2 topic echo`
        # DISPLAY artifact only, which is worth stating because it cost real
        # confusion once: the fields below look absent from the CLI and are present
        # on the wire.
        d = m.data
        self.state = d.split(" ", 1)[0]
        hit = re.search(r"\breason=(\S+)", d)
        self.reason = hit.group(1) if hit else ""
        for k in NUM_FIELDS:
            hit = re.search(rf"\b{k}=([-\d.]+|None)", d)
            self.near[k] = hit.group(1) if hit else ""
        for k in BOOL_FIELDS:
            # Separate pattern on purpose: these are words, and the numeric pattern
            # above would silently record an empty column for every sample rather
            # than fail, which is how a telemetry gap hides in plain sight.
            hit = re.search(rf"\b{k}=(true|false)\b", d)
            self.near[k] = hit.group(1) if hit else ""

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def _on_out(self, m):
        self.out = (m.linear.x, m.angular.z)

    def _on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        # Heading, for boxed-in attribution: "the costmap says blocked" and "Scott
        # says that direction is clear" can only be compared if the recording says
        # which way the robot was FACING at that instant. On 2026-08-10 that
        # comparison could not be made after the fact -- the rover had rotated
        # between the human observation and the measurement, so "east" and "the
        # robot's front" were no longer the same bearing and the discrepancy stayed
        # unresolved.
        self.odom = (p.x, p.y)
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _sample(self):
        self.w.writerow([
            round(time.monotonic() - self.t0, 2), self.state, self.reason,
            *[self.near[k] for k in FIELDS],
            round(self.cmd[0], 3), round(self.cmd[1], 3),
            round(self.out[0], 3), round(self.out[1], 3),
            round(self.odom[0], 3), round(self.odom[1], 3),
            round(math.degrees(self.yaw), 1),
        ])


def main():
    path = OUT or time.strftime("/home/jsperson/run_%Y%m%d_%H%M%S.csv")
    rclpy.init()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "state", "reason", *FIELDS,
                    "cmd_vx", "cmd_wz", "out_vx", "out_wz",
                    "odom_x", "odom_y", "odom_yaw_deg"])
        n = Recorder(w)
        print(f"recording -> {path} for {DURATION:.0f}s (Ctrl-C to stop early)")
        end = time.monotonic() + DURATION
        try:
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.2)
                f.flush()          # flush as we go: a run that crashes still leaves data
        except KeyboardInterrupt:
            pass
        n.destroy_node()
    print(f"wrote {path}")
    rclpy.shutdown()


main()
