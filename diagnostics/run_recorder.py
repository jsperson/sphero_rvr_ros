"""Record what happened during a chassis run, so a failure is diagnosable afterwards.

Chassis runs are the scarce resource on this project, and one was wasted on
2026-08-09: the rover made contact with obstacles and the brake state was never
captured, so "did the brake fire?" could not be answered and the run had to be
written off. Anything that is not recorded may have to be repeated.

Records to a CSV, one row per sample:
  t, state, front, rear, left, right, cmd_vx, cmd_wz, out_vx, out_wz, odom_x, odom_y

  * state/front/rear/... come from /collision_stop/state -- did the brake see it,
    and did it act?
  * cmd_* is what the controller asked for; out_* is what the supervisor allowed.
    The gap between them IS the brake's behaviour.
  * odom_* says whether the robot actually moved, which is what separates "stuck"
    from "commanded to stop".

Run it alongside a mission; Ctrl-C or let it time out.

Usage: python3 run_recorder.py [seconds] [outfile]     (default 600, ~/run_<ts>.csv)
"""

import csv
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

FIELDS = ("front", "rear", "left", "right")


class Recorder(Node):
    def __init__(self, writer):
        super().__init__("run_recorder")
        self.w = writer
        self.state = ""
        self.near = {k: "" for k in FIELDS}
        self.cmd = (0.0, 0.0)
        self.out = (0.0, 0.0)
        self.odom = (0.0, 0.0)
        self.t0 = time.monotonic()
        self.create_subscription(String, "/collision_stop/state", self._on_state, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Twist, "/cmd_vel_motor", self._on_out, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_timer(0.1, self._sample)   # 10 Hz is plenty and keeps files small

    def _on_state(self, m):
        d = m.data
        self.state = d.split(" ", 1)[0]
        for k in FIELDS:
            hit = re.search(rf"\b{k}=([-\d.]+|None)", d)
            self.near[k] = hit.group(1) if hit else ""

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def _on_out(self, m):
        self.out = (m.linear.x, m.angular.z)

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)

    def _sample(self):
        self.w.writerow([
            round(time.monotonic() - self.t0, 2), self.state,
            *[self.near[k] for k in FIELDS],
            round(self.cmd[0], 3), round(self.cmd[1], 3),
            round(self.out[0], 3), round(self.out[1], 3),
            round(self.odom[0], 3), round(self.odom[1], 3),
        ])


def main():
    path = OUT or time.strftime("/home/jsperson/run_%Y%m%d_%H%M%S.csv")
    rclpy.init()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "state", "front", "rear", "left", "right",
                    "cmd_vx", "cmd_wz", "out_vx", "out_wz", "odom_x", "odom_y"])
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
