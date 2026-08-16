"""Record what happened during a chassis run, so a failure is diagnosable afterwards.

Chassis runs are the scarce resource on this project, and one was wasted on
2026-08-09: the rover made contact with obstacles and the brake state was never
captured, so "did the brake fire?" could not be answered and the run had to be
written off. Anything that is not recorded may have to be repeated.

Records to a CSV, one row per sample:
  t, state, reason, front, rear, left, right, cam_cloud_age, cam_nearest, cam_scale,
  cam_considered, pivot_veto, avoid_offset, cmd_vx, cmd_wz, out_vx, out_wz, odom_x,
  odom_y, odom_yaw_deg

  * state/front/rear/... come from /collision_stop/state -- did the brake see it,
    and did it act?
  * reason is the supervisor's own word for WHICH gate acted. `state` is too coarse
    to diagnose with: several different paths all report SLOW, and one of them
    zeroes a commanded pivot outright. Recording state without reason is what left
    run 20260810_185048 undiagnosable.
  * avoid_offset is the STEERING LAW's own answer to "did you engage, and how
    hard": the heading offset it applied this cycle, in radians, zeros included.
    Gauntlet 1 flew it unobservable and the question "was it leaning before the
    stop?" had no artifact that could answer -- so it is a column now.
  * cam_nearest / cam_scale say what the camera brake DID: scale 0.0 is the hard
    cut at 0.50 m that ended most of run 114626's goals, and cam_nearest is the
    range it cut at. Without them, "the lidar said SLOW but the motors got zero"
    has to be reasoned about instead of read.
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
from std_msgs.msg import Float32, String

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
OUT = sys.argv[2] if len(sys.argv) > 2 else None

# Numeric-or-None fields scraped from /collision_stop/state.
# cam_cloud_age is D22's instrument: the camera pivot veto and the forward camera
# brake both fail OPEN on a stale cloud, and on 2026-08-10 the supervisor was
# measured starving its own cloud subscription under load while an independent
# observer received every message. Without this column a run recording cannot say
# whether the camera layer was actually live when it mattered.
#
# cam_nearest / cam_scale were added 2026-08-11 after grading gauntlet run 114626,
# where 55 s of the mission's 86 s of "commanding but not moving" had to be
# ATTRIBUTED TO THE CAMERA BY ARITHMETIC rather than simply read: the core's
# min_forward_scale is 0.70, so a row reading `SLOW / front_slow` proves the lidar
# emitted >= 0.14 m/s, and a motor output of 0.000 in the same row can only be the
# camera brake. The supervisor already publishes both numbers on
# /collision_stop/state; nothing was recording them, so the run also cannot say
# whether those stops were real low obstacles or D27 sun phantoms. Two columns of an
# existing instrument, and they turn a derivation into a reading.
#
# cam_considered was added 2026-08-15 after autopsy #2, and it is the column that
# autopsy needed and did not have. `tof_obstacles` counts rule-B zones over the
# SENSOR'S WHOLE REACH while cam_scale acts only within the brake's range window and
# swept path -- two different populations, one log line, and for two sessions the
# first stood in as a proxy for the second. "Zones cycled 0->10 while cam_scale never
# left 1.00" was read as detections being lost between the sensor and the brake; they
# were at 0.54-1.56 m against a 0.60 m reach, correctly ignored. This column is the
# count AFTER the brake's own filters, so the comparison stops being a guess. EMPTY
# means the brake did not look; 0 means it looked and found nothing.
NUM_FIELDS = ("front", "rear", "left", "right", "cam_cloud_age",
              "cam_nearest", "cam_scale", "cam_considered")
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
        # Initialised, not left to the first message: _sample() runs on a timer from
        # the moment the node exists, and an un-set attribute here would raise inside
        # the timer callback -- which in this node means the recording quietly stops
        # while the mission carries on believing it is being recorded.
        self.avoid_offset = 0.0
        # THE ToF, RECORDED BESIDE THE CAMERA IT WILL REPLACE. Stage (i) of
        # docs/tof_navigation_design.md: the sensor has no motion authority, so the
        # ONLY way stage (ii) can compare the two is if both are in the same rows of
        # the same file on the same missions. Columns cost nothing; a side-by-side
        # comparison that has to align two recordings costs a day.
        self.tof_obstacles = ""
        self.tof_rule_a = ""
        self.tof_rule_b = ""
        self.tof_state = ""
        self.tof_rate = ""
        # Rule B's health, which is the whole point of flying both sensors together:
        # "ok" means the lidar background was usable, anything else means rule B was
        # UNAVAILABLE and the ToF was running on rule A alone. A column of obstacle
        # counts without this cannot be interpreted afterwards.
        self.tof_background = ""
        self.odom = (0.0, 0.0)
        self.yaw = 0.0
        self.t0 = time.monotonic()
        self.create_subscription(String, "/collision_stop/state", self._on_state, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Twist, "/cmd_vel_motor", self._on_out, 10)
        self.create_subscription(
            Float32, "/decisive_controller/avoid_offset", self._on_avoid, 10)
        self.create_subscription(String, "/tof/state", self._on_tof, 10)
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

    def _on_avoid(self, m):
        self.avoid_offset = m.data

    def _on_tof(self, m):
        """Scrape the ToF's own health line. Read-only, like every other column: the
        recorder never asks a node a question it does not already publish."""
        self.tof_state = (m.data.split() or [""])[0]
        for field in m.data.split():
            if field.startswith("obstacle_zones="):
                self.tof_obstacles = field.split("=", 1)[1]
            elif field.startswith("rule_a_zones="):
                self.tof_rule_a = field.split("=", 1)[1]
            elif field.startswith("rule_b_zones="):
                self.tof_rule_b = field.split("=", 1)[1]
            elif field.startswith("background="):
                self.tof_background = field.split("=", 1)[1]
            elif field.startswith("rate_hz="):
                self.tof_rate = field.split("=", 1)[1]

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
            round(self.avoid_offset, 3),
            self.tof_state, self.tof_rate, self.tof_obstacles,
            self.tof_rule_a, self.tof_rule_b, self.tof_background,
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
        w.writerow(["t", "state", "reason", *FIELDS, "avoid_offset",
                    "tof_state", "tof_rate", "tof_obstacles",
                    "tof_rule_a", "tof_rule_b", "tof_background",
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
