"""A/B the camera brake: does the swept-path check catch an obstacle the fixed cone
misses during a turn?  (Regression test for the chair-leg fix.)

Place a LOW obstacle roughly 0.55 m ahead and 0.30 m to one side (~30 deg) -- outside
the 21 deg fallback cone but inside the camera's 38 deg left FOV, so the two logics
genuinely disagree. Run once as-is, then restart the supervisor with
`-p low_obstacle_swept_path:=false` and run again.

Expected (measured 2026-08-06): swept catches it on the left arcs (scale ~0.68-0.70)
while the cone never sees it at all (stays 1.00). Straight ahead, a too-tight arc and
a right turn are all correctly ignored -- the check is direction-aware, not just
more conservative.

Gotchas that produced false negatives before being caught:
  * stop `decisive_controller` first -- it publishes zeros to /cmd_vel when idle and
    clobbers the probe commands;
  * sample the state WHILE still commanding (requested_cmd_timeout_s is 0.25 s).

Safe by construction -- run with rvr_node STOPPED. The supervisor still evaluates the
commanded twist and reports cam_nearest/cam_scale on /collision_stop/state, but
nothing reaches the motors, so the rover cannot move.

Reports where the obstacle actually is (so placement can be checked), then commands
a straight push and a left arc and records what the brake decides for each.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String

LABEL = sys.argv[1] if len(sys.argv) > 1 else "arm"

rclpy.init()
n = rclpy.create_node("swept_ab")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
st = {"pts": [], "state": ""}
n.create_subscription(PointCloud2, "/camera/low_obstacles",
                      lambda m: st.__setitem__("pts", [(p[0], p[1]) for p in
                                                       pc2.read_points(m, field_names=("x", "y"), skip_nans=True)]), 5)
n.create_subscription(String, "/collision_stop/state", lambda m: st.__setitem__("state", m.data), 10)


def spin(s):
    end = time.monotonic() + s
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)


def field(key):
    for tok in st["state"].split():
        if tok.startswith(key + "="):
            return tok.split("=", 1)[1]
    return "?"


spin(3.0)
# --- where is the obstacle, really? ---
near = [(math.hypot(x, y), math.degrees(math.atan2(y, x)), x, y)
        for x, y in st["pts"] if 0.30 < math.hypot(x, y) < 1.30]
near.sort()
print("=== %s ===" % LABEL)
print("camera cloud: %d pts, %d in 0.3-1.3 m" % (len(st["pts"]), len(near)))
if near:
    print("nearest few (range, bearing +=left):")
    for r, b, x, y in near[:5]:
        print("   %.2f m at %+5.1f deg   (x=%.2f, y=%+.2f)" % (r, b, x, y))
    inside21 = [p for p in near if abs(p[1]) <= 21.0]
    print("  -> %d of them inside the 21 deg cone, %d outside it"
          % (len(inside21), len(near) - len(inside21)))
else:
    print("  NO obstacle detected in 0.3-1.3 m -- check placement/lighting")

# --- probe the brake with two commanded motions (no motion: driver is stopped) ---
ARCS = [("STRAIGHT      w= 0.00  (R=inf)", 0.10, 0.00)]
for w in (0.10, 0.15, 0.18, 0.25, 0.40):
    ARCS.append(("LEFT ARC      w=%+.2f  (R=%.2f m)" % (w, 0.10 / w), 0.10, w))
ARCS.append(("RIGHT ARC     w=-0.18  (R=0.56 m)", 0.10, -0.18))
for name, lin, ang in ARCS:
    # Sample WHILE still commanding: requested_cmd_timeout_s is 0.25 s, so if we
    # stop publishing and then read, the supervisor has already zeroed the command
    # and the camera brake early-returns -- measuring nothing.
    end = time.monotonic() + 3.0
    snap = None
    while rclpy.ok() and time.monotonic() < end:
        tw = Twist()
        tw.linear.x = lin
        tw.angular.z = ang
        pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.02)
        if time.monotonic() > end - 1.0 and "cam_scale" in st["state"]:
            snap = st["state"]
        time.sleep(0.05)
    saved = st["state"]
    st["state"] = snap or saved
    # cam_considered is what separates the two ways an arm can read "no brake":
    # the swept path was genuinely clear, or nothing was in range to consider.
    # An A/B on swept-vs-cone gating is exactly where that distinction decides
    # whether an arm proved anything.
    print("  %s -> cam_nearest=%-7s cam_scale=%-6s considered=%-4s out_linear=%s"
          % (name, field("cam_nearest"), field("cam_scale"),
             field("cam_considered"), field("cam_output_linear")))
    st["state"] = saved
for _ in range(10):
    pub.publish(Twist())
    time.sleep(0.05)
print("  (stopped publishing)")
