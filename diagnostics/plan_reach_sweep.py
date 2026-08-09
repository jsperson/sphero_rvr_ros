"""How far ahead can the planner actually reach? Baseline check for any gap test.

Asks ComputePathToPose for a fan of goals at increasing distance straight ahead (and
optionally off to each side) and reports PATH / NO PATH for each, plus the costmap
cost of the robot's own cell and of every goal cell. No motion.

Why: "NO PATH to X" on its own is uninterpretable -- the goal may simply be past a
wall, or the robot may be sitting in inflation so nothing at all can be planned
(the 2026-08-07 rule: robot_radius 0.14 + inflation_radius 0.16 = 0.30 m of required
clearance, below which the START pose is in collision and every goal fails). This
separates "the gap is blocked" from "this test was invalid".

Usage:  python3 plan_reach_sweep.py                       # 0.5..2.5 m ahead
        python3 plan_reach_sweep.py --lateral 0.4         # also +/- 0.4 m fans
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
import tf2_ros

COST = {-1: "unknown", 0: "free"}


def label(v):
    if v is None:
        return "off-map"
    if v in COST:
        return COST[v]
    if v >= 100:
        return "LETHAL"
    if v >= 99:
        return "inscribed"
    return f"cost{v}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lateral", type=float, default=0.0)
    ap.add_argument("--max", type=float, default=2.5)
    ap.add_argument("--step", type=float, default=0.25)
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node("plan_reach_sweep")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    got = {}
    n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: got.__setitem__("cm", m), qos)

    end = time.monotonic() + 15
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
        if "cm" in got and buf.can_transform("map", "base_link", rclpy.time.Time()):
            break
    cm = got["cm"]
    t = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
    q = t.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    rx, ry = t.translation.x, t.translation.y
    res = cm.info.resolution
    ox, oy = cm.info.origin.position.x, cm.info.origin.position.y

    def cost(wx, wy):
        c, r = int((wx - ox) / res), int((wy - oy) / res)
        if not (0 <= c < cm.info.width and 0 <= r < cm.info.height):
            return None
        return cm.data[r * cm.info.width + c]

    rc = cost(rx, ry)
    print(f"robot ({rx:.2f}, {ry:.2f}) yaw {math.degrees(yaw):+.0f} deg -- own cell: {label(rc)}")
    if rc is not None and rc >= 99:
        print("  !! START POSE IS IN INFLATION -- every goal will fail regardless of the gap.")
        print("     Needs >0.30 m clearance all round (robot_radius 0.14 + inflation 0.16).")

    ac = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
    if not ac.wait_for_server(timeout_sec=10.0):
        raise SystemExit("compute_path_to_pose unavailable")

    def ask(fwd, lat):
        gx = rx + fwd * math.cos(yaw) - lat * math.sin(yaw)
        gy = ry + fwd * math.sin(yaw) + lat * math.cos(yaw)
        g = ComputePathToPose.Goal()
        g.goal = PoseStamped()
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x, g.goal.pose.position.y = gx, gy
        g.goal.pose.orientation.w = 1.0
        g.use_start = False
        fut = ac.send_goal_async(g)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        rf = fut.result().get_result_async()
        while rclpy.ok() and not rf.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        r = rf.result()
        poses = len(r.result.path.poses) if r.status == 4 else 0
        return gx, gy, poses

    lats = [0.0] if args.lateral == 0 else [args.lateral, 0.0, -args.lateral]
    for lat in lats:
        print(f"\n  lateral {lat:+.2f} m")
        print(f"  {'fwd_m':>6} {'goal cell':>10}  {'result':>9}  poses")
        f = args.step
        while f <= args.max + 1e-6:
            gx, gy, poses = ask(f, lat)
            print(f"  {f:6.2f} {label(cost(gx, gy)):>10}  {'PATH' if poses else 'NO PATH':>9}  {poses}")
            f += args.step

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
