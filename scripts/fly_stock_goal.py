#!/usr/bin/env python3
"""Liftoff for the stock middle: ONE verified NavigateToPose goal, then the result.

    python3 scripts/fly_stock_goal.py --x 1.50 --y 0.10       # explicit map goal
    python3 scripts/fly_stock_goal.py --ahead 1.5             # along the current nose

This tool owns arming for the stock middle -- `launch_and_arm.py --stack stock`
deliberately never sends a goal. One goal per invocation by construction: it sends
exactly one, stays attached logging feedback, prints the terminal status, and
SIGINT cancels cleanly. It never retries and never re-sends.

BEFORE ANY SEND, two gates, both learned in the field on 2026-08-18:

  LIVENESS -- bt_navigator, controller_server and slam_toolbox must each report
  lifecycle ACTIVE (asked via their own get_state service, not inferred from a node
  listing). A goal fired into a half-up stack is how the acknowledge-orphan class
  is reborn one tool-composition mistake later.

  THE TRINITY -- the goal cell must be (1) MAPPED FREE in /map: goal 1 of run 3c
  was 1.5 m "dead ahead" and physically inside the couch, in SLAM-unknown space --
  the operator's mental map and the furniture are the same thing the planner is
  asked about, so the map is consulted, not the mind's eye; (2) COST 0 in the
  global costmap; (3) PLANNABLE by a ComputePathToPose dry-run. Every verdict is
  printed as the announce block before liftoff.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

# ROS imports are guarded so the PURE policy below (trinity_verdict) stays
# importable on the dev machine, where there is no rclpy by design and the tests
# hold the policy against the couch/boot fixtures. Running the tool needs ROS.
try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from geometry_msgs.msg import PoseStamped
    from tf2_ros import Buffer, TransformListener
    HAVE_ROS = True
except ImportError:            # pure-policy import on the dev machine
    HAVE_ROS = False
    Node = object

#: The nodes a goal's life depends on. planner_server is proven by the dry-run
#: itself; behavior_server failures surface as recovery refusals, not silence.
LIVENESS_NODES = ("bt_navigator", "controller_server", "slam_toolbox")


def say(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def trinity_verdict(map_val, cost_val):
    """(ok, reason) for a goal cell, pure. The couch is case one.

    `None` means the cell is outside the grid entirely; -1 in /map is SLAM-unknown.
    Cost > 0 is refused outright rather than graded: a goal in the inflation band
    invites RPP to finish inside cost it will then flinch at, and open floor was
    always available to whoever picked the goal.
    """
    if map_val is None or cost_val is None:
        return False, "outside the mapped grid"
    if map_val == -1:
        return False, "SLAM-unknown -- nobody has seen this floor (the couch lesson)"
    if map_val != 0:
        return False, f"occupied in /map (value {map_val})"
    if cost_val != 0:
        return False, f"global costmap holds cost {cost_val} here"
    return True, "mapped free, cost 0"


def qprof():
    p = QoSProfile(depth=1)
    p.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    p.reliability = QoSReliabilityPolicy.RELIABLE
    return p


class GoalTool(Node):
    def __init__(self):
        super().__init__("fly_stock_goal")
        self.grids = {}
        self.create_subscription(
            OccupancyGrid, "/map",
            lambda m: self.grids.__setitem__("map", m), qprof())
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap",
            lambda m: self.grids.__setitem__("cost", m), qprof())
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)

    def cell(self, which, x, y):
        grid = self.grids.get(which)
        if grid is None:
            return None
        info = grid.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return None
        return grid.data[my * info.width + mx]


def check_liveness(node) -> bool:
    ok = True
    for name in LIVENESS_NODES:
        client = node.create_client(GetState, f"/{name}/get_state")
        if not client.wait_for_service(timeout_sec=5.0):
            say(f"LIVENESS: /{name} get_state unavailable")
            ok = False
            continue
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        result = future.result()
        label = result.current_state.label if result else "no reply"
        say(f"LIVENESS: /{name} = {label}")
        ok = ok and label == "active"
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--x", type=float, help="map-frame goal x (with --y)")
    group.add_argument("--ahead", type=float,
                       help="metres ahead along the current nose")
    ap.add_argument("--y", type=float, help="map-frame goal y (with --x)")
    ap.add_argument("--tf-wait-s", type=float, default=30.0)
    args = ap.parse_args()
    if args.x is not None and args.y is None:
        ap.error("--x needs --y")
    if not HAVE_ROS:
        say("this tool flies a robot; run it on the Pi under a sourced workspace")
        return 2

    rclpy.init()
    node = GoalTool()

    if not check_liveness(node):
        say("REFUSED: stack is not fully active -- a goal into a half-up stack "
            "is the acknowledge-orphan class reborn. Bring it up with "
            "launch_and_arm.py --stack stock first.")
        return 2

    deadline = time.monotonic() + args.tf_wait_s
    tf = None
    while time.monotonic() < deadline and (tf is None or len(node.grids) < 2):
        rclpy.spin_once(node, timeout_sec=0.5)
        try:
            tf = node.tfb.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            tf = None
    if tf is None or len(node.grids) < 2:
        say(f"REFUSED: missing {'TF' if tf is None else ''} "
            f"{'grids' if len(node.grids) < 2 else ''} after {args.tf_wait_s}s")
        return 2

    t = tf.transform.translation
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if args.x is not None:
        gx, gy = args.x, args.y
        gyaw = math.atan2(gy - t.y, gx - t.x)
    else:
        gx = t.x + args.ahead * math.cos(yaw)
        gy = t.y + args.ahead * math.sin(yaw)
        gyaw = yaw

    ok, reason = trinity_verdict(node.cell("map", gx, gy), node.cell("cost", gx, gy))
    say(f"ANNOUNCE: pose ({t.x:.3f},{t.y:.3f}) yaw {math.degrees(yaw):.0f} deg; "
        f"goal ({gx:.3f},{gy:.3f}), {math.hypot(gx - t.x, gy - t.y):.2f} m away; "
        f"cell verdict: {reason}")
    if not ok:
        say("REFUSED: the goal cell fails the trinity -- pick a spot the map has "
            "actually seen free.")
        return 2

    planner = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not planner.wait_for_server(timeout_sec=15.0):
        say("REFUSED: no compute_path_to_pose server")
        return 2
    dry = ComputePathToPose.Goal()
    dry.goal = PoseStamped()
    dry.goal.header.frame_id = "map"
    dry.goal.pose.position.x, dry.goal.pose.position.y = gx, gy
    dry.goal.pose.orientation.z = math.sin(gyaw / 2)
    dry.goal.pose.orientation.w = math.cos(gyaw / 2)
    dry.use_start = False
    send = planner.send_goal_async(dry)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        say("REFUSED: dry-run not accepted")
        return 2
    rf = handle.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=20.0)
    res = rf.result()
    poses = len(res.result.path.poses) if res and res.status == 4 else 0
    say(f"ANNOUNCE: dry-run {'PLANS ' + str(poses) + ' poses' if poses else 'REFUSED'}")
    if not poses:
        say("REFUSED: the planner cannot reach this goal from here.")
        return 2

    nav = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not nav.wait_for_server(timeout_sec=15.0):
        say("REFUSED: no navigate_to_pose server")
        return 2
    goal = NavigateToPose.Goal()
    goal.pose = dry.goal
    goal.pose.header.stamp = node.get_clock().now().to_msg()

    last = [0.0]

    def on_feedback(fb):
        now = time.monotonic()
        if now - last[0] >= 5.0:
            last[0] = now
            f = fb.feedback
            p = f.current_pose.pose.position
            say(f"feedback: pose ({p.x:.2f},{p.y:.2f}) dist_remaining "
                f"{f.distance_remaining:.2f} recoveries {f.number_of_recoveries} "
                f"time {f.navigation_time.sec}s")

    say("SENDING GOAL (this is liftoff)")
    send = nav.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        say("FAIL: goal not accepted")
        return 2
    say("goal ACCEPTED")

    result_future = handle.get_result_async()
    try:
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        say("SIGINT: cancelling goal")
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=10.0)
        return 130

    status = result_future.result().status
    names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
    say(f"RESULT: status={status} ({names.get(status, 'other')})")
    return 0 if status == 4 else 1


if __name__ == "__main__":
    sys.exit(main())
