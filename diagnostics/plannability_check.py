"""How much of the ground /map calls reachable will the PLANNER actually route to?

No motion. Brings nothing up itself — run it against a live stack that has SLAM, a
global costmap and planner_server (the frontier_diag bench stack is enough).

This is the coverage explorer's selection loop, run once and printed. The explorer
proposes targets from SLAM's /map nearest-first and asks ComputePathToPose about
each until one plans; this shows the whole candidate list with the planner's verdict
on every entry, instead of just the winner.

Read the result two ways:

  * As a regression check on the explorer. The first PLANNABLE candidate is the goal
    the node would send, and the count above it is what the planner rejected on the
    way. A handful of rejections is normal and cheap (one query each). Rejections
    everywhere means the explorer is about to report "targets wanted but none
    plannable", which is a real condition, not a bug.

  * As the narrow-gap measurement. Every NO PATH is a place SLAM says is free and
    open, connected to the robot through free cells, that the costmap will not let
    the planner enter. That is the same disagreement the 2026-08-08 A/B found at the
    doorway (109 cells lethal in the costmap but free in /map). Run it camera-on and
    camera-off to see how much of the disagreement the camera layer accounts for.

Usage: python3 plannability_check.py [max_candidates]     (default 12)
"""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
import tf2_ros

from sphero_rvr_core.coverage_exploration import (
    CoverageConfig,
    candidate_goals,
    cell_center_world,
    stamp_coverage,
)

MAX_CANDIDATES = int(sys.argv[1]) if len(sys.argv) > 1 else 12


class Checker(Node):
    def __init__(self):
        super().__init__("plannability_check")
        self.map = None
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

    def _on_map(self, m):
        self.map = m

    def robot_pose(self, frame):
        tf = self.tfb.lookup_transform(frame, "base_link", rclpy.time.Time())
        return tf.transform.translation.x, tf.transform.translation.y

    def plan_ok(self, frame, wx, wy):
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = frame
        goal.goal.pose.position.x = float(wx)
        goal.goal.pose.position.y = float(wy)
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False
        if not self.planner.wait_for_server(timeout_sec=5.0):
            return None
        fut = self.planner.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=8.0)
        res = rfut.result()
        if res is None:
            return False
        return res.status == 4 and len(res.result.path.poses) > 0


def main():
    rclpy.init()
    n = Checker()
    t0 = time.monotonic()
    while rclpy.ok() and n.map is None and time.monotonic() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.5)
    if n.map is None:
        print("NO /map received")
        return
    m = n.map
    info = m.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    w, hh = info.width, info.height
    frame = m.header.frame_id or "map"

    rp = None
    for _ in range(15):
        try:
            rp = n.robot_pose(frame)
            break
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.3)
    if rp is None:
        print("NO robot pose (map->base_link)")
        return
    wx, wy = rp
    rcx = int((wx - ox) / res)
    rcy = int((wy - oy) / res)

    covered = set()
    stamp_coverage(covered, wx, wy, res, 0.75)
    cfg = CoverageConfig(max_candidates=MAX_CANDIDATES)
    cells = candidate_goals(m.data, w, hh, ox, oy, res, rcx, rcy, covered, set(), cfg)

    print(f"=== planner verdict on {len(cells)} /map candidates "
          f"(robot at {wx:.2f},{wy:.2f}) ===")
    if not cells:
        print("  no candidates — /map says everything reachable is covered")
    ok = bad = 0
    chosen = None
    for cell in cells:
        gwx, gwy = cell_center_world(cell[0], cell[1], ox, oy, res)
        p = n.plan_ok(frame, gwx, gwy)
        label = "PLANNABLE" if p else ("NO PATH" if p is False else "PLANNER?")
        if p:
            ok += 1
            if chosen is None:
                chosen = (cell, gwx, gwy)
        else:
            bad += 1
        print(f"  candidate {cell} world ({gwx:.2f},{gwy:.2f}) -> {label}")
    print(f"=== {ok}/{len(cells)} plannable; {bad} refused by the costmap ===")
    if chosen:
        cell, gwx, gwy = chosen
        print(f"the explorer would drive to {cell} ({gwx:.2f},{gwy:.2f})")
    elif cells:
        print("the explorer would report: targets wanted but NONE plannable")
    n.destroy_node()
    rclpy.shutdown()


main()
