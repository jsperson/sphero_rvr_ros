"""Bench validation for the coverage churn fix (no motion). Brings nothing up
itself — run it against a live frontier_diag stack (lidar+SLAM+global costmap +
planner_server). It repeatedly runs the coverage core's select_next_goal and asks
the PLANNER (ComputePathToPose) whether each selected target is reachable. With
navigability-aware selection every selected target should be PLANNABLE; with it
disabled (inscribed_radius=0, the old free-only behavior) some should be NO PATH.

Usage: python3 plannability_check.py [inscribed_radius_m]   (default 0.14)
"""

import math
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
    cell_center_world,
    select_next_goal,
    stamp_coverage,
    world_grid,
)

INSCRIBED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.14


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
    blacklist = set()
    cfg = CoverageConfig(inscribed_radius_m=INSCRIBED)

    print(f"=== plannability check: inscribed_radius_m={INSCRIBED} (0.0 = old free-only) ===")
    ok = bad = checked = 0
    for _ in range(12):
        cell = select_next_goal(m.data, w, hh, ox, oy, res, rcx, rcy, covered, blacklist, cfg)
        if cell is None:
            print(f"select_next_goal -> None after {checked} targets")
            break
        gwx, gwy = cell_center_world(cell[0], cell[1], ox, oy, res)
        p = n.plan_ok(frame, gwx, gwy)
        checked += 1
        label = "PLANNABLE" if p else ("NO PATH" if p is False else "PLANNER?")
        if p:
            ok += 1
        else:
            bad += 1
        print(f"  target {cell} world ({gwx:.2f},{gwy:.2f}) -> {label}")
        r = int(math.ceil(0.3 / res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                blacklist.add(world_grid(gwx + dx * res, gwy + dy * res, res))
    print(f"=== RESULT: {ok}/{checked} selected targets PLANNABLE ({bad} not) ===")
    n.destroy_node()
    rclpy.shutdown()


main()
