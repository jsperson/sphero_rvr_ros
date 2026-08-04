"""Subscribe once to an OccupancyGrid (/map or /global_costmap/costmap) and report
the frontier-relevant content: cell histogram (free / unknown / lethal / inflation),
the number of FRONTIER cells (free cells adjacent to unknown = explore_lite's
condition), and the robot's own cell value (in inflation/lethal would break
explore's search-from-robot). Bench test for open item #1b.

Usage:
  python3 costmap_analyze.py            # defaults to /global_costmap/costmap
  python3 costmap_analyze.py /map       # SLAM output (frontier source via static layer)
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/global_costmap/costmap"


class CostmapAnalyzer(Node):
    def __init__(self):
        super().__init__("costmap_analyzer")
        self.got = False
        # map/costmap topics latch with TRANSIENT_LOCAL + RELIABLE.
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, TOPIC, self.cb, qos)

    def cb(self, msg):
        if self.got:
            return
        self.got = True
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        data = msg.data

        free = unknown = lethal = infl = 0
        for v in data:
            if v < 0:
                unknown += 1
            elif v == 0:
                free += 1
            elif v >= 100:
                lethal += 1
            else:
                infl += 1

        def at(x, y):
            return data[y * w + x]

        frontier = 0
        for y in range(h):
            for x in range(w):
                if at(x, y) != 0:
                    continue
                if (
                    (x > 0 and at(x - 1, y) < 0)
                    or (x < w - 1 and at(x + 1, y) < 0)
                    or (y > 0 and at(x, y - 1) < 0)
                    or (y < h - 1 and at(x, y + 1) < 0)
                ):
                    frontier += 1

        rx = int((0.0 - ox) / res)
        ry = int((0.0 - oy) / res)
        if 0 <= rx < w and 0 <= ry < h:
            rv = at(rx, ry)
        else:
            rv = "OUT_OF_BOUNDS"

        print(f"{TOPIC}: {w}x{h} res={res:.3f} origin=({ox:.2f},{oy:.2f}) cells={len(data)}")
        print(f"  free(0)         = {free}")
        print(f"  unknown(-1)     = {unknown}   <-- must be >0 for any frontier to exist")
        print(f"  lethal(>=100)   = {lethal}")
        print(f"  inflation(1-99) = {infl}")
        print(f"  FRONTIER cells (free next to unknown) = {frontier}   <-- the #1b answer")
        print(f"  robot cell ({rx},{ry}) value = {rv}   (0=free  -1=unknown  >=100=lethal  1-99=inflation)")


def main():
    rclpy.init()
    node = CostmapAnalyzer()
    t0 = time.monotonic()
    while rclpy.ok() and not node.got and time.monotonic() - t0 < 25:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not node.got:
        print(f"NO MESSAGE RECEIVED on {TOPIC} within 25 s")
    node.destroy_node()
    rclpy.shutdown()


main()
