"""Replay exhibit: what does the STOCK local costmap say at the moments bespoke froze?

Feed a recorded mission's /scan and /tf into a live Nav2 local costmap (chassis off, no
motion possible) and sample, continuously:

  * how many lethal / inscribed cells the local costmap holds,
  * whether the ROBOT'S OWN CELL is at or above inscribed cost -- the D43 condition that
    ended mission 1 as START POSE BLOCKED,
  * whether any free space exists within the rolling window at all.

Then line those samples up against what the bespoke stack DID at the same instants, which
the mission's launch log records: FREEZE at (x,y), which ladder rung ran, which goal was
abandoned.

CLAIM BOUNDARY -- read before quoting any number this produces:

  PROVEN     : what the stock costmap CONTAINS given the recorded sensor inputs, and
               therefore what stock's collision-checked recoveries would have had to
               reason about. Component decisions on recorded inputs.
  NOT PROVEN : what the mission would have DONE. A replay is open-loop. The moment stock
               chose differently the robot would have moved differently and seen different
               scans, so nothing here forecasts an outcome. Anyone who reads these numbers
               as "stock would have escaped" has over-read them.

Run on the Pi with the stock middle up in replay mode:

    ros2 launch sphero_rvr_driver bringup_stationary_test.launch.py \
         start_lidar:=false static_odom:=false use_sim_time:=true
    python3 diagnostics/replay_stock_costmap_probe.py --out ~/replay_probe.csv &
    ros2 bag play ~/bag_20260816_171333 --clock
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time

import rclpy
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import Buffer, TransformListener

#: Nav2 cost values. 254 = lethal, 253 = inscribed (a footprint centred here collides).
LETHAL = 254
INSCRIBED = 253


class Probe(Node):
    def __init__(self, out_path: str, period_s: float):
        super().__init__("replay_stock_costmap_probe")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.costmap: Costmap | None = None
        self.rows: list[dict] = []
        self.out_path = out_path

        transient = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Costmap, "/local_costmap/costmap_raw", self._on_costmap, transient
        )
        self.create_timer(period_s, self._sample)

    def _on_costmap(self, msg: Costmap) -> None:
        self.costmap = msg

    def _robot_xy(self):
        try:
            tf = self.buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _sample(self) -> None:
        grid = self.costmap
        if grid is None:
            return
        pose = self._robot_xy()
        if pose is None:
            return

        meta = grid.metadata
        width, height, res = meta.size_x, meta.size_y, meta.resolution
        ox, oy = meta.origin.position.x, meta.origin.position.y

        lethal = inscribed = free = unknown = 0
        for value in grid.data:
            if value >= LETHAL:
                lethal += 1
            elif value >= INSCRIBED:
                inscribed += 1
            elif value == 255:
                unknown += 1
            elif value == 0:
                free += 1

        col = int((pose[0] - ox) / res)
        row = int((pose[1] - oy) / res)
        own_cell = None
        if 0 <= col < width and 0 <= row < height:
            own_cell = int(grid.data[row * width + col])

        self.rows.append(
            {
                "stamp_s": round(self.get_clock().now().nanoseconds / 1e9, 3),
                "robot_x": round(pose[0], 4),
                "robot_y": round(pose[1], 4),
                "own_cell_cost": "" if own_cell is None else own_cell,
                # THE D43 CONDITION, evaluated against a stock costmap.
                "own_cell_blocked": ""
                if own_cell is None
                else int(own_cell >= INSCRIBED),
                "lethal_cells": lethal,
                "inscribed_cells": inscribed,
                "free_cells": free,
                "unknown_cells": unknown,
                "window_m": round(width * res, 2),
            }
        )

    def write(self) -> None:
        if not self.rows:
            print("no samples -- was the costmap publishing and TF resolving?")
            return
        with open(self.out_path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        blocked = sum(1 for r in self.rows if r["own_cell_blocked"] == 1)
        print(f"wrote {len(self.rows)} samples to {self.out_path}")
        print(f"samples with the robot's OWN CELL at/above inscribed: {blocked}")
        print(
            "max lethal cells in one sample: "
            f"{max(r['lethal_cells'] for r in self.rows)}"
        )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="replay_probe.csv")
    parser.add_argument("--period-s", type=float, default=0.25)
    parser.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    args = parser.parse_args(argv)

    rclpy.init()
    node = Probe(args.out, args.period_s)
    started = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.seconds and time.time() - started > args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.write()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
