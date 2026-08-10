"""Does the supervisor let a pivot through when it should not? No hardware, no motion.

Drives the REAL `lidar_collision_stop_supervisor` node through synthetic `/scan` and
`/camera/low_obstacles`, publishes a pivot on `/cmd_vel`, and reads back what reached
`/cmd_vel_motor`. The driver is absent, so nothing can move even if a case fails.

Synthetic inputs rather than a placed obstacle because the whole question is geometric:
the cases that matter are a few centimetres apart, and they sit at bearings a hand
measurement cannot hit repeatably.

Covers the four pivot defects found by review on 2026-08-09:

  D18  the swept circle is the footprint's CORNER radius, hypot(max(front,rear),
       max(left,right)) + margin = 0.209 m -- not max(front,rear) + margin = 0.180 m
  D17  the gate must read EVERY bearing; front/rear/left/right leave 60 deg unread and
       the corners sit at +/-42.3 and +/-148.0 deg, squarely in those gaps
  D19  a camera point inside the circle must refuse the turn, because the lidar plane
       at 0.19 m reports the wall behind a shoe rather than the shoe
  D21  the reported state must stay STOPPED -- reporting SLOW cleared the latch and
       published CLEAR with the obstacle still there

Run WITH the supervisor up and the driver down:
    ros2 run sphero_rvr_driver lidar_collision_stop_supervisor --ros-args \
        --params-file <collision_stop.yaml>
    python3 pivot_gate_probe.py
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String

FRONT, REAR, LEFT, RIGHT, MARGIN = 0.11, 0.16, 0.10, 0.10, 0.02
SWEPT = math.hypot(max(FRONT, REAR), max(LEFT, RIGHT)) + MARGIN   # 0.2087 m
N = 360


def scan_msg(stamp, obstacle_m, lo_deg, hi_deg, elsewhere=2.0):
    m = LaserScan()
    m.header.stamp = stamp
    m.header.frame_id = "laser"
    m.angle_min = -math.pi
    m.angle_increment = (2.0 * math.pi) / N
    m.range_min, m.range_max = 0.05, 8.0
    ranges = []
    for i in range(N):
        # The laser frame is rotated ~179 deg from base_link, so a bearing quoted in
        # BASE terms has to be emitted at the corresponding laser index. Read the
        # rotation from the deployed TF rather than assuming it (see
        # scan_bearing_check.py); 179 deg is used here only to place synthetic points.
        base_deg = math.degrees(m.angle_min + i * m.angle_increment) + 178.99
        base_deg = (base_deg + 180.0) % 360.0 - 180.0
        ranges.append(obstacle_m if lo_deg <= base_deg <= hi_deg else elsewhere)
    m.ranges = ranges
    return m


def cloud_msg(stamp, points):
    m = PointCloud2()
    m.header.stamp = stamp
    m.header.frame_id = "base_link"
    m.height = 1
    m.width = len(points)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(("x", "y", "z"))]
    m.is_bigendian = False
    m.point_step = 12
    m.row_step = 12 * len(points)
    m.is_dense = True
    import struct
    m.data = b"".join(struct.pack("<fff", *p) for p in points)
    return m


class Probe(Node):
    def __init__(self):
        super().__init__("pivot_gate_probe")
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/camera/low_obstacles", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.out = None
        self.state = ""
        self.create_subscription(Twist, "/cmd_vel_motor", self._on_out, 10)
        self.create_subscription(String, "/collision_stop/state", self._on_state, 10)

    def _on_out(self, m):
        self.out = (m.linear.x, m.angular.z)

    def _on_state(self, m):
        self.state = m.data.split(" ", 1)[0]

    def run_case(self, obstacle_m, lo_deg, hi_deg, cloud_points, seconds=2.5):
        """Hold the inputs steady and keep commanding a pivot; report what got out.

        Sampled WHILE still publishing: requested_cmd_timeout_s is 0.25 s, so reading
        after the commands stop measures a zeroed command, not the gate's answer.
        """
        self.out = None
        end = time.monotonic() + seconds
        angs = []
        while rclpy.ok() and time.monotonic() < end:
            now = self.get_clock().now().to_msg()
            self.scan_pub.publish(scan_msg(now, obstacle_m, lo_deg, hi_deg))
            self.cloud_pub.publish(cloud_msg(now, cloud_points))
            t = Twist()
            t.angular.z = 0.4          # pure pivot: no forward component at all
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.out is not None and time.monotonic() > end - 1.0:
                angs.append(abs(self.out[1]))
        turned = bool(angs) and max(angs) > 1e-6
        return turned, self.state


def main():
    rclpy.init()
    n = Probe()
    time.sleep(1.0)
    print(f"swept corner radius = {SWEPT:.4f} m\n")
    cases = [
        # (label, obstacle_m, lo_deg, hi_deg, cloud, expect_turn, defect)
        ("open floor all round",            2.00, 999, 999, [], True,  "-"),
        ("lidar 0.25 m ahead (outside)",    0.25, -10, 10,  [], True,  "-"),
        ("lidar 0.20 m ahead (inside)",     0.20, -10, 10,  [], False, "D18"),
        ("lidar 0.181 m ahead (inside)",    0.181, -10, 10, [], False, "D18"),
        ("lidar 0.09 m at 137-149 (corner)", 0.09, 137, 149, [], False, "D17"),
        ("lidar 0.09 m at -45..-30 (gap)",  0.09, -45, -30, [], False, "D17"),
        ("camera point 0.15 m left",        2.00, 999, 999,
         [(0.0, 0.15, 0.02)], False, "D19"),
        ("camera point 0.40 m left (clear)", 2.00, 999, 999,
         [(0.0, 0.40, 0.02)], True,  "-"),
    ]
    fails = 0
    for label, dist, lo, hi, cloud, expect, defect in cases:
        turned, state = n.run_case(dist, lo, hi, cloud)
        ok = (turned == expect)
        fails += (not ok)
        verdict = "PASS" if ok else "**FAIL**"
        print(f"{verdict}  {label:34s} turn={'yes' if turned else 'no ':3s} "
              f"expected={'yes' if expect else 'no'}  state={state:8s} [{defect}]")
    print()
    # D21: with an obstacle inside the stop distance the state must never read CLEAR,
    # turn or no turn.
    turned, state = n.run_case(0.20, -10, 10, [])
    d21 = state != "CLEAR"
    fails += (not d21)
    print(f"{'PASS' if d21 else '**FAIL**'}  D21 state with obstacle at 0.20 m: "
          f"{state} (must not be CLEAR)")
    print(f"\n=== {'ALL PASS' if not fails else str(fails) + ' FAILURE(S)'} ===")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


main()
