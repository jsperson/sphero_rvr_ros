"""Does the supervisor let a pivot through when it should not? No hardware, no motion.

Drives the REAL `lidar_collision_stop_supervisor` node through synthetic `/scan` and
`/camera/low_obstacles`, publishes commands on `/cmd_vel`, and reads back what reached
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
  D21  while an obstacle is latched inside the stop distance the reported state must
       stay STOPPED -- reporting SLOW cleared the latch and published CLEAR with the
       obstacle still there

THE LATCH MATTERS -- the first version of this probe got it wrong. The swept-circle
gate is the FRONT-STOP ESCAPE gate: it runs only while the supervisor is latched
STOPPED (the D17 field scenario -- front-stopped at a table leg, chair leg off the
rear quarter, explorer pivots toward open floor). A pivot commanded from CLEAR takes
a different, deliberate path: the projected-trajectory check, which samples the
rotating footprint over a short horizon and re-evaluates on every scan as the robot
actually turns. Probing a pivot from CLEAR therefore measures the trajectory checker,
not the escape gate, and reports gate failures that are not gate failures. Each lidar
case here first LATCHES a front stop (forward command against an obstacle inside the
0.30 m stop distance), then commands the pure pivot the gate must judge.

Run WITH the supervisor up and the driver down. The scan is published in the `laser`
frame, so the deployed base_link->laser static TF must also be up (without it the
node falls back to an identity transform and every synthetic bearing is misread by
~179 deg):
    ros2 run tf2_ros static_transform_publisher --x 0.0045 --y -0.011 --z 0.1905 \
        --roll 0 --pitch 0 --yaw 3.1239668018215028 \
        --frame-id base_link --child-frame-id laser
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
FRONT_STOP = 0.30       # deployed stop_distance_m: latch bait must sit inside this
N = 360


def scan_msg(stamp, bands, elsewhere=2.0):
    """Synthetic scan: `bands` is a list of (range_m, lo_deg, hi_deg) in BASE bearings."""
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
        value = elsewhere
        for dist, lo, hi in bands:
            if lo <= base_deg <= hi:
                value = min(value, dist)
        ranges.append(value)
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

    def _drive(self, bands, cloud_points, linear_x, angular_z, seconds,
               until_state=None, sample_last=0.0):
        """Publish scan/cloud/command at FIELD rates; return samples from the last
        window.

        Fixed rates (scan 10 Hz, cloud 5 Hz, cmd 20 Hz), not publish-per-callback:
        the first version published once per received message, a positive feedback
        loop that settled at ~130 Hz on every topic. At that pressure -- 6x field
        rates -- the supervisor's executor starves its camera-cloud subscription for
        >0.6 s stretches (measured: an independent observer saw every cloud arrive
        while the supervisor's veto went stale), the cloud goes stale, and the
        fail-open veto opens. That is a real overload behaviour worth knowing about,
        but it is not the question this probe asks; the gate must be judged at the
        rates the field actually produces.

        Sampled WHILE still publishing: requested_cmd_timeout_s is 0.25 s, so reading
        after the commands stop measures a zeroed command, not the gate's answer.
        `until_state` returns early once the state matches (latch/release phases).
        """
        start = time.monotonic()
        end = start + seconds
        next_scan = next_cloud = next_cmd = start
        lin, ang, states = [], [], set()
        while rclpy.ok() and time.monotonic() < end:
            tick = time.monotonic()
            now = self.get_clock().now().to_msg()
            if tick >= next_scan:
                self.scan_pub.publish(scan_msg(now, bands))
                next_scan += 0.1
            if tick >= next_cloud:
                self.cloud_pub.publish(cloud_msg(now, cloud_points))
                next_cloud += 0.2
            if tick >= next_cmd and (linear_x or angular_z):
                t = Twist()
                t.linear.x = linear_x
                t.angular.z = angular_z
                self.cmd_pub.publish(t)
                next_cmd += 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
            if until_state is not None and self.state == until_state:
                return [], [], {self.state}
            if self.out is not None and time.monotonic() > end - sample_last:
                lin.append(abs(self.out[0]))
                ang.append(abs(self.out[1]))
                states.add(self.state)
        return lin, ang, states

    def release(self):
        """Open floor, no commands, until the latch auto-releases to CLEAR."""
        _, _, states = self._drive([], [], 0.0, 0.0, 4.0, until_state="CLEAR")
        return "CLEAR" in states

    def latch(self, bands, cloud_points):
        """Drive forward at the obstacle until the front stop latches."""
        _, _, states = self._drive(bands, cloud_points, 0.15, 0.0, 3.0,
                                   until_state="STOPPED")
        return "STOPPED" in states

    def pivot(self, bands, cloud_points, seconds=2.5):
        """Command a pure pivot; report (turned, forward_leaked, states) from the
        final 1 s window."""
        self.out = None
        lin, ang, states = self._drive(bands, cloud_points, 0.0, 0.4, seconds,
                                       sample_last=1.0)
        turned = bool(ang) and max(ang) > 1e-6
        leaked = bool(lin) and max(lin) > 1e-6
        return turned, leaked, states

    def reverse(self, bands, cloud_points, seconds=2.5):
        """Command a straight reverse; report (moved, states) over the WHOLE phase.

        The reverse escape's SLOW is transient: the first reverse decision routes
        through the escape checks and reports SLOW, which dissolves the latch, and
        the following decisions take the normal path (front checks gate forward
        only) and read CLEAR while backing away. Sampling only the tail misses the
        escape's signature entirely.
        """
        self.out = None
        lin, _, states = self._drive(bands, cloud_points, -0.10, 0.0, seconds,
                                     sample_last=seconds)
        moved = bool(lin) and max(lin) > 1e-6
        return moved, states


def main():
    rclpy.init()
    n = Probe()
    time.sleep(1.0)
    print(f"swept corner radius = {SWEPT:.4f} m   front stop = {FRONT_STOP:.2f} m\n")
    # (label, latch?, scan bands, cloud, expect_turn, allowed pivot states, defect)
    # Latched cases: the state while pivoting must stay STOPPED -- SLOW or CLEAR with
    # the obstacle still inside the stop distance is D21's lying telemetry.
    cases = [
        ("pivot from CLEAR, open floor",
         False, [], [], True, {"CLEAR"}, "-"),
        ("latched, front 0.25 (outside swept)",
         True, [(0.25, -10, 10)], [], True, {"STOPPED"}, "D21"),
        ("latched, front 0.20 (inside swept)",
         True, [(0.20, -10, 10)], [], False, {"STOPPED"}, "D18"),
        ("latched, front 0.181 (inside swept)",
         True, [(0.181, -10, 10)], [], False, {"STOPPED"}, "D18"),
        ("latched 0.28 + 0.09 at 137..149",
         True, [(0.28, -10, 10), (0.09, 137, 149)], [], False, {"STOPPED"}, "D17"),
        ("latched 0.28 + 0.09 at -45..-30",
         True, [(0.28, -10, 10), (0.09, -45, -30)], [], False, {"STOPPED"}, "D17"),
        ("camera 0.15 m left, from CLEAR",
         False, [], [(0.0, 0.15, 0.02)], False, {"CLEAR"}, "D19"),
        ("camera 0.40 m left, from CLEAR",
         False, [], [(0.0, 0.40, 0.02)], True, {"CLEAR"}, "-"),
        ("latched 0.25 + camera 0.15 m left",
         True, [(0.25, -10, 10)], [(0.0, 0.15, 0.02)], False, {"STOPPED"}, "D19"),
    ]
    fails = 0
    for label, needs_latch, bands, cloud, expect_turn, ok_states, defect in cases:
        if not n.release():
            print(f"**FAIL**  {label:38s} could not reach CLEAR between cases")
            fails += 1
            continue
        if needs_latch and not n.latch(bands, cloud):
            print(f"**FAIL**  {label:38s} front stop never latched (inconclusive)")
            fails += 1
            continue
        turned, leaked, states = n.pivot(bands, cloud)
        if not states:
            # No motor output sampled at all. turned=False would read as a PASS on
            # every refusal case, vacuously -- a dead supervisor must not grade as a
            # safe one.
            print(f"**FAIL**  {label:38s} no /cmd_vel_motor output sampled "
                  f"(inconclusive) [{defect}]")
            fails += 1
            continue
        ok = (turned == expect_turn) and not leaked and states <= ok_states
        fails += (not ok)
        verdict = "PASS" if ok else "**FAIL**"
        print(f"{verdict}  {label:38s} turn={'yes' if turned else 'no ':3s} "
              f"expected={'yes' if expect_turn else 'no'}  leak={'yes' if leaked else 'no'}"
              f"  states={sorted(states)} [{defect}]")
    # D21's second clause: the latch must SURVIVE a granted escape turn, because the
    # reverse-escape guard requires it -- the old SLOW report cleared the latch and
    # silently disabled that guard after any turn. Measure it comparatively: a
    # latch-then-reverse with no turn is the baseline; a latch-turn-reverse must
    # behave identically (motion flows, and the state walks the same
    # STOPPED -> SLOW escape -> CLEAR trajectory). A missing SLOW would mean the
    # escape checks were bypassed; a persistent STOPPED would mean reverse is
    # refused after a turn.
    label = "post-turn reverse escape (latch survives)"
    bands = [(0.25, -10, 10)]
    ok = n.release() and n.latch(bands, [])
    base_moved, base_states = (n.reverse(bands, []) if ok else (False, set()))
    ok = ok and base_moved and n.release() and n.latch(bands, [])
    if ok:
        turned, _, states = n.pivot(bands, [])
        ok = turned and states <= {"STOPPED"}
    if ok:
        moved, states = n.reverse(bands, [])
        # Both runs must show the escape's SLOW and the subsequent CLEAR. Exact
        # set equality would flake on a transient sampled in one run and not the
        # other; the load-bearing claim is only that the turn did not remove the
        # escape path.
        need = {"SLOW", "CLEAR"}
        ok = moved and need <= states and need <= base_states
        print(f"{'PASS' if ok else '**FAIL**'}  {label:38s} reverse={'yes' if moved else 'no '}"
              f" states={sorted(states)} baseline={sorted(base_states)} [D21]")
    else:
        print(f"**FAIL**  {label:38s} setup (latch, baseline or turn) failed [D21]")
    fails += (not ok)
    print(f"\n=== {'ALL PASS' if not fails else str(fails) + ' FAILURE(S)'} ===")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


main()
