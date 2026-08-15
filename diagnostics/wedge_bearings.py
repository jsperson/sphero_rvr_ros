"""By-bearing clearance at a recorded pose — the ONE audited tool for wedge tables.

Every "o'clock" this project prints must come from here, because on 2026-08-14 an
ad-hoc script bucketed by counter-clockwise bearing and mirrored left for right in
every label except 12 and 6. The conversion now lives in
:mod:`sphero_rvr_core.bearings`, pinned by tests/test_bearings.py against physically
known directions.

Design rules this tool follows, each bought by a past error:

* The clock label comes from ``bearing_deg_to_clock`` — never re-derived here.
* Every table prints the RAW BEARING alongside the label, so a reader can re-check
  the convention without trusting it.
* ``base_link <- laser`` is read from the bag's own ``/tf_static``, never assumed;
  the data have only ever shown a bearing OFFSET, not a mounting fact.
* Poses are taken from the bag at the requested stamps, and the odom yaw is printed
  at each, because a robot-frame table is not transferable across a yaw change (the
  2026-08-10 attribution error).

Usage:
    python3 wedge_bearings.py <bag.mcap> <stamp> [stamp ...] [--open-m 0.30]
"""

import argparse
import math
import sys

from rclpy.serialization import deserialize_message
import rosbag2_py
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from sphero_rvr_core.bearings import SECTOR_DEG, bearing_deg_to_clock


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    scans, odoms, laser_yaw = [], [], None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/scan":
            scans.append((t * 1e-9, deserialize_message(data, LaserScan)))
        elif topic == "/odom":
            odoms.append((t * 1e-9, deserialize_message(data, Odometry)))
        elif topic == "/tf_static" and laser_yaw is None:
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.child_frame_id.endswith("laser"):
                    laser_yaw = math.degrees(_yaw(tr.transform.rotation))
    return scans, odoms, laser_yaw


def clock_table(scan, laser_yaw_deg):
    """{clock: (min_range, bearing_of_that_return)} in body frame."""
    best = {}
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        bearing = math.degrees(scan.angle_min + i * scan.angle_increment) + laser_yaw_deg
        bearing = (bearing + 180.0) % 360.0 - 180.0
        clock = bearing_deg_to_clock(bearing)
        if clock not in best or r < best[clock][0]:
            best[clock] = (r, bearing)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("stamps", nargs="+", type=float)
    ap.add_argument("--open-m", type=float, default=0.30)
    args = ap.parse_args()

    scans, odoms, laser_yaw = read_bag(args.bag)
    if not scans:
        print("NO SCANS IN BAG")
        return 1
    print(f"bag: {len(scans)} scans, {len(odoms)} odom")
    print(f"base_link<-laser yaw = {laser_yaw:+.2f} deg (from the bag's /tf_static)")
    print(f"clock sector = {SECTOR_DEG:.0f} deg, OPEN threshold = {args.open_m} m")
    print("CONVENTION: +y LEFT, clock runs CLOCKWISE -> 3 o'clock is the RIGHT side\n")

    agg = {}
    for stamp in args.stamps:
        ts, scan = min(scans, key=lambda kv: abs(kv[0] - stamp))
        line = f"=== stamp {stamp:.3f}  (scan dt {ts - stamp:+.3f} s)"
        if odoms:
            to, od = min(odoms, key=lambda kv: abs(kv[0] - stamp))
            p = od.pose.pose.position
            line += (f"  pose x={p.x:+.3f} y={p.y:+.3f} "
                     f"odom_yaw={math.degrees(_yaw(od.pose.pose.orientation)):+.1f}")
        print(line + " ===")

        table = clock_table(scan, laser_yaw)
        print("  clock  min_m   at_bearing   state")
        for clock in [12] + list(range(1, 12)):
            if clock not in table:
                print(f"  {clock:>5}    ---        ---     no-return")
                continue
            rng, bearing = table[clock]
            agg.setdefault(clock, []).append(rng)
            state = "OPEN" if rng >= args.open_m else "blocked"
            side = "LEFT" if bearing > 0.5 else ("RIGHT" if bearing < -0.5 else "centre")
            print(f"  {clock:>5}  {rng:6.3f}   {bearing:+7.1f}    {state:<7} ({side})")
        lo = min(table, key=lambda k: table[k][0])
        opens = sorted(k for k, v in table.items() if v[0] >= args.open_m)
        print(f"  MIN {table[lo][0]:.3f} m at {lo} o'clock "
              f"(bearing {table[lo][1]:+.1f} deg)")
        print(f"  OPEN {len(opens)}/12: {opens}\n")

    if len(args.stamps) > 1:
        print("=== stability across stamps (min .. max per clock) ===")
        for clock in [12] + list(range(1, 12)):
            vs = agg.get(clock, [])
            if not vs:
                continue
            allopen = all(v >= args.open_m for v in vs)
            allblocked = all(v < args.open_m for v in vs)
            verdict = ("OPEN all" if allopen
                       else "blocked all" if allblocked else "**FLIPS**")
            print(f"  {clock:>3}  {min(vs):6.3f} .. {max(vs):6.3f}   {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
