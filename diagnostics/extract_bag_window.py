"""Cut a brake-release window out of a mission bag into a replayable JSON fixture.

    python3 diagnostics/extract_bag_window.py <bag_dir> <out.json>

RUN IT ON THE PI. It needs `rosbag2_py` and the message packages, which the robot has
and the Mac does not; that asymmetry is the whole reason fixtures are extracted there
and replayed here.

FOUND BY SIGNATURE, NOT BY CLOCK. A recorder CSV's t=0 is not the bag's -- they have
been 53 s apart -- so the window is located by what the data itself does: the frame
where `cam_scale` leaves 0.00 while `cam_nearest` jumps OUTWARD. That conjunction is
the D39 release and nothing else in a run looks like it. Every timestamp written out
is relative to that frame, so the fixture carries no borrowed clock at all.

Pairs `/tof/obstacles`, `/collision_stop/state` and `/odom` FROM THE SAME BAG, so a
replay needs no cross-topic inference and no clock fit -- the method autopsy #2 used
after a relayed timestamp sent it to the wrong part of the wrong recording.

The odom is what lets a replay transport a belief by MEASURED motion. Without it a
test would have to assume the rover held still, which is exactly the kind of unstated
assumption that makes a green test worthless.
"""
import json
import re
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

if len(sys.argv) < 3:
    raise SystemExit(__doc__.strip().splitlines()[2].strip())
BAG, OUT = sys.argv[1], sys.argv[2]
WANT = ("/tof/obstacles", "/collision_stop/state", "/odom")


def reader():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    return r, types


def num(text, key):
    m = re.search(rf"\b{key}=([-\d.]+|None)", text)
    if not m or m.group(1) == "None":
        return None
    return float(m.group(1))


def main():
    r, types = reader()
    r.set_filter(rosbag2_py.StorageFilter(topics=list(WANT)))
    states, clouds, odom = [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == "/collision_stop/state":
            states.append((t, msg.data))
            continue
        if topic == "/odom":
            q = msg.pose.pose.orientation
            import math as _m
            yaw = _m.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            odom.append((t, round(msg.pose.pose.position.x, 6),
                         round(msg.pose.pose.position.y, 6), round(yaw, 6)))
            continue
        if True:
            pts = [(round(float(p[0]), 5), round(float(p[1]), 5))
                   for p in point_cloud2.read_points(msg, field_names=("x", "y"),
                                                     skip_nans=True)]
            clouds.append((t, pts))

    # THE RELEASE: cam_scale rises off 0.00 while cam_nearest moves OUTWARD. That
    # conjunction is the defect's signature and nothing else in the run looks like it.
    release = None
    for i in range(1, len(states)):
        p_s, c_s = num(states[i - 1][1], "cam_scale"), num(states[i][1], "cam_scale")
        p_n, c_n = num(states[i - 1][1], "cam_nearest"), num(states[i][1], "cam_nearest")
        if p_s == 0.0 and c_s and c_s > 0.0 and p_n and c_n and c_n > p_n and p_n < 0.25:
            release = i
            print(f"release at index {i}: scale {p_s}->{c_s}, nearest {p_n}->{c_n}")
            break
    if release is None:
        raise SystemExit("no release signature found -- do not guess a window")

    t0 = states[release][0]
    lo, hi = t0 - 5_000_000_000, t0 + 4_000_000_000     # ns
    rows = [{"t_rel_s": round((t - t0) / 1e9, 3),
             "cam_nearest": num(d, "cam_nearest"), "cam_scale": num(d, "cam_scale"),
             "cam_considered": num(d, "cam_considered"),
             "state": d.split(" ", 1)[0]}
            for t, d in states if lo <= t <= hi]
    frames = [{"t_rel_s": round((t - t0) / 1e9, 3), "points_xy": pts}
              for t, pts in clouds if lo <= t <= hi]

    poses = [{"t_rel_s": round((t - t0) / 1e9, 3), "x": x, "y": y, "yaw": yaw}
             for t, x, y, yaw in odom if lo <= t <= hi]

    json.dump({
        "source_bag": BAG,
        "found_by": "release signature (cam_scale 0.00 -> >0 with cam_nearest moving outward)",
        "release_t_rel_s": 0.0,
        "note": ("t_rel_s is relative to the RELEASE FRAME, not to any recorder clock. "
                 "Clouds and states are paired from the same bag; no clock fit."),
        "states": rows, "clouds": frames, "odom": poses,
    }, open(OUT, "w"), indent=1)
    print(f"wrote {OUT}: {len(rows)} states, {len(frames)} clouds, {len(poses)} odom")


main()
