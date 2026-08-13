"""Instrument one narrow-gap crossing so two arms can be compared on numbers.

The 2026-08-08 regression ("stopping in the narrowest point", "fighting itself") has
two candidate sources, both fed by the same camera cloud: the collision brake and the
costmap obstacle layer. Telling them apart needs the SAME maneuver measured twice --
Arm A with the brake on, Arm B with the supervisor restarted `-p
low_obstacle_brake_enable:=false` (marks still in the costmap, brake inert).

Usage (on the Pi, motion stack up, ATTENDED):
    python3 gap_run_capture.py A --forward 2.0            # goal 2.0 m straight ahead
    python3 gap_run_capture.py B --forward 2.0
    python3 gap_run_capture.py A --forward 2.0 --left 0.3 --timeout 90

The goal is expressed RELATIVE to the pose at t0 and converted to the map frame, so
the same command means the same physical maneuver in both arms even though a restart
rebuilds the SLAM map. Stage the rover on the same physical spot for both.

Writes ~/gap_run_<label>.jsonl (one row per supervisor state message, ~10 Hz) and
prints a summary. `--compare A B` re-reads two captures and diffs them.

What the summary answers:
  * did the CAMERA brake throttle (cam_scale < 1) and for how long -- Arm A only if
    the brake is the culprit;
  * did the LIDAR brake throttle (reason=SLOW/STOP) -- present in both arms if the
    costmap/planner is squeezing the rover into walls;
  * stalls with a live request (output linear 0 while requested > 0) = the "stopping
    in the narrowest point";
  * command sign reversals and angular churn = "fighting itself";
  * path length vs straight line, and whether the goal was reached.

Reads only; publishes nothing except the nav goal. The supervisor keeps full veto.
"""
import argparse
import json
import math
import os
import sys
import time


def summarize(rows, meta):
    """Reduce a capture to the handful of numbers the A/B turns on."""
    if not rows:
        return {"rows": 0}
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    dt = [rows[i + 1]["t"] - rows[i]["t"] for i in range(len(rows) - 1)] + [0.0]

    def secs(pred):
        return round(sum(d for r, d in zip(rows, dt) if pred(r)), 2)

    wants = lambda r: r["req_lin"] > 0.005  # noqa: E731  (a live forward request)
    cam_throttled = lambda r: r["cam_scale"] is not None and r["cam_scale"] < 0.995  # noqa: E731
    lidar_throttled = lambda r: r["reason"] in ("SLOW", "STOP") or r["state"] in ("SLOW", "STOP")  # noqa: E731
    stalled = lambda r: wants(r) and abs(r["out_lin"]) < 0.005  # noqa: E731

    path = 0.0
    for i in range(len(rows) - 1):
        path += math.dist((rows[i]["x"], rows[i]["y"]), (rows[i + 1]["x"], rows[i + 1]["y"]))
    net = math.dist((rows[0]["x"], rows[0]["y"]), (rows[-1]["x"], rows[-1]["y"]))

    reversals = 0
    prev = 0
    for r in rows:
        s = (r["out_lin"] > 0.01) - (r["out_lin"] < -0.01)
        if s and prev and s != prev:
            reversals += 1
        if s:
            prev = s

    cams = [r["cam_scale"] for r in rows if r["cam_scale"] is not None]
    near = [r["cam_nearest"] for r in rows if r["cam_nearest"] is not None]
    marks = [r["marks"] for r in rows]
    return {
        "label": meta.get("label"),
        "result": meta.get("result"),
        "duration_s": round(t1 - t0, 2),
        "requesting_s": secs(wants),
        "path_m": round(path, 3),
        "net_m": round(net, 3),
        "wander_ratio": round(path / net, 2) if net > 0.05 else None,
        "stalled_s": secs(stalled),
        "stall_fraction": round(secs(stalled) / max(secs(wants), 1e-6), 3),
        "camera_throttled_s": secs(cam_throttled),
        "camera_min_scale": round(min(cams), 3) if cams else None,
        "camera_nearest_min_m": round(min(near), 3) if near else None,
        "lidar_throttled_s": secs(lidar_throttled),
        "lidar_front_min_m": round(min(r["front"] for r in rows if r["front"] is not None), 3),
        "direction_reversals": reversals,
        "angular_abs_mean": round(sum(abs(r["out_ang"]) for r in rows) / len(rows), 3),
        "camera_marks_mean": round(sum(marks) / len(marks), 1),
        "camera_marks_max": max(marks),
    }


def compare(a, b):
    ra = [json.loads(x) for x in open(os.path.expanduser(f"~/gap_run_{a}.jsonl"))]
    rb = [json.loads(x) for x in open(os.path.expanduser(f"~/gap_run_{b}.jsonl"))]
    ma, mb = ra[0], rb[0]
    sa, sb = summarize(ra[1:], ma), summarize(rb[1:], mb)
    keys = [k for k in sa if k not in ("label",)]
    w = max(len(k) for k in keys)
    print(f"{'metric':<{w}}  {a:>12}  {b:>12}")
    for k in keys:
        print(f"{k:<{w}}  {str(sa.get(k)):>12}  {str(sb.get(k)):>12}")
    print(
        "\nRead: camera_throttled_s ~0 in the brake-off arm is the control check. "
        "If stalled_s / direction_reversals stay high there too, the brake is NOT the "
        "cause -- pull camera_low from lean_nav2.yaml and repeat."
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label")
    ap.add_argument("--forward", type=float, default=2.0, help="goal metres ahead of the start pose")
    ap.add_argument("--left", type=float, default=0.0, help="goal metres left of the start pose")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--no-goal", action="store_true", help="record only; someone else drives")
    ap.add_argument("--compare", nargs=2, metavar=("A", "B"))
    args = ap.parse_args()

    if args.compare:
        compare(*args.compare)
        return

    import rclpy
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
    from std_msgs.msg import String
    import tf2_ros

    CLEAR_RANGE = 1.8  # low_obstacle clear_range_m; those endpoints are not marks

    rclpy.init()
    n = rclpy.create_node("gap_run_capture")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    st = {"state": "", "marks": 0, "nearest_mark": None, "req": (0.0, 0.0)}

    def on_cloud(msg):
        marks = 0
        nearest = None
        for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            r = math.hypot(float(p[0]), float(p[1]))
            if abs(r - CLEAR_RANGE) < 0.01:
                continue  # clear-ray endpoint, not an obstacle
            marks += 1
            if nearest is None or r < nearest:
                nearest = r
        st["marks"], st["nearest_mark"] = marks, nearest

    n.create_subscription(String, "/collision_stop/state", lambda m: st.__setitem__("state", m.data), 10)
    n.create_subscription(PointCloud2, "/camera/low_obstacles", on_cloud, 5)
    n.create_subscription(Twist, "/cmd_vel", lambda m: st.__setitem__("req", (m.linear.x, m.angular.z)), 10)

    def spin(s):
        end = time.monotonic() + s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    def field(key, cast=float):
        for tok in st["state"].split():
            if tok.startswith(key + "="):
                v = tok.split("=", 1)[1]
                if v in ("", "None"):
                    return None
                try:
                    return cast(v)
                except ValueError:
                    return None
        return None

    def pair(key):
        for tok in st["state"].split():
            if tok.startswith(key + "=("):
                a, b = tok[len(key) + 2 : -1].split(",")
                return float(a), float(b)
        return (0.0, 0.0)

    def pose():
        try:
            t = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
            q = t.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.translation.x, t.translation.y, yaw
        except Exception:
            return None

    spin(3.0)
    p0 = pose()
    if p0 is None:
        sys.exit("no map->base_link TF; is SLAM up?")
    x0, y0, yaw0 = p0
    gx = x0 + args.forward * math.cos(yaw0) - args.left * math.sin(yaw0)
    gy = y0 + args.forward * math.sin(yaw0) + args.left * math.cos(yaw0)
    print(f"start ({x0:.2f},{y0:.2f}) yaw {math.degrees(yaw0):.0f} deg -> goal ({gx:.2f},{gy:.2f})")

    result = "not-sent"
    goal_future = None
    if not args.no_goal:
        ac = ActionClient(n, NavigateToPose, "navigate_to_pose")
        if not ac.wait_for_server(timeout_sec=10.0):
            sys.exit("navigate_to_pose action server not available")
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw0 / 2), math.cos(yaw0 / 2)
        goal_future = ac.send_goal_async(g)
        result = "running"

    path = os.path.expanduser(f"~/gap_run_{args.label}.jsonl")
    out = open(path, "w")
    meta = {"label": args.label, "start": [x0, y0, yaw0], "goal": [gx, gy], "result": result}
    out.write(json.dumps(meta) + "\n")

    t_start = time.monotonic()
    handle = None
    res_future = None
    seen = ""
    print("recording... (Ctrl-C to stop early)")
    try:
        while rclpy.ok() and time.monotonic() - t_start < args.timeout:
            spin(0.1)
            if goal_future is not None and goal_future.done() and handle is None:
                handle = goal_future.result()
                if not handle.accepted:
                    meta["result"] = "REJECTED"
                    break
                res_future = handle.get_result_async()
            if res_future is not None and res_future.done():
                # action_msgs/GoalStatus: 4 SUCCEEDED, 5 CANCELED, 6 ABORTED.
                meta["result"] = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(
                    res_future.result().status, f"status{res_future.result().status}"
                )
                break
            p = pose()
            if p is None or not st["state"]:
                continue
            req_lin, req_ang = pair("requested")
            out_lin, out_ang = pair("output")
            tok = st["state"].split()
            row = {
                "t": round(time.monotonic() - t_start, 3),
                "state": tok[0] if tok else "",
                "reason": field("reason", str),
                "x": round(p[0], 4), "y": round(p[1], 4), "yaw": round(p[2], 4),
                "req_lin": req_lin, "req_ang": req_ang,
                "out_lin": out_lin, "out_ang": out_ang,
                "front": field("front"),
                "cam_nearest": field("cam_nearest"),
                "cam_scale": field("cam_scale"),
                "marks": st["marks"],
                "nearest_mark": st["nearest_mark"],
            }
            out.write(json.dumps(row) + "\n")
            if row["state"] != seen:
                seen = row["state"]
                print(f"  t={row['t']:6.1f} {seen:<6} reason={row['reason']} front={row['front']} "
                      f"cam={row['cam_nearest']}/{row['cam_scale']} out={out_lin:.2f}")
    except KeyboardInterrupt:
        meta["result"] = "interrupted"
    finally:
        if meta["result"] == "running" and handle is not None:
            handle.cancel_goal_async()
            meta["result"] = "TIMEOUT"
        out.close()

    rows = [json.loads(x) for x in open(path)][1:]
    # Rewrite the header with the final result so summaries are self-contained.
    body = open(path).read().split("\n", 1)[1]
    with open(path, "w") as f:
        f.write(json.dumps(meta) + "\n" + body)
    print(f"\nwrote {path}")
    print(json.dumps(summarize(rows, meta), indent=2))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
