#!/usr/bin/env python3
"""Was every /cmd_vel burst OWNED by a live navigation goal? The orphan tripwire.

Part of the archive protocol (run it over each flight's bag + launch log before the
set is filed). It exists because of 2026-08-18 run 3c, goal 3: bt_navigator's 20 ms
ack budget expired, the mission ABORTED — and controller_server, which had accepted
the goal server-side, drove the robot for 2.8 s afterwards with no owner. The config
fix (bt_navigator.default_server_timeout) removes that birth condition; this check
is the detector for the residual class the fix does not cover (e.g. a bt_navigator
death mid-goal leaving the controller to finish an ownerless path). It has NO
runtime authority — it reads recordings and complains, which is the proportionate
response to an accepted, named residual.

    python3 diagnostics/check_unowned_motion.py <bag.mcap> <launch.log> [--grace 1.0]

Exit 0: every burst inside a goal window (+grace). Exit 1: unowned motion, printed
with timestamps. Needs `mcap` + `mcap-ros2-support` (analysis-side; see the
read-bags-on-the-Mac note — this is deliberately NOT a robot-runtime tool).

Both timestamp streams are the same host's ROS clock (the launch log's bracketed
epochs and the bag's message stamps), so pairing them is not the CSV-alignment trap
— no fitting, no offset, same clock.
"""

from __future__ import annotations

import argparse
import re
import sys

#: bt_navigator's own words for a goal's life. "Begin navigating" opens a window;
#: any of the enders closes it. The behavior_server's recovery motion happens INSIDE
#: a navigate_to_pose window, so goal windows cover it.
GOAL_OPEN = re.compile(r"\[(\d+\.\d+)\].*\[bt_navigator\]: Begin navigating")
GOAL_CLOSE = re.compile(
    r"\[(\d+\.\d+)\].*\[bt_navigator\]: Goal (succeeded|failed|canceled|was canceled)"
)


def goal_windows(log_text: str) -> list[tuple[float, float]]:
    """[(open_epoch, close_epoch)] from a launch log, in order. An unclosed window
    (log truncated mid-goal) extends to +inf rather than silently vanishing."""
    events: list[tuple[float, str]] = []
    for line in log_text.splitlines():
        m = GOAL_OPEN.search(line)
        if m:
            events.append((float(m.group(1)), "open"))
            continue
        m = GOAL_CLOSE.search(line)
        if m:
            events.append((float(m.group(1)), "close"))
    windows: list[tuple[float, float]] = []
    open_at: float | None = None
    for stamp, kind in events:
        if kind == "open":
            if open_at is None:
                open_at = stamp
        elif open_at is not None:
            windows.append((open_at, stamp))
            open_at = None
    if open_at is not None:
        windows.append((open_at, float("inf")))
    return windows


def unowned_bursts(
    cmd_times: list[float],
    windows: list[tuple[float, float]],
    grace_s: float = 1.0,
    burst_gap_s: float = 1.5,
) -> list[tuple[float, float, int]]:
    """Nonzero-command times outside every window (+/- grace), clustered into
    (first, last, count) bursts. Grace absorbs the ordinary straggle of a command
    already in flight when the goal ends — 3d's marker fix uses the same shape of
    argument: bounded tolerance, stated, not silence."""
    outside = [
        t for t in cmd_times
        if not any(a - grace_s <= t <= b + grace_s for a, b in windows)
    ]
    bursts: list[tuple[float, float, int]] = []
    for t in sorted(outside):
        if bursts and t - bursts[-1][1] <= burst_gap_s:
            first, _, n = bursts[-1]
            bursts[-1] = (first, t, n + 1)
        else:
            bursts.append((t, t, 1))
    return bursts


def _cmd_times_from_bag(bag_path: str, topic: str) -> list[float]:
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError:
        sys.exit(
            "needs mcap + mcap-ros2-support (analysis venv) -- this is an "
            "archive-side tool, not a robot-runtime one"
        )
    times = []
    for m in read_ros2_messages(bag_path, topics=[topic]):
        r = m.ros_msg
        if abs(r.linear.x) > 0.005 or abs(r.angular.z) > 0.02:
            times.append(m.log_time_ns * 1e-9)
    return times


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag")
    ap.add_argument("launch_log")
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--grace", type=float, default=1.0)
    args = ap.parse_args()

    windows = goal_windows(open(args.launch_log).read())
    cmd_times = _cmd_times_from_bag(args.bag, args.topic)
    bursts = unowned_bursts(cmd_times, windows, grace_s=args.grace)

    print(f"goal windows: {len(windows)}; nonzero {args.topic} msgs: {len(cmd_times)}")
    for a, b in windows:
        print(f"  window {a:.1f} -> {b if b != float('inf') else 'EOF'}")
    if not bursts:
        print("OWNED: every command burst sits inside a goal window (+grace)")
        return 0
    print(f"UNOWNED MOTION: {len(bursts)} burst(s) outside every goal window:")
    for first, last, n in bursts:
        print(f"  {first:.2f} -> {last:.2f}  ({n} msgs, {last - first:.1f} s)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
