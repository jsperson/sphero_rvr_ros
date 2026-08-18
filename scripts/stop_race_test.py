#!/usr/bin/env python3
"""The stop-race rig: measure zero->wire latency and the stale-flap under pivot load.

Run against a live sim_closed_loop stack (chassis-off, sim port):

    python3 scripts/make_open_rig_map.py --out-dir /tmp
    ros2 launch sphero_rvr_driver sim_closed_loop.launch.py map_yaml:=/tmp/open_rig_room.yaml
    python3 scripts/stop_race_test.py --trials 20

WHAT IT MEASURES (the 2026-08-18 ride-along's discovery, per trial): publish pivot
commands on /cmd_vel at the Spin behavior's shape (~10 Hz, 3.55 rad/s) for
--pivot-s seconds, then ONE zero — and clock (a) ZERO->WIRE: time from the zero's
publish to the first zero on /cmd_vel_motor after which no nonzero follows
(the lasting stop); (b) TAIL REPLAYS: any nonzero motor command later than 200 ms
after the zero; (c) FLAP COUNT: CLEAR<->SENSOR_STALE transitions on
/collision_stop/state during the pivot window.

PRE-REGISTERED DECISION RULE, stated before any trial runs (consensus, 2026-08-18
night): N = --trials (default 20) per arm. The race is PROBABILISTIC — the field
saw one 1.218 s tail and one 63 ms stop with the same code — so:

  * The >1 s tail claim is FALSIFIED-then-CERTIFIED only if the PRE-FIX arm
    reproduces at least one tail > 1.0 s in N trials. If it cannot, that leg is
    INCONCLUSIVE-NOT-PASSED and is not claimed either way.
  * The fix's DETERMINISTIC legs stand regardless: post-fix p95 zero->wire must
    collapse versus pre-fix p95, post-fix tail-replays must be ZERO in N trials,
    and the flap count during the pivot window must drop materially.

Publishing on /cmd_vel is the supervisor's own input seam — no BT, no navigation,
no chassis; the sim port absorbs the wheels.
"""

import argparse
import math
import statistics
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

PIVOT_WZ = 3.55           # the floor rate, the Spin behavior's opening ask
CMD_RATE_HZ = 10.0        # Spin's observed publish cadence (57 msgs / 5.6 s)
TAIL_GRACE_S = 0.2        # motor nonzeros later than this after the zero = replay


def say(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class Probe(Node):
    def __init__(self):
        super().__init__("stop_race_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.motor = []          # (t, wz)
        self.states = []         # (t, state_token)
        self.create_subscription(Twist, "/cmd_vel_motor", self._on_motor, 50)
        self.create_subscription(String, "/collision_stop/state", self._on_state, 50)

    def _on_motor(self, msg):
        self.motor.append((time.monotonic(), msg.angular.z))

    def _on_state(self, msg):
        tok = msg.data.split()
        self.states.append((time.monotonic(), tok[0] if tok else "?"))


def run_trial(probe, pivot_s):
    probe.motor.clear()
    probe.states.clear()
    t0 = time.monotonic()
    period = 1.0 / CMD_RATE_HZ
    msg = Twist()
    msg.angular.z = PIVOT_WZ
    while time.monotonic() - t0 < pivot_s:
        probe.pub.publish(msg)
        time.sleep(period)
    zero = Twist()
    t_zero = time.monotonic()
    probe.pub.publish(zero)
    time.sleep(3.0)                        # settle window

    flaps = 0
    prev = None
    for t, s in probe.states:
        if t0 <= t <= t_zero and s != prev:
            if prev is not None and {prev, s} == {"CLEAR", "SENSOR_STALE"}:
                flaps += 1
            prev = s
    nonzero_after = [t for t, wz in probe.motor if t > t_zero and abs(wz) > 0.02]
    zeros_after = [t for t, wz in probe.motor if t > t_zero and abs(wz) <= 0.02]
    if nonzero_after:
        lasting_zero = max(nonzero_after)        # stop is real only after the last replay
        lasting = [t for t in zeros_after if t > lasting_zero]
        stop_at = lasting[0] if lasting else None
    else:
        stop_at = zeros_after[0] if zeros_after else None
    tail = [t - t_zero for t in nonzero_after if t - t_zero > TAIL_GRACE_S]
    return {
        "zero_to_wire_s": (stop_at - t_zero) if stop_at else None,
        "tail_replays": len(tail),
        "worst_tail_s": max(tail) if tail else 0.0,
        "flaps": flaps,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--pivot-s", type=float, default=4.0)
    ap.add_argument("--label", default="unlabeled",
                    help="pre-fix / post-fix -- goes in every output line")
    args = ap.parse_args()

    rclpy.init()
    probe = Probe()
    spinner = threading.Thread(target=rclpy.spin, args=(probe,), daemon=True)
    spinner.start()
    time.sleep(2.0)

    results = []
    for i in range(args.trials):
        r = run_trial(probe, args.pivot_s)
        results.append(r)
        say(f"[{args.label}] trial {i + 1}/{args.trials}: "
            f"zero->wire {r['zero_to_wire_s'] if r['zero_to_wire_s'] is None else round(r['zero_to_wire_s'], 3)}s "
            f"tails {r['tail_replays']} (worst {r['worst_tail_s']:.3f}s) "
            f"flaps {r['flaps']}")
        time.sleep(1.0)

    lat = sorted(r["zero_to_wire_s"] for r in results if r["zero_to_wire_s"] is not None)
    tails = [r for r in results if r["tail_replays"] > 0]
    worst = max((r["worst_tail_s"] for r in results), default=0.0)
    flaps = [r["flaps"] for r in results]
    if lat:
        p95 = lat[min(len(lat) - 1, math.ceil(0.95 * len(lat)) - 1)]
        say(f"[{args.label}] SUMMARY n={len(results)}: zero->wire median "
            f"{statistics.median(lat):.3f}s p95 {p95:.3f}s max {lat[-1]:.3f}s; "
            f"trials-with-tails {len(tails)}/{len(results)} worst tail {worst:.3f}s; "
            f"flaps/trial median {statistics.median(flaps):.0f} max {max(flaps)}")
    verdict = ("YES" if worst > 1.0 else
               "NO -- per the pre-registered rule, the tail leg is "
               "INCONCLUSIVE-NOT-PASSED if this is the pre-fix arm")
    say(f"[{args.label}] tail>1.0s reproduced: {verdict}")
    if rclpy.ok():
        rclpy.shutdown()
    time.sleep(0.5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
