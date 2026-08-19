#!/usr/bin/env python3
"""The explore-on-stock mission runner: arms, watches, scores the RATIFIED bars.

    python3 rig_mission.py --arm f1     # falsifier: arming withheld, bars must FAIL
    python3 rig_mission.py --arm f2     # falsifier: blocked start, honest end required
    python3 rig_mission.py --arm cert   # the certifying autonomous mission

Runs BESIDE an already-up sim_closed_loop rig (start_coverage_explorer:=true); it
launches nothing and publishes no velocity — it arms, samples, and judges. The bars
and the per-arm predictions below were ratified by the PM round of 2026-08-18 BEFORE
this file existed; changing either is a bar change and goes back to that round in
daylight, never edited quietly to make an arm pass (falsifier-before-certifier).

ARM-VERDICT vs PACKAGE-VERDICT (ratification rider 2): a falsifier arm is
PACKAGE-PASS precisely when the bars return FAIL as predicted. The summary prints
both, per bar, as predicted/deviated — so nobody later reads "F1: bars FAILED" as
a broken rig. Exit code speaks the PACKAGE verdict:

    0 = package PASS (every bar matched its prediction)
    1 = package FAIL (any bar deviated from prediction)
    2 = RIG-INCONCLUSIVE (the question could not be asked: no explorer status,
        rig already armed, service absent). Never a pass, never a failure.

B5's zero-marks clause (ratification rider 1) counts NON-EMPTY /contact_marks
messages: contact_marker publishes an empty cloud every republish tick as its
designed heartbeat, so "zero traffic" taken literally would fail vacuously on a
healthy quiet marker. A cloud with points in this rig — which has no contact
physics at all — is a phantom marker firing, and that is the finding the clause
exists to catch. Promotions are counted literally (every message is a firing).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

#: THE RATIFIED NUMBERS (PM round 2026-08-18). B1 30 s, B2 window per arm,
#: B3 15 m + 3 goals, F1 window 300 s, F2 window 600 s, cert budget 1800 s.
ARM_WINDOW_S = {"f1": 300.0, "f2": 600.0, "cert": 1800.0}
B1_ARM_S = 30.0
B3_PATH_M = 15.0
B3_GOALS = 3

#: Per-arm PRE-REGISTERED predictions, bar -> expected pass/fail. The package
#: verdict is "every actual == predicted". f1 never arms, so B1/B2/B3 fail and
#: no report ever exists (B4 fail); f2 arms into a blocked start, must END
#: HONESTLY (B4 pass: the report names the failure) without completing (B2 fail).
PREDICTED = {
    "f1":   {"B1": False, "B2": False, "B3": False, "B4": False, "B5": True},
    "f2":   {"B1": True,  "B2": False, "B3": False, "B4": True,  "B5": True},
    "cert": {"B1": True,  "B2": True,  "B3": True,  "B4": True,  "B5": True},
}

REPORT_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class Watch(Node):
    def __init__(self):
        super().__init__("rig_mission_runner")
        self.status = None            # latest parsed status dict
        self.report = None            # latest parsed report dict
        self.path_m = 0.0
        self.nonempty_marks = 0
        self.promotes = 0
        self._last_xy = None
        self._lock = threading.Lock()
        self.create_subscription(String, "/coverage_explorer/status",
                                 self._on_status, 10)
        self.create_subscription(String, "/coverage_explorer/report",
                                 self._on_report, REPORT_QOS)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(PointCloud2, "/contact_marks",
                                 self._on_marks, 10)
        self.create_subscription(PointCloud2, "/contact_marks/promote",
                                 lambda m: setattr(self, "promotes",
                                                   self.promotes + 1), 10)

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass                      # a malformed line is not evidence either way

    def _on_report(self, msg):
        try:
            self.report = json.loads(msg.data)
        except ValueError:
            pass

    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self._lock:
            if self._last_xy is not None:
                self.path_m += math.hypot(x - self._last_xy[0],
                                          y - self._last_xy[1])
            self._last_xy = (x, y)

    def _on_marks(self, msg):
        if msg.width > 0:
            self.nonempty_marks += 1


def spin_quietly(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def call_trigger(node, name, timeout_s=10.0):
    client = node.create_client(Trigger, name)
    if not client.wait_for_service(timeout_sec=timeout_s):
        return None
    future = client.call_async(Trigger.Request())
    t0 = time.monotonic()
    while not future.done() and time.monotonic() - t0 < timeout_s:
        time.sleep(0.1)
    return future.result() if future.done() else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--arm", required=True, choices=("f1", "f2", "cert"))
    args = ap.parse_args()
    window_s = ARM_WINDOW_S[args.arm]
    predicted = PREDICTED[args.arm]
    arm_the_mission = args.arm != "f1"

    def say(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    rclpy.init()
    watch = Watch()
    spinner = threading.Thread(target=spin_quietly, args=(watch,), daemon=True)
    spinner.start()

    def finish(code):
        rclpy.try_shutdown()
        spinner.join(timeout=3.0)
        return code

    def inconclusive(why):
        print(f"RIG-INCONCLUSIVE: {why} -- the question could not be asked; "
              f"this is neither a pass nor a failure.", flush=True)
        return finish(2)

    say(f"arm={args.arm} window={window_s:.0f}s predictions={predicted}")

    # The rig must be up and DISARMED before any arm means anything.
    t0 = time.monotonic()
    while watch.status is None and time.monotonic() - t0 < 30.0:
        time.sleep(0.2)
    if watch.status is None:
        return inconclusive("no /coverage_explorer/status within 30 s (rig down?)")
    if watch.status.get("armed"):
        return inconclusive("explorer ALREADY ARMED at runner start -- dirty rig; "
                            "tear down and re-launch before judging anything")

    # ---- B1: arming ------------------------------------------------------------
    b1 = False
    t_armed = None
    if arm_the_mission:
        say("calling /coverage_explorer/mission/start ...")
        result = call_trigger(watch, "/coverage_explorer/mission/start")
        if result is None:
            return inconclusive("mission/start service absent or never answered")
        t_call = time.monotonic()
        while time.monotonic() - t_call < B1_ARM_S:
            if watch.status and watch.status.get("armed"):
                b1 = True
                t_armed = time.monotonic()
                break
            time.sleep(0.5)
        say(f"B1 armed={b1} (service success={result.success})")
    else:
        say("B1: arming DELIBERATELY WITHHELD (f1's whole point)")

    # ---- the watch window --------------------------------------------------------
    # Hard-capped: this loop cannot hang. An arm that would hang is a FAIL of the
    # package (pre-registered for f2), and the cap is what turns a hang into data.
    t_start = t_armed or time.monotonic()
    done_seen = False
    while time.monotonic() - t_start < window_s:
        s = watch.status or {}
        if s.get("done"):
            done_seen = True
            say(f"done=true after {time.monotonic() - t_start:.0f}s")
            break
        time.sleep(1.0)
    if not done_seen:
        say(f"window expired ({window_s:.0f}s) without done=true")
        if arm_the_mission:
            # leave the rig quiescent for teardown; a goal in flight after the
            # verdict is a leaked mission, not evidence
            call_trigger(watch, "/coverage_explorer/mission/stop", timeout_s=5.0)
    time.sleep(3.0)   # let the terminal report land (TRANSIENT_LOCAL, but be kind)

    # ---- scoring -------------------------------------------------------------
    s = watch.status or {}
    goals_ok = int(s.get("goals_succeeded", 0))
    b2 = done_seen
    b3 = watch.path_m >= B3_PATH_M and goals_ok >= B3_GOALS
    report = watch.report
    if report is None:
        b4 = False
    elif done_seen:
        b4 = bool(report.get("complete"))
    else:
        b4 = report.get("outcome") not in (None, "COMPLETE")
    b5 = watch.nonempty_marks == 0 and watch.promotes == 0

    actual = {"B1": b1, "B2": b2, "B3": b3, "B4": b4, "B5": b5}
    detail = {
        "path_m": round(watch.path_m, 2), "goals_succeeded": goals_ok,
        "goals_sent": int(s.get("goals_sent", 0)),
        "report_outcome": (report or {}).get("outcome"),
        "nonempty_marks": watch.nonempty_marks, "promotes": watch.promotes,
    }
    print(f"\nARM {args.arm}: bar-by-bar (arm-verdict vs prediction):", flush=True)
    package_pass = True
    for bar in ("B1", "B2", "B3", "B4", "B5"):
        match = actual[bar] == predicted[bar]
        package_pass = package_pass and match
        print(f"  {bar}: actual={'PASS' if actual[bar] else 'FAIL':4} "
              f"predicted={'PASS' if predicted[bar] else 'FAIL':4} -> "
              f"{'as predicted' if match else 'DEVIATED'}", flush=True)
    print("VERDICT " + json.dumps({
        "arm": args.arm, "package": "PASS" if package_pass else "FAIL",
        "bars_actual": actual, "bars_predicted": predicted, **detail,
    }, sort_keys=True), flush=True)
    return finish(0 if package_pass else 1)


if __name__ == "__main__":
    sys.exit(main())
