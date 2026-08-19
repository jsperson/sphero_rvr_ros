#!/usr/bin/env python3
"""ONE resident process for every post-launch bringup gate. Fail-closed.

    python3 bringup_gates.py --stack stock --csv <path> --bag <dir> [--timeout 90]

WHY THIS EXISTS (Scott: "speed up the spin up"; measured 2026-08-18): the old
gate ceremony spent ~52 s running the same checks this file runs in single-digit
seconds, because every gate paid a fresh 2-4 s `ros2` CLI process bringup — the
instrument-DDoS lesson in slow motion. And a fixed 30 s settle sleep waited for
lifecycles that activate in half that. Same gates, same receipts, same
die-on-fail; only the VEHICLES were removed. Each preserved gate names its
incident inline, exactly as the old functions did.

CONTRACT WITH THE CALLER (launch_and_arm): exit 0 = every gate passed, and the
last line is one machine-parseable READY payload. ANY other exit — a failed
gate, a timeout, this process crashing — means the bringup is NOT cleared, and
the caller must refuse: the probe dying is a refusal, never a shrug
(fail-closed by exit-code, per the consensus requirement).

Gate order: lifecycles first (they are the settle — polled, not slept), then
everything else CONCURRENTLY off one node's subscriptions and service clients.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from sphero_rvr_core.tof_gate import MIN_COUNTER_SPAN_S, tof_gate_verdict
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

#: Same derivation as the old gate_tof: below 6.5 Hz two frames exceed the
#: 0.30 s staleness bound and the brake stops looking (gauntlet mission 1).
TOF_RATE_MIN_HZ = 6.5
BATTERY_FLOOR = 0.25

LIFECYCLES = {
    "stock": ["slam_toolbox", "planner_server", "controller_server",
              "bt_navigator", "behavior_server"],
    # stock-explore rides the SAME stock middle; the explorer itself is not a
    # lifecycle node -- its gate is the disarmed check below.
    "stock-explore": ["slam_toolbox", "planner_server", "controller_server",
                      "bt_navigator", "behavior_server"],
    # bespoke never had a lifecycle gate (its arming gate is gate_disarmed);
    # same gates as before, not more, not fewer.
    "bespoke": [],
}

#: The safety constants, read FROM THE ROBOT (a config file is a claim; the
#: running node is the robot) — identical expectations to the old gate_params.
SUPERVISOR_PARAMS = {
    "low_obstacle_hold_on_vanish_enable": "true",   # D39 hold must be enabled
    "footprint_front_m": "0.0965",                  # measured footprint (tape)
    "footprint_rear_m": "0.1145",                   # measured + cable allowance
}


class GateProbe(Node):
    def __init__(self):
        super().__init__("bringup_gate_probe")
        self.state_line = None
        self.tof_line = None
        self.battery = None
        self.explorer_status = None
        self.create_subscription(String, "/collision_stop/state",
                                 lambda m: setattr(self, "state_line", m.data), 10)
        self.create_subscription(String, "/tof/state",
                                 lambda m: setattr(self, "tof_line", m.data), 10)
        self.create_subscription(String, "/coverage_explorer/status",
                                 lambda m: setattr(self, "explorer_status", m.data), 10)
        self.create_subscription(BatteryState, "/battery_state",
                                 lambda m: setattr(self, "battery", m.percentage),
                                 qos_profile_sensor_data)


def spin_quietly(probe):
    """Daemon-thread spin; rclpy.shutdown() raising in the spinner must never
    scribble a traceback after the READY line (the caller parses stdout)."""
    try:
        rclpy.spin(probe)
    except Exception:
        pass


def wait_until(predicate, timeout_s, period_s=0.2):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = predicate()
        if value is not None:
            return value
        time.sleep(period_s)
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stack", required=True,
                    choices=("stock", "bespoke", "stock-explore"))
    ap.add_argument("--csv", required=True, help="recorder csv path (growth gate)")
    ap.add_argument("--bag", required=True, help="bag directory (growth gate)")
    ap.add_argument("--timeout", type=float, default=90.0,
                    help="lifecycle-settle budget (phase A polls to 0.7x this); "
                         "phase B gates carry their own independent ceilings")
    args = ap.parse_args()

    t_start = time.monotonic()
    receipts = {}

    rclpy.init()
    probe = GateProbe()
    spinner = threading.Thread(target=spin_quietly, args=(probe,), daemon=True)
    spinner.start()

    def finish(code):
        """Every exit -- pass or fail -- walks through here: stop the spinner
        BEFORE the interpreter starts tearing down, or rcl aborts the whole
        process from the still-spinning daemon thread (measured: exit 134 and a
        core dump where exit 1 was meant)."""
        rclpy.try_shutdown()
        spinner.join(timeout=3.0)
        return code

    def fail(gate, detail):
        print(f"GATE FAIL {gate}: {detail}", flush=True)
        return finish(1)

    # ADVISORY FIX (review 2026-08-18): an unforeseen exception anywhere in the
    # gate body (malformed status json, csv vanishing mid-read) must still walk
    # out through finish() -- otherwise the daemon spinner is live at interpreter
    # teardown and the refusal arrives as a core dump instead of a GATE FAIL line.
    try:
        # ---- PHASE A: lifecycles ARE the settle (polled, never slept) ---------------
        clients = {}
        for name in LIFECYCLES[args.stack]:
            clients[name] = probe.create_client(GetState, f"/{name}/get_state")
        pending = set(clients)
        deadline = time.monotonic() + args.timeout * 0.7
        while pending and time.monotonic() < deadline:
            for name in sorted(pending):
                client = clients[name]
                if not client.service_is_ready():
                    continue
                future = client.call_async(GetState.Request())
                t0 = time.monotonic()
                while not future.done() and time.monotonic() - t0 < 2.0:
                    time.sleep(0.05)
                result = future.result() if future.done() else None
                if result is not None and result.current_state.label == "active":
                    receipts[f"lifecycle.{name}"] = "active"
                    pending.discard(name)
            if pending:
                time.sleep(0.5)
        if pending:
            return fail("lifecycles", f"never active: {sorted(pending)} -- a node that "
                        "is up but not active publishes nothing and refuses nothing")
        receipts["settle_s"] = round(time.monotonic() - t_start, 1)
        print(f"GATE PASS lifecycles ({receipts['settle_s']}s)", flush=True)

        # ---- PHASE B: everything else, concurrently off the one node ----------------
        # Every wait below is a poll with a CEILING, never a sleep: on a healthy
        # bringup each returns in well under a second, and the generous ceilings only
        # matter on the bringups that were going to fail anyway. (bespoke has no
        # lifecycle phase, so these ceilings are also its whole settle.)
        # supervisor params -- the robot's own declared values, not the YAML
        param_client = probe.create_client(
            GetParameters, "/lidar_collision_stop_supervisor/get_parameters")
        if not wait_until(lambda: param_client.service_is_ready() or None, 60.0):
            return fail("params", "supervisor parameter service unavailable")
        request = GetParameters.Request(names=list(SUPERVISOR_PARAMS))
        future = param_client.call_async(request)
        t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        result = future.result() if future.done() else None
        if result is None:
            return fail("params", "get_parameters timed out")
        for name, value in zip(SUPERVISOR_PARAMS, result.values):
            got = (str(bool(value.bool_value)).lower() if value.type == 1
                   else f"{value.double_value:.4f}".rstrip("0").rstrip("."))
            want = SUPERVISOR_PARAMS[name]
            if got != want:
                return fail("params", f"{name} is {got}, expected {want}")
            receipts[f"param.{name}"] = got
        print("GATE PASS params", flush=True)

        # tof: the sensor's own words (rule-B-off-with-tests-green, never again)
        tof = wait_until(lambda: probe.tof_line, 30.0)
        if tof is None:
            return fail("tof", "no /tof/state -- the brake FAILS OPEN with no producer")
        fields = dict(re.findall(r"(\w+)=([^\s']+)", tof))
        if int(fields.get("obstacle_consumers", "0")) < 1:
            return fail("tof", "obstacle_consumers=0 -- supervisor not subscribed")
        # D59: the rate is measured from the sensor's FRAMES COUNTER across
        # >=10 s of its own clock, never from one sample of the noisy 5 s
        # rate_hz window -- a single sample of that field against the hard band
        # stopped the 2026-08-19 bench sitting at 6.30/6.43 on a sensor whose
        # counter never ran below 6.65 (bag_bench_20260819_122918; the replay
        # is tests/test_tof_gate.py). Counters-not-levels, applied to our own
        # gate; the band and its staleness derivation are unchanged above.
        samples = []
        verdict, rate = None, None
        counter_deadline = time.monotonic() + MIN_COUNTER_SPAN_S + 10.0
        while time.monotonic() < counter_deadline:
            fields = dict(re.findall(r"(\w+)=([^\s']+)", probe.tof_line))
            try:
                pair = (int(fields["frames"]), float(fields["uptime_s"]))
            except (KeyError, ValueError):
                return fail("tof", "state line lacks frames/uptime_s -- the "
                            "counter gate cannot run; fix the producer, do not "
                            "fall back to the one-sample rate_hz field")
            if not samples or pair != samples[-1]:
                samples.append(pair)
                verdict, rate = tof_gate_verdict(samples, TOF_RATE_MIN_HZ)
                if verdict is not None:
                    break
            time.sleep(0.2)
        if verdict is None:
            return fail("tof", f"frames counter never spanned "
                        f"{MIN_COUNTER_SPAN_S:.0f}s of sensor uptime -- the "
                        "producer is up but its clock/counter is not advancing")
        if not verdict:
            return fail("tof", f"{rate} Hz over >={MIN_COUNTER_SPAN_S:.0f}s of "
                        f"the sensor's own frames counter, below the "
                        f"{TOF_RATE_MIN_HZ} band")
        receipts["tof.rate_hz"] = rate
        receipts["tof.i2c_errors"] = int(fields.get("i2c_errors", "-1"))
        print(f"GATE PASS tof ({rate} Hz by frames counter over "
              f">={MIN_COUNTER_SPAN_S:.0f}s, i2c_errors="
              f"{receipts['tof.i2c_errors']})", flush=True)

        # brake state: the D39 hold must not already be engaged
        state = wait_until(lambda: probe.state_line, 20.0)
        if state is None or "cam_hold_active" not in state:
            return fail("brake", "no /collision_stop/state with cam_hold fields")
        hold = re.search(r"cam_hold_active=(\w+)", state).group(1)
        if hold != "false":
            return fail("brake", f"D39 hold already engaged before arming ({hold})")
        receipts["brake.cam_hold"] = hold
        print("GATE PASS brake", flush=True)

        # bespoke + stock-explore: D29 makes bringup disarmed; an already-armed
        # explorer here means a mission is running and the caller is about to
        # lose the start of it
        if args.stack in ("bespoke", "stock-explore"):
            status = wait_until(lambda: probe.explorer_status, 30.0)
            if status is None:
                return fail("disarmed", "no /coverage_explorer/status")
            st = json.loads(status)
            if st["armed"]:
                return fail("disarmed", "the explorer is ALREADY ARMED before bringup "
                            "armed it -- a mission is running")
            receipts["explorer.armed"] = st["armed"]
            receipts["explorer.done"] = st["done"]
            print("GATE PASS disarmed", flush=True)

        # recording: growth, not existence (poll-with-timeout keeps the semantics --
        # a recorder that opened a file and died leaves a header behind)
        def size(path):
            try:
                if os.path.isdir(path):
                    return sum(os.path.getsize(os.path.join(path, f))
                               for f in os.listdir(path))
                return os.path.getsize(path)
            except OSError:
                return 0

        a_csv, a_bag = size(args.csv), size(args.bag)
        grown = wait_until(
            lambda: (True if size(args.csv) > a_csv and size(args.bag) > a_bag
                     else None), 15.0, period_s=0.5)
        if not grown:
            return fail("recording", f"csv {a_csv}->{size(args.csv)} bag "
                        f"{a_bag}->{size(args.bag)} -- an unrecorded run has to be "
                        f"repeated")
        header = open(args.csv).readline()
        for column in ("cam_hold_active", "cam_hold_reason"):
            if column not in header:
                return fail("recording", f"recorder csv lacks {column}")
        receipts["recording"] = "growing"
        print("GATE PASS recording", flush=True)

        # stock's marks + battery run LAST on purpose: the battery percentage in
        # the READY receipt is the freshest read this process can give the operator
        # before liftoff, not a number from the top of the gate suite.
        # stock: the touch port's producer must be on the wire
        if args.stack in ("stock", "stock-explore"):
            pubs = wait_until(
                lambda: (probe.count_publishers("/contact_marks") or None), 10.0)
            if not pubs:
                return fail("marks", "/contact_marks has no publisher -- a flight "
                            "without it has no touch response at all")
            receipts["marks.publishers"] = pubs
            print(f"GATE PASS marks ({pubs} publisher)", flush=True)

            battery = wait_until(lambda: probe.battery, 20.0)
            if battery is None:
                return fail("battery", "no /battery_state -- driver not reporting; "
                            "chassis off?")
            if battery < BATTERY_FLOOR:
                return fail("battery", f"{battery * 100:.0f}% under the "
                            f"{BATTERY_FLOOR * 100:.0f}% floor")
            receipts["battery_pct"] = round(battery * 100)
            print(f"GATE PASS battery ({receipts['battery_pct']}%)", flush=True)


        receipts["gates_total_s"] = round(time.monotonic() - t_start, 1)
        print("READY " + json.dumps(receipts, sort_keys=True), flush=True)
        return finish(0)
    except SystemExit:
        raise
    except Exception as e:
        return fail("probe", f"unhandled: {e!r}")


if __name__ == "__main__":
    sys.exit(main())
