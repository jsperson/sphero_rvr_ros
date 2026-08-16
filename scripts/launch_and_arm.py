#!/usr/bin/env python3
"""Staged rover to turning wheels, in one command, with every gate read on the way.

    python3 scripts/launch_and_arm.py                 # gates, bringup, record, ARM
    python3 scripts/launch_and_arm.py --no-arm        # everything except the arming
    python3 scripts/launch_and_arm.py --teardown      # stop what a previous run started

WHY THIS EXISTS. Bringup is five commands across three panes, in an order that matters,
with eight things to read before arming. Assembled by hand it takes minutes and it
differs from the last time in ways nobody wrote down -- which is how run 185048's entire
53 s mission ran and died *during* the gate checks, with the operator watching a stopped
rover having no idea a mission had happened.

WHAT IT WILL NOT DO. It will not arm on a failed gate, and it will not arm on a gate it
could not read. Every check below either passes with a value printed or stops the run;
there is no branch that shrugs. That asymmetry is the whole point -- this script exists
to make arming FASTER, never EASIER.

WHAT IT DELIBERATELY DOES NOT START. The camera, and the monocular low-obstacle
detector. Scott's charter of 2026-08-16: the camera is an intelligence sensor invoked on
demand by Track 2, never part of default bringup, never in the safety stack or direct
navigation. On gauntlet mission 1 those two took ~66% of a CPU on a Pi that reached load
10.7 and starved the ToF to 5.4 Hz -- below the rate its own staleness bound assumes.

NOT A REPLACEMENT FOR THE RUN CARD. The card says what to watch and what a run is for;
this only gets the wheels turning identically every time.
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

WS = os.path.expanduser("~/ros2_ws")
REPO = os.path.join(WS, "src", "sphero_rvr_ros")
PIDFILE = "/tmp/launch_and_arm.pids"
SETUP = f"source /opt/ros/jazzy/setup.bash && source {WS}/install/setup.bash"

#: The rate band the ToF's staleness bound is derived from: low_obstacle_max_age_s
#: (0.30 s) is "about two frames" at 6.5-7.6 Hz. Below 6.5 Hz two frames exceed the
#: bound, so one dropped frame ages the cloud out and the brake stops looking.
TOF_RATE_MIN_HZ = 6.5


def sh(cmd, timeout=30):
    """Run under a sourced ROS environment. Returns (rc, stdout+stderr)."""
    p = subprocess.run(["bash", "-lc", f"{SETUP} && {cmd}"],
                       capture_output=True, text=True, timeout=timeout)
    return p.returncode, (p.stdout or "") + (p.stderr or "")


def spawn(cmd, logfile):
    """Start a detached process, record its PID, return it. `setsid` so it survives this
    script and the ssh session that ran it."""
    with open(logfile, "ab") as fh:
        p = subprocess.Popen(["bash", "-lc", f"{SETUP} && exec {cmd}"],
                             stdout=fh, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, start_new_session=True)
    with open(PIDFILE, "a") as fh:
        fh.write(f"{p.pid} {cmd.split()[0]}\n")
    return p.pid


def say(stage, msg):
    print(f"[{time.strftime('%H:%M:%S')}] {stage:<9} {msg}", flush=True)


def die(msg, remedy=""):
    print(f"\n*** STOPPED: {msg}", flush=True)
    if remedy:
        print(f"    -> {remedy}", flush=True)
    print("    Nothing was armed. `--teardown` stops anything already started.",
          flush=True)
    sys.exit(1)


# --- gates ---------------------------------------------------------------------------

def gate_preflight():
    say("preflight", "running scripts/preflight_pi.py ...")
    rc, out = sh(f"cd {REPO} && python3 scripts/preflight_pi.py", timeout=180)
    for line in out.splitlines():
        if line.startswith(("PASS", "FAIL", "UNKNOWN", "NOT CLEARED", "CLEARED")):
            print("           " + line, flush=True)
    if rc != 0 or "NOT CLEARED" in out:
        die("preflight did not clear",
            "read its remedy above; a dead chassis is the usual answer")


def gate_params():
    """The safety constants, read FROM THE ROBOT rather than from the file. A config
    file is a claim; the running node is the robot."""
    checks = {
        "low_obstacle_hold_on_vanish_enable": ("true", "D39 hold must be enabled"),
        "footprint_front_m": ("0.0965", "measured footprint (2026-08-15 tape)"),
        "footprint_rear_m": ("0.1145", "measured footprint + cable allowance"),
    }
    for name, (want, why) in checks.items():
        rc, out = sh(f"ros2 param get /lidar_collision_stop_supervisor {name}")
        m = re.search(r"value is:\s*(\S+)", out)
        if not m:
            die(f"could not read parameter {name} ({why})",
                "is lidar_collision_stop_supervisor up?")
        got = m.group(1).lower()
        if got != want.lower():
            die(f"{name} is {got}, expected {want} ({why})")
        say("gate", f"{name} = {got}")


def gate_tof():
    rc, out = sh("timeout 20 ros2 topic echo /tof/state --once --full-length", timeout=40)
    if "rate_hz" not in out:
        die("no /tof/state", "the ToF is the collision brake's only producer")
    rate = float(re.search(r"rate_hz=([\d.]+)", out).group(1))
    consumers = int(re.search(r"obstacle_consumers=(\d+)", out).group(1))
    errors = int(re.search(r"i2c_errors=(\d+)", out).group(1))
    say("gate", f"tof rate_hz={rate} consumers={consumers} i2c_errors={errors}")
    if consumers < 1:
        die("obstacle_consumers=0 -- the supervisor is NOT subscribed to the ToF",
            "the brake fails OPEN with no producer, whatever its config says")
    if rate < TOF_RATE_MIN_HZ:
        die(f"ToF at {rate} Hz, below the {TOF_RATE_MIN_HZ} Hz its staleness bound "
            f"assumes -- one dropped frame ages the cloud out",
            "shed CPU load (nothing but the motion stack should be running) and retry")


def gate_brake_state():
    rc, out = sh("timeout 20 ros2 topic echo /collision_stop/state --once --full-length",
                 timeout=40)
    if "cam_hold_active" not in out:
        die("no /collision_stop/state, or it lacks cam_hold fields",
            "is the supervisor running the current binary?")
    hold = re.search(r"cam_hold_active=(\w+)", out).group(1)
    reason = re.search(r"cam_hold_reason=(\S+)", out).group(1)
    say("gate", f"cam_hold_active={hold} cam_hold_reason={reason}")
    if hold != "false":
        die(f"the D39 hold is already engaged before arming (reason={reason})",
            "a hold stuck at startup means arming into a permanent forward clamp; "
            "if reason=held_no_pose, TF is not flowing yet")


def gate_disarmed():
    rc, out = sh("timeout 20 ros2 topic echo /coverage_explorer/status --once "
                 "--full-length", timeout=40)
    m = re.search(r"(\{.*\})", out, re.S)
    if not m:
        die("no /coverage_explorer/status")
    st = json.loads(m.group(1).replace("'", ""))
    say("gate", f"explorer armed={st['armed']} done={st['done']}")
    if st["armed"]:
        die("the explorer is ALREADY ARMED before this script armed it",
            "D29 makes bringup disarmed; an armed explorer here means a mission is "
            "running and you are about to lose the start of it")


def gate_recording(csv_path, bag_dir):
    """Growth, not existence. A recorder that opened a file and then died leaves a
    header behind, and 'the file is there' has been mistaken for 'it is recording'."""
    def size(p):
        try:
            if os.path.isdir(p):
                return sum(os.path.getsize(os.path.join(p, f)) for f in os.listdir(p))
            return os.path.getsize(p)
        except OSError:
            return 0
    a_csv, a_bag = size(csv_path), size(bag_dir)
    time.sleep(6)
    b_csv, b_bag = size(csv_path), size(bag_dir)
    say("gate", f"recorder {a_csv}->{b_csv} B, bag {a_bag}->{b_bag} B")
    if b_csv <= a_csv:
        die("the recorder CSV is not growing", "an unrecorded run has to be repeated")
    if b_bag <= a_bag:
        die("the bag is not growing", "an unrecorded run has to be repeated")
    header = open(csv_path).readline()
    for col in ("cam_hold_active", "cam_hold_reason"):
        if col not in header:
            die(f"the recorder CSV has no {col} column",
                "the D39 hold's episodes cannot be reconstructed without it")
    say("gate", "recorder header carries the cam_hold columns")


# --- teardown ------------------------------------------------------------------------

def teardown():
    """Stop what a previous run started, in the order the protocol requires: lidar motor
    by SERVICE first (killing the node leaves the disc spinning ownerless), then
    processes by EXPLICIT PID -- `pkill -f` matches this script's own command line and
    has killed an operator's ssh session four times."""
    say("teardown", "stopping lidar motor by service ...")
    sh("timeout 15 ros2 service call /stop_motor std_srvs/srv/Empty", timeout=30)
    if not os.path.exists(PIDFILE):
        say("teardown", "no pidfile; nothing recorded as started by this script")
        return
    pids = []
    for line in open(PIDFILE):
        parts = line.split()
        if parts and parts[0].isdigit():
            pids.append(int(parts[0]))
    for pid in reversed(pids):                    # recorders last started, stopped first
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
            say("teardown", f"SIGINT -> pgid of {pid}")
        except (ProcessLookupError, PermissionError):
            pass
    time.sleep(8)
    os.remove(PIDFILE)
    rc, out = sh("timeout 20 ros2 node list", timeout=40)
    remaining = [n for n in out.splitlines() if n.startswith("/")]
    say("teardown", f"{len(remaining)} nodes remain: {remaining}")
    sh("ros2 daemon stop")


# --- main ----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-arm", action="store_true",
                    help="run every gate and stop before mission/start")
    ap.add_argument("--teardown", action="store_true",
                    help="stop what a previous run of this script started")
    ap.add_argument("--settle-s", type=float, default=30.0,
                    help="seconds to let the stack come up before reading gates")
    args = ap.parse_args()

    if args.teardown:
        teardown()
        return

    if os.path.exists(PIDFILE):
        die("a previous run's pidfile exists", f"run --teardown first, or rm {PIDFILE}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    home = os.path.expanduser("~")
    csv_path = f"{home}/run_{stamp}.csv"
    bag_dir = f"{home}/bag_{stamp}"
    launch_log = f"{home}/launch_{stamp}.log"

    gate_preflight()

    say("record", f"recorder -> {csv_path}")
    spawn(f"cd {REPO}/diagnostics && python3 run_recorder.py 1800 {csv_path}",
          f"{home}/recorder_{stamp}.log")

    say("bringup", "explore.launch.py (no camera, no monocular detector) ...")
    spawn("ros2 launch sphero_rvr_driver explore.launch.py start_motion_stack:=true "
          "start_explore:=true use_coverage_explorer:=true "
          "use_decisive_controller:=true", launch_log)
    say("bringup", f"settling {args.settle_s:.0f}s ...")
    time.sleep(args.settle_s)

    say("record", f"bag -> {bag_dir}")
    spawn(f"ros2 bag record -s mcap -o {bag_dir} /cmd_vel /cmd_vel_motor "
          f"/collision_stop/state /odom /scan /tf /tf_static /tof/obstacles "
          f"/tof/points /tof/state", f"{home}/bag_{stamp}.log")
    time.sleep(8)

    gate_params()
    gate_tof()
    gate_brake_state()
    gate_disarmed()
    gate_recording(csv_path, bag_dir)

    if args.no_arm:
        say("done", "all gates PASSED. Not arming (--no-arm).")
        say("done", "arm with: ros2 service call "
                    "/coverage_explorer/mission/start std_srvs/srv/Trigger")
        return

    say("ARM", "all gates passed -- calling mission/start")
    rc, out = sh("timeout 30 ros2 service call /coverage_explorer/mission/start "
                 "std_srvs/srv/Trigger", timeout=60)
    if "success=True" not in out.replace(" ", ""):
        die(f"mission/start did not succeed: {out.strip()[:200]}")
    say("ARM", "MISSION ARMED")
    print(f"\n  artifacts: {csv_path}\n             {bag_dir}\n             {launch_log}")
    print(f"  teardown : python3 scripts/launch_and_arm.py --teardown\n")


if __name__ == "__main__":
    main()
