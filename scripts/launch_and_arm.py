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


# --- what each stack actually launches and records (pure; the tests hold these) ------

def launch_command(stack, imu_fusion=True):
    """The exact bringup command per stack. PURE so the tests can pin it.

    stock: the §3a middle -- no explorer, no decisive controller, RPP + bt_navigator
    on lean_nav2_stock.yaml (resolved from the DEPLOYED share, not the source tree),
    contact_marker via the launch's own default (it is part of the stock middle).
    enable_imu_fusion defaults TRUE per the protocol's standing open-decision
    ("include it unless the run reproduces a wheel-odom-only baseline").

    bespoke: the pre-§3a coverage stack, byte-for-byte the command this script has
    always run -- plus an explicit start_contact_marker:=false, because the marker
    is the STOCK middle's touch port (the bespoke controller has its own freeze
    marks) and an idle marker still costs ~14% of a Pi core.
    """
    if stack == "stock":
        return (
            "ros2 launch sphero_rvr_driver explore.launch.py "
            "start_motion_stack:=true start_explore:=false "
            "use_coverage_explorer:=false use_decisive_controller:=false "
            f"enable_imu_fusion:={'true' if imu_fusion else 'false'} "
            'nav2_params_file:="$(ros2 pkg prefix sphero_rvr_driver)'
            '/share/sphero_rvr_driver/config/lean_nav2_stock.yaml"'
        )
    if stack == "bespoke":
        return (
            "ros2 launch sphero_rvr_driver explore.launch.py "
            "start_motion_stack:=true start_explore:=true "
            "use_coverage_explorer:=true use_decisive_controller:=true "
            "start_contact_marker:=false"
        )
    raise ValueError(f"unknown stack {stack!r}")


def bag_topics(stack):
    """The record list per stack. PURE so the tests can pin it.

    Both sides of the driver seam always (/cmd_vel + /cmd_vel_motor + /diagnostics
    -- the 2026-08-16 autopsy's lesson). Stock adds the touch port's evidence chain
    (/contact_marks, /plan) and BOTH costmaps' raw+updates streams, so paint/clear
    at the layers is OBSERVED, not inferred -- run 3d's horizontal-leg analysis had
    to reconstruct tof_layer behaviour because these were not in the bag.
    """
    base = [
        "/cmd_vel", "/cmd_vel_motor", "/diagnostics", "/collision_stop/state",
        "/odom", "/scan", "/tf", "/tf_static",
        "/tof/obstacles", "/tof/points", "/tof/state",
    ]
    if stack == "stock":
        base += [
            "/contact_marks", "/plan",
            "/local_costmap/costmap_raw", "/local_costmap/costmap_raw_updates",
            "/global_costmap/costmap_raw", "/global_costmap/costmap_raw_updates",
        ]
    return base


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


def gate_lifecycles_active():
    """STOCK: every lifecycle node the goal path depends on must say ACTIVE itself.

    slam_toolbox is the card's P2 MUST -- launched without autostart it sits
    unconfigured looking exactly like a healthy node (tools-that-lie family). The
    nav2 four are the goal path. Asking each node is the gate; a launch log that
    scrolled past "Activating" is narration.
    """
    for node in ("slam_toolbox", "planner_server", "controller_server",
                 "bt_navigator", "behavior_server"):
        rc, out = sh(f"timeout 15 ros2 lifecycle get /{node}", timeout=30)
        state = out.strip().splitlines()[-1] if out.strip() else ""
        if rc != 0 or not state.startswith("active"):
            die(f"/{node} lifecycle is {state or 'unreadable'}, not active",
                "a node that is up but not active publishes nothing and refuses "
                "nothing -- do not send a goal into it")
        say("gate", f"/{node} active")


def gate_marks_publisher():
    """STOCK: the touch port's producer must be ON THE WIRE before wheels.

    contact_marker shipped subscribed-to-by-both-costmaps and launched by NOTHING
    (caught in pre-flight 2026-08-18, the never-launched-node family). The launch
    now starts it; this gate is the tripwire if that ever regresses -- a flight
    without it silently has no touch response at all.
    """
    rc, out = sh("timeout 10 ros2 topic info /contact_marks", timeout=20)
    m = re.search(r"Publisher count:\s*(\d+)", out)
    if not m or int(m.group(1)) < 1:
        die("/contact_marks has no publisher -- the touch port's producer is not up",
            "is start_contact_marker true? without it a contact plants no mark and "
            "the planner never learns")
    say("gate", f"/contact_marks publishers={m.group(1)}")


def gate_battery(floor_fraction=0.25):
    """STOCK: read the battery ALOUD at connect, and hold the 25% floor.

    The floor is chassis-run protocol; reading it aloud is the operator contract
    (the PM relays it to Scott before liftoff). rvr_node publishes battery_state
    every 5 s once the driver is connected, so no message inside 20 s is its own
    finding -- a driver that is not talking, not a quiet battery.
    """
    rc, out = sh("timeout 20 ros2 topic echo /battery_state --once", timeout=40)
    m = re.search(r"percentage:\s*([0-9.]+)", out)
    if not m:
        die("no /battery_state within 20 s -- the driver is not reporting",
            "chassis off, or rvr_node down; do not fly blind on power")
    pct = float(m.group(1))
    say("gate", f"BATTERY {pct * 100.0:.0f}%")
    if pct < floor_fraction:
        die(f"battery {pct * 100.0:.0f}% is under the {floor_fraction * 100.0:.0f}% "
            f"floor", "charge before flying; an idle stack drained a battery flat "
            "on 2026-08-03")


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
    ap.add_argument("--stack", choices=("stock", "bespoke"),
                    help="REQUIRED (except --teardown). Which stack flies: 'stock' "
                         "= the §3a middle (RPP + bt_navigator on lean_nav2_stock, "
                         "contact_marker up, NEVER arms -- liftoff belongs to "
                         "scripts/fly_stock_goal.py); 'bespoke' = the coverage "
                         "stack, armed via mission/start as always. No default: a "
                         "wrong-stack launch costs a flight, so the operator says "
                         "which.")
    ap.add_argument("--no-arm", action="store_true",
                    help="bespoke only: run every gate and stop before mission/start")
    ap.add_argument("--no-imu-fusion", action="store_true",
                    help="stock only: reproduce a wheel-odom-only baseline (the "
                         "protocol's open decision defaults fusion ON)")
    ap.add_argument("--teardown", action="store_true",
                    help="stop what a previous run of this script started")
    ap.add_argument("--settle-s", type=float, default=30.0,
                    help="seconds to let the stack come up before reading gates")
    args = ap.parse_args()

    if args.teardown:
        teardown()
        return
    if not args.stack:
        ap.error("--stack {stock,bespoke} is required to bring anything up")

    if os.path.exists(PIDFILE):
        die("a previous run's pidfile exists", f"run --teardown first, or rm {PIDFILE}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    home = os.path.expanduser("~")
    csv_path = f"{home}/run_{stamp}.csv"
    bag_dir = f"{home}/bag_{stamp}"
    launch_log = f"{home}/launch_{stamp}.log"

    gate_preflight()

    say("record", f"recorder -> {csv_path}")
    # Absolute path, no `cd`. spawn() wraps every command in `bash -lc "SETUP && exec CMD"`,
    # and `exec` takes an EXECUTABLE -- so a command beginning with the shell builtin `cd`
    # dies instantly with "exec: cd: not found" and the process never starts. This was the
    # only spawn that used `cd`, so it was the only one that never ran: the first real
    # mission use of this script brought the whole stack up with NO recorder at all.
    # gate_recording caught it, which is the one reason this cost minutes and not a flight.
    spawn(f"python3 {REPO}/diagnostics/run_recorder.py 1800 {csv_path}",
          f"{home}/recorder_{stamp}.log")

    say("bringup", f"{args.stack} stack (no camera, no monocular detector) ...")
    spawn(launch_command(args.stack, imu_fusion=not args.no_imu_fusion), launch_log)
    say("bringup", f"settling {args.settle_s:.0f}s ...")
    time.sleep(args.settle_s)

    say("record", f"bag -> {bag_dir}")
    # /diagnostics is NOT optional, and its absence cost the 2026-08-16 autopsy its
    # answer. Mission 1 recorded 41 commands at 0.4 rad/s on /cmd_vel_motor against an
    # /odom that never moved, and nothing in the bag could say whether the driver turned
    # those commands into motor packets. rvr_node was publishing exactly that the whole
    # time -- motor_transport_write_count, motion_transport_write_count,
    # last_motor_payload_hex, last_motor_transport_write_epoch_s, fail_safe_active,
    # motor_stall, motor_fault -- on /diagnostics, unrecorded. The owner published the
    # fact; the recording dropped it, so the analysis had to infer across the seam and
    # convicted the wrong component.
    spawn("ros2 bag record -s mcap -o " + bag_dir + " "
          + " ".join(bag_topics(args.stack)), f"{home}/bag_{stamp}.log")
    time.sleep(8)

    gate_params()
    gate_tof()
    gate_brake_state()
    if args.stack == "bespoke":
        gate_disarmed()
    else:
        # Stock has no explorer to be disarmed; its liftoff IS the goal, and the
        # gates that replace gate_disarmed are the ones a goal depends on.
        gate_lifecycles_active()
        gate_marks_publisher()
        gate_battery()
    gate_recording(csv_path, bag_dir)

    if args.stack == "stock":
        # NEVER ARMS. For the stock middle, arming means sending a goal, and the
        # goal tool owns that -- one goal per invocation, verified before send.
        say("done", "all gates PASSED. STOCK STACK UP, DISARMED by construction.")
        say("done", "liftoff: python3 scripts/fly_stock_goal.py --x <X> --y <Y> "
                    "(verifies mapped-free + cost-0 + dry-run plan before sending)")
        print(f"\n  artifacts: {csv_path}\n             {bag_dir}\n"
              f"             {launch_log}")
        print(f"  teardown : python3 scripts/launch_and_arm.py --teardown\n")
        return

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
