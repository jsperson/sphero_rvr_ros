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
import filecmp
import glob
import json
import os
import signal
import subprocess
import sys
import time

WS = os.path.expanduser("~/ros2_ws")
REPO = os.path.join(WS, "src", "sphero_rvr_ros")
PIDFILE = "/tmp/launch_and_arm.pids"
SETUP = f"source /opt/ros/jazzy/setup.bash && source {WS}/install/setup.bash"
BRANCH = "prototype/stock-middle"


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

def launch_command(stack, imu_fusion=True, ride_along_watcher=False):
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
        # ride_along_watcher: the D RIDE-ALONG OVERRIDE. start_refusal_watcher
        # defaults FALSE in the launch until a flight ride-along clears the
        # mechanism; passing this flag IS that ride-along, and the deviation is
        # logged at bringup so the run's record says so explicitly.
        watcher = (" start_refusal_watcher:=true" if ride_along_watcher else "")
        return (
            "ros2 launch sphero_rvr_driver explore.launch.py "
            "start_motion_stack:=true start_explore:=false "
            "use_coverage_explorer:=false use_decisive_controller:=false "
            f"enable_imu_fusion:={'true' if imu_fusion else 'false'} "
            'nav2_params_file:="$(ros2 pkg prefix sphero_rvr_driver)'
            '/share/sphero_rvr_driver/config/lean_nav2_stock.yaml"'
            + watcher
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
            # D's own seam: the request lane records every firing the watcher
            # made, including ones the marker rejects -- the promotion story is
            # reconstructable from the bag alone.
            "/contact_marks/promote",
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

def gate_verify():
    """PHASE 0: the code about to fly is the code that was reviewed. Three claims,
    each checked against a different authority: HEAD == origin/BRANCH (the review
    lives at origin -- deploy verification is against ORIGIN, not the local clone's
    idea of itself), the tree is clean (an uncommitted edit flies unreviewed), and
    the INSTALLED tree byte-matches the source tree (a clean repo above a stale
    `colcon build` runs last week's code while every SHA check smiles -- the
    pycache-same-length family's big sibling). Refuses loudly on all three."""
    rc, out = sh(f"cd {REPO} && timeout 25 git fetch origin {BRANCH}", timeout=40)
    if rc != 0:
        die("git fetch failed -- cannot verify HEAD against origin",
            "no network? verify is fail-closed: fix connectivity, don't skip it")
    rc, out = sh(f"cd {REPO} && git rev-parse HEAD origin/{BRANCH}")
    hashes = [l.strip() for l in out.splitlines() if len(l.strip()) == 40]
    if rc != 0 or len(hashes) != 2:
        die(f"could not read HEAD/origin SHAs: {out.strip()[:120]}")
    head, origin = hashes
    if head != origin:
        die(f"HEAD {head[:7]} != origin/{BRANCH} {origin[:7]}",
            "pull (or push) until they agree; the reviewed code is the one at origin")
    rc, out = sh(f"cd {REPO} && git status --porcelain")
    if out.strip():
        die(f"working tree is dirty:\n{out.strip()[:300]}",
            "commit or stash; an uncommitted edit flies unreviewed")
    say("verify", f"HEAD == origin/{BRANCH} == {head[:7]}, tree clean")

    # installed tree: index install/ once by basename, then demand every shipped
    # source file has a byte-identical installed twin under its own parent dir.
    index = {}
    for root, _dirs, files in os.walk(os.path.join(WS, "install")):
        for f in files:
            index.setdefault(f, []).append(os.path.join(root, f))
    checked, missing, stale = 0, [], []
    for reldir, pattern in (("src/sphero_rvr_driver", "*.py"),
                            ("src/sphero_rvr_core", "*.py"),
                            ("config", "*.yaml"), ("launch", "*.py")):
        parent = os.path.basename(reldir)
        for src_path in sorted(glob.glob(os.path.join(REPO, reldir, pattern))):
            name = os.path.basename(src_path)
            twins = [p for p in index.get(name, [])
                     if os.path.basename(os.path.dirname(p)) == parent]
            if not twins:
                missing.append(f"{parent}/{name}")
                continue
            for twin in twins:
                if not filecmp.cmp(src_path, twin, shallow=False):
                    stale.append(f"{parent}/{name}")
            checked += 1
    if missing or stale:
        die(f"installed tree does not match source -- missing: {missing[:5]} "
            f"stale: {stale[:5]}",
            f"cd {WS} && colcon build --packages-select sphero_rvr_driver, "
            f"then rerun; a stale install runs last week's code under this week's SHA")
    say("verify", f"installed tree matches source ({checked} files byte-compared)")
    return head[:7]


def run_gate_probe(stack, csv_path, bag_dir):
    """ONE resident rclpy process (scripts/bringup_gates.py) replaces the fixed
    settle sleep and every per-gate `ros2` CLI spawn -- same gates, same receipts,
    measured 2026-08-18 at ~82 s of ceremony for checks that cost single-digit
    seconds asked directly. FAIL-CLOSED BY EXIT CODE: any exit but 0-with-a-READY-
    line -- a failed gate, a timeout, the probe crashing -- refuses the bringup.
    The probe dying is a refusal, never a shrug."""
    cmd = (f"python3 {REPO}/scripts/bringup_gates.py --stack {stack} "
           f"--csv {csv_path} --bag {bag_dir}")
    p = subprocess.Popen(["bash", "-lc", f"{SETUP} && {cmd}"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         stdin=subprocess.DEVNULL, text=True)
    ready = None
    for line in p.stdout:
        line = line.rstrip()
        if not line:
            continue
        say("gate", line)
        if line.startswith("READY "):
            ready = line[len("READY "):]
    rc = p.wait()
    if rc != 0 or ready is None:
        die(f"the gate probe did not clear the bringup (exit {rc}, "
            f"READY {'seen' if ready else 'never printed'})",
            "read the GATE FAIL line above; every probe death is a refusal")
    return json.loads(ready)


def gate_preflight():
    say("preflight", "running scripts/preflight_pi.py ...")
    rc, out = sh(f"cd {REPO} && python3 scripts/preflight_pi.py", timeout=180)
    for line in out.splitlines():
        if line.startswith(("PASS", "FAIL", "UNKNOWN", "NOT CLEARED", "CLEARED")):
            print("           " + line, flush=True)
    if rc != 0 or "NOT CLEARED" in out:
        die("preflight did not clear",
            "read its remedy above; a dead chassis is the usual answer")


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
    ap.add_argument("--ride-along-watcher", action="store_true",
                    help="stock only: THE D RIDE-ALONG OVERRIDE -- start "
                         "refusal_watcher despite its flight default of false. "
                         "Logged at bringup as the deviation it is; use only for "
                         "the ride-along flight the mechanism's clearance "
                         "requires (docs/design_tof_planner_visibility.md).")
    ap.add_argument("--teardown", action="store_true",
                    help="stop what a previous run of this script started")
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

    t_staged = time.monotonic()
    sha = gate_verify()
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
    if args.ride_along_watcher:
        say("DEVIATION", "start_refusal_watcher:=true -- the D ride-along "
                         "override; the launch default is false until this very "
                         "flight clears the mechanism")
    spawn(launch_command(args.stack, imu_fusion=not args.no_imu_fusion,
                         ride_along_watcher=args.ride_along_watcher), launch_log)

    say("record", f"bag -> {bag_dir}")
    # Spawned IMMEDIATELY after the launch -- no settle sleep before it, no sleep
    # after it. `ros2 bag record` subscribes to topics as their publishers appear,
    # so nothing is gained by waiting, and the probe's recording gate still demands
    # growth before anything clears. (The old fixed 30 s settle + 8 s bag pause were
    # measured 2026-08-18 as pure dead time on a stack whose lifecycles activate in
    # about half the settle.)
    #
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

    receipts = run_gate_probe(args.stack, csv_path, bag_dir)
    receipts["sha"] = sha
    receipts["stack"] = args.stack
    receipts["staged_to_ready_s"] = round(time.monotonic() - t_staged, 1)
    # THE machine-parseable liftoff receipt: one line, sha + battery + every gate,
    # for whatever mission layer sits above this verb (the NL front door reads this,
    # not the narration above it).
    print("READY " + json.dumps(receipts, sort_keys=True), flush=True)

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
