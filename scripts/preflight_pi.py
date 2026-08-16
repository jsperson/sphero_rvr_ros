#!/usr/bin/env python3
"""Staged-to-wheels preflight, run ON THE PI. One gate list, pass/fail per line.

WHY THIS EXISTS. Batch C's brief is blunt about it: bringup was a tax, and every
minute of it was avoidable. The chassis was off and nothing said so until serial
timeouts were read by hand -- and everything downstream (odom, then the SLAM anchor,
then the map, then goals) is a CASCADE from that one fact, with each layer failing in
a way that looks like its own problem. The installed tree was stale twice and a
sha256 loop caught it both times, retyped from a paragraph each time. The supervisor
latch needs `clear_estop` BEFORE `reset`, an ordering written down nowhere and
discovered live.

SO THE ORDER OF THE GATES IS THE POINT, not the gates themselves. Chassis-alive runs
FIRST because a false answer there is the one that costs an hour of debugging the
wrong layer.

DISCIPLINES THAT ARE NOT NEGOTIABLE HERE, each bought with a session:

  * `pkill -f <pattern>` MATCHES YOUR OWN SSH COMMAND LINE and has killed the
    operator's session mid-teardown four times. Nothing here pattern-kills. PIDs are
    collected, this process and its ancestors are excluded, and kills are by explicit
    PID.
  * `ros2 run` SPAWNS THE NODE AS A CHILD. Killing the wrapper leaves the node alive
    holding the serial device. So the child is tracked separately, and the check that
    matters afterwards is that the PORT IS FREE (`fuser`), never that a PID is gone.
    A previous version of this very probe orphaned an `rvr_node` holding
    /dev/ttyAMA0, produced 874 "dispatcher reader failed", and nearly manufactured
    the symptom it exists to detect.
  * GATE ON THE ROBOT'S OWN STATE LINE, not on the config. `/tof/state` once
    reported `rules=rule_a_only` with the pinned margin sitting in the config and
    every test green -- one gate away from flying "the first rule-B mission" with
    rule B off.

STAGES. The gates split by what must be true when they run, and the stage is stated
rather than inferred: `pre` needs an EMPTY graph, `post` needs the stack UP. Asking
for the wrong one is refused instead of producing confident nonsense.

    ./preflight_pi.py            # pre-bringup gates (the default)
    ./preflight_pi.py --stage post
    ./preflight_pi.py --stage pre --skip chassis_alive

Exit 0 only if every gate in the stage passed. Any FAIL exits 1; a gate that could
not be evaluated is UNKNOWN and also exits 1, because "could not tell" is not "fine".
"""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

PASS, FAIL, UNKNOWN, SKIP = "PASS", "FAIL", "UNKNOWN", "SKIP"

SERIAL_PORT = "/dev/ttyAMA0"

#: The LIVE workspace. Determined from the robot rather than assumed: `~/.bashrc`
#: sources `~/ros2_ws/install/setup.bash` and `~/start_cov.sh` runs out of the same
#: tree, while `~/ros2_ws_mission_stack` is a July leftover. The first draft of this
#: script guessed `~/rvr_ws`, which does not exist -- a hardcoded path is a claim like
#: any other, so this one names its evidence and `--ws` overrides it.
DEFAULT_WS_ROOT = Path(os.path.expanduser("~/ros2_ws"))
WS_ROOT = DEFAULT_WS_ROOT
SRC_TREE = WS_ROOT / "src" / "sphero_rvr_ros"
INSTALL_TREE = WS_ROOT / "install"


def _set_workspace(root: Path):
    """Point the install-verification gate at a workspace."""
    global WS_ROOT, SRC_TREE, INSTALL_TREE
    WS_ROOT = Path(os.path.expanduser(str(root)))
    SRC_TREE = WS_ROOT / "src" / "sphero_rvr_ros"
    INSTALL_TREE = WS_ROOT / "install"


@dataclass
class Result:
    verdict: str
    detail: str = ""
    #: What the operator should DO. A gate that fails without saying what to try next
    #: is a gate that gets ignored on the third run.
    remedy: str = ""
    elapsed_s: float = 0.0


@dataclass
class Gate:
    name: str
    stage: str
    run: Callable[[], Result]
    why: str = ""
    result: Optional[Result] = field(default=None)


# --------------------------------------------------------------------------
# process helpers -- the pkill / child-pid disciplines live here, once
# --------------------------------------------------------------------------

def sh(cmd, timeout=10.0):
    """Run a command, never raise, return (rc, stdout+stderr)."""
    try:
        proc = subprocess.run(
            cmd, shell=isinstance(cmd, str), capture_output=True, text=True,
            timeout=timeout,
        )
        return proc.returncode, (proc.stdout or "") + (proc.stderr or "")
    except subprocess.TimeoutExpired:
        return 124, f"timed out after {timeout}s"
    except Exception as exc:                       # noqa: BLE001
        return 127, str(exc)


def _own_process_tree():
    """This process and every ancestor, so a kill list can never include us.

    The `pkill -f` trap in a different form: any PID collection that does not
    explicitly exclude the caller's own chain can terminate the SSH session running
    the preflight.
    """
    own = set()
    pid = os.getpid()
    for _ in range(12):                            # bounded: no cycles, no surprises
        if pid <= 1:
            break
        own.add(pid)
        rc, out = sh(["ps", "-o", "ppid=", "-p", str(pid)], timeout=5)
        if rc != 0 or not out.strip().isdigit():
            break
        pid = int(out.strip())
    return own


def port_holders(port=SERIAL_PORT):
    """PIDs holding the serial device, via fuser. Empty list means genuinely free.

    `fuser` is the authority here rather than a PID check, because the failure this
    guards against is precisely a process that is gone from one view and still
    holding the device in another.
    """
    rc, out = sh(["fuser", port], timeout=8)
    if rc not in (0, 1):
        return None                                # could not tell
    return [int(tok) for tok in out.split() if tok.isdigit()]


def kill_pids(pids, label=""):
    """TERM then KILL, by explicit PID, never by pattern."""
    own = _own_process_tree()
    targets = [p for p in pids if p not in own and p > 1]
    for pid in targets:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except PermissionError:
            pass
    if not targets:
        return targets
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        alive = [p for p in targets if _alive(p)]
        if not alive:
            return targets
        time.sleep(0.2)
    for pid in [p for p in targets if _alive(p)]:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    return targets


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def ros_nodes(timeout=12.0):
    """`ros2 node list` as a list, or None if the command could not run.

    Deliberately NOT `ps`: the graph is the question, and a node can be present in
    `ps` while absent from the graph and vice versa. Trusting `ps` here is the class
    of error that has produced wrong verdicts before.
    """
    rc, out = sh("ros2 node list", timeout=timeout)
    if rc != 0:
        return None
    return [line.strip() for line in out.splitlines() if line.strip().startswith("/")]


def topic_once(topic, timeout=10.0):
    """One message from a topic as text, or None. `--full-length` because
    `ros2 topic echo` TRUNCATES arrays, and a truncated scan is a scan that cannot
    answer an occlusion question."""
    rc, out = sh(
        f"ros2 topic echo --once --full-length {shlex.quote(topic)}", timeout=timeout)
    if rc != 0 or not out.strip():
        return None
    return out


# --------------------------------------------------------------------------
# PRE-BRINGUP gates
# --------------------------------------------------------------------------

def gate_graph_empty():
    """Nothing may already be on the graph.

    Leftover nodes from a previous run or an aborted test suite hold devices, publish
    stale topics, and make every later gate ambiguous. Fake Nav2 servers have
    outlived a pytest run and stayed on the graph long after the suite finished.
    """
    nodes = ros_nodes()
    if nodes is None:
        return Result(UNKNOWN, "`ros2 node list` did not run",
                      remedy="source the workspace setup.bash, then retry")
    live = [n for n in nodes if n not in ("/rosout",)]
    if live:
        return Result(FAIL, f"{len(live)} node(s) already on the graph: {live[:6]}",
                      remedy="tear the stack down and re-check; do NOT pkill -f, it "
                             "matches your own ssh command line")
    return Result(PASS, "graph is empty")


def gate_serial_port_free():
    """Nothing may be holding the chassis serial device.

    Runs BEFORE the chassis probe, because probing while another process owns the
    port produces serial timeouts -- which is exactly what a dead chassis looks like.
    A false "chassis is off" verdict sends the operator to the power switch while the
    real fault is an orphan from the last run.
    """
    holders = port_holders()
    if holders is None:
        return Result(UNKNOWN, f"fuser could not report on {SERIAL_PORT}",
                      remedy="check the device exists and fuser is installed")
    if holders:
        return Result(FAIL, f"{SERIAL_PORT} held by PIDs {holders}",
                      remedy="kill those PIDs explicitly, then re-run; a wrapper that "
                             "looks dead can still have a live child on the port")
    return Result(PASS, f"{SERIAL_PORT} is free")


def gate_installed_tree_matches():
    """The installed tree must match the source tree, file by file.

    This check earned its place twice in one day. `colcon build --symlink-install`
    makes most of it symlinks, so a mismatch means a genuinely stale copy -- which is
    the case that has flown twice.
    """
    if not SRC_TREE.exists():
        return Result(UNKNOWN, f"source tree not found at {SRC_TREE}",
                      remedy="adjust WS_ROOT at the top of this script")
    if not INSTALL_TREE.exists():
        return Result(FAIL, f"no install tree at {INSTALL_TREE}",
                      remedy="colcon build --symlink-install")

    mismatched, checked, missing = [], 0, []
    for src in sorted((SRC_TREE / "src").rglob("*.py")):
        rel = src.relative_to(SRC_TREE / "src")
        matches = list(INSTALL_TREE.rglob(str(rel)))
        if not matches:
            missing.append(str(rel))
            continue
        checked += 1
        want = hashlib.sha256(src.read_bytes()).hexdigest()
        for installed in matches:
            try:
                got = hashlib.sha256(installed.read_bytes()).hexdigest()
            except OSError:
                mismatched.append(f"{rel} (unreadable)")
                continue
            if got != want:
                mismatched.append(str(rel))
                break

    if mismatched:
        return Result(FAIL,
                      f"{len(mismatched)} of {checked} installed file(s) differ: "
                      f"{mismatched[:5]}",
                      remedy="colcon build --symlink-install, then re-run this gate; "
                             "a stale install is code you are not testing")
    if missing:
        return Result(FAIL, f"{len(missing)} source file(s) not installed at all: "
                            f"{missing[:5]}",
                      remedy="colcon build --symlink-install")
    return Result(PASS, f"{checked} installed file(s) match source")


def gate_chassis_alive():
    """FIRST GATE, and the one this whole script was written around.

    The chassis being off is invisible from everywhere except the serial link, and
    every downstream layer fails in a way that looks like its own bug. This starts
    the driver briefly, asks whether real data came back, and then TEARS DOWN
    PROPERLY -- which is the part that went wrong last time.

    The teardown is the delicate half. `ros2 run` spawns the node as a CHILD, so the
    wrapper's PID is not the node's; killing the wrapper alone leaves the node
    holding the port. Here the whole process GROUP is signalled, and then the verdict
    is taken from `fuser` -- the PORT being free -- rather than from any PID being
    gone.
    """
    holders = port_holders()
    if holders:
        return Result(FAIL, f"{SERIAL_PORT} already held by {holders}; cannot probe",
                      remedy="run the serial_port_free gate first and clear it")

    proc = None
    try:
        proc = subprocess.Popen(
            ["ros2", "run", "sphero_rvr_driver", "rvr_node"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            start_new_session=True,          # its own group, so teardown is precise
        )
        deadline = time.monotonic() + 20.0
        odom = None
        while time.monotonic() < deadline and odom is None:
            time.sleep(2.0)
            odom = topic_once("/odom", timeout=6.0)
        if odom is None:
            return Result(
                FAIL, "no /odom within 20 s of starting the driver",
                remedy="THE CHASSIS IS PROBABLY OFF (or asleep) -- check the power "
                       "switch and the battery before debugging anything downstream; "
                       "odom, the SLAM anchor, the map and every goal cascade from "
                       "this one fact and each fails looking like its own problem")
        return Result(PASS, "driver produced /odom")
    except FileNotFoundError:
        return Result(UNKNOWN, "`ros2` not on PATH",
                      remedy="source the workspace setup.bash")
    finally:
        if proc is not None:
            # The GROUP, not the wrapper: the node is a child and outlives its parent.
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass


def gate_port_free_after_probe():
    """THE PROBE MUST NOT LEAVE AN ORPHAN. Run immediately after chassis_alive.

    A previous version of the chassis probe orphaned an `rvr_node` child holding
    /dev/ttyAMA0. The result was 874 "dispatcher reader failed" and no odom -- a
    state indistinguishable from a dead chassis. The probe nearly manufactured the
    symptom it exists to detect, so the teardown gets its own gate rather than being
    assumed to have worked.
    """
    deadline = time.monotonic() + 8.0
    holders = port_holders()
    while holders and time.monotonic() < deadline:
        time.sleep(0.5)
        holders = port_holders()
    if holders is None:
        return Result(UNKNOWN, "fuser could not report")
    if holders:
        kill_pids(holders, "orphaned driver")
        time.sleep(1.0)
        still = port_holders()
        if still:
            return Result(FAIL, f"{SERIAL_PORT} STILL held by {still} after cleanup",
                          remedy="kill those PIDs by hand; the next bringup will fail "
                                 "with serial timeouts that look like a dead chassis")
        return Result(PASS, "probe left an orphan; it was cleaned up by explicit PID")
    return Result(PASS, "probe left the port free")


# --------------------------------------------------------------------------
# POST-BRINGUP gates
# --------------------------------------------------------------------------

def gate_lidar_not_occluded():
    """A quick sanity check that the lidar is seeing a room, not a bag or a wall.

    Deliberately crude: this is not a calibration, it is the two-second question
    "is anything obviously wrong with the scan" that bench runs at desk height have
    repeatedly answered too late.
    """
    text = topic_once("/scan", timeout=15.0)
    if text is None:
        return Result(UNKNOWN, "no /scan message",
                      remedy="is the lidar node running and the motor spinning?")
    match = re.search(r"ranges:\s*\n((?:\s*-\s*[^\n]+\n)+)", text)
    if not match:
        return Result(UNKNOWN, "could not parse ranges from /scan")
    values = []
    for token in re.findall(r"-\s*([0-9.eE+-]+|\.inf|inf|nan)", match.group(1)):
        try:
            values.append(float(token))
        except ValueError:
            values.append(float("nan"))
    if not values:
        return Result(UNKNOWN, "no range values parsed")
    finite = [v for v in values if v == v and v not in (float("inf"),)]
    frac = len(finite) / len(values)
    if frac < 0.20:
        return Result(FAIL, f"only {frac:.0%} of {len(values)} rays returned a range",
                      remedy="lidar may be occluded, or the room is very large; look "
                             "at the mount before trusting any navigation")
    near = [v for v in finite if v < 0.15]
    if len(near) > 0.5 * len(finite):
        return Result(FAIL, f"{len(near)}/{len(finite)} returns are under 0.15 m",
                      remedy="something is against the lidar -- a bag, a cable, or "
                             "the rover is nose-in against a wall")
    return Result(PASS, f"{frac:.0%} of rays returned; {len(near)} closer than 0.15 m")


def gate_tof_state():
    """GATE ON THE ROBOT'S OWN WORDS. This is rule 6, and it has already saved a run.

    `/tof/state` once reported `rule_b=UNPINNED margin_m=0.1 rules=rule_a_only` while
    the pinned margin sat in the config and every test was green -- because the node
    declared its own literal parameter defaults, which overrode the config. One gate
    away from flying "the first rule-B mission" with rule B off. So this reads what
    the sensor SAYS about itself and refuses to consult the YAML at all.
    """
    text = topic_once("/tof/state", timeout=12.0)
    if text is None:
        return Result(FAIL, "no /tof/state message",
                      remedy="tof node is not running -- the sub-lidar brake FAILS "
                             "OPEN with no producer, so the stack believes it has a "
                             "brake it does not have. It is in explore.launch.py as "
                             "start_tof (default true).")
    fields = dict(re.findall(r"(\w+)=([^\s']+)", text))
    problems = []
    if fields.get("rules") != "rule_a+b":
        problems.append(f"rules={fields.get('rules')}")
    if fields.get("rule_b") != "pinned":
        problems.append(f"rule_b={fields.get('rule_b')}")
    if fields.get("i2c_errors") not in (None, "0"):
        problems.append(f"i2c_errors={fields.get('i2c_errors')}")
    rate = fields.get("rate_hz")
    if rate in (None, "None"):
        problems.append(f"rate_hz={rate} (window empty -- sensor may have stopped; "
                        f"uptime_s={fields.get('uptime_s')})")
    else:
        try:
            if float(rate) < 5.0:
                problems.append(f"rate_hz={rate} below band")
        except ValueError:
            problems.append(f"rate_hz={rate} unparseable")
    if problems:
        return Result(FAIL, "; ".join(problems),
                      remedy="the sensor's own state line disagrees with what this "
                             "run needs; do not fix it by editing the YAML -- check "
                             "what the node actually declared")
    return Result(PASS, f"rules={fields.get('rules')} rule_b={fields.get('rule_b')} "
                        f"rate_hz={rate} i2c_errors={fields.get('i2c_errors', '0')}")


def gate_supervisor_latch_cleared():
    """clear_estop BEFORE reset. That ordering was written down nowhere.

    `reset` refuses while the estop is latched, so calling it first produces a
    plausible-looking failure that sends the operator looking in the wrong place. It
    was discovered live, cost part of a session, and lives here now.
    """
    rc1, out1 = sh("ros2 service call /collision_stop/clear_estop std_srvs/srv/Trigger",
                   timeout=20.0)
    if rc1 != 0:
        return Result(UNKNOWN, "clear_estop call failed",
                      remedy="is collision_stop running?")
    rc2, out2 = sh("ros2 service call /collision_stop/reset std_srvs/srv/Trigger",
                   timeout=20.0)
    if rc2 != 0:
        return Result(UNKNOWN, "reset call failed")
    if "success=True" not in out2.replace(" ", ""):
        return Result(FAIL, f"reset refused: {out2.strip()[-160:]}",
                      remedy="clear_estop must succeed BEFORE reset; if it did and "
                             "reset still refuses, the supervisor sees a live "
                             "obstacle and is right to refuse")
    return Result(PASS, "clear_estop then reset, in that order, both accepted")


# --------------------------------------------------------------------------

GATES = [
    Gate("graph_empty", "pre", gate_graph_empty,
         "leftover nodes hold devices and make every later gate ambiguous"),
    Gate("serial_port_free", "pre", gate_serial_port_free,
         "a held port produces serial timeouts that read as a dead chassis"),
    Gate("installed_tree_matches", "pre", gate_installed_tree_matches,
         "a stale install is code you are not testing; caught twice in one day"),
    Gate("chassis_alive", "pre", gate_chassis_alive,
         "FIRST: everything downstream cascades from this one fact"),
    Gate("port_free_after_probe", "pre", gate_port_free_after_probe,
         "the probe must not leave the orphan it exists to detect"),
    Gate("lidar_not_occluded", "post", gate_lidar_not_occluded,
         "a bagged lidar looks like an empty room"),
    Gate("tof_state", "post", gate_tof_state,
         "the sensor's own words, never the YAML"),
    Gate("supervisor_latch_cleared", "post", gate_supervisor_latch_cleared,
         "clear_estop BEFORE reset -- written down nowhere until now"),
]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--stage", choices=("pre", "post"), default="pre",
                        help="pre-bringup gates (default) or post-bringup gates")
    parser.add_argument("--skip", action="append", default=[],
                        help="gate name to skip; repeatable")
    parser.add_argument("--ws", default=str(DEFAULT_WS_ROOT),
                        help="colcon workspace root for the install-verify gate")
    args = parser.parse_args(argv)
    _set_workspace(Path(args.ws))

    selected = [g for g in GATES if g.stage == args.stage]
    started = time.monotonic()

    print(f"PREFLIGHT stage={args.stage}  gates={len(selected)}")
    print("=" * 78)
    for gate in selected:
        if gate.name in args.skip:
            gate.result = Result(SKIP, "skipped by request")
            continue
        t0 = time.monotonic()
        try:
            gate.result = gate.run()
        except Exception as exc:                   # noqa: BLE001
            gate.result = Result(UNKNOWN, f"gate raised: {exc}")
        gate.result.elapsed_s = time.monotonic() - t0
        print(f"{gate.result.verdict:<8} {gate.name:<24} "
              f"{gate.result.elapsed_s:5.1f}s  {gate.result.detail}")
        if gate.result.verdict in (FAIL, UNKNOWN) and gate.result.remedy:
            print(f"{'':<8} {'':<24} {'':>5}   -> {gate.result.remedy}")

    print("=" * 78)
    elapsed = time.monotonic() - started
    counts = {}
    for gate in selected:
        counts[gate.result.verdict] = counts.get(gate.result.verdict, 0) + 1
    summary = "  ".join(f"{k}={v}" for k, v in sorted(counts.items()))
    print(f"{summary}   total {elapsed:.1f}s"
          + ("" if elapsed <= 120 else "   [OVER the 2-minute target]"))

    bad = [g.name for g in selected if g.result.verdict in (FAIL, UNKNOWN)]
    if bad:
        # UNKNOWN exits non-zero too: "could not tell" is not "fine", and a preflight
        # that shrugs is a preflight that gets believed.
        print(f"NOT CLEARED FOR BRINGUP: {bad}")
        return 1
    print("CLEARED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
