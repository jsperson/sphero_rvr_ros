"""Bench-prove every ladder rung against the REAL supervisor. No ROS, no motor.

The stall survival ladder (docs/stall_survival_ladder.md) is built on a claim: when
one escape is refused, another will be granted. That claim was measured at ONE pose
from run 190528, using a scan populated in only four sectors -- and I flagged in the
design note that such a scan reads FREE at every bearing between those sectors, so
any verdict that comes from the supervisor's projected-trajectory gate is optimistic.

This probe is that flag being cashed in. It builds scans with returns at the bearings
a four-sector scan cannot express -- specifically the swept-circle CORNERS at
+/-42.3 and +/-148.0 deg, which is exactly the 60 deg of unread arc the pre-D19 pivot
gate missed -- and asks the real `CollisionStopSupervisor` what it would grant.

Two parts:

  PART A  static grant/refuse table: every rung x every scenario x every LATCH state.
          Latch matters because the D25 fix made escape grants latch-dependent: the
          reverse-escape branch fires only under `front_stop`, so an identical
          command can be granted or refused depending on how the supervisor was
          latched. A rung that is refused in EVERY scenario is not a rung, and this
          table is how that gets found before a room finds it.

  PART B  closed loop: the real `StallLadder` proposing, the real supervisor ruling,
          the ladder observing what actually came out and escalating. Answers the
          only question that matters -- does the rover get out?

Run:  python3 diagnostics/ladder_rung_probe.py [--json out.json]
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sphero_rvr_core.stall_ladder import (  # noqa: E402
    RUNG_ORDER, LadderConfig, StallLadder,
)
from sphero_rvr_driver.collision_stop import (  # noqa: E402
    CollisionStopConfig, CollisionStopSupervisor, ScanInput, Transform2D,
    TwistCommand,
)

def deployed_config():
    """Build the config the ROBOT runs, from config/collision_stop.yaml.

    NOT CollisionStopConfig() -- the dataclass defaults differ from the deployed
    YAML in 13 fields, and several of them decide this probe's verdicts outright:
    footprint_front_m is 0.11 deployed vs 0.22 default (the padding that was halved),
    measured_stop_time_s 0.25 vs 0.5 (which halves the trajectory horizon and so
    changes what the gate blocks), min_forward_scale 0.7 vs 0.0, payload_margin_m
    0.02 vs 0.05, and reset_policy auto_after_clear vs manual (which decides whether
    a latch clears itself at all).

    A probe run against dataclass defaults produces a confident table describing a
    robot that does not exist. Same failure as the four-sector scan: plausible,
    self-consistent, and about the wrong machine.
    """
    import yaml
    here = os.path.dirname(__file__)
    raw = yaml.safe_load(open(os.path.join(here, "..", "config", "collision_stop.yaml")))

    def find(d):
        for _k, v in (d or {}).items():
            if isinstance(v, dict):
                if "ros__parameters" in v:
                    return v["ros__parameters"]
                found = find(v)
                if found:
                    return found
        return None

    params = find(raw) or {}
    fields = CollisionStopConfig.__dataclass_fields__
    kwargs = {k: v for k, v in params.items() if k in fields}
    return CollisionStopConfig(**kwargs)


CFG = deployed_config()
COUNT = 360
NOW = 5.0

# The corner bearings. A pivot sweeps the footprint's circumscribed circle, whose
# corners sit here -- and a four-sector scan (front/rear/left/right) leaves every one
# of them unread.
CORNERS = (42.3, -42.3, 148.0, -148.0)


def scan(points=(), default=6.0, stamp=NOW):
    """Build a scan. `points` is ((bearing_deg, range_m), ...) in the BASE frame.

    Everything unspecified reads `default` (open). Bearings are placed by index, so
    arbitrary angles are expressible -- which is the entire point of this probe.
    """
    ranges = [default] * COUNT
    inc = 2.0 * math.pi / COUNT
    amin = -math.pi
    for bearing_deg, r in points:
        idx = int(round((math.radians(bearing_deg) - amin) / inc)) % COUNT
        ranges[idx] = r
        # One ray is a glint; obstacles subtend width. Three cells keeps the return
        # honest without smearing it across a sector.
        ranges[(idx - 1) % COUNT] = r
        ranges[(idx + 1) % COUNT] = r
    return ScanInput(
        ranges=tuple(ranges), angle_min=amin, angle_increment=inc,
        range_min=0.05, range_max=8.0, stamp=stamp, received_at=stamp,
        frame_id="laser", transform_to_base=Transform2D(),
    )


def _arc(bearing_deg, r, half_width_deg=25.0, step=2.0):
    n = int(half_width_deg / step)
    return tuple((bearing_deg + i * step, r)
                 for i in range(-n, n + 1))


SCENARIOS = {
    # Wide open: the control. Everything must be granted here or the probe is wrong.
    "open": (),
    # Run 190528's abort pose, where the mission actually died.
    "rear_blocked_190528": _arc(180.0, 0.228) + _arc(0.0, 0.583, 20.0)
                           + _arc(90.0, 0.437, 20.0) + _arc(-90.0, 0.403, 20.0),
    # Run 185048's pocket: 0.22 m on two sides, the pose nothing could leave.
    "corner_pocket_185048": _arc(180.0, 0.221) + _arc(90.0, 0.220)
                            + _arc(0.0, 0.475, 20.0) + _arc(-90.0, 1.158, 20.0),
    # THE PREDICTION. Sides and front/rear read clear in a four-sector view, but the
    # swept circle a pivot traces is occupied at its corners.
    "swept_circle_corners": tuple((b, 0.20) for b in CORNERS),
    # Same, further out: inside the corner radius+margin but not by much.
    "swept_circle_corners_marginal": tuple((b, 0.26) for b in CORNERS),
    # Boxed on three sides, open forward only -- the case where rung 4 is the ONLY
    # escape and every retreat must be refused.
    "open_forward_only": _arc(180.0, 0.20) + _arc(90.0, 0.20) + _arc(-90.0, 0.20),
}

LATCHES = ("none", "front_stop", "operator_stop", "non_finite")

# What the decisive controller asks for when it is simply driving to a goal.
NOMINAL_DRIVE_MPS = 0.10


_SCAN_PTS = {}


def _pts_of(sc):
    return _SCAN_PTS[id(sc)]


def make_supervisor(sc, latch):
    # `now` is a FLOAT here, not a clock callable -- it seeds _started_at and
    # is used in arithmetic. Passing a lambda works right up until a code path
    # touches the startup-grace comparison, which is a fine way to get a probe
    # that looks correct on the happy path and dies on the interesting one.
    # STAMPS MUST ADVANCE. Feeding two scans with the SAME stamp trips
    # ScanStampTracker's non_advancing_scan_stamp health check, and the supervisor
    # then reports SENSOR_STALE for everything -- which silently turned the entire
    # front_stop column of this table into "refused" for a reason that had nothing to
    # do with latching. A probe that manufactures its own failure is worse than no
    # probe, so each step here gets its own timestamp.
    sup = CollisionStopSupervisor(CFG, now=0.0)
    t = NOW
    sup.update_scan(scan(_pts_of(sc), stamp=t), now=t)
    if latch == "front_stop":
        # Latch by driving at a genuinely blocking obstacle, then restore the real
        # scan: the latch persists, which is precisely the state under test.
        t += 0.1
        sup.update_scan(scan(_arc(0.0, 0.10, 30.0), stamp=t), now=t)
        sup.apply_command(TwistCommand(0.15, 0.0), now=t)
        t += 0.1
        sup.update_scan(scan(_pts_of(sc), stamp=t), now=t)
    elif latch == "operator_stop":
        sup.stop(now=t)
    elif latch == "non_finite":
        sup.apply_command(TwistCommand(float("nan"), 0.0), now=t)
    return sup, t


def rung_commands(cfg=LadderConfig(), open_bearing=1.0):
    """The exact commands the ladder emits, taken from the ladder itself."""
    out = {}
    for i, rung in enumerate(RUNG_ORDER):
        ladder = StallLadder(cfg)
        ladder._rung_index = i          # probe the rung directly, not via 12 s of clock
        out[rung] = ladder._rung_command(open_bearing)
    return out


def part_a():
    print("=" * 78)
    print("PART A — grant/refuse per rung, per scenario, per latch state")
    print("=" * 78)
    cmds = rung_commands()
    print("rung commands from the real ladder: "
          + ", ".join(f"{k}=({v[0]:+.2f},{v[1]:+.2f})" for k, v in cmds.items()))
    findings, rows = [], []
    for name, pts in SCENARIOS.items():
        sc = scan(pts)
        _SCAN_PTS[id(sc)] = pts
        print(f"\n--- {name} ---")
        print("  %-18s %-14s %-9s %-26s %s"
              % ("rung", "latch", "state", "reason", "output"))
        for rung, (vx, wz) in cmds.items():
            granted_any = False
            for latch in LATCHES:
                sup, t = make_supervisor(sc, latch)
                d = sup.apply_command(TwistCommand(vx, wz), now=t)
                moves = (abs(d.output.linear_x) > 1e-9
                         or abs(d.output.angular_z) > 1e-9)
                granted_any = granted_any or moves
                rows.append(dict(scenario=name, rung=rung, latch=latch,
                                 state=d.state.name, reason=d.reason,
                                 out=[d.output.linear_x, d.output.angular_z],
                                 granted=moves))
                print("  %-18s %-14s %-9s %-26s (%+.3f,%+.3f) %s"
                      % (rung, latch, d.state.name, d.reason,
                         d.output.linear_x, d.output.angular_z,
                         "GRANT" if moves else "refuse"))
            if not granted_any:
                findings.append((name, rung))
    return rows, findings


def part_b():
    print("\n" + "=" * 78)
    print("PART B — closed loop: does the ladder actually get out?")
    print("=" * 78)
    hz, cfg = 20.0, LadderConfig()
    results = {}
    for name, pts in SCENARIOS.items():
        sc = scan(pts)
        _SCAN_PTS[id(sc)] = pts
        for latch in LATCHES:
            sup, t0 = make_supervisor(sc, latch)
            ladder = StallLadder(cfg)
            x = y = yaw = 0.0
            seen, escaped, exhausted = [], False, False
            for i in range(int(60 * hz)):
                t = t0 + 0.1 + i / hz
                sup.update_scan(scan(pts, stamp=t), now=t)
                res = ladder.step(x=x, y=y, yaw=yaw, now=t,
                                  commanding=True,
                                  output_moving=getattr(ladder, "_last_moved", False),
                                  open_bearing_rad=1.0)
                if res.exhausted:
                    exhausted = True
                    break
                if res.rung and res.rung not in seen:
                    seen.append(res.rung)
                # When the ladder is NOT running a rung, the controller is still
                # driving toward its goal -- so the harness must command the nominal
                # drive, not zero. Commanding zero here made the OPEN scenario
                # "stall" and exhaust its budget, which measured the harness rather
                # than the ladder. The rover only fails to move when the SUPERVISOR
                # refuses, which is the condition under test.
                vx, wz = ((res.linear_x, res.angular_z) if res.action == "rung"
                          else (NOMINAL_DRIVE_MPS, 0.0))
                d = sup.apply_command(TwistCommand(vx, wz), now=t)
                moved = (abs(d.output.linear_x) > 1e-9
                         or abs(d.output.angular_z) > 1e-9)
                ladder._last_moved = moved
                # Integrate what the SUPERVISOR permitted, not what we asked for.
                yaw += d.output.angular_z / hz
                x += d.output.linear_x * math.cos(yaw) / hz
                y += d.output.linear_x * math.sin(yaw) / hz
                if math.hypot(x, y) >= 0.25 or abs(yaw) >= 0.8:
                    escaped = True
                    break
            verdict = "ESCAPED" if escaped else ("exhausted" if exhausted else "ran out of time")
            results[(name, latch)] = verdict
            print("  %-32s %-14s -> %-15s via %s"
                  % (name, latch, verdict, "->".join(seen) or "(none)"))
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json")
    args = ap.parse_args()
    rows, findings = part_a()
    loop = part_b()
    print("\n" + "=" * 78)
    print("FINDINGS")
    print("=" * 78)
    if findings:
        for name, rung in findings:
            print(f"  ** {rung} refused in EVERY latch state under '{name}'")
    else:
        print("  no rung was refused across all latch states in any scenario")
    stuck = [k for k, v in loop.items() if v != "ESCAPED"]
    print(f"\n  closed loop: {len(loop) - len(stuck)}/{len(loop)} scenario-latch "
          f"combinations escaped")
    for k in stuck:
        print(f"  ** NO ESCAPE: {k[0]} / latch={k[1]} -> {loop[k]}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"table": rows,
                       "closed_loop": {f"{k[0]}|{k[1]}": v for k, v in loop.items()}},
                      f, indent=2)
        print(f"\n  artifacts -> {args.json}")


if __name__ == "__main__":
    main()
