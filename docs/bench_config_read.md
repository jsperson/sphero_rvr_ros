# The bench config read — turning "the file says X" into "the robot says X"

**Cost: one launch, about twenty seconds, no chassis, no lidar, no rover, no Scott.**

## Why this exists, and what not having it cost

On 2026-07-23 `57e26be` added launch-argument overrides for the supervisor's
forward slow corridor. On 2026-08-02 `4bb920d` narrowed that corridor in
`config/collision_stop.yaml` from ±45° to ±35° — a deliberate safety tune, commit
message "trim timid brake". **It did nothing.** ROS 2 launch lets the later
`parameters=` entry win, so the node kept taking ±45 from the launch default and
the YAML's ±35 was never in effect.

**It stayed that way for 23 days, and nothing in the project could have revealed
it**, because every check we had read the *file*. The tests read the YAML. The
config-derivation guards read the YAML. The run card quoted the YAML. **Not one of
them asked the running node what it was actually using.**

That is standards rule 6 — *a config file is a claim; the robot's state line is the
robot* — and this document is the cheapest possible way to obey it.

## The read

```bash
ros2 launch sphero_rvr_driver supervised_rvr.launch.py \
     serial_port:=SIMULATED_CHASSIS_NOT_A_REAL_ROBOT
```

`rvr_node` accepts `SIMULATED_CHASSIS_PORT = "SIMULATED_CHASSIS_NOT_A_REAL_ROBOT"`
and talks to nothing. **This is the real flight launch file** — the same one a
mission uses — so what it produces is what a flight would produce. Then:

```bash
ros2 topic echo --once --full-length /collision_stop/state
```

Read the fields off the line. Any disagreement with the deployed YAML is a
shadowed parameter, and the run does not proceed until it is explained.

**Known-good as of `44c2654` (2026-08-31, read from a live node):**

```
front_slow_min_angle_deg=-35.0  front_slow_max_angle_deg=35.0
stop_distance_m=0.3             slow_distance_m=0.5
```

## Three things that make this trustworthy, and one that does not

* **It runs the file that gets flown.** `supervised_rvr.launch.py`, unmodified.
* **It needs no hardware.** The simulated chassis port means no serial, no motion,
  and nothing publishes `/cmd_vel`, so nothing can move even in principle.
* **It reads the node, not the file.** The state line is published by the deployed
  binary using the parameters it actually resolved.

**AND THE ONE THAT DOES NOT — READ THIS BEFORE SUBSTITUTING THE RIG.** The obvious
shortcut is to read the corridor off `sim_closed_loop.launch.py`, which is already
up for other reasons. **That check cannot fail and therefore proves nothing.** The
rig declares its own supervisor node with `parameters=[collision_stop.yaml]` — the
YAML alone, no overrides, and it never had any. It would have reported −35.0 on
2026-08-24, with the defect fully present on the flight path. Verified against the
pre-fix tree:

```
$ git show 44c2654^:launch/sim_closed_loop.launch.py
        parameters=[str(share / "config" / "collision_stop.yaml")],   # identical
```

**An instrument that cannot show the failure cannot confirm the fix.** The rig is
the wrong population for this question; only the flight launch path will do.

## Where it belongs

**In the standard pre-flight, not on the field card.** A field card is read when
Scott is standing over a staged rover; by then the config has been wrong for
however long it has been wrong. This costs seconds on a bench with nobody waiting.

**The obvious next step, deliberately not taken here:** make it a gate in
`scripts/preflight_pi.py` that diffs every field the state line publishes against
the deployed YAML and fails on any disagreement. That is a code change and wants
its own review round — but note what the 23 days argue: **a rule that lives only
in a document gets re-learned by everyone who has not read it.**
