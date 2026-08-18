# CERTIFICATION: the `max_angular_accel` fix, chassis-off closed-loop proof

**2026-08-17. Branch `prototype/stock-middle`. Chassis never powered; nothing here was
tested on wheels.** This document states exactly what is proven, exactly what is not, and
the two known behaviours that may appear in the field anyway.

## The fix

```
max_angular_accel   1.20 -> 80.0   (controller_server / FollowPath)
rotational_acc_lim  1.20 -> 80.0   (behavior_server, Spin has the identical defect)
```

Derived, not chosen: RPP clamps its rotation command to an acceleration window measured
from the **current speed reported by odom**. At 1.20 the first command from rest is
`1.20 × 0.05 = 0.060 rad/s` — and §3a measured exactly `cmd_wz = 0.06`. Every sub-floor
command is raised by the driver to floor duty and executed at 3.55 rad/s, so the robot
snaps while the controller believes it is easing. Requirement:
`accel × dt ≥ minimum_clean_rate(pivot_min_duty) = 3.55 / 0.05 = 71.0`. Set 80.0.

## THE THREE-CLAUSE CLAIM

**(a) LEVEL 1 — absolute, adapter-guaranteed, unconditional.**
**No sub-floor duty ever reaches the wheels.** `plan_pivot` returns either 0 or a duty at
or above `pivot_min_duty`; there is no third outcome. This holds whatever any config says
and whatever any controller asks for. Proven by unit test and by the closed-loop walk-band
tripwire. **It does not depend on this fix at all** — it is why the broken config remained
driveable.

**(b) GENERATION ELIMINATED where no sign reversal occurs — demonstrated.**
Path-verified 180°-turn goal, same rig, dynamics frozen, only the acceleration limits
differing:

| | broken 1.20 | fixed 80.0 |
|---|---|---|
| goal outcome | SUCCEEDED | SUCCEEDED |
| sub-floor rotation commands | **100 % (8)** | **0 % (0)** |
| longest consecutive train | **8** | **0** |

**(c) SUSTAINED TRAINS ARE BOUNDED AND SELF-TERMINATING — by mechanism, for any
scenario.** This is the load-bearing clause, and the arithmetic is below rather than
asserted.

A reverse command is clamped to `m − W` where `m` is the measured speed and
`W = accel × dt`. It is sub-floor while `|m − W| < F` (F = 3.55), i.e. while `m > W − F`.
Meanwhile the adapter executes full reverse regardless, so `m` decays toward `−F` with the
fitted `τ = 0.22 s`. The train therefore **always terminates**, and its length is:

```
accel 1.20  ->  W = 0.06  ->  sub-floor while m > -3.49  ->  21.0 cycles  (1.05 s)
accel 80.0  ->  W = 4.00  ->  sub-floor while m >  0.45  ->   2.5 cycles  (0.13 s)
accel 142   ->  W = 7.10  ->  sub-floor never              ->   0 cycles
```

**The fix shortens the worst-case reversal train roughly 8×, from ~21 cycles to ~2.5.**
The 21-cycle figure at 1.20 is what a sustained train looks like, and it matches the
field: §3a showed 35 sub-floor commands of 64, clustered post-reversal.

**Correction to a stronger claim that was proposed and does not hold:** this bound is
**not** "2 cycles by pure arithmetic, everywhere". The observed 2 came from a
*stop-then-reverse*, where the measured speed had already decayed. A **direct** full-speed
reversal takes ~2.5 cycles, and the number depends on `τ` — a **fitted** quantity, not a
pure constant. What *is* structural is **self-termination**: the adapter forces full-rate
reversal, which drives the measured speed through the threshold, so no train can sustain.
At 1.20 the window is so narrow (0.06) that it never catches up, which is precisely why
the field's trains sustained.

## WHAT IS NOT CERTIFIED

**That no other hunt-like behaviour exists at 80.** No sim scenario was produced in which
the broken config hunts and the fixed one does not — the one scenario that showed severity
(90°, 1.0 m: broken 80 % sub-floor, economy 610 °/m) is one where **both** arms stall on
the final approach, so it could not discriminate. Behavioural dominance (yaw economy,
alternation) is **not demonstrated**: on the clean scenario the broken config also
succeeds, because Level 1 protects it.

**The field is the instrument for that**, and this rig's job was to make the field flight a
confirmation rather than an experiment.

Also not certified, by construction: **arcs** (ideal kinematics, unmeasured regime —
`docs/run_card_arc_rate_FUTURE.md`), collision/inflation behaviour beyond what the raycast
map provides, and anything about real-floor friction.

## TWO NAMED FOLLOW-UPS — expected, not mysteries

**1. Approach stall near clutter.** Both arms, on the 90°/1.0 m goal, ended 0.22 m short of
a 0.12 m tolerance with **`Failed to make progress` and zero collision aborts**. Affects
both configs equally, is **not** rotation, and belongs to the goal-tolerance/inflation
family. **If Scott sees this signature — short of goal, near clutter, progress-checker
abort, no collision errors — it is this, named in advance, not a new mystery.**

**2. The severity scenario stays banked.** The 90°/1.0 m case remains the one where the
broken config showed field-like severity. It is available to fidelity iteration 2 (noise /
encoder quantisation), which is **unspent** — the pre-committed budget was two iterations
and only one (rotation dynamics) was used.

## Provenance of the numbers

Everything traceable, nothing tuned to a target:

* curve and floor — `03_validation/breakaway_2026-08-16/` (four runs)
* `τ = 0.22 s` — fitted from breakaway run 1's in-burst ramps (duty 12 → 0.267 s,
  duty 16 → 0.188 s, midpoint), **frozen before the falsifier ran**
* field signatures — `03_validation/3a_stock_middle_2026-08-17/`
* the rig — `launch/sim_closed_loop.launch.py`, real recorded map, real 179° laser mount,
  odom at 10.0 Hz against the field's 9.89

**Recorder note for the ops record:** `ros2 bag` produced 0-byte mcap files twice under
SIGINT even with a 10 s grace, so all sim data was captured with
`diagnostics/run_recorder.py`, which writes incrementally. Do not "upgrade" back to
`ros2 bag` mid-incident.
