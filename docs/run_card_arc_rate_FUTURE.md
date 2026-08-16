# RUN CARD (FUTURE, UNSCHEDULED) — the arc rate curve

**Status: NOT FLOWN. No date. Rides any future staging that already has the rover on the
floor with clearance — it needs no dedicated session.**

## Why this exists

2026-08-16 measured the **in-place pivot** curve and made it the single authority on pivot
rate (`src/sphero_rvr_core/pivot_curve.py`,
`03_validation/breakaway_2026-08-16/README_run4.md`):

```
rate ≈ 0.1344 × duty − 0.213     valid for tank duties 23..45
dead zone ≤ 10 · bimodal walk band 12..22
```

**`max_angular_rad_s` was deliberately NOT re-derived from it, and this card is that
decision's close path.** That constant governs **arcs while driving** — a different
command path (`drive_tank_si_units`, or `drive_rc`'s angular fraction) in a different
regime, where both treads drive forward at different speeds rather than opposing each
other. Run 4 measured none of it.

The arithmetic that makes the temptation obviously wrong: applying the pivot ceiling
(5.83 rad/s) to an arc would command a tread differential of
`angular × wheel_track = 5.83 × 0.2507 ≈ **±0.73 m/s**`, against a deployed
`max_linear_mps` of **0.20**. Nearly 4× the rover's own linear limit, on a regime nobody
has measured.

**That is the same error class that put a raw-motor 0–255 duty figure onto a ±127 tank
scale and cost this project a wrong autopsy (D45).** Two paths, matching units, unmeasured
equivalence. The gap is admitted in code
(`safety.clamp_velocity_for_path`) rather than silently absorbed — and admitted gaps are
supposed to have a way to close.

## What it measures

**The achieved yaw rate and achieved linear speed for commanded arcs**, across the
deployed linear range, so `max_angular_rad_s` can stop being folklore.

Sweep the commanded `(linear, angular)` pairs the stack actually issues:

| linear (m/s) | angular sweep (rad/s) |
|---|---|
| 0.05 | 0.2, 0.4, 0.8, 1.2 |
| 0.10 | 0.2, 0.4, 0.8, 1.2 |
| 0.20 | 0.2, 0.4, 0.8, 1.2 |

For each: gyro yaw rate (body), encoder-derived yaw rate **and** forward speed over the
**same whole-burst window** (the run-1 windowing bug — do not compare a steady-half gyro
mean against a whole-burst encoder rate), plus whether either tread saturates.

**The specific questions:**

1. Is there an arc dead zone, as there is for pivots? A commanded arc whose *slower* tread
   falls under breakaway may drive straight instead of curving — silently.
2. Does the commanded angular rate come out as the achieved one, or is there an arc
   equivalent of the pivot path's 8.9× overshoot?
3. What is the largest angular rate that does not saturate a tread at `max_linear_mps`?
   **That number is what `max_angular_rad_s` should be**, and it is currently a guess.

## Safety envelope

**An arc TRANSLATES by design** — this is not an in-place test, and the 5 cm translation
abort that guards the pivot sweep is meaningless here. Consequences:

* **Needs a clear run, not a clear circle.** At 0.20 m/s for a 2 s burst the rover covers
  ~40 cm per rung plus curvature. Budget several metres, or shorten bursts.
* **Scott present, hand on the power switch**, as ever. This is the first deliberate
  driving sweep since the drivetrain's rate behaviour was found to be discontinuous.
* Bounded bursts with stops between, and a hard stop on any tread saturation that produces
  a straight line where a curve was commanded.
* Battery recorded at both ends: the pivot curve is voltage-dependent and arcs will be too.

## What it feeds

* `max_angular_rad_s` in **both** the collision supervisor and `rvr_node` — one constant,
  one author, derived rather than assumed.
* The `UNMEASURED` annotation in `safety.clamp_velocity_for_path` gets deleted, and the
  arc path joins the pivot path in being governed by a curve.
* Stock RPP's angular limits under Option A, which drives arcs constantly.

## Tooling

`diagnostics/pivot_duty_sweep.py` is **not** the right instrument — it commands opposing
treads by construction. This needs a sibling that commands `(linear, angular)` through the
driver's ordinary path and records the same evidence. Reuse its refusals wholesale: the
`--arm` gate, the serial-exclusivity check, the battery floor, the instrument-alive proof,
and the CSV header carrying the whole run configuration.
