# Autopsy: the freezes were phantom, and the robot was told to do something it cannot do

**Gauntlet mission 1, 2026-08-16. Five freezes, no contact, mission dead with 0.78 m of
open floor ahead.**

Scott, from the floor, before any of this analysis: **there was no contact behind the
rover.** He was right, and the recording says why.

---

> ## ⚠⚠ SECOND CORRECTION, 2026-08-16 afternoon — MEASURED ON THE FLOOR. THE CAUSAL CHAIN BELOW IS FALSIFIED.
>
> **The duty ceiling did not cause these freezes. Two independent lines of evidence kill it.**
>
> **1. The measurement.** The breakaway sweep ran on rubber gym flooring — the operating
> surface — at 83 % battery, binary `5b880aa`. Tank duties 2–10 produce *exactly* zero
> rotation with 41 motor packets written per burst; duty 12 rotates. **Breakaway is
> between tank 10 and 12.** Every deployed constant is far above it: `pivot_min_duty` 28
> is ~2.5× breakaway, `pivot_max_duty` 45 is ~4.1×. The pivot loop clamps to *at least*
> `pivot_min_duty` on its first cycle, so the very first packet of any pivot was already
> well over twice the duty needed to turn this robot. The chain's second box — "motors do
> not turn" — is false. (Archive:
> `03_validation/breakaway_2026-08-16/`. The `≤128 does not move` figure this autopsy
> reasoned from is a **raw-motor, carpet** measurement and transfers to neither the tank
> scale nor this surface.)
>
> **2. The episode contains a command the pivot path never touches.** Re-read row by row,
> convicted episode A is not 32 rows of pivot. It is **20 rows of pure rotation followed
> by 11 rows of pure reverse at −0.1 m/s** — and the robot moved for neither. A straight
> reverse goes through `drive_tank_si_units` at the bottom of the control loop and never
> enters the pivot branch at all. **No pivot-ceiling story can explain a reverse that also
> produced nothing.** Whatever stopped the robot stopped both command shapes.
>
> **What IS established, from the bag rather than inferred:**
> - `/cmd_vel_motor` carried 0.400 rad/s **densely — ~30–50 Hz, no gap over 0.1 s — for
>   2.04 s**, then −0.100 m/s for 1.17 s. This is `rvr_node`'s own subscribed topic
>   (`supervised_rvr.launch.py` remaps its `cmd_vel` to `/cmd_vel_motor`), so the commands
>   were genuinely at the driver's door. The recorder CSV's 41 held samples are confirmed
>   by the real message stream, not an artifact of a 10 Hz sampler holding a stale value.
> - `/odom` was **healthy throughout — 10 Hz, no gap** — and reported `twist.linear.x` and
>   `twist.angular.z` of **exactly 0.000**, with pose bit-identical to 0.1 mm. The wheels
>   did not turn. This is not a stale-publisher artifact; it was checked.
>
> **So the defect is INSIDE the driver, between a Twist arriving and a motor packet
> reaching the wire — and the recording cannot see it.** `rvr_node` publishes
> `motor_transport_write_count`, `motion_transport_write_count`, `last_motor_payload_hex`,
> `last_motor_transport_write_epoch_s`, `fail_safe_active`, `motor_stall` and
> `motor_fault` on `/diagnostics`, at 10 Hz, and **always did**. The mission bag did not
> record `/diagnostics`. The owner published the fact; the recording dropped it; the
> analysis inferred across the seam and convicted the wrong component. Fixed in
> `scripts/launch_and_arm.py` with a guard test.
>
> **Everything downstream of box one still stands** — the freeze classifier, D42's
> permanent marks, D43's truthful block. The mission still died the way the chain below
> describes. **What changed is the first box: the actuator ceiling is innocent, and the
> reason the wheels did not turn is not yet known.**
>
> Reclassifications this forces:
> - **D45 — REFUTED as written.** Not "an actuator ceiling no layer above the driver knows
>   about". The ceiling is fine.
> - **The historical freeze record is no longer suspect *for this reason*.** The advice to
>   re-read prior freezes against "which driver path was executing" is withdrawn: the pivot
>   path was capable of moving the robot the whole time.
> - **`pivot_min_duty` 23/28 is CORRECT, not defective.** Duty 12 was measured **bimodal**
>   — a clean 1.48 rad/s pivot in one run, a 0.84 rad/s one-tread arc that walked 15.9 cm
>   six minutes later. A floor exists to stay clear of exactly that margin.
> - **NEW, unexplained:** a driver that receives dense commands and writes nothing to the
>   wheels, for both a pivot and a reverse, for ~3 s, then releases. Mechanism unknown.
>   Candidates that survive the evidence: a latched `fail_safe_active` (which makes
>   `set_velocity` raise), motion-generation invalidation, or transport starvation. All
>   three are distinguishable from `/diagnostics` alone on the next mission.

---

> ## ⚠ CORRECTION, same night, before this was ever acted on
>
> **The mechanism first published here was WRONG, and the review loop caught it.**
> I wrote that the supervisor's 0.4 rad/s clamp sat below the drivetrain's breakaway
> rate, so the motors could not execute the commanded pivot.
>
> **The commanded rate never reaches the motors on this path at all.** For a pure
> rotation (`|linear| < 0.005`, `|angular| > 0`) the driver takes a **closed-loop pivot
> controller** (`driver.py:708`) which uses the commanded `angular_rad_s` **only for its
> sign**, and then drives toward a fixed internal target of **1.3 rad/s**:
>
> ```python
> sign  = 1.0 if velocity.angular_rad_s > 0.0 else -1.0
> error = self._pivot_target_rate_rad_s - abs(self._measured_yaw_rate)   # 1.3, always
> self._pivot_duty_cmd += self._pivot_duty_gain * error
> self._pivot_duty_cmd = min(pivot_max_duty, max(pivot_min_duty, self._pivot_duty_cmd))
> ```
>
> So a commanded 0.4 and a commanded 0.9 produce **identical** behaviour. The 0.4 clamp
> is real and is a defect elsewhere, but it is **not** why the robot failed to turn.
>
> **The corrected mechanism is below, and it is worse.**

> ## ⚠ SECOND CORRECTION, 2026-08-16 ~03:40, while building the sweep tool
>
> **The ceiling quoted below is the DATACLASS DEFAULT, not what the mission ran.**
>
> `rvr_node.RVRNodeConfig` defaults to `pivot_min_duty 23 / pivot_max_duty 32 /
> pivot_duty_gain 0.6`, and `config/rvr.yaml` sets none of the three, so those are the
> numbers for `rvr.launch.py` and `supervised_rvr.launch.py` on their own. But the
> gauntlet was launched by `scripts/launch_and_arm.py` → `explore.launch.py`, whose
> `rvr_params_file` defaults to `config/lean_rvr_tank_si.yaml` and is passed straight
> through `supervised_rvr.launch.py` into the driver node. That file sets:
>
> ```yaml
> pivot_min_duty: 28
> pivot_max_duty: 45      # <- the ceiling mission 1 actually ran
> pivot_duty_gain: 1.0
> ```
>
> **What changes:** `45` on the ±127 scale is **90/255**, not 64/255 — so the ceiling is
> about **70%** of the documented no-move duty of 128, not "about half". And the
> integrator climbs at gain 1.0 × error 1.3 = 1.3 duty per 0.05 s cycle, so it walks
> from 28 to 45 in roughly **0.65 s**, not two seconds.
>
> **What does not change:** 90 is still below 128. Both deployed ceilings are still
> under the duty the driver's own comment says produces no motion at all, the 3.2 s and
> 2.0 s episodes are still far longer than the saturation time, and the finding stands.
> **The measurement is what settles it, and now it has to span both ceilings** —
> `diagnostics/pivot_duty_sweep.py` samples 23, 28, 32 and 45 explicitly, with a test
> that fails if any of them leaves the ladder.
>
> This is [[probe-the-deployed-config]] again, in the document that exists to correct
> an inference: two numbers were read from a dataclass instead of from the YAML the
> launch file actually passes.

## The finding (corrected)

**The in-place pivot controller cannot reach the duty this drivetrain needs to move,
because it clamps at roughly half of it. The commanded rate is irrelevant — the pivot
path discards it.**

Two numbers from our own source, and they do not fit together:

```
driver.py:717  (the code's own carpet measurement, raw-motor angular duty, 0-255 scale)
    "angular duty <=128 does not move at all, 140-160 breaks away then bogs"

rvr_node.py:46  pivot_max_duty = 32          <- the closed-loop pivot's ceiling
commands.py:164 drive_tank_normalized clamps to +/-127   <- the scale it is sent on
```

**32 on a ±127 scale is ≈64 on the 0–255 scale the measurement used — about half the
128 that the driver's own comment says produces no motion at all.**

The integrator makes it worse: `pivot_duty_gain` 0.6 against a constant error of ~1.3
adds ~0.8 duty per 0.05 s cycle, so it reaches its ceiling of 32 in about two seconds
and then stops climbing, permanently, no matter how long the pivot is commanded.

**Caveat, stated rather than buried:** the two figures come from different firmware
paths — `drive_tank_normalized` versus raw-motor duty — and I have not measured that
they are equivalent in torque. The scale conversion is arithmetic; the equivalence is an
inference. **This is exactly what the floor test must settle**, and it is why the
breakaway measurement is now a DUTY sweep rather than a rate sweep
(`docs/run_card_breakaway_2026-08-16.md`).

What the field data shows is consistent with it: **41 commanded pure rotations, 0–1 mm
of motion**, over episodes of 3.2 s and 2.0 s — long enough for the integrator to
saturate and still produce nothing.

**And the guarantee above it is void either way.** The controller documents that it
"must NEVER command a below-breakaway speed" and pivots at 0.9
(`decisive_control.py:35`, `docs/decisive_controller.md:59`). Two clamps then rewrite it
— `collision_stop.py:1048` to 0.4, and `rvr_node`'s own `max_angular_rad_s: 0.4` at the
driver's door — and the pivot path discards the number entirely regardless. **Three
layers each hold an opinion about the pivot rate and none of them is the one that
executes.**

---

## The measurement

From `run_20260815_235211.csv` (14,493 rows), detected **by signature, not by clock or
position** — "commanded motion, no movement" computed directly, so no frame or timestamp
alignment is involved:

```
12 sustained-command episodes (>= 1.5 s)
 2 were COMMANDED BUT DID NOT MOVE (< 2 cm and < 5 deg):

 rows 10009-10040   3.2 s   moved 1 mm / 2.2 deg   at (-0.93,-1.80)
     pure rotation: 21 rows, ALL at 0.400 rad/s     below 0.9: 21/21
 rows 10127-10146   2.0 s   moved 0 mm / 1.8 deg   at (-0.93,-1.81)
     pure rotation: 20 rows, ALL at 0.400 rad/s     below 0.9: 20/20
```

**41 consecutive commanded rotations at exactly 0.400 rad/s, producing 0–1 mm of
motion.** Not "roughly 0.4" — the clamp value, exactly, every row.

### Why only 2 of 5

The detector requires ≥ 1.5 s of sustained command and < 2 cm of motion. Shorter freezes
and ones with slight drift fall outside it. **Two convicted episodes with an identical
signature is the finding; the count is not.** The other three are consistent with it and
are not claimed as proven.

### A frame gap found on the way, worth its own row

The controller logs `FREEZE at (0.11,-0.03)`; the nearest `/odom` sample in the recorder
is **6–14 cm away**, consistently, for all four logged positions. Two pose sources
disagree by a repeatable offset — most likely map-frame vs odom-frame. **Any analysis
that matched freeze positions to recorder rows by coordinate would have silently
compared the wrong rows.** This autopsy avoided it only by matching on behaviour instead.

---

## The causal chain, end to end

```
controller commands a pivot (0.9, clamped to 0.4, discarded anyway)
        v
closed-loop pivot targets 1.3 rad/s, ramps duty, SATURATES AT 32/127
        v
motors do not turn; robot does not move
        v
freeze classifier: "permitted to move and did not"
   -> "an obstacle no sensor on this robot can see"     <-- FALSE
        v
plants a freeze mark
        v
mark paints a lethal disc; costmap inflates it to ~0.56 m sterilised   (D42)
        v
5 marks around a stationary robot bury its own cell at inscribed cost
        v
START POSE BLOCKED fires -- TRUTHFULLY                                 (D43)
        v
explorer stops issuing goals. Mission ends INCOMPLETE_START_BLOCKED
with 0.78 m of open floor dead ahead.
```

**Every step after the first is a component behaving exactly as designed.** The mission
was killed by an actuator ceiling that no layer above the driver knows exists.

---

## What this reclassifies

- **D43 — the block was TRUE.** Not costmap fidelity, not a pose offset. Our own marks.
  (The dump that was supposed to prove this reported the inverse; fixed at `2e41cbf`.)
- **D42 — field-convicted as the amplifier.** Permanent marks turned a transient
  misclassification into a permanent prison.
- **D25/D33 phantom-freeze suspicions — a mechanism now exists.** Prior freezes recorded
  as "an obstacle no sensor can see" should be re-read against WHICH DRIVER PATH was
  executing. **The historical freeze record is suspect wherever the command was a pure
  in-place pivot**, because that path could not have moved the robot regardless of what
  was in the room.
- **Scott's touch-tolerance doctrine is unaffected but was being spent on nothing.** The
  freeze signature was meant to be the robot's touch sense. Here it fired on an
  actuator limit.

---

## What is NOT yet known

**The moving duty has never been measured on the path production actually uses.** The
only figure in the repo is a comment about the RAW-MOTOR branch ("<=128 does not move,
140-160 breaks away"), while in-place pivots go through `drive_tank_normalized` on a
±127 scale with a ceiling of 32. Whether those are equivalent in torque is an
**inference, not a measurement**, and every constant in this chain rests on it.

**The measurement must therefore sweep DUTY on the pivot path, not commanded rate.**
Sweeping commanded angular rate would measure nothing at all: the pivot controller
discards the magnitude, so every step would return the same answer and the sweep would
"prove" a drivetrain that cannot turn — a wrong autopsy manufactured by its own
procedure. Card: `docs/run_card_breakaway_2026-08-16.md`.

**That second unknown is now CLOSED, and it makes the trap worse.** The loop's feedback
comes from **wheel-encoder odometry** (`rvr_node.py:563` feeds
`driver.set_measured_yaw_rate(sample.angular_rad_s)` from `_odom_tracker`). When the
motors are stalled the wheels do not turn, so the encoders correctly report ~0 rad/s,
so `error = 1.3 - 0 = 1.3` — the **maximum** error — and the integrator ramps at full
rate. **Every part of the loop behaves correctly. It demands more duty, honestly, and
the ceiling of 32 refuses to give it any.** The controller is not mis-tuned; it is
capped below the range where it could ever succeed.

---

## Candidate fixes, in preference order

1. **Raise `pivot_max_duty` to above the measured moving duty.** This is now the
   primary fix: the ceiling, not the commanded rate, is what stops the robot turning.
   `max_angular_rad_s` (0.4, in BOTH the supervisor and `rvr_node`) still wants raising
   above breakaway for the non-pivot paths, but it is no longer the headline.
2. **The supervisor must not silently rewrite a command into an unexecutable one.** If a
   request is below the drivetrain's floor, the honest answers are *refuse it* (and say
   so in `reason`) or *round it up to the floor* — never "deliver something the hardware
   ignores". This is the same class as D40's un-grantable-by-construction, at a different
   seam: the arbiter's own output was physically impossible.
3. **The freeze classifier must not conclude "invisible obstacle" from an unexecutable
   command.** It should require that the delivered command was physically capable of
   producing motion before blaming the world. A freeze is a claim about the room; this
   was a claim about the motors.

Fix 3 is the narrowest and belongs with the classifier regardless of the architecture
decision. Fixes 1 and 2 need the measurement first.

---

## And the connection to the reckoning

`docs/navigation_reckoning.md` argues that the decisive controller — and the whole
bespoke middle layer — exists because stock Nav2 ground the motors with sub-breakaway
in-place rotation, and notes that our own Nav2 config still carries
`rotate_to_heading_angular_vel: 0.4`.

**Tonight the bespoke stack ground the motors with sub-breakaway in-place rotation, at
exactly 0.4 rad/s.**

We forked the middle of Nav2 to escape a problem, and then rebuilt the identical problem
one layer down, where the guarantee written to prevent it could not see it. That is the
strongest single argument in the reckoning, and it was not available when the reckoning
was written.
