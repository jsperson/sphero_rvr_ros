# Autopsy: the freezes were phantom, and the robot was told to do something it cannot do

**Gauntlet mission 1, 2026-08-16. Five freezes, no contact, mission dead with 0.78 m of
open floor ahead.**

Scott, from the floor, before any of this analysis: **there was no contact behind the
rover.** He was right, and the recording says why.

---

## The finding

**The supervisor clamps every angular command to 0.4 rad/s. The drivetrain cannot
rotate in place at 0.4 rad/s. So a commanded pivot produces no motion, and the freeze
classifier — which defines a freeze as "permitted to move and did not" — reports an
obstacle that does not exist.**

Four files, each correct on its own terms:

```
decisive_control.py:35     pivot_rate_rad_s = 0.9
                           "In-place pivot rate -- decisive, above breakaway
                            (never a slow creep)."
docs/decisive_controller.md:59
                           "All above the motor breakaway ON PURPOSE -- the controller
                            must NEVER command a below-breakaway speed."
config/collision_stop.yaml:156     max_angular_rad_s: 0.4
collision_stop.py:1048     angular = clamp(command.angular_z, +/-0.4)
```

**The controller's central safety-of-hardware guarantee — never command below breakaway —
is void in production, because the layer beneath it silently rewrites the command to a
value the guarantee exists to forbid.**

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
supervisor clamps pivot 0.9 -> 0.4 rad/s          (below breakaway)
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
was killed by a unit mismatch at a seam.

---

## What this reclassifies

- **D43 — the block was TRUE.** Not costmap fidelity, not a pose offset. Our own marks.
  (The dump that was supposed to prove this reported the inverse; fixed at `2e41cbf`.)
- **D42 — field-convicted as the amplifier.** Permanent marks turned a transient
  misclassification into a permanent prison.
- **D25/D33 phantom-freeze suspicions — a mechanism now exists.** Prior freezes recorded
  as "an obstacle no sensor can see" should be re-read against the commanded angular
  rate at the time. **The historical freeze record is suspect wherever the command was
  a pivot.**
- **Scott's touch-tolerance doctrine is unaffected but was being spent on nothing.** The
  freeze signature was meant to be the robot's touch sense. Here it fired on an
  actuator limit.

---

## What is NOT yet known

**The breakaway threshold has never been measured.** We know 0.9 was chosen to be above
it and that 0.4 is below it — so it lies in **(0.4, 0.9]** — but the actual value is
undocumented anywhere in the repo. Every constant in this chain is derived from a number
nobody has measured.

**That measurement is the first thing to do**, and it is cheap: command in-place rotation
at 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90 rad/s for 2 s each and record `odom_yaw`.
Fifteen minutes on the floor, and it turns three constants from folklore into
derivations.

---

## Candidate fixes, in preference order

1. **Raise `max_angular_rad_s` above breakaway** (once measured). It exists to bound
   speed for safety, but a cap that produces *grinding without motion* is not a safety
   feature — it is a way to damage motors while standing still.
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
