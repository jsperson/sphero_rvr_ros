# The navigation reckoning

**Written 2026-08-16, the night gauntlet mission 1 ended `INCOMPLETE_START_BLOCKED`
with 0.78 m of open floor in front of the rover.**

Scott: *"YOU HAVE GOT TO FIGURE OUT THE NAVIGATION STUFF. NO MORE WEEKS OF WORK!!! …
We have an LLM we don't have to have perfect deterministic code."*

This document answers that. It is an audit, not a plan to write more code.

---

## VERDICT

**We deleted the middle of Nav2 and spent three weeks rebuilding a worse version of it
by hand. The evidence is not circumstantial — our own launch file says so in a comment.**

`launch/explore.launch.py:36`:

> *"Decisive mode drops controller_server (and with it Nav2's local costmap), so the
> stock BT's local-costmap clears break bt_navigator bringup."*

That single decision is the root of the defect family that has consumed the project:

| what we lost with `controller_server` | what we hand-rolled to replace it | what it cost |
|---|---|---|
| local costmap | nothing — we run **blind below the global map** | D42, D43, tonight |
| stock local planning (RPP, already configured) | `decisive_control.py` | D40 vocabulary problem |
| collision-checked Spin/BackUp/DriveOnHeading | `stall_ladder.py`, give-up escape, 2a/2b/2c | D36, D40, three "recovery that never runs" defects |
| progress checkers | our freeze classifier | tonight's 5 freezes — **phantom, convicted (D45)** |
| obstacle memory with decay | freeze-mark discs | D42 mark prison, **tonight's ending** |

**`behavior_server` is running in our stack right now with Spin, BackUp and Wait
configured — and has been all along.** Nav2's behaviors collision-check against the
*local costmap*. There is no local costmap. That is why D36 measured **2 ms refusals**:
the stock recoveries were present, wired, and structurally unable to succeed. We
diagnosed that as our escape being wrong and built three more escapes.

**Recommendation: restore the stock middle. Keep the floor. Put the LLM on top.**

- **KEEP (ours, hardware-true, field-proven):** the RVR driver, the reflex collision
  supervisor, the ToF geometry, and the blind-band hold — which flew tonight and
  worked. No stock component knows these truths.
- **RESTORE (config, not code):** `controller_server` + `local_costmap`, a stock local
  planner, stock BT recoveries with progress checkers, and **decaying obstacle memory
  instead of permanent marks**.
- **LLM ON TOP:** stuck strategy, exploration targeting, mission logic — the
  latency-tolerant judgment that 2a/2b/2c approximated in deterministic code.

### The closing argument, found after this document was drafted

**The bespoke stack ground the motors with sub-breakaway in-place rotation on gauntlet
mission 1 — the exact failure it was built to prevent.**

**41 consecutive commanded pure rotations produced 0–1 mm of motion.** The freeze
classifier called it an invisible obstacle, planted marks, and the marks buried the
rover's own cell.

The mechanism is worse than a mis-tuned rate, and my first account of it was wrong
(corrected in the autopsy after review). A pure in-place pivot does not execute the
commanded rate at all: `driver.py:708` takes a closed-loop controller that reads only the
command's **sign**, targets a fixed 1.3 rad/s, and ramps a duty that **saturates at 32**
on a ±127 scale — against the driver's own carpet note that duty ≤128 on the 0–255 scale
"does not move at all". **Three layers hold an opinion about the pivot rate — the
controller's 0.9, the supervisor's 0.4 clamp, `rvr_node`'s second 0.4 clamp — and none of
them is the one that executes.**

We forked the middle of Nav2 to escape this failure, then rebuilt it one layer down,
where the guarantee written to prevent it could not see it. **A unit mismatch at a seam
between two of our own components killed the mission.** Full chain:
`docs/autopsy_phantom_freeze_2026-08-16.md`.

---

## THE DECISION — Scott's, and these are the three real options

**A. Restore the stock middle (recommended).** Phases in §6, ~2–3 days, each revertable
with a physical kill criterion. Start with the one-hour experiment in 3a. Risk: a
migration during a period when the robot already doesn't work. Mitigation: the replay
acceptance set already exists in the vault, and the floor we keep is the part that has
never failed.

**B. Fix the seam and keep the bespoke middle.** Cheapest by far: raise the supervisor
clamp above breakaway and tonight's mission-killer disappears. Honest case for it — the
bespoke layer's individual defects are now each understood and most are one-line fixes.
Honest case against — that has been true for three weeks, and each fix has revealed the
next defect in the same layer. This option bets that D45 was the last one.

**C. Measure first, decide after (the null option, and it is not cowardice).** The
breakaway sweep is 15 minutes and is required by **both** A and B. Nothing in A can be
validated without it, and B's fix *is* it. **Whatever Scott chooses, the sweep happens
first** — see `docs/run_card_breakaway_2026-08-16.md`.

**My recommendation is A, and my strongest argument for it is not the defect count — it
is that every defect this week lived in the layer we wrote, and none lived in the layer
we kept.**

---

### The sharpest finding, which may make most of A cheap

The decisive controller exists because stock **RPP + RotationShim ground the motors** —
a real hardware failure that powered the rover down twice. But Nav2's tuning guide
recommends **RPP for exactly this class of robot**, and RPP's `use_rotate_to_heading`
*is* the decisive controller's pivot-then-drive policy, stock.

**We configured its rotation rate at `0.4` rad/s against a drivetrain breakaway of
`0.9`** (`lean_nav2.yaml:48` vs `pivot_rate_rad_s: 0.9`).

**So the most likely explanation for three weeks of bespoke middleware is a single
mis-tuned parameter — and it has never been retested.** §2 and §6 make that the first
hour of work, ahead of everything else, because if RPP drives cleanly at a decisive
rotation rate then most of the migration is deletion rather than construction.

**Confidence: high on the diagnosis, high on package availability (verified on the Pi,
§7), medium on the estimate.** The single largest unknown is now step 3a's outcome, and
it costs an hour to remove.

---

## 1. Tonight's run is the whole argument in miniature

Nothing in tonight's failure was a sensor problem, a driver problem, or a physics
problem. Every layer we *own* performed:

- **The ToF hold worked on its first flight.** 299 rows; clamped at beliefs of 0.152,
  0.154, 0.155, 0.179 m — inside the sensor's structural blind band, the exact ranges
  where the brake released and struck a table leg on 08-15 — and the rover did not
  advance. 183 of those rows held the belief while reversing and pivoting, so it never
  trapped the rover. **No contact all run.**
- **The measured footprint held.** The 08-14b wedge went from 28 returns "inside the
  footprint" to zero, because the 28 were inside the *padding*.
- **The mission clock was honest** (D41 fixed): 128.8 s, 6.8 m².

And then the middle layer put the robot in a prison it built itself:

1. The freeze classifier fired **5 times**, the first **3.0 s after arm**.
2. Each freeze planted a mark. Our `freeze_layer` is a stock `ObstacleLayer` configured
   `marking: true, clearing: false` — **marks that cannot be cleared by construction.**
   The config's own comment admits it: *"a mark therefore lasts the mission … If a mark
   ever needs revoking mid-mission that needs a real mechanism, not this one."*
3. Each mark inflates to a ~0.56 m sterilised disc (D42).
4. The rover's own cell went to inscribed cost. `START POSE BLOCKED` — **correctly**.
5. It sat still with 0.78 m of open floor ahead until we stopped it.

**Phantom freezes → real marks → real inscribed burial → truthful block.** Scott
confirmed *no contact behind the rover*, and the autopsy since written
(`docs/autopsy_phantom_freeze_2026-08-16.md`) **convicts step 1 from the recording**:

> The supervisor clamps every angular command to **0.4 rad/s**
> (`collision_stop.yaml:156`). The decisive controller commands pivots at **0.9**
> precisely because *"the controller must never command a below-breakaway speed"*
> (`docs/decisive_controller.md:59`). **41 consecutive commanded rotations at exactly
> 0.400 rad/s produced 0–1 mm of motion.** The motors could not execute the command,
> the robot did not move, and the freeze classifier blamed an invisible obstacle.

**So the entire chain is an artifact of code we wrote — and worse, of a guarantee
defeated by the layer beneath it.** We forked the middle of Nav2 to escape
sub-breakaway in-place rotation, then rebuilt that exact failure one layer down, where
the guarantee written to prevent it could not see it. This is the single strongest
argument in this document and it was found after the document was drafted.

**Every link in that chain is a component Nav2 ships and we replaced.**

---

## 2. The original sin, and the honest case for it

The fork was not stupid, and I had it wrong in the first draft of this document. I wrote
that stock "dithered". **It did not dither — it destroyed motors.** From
`docs/decisive_controller.md`, verbatim:

> *"The RVR tank drive is … **bad at slow, precise moves** — below a 'breakaway' speed
> the motors don't turn, they grind in place. Nav2's default controller (RPP +
> RotationShim) … stops to pivot in place to re-face the path. On this drivetrain that
> means constant slow in-place pivots — which grind the motors (observed repeatedly;
> **the rover had to be powered down twice**)."*

**That is a hardware truth, and by this document's own §4 criterion it is legitimately
ours.** The RVR has a breakaway speed. Any controller that commands sub-breakaway
in-place rotation damages the robot. That is not a tuning preference.

**But the constraint and the component are different things — and this is the crux of
the whole reckoning.** The response to "never command below breakaway" is a set of
velocity floors and a heading policy:

- never command angular rate below breakaway (`pivot_rate_rad_s: 0.9`)
- a heading deadband, so small errors are not corrected at all (~10°)
- prefer arcs over pivots — keep both tracks rolling

**Every one of those is expressible as controller configuration.** They are reasons to
constrain a planner, not reasons to own one.

### And here is the fact that decides it

**The failure was measured against `RotationShimController` + RPP — the one stock
controller whose entire purpose is to stop and rotate in place to face the path.** Our
own config still says so (`config/lean_nav2.yaml:46`):

```yaml
plugin: "nav2_rotation_shim_controller::RotationShimController"
primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
rotate_to_heading_angular_vel: 0.4     # <-- BELOW the 0.9 breakaway pivot rate
```

**We picked the stock controller most guaranteed to produce the failure, configured its
rotation rate at less than half the known breakaway threshold, and concluded that stock
local planning does not work on this robot.**

### The conclusion I did not expect, and the one to test first

My first instinct was "use MPPI instead". **The PM's research says the opposite and is
right: Nav2's own tuning guide recommends Regulated Pure Pursuit for small differential
drives, and warns that MPPI is computationally expensive** — a warning this project has
independent evidence for, since tonight the Pi hit **load average 10.69** and starved the
ToF to 5.4 Hz until we shed two nodes.

So the two findings collide productively:

- RPP is the *right* controller for this robot.
- RPP's `use_rotate_to_heading` **is** pivot-then-drive — *the decisive controller's
  entire policy, stock.*
- And we configured its rotation at **0.4 rad/s against a 0.9 rad/s breakaway.**

**Which means the most likely truth is the most uncomfortable one: we may have forked
the middle of Nav2 over a single mis-tuned parameter.**

I am not asserting that as fact — I am asserting it is **cheap to test and has never
been tested**, and that no amount of further bespoke work is justified until it has been.
The test is one line:

```yaml
rotate_to_heading_angular_vel: 0.4   ->   0.9    # above breakaway
# and check max_angular_accel: 0.8 does not keep the ramp sub-breakaway
```

If RPP with a decisive rotation rate drives this robot without grinding, then the
decisive controller, the stall ladder, the give-up escape, 2a/2b/2c and the freeze-mark
system were all built on a parameter.

**Consequences for the migration:**

1. **Restoring `controller_server` as currently configured would reproduce the motor
   grinding** — 0.4 rad/s is still in the file. Fix the rate *in the same commit* that
   launches the node, or reproduce the original injury.
2. The steelman for the decisive controller narrows to: *"one stock controller,
   mis-configured below the drivetrain's breakaway threshold, damaged the motors."*
   That is a configuration defect, not an architectural finding.
3. **MPPI moves from first choice to fallback**, and carries a compute gate: if tried, it
   must be measured against tonight's baseline (ToF ≥ 6.5 Hz, load < 4) before it is
   allowed near a mission.

---

## 3. Component audit

| ours | stock equivalent | maturity | our defect cost |
|---|---|---|---|
| `freeze_layer` marks (permanent discs) | **ObstacleLayer with `clearing: true` + `observation_persistence` + DenoiseLayer** (cheap form); STVL only if per-voxel decay proves necessary | stock; STVL is **not installed** and is compute-heavy for a small robot | **D42**, tonight's ending, the 0.30 m escape constant that exists only to escape our own marks |
| `stall_ladder.py` + give-up escape + 2a/2b/2c | BT `RecoveryNode` + `Spin`/`BackUp`/`DriveOnHeading`, collision-checked | stock, running in our stack already | **D36** (2 ms refusals), **D40** (un-grantable by construction), 3 forms of "recovery that never runs" |
| freeze classifier | `nav2_controller` progress checkers (`PoseProgressChecker`, oscillation) | stock, **already in our YAML at line 25, never launched** | tonight's 5 freezes → the mark prison |
| `decisive_control.py` | **RPP with `use_rotate_to_heading` and a rotation rate above breakaway** (Nav2's own recommendation for small diff-drive); MPPI only as fallback | stock, and already our configured controller — at the wrong rotation rate | **D40** vocabulary problem, avoid-offset machinery, the entire escape-shape family |
| `coverage_explorer` goal churn/blacklisting | `nav2_wfd` / `frontier_exploration_ros2` / `m_explore_ros2` — **both already vendored on the Pi** | stock | D43's guard, `planner_rejections: 7`, `INCOMPLETE_NO_PLANNABLE_TARGETS` |
| low-obstacle costmap injection | STVL with `/tof/points` as a native observation source | stock | this is **batch B delivered as YAML** |
| — | `nav2_collision_monitor` (polygon stop/slow zones) | stock | *see §4 — this one is genuinely contested* |

---

## 4. What is genuinely ours, and stays

**The RVR driver.** Sphero's serial protocol, the dispatcher, fail-safe. No stock code
exists. Keep.

**The ToF sensor model** (`tof_frame.py`) — mount geometry, rule A/B, the blind-band
derivation. This is a physical description of *our* sensor on *our* robot. Keep.

**The blind-band hold.** Nothing in Nav2 or the literature models "the sensor is
structurally incapable of reporting at this range, so silence is not clearance." It is
novel, it is hardware-specific, it flew tonight and it worked. **Keep, and be proud of
it.**

**The reflex collision supervisor — contested, and I'll argue both sides honestly.**
Nav2 ships `nav2_collision_monitor`, which does polygon-based stop/slow zones from
sensor sources and is the stock answer to "reflex layer below the planner". By this
document's own logic I should recommend replacing ours.

I don't, for one reason that is about the robot and not about pride: our supervisor is
the **arbitration point** where the ToF hold, the lidar sectors, the trajectory
projection and the pivot gate all compose, and it is the layer that has been hardened by
every field defect this project has produced. It is also the only layer that never
failed us — including tonight. Replacing a working safety floor while simultaneously
replacing the entire middle is two risky migrations at once. **Recommendation: keep it,
and revisit `collision_monitor` only after the middle is stock and stable.** That is a
sequencing argument, not a claim that ours is better.

---

## 5. Where the LLM actually goes

The published pattern (OrionNav and the 2025-26 agentic-exploration work) is:

> **LLM → constrained discrete choice → deterministic BT/FSM executing gated primitives**

with a lightweight semantic map as stored relations. Nobody publishes hand-rolled escape
ladders, because the judgment those ladders encode is exactly what a language model does
better, and the execution is exactly what a behaviour tree does better.

**This validates 2a/2b/2c's *shape* and indicts its *implementation*.** Survey → rank →
execute is the right decomposition. We wrote the ranking as deterministic Python with
cause-conditioned rules, vouching hierarchies and grantability pre-checks — roughly 900
lines encoding judgment. That judgment is:

- **latency-tolerant** (the robot is stopped; a 2 s LLM call costs nothing)
- **not safety-critical** (the supervisor still arbitrates every command)
- **exactly what LLMs are good at** (weighing heterogeneous evidence into a choice)

**So: 2a's survey survives as the LLM's input** — it is a structured measurement, and it
is genuinely good work. **2b/2c's ranking and sequencing become a prompt** plus a
constrained output over stock primitives. The grantability check survives as a
*validator* on the LLM's choice, not as a ranking engine.

Same for exploration targeting: "which frontier next" is judgment, and "find the red
mug" is the actual product Scott wants.

---

## 6. Migration shape, and what proves it

**Three phases, each independently revertable, each with a kill criterion.**

**Phase 1 — restore the local costmap (half a day).** Launch `controller_server` and
`local_costmap` (both already fully configured in `lean_nav2.yaml`, just not started).
Feed the local costmap from lidar **and `/tof/points`**. Keep the decisive controller
driving for now.
*Proves:* stock recoveries stop refusing in 2 ms. **Kill criterion:** if `behavior_server`
still refuses with a live local costmap, the D36 diagnosis was wrong and everything
downstream needs rethinking.

**Phase 2 — decaying obstacle memory (half a day).** Replace `freeze_layer` with STVL
(or, if STVL is unavailable, an ObstacleLayer with `clearing: true` and honest raytrace
ranges). Delete the freeze-mark disc code and the 0.30 m escape constant that exists to
escape it.
*Proves:* replay tonight's mark-prison — the rover must not bury its own cell.
**Kill criterion:** marks that decay let the rover re-enter a place that froze it. If
that produces repeat contacts, permanence was load-bearing and we need a middle ground.

**Phase 3 — stock local planning + BT recoveries (1–1.5 days), and it starts with a
one-line experiment.**

*Step 3a (one hour, do this before anything else in the whole plan):* launch
`controller_server` with RPP and `rotate_to_heading_angular_vel` raised **above
breakaway** (0.9, matching `pivot_rate_rad_s`), `max_angular_accel` checked so the ramp
does not sit sub-breakaway, and drive a short path. This is the cheapest possible test of
the most expensive assumption this project has made. **If it drives cleanly, Phase 3 is
mostly deletion, and the architecture question is largely answered.**

*Step 3b:* only if 3a fails, try MPPI — with the compute gate below.

Then: delete the stall ladder and give-up escape; use the BT's `RecoveryNode`. Retire
2b/2c's deterministic ranking in favour of the LLM path.
*Proves:* the three archived wedge poses + tonight's pose, replayed.
**Kill criterion — and it is a physical one, not an aesthetic one:** *any commanded
in-place angular rate below the breakaway threshold is an immediate stop.* That is the
failure that powered the rover down twice. It is measurable from `/cmd_vel_motor` in a
bag, and it should be a **test**, not a judgement call — assert no sustained command
below breakaway with zero linear velocity. If MPPI cannot be constrained to satisfy it,
the decisive controller has earned its place and the recommendation is wrong.

**Total: 2–3 days**, matching Scott's tolerance — *if* the packages are present.

### Replay acceptance set (all already in the vault)

1. Run 1's vanish sequence (`tests/fixtures/run1_vanish_20260815.json`) — the hold must
   still hold.
2. The three wedge poses (`wedge_20260814b`, `wedge_20260815_postspin`,
   `wedge_mission2_2026-08-14`) — stock recoveries must find an exit.
3. Tonight's mark prison (`gauntlet_2026-08-16_mission1`) — must not recur.
4. The gauntlet itself: 3 missions, Scott's bar, no contact.

---

## 7. Package availability — VERIFIED on the Pi, 2026-08-16 00:23

Checked directly after the Pi came back on mains (fresh boot, 1 min uptime):

```
nav2_mppi_controller          PRESENT     <- Phase 3 is unblocked
nav2_dwb_controller           PRESENT     <- fallback available
nav2_controller               PRESENT     <- progress checkers, already in our YAML
nav2_rotation_shim_controller PRESENT     <- already referenced by our config
nav2_behaviors                PRESENT     <- Spin/BackUp/Wait, already running
nav2_collision_monitor        PRESENT     <- the §4 contested option, if ever wanted
nav2_smac_planner             PRESENT     <- already our global planner
explore_lite / _msgs          PRESENT     <- a stock explorer is available
spatio_temporal_voxel_layer   ***ABSENT***
```

**This materially changes two phases, both in our favour except one:**

- **Phase 3 needs no installation.** MPPI is there. The most valuable swap is pure
  configuration.
- **Phase 2 must use the fallback.** STVL is *not* installed and is a separate
  build (it is not part of `nav2_bringup`). **Do not put an apt/colcon build of STVL on
  the critical path.** The fallback — a stock `ObstacleLayer` with `clearing: true`,
  honest `raytrace_max_range`, and `observation_persistence` — needs no new package and
  fixes the actual defect, which is that our marks *cannot be cleared at all*. Going
  from "never clears" to "clears on observation" is the whole win; per-voxel decay
  curves are a refinement. **Revised Phase 2: still half a day, no dependency risk.**

**Correction to an earlier draft of this document:** I wrote that two frontier explorers
were "already vendored on the Pi". `explore_lite` and `explore_lite_msgs` are built and
registered; I have not confirmed `frontier_exploration_ros2` beyond a source directory.
Only claim what is registered.

### Still genuinely unverified

- **Why the decisive controller was written.** I inferred the justification from code
  and docs. `docs/decisive_controller.md` must be re-read in full before Phase 3 —
  **if it was fighting something MPPI also does, that is the decisive fact of this whole
  document**, and it is the one thing I have not personally confirmed.

---

## 8. What survives — most of it, honestly

This is not three weeks wasted, and the register is why:

- **The ToF hold, the footprint truth, the sensor model** — kept entirely.
- **The instrumentation** — the recorder columns, the state line, the costmap dump, the
  preflight. These are how tonight was diagnosed in twenty minutes instead of a session.
- **The fixtures** — run 1's vanish, three wedge poses, tonight's prison. **These are the
  acceptance suite for the migration.** They exist because we did the hard forensic work.
- **2a's survey** — becomes the LLM's structured input.
- **The engineering standards** — 11 rules, each bought by an incident. They apply
  regardless of whose code runs the middle.

**What does not survive:** the stall ladder, the give-up escape, 2b/2c's deterministic
ranking, the freeze-mark disc system, and probably the decisive controller. Roughly
2,000 lines. Every one of them was written to work around the absence of a local
costmap.

---

## 9. The one-sentence version

**We removed the load-bearing wall of Nav2 to fix a dithering problem, spent three weeks
propping up the ceiling by hand, and tonight the ceiling fell in a way that stock Nav2
has had a configuration option for since 2020 — so put the wall back, keep the four
things that are genuinely ours, and let the LLM do the judgment we tried to hard-code.**
