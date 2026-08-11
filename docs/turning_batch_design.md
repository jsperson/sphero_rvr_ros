# Design note — the turning batch

Four items, one contract. **Status: design only. Nothing here is implemented.**

The contract every item is written against is the driver's *actual* pivot behaviour —
**sign-only input, self-regulated rate** (§5, D32). Three of the four items have been
designed at least once against the commanded rate instead, and each time the result was
code that is correct about a number the robot does not use.

| # | Item | Gated on the bench session? | Safety path touched? |
|---|---|---|---|
| 1 | Gentle turn-away (the mission-killer) | **No** — buildable now | **No** |
| 2 | Arrival-orientation stage | Yes (segments A, B, D) | No |
| 3 | D32 grind-yaw guard redesign | Yes (segments A, B, E) | No |
| 4 | Pivot contract documentation | Yes (segments A–D) | No |

Zero lines of `collision_stop.py`, the camera brake, or the D17–D21 gates change in this
batch. That is checkable by empty diffstat, and it is a design commitment, not an
aspiration: every fix below is on the *asking* side of the supervisor.

---

## 0. PREMISE CORRECTION — measured before designing, and it moves item 1

The batch brief frames item 1 as *lidar-visible-obstacle timidity*: 10 of 12 aborts had a
min front range inside the brake envelope, so blend steering while passing through the
lidar SLOW band (0.50 → 0.30 m). I re-graded run 114626 from its own recorder CSV before
designing against that, and **the lidar is not the gate that stopped the robot.**

**Population, stated first** (the denominator has bitten this project four times): rows of
`run_20260811_114626.csv` between `mission STARTED` (t=66.8) and the terminal report
(t=376.2) — 3093 rows at 10 Hz, 309 s of mission. "Zeroed" means `/cmd_vel` non-zero while
`/cmd_vel_motor` was exactly (0, 0).

| | time | share of mission |
|---|---|---|
| moving (motor output non-zero) | 191 s | 62% |
| **zeroed (asking, not moving)** | **86 s** | **28%** |
| idle (asking for nothing) | 32 s | 10% |

Attribution of the 86 s, by the supervisor state published in the same row:

| mechanism | evidence in the row | time |
|---|---|---|
| **CAMERA low-obstacle brake** | state `SLOW`/`CLEAR`, reason `front_slow` or `command`/`scan`, `cmd_vx=+0.20`, `out_vx=0.000` | **55 s (63%)** |
| lidar front latch | state `STOPPED`, reason `reset_required` / `front_stop` | 20 s |
| lidar `rear_hold` (reverse refused) | reason `rear_hold` | 10 s |
| lidar trajectory gate | `*_trajectory_blocked` | 1 s |

The camera attribution is not an inference from timing, it is forced by the arithmetic of
the code. `front_slow` publishes `state = SLOW if output.linear_x > 0 else STOPPED`
(`collision_stop.py:980`) and scales forward by `min_forward_scale` **0.70** — so a row
that reads `SLOW / front_slow` *proves the lidar core emitted ≥ 0.14 m/s*. The recorder's
`out_vx` is `/cmd_vel_motor`, which is published **after** `_apply_camera_brake`
(`collision_stop_node.py:566-593`). The only thing in the stack that can turn 0.14 into
0.000 is the camera brake. `cam_cloud_age` was fresh in 100% of mission rows (0 s stale),
so the layer was live throughout; `pivot_veto` fired **0 times** all run.

The run contains its own control, two seconds apart, same label, same state:

```
135.71  SLOW  front_slow  front 0.36  cmd(+0.20, 0.00)  out(0.000, 0.00)   <- camera cutting
137.71  SLOW  front_slow  front 0.44  cmd(+0.20,-0.80)  out(0.140,-0.40)   <- camera clear: exactly 0.70x
```

Same reason, same state, forward request identical — and the motor sees 0.000 in one and
the textbook `0.20 × 0.70` in the other. Nothing about the lidar core differs between those
rows. (The second row also shows the supervisor clipping the controller's −0.80 to its
−0.40 cap: see C5.)

And the per-abort picture is unanimous, not 10-of-12. In the 8 s before **each of the 12
aborts**:

| abort | camera-zeroed | lidar-latched | reverse commanded | net displacement |
|---|---|---|---|---|
| 1–12 (range) | **1.8 – 4.4 s, all 12** | 0 s in 10 of 12 | **1.6 – 4.1 s, all 12** | 0.10 – 0.47 m |

Every abort in the run ends in the same cycle, and the recorder shows it directly
(t=274–281, abort 9):

```
274.0  CLEAR  command     front 0.43  cmd(-0.10, 0.00)  out(-0.10, 0.00)   <- ladder rung 1, reversing
274.9  CLEAR  scan        front 0.48  cmd(+0.20, 0.00)  out(-0.10, 0.00)
275.0  SLOW   front_slow  front 0.49  cmd(+0.20, 0.00)  out( 0.00, 0.00)   <- camera stop; core wanted 0.14
...    (20 consecutive rows / 1.9 s, pose frozen at 1.380, 0.658)
277.0  CLEAR  scan        front 0.49  cmd(-0.10, 0.00)  out(-0.10, 0.00)   <- ladder rung 1 again
278.7  SLOW   front_slow  front 0.56  cmd(+0.20, 0.00)  out( 0.00, 0.00)   <- camera stop again
```

**The mission-killer, mechanism-complete:**

1. Controller commands cruise toward the goal.
2. A camera low-obstacle point lands within 0.50 m of the swept path. `forward_speed_scale`
   is **zero at ≤ 0.50 m** — a cliff, not a band — so forward output becomes exactly 0.
   Angular is untouched (the camera brake never limits rotation).
3. 20 consecutive suppressed cycles (2.0 s at 10 Hz) trip the ladder's condition 3. Not a
   freeze: output was zero, so the supervisor gets the blame and no mark is planted (this
   is correct, and it is why the run has 1 freeze mark and not 12).
4. Rung 1 reverses. The camera brake ignores reverse, so it is **always granted** — 0.12 m,
   `escape_distance_m` met, rung "cleared", control handed back.
5. The controller drives at the same goal on the same heading, and step 2 repeats.
   Invocation 2, then `max_invocations_per_goal` (2) is spent → `budget_exhausted` → abort.
6. The explorer counts an abort. Five in a row ends the mission.

**Corroboration from the launch log.** The ladder was invoked **25 times** in 365 s
(14 `position_and_yaw_stalled`, 11 `output_suppressed`). Every exhaustion it reported was
of one kind:

| exhaustion outcome | count |
|---|---|
| `budget_exhausted` — invocations spent, nothing left to try on this goal | **13** |
| `all_rungs_ineffective` — ran the whole ladder, none of it helped | **0** |
| `genuinely_wedged` — every rung refused outright | **0** |

(13 is the log-line count; two pairs land within 5 ms of each other from concurrent execute
loops, so the number of *goals* ending this way is at most 12. The vault records 7 for this
run — a narrower count. Nothing below turns on which is right: the zero rows are the point.)

**No ladder in this run ever ran out of rungs. Every one ran out of invocations.** That is
step 4–5 stated by the ladder itself: rung 1 keeps working (the camera never brakes
reverse, so the reverse is always granted and always "clears"), and the goal keeps
re-stalling on the same obstacle until the per-goal budget is gone. The ladder is not
failing here — it is succeeding, twice per goal, at an escape that does not address what
stopped the robot. A deeper or longer ladder would not have helped; the escape has to be
*sideways*, and it has to happen before the stop.

That is Scott's sentence — *"still going back and forth three times before giving up"* —
as a control-flow trace. It also explains why the newly-landed budget-spent outcome fired
so often in this run and was, every time, telling the truth: the budget really had been
spent, on an escape that really had been granted.

**What this changes about the design, and what it does not.**

- Scott's frame survives intact and generalises: *a stop should be an easy turn away.*
- The **gate to turn away from is the camera's, at 0.50 m**, not the lidar's SLOW band.
  A steering law that reads only lidar sectors would leave 63% of the zeroed time untouched.
- Steering must **engage well before 0.50 m** — §1.2 derives that the robot physically
  cannot get out of its own way inside the brake band at the deployed angular cap.
- The steering law's inputs must include the camera cloud. The controller does not
  subscribe to it today.

**What this note deliberately does NOT conclude:** whether those camera points were real
low obstacles (chair legs, couch skirt) or D27 sun phantoms. This run cannot say — the
recorder does not capture `cam_nearest`/`cam_scale` even though the supervisor already
publishes both on `/collision_stop/state`, and the cloud itself was not recorded. §6.4(b)
adds the two columns. Note that the answer does not change item 1: the rover must curve
around a phantom as gracefully as around a chair leg, and steering earlier is the correct
response to both. It changes only how loudly D27 should shout.

**Where the brief's number came from, and why both are right.** "Min front range inside the
brake envelope" is a true description of these rows — the lidar front sector *was* often
inside 0.50 m. It is a correct correlation and the wrong cause: at those ranges the lidar
core scales to 0.14 m/s and keeps driving. The distinction matters because the two
readings prescribe different fixes.

---

## 0b. BUILD CORRECTIONS (2026-08-11, during item 1's implementation)

Recorded here rather than edited into the sections below. An approved design note
that is quietly reconciled with whatever got built stops being a record of what was
approved — that is exactly how two clauses of `design_d25_freeze.md` drifted through
three reviews.

**BC1 — corridor half-width 0.12 → 0.18 m, engagement radius 0.85 → 0.90 m.** §1.2
derived the corridor from the robot's own half-width plus the trajectory margin
(0.10 + 0.02). Wrong reference: what stops the rover is the camera brake, and it
tests its *swept path* at `camera_half_width_m` **0.16**. Clearing the body still
leaves the obstacle inside the gate's corridor, so the fix would have stopped just
short of working. 0.18 = 0.16 + margin, and the derivation chain is unchanged —
`engage = stop_ref + R(1 − cos θ) travel`, now 0.50 + 0.384 → 0.90.

**BC2 — which half of the camera range filter is load-bearing.** §1.3 said a
consumer without the filter "would steer away from proof that the floor is clear".
A mutation run refuted that for *this* consumer at the deployed config: clear rays
sit at 1.8 m and the engagement radius (0.90) already excludes them, so deleting the
max-range filter changed nothing and the first version of the node-level test passed
for the wrong reason. Corrected: **the near limit (0.40 m) is load-bearing today** —
without it a point at 0.30 m, in the band the detector is not trusted in, becomes a
blocker at maximum urgency. The far limit is defence-in-depth, and it is now pinned
by a test that raises the engagement radius past the clear range so the protection
does not rest on a coincidence between two unrelated numbers.

**BC3 — a second instrument defect, found while grading and now fixed with §6.4(a).**
The rung logger's throttle hid every escalation line in run 114626. Named here
because it changes how the *next* run is read: rungs 2–4 executing with no `_failed->`
line in the log is a logging artifact, not evidence that they were never entered.

---

## 1. Item 1 — gentle turn-away

**Objective:** the rover curves around what stops it, while still moving, instead of
stopping into the ladder. It recovers most of the 86 s (28%) of asking-but-not-moving and,
more importantly, stops feeding the give-up counter.

### 1.1 Where it lives, and what it may touch

In the **decisive controller**, as a modification of the *target heading* before
`compute_drive_command`. Not in the supervisor, not in the camera brake, not in a new node.

The supervisor's role is **UNCHANGED and final**. It still clamps every command
(`max_forward_mps` 0.20, `max_angular_rad_s` 0.40), still owns the stop, still owns the
camera brake and the pivot veto. This design only changes what the controller *asks for*.
If the steering law asks for something dangerous, the supervisor refuses it exactly as it
refuses everything else today, and the ladder sees the refusal exactly as it does today.

**D27 boundary, checked explicitly:** Scott retired the optical low-obstacle path and
banned further optical fixes. This item changes no detector, no threshold, no brake — it
consumes `/camera/low_obstacles` read-only and reacts earlier. I read that as inside the
rule, but it is the supervising session's call to confirm, because it is the one place
this batch leans on the camera at all.

### 1.2 The engagement radius is derived from the robot, not the room

At cruise the supervisor clamps angular to **0.40 rad/s**, so the tightest arc actually
executable at 0.20 m/s has radius `R = v/w = 0.50 m`. Displacing the robot's own half-width
plus the trajectory margin (0.10 + 0.02 = 0.12 m) off a point dead ahead needs
`θ = acos(1 − 0.12/R)`:

| speed | R | θ | **forward travel needed** | time |
|---|---|---|---|---|
| 0.20 m/s (cruise) | 0.50 m | 40.5° | **0.33 m** | 1.8 s |
| 0.14 m/s (lidar SLOW floor, 0.70×) | 0.35 m | 48.9° | **0.26 m** | 2.1 s |
| 0.12 m/s (camera SLOW floor, 0.60×) | 0.30 m | 53.1° | **0.24 m** | 2.3 s |

Two conclusions fall straight out, and both are properties of this robot at this config:

1. **The timidity is structural, not a tuning failure.** An obstacle first *acted on* at
   0.50 m leaves 0.33 m of band before the camera cliff (0 m) or 0.325 m before the lidar
   stop (0.175 m). The turn needs essentially the whole band, with no margin for the
   detection lag or a second obstacle. There is no gain value that fixes this; the
   information has to arrive sooner.
2. **Slowing genuinely helps the turn** — halving speed nearly halves the forward run.
   Scott's "gentle turn inside the SLOW band" is right about the physics; the band just
   has to start further out.

**Engagement radius `avoid_engage_m` = 0.85 m**, derived as `camera_stop_distance_m (0.50)
+ forward-travel-at-cruise (0.33)`, rounded up. Both terms are deployed-config or robot
geometry. It sits inside the camera's useful range (`camera_max_range_m` 1.20) and far
inside the lidar's, so both sensors can supply it.

### 1.3 The steering law

Once per control cycle, before `compute_drive_command`:

```
blockers = lidar rays  with 0 < range < avoid_engage_m and |bearing| <= 90°
         + camera pts  with camera_min_range_m <= range <= camera_max_range_m   # 0.40 .. 1.20
                       and |bearing| <= 90°
```

For the nearest blocker at `(range r, bearing β)` whose bearing lies inside the corridor
the robot would actually sweep (`|r·sin β| <= half_width + margin`):

```
urgency  = clamp((avoid_engage_m - r) / (avoid_engage_m - stop_ref_m), 0, 1)
away     = -sign(β)            # β == 0 -> steer toward the more open side (see below)
delta    = away * urgency * avoid_max_rad          # a heading-error offset, in radians
heading_error' = wrap(heading_error + delta)
```

`compute_drive_command` then runs unchanged on `heading_error'`, so the existing
straight / arc / pivot regimes, the deadband and the goal tolerance all still apply and
nothing new can emit a raw twist.

Five clauses that are load-bearing, each with a reason that is not "it felt right":

- **`stop_ref_m` = `camera_stop_distance_m` (0.50)** — urgency reaches 1 exactly where the
  cliff is, so full steering is applied at the last moment it can still be executed.
- **`avoid_max_rad`** is capped so the *resulting* angular never exceeds the supervisor's
  0.40 rad/s. Asking for more is a command that gets silently clamped, which is how a
  controller ends up lying to its own ladder about what it requested. With `arc_gain` 1.2,
  0.40 rad/s is reached at a heading error of 0.33 rad, so `avoid_max_rad` = **0.33 rad**
  and the whole law is expressible as "up to one full arc's worth of lean".
- **Ties and dead-ahead (`β ≈ 0`)** resolve toward the side with more free space, using the
  circular widest-gap search the controller already computes for the ladder
  (`_on_scan` → `_open_bearing`, already TF-rotated into base_link). No new geometry, and
  the N1 laser-frame trap cannot reappear because that code already reads the rotation from
  TF and returns None when it cannot.
- **The camera cloud is NOT a pure obstacle cloud.** `low_obstacle_node` emits *clear-ray
  endpoints* at `clear_range_m` **1.8 m** into the same cloud, at the same `z=0`,
  indistinguishable from marks. The brake only avoids steering into them because
  `camera_max_range_m` is 1.20. **Any new consumer must apply the same range filter or it
  will steer away from proof that the floor is clear.** This is written here because it is
  exactly the kind of shared-topic trap that has cost this project three defects.
- **Rate limit** on `delta` (max change per cycle) so a single noisy frame cannot snap the
  heading; the camera runs at ~5 Hz into a 10 Hz loop, so every camera-driven blocker is
  two cycles stale half the time.

### 1.4 Interaction with the ladder — when a curve obviates a stall, and when it does not

**The ladder's predicate does not change.** No new suppression, no new exemption. What
changes is how often it is reached.

| situation | before | after |
|---|---|---|
| single obstacle, free space to one side | zeroed 2 s → ladder → reverse → re-approach → repeat → abort | curve begins at 0.85 m; output never reaches zero; ladder never fires |
| corridor closed on both sides | zeroed → ladder → rungs | steering finds no free bearing, `delta`→0, output zeroed → **ladder fires exactly as today** |
| ladder rung running (`ladder.active`) | — | **steering is bypassed**: the rung owns the command. A rung's whole point is that normal control has already failed |
| obstacle behind (reverse rungs) | — | steering is forward-only (`|β| <= 90°`, `x > 0`), so it cannot fight rung 1 or 2 |

The honest failure mode of steering is an **orbit** — curving around an obstacle forever
without approaching the goal. It is bounded by two mechanisms already in the tree: the
repulsion drops out once the blocker's bearing passes ±90°, and the explorer's
`goal_progress_timeout_s` (6 s of < 0.10 m progress) cancels a goal that is moving without
arriving. No new anti-livelock machinery. A pre-registered falsifier is in §1.6.

### 1.5 Arrival semantics — a goal satisfied at coverage radius

Four of the twelve aborts moved **≤ 0.14 m in their final 8 seconds** (0.098, 0.103, 0.125,
0.133 m — aborts 1, 11, 4, 8): parked in front of something, with the goal a short distance
beyond it. The brief's reading of the same episodes — 8 s at zero output with front 0.54 m —
is the same picture from the other side.

The disagreement being spent there is a contract mismatch. The controller's contract is
`goal_tolerance_m` **0.10 m from the path end**; the mission's contract is
`coverage_radius_m` **0.75 m from the target cell**. Those differ by 0.65 m, and every
metre of it is driven into an obstacle.

The explorer already knows this — it offers stand-off points at 0.5× and 0.9× coverage
radius (`_approach_points`) precisely so that "reaching any of them counts as covering the
target". It just has no way to *stop* once the last stand-off is also unreachable.

**Design: the explorer declares arrival, not the controller.** It owns coverage semantics;
the controller owns motion and cannot see the target cell. When *both* hold:

- the robot is within `coverage_radius_m` of the active goal's **target cell**, and
- the drive is being refused — `ladder_active` is true, or `_goal_stalled` would fire,

then the goal is **satisfied**: cancel it, count it as a success (which resets
`_consecutive_failures`, `_consecutive_freezes` and `_unstick_attempts` exactly as a
`SUCCEEDED` result does today), and reselect. The cell is already stamped covered, so it
will not be re-picked.

**Why this narrow trigger and not "cancel when covered".** Cancelling as soon as the target
became covered is a bug this explorer already had and already fixed: coverage radius 0.75 m
means a target 0.8 m away is covered after ~10 cm of driving, and the explorer reissued
every tick — 13 goals in 15 s, each halting the previous `follow_path`
(`coverage_explorer_node.py:483-495`). Gating on *blocked* keeps the commit-to-a-goal rule
intact: we only shortcut a drive that is going nowhere, and going nowhere is already the
condition under which the goal was about to die anyway. The choice is between ending it as
a **success that changes nothing about where we are**, and ending it as a **failure that
counts toward mission death**. The second is a lie about a goal whose purpose was achieved.

### 1.6 Item 1 — deletions, non-goals, revert-proofs

**DELETED / CHANGED:**

| what | where | why |
|---|---|---|
| `max_arc_angular_rad_s: 0.8` | `decisive_controller_node.py` default + any config | Unreachable fiction: the supervisor clamps angular to 0.40, so half of this parameter's range has never reached a motor. Set to **0.40** with the derivation in the comment, or delete and read the cap. A parameter whose top half is inert is how "commanded" and "achieved" drifted apart in the first place (D32's root). |
| abort-on-blocked-at-coverage-radius | `coverage_explorer_node.py` | Replaced by satisfied-at-coverage-radius (§1.5). The abort path stays for goals that are genuinely not achieved. |
| "the decisive controller's own back-off reflex handles the common boxed-in case" | `behavior_trees/navigate_to_pose_decisive.xml` comment | The back-off reflex was deleted in `dbbc8b1`. A comment describing a mechanism that no longer exists is the cheapest possible lying diagnostic. |

**EXPLICITLY NOT BUILDING:** any change to `collision_stop.py`, the camera brake's
distances/scales, or the pivot veto; any change to the low-obstacle **detector** (D27 is
parked, and this batch does not reopen it); a costmap layer for camera points (that is the
"route around" half, a separate decision, and it needs the phantom question answered
first); a new node; velocity-obstacle / DWA-style local planning (that is re-adding the
local costmap decisive mode exists to remove); any retune of `coverage_radius_m`.

**REVERT-PROOFS** (each must fail against the code it indicts, and be mutation-checked):

1. `steering_curves_before_the_brake_fires` — pure core. Synthetic corridor, blocker at
   0.60 m, 8° off axis, free space to the left. Pre-fix: commanded heading unchanged →
   the robot arrives at the blocker head-on. Post-fix: `|delta| > 0` away from the blocker
   and the projected path clears half-width + margin before 0.50 m. **Fails against HEAD**
   (no law exists). Mutation: zeroing `urgency` must break it.
2. `camera_clear_rays_never_steer` — a cloud containing only clear-ray endpoints at 1.8 m
   produces `delta == 0`. **Fails against a naive implementation** that consumes the cloud
   without the range filter — which is the implementation someone will write.
3. `steering_never_suppresses_the_ladder` — with the corridor closed on both sides, output
   is zeroed and the ladder fires on the same cycle it does today. Mutation: making
   steering emit a nonzero angular in the closed case must break it.
4. `rung_owns_the_command` — while `ladder.active`, the published twist equals the rung's
   twist exactly.
5. `replay_run_114626_abort_9` — replay the recorded (front, cmd, out) sequence of
   t=274–281 through the controller with a synthetic blocker reproducing the camera stop.
   Pre-fix reproduces forward→zero→reverse→forward; post-fix keeps output non-zero.
   **Honest limitation, stated in the test:** the raw scan and camera cloud were not
   recorded, so the blocker is reconstructed from the state line rather than replayed. This
   is weaker evidence than the chair-pin replay and must not be described as equivalent.
6. `goal_satisfied_at_coverage_radius_when_blocked` — blocked inside coverage radius →
   success, counter reset, cell not re-picked. **Fails against HEAD** (aborts and
   increments). Falsifier for the regression it could reintroduce, pre-registered:
   **goals-sent-per-minute must not rise** versus run 114626 (20 goals / 365 s); the churn
   bug looked like 13 goals in 15 s.

---

## 2. Item 2 — arrival-orientation stage

`NavigateToPose` carries a goal orientation. Nothing in decisive mode reads it: the
controller consumes only path *positions* (`decisive_controller_node.py:424`) and stops on
distance, and Nav2's `SimpleGoalChecker` (`yaw_goal_tolerance` 0.35) lives inside
`controller_server`, which decisive mode does not run. The return-to-start demo landed
position to 6.5 cm and missed orientation twice.

**Design: a final rotate-to-heading stage inside the decisive controller**, entered when
`compute_drive_command` returns `arrived` and a goal orientation is present, exited on a
settle criterion or a bounded timeout. It publishes pure pivots (`linear == 0`) through the
supervisor like everything else.

Four requirements, each already paid for by a failed demo:

1. **Reject impossible yaw readings.** Same failure family as D32: a slip burst or an
   estimator catch-up must not be integrated as rotation. The bound comes from the measured
   pivot contract (§5), not from the commanded rate — that substitution is what broke the
   grind-yaw guard. Bench segment A supplies the noise floor and B the achievable ceiling.
2. **Settle on a deadband held for N consecutive cycles**, never a single instant. A
   single-sample criterion on a signal that swings ±80° in 0.3 s under slip is a coin toss.
3. **Frame ruling, decided here.** The demo's second failure was regulating a pivot against
   a **still-settling map frame**: SLAM shifts `map→odom` on loop closure, so the error
   signal moved while the robot was correcting it. Ruling: **the target is captured ONCE in
   the map frame at stage entry and immediately converted to an odom-frame target; the
   control loop then closes on odom yaw only.** Map defines *where to point*; odom is the
   only frame stable enough to *regulate against* over a 1–3 s pivot. Consequence,
   accepted deliberately: a loop closure during the pivot leaves a bounded map-frame error
   — smaller than the pivot quantum below, and correctable by re-entering the stage rather
   than by chasing a moving reference.
4. **Written against the real pivot contract.** This is the requirement that decides
   whether the stage can work at all, and it is why segment D of the bench session is
   mandatory rather than nice-to-have:

   > At the measured rate (p50 **2.46 rad/s**, p90 3.70) and the 10 Hz control loop, **one
   > control cycle of pivot is ≈ 0.25 rad (14°)** — and one cycle is the smallest command
   > this stack can issue. A yaw tolerance below roughly `rate × period × 1.5 ≈ 0.37 rad
   > (21°)` is therefore **not achievable by pulsing at 10 Hz**; the loop will overshoot and
   > hunt. run_pivot.csv shows exactly that: three consecutive episodes at t=286–290 each
   > command ±0.40 for 1–3 s and net only **+52.7°, −6.2°, +29.3°**, while their per-sample
   > rates run at a median 1.4–1.9 rad/s with peaks to 3.7. Lots of turning, almost no net
   > rotation. That is a hunting loop, and it is the most likely mechanism of both demo
   > failures — a mechanism the frame ruling (3) alone would not have fixed.

   So the stage has two honest options, and the batch must pick one **from bench data**:
   **(a)** accept a ~0.35–0.40 rad tolerance and document that as the robot's angular
   resolution, or **(b)** shrink the quantum by lowering `pivot_min_duty` (28) toward the
   measured breakaway (~20), which is a driver change with its own bench proof. **(a)** is
   the default; **(b)** only if segment D shows the duty floor is what sets the rate.

**DELETED / CHANGED:** the explorer's `goal.pose.pose.orientation.w = 1.0`
(`coverage_explorer_node.py:857`, and the same line in `_planner_can_reach`). Today that
means "don't care" and is harmless because nothing reads it. **The moment the controller
honours orientation, every coverage goal would demand map-yaw 0** and the rover would
rotate to a fixed compass heading after every leg. The explorer must express
"unconstrained" explicitly — a flag on the goal, or a sentinel the controller understands —
*in the same commit* that adds the stage. This is a trap, not a detail: it converts a fix
into a mission-wide regression.

**EXPLICITLY NOT BUILDING:** a yaw controller with integral action or gain scheduling; use
of the map frame inside the regulation loop; any orientation requirement on coverage goals;
re-enabling Nav2's goal checker.

**REVERT-PROOFS:** `settle_requires_n_consecutive_cycles` (a single in-band sample amid
noise must NOT settle — fails against a single-instant criterion);
`impossible_yaw_is_not_progress` (a replayed slip burst must not satisfy the stage);
`target_is_captured_once` (shifting `map→odom` mid-stage must not move the target — fails
against a map-frame loop); `coverage_goals_do_not_rotate` (an unconstrained goal reaches
`arrived` and publishes no pivot — fails against the naive orientation.w = 1.0 version).

---

## 3. Item 3 — D32 grind-yaw guard redesign

**The state of the evidence, stated so nobody re-derives it:**

- The bound `max_yaw_rate_rad_s` **0.6** was derived from the supervisor's commanded cap
  (0.40 + margin). Real pivots run **2.9–4.2 rad/s** (D32; my own re-measure of
  run_pivot.csv over 17 episodes: per-sample p50 2.46, p90 3.70, max 4.42). Every real
  pivot is therefore "impossible".
- **Magnitude cannot discriminate**: real pivot 179 °/s vs chair-slip 169 °/s.
- **Sign-coherence is refuted and inverted**: slip coherence 1.00 (a single monotonic
  catch-up lurch) vs real-pivot 0.80.
- **The discriminator is UNIDENTIFIED.** This note does not propose one.

**Two consequences at HEAD, both worth writing down before the fix:**

1. *Rung 3 can never be credited* — the known D32 symptom. Its failure direction is
   benign: the ladder escalates to rung 4 instead of returning control, costing 3 s and
   emitting a wrong label.
2. *A granted pivot longer than 2 s trips the ladder **and plants a phantom freeze mark***
   — not previously stated. In `StallLadder.step` a real pivot exceeds the bound, so
   `turned` is zeroed **and the yaw reference is re-marked**, while position is stationary
   by definition; both progress measures are dead and `stall_time_s` (2.0) expires. Worse,
   the freeze vote is counting all the while: the supervisor *is* permitting the rotation,
   so `output_moving` is true every cycle (`_on_motor_out`'s angular floor is 0.05 rad/s
   against a granted 0.40), the majority vote passes, and `_begin` returns `freeze=True`.
   The controller then marks a place the robot could not pass — while the robot was
   turning perfectly well.

   **Honest status: code-derived, not yet observed.** Most pivots finish inside 2 s at
   3 rad/s, which is why it has not bitten. I checked the one freeze in run 114626 (t=139.9)
   in case it was this: it is not — output was a permitted (0.14, −0.40) for 2.0 s with the
   pose frozen at (−0.789, −1.028). That freeze was correctly classified. What makes this
   reachable is a *longer* pivot: a partly-refused one, or item 2's rotate-to-heading stage.
   **Item 2 must not be built on top of this.**

**Design: specify the measurement, not the answer.** The bench session produces two labelled
populations — clean commanded pivot (segment B) and provoked slip (segment E) — each with
gyro `ω_z`, wheel-odom `ω_z`, and IMU linear acceleration, time-aligned. Candidate
discriminators are then scored against **one pre-registered rule**:

> A candidate is accepted only if a single fixed threshold separates the two populations
> with **no overlap between the real-pivot p1 and the slip p99** over a sustained window of
> ≥ 3 consecutive samples, in **both** rotation directions. Anything weaker is recorded as
> dead, with its numbers, in this note's appendix — the same burial the three D27
> candidates got.

Candidates, in the order the physics recommends:

| # | candidate | why it might work | pre-registered prediction |
|---|---|---|---|
| A | **gyro vs wheel-odom disagreement** `|ω_gyro − ω_wheel|` | The physical difference *is* "body rotates" vs "tracks spin and body does not". The gyro measures the body directly; wheel odom measures the tracks. | Slip: large sustained disagreement. Real pivot: small. This is the candidate the whole IMU session exists to test. |
| B | **gyro magnitude alone** | During a slip the body barely rotates. | Slip `|ω_gyro|` ≈ 0; real pivot ≈ 2–4 rad/s. Simpler than A — if it holds, A is unnecessary. |
| C | **accel/vibration signature** | Grinding is high-frequency structure-borne. | Slip: elevated linear-acceleration variance. Weakest physically; test only if A and B both fail. |

**Precondition, and it is the real reason for the session:** no gyro data exists in any
recorded run. `enable_imu_fusion` has been false throughout; the driver *can* publish `imu`
(`rvr_node.py:203`) and never has. So **step one is establishing whether the gyro yaw is
trustworthy at all** (segment A). The 2026-08-03 fusion validation predates everything
current, and config is not measurement. If the gyro fails segment A, candidates A–C all
die at once and item 3 stops until the hardware question is answered — that outcome gets
written down, not worked around.

**Interim behaviour, pending the measurement:** leave the 0.6 bound **as-is**. It fails
toward escalation (safe direction) and changing it blind trades "never credit a real pivot"
for "credit a slip and hand control back while pinned", which is the D25 grind. What ships
in the interim is **honesty, not a number**: the rung telemetry stops claiming
`pivot_open_cleared` semantics it cannot support, and the comment stops asserting that
"anything above the supervisor's cap plus generous margin is physically impossible" — the
robot's ordinary pivot is 5× that cap, and the sentence is the reason the bound looked
sound through F4, N3 and a re-review.

**DELETED / CHANGED:**

| what | where | why |
|---|---|---|
| the `max_yaw_rate_rad_s` **rationale comment** ("bursts of ±80° in 0.3 s … anything above the supervisor's cap plus margin is physically impossible") | `stall_ladder.py:74-78` | The premise is false on this robot. Delete now, ahead of the fix — a comment that teaches the wrong model is how this bug survived F4, N3 and a re-review. |
| any test that feeds **commanded** rates to the guard | `tests/test_stall_ladder.py` (N3 family) | Measure-the-wrong-population, inside the guard built to catch impossible motion. Replaced by tests fed **achieved** rates from the bench recording. |
| the folklore line "big in-place pivots do NOT grind at the decisive rate (0.4–0.9 rad/s)" | vault Hard-won facts | 0.4–0.9 are *commands*. The achieved rate is 2–4 rad/s. Same substitution, different document. |

**EXPLICITLY NOT BUILDING:** a discriminator asserted without data; a contact/stall detector
from the yaw signature (real, separate defect); any change to the freeze classifier; IMU
fusion as a *deployment* decision (this batch measures the gyro, it does not adopt it).

**REVERT-PROOFS:** `real_granted_pivot_is_credited` (replay segment B → rung 3 clears —
fails against HEAD); `replayed_chair_pin_is_still_rejected` (replay the run-142641 trace →
not credited — this is the mutation guard on the fix, and it is the one that catches an
over-loose threshold); `long_granted_pivot_does_not_trip_the_ladder` (4 s granted pivot at
the measured rate → no ladder invocation **and no freeze mark** — **fails against HEAD on
both counts**, per consequence 2 above).

---

## 4. Item 4 — the pivot contract, documented from data

Read out of the deployed code and confirmed against `run_pivot.csv`. **Every clause below
is marked with what confirms it**, and the unconfirmed ones are the point of the bench
session.

**C1 — Sign-only input.** For `|linear| < 0.005` and `|angular| > 0`, the driver takes only
`sign(angular_z)`; the magnitude is discarded (`driver.py:708-740`). *(Confirmed: code +
field — commanded 0.40 and 0.90 produce indistinguishable rates.)*

**C2 — Closed-loop duty, wheel-odom feedback.** The loop regulates `|measured_yaw_rate|`
toward `pivot_target_rate_rad_s` (**1.3**, node default, absent from
`lean_rvr_tank_si.yaml`), integrating `duty += gain(1.0) × error` each 0.05 s control
period, clamped to `[pivot_min_duty 28, pivot_max_duty 45]`. The feedback signal is
**encoder-derived** yaw rate from `_publish_odom_sample` at 10 Hz (`rvr_node.py:563`) —
i.e. **the same wheel-odometry signal that cannot tell a pivot from a slip.** *(Confirmed:
code.)*

**C3 — The loop is saturated at its floor, so the target is inert.** Measured rates
(2.5–4.2) exceed the 1.3 target continuously, so the error is always negative, the
integrator walks to `pivot_min_duty` within ~15 cycles (0.75 s), and every pivot longer
than a second runs at **duty 28 open-loop**. The "self-regulated rate" is simply what duty
28 does on this floor. *(Inferred from code + measured rates. **Bench segment D is the
test.**)*

**C4 — Achieved rate.** 17 pivot episodes in run_pivot.csv (`out_vx == 0`, `|out_wz| > 0`,
≥ 0.5 s): per-sample `|ω|` **p50 2.46, p90 3.70, max 4.42 rad/s**. Sustained episodes:
−151.8° in 0.9 s (mean 2.94 rad/s) and −585.6° in 4.9 s (mean 2.09 rad/s). The same holds
inside the gauntlet mission, not just the demo: run 114626 t=144.7–145.4, commanded
(0.0, +0.40), yaw −40.2° → +88.1° — **128° in 0.7 s = 3.2 rad/s**. Sign is honoured;
magnitude is not. *(Confirmed: field, two independent runs — but from **wheel odom**, which
segment B exists to corroborate against the gyro.)*

**C5 — The supervisor is the final angular gate.** `max_angular_rad_s` **0.40** clamps every
command, so the controller's `pivot_rate_rad_s` 0.9 and `max_arc_angular_rad_s` 0.8 are both
clipped before they reach the driver, which then ignores the magnitude anyway. **Two layers
of magnitude are inert.** *(Confirmed: code + field — cmd −0.90 → out −0.40 in run_pivot at
t=127.6; cmd −0.80 → out −0.40 in run 114626 at t=137.3, a whole second of it.)*

**C6 — Consequences that other items must respect.**
- The smallest issuable rotation is one control cycle ≈ **0.25 rad (14°)** → item 2's
  tolerance floor.
- To make a *slower* pivot, lower `pivot_min_duty`; changing `pivot_target_rate_rad_s`
  does nothing while C3 holds.
- Any yaw-rate threshold anywhere in this stack must be derived from C4, never from C5.

**Where it lands:** these clauses become `docs/pivot_contract.md` **after** the bench session
confirms or corrects them, and the confirmed version is copied into the driver docstring at
`driver.py:708`. One document, no new code, no new tooling.

**REVERT-PROOF:** `pivot_ignores_commanded_magnitude` — a driver unit test asserting that
`set_velocity(0, 0.4)` and `set_velocity(0, 3.0)` emit the same duty. It fails the day
someone makes magnitude matter, which is the day every threshold in this note needs
revisiting.

---

## 5. The merged bench session — measurement prerequisite for items 2, 3 and 4

**One chassis-powered session, ~15 min, Scott attending.** It is the first recording in this
project's history to contain gyro data. Item 1 consumes **nothing** from it and must not
wait behind it.

**Preconditions** (all checkable before power-on): rover **on the floor**, not a desk — a
desk bench cannot slip and cannot corroborate traction, and the whole session is about what
the tracks do against the floor; ≥ 0.5 m clearance all round; battery ≥ 25%; clock
synchronised (`timedatectl`); Pi at the reviewed SHA and rebuilt; Scott within reach of the
power switch throughout; teardown promptly on completion (an idle stack has flattened this
battery before).

**Configuration:** launch with `enable_imu_fusion:=true`. That is the only switch that makes
the driver publish `/imu` (`supervised_rvr.launch.py:46`). **Recorded confound, checked in
advance:** the same switch also starts the EKF and moves `odom→base_link` TF ownership to
it. It does **not** change what this session measures — `/odom` still carries raw
wheel-derived yaw rate, `/imu` carries raw gyro, and the closed-loop pivot keeps feeding on
wheel odom either way (`rvr_node.py:563` is unconditional). Analysis reads **topics, not
TF**, so the EKF sits outside every measurement below. Stated explicitly because "the
instrument changed the thing it measured" is a plausible objection and it has been checked
rather than dismissed.

**Recording:** `ros2 bag record /imu /odom /cmd_vel /cmd_vel_motor /collision_stop/state
/diagnostics /tf /tf_static` — a standard tool, no new code, and it captures the raw gyro
stream the CSV recorder has no column for. Run `run_recorder.py` alongside as usual for the
state line. Per the scaffolding cap: **no new probe, no new harness** for this session.

| seg | what | duration | procedure | consumed by |
|---|---|---|---|---|
| **A** | **Gyro trust** | 3 min | (i) 30 s stationary → gyro-z bias and noise floor. (ii) Scott marks the start heading (tape/floor reference) and calls the finish; command one 360° pivot each way; compare **integrated gyro yaw** against Scott's observation. The human is the independent measurement — the point is not to infer trust from a second instrument that shares the failure. **If this fails, segments C–E are void and item 3 stops.** | 2, 3, 4 |
| **B** | **Clean commanded pivot** | 3 min | Open floor, no goal, no hunting loop: publish a fixed `Twist(0, ±0.4)` through `/cmd_vel` (the deployed path, supervisor included) for 5 s each direction. | 2, 3, 4 |
| **C** | **Commanded-magnitude sweep** | 2 min | Same, at `angular_z` ∈ {0.2, 0.4, 3.0} (the supervisor clamps to 0.4; the driver ignores magnitude). **Prediction: all three produce the same achieved rate.** Confirms C1 by measurement instead of code reading. | 4 |
| **D** | **Duty-floor test** | 4 min | Repeat B with `pivot_min_duty` 28 → 22 (driver restart between values; the parameter is read at construction). **Prediction (C3): the achieved rate drops with the floor and is unchanged by `pivot_target_rate_rad_s`.** Decides item 2's option (a) vs (b). | 2, 4 |
| **E** | **Provoked slip** | 3 min | Rover pinned against a heavy immovable object — the desk chair, the same object as the D25 contact — and commanded to pivot for ~3 s each way. **Safety: no hands near the tracks; obstacle placed, not held; Scott's finger on the power switch; abort on any smell/heat/stall alarm.** ~3 s is enough for the signature and short of thermal risk. | 3 |

**What each item consumes, explicitly:**

- **Item 1 (gentle turn-away):** nothing. Gated only on this note's review.
- **Item 2 (arrival orientation):** A(i) → the impossible-reading bound and the settle
  deadband floor; A(ii) → whether a gyro-closed loop is even an option; B → the per-cycle
  pivot quantum that sets the achievable yaw tolerance; D → whether the quantum can be
  shrunk (option (b)).
- **Item 3 (D32 guard):** A → gyro trustworthiness, the gate on everything else; B + E →
  the two labelled populations the discriminator rule (§3) is scored against.
- **Item 4 (contract):** A–D → C1 through C5 promoted from code-read to measured, or
  corrected.

**Session-level falsifier, pre-registered:** if segment B's gyro-integrated rotation and
wheel-odom-integrated rotation agree to within the noise floor *and* segment E shows the
same agreement, then no gyro-based discriminator exists, candidates A and B of §3 are dead
on the same day, and item 3 is reported as blocked on hardware rather than quietly retried.

---

## 6. Batch-level

### 6.1 Sequencing

1. **Item 1 alone, first**, in its own commit(s). It is the mission-killer, it is gated on
   nothing, and the gauntlet cannot resume without it.
2. The bench session.
3. Items 4 (contract doc), 3 (guard), 2 (orientation stage) — in that order, because 3 and
   2 both consume 4's numbers, and 2 must not be built on the ladder defect in §3.

### 6.2 Full deletion list

| # | deleted / changed |
|---|---|
| 1 | `max_arc_angular_rad_s` 0.8 → 0.40 (or derived); abort-on-blocked-at-coverage-radius → satisfied; the stale back-off-reflex comment in the decisive BT |
| 2 | the explorer's implicit `orientation.w = 1.0` "don't care" |
| 3 | the false `max_yaw_rate_rad_s` rationale comment; commanded-rate-fed guard tests; the "0.4–0.9 rad/s pivots" folklore in the vault |
| 4 | nothing deleted; `pivot_target_rate_rad_s` marked **inert** pending segment D, then either deleted or made effective |

### 6.3 What this batch is explicitly NOT building

No edit to `collision_stop.py`, the camera brake, the pivot veto, or any D17–D21 gate — the
batch is verifiable by **empty diffstat on the safety path**. No optical-detector work (D27
parked). No new node, no new probe, no new harness. No costmap layer for camera points. No
IMU-fusion deployment decision. No discriminator asserted ahead of its measurement. No
retune of `coverage_radius_m`, `stall_time_s`, or any supervisor distance.

### 6.4 Two instrument fixes, both to instruments that already exist

**(a) The rung log throttles away the evidence it exists to produce.** The ladder's single
`get_logger().info` carries `throttle_duration_sec=1.0` and serves both the per-cycle
`{rung}_running` lines and the one-shot `{rung}_failed->{next}` transitions
(`decisive_controller_node.py:546-551`). A transition that lands within a second of the
previous line is dropped. In run 114626 that is not hypothetical: `reverse_arc_running`,
`pivot_open_running` and `drive_open_running` all appear in the log while **not one**
`_failed->` transition does — so the record shows rungs 2–4 executing with no trace of how
they were entered, and I misread the run once because of it before checking. Fix: throttle
the `_running` lines only; escalations and exhaustions log unconditionally. They are rare
by construction, and they are the ladder's entire testimony.

**(b) Two recorder columns.** Add **`cam_nearest`** and **`cam_scale`** to `run_recorder.py`.
Both are already published on `/collision_stop/state` (`collision_stop_node.py:614-616`) and
neither is captured. Their absence is why §0's attribution had to be *derived* from
`min_forward_scale` arithmetic instead of simply *read*, and why this run cannot say whether
the camera stops were chair legs or sun phantoms.

Neither fix is a new tool, probe or harness: one is a log-throttle correction, the other two
fields on an existing instrument — the same class as the sanctioned D22 column, and the
direct application of the standing rule that the component which knows a fact should publish
it rather than have a peer infer it.

### 6.5 Open questions for the supervising session

1. **Premise correction (§0)** — item 1 is designed against the camera brake as the dominant
   gate. Confirm before I build, since it changes the steering law's inputs.
2. **D27 boundary** — the controller consuming `/camera/low_obstacles` read-only: inside or
   outside Scott's "no further optical fixes" ruling? I read it as inside (no detector
   change, no brake change) but will not assume it.
3. **Item 2 option (a) vs (b)** — accept a ~0.35 rad orientation tolerance as the robot's
   angular resolution, or spend a driver change on `pivot_min_duty` to earn a finer one.
   Segment D decides the physics; the trade-off is a product call.
4. **Session length** — segments A–E total ~15 min of motion. Segment D costs two driver
   restarts. If the session must be shorter, D is the one to cut, and item 2 then defaults
   to option (a) without further discussion.
