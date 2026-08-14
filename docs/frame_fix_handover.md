# HANDOVER — the ToF frame fix, and everything that rides with it

Written 2026-08-13 at the end of a long session, deliberately *instead of* doing the
work tired. Everything below is verified; nothing is guessed. A fresh session should be
able to execute this without re-deriving the analysis.

**Do not start this by editing constants.** The constants are wrong because the *frame*
is wrong in three places. Fix the frame, then re-derive. Editing numbers first produces
a stack that is self-consistently wrong.

---

## 1. What is broken — three bugs, one root cause

The word **"range"** is used as if it had one meaning. It has two: a distance measured
**from the sensor** (what the ToF reports) and a distance measured **from `base_link`**
(what every consumer assumes). The sensor sits **0.10 m forward** of `base_link`.

| # | Where | What it does | Error |
|---|---|---|---|
| 1 | `zone_point` (`tof_frame.py`) | omits `mount_x_m` entirely; clouds publish in `base_link` | every point **0.10 m too near** |
| 2 | `rule_a_applies` | compares `_floor_reading_m` (a SENSOR reading) against `stop_distance_m` (a base_link concept) | the authority bound, and therefore the ROW SET, decided in the wrong frame |
| 3 | `rule_b_applies` | derives a return's HEIGHT from a `range_m` its caller passes in **base_link**, using a formula that assumes a ray length **from the sensor** | **+22 mm of height error at 0.40 m**, growing with range |

**Bug 3 is the trap.** It is not fixed by adding `mount_x_m` — adding it makes bug 3
*worse*, because the caller's `ground` grows by 0.10 m while the formula still treats it
as a ray length. Anyone who applies the parked patch and stops there has made the height
gate wrong in exchange for making the position right.

**How #1 was found**, for calibration on how much to trust the rest: bench item J's very
first 20 s probe, before the wall was even staged. The ToF read a **median 0.10 m nearer
than the lidar on the same surfaces, constant from 0.72 m to 1.90 m**. A constant offset
across a 2.6× range change is a missing translation; a sensor artifact would have scaled.
Data: `03_validation/escape_provocation_2026-08-13/J_probe2.csv`.

---

## 2. The fix: make the POINT the single source of truth

Not three patches — one refactor that kills the class:

* `zone_point` returns **true `base_link`** `(x, y, z)`, including `mount_x_m`.
* Every downstream predicate reads **that point's own `z` and ground range**, instead of
  re-deriving from a scalar whose frame is ambiguous.
  * `rule_b_applies` should take the point (or its `z`) and test `z < lidar_plane_m`.
    The height is already correct in the point; recomputing it is what introduced bug 3.
  * `rule_a_applies` needs a **floor ground-range in `base_link`** helper to compare
    against the authority bound — not the raw reading.
* `TofConfig` gains `mount_x_m: float = 0.10`; the node already declares the parameter
  and must now pass it through (it currently feeds only the static TF).

The parked starting patch is `docs/frame_fix_starting_patch.diff` — it does **only bug 1**
and is a starting point, not the fix. It leaves three tests failing, correctly.

---

## 3. Re-derivation checklist — every constant, with its operand named

The operands lesson applies throughout: *the arithmetic was right every time; the
operands were wrong*. For each, name the frame before computing.

| Constant | Where | Operand it must be derived FROM |
|---|---|---|
| rule A reach | derived | floor reading **minus `floor_margin_m`**, converted to base_link ground range. Was 0.298 m in the sensor frame; expect ~0.398 m true. **Not** the floor distance — the margin sits between them |
| `stop_distance_m` (rule A authority bound) | `TofConfig` | must be a **base_link** distance now that it is compared against one. Currently 0.45 in a mixed frame |
| `low_obstacle_stop_distance_m` | `config/collision_stop.yaml` | `> footprint_front 0.11 + payload_margin 0.02 + braking_margin 0.02` (= 0.150; the config comment's 0.045 braking term gives 0.175 — **prefer 0.175 while `floor_margin_m` is provisional**), and `<` rule A's true reach. **Never** compare against the lidar's `stop_distance_m: 0.30` — that is a THRESHOLD, not a requirement, and confusing them produced a retracted conclusion (design §11.2) |
| `low_obstacle_slow_distance_m` | same | `>=` rule A's true reach, so the rover is easing off through the whole approach rather than braking flat from cruise |
| `low_obstacle_min_range_m` | same | just below the **nearest reportable ground range in the true frame** (0.052 + 0.10 ≈ 0.152 m) |
| `low_obstacle_max_range_m` | same | `>=` rule B's true reach (0.498 + 0.10 ≈ 0.598 m) so pinning rule B needs no second change |
| `low_obstacle_max_age_s` | same | `age × cruise (0.20 m/s)` **must stay below the approach window** (reach − stop). The window moves with the re-derivation, so this moves too. ~2 frames at the measured 6.5–7.6 Hz |
| `min_obstacle_height_m` | `TofConfig` | **unchanged at 0.018** — `mount_x_m` is an x-offset and does not touch z |

**Sanity check that will catch a frame slip immediately:** after the fix, re-run J's
probe analysis. The median per-column ToF-vs-lidar disagreement should collapse from
**~0.10 m to ~0.00 m**. If it does not, the frame is still wrong somewhere.

---

## 4. Riding with this batch

* **Split the abort counter (ruled 2026-08-13).** `_goals_aborted` conflates *"we tried
  here and could not move"* with *"we never got to try"*. Count them separately so the
  mission report names which kind dominated. Evidence and mechanism:
  `docs/reverse_before_give_up_design.md`, amendment 2026-08-13.
* **`consumers=none_stage_i`** in `tof_node.py`'s state line. False since the supervisor
  subscribed — a literal that lies, the exact class the parameter rename was for.
* **D31 flake sighting**, for that defect's census: `test_driver_safety`'s
  stale-command test, 2026-08-13, failed 1 of 4 runs under back-to-back suite load,
  passed 3/3 alone and in every full run since.
* **Gauntlet no-count rule**, to be written into the gauntlet notes: if a run's
  `ABORTED_GOALS_KEEP_FAILING` ending is dominated by never-tried aborts, the run does
  **not** count toward the three, and the starvation root cause jumps the queue.

---

## 5. Assigned to a fresh session — the starvation analysis (NOT tonight, not blocking)

**Why does ladder execution starve `FollowPath` goal acknowledgement?**

Observed: while the controller ran the ladder continuously for 11 s on one goal,
`bt_navigator` logged *"Timed out while waiting for action server to acknowledge goal
request for follow_path"*, the goal aborted, and the explorer counted it — with nothing
attempted on that goal.

The node already uses a `MultiThreadedExecutor` with a `ReentrantCallbackGroup`
**precisely to prevent this**, so one of three things is true, and the analysis should
say which:

1. the ladder holds a lock the goal-acceptance path also needs;
2. the executor's threads are exhausted;
3. the callback-group assignment does not actually cover the goal-acceptance callback —
   i.e. it *says* reentrant and is not.

Option 3 would be the unreachable-clause pattern again: a mechanism that reads as
protection and is not wired to the thing it protects. **Check that first** — it is the
cheapest to confirm and the most likely given this week's record.

---

## 6. Before flying anything built from this

1. Pi: pull, rebuild, **installed-tree verify by import and sha256** — not by reading the
   source tree. The Pi has silently run stale installed code before.
2. Bench item J re-run (~5 min, chassis off): bare wall for the per-column distribution
   that sets rule B's margin (**pass = disagreement inside the margin in ≥99% of frames**,
   margin set FROM that distribution, never guessed), then the 5 cm object.
   **The background is PER COLUMN — a uniform number is not a stand-in for it.** Feeding
   a flat value produced a retracted finding; see design §9.8.
3. Rule B's gate flips only with a `# RULE B PINNED BY: <capture>` citation in the
   deployed config. A test enforces it, and rejects placeholders.
4. If Scott has flashed the SEN0628 **v1.3 firmware**, the rail retest folds into the
   same bench block, and every `(F)`-tagged number in the design note is due for
   recomputation rather than re-argument.

---

## 7. State at handover

* `main` = `c3d68bd`, pushed, tree clean, **696 tests pass**.
* Pi = `5b9d842`, stack **down**, graph verified at 0 nodes, lidar motor stopped by
  service. Chassis off. Battery was 95%.
* Deployed binary is **honest but known-biased**: the ToF holds the low-obstacle brake,
  rule A live, rule B gated, and every ToF range is 0.10 m short — which brakes **early**,
  the safe direction, but makes the numbers wrong.
* Unspent: the give-up escape's field observation (its branch is not reachable by hand
  staging — the gauntlet is expected to produce it organically), and gauntlet missions
  1–3.

---

## 8. The starvation analysis — ASSIGNED IN §5, ANSWERED HERE (2026-08-13)

**No fix was made. This is analysis only, as instructed.**

### The short version

**The premise of the question is wrong.** §5 asks "why does ladder execution starve
`FollowPath` goal acknowledgement?" — it does not. The same run contains a **47.0 s
continuous ladder episode with zero acknowledgement timeouts**, and the one timeout on
record happened during a *shorter* (12.1 s) episode. All three candidates in §5 are
intra-node concurrency defects, and the field evidence shows the starvation **crossed
process boundaries**, which no callback group or thread pool inside
`decisive_controller_node` can do.

The mechanism the evidence supports is **host-level CPU saturation**. The ladder is a
measurable *contributor* to that load, not the cause.

### The corpus, and the honest size of it

Every archived launch log was searched. `"acknowledge goal request"` appears **exactly
once in the entire recorded history of this project** — at t+645.9 s of the 997 s
contaminated run of 2026-08-13 (`03_validation/CONTAMINATED_mission_2026-08-13_mismounted_camera/launch_20260813_173529.log`).
Ten launch logs, one event. **n=1**, and every conclusion below inherits that. This
analysis can refute the ranked candidates, because refutation needs only one clean
counter-example and there is one; it cannot establish the positive mechanism with the
confidence a fix would want.

### Candidate 1 — "the ladder holds a lock the goal-acceptance path also needs"

**REFUTED by code read.** `_follow_path_goal_callback` acquires nothing. Its entire
body reads one boolean (`self._escape_active`), optionally logs, and returns a
`GoalResponse`. There is no `with` block in it and no call that can block. It cannot
queue behind `_execute` because it never asks for anything `_execute` holds.

Worth recording for the future, since it is the near miss: `_escape_goal_callback` —
the *other* goal callback on this node — **does** take `self._goal_lock`, which
`_execute` also takes. Today that critical section is two statements and nothing
blocks, but this is where candidate 1's shape would become real if the lock were ever
widened. Pinned by `tests/test_goal_acceptance_is_nonblocking.py`.

### Candidate 3 — "the callback group does not actually cover goal acceptance"

**REFUTED by code read.** Both action servers are constructed with
`callback_group=self._callback_group`, and that group is a `ReentrantCallbackGroup`
(`decisive_controller_node.py:324-356`). Acceptance is covered by the assignment. This
was the ranked candidate — "check that first, it is the cheapest to confirm and the
most likely given this week's record" — and it is simply not what happened. The
unreachable-clause pattern did not repeat here.

*(Caveat, stated because it is the limit of a code read: whether rclpy's `ActionServer`
honours that kwarg for every entity it owns could not be verified from source on this
host — there is no rclpy on the Mac. What follows does not depend on it.)*

### Candidate 2 — "the executor's threads are exhausted"

**NOT SUPPORTED as the primary mechanism**, on evidence that also disposes of 1 and 3:
in the ±10 s around the timeout, processes *other than the controller* were missing
their deadlines at many times their own baseline for the run.

| Symptom | Process | Run baseline | ±10 s of the timeout | Ratio |
|---|---|---|---|---|
| "Failed to meet update rate" | `ekf_node` | 0.13/s | 1.30/s | **10.3×** |
| serial "timed out waiting for response" | `rvr_node` | 0.02/s | 0.30/s | **15.7×** |
| costmap "timestamp earlier than the transform cache" | `planner_server` | 0.05/s | 0.25/s | 4.6× |
| "message filter queue is full" | `slam_toolbox` | 0.03/s | 0.10/s | 3.7× |

`ekf_node` and `rvr_node` are **separate processes with their own executors**. Nothing
about `decisive_controller`'s callback groups or its thread pool can starve them.
Whatever was short, it was short host-wide. The acknowledgement timeout is one victim
in a burst, not the event itself.

### The control case — what actually kills the ladder hypothesis

Ten ladder episodes in this run, 128 s total (12.9% of it). **One** contained an
acknowledgement timeout, and it was not the longest:

| | duration | ack timeouts | EKF misses |
|---|---|---|---|
| the run's LONGEST ladder episode (t+786.9) | **47.0 s** | **0** | 0.468/s |
| the episode with the timeout (t+634.6) | 12.1 s | 1 | **1.318/s** |

At bt_navigator's measured ~1 Hz `follow_path` resend rate, the 47 s episode delivered
roughly **47 consecutive acknowledgements during uninterrupted ladder execution**. If
ladder execution starved acknowledgement, that episode could not exist.

The ladder is nonetheless a real contributor to load: EKF deadline misses run at
**0.390/s during ladder episodes vs 0.087/s outside them — 4.5×**. It pushes the host
toward the cliff. It does not by itself go over it.

### What separates the two episodes — and what does not

* **Not duration.** The clean one was 3.9× longer.
* **Not the controller's own health.** Its 1 s-throttled `_running` lines came at
  median 1.15 s in the failing episode vs 1.06 s in the clean one — and the worst
  single gap in the whole run (4.48 s) is in the **clean** episode. The controller was
  not conspicuously more starved when the acknowledgement failed.
* **Not per-task cost.** EKF cycle overruns during the failing episode: median 102 ms,
  max 196 ms — against a run-wide median of 104 ms. Individual cycles were *not*
  slower.
* **It is CONTENTION FREQUENCY.** Same per-cycle cost, 2.8× as many missed deadlines,
  and the timeout sits inside the single largest EKF-miss burst of the run (26 misses
  in 9.7 s). More things wanting the CPU at once, not any one thing taking longer.

### A correction to §5's own account

§5 records that the goal "aborted, and the explorer counted it — with nothing attempted
on that goal." For this instance that is **not so**. `bt_navigator` began navigating to
(0.24, −0.84) at 1786661252.923; the ladder ran from 1786661255.236 to 1786661268.302
and tried **four rungs** (`pivot_open` → `drive_open` → `reverse_arc` →
`reverse_straight`), ending `all_rungs_ineffective`. The rover attempted a great deal
on that goal. The abort-counter split proposed in §4 stands on its own evidence, but
this event is not an example of "we never got to try".

### What would settle it, and what should not be built yet

The §5 fix candidates all target intra-node concurrency, which the evidence does not
support. Building one now would be fixing the mechanism we can see the code of rather
than the one the logs point at.

What is missing is per-process CPU during a run. The recorder captures none, so
"the host was saturated" is an inference from four independent deadline-miss streams
rather than a measurement. **One `top -b`-class sample stream alongside the next
mission recording would convert every ratio in this section into an attribution** —
and it costs nothing, needs no chassis, and would also serve D22's open question about
where between 1× and 6× load the camera-age cliff sits.

Only then is it worth asking whether the ladder's contribution should be reduced.

### Artefacts

* `tests/test_goal_acceptance_is_nonblocking.py` — structural guard: acceptance stays
  lock-free and non-blocking, both servers stay on the reentrant group, the escape's
  critical section stays a plain state read. Five mutations, each caught.
* **The reproducing test assigned in the brief (fake action client, ladder busy,
  measure ack latency) was NOT built.** There is no rclpy on the Mac, so there is no
  real action server, executor or acknowledgement to time; a hand-rolled executor model
  would only prove things about the model. It is also now aimed at a refuted mechanism.
  If a latency test is still wanted, it belongs on the Pi and should measure
  acknowledgement latency **against host CPU**, which is the variable this analysis
  says matters.

---

## 9. CORRECTIONS TO THIS DOCUMENT, made while executing it (2026-08-13)

Appended, not edited, on the same basis as §8. The handover asked to be treated with
suspicion; this is what the suspicion found. **Two of §1's three bugs were live. The
third was not, and its dormancy is the strongest argument for the shape §2 prescribes.**

### 9.1 Bug 3 was LATENT, not live — and the two errors cancelled EXACTLY

§1's table lists bug 3 as an error in the deployed binary, "+22 mm of height error at
0.40 m, growing with range". Measured against the shipped code at row 3, column 3, a
500 mm reading:

    zone_point's z                     0.0286 m
    rule_b_applies' reconstruction     0.0286 m      error 0.0000 m

They agree to the last digit, and they must: `rule_b_applies` reconstructed the height
from a ground range that its caller computed as `hypot(point.x, point.y)`, and while
`zone_point` omitted `mount_x_m` that ground range was exactly `r · hypot(ux, uy)` —
precisely the quantity the reconstruction assumed. **Bug 1 and bug 3 were the same
omission, applied twice, cancelling.** Apply the parked patch and the same zone gives:

    zone_point's z                     0.0286 m
    reconstruction                     0.0060 m      error −0.0225 m

So §2's warning is right and its reasoning needs one correction: the parked patch does
not make an existing error worse, it **ACTIVATES a dormant one**. The distinction
matters for the deployed binary's status — §7 calls it "honest but known-biased", and
that remains true: the height gate in flight was correct, and only positions were short.

### 9.2 The activation is in the PERMISSIVE direction

The reconstruction UNDER-states z. A return that looks lower than it is looks further
below the lidar plane than it is, so `rule_b_applies` would extend rule B's authority
**above** the plane — where the lidar sees perfectly well and disagreement means
occlusion geometry, not a sub-lidar object. That is the unsafe side, and it is the
second reason the parked patch must never ship alone.

`tall_enough_to_matter` was already immune because it reads `point[2]` directly. It is
not a lucky exception; it is the pattern, and the refactor is that pattern applied to
every predicate.

### 9.3 §3's predicted constants — two confirmed, one wrong in a way that matters

| §3 predicted | measured in the true frame | |
|---|---|---|
| rule A reach ~0.398 m | **0.397 m** for row 4 | confirmed |
| rule B reach 0.598 m | **0.598 m** for row 3 | confirmed |
| nearest reportable 0.152 m | **0.1517 m** | confirmed |
| rule A's reach becomes ~0.398 m | **it becomes 0.297 m** | **wrong, and instructively** |

Every row's reach moved by +0.100 m as predicted. But rule A's reach is the reach of
the furthest row that still holds AUTHORITY, and the authority bound moved too — a
zone's floor is 0.070–0.108 m further from `base_link` than it READS, because that
comparison crosses the boresight projection as well as the mount offset. **Row 4 fell
out of the row set** (its floor is 0.512 m from base_link, outside the 0.45 m bound),
and the reach fell back to row 5's 0.297 m.

The number barely moved (0.298 → 0.297) and the row that supplies it changed. Anyone
checking this batch by confirming that the reach "still looks about right" would have
seen nothing.

### 9.4 `stop_distance_m` did not need to move, and now has an operand

§3 says it "must be a base_link distance now that it is compared against one" without
naming what sets it. Derived: rule A may conclude only where a WALL is already
commanding a stop from another authority, which is the lidar supervisor's own
`stop_distance_m: 0.30` — a base_link distance (the scan is transformed before any
range comparison). Rule A's reach must therefore stay inside 0.30 m: row 4's 0.397 m
does not, rows 5–7 do. Expressed on the floor's ground range, admitting {5,6,7} and
excluding 4 requires a bound in **[0.421, 0.5125)**, and the shipped 0.45 sits inside
it. Unchanged value, derived provenance.

This is **not** the comparison design §11.2 retracted. That error measured a detection
REACH against 0.30 as though 0.30 were a physical stopping requirement. This asks when
another brake acts, and a threshold is the correct operand for that question.

### 9.5 The fixture layer carried the same bug, in two places

Neither is in §1's table, and both were making tests pass for the wrong reason:

* `TILT_WALL_060_SENSOR_M = 0.578` is a SENSOR-frame distance whose comment claimed "a
  lidar looking at this wall would report approximately this". It was being fed to
  `lidar_disagreement` as a background, which compares against `base_link` ground
  ranges. Corrected to `TILT_WALL_060_BASE_M = 0.678` — **and that number is an
  independent lidar spot probe of the same wall, not 0.578 + 0.10.** Two routes to the
  same figure is the strongest single confirmation that `mount_x_m` is 0.10 m.
* `TILT_BOX_050_OBJECT_M = 0.500` (a sensor reading) was the "the lidar sees the object
  itself" background. A background 0.10 m nearer than the ToF's own ground range can
  never be disagreed with, so that assertion held against any implementation at all —
  including one that ignored the lidar entirely.

### 9.6 THE FINDING THAT OUTLIVES THIS BATCH: design §9.4's 0.178 m was cross-frame

Design 9.4 justified rule B on the 5 cm object with "a disagreement of 0.178 m, nearly
twenty times the margin rule A had to spare". That figure is `0.678 − 0.500`: a lidar
range from `base_link` minus a **sensor reading**. In one frame the object's real
disagreement is **0.089 m**, and the shipped `disagreement_margin_m` of 0.10 m sits
ABOVE it — so at today's deployed margin **rule B does not fire on the object it was
argued from, 0 of 40 recorded frames.**

This does not make rule B wrong. It makes the *guess* wrong, in a direction nobody
could see while the frame was broken, and it sharpens bench item J rather than
threatening it. Measured bounds for the margin J must pin:

    bare wall (TILT_WALL_060, base_link background)   worst apparent  +0.009 m
    J probe wall columns, re-analysed                 +0.001 m at 0.83 m
                                                      +0.024 m at 1.80 m
    the 5 cm object at 0.50 m                          real            +0.089 m

**Predicted window: roughly [0.033, 0.089).** It is a prediction, not a value — the
margin is set from J's own per-column distribution or not at all. The constant is
deliberately left at 0.10 so J measures against an unmoved target.

### 9.7 The J re-analysis gate — PASSED

Per §3's sanity check, `J_probe2.csv` re-analysed through the fixed geometry
(`diagnostics/tof_lidar_frame_check.py`, added so the next capture need not re-derive
this by hand):

| | as recorded | re-analysed |
|---|---|---|
| overall median disagreement | **+0.1095 m** | **+0.0133 m** |
| near columns (~0.83 m) | +0.099 m | **+0.0008 m** |
| far columns (~1.80 m) | +0.126 m | +0.0241 m |

The constant offset is gone. What remains scales with range — near-zero at 0.83 m,
0.024 m at 1.80 m — which is the signature of each zone reporting the nearest part of
its cone (design 9.7), not of a translation. **A residual translation would have stayed
flat; this does not.**

One column does not collapse: column 4 holds +0.487 m after correction, at z = 0.119 m,
below the 0.1905 m lidar plane. That is a real sub-lidar return — rule B's true-positive
class — and not a frame error. It is also §9.8's per-column trap in the flesh: pooling
it with the wall columns produces a "margin" of 0.74 m against wall columns at 0.03 m.

`J_probe.csv` (the first probe) is **entirely floor returns** and cannot test the frame
at all: the ToF and the lidar were not looking at the same surface.
