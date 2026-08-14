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
