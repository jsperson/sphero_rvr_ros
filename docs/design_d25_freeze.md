<!-- COMMITTED 2026-08-11, HISTORICAL. Written and approved 2026-08-10 evening;
     it lived only in a scratchpad until now, and that is exactly how two clauses of
     it drifted from the implementation and survived three reviews: an approved design
     that is not in the repo cannot be diffed against the code.

     IMPLEMENTATION STATUS as of 2026-08-11:
       2b geometry  -- DRIFTED, now corrected. The mark was a POINT at the robot
          CENTRE; the design specifies a 0.14 m disc at the frozen footprint's
          LEADING EDGE. Both axes fixed; see tests/test_freeze_mark_geometry.py.
       2c counters  -- implemented, but its freeze/abort correlation was by wall
          clock and broke when the stall ladder lengthened recovery. Now correlated
          by GOAL; freeze_correlation_window_s deleted.
       layer config -- implemented as specified (separate ObstacleLayer,
          clearing:false, publisher-owned TTL); verified live in the costmap on
          2026-08-11 at 100/99 with inflation.
     Kept AS WRITTEN below rather than edited to match the code: this is the record
     of what was approved, and the value of a design note is lost the moment it is
     quietly reconciled with whatever got built. -->

# Design note — D25 escape order + freeze-as-sensor (pre-implementation)

Scott's principle: *"Hitting an obstacle and stopping should be just another data
point... Just backup exactly like you came and plan an exit."* Everything below serves
that. **No code written yet.**

## 0. Ownership map — and a correction to the brief's premise

The brief asks whether the 2.5 s grinding pivot came from collision_stop's escape
grant or decisive_control's ProgressGuard back-off. **Neither, on its own. It is an
emergent interaction, and ProgressGuard is innocent — it never pivots.**

`ProgressGuard.step` (decisive_control.py:192) has exactly three outputs: `drive`,
`reverse`, `abort`. On no-progress it commands **straight reverse** at 0.10 m/s
(`back_off_speed_mps`), which is already reverse-first. Item 1's headline behaviour
partly exists.

What actually produced the grind, from the 14:29:01 CSV rows:
1. decisive_controller was commanding a legitimate forward **arc**: `cmd = (0.20, -0.80)`.
2. lidar front hit 0.29 m → collision_stop's front stop fired → linear correctly zeroed.
3. collision_stop's **turn escape** (`front_stop_turn_escape`, collision_stop.py:878)
   passes the *residual angular* through: output became `(0.00, -0.40)`.
4. Net effect: **an in-place pivot nobody requested**, ground against the chair for the
   ~2 s of `stall_time_s` before ProgressGuard's clock even expired.

The turn escape was built (D17–D21) for a *deliberate* pivot — "front-stopped, explorer
wants to rotate toward open floor". It cannot currently tell a deliberate pivot from the
leftover angular of an arc it just amputated. **That is the defect.**

## 1. Item 1 — escape order (D25)

**Change, in collision_stop (SAFETY PATH — Fable review pause before chassis):**
when the front stop zeroes linear on a command whose `linear_x > 0`, do **not** grant the
residual angular. Grant the turn escape only for a command that was *already* a pure
pivot (`linear_x == 0`). One condition, at the existing escape branch.
Rationale: a caller asking to arc forward has not asked to pivot in place; converting one
into the other is the stack inventing a motion. A caller that genuinely wants to rotate
out sends a pure pivot and still gets it — the D17–D21 gates and their probe are untouched.

**Change, in decisive_control (not safety path):** nothing to the reverse mechanism.
Consider `stall_time_s` 2.0 → ~1.0 for freeze-classified stalls only (see §2); 2 s of
pushing on an unseen obstacle is the wear Scott heard. Flagged as a tuning question,
not bundled silently.

**Trail-following reverse: YAGNI, and I recommend against v1.** Straight reverse is what
freed the rover twice in the field, immediately, with zero slip (0.19 m and 1.0 m). The
entry segment immediately behind the robot is straight in every observed case because
the robot was driving forward when it froze. Trail-following adds a path buffer, a replay
controller and a new failure mode (replaying a trail whose far end is now occupied) to
solve a problem not yet observed. Revisit if a freeze is ever preceded by a tight arc.

## 2. Item 2 — freeze-as-sensor

### 2a. Detection — who, and what exactly
**Owner: `decisive_controller` (decisive_control.py + its node), NOT a new node.**
It already owns the boxed-in verdict, the pose stream and the back-off state machine.

**One addition it needs: a subscription to `/cmd_vel_motor`.** Read-only. This is the
crux — today ProgressGuard cannot distinguish:
- *supervisor braked me* (output linear **== 0**) — normal, expected, not news; from
- *supervisor let me drive and I did not move* (output linear **!= 0**, motion ≈ 0) —
  **the freeze**, i.e. an obstacle no sensor can see.

Without the output topic those are identical, which is why every stall today looks the
same. Classification rule:
`FREEZE := output_linear_x > 0 sustained ≥ stall_time_s AND |Δpose| < progress_epsilon_m`.
(The "all sectors clear" clause in the brief is redundant given output ≠ 0 — if a sector
were blocking, the supervisor would have zeroed the output. Simpler and strictly safer.)

### 2b. The mark reaching the PLANNER — the hard part, with a trap
The global costmap (`lean_nav2.yaml:159`) is `static_layer + obstacle_layer +
inflation_layer`, and `obstacle_layer.observation_sources: scan` with `clearing: true`.

**TRAP: adding freeze marks as another source of `obstacle_layer` cannot work.** That
layer raytrace-clears from `scan`, and the whole premise is that the lidar sees straight
through this obstacle — so the marks would be erased by the one sensor that is blind to
them, within a scan or two. This must be stated because it is the obvious implementation
and it silently does nothing.

**Design: a second, separate `ObstacleLayer` instance on the global costmap**
(`plugins: [static_layer, obstacle_layer, freeze_layer, inflation_layer]`) with
`observation_sources: freeze`, `marking: true`, **`clearing: false`**, and
`observation_persistence: 0.0`. A separate layer is not touched by scan raytracing.
With persistence 0 only the newest message counts, so **the publisher owns TTL exactly**:
it republishes the full live mark set at ~2 Hz and simply stops including expired ones.
No layer-side state to reason about, and "which marks exist" has one owner.

- **Geometry:** one mark = a small disc at the frozen footprint's leading edge, radius
  ≈ `robot_radius` (0.14 m), placed at the pose where motion stopped. Rationale: we know
  *the robot could not pass here*; we do not know the obstacle's true extent, so mark
  the footprint we proved is blocked, not a guess about the object.
- **TTL:** default 300 s, parameterised. Long enough to shape a mission, short enough
  that a moved chair does not haunt the map forever. Freezes are rare, so a generous TTL
  is cheap.
- **Serialising with the mission map:** the marks are *robot-derived belief*, not
  geometry SLAM measured, so they must NOT be written into the saved PGM (that map is
  the room). They go in the mission report as a list of poses + timestamps —
  diagnosable, and Scott can see where it got stuck. **Field name superseded by D35:**
  this shipped as `freeze_marks`, one entry per EVENT under a name that reads as
  places, and run 112721 duly filed nine for six. It is now `freeze_events` +
  `freeze_positions` + `freeze_mark_counts{events, distinct_positions,
  merge_radius_m}`.

### 2c. Counter semantics — the redesign Scott asked for
`max_consecutive_failures` exists to catch *"the stack is broken, it would fail
anywhere"* (D4/D11). A freeze is the opposite: **positive evidence about the room.**

- **Freeze-classified failure → DISCOVERY.** Does not touch `_consecutive_failures`.
  Does still suppress the cell (a real obstacle is there) and does publish a mark.
- **Non-freeze failure → counts as today** (planner rejection, nav abort while moving
  freely, watchdog cancel with output == 0).
- **Guard against the obvious hole:** an unbounded discovery exemption means a rover
  wedged in a corner freezes forever and never gives up. So a **separate**
  `max_consecutive_freezes` (default 5) ends the mission with its own outcome,
  `INCOMPLETE_BLOCKED_BY_UNSEEN_OBSTACLES` — which is a *truthful and useful* ending,
  unlike today's "the stack is broken" for what is really a cluttered room.
- **Plumbing:** decisive_controller publishes freeze events on
  `/decisive_controller/freeze` (the same source that feeds the mark cloud); the
  explorer subscribes and classifies an abort as freeze-caused if a freeze event landed
  within ~3 s of it. Time-correlation, not a new action-result field — it needs no
  interface change.

## 3. Added / deleted
**Added:** `/cmd_vel_motor` subscription + freeze classifier in decisive_controller; a
freeze-mark PointCloud2 publisher + TTL set; `freeze_layer` in lean_nav2.yaml; explorer
freeze subscription + `max_consecutive_freezes` + new outcome constant; the freeze
marks in the mission report (shipped as `freeze_marks`, renamed by D35 — see above).
**Deleted:** the unconditional angular pass-through in the front-stop escape (replaced by
the pure-pivot-only condition).
**Unchanged:** every D17–D21 gate and `pivot_gate_probe`; the supervisor stays the sole
`/cmd_vel_motor` publisher; ProgressGuard's reverse mechanism.

## 4. Proving it — harness scenarios, with revert proof
In `test_coverage_explorer_mission.py` / a new `test_decisive_freeze.py` (pure core where
possible):
1. **`freeze_is_classified_not_counted`** — fake output nonzero + frozen pose → guard
   emits freeze; `_consecutive_failures` unchanged; cell suppressed. **MUST FAIL against
   current code** (today it increments). This is the revert proof.
2. **`brake_stop_is_not_a_freeze`** — output == 0 + frozen pose → NOT freeze; counter
   increments as before. Guards against the exemption swallowing real failures.
3. **`repeated_freezes_end_the_mission`** — 5 freezes → `INCOMPLETE_BLOCKED_BY_UNSEEN_
   OBSTACLES`, not `ABORTED_GOALS_KEEP_FAILING`. Fails against current code.
4. **`freeze_publishes_a_mark_and_it_expires`** — pure TTL test on the mark set.
5. **`arc_amputated_by_front_stop_does_not_pivot`** — collision_stop unit test: command
   (0.20, −0.80) with front inside the stop → output angular **0.0**; and pure pivot
   (0.0, −0.40) with the same scan → still granted. Fails against current code.
6. Re-run `pivot_gate_probe.py` unchanged on the Pi — D17–D21 must still ALL PASS.

## 5. What I am explicitly NOT building
Trail-following reverse (§1). A new node of any kind. Any change to the camera pipeline
or to the D17–D21 gate geometry. Contact detection from the yaw-garbage signature
(real, but a separate finding and a separate defect). Retuning `stall_time_s` silently.
Anything that writes freeze marks into the saved map.

## 6. Sequencing request
Item 1's collision_stop change is safety-path: I will implement it in its own commit,
flagged for the Fable pause, and will not expose it to the chassis until that clears.
Items 2 and 3 (D22) are not safety-path and can land alongside. The two approved
protocol one-liners ride in the docs commit.
