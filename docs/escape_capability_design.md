# BATCH A — escape capability (D40 + D39), design note

**Prose only. No code.** Held for review before anything is implemented.

Evidence base: `03_validation/gauntlet_2026-08-14b_datarun/` (tonight, hash-verified) and
`03_validation/gauntlet_2026-08-14/` (mission 2). Every number below is re-derived from those
recordings, not quoted from the fix-wave brief — that rule caught the brief's own D39 band-edge
ambiguity (0.20 vs 0.22 vs 0.25) and one over-claim of mine tonight.

---

## 0. The headline: the biggest piece of D40 is not a geometry problem

The fix-wave brief framed A3 (arc-sweep gating) as the hard, load-bearing item and A1/A2 as
capability additions. **Tonight's recording says the single highest-value fix is none of those.
It is a command-shape mismatch that makes the give-up escape un-grantable by construction.**

The chain, all of it arithmetic:

1. The give-up escape issues **one straight reverse** — `src/sphero_rvr_driver/decisive_controller_node.py:702`, and
   the docstring is explicit that this is deliberate: *"Deliberately NOT an escalating ladder."*
   The command is `(-reverse_speed, 0.0)`. Angular is exactly zero.
2. The supervisor's `rear_hold` gate (`src/sphero_rvr_driver/collision_stop.py:950-952` — the
   **driver** tree, not `sphero_rvr_core/`) fires when a reverse is
   commanded and the **rear sector** minimum is within `reverse_stop_distance_m` (deployed:
   **0.25 m**). It sets `linear_x = 0.0` and **passes `angular_z` through untouched**.
3. At tonight's wedge the rear bearings are 6/7/8 o'clock = 0.290 / **0.150** / 0.176 m. The
   sector minimum is at most 0.176, comfortably inside 0.25. **`rear_hold` fires with
   certainty.**
4. `rear_hold` zeroes the linear term; the escape supplied zero angular; the output is
   `(0.0, 0.0)`. The explorer measured exactly that: *"refused: 0.000 m in 6.0 s; supervisor:
   rear_hold"*, **four times, at one pose, over 21 s.**

**And the stall ladder already knows this.** `src/sphero_rvr_core/stall_ladder.py:506-512`, verbatim:

> Reverse arc rather than straight reverse, because `rear_hold` refuses a straight reverse
> outright but passes ANGULAR through untouched — so this rung is granted at exactly the poses
> where rung 1 is refused. Measured against the supervisor core at run 190528's abort geometry.

The ladder learned this in August and encoded it as rung 2 (`REVERSE_ARC`). The give-up escape,
built later for D36, chose a single straight reverse for good reasons — one author per motion,
no second escalating ladder — and in doing so **re-adopted precisely the command shape that
`rear_hold` is guaranteed to zero.**

This is the project's recurring **unreachable-recovery class**, in a new form. The pivot
controller was unreachable because the code path never ran. The explorer unstick was unreachable
because its trigger never fired. This one runs, is reached, and is *un-grantable*: its command
shape guarantees refusal at exactly the poses it exists for. **A recovery that can only be
requested in a form the supervisor must refuse is not a recovery.**

**The class now has three forms, and each needs its own check** — this is the standards-doc
addition:

| form | failure | the check |
|---|---|---|
| **unreachable** | the code path never runs | does this path execute? |
| **never-triggered** | it runs, its condition never occurs | does the trigger fire in real run logs? |
| **un-grantable by construction** | it runs, is reached, and is refused every time because of its **command shape** | **can this command shape ever be granted at the poses where it is meant to fire?** |

Form 3 evades both existing checks: the code runs fine, the trigger fires correctly, and the
refusal happens in a *different module*. Only the interaction between the command's shape and the
arbiter's gates reveals it. The check must be run against the arbiter, not the caller.

**Consequence for sequencing:** this is a small, well-understood change with an existing proven
primitive to copy (`REVERSE_ARC`), and it does not require A3's geometry work. It should land
first and be provable on its own.

---

## 1. The two specimens, and why we needed both

Two wedge poses, both with the escape stack refused, and they indict *different* things. Neither
alone would have produced the right fix.

| | mission 2 (08-14) | tonight (08-14b) |
|---|---|---|
| forward (12 o'clock) | **0.387 m OPEN**, never commanded in 683 rows | **0.266 m blocked** |
| rear | clear by **3 mm** | **0.150 m — genuinely blocked** |
| what was open | 12→6 o'clock, to 2.028 m | 1→5 and 9→10 o'clock, to **2.180 m** |
| refusing gate | reverse trajectory projection ×350, right arc ×92 | **`rear_hold` ×4** |
| the honest indictment | the rung SET — an available move was never tried | the escape's **directional vocabulary** — the only move it tries is the one direction physically blocked |

Tonight's table is authoritative and unusually clean: re-derived from the bag at all four refusal
stamps, `base_link<-laser` read from the bag's own `/tf_static`, and **`odom_yaw = −146.1°` and
pose `x=−1.664 y=+0.493` at every one of the four** — the rover held station in position *and*
heading for the whole episode. No bearing flips verdict across the four samples and the spread
never exceeds 7 mm. **7 of 12 bearings open, 2.18 m of floor at 4 o'clock, and the robot declared
itself out of options.**

Two consequences that the mission-2 data alone could not have given us:

**(a) A2 is necessary but proven insufficient.** A straight-forward rung would have saved mission
2. Tonight forward reads 0.266 m and a forward rung would have been refused, correctly. So the
rung candidate cannot be "forward"; it must be **selected on sector evidence across the full
360°**, which is the shape already pre-approved. Tonight the evidence points at 4 o'clock.

**(b) A1 gets promoted, and its naive form gets falsified.** The argument for entry-trail retrace
has been *"the rover drove in, so a clear corridor exists by construction."* Tonight is a
counterexample to the naive reading: the rover drove in, and the rear is blocked at 0.150 m.
**Straight back is not the way it came.** Whatever path it entered on, it curved. So:

> **The trail is not a heading.**

Blind reverse hits the 7 o'clock object. The recorded trail does not. On mission 2 the rear sector
was clear by 3 mm and blind reverse would have worked, so that data could never have made this
argument. Tonight blind reverse is refused *correctly* and the trail is still available. That
moves trail-retrace from "cheaper than adjudicating a marginal sector" to **"the only primitive
that works at this pose."**

---

## 2. A1 — entry-trail retrace as a first-class primitive

**Ownership, per assert-don't-infer.** The controller records its own pose trail and retraces its
own trail. It owns the fact, so it publishes the fact; nothing infers a corridor from a costmap,
a map-frame guess, or another node's state. This is the rule that three defects in two days were
bought with.

**The split that makes it safe** — and this is the load-bearing sentence:

> A retrace is **proposed by the trail** and **permitted by live sensing**.

The trail says where the robot has physically been; the supervisor says whether that ground is
still clear *now*. Neither is sufficient alone. A chair that arrived after the rover passed must
be answered, not hoped away, and the answer is that the retrace goes through the supervisor like
every other motion — it gets no special authority. The trail's job is to propose a *better
sequence of commands* than a blind heading, not to bypass the arbiter.

**What the trail must record.** Pose samples with timestamps, at the controller's own rate. The
retrace consumes them newest-first, converting successive poses into commanded (v, ω) segments —
which is exactly why "the trail is not a heading": the output is a *sequence* of arcs, and at
tonight's pose the first segment curves away from the 7 o'clock object rather than into it.

### 2.1 Which way does the arc turn? RULED — by the sensor-trust hierarchy

When a reverse must become a reverse *arc*, the turn direction comes from one of two sources: the
ladder's `open_bearing`, or the trail's own first segment. Tonight both point the same way (4
o'clock), so tonight's recording cannot discriminate between them. That is not a licence to choose
by taste — **the established sensor-trust hierarchy decides it**, and it decides for the trail:

* **The trail is an all-heights eyewitness.** The rover's own body physically swept that corridor
  minutes ago. That is positive evidence of clearance **at every height up to the robot's body**,
  including the sub-lidar band.
* **`open_bearing` is a lidar-height measurement**, taken at 0.19 m. It is structurally blind to
  exactly the object class that causes freezes — the sub-lidar obstacles that tonight's two
  freezes were, verbatim *"an obstacle no sensor on this robot can see."*

So an `open_bearing` that reads 2.18 m of clear floor is a claim about one horizontal plane, while
a trail segment is a claim about a volume the robot has already occupied. The trail strictly
dominates. **Direction comes from the trail's first segment wherever a valid trail exists;
`open_bearing` is the fallback** when there is no valid trail — invalidated by a since-planted
freeze mark or a long stationary gap, per the invalidation rules above.

This also explains an asymmetry worth stating: the fallback is not merely "less good", it is
*blind in the dimension that matters most for this failure mode*. A give-up escape steered by
`open_bearing` alone can drive confidently into a shoe. One steered by the trail cannot, because
the robot has already been there.

**How far back to keep** — answerable from the recordings, and to be answered before code. The
honest unit is **distance along the trail**, not elapsed time or pose count: what matters is how
far the robot must travel to be out of the trap, and tonight's trail ran 1.719 m at the
comparable mission-2 pose. Time is the wrong unit because a robot that sat still for 60 s has
accumulated no useful trail; pose count is the wrong unit because it varies with speed.

**What invalidates a trail**, each requiring an explicit answer rather than a hope:
* **A freeze mark planted since.** The robot has proven an obstacle exists on that ground. A
  trail segment through a freeze mark is dead and the retrace stops there.
* **A long stationary gap.** Samples either side of a long pause do not describe a driven path.
* **Live refusal.** The supervisor refuses a segment; that is the world having changed, and it is
  handled by the composition rules below rather than by the trail.

**Composition with D34 — non-negotiable.** No goal starts mid-escape. D34 cost 26 s of pushing a
weight bench because a lifecycle trigger fired inside a live ladder, and the class lesson is that
**no lifecycle trigger — cancel, preempt, or abort — is innocent of an active escape.** A retrace
is an escape. It must prove its active-escape interaction in the harness before it ships, and the
existing `ladder_active` liveness signal is the mechanism to reuse, not to reinvent.

**Composition with the freeze machinery.** A retrace that freezes marks and stops, exactly like
the give-up escape does. It does not push. Tonight's two freezes show the machinery working
correctly — contact became data, the rover marked and backed out, and the mission continued — and
a retrace must inherit that behaviour rather than acquire its own.

**Who owns it.** Both the ladder and the give-up escape want this capability. It should be one
primitive on the controller, offered to both, rather than two implementations — the "second author
on the same motion" failure the ladder exists to have ended.

---

## 3. A2 — a rung candidate selected on sector evidence

Not a ladder reorder, and not a fixed forward slot. A **candidate** chosen from the sector
evidence at the stalled pose, because the two specimens disagree about which direction is right
and any fixed ordering is wrong at one of them.

The machinery to reuse already exists: `RUNG_ORDER` and `VISIBLE_RUNG_ORDER` (`src/sphero_rvr_core/stall_ladder.py:55-56`)
already show the ladder selecting an order from evidence, and `open_bearing` is already threaded
through `_rung_command` to aim the arc and drive rungs. What is missing is a rung whose *direction*
is the open bearing rather than a fixed forward/reverse, and a selection rule that can reach it.

Reverse-first exists for a good reason — reversing out of a nose-in wedge is usually right — so
the candidate is added to the selection, not stapled to the front. At tonight's pose the sector
evidence nominates 4 o'clock (2.180 m); at mission 2's it nominates 12 o'clock (0.387 m). One rule,
two correct answers, which is the test of whether the rule is real.

---

## 4. A3 — arc-sweep gating derived per command

**The geometry already exists in this codebase and is already in service.** `swept_path_obstacle`
(`src/sphero_rvr_core/low_obstacle_brake.py:37`) computes the swept region for a commanded (v, ω), and the supervisor
already uses it for the low-obstacle brake, with a docstring that states the exact principle A3
needs: *"turning, a differential drive pivots about a centre off to one side, so the flank leads
and sweeps ground the nose never covers. A straight-ahead cone reports CLEAR while the flank hits
a chair leg."* `evaluate_projected_trajectory` likewise already projects the commanded trajectory
against the scan.

So A3 is **not** "invent swept-arc geometry." It is **"apply the primitive the codebase already
trusts, at the gates that still use sector minima."** That is a far smaller and safer change than
the brief anticipated, and it reuses tested code rather than adding a second author of swept-region
maths.

**The structural finding that makes this concrete:** `rear_hold` is evaluated at
`src/sphero_rvr_driver/collision_stop.py:950`, and the trajectory projection at `:953` — **the coarse sector gate is
checked first and returns, so a reverse command blocked by the rear sector never reaches the
precise swept-arc gate at all.** The more accurate mechanism is unreachable for exactly the case
where it would help. Whether the ordering should change, or `rear_hold` should itself consult the
swept region, is the design question to settle at review; both preserve fail-closed, because both
refuse on a derived swept region rather than on a bearing minimum.

**Fail-closed is respected by deriving, not by vetoing.** The D17 lesson stands and the correct
conservatism is to compute what the command actually sweeps and refuse on *that*. A gate that
refuses every rotation because one bearing is close is fail-closed the way a brick is a safe car.

**What must NOT be claimed.** At mission 2's pose the pure-pivot refusal was **correct** — sweep
radius 0.189 m against an object at 0.165 m — and direction-awareness would not have saved it. Any
fix premised on "the veto ignores rotation direction" would be built on a false reading of that
pose. Tonight's `rear_hold` refusal was likewise *correct on its own terms*: the rear really is
blocked. **Both gates were right and the robot was still trapped.** That is the honest shape of
D40 — a capability defect, not a gate that lies — and the fix must leave both refusals intact
while finding the moves physics allowed.

---

## 5. D39 — the stop distance that sits in the blind zone

**The constant moves inside the visibility band of the protected height.** The deployed
`low_obstacle_stop_distance_m` is **0.20 m** (`config/collision_stop.yaml:165`), and a 5 cm object
is blind below roughly the band's near edge, so the stop can never fire for that height — and
worse, the brake *releases* as the object vanishes, so the rover accelerates into it.

**The band edge must be re-derived before the constant is set.** The brief says the band starts
~0.25 m; Current Status says the close blind zone is ~0.22 m. Those are two values for the same
edge, and D39's entire fix is "put the constant inside the band," so the edge is the thing the
constant is derived against. Neither number gets adopted; it gets re-derived from the ToF
characterisation data, and that derivation is part of this batch.

**The rule that prevents recurrence:** a stop distance must be at least the near edge of the
visibility band for the shortest object we claim to stop for, and since that edge moves with
object height, **the claimed height is stated beside the constant** — the same discipline design
§12.3 already imposes on quoting rule B's reach. A derived-constants CI test in the established
style pins it.

**Tonight's two freezes are D39 specimens**, both verbatim *"an obstacle no sensor on this robot
can see"*, at (−0.63, −0.52) and (−1.03, −0.38), with approach context in the bag. They need
classifying into the three bins (in-envelope miss / out-of-envelope / by-spec) before the
constant is called done.

**The process rule, for the standards doc — and it has already fired twice.** *A new envelope
model obliges a re-check of every constant derived under the old one.* Tonight it fired in a
completely unrelated subsystem: D29 moved the arm event, and `duration_s` — anchored to node-ready
— was never re-anchored, over-reporting 2.94×. That argues the rule is not about ToF envelopes at
all but about **any re-anchoring change**, and it belongs in the standards doc in that general
form.

---

## 6. Proofs (to be built with the code, not after)

Per the chassis-run protocol: revert-proofs must **fail against the code they indict**, mutations
run and recorded, the deployed config probed rather than dataclass defaults (13 fields have
differed before, and a verdict flipped between them), every guard checked for both existence and
defeatability.

* **Proof 1a — mission 2's wedge pose replayed.** The fixed gates must find the move physics
  allowed (forward at 0.387 m, and/or the retrace) while still **refusing the pure pivot**
  (0.165 < 0.189). A fix that grants the pivot has failed, not passed.
* **Proof 1b — tonight's wedge pose replayed.** Four refusal frames, static geometry, ±7 mm. The
  fixed escape must obtain motion — via reverse arc, via a 4-o'clock rung candidate, or via
  retrace — while still **refusing straight reverse into the 0.150 m object at 7 o'clock**. This
  is the stronger of the two proofs because the world does not move across the whole episode.
* **Proof 1c — the un-grantable-command proof.** A test that fails against HEAD by asserting the
  give-up escape's commanded shape can be granted at a pose where `rear_hold` fires. This is the
  §0 defect's own revert-proof and it is independent of any geometry change.
* **Trail invalidation proof.** A retrace whose trail crosses a freeze mark planted since must
  stop at the mark, not drive through it.
* **D34 composition proof.** No goal starts mid-retrace; the lifecycle interaction is exercised in
  the harness, per the class lesson that no trigger is innocent of an active escape.
* **Defeat-check.** For each new guard, an adversary asks the three questions that verification
  artifacts fail on: is the mechanism absent, is the check defeatable, is the input fabricable.
  Mutation testing catches only the first two.

**Acceptance is not a green suite.** Per the brief, and it stands: the acceptance is a rover that
gets out of a corner it drove into, and the proof is a field recording of it doing so.

---

## 7. What this note does not settle

Flagged rather than assumed, for review:

* The exact trail-retention distance, pending the trail-length derivation from both recordings.
* Whether `rear_hold` consults the swept region itself, or the gate ordering changes so the
  trajectory projection is reached. Both preserve fail-closed; they differ in blast radius.
* The D39 band edge (0.22 vs 0.25), pending re-derivation.
The direction question that stood here — `open_bearing` vs the trail's first segment — is now
**ruled**, in §2.1 below. It was never a taste question; the discriminator was a principle this
project already holds, and I had failed to reach for it.
