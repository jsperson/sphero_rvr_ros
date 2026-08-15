# FIX WAVE — design brief, 2026-08-14

**Prose only. No code, no constants.** Written the night the gauntlet paused, while the
evidence framing is fresh; the derivations happen tomorrow against the recordings, not
tonight from memory. Anything here that reads like a number is quoted evidence, not a
decision.

**What triggered it.** Gauntlet mission 2 was ruled not-a-legitimate-stop by Scott:
*"The rover drove in there and there is at least 180 degrees from about 1 o'clock to
seven o'clock to move into."* The measurement agrees with him — eight of twelve clock
positions open, up to 2.028 m — and the stack declared GENUINELY WEDGED anyway. That is
the robot-cannot-move class, which under the standing rules outranks everything.

**The evidence base** is `03_validation/gauntlet_2026-08-14/`: bag, recorder CSV, both
mission reports and maps, hash-verified on both machines. Every claim below is
reproducible from it. **Re-derive from the recordings tomorrow rather than trusting the
numbers quoted here** — that rule has caught three of my own errors today alone.

---

## 0. What the day actually proved, stated once

Two things flew for the first time and **both worked**:

* **Rule B held brake authority for two missions.** It produced real detections, drove
  86 brake engagements, and caused **zero** escape refusals — verified by
  `pivot_veto` false in all 571 refusals and the low-obstacle scale never below 1.0 in
  any of them. It was present without being in the way.
* **The give-up escape fired in the field for the first time** — four attempts. It had
  been unobservable since it was built.

And the thing that failed is not either of those. It is that **all four give-up attempts
were refused, and the ladder before them was refused, by gates that cannot use what the
rover already knows.** D36's wiring is sound; the escape stack now funnels into gates
that strangle it. That is one defect with three faces, and Batch A is those three faces.

---

## 1. BATCH A — D40: give the escape stack capability its gates can act on

### A1. Entry-trail retrace, as a first-class escape primitive

**The argument in one line: the rover drove in, so a clear corridor exists by
construction, and nothing in the stack exploits it.**

At the wedge, 1.719 m of the rover's own trail sat behind it, sampled at 10 Hz, dense
and recent. Meanwhile the ladder's reverse rungs were refused 350 times by trajectory
projection while the rear sector read clear by three millimetres. **The retrace makes
that three-millimetre argument moot** — it does not need the projection to adjudicate a
marginal sector, because it is following ground the robot's own footprint occupied
minutes ago.

Scott named this a long time ago as *"back up exactly like you came"*, and it was never
implemented. The ladder reverses blind.

Design shape, and the standing rule decides it: **the owner publishes the fact.** The
controller knows its own pose history; it records its own trail and retraces its own
trail. Nothing infers a corridor from a costmap or a map-frame guess.

Open questions for tomorrow, each answerable from the recordings:
* How far back is worth keeping, and in what units — time, distance, or pose count? The
  answer should come from how far the trail actually needs to run to clear a real wedge,
  measured against today's two.
* What invalidates a trail? A freeze mark planted since; a long stationary gap; the
  world having moved. A retrace into a chair that arrived after the rover passed is the
  obvious failure and must be answered rather than hoped away — probably by the retrace
  still going through the supervisor, so it is *proposed* by the trail and *permitted*
  by live sensing rather than either alone.
* Does the trail belong to the ladder, the give-up escape, or both? The two are already
  distinct mechanisms with distinct triggers, and this is a capability both want.

### A2. Straight forward, as an explicit rung candidate

**The untried move.** At the wedge, the front sector read 0.387 m clear, and across 683
commanded rows at that pose **not one straight-forward command appears**. The rung set
spent itself on reverse and rotation while the one direction that was open went unasked.

This is the cheapest item in the wave and possibly the highest value: a rung that tries
forward when the front sector is open, ordered so it is reached rather than starved by
the rungs ahead of it. The ordering question is real — reverse-first exists because
reversing out of a nose-in wedge is usually right — so the honest form is a rung
*candidate* selected on sector evidence, not a fixed slot appended to the ladder.

### A3. Arc-sweep gating derived per command

**This is the one I refused to assert and it stays refused until geometry answers it.**

At the wedge the pure-pivot refusal was **correct**: the footprint's rotation sweep
radius is 0.189 m and the left object sat at 0.165 m, inside it. Direction would not
have saved a pivot, and any fix premised on "the veto ignores rotation direction" would
have been built on a false reading of this pose.

What is genuinely open: the forward-right arc was refused 92 times with 1.253 m open to
the right. Turning right while moving forward puts the **left flank on the outside of
the turn**, sweeping a larger radius — so the refusal may be correct too. The fix must
answer this with the actual swept region for the actual commanded (v, ω), not by
asserting either way.

**The D17 fail-closed lesson is respected by DERIVING THE SWEPT ARC, not by vetoing
everything.** A gate that refuses every rotation because one bearing is close is
fail-closed in the same sense that a brick is a safe car. The correct conservatism is to
compute what the command actually sweeps and refuse on that.

### A4. D39 — the stop distance that sits in the blind zone

Rides this batch because it is the same class: a constant derived against one
consideration while another one governs.

Measured today: a 5 cm object is visible to the ToF from about 0.25 m to about 0.60 m
and **blind at 0.20 m**, where every row's ray passes above it. The deployed low-obstacle
stop distance is 0.20 m. **The stop threshold sits inside the blind zone**, so the stop
can never fire for an object that height — and worse, the brake *releases* when the
object vanishes, so the rover accelerates into it. That is the mechanism behind Scott's
*"it's running into low objects"*, and it is arithmetic rather than circumstance.

Two things change:
* the constant moves inside the visible band;
* **the rule that prevents it recurring**: a stop distance must be at least the near edge
  of the visibility band for the shortest object we claim to stop for, and since that
  edge moves with object height, the claimed height is stated beside the constant. Same
  discipline design §12.3 already imposes on quoting rule B's reach.

I derived that band this morning and did not connect it to the stop constant. Worth
recording as the lesson: **a new envelope model obliges a re-check of every constant
that assumed the old one**, and nothing prompted me to do it.

---

## 2. BATCH B — the ToF becomes something the planner can use

Today the ToF could only brake. It saw low objects, slowed for them, and then the
planner — which knows nothing of them — routed straight back into the same places. The
sensor's knowledge dies at the end of each frame.

**Shape: `/tof/obstacles` becomes an observation source for the global costmap**, so what
the ToF sees becomes something routes are planned *around* rather than braked *at*.

**The tiering was settled by Scott tonight and is the load-bearing part:**

| what | persistence |
|---|---|
| **seen** (ToF observed it) | decays in tens of seconds unless re-confirmed |
| **touched** (freeze mark, the robot proved it) | mission lifetime |
| **walls** (lidar structure) | mission and beyond |

Decay is the whole design. A low obstacle observed once is a *belief*, and the sensor has
a narrow visibility band — an object drops out of view as the rover closes on it, which
is exactly the D39 mechanism. Marks that never decay would fill the costmap with
ghosts of things the rover can no longer confirm, and the 2026-08-11 blacklist
experience is the precedent: a permanent-marking scheme took 73% of free space.

**Explicitly out of scope**: between-mission persistence. That waits for saved-map
workflows, and even then **low marks never persist across sessions** — the room changes,
and a shoe is not a wall.

The interesting question tomorrow is what the decay clock should be anchored to. Elapsed
time is the obvious answer and probably wrong: an obstacle the rover has driven away from
and cannot re-observe is not evidence-stale in the same way as one it is looking at and
no longer seeing. Confirmation opportunity, not wall-clock, may be the right unit.

---

## 3. BATCH C — staged-to-wheels in under two minutes

Not a fix, a tax. Today's session spent a large fraction of Scott's attention on bringup,
and every minute of it was avoidable:

* the chassis was off and nothing said so until I read serial timeouts by hand — **a
  chassis-alive check belongs on the first line of preflight**, because everything
  downstream (odom, SLAM anchor, map, goals) is a cascade from it and each layer failed
  in a way that looked like its own problem;
* the installed tree was stale and the sha256 loop caught it — **that check earned its
  place twice today** and should be a script, not a paragraph someone retypes;
* `tof_node` is not in the launch file, so the ToF brake is one forgotten command away
  from being absent on a run that believes it has it;
* the supervisor latch needs `clear_estop` *before* `reset`, an ordering written down
  nowhere, discovered live.

One preflight script on the Pi that runs the whole gate list and prints pass/fail per
line. `tof_node` into the launch. The reset ordering documented in the run card. This can
land any time and does not gate the gauntlet.

---

## 4. Sequencing, and what restarts the count

**A and B before the gauntlet restarts at 0.** A alone would let the rover escape wedges
it currently dies in; B alone would let it avoid entering them. Neither is sufficient:
without A it still strands itself somewhere eventually, and without B it re-enters the
same trap on the next goal because nothing remembers. C lands whenever.

**Safety-path changes get the full apparatus** — revert-proofs that fail against the code
they indict, mutations run and recorded, the deployed-config probe rather than dataclass
defaults. That is not ceremony here. It is the layer that stops the robot hitting things,
and today proved the apparatus catches real things: the stale install and the unpinned
margin were both caught by gates, before wheels turned.

**But the apparatus serves making the robot move this time**, and that changes what
"done" means for this wave. The acceptance is not a green suite. It is a rover that gets
out of a corner it drove into, and the proof is a field recording of it doing so — not a
test asserting that it would.
