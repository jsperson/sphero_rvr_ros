# Position paper: the planner cannot see what the ToF sees — options, costs, receipts

**Status: POSITIONS ONLY. No decision is taken in this document.** Worker's
recommendation at the end, PM argues on review, Scott decides. Written 2026-08-18
night shift, from the day's field receipts.

## 1. The problem, one paragraph

`tof_layer` exists only in the LOCAL costmap; the planner plans on the GLOBAL one.
So for a sub-lidar obstacle the ToF sees steadily, the planner plans THROUGH it
while RPP refuses to drive the plan — forever. Field receipt, run 3c boot test
(goal 4, 13:56:01–20): a boot on its side at 0.65 m, painted lethal in the local
costmap, invisible in the global; **16 RPP "collision ahead" refusals, 10
recoveries, ~0.2 m of shuffling, 0 progress, honest abort in 19 s.** No contact, so
the touch chain — the one mechanism that CAN put a sub-lidar obstacle in front of
the planner — never fired: the ToF defended the obstacle from the very system that
could have learned it. The failure is honest, safe, and structural: a livelock
between two components reading different maps.

Sibling receipt, same day (run 3d, the horizontal table leg): the ToF held the leg
for 4 s of approach (0.57→0.27 m, ~40 in-envelope returns), lost it inside ~0.27 m
(obstacle_min_range 0.17 + cone geometry), and with `clearing: true` the stale
paint was plausibly raytrace-erased as the body closed the last 0.14 m — three
contacts followed. Whatever option is chosen must not make THAT class worse.

## 2. History and constraints, quoted where they bind

* **D27 is parked, not extinct.** The phantom class (dropped/shortened returns
  reading as near obstacles) was retired WITH THE OPTICAL DETECTOR, not disproven
  for ToF. Rule B holds brake authority pinned against an indoor wall under indoor
  light; **the sensor has never been measured in direct sun** and the protocol
  treats sun as an unstaged condition until Scott's sun capture exists.
* **The 08-08 precedent** kept belief sensors out of the global map for exactly the
  poisoning reason: a phantom in the LOCAL map costs a flinch that clears; a
  phantom in the GLOBAL map redirects every future plan.
* **The layer split of 2026-08-18** (tof_layer separated from touch_layer) was made
  so the ToF may erase its own returns while touch marks are erased by nothing.
  Any promotion design inherits that asymmetry deliberately.
* **nav2 mechanics that bound the design space:** an ObstacleLayer cell, once
  marked in a global costmap, persists until a clearing RAY crosses it — and
  clearing rays exist only within raytrace range (0.60 m) of wherever the robot
  currently is. There is no TTL. So "just add tof to global" makes every ToF mark
  quasi-permanent the moment the robot drives away from it.

## 3. The options, honestly costed

### A. Status quo: local-only, honest aborts accepted
The planner stays blind; goals into ToF-defended space end as goal 4 did. **Failure
mode: stuck-honest-aborts** — for a mission (not a directed goal) this is the
INCOMPLETE_NO_PLANNABLE_TARGETS shape wearing a new coat. Zero phantom risk, zero
new mechanism, and the operator (or the future NL layer) owns rerouting. Honest
and cheap; concedes the livelock as a permanent property.

### B. Promote tof_layer to the global costmap as-is (marking + clearing mirror)
Planner and controller finally read the same lethality. **Failure modes, two:**
(1) *phantom-in-the-planner's-map* — unmeasured in sun (D27's ghost, one sensor
over); (2) the bigger one on reflection: **quasi-permanence** — clearing only
happens near the robot, so any transient true obstacle (a foot, a moved chair, a
one-frame phantom) marked once becomes a global no-go region until the robot
happens to pass within 0.6 m again. That is contact-marks-local-only's disease,
mirrored: protection without honest expiry. Also plan-flapping risk while
paint/clear cycles at range boundaries.

### C. Dwell-gated promotion (N consecutive confirmations before a cell goes global)
Filters one-frame phantoms by construction. **Failure modes:** complexity plus a
latency that lands EXACTLY where the information is needed — the boot was painted
only UNDER APPROACH, so dwell accrues during the last seconds before RPP's refusal;
a dwell long enough to filter phantoms (seconds at 6.6 Hz) may not complete before
the refusal loop starts, leaving the planner blind at precisely the moment that
matters. Inherits B's quasi-permanence for whatever it does promote, unless it also
builds expiry — at which point it is a new belief layer, not a config.

### D. Refusal-marks: promote through the TOUCH pipeline, on demonstrated refusal
A small consumer watches for the livelock signature itself (K consecutive RPP
"collision ahead" refusals against a valid global plan — the banked
`IsBlindContact`/blind_contact machinery is adjacent to this shape) and, when it
fires, plants the offending local-costmap cells as marks through the EXISTING
/contact_marks pipeline — which already feeds BOTH costmaps, already has placement
provenance, and is already first in line for the revocation/try-harder work.
**Costs:** marks from a controller heuristic rather than a physical fact (the
invented-obstacle family — mitigated by requiring the local costmap to actually
hold lethal cells on the refused path, so it promotes an OBSERVATION, not a guess);
mission-permanence until revocation lands (shared with every mark today); one new
node. **What it buys:** zero phantom exposure during ordinary driving (it fires
only when the livelock is already happening), reuse of proven plumbing, and the
conservative failure direction — over-marking a place the controller demonstrably
refuses to drive anyway.

## 4. What would actually discriminate (cheap, mostly already possible)

* **Phantom rate and persistence cost for B/C** are measurable from existing bags
  (ToF return stability over the 3c/3d flights) plus the outstanding SUN CAPTURE —
  which is a precondition for ANY promotion option per the protocol's own rule.
* **The rig can now A/B this.** As of b215cb2 the stock bag records both costmaps'
  raw+updates streams, and the closed-loop rig drives the real middle: stage a
  sub-lidar-only obstacle in sim (tof source), run B and D against the same goal,
  count refusals/detours/false no-go area left behind. The falsifier standard
  applies: reproduce goal 4's 16-refusal livelock in the rig first.

## 5. Worker's recommendation (to be argued)

**Run the rig A/B before committing anything to a flight.** If forced to rank
today: **D first, B-with-eyes-open second, A as the honest floor, C last.**
Reasoning: D confines new risk to the exact moment the current architecture is
already failing (the livelock), reuses the one pipeline that provably moves
information from the ToF's world into the planner's (boot test: marks are the only
bridge), fails in the conservative direction, and converges with work already
committed (revocation/try-harder pulls forward for marks generally). B is simpler
and may win the A/B, but its quasi-permanence is a mission-killer shape we have
paid for twice under other names, and it buys phantom exposure during ALL driving
for a benefit needed only in the livelock. C's latency lands at the worst possible
moment and its filter defeats the boot's paint-under-approach signature. A remains
acceptable for directed-goal campaigns where an operator reroutes — which is
exactly this week — so nothing here is urgent enough to skip the rig.

## 5b. PM's side (review round, same night) — both positions travel to Scott

**D gains a third named cost: TRANSIENT-TRUE-OBSTACLE PROMOTION.** In a lived-in
home, K consecutive refusals accumulate exactly when a PERSON or a just-moved chair
blocks the path for a few seconds — and D would promote that moment into a
mission-permanent mark. That is this paper's own quasi-permanence critique of B,
gated narrower but not escaped: mission-permanent-until-revocation is
expiry-less-ness all the same. Mitigation to carry into any D design: the livelock
signature must require refusals SUSTAINED OVER TIME (≥T seconds, not count alone)
so ambulatory obstacles pass — and the rig A/B adds a TRANSIENT-OBSTACLE scenario
(stage, block, unblock: D must NOT promote it; measure K/T sensitivity). This does
not demote D; it prices it honestly.

**PM's ranking: D > A > B > C** (worker's, above: D > B > A > C — agreement on D
first and C last; the argument is A-vs-B). PM's case for A over B: without expiry,
B's quasi-permanence in a home where chairs move DAILY means the global map rots
within a single mission — while A's honest aborts are exactly the failure the
PRODUCT layer is designed to absorb: the NL/semantic top layer says "I can't reach
that, it's blocked," and asks or reroutes, which is the architecture's whole
thesis. **A is not just the floor; it is the option that degrades into a
conversation instead of a wall.** B earns a slot only after the sun capture AND an
expiry story exist — at which point it is C wearing fewer clothes.

Both rankings stand as written. Scott decides, with the rig A/B's numbers when it
runs.
