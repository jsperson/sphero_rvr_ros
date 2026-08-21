# Design note: the supervisor-owned escape primitive (D52/D60) — DESIGN ONLY

Status: DRAFT for **Scott-daylight consensus**. Supervisor-owned motion
authority is not built overnight, ever (standing rule, reaffirmed in tonight's
mandate). Nothing in this note is scheduled; it exists so the daylight round
starts from the evidence instead of from memory.

## The two defects this serves, by their receipts

* **D52** (landmine row, filed BEFORE the explore-on-stock build): the stock
  `behavior_server` collision-checks Spin/BackUp against costmaps that carry
  `touch_layer`'s mission-permanent contact marks — a rover marked at close
  contact can have its BT recoveries refuse over LIDAR-CLEAR floor (D36's exact
  class, which is why the bespoke escape lives in the decisive controller, not
  nav2_behaviors). FIRST FIELD RECEIPT 2026-08-19: the explorer requested its
  give-up escape FOUR TIMES from one pose in the table-leg field, got "escape
  UNAVAILABLE: no /decisive_controller/escape_in_place server" each time, ended
  honestly. Same flight: a gateway turn timed out against a table leg — the
  recovery chain has no turn-around-and-leave concept.
* **D60** (rig-proven N=3, deterministic): goal selection's gates all check
  reachability TO a candidate; nothing checks plannability FROM it. A goal can
  be planner-approved into an enterable-but-unexitable inscribed pocket,
  succeed, and strand the rover — nothing plans out, unstick requests D52's
  absent escape, honest end with candidates still wanted. Tonight's belief-basin
  specimen (self-paint + 0.30 inflation) is the same ending signature filed
  under start-clearance/self-paint.

## The specimen's geometry, MEASURED (flight-4 bag, analyzed 2026-08-20 overnight)

`bag_20260820_161932` replayed on the Mac (mcap, production frame math; scripts
banked in `artifacts/belief_basin_geometry_2026-08-20/`). The basin window is
the transcript's four failed gotos, 16:44:27–16:49:21. Findings, in order of
weight:

1. **The corridor from the stuck pose (0.209, 0.121) back to the former
   standing pose (0.866, −0.009) was OPEN in belief for the window's first
   221 s and CLOSED for its final 71 s** (connectivity through cells <253 on
   every global costmap_raw grid; 0 of 48 pre-window grids blocked). The two
   ABORTED gotos fell in the open phase — the controller refusing a
   high-gradient corridor (both poses sat at cost 202–207, pure inflation) —
   and the "planner found no path" endings fell in the closed phase. One
   ending signature, two mechanisms.
2. **The door that closed it: 447 cells, 1.12 m², whose nearest cell is
   0.12 m from the stuck rover** — the paint is at the rover's own skirts —
   and 0.56 m from the standing pose it was trying to reach.
3. **Physical truth: 0 of the 447 door cells are supported by ANY of the
   23,027 live lidar returns in the closed period, even at ±1-cell
   tolerance.** The door is 100% belief, 0% physics. (The wider ring near the
   standing pose DOES hold real furniture — 24.5% of window returns land
   there; the room is cluttered, the DOOR is imaginary. Scott's eyewitness,
   quantified.)
4. **The painter, named: 23 new LETHAL(254) seed cells — 23/23 coincide with
   `/contact_marks` points, 0/23 with ToF obstacles — amplified ×19 by the
   0.30 m inflation radius into 424 INSCRIBED(253) cells.** The bag's third
   `/contact_marks/promote` event fires at t+222.2 s; the corridor closes at
   t+222.8 s. A refusal promotion painted the door shut, and by design no
   sensor clears a contact mark (mission-permanence is the feature).

What the numbers change about the design space: **direction A below cannot
touch this class** — the goal was exit-plannable when chosen; the wall was
painted AFTER arrival by the rover's own protection layer. A kills D60's
map-geometry pockets; self-paint basins need B (an authority that reads live
lidar, which in this specimen saw an open floor) or C (paint-time hygiene).

## Three directions, weighed (the register names two; the geometry adds a third)

**A. The exit-plannability clause** (goal selection, no motion authority): plan
FROM the candidate back toward covered/open space before accepting it — the
trinity gate's missing fourth clause. PREVENTS D60's pocket entrapment; costs
one extra `ComputePathToPose` per accepted goal (bounded, measurable); touches
only `coverage_exploration` selection logic; certifiable by the archived pocket
replay (selection must REFUSE the recorded pocket goal, still choose an
exit-plannable goal at equal distance). It does NOTHING for D52: a rover
already wedged, or marked-in by its own touch paint, is past selection's reach.

**B. The supervisor-owned escape primitive** (motion authority, daylight-gated):
RECOVERS what prevention misses — field wedges, mark-painted poses, the
turn-timed-out-against-a-leg family. D36-proof by construction: the supervisor
reads LIVE lidar, not belief paint, and it is already the sole
`/cmd_vel_motor` publisher and already hosts one admission-gated motion verb
(the precise-turn gateway — the pattern this would follow).

**C. Paint-time hygiene** (new, from the measured geometry): a promotion (or
mark) that would DISCONNECT the rover's own pose from open space is exactly the
self-trap the specimen shows — and it is CHECKABLE at paint time with the same
connectivity test the analysis ran (BFS on the would-be costmap, milliseconds
at this map size). Option D's promotion already carries self-quench and
freshness concepts; this would be a third clause in that family: *protection
must not imprison the protected*. Weighed honestly: it prevents only
self-made doors (real walls still close corridors), it must fail OPEN (a
hygiene bug must never suppress a real mark — the bad-mark-closes-a-doorway
risk cuts both ways), and its interaction with mark permanence needs its own
falsifier. Named for the daylight round, deliberately not designed here.

**Recommendation to the daylight round: A first, then B; C weighed within
Option D's own family.** A is small, cheap, authority-free, and kills the
deterministic rig-proven trap; B is the real recovery organ — the measured
specimen is its cleanest possible case (100% lidar-open floor behind a
belief-only door) — and deserves its own unhurried round. (A could even be
built under night rules; it is NOT built tonight because D60's design round
was explicitly deferred to daylight and the falsifier map replay should be
staged with Scott present.)

## B's shape, proposed for the daylight round

* **Interface, built-in-first:** an action server at
  `/collision_stop/escape_in_place` with the decisive controller's existing
  escape contract. The coverage explorer ALREADY requests exactly this and
  already handles absence honestly — four field receipts prove the consumer
  path. Zero explorer changes; the server appearing IS the feature.
* **Behaviour, bounded and blind-honest:** (1) survey the LIVE scan for the
  open sector (the pure `escape_survey` logic is reusable); (2) reverse along
  the entry heading a bounded distance — contact-is-a-data-point doctrine:
  the way in is the one path known clear, and reverse is where lidar coverage
  exists (mount yaw ≈179°) while ToF is blind under 0.27 m (named constraint);
  (3) pivot toward the open sector through the supervisor's own admission;
  (4) end with a typed outcome (`escape_outcome` shapes), including honest
  failure. Hard caps: wall-clock, distance, duty (the clamp stays); any fresh
  contact or stall mid-escape stops and reports.
* **The admission question is the design's heart** (unreachable-recovery
  lesson, third form: un-grantable-by-construction): the escape is motion
  while close — the exact thing the supervisor refuses. The escape mode must
  carve a NARROW grantable shape (slow reverse away from the blocking sector,
  live rear-clearance checked per tick), designed WITH the admission rule, and
  proven grantable on the bench before any field word. A primitive whose own
  arbiter always refuses it is D52 wearing a new coat.
* **Lifecycle:** goal-cancel-kills-the-ladder (2026-08-11, 26 s pushing a
  bench leg): the escape must either be uninterruptible by mission lifecycle
  triggers or handle cancel as stop-and-report — never half-executed.

## Falsifiers before certifiers (pre-staged, both directions)

The archived pocket room (03_validation/pocket_trap_2026-08-19/) reproduces the
trap on the unfixed build — N=3 already banked, so the falsifier exists before
either fix. Bars, pre-registered here for the daylight round to ratify or
amend: for A, the replay refuses the pocket goal AND a same-distance
exit-plannable goal is still chosen (selection not merely narrowed); for B, a
rig mission that enters the archived pocket EXITS it and completes, plus a
D52-shaped bench: a mark-painted pose where BT recovery refuses and the
primitive succeeds over lidar-clear floor. B additionally needs its own bench
card (admission-grantability, bounded-stop on injected contact, outcome
honesty) before any field arm.

## What was deliberately NOT designed here

No parameters, no duty numbers, no sector geometry, no admission thresholds —
those are measurements for the round that builds it (no-room-specific-solutions:
constants derive from robot and sensor, on the bench, with Scott staging).
