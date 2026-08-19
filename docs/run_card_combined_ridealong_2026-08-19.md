# RUN CARD — combined ride-along, 2026-08-19

**One flight, two firsts and a receipt:** the watcher's first default-ON flight
(Scott's ratification, this morning — the watch item is the condition he attached)
and the frontier lane's first field exercise ever (explore-on-stock certified in
the rig 2026-08-19; a rig has no real room). Plus the spin-up A/B receipt, free.

**Staging:** rover at the start point, facing EAST, Scott present and at the
switch. Battery read at the gate, floor 25 %. Flight SHA must contain the flip
commit `9b2aebb` and equal origin — the SHA, not the narration.

**The flight:** `launch_and_arm --stack stock-explore --no-arm` on the Pi. Full
explore-till-done coverage mission in the real room, SLAM live from an empty map.
No watcher flag — **the default IS the experiment**. `--no-arm` because arming is
PM-gated this flight: checklist to PM, PASS, Scott confirms he is clear, then
`ros2 service call /coverage_explorer/mission/start std_srvs/srv/Trigger`.

---

## The three named watch items

### W1. WATCHER (the ratification's own condition)

Every promotion is recorded (`/contact_marks/promote` — every firing including
rejected ones — and `/contact_marks`, both in the stock-explore bag by
construction) and **inspected post-flight against the room's ground truth**
(Scott's eyewitness annotations + the map). **ONE false promotion reverts the
default the same day (one launch arg, no code)** — that is Scott's condition,
verbatim in the launch description. Zero promotions in a room with no sub-lidar
obstacle is a pass, and is exactly what the certified mission's bar B5 showed.

### W2. FRONTIER LANE (first exercise, ever)

Goal ordering under genuinely unknown space has never run outside the rig. Watch
live and note anything that looks wrong — goal ping-pong, revisiting covered
space, churn against unreachable frontiers, envelope refusals stacking on one
wall. No live intervention for "looks inefficient"; intervention only for safety.
Full post-flight read from `/coverage_explorer/status`, the TRANSIENT_LOCAL
`/coverage_explorer/report`, and `/plan`.

### W3. PIVOT QUALITY (Scott's watch — "really sloppy navigation pivots")

Post-flight, per-pivot, from the bag: commanded-vs-achieved rotation
(`/cmd_vel` yaw rate vs `/odom`/IMU achieved), plus a stutter count during
rotations (zero-gaps and sign flips mid-rotation). **Pre-named remedy, so
post-flight analysis cannot move the goalposts:** if the rate-based residual is
real, Spin recovery routes through the precise_turn gateway — which stays
bench-gated (`b24d8b4` ships un-runnable; the 2026-08-19 bench card's 7 items
flip the flag). This flight only measures; no pivot fix flies today.

### Free receipt: spin-up A/B

The READY line's `staged_to_ready_s` is the A/B number against the 133 s
baseline (estimate 30–40 s). Costs nothing; read it off the log.

---

## Bringup sequence (checklist goes to PM before anything arms)

1. Pi state clean: no stale pidfile, `ros2 node list` empty, lidar motor stopped.
2. Battery ≥ 25 % (the probe gates it; read the number anyway).
3. `python3 scripts/launch_and_arm.py --stack stock-explore --no-arm` — runs
   gate_verify (HEAD == origin == the flip commit, tree clean, installed tree
   byte-matches), preflight, recorder + bag, then every probe gate (lifecycles,
   params-from-robot, tof rate band, brake not engaged, D29 disarmed, marks,
   battery, recording growth).
4. Report to PM: SHA, gate roster result, `staged_to_ready_s`, battery.
5. Scott confirms clear of the rover → PM's word → mission/start. Liftoff is the
   service call, never the launch.

## Endings, reasoned (any ending's first act is teardown — the bag finalizes on
bag-time, and the bag is the autopsy)

- **DONE** — report says coverage complete. Full success; all three watch items
  still get their post-flight reads before anyone celebrates.
- **HONEST END** — the explorer stops with its reason (no reachable frontier,
  envelope refusal, coverage stall). An end, not a failure: the reason string is
  the artifact, reconciled against Scott's eyewitness view of what was left.
- **PROMOTION, MISSION CONTINUES** — the expected shape if something sub-lidar
  is out there: pin, promotion, self-quench, detour. The product behavior.
  Centroid inspected post-flight against the actual obstacle (d45bd24's standard:
  centimetres).
- **FALSE PROMOTION** — a mark where ground truth says nothing was. The
  ratification's tripwire: the default reverts today, one launch arg, no code.
  The mission itself may still finish; the card records both facts separately.
- **LIVELOCK, NO PROMOTION** — pin sustained, recoveries mounting, watcher
  silent. The gates are too tight for field conditions or freshness is refusing
  real evidence; `/contact_marks/promote` (empty) plus `/diagnostics` is the
  story. Not a same-day retune — a filed defect with a bag.
- **CONTACT / FREEZE** — contact is a data point, not a mission end. The touch
  port marks it; watch that the mark redirects rather than traps.
- **BATTERY FLOOR MID-RUN** — end honestly, tear down promptly. The floor is a
  teardown trigger, not a sprint-to-finish license.
- **STACK DEATH / DISCONNECT** — teardown, bag finalize, autopsy from the bag
  before anything reruns. The disconnect path is freshly certified (`7942287`);
  this is its first flight too, in the background.

Scott's eyewitness annotations reconcile against the bag post-flight — where
they disagree, the bag wins on timing and the eyewitness wins on what the room
actually contained.
