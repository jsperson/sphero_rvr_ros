# Design note (a): the general linear speed raise

*Scott's directive, post re-fly (2026-08-19, verbatim): "It moves pretty slowly.
If we can increase the velocity generally, that would probably be both easier on
the hardware and improve reliability of locomotion." Scoping addendum, same
hour: "Turns are plenty fast so no need to increase there." — LINEAR ONLY;
rotation rates untouched. Design round only; nothing moves without consensus.*

## The evidence base (the re-fly's read, bag_20260819_141021)

Two findings frame this note, and the second was not what anyone expected:

1. **Cruise never lives at its own cap.** Succeeded goals averaged 0.05–0.10
   m/s with only bursts touching the configured 0.20; long stretches ran at
   0.053–0.066 — BELOW `regulated_linear_scaling_min_speed` (0.10), i.e. RPP's
   curvature regulation dominates the speed budget in this furniture density.
2. **The stall-kill population was not friction.** Six of eight watchdog kills
   show real DRIVING (mean |vx| 0.05–0.18, path 0.13–0.83 m covered in-window);
   only one was pivot-dominated; only 2 firmware stalls occurred all flight.
   The kills are the explorer's 6 s / 0.10 m NET-DISPLACEMENT bar firing during
   slow maneuvering — the bar's floor is 0.017 m/s net, and curvature-regulated
   crawling sits close enough that ordinary careful driving is indistinguishable
   from stalling. Scott's morning hypothesis (friction wins at crawl speeds)
   remains true for FROM-REST starts — every morning stall was one — but the
   afternoon's kills are bar arithmetic, not physics.

So the raise has two co-equal halves: **cruise speed up** AND **the progress
bars re-derived against commanded speed**. Raising one without the other is
half a fix.

## The three gates that must move TOGETHER (the gap-crossing lesson)

| gate | file | today |
|---|---|---|
| RPP `desired_linear_vel` | lean_nav2_stock.yaml | 0.20 |
| driver `max_linear_mps` | lean_rvr_tank_si.yaml | 0.20 |
| supervisor `max_forward_mps` | collision_stop.yaml | 0.20 |

`max_forward_mps` is the HIDDEN FINAL GATE (memory: gap-crossing era — a raise
that skips it silently changes nothing). One decision, three diffs, one test
pinning their equality or their deliberate inequality.

## Margins that silently assume 0.20, each re-derived WITH RECEIPTS

- **Braking envelope**: the supervisor's own comment block records effective
  hard-stop = max(stop_distance_m, physics term). The physics term scales with
  v (v·latency + v²/2a); the implementation batch re-runs that arithmetic at
  the candidate speed with the MEASURED decel (bench receipt if none exists on
  file) and states the new effective stop distance. `stop_distance_m` 0.30 /
  `slow_distance_m` 0.50 move only if the arithmetic says so.
- **ToF staleness-vs-travel**: staleness bound 0.30 s; travel per bound at
  0.20 = 0.06 m, at 0.35 = 0.105 m, at 0.45 = 0.135 m — all against the tof
  brake's 0.45 m stop distance and the 8×8's ~7 Hz frame cadence
  (floor-per-frame at v). The derivation goes in the yaml comment beside
  whatever number wins.
- **Lidar reaction distance**: supervisor tick + scan age bounds × v.
- **Progress bars, BOTH of them**: the explorer's `goal_progress_timeout_s`
  6 s / `goal_progress_epsilon_m` 0.10, and the controller's
  PoseProgressChecker `required_movement_radius` 0.05 / 12 s. **PIN (consensus
  2026-08-19): the bars key on the REGULATED MINIMUM, never on cruise.** A
  cruise-keyed bar reopens the exact hole being fixed: cruise rises ~75%
  (0.20→0.35) while careful-maneuvering speed rises only ~50% (min 0.10→0.15),
  so a "k × v_cruise × window" bar tightens FASTER than crawl speeds up and
  the false stalls return at the new numbers. Honest base = the slowest
  LEGITIMATE sustained speed: "net displacement ≥ k × v_regulated_min ×
  window", k < 1 stated and justified against the re-fly's MEASURED crawl
  distribution — which ran 0.053–0.066 m/s, BELOW today's configured 0.10
  minimum, and the derivation must account for WHY (RPP regulates below its
  own floor in tight curvature and approach phases) before trusting the
  proportional raise of the minimum at all.
- **Lookahead**: RPP `lookahead_time` 1.3 s is velocity-scaled already
  (`use_velocity_scaled_lookahead_dist: true`, max 0.36 m) — verify the max
  doesn't clip at the new cruise (0.35 × 1.3 = 0.455 > 0.36 → the max WOULD
  clip; decide deliberately, don't inherit).

## Candidate landing zone (to be earned, not assumed)

Hardware headroom is large (tank SI handles ~1.5+ m/s per tread; the smooth
proof point was 0.05 m/s — Get-Well era caution, not a measured ceiling).
Candidate: **0.35 m/s cruise** if every derivation above clears; fallback 0.30.
`regulated_linear_scaling_min_speed` rises proportionally (0.10 → ~0.15) so
curvature crawl doesn't reopen the false-stall gap the bar re-derivation
closes. Rotation constants: UNTOUCHED (Scott's addendum), including
rotate_to_heading_angular_vel 3.55 and everything the pivot curve feeds.

## Certification

Rig first (falsifier: the rig must show the CURRENT config's bar-vs-crawl
false-stall shape before certifying the new one — we own the re-fly bag to
replay), then a field A/B on the same room: coverage rate, stall_killed count,
firmware stalls, supervisor brake engagements at speed. The re-fly's numbers
(174.5 s / 7.29 m² / 8 kills / 2 stalls) are the baseline row already banked.

## Rollback

All yaml; one commit reverts; no code paths change shape.
