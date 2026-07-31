# Milestone 8 Phase 1 — finish-only active-leg eligibility

## State

The finish gate was merged and deployed at `8ff975338b87743e96d93b3e67436ac145b38565`.
Attended physical attempt 1 ended at the bounded mission lease with clean
relock/cleanup, no contact, and about 0.925 m maximum displacement. It did not
meet the Phase 1 acceptance gate: its first target was 86.35 degrees to the
right, so the run is `geometry_ineligible`, and its 226 fragmented forward
windows contained no qualifying continuous one-second straight window. The
operator saw similar jitter and the rover became hung on a raised chair-mat
ridge. This yields no motor-breakaway verdict and does not route to Phase 0B.

## Selected fix

When a revalidated semantic `finish` becomes ready, the live controller now
waits for an explicit outcome from the currently accepted Nav2 leg. A finish is
eligible only when the adapter reports both:

- `state == "wait_planning"`; and
- `goal_active == false`.

`dispatching`, `navigating`, `holding`, missing/ambiguous status, or any active
goal defers only that semantic finish. The controller publishes
`semantic_finish_deferred_active_leg`, keeps `cancel_active_goal=false`, and
records one durable `semantic_finish_deferred` journal event per decision
generation. After the adapter reports the leg outcome, it records
`semantic_finish_released` and completes normally.

Nav2 remains the progress arbiter. Its fixed `PoseProgressChecker` accepts
either 0.02 m translation or 0.10 rad rotation within 15 seconds; a genuinely
stalled leg therefore reaches a normal Nav2 abort/outcome rather than being
preserved forever by this gate.

## Finish-only safety boundary

The gate is intentionally downstream of all independent safety and authority
checks and is restricted by an explicit `action == "finish"` predicate. It
does not alter semantic `wait`, any motion decision, or any motor/safety owner.

These paths never consult or wait on finish eligibility:

- operator STOP and ESTOP;
- collision-supervisor veto;
- mission cancellation;
- motion-critical stale evidence;
- command or mission lease expiry;
- invalid/expired authority or exact-SHA mismatch;
- adapter recovery and graph-integrity failure.

STOP, ESTOP, collision, cancellation, and stale-evidence regression cases prove
that a ready finish cannot delay the follower's immediate zero command. Mission
lease and authority remain independently enforced by the authority owner,
adapter, and private command bridge. The collision supervisor remains the sole
`/cmd_vel_motor` publisher, and the bridge remains the sole `/cmd_vel`
publisher.

## Why generation 2 selected finish

Phase 0A showed more than a vague “model gave up” hypothesis. The captured
generation-2 snapshot set `evaluation.recommend_finish=true` whenever there had
been at least one motion dispatch and any camera observation. At the same time,
its `active_goal` explicitly said `follower_state=navigating`, ETA 13.5 seconds,
and 1.35 m remaining. The semantic prompt tells the provider to prefer
`finish`/`partial` when `recommend_finish` is true. Those deterministic inputs
therefore encouraged the early finish despite the active leg.

That heuristic is not changed here because Phase 1 implements one selected
fix. Ranked follow-ups are:

1. Make `recommend_finish` reflect actual objective/coverage/traversal evidence
   and never infer mission exhaustion from one dispatch plus any observation.
2. Bias server-owned candidate/approach selection toward a safe reachable
   forward candidate when one exists, reducing behind-target starts.
3. If sustained straight 0.10 m/s output still fails to translate under load,
   return to the separately authorized Phase 0B breakaway sweep.

## Bench/replay validation

The focused tests cover:

- finish deferral for active and dispatching Nav2 states;
- release only after a settled Nav2 outcome;
- non-applicability to semantic `wait`;
- explicit bypass classification for STOP, ESTOP, collision, cancellation,
  stale evidence, lease expiry, and invalid authority;
- immediate zero/termination with a ready finish for STOP, ESTOP, collision,
  cancellation, and stale motion evidence;
- SHA-bound Phase 0 report regeneration with maximum angular magnitude added
  to each forward window for straight-window acceptance.

No ROS process or robot is required for these tests.

## Exact-SHA attended validation gate

The candidate may proceed only after review, a clean Pi build/no-motion check,
deployment with matching source/deployed/reviewed SHAs, and a new authenticated
browser approval while the operator is present in a level bounded room. All M7
safety gates and generated cleanup evidence remain mandatory.

The physical acceptance run must preserve the private synchronized trace and
pass all of the following. Before attributing any failure to motor breakaway,
the SHA-bound dispatch geometry must show an initial target bearing within
`±45°` of the rover heading. This is an evidence-eligibility rule, not a runtime
motion or safety gate. A target outside that forward sector makes the trial
geometry-ineligible: terminate/clean up normally and repeat only under a new
attended authorization. It produces no motor verdict and cannot route to Phase
0B.

1. The controller records a deferred finish without the adapter receiving
   `controller_complete` while its goal is active, if the model returns finish
   during the leg. A model that chooses a valid successor instead is acceptable;
   the deterministic replay already exercises the finish case.
2. The rover travels at least 0.50 m by aligned odometry during the accepted
   mission, with no contact and no STOP/ESTOP/collision/lease/freshness gate
   regression.
3. At least one motor-output forward window is continuous for at least 1.0
   second, has time-weighted mean linear command at least 0.09 m/s, maximum
   absolute angular command at most 0.05 rad/s, and produces at least 0.05 m
   aligned net odometry using the fixed 0.25-second response allowance.
4. Downstream angular sign reversals remain zero. Angular-floor duty, discrete
   angular jerk, absolute yaw per metre, and motion-start rate are compared
   against the Phase 0A baseline, and the operator records whether visible
   jitter is reduced. Any regression is reported rather than hidden by the
   longer run.
5. Terminal cleanup proves zero commands, relocked authority, inactive physical
   units, no UART/lidar owner, consumed activation files, and bounded evidence.

The 1.0-second straight window is the key load test: many sub-0.3-second
fragments do not qualify. The analyzer records one of these motion-evidence
outcomes; safety, cleanup, jitter, and operator-observation gates remain
separate and can still fail the attended run:

- `geometry_ineligible`: initial absolute target bearing is greater than 45°.
  The trial is invalid for forward-breakaway attribution; address/retest the
  ranked geometry/forward-bias follow-up, with no Phase 0B verdict.
- `forward_command_inconclusive`: eligible geometry, but no continuous
  motor-output window meets the duration, linear, and angular thresholds.
  Inspect path/controller/bridge command generation; this is not evidence of a
  motor stall and does not route to Phase 0B.
- `phase0b_breakaway_required`: eligible geometry and a qualifying sustained
  straight motor-output window exist, but none reaches 0.05 m aligned odometry.
  This is the only outcome that routes to the separately authorized Phase 0B
  breakaway sweep.
- `mission_distance_incomplete`: a qualifying window translates, but total
  aligned mission travel remains below 0.50 m. Forward breakaway is no longer
  the diagnosis; the run still fails Phase 1 acceptance.
- `motion_evidence_pass`: geometry, straight-window translation, and total
  distance meet the motion thresholds. This is necessary but not sufficient
  for the full attended pass.

Thus neither a rotation-dominant goal nor failure to produce the commanded
straight test window can be misreported as a motor-floor failure. Skipping 0B
before this fix still must not be interpreted as proof that the 0.10 m/s motor
floor is adequate.

The exact command for post-run analysis remains:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.drive_trace_analysis \
  /path/to/private-phase1-trace.previous.jsonl \
  /path/to/private-phase1-trace.jsonl \
  --context /path/to/sha-bound-phase1-context.json \
  --output /path/to/phase1-drive-analysis.json
```

Trace paths are chronological. A single unrotated trace remains accepted; for
rotated traces, concatenating the ordered segment bytes is the exact digest and
event stream analyzed. Reversing the segments fails closed because the stream
must begin with its one `trace_started` record and end with its one
`trace_summary` record.

## Safe-timeout evidence contract

A bounded lease expiry is a valid evidence terminal, not a successful mission.
The analyzer accepts it only when the mission-bound controller terminal is
exactly `recovery_required` / `mission_lease_expired` and the SHA-bound context
contains exactly one terminal proof: either the existing semantic completion or
a `mission_terminal` with `status=timeout`,
`controller_state=recovery_required`, and `reason=mission_lease_expired`.
Other recovery terminals remain rejected.

The browser's authenticated no-contact observation is likewise available after
a clean lease-expired canonical result only when durable evidence proves all of
the following: canonical proposal/result schemas and matching mission ID;
matching source/deployed SHAs; the exact canonical lease-expiry reason;
successful cleanup; motion authority false; restart/resume false; and a finite,
closed run interval. Normal completed missions retain their prior eligibility.
Every other timeout or terminal state remains ineligible. This observation
contract changes neither the mission result nor any motion, authority, lease,
STOP, ESTOP, collision, or cleanup behavior.
