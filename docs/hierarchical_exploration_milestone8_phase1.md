# Milestone 8 Phase 1 — finish-only active-leg eligibility

## State

The bench/replay implementation is complete. Physical deployment and attended
motion validation are deliberately pending a separate exact-SHA authorization.
This change reaches motor behavior indirectly by allowing an already accepted
Nav2 leg to continue, so it must not be deployed or exercised under the Phase
0A no-motion authority.

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
pass all of the following:

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
fragments do not qualify. If the motor window occurs but aligned odometry does
not reach 0.05 m—or if no qualifying straight window can be produced—the
attended result is not a Phase 1 pass. Phase 0B becomes the next action; this
must not be interpreted as proof that the 0.10 m/s motor floor is adequate.

The exact command for post-run analysis remains:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.drive_trace_analysis \
  /path/to/private-phase1-trace.jsonl \
  --context /path/to/sha-bound-phase1-context.json \
  --output /path/to/phase1-drive-analysis.json
```
