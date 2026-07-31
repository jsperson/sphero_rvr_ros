# M7 Phase 5 canonical physical evidence

## Result

M7.6 and M7.7 pass on the 2026-07-31 attended physical mission at executable
source `40ddb7934715b717bb483ab117ae96ae6c31ca38`.

The browser-authored proposal and authenticated approval started mission
`m7-canonical-efd37eb020b7440a818f9ed6769bcb5f`. The full live hierarchy used
two real `gpt-5.6-luna` decisions, dispatched server-resolved frontier geometry
through Nav2, observed nonzero physical odometry, finished with the model's
truthful partial outcome, recorded the authenticated operator no-contact
observation, and relocked all authority. The durable evaluator passed every
check with report digest
`7bdf0a7069f93f86bef0a93efbc658a12eb46a3841fcfe3fc95f5da5bd9c9621`.

## Accepted measurements

- The real provider calls completed in `11.209783 s` and `6.005611 s`.
- The initial decision selected frontier
  `3546e59aaad95c8c0adb1dd96ad9afb88769bbfd1a17af0f1fb124da84409c20`;
  the second decision finished partial while citing live camera evidence.
- One actual Nav2 path with 16 sampled poses was bound to the semantic
  dispatch.
- Odometry recorded 87 samples, including 26 nonzero samples, with
  `0.137053 m` maximum displacement.
- The initial real-provider wait lasted `12.005475 s`. Two short rolling-map
  frontier invalidations produced recoverable `0.130605 s` and `0.366851 s`
  planning holds rather than terminating the mission.
- Nineteen transient localization timing holds were accounted for; no required
  motion-sensor freshness violation occurred and the outer session did not
  terminate on those recoverable planning misses.
- The authenticated approval operator reported no physical contact. The
  observation is digest-bound in the report.
- Generated cleanup found both run units inactive, no motion nodes or motion
  processes, zero `/cmd_vel` and `/cmd_vel_motor` publishers, no rover serial
  owner, no evidence writer, consumed activation files, and the camera writer
  within its fixed 96-JPEG rolling limit.

## Evidence layout

- `report.json` is the self-contained, fail-closed M7.7 report recomputed from
  the MissionService database, physical-binding journal, live checkpoints,
  provider-time snapshots, actual Nav2 plan, terminal result, authenticated
  observation, and generated cleanup capture.
- `evaluate.log` records the evaluator command result.
- `committed_artifact_sha256.txt` records the committed artifact hashes.

The original generated report remains on `sphero-pi-2` at
`~/.local/state/sphero_rvr/m7-canonical/m7-canonical-efd37eb020b7440a818f9ed6769bcb5f/report.json`.

## Disclosed without concealment

1. The operator observed substantial rapid left/right turning jitter. The
   pulse-density breakaway change did not solve that drive-quality problem.
   This run proves safe full-loop execution and durable evidence, not smooth or
   efficient navigation.
2. The accepted run covered only `0.137053 m` before the second real provider
   call chose the permitted truthful partial finish. It is not evidence of a
   long room traversal or continuous short-hop handoff under real latency.
3. The current durable report records odometry, decisions, goals, plans, holds,
   and cleanup, but not the high-rate `/cmd_vel_motor` trace needed to measure
   angular sign reversals. Follow-up drive-quality work must capture that trace
   and diagnose DWB output, bridge modulation, collision scaling, and odometry
   together before another tuning claim.
4. Camera storage consists of three independent bounded 96-frame rolling
   stores plus three historical deployment images (291 files total). The
   canonical evaluator checks the active hierarchical writer's 96-frame bound;
   filesystem use was 53% with about 27 GB free after cleanup.
5. The camera node again required SIGKILL during launch teardown. The evaluator
   nevertheless recomputed a clean final state with no camera process or
   writer. This remains teardown hygiene, not hidden motion authority.

## Scope boundary

This evidence proves one attended, no-contact M7 canonical physical mission and
the durable M7.7 evidence/cleanup contract. It does not approve unattended
driving, operation near stairs or drop-offs, or claim that the remaining
turning jitter is acceptable.
