# Milestone 7 physical hierarchical exploration

## Phase status and scope

This document is the no-motion Milestone 7 Phase 0 contract. It begins from
merged Milestone 6 main
`97b53c612e95a6f06fb481cad747d11d30a906fa`, which contains the replay-proven
hierarchical engine and the durable Phase 4 real-provider evidence.

Phase 0 made the short-hop product decision, fixed the entry-gate order, and
maps the physical integration seams. It does not add a physical launch,
executor, ROS node, dependency, service, publisher, serial owner, or motion
authority. It does not deploy to the Pi or start sensors. Every installed
physical and live-execution flag remained false and the reviewed SHA remained
blank in that slice.

Phase 1 is complete at executable source
`9822ec6fe8c903191329ebdbb2646cac745e25ad`. Its exact-SHA Pi evidence is
recorded in
[hierarchical_exploration_milestone7_phase1.md](hierarchical_exploration_milestone7_phase1.md).
It passed recorded-map determinism/performance and the loopback-only ownership
graph without starting a driver, serial transport, live sensor, physical
executor, or motion authority. Later gates remain closed.

## Short-hop product decision

Milestone 7 accepts bounded, visible `wait_planning` pauses when a fresh
semantic successor is not ready at arrival. Continuous motion is required
between compatible ready goals; it is not promised for a hop shorter than the
real provider latency.

The initial physical implementation will retain one active semantic goal and
one asynchronously prefetched semantic successor. It will not speculate two or
three LLM decisions ahead from one old snapshot. The deterministic Nav2
follower may still hold its existing bounded two- or three-pose path queue, but
every queued pose must come from an individually validated, server-resolved
goal; it is not a queue of unvalidated model commitments.

This choice is based on the Phase 4 measurements:

- four warm real-provider calls took `14.545232`, `8.306876166`,
  `13.231004459`, and `7.929764292` seconds;
- interpolated p95 was `14.34809786885` seconds;
- three 25-second legs handed off atomically with no replay motor-zero
  interval;
- every 0.5 m / 5 second hop reached `wait_planning`, for pauses of
  `9.545232`, `3.306876166`, `8.231004459`, and `2.929764292` seconds.

Deeper semantic lookahead would front-load several calls, bind later decisions
to older geometry and perception, and increase discard risk when people,
objects, or frontiers change. Provider latency is not deterministic enough to
make reduced latency a safety or product guarantee. Longer legs may reduce
pause frequency, but may not override reachability, freshness, semantic value,
coverage, or operator intent.

### Physical timing contract

- Start the one allowed semantic prefetch when ETA first becomes less than or
  equal to the conservative provider p95 plus a `1.0 s` margin. For the Phase 4
  evidence that threshold is `15.34809786885 s`. If a leg begins below the
  threshold, dispatch immediately.
- A later measured p95 may increase the threshold, but must not reduce it below
  the committed Phase 4 p95 without a separately reviewed real-provider
  distribution.
- The initial physical semantic-call deadline is `20.0 s`. This is a bounded
  planning/UX deadline, never an extension of a motion or command lease.
- If the successor is ready and revalidates at arrival, hand it to Nav2
  atomically without a deliberate zero.
- If it is late, let the private-command lease expire to zero and enter the
  explicit `wait_planning` state. No previous path, velocity, or goal lease is
  extended merely because the provider is still running.
- If the call completes before its deadline, revalidate its snapshot,
  frontier/track identity, reachability, motion-critical freshness, and budget
  before starting a new controller session.
- If the deadline expires, cancel the call, remain at zero, discard any late
  result, and record `planning_timeout`. Recovery requires a fresh bounded
  decision or truthful terminal outcome; it never resumes an old command.

Every pause is first-class evidence: dispatch, arrival, zero-command time,
provider-ready/deadline time, revalidation result, resume time, and total pause
duration must be persisted and shown in the browser. An accepted bounded pause
is not reported as continuous motion.

## Safety and ownership invariants

The physical graph must preserve this exact ownership chain:

```text
Nav2 controller / behaviors
  -> /nav2_cmd_vel_request
  -> live_route_runner (sole /cmd_vel publisher; receipt-time lease and caps)
  -> collision_stop_node (sole /cmd_vel_motor publisher; independent veto)
  -> rvr_node (subscriber / hardware transport)
```

- Nav2, its behavior server, and costmaps are planning aids, not safety
  authorities.
- Nav2 may publish only to the private request topic. It may not publish
  `/cmd_vel` or `/cmd_vel_motor`.
- `live_route_runner` remains capped at `0.10 m/s` linear and `0.4 rad/s`
  angular with a command lease no longer than `0.50 s`.
- `collision_stop_node` independently owns collision slow/stop, STOP, ESTOP,
  stale-command zeroing, and final motor arbitration. Model inference cannot
  delay or clear a veto.
- Stale motion-critical lidar or localization, collision, STOP, ESTOP,
  cancellation, mission-lease expiry, and lost command lease stop
  deterministically. A stale planning snapshot is discarded and replanned.
- Restart never resumes a prior physical mission; it becomes
  `recovery_required`.
- The model continues to return only snapshot-bound semantic IDs and
  evidence-linked outcomes. Deterministic code owns geometry, paths, speeds,
  leases, thresholds, and revalidation.
- Drop-off detection remains unavailable. Runs are attended and limited to a
  level, bounded room with no stairs, ledges, or open drop-offs.

The replay launch is not a physical launch template. A later physical launch
must be separate, default every motor-capable group to false, use live time,
and be activatable only by the existing digest-bound exact-SHA approval owner.
No motor-capable unit may be enabled at boot.

## Entry gates and evidence order

Gates are sequential. Passing a later-looking software test cannot waive an
earlier physical or human gate.

| Gate | Required evidence | Authority |
| --- | --- | --- |
| M7.0 product decision | This reviewed short-hop and gate contract | No ROS, sensors, or motion |
| M7.1 exact-SHA no-motion Pi | WFD timing/RSS/determinism and ROS graph ownership on the Pi; driver and serial owner absent | Sensors or recorded inputs only under a separate no-motion session |
| M7.2 surveyed localization | Physical targets with surveyed map coordinates/ranges; per-method errors, sync, ambiguity, and uncertainty | Stationary sensing only; no driver |
| M7.3 collision validation | Attended slow/stop/manual-reset/no-contact evidence at the exact candidate | Separately approved bounded motor-capable validation |
| M7.4 moving perception | Live lidar/SLAM/camera freshness, transforms, mapped detections, and replan evidence while moving | Separately approved bounded motor-capable validation |
| M7.5 physical binding | Default-off live Nav2/semantic-goal adapter, authority owner, durable evidence, and browser surface | Build/replay first; no motor activation |
| M7.6 execution approval | Human approval binds the final candidate SHA, room restriction, mission, limits, and cleanup | Required immediately before the canonical run |
| M7.7 canonical mission | Attended end-to-end browser mission and terminal cleanup evidence | Only the approved exact SHA |

M7.1 must record:

- candidate SHA, Pi image/ROS/Python identity, map/input checksums, WFD settings,
  50-pass timing distribution, maximum RSS, frontier signatures, and cleanup;
- `ros2 node list`, topic publishers/subscribers, action servers, and lifecycle
  states proving only the allowlisted Nav2 controller/behavior nodes publish
  the private request topic, one `/cmd_vel` bridge publisher, one
  `/cmd_vel_motor` supervisor publisher, and no direct Nav2 public/motor
  publisher;
- no `rvr_node`, rover serial/UART owner, nonzero command, physical execution,
  or motion authority.

M7.2 must replace the provisional analytic/approximate Phase 2 claims. At least
three surveyed target positions per point-producing method and range band must
be recorded. The existing synchronization, pose-age, ambiguity rejection, and
bearing-only-never-a-point gates remain mandatory. Tolerances may tighten from
physical evidence; widening them requires an explicit review and rationale.
For unambiguous collection, the reviewed bands are fixed as near
`[0.30, 0.55) m`, mid `[0.55, 0.85) m`, and far `[0.85, 1.20] m`. See
[hierarchical_exploration_milestone7_phase2.md](hierarchical_exploration_milestone7_phase2.md).

M7.3 and M7.4 are motor-capable physical validations and each needs its own
exact-SHA approval and attended cleanup. Approval for either does not approve
the canonical mission. M7.6 is a new final approval after the complete physical
candidate and all prior evidence are reviewed.

## Delivery slices

Each slice is one reviewable pull request:

1. **Phase 0 — decision and gate contract:** this documentation-only slice.
2. **Phase 1 — no-motion Pi candidate:** add the default-off graph/audit tooling
   and collect M7.1 evidence; no driver or serial transport.
3. **Phase 2 — surveyed stationary localization:** collect and validate M7.2
   evidence without motion authority.
4. **Phase 3 — attended safety and moving perception:** perform M7.3 and M7.4
   only under separately approved exact SHAs.
5. **Phase 4 — physical hierarchical binding:** connect the M6 engine to the
   live graph, still installed and tested default-off.
6. **Phase 5 — canonical physical evidence:** obtain final exact-SHA approval,
   run the mission from the browser, persist/reopen it, and verify cleanup.

Phase 1 may begin only after human review confirms the short-hop product
decision and this gate sequence.

## Milestone 7 acceptance

Milestone 7 passes only when the canonical mission runs on the real rover and:

- live evidence causes materially different LLM-selected semantic goals;
- requested objects and enrolled/unknown faces are mapped truthfully with
  evidence and uncertainty;
- compatible long legs remain continuous while short hops use the documented
  bounded-pause behavior;
- collision, STOP, ESTOP, stale evidence, command lease, cancellation, and
  restart behavior remain independent of provider inference;
- goals, rationales, paths, detections, handoffs, pauses, coverage, terminal
  result, exact SHA, approvals, and cleanup are durable and reopenable by
  mission ID.

Exact distance or angle reproduction is not an acceptance gate.
