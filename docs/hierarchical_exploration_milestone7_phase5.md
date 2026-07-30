# Milestone 7 Phase 5 — canonical physical mission

## Status

Implementation candidate complete; physical M7.6/M7.7 evidence is not yet
claimed. The executable candidate must pass bounded local tests, exact-SHA Pi
build/tests, a no-motion Pi graph and storage preflight, and an attended room
check before the browser may create the physical approval. The current rover
location on a desk is not an acceptable room.

## Canonical mission

The browser accepts exactly:

> Explore this room, identify and map the shoes and any recognized people,
> inspect uncertain findings from another viewpoint, then return or stop
> safely.

The persisted `sphero_rvr.hierarchical_physical_proposal.v1` contains only that
objective, revision `1`, requested classes `shoe` and `person`, the mission ID,
the exact source SHA, creation time, the browser-selected server-owned mission
lease, and its canonical digest. The selected lease must be finite, positive,
and no greater than the unchanged `900 s` ceiling. It cannot contain poses,
routes, velocities, ROS names, or model-owned safety settings. Server-owned
deterministic code continues to resolve all model-selected semantic IDs to
geometry.

## One authenticated M7.6 approval

The browser displays an explicit approval envelope containing the exact
proposal digest, selected mission lease, authenticated operator,
source/deployed/reviewed SHAs, all three accepted prior-evidence bindings,
room restriction, fixed limits, and the full risk ledger. The lease selector
is visible before proposal creation; changing it after creation disables
approval until a new digest-bound proposal is generated. Four explicit
confirmations are required:

- the operator is present with chassis power cut reachable;
- the floor area is level and bounded;
- stairs, ledges, and drop-offs are absent;
- the operator understands that negative-obstacle sensing is unavailable.

The Pi creates the approval; the browser never submits a digest or ROS command.
The approval binds:

- source, deployed, and reviewed SHA equality;
- the proposal digest, mission ID, authenticated Tailscale identity, and
  approval ID;
- the accepted M7.3 collision evidence digest;
- the accepted paired directional-veto evidence digest;
- the accepted M7.4 moving-perception evidence digest;
- the room restriction above;
- `0.10 m/s`, `0.4 rad/s`, `0.50 s` command lease, `0.500 s`
  localization freshness, the selected mission lease, and its unchanged
  maximum of `900 s`.

Approval expiry must equal approval time plus the selected lease. Any missing,
stale, altered, replayed, or mismatched field fails closed.

Approval also requires a server-generated no-motion sensor preflight from
`rvr-telemetry.service`. The live Pi cache must simultaneously contain:

- a valid lidar scan received no more than `5.00 s` ago;
- a calibrated camera frame received no more than `5.00 s` ago;
- valid stationary SLAM localization received no more than `5.00 s` ago;
- a populated live SLAM occupancy map received no more than `5.00 s` ago.

Every source must explicitly report `motion_authority=false` and
`physical_execution_enabled=false`. The approval transaction persists a compact
digest-bound capture with receipt/source times, fixed age ceilings, source
identity, and value digests. Missing, invalid, stale, nonstationary, or
motion-capable evidence prevents approval creation, so the motor-capable
systemd unit cannot start.

## Activation and non-resumption

`SystemdHierarchicalMissionSession` writes proposal, approval, and session
environment files atomically with user-only permissions. It stops the
no-motion telemetry unit, starts only
`rvr-hierarchical-mission.service`, and verifies the unit is active. The
motor-capable unit has no install target and `Restart=no`.

The semantic controller remains locked after graph startup. Before it may
start the first provider call, the mission service generates a read-only
activation capture from exact command output. It proves the exact clean
source SHA and that `bt_navigator` is lifecycle-active. Earlier phases and
the fixed launch topology establish the command-owner chain; repeating five
non-gating ROS-discovery queries here created startup latency and discovery
races without adding authority. The readiness capture is digest-bound and
persisted with the mission. Failure or cancellation during this check stops
the unit without semantic planning or a motion request.

Cancellation, terminal completion, timeout, authority loss, controller
recovery, or service restart stops the unit. Cleanup consumes the three
approval/session files plus the active-graph capture. A prior approval cannot
be replayed and a restart never resumes a route.

The command chain is unchanged:

```text
LLM semantic decision
  -> deterministic semantic ID / server geometry resolution
  -> NavigateThroughPoses
  -> Nav2 private /nav2_cmd_vel_request
  -> live_route_runner (sole /cmd_vel publisher)
  -> collision_stop (sole /cmd_vel_motor publisher)
  -> driver
```

The browser owns proposal, confirmation, cancellation, and read-only
observation. It has no ROS, Twist, serial, or direct motor surface.

After a completed run, the same authenticated browser displays a separate
attended-observation control. The approved operator must explicitly confirm
that no rover contact occurred. The Pi persists one digest-bound
`sphero_rvr.hierarchical_no_contact_observation.v1`; the evaluator will not
accept a hand-entered manifest substitute.

## Live mapped-object provenance

The moving perception adapter now invokes the surveyed Phase 2
camera/lidar localizer with the measured camera and lidar extrinsics. A
single eligible lidar cluster produces `lidar_range`; a missing cluster may
use calibrated `floor_projection`; timestamp mismatch, stale pose, invalid
floor geometry, and ambiguous lidar clusters remain `bearing_only`.
Bearing-only detections may appear as camera evidence but cannot enter the
point-track store or generate an inspection viewpoint. The mission
controller preserves the live method, uncertainty, camera/scan evidence IDs,
and server-owned geometry rather than assigning a method itself.

The live controller also consumes the collision supervisor's explicit
`scan_healthy` state and receipt time. Missing, unhealthy, future-dated, or
older-than-`0.300 s` collision evidence becomes `motion_evidence_stale`,
forces motor zero through the independent collision supervisor, but preserves
an accepted Nav2 goal so it can resume after a transient receipt gap. Camera,
map, and localization freshness remain mandatory for creating or replacing a
semantic plan. Their transient staleness pauses planning without canceling an
already accepted route; Nav2 remains responsible for localization/path
viability and the collision supervisor remains the final motor-zero
authority. A transient `0.750 s` authority-heartbeat miss follows the same
hold-and-resume rule at the private command bridge. SHA mismatch, malformed
authority, lease expiry, STOP/ESTOP, and a true controller recovery remain
terminal.

An accepted Nav2 route is also preserved when its rolling-map frontier
signature disappears. A frontier signature is planning metadata, not a
physical hazard; Nav2 still owns path validity and the collision supervisor
still owns obstacle safety. Nav2 `ABORTED` results such as `no valid path`
discard that goal and return to `wait_planning` for a different semantic
target instead of escalating an ordinary exploration dead end to process
recovery. The physical WFD view uses the same `0.22 m` circular-footprint
clearance as Nav2, rather than the replay-only point-clear default. If Nav2
still aborts an exact digest-bound batch, its active frontier is excluded for
the rest of that mission so the provider cannot repeatedly select the same
unreachable target.

The attended office run also established a drivetrain breakaway floor:
sub-`0.07 m/s` Nav2 translation requests are raised to `0.07 m/s` only while
the collision state is `CLEAR`. The independent downstream supervisor still
owns `SLOW`, STOP, ESTOP, freshness, and the absolute `0.10 m/s` ceiling. The
same rule uses the previously measured `0.35 rad/s` breakaway command for
near-pure CLEAR turns after a live `0.036 rad/s` Nav2 turn produced no odometry;
mixed arcs retain Nav2's angular rate and the absolute ceiling remains
`0.4 rad/s`. The
same loaded-Pi run showed one false fail-safe when a recovery stop missed the
driver's former `0.10 s` scheduling allowance; the installed allowance is
`0.20 s`, still inside the fixed `0.30 s` collision-veto bound.

The first exact-SHA canonical attempt also exposed ordinary rolling-map churn:
three asynchronous model responses were correctly rejected after their
frontier signatures changed, but the controller then treated that planning
churn as a terminal `semantic_revalidation_exhausted` fault. The installed
policy keeps the stale-response rejection, records every three-rejection
rollover, and immediately continues safe replanning while preserving any
already accepted Nav2 route. It does not relax collision, localization,
geometry ownership, command-lease, or motion-limit gates.

A healthy collision `BLOCKED` state now remains an immediate downstream
motor-zero hold without terminating the semantic controller or duplicating a
terminal event at controller frequency. Nav2 keeps the accepted or
digest-bound pending action and may replan; a persistent blockage still reaches
Nav2's bounded progress timeout and ordinary failed-target exclusion. STOP,
ESTOP, cancel, stale scan evidence, command leases, and authority expiry are
not included in this resumable collision hold.

Near-horizon floor anchors also fail closed to `bearing_only` when any pixel or
floor-height uncertainty perturbation cannot intersect the floor. Such an
anchor cannot support a bounded mapped point, but it is ordinary rejected
evidence and must not terminate the live perception process.

The canonical graph uses a moving-rover SLAM configuration with a `0.500 s`
map update interval, leaving scheduling margin inside the fixed `1.000 s`
map gate.

## Honest latency behavior

The physical controller uses the Phase 4 real-provider distribution:

- p95 `14.34809786885 s`;
- prefetch margin `1.0 s`;
- threshold `15.34809786885 s`;
- provider deadline `20.0 s`.

A compatible ready successor may hand off continuously. A short hop whose
successor is not ready enters visible `wait_planning` with zero motion. The
runtime does not pad fast provider calls and never moves speculatively.
A model `wait()` is also nonterminal: it holds `wait_planning` until a fresh
accepted semantic event starts another bounded provider call. Only a
validated `finish(outcome, evidence_ids)` completes the mission. The complete
final `finish()` decision and rationale are persisted in the binding journal
and canonical report.

## Durable M7.7 evidence

MissionService persists the proposal, approval, state transitions, controller
and adapter checkpoints, terminal result, source/deployed SHA, and cleanup
state. The binding journal persists authority activation/relock, real provider
durations, the exact bounded world snapshot supplied to every provider call,
bounded camera detections and localization provenance, model decisions and
rationales, semantic dispatches, independently reproducible server-resolved
goal batches, the actual bounded Nav2 `/plan` paths, event replans, handoffs,
and pauses. Repeated publication of an unchanged Nav2 plan is deduplicated by
its content digest.

The browser can reconstruct a mission by ID with:

```text
https://<Pi Tailscale host>/?mission_id=<mission-id>
```

`rvr_hierarchical_m7_canonical_validate` recomputes its report from the
MissionService SQLite database, binding journal, and generated raw cleanup
capture. It does not accept hand-entered pass booleans. The evaluator requires:

- valid proposal, approval, exact SHAs, and all prior evidence bindings;
- a valid fresh no-motion lidar/camera/SLAM/localization preflight persisted
  atomically with the approval;
- a generated activation capture, made before semantic planning, whose raw
  command output recomputes the exact clean checkout and lifecycle-active
  Nav2 navigator;
- one authority activation followed by relock;
- at least two explicitly real provider completions with measured wall
  durations, closing the multi-call rather than single-smoke-call gate;
- one exact, digest-bound world/camera evidence capture for every real provider
  completion, with the recorded provider snapshot ID equal to the snapshot
  digest;
- at least two materially distinct semantic goals with rationales and no model
  geometry;
- server-resolved goal batches that exactly recompute from the recorded strict
  semantic dispatches, plus at least one digest-bound actual Nav2 path for
  every dispatched semantic queue;
- evidence-bound mapped tracks when tracks exist;
- reconstructable nonzero occupancy coverage samples;
- reconstructable controller/adapter handoffs and every `wait_planning`
  interval, derived from checkpoint receipt times rather than hand-entered
  durations;
- nonzero odometry and at least `0.02 m` displacement;
- transient sensor/authority timing misses do not terminate the outer session
  or cancel an accepted route; collision/lidar receipt loss still forces
  motor zero independently, while stale planning evidence blocks new plans;
- a complete terminal controller checkpoint and terminal MissionService state;
- one authenticated, approval-operator-bound no-contact observation;
- inactive hierarchical and telemetry units, absent motion nodes/processes,
  zero `/cmd_vel` and `/cmd_vel_motor` publishers, no rover serial owner, no
  camera/rosbag evidence writer, at most 96 bounded JPEGs, and consumed
  activation files.

The output report embeds the raw active-graph and cleanup captures, service
events, binding events, provider-time world/camera snapshots, WFD frontier
history, semantic dispatches, recomputed goal batches, actual Nav2 paths,
terminal result, and reconstructed wait intervals so that the final evidence
artifact can be reviewed without trusting summary booleans.

Example post-run evaluation:

```bash
rvr_hierarchical_m7_canonical_validate \
  --mission-id <mission-id> \
  --output ~/.local/state/sphero_rvr/m7-canonical/<mission-id>/report.json
```

The command exits nonzero if any gate is false.

## Storage contract

The canonical graph reuses the bounded perception writer: at most 96 JPEG
evidence frames, each no larger than 512,000 bytes. The evaluator records
cleanup only after the hierarchical and telemetry units are inactive. No
unbounded rosbag capture is part of this run.

## Evidence log

Physical evidence will be added here only after the exact-SHA Pi deployment,
attended mission, terminal cleanup, and evaluator all pass. Until then:

- `m7_6_canonical_physical_mission`: `not_proven`
- `m7_7_durable_physical_evidence`: `not_proven`
- `canonical_mission_complete`: `false`
