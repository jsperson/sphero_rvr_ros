# Continuous hierarchical exploration

## Status

This document is the approved Milestone 6 Phase 0 design decision for
continuous, recognition-driven exploration, verified against repository
baseline `121423ce808ceb998ca990cb80efa77ae7f2c956`.

Phase 1 now implements the default-off replay slice from baseline
`7b948c766384df36dcb8a9ea950c9297e879486b`. Its implementation and acceptance
evidence are recorded in
[hierarchical_exploration_phase1.md](hierarchical_exploration_phase1.md).
Phase 2 adds the recorded/offline camera-to-map localization layer from merged
Phase 1 baseline `6cf718fd0204aa75778f4372073396e0d65c2186`; its bounded
association, floor-projection, uncertainty, and ambiguity evidence is recorded
in [hierarchical_exploration_phase2.md](hierarchical_exploration_phase2.md).
Phase 3 adds replay-only semantic-goal selection, deterministic
Next-Best-View resolution, async prefetch, snapshot revalidation, and
event-triggered replanning from merged Phase 2 baseline
`9ddc2b64dc9c626e07df5e4945297ffea28fdb2a`; its acceptance evidence is
recorded in
[hierarchical_exploration_phase3.md](hierarchical_exploration_phase3.md).
Phase 4 adds durable multi-decision real-provider latency, handoff, pause, and
browser evidence in
[hierarchical_exploration_phase4.md](hierarchical_exploration_phase4.md).
Milestone 6 is replay-complete at merged main
`97b53c612e95a6f06fb481cad747d11d30a906fa`. The separately gated physical
realization and its explicit bounded short-hop pause decision are defined in
[hierarchical_exploration_milestone7.md](hierarchical_exploration_milestone7.md).
Physical hierarchical exploration remains unavailable.

## Decision

Adopt Nav2 for the deterministic continuous-motion layer. Do not grow
`live_route_runner` into a global planner, local planner, or costmap.

The initial replay integration should use:

- `SmacPlanner2D` for global planning;
- `DWBLocalPlanner` for path following;
- a Nav2 behavior tree that can update or extend the active goal without
  terminating `FollowPath`;
- `NavigateThroughPoses` semantics for passing through an intermediate
  frontier without stopping when a valid prefetched goal is ready;
- a private Nav2 command topic, gated and republished to `/cmd_vel` by the
  hierarchical mode of `live_route_runner_node`;
- the existing `collision_stop_node` as the sole `/cmd_vel_motor` publisher and
  independent final motion veto.

This is a starting configuration, not a claim of physical acceptance.
`SmacPlanner2D` is appropriate for a circular differential-drive model. The
first costmap should conservatively inscribe the complete rover and payload in
a circle. If that conservative footprint prevents useful coverage, Phase 1 may
replace only the planner with `SmacPlannerLattice` and a differential-drive
lattice. `SmacPlannerHybrid` is not selected because Nav2 documents it for
car-like/Ackermann platforms.

DWB is selected as the initial controller because it exposes explicit minimum
and maximum sampled linear/angular velocities and acceleration limits that can
represent the installed `0.10 m/s` linear and `0.4 rad/s` angular envelope.
Phase 1 must tune its samples and critics so it cannot request sub-breakaway
turning indefinitely, oscillate near a goal, or hide a lack of progress.

Regulated Pure Pursuit remains a comparison candidate, not the initial
controller. Its current Nav2 documentation requires a regulated minimum speed
greater than `0.1 m/s`, which conflicts with this rover's absolute `0.10 m/s`
ceiling when those regulation features are active. It may replace DWB only if a
reviewed configuration and replay prove that every minimum-speed,
rotate-to-heading, approach, acceleration, and cancellation behavior remains
inside the installed caps without relying on downstream clamping. A controller
failure is not permission to raise a physical limit.

MPPI is deferred. Its documented performance is from an Intel processor and
its default sampling workload is unnecessary until the simpler controller has
a measured failure. A later change may consider MPPI only after recording Pi
CPU, memory, command rate, deadline misses, and path-quality evidence under the
complete lidar, camera, SLAM, web, and mission-service workload.

Nav2 costmaps and controller collision checks are planning aids. They do not
replace, weaken, reset, or satisfy the independent collision supervisor.

## Scope and non-goals

Milestone 6 replaces the per-primitive LLM loop with three layers:

1. **L1 continuous motion:** deterministic path planning and following to a
   validated goal pose.
2. **L2 frontier and viewpoint generation:** deterministic candidates derived
   from occupancy, reachability, clearance, coverage, and observation geometry.
3. **L3 semantic goal selection:** the LLM selects one typed goal from bounded
   candidates and evidence; it never selects a route, velocity, motor command,
   safety threshold, or ROS surface.

Phase 0 does not:

- implement any of these layers;
- add Nav2 or a frontier package to the workspace;
- change the existing adaptive mission schema;
- modify physical approval or execution;
- resolve the remaining attended collision and moving-perception gates;
- claim detection, localization, or exploration acceptance.

## Existing seams

| Responsibility | Current seam | Milestone 6 relationship |
|---|---|---|
| LLM decision | `adaptive_mission_controller.py` | Evolves from primitive intent to snapshot-bound semantic goal selection. |
| Executor protocol | `AdaptiveMissionExecutor` | Evolves to accept one validated goal while retaining snapshot, cancellation, refresh, and evidence boundaries. |
| Replay | `ReplayAdaptiveMissionExecutor` | Remains the default no-authority integration seam. It gains deterministic map/frontier/goal-following replay behavior in later phases. |
| Physical adapter | `PhysicalAdaptiveMissionExecutor` | Later resolves a validated goal into the Nav2 goal-follower boundary; it never publishes motor commands. |
| Route transport | `RosLiveRouteExecutor` | Remains available for legacy primitive missions. Hierarchical mode uses the same exact-SHA and correlation principles. |
| Requested velocity | `live_route_runner_node.py` | Remains the sole hierarchical-mode `/cmd_vel` publisher, but only as a thin lease/bounds bridge for private Nav2 commands. |
| Final motor arbitration | `collision_stop_node.py` | Remains the sole `/cmd_vel_motor` publisher and independent STOP/ESTOP/collision/stale-command boundary. |
| Persistence and approval | `MissionService` | Persists goal proposals, decisions, prefetches, handoffs, outcomes, authority flags, and restart recovery. |
| Semantic perception | `stationary_perception.py` and `adaptive_mission_perception.launch.py` | Supply timestamped camera evidence and semantic tracks without motion authority. |
| Browser replay | `mission_web.py --mode adaptive-mission-replay` | Shows frontiers, active/prefetched goals, paths, evidence, decisions, and terminal truth. |

The current executor protocol is synchronous and primitive-shaped. Phase 1 and
Phase 3 must evolve it deliberately rather than hide a long-running Nav2 action
inside `execute()` while retaining misleading primitive result semantics.

## Target data and authority flow

```text
slam_toolbox occupancy grid + localized pose
  -> deterministic frontier / coverage / viewpoint candidates
  -> bounded semantic world snapshot
  -> LLM returns one snapshot-bound goal
  -> deterministic goal and evidence validation
  -> deterministic target pose + Nav2 path/controller
  -> private /nav2_cmd_vel_request
  -> live_route_runner hierarchical lease/bounds bridge
  -> /cmd_vel
  -> collision_stop_node
  -> /cmd_vel_motor
  -> sphero_rvr_driver
```

In replay, everything below the goal validator is simulated and all authority
flags remain false. In a future separately approved physical run, Nav2 and its
bridge start only inside the exact-SHA-bound supervised graph.

The private Nav2 command topic is not an authority surface. The bridge accepts
it only while all of the following agree:

- one active persisted goal and generation;
- the approved mission and goal leases;
- exact source, deployed, and reviewed SHAs;
- a fresh command receipt within the existing requested-command lease;
- configured linear and angular ceilings;
- fresh motion-critical evidence and a running supervised graph.

It clamps or rejects malformed/out-of-envelope commands, publishes zero when
the command lease expires, and records requested-versus-forwarded evidence.
It never publishes `/cmd_vel_motor`. Graph tests must prove that Nav2 cannot
publish `/cmd_vel` or `/cmd_vel_motor` directly.

The legacy primitive route path stays intact until a separately reviewed
migration removes it. Hierarchical and legacy modes must not simultaneously
own `/cmd_vel`.

## Goal-level contract

### Model output

The new allowlist is:

```text
go_to_frontier(frontier_id)
inspect(track_id)
search_region(region_id, target_classes)
return_to_start()
wait()
finish(outcome, evidence_ids)
```

Every provider response contains:

- schema version;
- exact `mission_id`;
- exact `snapshot_id`;
- server-issued decision generation;
- one allowlisted action and only its permitted arguments;
- concise rationale referencing candidate or evidence IDs.

The LLM does not emit a pose, route, path, speed, acceleration, footprint,
clearance, lease, retry count, costmap value, ROS name, or code. Deterministic
software resolves stable IDs into geometry.

### Server-owned bounds

Every accepted goal is bound to:

- the approved mission lease and remaining mission budgets;
- a finite goal lease and maximum goal runtime;
- a maximum resolved route length and map bounds;
- minimum deterministic clearance and reachability requirements;
- the source map revision and localization frame;
- a target signature and validation generation;
- a finite event/preemption generation;
- the installed command and acceleration ceilings.

These bounds are configuration-owned and digest-bound. The model cannot
provide, increase, renew, or reinterpret them.

### Goal resolution

- `go_to_frontier` resolves a still-valid frontier signature to a safe approach
  pose selected by L2.
- `inspect` invokes deterministic Next-Best-View generation for the specified
  track and resolves the best reachable safe viewpoint.
- `search_region` selects among deterministic frontiers/viewpoints intersecting
  the named region and requested classes.
- `return_to_start` resolves the persisted mission origin against the current
  map and localization state.
- `wait` commands no movement, preserves the finite mission lease, and awaits a
  defined event or fresh planning evidence.
- `finish` is accepted only when every terminal claim is supported by the
  supplied evidence IDs and recorded coverage limits.

An unresolved, missing, stale, unreachable, or out-of-bounds target is rejected.
The controller never silently substitutes a different target.

## World snapshot

The LLM receives a compact projection rather than a raw occupancy grid or
unrestricted video:

- mission objective, origin, lease, budgets, and requested object classes;
- authoritative pose, frame, covariance, quality, and freshness;
- map ID/revision and summarized coverage;
- frontier candidates with stable signatures, approach region, distance,
  clearance, reachability, information gain, and last validation time;
- semantic tracks with class/identity policy, position method, uncertainty,
  evidence IDs, observation history, and last-seen time;
- deterministic Next-Best-View candidates summarized under their track IDs;
- current goal, path progress, ETA, prior outcome, and prefetch state;
- STOP, ESTOP, collision, cancellation, command-lease, and capability state.

Candidate lists are bounded and deterministically ordered before prompting.
Raw ROS messages, unbounded maps, credentials, file paths, and motor surfaces
are excluded.

## Frontier contract

L2 runs Wavefront Frontier Detection over the `slam_toolbox` occupancy grid.
A frontier is a connected boundary between known free and unknown cells, not
an LLM-created coordinate.

Before exposure to L3, deterministic code:

1. verifies map frame, dimensions, resolution, origin, revision, and age;
2. filters malformed, tiny, occupied, inflated, unreachable, and
   insufficient-clearance frontiers;
3. computes a safe approach region rather than targeting an unknown cell;
4. records distance, estimated path cost, visible reveal area, and coverage
   contribution;
5. assigns a stable signature from quantized connected-cell geometry plus the
   source map identity;
6. tracks suppression, failed approaches, invalidation, and completion.

A map revision alone does not invalidate a frontier when its stable signature
and approach region remain equivalent. A changed or disappeared signature
does.

`frontier_exploration_ros2` is the preferred implementation candidate because
it advertises ROS 2 Jazzy support and exports its frontier/search/order core
separately from its Nav2-dispatching node. Phase 1 must inspect and pin an exact
reviewed revision, confirm its license and transitive dependencies, reproduce
its build/tests, and benchmark its CPU and memory on the Pi. Its published
performance numbers are leads, not acceptance evidence.

Milestone 6 should reuse the core candidate generation, not the package's
automatic frontier dispatch, because L3 and `MissionService` own semantic goal
selection and persistence.

## Next-Best-View contract

`inspect(track_id)` does not let the model invent a camera pose. Deterministic
Next-Best-View generation:

- samples poses around the track's uncertainty region;
- rejects occupied, unknown-without-clearance, unreachable, out-of-map, and
  unsafe-footprint poses;
- requires a valid camera bearing and expected line of sight;
- scores expected uncertainty reduction, view-angle diversity, travel cost,
  localization quality, and mission budget;
- returns a finite ordered candidate set with stable IDs;
- records why candidates were rejected.

If no viewpoint is safe and reachable, the result is truthful `blocked` or
`unreachable`; it is not replaced by an open-loop turn or approach.

## Camera-to-map localization

Geometry remains lidar- and localization-owned. No depth/RGB-D camera is added.

### Plane-crossing objects

For people, furniture, or another object intersecting the lidar plane:

1. pair camera and lidar observations by receipt/source timestamp within a
   configured bound, initially no more than `100 ms`;
2. convert a reviewed image anchor to a camera bearing using measured
   intrinsics and the camera-to-lidar extrinsic;
3. select a contiguous lidar cluster inside a bounded angular gate;
4. reject absent, multi-modal, occluded, or ambiguous associations;
5. transform the accepted range/bearing through the localized sensor pose;
6. propagate bearing, range, extrinsic, synchronization, and pose uncertainty.

### Floor objects

For shoes and other low objects below the lidar plane:

1. use the bottom-center contact anchor of the detection;
2. project through a calibrated floor-plane homography using measured camera
   height, pitch, intrinsics, and camera-to-base transform;
3. reject anchors above the horizon, outside the calibrated image/ground
   region, or paired with stale localization;
4. propagate pixel, calibration, floor-plane, and robot-pose uncertainty.

### Bearing-only fallback

When range or floor projection is invalid, retain a bounded bearing cone. It
may bias frontier or viewpoint value but cannot create a point marker or a
claim that the object was mapped.

Every track records `lidar_range`, `floor_projection`, or `bearing_only`,
source timestamps, calibration identity, map revision, uncertainty, and
evidence IDs. Phase 2 defines range-dependent quantitative tolerances from
recorded ground truth and includes an explicit ambiguous-association rejection
gate.

The Phase 2 replay gate uses a `100 ms` maximum camera/lidar source-time
delta; a recorded-point error model of `0.03 m + 0.04 × range`, capped at
`0.08 m`; a `0.05 m` calibrated analytic floor-geometry bound; and a
point-forbidden ambiguous-association test. These are offline software gates,
not physical accuracy certification.

## Continuous handoff

### Prefetch timing

Only one next-goal prefetch may be active. The controller starts it when:

```text
current ETA <= measured provider p95 latency + configured margin
```

The current measured low-effort Luna p95 is `12.691 s`; it is a baseline
observation, not a permanent constant. MissionService should derive the
threshold from a reviewed rolling latency profile and record the value used.
If a new goal begins already inside the threshold, prefetch starts immediately.

Every prefetch is bound to the current mission, objective revision, snapshot,
map revision, active-goal generation, and event generation. An operator
redirect or higher-priority event invalidates the in-flight result.

### Revalidation

Before a prefetched goal may become active, deterministic code rechecks:

- exact mission/objective/generation binding;
- provider response and planning-snapshot validity;
- target/frontier/track signature and coverage state;
- current map identity/revision compatibility;
- localization and required sensor freshness;
- current reachability, path length, clearance, and budgets;
- STOP, ESTOP, collision, cancellation, leases, and graph authority.

A failed check discards the result and persists the rejection reason. It is
never repaired or silently retargeted.

### Motion-layer handoff

The Nav2 integration must update or extend the active path before the current
frontier becomes a terminal action goal. Nav2 documents that
`NavigateThroughPoses` treats intermediate poses as path constraints and does
not stop or slow merely because it passes them. Its GoalUpdater/PipelineSequence
patterns also permit updated paths to reach `FollowPath` while the controller
continues running.

Phase 1 should use one navigator type for the entire exploration and verify one
of these mechanisms in replay:

- append the validated next pose to the active `NavigateThroughPoses` goal; or
- update the path through a GoalUpdater-based behavior tree without exiting
  `FollowPath`.

Switching navigator types requires explicit cancellation and is not a
zero-pause mechanism. Disabling Nav2's zero-on-goal-exit behavior is also not an
acceptable shortcut; zero remains required whenever a goal actually exits.

Atomic handoff passes only when continuous replay telemetry shows:

- no terminal exit of the motion controller;
- no deliberate zero request between compatible goals;
- one ordered active/prefetched generation;
- a path update bound to the revalidated next goal;
- immediate independent veto when safety changes.

### Late or invalid prefetch

If a prefetch is late while the current goal is still valid, movement toward
the current goal continues. At arrival, the rover safely holds zero in a
non-terminal `wait_planning` state while a fresh decision is obtained.

If a prefetched goal is invalid, it is discarded. The current goal continues
when still valid; otherwise the rover safely holds and replans. Latency or a
planning-invalid result does not by itself create a terminal mission, but it
also never authorizes aimless continuation.

Zero-pause handoff is a performance acceptance target. Safety always wins over
continuity.

## Event-triggered replanning

Events are prioritized:

1. STOP, ESTOP, collision latch, cancellation, lease expiry, and
   motion-critical freshness loss stop deterministically and are terminal.
2. Operator redirection invalidates older provider generations and acquires a
   new snapshot within the existing approved lease.
3. Invalid target, lost progress, blocked path, or loss of map growth stops or
   preempts the current Nav2 goal and replans from fresh evidence.
4. A stable new object/person, materially changed recognition, or material
   uncertainty change may preempt for semantic reconsideration.

Semantic events require deterministic stability, novelty, confidence, and
hysteresis gates. Repeated camera frames or small confidence changes must not
thrash the goal or provider. Multiple lower-priority events are coalesced into
one fresh snapshot and one new generation.

The LLM is never consulted before applying a class-1 safety event.

## Freshness policy

Freshness is role-specific:

| Evidence | Stale behavior |
|---|---|
| Lidar scan, required TF, odometry, or localization used for active movement | Immediate deterministic zero and terminal safety result; no automatic resume. |
| Requested Nav2 command lease | Bridge publishes zero; mission records command-stale terminal behavior. |
| Prefetched planning snapshot or map/frontier binding | Discard the result and obtain fresh planning evidence; not terminal by itself. |
| Camera or semantic map for geometric exploration | Withhold semantic evidence; continue only if the current geometric goal remains valid and its contract permits it. |
| Camera/track evidence required by `inspect` or object search | Invalidate or pause that goal and replan; never claim the observation. |
| Provider response after objective/event generation changed | Discard without execution. |

Restart remains `recovery_required`; no layer resumes an action automatically.

## MissionService and evidence

The persistent record must add:

- goal schema/version, proposal digest, and server-owned bounds;
- world-snapshot, map, frontier, track, and objective revision IDs;
- active and prefetched goal generations;
- provider start/end latency and prefetch threshold;
- validation, discard, preemption, and handoff events;
- resolved deterministic target pose and path identity;
- requested Nav2 and bridged `/cmd_vel` evidence;
- collision-supervisor decision and final `/cmd_vel_motor` ownership;
- coverage changes and goal outcomes;
- terminal reason and evidence-linked claims.

Proposal approval continues to bind the exact deployment, provider/model,
mission lease, safety policy, and authority profile. Later LLM goal revisions
remain inside that approved envelope. A changed prompt outside the existing
objective rules, a changed authority profile, an expired lease, a restart, or a
different deployed SHA requires a new proposal/approval.

## Safety invariants

The Milestone 6 brief's invariants remain authoritative and are reproduced
verbatim:

- `collision_stop_node` stays the sole `/cmd_vel_motor` publisher and an
  independent stop path. Nav2 / any costmap is planning-time avoidance, never a
  safety layer.
- No new motion authority by default: `RVR_ADAPTIVE_MISSION_ENABLED`,
  `RVR_LIVE_EXECUTION_ENABLED`, and the reviewed SHA stay false/blank; the
  replay executor is default.
- The LLM emits only typed, schema-validated, bounded, snapshot-bound goals — no
  routes, ROS topics, Twists, speeds, safety thresholds, shell actions,
  credentials, or model-generated code. Keep the Codex `--output-schema` path;
  strip `OPENAI_API_KEY`/`CODEX_API_KEY`; require ChatGPT OAuth.
- Freshness is split by role: stale **motion-critical** lidar/localization stays
  terminal (deterministic stop), independent of any in-flight model call or
  prefetch; a stale prefetched **planning** snapshot is normally discarded and
  replanned, not terminal. Lease expiry, collision, STOP, ESTOP, cancellation,
  and mission-lease expiry remain terminal.
- Digest-bound approval, `MissionService` persistence, and restart →
  `recovery_required` are preserved.

The selected architecture adds these consequences without weakening those
invariants:

- In hierarchical mode, Nav2 publishes only to a private request topic;
  `live_route_runner_node` is the sole `/cmd_vel` publisher and applies
  server-owned leases and caps.
- `RVR_ADAPTIVE_MISSION_ENABLED=false`,
  `RVR_LIVE_EXECUTION_ENABLED=false`,
  `RVR_STATIONARY_PERCEPTION_ENABLED=false`, and
  `RVR_APPROVAL_ACTIVATION_ENABLED=false` remain installed defaults; the
  reviewed SHA remains blank.
- The replay executor remains default and reports
  `motion_authority=false` and `physical_execution_enabled=false`.
- Drop-off detection remains unavailable. Physical exploration is restricted
  to attended, level, bounded rooms with no stairs, ledges, or open drop-offs
  until independent negative-obstacle sensing exists.
- No physical hierarchical exploration occurs until the existing attended
  collision and moving-perception gates pass and a separate exact-SHA
  execution request is explicitly approved.

## Phase 1 acceptance implications

Phase 1 remains replay/simulation only and contains no LLM schema change. It
must prove:

- deterministic WFD candidates from a real recorded `slam_toolbox` occupancy
  grid;
- stable frontier signatures, filtering, invalidation, and exhaustion;
- continuous Nav2 travel through at least two compatible frontier goals;
- no deliberate zero between those goals;
- a late/invalid next-goal safe-hold path;
- immediate supervisor veto during motion;
- one private Nav2 command source, one `/cmd_vel` bridge publisher, and one
  `/cmd_vel_motor` supervisor publisher;
- all authority flags false;
- bounded verbose test execution and complete cleanup.

ROS-free frontier/goal contract tests run on the development host. Nav2
integration runs in a ROS 2 Jazzy replay/simulation environment, with no rover
driver, serial owner, or physical command graph. Pi CPU and memory measurements
are no-motion evidence only.

Human review approved the Nav2 decision before Phase 1. Phase 1 results and
residuals are recorded in the linked evidence report.

## References

Primary implementation and platform references:

- [Nav2 Smac Planner](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)
- [Nav2 Regulated Pure Pursuit](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)
- [Nav2 DWB Controller](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)
- [Nav2 MPPI Controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
- [Nav2 Simple Commander API and preemption notes](https://docs.nav2.org/commander_api/index.html)
- [Nav2 continuous NavigateThroughPoses behavior](https://docs.nav2.org/migration/Foxy.html#navigatethroughposes-and-computepaththroughposes-actions-added)
- [Nav2 Follow Dynamic Point goal-update behavior tree](https://docs.nav2.org/behavior_trees/trees/follow_point.html)
- [frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2)
