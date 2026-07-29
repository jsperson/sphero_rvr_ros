# Milestone 7 Phase 4 physical hierarchical binding

## Result and scope

This slice installs the M7.5 live binding while keeping it physically inactive.
It connects the replay-proven semantic-goal contract to a live-time Nav2 action
adapter and connects Nav2's private velocity request to the existing supervised
command chain. It does not grant M7.6 execution approval and does not run the
canonical mission.

The installed chain is:

```text
server-owned semantic snapshot
  -> toolless semantic-ID model response
  -> async M6 controller (prefetch + event replan)
  -> deterministic resolver and current-snapshot revalidation
  -> /navigate_through_poses
  -> Nav2 controller/behavior servers
  -> /nav2_cmd_vel_request
  -> live_route_runner (sole /cmd_vel publisher)
  -> lidar_collision_stop_supervisor (sole /cmd_vel_motor publisher)
  -> rvr_node
```

The semantic/Nav2 adapter and authority owner publish no `Twist`, do not open
serial transport, and never publish to `/cmd_vel` or `/cmd_vel_motor`.
The live hierarchical mission controller has the same restriction. It consumes
the live trinary `/map`, localization and semantic-map evidence, runs
deterministic WFD and Next-Best-View generation, and is the sole publisher of
the semantic dispatch topic. The moving semantic-perception node is part of
the same all-or-nothing default-off group.

## Default-off physical surface

`hierarchical_exploration_physical.launch.py` is separate from the replay
launch. Its five groups all default to false:

- sensors;
- motion stack;
- live-time Nav2;
- exact-SHA authority owner;
- semantic-goal adapter.

An inconsistent request cannot briefly start the driver. The nested mapping
launch receives `start_rvr:=true` only when every required group, including
sensors and independent safety, is true.

`rvr-hierarchical-mission.service` has no `[Install]` section, no boot target,
and `Restart=no`. Its startup precondition requires:

- the reviewed binding-installed flag;
- a separate `RVR_HIERARCHICAL_M7_6_APPROVED=true`;
- exact equality of source, deployed, and reviewed SHAs;
- a non-empty M7.6 approval file.
- a non-empty browser-authored semantic proposal file whose digest is bound by
  that approval.

Installing this unit does not start or enable it.

## Authority contract

The physical authority owner is locked by default. Activation validates a
canonical SHA-256 approval envelope containing:

- exact source, deployed, and reviewed Git SHAs;
- mission, operator, proposal digest, approval ID, approval digest, and lease;
- the accepted M7.3 evidence digest;
- the accepted paired directional-veto digest;
- the accepted M7.4 moving-perception session digest;
- an attended, level, bounded room declaration with stairs, ledges, and
  drop-offs absent;
- fixed `0.10 m/s`, `0.4 rad/s`, `0.50 s` command lease, `0.500 s`
  localization freshness, and `900 s` maximum mission lease.

The owner emits a 10 Hz heartbeat. `live_route_runner` validates the
heartbeat independently on every control tick with a bounded `0.750 s`
receipt-age limit. Missing, malformed, stale, expired, or SHA-mismatched
authority makes `mission_lease_valid=false` and therefore publishes zero.
Collision evidence remains bounded at `0.300 s` and the private Nav2 command
lease remains `0.500 s`.

The journal is append-only SQLite evidence. If the last durable event says
authority was active and no relock event exists, a new owner process enters
`recovery_required`. It cannot reactivate that mission. Graceful cancellation,
lease expiry, and shutdown append a relock event.

## Semantic-goal and Nav2 boundary

The live adapter accepts only a digest-bound dispatch from the Pi-owned
hierarchical mission controller. Each item contains the original strict semantic-ID response, its
captured snapshot, and a current snapshot. The adapter:

1. validates the model response against the existing M6 strict schema;
2. rejects extra geometry, route, speed, lease, safety, ROS, or code fields;
3. resolves geometry with `DeterministicGoalResolver`;
4. revalidates the result against current map, frontier/track identity,
   objective/event generations, safety, route budget, and evidence;
5. applies the fixed physical `0.300 s` localization freshness gate;
6. sends one to three ordered poses through `NavigateThroughPoses`.

The model cannot provide a pose or velocity. Updated compatible batches use
Nav2 goal preemption/GoalUpdater behavior; late successors remain an explicit
`wait_planning` state rather than stale or speculative motion.

The physical proposal schema contains only mission ID, objective text,
objective revision, requested semantic classes, exact source SHA, and creation
time. Any geometry, route, speed, safety, lease, ROS, or code field is rejected.
The live controller runs the replay-proven asynchronous successor logic and
converts stable new detections or invalidated active frontiers into the existing
event-triggered replan contract. Nav2 feedback supplies remaining distance for
the p95-plus-margin prefetch threshold.

## Durable and browser evidence

The authority journal records activation and relock events with canonical
payload digests. The live controller records controller events and every
semantic dispatch in the same append-only journal. Goal batches have canonical
digests and retain semantic
generation, target ID/signature, map ID/revision, source SHA, approval digest,
controller session, and reason.

The Pi-local MissionService exposes the installed binding to the browser as a
read-only projection. In this slice it truthfully reports:

- `installed=true`;
- `state=locked`;
- `m7_6_execution_approved=false`;
- `canonical_mission_approved=false`;
- `motion_authority=false`;
- `physical_execution_enabled=false`;
- `restart_resume_allowed=false`.

The browser still has no ROS or direct command route.

## Preserved safety properties

- `0.10 m/s` linear and `0.4 rad/s` angular ceilings are unchanged.
- The physical Nav2 request lease is fixed at `0.50 s`; the downstream driver
  watchdog remains an independent backstop.
- Collision, STOP, ESTOP, cancellation, stale motion evidence, stale authority,
  and expired mission lease all fail closed independently of provider
  inference.
- Bearing-only observations never create mapped points.
- Drop-off sensing is unavailable. Physical use remains attended and restricted
  to a level bounded room without stairs, ledges, or open drop-offs.
- Restart never resumes a route.
- Camera evidence retention remains bounded to 96 JPEG thumbnails of at most
  512,000 bytes each; the live session does not run rosbag.

## Carried physical risks

M7.5 does not erase the M7.4 findings:

- localization reached `0.297610 s` against the fixed `0.300 s` gate; the new
  binding explicitly accepts that sample and rejects `0.301 s`;
- the accepted moving-perception proof covered only `0.051178355 m` at
  `0.05 m/s`, with low-speed ridge/stall attempts;
- camera pitch values were useful for the accepted residual checks but the
  centering method was not survey-grade, so no tolerance is tightened;
- real provider calls remain roughly `8–15 s`; compatible long legs can hand
  off continuously, while short hops may still visibly hold
  `wait_planning`.

These are Phase 5 physical observations, not reasons to relax a gate.

## Validation

Focused validation must use the repository's bounded verbose runner:

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_hierarchical_physical_binding.py \
  tests/test_hierarchical_exploration.py \
  tests/test_hierarchical_goal_selection.py
```

The full suite must use:

```bash
python3 scripts/run_pytest_bounded.py --timeout 90 -- -vv
```

The Pi validation for the exact candidate is no-motion only: build/install,
inspect launch defaults and systemd enablement, inspect nodes/topics/actions,
confirm no `rvr_node`, serial owner, driver, supervisor, route runner, Nav2
server, lidar, camera, rosbag, or motion publisher is active, and retain
cleanup evidence. Chassis activation, sensors, the physical launch, M7.6
approval, and the canonical mission are prohibited in this slice.

### Recorded candidate

Executable candidate `5b7c96f52b5b21a59655d2d9c102d0d3fee2f4cc`
was built on the aarch64 Pi in 2.17 seconds. The exact bounded focused command
passed 45/45 in 2.38 seconds. The final local full suite passed 1082/1082 in
39.42 seconds.

The Pi graph audit observed only `/live_mission_service` and the static
`/base_to_laser_static_tf` node. `/cmd_vel`, `/cmd_vel_motor`, the private Nav2
request, hierarchical authority/dispatch, `/scan`, and camera topics were
absent. No action server, hardware Python process, serial-device owner, or
recent camera/rosbag file was present. The hierarchical motor-capable unit was
inactive and not installed as an enableable user unit.

The first combined Pi suite exposed a one-ULP Darwin/Linux libm difference in a
reconstructed NBV clearance used by one historical OAuth-smoke test. That made
the reconstructed snapshot digest differ even though the committed decision
still carried its original exact captured ID. The corrected regression
validates that committed captured ID directly and separately proves a different
ID is rejected. No historical evidence, live validation rule, tolerance, or
runtime code was changed. Full commands and cleanup are retained in
`artifacts/m7_phase4_physical_binding/`.

## Next gate

Independent review may accept M7.5 as installed and locked. Phase 5 must then
create a new goal, bind an attended canonical mission to a fresh exact-SHA
M7.6 approval, run it once, persist/reopen the result by mission ID, and prove
terminal cleanup. M7.5 approval is not M7.6 approval.
