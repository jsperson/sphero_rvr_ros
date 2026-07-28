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
- fixed `0.10 m/s`, `0.4 rad/s`, `0.50 s` command lease, `0.300 s`
  localization freshness, and `900 s` maximum mission lease.

The owner emits a short-lived heartbeat. `live_route_runner` validates the
heartbeat independently on every control tick. Missing, malformed, stale,
expired, or SHA-mismatched authority makes `mission_lease_valid=false` and
therefore publishes zero.

The journal is append-only SQLite evidence. If the last durable event says
authority was active and no relock event exists, a new owner process enters
`recovery_required`. It cannot reactivate that mission. Graceful cancellation,
lease expiry, and shutdown append a relock event.

## Semantic-goal and Nav2 boundary

The live adapter accepts only a digest-bound dispatch from the Pi mission
service. Each item contains the original strict semantic-ID response, its
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

## Durable and browser evidence

The authority journal records activation and relock events with canonical
payload digests. Goal batches have canonical digests and retain semantic
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

## Next gate

Independent review may accept M7.5 as installed and locked. Phase 5 must then
create a new goal, bind an attended canonical mission to a fresh exact-SHA
M7.6 approval, run it once, persist/reopen the result by mission ID, and prove
terminal cleanup. M7.5 approval is not M7.6 approval.
