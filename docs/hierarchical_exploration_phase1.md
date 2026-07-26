# Milestone 6 Phase 1 replay evidence

## Verdict and scope

Phase 1 implements the replay-only deterministic frontier and Nav2 layer
approved in Phase 0. It remains default-off and grants no physical authority.
It does not change the LLM schema, start live sensors, deploy to the Pi
workspace, access serial, or move the rover.

The implementation is based on repository baseline
`7b948c766384df36dcb8a9ea950c9297e879486b`. The pull request and final
validation handoff identify the exact candidate SHA.

## Delivered behavior

- `hierarchical_exploration.py` loads trinary P2/P5 `slam_toolbox` maps,
  detects reachable wavefront frontiers, assigns stable content signatures,
  filters approach cells by clearance, and tracks
  `active/visited/suppressed/failed/invalidated` state.
- `HierarchicalReplayAdaptiveMissionExecutor` extends the existing replay seam
  without motion authority.
- `ContinuousGoalFollowerReplay` models a bounded two- or three-goal queue.
  A ready and valid long-leg prefetch extends the same controller session;
  a short hop with no ready successor enters an explicit zero
  `wait_planning` hold and starts a new session only after planning completes.
- Invalid planning snapshots are discarded and replanned. Collision, STOP,
  ESTOP, cancellation, stale motion evidence, and command-lease loss force
  zero.
- Nav2 uses `SmacPlanner2D`, a circular `0.22 m` footprint, DWB capped at
  `0.10 m/s` and `0.4 rad/s`, and a three-pose
  `NavigateThroughPoses` tree. The tree retains one `FollowPath` session over
  the short acceptance route.
- Nav2 and recovery behaviors publish only to
  `/nav2_cmd_vel_request`; `live_route_runner` is the sole `/cmd_vel`
  publisher; `collision_stop_node` remains the sole `/cmd_vel_motor`
  publisher.
- All four launch groups are independently default `false`:
  `start_nav2`, `start_bridge`, `start_supervisor`, and `start_loopback`.

The pinned `frontier_exploration_ros2` revision remains a reviewed workspace
dependency candidate. Its dispatcher is not started because it would bypass
the project-owned frontier registry, MissionService boundary, and command
ownership graph. Phase 1 uses a small ROS-free project adapter with equivalent
WFD boundary semantics.

## Recorded input

The acceptance map is historical Pi output, not a Phase 1 mapping run:

- source:
  `/home/jsperson/maps/kanban_full_mapping_smoke_clean_retry_20260626_140702_tui2.pgm`;
- PGM SHA-256:
  `c05ae6ec457d5ce389a4278e055fcf8cce1a77752de2a05244c0f43ae55e1d76`;
- dimensions/resolution: `70 x 71`, `0.05 m/cell`;
- trinary counts: 177 occupied, 2,759 unknown, and 2,034 free cells.

The committed manifest records both source checksums and the false authority
flags. At the accepted robot seed, WFD deterministically returns 13 frontiers
with `minimum_frontier_cells=3` and `minimum_clearance_m=0.10`.

## ROS 2 Jazzy replay acceptance

The macOS host does not provide ROS 2 Jazzy, so the ROS graph was validated in
the disposable arm64 public ROS image
`public.ecr.aws/docker/library/ros:jazzy-ros-base`. The project source remained
in the normal repository; the container had a read-only bind mount and no
device mapping.

Launch command:

```bash
ros2 launch sphero_rvr_driver hierarchical_exploration_replay.launch.py \
  start_nav2:=true \
  start_bridge:=true \
  start_supervisor:=true \
  start_loopback:=true
```

Handoff validator:

```bash
ros2 run sphero_rvr_driver rvr_hierarchical_nav2_replay_validate \
  --mode handoff --timeout 90
```

Accepted result:

- one three-pose `NavigateThroughPoses` goal, status `SUCCEEDED`, error 0;
- final goal error `0.061626 m`;
- intermediate-route minimum distances `0.028122 m` and `0.092535 m`;
- 111 and 75 motor samples in the two handoff windows;
- no sustained planning hold: maximum zero intervals were `0.135545 s` and
  `0.0 s`, both within the explicit `0.15 s` non-hold threshold;
- maximum commands were exactly `0.10 m/s` and `0.4 rad/s`;
- no recovery occurred in the corresponding accepted run;
- graph ownership matched the private bridge and supervisor chain;
- `rvr_node` was absent and all authority flags were false.

Supervisor-veto validator:

```bash
ros2 run sphero_rvr_driver rvr_hierarchical_nav2_replay_validate \
  --mode veto --timeout 20
```

The independent `/stop` boundary produced a downstream zero in `0.002272 s`
while motion was active. The action was then cancelled. No driver service,
serial owner, or physical node existed.

The Jazzy `nav2_loopback_sim` 1.0.0 package has two replay defects covered by
the narrow `nav2_loopback_compat.py` adapter: an invalid all-zero initial odom
quaternion, and a constructor-time map request that receives a zero-resolution
grid before lifecycle activation. Global planning still uses the recorded map;
the simulator publishes deterministic max-range scans solely for the local
replay/supervisor stream.

## Pi no-motion measurement

No code was deployed into the Pi workspace. A temporary directory under
`/tmp` received the ROS-free Python package and recorded map, ran 50 identical
WFD passes, and was removed.

- host: `sphero-pi-2`, aarch64, Python 3.12.3;
- 50 passes: `1.347937 s`;
- mean: `26.958748 ms/pass`;
- maximum resident set: `31,504 KiB`;
- deterministic result: 13 frontiers for every pass;
- motion authority and live sensors: false;
- cleanup:
  `CLEANUP_OK=/tmp/sphero-phase1-bench.gkihma`.

## Dependency verification and residuals

`frontier_exploration_ros2` is pinned to
`ec530d2a813739cd25dd0c438d2365c510b9fad8`. It built successfully in the
Jazzy arm64 container. Six upstream test executables completed 118 tests with
zero failures. The seventh executable,
`test_control_service_and_idle`, exceeded its own 60-second CTest timeout
while entering `ControlServiceCanBeDisabledWhenAutostartIsTrue`; CTest
therefore reported a missing result plus one timeout failure. This is recorded
as an upstream ROS-node test-infrastructure residual, not as a pass and not as
a failure of the project-owned WFD implementation. The upstream dispatcher is
not enabled at runtime.

The Phase 0 latency residual remains real. A depth-three queue reduces exposed
planning latency, but it cannot make a model with approximately `12.7 s` p95
latency disappear in every small-room hop. Long legs use the continuous
handoff path; short legs truthfully enter `wait_planning` when their next goal
is not ready. Reducing model latency and preferring fewer, longer validated
legs remain parallel follow-up work.

The replay validates positive-obstacle handling only. Drop-off detection is
still unavailable, so no physical hierarchical exploration is approved.

## Test policy

Repository tests must use the bounded verbose runner from `AGENTS.md`. The
final pull-request handoff records the exact candidate SHA, commands,
durations, results, process cleanup, and disposable-container cleanup.
