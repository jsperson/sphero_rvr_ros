# Milestone 7 Phase 1: exact-SHA Pi no-motion gate

## Outcome

M7.1 passes for executable source
`9822ec6fe8c903191329ebdbb2646cac745e25ad`, built from an immutable Git
archive on `sphero-pi-2`. The rover remained powered off. This phase added
audit tooling and exercised recorded-map WFD plus a loopback-only Nav2 graph;
it did not add or enable a physical adapter, driver, serial transport, live
sensor launch, or motor authority.

The durable reports are in
[`artifacts/m7_phase1_pi_no_motion`](../artifacts/m7_phase1_pi_no_motion/README.md).

## WFD evidence

The ROS-free audit ran the project-owned deterministic WFD 50 times against
the recorded Phase 1 SLAM map:

| Measurement | Result |
| --- | --- |
| Passes | 50 |
| Deterministic | yes |
| Frontiers per pass | 13 |
| Mean | 26.594 ms |
| p95 | 26.817 ms |
| Maximum | 27.509 ms |
| Maximum RSS | 31,816 KiB |
| Map image SHA-256 | `c05ae6ec457d5ce389a4278e055fcf8cce1a77752de2a05244c0f43ae55e1d76` |
| Map YAML SHA-256 | `87d4e71f7166c642eb07f3756656d65af5ac98a44a0e7538a9bb2538b9f586fa` |

All 13 ordered frontier signatures were identical across all 50 passes. The
full samples and signatures are retained in `wfd.json`.

## Ownership graph evidence

Only the replay/simulation groups were explicitly enabled in an isolated ROS
domain. The launch remains default-off. The resulting chain was:

```text
controller_server + behavior_server
  -> /nav2_cmd_vel_request
  -> live_route_runner
  -> /cmd_vel
  -> lidar_collision_stop_supervisor
  -> /cmd_vel_motor
  -> loopback_simulator
```

The allowed private-topic publisher node set was exactly
`controller_server` plus `behavior_server`. Nav2 behavior plugins expose
multiple publisher endpoints under the same behavior-server node, which is
why the raw endpoint list contains repeated node names.

The audit confirmed:

- `map_server`, `planner_server`, `controller_server`, `behavior_server`, and
  `bt_navigator` were active;
- `NavigateThroughPoses` was ready;
- `live_route_runner` was the sole `/cmd_vel` publisher;
- `lidar_collision_stop_supervisor` was the sole `/cmd_vel_motor` publisher;
- the loopback simulator was the sole motor-topic subscriber and no hardware
  sink was present;
- 183 downstream motor samples were observed and all 183 were zero;
- no `rvr_node`, lidar, camera, SLAM, stationary-perception, or other
  prohibited executable was running in the audit session; and
- `/dev/ttyAMA0` and `/dev/ttyUSB0` had no owners.

The graph validator's `audit` mode never sends a navigation goal. It observes
readiness and zero output only. Both transform startup bounds were raised from
5 to 15 seconds, and lifecycle activation now waits eight seconds for the
loopback `/initialpose` and `map -> odom` transform after cold Pi DDS
discovery. Command leases and speed caps were not changed. The private bridge
lease remains 0.25 seconds.

## Reproduction

The executable SHA was deployed as an archive with SHA-256
`73de13807ac6c6ac1ac71305e70ec7cbee854cfd96cbcebabe9f50fed71ad49d`.
After a package-select build, the two evidence commands were equivalent to:

```bash
ros2 run sphero_rvr_driver rvr_hierarchical_m7_phase1_audit \
  artifacts/phase1_recorded_slam_map/phase1_recorded_slam_map.yaml \
  --source-sha 9822ec6fe8c903191329ebdbb2646cac745e25ad \
  --repetitions 50 --output wfd.json

ROS_DOMAIN_ID=77 ros2 launch sphero_rvr_driver \
  hierarchical_exploration_replay.launch.py \
  start_nav2:=true start_bridge:=true start_supervisor:=true \
  start_loopback:=true

ROS_DOMAIN_ID=77 ros2 run sphero_rvr_driver \
  rvr_hierarchical_nav2_replay_validate --mode audit \
  --source-sha 9822ec6fe8c903191329ebdbb2646cac745e25ad \
  --observe-seconds 5 > graph.json
```

The launch was externally bounded to 45 seconds. Its GNU `timeout` exit `124`
means the deliberate observation window expired; it is not a bounded-pytest
verdict. The report completed before the bound, all descendants were reaped,
temporary workspaces were removed, services capable of live perception were
confirmed inactive, and both serial candidates were confirmed ownerless.

## Gate boundary

This evidence closes only M7.1. M7.2 still requires separately approved,
stationary, surveyed localization evidence with no driver. It does not inherit
permission to start the camera, lidar, SLAM, serial transport, or rover.
Motor-capable work remains behind later exact-SHA human approvals.
