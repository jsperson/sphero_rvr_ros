# Architecture Map

Updated: 2026-07-31

This is the single maintained map of current module, process, topic, and
authority ownership. Read it immediately after the canonical Obsidian status
note at `Projects/Sphero RVR ROS/Current Status.md`. Phase and design documents
record decisions and evidence; they are not competing architecture maps.

## Current seams and owners

| Responsibility | Current implementation | Runtime surface | Owns / must not own |
|---|---|---|---|
| Browser proposal and observation | `mission_web.py`: `LiveMissionWebAdapter`; executable `rvr_mission_web` | Mission Service Unix socket and HTTP UI | Owns proposal, authenticated approval/cancel, and no-contact observation. No ROS, `Twist`, serial, geometry, or motor surface. |
| Persistent mission state and API boundary | `mission_service.py`: `MissionService`; `live_mission_service_node.py`: node `/live_mission_service`, executable `live_mission_service` | `~/.local/state/sphero_rvr/missions.sqlite3`, Pi-local Unix socket, `/mission_api/v2/*` status surfaces | Owns durable proposals, events, results, approval records, and restart recovery. Does not publish motion commands. |
| Single-approval physical session | `hierarchical_physical_live_controller.py`: `HierarchicalPhysicalMissionController`; `hierarchical_physical_session.py`: `SystemdHierarchicalMissionSession` | `rvr-hierarchical-mission.service` (`Type=simple`, `Restart=no`) | Consumes one exact-SHA/digest-bound approval, activates the default-off graph, then relocks and cleans up. Restart never resumes. |
| Physical authority heartbeat | `hierarchical_authority_node.py`: node `hierarchical_physical_authority`, executable `hierarchical_physical_authority` | `/mission_api/v2/hierarchical/authority` | Owns the bounded authority heartbeat only. It cannot emit a goal or velocity. |
| Live sensors and localization | `hierarchical_exploration_physical.launch.py` includes `mapping.launch.py`; RPLidar, Camera 3, `slam_toolbox`, static TF | `/scan`, `/camera_node/image_raw`, `/camera_node/camera_info`, `/map`, `/odom`, TF | Own raw sensing, occupancy, and localization evidence. Sensors do not grant motion authority. |
| Semantic perception | `stationary_perception_node.py`: `StationaryPerceptionNode`, launched as node `hierarchical_semantic_perception`, executable `stationary_perception` | `/mission_api/v2/camera/status`, `/mission_api/v2/lidar/status`, `/mission_api/v2/localization/status`, `/mission_api/v2/map/status` | Owns deterministic detection, tracking, camera/lidar localization, and bounded evidence images. No motion publisher. |
| Frontier and viewpoint generation | `hierarchical_exploration.py`: WFD/frontier registry and deterministic target data | Runs inside `hierarchical_mission_controller` from `/map` and fresh localization/perception | Owns candidate IDs, geometry, reachability, clearance, coverage, and invalidation. The model never owns these. |
| Semantic goal selection | `hierarchical_goal_selection.py`: `CodexOAuthSemanticGoalProvider`, `AsyncSemanticGoalController` | Warm `gpt-5.6-luna` OAuth provider inside `hierarchical_mission_controller` | Returns bounded snapshot-bound actions and semantic IDs with rationale. No geometry, paths, ROS, velocity, or safety parameters. |
| Hierarchical orchestration | `hierarchical_mission_node.py`: node/executable `hierarchical_mission_controller` | Publishes `/mission_api/v2/hierarchical/goal_dispatch` and `/mission_api/v2/hierarchical/controller_status`; consumes authority, map, perception, collision, `/plan`, and adapter status | Owns snapshots, async provider calls, revalidation, prefetch, event replanning, wait/finish truth, and durable checkpoints. No `Twist` publisher. |
| Semantic goal to Nav2 | `hierarchical_nav2_adapter_node.py`: node/executable `hierarchical_nav2_adapter` | `/mission_api/v2/hierarchical/goal_dispatch` -> `/navigate_through_poses`; status `/mission_api/v2/hierarchical/status` | Resolves already server-owned goal batches into `NavigateThroughPoses`. No `Twist` or direct motor publisher. |
| Global planning and path following | `hierarchical_nav2_physical.yaml`: `nav2_smac_planner::SmacPlanner2D` and `dwb_core::DWBLocalPlanner`; Nav2 planner/controller/behavior/BT nodes | `/navigate_through_poses`, `/plan`, private `/nav2_cmd_vel_request` | Owns paths and requested local velocity only. Nav2 may not publish `/cmd_vel` or `/cmd_vel_motor`. |
| Private velocity bridge | `live_route_runner_node.py`: node/executable `live_route_runner` | `/nav2_cmd_vel_request` -> `/cmd_vel`; also consumes authority, scan, odom, encoders, collision and STOP/ESTOP state | Sole hierarchical `/cmd_vel` publisher. Owns receipt-time lease, caps, drivetrain breakaway/escape adaptation, and zero on stale/cancel/lost authority. It never publishes `/cmd_vel_motor`. |
| Independent collision and final arbitration | `collision_stop_node.py`: `LidarCollisionStopSupervisorNode`, node `lidar_collision_stop_supervisor`, executable `lidar_collision_stop_supervisor` | `/cmd_vel` + `/scan` + STOP/ESTOP -> `/cmd_vel_motor` | Sole `/cmd_vel_motor` publisher. Owns slow/stop, directional escape veto, stale-command zeroing, STOP, ESTOP, and reset rules independently of model/Nav2. |
| Hardware transport and odometry | `rvr_node.py`: `SpheroRVRNode`, node `sphero_rvr_driver`, executable `rvr_node`; supervised launch remaps its `cmd_vel` input | `/cmd_vel_motor` -> `/dev/ttyAMA0`; publishes `/odom`, `/encoder_counts`, TF, battery and diagnostics | Sole rover UART owner and motor transport. It does not decide missions, paths, or collision policy. |
| Durable physical evaluation | `hierarchical_physical_binding.py`: schemas/digests/journal; `hierarchical_m7_canonical_validation.py`: executable `rvr_hierarchical_m7_canonical_validate` | Mission DB, binding journal, provider-time snapshots, paths, checkpoints, cleanup capture | Recomputes M7 evidence; it cannot create authority or accept hand-entered pass claims. |

## Canonical physical flow

```text
browser proposal + authenticated approval
  -> MissionService + one-shot systemd session
  -> hierarchical_physical_authority heartbeat
  -> lidar/camera + slam_toolbox + deterministic WFD/perception
  -> hierarchical_mission_controller
  -> bounded LLM semantic decision (IDs only)
  -> server revalidation and /mission_api/v2/hierarchical/goal_dispatch
  -> hierarchical_nav2_adapter
  -> /navigate_through_poses
  -> SmacPlanner2D + DWB
  -> private /nav2_cmd_vel_request
  -> live_route_runner (sole /cmd_vel publisher)
  -> lidar_collision_stop_supervisor (sole /cmd_vel_motor publisher)
  -> sphero_rvr_driver -> UART -> rover
```

## Fixed current names and bounds

| Contract | Current value |
|---|---|
| Physical launch | `launch/hierarchical_exploration_physical.launch.py` |
| Authority topic | `/mission_api/v2/hierarchical/authority` |
| Semantic dispatch | `/mission_api/v2/hierarchical/goal_dispatch` |
| Controller status | `/mission_api/v2/hierarchical/controller_status` |
| Nav2 action | `/navigate_through_poses` |
| Private Nav2 velocity request | `/nav2_cmd_vel_request` |
| Supervisor request | `/cmd_vel` |
| Final motor command | `/cmd_vel_motor` |
| Linear/angular ceilings | `0.10 m/s` / `0.4 rad/s` |
| Command lease | `0.50 s` |
| Authority heartbeat maximum age | `0.75 s` |
| Planning localization maximum age | `0.50 s` |
| Motion-critical collision/scan receipt age | `0.30 s` |
| Maximum mission lease | `900 s` |
| Active perception image retention | 96 JPEGs, each at most 512,000 bytes |

All five physical launch groups default false: `start_sensors`,
`start_motion_stack`, `start_nav2`, `start_authority`, and
`start_semantic_adapter`. The physical systemd unit is single-session,
`Restart=no`, and has no `[Install]` section, so it cannot be enabled at boot.
Drop-off sensing remains unavailable; physical runs require an attended level
bounded room without stairs, ledges, or open drop-offs.

## Legacy and non-canonical paths

| Path | Current role |
|---|---|
| `adaptive_mission_controller.py`, `AdaptiveMissionExecutor`, `PhysicalAdaptiveMissionExecutor` | Earlier primitive/adaptive mission path; not the canonical M7 hierarchical physical loop. |
| `RosLiveRouteExecutor` and `/mission_api/v2/live_route/request` | Legacy bounded primitive-route transport; hierarchical mode disables this input and consumes the private Nav2 request instead. |
| `hierarchical_exploration_replay.launch.py` and replay validators | ROS-free/no-hardware acceptance and regression seams; never evidence of physical authority. |
| `range_motion_controller` and manual TUI commands | Manual/primitive control and calibration surfaces; not hierarchical goal selection. |
| Phase documents | Historical design, gate, and evidence records. They must link here rather than maintain another current seam table. |

## Maintenance rule

Any change to an owner, executable, node, topic, action, command chain, fixed
bound, or default-off gate must update this file in the same PR. `STATUS.md`
must remain only a pointer, and the Obsidian `Current Status.md` remains the
only canonical statement of current milestone, work package, and next action.
