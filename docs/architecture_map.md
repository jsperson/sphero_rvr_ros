# Architecture Map

Updated: 2026-08-06

The single maintained map of what actually runs: modules, topics, and who owns
what. Read it after the canonical status note at
`Projects/Sphero RVR ROS/Current Status.md` (Obsidian).

> Rewritten 2026-08-06. The previous version described the pre-cull "cathedral"
> (mission web/service, hierarchical controllers, authority heartbeats, systemd
> one-shot sessions). **All of that was removed in `f194365`** — none of those
> modules exist. If you are looking for `mission_service`, `hierarchical_*`, or
> `stationary_perception`, they are gone; the vault's Milestone 6/7/8, MVP0, Web
> Interface and Pi Mission Stack docs describe that dead era.

## The spine

```text
        /cmd_vel  (whoever is driving: Nav2, teleop, decisive controller)
             |
             v
  lidar_collision_stop_supervisor      <- /scan  +  /camera/low_obstacles
   (SOLE /cmd_vel_motor publisher, final speed gate)
             |
             v
        /cmd_vel_motor
             |
             v
        rvr_node  ->  /dev/ttyAMA0  ->  rover
        (SOLE UART owner; publishes /odom, TF, diagnostics)
```

## Seams and owners

| Responsibility | Module / executable | Surface | Owns / must not own |
|---|---|---|---|
| Hardware transport + odometry | `rvr_node.py` → `rvr_node` | `/cmd_vel_motor` → UART; publishes `/odom`, `/encoder_counts`, TF, battery, `/diagnostics` | Sole rover UART owner. Decides no policy. |
| Safety arbitration + final speed gate | `collision_stop_node.py` → `lidar_collision_stop_supervisor` | `/cmd_vel` + `/scan` + `/camera/low_obstacles` → `/cmd_vel_motor`; `/collision_stop/state`, `stop`/`estop`/`clear_estop` | **Sole `/cmd_vel_motor` publisher.** Owns slow/stop, STOP/ESTOP, stale-command zeroing, and `max_forward_mps` — which clamps EVERY command. |
| Path following (opt-in) | `decisive_controller_node.py` → `decisive_controller` | Nav2 `FollowPath` action server; replaces `controller_server` when `use_decisive_controller:=true` | Drives straight in the deadband, arcs, pivots only for large errors; back-off reflex; preempts stale goals. Needs `navigate_to_pose_decisive.xml` (no local costmap in this mode). |
| Coverage + frontier exploration | `coverage_explorer_node.py` → `coverage_explorer` | reads `/map`, sends `NavigateToPose` | Drives until every reachable cell is seen AND approached within 0.75 m. Navigability-aware selection + goal-progress watchdog. |
| Low-obstacle perception (sub-lidar blind spot) | `low_obstacle_node.py` → `low_obstacle` | camera → `/camera/low_obstacles` (PointCloud2, base_link) | Height-gated floor-boundary contacts (only obstacles below the lidar plane) **plus clear-ray endpoints on obstacle-free bearings** — without those, costmap marks never clear. |
| VLM scene understanding | `vlm_scene_node.py` → `vlm_scene` | `describe_scene` service → `/vlm_scene/description` | On-demand only; one cloud call per invocation. |
| VLM-driven exploration | `vlm_explorer_node.py` → `vlm_explorer` | camera → VLM → `NavigateToPose`; `/vlm_explorer/decision` | Chooses *where to look next* only; planner + brake keep it safe. Camera-frame reasoning; not yet map/frontier aware. |
| Planning / costmaps | Nav2 `planner_server` (SmacPlanner2D), `bt_navigator`; `config/lean_nav2.yaml` | `/navigate_to_pose`, `/plan`, `/global_costmap/*` | Costmap obstacle sources: `scan` **and** `camera_low`. Nav2 never publishes `/cmd_vel_motor`. |
| Mapping / localization | `slam_toolbox` (`mapping.launch.py`); optional `robot_localization` EKF (`enable_imu_fusion:=true`) | `/map`, `map→odom`, `odom→base_link` | EKF fuses wheel `vx` + gyro yaw-RATE only (not the absolute quaternion). |
| Sensors | `lidar.launch.py` (RPLidar), `camera.launch.py` (Pi Camera 3 NoIR) | `/scan`, `/camera_node/image_raw`, `/camera_node/camera_info` | Camera requires the pinned `~/.local/rpi-libcamera`; decode images honoring `step` (see `sphero_rvr_core/image_decode.py`). |

## Pure cores (no ROS, unit-tested)

`sphero_rvr_core/` holds the logic; the driver nodes are thin wrappers. This is
the main structural rule of the lean spine — if it can be tested without ROS or
hardware, it belongs here.

- RVR protocol/transport: `packet.py`, `commands.py`, `responses.py`, `driver.py`,
  `transport.py`, `serial_transport.py`, `fake_transport.py`, `dispatcher.py`,
  `command_queue.py`, `state.py`, `sensor_streaming.py`, `safety.py`, `odometry.py`
- Navigation/decisions: `coverage_exploration.py`, `decisive_control.py`
- Camera: `image_decode.py`, `floor_obstacle_detection.py`, `ground_projection.py`,
  `low_obstacle_brake.py`, `vlm_client.py`

## Invariants worth keeping

1. **One motor publisher.** Only the collision-stop supervisor writes
   `/cmd_vel_motor`; only `rvr_node` writes the UART.
2. **`max_forward_mps` is the final gate** — it silently clamps everything, so it
   must match the driver/controller cruise.
3. **The camera layer is complementary, not redundant.** It publishes only
   sub-lidar-plane obstacles; height gating is what keeps it from stopping at
   every wall and breaking gap crossing.
4. **Clear rays are load-bearing.** `low_obstacle`'s clear-range endpoints must sit
   between the costmap's `obstacle_max_range` and `raytrace_max_range`.
5. **Bringup is inert.** Launching brings up sensing/planning with nothing
   commanding motion; exploration is opt-in.

## Deferred / disabled

- `range_motion_node.py`, `live_route_runner_node.py` — disabled, slated for
  removal (needs a `supervised_rvr.launch.py` edit).
- `diagnostics/` — bench tools that run with no chassis: `frontier_diag.launch.py`,
  `costmap_analyze.py`, `plannability_check.py`, `lowobs_costmap_check.py`.
