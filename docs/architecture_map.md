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
        /cmd_vel  (whoever is driving: Nav2 or teleop)
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
- Navigation/decisions: `coverage_exploration.py`
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

## Retired

- `range_motion` and `live_route_runner` were **deleted 2026-08-07** (~3,500 lines
  including tests, configs and launch plumbing). They had been default-off for weeks
  and were 27% of all source. Dead-but-plausible code is this project's demonstrated
  failure mode -- the closed-loop pivot controller sat unreachable below a branch that
  always fired and cost a full debugging session -- so retired code gets removed, not
  disabled.

## Diagnostics

`diagnostics/` holds the tools that produced the validation evidence. Prefer running
one of these over hand-reasoning — on 2026-08-06 three confident hypotheses were
refuted by measurement, and one would have injected an error into a correct calibration.

| Tool | Chassis? | What it answers |
|---|---|---|
| `frontier_diag.launch.py` | no | Bench stack: lidar → SLAM → global costmap with a fake static odom TF |
| `costmap_analyze.py` | no | Cell histogram + frontier count + ASCII view around the robot |
| `plannability_check.py` | no | Are selected coverage targets actually plannable? |
| `lowobs_costmap_check.py` | no | Do low obstacles MARK the costmap and does the planner route around? (synthetic cloud, works in the dark) |
| `camera_calibration_check.py` | no | Is the camera's horizontal calibration right? (blue-tape landmark vs lidar) — **run after any mount change** |
| `swept_brake_ab.py` | no (driver off) | Does the swept-path brake catch what the cone misses? Regression test for the chair-leg fix |
| `pivot_scale_check.py` | **yes** | Commanded vs odom vs gyro rotation — accumulates per sample, since start-vs-end yaw aliases past ±180° |
| `pivot_duty_measure.py` | **yes** | Steady-state pivot rate for the current duty settings |

Two traps these encode, both of which produced false results first:
1. **Raw `/scan` angles are not base_link bearings** — the lidar is mounted backwards
   (yaw ≈ 179°). Rotate, or the rear reads as the front.
2. **A parameter sweep that changes nothing may mean the code path is dead**, not that
   the parameter is irrelevant — that is how the unreachable pivot controller was found.
