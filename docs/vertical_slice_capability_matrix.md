# Canonical shoe-mapping vertical slice capability matrix

Updated: 2026-07-19T04:31:43Z

This is the VS01 foundation handoff for downstream VS02+ workers. It records what may be treated as current evidence for replay-first semantic mapping work, and what still requires a human/physical approval gate.

## Repository and deployment baseline

| Surface | Verified state | Evidence command/result |
|---|---|---|
| Mac worktree | Clean worktree on `wt/t_6f12a8df`; `HEAD` equals `origin/main` after `git fetch origin main`. | `git status --short --branch` -> `## wt/t_6f12a8df`; `git rev-parse HEAD` and `git rev-parse origin/main` -> `5531c912c9f3e78577a43d4c21e79dda623b4d33`. |
| GitHub remote | `origin/main` is `5531c912c9f3e78577a43d4c21e79dda623b4d33`. | `git fetch origin main`; `git rev-parse origin/main`. |
| Pi deployed source | `sphero-pi-2:/home/jsperson/ros2_ws/src/sphero_rvr_ros` is clean on `main...origin/main`; deployed `HEAD` equals `origin/main`. | SSH no-hardware check: `git -C ~/ros2_ws/src/sphero_rvr_ros status --short --branch`; `git rev-parse HEAD`; `git rev-parse origin/main` -> `5531c912c9f3e78577a43d4c21e79dda623b4d33`. |
| Target ROS runtime | Ubuntu/ROS 2 Jazzy workspace at `sphero-pi-2:/home/jsperson/ros2_ws`. | `ros2 bag info` succeeded against the replay bag using sourced `/opt/ros/jazzy/setup.bash` and `~/ros2_ws/install/setup.bash`. |

Downstream workers should pin their assumptions to commit `5531c912c9f3e78577a43d4c21e79dda623b4d33` unless they first re-verify both Mac `origin/main` and the Pi checkout.

## Replay asset inventory

Primary reusable bag:

```text
/home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag
```

Bag metadata from `ros2 bag info`:

| Field | Value |
|---|---|
| Storage | `mcap`, ROS distro `jazzy`, file compression `zstd`, file `bag_0.mcap.zstd` |
| Duration | `20.844579921s` |
| Start | `2026-07-17 11:24:21.290083213` local Pi clock output from `ros2 bag info` |
| End | `2026-07-17 11:24:42.134663134` local Pi clock output from `ros2 bag info` |
| Messages | `1397` total |
| Files observed | `bag_0.mcap.zstd` reported by rosbag metadata; an uncompressed `bag_0.mcap` was also present in the directory during inventory. |
| Run manifest | No `run_manifest.json` was present beside this bag during VS01 inventory; rely on bag metadata plus this document. |

Topics in the bag:

| Topic | Type | Count | Availability note |
|---|---|---:|---|
| `/camera_node/camera_info` | `sensor_msgs/msg/CameraInfo` | 596 | Available and calibrated/nonzero. |
| `/camera_node/image_raw` | `sensor_msgs/msg/Image` | 596 | Available; 800x600 Pi Camera 3 stream. |
| `/scan` | `sensor_msgs/msg/LaserScan` | 202 | Available; frame `laser`. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | Available; contains `base_link -> camera_link`, `camera_link -> camera_optical_frame`, and `base_link -> laser`. |
| `/odom` | `nav_msgs/msg/Odometry` | 0 | Not present in this bag. Do not use this bag alone to validate odometry-dependent SLAM/localization behavior. |
| `/tf` | `tf2_msgs/msg/TFMessage` | 0 | Not present in this bag. Only static TF is available. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 0 | Not present in this bag. |
| `/cmd_vel`, `/cmd_vel_motor`, motor/teleop topics | n/a | 0 | Not present; the bag is data-only for replay. |

Message samples collected by replaying only `/scan`, `/camera_node/camera_info`, and `/tf_static` with no hardware/sensor launch:

| Surface | Verified values |
|---|---|
| `/scan` frame | `laser` |
| `/scan` range shape | 720 ranges, 625 finite valid ranges in sampled scan; `range_min=0.15000000596046448`, `range_max=16.0`, `angle_min=-3.1241390705108643`, `angle_max=3.1415927410125732`, `angle_increment=0.008714509196579456`. |
| `/camera_node/camera_info` frame | `camera_optical_frame` |
| CameraInfo dimensions/model | `width=800`, `height=600`, `distortion_model=plumb_bob`, nonzero K/D. |
| Bag CameraInfo normalized checksum | `a00b1dabfe61274c7f6dcb74f69a8c1efeca4078c5dd0f058f4660bbff257359` over width/height/distortion/K/D/R/P/binning/ROI JSON-normalized fields. |
| Robot-local CameraInfo file checksum | `f5c0de153eeb773ce4940d78b4956cd6a12e22de722803af7499038824761310  /home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml` on `sphero-pi-2`. |
| Static TF | `base_link -> camera_link`: translation `[0.0587375, -0.0301625, 0.1143]`, identity rotation. |
| Static TF | `camera_link -> camera_optical_frame`: translation `[0,0,0]`, optical-frame quaternion approximately `[-0.5, 0.5, -0.5, 0.5]`. |
| Static TF | `base_link -> laser`: translation `[-0.0074295, -0.009525, 0.1905]`, quaternion `[0, 0, 0.9999611664200241, 0.008812811804695843]` matching measured yaw `3.1239668018215028`. |

Safe replay commands for this bag:

```bash
# Dry-run the repo helper; prints the safe ros2 bag play command and rejects motor topics by default.
ros2 run sphero_rvr_driver rvr_rosbag_replay /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag \
  --topic /scan \
  --topic /camera_node/image_raw \
  --topic /camera_node/camera_info \
  --topic /tf_static

# Execute data-only replay. This publishes only recorded sensor/static-TF topics;
# it does not start lidar, camera, RVR driver, mapping, teleop, or /cmd_vel.
ros2 run sphero_rvr_driver rvr_rosbag_replay --execute /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag \
  --topic /scan \
  --topic /camera_node/image_raw \
  --topic /camera_node/camera_info \
  --topic /tf_static

# Equivalent direct ROS command when the package helper is unavailable.
ros2 bag play /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag \
  --topics /scan /camera_node/image_raw /camera_node/camera_info /tf_static
```

Do not replay `/cmd_vel`, `/cmd_vel_motor`, raw motor, teleop, or velocity-like topics while any live driver graph is running. This bag does not contain those topics; keep it that way unless doing offline developer analysis with no robot graph.

## Downstream capability matrix

| Capability | Current reliable state | Downstream may rely on | Limits/no-go |
|---|---|---|---|
| Mapping scaffold | `launch/mapping.launch.py` defaults to `start_rvr:=false`, `start_lidar:=true`, `start_camera:=false`, `start_slam:=true`; `start_camera:=true start_rvr:=false` adds camera without motor driver. | VS02 may use replay/no-hardware launch planning and SLAM Toolbox configuration from `config/slam_toolbox.yaml`. | Live mapping with `start_rvr:=true` is motor-capable and requires the physical/human gate. The current primary bag has no `/odom` or dynamic `/tf`, so it cannot prove odometry-backed SLAM by itself. |
| Camera calibration | Pi Camera 3 800x600 calibration exists as robot-local runtime YAML; launch defaults point to `file:///home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml`. Replay CameraInfo is nonzero and calibrated. | VS04/VS05 may consume replay image + CameraInfo for detector/projection work, with frame `camera_optical_frame`. | Do not commit the robot-local YAML unless separately approved. Semantic localization must reject missing/zero CameraInfo. |
| Lidar calibration/TF | `config/lidar.yaml` and `launch/lidar.launch.py` use `/dev/rplidar`, baud `460800`, frame `laser`, measured `base_link -> laser` transform. Replay scan matches frame `laser`. | VS02/VS03/VS05 may use `base_link`, `laser`, and measured static TF values above. | Lidar-only live launch is sensor activation, not motor activation; only run when the task explicitly allows sensor checks. This VS01 run did not start live sensors. |
| STOP/ESTOP/collision supervisor | `supervised_rvr.launch.py` remaps the live driver away from public `/cmd_vel` to `/cmd_vel_motor`; public `/stop`, `/estop`, and `/clear_estop` are owned by `lidar_collision_stop_supervisor`. `config/collision_stop.yaml` fails closed on stale/missing TF/scan and uses manual reset. | VS03/VS08/VS10 may rely on the supervised graph contract: ordinary command sources publish `/cmd_vel`; only the supervisor publishes `/cmd_vel_motor`. | Any launch containing `rvr_node`, `/cmd_vel`, `/cmd_vel_motor`, STOP/ESTOP service calls against the live driver, teleop, or `rvr-console` is motor-capable and needs explicit physical approval. |
| Range motion | `range_motion_controller` is implemented as an optional node above `/cmd_vel` and below mission/navigation behavior; it uses lidar target clearance and odometry cross-checks, and stops on stale samples, target loss, unsafe clearances, stalls, timeout, displacement cap, and odom/lidar disagreement. | VS03 may use it as a deterministic segment primitive interface, not as a planner. Example 4-inch target clearance is `0.1016 m`. | `start_range_motion:=true` belongs inside a supervised motor-capable launch. Replay bag lacks `/odom`, so this bag can test scan-facing pieces but not full odom/lidar disagreement behavior. |
| Replay rosbag helper | `rvr_rosbag_capture`, `rvr_rosbag_replay`, and `rvr_rosbag_inspect` are installed ROS package executables; invoke on the Pi with `ros2 run sphero_rvr_driver <executable>`. The helper rejects unsafe topics by default and defaults to dry-run unless `--execute` is passed. | VS02/VS04/VS05 should prefer the helper commands above for repeatable no-hardware data replay. | `--allow-unsafe-topics` is developer-only offline analysis. Never use it while a live robot graph exists. |
| Mission/API/UI/plain-English future layers | No deterministic mission state machine, web/PWA controls, or constrained natural-language translator are established by VS01. | VS07/VS08/VS09 can treat this document as foundation evidence only. | LLM/plain-English layers must map only to allowlisted deterministic workflows; they must not publish arbitrary `/cmd_vel` or bypass supervisor gates. |

## Explicit human gates and no-go boundaries

These require a fresh human/physical approval card before action:

- Starting `ros2 launch sphero_rvr_driver rvr.launch.py` or any launch path containing `rvr_node`.
- Starting `ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=true` or `/mapping full confirm`.
- Running `rvr-console`, `rvr_tui`, teleop, Nav2, mission APIs, or anything that can publish `/cmd_vel` to a live graph.
- Calling live `/stop`, `/estop`, `/clear_estop`, `/rvr_driver/*` services unless the approval specifically covers live driver interaction.
- Running range-motion goals against live hardware.
- Activating live sensors outside a task that explicitly permits sensor-only checks. Sensor-only is no-motor, but it is still physical-device activation.
- Destructive cleanup of preserved replay runs, maps, calibration YAML, or logs.

Permitted without the physical gate, assuming commands are run exactly as scoped:

- Git status/fetch/diff checks.
- Local unit tests, compile checks, and docs validation.
- Pi package build/import/no-launch checks after sourcing ROS.
- `ros2 bag info` on preserved bags.
- `ros2 run sphero_rvr_driver rvr_rosbag_replay` dry-run, and helper `--execute`/direct `ros2 bag play` limited to non-motor topics from the replay bag when no live driver graph is running.

## Evidence commands run for this handoff

Mac/local:

```bash
git fetch origin main
git status --short --branch
git rev-parse HEAD
git rev-parse origin/main
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m compileall -q src
git diff --check
```

Result for this handoff: `426 passed`, compileall passed, and `git diff --check` passed.

Pi/no-hardware:

```bash
ssh sphero-pi-2 'git -C ~/ros2_ws/src/sphero_rvr_ros status --short --branch; git -C ~/ros2_ws/src/sphero_rvr_ros rev-parse HEAD; git -C ~/ros2_ws/src/sphero_rvr_ros rev-parse origin/main'
ssh sphero-pi-2 'sha256sum /home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml'
ssh sphero-pi-2 'cd ~/ros2_ws; source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 bag info /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag'
ssh sphero-pi-2 'cd ~/ros2_ws; source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 pkg executables sphero_rvr_driver'
ssh sphero-pi-2 'cd ~/ros2_ws; source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 run sphero_rvr_driver rvr_rosbag_replay /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag --topic /scan --topic /camera_node/image_raw --topic /camera_node/camera_info --topic /tf_static'
ssh sphero-pi-2 'cd ~/ros2_ws; source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch sphero_rvr_driver mapping.launch.py --show-args'
ssh sphero-pi-2 'cd ~/ros2_ws; source /opt/ros/jazzy/setup.bash; source install/setup.bash; colcon build --symlink-install --packages-select sphero_rvr_driver'
```

Result for this handoff: package executables listed, replay helper dry-run printed the exact safe `ros2 bag play` command without starting a rosbag process, `mapping.launch.py --show-args` showed `start_rvr=false` and measured camera/lidar defaults, and Pi `colcon build --symlink-install --packages-select sphero_rvr_driver` passed.

No live driver, live lidar, live camera, `/cmd_vel`, STOP/ESTOP service, teleop, or motor-capable process was started for this inventory.
