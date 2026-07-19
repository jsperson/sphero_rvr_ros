# VS02 replay-first SLAM map save/reload

This document is the no-hardware execution plan for SLAM Toolbox replay mapping,
map save, map reload, and localization gating. It consumes the VS01 capability
matrix in `docs/vertical_slice_capability_matrix.md`.

## Replay asset

Primary bag from VS01:

```text
/home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag
```

Useful topic counts:

```text
/scan: 202
/tf_static: 3
/camera_node/image_raw: 596
/camera_node/camera_info: 596
/odom: 0
/tf: 0
```

The bag is safe for replay-first mapping surface checks because it has no
`/cmd_vel`, `/cmd_vel_motor`, teleop, raw motor, or live-driver topics. It is not
sufficient for odometry-backed localization validation because `/odom` and
dynamic `/tf` are absent.

## Dry-run command planner

The repository helper prints the exact commands without starting ROS:

```bash
ros2 run sphero_rvr_driver rvr_slam_replay_plan \
  /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag \
  --map-name vs02_replay_map \
  --map-dir ~/maps \
  --topic-count /scan=202 \
  --topic-count /tf_static=3 \
  --topic-count /odom=0 \
  --topic-count /tf=0
```

Equivalent developer-host script:

```bash
PYTHONPATH=src scripts/rvr-slam-replay-plan \
  /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag \
  --map-name vs02_replay_map \
  --map-dir ~/maps \
  --topic-count /scan=202 \
  --topic-count /tf_static=3 \
  --topic-count /odom=0 \
  --topic-count /tf=0
```

Expected dry-run surfaces:

```text
ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=false start_lidar:=false start_camera:=false start_slam:=true use_sim_time:=true
ros2 bag play /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag --topics /scan /tf_static /camera_node/image_raw /camera_node/camera_info
ros2 run nav2_map_server map_saver_cli -f ~/maps/vs02_replay_map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/maps/vs02_replay_map.yaml -p use_sim_time:=true
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic echo --once /map
```

Important: replay mapping disables live lidar and camera launch with
`start_lidar:=false start_camera:=false`; the bag publishes recorded sensor data.
`start_rvr:=false` must remain present. Do not run `start_rvr:=true`, teleop,
range-motion goals, `/cmd_vel`, `/cmd_vel_motor`, or any live RVR driver path for
this replay slice.

## Map save and reload artifacts

Committed deterministic reload fixture:

```text
artifacts/vs02_slam_replay_fixture_map/vs02_replay_fixture_map.yaml
artifacts/vs02_slam_replay_fixture_map/vs02_replay_fixture_map.pgm
artifacts/vs02_slam_replay_fixture_map/manifest.json
```

This fixture exists so downstream workers can validate `nav2_map_server` YAML/PGM
reload behavior without pretending the VS01 bag produced a full occupancy map.
When a ROS host has enough TF/odom data, replace the fixture with a map saved by:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/<safe-map-name>
```

Then reload and inspect `/map`:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/maps/<safe-map-name>.yaml -p use_sim_time:=true
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic echo --once /map
```

## Localization gate

Full SLAM Toolbox localization validation requires, at minimum, replay messages
for:

```text
/scan
/odom
/tf
/tf_static
```

The VS01 bag has `/scan` and `/tf_static`, but `/odom` and dynamic `/tf` each have
zero messages. Therefore VS02 can validate replay command construction, safe
mapping launch arguments, map-save surfaces, and map YAML/PGM reload surfaces; it
cannot honestly claim odometry-backed localization outputs from that bag. Use a
new approved replay bag with `/odom` and `/tf` before claiming localization pass.
