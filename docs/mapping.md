# RVR lidar mapping bring-up

This is the ROS mapping scaffold for the Sphero RVR + SLAMTEC RPLIDAR C1 path.

## Current prerequisites

Verified so far:

- SLAMTEC C1 publishes `/scan` via upstream `rplidar_ros` `ros2` branch.
- `/dev/rplidar` udev alias exists for the C1 USB adapter.
- RVR UART is no longer used as Linux serial console/getty.
- RVR encoder odometry has an initial straight-line floor calibration:
  `odom_counts_per_meter: 4337.768`.

Still needs field validation:

- repeated odometry calibration samples
- measured lidar mount offsets: `laser_x`, `laser_y`, `laser_z`, `laser_yaw`
- real manual mapping run with careful stop/estop supervision

## Launch files

### Lidar only: no motors

```bash
ros2 launch sphero_rvr_driver lidar.launch.py
```

Publishes:

```text
/scan
/tf_static  # base_link -> laser
```

### Mapping scaffold, no RVR driver by default

```bash
ros2 launch sphero_rvr_driver mapping.launch.py
```

Default behavior:

```text
start_lidar:=true
start_slam:=true
start_rvr:=false
```

This starts lidar + `slam_toolbox`, but without the RVR driver there is no live
`odom -> base_link` transform. That default is intentional: it is safe to inspect
configuration and lidar/SLAM startup without exposing `/cmd_vel`.

### Full live mapping graph

WARNING: this can start the RVR motors.

Only run this when the robot is physically supervised and you are ready to stop
or power it off:

```bash
ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=true
```

Expected TF tree:

```text
map -> odom -> base_link -> laser
```

Expected topics:

```text
/scan
/odom
/tf
/tf_static
/map
/cmd_vel
```

The launch exposes `/cmd_vel` through the RVR driver. Do not run teleop/TUI until
stop/estop and odometry behavior are verified in the current room.

## Manual mapping workflow

The preferred operator path is through `rvr-console` / `rvr_tui`, which owns and cleans up the launch process it starts:

```text
/lidar start             # lidar.launch.py only, no RVR driver
/lidar stop              # stop the TUI-owned lidar launch
/mapping start           # mapping.launch.py start_rvr:=false, lidar + SLAM only
/mapping stop            # zero velocity, disarm, stop the TUI-owned launch
/mapping full            # warning only; does not launch the motor-capable graph
/mapping full confirm    # mapping.launch.py start_rvr:=true, MOTOR-CAPABLE
```

`/mapping full confirm` displays/logs `WARNING: this can start the RVR motors`, starts the full graph, and leaves the TUI disarmed until a separate `/arm confirm`. For parser/state-machine checks on a development host, run `rvr-console --dry-run`; the dry-run path does not source ROS, launch ROS processes, or open robot/lidar devices.

1. Start full live mapping graph with `/mapping full confirm` after explicit motor warning.
2. Verify topics/TF:

   ```bash
   ros2 topic list
   ros2 topic echo --once /scan
   ros2 topic echo --once /odom
   ros2 run tf2_ros tf2_echo base_link laser
   ros2 run tf2_ros tf2_echo odom base_link
   ```

3. Use tiny, supervised calibration-style motion first; do not jump straight to teleop.
4. Once TF and odom look sane, drive slowly by an approved ROS client.
5. Save map after a successful small-room pass:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/rvr_first_map
   ```

## Configuration files

- `config/lidar.yaml` — RPLIDAR C1 serial/frame settings.
- `config/rvr.yaml` — RVR driver safety/odometry settings.
- `config/slam_toolbox.yaml` — conservative online async SLAM Toolbox config.

## Notes

SLAM Toolbox can produce plausible-looking nonsense if odometry or TF is wrong.
Treat the first maps as diagnostics, not navigation-ready artifacts.
