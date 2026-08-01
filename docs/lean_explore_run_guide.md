# Tank-SI Mapping Gate (Before Lean Explore)

`explore.launch.py` and every autonomous Nav2 goal were **quarantined** after the
2026-08-01 speed-control incident. Scott's attended floor check now records a
mapping `PASS` at both `0.03 m/s` and `0.05 m/s`. That result unlocks only the
separately attended forward-goal procedure below; it does not authorize an
unattended launch or goal.

The corrected future command path is:

```text
/navigate_to_pose -> Nav2 -> /cmd_vel
  -> lidar_collision_stop_supervisor (sole /cmd_vel_motor publisher)
  -> sphero_rvr_driver
  -> RVRDriver.set_velocity(native_tank_si)
  -> drive_tank_si_units(left_mps, right_mps)
```

The dedicated `config/lean_rvr_tank_si.yaml` caps requested linear speed at
`0.05 m/s` and angular speed at `0.4 rad/s`. The deployed `config/rvr.yaml` is
unchanged. The unsafe RC-SI mode is rejected by `RVRDriver`.

## What this gate measures

`rvr_tank_si_mapping_validate` opens the normal `SerialTransport`, constructs
the real `RVRDriver` in `native_tank_si` mode, and refreshes motion exclusively
through `RVRDriver.set_velocity()`. It never calls a direct drive packet or raw
motor bypass. It reads onboard left/right encoder counts and converts them with
the same `4337.768 counts/m` calibration used by ROS odometry.

Each bounded trial has a 0.5 s warmup, a 2.0 s measured interval, an immediate
driver STOP, and a 0.75 s stationary check. It reports commanded versus measured
velocity and post-STOP travel for `0.03` and `0.05 m/s`. The process has a 30 s
overall timeout and rejects any requested speed above `0.05 m/s`.

Floor acceptance requires:

- the `0.05 m/s` measured average to be within ±20% (`0.04–0.06 m/s`);
- post-STOP encoder travel of at most `0.01 m` for every trial;
- final report `acceptance: PASS`.

A wheels-up result is only a packet/direction precheck and is never ground-speed
acceptance.

## Prepare the Pi

Build the focused PR, then make the rover UART exclusively available:

```bash
ssh sphero-pi-2
cd /home/jsperson/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sphero_rvr_driver
source install/setup.bash

systemctl --user stop rvr-hierarchical-mission.service rvr-telemetry.service
sudo fuser /dev/ttyAMA0
```

`fuser` must print no owner. Do not start `explore.launch.py`, Nav2, the collision
supervisor, or another driver beside this exclusive validation harness.

## Default no-motion check

This exact command prints the bounded plan and exits without opening serial:

```bash
ros2 run sphero_rvr_driver rvr_tank_si_mapping_validate
```

Required: `motion` is `DISABLED` and `MOTION_SKIPPED` is printed.

## Optional wheels-up precheck

With Scott present, chassis securely supported, treads clear, and a hand at
power, run:

```bash
ros2 run sphero_rvr_driver rvr_tank_si_mapping_validate \
  --armed --attended --surface wheels-up
```

Stop immediately for wrong direction, vibration, or unexpected speed. The
expected final label is `BENCH_ONLY_NOT_GROUND_SPEED_ACCEPTANCE`; proceed only
if the behavior is calm and forward.

## Mandatory attended floor check

Place the rover on a clear, level, bounded floor with no stairs, ledges,
drop-offs, obstacles, or chair-mat ridge. Aim along at least 0.5 m of clear
travel. Scott keeps one hand at chassis power and runs exactly:

```bash
ros2 run sphero_rvr_driver rvr_tank_si_mapping_validate \
  --armed --attended --surface floor --floor-area-clear \
  --output /tmp/rvr-tank-si-mapping.json
```

Cut chassis power immediately for unsafe motion; do not wait for software. The
harness also sends STOP and disconnects on normal completion, failure, timeout,
or Ctrl-C.

Read the printed table-equivalent JSON for each trial's `commanded_mps`,
`measured_mps`, `relative_error`, and `post_stop_travel_m`. A nonzero exit or any
result other than `acceptance: PASS` fails the gate.

## Stop after the gate

After either outcome, verify the UART is released:

```bash
sudo fuser /dev/ttyAMA0
```

Do not run `explore.launch.py` and do not send a Nav2 goal as part of the mapping
procedure itself. A mapping `PASS` only unlocks the separate attended step below.

## Phase 2c attended 0.60 m forward-goal rerun

Run this only after the Phase 2c commit is merged and deployed, with Scott
present, the same clear level corridor, more than `0.60 m` of front clearance,
and one hand at chassis power. This deliberately retains native tank-SI and the
`0.05 m/s` cap. The roughly 19-degree yaw drift from the prior run is a separate
odom-yaw / left-right tank-calibration watch-item, not a reason to change this
goal or its terminal tolerances.

In terminal 1:

```bash
ssh sphero-pi-2
cd /home/jsperson/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sphero_rvr_driver
source install/setup.bash

systemctl --user stop rvr-hierarchical-mission.service rvr-telemetry.service
sudo fuser /dev/ttyAMA0 /dev/ttyUSB0
ros2 launch sphero_rvr_driver explore.launch.py start_motion_stack:=true
```

`fuser` must print no owner before launch. Wait until Nav2 is active and the map,
scan, odometry, and independent STOP service are present. In terminal 2:

```bash
ssh sphero-pi-2
cd /home/jsperson/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.60, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --feedback
```

Acceptance is action status `SUCCEEDED`, a smooth arrival, and an independently
responsive brake. To exercise or require the independent brake at any point,
run this in terminal 2 and then stop the launch with Ctrl-C in terminal 1:

```bash
ros2 service call /stop std_srvs/srv/Trigger "{}"
```

Cut chassis power immediately for unsafe motion; do not wait for software. This
guide does not authorize Codex or any unattended agent to run the rover.

## Phase 3 attended frontier exploration

`explore.launch.py` now adds the pinned upstream `explore_lite` node to the
proven lean spine. It reads the Nav2 global costmap, whose static layer reads
slam_toolbox `/map`, and sends ordinary `/navigate_to_pose` goals. The existing
velocity and safety path is unchanged:

```text
explore_lite -> /navigate_to_pose -> Nav2 -> /cmd_vel
  -> lidar_collision_stop_supervisor (sole /cmd_vel_motor publisher)
  -> sphero_rvr_driver (native_tank_si, 0.05 m/s cap)
```

This run is attended only. Use a small, cleared, level, bounded room with no
stairs, ledges, drop-offs, pets, or people in the rover's path. Keep one hand at
chassis power. Before launch, import and build the pinned source dependency:

```bash
ssh sphero-pi-2
cd /home/jsperson/ros2_ws
source /opt/ros/jazzy/setup.bash
vcs import --skip-existing src < src/sphero_rvr_ros/workspace.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to explore_lite sphero_rvr_driver
source install/setup.bash

systemctl --user stop rvr-hierarchical-mission.service rvr-telemetry.service
sudo fuser /dev/ttyAMA0 /dev/ttyUSB0
```

`fuser` must print no owner. Then start the complete exploration graph:

```bash
ros2 launch sphero_rvr_driver explore.launch.py start_motion_stack:=true
```

Stop immediately if repeated turns stall, motion wanders instead of acquiring
frontiers, or speed is unsafe. A stall is a turn-mapping finding to report before
any tuning; the known roughly 20-degree left drift is only a watch-item. Cut
chassis power for unsafe motion rather than waiting for software.

When the room is covered, stop motion and save the map from a second sourced
terminal:

```bash
ros2 service call /stop std_srvs/srv/Trigger "{}"
ros2 run nav2_map_server map_saver_cli -f /home/jsperson/room_map
```

Finally press Ctrl-C in the launch terminal and verify both devices are released:

```bash
sudo fuser /dev/ttyAMA0 /dev/ttyUSB0
```

Acceptance is successive autonomous frontier goals, a map covering the bounded
room, an independently responsive brake, and a clean shutdown. Do not run this
procedure unattended.
