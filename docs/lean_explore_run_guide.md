# Lean Explore: One Forward Goal

This is the Get-Well Phase 2 command path:

```text
/navigate_to_pose
  -> SmacPlanner2D
  -> Regulated Pure Pursuit
  -> /cmd_vel (geometry_msgs/Twist)
  -> lidar_collision_stop_supervisor
  -> /cmd_vel_motor
  -> sphero_rvr_driver (native_rc_si)
  -> /dev/ttyAMA0
```

It deliberately omits the camera, mission service, hierarchical controller,
semantic adapter, live-route bridge, adaptive controller, LLM, evidence
machinery, rotation shim, DWB, deadband, and velocity smoother. The Pi has the
Regulated Pure Pursuit plugin installed, so this path uses it directly. The
package-supplied `navigate_w_replanning_only_if_goal_is_updated.xml` is the
standard minimal NavigateToPose tree used for this single goal; it has no
automatic spin or back-up recovery sequence.

The driver parameters are in `config/lean_rvr_native_si.yaml`. The deployed
`config/rvr.yaml` remains unchanged and continues to select `raw_motor` for the
old stack. `explore.launch.py` passes the lean file explicitly through
`supervised_rvr.launch.py`.

## Safety boundary

Run only while Scott is present in a clear, level, bounded room with no stairs,
ledges, drop-offs, or chair-mat ridge. Keep a second terminal ready for STOP and
ESTOP. The collision supervisor remains the only `/cmd_vel_motor` publisher and
retains scan freshness, collision braking, command timeout, STOP, and ESTOP.
The driver and supervisor independently cap output at `0.10 m/s` and
`0.4 rad/s`.

Do not send a goal when collision state is anything except `CLEAR`. A `SLOW` or
vetoed run is safe but inconclusive for drive-quality acceptance.

## Build on the Pi

From the deployed workspace after the focused PR is available:

```bash
ssh sphero-pi-2
cd /home/jsperson/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sphero_rvr_driver
source install/setup.bash
```

Before opening either device, verify that no older hardware graph owns it:

```bash
systemctl --user is-active rvr-hierarchical-mission.service rvr-telemetry.service
sudo fuser /dev/ttyAMA0 /dev/ttyUSB0
```

Both services must be inactive and `fuser` must print no owner. Stop and resolve
any owner; do not launch competing stacks.

## No-motion bringup

The following starts the complete graph, including the motor transport, but
sends no goal and therefore commands no wheel motion:

```bash
ros2 launch sphero_rvr_driver explore.launch.py \
  start_motion_stack:=true \
  serial_port:=/dev/ttyAMA0 \
  lidar_serial_port:=/dev/ttyUSB0
```

In a second Pi shell:

```bash
cd /home/jsperson/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 lifecycle get /slam_toolbox
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 action list -t
ros2 topic info --verbose /cmd_vel
ros2 topic info --verbose /cmd_vel_motor
ros2 topic echo --once /collision_stop/state
ros2 topic echo --once /map
ros2 topic echo --once /odom
```

Required result before a physical goal:

- the five lifecycle nodes report `active`;
- `/navigate_to_pose` has type `nav2_msgs/action/NavigateToPose`;
- `/cmd_vel_motor` has exactly one publisher,
  `/lidar_collision_stop_supervisor`, and one subscriber,
  `/sphero_rvr_driver`;
- `/cmd_vel` publishers are Nav2 only and its subscriber is the collision
  supervisor;
- collision state is `CLEAR`, and map/odometry messages are live;
- no `live_route_runner`, hierarchical, adaptive, mission, or camera node is
  present.

Do not send the goal during this no-motion checkpoint. End with Ctrl-C and
confirm the launch exits cleanly.

## Attended one-goal run

Restart the same launch with Scott present. Place the rover in the confirmed
clear corridor, aimed away from obstacles. The exact goal below is `0.60 m`
straight ahead with zero relative yaw, so it is inside the required
`0.5–0.75 m` distance and `±15°` initial-bearing window. The selected standard
tree computes the path only when the goal is issued or explicitly updated, so
the `base_link`-relative pose is transformed once rather than becoming a moving
target.

First confirm `CLEAR` again:

```bash
ros2 topic echo --once /collision_stop/state
```

Then Scott sends exactly one goal:

```bash
ros2 action send_goal --feedback \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: base_link}, pose: {position: {x: 0.60, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

Keep these independent brake commands ready in the second shell:

```bash
ros2 service call /stop std_srvs/srv/Trigger "{}"
ros2 service call /estop std_srvs/srv/Trigger "{}"
```

Use STOP or ESTOP immediately for unsafe motion; do not wait for Nav2. A brake
intervention makes the drive-quality trial inconclusive, which is the correct
safe result. After a normal arrival, call STOP, end the launch with Ctrl-C, and
verify both device owners are gone:

```bash
ros2 service call /stop std_srvs/srv/Trigger "{}"
sudo fuser /dev/ttyAMA0 /dev/ttyUSB0
```

Acceptance is visual and attended: one smooth drive to the goal, no destructive
jitter, collision state remaining `CLEAR`, and the independent brake surface
available. This run does not authorize unattended motion.
