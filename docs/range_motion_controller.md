# Closed-loop lidar range-motion controller

`range_motion_controller` is the reusable target-clearance motion primitive that lives between mission/navigation behavior and the independent collision-stop supervisor:

```text
mission / navigation behavior
  -> range_motion_controller
  -> /cmd_vel
  -> lidar_collision_stop_supervisor
  -> /cmd_vel_motor
  -> sphero_rvr_driver
```

It never publishes to `/cmd_vel_motor`, never replaces STOP/ESTOP, and never weakens stale-scan/TF/supervisor fail-closed behavior. The supervisor remains authoritative for the final motor command.

## ROS-free core

The tested core API is `sphero_rvr_driver.range_motion`:

- `MotionGoal`: direction (`forward`/`backward`), mode (`approach`/`retreat`), target clearance, optional measured-displacement cap, optional timeout.
- `RangeMotionConfig`: max/min speed, acceleration ramp, slowdown distance, target tolerance, stale-sample age, minimum front/rear/side clearances, range-jump gate, stall threshold, odom/lidar disagreement threshold.
- `RangeMotionSample`: timestamped tracked target clearance, multidirectional safety clearances, optional odom displacement.
- `RangeMotionTelemetry`: requested velocity, forwarded velocity, lidar range rate, odom velocity, measured displacement, confidence, current/target clearance, stop reason.

Distance is measured from lidar progress and odometry cross-checks. Production code must not infer travel from `commanded_velocity * duration`.

## ROS seam

The ROS node is intentionally interface-light until a custom action package is added. It exposes a start service, `range_motion/start` (`std_srvs/Trigger`), which reads the `service_goal_*` parameters, and it also accepts JSON goals on `/range_motion/goal` (`std_msgs/String`) for callers that need per-request values without a custom interface. It publishes JSON telemetry on `/range_motion/status` plus diagnostics. Cancel is exposed as `range_motion/cancel` (`std_srvs/Trigger`).

The node tracks a stable sector surface cluster from fresh scans before feeding target clearance into the core controller, rather than differentiating a one-scan nearest-point speckle.

Example 4-inch approach goal:

```json
{"direction":"forward","mode":"approach","target_clearance_m":0.1016,"max_measured_displacement_m":1.0,"timeout_s":8.0}
```

Example reverse release from a front obstacle until 4 inches clear:

```json
{"direction":"backward","mode":"retreat","target_clearance_m":0.1016,"timeout_s":8.0}
```

The default supervised launch keeps this optional and off until a mission/controller explicitly opts in:

```bash
ros2 launch sphero_rvr_driver supervised_rvr.launch.py start_range_motion:=true
```

Example service start with parameterized goal values:

```bash
ros2 param set /range_motion_controller service_goal_target_clearance_m 0.1016
ros2 service call /range_motion/start std_srvs/srv/Trigger {}
```

## Stop reasons

The core stops/fails closed on:

- target reached from measured clearance;
- stale lidar sample;
- target lost;
- implausible target jump;
- unsafe front/rear/side clearance;
- stall/no-progress;
- timeout;
- measured displacement cap;
- excessive odom/lidar disagreement;
- operator stop, ESTOP, driver fault, or cleanup uncertainty.

## Navigation integration seam

Systematic exploration should treat this as a segment primitive, not a random wandering behavior. A deterministic planner chooses a segment and target clearance, invokes range motion, receives measured progress/status, then chooses the next safe turn/segment from the map/lidar state after the controller stops. Random bump-and-turn wandering is explicitly out of scope.
