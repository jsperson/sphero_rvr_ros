# RVR odometry and TF design for SLAM readiness

## Decision

The ROS adapter publishes a conservative `nav_msgs/msg/Odometry` estimate on `/odom` and, by default, broadcasts the matching `odom -> base_link` transform. The estimate is derived from typed RVR encoder counts exposed by `sphero_rvr_core` and is meant as a scan-matching aid for `slam_toolbox`, not as a high-trust navigation source.

## Available telemetry inputs

| Input | Current core surface | Used for `/odom`? | Notes |
|---|---|---:|---|
| Left/right encoder counts | `RVRDriver.get_encoder_counts()` → `EncoderCounts(left, right)` | Yes | Best current input for incremental planar odometry. Counts are signed 32-bit values; the pure tracker unwraps boundary crossings. |
| Yaw reference/reset | `reset_yaw()` service, `drive_with_heading()` support | No | There is a reset/control surface, but no typed absolute yaw reading is currently exposed. Do not pretend yaw feedback exists. |
| Locator reset/flags | `reset_locator()`, `set_locator_flags()` | Reset only | Useful to reset local odom state alongside the robot's locator reference. No typed locator pose readout is currently in the safe ROS surface. |
| Gyro/IMU-like signals | `enable_gyro_max_notify()` only | No | The exposed event reports saturation/limit flags, not orientation or angular velocity. |
| Magnetometer | `get_magnetometer()` | No | Available in core, but not a reliable indoor yaw source without calibration/fusion. Keep out of default odometry. |
| Streaming service bytes | configure/start/stop/clear + raw event bytes | No | Opaque packet stream is intentionally not decoded or published in the default ROS graph. It can be investigated later if official SDK/protocol details justify it. |

## Frames

```text
map
  -> odom          # published later by slam_toolbox/localization
    -> base_link   # this package: encoder odometry estimate and TF
      -> laser     # lidar integration task: measured static transform
```

This task owns only `odom -> base_link`. Sensor mounting transforms such as `base_link -> laser` are handled by the lidar integration slice.

Config parameters:

- `odom_frame_id` default: `odom`
- `base_frame_id` default: `base_link`
- `odom_publish_tf` default: `true`
- `odom_publish_period` default: `0.1s`

## Math

`DifferentialOdomTracker` is pure Python and ROS-free:

1. Poll encoder counts.
2. Compute wrapped signed deltas for each track.
3. Convert counts to meters with `odom_counts_per_meter`.
4. Use differential-drive/skid-steer approximation:
   - `distance = (left_m + right_m) / 2`
   - `delta_yaw = (right_m - left_m) / wheel_track_m`
   - integrate at midpoint heading.
5. Normalize yaw to `[-pi, pi]`.
6. Emit pose, twist, covariance arrays, source, and quality note.

## Covariance and honesty tax

The defaults are deliberately nonzero and tunable:

- `odom_pose_xy_covariance`: `0.05`
- `odom_pose_yaw_covariance`: `0.25`
- `odom_twist_linear_covariance`: `0.10`
- `odom_twist_angular_covariance`: `0.50`

These are placeholders for low-speed scan matching, not measured sensor statistics. Tracked/skid-steer robots slip while turning, and prior floor testing showed turns can bog down depending on carpet/floor friction. `slam_toolbox` can often tolerate weak odometry when scan geometry is good and driving is slow, but mapping quality must be validated on the physical RVR.

## ROS import seam

All odometry math lives in `sphero_rvr_driver.odometry`, which imports only core response dataclasses and the standard library. `rclpy`, `nav_msgs`, `geometry_msgs`, and `tf2_ros` are imported only inside `rvr_node.main()`, preserving local unit-testability on hosts without ROS 2 installed.

## Validation added

Pure tests cover:

- differential integration from fake encoder telemetry;
- yaw normalization;
- non-monotonic timestamp re-baselining;
- signed 32-bit encoder rollover handling;
- covariance array slot layout;
- invalid config rejection.

ROS-free checks:

```bash
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests/test_odometry.py tests/test_ros_safe_surfaces.py tests/test_ros_node_config.py -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m compileall -q src
```

ROS/Pi checks after sourcing the Jazzy workspace:

```bash
ros2 launch sphero_rvr_driver rvr.launch.py
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Per the project safety rule, launching the RVR driver is motor-capable and needs explicit approval first.
