# RVR IMU fusion (Stage B) design

## Goal

Remove the ~20 deg wheel-only yaw drift that limits precise heading (narrow-gap
approaches, long straight tracking). Fuse the RVR's onboard IMU with wheel
odometry so `odom -> base_link` heading stays accurate.

## Pipeline

```
RVR firmware IMU (ST processor, sensor streaming)
  -> StreamingServiceData (DID_SENSOR 0x3D)                 [core dispatcher]
  -> sensor_streaming.decode_streaming_packet + imu_sample  [typed decode]
  -> RVRDriver IMU callback / get_imu_sample()              [core driver]
  -> /imu  sensor_msgs/Imu                                  [rvr_node, when publish_imu]
  -> robot_localization ekf_node                            [fuses /odom vx + /imu yaw]
  -> odom -> base_link  (drift-corrected)                   [EKF owns the TF]
```

- **Streaming set** (`sensor_streaming.IMU_STREAM_SERVICES`): Quaternion (id
  `0x0000`), Accelerometer (`0x0002`), Gyroscope (`0x0004`), all 32-bit on the
  ST processor (target MCU), slot token 1. Constants transcribed from the Sphero
  public SDK (`sphero_sdk/common/sensors/`) — see `sensor_streaming.py` header.
- **Decode**: normalized big-endian uints -> floats via per-attribute min/max;
  gyro deg/s -> rad/s, accel g -> m/s^2. Fully unit-tested
  (`tests/test_sensor_streaming.py`, `tests/test_driver_imu_streaming.py`).
- **EKF** (`config/ekf.yaml`): 2D mode. Wheel odom contributes forward velocity
  (`vx`) only; the IMU owns absolute `yaw` + `vyaw`. We deliberately do **not**
  fuse wheel yaw or wheel x/y pose (both are derived from the drifting wheel
  heading) — the EKF integrates `vx` along the IMU-corrected heading.

## Frame handoff (important)

When fusion is on, the EKF is the **sole** publisher of `odom -> base_link`. The
launch enforces this: `enable_imu_fusion:=true` sets the driver's
`odom_publish_tf:=false` and `publish_imu:=true`. With fusion off, the driver
publishes the TF exactly as before (no behavior change). A static
`base_link -> imu_link` TF is published (default identity; override
`imu_x/y/z/yaw` once the mount is measured).

## How to enable

```bash
ros2 launch sphero_rvr_driver explore.launch.py \
    start_motion_stack:=true enable_imu_fusion:=true
```

Requires the `robot_localization` package (added to `package.xml`; on the Pi:
`sudo apt install ros-jazzy-robot-localization`).

## The one hardware-dependent assumption: axis mapping

Everything above the axis mapping is specified by the SDK and unit-tested. The
RVR-body-frame -> ROS REP-103 (x-forward, y-left, z-up) conversion in
`sensor_streaming.imu_sample_from_packet` assumes the RVR reports body axes as
x-forward, **y-right**, z-up, so y and the z-rotation sign are negated. **This
must be verified on hardware before trusting the EKF:**

1. Bring up with `publish_imu:=true` (no motion needed — chassis can stay put on
   a table; the RVR board must be powered).
2. `ros2 topic echo /imu`. Checks:
   - **Gravity**: at rest, `linear_acceleration.z ~ +9.8` (z-up). If it reads
     ~-9.8, flip the z sign.
   - **Yaw rate**: rotate the robot CCW (viewed from above = +z). Expect
     `angular_velocity.z > 0`. If negative, flip the z gyro/quaternion sign.
   - **Orientation**: rotate +90 deg CCW; the quaternion yaw should increase.
3. If any check fails, correct the signs in `imu_sample_from_packet` (isolated,
   one-line changes) and re-verify.

## Validation plan (after axis verification)

1. **Stationary sanity** (no motion): `/imu` publishes stable orientation, near-
   zero angular velocity, ~1g on z.
2. **Drift comparison** (attended, chassis on): drive a straight ~2 m leg and a
   360 deg rotation. Compare fused yaw (EKF `odom -> base_link`) against wheel-
   only yaw — the fused heading should not accumulate the ~20 deg error.
3. **Re-test autonomous narrow-gap crossing**: the head-on approach should hold
   alignment through the gap now that heading is accurate (Stage A open item #1).

## Status

- Wire protocol + decode + driver + `/imu` publisher + EKF config + launch:
  implemented, unit-tested, builds. **On `feat/stage-b-imu-fusion` (unmerged).**
- Pending hardware: axis-sign verification, EKF noise tuning against real data,
  then the drift + gap-crossing validation runs.
