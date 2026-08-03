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

## Axis mapping (verified on hardware 2026-08-03)

The RVR IMU body frame is **x-forward, y-left, z-up — already ROS REP-103**, so
`sensor_streaming.imu_sample_from_packet` uses an **identity** mapping (the only
transform is reordering the quaternion from RVR `(W,X,Y,Z)` to ROS `(x,y,z,w)`).
Confirmed with live streaming:

- **Gravity**: at rest, raw accel Z = **+0.94 g** → `linear_acceleration.z ~ +9.2`
  (z-up). No z negation.
- **Yaw rate / orientation**: a commanded CCW/left pivot produced raw gyro
  Z = **+172 deg/s** and quaternion Z **+0.54** → right-handed, yaw+ = CCW =
  ROS +z. No negation.

(Superseded the initial y-right assumption, which would have negated y and z.)

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
