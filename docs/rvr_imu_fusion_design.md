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

## Validation

1. **Stationary sanity** — DONE (2026-08-03): `/imu` publishes at ~20 Hz,
   `linear_acceleration.z ~ +9.2` (z-up), angular velocity ~0 at rest.
2. **EKF wiring** — DONE: `enable_imu_fusion` brings up driver + EKF + IMU TF;
   `/odometry/filtered` publishes at ~20 Hz; the EKF is sole publisher of
   `odom -> base_link`; with `imu0_differential: true` the fused yaw starts
   aligned with the odom frame (wheel 0.0, EKF -0.0 at rest — no offset).
3. **Drift comparison** — TODO (attended): drive a straight ~2 m leg and a full
   rotation with a *deterministic* drive mode (native tank-SI, not raw_motor —
   raw_motor's pivot direction was inconsistent during testing and confounds
   sign checks). Compare fused vs wheel-only yaw; fused should not accumulate the
   ~20 deg error.
4. **Re-test autonomous narrow-gap crossing** — TODO: the head-on approach should
   hold alignment now that heading is accurate (Stage A open item #1).

## Status

- Wire protocol + decode + driver + `/imu` + EKF + launch: implemented,
  unit-tested (345 pass), and **hardware-validated** — streaming, axis mapping,
  and EKF fusion all confirmed on the RVR (2026-08-03). On
  `feat/stage-b-imu-fusion` (unmerged).
- Two hardware bugs found + fixed during validation: (a) streaming config must be
  unpadded (zero-padding → firmware err 7); (b) IMU frame is ROS REP-103 already
  (identity, not the assumed y-right); (c) EKF must fuse IMU yaw differentially.
- Remaining before merge: the drift-comparison drive (deterministic mode) and the
  narrow-gap re-test; optional EKF noise tuning against the drive data.
