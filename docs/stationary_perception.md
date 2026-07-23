# Live stationary perception

Stage C runs live Pi lidar, `slam_toolbox`, and camera perception while the
rover remains physically stationary. The stationary launch contains only the
RPLidar driver, camera driver, static transforms, `slam_toolbox`, and the
`stationary_perception` evidence node. It does not contain the rover driver,
route runner, collision-to-motor graph, `cmd_vel`, or rover UART transport.
The lidar launch explicitly resolves the upstream C1-compatible
`rplidar_ros` build from the sensor workspace; Ubuntu's older SDK can identify
this C1 but cannot start its scan.

Because Stage C is immobile, the launch publishes a fixed identity
`odom -> base_link` transform. `slam_toolbox` owns `map -> odom`, `/map`, and
the resulting localized pose. The identity odometry transform is truthful only
for this stationary session and must be replaced by real motion-state
estimation before any later driving stage.

Set `RVR_STATIONARY_PERCEPTION_ENABLED=true` only while
`RVR_LIVE_EXECUTION_ENABLED=false`. The mission service then exposes a
digest-bound stationary proposal. Approval starts continuous versioned live
snapshots and asynchronous ChatGPT-OAuth observation-intent revisions. Sensor
freshness and finite leases are checked independently of the provider worker.

Face identities come only from explicitly named subdirectories under
`RVR_FACE_ENROLLMENT_DIR`. Each usable image must contain exactly one detected
face. Enrollment files are SHA-256 referenced in detections and tracks; any
unsupported match remains `unknown`. The LLM cannot create or promote an
identity.

The pinned Pi camera stack derives its calibration identity from the current
libcamera device path. Stage C therefore uses a runtime-only copy of the
measured 800x600 calibration at `RVR_STATIONARY_CAMERA_INFO_URL`, changing only
the YAML `camera_name` to the exact current `camera_ros` identity. The original
measured calibration remains unchanged; both checksums belong in validation
evidence.

`rvr-stationary-perception.service` is installed but not automatically enabled
or started. Before starting it, verify `RVR_LIVE_EXECUTION_ENABLED=false`, no
rover driver or route executor process, no motion topics, and no owner of the
rover serial device. Stopping the service interrupts live sensors and causes a
running Stage C mission to terminate on freshness. Restarting sensors only
creates new live evidence; it cannot restore an expired mission intent.
