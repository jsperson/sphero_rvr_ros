# Stage C — camera layer for low obstacles (design)

**Status: camera UNBLOCKED 2026-08-05 — `/camera_node/image_raw` streams (800x600
rgb8) + calibrated `camera_info`. Perception pipeline still to build. This doc is
the plan for that.**

## Camera is working (how)
Runs via a hand-built PiSP libcamera pinned at `~/.local/rpi-libcamera` (libpisp
1.5.0, matches the kernel) + a source `camera_ros` linked to it. Launch:
`ros2 launch sphero_rvr_driver camera.launch.py`. The apt libcameras do NOT work
(system 0.2.0 = vc4-only; ros-jazzy 0.7.1 = libpisp 1.3.0 mismatch → "cannot acquire
CFE"). The launch + config were culled and recovered from git 2026-08-05; the pinned
libcamera + source camera_ros live outside the repo (`~/.local`, `~/ros2_ws/src`) —
DO NOT delete. (See memory `camera-pinned-libcamera`.) libcamera was NOT the design
problem after all — the sensor-choice discussion below still stands.

## Problem
The 2D RPLIDAR sits at ~0.19 m. It is blind to anything below its plane — table/
chair legs, low clutter — and to overhangs. On 2026-08-04 the rover drove into
chair legs the lidar (and therefore the collision brake) could not see. Stage C
adds a **camera layer** that detects low obstacles and feeds them to the costmap
and/or the collision brake so the rover stops for / plans around them.

## Hardware present
- **Raspberry Pi Camera Module 3 NoIR** = Sony **IMX708** (monocular RGB, ~12 MP,
  autofocus, no IR-cut filter → better in low/IR light, but still **RGB, not
  depth**). CSI-connected; kernel-bound (`media-ctl` shows `imx708_wide_noir`
  ENABLED on `rp1-cfe`). `camera_ros` (ROS 2 libcamera bridge) is installed.

## BLOCKER — libcamera enumerates zero cameras (fix first)
Symptom: `camera_ros`/`cam --list` report **"no cameras available"** even though
the sensor is kernel-bound. Both the system libcamera (`libcamera0.2` **0.2.0**,
Ubuntu 24.04) and the ROS one (`ros-jazzy-libcamera` **0.7.1**, used by
`camera_ros`) fail. Root cause: neither libcamera has a working **pisp** pipeline
handler for the Pi 5's `rp1-cfe`/`pispbe` path (the vc4 tuning under
`ipa/rpi/vc4/` is Pi-4 era; no `pisp` pipeline present).

**Fix-plan (do WITH Scott, needs a reboot):**
1. Install the Raspberry Pi camera stack for the Pi 5 pisp pipeline — `rpicam-apps`
   + its libcamera (from the Raspberry Pi apt archive, or the distro's
   pisp-capable libcamera if newer). Verify `rpicam-hello --list-cameras` shows
   `imx708`.
2. Ensure `config.txt` autodetect is on (it is: `camera_auto_detect=1`) and, if
   needed, pin `dtoverlay=imx708`. Reboot.
3. Make `camera_ros` use the pisp-capable libcamera (matching lib versions, or
   rebuild `camera_ros` against it). Confirm `ros2 run camera_ros camera_node`
   publishes `/camera/image_raw`.
4. Capture a frame; record intrinsics from the published `camera_info`.

## Sensor assessment (important design decision for Scott)
Monocular RGB (Camera 3) can localize low obstacles **only with a flat-floor
assumption + inverse-perspective mapping (IPM)** and **floor/obstacle segmentation**
— both fragile (lighting, floor texture, non-flat floors). It gives no direct depth.

**Recommendation:** for robust low-obstacle sensing, a **depth camera** (OAK-D
Lite, RealSense D435, or a ToF) is far better — it yields 3D points directly, which
drop straight into a costmap obstacle layer. The Camera 3 NoIR can do a first cut
(floor-plane IPM), and is great for the *semantic* north-star (LLM scene
understanding), but is a weak primary sensor for reliable low-obstacle stopping.
Suggest: use the Camera 3 for semantics + a first-cut floor-plane obstacle detector;
plan a depth sensor for dependable low-obstacle safety.

## Pipeline (target architecture)
```
camera (/camera/image_raw + camera_info)
  -> low-obstacle detection
       monocular: floor segmentation -> obstacle pixels -> IPM to ground points
       (or depth: obstacle voxels straight to 3D points)
  -> points in base_link
  -> publish sensor_msgs/PointCloud2 on e.g. /camera/low_obstacles
  -> consumed by:
     (a) global/local costmap as an ObstacleLayer observation source (marking),
         so the PLANNER routes around low obstacles; and/or
     (b) the collision-stop supervisor as a second range input, so the BRAKE
         stops for low obstacles the lidar misses (the real safety win).
```
Integration points already exist: the costmap `obstacle_layer` takes multiple
`observation_sources` (add a `pointcloud` source); the collision supervisor is the
sole `/cmd_vel_motor` publisher and already fuses lidar sectors — a camera-derived
front range can gate it too.

## Calibration needed
- Camera mount height + pitch (measure), extrinsics `base_link -> camera` (static
  TF, like the lidar's).
- Intrinsics from `camera_info` (camera_ros publishes it once libcamera works).
- Floor model (for monocular IPM): assume z=0 plane in base frame.

## Next steps (morning, with chassis)
1. Unblock libcamera (fix-plan above) → confirm `/camera/image_raw`.
2. Decide sensor strategy (Camera 3 first-cut vs. add depth cam).
3. Prototype detection on captured stationary frames (no motion).
4. Publish `/camera/low_obstacles` PointCloud2; wire into the costmap obstacle
   layer first (planner avoidance), then optionally the brake (hard stop).
5. Hardware test against the actual chair-leg scenario.
