# Pi Camera 3 and RPLIDAR calibration runbook

This runbook records the validated Pi Camera 3 and RPLIDAR static transform
defaults for Scott's RVR. The values are measured for the current physical mount
and should be remeasured if the payload deck, camera, lidar, or base reference
changes.

## Safety boundary

Camera and lidar checks are no-motor when they do not launch the RVR driver,
`rvr-console`, teleop, mapping full mode, `/cmd_vel`, or any live RVR service.
The camera/lidar calibration commands here are intended to run with the robot
stationary. Any workflow that starts the RVR driver remains covered by the live
motor warning/approval policy in `README.md` and `STATUS.md`.

## Current configuration surfaces

Camera launch:

```bash
ros2 launch sphero_rvr_driver camera.launch.py --show-args
```

Important inputs:

```text
camera_info_url:=file:///home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml
camera_x:=0.0587375 camera_y:=-0.0301625 camera_z:=0.114300
camera_roll:=0.0 camera_pitch:=0.0 camera_yaw:=0.0
camera_frame_id:=camera_link
camera_optical_frame_id:=camera_optical_frame
```

The calibrated `camera_info_url` is a robot-local runtime artifact, not a sample
intrinsics file. It was validated with 800x600 `rgb8` images published from the
`BGR888` camera mode with `step=2432`; row-wise pixel inspection must crop each
row to 2400 bytes before reshaping.

Operational dependency: the camera stack expects this file to exist on the robot
at `/home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml`. Keep it owned
by the robot user, readable by the ROS launch environment, backed up with the
robot deployment notes, and reinstalled after Pi/workspace rebuilds before using
semantic localization. Record a checksum after calibration and after restore, for
example:

```bash
sha256sum /home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml
```

Do not commit generated CameraInfo YAML artifacts to this repo unless a separate
review explicitly decides to version a sanitized fixture/sample.

Camera-only validation example:

```bash
ros2 launch sphero_rvr_driver camera.launch.py \
  camera_info_url:=file:///home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml
```

Lidar launch:

```bash
ros2 launch sphero_rvr_driver lidar.launch.py --show-args
```

Important inputs:

```text
laser_x:=0.004500 laser_y:=-0.011000 laser_z:=0.190500
laser_roll:=0.0 laser_pitch:=0.0 laser_yaw:=3.1239668018215028
frame_id:=laser
base_frame:=base_link
```

The lidar yaw was established electronically from a known flat target centered on
the rover +x centerline. The raw target center appeared at `-3.123966801821503`
radians in the LaserScan frame, so the static transform uses
`laser_yaw = +3.1239668018215028` radians to map that known target to
`base_link` +x.

Mapping launch can include the camera without starting it by default:

```bash
ros2 launch sphero_rvr_driver mapping.launch.py --show-args
ros2 launch sphero_rvr_driver mapping.launch.py start_camera:=true start_rvr:=false
```

## Camera calibration inputs

Before running calibration, choose and record:

- checkerboard interior corners: `columns x rows` where the count is internal
  corners, not printed squares;
- square size in meters, measured with calipers or a ruler, for example
  `0.0245` for 24.5 mm squares;
- camera mode used for semantic mapping, currently 800x600 `BGR888` through Pi
  Camera 3 / `camera_ros`, which publishes `rgb8` with `step=2432`;
- target frame IDs: image header should use `camera_optical_frame`; TF should
  contain `base_link -> camera_link -> camera_optical_frame`.

Do not copy sample intrinsics from another camera. The Pi Camera 3 lens module,
focus, resolution, and mounting can all change K/D.

## Image collection process

1. Print or display a flat checkerboard with known square size.
2. Keep the RVR powered safely and stationary; this is a camera-only workflow.
3. Start the camera with the measured runtime file or a previous candidate file:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   "$(ros2 pkg prefix sphero_rvr_driver)/share/sphero_rvr_driver/scripts/rvr-camera-node" \
     --ros-args -p width:=800 -p height:=600
   ```

4. Move the checkerboard through the field of view: center, corners, near/far,
   tilted around roll/pitch/yaw. Avoid motion blur and glare.
5. Confirm the stream and frame ID:

   ```bash
   ros2 topic echo /camera_node/image_raw --once
   ros2 topic hz /camera_node/image_raw
   ```

## Calibration command and output format

Install the ROS camera calibration package if needed:

```bash
sudo apt install -y ros-jazzy-camera-calibration
```

Run calibration with the measured board dimensions. Example only; replace the
checkerboard and square size values with the physical target:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.0245 \
  --ros-args \
  -r image:=/camera_node/image_raw \
  -r camera:=/camera_node
```

When the calibrator reports a stable solution, install the generated YAML as a
robot-local runtime artifact, for example:

```text
/home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml
```

The YAML should contain `image_width: 800`, `image_height: 600`,
`camera_matrix`, `distortion_model`, `distortion_coefficients`,
`rectification_matrix`, and `projection_matrix`. Point `camera_info_url` at that
file with a `file://` URL.

## Camera validation criteria

After restarting the camera with the measured file:

```bash
ros2 topic echo /camera_node/camera_info --once
ros2 topic echo /tf_static --once
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo camera_link camera_optical_frame
```

Accept only if:

- `/camera_node/camera_info.width == 800` and `height == 600` for the configured mode;
- K must not be all zeros; `K[0]`, `K[4]`, and `K[8]` are nonzero;
- `D` is present, even if some coefficients are exactly zero;
- `distortion_model` is populated, commonly `plumb_bob` unless the calibrator
  produced a different valid model;
- reprojection error/RMS from `camera_calibration` is reasonable for the target
  and image quality; retake images if it is unstable or visibly bad;
- image header frame ID matches the optical frame used by semantic projection;
- TF contains `base_link -> camera_link -> camera_optical_frame`;
- persistence after restart is proven: stop the camera node, restart it with the
  same `camera_info_url`, and verify `/camera_node/camera_info` still carries the
  same dimensions, K/D, and distortion model.

Semantic localization must reject or loudly report empty intrinsics. This repo's
`require_configured_camera_info()` helper treats width/height zero, all-zero K,
missing focal terms, or missing `distortion_model` as unconfigured.

## Physical measurement procedure

Use the same `base_link` convention as the RVR odometry/TF design. The origin is
the center of the RVR tread/contact footprint at floor level: halfway between
left/right tread contact centerlines and halfway between front/rear tread contact
extents. Axes are `+x` forward, `+y` left, and `+z` up. Record signs and units in
meters/radians.

### Lidar: `base_link -> laser`

The 2026-07-27 tread-contact survey measured from the lidar scan origin to the
most forward, rearward, right, and left tread-contact extents:

| Direction from lidar | Distance |
| --- | ---: |
| forward | `0.200 m` |
| rearward | `0.209 m` |
| right | `0.213 m` |
| left | `0.235 m` |

The tread footprint is therefore `0.409 m` long and `0.448 m` wide. Its
midpoint places the lidar `0.0045 m` forward and `0.0110 m` right of
`base_link`, so the reviewed translation is
`base_link -> laser = [0.0045, -0.0110, 0.1905] m`. The yaw measurement is
unchanged.

1. Mark the RVR `base_link` origin at the tread-footprint center on the floor
   projection; do not use an undefined payload-deck drawing datum.
2. Mark the RPLIDAR scan origin, not merely the case center if the datasheet
   gives an offset.
3. Measure translation:
   - `laser_x`: forward positive from `base_link` to lidar scan origin;
   - `laser_y`: left positive;
   - `laser_z`: up positive.
4. Measure orientation:
   - keep `laser_roll` and `laser_pitch` near zero if the lidar is level;
   - set `laser_yaw` so scan angles align with the robot forward axis.
5. Launch lidar-only and validate without motors:

   ```bash
   ros2 launch sphero_rvr_driver lidar.launch.py \
     laser_x:=<m> laser_y:=<m> laser_z:=<m> \
     laser_roll:=<rad> laser_pitch:=<rad> laser_yaw:=<rad>
   ros2 run tf2_ros tf2_echo base_link laser
   ```

### Camera: `base_link -> camera_link`

1. Mark the camera optical center as closely as practical. Use the module/lens
   datasheet when the optical center is not the board center.
2. Measure translation from `base_link` to that optical center:
   - `camera_x`: forward positive;
   - `camera_y`: left positive;
   - `camera_z`: up positive.
3. Measure camera body orientation as `camera_roll`, `camera_pitch`, and
   `camera_yaw` in radians. A slight downward tilt is negative or positive only
   according to the chosen ROS frame convention; verify with TF/RViz instead of
   guessing.
4. Keep the optical-frame convention fixed: `camera_link -> camera_optical_frame`
   uses x right, y down, z forward via the static transform in `camera.launch.py`.
5. Validate:

   ```bash
   ros2 launch sphero_rvr_driver camera.launch.py \
     camera_info_url:=file:///home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml \
     camera_x:=<m> camera_y:=<m> camera_z:=<m> \
     camera_roll:=<rad> camera_pitch:=<rad> camera_yaw:=<rad>
   ros2 run tf2_ros tf2_echo base_link camera_link
   ros2 run tf2_ros tf2_echo camera_link camera_optical_frame
   ```

## Documentation rules

- Placeholder values stay labeled as placeholders; do not reintroduce them as
  mapping defaults over measured camera/lidar values.
- Measured values should include date, measurement method, and robot hardware
  layout.
- Calibration is not complete until CameraInfo, K/D, TF, reprojection quality,
  and persistence after restart all pass.
