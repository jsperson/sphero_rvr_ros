# Pi Camera 3 and RPLIDAR calibration runbook

This runbook adds configuration surfaces only. It does not claim the Pi Camera 3
or RPLIDAR mount are calibrated. The checked-in defaults are placeholders until a
human captures images, measures the hardware, and writes the resulting values
back into launch/config inputs.

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
camera_info_url:=file:///tmp/UNCONFIGURED_RVR_PI_CAMERA3_CALIBRATION.yaml
camera_x:=0.0 camera_y:=0.0 camera_z:=0.0
camera_roll:=0.0 camera_pitch:=0.0 camera_yaw:=0.0
camera_frame_id:=camera_link
camera_optical_frame_id:=camera_optical_frame
```

`camera_info_url` is intentionally invalid by default. Replace it with a measured
calibration file URL such as:

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
laser_x:=0.0 laser_y:=0.0 laser_z:=0.15
laser_roll:=0.0 laser_pitch:=0.0 laser_yaw:=0.0
frame_id:=laser
base_frame:=base_link
```

The `laser_*` defaults are placeholders until measured. The existing `z=0.15`
only preserves the historical scaffold value; do not treat it as validated mount
geometry.

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
- camera mode used for semantic mapping, currently expected to be 800x600 BGRA
  through Pi Camera 3 / `camera_ros`;
- target frame IDs: image header should use `camera_optical_frame`; TF should
  contain `base_link -> camera_link -> camera_optical_frame`.

Do not copy sample intrinsics from another camera. The Pi Camera 3 lens module,
focus, resolution, and mounting can all change K/D.

## Image collection process

1. Print or display a flat checkerboard with known square size.
2. Keep the RVR powered safely and stationary; this is a camera-only workflow.
3. Start the camera with the unconfigured default or a previous candidate file:

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
   ros2 topic echo /camera/image_raw --once
   ros2 topic hz /camera/image_raw
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
  -r image:=/camera/image_raw \
  -r camera:=/camera
```

When the calibrator reports a stable solution, save/commit the generated YAML as
a robot-local runtime artifact, for example:

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
ros2 topic echo /camera/camera_info --once
ros2 topic echo /tf_static --once
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo camera_link camera_optical_frame
```

Accept only if:

- `/camera/camera_info.width == 800` and `height == 600` for the configured mode;
- K must not be all zeros; `K[0]`, `K[4]`, and `K[8]` are nonzero;
- `D` is present, even if some coefficients are exactly zero;
- `distortion_model` is populated, commonly `plumb_bob` unless the calibrator
  produced a different valid model;
- reprojection error/RMS from `camera_calibration` is reasonable for the target
  and image quality; retake images if it is unstable or visibly bad;
- image header frame ID matches the optical frame used by semantic projection;
- TF contains `base_link -> camera_link -> camera_optical_frame`;
- persistence after restart is proven: stop the camera node, restart it with the
  same `camera_info_url`, and verify `/camera/camera_info` still carries the
  same dimensions, K/D, and distortion model.

Semantic localization must reject or loudly report empty intrinsics. This repo's
`require_configured_camera_info()` helper treats width/height zero, all-zero K,
missing focal terms, or missing `distortion_model` as unconfigured.

## Physical measurement procedure

Use the same `base_link` convention as the RVR odometry/TF design. Record signs
and units in meters/radians.

### Lidar: `base_link -> laser`

1. Mark the RVR `base_link` origin on the payload deck reference drawing.
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

- Placeholder values stay labeled as placeholders.
- Measured values should include date, measurement method, and robot hardware
  layout.
- Calibration is not complete until CameraInfo, K/D, TF, reprojection quality,
  and persistence after restart all pass.
