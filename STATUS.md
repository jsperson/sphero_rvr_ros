# Sphero RVR ROS Project Status

Updated: 2026-06-25T00:00:00Z

## Current repo state

- Branch: `main`
- Latest deployed code baseline before the API-parity documentation handoff: `a344718 Revert "tune: tolerate slower RVR turn key repeat"`
- Local `HEAD` is `main`; the workspace includes uncommitted API-parity, safe-ROS-surface, test, and documentation changes awaiting review/deploy.
- `sphero-pi-2` was verified on the deployed code baseline `a344718` before this docs-only update; a follow-up SSH pull for this status file timed out.
- Local test suite at API parity validation handoff: `227 passed`.
- Driver capability coverage has sentinel tests for every public async `RVRDriver` method, every public `RVRCommands` builder, explicit capability-matrix test-state tokens, parser/omission classification for response/event payloads, ROS-exposure test coverage for `ros-exposed` matrix rows, and the README validation checklist.
- Target Pi workspace: `/home/jsperson/ros2_ws/src/sphero_rvr_ros`
- Target Pi install: pulled/rebuilt with `colcon build --symlink-install --packages-select sphero_rvr_driver`
- Background driver running through `ros2 launch sphero_rvr_driver rvr.launch.py`.
- Convenience wrapper installed: `/home/jsperson/.local/bin/rvr-console`

## Target hardware/runtime

- Target host: `sphero-pi-2`
- OS/runtime: Ubuntu Server 24.04 LTS, ROS 2 Jazzy
- UART: `/dev/ttyAMA0`, 115200 baud
- ROS workspace: `/home/jsperson/ros2_ws`
- Driver package: `sphero_rvr_driver`

## Safety-critical context

Earlier testing showed unsafe forward motor activity around `rvr-console` `/exit` and repeated keyboard commands. The root stop-path fix is still in place:

- `RVRCommands.stop()` sends validated `raw_motors(0, 0, 0, 0)`.
- `RVRCommands.emergency_stop()` sends validated `raw_motors(0, 0, 0, 0)` and software-gates future velocity.
- `RVRDriver.clear_emergency_stop()` is software-only.
- `rvr-console` shell cleanup and `RVRROSClient.close()` no longer call ROS `/stop`.
- TUI `/exit`/safe-stop publishes zero `/cmd_vel`.

Current safety rule for future sessions:

- Before any live motor-capable action, explicitly warn: `WARNING: this can start the RVR motors`.
- Do not run `rvr-console`, `rvr_tui`, ROS driver launches, `/cmd_vel`, `/stop`, or any live driver command without explicit user approval after that warning.
- Build/install/unit-test/fake-client checks are okay without motor approval because they do not talk to the robot.

## API parity validation state

The capability matrix lives at `docs/rvr_capability_matrix.md`. It requires every official SDK/API row to declare explicit validation state using `builder-test`, `parser-test`, `driver-test`, `notification-test`, `ros-exposure-test`, `fake-transport-test`, or `documented-omission`. Response/event payload rows must either declare `parser-test` or carry a documented omission for known protocol-research mismatches.

The ROS exposure policy lives at `docs/rvr_ros_exposure_policy.md`. Full API parity belongs in `sphero_rvr_core`; default ROS exposure remains the typed, bounded operational subset only. Raw motors, arbitrary packet sends, firmware/admin/factory/update flows, calibration, opaque streaming bytes, autonomous IR motion behaviors, and stable identifiers remain core-only or intentionally omitted from the routine ROS graph.

Current no-hardware validation gates:

```bash
python3 -m venv /tmp/sphero-rvr-ros-test
/tmp/sphero-rvr-ros-test/bin/python -m pip install -e '.[dev]'

PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests/test_missing_command_builders.py tests/test_response_parsers.py tests/test_dispatcher.py tests/test_driver_capability_coverage.py -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests/test_ros_safe_surfaces.py tests/test_ros_node_config.py tests/test_diagnostics.py -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m compileall -q src
git diff --check
```

Pi / ROS environment no-motion gate after sourcing ROS and the workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select sphero_rvr_driver
source install/setup.bash
ros2 pkg executables sphero_rvr_driver
python3 - <<'PY'
from sphero_rvr_driver.rvr_node import RVRNodeConfig
print(RVRNodeConfig())
PY
```

This gate intentionally stops before `ros2 launch sphero_rvr_driver rvr.launch.py`; launching the node is live UART access and must use the hardware-smoke policy below.

Live hardware smoke remains gated by the exact warning above (`WARNING: this can start the RVR motors`) and explicit approval. After approval, keep the robot restrained/suspended and scope the first smoke to topic/service listing, battery/diagnostics/ambient/odom reads, TF echo, `/stop`, one conservative `ros2 topic pub --once /cmd_vel ...`, and a final `/stop`; sustained driving, TUI, mapping, or autonomy require a separate approval scope.

## Safe ROS surfaces pending deploy/review

Safe ROS surfaces added locally for the next deploy:

- `/odom` and `odom -> base_link` TF from typed encoder-count deltas, with configurable `odom_counts_per_meter`, `odom_wheel_track_m`, `odom_frame_id`, `base_frame_id`, `odom_publish_tf`, and nonzero covariance defaults. Design/limitations are documented in `docs/rvr_odometry_tf_design.md`.
- `ambient_light`, left/right motor temperature, battery, and richer diagnostics key-values for battery voltage state, motor fault, firmware version, board revision, processor, and uptime.
- `reset_yaw`, `reset_locator`, and `release_led_requests` Trigger services, plus bounded `set_all_leds` `ColorRGBA` subscription.
- Explicitly still not exposed: raw motors, firmware/admin/update/factory flows, calibration, arbitrary packet sends, opaque streaming bytes, MAC/stats IDs.

ROS validation still needs to run in the ROS 2 Jazzy environment on `sphero-pi-2` after build; local host validation is ROS-free unit/compile coverage only. Do not treat macOS fake/unit green as live robot validation.

## Current deployed control behavior

Driver velocity scaling and turn direction have been tuned from live floor testing:

- Positive angular commands now map to physical left turns correctly.
- `max_raw_motor_duty` is `160` in `config/rvr.yaml` and `RVRNodeConfig`.
- Max turn payloads at duty cap:
  - left: `02a001a0`
  - right: `01a002a0`
- TUI timing constants:
  - `KEY_STOP_SECONDS = 0.30`
  - `TURN_KEY_STOP_SECONDS = 0.09`
  - `TURN_HOLD_DETECT_SECONDS = 0.15`
  - `KEY_REPEAT_SECONDS = 0.10`

Pure turn behavior:

- First left/right tap is a short nudge: one turn command, then zero after `0.09s`.
- If the same turn key repeats within `0.15s`, the TUI treats it as a held turn and internally republishes at `0.10s` cadence until the normal `0.30s` timeout.
- Forward/reverse still use the normal `0.30s` timeout and internal repeat.

## Live floor-test findings

Best current version is commit `a344718`.

Observed behavior on the physical RVR:

- Original turn direction was reversed; fixed by changing the tank mix to `left = linear - angular`, `right = linear + angular`.
- Suspended turns worked at lower duty, but on the floor the RVR needed much more torque because skid-steer turning must overcome tread/floor friction.
- Raising `max_raw_motor_duty` from `64` to `96` improved turns but was still weak.
- Raising `max_raw_motor_duty` to `160` gave usable floor-turn authority.
- `/speed` affects forward/reverse only; left/right uses `/turn`.
- Useful turn range during testing:
  - `/turn 0.3` did not reliably turn.
  - `/turn 0.4` turned too fast.
  - `/turn 0.35` was the main working test value.
- A long turn sustain timeout (`0.75s`) caused one tap to pivot about 90 degrees, which was too coarse.
- A short turn nudge (`0.09s`) gives good one-click precision.
- Attempts to make held turns more tolerant by extending held-turn detect/stop windows made continuous turning too fast.
- Current best tradeoff: tap precision is good; continuous turns are still not fully reliable and may bog down.

## Current operator commands

Start TUI on the Pi:

```bash
rvr-console
```

Inside TUI:

```text
/battery
/status
/arm
/speed 0.2
/turn 0.35
/disarm
/stop
/estop
/clear-estop
/exit
```

Keyboard controls after `/arm`:

```text
↑ / w      forward
↓ / s      reverse
← / a      turn left
→ / d      turn right
space      stop
q          quit
```

Logs:

```bash
tail -100 ~/.local/state/sphero_rvr/rvr-console.log
tail -100 ~/.local/state/sphero_rvr/rvr-driver.log
```

## Suggested next engineering work

1. Keep the current tap behavior as baseline; do not extend the one-tap timeout again without an explicit nudge/hold split.
2. Add a dedicated turn profile instead of tuning only timeouts:
   - tap: short high-duty nudge
   - hold: lower sustained duty or cadence-controlled turn
   - optional breakaway kick for 100-200ms, then lower sustain duty
3. Add `/nudge-turn` or TUI mode settings if manual tuning remains necessary.
4. Add a `--dry-run`/fake-driver mode for `rvr-console` so TUI timing behavior can be tested interactively without hardware.
5. Add driver-level velocity coalescing so repeated `/cmd_vel` updates cannot queue stale nonzero motor packets.
6. Add a mock ROS/TUI integration test that simulates repeated direction commands followed by `/exit` and asserts the final command path is zero velocity/raw motor off.

## Lidar, SLAM, and autonomy roadmap

Goal: add a 2D lidar so the RVR can first build maps while manually driven, then later use those maps for cautious autonomous navigation.

### Practical verdict

Yes, it is reasonable to mount a ROS-supported 2D lidar and map distances while driving the RVR around. The ROS pattern is **SLAM**: stream lidar scans, estimate robot pose over time, and build a 2D occupancy map. Start with manual mapping; do not jump directly to Nav2 autonomy.

### Target ROS graph

```text
RVR base driver
  subscribes /cmd_vel
  publishes /odom or equivalent odometry estimate
  publishes tf: odom -> base_link

Lidar driver
  publishes /scan as sensor_msgs/msg/LaserScan
  publishes/static tf: base_link -> laser

slam_toolbox
  subscribes /scan + tf/odom
  publishes /map and tf: map -> odom

Later Nav2
  consumes map/localization/costmaps
  publishes /cmd_vel goals through the existing safety-gated base driver
```

Mapping needs more than raw distances: each scan must be tied to an estimated robot pose. Lidar-only visualization is easy; usable mapping depends on at least rough odometry/tf and careful low-speed driving.

### Hardware received: SLAMTEC RPLIDAR C1

The SLAMTEC C1 lidar hardware has arrived. Next milestone is a **no-motion lidar bring-up** on `sphero-pi-2`: prove the C1 publishes `/scan` before mounting it, launching the RVR driver, or involving any motor-capable workflow.

Bench-test sequence:

1. Keep the RVR driver/TUI stopped; this test should not touch motors.
2. Plug the C1 USB adapter into `sphero-pi-2` and identify the device:
   ```bash
   ls -l /dev/serial/by-id/
   dmesg | tail -50
   ```
3. Build the SLAMTEC ROS 2 driver if it is not already present:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
   cd ~/ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install --packages-select rplidar_ros
   source install/setup.bash
   ```
4. Launch the C1 temporarily using the detected serial device, commonly `/dev/ttyUSB0`:
   ```bash
   ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyUSB0
   ```
5. Verify scan output from another shell:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 topic list | grep scan
   ros2 topic echo /scan --once
   ros2 topic hz /scan
   ```
6. Once the USB adapter identity is known, add a udev rule for stable `/dev/rplidar` and switch launches to:
   ```bash
   ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/rplidar
   ```

Expected C1 bring-up characteristics:

- ROS topic: `/scan`
- Message type: `sensor_msgs/msg/LaserScan`
- Typical scan rate: about `10 Hz`
- Serial baud: `460800`
- Frame: `laser` until the project adds measured `base_link -> laser` mounting geometry

After `/scan` is proven, add the repo integration slice: `rplidar` launch file, static `base_link -> laser` transform, RViz scan config, and then `slam_toolbox` mapping launch. Motor warning/approval is not needed for the bench `/scan` test, but it is required before launching the RVR driver, TUI, teleop, `/cmd_vel`, or any workflow that exposes drive controls.

### Phased plan

1. **Lidar bring-up**
   - Choose a ROS 2-supported USB lidar module.
   - Prefer a USB serial adapter kit over bare TTL UART so the RVR control UART remains isolated.
   - Add a stable udev alias such as `/dev/rplidar` instead of relying on `/dev/ttyUSB0` ordering.
   - Publish `/scan` and verify scan geometry in RViz with the robot stationary.
   - Add `base_link -> laser` static transform with measured mounting offsets.

2. **Manual mapping**
   - Run `slam_toolbox` online async mode.
   - Drive slowly with `rvr-console` or another teleop client.
   - Save the map with `nav2_map_server` once the room-level map is coherent.
   - Treat this phase as “make RViz draw the room as we drive,” not autonomy.

3. **Odometry and localization sanity**
   - Publish `/odom` from RVR encoder estimates if good enough.
   - If odometry is weak, keep speeds conservative and let lidar scan matching do more work.
   - Consider adding IMU/yaw and `robot_localization` if maps/localization drift too much.

4. **Autonomous navigation later**
   - Use a saved map with AMCL or `slam_toolbox` localization mode.
   - Add Nav2 only after `/cmd_vel`, stop/estop, stale-command timeout, `/scan`, tf, map save/load, and localization are reliable.
   - Tune conservatively: low speed, small robot footprint, inflation radius, cautious local planner, and explicit operator stop path.

### Autonomy prerequisites

Before allowing goal navigation:

- `/cmd_vel` behavior is predictable on the floor.
- stop, estop, stale-command timeout, and shutdown stop paths are regression-tested.
- lidar publishes stable `/scan` with correct orientation and frame.
- tf tree is valid: `map -> odom -> base_link -> laser`.
- odometry is not nonsense, even if imperfect.
- a manually built map can be saved and reloaded.
- localization can track the robot on that saved map.
- Nav2 is tuned for very low-speed indoor operation.

Autonomy is realistic, but it should come after manual SLAM. Nav2 is a later milestone, not the next task.

## Camera, object recognition, and semantic mapping roadmap

Goal: extend room mapping from geometry-only SLAM to a semantic map that can answer requests like “map the room and identify all of the objects and label them on the map.”

### Practical camera recommendation

The ideal single camera for accurate object-on-map placement is a depth/AI camera such as a Luxonis OAK-D Lite, because it provides RGB, stereo depth, and onboard neural inference. It is relatively expensive, so the first budget-conscious implementation should use a cheaper camera backend while keeping the software interface swappable.

Recommended staged hardware path:

| Option | Role | Tradeoff |
| --- | --- | --- |
| USB webcam with tripod thread | Cheapest useful first camera | Good for object inventory and approximate labels; no real depth. |
| Raspberry Pi Camera Module 3 | Cheap Pi-native camera | Cleaner integration than many webcams; no true depth and ribbon routing matters. |
| Used Intel RealSense D435/D435i | Midrange RGB-D option | Better object placement; more USB/power/ROS fuss. |
| Luxonis OAK-D Lite | Best semantic-mapping camera | RGB + stereo depth + onboard AI; cost is the downside. |

Start with a USB webcam or Pi Camera Module 3 if budget matters. Design the ROS messages and semantic mapper so a later RealSense/OAK-D upgrade only improves depth/object placement; it should not require rewriting the mapping workflow.

### Target semantic mapping graph

```text
RVR base driver
  publishes /odom or equivalent odometry estimate
  publishes tf: odom -> base_link

Lidar + slam_toolbox
  publishes /scan, /map, and tf: map -> odom

Camera backend
  publishes RGB image stream
  optionally publishes depth image / point cloud
  publishes/static tf: base_link -> camera_link

Object detector
  subscribes camera images
  publishes labels, confidences, bounding boxes, and optional depth/range estimates

Semantic mapper
  combines detections + camera calibration + tf + SLAM pose
  saves object labels in map coordinates
  publishes RViz markers / semantic map layer
```

### Label placement strategy

Object recognition alone is not enough to label a map precisely. Each label needs at least:

1. Detection label and confidence, such as `chair` at 0.86.
2. Robot pose in the map frame from SLAM.
3. Camera frame transform, such as `base_link -> camera_link`.
4. Camera bearing from the detection bounding box.
5. Depth/range estimate if available.

With a cheap non-depth camera, labels should be treated as approximate: “object seen near this robot pose and camera bearing.” With RealSense/OAK-D depth, labels can be projected more directly into `map` coordinates.

Example persisted semantic layer:

```yaml
objects:
  - label: chair
    confidence: 0.87
    map_position:
      x: 2.14
      y: -0.62
    depth_m: null
    observations: 5
```

### Mounting approach

Use a printable two-part mounting strategy:

1. **RVR payload deck**: a flat printed top plate with M3 holes, zip-tie slots, and optional 1/4-20 insert/GoPro-style adapter points.
2. **Sensor-specific brackets**: a raised lidar plate for clear 360° scan visibility and a forward camera bracket with slight downward pitch.

Suggested physical stack:

```text
      2D lidar, level, 360° clear
             |
      short printed riser
             |
  forward camera, slight downward angle
             |
      printed RVR payload deck
             |
           RVR body
```

Prefer PETG over PLA for durability/heat, M3 screws or heat-set inserts for repeatability, and zip-tie/Velcro slots for early sensor experiments.

### Implementation phases

1. **No-motion camera bring-up**
   - Add a camera backend launch for `usb_cam`/`v4l2_camera` or Pi camera.
   - Publish RGB images and camera info.
   - Add `base_link -> camera_link` static transform.
   - Verify image stream and TF without launching motor-capable workflows.

2. **Detector bring-up**
   - Start with a lightweight detector such as a YOLO-nano/MobileNet-class model.
   - Publish a stable detector output independent of the camera backend.
   - Add fake detections for tests and demos without camera hardware.

3. **Approximate semantic labels**
   - Attach detections to the current SLAM pose and camera bearing.
   - Save a semantic object layer alongside the occupancy map.
   - Publish RViz markers for label visualization.

4. **Depth upgrade path**
   - Add RealSense or OAK-D backend when budget allows.
   - Fill `depth_m` for detections and project labels into the map more accurately.
   - Keep the semantic mapper API stable.

5. **AI workflow integration**
   - Add `semantic_map_room` as a deterministic allowlisted workflow.
   - Preflight lidar, camera, detector, tf, SLAM, map saver, and stop/estop.
   - Require the existing explicit motor-capable approval before exposing teleop or driving controls.
   - Save and verify both the occupancy map and semantic object layer.

## AI command layer roadmap

Goal: support high-level natural-language requests such as “map the room” without letting an LLM directly drive motors or bypass safety gates.

### Design principle

The AI layer should be an **intent planner/orchestrator**, not a raw robot controller. It may choose from a small set of named, prebuilt robot workflows. It should not publish arbitrary `/cmd_vel`, invent ROS commands, or open the RVR UART.

Good shape:

```text
User: “map the room”
  -> AI parses intent: create_room_map
  -> confirms motor-capable workflow and safety preconditions
  -> calls a deterministic ROS workflow/action
  -> workflow launches lidar + SLAM + teleop/map-save helpers
  -> operator manually drives or approves bounded autonomous behavior
  -> workflow verifies map artifact and reports result
```

Bad shape:

```text
User: “map the room”
  -> LLM writes/publishes arbitrary /cmd_vel
  -> LLM edits launch files live
  -> LLM decides when the robot is safe
```

### Initial command vocabulary

Start with a small allowlist of robot skills/actions:

- `map_the_room`: start lidar, start `slam_toolbox`, start/manual-drive workflow, save map when complete.
- `show_lidar`: launch lidar driver and RViz/scan verification without movement.
- `save_current_map`: call map saver for the active SLAM session.
- `semantic_map_room`: start lidar, SLAM, camera, object detector, and semantic label collection; save occupancy map plus object layer.
- `localize_on_map`: load a saved map and verify localization.
- `stop_robot`: publish zero velocity and/or call the validated stop path.
- `emergency_stop`: call estop and block further motion until explicit clear.
- Later: `go_to_named_place`, only after Nav2/localization are proven.

### “Map the room” target behavior

A first safe implementation should do this:

1. Resolve the requested map name/location and output path.
2. Check prerequisites:
   - RVR driver reachable and healthy.
   - lidar `/scan` active or launchable.
   - tf tree contains `odom -> base_link -> laser`.
   - `slam_toolbox` available.
   - stop/estop paths reachable.
3. Warn clearly that this is motor-capable if it will launch teleop or expose drive controls.
4. Start the mapping launch stack.
5. Put the robot in **manual mapping mode** first: the operator drives slowly while SLAM builds the map.
6. On user command or completion, save the map with `nav2_map_server`.
7. Verify `.yaml` + `.pgm`/`.png` map files exist and report the saved path.

### Interfaces to consider

- ROS 2 action server: `MapRoom.action` for deterministic workflow state/progress/cancel.
- A narrow Hermes/CLI wrapper: maps natural-language commands to allowlisted ROS actions.
- Voice/Discord command later, but only through the same allowlisted action layer.
- Status feedback: current phase, health checks, active topics, saved map path, and explicit stop/estop options.

### Safety boundaries

- AI commands that can start motors must require the same explicit motor warning/approval discipline as TUI/teleop.
- The AI layer may request `stop_robot`/`emergency_stop` immediately, but all other motor-capable workflows require approval.
- Keep all continuous motion inside ROS/Nav2/controllers with watchdogs; do not put LLMs in a control loop.
- Prefer bounded workflows with cancellation, timeout, and artifact verification over open-ended “drive around until done.”
