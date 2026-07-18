# sphero_rvr_ros

Concurrency-safe Sphero RVR core driver plus a ROS 2 adapter package.

This project is intentionally starting fresh from the older MCP implementation. The MCP repo remains useful as a protocol reference, but this repo is built around ROS 2 needs: one serial owner, request/response dispatching, safety preemption, and continuous velocity control.

## Documentation map for operators and maintainers

- [docs/rvr_capability_matrix.md](docs/rvr_capability_matrix.md) is the full official SDK/protocol capability matrix. It is the source of truth for core API parity, ROS exposure decisions, and required validation tokens.
- [docs/rvr_ros_exposure_policy.md](docs/rvr_ros_exposure_policy.md) explains the ROS exposure policy: full API parity belongs in `sphero_rvr_core`; ROS exposes only typed, bounded, operational surfaces.
- [docs/rvr_api_gap_report.md](docs/rvr_api_gap_report.md) is the parity handoff: what is implemented, what remains intentionally core-only/omitted, and which commands validate the state.
- [docs/rvr_odometry_tf_design.md](docs/rvr_odometry_tf_design.md) documents the current encoder-derived `/odom` and `odom -> base_link` TF design and limitations.
- [docs/mapping.md](docs/mapping.md) covers the lidar/SLAM launch scaffold, safe defaults, TF expectations, and manual mapping workflow.
- [docs/rosbag_capture_replay.md](docs/rosbag_capture_replay.md) covers the dry-run-first rosbag2 capture/replay workflow, run manifest format, storage layout, cleanup, and motor-topic safety boundaries.
- [docs/camera_lidar_calibration.md](docs/camera_lidar_calibration.md) is the no-hardware Pi Camera 3 / RPLIDAR calibration runbook, including measured camera intrinsics, measured TF defaults, and physical measurement steps.
- [docs/rvr_control_interface_plan.md](docs/rvr_control_interface_plan.md) defines the safer `rvr-console` / curses TUI control interface for lidar mapping: status pane, STOP/ESTOP semantics, mapping launch states, nudge commands, dry-run mode, and validation gates.
- [docs/lidar_collision_stop_supervisor.md](docs/lidar_collision_stop_supervisor.md) is the source-of-truth design for the independent lidar collision-stop supervisor and final `/cmd_vel` arbitration contract.
- [docs/motion_calibration.md](docs/motion_calibration.md) records the gated motion/odometry calibration helper and current encoder scale.
- [docs/udev/99-rplidar.rules](docs/udev/99-rplidar.rules) is the Pi udev rule for the stable `/dev/rplidar` alias.

Installed package data includes launch: `rvr.launch.py`, `supervised_rvr.launch.py`, `lidar.launch.py`, `mapping.launch.py`, `camera.launch.py`; config: `rvr.yaml`, `collision_stop.yaml`, `lidar.yaml`, `slam_toolbox.yaml`, `camera.yaml`; helper scripts: `install-rvr-pi`, `rvr-camera-node`, `rvr-console`, `rvr_motion_calibration.py`.

## Current base-driver status

Hardware-smoked on a Raspberry Pi 5 running Ubuntu Server 24.04 + ROS 2 Jazzy:

- `/cmd_vel` subscriber using `geometry_msgs/msg/Twist`
- `stop`, `estop`, and `clear_estop` services using `std_srvs/srv/Trigger`
- `battery_state` publisher using `sensor_msgs/msg/BatteryState`
- `diagnostics` publisher using `diagnostic_msgs/msg/DiagnosticArray`

Implemented locally with ROS-free fake/unit coverage and pending Pi/ROS validation:

- `reset_yaw`, `reset_locator`, and `release_led_requests` services using `std_srvs/srv/Trigger`
- `set_all_leds` subscriber using `std_msgs/msg/ColorRGBA` for bounded operator feedback LEDs
- `left_motor_temperature` and `right_motor_temperature` publishers using `sensor_msgs/msg/Temperature`
- `ambient_light` publisher using `sensor_msgs/msg/Illuminance`
- `odom` publisher using `nav_msgs/msg/Odometry`, plus `odom -> base_link` TF from encoder-count deltas
  - design/limitations: [docs/rvr_odometry_tf_design.md](docs/rvr_odometry_tf_design.md)
- conservative safety defaults in `config/rvr.yaml`:
  - serial port: `/dev/ttyAMA0`
  - max linear: `0.10 m/s`
  - max angular: `0.4 rad/s`
  - raw motor duty cap: `64`
  - stale `/cmd_vel` timeout: `0.5s`

Validated ROS 2 path:

```text
ros2 topic pub /cmd_vel
  -> sphero_rvr_driver node
  -> RVRDriver.set_velocity()
  -> UART /dev/ttyAMA0
  -> RVR motors
```

## SLAMTEC RPLIDAR C1 lidar

The Pi is configured to expose the SLAMTEC C1 USB adapter as a stable serial alias:

```text
/dev/rplidar -> /dev/ttyUSB0
```

The C1 requires the newer upstream `rplidar_ros` ROS 2 branch; the Ubuntu apt package can identify the device but failed to start scanning on this module. The working path is:

```bash
cd ~/ros2_ws
vcs import src < src/sphero_rvr_ros/workspace.repos
colcon build --symlink-install --packages-select rplidar_ros sphero_rvr_driver
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sphero_rvr_driver lidar.launch.py
```

The `/dev/rplidar` alias is documented in `docs/udev/99-rplidar.rules`. Install it on the Pi with:

```bash
sudo cp ~/ros2_ws/src/sphero_rvr_ros/docs/udev/99-rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger /dev/ttyUSB0 || true
ls -l /dev/rplidar /dev/ttyUSB0 /dev/serial/by-id/*
```

Verified C1 settings:

```text
serial_port: /dev/rplidar
serial_baudrate: 460800
frame_id: laser
scan_mode: Standard
scan topic: /scan
```

`lidar.launch.py` also publishes a static `base_link -> laser` transform. Measure the final mount and override the defaults as needed:

```bash
ros2 launch sphero_rvr_driver lidar.launch.py laser_x:=0.0 laser_y:=0.0 laser_z:=0.15 laser_yaw:=0.0
```

This launch file is lidar-only and does not talk to the RVR motors. Full mapping still needs odometry / TF integration before `slam_toolbox` should be treated as a real robot map.

## Mapping scaffold

A conservative SLAM Toolbox scaffold is available after lidar + odometry bring-up:

```bash
ros2 launch sphero_rvr_driver mapping.launch.py
```

By default this starts lidar + SLAM only and **does not** start the live RVR driver:

```text
start_rvr:=false
```

Full live mapping is motor-capable and now uses the lidar collision-stop supervisor by default:

```text
/cmd_vel -> lidar_collision_stop_supervisor -> /cmd_vel_motor -> sphero_rvr_driver
```

Ordinary publishers keep targeting `/cmd_vel`; the live driver is remapped away from public `/cmd_vel` to `/cmd_vel_motor` in `supervised_rvr.launch.py`, and the supervisor owns public `/stop`, `/estop`, and `/clear_estop`.

```bash
ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=true
```

See `docs/mapping.md` and `docs/lidar_collision_stop_supervisor.md` before running live mapping.

## Roadmap: lidar, SLAM, cameras, autonomy, and AI commands

Near-term roadmap after the base driver/TUI is stable:

1. **Lidar + scan visualization** — add a ROS 2-supported USB 2D lidar, publish `/scan`, add `base_link -> laser`, and verify in RViz while stationary.
2. **Manual SLAM mapping** — run `slam_toolbox`, drive slowly with teleop/TUI, and save room maps with `nav2_map_server`.
3. **Camera + object recognition** — start with a cheap USB webcam or Raspberry Pi Camera Module 3 for object inventory; keep the software interface swappable for a later depth camera such as a used RealSense D435 or Luxonis OAK-D Lite.
4. **Semantic map labels** — combine object detections with robot pose, camera bearing, and later depth estimates to place labels on the saved map.
5. **Localization + cautious Nav2** — only after map save/load, odometry/tf, stop/estop, stale-command timeout, and localization are reliable.
6. **AI command layer** — support high-level requests such as “map the room” or “map the room and identify objects” by mapping natural language to allowlisted deterministic ROS workflows/actions. The LLM must not directly publish arbitrary `/cmd_vel` or bypass motor safety gates.

Target AI shape:

```text
User: “map the room”
  -> AI parses intent: map_the_room
  -> checks prerequisites and safety gates
  -> calls deterministic ROS workflow/action
  -> workflow launches lidar + SLAM + manual mapping helpers
  -> map files are saved, verified, and reported

User: “map the room and identify all objects”
  -> AI parses intent: semantic_map_room
  -> checks lidar, camera, detector, tf, SLAM, and safety gates
  -> workflow launches lidar + SLAM + camera + object detector
  -> operator manually drives the mapping route
  -> occupancy map + semantic object layer are saved and verified
```

Detailed planning lives in [STATUS.md](STATUS.md#lidar-slam-and-autonomy-roadmap), [STATUS.md](STATUS.md#camera-object-recognition-and-semantic-mapping-roadmap), and [STATUS.md](STATUS.md#ai-command-layer-roadmap).

## Safe ROS operational surface notes

The ROS adapter deliberately exposes only the safe subset selected in `docs/rvr_ros_exposure_policy.md`:

- routine motion stays on ordinary `/cmd_vel`, then passes through `lidar_collision_stop_supervisor` before the final `/cmd_vel_motor` driver sink in supervised motor-capable launches;
- read-only telemetry is published as typed topics (`battery_state`, motor temperatures, `ambient_light`, `odom`) and diagnostics key-values;
- `reset_yaw` and `reset_locator` are explicit reference-frame reset services, not hidden side effects;
- LEDs are limited to bounded `ColorRGBA` all-LED feedback plus `release_led_requests`; raw LED masks/palettes remain core-only;
- raw motors, firmware/admin/update/factory operations, calibration flows, opaque streaming bytes, and identifier publishing remain out of the default ROS graph.

ROS-free validation on development hosts:

```bash
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests/test_ros_safe_surfaces.py tests/test_ros_node_config.py tests/test_diagnostics.py -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m compileall -q src
```

ROS-environment validation on the Pi after sourcing ROS and the workspace:

```bash
colcon build --symlink-install --packages-select sphero_rvr_driver
source install/setup.bash
ros2 launch sphero_rvr_driver rvr.launch.py
ros2 topic list | grep -E 'cmd_vel|battery_state|diagnostics|ambient_light|odom|set_all_leds'
ros2 service list | grep -E 'stop|estop|clear_estop|reset_yaw|reset_locator|release_led_requests'
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Per `STATUS.md`, get explicit approval before launching anything that can talk to the live RVR driver because the node owns motor-capable command paths. Tiny robot, still a robot.

## API parity validation checklist

Use these gates to validate the RVR API parity surface without blurring fake/unit tests, ROS environment checks, and live hardware smoke.

### macOS / no-ROS fake validation

These commands are safe on development hosts because they use fake transports and do not open the RVR UART. They are the default validation path for code and documentation changes on macOS:

```bash
python3 -m venv /tmp/sphero-rvr-ros-test
/tmp/sphero-rvr-ros-test/bin/python -m pip install -e '.[dev]'

PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest \
  tests/test_missing_command_builders.py \
  tests/test_response_parsers.py \
  tests/test_dispatcher.py \
  tests/test_driver_capability_coverage.py -q

PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest \
  tests/test_ros_safe_surfaces.py \
  tests/test_ros_node_config.py \
  tests/test_diagnostics.py -q

PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m compileall -q src
git diff --check
```

Expected coverage gates:

- every capability matrix row has an explicit `Test status` token (`builder-test`, `parser-test`, `driver-test`, `notification-test`, `ros-exposure-test`, `fake-transport-test`, or `documented-omission`);
- packet builders, parsers, driver methods, dispatcher notification routes, fake transport fixtures, and ROS-safe node/config surfaces have fake/unit coverage;
- documented protocol mismatches and unsafe/admin/ROS omissions stay classified instead of becoming silent gaps.

### Pi / ROS environment no-motion validation

Run this on `sphero-pi-2` after sourcing ROS and rebuilding the workspace. This verifies install/package wiring and importable ROS surfaces; it does not launch the motor-capable driver and does not publish motion commands.

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

If the driver is launched for ROS graph inspection, treat it as live hardware access and apply the warning/approval rule below first, even when the intended check is “no motion.” The node owns `/dev/ttyAMA0` and has motor-capable command paths.

### Live hardware smoke gate

Live motor-capable validation remains opt-in. Before running `rvr-console`, `ros2 launch sphero_rvr_driver rvr.launch.py`, `/stop`, `/cmd_vel`, or any command that talks to the live driver, explicitly warn:

```text
WARNING: this can start the RVR motors
```

Only after approval, keep the robot suspended or restrained for the first run and scope the smoke narrowly:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sphero_rvr_driver rvr.launch.py

# In a second approved shell:
ros2 topic echo /battery_state --once
ros2 topic echo /diagnostics --once
ros2 topic echo /odom --once
ros2 service call /stop std_srvs/srv/Trigger {}
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
ros2 service call /stop std_srvs/srv/Trigger {}
```

Anything beyond that smoke — sustained driving, mapping, TUI operation, or autonomy — needs a separate explicit scope and approval.

## Recommended hardware/software

- Raspberry Pi 5, 64-bit
- 64 GB microSD or larger; A2-rated card or SSD preferred
- Ubuntu Server 24.04 LTS 64-bit
- ROS 2 Jazzy
- Sphero RVR connected to the Pi UART exposed as `/dev/ttyAMA0`

Raspberry Pi OS can work with containers/source builds, but this repo's documented path assumes Ubuntu 24.04 because ROS 2 Jazzy has clean binary packages there.

## Full Raspberry Pi install

### 1. Flash Ubuntu Server

Use Raspberry Pi Imager:

```text
Device: Raspberry Pi 5
OS:     Ubuntu Server 24.04 LTS (64-bit)
Storage: target microSD/SSD
```

Advanced settings used for the tested install:

```text
hostname: sphero-pi-2
username: jsperson
SSH:      enabled
Wi-Fi:    configured, unless using Ethernet
```

Boot the Pi, then verify from your workstation:

```bash
ssh jsperson@sphero-pi-2.local
cat /etc/os-release
uname -m
```

Expected:

```text
Ubuntu 24.04.x LTS
aarch64
```

### 2. Optional: enable passwordless sudo

This is convenient for robot bring-up, but it is a local security tradeoff. Use only on a trusted robot LAN/tailnet.

```bash
echo 'jsperson ALL=(ALL) NOPASSWD:ALL' | sudo tee /etc/sudoers.d/010-jsperson-nopasswd
sudo chmod 0440 /etc/sudoers.d/010-jsperson-nopasswd
sudo visudo -cf /etc/sudoers.d/010-jsperson-nopasswd
sudo -k
sudo -n true && echo "passwordless sudo OK"
```

### 3. Optional: install Tailscale

Tailscale is useful for reliable SSH when the robot changes networks.

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --ssh --hostname=sphero-pi-2
```

Open the printed `https://login.tailscale.com/a/...` URL on another device and approve the Pi. Verify:

```bash
tailscale status
tailscale ip -4
systemctl is-enabled tailscaled
systemctl is-active tailscaled
```

Then you can SSH through the tailnet hostname:

```bash
ssh jsperson@sphero-pi-2
```

### 4. Fix locale and apt sources

The tested Ubuntu Pi image had `noble-updates` packages installed, but `noble-updates` was missing from apt sources. Without this, ROS 2 dependencies can fail with exact-version conflicts such as `liblz4-dev` vs `liblz4-1`.

Run:

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl gnupg lsb-release git python3-pip python3-venv
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
sudo add-apt-repository universe -y
```

Ensure the Ubuntu source includes updates/backports:

```bash
sudo cp /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak.$(date +%Y%m%d%H%M%S)
sudo python3 - <<'PY'
from pathlib import Path
p = Path('/etc/apt/sources.list.d/ubuntu.sources')
s = p.read_text()
if 'Suites: noble noble-updates' not in s:
    s = s.replace(
        'Suites: noble\nComponents: main restricted universe multiverse',
        'Suites: noble noble-updates noble-backports\nComponents: main restricted universe multiverse',
        1,
    )
p.write_text(s)
PY
sudo apt update
```

### 5. Install ROS 2 Jazzy

Add the ROS 2 apt repository:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update
```

Install ROS and build tools:

```bash
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-map-server \
  libcamera-ipa \
  libcamera-tools \
  v4l-utils \
  build-essential \
  libyaml-dev \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete
```

Initialize rosdep and source ROS in future shells:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update

grep -q 'source /opt/ros/jazzy/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

source /opt/ros/jazzy/setup.bash
ros2 pkg prefix rclpy
```

Expected final command output:

```text
/opt/ros/jazzy
```

### 6. Configure the RVR UART

On the tested Pi, the RVR UART appears as `/dev/ttyAMA0`. The default Ubuntu serial console also tried to own it, so disable the getty and make the device available to the `dialout` group.

```bash
sudo systemctl disable --now serial-getty@ttyAMA0.service

printf 'KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"\n' \
  | sudo tee /etc/udev/rules.d/99-rvr-uart.rules >/dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger /dev/ttyAMA0 || true
sudo chgrp dialout /dev/ttyAMA0
sudo chmod 660 /dev/ttyAMA0
```

Verify:

```bash
ls -l /dev/ttyAMA0
id
```

Expected:

```text
crw-rw---- 1 root dialout ... /dev/ttyAMA0
```

The user should be in `dialout`.

### 7. Clone and build the ROS driver

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/jsperson/sphero_rvr_ros.git
# Or, if already cloned:
# cd sphero_rvr_ros && git pull --ff-only

~/ros2_ws/src/sphero_rvr_ros/scripts/install-rvr-pi
```

The install helper installs the ROS apt repository if needed, installs all apt/runtime and source-build dependencies, builds Raspberry Pi's PiSP-capable `libcamera` in release mode under `~/.local/rpi-libcamera`, imports pinned `camera_ros` plus upstream `rplidar_ros` from `workspace.repos`, and builds `camera_ros` against that exact libcamera installation. It then runs `rosdep`, builds the RVR workspace, installs udev rules, and verifies the lidar/mapping launch files, map saver, and camera discovery tools. The known-good libcamera and `camera_ros` revisions are pinned for reproducibility; set `RPI_LIBCAMERA_REF` only when deliberately validating a newer compatible pair. Set `RVR_ROS_WS` if the ROS workspace is not `~/ros2_ws`; both the installer and `rvr-camera-node` honor it.

Base ROS/lidar-only fallback commands, if you are deliberately not using the helper. These do not reproduce the pinned PiSP camera build; use the helper for Camera Module 3 support:

```bash
cd ~/ros2_ws/src
vcs import . < sphero_rvr_ros/workspace.repos

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rplidar_ros sphero_rvr_driver
source install/setup.bash
```

Verify the package:

```bash
ros2 pkg prefix sphero_rvr_driver
ros2 pkg executables sphero_rvr_driver
ros2 launch sphero_rvr_driver lidar.launch.py --show-args
ros2 launch sphero_rvr_driver mapping.launch.py --show-args
ros2 launch sphero_rvr_driver camera.launch.py --show-args
PATH="$HOME/.local/rpi-libcamera/bin:$PATH" \
LD_LIBRARY_PATH="$HOME/.local/rpi-libcamera/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  cam --list
v4l2-ctl --list-devices
python3 - <<'PY'
from sphero_rvr_driver.rvr_node import RVRNodeConfig
print(RVRNodeConfig())
PY
```

Expected:

```text
/home/jsperson/ros2_ws/install/sphero_rvr_driver
sphero_rvr_driver rvr_node
sphero_rvr_driver rvr_tui
RVRNodeConfig(serial_port='/dev/ttyAMA0', ...)
```

Pi Camera 3 no-motion sanity check:

```bash
PATH="$HOME/.local/rpi-libcamera/bin:$PATH" \
LD_LIBRARY_PATH="$HOME/.local/rpi-libcamera/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  cam --list
v4l2-ctl --list-devices
"$(ros2 pkg prefix sphero_rvr_driver)/share/sphero_rvr_driver/scripts/rvr-camera-node" \
  --ros-args -p width:=640 -p height:=480
```

Expected: the PiSP-capable libcamera build lists the Pi Camera 3 `imx708` sensor, V4L2 lists the `rp1-cfe`/`pispbe` video devices, and `rvr-camera-node` publishes `/camera_node/image_raw` through ROS. Camera checks are safe/no-motor; they do not launch the RVR driver or publish `/cmd_vel`.

Pi Camera 3 calibration defaults are measured for the current payload. `camera.launch.py` exposes `camera_info_url` and `base_link -> camera_link -> camera_optical_frame` TF inputs; the default `camera_info_url` points at the robot-local file `file:///home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml`. That generated CameraInfo file is an operational dependency, not committed source: install/restore it on the Pi, keep a backup, and verify its checksum and `/camera_node/camera_info` contents before semantic localization.

Current accepted rack layout:

- Use the lightweight one-level rack with a narrow lidar tower.
- Keep the robot payload to the Pi 5, Pi battery, lidar, and Pi Camera 3.
- Do not use the old three-level rack for floor driving; it put too much weight/high center of gravity on the RVR and caused weak turning/drive behavior.
- Live `rvr-console` testing with the one-level rack ran normally, confirming the rack redesign fixed the weight/CG issue.

Installed package data includes:

- launch: `rvr.launch.py`, `lidar.launch.py`, `mapping.launch.py`, `camera.launch.py`
- config: `rvr.yaml`, `lidar.yaml`, `slam_toolbox.yaml`, `camera.yaml`
- docs: `mapping.md`, `motion_calibration.md`, `rosbag_capture_replay.md`, `camera_lidar_calibration.md`, `udev/99-rplidar.rules`
- helper scripts: `install-rvr-pi`, `rvr-camera-node`, `rvr-console`, `rvr_motion_calibration.py`

## No-motion smoke test

This is a live-hardware smoke because it launches the driver against the RVR UART. Before running it, explicitly warn the operator and get approval:

```text
WARNING: this can start the RVR motors
```

After approval, power on the RVR, keep it restrained/suspended for first bring-up, then launch the driver:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sphero_rvr_driver rvr.launch.py
```

In another shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic list
ros2 service list | grep -E 'stop|estop|clear_estop|reset_yaw|reset_locator|release_led_requests'
ros2 topic echo /diagnostics --once
ros2 topic echo /battery_state --once
ros2 topic echo /ambient_light --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 service call /stop std_srvs/srv/Trigger {}
```

Expected topics:

```text
/battery_state
/cmd_vel
/diagnostics
/ambient_light
/left_motor_temperature
/right_motor_temperature
/odom
/parameter_events
/rosout
/set_all_leds
```

Expected services:

```text
/clear_estop
/estop
/release_led_requests
/reset_locator
/reset_yaw
/stop
```

A successful no-motion smoke should show diagnostics connected, battery voltage/percentage, typed sensor topics publishing when the RVR responds, odometry/TF available from encoder polling, and a successful stop response.

## First motor test

⚠️ **Motor warning:** publishing non-zero `/cmd_vel` starts the RVR motors. Keep the robot suspended/restrained for the first validation run.

With the driver launch still running, publish one conservative command:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
sleep 1
ros2 service call /stop std_srvs/srv/Trigger {}
```

The driver also has a stale-command timeout, but call `/stop` afterward anyway. Belt, suspenders, and robot treads.

## Terminal control app

The driver package includes a small text user interface so operators do not have to remember raw ROS commands during normal bring-up and testing.

Run the TUI from an already-sourced ROS shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run sphero_rvr_driver rvr_tui
```

Or use the startup wrapper from the repo checkout:

```bash
~/ros2_ws/src/sphero_rvr_ros/scripts/rvr-console
```

The wrapper sources ROS, sources the workspace, starts the driver launch if needed, verifies required topics/services, starts the TUI, and logs cleanup on exit. It does not call the ROS `/stop` service during shell cleanup; the TUI publishes zero `/cmd_vel` on exit, and the driver shutdown path sends the validated raw-motor-off packet.

To exercise the console without ROS, lidar, UART, or RVR hardware, use dry-run mode:

```bash
~/ros2_ws/src/sphero_rvr_ros/scripts/rvr-console --dry-run
```

Dry-run mode starts the same TUI against an in-process fake client. The status pane is labeled `DRY-RUN`, simulated battery/odom/scan/service data are shown as fake/fresh, STOP/ESTOP responses are simulated, and drive keys append intended `DRY-RUN cmd_vel` commands to the TUI history instead of publishing to a live ROS `/cmd_vel` topic.

TUI behavior:

- Talks to the robot only through ROS 2 topics/services:
  - publishes `/cmd_vel`
  - calls `/stop`, `/estop`, `/clear_estop`
  - subscribes `/battery_state`, `/diagnostics`, `/odom`, and `/scan`
- Supports `--dry-run` from `rvr-console` for ROS-free UI testing with fake battery, odom, scan, STOP/ESTOP, and command-publish logs.
- Does not import `RVRDriver` or open `/dev/ttyAMA0`; the ROS driver node remains the only UART owner.
- Starts disarmed. Non-zero drive keys do nothing until the user explicitly arms the TUI.
- Supports keyboard driving with arrow keys and/or WASD, plus space for stop.
- Supports slash commands: `/battery`, `/status`, `/speed <mps>`, `/turn <rad_s>`, `/stop`, `/estop`, `/clear-estop`, `/arm`, `/disarm`, `/help`, and `/quit`.
- Manages lidar/mapping launch processes from inside the TUI:
  - `/lidar start` launches `lidar.launch.py` only; no RVR driver and no motor path.
  - `/lidar stop` stops the TUI-owned lidar launch process.
  - `/mapping start` launches `mapping.launch.py start_rvr:=false` for lidar + SLAM checks without the RVR driver.
  - `/mapping stop` publishes zero velocity, disarms the TUI, and stops the TUI-owned launch process.
  - `/mapping full` only displays `WARNING: this can start the RVR motors` and the required confirmation form.
  - `/mapping full confirm` launches `mapping.launch.py start_rvr:=true`; this is motor-capable and leaves the TUI disarmed until `/arm confirm`.
- Supports calibrated fixed-distance nudges for early mapping validation:
  - `/nudge forward <distance>` and `/nudge back <distance>` warn only and do not publish motion.
  - `/nudge forward <distance> confirm` and `/nudge back <distance> confirm` require the TUI to already be armed, cap distance at 6 inches, publish a bounded fixed-duration velocity, then always publish zero velocity and disarm.
  - Distance accepts meters by default plus `m`, `in`, `inch`, or `inches` suffixes, for example `/nudge forward 0.02 confirm` or `/nudge back 3in confirm`.
  - The initial calibration uses the 2026-06-24 measured `linear=0.05` for `duration=1.00` movement of about 9 inches, with `odom_counts_per_meter: 4337.768` for expected encoder-count logging.
- Saves the current SLAM map with `/map save <name>`:
  - map names are sanitized into safe filename stems;
  - files are written under `~/maps/` as `<safe-name>.yaml` and `<safe-name>.pgm`;
  - live mode runs `ros2 run nav2_map_server map_saver_cli -f ~/maps/<safe-name>`;
  - dry-run mode logs the intended save path without running `ros2`.
- `rvr-console --dry-run` starts the fake TUI path without sourcing ROS, launching ROS processes, opening `/dev/ttyAMA0`, or touching `/dev/rplidar`; it can exercise `/lidar ...` and `/mapping ...` state transitions safely.
- Stops on key timeout, quit, crash, or Ctrl+C.
- Logs startup, driver launch, topic/service verification, and cleanup details to `~/.local/state/sphero_rvr/rvr-console.log`; driver output goes to `~/.local/state/sphero_rvr/rvr-driver.log`.

### TUI calibrated nudge hardware validation

Do not run this validation without explicit approval after showing:

```text
WARNING: this can start the RVR motors
```

After approval, keep the RVR restrained/suspended for the first run or place it in a clear controlled area. Start with dry-run, then validate the live graph, then try the smallest nudge:

```bash
# Safe fake path first: no ROS, UART, lidar, or live /cmd_vel.
~/ros2_ws/src/sphero_rvr_ros/scripts/rvr-console --dry-run
# In the TUI: /mapping full confirm, /arm confirm, /nudge forward 0.02 confirm

# Live motor-capable path only after approval and physical safety setup.
~/ros2_ws/src/sphero_rvr_ros/scripts/rvr-console
# In the TUI:
#   /mapping full confirm
#   /arm confirm
#   /nudge forward 0.02 confirm
#   /status
```

Expected result for the live tiny nudge: the TUI publishes one bounded forward command, publishes zero velocity at completion, disarms, and logs the requested distance, estimated duration, expected encoder counts, and stopped/disarmed result. If anything looks wrong, use `/stop`, then `/estop`, then power off the RVR if needed. Do not test larger nudges until the 0.02 m run has stopped cleanly and odometry is plausible.

Suggested one-command install convenience:

```bash
mkdir -p ~/.local/bin
ln -sf ~/ros2_ws/src/sphero_rvr_ros/scripts/rvr-console ~/.local/bin/rvr-console
```

Then run:

```bash
rvr-console
```

## Development install without ROS 2

For core-driver development on a non-ROS machine:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
python -m pytest -q
```

## Troubleshooting

### `rosdep` cannot resolve package keys

Use ROS/Ubuntu package names in `package.xml`; this repo currently depends on `python3-serial`, not PyPI-only `pyserial`, for the ROS install path.

### `colcon build` fails parsing setup metadata

Keep package metadata in `setup.py` for `ament_python`. A rich PEP 621 `[project]` table in `pyproject.toml` can confuse `colcon`/`setuptools` on Ubuntu 24.04 because generated setup metadata may include non-literal objects.

### `/dev/ttyAMA0` is busy

Check for the serial console:

```bash
systemctl is-active serial-getty@ttyAMA0.service || true
sudo fuser -v /dev/ttyAMA0 || true
```

Disable it:

```bash
sudo systemctl disable --now serial-getty@ttyAMA0.service
```

### Battery query times out

Check the basics before debugging packets:

- RVR is powered on
- UART wiring is connected
- `/dev/ttyAMA0` exists
- `jsperson` is in `dialout`
- no process is holding `/dev/ttyAMA0`

### Stop the driver

If launched in the foreground, press `Ctrl+C`. If a process is holding the UART:

```bash
sudo fuser -v /dev/ttyAMA0
```
