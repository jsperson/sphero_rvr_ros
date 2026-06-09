# sphero_rvr_ros

Concurrency-safe Sphero RVR core driver plus a ROS 2 adapter package.

This project is intentionally starting fresh from the older MCP implementation. The MCP repo remains useful as a protocol reference, but this repo is built around ROS 2 needs: one serial owner, request/response dispatching, safety preemption, and continuous velocity control.

## Current base-driver status

Implemented and hardware-smoked on a Raspberry Pi 5 running Ubuntu Server 24.04 + ROS 2 Jazzy:

- `/cmd_vel` subscriber using `geometry_msgs/msg/Twist`
- `stop`, `estop`, and `clear_estop` services using `std_srvs/srv/Trigger`
- `battery_state` publisher using `sensor_msgs/msg/BatteryState`
- `diagnostics` publisher using `diagnostic_msgs/msg/DiagnosticArray`
- conservative safety defaults in `config/rvr.yaml`:
  - serial port: `/dev/ttyAMA0`
  - max linear: `0.25 m/s`
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
# cd sphero_rvr_ros && git pull --ff-only && cd ..

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select sphero_rvr_driver
source install/setup.bash
```

Verify the package:

```bash
ros2 pkg prefix sphero_rvr_driver
ros2 pkg executables sphero_rvr_driver
python3 - <<'PY'
from sphero_rvr_driver.rvr_node import RVRNodeConfig
print(RVRNodeConfig())
PY
```

Expected:

```text
/home/jsperson/ros2_ws/install/sphero_rvr_driver
sphero_rvr_driver rvr_node
RVRNodeConfig(serial_port='/dev/ttyAMA0', ...)
```

## No-motion smoke test

Power on the RVR, then launch the driver:

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
ros2 service list | grep -E 'stop|estop|clear_estop'
ros2 topic echo /diagnostics --once
ros2 topic echo /battery_state --once
ros2 service call /stop std_srvs/srv/Trigger {}
```

Expected topics:

```text
/battery_state
/cmd_vel
/diagnostics
/parameter_events
/rosout
```

Expected services:

```text
/clear_estop
/estop
/stop
```

A successful no-motion smoke should show diagnostics connected, battery voltage/percentage, and a successful stop response.

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

The wrapper sources ROS, sources the workspace, starts the driver launch if needed, verifies required topics/services, starts the TUI, and calls `/stop` on exit.

TUI behavior:

- Talks to the robot only through ROS 2 topics/services:
  - publishes `/cmd_vel`
  - calls `/stop`, `/estop`, `/clear_estop`
  - subscribes `/battery_state`, `/diagnostics`
- Does not import `RVRDriver` or open `/dev/ttyAMA0`; the ROS driver node remains the only UART owner.
- Starts disarmed. Non-zero drive keys do nothing until the user explicitly arms the TUI.
- **Motion arming is disabled by default after unsafe queued-motion behavior was observed.** `/arm` refuses to enable keyboard drive unless `RVR_TUI_ENABLE_MOTION=1` is set. Leave it disabled until the control/stop path is fixed and revalidated.
- Supports keyboard driving with arrow keys and/or WASD, plus space for stop.
- Supports slash commands: `/battery`, `/status`, `/speed <mps>`, `/turn <rad_s>`, `/stop`, `/estop`, `/clear-estop`, `/arm`, `/disarm`, `/help`, and `/quit`.
- Stops on key timeout, quit, crash, or Ctrl+C.
- Logs startup, driver launch, topic/service verification, and cleanup details to `~/.local/state/sphero_rvr/rvr-console.log`; driver output goes to `~/.local/state/sphero_rvr/rvr-driver.log`.

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
