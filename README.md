# sphero_rvr_ros

Concurrency-safe Sphero RVR core driver plus a ROS 2 adapter package.

This project is intentionally starting fresh from the older MCP implementation. The MCP repo remains useful as a protocol reference, but this repo is built around ROS 2 needs: one serial owner, request/response dispatching, safety preemption, and continuous velocity control.

## Initial scope

- `sphero_rvr_core`: async Python core driver and transport abstractions
- `sphero_rvr_driver`: ROS 2-facing adapter/node package

First hardware milestone: `/cmd_vel` teleop with timeout stop, emergency stop, diagnostics, and battery publishing.

## Base driver status

Implemented for the base ROS 2 driver slice:

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

## Install

### ROS 2 workspace install

On the Raspberry Pi, after ROS 2 is installed and sourced:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/jsperson/sphero_rvr_ros.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Core-only development install

For local development without ROS 2:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
python -m pytest -q
```

## Launch

On the Raspberry Pi, from a sourced ROS 2 workspace:

```bash
ros2 launch sphero_rvr_driver rvr.launch.py
```

Smoke the no-motion surfaces first:

```bash
ros2 topic echo /battery_state --once
ros2 topic echo /diagnostics --once
ros2 service call /stop std_srvs/srv/Trigger {}
```

⚠️ **Motor warning:** publishing non-zero `/cmd_vel` starts the RVR motors. Keep the robot suspended/restrained for the first validation run.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```
