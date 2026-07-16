# RVR staged hardware smoke plan

This is a **plan/checklist only** for validating the API-parity implementation on a live Sphero RVR. Do not run it automatically, and do not execute any live UART/ROS driver command without the approval gate below.

## Approval and safety gate

Before any command that opens `/dev/ttyAMA0`, launches the ROS driver, calls the live driver, or reaches a motor-capable service, show the operator this exact warning and wait for explicit approval:

```text
WARNING: this can start the RVR motors
```

Scope approval narrowly. Approval for the non-motion smoke below does **not** approve `/cmd_vel`, suspended pulses, floor driving, TUI operation, mapping, or autonomy.

Preconditions after approval:

- RVR is on a clear bench, powered, and reachable from `sphero-pi-2`.
- Use the accepted lightweight hardware layout for floor tests: one-level rack with a narrow lidar tower, carrying only the Pi 5, Pi battery, lidar, and Pi Camera 3. Do not baseline floor behavior with the old three-level rack; it was too heavy/high-CG and caused weak turning/drive behavior.
- Treads are clear; for any future movement stage, the RVR is suspended or physically restrained.
- One shell is reserved for logs and one for commands.
- Operator can physically power off the robot if software stop fails.
- No `rvr-console`, `rvr_tui`, mapping, or autonomy process is already running.

## Stage 0 — no-hardware rehearsal

Run this before touching the robot. It validates the code and package wiring without opening the UART.

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

Pass criteria:

- build exits successfully;
- `sphero_rvr_driver` executables are listed;
- `RVRNodeConfig()` imports and prints defaults;
- no ROS launch, UART open, `/stop`, `/cmd_vel`, or live driver command has run.

## Stage 1 — direct core liveness smoke, no movement

Preferred script shape: a dedicated `scripts/rvr_api_parity_smoke.py --no-motion` or equivalent direct-driver script. It should use `RVRDriver`/`SerialTransport`, print each step and response, and always run cleanup in `finally`.

Checklist sequence:

1. **Connect/wake**
   - Open `/dev/ttyAMA0` at `115200`.
   - Call `driver.connect()`; this sends wake and starts the dispatcher.
   - Drain/log stale packets without treating unsolicited notifications as failures.
2. **Battery**
   - `get_battery_percentage()`.
   - `get_battery_voltage()`.
   - `get_battery_voltage_state()`.
   - Optionally `get_battery_thresholds()`.
   - Abort before later stages if battery is low/critical.
3. **Firmware/system info**
   - `get_firmware_version()`.
   - `get_bootloader_version()`.
   - `get_board_revision()`.
   - `get_processor_name()`.
   - `get_sku()`.
   - `get_core_uptime()`.
   - Do **not** print stable identifiers such as MAC/stats ID unless the operator explicitly asks for debug identity data.
4. **Visible LED feedback**
   - Set a low-brightness, non-persistent all-LED color, e.g. dim blue/green.
   - Confirm visually that the LED changed.
   - Call `release_led_requests()` at cleanup so normal idle indication can resume.
5. **Stop/off belt-and-suspenders**
   - Send the project stop path (`driver.stop()`).
   - Send `raw_motors(0, 0, 0, 0)` as a zero-output cleanup command.
   - This is still live motor-capable access; it belongs inside the approved hardware smoke, not in no-hardware rehearsal.
6. **Selected read-only sensors**
   - `get_ambient_light()`.
   - `get_rgbc_sensor_values()` if useful for color-sensor sanity.
   - `get_encoder_counts()` before/after a short stationary interval; pass when deltas are near zero.
   - `get_thermal_protection_status()`.
   - `get_motor_fault_state()`.
   - Optional debug-only: `get_ir_readings()` and `get_magnetometer()`.
7. **Notification checks, only if safe**
   - Register local callbacks before enabling firmware notifications.
   - Prefer passive/diagnostic events first: battery voltage state, motor fault, motor stall, gyro max, motor thermal protection.
   - Enable each notification, wait briefly for either a cached event or a clean timeout, then disable it.
   - Do not enable high-rate streaming service or color interval notifications in the first smoke unless a rate/timeout/backpressure limit is included.

Pass criteria:

- wake/connect succeeds;
- battery and voltage read successfully;
- system info reads without parser errors;
- LED visibly changes and is released during cleanup;
- stop/off cleanup completes without unexpected motion;
- selected sensors return typed values;
- notification enable/disable calls do not break ordinary request/response traffic.

Abort criteria:

- unexpected tread movement at any point;
- low/critical battery state;
- motor fault/stall active;
- thermal protection elevated;
- UART decode errors repeat after reconnect;
- dispatcher logs show unmatched responses for ordinary request/response commands.

## Stage 2 — ROS graph non-motion smoke

This stage launches the ROS node, so it uses the same warning/approval gate even though the intended checks are read-only.

Shell A:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sphero_rvr_driver rvr.launch.py
```

Shell B:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep -E 'battery_state|diagnostics|ambient_light|odom|set_all_leds|cmd_vel'
ros2 service list | grep -E 'stop|estop|clear_estop|reset_yaw|reset_locator|release_led_requests'
ros2 topic echo /battery_state --once
ros2 topic echo /diagnostics --once
ros2 topic echo /ambient_light --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Optional non-motion operator feedback, still under the same approval:

```bash
ros2 topic pub --once /set_all_leds std_msgs/msg/ColorRGBA '{r: 0.0, g: 0.0, b: 0.25, a: 1.0}'
ros2 service call /release_led_requests std_srvs/srv/Trigger {}
ros2 service call /stop std_srvs/srv/Trigger {}
```

Pass criteria:

- expected topics/services exist;
- telemetry topics publish once;
- diagnostics include connected state and metadata;
- `/odom` and `odom -> base_link` publish without requiring robot movement;
- LED command is visible and release succeeds, if run;
- final stop service succeeds, if run.

## Stage 3 — movement smoke, separate approval only

Movement is deliberately **not** part of the parity non-motion smoke. Treat this as a separate card/run with a fresh warning and explicit approval:

```text
WARNING: this can start the RVR motors
```

Additional movement preconditions:

- RVR is suspended or physically restrained for the first movement run.
- Operator approves exactly one conservative movement pulse and final stop.
- Start with the driver-level suspended smoke path, not floor driving or TUI.
- Capture encoder deltas before/after and stop twice.

Candidate commands after separate approval:

```bash
python3 scripts/rvr_velocity_smoke.py --port /dev/ttyAMA0 --armed --linear 0.1 --angular 0.0 --duration 0.2 --max-duty 64
# or, for ROS path validation only after the direct-driver pulse is understood:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
ros2 service call /stop std_srvs/srv/Trigger {}
```

Do not combine this with TUI, mapping, lidar, autonomy, sustained driving, or floor testing. Those are separate scopes; robots do not respect “while we’re here.”

## Script implementation notes

If this plan becomes an executable smoke script, keep these properties:

- default mode is `--no-motion`; movement requires an explicit `--armed-movement` flag plus an interactive confirmation unless running in CI fake mode;
- print the warning before opening the live port;
- emit structured step results (`PASS`, `WARN`, `FAIL`, `SKIP`) so logs can be pasted into a handoff;
- use bounded timeouts for every request and every notification wait;
- disable firmware notifications in `finally` when enabled;
- call `release_led_requests()`, `stop()`, and `raw_motors(0, 0, 0, 0)` in cleanup best-effort;
- never call `sleep()`, calibration, firmware/admin/update/factory commands, raw nonzero motors, IR follow/evade, or streaming-service firehoses in the default smoke.
