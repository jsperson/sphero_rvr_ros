# Sphero RVR API parity handoff

This report is the operator handoff for the full API-parity documentation set. It replaces the earlier gap-audit wording that listed missing builders/parsers before the parity implementation cards landed.

## Source documents

- Capability matrix and validation ledger: `docs/rvr_capability_matrix.md`
- ROS exposure policy: `docs/rvr_ros_exposure_policy.md`
- Notification/event inventory: `docs/rvr_notification_events.md`
- Odometry/TF design notes: `docs/rvr_odometry_tf_design.md`
- Operator status and validation gates: `STATUS.md`

## Current parity state

The core driver now has fake/unit coverage for the official local-control RVR API surface tracked in the capability matrix:

- public `RVRCommands` builders are classified in `docs/rvr_capability_matrix.md`;
- public async `RVRDriver` methods are classified in `docs/rvr_capability_matrix.md`;
- request/response payloads have parser coverage or an explicit `documented-omission` for protocol-research mismatches;
- unsolicited notification/event packet routing has fake-transport tests;
- current ROS-exposed rows declare `ros-exposure-test` coverage.

The matrix uses these validation tokens: `builder-test`, `parser-test`, `driver-test`, `notification-test`, `ros-exposure-test`, `fake-transport-test`, and `documented-omission`.

## ROS exposure policy summary

Full API parity is a `sphero_rvr_core` property, not a promise that every packet becomes a ROS topic or service.

Routine ROS exposure stays intentionally small and typed:

- `/cmd_vel`, bounded by max linear/angular speed, raw motor duty cap, stale-command timeout, and software estop;
- `stop`, `estop`, and `clear_estop` Trigger services;
- `battery_state`, motor temperatures, `ambient_light`, `/odom`, diagnostics, and `odom -> base_link` TF;
- `reset_yaw`, `reset_locator`, and `release_led_requests` Trigger services;
- bounded all-LED feedback through `set_all_leds` `ColorRGBA`.

These remain core-only or intentionally omitted from routine ROS exposure unless a later design explicitly proves a safer abstraction: raw motors, arbitrary packet send, firmware/admin/update/factory flows, calibration, opaque streaming bytes, autonomous IR follow/evade/broadcast behaviors, palette persistence, sleep/power-management controls, MAC/stats IDs, and protocol-shaped debug plumbing.

## Known protocol-research caveats

The following are tracked explicitly instead of being silent gaps:

- IR send/broadcast command shapes differ between the older repo extensions and the official SDK surface.
- `get_current_detected_color_reading()` has official event-result semantics that may differ from direct request/response convenience behavior.
- Motor temperature has an official per-motor API shape while the repo also has a paired temperature convenience query.
- Opaque streaming-service data is implemented in core but should only reach ROS through future typed topics with bounded rates and cleanup semantics.

See `docs/rvr_capability_matrix.md` for the row-level classification and `documented-omission` notes.

## macOS / no-ROS fake validation

Safe on development hosts; these commands do not open the RVR UART:

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

## Pi / ROS environment no-motion validation

Run on `sphero-pi-2` after sourcing ROS and rebuilding. This verifies install/package wiring and importable ROS surfaces; it intentionally does not launch the live driver or publish motion:

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

## Live hardware smoke policy

Any command that launches the driver against the RVR UART or can reach motor-capable paths requires this exact warning and explicit operator approval first:

```text
WARNING: this can start the RVR motors
```

This applies to `rvr-console`, `rvr_tui`, `ros2 launch sphero_rvr_driver rvr.launch.py`, `/stop`, `/cmd_vel`, and any live driver command. A “no-motion smoke” that launches the driver is still live hardware access because the node owns `/dev/ttyAMA0` and exposes motor-capable command paths.

After approval, keep the robot restrained or suspended for first bring-up and scope the smoke narrowly: topic/service listing, battery/diagnostics/ambient/odom reads, TF echo, `/stop`, one conservative `ros2 topic pub --once /cmd_vel ...`, and a final `/stop`. Sustained driving, mapping, TUI operation, or autonomy requires a separate explicit scope and approval.
