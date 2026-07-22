# RVR full calibration gate before SLAM/Nav2/web autonomy

This plan defines the calibration work required before any web/autonomy feature treats RVR odometry, TF, lidar geometry, SLAM localization, or Nav2 navigation as trustworthy.

It is intentionally a **non-hardware execution plan**. Do not run the live commands below from an agent session unless the operator has first been warned and explicitly approved the exact motor-capable scope.

## Current known values and uncertainty

Known from first-pass floor calibration:

- Initial straight-line encoder scale: `odom_counts_per_meter: 4337.768`.
- Test conditions: `linear=0.05`, `duration=1.00`, `max-duty=128`.
- Current config uses the two-run estimate `odom_counts_per_meter: 4337.768` and the still-unvalidated `odom_wheel_track_m: 0.18` in `config/rvr.yaml`.
- ROS mission velocity currently uses the explicit `raw_motor` backend; native
  RC-SI is diagnostic-only. The June 24 scale does not transfer between packet
  backends.
- Odometry currently uses only left/right encoder deltas and a differential-drive/skid-steer approximation:
  - `left_m = left_delta / odom_counts_per_meter`
  - `right_m = right_delta / odom_counts_per_meter`
  - `distance = (left_m + right_m) / 2`
  - `delta_yaw = (right_m - left_m) / odom_wheel_track_m`

Still uncertain:

- Repeatability of `4337.768 counts/m` across distance, direction, speed, battery level, and floor surface.
- Low-speed breakaway/deadband threshold for forward, reverse, left turn, and right turn.
- Left/right wheel or tread imbalance under straight commands.
- Effective skid-steer wheel track for rotation on the actual floor.
- Whether odometry drift is acceptable for `slam_toolbox` scan matching at mapping speeds.
- Exact mounted transform `base_link -> laser` for the SLAMTEC RPLIDAR C1.

## Safety rule

Before any live motor-capable action, the operator must see and approve this exact warning:

```text
WARNING: this can start the RVR motors
```

Commands that require approval include:

- `rvr-console`, `rvr_tui`, or any teleop/TUI command path;
- `ros2 launch sphero_rvr_driver rvr.launch.py`, because the node owns `/dev/ttyAMA0` and exposes motor-capable command paths;
- publishing nonzero `/cmd_vel`;
- calling `/stop`, `/estop`, `/clear_estop`, `reset_yaw`, or `reset_locator` against the live driver;
- running hardware scripts such as `scripts/rvr_velocity_smoke.py`, `scripts/rvr_steering_smoke.py`, `scripts/rvr_hardware_smoke.py`, or `scripts/rvr_encoder_mapping.py`.

Safe without motor approval:

- local unit tests, fake transport tests, static docs edits, `git diff --check`;
- Pi package/import/build checks that do **not** launch the RVR driver and do **not** open `/dev/ttyAMA0`;
- lidar-only bench bring-up with the RVR driver stopped.

## Exact safe command ladder

Use this ladder for every live session. Do not skip ahead because a previous day worked; floors, batteries, payloads, and humans all drift.
The stop/estop protocol is part of the ladder, not a cleanup step at the end.

| Step | Motion? | Command scope | Stop/estop protocol | Advance when |
|---|---:|---|---|---|
| 0 | No | Local tests/build/docs only | No live stop needed; no UART access | fake/unit checks pass |
| 1 | No | Pi build/import checks only, driver not launched | No live stop needed; RVR driver stopped | ROS package imports/builds |
| 2 | No intended motion | Launch live driver after warning/approval; echo diagnostics, battery, `/odom` | Call `/stop` once before any movement | telemetry works and no spontaneous motion |
| 3 | Tiny motion | One `linear.x: 0.03` one-shot `/cmd_vel`, then stop | `/stop` immediately, then verify stale timeout produced zero output | robot stops cleanly |
| 4 | Estop proof | Repeat tiny motion, then `/estop` | Verify later `/cmd_vel` is blocked; `/clear_estop` must not move by itself | estop/clear-estop semantics hold |
| 5 | Calibration nudges | Phase-specific low-speed straight or turn commands | `/stop` after every trial; physical power button reachable | three clean repeats at the current rung |
| 6 | Manual mapping | Low-speed manual driving with lidar/SLAM active | `/stop` and `/estop` commands visible; operator ready to intervene | map and TF remain coherent |

Initial nonzero command limits for calibration sessions:

- start straight-line trials at `linear.x = 0.03 m/s` for <= 0.25 s;
- start turn trials at `angular.z = +/-0.05 rad/s` for <= 0.25 s;
- increase only one variable at a time: command value, duration, or path length;
- after every trial: stop, wait at least 0.5 s, record encoder/odom values, then decide the next rung.

## Calibration phases and gates

### Phase 0: No-motion readiness checks

Goal: prove the software stack, config, topics, and lidar bench setup are sane before exposing motor paths.

Run locally or on the Pi as appropriate:

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_odometry.py tests/test_ros_safe_surfaces.py tests/test_ros_node_config.py
python3 scripts/run_pytest_bounded.py --timeout 90 -- -vv
PYTHONPATH=src python3 -m compileall -q src
git diff --check
```

On `sphero-pi-2`, without launching the driver:

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

Data to record:

- commit or workspace diff identifier;
- `RVRNodeConfig` values for `max_linear_mps`, `max_angular_rad_s`, `max_raw_motor_duty`, `cmd_vel_timeout`, `odom_counts_per_meter`, and `odom_wheel_track_m`;
- battery charge before live tests;
- floor surface, tire/tread condition, payload stack, and whether lidar is mounted.

Pass criteria:

- all no-motion tests/build checks pass;
- config values are documented in the calibration sheet;
- stop/estop behavior is already covered by fake/unit tests;
- no live driver, TUI, `/cmd_vel`, `/stop`, or UART access was started accidentally.

### Phase 1: Tiny-motion safety and stop-path gate

Goal: prove the live robot can be stopped before collecting calibration data.

Preconditions:

- operator has approved the exact motor-capable warning scope;
- robot is on a clear floor or initially suspended/restrained;
- battery is reasonably charged;
- there is physical access to the robot power button;
- terminal windows are ready for driver logs and command shell;
- `stop` and `estop` commands are known and visible.

Protocol:

1. Start from estopped or disarmed state.
2. Launch only the RVR driver or TUI needed for the approved test.
3. Verify telemetry first: diagnostics, battery, `/odom` presence, and encoder counts changing only when motion occurs.
4. Send one very short forward nudge at the lowest planned command.
5. Immediately call `/stop` or the TUI `/stop`.
6. Repeat once with `/estop`, then verify `/clear-estop` does not itself move the robot.

Example approved ROS shell sequence:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sphero_rvr_driver rvr.launch.py
```

Second approved shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /diagnostics --once
ros2 topic echo /battery_state --once
ros2 topic echo /odom --once
ros2 service call /stop std_srvs/srv/Trigger {}
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.03}, angular: {z: 0.0}}'
ros2 service call /stop std_srvs/srv/Trigger {}
ros2 service call /estop std_srvs/srv/Trigger {}
ros2 service call /clear_estop std_srvs/srv/Trigger {}
```

Pass criteria:

- `/stop` reliably sends zero motor output;
- `/estop` blocks later motion until cleared;
- `/clear_estop` does not move the robot by itself;
- stale-command timeout stops a one-shot `/cmd_vel` without requiring a repeated command;
- no unexplained forward activity occurs on exit/shutdown.

Do not continue to calibration if this phase fails. Fix stop behavior first; calibration on a robot that will not politely stop is how tiny treads become a lab management problem.

## Measurement setup

Use one calibration sheet per run. Minimum fields:

| Field | Required value |
|---|---|
| Date/time | ISO timestamp |
| Repo state | commit SHA or `git diff --stat` summary |
| Config | `max_raw_motor_duty`, `max_linear_mps`, `max_angular_rad_s`, `cmd_vel_timeout`, odom params |
| Battery | percentage and voltage if available |
| Surface | floor material, mat/carpet, slope, debris |
| Payload | bare RVR, Pi, lidar, camera, battery pack, mounts |
| Run ID | unique number |
| Command | exact command, speed, duration, direction |
| Measured path | tape/marks coordinates and uncertainty |
| Encoder counts | before/after left/right and deltas |
| ROS odom | start/end `/odom` pose, yaw, twist if recorded |
| Outcome | moved, stalled, slipped, drifted, stopped cleanly |
| Notes | visible skew, bogging, wheel lift, cable drag |

Recommended physical setup:

- painter's tape centerline on a flat floor;
- 0.25 m, 0.50 m, 1.00 m, 1.50 m, and 2.00 m marks;
- square or laser line for start heading;
- tape measure resolution at least 5 mm;
- protractor/angle mat or AprilTag/printed compass sheet for rotation checks;
- phone video from above if available, used only as supporting evidence.

## Phase 2: Straight-line repeated distance calibration

Goal: replace the first-pass `4337.768 counts/m` with a repeatable encoder scale and understand variance.

Preconditions:

- Phase 1 stop gate passed;
- robot is on the target floor with final mapping payload installed if possible;
- use the safe `/cmd_vel` path, not raw motor commands, unless explicitly running a restrained low-level diagnostic script.

Test matrix:

| Direction | Command linear x | Target distance | Repeats |
|---|---:|---:|---:|
| Forward | `0.03 m/s` | `0.25 m` | 5 |
| Forward | `0.05 m/s` | `0.50 m` | 5 |
| Forward | `0.08 m/s` | `1.00 m` | 5 |
| Forward | `0.10 m/s` | `2.00 m` | 3 |
| Reverse | `-0.03 m/s` | `0.25 m` | 3 |
| Reverse | `-0.05 m/s` | `0.50 m` | 3 |
| Reverse | `-0.08 m/s` | `1.00 m` | 3 |

For each run:

1. Align the RVR on the marked centerline.
2. Record starting left/right encoder counts and `/odom` pose.
3. Reset local odometry/locator if the approved test scope includes it.
4. Command the selected velocity until the measured target distance is reached or until the timed command duration expires.
5. Stop, wait 0.5 s, record ending encoder counts and `/odom` pose.
6. Measure actual distance from start reference to final body reference.
7. Record lateral offset from the centerline and final heading error.

Derived values:

```text
left_counts_per_meter = abs(left_delta) / measured_distance_m
right_counts_per_meter = abs(right_delta) / measured_distance_m
mean_counts_per_meter = (abs(left_delta) + abs(right_delta)) / (2 * measured_distance_m)
left_right_ratio = abs(left_delta) / max(abs(right_delta), 1)
odom_distance_error_pct = 100 * (odom_distance_m - measured_distance_m) / measured_distance_m
```

Pass criteria for adopting `odom_counts_per_meter`:

- use the median of valid forward 0.50 m, 1.00 m, and 2.00 m runs, not a single run;
- coefficient of variation for accepted `mean_counts_per_meter` runs is <= 5%;
- forward/reverse scale mismatch is <= 8% or documented separately;
- lateral drift over 1.00 m is <= 5 cm at mapping speed, or compensation/acceptance must be explicitly downgraded;
- no individual accepted run has obvious slip, collision, cable drag, or stop failure.

Output:

- proposed `odom_counts_per_meter` value;
- variance and rejected-run notes;
- whether current `4337.768` remains valid, needs adjustment, or is valid only at `linear=0.05`, `max-duty=128`.

## Phase 3: Low-speed deadband and friction threshold mapping

Goal: identify the minimum commands that reliably start and sustain movement without overdriving the robot.

Test matrix:

| Motion | Values to sweep | Repeat rule |
|---|---|---|
| Forward | `0.01` to `0.12 m/s` in `0.01` steps | 3 trials per value near threshold |
| Reverse | `-0.01` to `-0.12 m/s` in `0.01` steps | 3 trials per value near threshold |
| Left turn | `0.05` to `0.40 rad/s` in `0.05` steps | 3 trials per value near threshold |
| Right turn | `-0.05` to `-0.40 rad/s` in `0.05` steps | 3 trials per value near threshold |

For each trial, use a short duration such as `0.25 s`, stop, wait, and record:

- whether motion started within 0.25 s;
- encoder deltas left/right;
- measured displacement or angle;
- audible/visible bogging;
- whether it stopped cleanly;
- battery percentage/voltage.

Derived thresholds:

- `breakaway_linear_forward`: smallest `linear.x` that moves in 3/3 trials;
- `sustain_linear_forward`: smallest `linear.x` that continues smoothly for 1 s;
- same for reverse;
- `breakaway_angular_left/right` and `sustain_angular_left/right`;
- recommended mapping speed range: above sustain threshold but below speed that causes visible slip or large scan smear.

Pass criteria:

- selected mapping speeds must exceed sustain thresholds by a modest margin, not by heroics;
- turn commands must work on the floor, not only suspended;
- thresholds must be recorded separately for final payload and floor surface;
- if left/right turn thresholds differ by > 15%, flag wheel-balance or skid asymmetry for Phase 4/5.

## Phase 4: Left/right wheel balance checks

Goal: decide whether straight commands produce symmetric tread motion and whether software compensation is needed later.

Straight-line balance:

1. Use the selected mapping forward speed from Phase 3.
2. Run 1.00 m forward along the centerline, 5 repeats.
3. Record encoder deltas, lateral offset, and heading error.
4. Repeat reverse for 0.50 m, 3 repeats.

Single-side diagnostic, only if explicitly approved and restrained/suspended:

```bash
python3 scripts/rvr_encoder_mapping.py --port /dev/serial0 --speed 128 --duration 0.5 --settle 0.35
```

This script sends raw motor pulses and is therefore not part of routine ROS operation.

Derived values:

```text
straight_encoder_ratio = abs(left_delta) / max(abs(right_delta), 1)
heading_error_per_meter_deg = final_heading_error_deg / measured_distance_m
lateral_error_per_meter = lateral_offset_m / measured_distance_m
```

Pass criteria:

- straight encoder ratio median is between 0.95 and 1.05 under normal forward mapping speed;
- heading error is <= 5 degrees per meter for mapping-speed straight runs;
- lateral error is <= 5 cm per meter;
- any systematic bias is documented before considering code compensation.

If these fail, do not hide it by tuning covariance alone. Either adjust mechanics/payload/treads first or create a dedicated wheel-balance compensation task.

## Phase 5: Rotation and effective wheel-track calibration

Goal: calibrate `odom_wheel_track_m` for skid-steer rotation on the actual floor.

Preconditions:

- Phase 3 found reliable left/right turn commands;
- Phase 4 does not show severe straight imbalance;
- use final payload and target floor.

Test matrix:

| Turn | Angular command | Target angle | Repeats |
|---|---:|---:|---:|
| Left | selected low reliable value, e.g. `+0.25 rad/s` | 45 deg | 5 |
| Left | selected low reliable value | 90 deg | 5 |
| Left | selected low reliable value | 180 deg | 3 |
| Right | selected low reliable value, e.g. `-0.25 rad/s` | 45 deg | 5 |
| Right | selected low reliable value | 90 deg | 5 |
| Right | selected low reliable value | 180 deg | 3 |

For each run:

1. Mark start heading and center point.
2. Record starting encoder counts and `/odom` yaw.
3. Command the turn, stop at target measured angle, wait 0.5 s.
4. Record ending encoder counts and `/odom` yaw.
5. Measure physical angle with an angle mat, protractor, overhead video, or fiducial method.
6. Record any visible translation during turn.

Derived value:

```text
left_m = left_delta / odom_counts_per_meter
right_m = right_delta / odom_counts_per_meter
measured_yaw_rad = measured_yaw_deg * pi / 180
effective_wheel_track_m = (right_m - left_m) / measured_yaw_rad
```

Use signed values and check direction. For a physical left turn with positive ROS `angular.z`, odometry yaw should increase; if not, the sign convention has regressed.

Pass criteria for adopting `odom_wheel_track_m`:

- left and right turn estimates agree within 10% at 45/90 degrees;
- 180-degree turns do not dominate the fit if they visibly slip;
- median yaw odometry error after tuning is <= 10 degrees for 90-degree turns;
- translation during in-place turns is documented and acceptable for mapping, or mapped as a skid limitation.

## Phase 6: Odometry validation against measured paths

Goal: validate tuned odometry over paths that look like manual mapping, not just isolated primitives.

Run these measured paths at the selected mapping speed:

1. **Straight out/back:** 1.00 m forward, stop, 1.00 m reverse.
2. **Box path:** 0.75 m forward + 90 deg turn, repeated 4 sides.
3. **Figure eight:** two slow loops around taped markers, manually driven.
4. **Room-edge pass:** slow manual drive along one wall/room perimeter with lidar running but not used as the odom truth.

For each path record:

- physical start/end pose error: x, y, yaw;
- `/odom` start/end pose error;
- max obvious drift during path;
- stop/estop availability;
- notes on slip, floor transitions, or cable drag.

Pass criteria before SLAM can rely on odometry as a scan-matching aid:

- 1.00 m straight odom distance error <= 5% median;
- 90-degree turn odom yaw error <= 10 degrees median;
- box path final position error <= 20 cm and yaw error <= 20 degrees;
- no single slow command causes uncontrolled acceleration, continuous drift after stop, or missed stale timeout;
- covariance remains nonzero and documentation continues to call odometry a low-trust aid, not ground truth.

Failure does not necessarily block manual SLAM visualization, but it blocks Nav2/autonomy and any web command that assumes localization quality.

## Phase 7: Lidar mount transform measurement (`base_link -> laser`)

Goal: publish a measured static transform so `/scan` aligns with `base_link` and the `map -> odom -> base_link -> laser` tree is physically meaningful.

Frame convention:

- `base_link`: RVR body frame, x forward, y left, z up.
- `laser`: lidar optical/scan frame per driver convention; confirm scan orientation in RViz.

Measurements to collect with the final mount installed:

| Transform field | How to measure |
|---|---|
| `x` | horizontal distance from `base_link` origin to lidar scan center, positive forward |
| `y` | horizontal offset from centerline to scan center, positive left |
| `z` | vertical distance from floor/body frame reference to scan plane |
| `roll` | lidar tilt around x; should be near 0 |
| `pitch` | lidar tilt around y; should be near 0 |
| `yaw` | lidar forward direction relative to `base_link` x; should be 0 or documented offset |

Practical measurement protocol:

1. Mark the RVR centerline and expected `base_link` origin on the payload deck drawing or body reference.
2. Measure lidar scan center, not the outer shell edge.
3. Record offsets in meters to the nearest 1 mm if possible, or 5 mm minimum.
4. Start lidar only with the RVR driver stopped and verify `/scan` orientation in RViz against a wall in front of the robot.
5. Add the static transform only after orientation is confirmed.

Example static transform shape, values are placeholders and must not be copied without measuring:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.000 --y 0.000 --z 0.000 \
  --roll 0.000 --pitch 0.000 --yaw 0.000 \
  --frame-id base_link --child-frame-id laser
```

Pass criteria:

- transform values are measured and committed to launch/config, not guessed;
- `ros2 run tf2_ros tf2_echo base_link laser` shows the expected static transform;
- RViz shows a flat wall in front of the robot as in front, not behind/rotated;
- final TF tree contains `map -> odom -> base_link -> laser` during SLAM validation.

## Phase 8: Live mapping validation gate

Goal: prove lidar, TF, odometry, and stop paths work together before allowing any web/autonomy dependency.

Preconditions:

- Phases 1-7 have passed or have explicit documented exceptions;
- lidar publishes stable `/scan` at expected rate;
- static `base_link -> laser` is measured;
- tuned odometry params are applied in `config/rvr.yaml` or launch overrides;
- operator approves the motor-capable mapping scope.

Manual mapping validation:

1. Launch lidar and verify `/scan`.
2. Launch the RVR driver after approval.
3. Verify:
   - `/odom` publishes;
   - `odom -> base_link` TF exists;
   - `base_link -> laser` TF exists;
   - stop/estop services are reachable;
   - diagnostics are OK or warnings are understood.
4. Launch `slam_toolbox` in online async mode.
5. Manually drive at selected low mapping speed.
6. Save the map.
7. Reload/inspect the map and record visible distortions.

Pass criteria before web/autonomy can depend on calibration:

- no TF gaps or frame-name mismatches during a short mapping run;
- `/scan` aligns with physical obstacles in RViz;
- map does not visibly shear, rotate, or double walls during slow manual driving;
- stop/estop can interrupt the mapping workflow immediately;
- saved map files exist and can be reloaded;
- localization can track a short manual loop on the saved map;
- all accepted calibration values and exceptions are documented.

## Final acceptance checklist

Do not green-light SLAM/Nav2/web autonomy dependency until this checklist is complete:

- [ ] `odom_counts_per_meter` chosen from repeated measured straight-line runs, not one first-pass run.
- [ ] `odom_wheel_track_m` chosen from measured rotations with left/right agreement.
- [ ] Low-speed forward/reverse/turn deadband thresholds recorded for final payload and floor.
- [ ] Left/right balance documented; any systematic bias has a follow-up task or explicit acceptance.
- [ ] Odometry validation paths meet distance/yaw/path error thresholds, or autonomy scope is downgraded.
- [ ] `base_link -> laser` measured and verified in TF/RViz.
- [ ] Stop, estop, stale timeout, and shutdown stop paths verified before and during mapping.
- [ ] Lidar-only bench bring-up succeeded before motor-capable mapping.
- [ ] Manual SLAM mapping produced a coherent saved map.
- [ ] Config/docs call odometry a conservative aid with nonzero covariance, not ground truth.
- [ ] Any future web/AI command layer treats motor-capable workflows as allowlisted, deterministic, and approval-gated.

Recommended config update after successful calibration:

```yaml
sphero_rvr_driver:
  ros__parameters:
    odom_counts_per_meter: <measured median>
    odom_wheel_track_m: <measured effective track>
    odom_pose_xy_covariance: <measured conservative value or keep nonzero default>
    odom_pose_yaw_covariance: <measured conservative value or keep nonzero default>
```

Keep raw measurements in a separate dated log or CSV. This document is the gate; the measurements are the evidence.
