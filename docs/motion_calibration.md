# RVR motion and odometry calibration

The RVR motion path is not calibrated yet. A nominal command of `0.20 m/s` for
`3s` moved roughly 6 ft, so do not use duration-based distance assumptions for
mapping until the scale is measured.

## Safety baseline

Before any armed calibration run:

1. Put the RVR on a clear straight path and mark the start position.
2. Keep the run tiny first: `--linear 0.02 --duration 0.25 --max-duty 32`.
3. Be ready to pick up or power off the robot.
4. Never run the TUI/teleop for calibration; use the gated script.

The calibration script defaults to no motion. It only moves with `--armed`.

## Verify no-motion telemetry

```bash
cd ~/ros2_ws/src/sphero_rvr_ros
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 scripts/rvr_motion_calibration.py
```

Expected: battery and encoder counts print, followed by `MOTION_SKIPPED`.

## First tiny armed nudge

WARNING: this can start the RVR motors.

```bash
cd ~/ros2_ws/src/sphero_rvr_ros
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 scripts/rvr_motion_calibration.py --armed --linear 0.02 --duration 0.25 --max-duty 32
```

Record:

- physical distance moved
- `encoder_delta_left`
- `encoder_delta_right`
- `encoder_delta_mean_abs`

If it does not move, increase only one thing at a time, cautiously:

```bash
python3 scripts/rvr_motion_calibration.py --armed --linear 0.02 --duration 0.50 --max-duty 32
python3 scripts/rvr_motion_calibration.py --armed --linear 0.03 --duration 0.50 --max-duty 32
python3 scripts/rvr_motion_calibration.py --armed --linear 0.03 --duration 0.50 --max-duty 48
```

Stop as soon as the robot produces a measurable small movement.

## Compute odometry scale

After measuring physical travel, rerun the same command with distance supplied,
or compute manually.

Example using feet:

```bash
python3 scripts/rvr_motion_calibration.py \
  --armed --linear 0.02 --duration 0.25 --max-duty 32 \
  --actual-distance-ft 0.25
```

The script prints:

```text
counts_per_meter=<value>
Suggested config/rvr.yaml update:
  odom_counts_per_meter: <value>
```

Manual formula:

```text
counts_per_meter = encoder_delta_mean_abs / actual_distance_m
```

Update `config/rvr.yaml`:

```yaml
odom_counts_per_meter: <measured value>
```

Then rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select sphero_rvr_driver
```

## What counts as good enough for SLAM prep

Before adding `slam_toolbox`, verify all of this:

- repeated short nudges produce similar counts-per-meter
- straight motion produces roughly similar left/right encoder deltas
- `/odom` reports physical distance within a tolerable rough range
- `odom -> base_link -> laser` exists while lidar launch is running

Until then, `/scan` is real but mapping will be map-shaped fiction.

## Current measured floor calibration

2026-06-24 measured run:

```text
command: --linear 0.05 --duration 1.00 --max-duty 128
encoder_delta_left: 975
encoder_delta_right: 983
encoder_delta_mean_abs: 979
measured_distance: 9 in / 0.2286 m
counts_per_meter: 4282.590
```

This value is now staged in `config/rvr.yaml`:

```yaml
odom_counts_per_meter: 4282.590
```

Repeat this measurement a few times before treating odometry as SLAM-quality.


2026-06-24 validation run:

```text
command: --linear 0.05 --duration 1.00 --max-duty 128
encoder_delta_left: 1020
encoder_delta_right: 1016
encoder_delta_mean_abs: 1018
measured_distance: 9.125 in / 0.231775 m
counts_per_meter: 4392.191
```

Combined two-run calibration:

```text
run_1: 979 counts over 9.000 in => 4282.590 counts/m
run_2: 1018 counts over 9.125 in => 4392.191 counts/m
combined: 4337.768 counts/m
```

`config/rvr.yaml` now uses:

```yaml
odom_counts_per_meter: 4337.768
```

## Route-local versus absolute odometry evidence

The live route runner's historical `measured_distance_m` is the sum of each
translation projected from that segment's start pose along its start heading. It
is intentionally route-local. A later raw `/odom` position is absolute relative
to the driver odometry origin, so the two values cannot be compared unless the
route's start pose, final pose, and sampling time are retained. This missing
baseline—not a proven calibration factor—explains why the earlier 0.3234 m route
measurement could not be reconciled with a later `(0.6091, -0.2820)` snapshot.

The driver now publishes read-only `/encoder_counts` evidence beside `/odom`.
Each terminal route manifest records:

- absolute route start and final poses and their timestamps;
- route-frame `delta_x`, `delta_y`, displacement, heading change, and final
  absolute heading;
- raw left/right encoder count deltas and per-track distances;
- encoder start/final timestamps, with stale or unchanged samples rejected;
- the same start/final pose, heading, and track evidence for each completed
  segment;
- the existing segment-local projected distance and signed turn measurement.
- whether fresh pose/encoder evidence remained settled for the configured
  terminal window, how long settling took, and the final target error.

A reached target is no longer a terminal result by itself. The runner first
publishes zero, and the core driver converts the nonzero-to-zero transition into
the validated immediate raw-motor stop rather than a slew-enabled RC zero. The
runner then requires 0.50 seconds of stable fresh evidence, permits at most 2.0
seconds to settle, and fails with `motion_not_settled` if motion continues. A
settled translation more than 0.03 m from its target or a settled turn more than
5 degrees from its target fails with `target_error`. These are acceptance
bounds, not calibration corrections; do not change them merely to make a
physical test pass.

Null track fields mean the evidence was unavailable or its scale disagreed with
the reviewed runner configuration. Do not infer zeros and do not advance a
physical stage when any required field is null, the two track distances are
inconsistent, heading drift is unexplained, or the final pose continues changing
after the terminal result.

## Prepared prompt-drive stages

These are procedures, not authorization. Keep live execution locked off until
Scott is present and explicitly approves the exact digest printed for that stage.
Run one stage at a time, with the rover restrained for the first two:

1. `Move forward 10 centimeters, then stop.`
2. `Turn left 45 degrees, then stop.`
3. `Move forward 10 centimeters, turn left 45 degrees, then move forward 10 centimeters, then stop.`

For every stage, capture the proposal, full digest, source/deployed SHAs, route
start/final pose, final heading, left/right deltas, per-track distances, collision
state, terminal reason, and stationary post-terminal samples. Stop after any
STOP/ESTOP/collision activity, stale or missing evidence, inconsistent
measurement, executor error, or incomplete process/device cleanup.
