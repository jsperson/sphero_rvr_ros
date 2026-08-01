# RVR motion and odometry calibration

## July 24 straight-run stall-classifier finding

The first attended, approved 10 cm browser-to-track run at source SHA
`984defb65cc953b62e4d23bda6f334f69a698606` failed closed with terminal reason
`stall` after 0.04438 m of measured travel. Scott independently observed about
1–2 inches of motion and reported everything else normal. The tracks remained
matched (193/192 encoder counts), collision stayed `CLEAR`, the driver reported
no firmware stall or motor fault, and battery voltage remained healthy.
Requested and supervised speed never exceeded 0.08 m/s, and the final
motor-bound command was zero.

The recorded odometry advanced throughout the commanded interval, but the
controller refreshed its activity timer only after each additional 0.015 m.
That checkpoint was too coarse for the slow tail of this trace: continuous
sub-checkpoint progress was classified as inactivity for 0.75 seconds. The live
route controller's translation activity quantum is therefore 0.005 m, about
22 counts at the reviewed 4337.768 counts/m scale. A replay regression preserves
the complete observed slow-progress prefix and a separate test proves that zero
progress still terminates as `stall`.

This correction does not change the 0.01 m goal tolerance, 0.03 m terminal error
bound, 0.50 s stationary-evidence window, speed ceilings, intent timeout, or any
collision/STOP/ESTOP/stale-data rule. The failed run is evidence for the
classifier change, not straight-distance acceptance. A newly built exact SHA
still requires a separately authorized attended 10 cm revalidation before the
turn stage.

## July 24 Adaptive mission stopping-horizon finding

The first real-OAuth physical Adaptive mission intent at
`a7b9a3771db9c9f332ec348ab54aeac4455ac8bf` used the installed Adaptive mission
0.10 m/s ceiling, which maps to effective duty 64. That resolved the duty-51
stall, but the deterministic terminal gate correctly rejected the run as
`target_error`: a 0.10 m intent settled at 0.138780 m. The tracks remained
matched at 600/604 counts, heading changed only 0.211 degrees, collision stayed
`CLEAR`, and two independent post-terminal samples proved the rover stationary.
The mission recorded one real LLM revision and ended with
`auto_resume=false`; no second intent was requested.

The bag shows why: odometry was 0.074692 m on the last sample that should have
continued, then 0.100511 m one 10 Hz sample later when the route first emitted
zero. The drivetrain coasted another 0.038268 m before settling. The route
controller now reserves a 0.25 second target stopping horizon, covering one
odometry period plus the observed coast. It estimates progress rate from
consecutive authoritative odometry samples and caps that estimate at the
requested primitive speed so one encoder jump cannot manufacture an unbounded
braking distance. The existing stationary and 0.03 m terminal-error gates remain
authoritative; the horizon cannot turn an undershoot or overshoot into success.
A replay regression projects the observed post-zero coast from the earlier
release point and requires a settled 0.1134 m terminal measurement.

This is a controller correction, not acceptance evidence. A rebuilt exact SHA
still requires a fresh attended 10 cm Adaptive mission run before physical turn or room
exploration.

## Provisional effective turn geometry

The July 23 attended 45-degree attempt ended with 699 differential encoder
counts / 0.161143 m of differential track travel. Encoder odometry using the
old 0.18 m track reported 51.293 degrees. Symmetric registration of 20 matched
stationary before/after lidar scans instead measured a median 36.829 degrees
(36.802–36.866 degrees), implying an effective skid-steer track of 0.250696 m.

Scott explicitly approved the rounded provisional value
`odom_wheel_track_m: 0.2507`. A subsequent restrained run independently
validated it: encoder odometry measured 26.765 degrees, fixed-wall lidar
measured 26.567 degrees, and full-scan registration measured a 26.751-degree
median. The run stalled mechanically while 0.25 rad/s was still commanded, so
Scott approved raising the turn-primitive-only ceiling to
`max_turn_speed_rad_s: 0.35`. This is a restrained midpoint below the prior
0.4 rad/s run: it seeks adequate sustained pivot torque while limiting coast.
The 2-degree control tolerance and, at the time of these trials, 5-degree
terminal acceptance gate were unchanged.

Two exact-SHA Adaptive mission traces then bounded the release behavior at that command.
Without predictive release, a `0.35 rad/s` request reached 43.256 degrees before
zero and settled at 60.326 degrees. A first measured-rate correction reused the
straight-run `0.25 s` horizon; it released at 18.757 degrees and settled at
30.506 degrees. Both runs failed closed as `target_error`, issued no retry, and
ended with independent STOP, stationary evidence, and relock. The second bag
measured 11.749 degrees of travel from the last pre-zero odometry sample and
only 0.158 degrees after the first post-zero sample, with zero requested
0.050 seconds after the release sample.

The next exact-SHA run tested a `0.15 rad/s` operating request with the separate
`0.10 s` horizon. That request moved only 2.318 degrees in 1.50 seconds before
the deterministic stall gate stopped the route. The encoders changed by only
-24/+20 counts and the settled path displacement was 0.000461 m. The provider
was not called again, independent STOP succeeded, and the graph was stationary,
cleaned up, and relocked. Together with the earlier `0.25 rad/s` stall, this
shows that lowering the continuous request below the drivetrain breakaway point
cannot control the turn.

The turn primitive therefore returns to the lowest request that demonstrated
sustained motion, `0.35 rad/s`, while the collision supervisor and driver retain
their installed absolute `0.4 rad/s` ceiling. The `0.10 s` turn horizon now
retains the last authoritative 10 Hz yaw rate and projects it only to the
current 20 Hz control tick. This permits a zero command between odometry
samples, instead of waiting for the next sample to jump past the braking point.
The stale-odometry gate bounds the projection. Translation keeps its independent
`0.25 s` horizon and `0.10 m/s` ceiling. Stationary evidence and the configured
settled-angle error remain authoritative.

The first exact-SHA between-sample revalidation exposed a second source of
variation. The same 0.35 rad/s request now peaked at 1.81 rad/s measured yaw
instead of the earlier 3.21 rad/s. Zero was requested after 0.50 seconds at
34.826 degrees, and the rover settled at 36.670 degrees, 8.330 degrees short.
The route failed closed as `target_error`, made no retry, and was independently
stopped and relocked. The trace also showed asymmetric -665/+31 encoder counts
and 0.0722 m of route displacement, so a one-shot stopping horizon is not
sufficiently repeatable on the installed drivetrain and floor.

The turn executor now remains closed-loop inside the same correlated intent:
after zero, it first requires the normal stationary window. If the settled
result is still outside the configured terminal tolerance, it may re-engage the
same bounded 0.35 rad/s command at most three times, always within the original
5-second timeout.
Every correction tick still passes the fresh odometry, scan, collision,
STOP/ESTOP, cancellation, and route-runtime gates. It never corrects an
overshoot or any terminal safety/failure result, and it does not create a new
LLM intent or approval. Correction count is included in the terminal manifest.
After the attended run settled at 39.568 degrees for a requested 45-degree
turn, the product decision was to prioritize closed-loop exploration capability
over tighter turn precision. Production Adaptive mission therefore accepts a stationary
turn within 10 degrees. The measured 5.432-degree error is inside that
capability threshold. Collision, freshness, STOP/ESTOP, speed, intent duration,
and lease gates are unchanged; this tolerance is not a safety boundary.

The straight-line encoder scale has a two-run first estimate, but the complete
motion path is not calibrated yet. Do not use duration-based distance assumptions
for mapping until the scale is repeated on the current floor and payload.

## Why the suspended 10 cm test overshot

The June 24 floor measurements used explicit raw-motor duty packets. A later
driver change routed straight ROS velocity commands through the RVR's native
RC-SI command while leaving the raw-duty caps and calibration values in config.
That made `max_linear_raw_motor_duty` ineffective. In the suspended June 26 test,
a requested 0.10 m produced 1,213/1,187 encoder counts and a measured 0.2766 m
before the corrected terminal validator failed closed with `target_error`.

ROS missions now explicitly use `velocity_control_mode: raw_motor`. At the
current limits, the normal 0.08 m/s prompt speed maps to duty 51 on each track.
The native RC-SI backend remains opt-in for isolated diagnostics and its pivot
path now respects angular magnitude instead of jumping to full normalized duty.
The June 24 scale applies only to the restored raw-duty path; it still needs
fresh ground confirmation.

## Safety baseline

Before any armed calibration run:

1. Put the RVR on a clear straight path and mark the start position.
2. Keep the run tiny first: `--linear 0.02 --duration 0.25 --max-duty 32`.
3. Be ready to pick up or power off the robot.
4. Never run the TUI/teleop for calibration; use the gated script.

The calibration script defaults to no motion. It only moves with `--armed`, and
defaults to the same `raw_motor` packet backend used by ROS missions.
The `native_rc_si` control mode is quarantined after its 2026-08-01 straight
speed mapping measured roughly ten times the request. `RVRDriver` rejects it;
use the dedicated tank-SI mapping gate in `docs/lean_explore_run_guide.md`.

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

## Attended ground-distance procedure

Do this only with Scott present. It is a measured calibration series, not a
request to make the success tolerance wider.

1. Use a flat, clear lane at least 1 m long with the normal Pi/lidar/camera
   payload installed. Mark one repeatable body reference at the start and keep a
   tape measure aligned with the intended path.
2. Confirm the deployed SHA matches the validated SHA, config reports
   `velocity_control_mode: raw_motor`, collision is `CLEAR`, STOP is `READY`,
   ESTOP is `CLEAR`, odom/encoders are fresh, and `/cmd_vel_motor` is zero.
3. Submit `Move forward 25 centimeters, then stop.` in the live web console.
   Review the complete proposal and use the single **Approve and run** action.
4. At terminal state, require zero motor output and stationary evidence before
   measuring. The reviewed exact-SHA execution gate may remain enabled for the
   three-run series; no GUID, digest, code, or hash is entered. Measure actual
   travel from the same body reference; do not use odometry as the tape
   measurement.
5. Download the page's Terminal result JSON and analyze it, for example:

   ```bash
   python3 scripts/analyze_ground_calibration.py terminal-result.json \
     --actual-distance-in 9.5 --output ground-sample-01.json
   ```

6. Repeat the 0.25 m stage three times from the same setup. Preserve every raw
   terminal artifact and its analyzed sample, including rejected runs. Aggregate
   the three samples:

   ```bash
   python3 scripts/aggregate_ground_calibration.py \
     ground-sample-01.json ground-sample-02.json ground-sample-03.json \
     --output ground-set-025m.json
   ```

   Exit 0 and `eligible_for_config_review=true` require three distinct mission
   executions from one exact source SHA, every individual safety/evidence gate
   to pass, and each counts-per-meter value to be within 5% of the median.
   Identical approved proposals intentionally share a digest-derived route ID;
   their persisted mission IDs distinguish the physical repeats. Exit 2 means
   the set was analyzed but rejected. Neither analyzer changes config.
   Relock execution immediately after the third run or after any rejected or
   unsafe result.
7. If the 0.25 m set is eligible, repeat the complete three-sample procedure at
   0.50 m. Compare both sets, floor, payload, battery, heading drift, and visible
   slip before human review of any config change. Rebuild and revalidate after a
   reviewed change; never copy a suggested value automatically.

Stop the series after any collision/STOP/ESTOP/cancel activity, stale or missing
evidence, failure to settle, more than 10% left/right count mismatch, unexplained
heading drift, incomplete cleanup, or movement beyond the clear lane. A settled
`target_error` result remains valid calibration evidence because its error is
what the tape measurement is correcting. Turn calibration remains a later,
separate stage; do not use the current full angular-duty ceiling on the ground
until straight translation and STOP behavior are confirmed.

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
10 degrees from its target fails with `target_error`. These are capability
acceptance bounds, not claims of mapping-grade precision. The wider turn bound
reflects the product decision recorded above and does not weaken safety gates.

Null track fields mean the evidence was unavailable or its scale disagreed with
the reviewed runner configuration. Do not infer zeros and do not advance a
physical stage when any required field is null, the two track distances are
inconsistent, heading drift is unexplained, or the final pose continues changing
after the terminal result.

## Prepared prompt-drive stages

These are procedures, not authorization. Keep live execution locked off until
Scott is present, reviews the exact current proposal, and uses the authenticated
**Approve and run** action for that stage. Run one stage at a time, with the rover
restrained for the first two:

1. `Move forward 10 centimeters, then stop.`
2. `Turn left 45 degrees, then stop.`
3. `Move forward 10 centimeters, turn left 45 degrees, then move forward 10 centimeters, then stop.`

For every stage, capture the proposal, full digest, source/deployed SHAs, route
start/final pose, final heading, left/right deltas, per-track distances, collision
state, terminal reason, and stationary post-terminal samples. Stop after any
STOP/ESTOP/collision activity, stale or missing evidence, inconsistent
measurement, executor error, or incomplete process/device cleanup.
