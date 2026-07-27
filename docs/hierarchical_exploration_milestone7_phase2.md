# Milestone 7 Phase 2 surveyed stationary localization

## Scope

M7.2 replaces the Milestone 6 analytic and approximate localization evidence
with surveyed physical ground truth. It uses the Pi, camera, and lidar while the
rover is motionless during every capture and RVR power and motors are
unavailable. Between captures, an operator may manually reposition the
powered-down rover if its fixed-map pose is independently surveyed and supplied
identically to the launch and sample record. It does not
start `rvr_node`, the RVR serial transport, Nav2, `live_route_runner`,
`collision_stop_node`, `/cmd_vel`, `/cmd_vel_motor`, physical execution, or
motion authority.

The separately scoped
`m7_stationary_localization.launch.py` is default-off and contains only:

- the camera and its measured static transform;
- the lidar and its measured static transform;
- a static surveyed `map -> base_link` origin.

The evaluator in `m7_surveyed_localization.py` is ROS-free. It recomputes every
result from the recorded anchor, scan, calibration, and surveyed pose; a
manifest cannot inject a mapped result.

## Reviewed range bands and sample count

The range bands are fixed before evidence collection:

| Band | Surveyed target range |
| --- | --- |
| `near` | `0.30 m <= range < 0.55 m` |
| `mid` | `0.55 m <= range < 0.85 m` |
| `far` | `0.85 m <= range <= 1.20 m` |

Collect at least three distinct surveyed target-to-rover configurations for
each point-producing method in every band. The target may move while the rover
pose stays fixed, or one fixed target may be reused while the powered-down rover
is manually repositioned and its map pose is surveyed. That is at least 18 point
samples:

- nine `lidar_range` samples using a broad vertical target that intersects the
  lidar plane and has a reviewed camera anchor;
- nine `floor_projection` samples using a surveyed floor-contact marker and
  its bottom-contact image anchor.

Use lateral variation within each band rather than three captures of one
unchanged relative configuration. Coverage is keyed by the target coordinate
expressed in the surveyed base pose, not by target label, so renaming one fixed
object cannot game the gate. Record an additional real ambiguous
lidar-association control which must return `bearing_only`, `point: null`, and
`ambiguous_lidar_clusters`.

## Survey requirements

Define one fixed `map` with the initial `base_link` at `(0, 0)`, positive x
forward, and positive y left. Mark every target coordinate and every manually
changed rover pose on a level floor using a steel tape or laser measure and a
perpendicular square. Record the technique, surveyor, time, target map
coordinate, rover map pose, derived target-to-rover range, and stated position
uncertainty. The evaluator rejects uncertainty greater than `0.02 m` and
rejects range values inconsistent with the surveyed map geometry.

The revised 2026-07-27 tread-contact survey places the lidar scan origin at
`(x=+0.0045 m, y=-0.0110 m)` in `base_link`. Operators may use its visible
rotation axis as the physical datum. For example, the nominal
`base_link=(0.45, 0.0) m` center placement is `0.4455 m` forward and `0.0110 m`
left of the lidar axis. Record actual measurements rather than merely copying
the nominal values.

Preserve these existing gates:

- camera/lidar timestamp delta no greater than `100 ms`;
- image/pose age no greater than `150 ms`;
- multiple eligible lidar clusters are rejected rather than selected;
- `bearing_only` never contains a point;
- `lidar_range` error no greater than
  `min(0.08 m, 0.03 m + 0.04 * surveyed_range_m)`;
- `floor_projection` error no greater than `0.05 m`.

The report records per-sample error, bound, synchronization, pose age,
uncertainty, evidence IDs, method, reason, and mapped result. It also reports
minimum, median, p95, maximum, and RMSE per method and range band, plus a
candidate reviewed tolerance. Evidence may tighten a provisional tolerance.
The tool refuses a manifest that widens one.

## Prepare an exact-source session

After the tooling commit is reviewed locally, use its exact 40-character SHA:

```bash
source /opt/ros/jazzy/setup.bash
source /home/jsperson/ros2_ws/install/setup.bash

ros2 run sphero_rvr_driver rvr_m7_surveyed_localization plan \
  --run-id m7-phase2-YYYYmmddTHHMMSSZ \
  --output /tmp/m7-phase2-capture-plan.json

ros2 run sphero_rvr_driver rvr_m7_surveyed_localization template \
  --source-sha EXACT_EXECUTABLE_SOURCE_SHA \
  --output /tmp/m7-phase2-session.json
```

Inspect the plan and graph before enabling stationary inputs. Confirm the RVR
driver and RVR serial owner are absent. The plan emits, but never executes, the
two explicit commands:

```bash
ros2 launch sphero_rvr_driver m7_stationary_localization.launch.py \
  survey_session_enabled:=true \
  survey_base_x:=SURVEYED_MAP_X \
  survey_base_y:=SURVEYED_MAP_Y \
  survey_base_yaw:=SURVEYED_MAP_YAW

ros2 run sphero_rvr_driver rvr_rosbag_capture \
  --execute --until-interrupted --hardware-active \
  --output-root /home/jsperson/rvr_runs \
  --run-id m7-phase2-YYYYmmddTHHMMSSZ \
  --topic /scan \
  --topic /camera_node/image_raw \
  --topic /camera_node/camera_info \
  --topic /tf \
  --topic /tf_static
```

Capture one stable observation per placement, retaining the complete rosbag.
The completed manifest must inventory checksums for the bag, survey layout,
camera calibration, and cleanup log. Camera/lidar evidence IDs and source
timestamps must bind every compact sample back to that bag.

While rosbag is recording, capture the paired compact observation for each
stable placement:

```bash
ros2 run sphero_rvr_driver rvr_m7_surveyed_localization snapshot \
  --output /tmp/m7-samples/lidar-near-1-snapshot.json \
  --image-output /tmp/m7-samples/lidar-near-1.ppm
```

Review the PPM and record the target anchor pixel. Then bind that observation
to the independently surveyed coordinate:

```bash
ros2 run sphero_rvr_driver rvr_m7_surveyed_localization sample \
  /tmp/m7-samples/lidar-near-1-snapshot.json \
  --sample-id lidar-near-1 \
  --target-id vertical-target-near-left \
  --expected-method lidar_range \
  --target-x 0.40 --target-y 0.05 \
  --pose-x SURVEYED_MAP_X \
  --pose-y SURVEYED_MAP_Y \
  --pose-yaw SURVEYED_MAP_YAW \
  --anchor-u REVIEWED_U --anchor-v REVIEWED_V \
  --map-revision m7-survey-grid-YYYYmmdd \
  --surveyor OPERATOR \
  --survey-technique "steel tape and perpendicular square" \
  --surveyed-at-utc YYYY-MM-DDTHH:MM:SSZ \
  --survey-uncertainty-m 0.005 \
  --output /tmp/m7-samples/lidar-near-1.json
```

The snapshot node creates subscriptions only. It requires an image, camera
calibration, and scan with image/scan delta no greater than 100 ms, converts the
image to a checksum-bound PPM, retains all finite in-range scan returns, then
destroys the subscriptions and exits. It never creates a publisher.

## Evaluate and clean up

Stop rosbag cleanly before stopping the camera/lidar launch. Verify the three
sensor/static-transform nodes are gone, the prohibited motion nodes never
appeared, the RVR serial port has no owner, and no velocity publisher appeared.
Generate those checks rather than entering cleanup booleans by hand:

```bash
ros2 run sphero_rvr_driver rvr_m7_surveyed_localization audit \
  --source-sha EXACT_EXECUTABLE_SOURCE_SHA \
  --source-repo /home/jsperson/ros2_ws/src/sphero_rvr_ros \
  --output /tmp/m7-phase2-cleanup.json
```

The audit fails closed unless the checkout is clean at the exact source SHA,
sensor/rosbag/motion processes and prohibited ROS nodes are absent, all three
motion topics have zero publishers, and the lidar plus every candidate rover
serial device is ownerless. Copy the generated `cleanup` object and complete
audit into the session manifest and inventory the audit artifact checksum.

```bash
ros2 run sphero_rvr_driver rvr_m7_surveyed_localization evaluate \
  /tmp/m7-phase2-session.json \
  --output /tmp/m7-phase2-report.json
```

M7.2 passes only when the evaluator returns zero with every method/band
coverage gate, every localization error gate, the real ambiguity control,
stationary authority, artifact inventory, and cleanup true. This does not
approve moving perception, collision testing, physical execution, or M7.3.

## Local software verification

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_m7_surveyed_localization.py \
  tests/test_camera_lidar_localization.py \
  tests/test_package_metadata.py
```
