# Lidar Motion Validation

`rvr_lidar_motion_validation` provides an independent, read-only check of rover
translation and heading. It does not publish ROS messages, open motor devices,
approve missions, or participate in the collision-safety path.

## Measurement target

Use a fixed, flat wall or rigid board that dominates the selected lidar sector
before and after motion. Do not treat one range beam as ground truth: after a
translation or turn, that beam may hit a different object without warning.

A single wall provides:

- translation normal to the wall;
- rover heading change from the change in wall-normal angle;
- fit residuals and scan-to-scan median absolute deviations.

It cannot measure translation parallel to that wall. Keep the tape measurement
for straight calibration, or use two nonparallel walls/full-scan localization
when two-dimensional translation matters.

## Attended capture

With the rover stationary and lidar already running, capture at least 20 scans:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws_mission_stack/install/setup.bash
rvr_lidar_motion_validation capture before.json --scan-count 20
```

Run the separately reviewed mission, wait for settled terminal evidence, and
capture the same fixed target again:

```bash
rvr_lidar_motion_validation capture after.json --scan-count 20
rvr_lidar_motion_validation compare before.json after.json \
  --sector-min-deg -80 --sector-max-deg 80 \
  --output comparison.json
```

Choose a narrower sector when needed to isolate the reviewed target. Preserve
the raw captures and comparison beside the mission terminal artifact. Sector
angles are expressed in the `laser` frame; the measured before/after heading
delta remains valid because the lidar mount is fixed to the rover.

## Acceptance

- The same fixed target must dominate both captures.
- Moving people and objects must stay outside the selected sector.
- Both captures must use the same lidar frame and stationary rover state.
- Review wall distance/angle MAD, inlier fraction, and residual RMS before using
  the reported delta.
- Compare lidar with tape, encoder/odometry, and operator observation; do not
  silently replace conflicting evidence.
- Lidar comparison remains validation-only until repeated physical evidence
  justifies a separately reviewed localization or control integration.
