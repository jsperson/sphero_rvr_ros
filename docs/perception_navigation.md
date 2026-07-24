# Perception-guided navigation replay contract

This is the no-motion Stage 1 boundary for map-relative navigation. It does not
publish ROS messages, velocity, or motor commands and cannot enable physical
execution.

## Selected localization seam

The smallest existing live boundary is the mission owner's subscribed
`/mission_api/v2/localization/status` JSON topic. Stage 2 should populate that
topic from a lidar-authoritative `map -> base_link` estimate, preferably the
existing SLAM Toolbox localization transform plus its scan-matching health.
Encoder `/odom` remains a motion prior and disagreement check; it is not allowed
to become the authoritative pose when lidar localization is missing.

The existing fixed-wall lidar validator remains useful replay evidence for
heading and wall-normal displacement. It is not a general two-dimensional
localizer. The Stage 1 attempt-5 corpus preserves its recorded lidar-derived
heading alongside encoder and per-track evidence.

## Contracts

`LocalizationEstimate` reports:

- state: `valid`, `degraded`, `stale`, or `lost`;
- a timestamped `Pose2D` in an explicit frame;
- source and normalized quality;
- planar and heading covariance;
- lidar-versus-odometry translation and heading disagreement;
- an optional diagnostic detail.

Only `lidar*` and `slam*` sources are accepted as authoritative. Missing pose,
stale age, low quality, unacceptable covariance, frame mismatch, or excessive
odometry disagreement prevents another horizon.

`GoalRegion` binds:

- map-frame center and endpoint radius;
- optional heading range;
- minimum clearance;
- maximum runtime;
- cumulative translation budget.

`MotionHorizon` is a bounded navigation request, not a motor command. Stage 1
emits only translation or rotation requests with a maximum distance/angle,
timeout, clearance, reason, and sequence number. A future physical adapter must
send these through:

```text
bounded navigation request
  -> deterministic horizon controller
  -> collision supervisor
  -> rover
```

The navigator has no `Twist`, `/cmd_vel`, `/cmd_vel_motor`, serial, or route
publication surface.

## Feedback behavior

After each observation the pure replay controller:

1. checks ESTOP, STOP, cancellation, runtime, and localization health;
2. records the lidar-authoritative pose and odometry disagreement;
3. checks per-track evidence for no progress, wrong direction, or severe
   asymmetry;
4. accounts for cumulative localized translation;
5. chooses a bounded obstacle alternate when a reviewed side remains clear;
6. recomputes heading and distance from the current pose;
7. emits a correction horizon, reaches the goal region, or terminates with a
   truthful reason.

Terminal outcomes are `reached`, `partial`, `blocked`, `localization_lost`,
`progress_failed`, `cancelled`, `stopped`, and `estopped`. Terminal output
always says that zero output is required.

## Replay

The committed attempt-5 replay is derived from the preserved `c13cc51` terminal,
fixed-wall lidar, and per-track artifacts:

```bash
ros2 run sphero_rvr_driver rvr_perception_navigation_replay \
  artifacts/perception_navigation_replay/turn_attempt_5_replay.json
```

Developer-host equivalent:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.perception_navigation \
  artifacts/perception_navigation_replay/turn_attempt_5_replay.json
```

The result is explicitly `motion_authority=false` and
`physical_execution_enabled=false`. The recorded left/right travel
`-0.26557/+0.01360 m` stops as `severe_tread_asymmetry`; it cannot be mistaken
for a valid pivot or followed by an automatic correction.

Focused replay tests cover nominal progress, heading overshoot correction,
obstacle alternate selection, no-clear blocked behavior, stale/lost/low-quality
localization, odometry disagreement, severe tread asymmetry, cancellation,
STOP, and ESTOP.

## Mission service and web evidence

The mission owner validates localization or navigation-result JSON received on
the selected topic. Invalid payloads are marked invalid in the live cache.

When fresh navigation evidence exists, the web adapter can render the
authoritative lidar pose, goal region, next horizon, traveled path, quality, and
odometry disagreement. It continues to label occupancy and semantic-object
layers unavailable rather than substituting fixtures. The live page starts
polling immediately after its first state render so a cold `SENSOR_STALE`
snapshot cannot remain frozen.

## Stage boundary

This completes the ROS-free contract/replay portion only. Stage 2 still must
prove a real stationary lidar localization publisher, covariance/quality
truthfulness, freshness transitions, restart behavior, and browser updates with
the rover driver absent. No claim of live localization or physical navigation
is made here.
