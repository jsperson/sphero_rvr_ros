# Independent lidar collision-stop supervisor design

Source-of-truth implementation contract for a lidar-backed collision-stop supervisor that remains authoritative over every ordinary command source: TUI/key tap, browser/AI control, mission APIs, Nav2, teleop, rosbag replay, and future planners.

The Adaptive mission physical adapter is one such upstream mission source. It submits
bounded work to `/cmd_vel` from the active controller and consumes the
supervisor's scan/TF health plus requested/output evidence. It does not change
this document's topic ownership: the supervisor remains the sole
`/cmd_vel_motor` publisher.

This is a design document only. It must not activate hardware, launch the RVR driver, publish motion, capture calibration data, or validate physically.

## Current code audited

Current `origin/main` at design time is `44912af` (`feat: expose camera lidar calibration surfaces (#30)`). The design below is based on these current surfaces, not draft PRs or stale board claims.

- `src/sphero_rvr_driver/rvr_node.py`:
  - subscribes to `cmd_vel` and maps `geometry_msgs/msg/Twist` into `RVRDriver.set_velocity()`;
  - exposes `stop`, `estop`, and `clear_estop` as `std_srvs/srv/Trigger`;
  - publishes telemetry including `battery_state`, `odom`, `diagnostics`, and `odom -> base_link` TF;
  - reads `cmd_vel_timeout`, `base_frame_id`, and odometry parameters from `RVRNodeConfig`.
- `src/sphero_rvr_core/driver.py`:
  - has the final UART command loop today;
  - applies stale-command stop when `_last_velocity_update` exceeds `command_timeout`;
  - `stop()` clears desired velocity and sends a high-priority stop;
  - `emergency_stop()` latches `_emergency_stopped`, clears desired velocity, and sends an emergency-priority stop;
  - `clear_emergency_stop()` only clears the local latch.
- `config/rvr.yaml`:
  - current conservative defaults include `cmd_vel_timeout: 0.5`, `max_linear_mps: 0.10`, `max_angular_rad_s: 0.4`, and calibrated `odom_counts_per_meter: 4337.768`.
- `launch/rvr.launch.py`:
  - starts the live `sphero_rvr_driver` node with `config/rvr.yaml`.
- `launch/lidar.launch.py` and `config/lidar.yaml`:
  - start `rplidar_ros/rplidar_composition` as node `/rplidar_node`, publish `/scan`, and publish `base_link -> laser` via configurable `laser_x/y/z/roll/pitch/yaw` placeholders;
  - default lidar device is `/dev/rplidar` at `460800`, frame `laser`, scan mode `Standard`.
- `launch/mapping.launch.py`:
  - defaults to `start_rvr:=false`, `start_lidar:=true`, `start_camera:=false`, `start_slam:=true`;
  - full motor-capable mapping is opt-in with `start_rvr:=true`.
- `src/sphero_rvr_driver/tui_ros.py` and `docs/rvr_control_interface_plan.md`:
  - TUI status already observes `/scan`, service availability, TF health, and lazily enables its `/cmd_vel` publisher only in motor-capable state.
- `docs/rosbag_capture_replay.md`:
  - capture/replay helpers reject `/cmd_vel` and motor-like topics by default; unsafe replay must stay unable to reach hardware in supervised launches.

## Non-negotiable safety invariant

There must be exactly one ROS node that owns the final motor-bound velocity topic in a supervised motor-capable launch:

```text
ordinary command sources -> /cmd_vel -> lidar_collision_stop_supervisor -> /cmd_vel_motor -> sphero_rvr_driver -> UART -> RVR motors
```

The live RVR driver must not subscribe to public `/cmd_vel` in supervised launches. It must be remapped to a private motor-bound topic, `cmd_vel:=cmd_vel_motor`, and only the supervisor may publish that topic.

This avoids unsafe multiple publishers racing on the current public motor command topic. It also preserves ROS convention: Nav2, teleop, TUI, browser/AI adapters, and mission APIs can keep targeting `/cmd_vel`, but `/cmd_vel` becomes the requested-command ingress, not the hardware-bound command.

## ROS graph contract

### Nodes

| Node | Required name | Role |
|---|---|---|
| `sphero_rvr_driver` | `/sphero_rvr_driver` | Live UART owner; consumes only supervised `/cmd_vel_motor` in motor-capable launches. |
| `lidar_collision_stop_supervisor` | `/lidar_collision_stop_supervisor` | Final velocity arbiter; consumes requested `/cmd_vel`, scan/TF/diagnostics, publishes `/cmd_vel_motor`, exposes operator safety services/status. |
| `rplidar_node` | `/rplidar_node` | Publishes `/scan` in frame `laser`. |
| `base_to_laser_static_tf` | `/base_to_laser_static_tf` | Publishes measured or placeholder `base_link -> laser`. |

### Topics

| Topic | Type | Direction | Contract |
|---|---|---:|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | subscribed by supervisor | Ordinary requested velocity from TUI, Nav2, teleop, browser/AI, mission APIs, and tests. Never motor-bound directly in supervised launch. |
| `/cmd_vel_motor` | `geometry_msgs/msg/Twist` | published by supervisor, subscribed by driver | Final motor-bound velocity after collision-stop arbitration. No other publisher is allowed. |
| `/scan` | `sensor_msgs/msg/LaserScan` | subscribed by supervisor | Required live lidar input. Must be fresh, sane, and transformable to `base_link` before nonzero forward motion is allowed. |
| `/odom` | `nav_msgs/msg/Odometry` | observed by supervisor | Optional for diagnostics/speed context; not required for immediate stop. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | published by supervisor | Supervisor state, scan health, stop reason, active distances, command age, and arbitration decision. |
| `/collision_stop/state` | `std_msgs/msg/String` initially, typed custom msg later | published by supervisor | Operator-readable state stream for TUI/status panes without parsing diagnostics. |
| `/collision_stop/events` | `std_msgs/msg/String` initially, typed custom msg later | published by supervisor | Latched-ish event log: transition, stop asserted, stale scan, malformed scan, ESTOP, reset accepted/rejected. |

Use relative names in code so launch remapping can namespace them, but the default launch contract above must resolve to these absolute names at the top level.

### Services

Supervisor-facing services use `std_srvs/srv/Trigger` in the first implementation to match current driver surfaces and avoid custom message packaging churn.

| Public service | Driver service called | Semantics |
|---|---|---|
| `/stop` | `/rvr_driver/stop` or remapped driver `stop` | Motion-reducing. Immediately publish zero on `/cmd_vel_motor`, enter `STOPPED`, then call driver stop. Never requires confirmation. |
| `/estop` | `/rvr_driver/estop` or remapped driver `estop` | Strongest software safety command. Immediately publish zero, latch `ESTOPPED`, call driver estop. Blocks all nonzero commands until explicit clear. |
| `/clear_estop` | `/rvr_driver/clear_estop` or remapped driver `clear_estop` | Clears software ESTOP only when operator explicitly requests it. Leaves state `STOPPED` or `CLEAR` depending on scan health, but never forwards a stored nonzero command. |
| `/collision_stop/reset` | none | Clears collision `STOPPED` only when scan is fresh/clear and no ESTOP is active. Must reject reset while obstacle/stale/malformed. |
| `/collision_stop/disable` | none | Optional development-only service. See no-go boundaries; disabled mode is forbidden in operator motor-capable launch unless an explicit launch parameter allows it. |

Remap driver safety services under a private namespace in the supervised launch, for example:

```text
sphero_rvr_driver:
  cmd_vel       -> /cmd_vel_motor
  stop          -> /rvr_driver/stop
  estop         -> /rvr_driver/estop
  clear_estop   -> /rvr_driver/clear_estop

lidar_collision_stop_supervisor:
  stop          -> /stop
  estop         -> /estop
  clear_estop   -> /clear_estop
```

The supervisor owns the public safety services so STOP/ESTOP always pass through the same final-arbiter state machine before touching the driver.

## Arbitration rules

The supervisor is a velocity filter plus safety state machine. It does not plan paths. It accepts the latest requested `Twist`, bounds it to configured limits, and either forwards it, slows it, holds it at zero, or stops.

Priority order, strongest first:

1. `ESTOPPED`: output zero only. Ignore all requested nonzero commands. Requires `/clear_estop` success and then a separate normal command/reset path.
2. `SENSOR_STALE` or malformed scan: output zero only. No operator reset can clear this; only fresh valid scans can.
3. `STOPPED` due to collision hazard: output zero only. Requires obstacle clearance plus `/collision_stop/reset` or an explicit new safe command after hysteresis, depending on implementation parameter `reset_policy`.
4. `SLOW` or `HOLD`: reduce or zero forward motion as distances approach the
   front threshold; hold a turn only when its projected swept footprint
   intersects an observed point.
5. `CLEAR`: forward bounded requested velocity.
6. Ordinary stale `/cmd_vel`: output zero and let the driver stale timeout remain a second line of defense.

STOP and ESTOP remain stronger than ordinary commands because they are handled as supervisor state transitions, not just another `/cmd_vel` message. A later ordinary `/cmd_vel` must not clear `STOPPED` or `ESTOPPED` by itself.

## LaserScan health and geometry rules

### Freshness

Parameters:

| Parameter | Type | Default | Meaning |
|---|---:|---:|---|
| `scan_topic` | string | `/scan` | Input LaserScan topic. |
| `base_frame` | string | `base_link` | Footprint frame. Must match current RVR node default. |
| `laser_frame` | string | `laser` | Expected scan frame; normally from `config/lidar.yaml`. |
| `tf_timeout_s` | double | `0.05` | Bounded TF lookup timeout for transforming scan samples into `base_link`. |
| `max_scan_age_s` | double | `0.30` | Maximum time since the latest scan was received. This detects a stopped or delayed stream and must be less than driver `cmd_vel_timeout` (`0.5s`). |
| `max_scan_stamp_age_s` | double | `0.75` | Source-provenance sanity ceiling. The installed RPLidar stamps before acquiring a full revolution, so this is not the stream-loss clock. Frozen/replayed evidence is rejected immediately when stamps fail to advance; receipt age still forces zero before the driver timeout. |
| `startup_grace_s` | double | `2.0` | Startup wait for first scan. Output zero until healthy. |
| `min_valid_ranges` | int | `12` | Minimum valid ranges in relevant sectors. |
| `min_valid_fraction` | double | `0.05` | Minimum valid fraction over relevant sectors. |
| `malformed_scan_policy` | enum string | `stop` | `stop` is the only allowed operator default. |

A scan is unhealthy if any of these are true:

- missing beyond `startup_grace_s` after node activation;
- time since the latest scan receipt exceeds `max_scan_age_s`;
- source message-stamp age exceeds `max_scan_stamp_age_s`;
- source message stamps stop advancing or move backward;
- `ranges` is empty or too short to cover the configured sectors;
- `angle_increment <= 0`, `angle_min/angle_max` are non-finite, or metadata cannot map ranges to angles;
- valid range count/fraction is below threshold;
- `laser` (or scan frame) -> `base_link` transform is unavailable, stale, or malformed when `fail_on_missing_tf:=true`;
- `range_min`, `range_max`, or range values are non-finite in a way that prevents filtering.

Unhealthy scan means `SENSOR_STALE` or `STOPPED` with zero output. Treat sensor ambiguity as occupied space. Tiny robot, tiny margin for optimism.

### Valid-range filtering

For each range sample:

- reject `NaN`, `+Inf`, `-Inf`;
- reject values below `max(scan.range_min, min_range_m)`;
- reject values above `min(scan.range_max, max_range_m)`;
- optionally reject isolated single-bin spikes by median filtering over a small odd window, default off until tested;
- never treat invalid/unknown samples as proof of clear space. Too many unknowns in a sector makes that sector blocked/unknown.

Parameters:

| Parameter | Type | Default |
|---|---:|---:|
| `min_range_m` | double | `0.08` |
| `max_range_m` | double | `6.0` |
| `sector_unknown_policy` | enum string | `blocked` |

### Footprint and payload envelope

Use a conservative rectangular footprint around `base_link` until measured geometry is committed:

| Parameter | Type | Default | Notes |
|---|---:|---:|---|
| `footprint_front_m` | double | `0.22` | Forward from `base_link`, includes RVR nose plus payload overhang margin. |
| `footprint_rear_m` | double | `0.16` | Rear from `base_link`. |
| `footprint_left_m` | double | `0.14` | Left from centerline. |
| `footprint_right_m` | double | `0.14` | Right from centerline. |
| `payload_margin_m` | double | `0.05` | Extra clearance around measured payload/lidar/camera mount. |

The parent calibration task intentionally left `laser_x/y/z/roll/pitch/yaw` as measured/configured launch surfaces. The supervisor must use those configured transforms and must report whether they are placeholders. It must not bake physical offsets into code.

### Angular sectors

Default sector checks in `base_link` frame:

| Sector | Angles | Applies to | Action |
|---|---:|---|---|
| front_stop | `[-30°, +30°]` | positive linear.x | STOP/HOLD if obstacle within stop distance. |
| front_slow | `[-45°, +45°]` | positive linear.x | Clamp forward speed when obstacle within slow distance. |
| rear_stop | `[150°, 180°]` and `[-180°, -150°]` | negative linear.x | STOP/HOLD reverse motion. |
| left_spin | `[45°, 135°]` | positive angular.z / left arcs | Scan-health and diagnostic coverage; not a fixed-distance veto. |
| right_spin | `[-135°, -45°]` | negative angular.z / right arcs | Scan-health and diagnostic coverage; not a fixed-distance veto. |

The implementation transforms polar scan samples into `base_link` with TF2 and applies sector tests there. With `fail_on_missing_tf:=true`, missing, stale, or malformed transforms are reported as unsafe scan health (`missing_tf`, `stale_tf`, or `malformed_tf`) and force zero output.

### Projected trajectory envelope

Commands are not judged solely by a fixed distance applied to an entire broad
sector. The supervisor projects the current bounded `linear.x` / `angular.z`
command over:

```text
requested_cmd_timeout_s + measured_stop_time_s
```

The default horizon is `0.25 + 0.50 = 0.75 s`. It samples the constant-twist
arc at no more than 1 cm linear or 2° angular increments, transforms every valid
scan point into `base_link`, and sweeps the configured rectangular footprint
along those poses. Every footprint edge is expanded by
`trajectory_clearance_margin_m` (`0.02 m` by default). A command is held at zero
if a point enters that projected envelope; the check is repeated on every
command, scan, and tick while motion is requested. The existing conservative
front/rear stop and forward-slow rules remain additional gates.

For a point already intersecting the lidar-height rectangle, the supervisor
evaluates the entire sampled command trajectory relative to that known point.
It may exclude the point from that one sweep only when signed clearance never
decreases and either signed clearance or radial separation strictly increases.
Any initial approach, later re-entry, or motion that does not measurably
increase separation remains a veto. The rule is obstacle-relative rather than
front/rear-specific: a command may move away from a known point in any
direction, while the opposite command toward the same point remains blocked.
The independent front/rear sector rules are unchanged.

This means an obstacle behind or beside the rover does not block a forward or
turn command merely because it lies in a broad sector. A point that the current
trajectory will approach within the stopping horizon still blocks before
contact. Missing/malformed TF or scan data remains fail-closed.

The fixed `left_spin` and `right_spin` sectors remain useful for scan-health
coverage, nearest-side diagnostics, and operator visibility. They no longer
substitute a radial threshold for actual swept geometry.

### Stopping and slowdown distances

Parameters:

| Parameter | Type | Default | Meaning |
|---|---:|---:|---|
| `stop_distance_m` | double | `0.35` | Forward obstacle threshold including footprint/payload margin. |
| `slow_distance_m` | double | `0.60` | Start linear slowdown. |
| `reverse_stop_distance_m` | double | `0.25` | Rear obstacle threshold. |
| `trajectory_clearance_margin_m` | double | `0.02` | Margin added to each footprint edge during current-command trajectory projection. |
| `release_distance_m` | double | `0.45` | Front threshold required to release STOPPED. Must be greater than stop distance. |
| `release_time_s` | double | `0.50` | Scan must remain clear for this duration before release/reset can succeed. |
| `min_forward_scale` | double | `0.0` | Lowest scale in SLOW. `0.0` becomes HOLD. |
| `max_forward_mps` | double | `0.10` | Match current `config/rvr.yaml` unless lowered. |
| `max_angular_rad_s` | double | `0.4` | Match current `config/rvr.yaml` unless lowered. |

Slowdown shape:

```text
if nearest_front <= stop_distance_m: linear.x = 0, state = STOPPED
elif nearest_front < slow_distance_m: linear.x *= scale(nearest_front)
else: pass through bounded linear.x
```

Use monotonic scaling from `0.0` at `stop_distance_m` to `1.0` at
`slow_distance_m`. Angular velocity may pass through only if the projected
trajectory envelope is clear and `ESTOPPED/SENSOR_STALE/STOPPED` are inactive.

### Hysteresis

Collision STOP should not chatter at the threshold.

- Enter `STOPPED` immediately when stop threshold is crossed.
- Remain `STOPPED` until all relevant stop sectors are clear beyond `release_distance_m` for `release_time_s`.
- Require `/collision_stop/reset` for release in operator/live mode by default (`reset_policy:=manual`).
- In simulation tests, `reset_policy:=auto_after_clear` may be used to validate hysteresis without operator service calls.

## States and transitions

```text
DISABLED/STARTUP
  node starts, no scan yet                     -> STARTUP (zero output)
  first fresh valid scan and not disabled       -> CLEAR
  disable requested and allowed by launch       -> DISABLED

CLEAR
  requested command clear                       -> CLEAR, forward bounded /cmd_vel_motor
  obstacle in slow band                         -> SLOW
  obstacle at/inside stop band                  -> STOPPED + zero + event
  scan missing/stale/malformed                  -> SENSOR_STALE + zero + event
  /stop                                         -> STOPPED + zero + driver stop
  /estop                                        -> ESTOPPED + zero + driver estop

SLOW
  obstacle clears beyond slow distance          -> CLEAR
  obstacle reaches stop distance                -> STOPPED
  scan unhealthy                                -> SENSOR_STALE
  /stop                                         -> STOPPED
  /estop                                        -> ESTOPPED

STOPPED
  obstacle still inside release threshold       -> STOPPED, reject reset
  obstacle clear for release_time_s + reset     -> CLEAR, but do not replay old command
  /estop                                        -> ESTOPPED
  scan unhealthy                                -> SENSOR_STALE

SENSOR_STALE
  scan fresh and clear                          -> CLEAR or STOPPED depending nearest obstacle; old command is discarded
  scan fresh but blocked                        -> STOPPED
  /estop                                        -> ESTOPPED

ESTOPPED
  /clear_estop succeeds and scan fresh/clear    -> STOPPED or CLEAR with zero output only
  /clear_estop succeeds but scan unhealthy      -> SENSOR_STALE
  ordinary /cmd_vel                             -> ignored except for telemetry

DISABLED
  Only allowed in development/fake launch. In live operator motor-capable launch this state is a no-go unless an explicit `allow_disable:=true` parameter is set and diagnostics show ERROR/WARN.
```

Reset requirements:

- `/stop` can be cleared by `/collision_stop/reset` after scan-clear hysteresis, or by a new safe command only if `reset_policy:=auto_after_clear`.
- `/estop` requires `/clear_estop` and never preserves a prior nonzero command.
- `SENSOR_STALE` cannot be manually cleared; fresh valid scan data is required.
- Startup never forwards a nonzero command received before first healthy scan.

## Startup, shutdown, crash, and partition behavior

Startup:

1. Launch supervisor before or with the live driver.
2. Remap driver `cmd_vel` to `/cmd_vel_motor` before exposing any motor-capable graph.
3. Supervisor starts in `STARTUP` and publishes zero at `zero_publish_period_s` until scan health is established.
4. Public `/cmd_vel` may exist during startup, but requested commands are held and not replayed.
5. If lidar never appears, state is `SENSOR_STALE`; output remains zero.

Shutdown:

- Supervisor publishes zero on `/cmd_vel_motor` before shutdown when possible.
- Driver `disconnect()` already attempts `stop()` when connected and not emergency-stopped; keep that as a second line of defense.
- Launch shutdown order should stop ordinary command sources, then supervisor, then driver where possible.

Supervisor crash:

- Driver receives no fresh `/cmd_vel_motor`; current driver stale timeout (`cmd_vel_timeout: 0.5`) must stop motion.
- Ordinary sources publishing `/cmd_vel` cannot reach driver because of remap.
- Supervisor process death must be visible in launch logs and diagnostics; launch should use `respawn:=true` only if restart starts in `STARTUP` zero-output state and does not replay old commands.

Driver crash:

- Supervisor remains stateful and reports driver command sink unavailable.
- Public STOP/ESTOP should return failure if driver service calls fail, but local `ESTOPPED` latch remains active after `/estop`.

ROS graph partition:

- If supervisor loses `/scan` or TF, it goes `SENSOR_STALE` and outputs zero while it can.
- If driver loses supervisor, driver stale timeout stops.
- If ordinary source loses supervisor, it cannot reach hardware.

Lidar disappears while commands are active:

- On first freshness violation, publish zero, enter `SENSOR_STALE`, emit event, call driver `/stop` once per transition, and continue zero publishing at a bounded period until scan health returns.

## Parameters

All parameters must be declared with explicit types and sane ranges. Invalid parameters should make the supervisor fail closed at startup.

Minimum parameter surface:

```yaml
lidar_collision_stop_supervisor:
  ros__parameters:
    requested_cmd_topic: /cmd_vel
    motor_cmd_topic: /cmd_vel_motor
    scan_topic: /scan
    diagnostics_topic: /diagnostics
    state_topic: /collision_stop/state
    events_topic: /collision_stop/events
    base_frame: base_link
    laser_frame: laser
    tf_timeout_s: 0.05
    max_scan_age_s: 0.30
    max_scan_stamp_age_s: 0.75
    startup_grace_s: 2.0
    min_valid_ranges: 12
    min_valid_fraction: 0.05
    min_range_m: 0.08
    max_range_m: 6.0
    sector_unknown_policy: blocked
    footprint_front_m: 0.22
    footprint_rear_m: 0.16
    footprint_left_m: 0.14
    footprint_right_m: 0.14
    payload_margin_m: 0.05
    front_stop_min_angle_deg: -30.0
    front_stop_max_angle_deg: 30.0
    front_slow_min_angle_deg: -45.0
    front_slow_max_angle_deg: 45.0
    rear_stop_angle_width_deg: 30.0
    stop_distance_m: 0.35
    slow_distance_m: 0.60
    reverse_stop_distance_m: 0.25
    trajectory_clearance_margin_m: 0.02
    release_distance_m: 0.45
    release_time_s: 0.50
    reset_policy: manual
    zero_publish_period_s: 0.10
    max_forward_mps: 0.10
    max_angular_rad_s: 0.4
    allow_disable: false
    fail_on_missing_tf: true
```

## Diagnostics, operator status, and logging

Diagnostics must include at least:

- current state;
- previous state and transition reason;
- last scan age, frame ID, valid counts, min valid range per sector;
- whether scan-frame -> `base_link` TF is available, plus the TF failure reason and timeout;
- nearest front/rear/side obstacle distances;
- projected-trajectory horizon, minimum clearance, collision time/point, and
  configured footprint margin;
- requested command age and values;
- output command values;
- active limits/scales;
- driver service availability and last STOP/ESTOP result;
- whether launch parameters appear to be placeholders for lidar transform values.

Events should be sparse and meaningful:

```text
STARTUP_WAITING_FOR_SCAN
CLEAR
SLOW front=0.52 scale=0.68
STOPPED front=0.31
SENSOR_STALE age=0.42
MALFORMED_SCAN reason=angle_increment
ESTOPPED source=/estop driver_call=success
RESET_REJECTED reason=front_blocked
RESET_ACCEPTED
```

The TUI/status pane should consume `/collision_stop/state`, `/collision_stop/events`, and diagnostics so the operator sees the collision-stop state next to existing scan, TF, STOP/ESTOP, and armed fields.

## Test seams and fake-scan matrix

Implementation should keep pure Python functions for scan filtering and arbitration so unit tests do not need ROS installed. Suggested file split:

| File | Purpose |
|---|---|
| `src/sphero_rvr_driver/collision_stop.py` | Pure dataclasses/enums, sector filtering, hysteresis, arbitration. |
| `src/sphero_rvr_driver/collision_stop_node.py` | ROS 2 node wrapper, publishers/subscribers/services/diagnostics. |
| `launch/supervised_mapping.launch.py` or updated `mapping.launch.py` | Starts lidar + supervisor + remapped driver for motor-capable supervised graph. |
| `config/collision_stop.yaml` | Typed parameter defaults above. |
| `tests/test_collision_stop.py` | ROS-free scan/filter/state tests. |
| `tests/test_collision_stop_node_contract.py` | Import/AST/package/launch contract tests that do not require ROS runtime. |

Fake-scan matrix:

| Scenario | Expected |
|---|---|
| no scan at startup | `STARTUP` then `SENSOR_STALE`, zero output |
| fresh empty scan | malformed/blocked, zero output |
| NaN/Inf-only front sector | blocked/unknown, zero output |
| clear 360-degree scan | `CLEAR`, bounded command passes |
| front obstacle inside stop distance | `STOPPED`, zero, event emitted |
| front obstacle between stop and slow | `SLOW`, forward velocity scaled |
| obstacle clears just below release distance | remain `STOPPED` |
| obstacle clears beyond release distance for release time + reset | `CLEAR`, old command discarded |
| stale scan while moving | `SENSOR_STALE`, zero, driver stop called |
| reverse command with rear obstacle | reverse zero/hold |
| command whose full sampled trajectory monotonically increases separation from a known overlapping point | point may be excluded from that sweep; the opposite/approaching command remains blocked |
| angular command with side obstacle outside projected sweep | turn passes |
| angular command whose projected sweep reaches an obstacle | turn held at zero with trajectory evidence |
| `/stop` while clear | `STOPPED`, zero before driver service wait |
| `/estop` while clear | `ESTOPPED`, zero before driver service wait, future commands ignored |
| `/clear_estop` with stale scan | `SENSOR_STALE`, zero, no replay |
| malformed scan metadata | stop/fail closed |
| rosbag replay includes `/cmd_vel` | cannot reach driver in supervised launch; replay helpers still reject by default |
| supervisor crash simulated by no `/cmd_vel_motor` | driver stale timeout is relied on and documented in test/launch contract |

## Compatibility

### Manual TUI and key-tap

TUI can continue publishing requested velocity to `/cmd_vel` after motor-capable arming. In supervised launch, that topic is not the driver input; the supervisor arbitrates it. Existing STOP/ESTOP UI commands should call public `/stop` and `/estop`, which are supervisor-owned.

### Nav2

Nav2 normally publishes `Twist` to `/cmd_vel`. Keep that convention. Nav2 must not be remapped directly to `/cmd_vel_motor`; doing so is a release blocker.

### Browser, AI, and mission API

These layers may only request deterministic actions or publish/request ordinary velocity through the same `/cmd_vel` ingress. They must not call driver-private services or publish `/cmd_vel_motor`.

### Rosbag replay

Replay must stay hardware-safe:

- default replay helpers already reject `/cmd_vel` and motor-like topics;
- supervised launch prevents replayed `/cmd_vel` from reaching hardware directly;
- `/cmd_vel_motor` must be excluded from capture/replay defaults and rejected as unsafe;
- offline developer replay with unsafe topics may be allowed only when the live driver is not running.

## Migration plan

1. Add pure arbitration module and ROS-free tests.
2. Add `collision_stop_node.py`, `config/collision_stop.yaml`, and package entry point.
3. Add launch contract that starts supervisor and remaps driver:

   ```text
   /cmd_vel -> supervisor -> /cmd_vel_motor -> rvr driver
   driver stop/estop/clear_estop -> /rvr_driver/*
   supervisor stop/estop/clear_estop -> public services
   ```

4. Keep existing `rvr.launch.py` for low-level development only, but label it motor-capable and not collision-supervised.
5. Make motor-capable `mapping.launch.py start_rvr:=true` use the supervised graph by default, or add `start_collision_stop:=true` that must default true whenever `start_rvr:=true`.
6. Update `rvr-console` status pane and command gating to require supervisor healthy before `/arm confirm`.
7. Add tests that fail if a supervised launch lets the driver subscribe to public `/cmd_vel`.
8. Update rosbag unsafe-topic denylist to include `/cmd_vel_motor` explicitly.
9. Run ROS-free test suite and launch/package metadata checks.
10. Only after review, plan separate no-motion ROS graph validation. Physical validation is outside this design task.

Rollback:

- Revert launch changes to restore current direct `/cmd_vel -> sphero_rvr_driver` path only for controlled development.
- Do not use rollback for operator mapping unless the operator explicitly accepts that lidar collision-stop is absent.
- Keep docs warning that direct driver launch is motor-capable and not collision-supervised.

## No-go boundaries

- No implementation may allow ordinary sources to publish directly to `/cmd_vel_motor`.
- No launch may run Nav2/teleop/TUI/browser/AI directly against the driver `cmd_vel` subscription in motor-capable supervised mode.
- No stale/missing/malformed scan may be treated as clear.
- No ESTOP clear may replay a stored nonzero command.
- No rosbag replay path may reach hardware with `/cmd_vel_motor`.
- No physical validation, Pi deployment, UART access, lidar activation, calibration capture, or motion is authorized by this document.

## Acceptance criteria for implementation

- ROS-free tests cover scan health, sector filtering, velocity arbitration, hysteresis, STOP/ESTOP, stale scan, malformed scan, reset, and old-command discard.
- Launch/package tests verify the supervised graph remaps driver `cmd_vel` away from public `/cmd_vel` and exposes only supervisor-owned public safety services.
- `mapping.launch.py` cannot start a motor-capable graph without collision-stop supervision unless a deliberately named development override is set.
- TUI arming requires supervisor state `CLEAR` or explicitly acceptable `SLOW`, fresh scan, public STOP/ESTOP availability, and no ESTOP latch.
- Diagnostics and state/events topics expose enough information for operator status without source-code inspection.
- `git diff --check`, compile checks, package metadata tests, and full unit tests pass.
- Physical validation remains a separate gated plan after implementation review.
