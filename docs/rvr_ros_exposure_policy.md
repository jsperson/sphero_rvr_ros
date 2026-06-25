# RVR ROS exposure policy

This policy defines which Sphero RVR capabilities should become ROS topics, services, actions, parameters, or diagnostics after `sphero_rvr_core` reaches full API parity.

The short version: **`sphero_rvr_core` is allowed to know the whole local RVR API; `sphero_rvr_driver` should expose only safe robot operations.** Full core parity does not mean turning every packet builder into a ROS service. A ROS graph is often shared by launch files, teleop tools, autonomy stacks, bags, scripts, and curious humans. That makes it the wrong place for a hardware footgun buffet.

## Source and scope

This policy follows the planning decisions captured for the API parity board:

1. The official Sphero Python SDK is the primary API source of truth; protocol docs resolve packet-level details when needed.
2. Firmware, admin, update, and factory-style commands are inventoried, but default to `core-only` or intentionally omitted unless they are safe and useful.
3. Deprecated/private SDK methods are inventoried when visible, but not implemented unless public/useful; omissions are explicit in the matrix.
4. Full API parity belongs in `sphero_rvr_core`.
5. ROS exposure remains conservative: operational robot surfaces only, not every low-level hardware/admin function.
6. Odometry/TF stays on the ROS/SLAM bridge path, downstream of core API parity.

Related documents:

- `docs/rvr_api_parity_scope.md` defines the core parity boundary.
- `docs/rvr_capability_matrix.md` inventories official SDK/protocol capabilities.
- `docs/rvr_api_gap_report.md` compares that matrix to the current repo.
- `docs/rvr_notification_events.md` inventories notification/event routing.

## Exposure vocabulary

| Decision | Meaning |
|---|---|
| `ros-exposed` | Intended for routine ROS graph use as a topic, service, action, parameter, transform, or diagnostic field. Must be safe, typed, testable, and operationally useful. |
| `core-only` | Implement or keep in `sphero_rvr_core`, but do not publish as a normal ROS surface. May be used internally by the ROS node or by diagnostic scripts. |
| `intentionally-omitted` | Deliberately not implemented in ROS, even if core supports it. Usually too niche, protocol-shaped, persistent-stateful, or better represented by a safer abstraction later. |
| `unsafe-to-expose` | Valid hardware capability that should not be reachable from generic ROS callers because it can cause uncontrolled motion, persistent device changes, firmware/admin transitions, or difficult-to-debug state. |

`unsafe-to-expose` is stronger than `intentionally-omitted`: it means future work should not add a ROS bridge without a new explicit safety design and human approval.

## General ROS exposure rules

Expose a capability to ROS only when it satisfies all of these:

1. **Operational value:** it helps drive, stop, observe, diagnose, map, localize, or safely operate the robot.
2. **Stable abstraction:** it maps cleanly to a typed ROS message/service/action instead of leaking raw DID/CID/payload mechanics.
3. **Safety bounded:** motor effects are gated by the existing velocity/stop/estop model or by an equally clear policy.
4. **Routine-safe:** a launch file, teleop tool, autonomy node, or script can call it without surprising persistent device changes.
5. **Testable without hardware damage:** fake-driver/unit tests can validate the behavior, and live tests do not require update/factory/admin flows.
6. **Observable failure:** errors surface through service responses, diagnostics, logs, or topic state rather than silently changing hidden robot state.

Keep a capability out of ROS when it is primarily:

- packet/protocol plumbing;
- firmware/admin/factory/update behavior;
- persistent calibration or palette state;
- raw motor or drive-mode control below the safe velocity abstraction;
- high-rate opaque bytes without typed interpretation;
- deprecated/private SDK surface;
- useful only for one-off debugging; or
- a feature whose failure mode is “the robot moves weirdly and nobody knows why.”

## Category policy matrix

| Capability category | ROS decision | Allowed ROS shape | Rationale / guardrails |
|---|---|---|---|
| `/cmd_vel` velocity control | `ros-exposed` | `geometry_msgs/msg/Twist` subscription | Primary operational base interface. It must remain bounded by configured max linear/angular speeds, raw motor duty cap, stale-command timeout, and software estop state. ROS should never expose raw motor duty as the normal base-control path. |
| `stop` | `ros-exposed` | `std_srvs/srv/Trigger` service or equivalent safe action | Operational safety surface. Sends zero motor command promptly and should be safe to call repeatedly. |
| `estop` / `clear_estop` | `ros-exposed` | `std_srvs/srv/Trigger` services | Estop must gate future motion in software. Clearing estop is allowed because it does not itself command motion; operators still need an explicit subsequent velocity command. |
| Driver diagnostics | `ros-exposed` | `diagnostic_msgs/msg/DiagnosticArray` | Required for operator confidence and automation gates: connected state, estop state, last velocity, polling health, stale-command status, fault/thermal/battery warnings as they become available. |
| Battery percentage and voltage | `ros-exposed` | `sensor_msgs/msg/BatteryState` plus diagnostics | Routine telemetry. Safe read-only data used by operators and autonomy. Voltage-state events/thresholds can be diagnostics once notification routing is stable. |
| Odometry | `ros-exposed` once implemented | `/odom` as `nav_msgs/msg/Odometry` | Required for SLAM/localization readiness. Must be documented as estimate quality, especially if derived from encoders/locator/yaw with skid-steer drift. |
| TF | `ros-exposed` once implemented | `odom -> base_link`; static `base_link -> sensor` transforms | Required ROS integration surface for lidar, camera, SLAM, and Nav2. Frame naming and timestamp semantics matter more than exposing extra raw sensors. |
| Mapping/SLAM sensor topics | `ros-exposed` when typed/stable | `/scan`, selected IMU/yaw/encoder/locator/color/ambient topics as appropriate | Expose selected read-only sensor data when it helps mapping, localization, perception, diagnostics, or operator feedback. Avoid dumping generic streaming bytes into ROS without a typed topic contract. |
| Reset yaw / reset locator | `ros-exposed` candidate | Guarded `Trigger` services | Operationally useful during localization setup, but affects reference frames. Service names and responses must make the state reset explicit. |
| Firmware/main app/bootloader version, SKU, board revision, processor, uptime | `ros-exposed` as diagnostics or info service | Diagnostics key-values or read-only info service | Safe read-only metadata. Good for support/debug. Avoid exposing privacy-ish identifiers like MAC/stats ID by default. |
| Bluetooth advertising name | `core-only` by default | Maybe diagnostics/debug service later | Safe read-only, but rarely needed by ROS graph consumers. Keep out unless a diagnostic workflow needs it. |
| MAC address / stats ID | `core-only` or `intentionally-omitted` | No routine ROS topic | Identifiers are useful for debugging but not normal robot operation; avoid publishing stable identifiers broadly. |
| LEDs | `ros-exposed` candidate for safe conveniences; raw forms `core-only` | Guarded services for simple colors/groups; optional lifecycle cleanup via `release_led_requests` | LEDs are convenient operator feedback. Expose high-level bounded color/group controls, not arbitrary mask/vector payloads unless a later UI explicitly needs them. Palette persistence stays out of ROS. |
| Color/RGBC/ambient readings | `ros-exposed` candidate | Typed sensor topics or low-rate polling services | Safe read-only perception/debug data. Color detection notifications require notification routing and protocol confirmation before ROS topics. |
| Motor temperature / thermal protection | `ros-exposed` as diagnostics/topics after protocol parity | `sensor_msgs/msg/Temperature`, diagnostics, event topics | Safe telemetry used to protect hardware. Keep write/enable mechanics internal; expose typed status, not callback plumbing. |
| Motor stall/fault notifications | `ros-exposed` as diagnostics/events after notification routing | Diagnostics or typed event topics | Operationally important safety telemetry. The ROS node may enable and subscribe internally, but callers should consume typed state/events. |
| Gyro max notifications | `ros-exposed` candidate as diagnostics/event | Typed event/diagnostics | Useful health/limit indication. Do not expose only the raw bitmask without interpretation. |
| IR readings / IR received events | `ros-exposed` candidate for debug/perception | Low-rate typed topic or debug service | Read-only IR data can be useful. Keep motion-affecting IR behaviors out until a deliberate safety design exists. |
| Echo / transport ping | `core-only` | No normal ROS surface; optional debug-only service if needed | Useful for protocol tests, not routine robot operation. If exposed, it must be clearly debug-only and bounded. |
| Sleep / wake | `core-only`; sleep `intentionally-omitted` from routine ROS | Wake may remain node lifecycle/internal; no casual sleep service | Wake is already part of connect behavior. Sleep is disruptive and can strand the driver; expose only with an explicit guarded power-management design. |
| Streaming service configure/start/stop/clear/data | `core-only` until typed design | No opaque byte topic by default | High-rate generic stream configuration is protocol plumbing. Expose derived typed topics only after choosing exactly which sensors, rates, QoS, and cleanup semantics are safe. |
| Palettes, color identification helpers, palette load/save | `core-only` or `intentionally-omitted` | No routine ROS surface | Persistent-ish LED/color state and calibration/debug helpers do not belong in the default graph. High-level LED services can use core helpers internally. |
| Calibration commands | `intentionally-omitted` or `unsafe-to-expose` depending on effect | No default ROS surface | Calibration can degrade future readings if run casually. Any future exposure needs an operator workflow, preconditions, and explicit confirmation. |
| Raw motors | `unsafe-to-expose` | Never as generic ROS service/topic | Raw motors bypass `/cmd_vel` velocity limits, stale timeout semantics, and motion policy; generic raw motors belong below the ROS safety boundary. Core can implement them because packets exist; ROS should use safe velocity/stop abstractions. |
| Drive modes / drive-with-heading / autonomous drive primitives | `core-only` or guarded `ros-exposed` candidate | Prefer high-level action with safety constraints, not raw parameter service | Some are useful but motor-affecting. Expose only when bounded by speed, timeout, cancel/stop behavior, and clear frame semantics. |
| IR follow/evade/broadcast motion behaviors | `intentionally-omitted` initially; possibly `unsafe-to-expose` for routine graph | No default ROS surface | These can produce robot motion or interaction effects outside the normal `/cmd_vel` path. Keep core-only until explicitly designed as safe behaviors. |
| Firmware/admin/update/factory functions | `unsafe-to-expose` or `intentionally-omitted` | No ROS surface | ROS should not casually expose firmware flashing, bootloader/admin, factory reset, manufacturing, or hidden service commands. Inventory them; do not bridge them. |
| Deprecated/private SDK methods | `intentionally-omitted` | No ROS surface | Inventory if discovered, but do not implement or bridge unless they are the only safe public path for an operational capability. |

## Current and target operational base surfaces

Current ROS base surface from `sphero_rvr_driver`:

- subscribes: `/cmd_vel`
- services: `stop`, `estop`, `clear_estop`
- publishes: `battery_state`, `left_motor_temperature`, `right_motor_temperature`, `ambient_light`, `odom`, `diagnostics`, and `odom -> base_link` TF

Current additional safe operator surfaces:

- subscribes: `set_all_leds` (`std_msgs/msg/ColorRGBA`) for bounded all-LED feedback;
- services: `reset_yaw`, `reset_locator`, and `release_led_requests` (`std_srvs/srv/Trigger`).

These are the right defaults. Future expansion should add operational state, not raw device reachability:

- richer diagnostics for firmware/version, battery voltage state/thresholds, motor fault/stall/thermal state;
- typed sensor topics for selected read-only sensors that support mapping, perception, or debugging.

## Mapping and SLAM readiness

Mapping/SLAM needs a small, reliable ROS surface rather than the whole RVR API:

1. `/cmd_vel` remains the only routine base motion command.
2. `stop` and `estop` remain operator/autonomy safety gates.
3. `/odom` should publish the best available RVR odometry estimate with honest covariance/quality notes.
4. `tf` should provide at least `odom -> base_link`; lidar/camera launch files should add measured static transforms such as `base_link -> laser`.
5. Selected sensor topics should be typed and purposeful:
   - encoder/locator/yaw/IMU-like data only if it improves odometry/localization;
   - ambient/RGBC/color topics only if used by perception/debug workflows;
   - fault, thermal, stall, and battery events as diagnostics, not generic raw notifications.
6. Opaque streaming-service bytes should not be exposed directly. Configure streaming internally only to support named typed topics with bounded rates and cleanup.

## Convenience and admin capabilities

Convenience/admin capabilities split into three buckets:

- **Safe read-only diagnostics:** firmware/main app version, bootloader version, SKU, board revision, processor name, uptime, battery thresholds, thermal/fault state. These can appear in diagnostics or a read-only info service.
- **Operator convenience:** LEDs and reset services can be useful, but should be high-level, bounded, and reversible. Raw LED masks, palettes, and persistent saves stay core-only.
- **Admin/persistent/disruptive:** firmware update, bootloader/admin, factory reset, sleep, calibration, palette persistence, MAC/stats identifiers. These are not routine ROS graph surfaces.

## Dangerous and low-level controls

The following are not acceptable default ROS surfaces:

- raw motor duty commands;
- arbitrary DID/CID packet send services;
- drive-mode setters that bypass the velocity safety model;
- firmware/update/admin/factory flows;
- calibration routines without explicit workflow safeguards;
- autonomous IR follow/evade/broadcast behaviors;
- low-level streaming byte configuration as a generic user-facing API.

If future work wants any of these, it needs a new design doc that answers:

1. Who is the caller?
2. What prevents accidental motion or persistent device damage?
3. How does stop/estop cancel or override it?
4. What state changes persist after node restart?
5. How is it tested without hardware risk?
6. Why is a safer high-level abstraction insufficient?

Until those answers exist, the policy is no. Tiny robot, real treads, surprisingly large consequences.

## Implementation checklist for future ROS exposure cards

Before adding a new ROS topic/service/action:

1. Classify the underlying core capability using this document.
2. Confirm the packet/API semantics in `docs/rvr_capability_matrix.md` and protocol research notes.
3. Prefer typed ROS messages and stable names over protocol-shaped payloads.
4. Define safety limits, timeout/cancel behavior, and estop interaction.
5. Add fake-driver or node-level tests that exercise success, failure, and shutdown behavior.
6. Update this policy or the capability matrix if the exposure decision changes.
7. For live robot validation, follow `STATUS.md` safety rule: warn that the action can start RVR motors and get explicit approval before motor-capable commands.
