# RVR API parity scope

This document defines what "entire RVR API" means for this repository before the parity implementation work begins.

## Decision summary

`entire RVR API` means: **all public, local-control Sphero RVR capabilities exposed by the official Sphero Python SDK, inventoried with an implementation decision for `sphero_rvr_core`, and exposed to ROS only when the surface is operationally useful and safe.**

Full parity is a **core-driver goal**, not a promise that every command becomes a ROS topic/service/action.

## Authoritative sources

Use this source hierarchy when building the API matrix or resolving disagreements:

1. **Official Sphero Python SDK public RVR API** — primary source of truth for what the project should inventory.
   - Current SDK package used for planning: `sphero-sdk==0.3.4.post2`.
   - Primary class/module family: `sphero_sdk.asyncio.client.toys.sphero_rvr_async.SpheroRvrAsync` plus its command modules under `sphero_sdk.common.commands`.
   - The async API is preferred because this project is already concurrency/dispatcher based.
2. **Official packet/protocol documentation and SDK command builders** — authoritative for device IDs, command IDs, targets, payload layouts, response payload layouts, notification payload layouts, and packet semantics.
   - Use this when the public SDK method name is clear but packet details are incomplete or ambiguous.
3. **Installed SDK introspection** — allowed as a reproducible snapshot of the official SDK surface, especially to generate method inventories.
   - Introspection is not a higher authority than the official SDK intent; it is a way to avoid hand-copying mistakes.
   - Record SDK version when generating the matrix.
4. **Prior implementation reconnaissance and this repo's current implementation** — useful evidence, not authoritative.
   - Existing files such as `src/sphero_rvr_core/commands.py`, `driver.py`, and tests show what already works, but they must not silently define the final parity boundary.

If sources conflict, prefer the official SDK public API for inclusion decisions and protocol docs / SDK command builders for wire-format details.

## Initial public SDK inventory boundary

The parity matrix should start from the public coroutine methods on `SpheroRvrAsync` and the public command-builder functions in these modules:

- `sphero_sdk.common.commands.api_and_shell`
- `sphero_sdk.common.commands.connection`
- `sphero_sdk.common.commands.drive`
- `sphero_sdk.common.commands.io`
- `sphero_sdk.common.commands.power`
- `sphero_sdk.common.commands.sensor`
- `sphero_sdk.common.commands.system_info`

Public RVR SDK capabilities to inventory include, at minimum:

- Drive/control: raw motors, drive with heading, reset yaw, reset locator, locator flags.
- LEDs/palettes: set all LEDs, load/save/activate palettes, release LED requests.
- Power: wake, sleep, battery percentage/voltage/state/thresholds, current sense, sleep notifications.
- Sensors: RGB/clear sensor, ambient light, color detection, detected color, streaming service configure/start/stop/clear, gyro max, motor temperature, thermal protection, infrared readings.
- Infrared/robot-to-robot: send infrared message, broadcast/follow/evade start-stop, infrared message notifications.
- Motor protection: motor fault/stall state and notifications, thermal protection status and notifications.
- System info / connection / API shell: firmware/main app version, bootloader version, board revision, processor name, SKU, MAC address, stats ID, core uptime, Bluetooth advertising name, echo.

The inventory must assign every public item one of these statuses:

- `core-implemented`
- `core-planned`
- `core-only`
- `ros-exposed`
- `ros-intentionally-omitted`
- `omitted-deprecated-or-private`
- `omitted-unsafe-or-admin`
- `needs-protocol-research`

## Deprecated and private SDK methods

Deprecated/private SDK methods are **inventory-only by default**.

Include them in the matrix only when they are visible during SDK inspection or referenced by official docs, and mark them explicitly:

- Methods beginning with `_` are private and not part of the implementation target unless a public method delegates to them and the behavior is required internally.
- Deprecated methods are not implemented unless they are the only public path for a useful RVR capability.
- Test/support/demo helpers are out of scope unless they expose a real local RVR command that is missing elsewhere.

The matrix should explain each omission briefly so future workers do not rediscover the same trap with a flashlight and optimism.

## Firmware, admin, update, and factory-style commands

Firmware/admin/update/factory-style capabilities are **in scope for inventory** but **not automatically in scope for implementation**.

Default disposition:

- **Core driver:** implement only when the command is safe, useful for local operation, and can be tested without bricking/update flows or hidden state transitions.
- **ROS adapter:** omit by default unless the command is clearly operational and safe for routine robot use.
- **Matrix:** record unsupported/admin commands as `core-only`, `omitted-unsafe-or-admin`, or `needs-protocol-research` with rationale.

Examples that should normally stay out of ROS exposure:

- Firmware flashing/update flows.
- Factory reset/calibration routines that can degrade the robot if run casually.
- Bootloader/admin operations.
- Manufacturing, diagnostics, or hidden service commands not represented as ordinary local-control SDK APIs.

Read-only firmware/version/system-info queries are safe candidates for core implementation and diagnostics-style ROS exposure.

## Notification and event APIs

Notification/event APIs are **in scope** for core parity when represented by the official local RVR API.

Expected core routing semantics:

1. The dispatcher owns the single serial stream and demultiplexes packets into:
   - request/response completions keyed by sequence ID; and
   - unsolicited notifications keyed by `(device_id, command_id[, target])`.
2. Core exposes notification registration/subscription APIs that allow multiple local consumers without stealing packets from request/response handling.
3. Notification enable/disable commands are separate from callback registration. Registering a callback must not silently enable robot-side streaming unless the API explicitly documents that behavior.
4. Notification callbacks must be non-blocking from the serial reader's perspective; slow consumers should be queued, dropped with metrics, or otherwise isolated.
5. Shutdown/disconnect disables or drains notification routes cleanly where the robot protocol supports it.

Notification APIs to inventory include at least:

- battery voltage state change
- will sleep / did sleep
- color detection
- gyro max
- motor fault
- motor stall
- motor thermal protection status
- robot-to-robot infrared message received
- streaming service data

ROS exposure should be conservative:

- Expose operational telemetry as topics or diagnostics when it helps robot operation.
- Do not expose low-level callback plumbing directly as a bag of arbitrary protocol events.
- Prefer typed messages or diagnostics over opaque byte dumps unless a debug-only interface is explicitly added.

## Unsafe-but-valid core capabilities

Some capabilities are valid local RVR APIs but unsafe or too low-level for ordinary ROS exposure. They still count as complete when the core driver has:

- a typed command builder with documented packet target, payload, and response parser when applicable;
- async driver method or intentionally internal method matching the SDK capability;
- dispatcher-safe request/response or notification handling;
- validation/clamping for payload ranges;
- tests using fake transport/packet assertions;
- explicit safety notes in docs or the parity matrix; and
- a clear ROS decision of `ros-exposed` or `ros-intentionally-omitted`.

For ROS, completeness means a deliberate bridge decision, not maximum exposure. Motor-capable, persistent-state, calibration, admin, and high-rate streaming capabilities need extra scrutiny before becoming ROS services/topics/actions.

## ROS exposure policy

`sphero_rvr_core` is the parity layer. `sphero_rvr_driver` is the ROS operational layer.

Expose to ROS when the capability is:

- useful for robot operation, diagnostics, autonomy, or operator feedback;
- safe under this project's motor-safety model;
- representable as a stable topic/service/action without leaking protocol footguns; and
- testable without requiring hidden robot state or risky update/factory flows.

Keep core-only when the capability is:

- primarily protocol/admin/debug oriented;
- useful for scripts/tests but not routine ROS graph consumers;
- too easy to misuse from a generic service call; or
- better surfaced later through a higher-level, safer abstraction.

Odometry/TF remains a ROS/SLAM bridge concern downstream of core parity. Encoder, locator, yaw, and IMU-like data can be core parity items; publishing `/odom` and `tf` is a separate ROS integration decision.

## Explicitly out of scope

The following are not part of `entire RVR API` for this board unless a later task explicitly adds them:

- Sphero mobile app UX, app screens, macros, lessons, blocks, or user workflows.
- Cloud services, account management, user identity, registration, device ownership, fleet/account APIs, or telemetry upload not represented in the local RVR SDK/protocol.
- Bluetooth/mobile connection management beyond local RVR connection/protocol capabilities needed by this project.
- Non-RVR Sphero toys except where shared protocol code is needed to understand RVR packet semantics.
- Reverse-engineering hidden firmware/manufacturing commands not present in the official SDK/protocol docs.
- Autonomous navigation, SLAM behavior, perception, AI command UX, and mobile-app-equivalent features. Those may consume the driver later, but they do not define API parity.

## Acceptance criteria for the next parity task

A future API matrix/implementation task is complete only when it:

1. records the SDK version and source files used for inventory;
2. lists every public local RVR SDK capability in the inventory boundary above;
3. records current repo support and target disposition for each capability;
4. explicitly marks deprecated/private/admin/unsafe omissions;
5. separates `sphero_rvr_core` parity from ROS exposure decisions; and
6. identifies every `needs-protocol-research` item with the missing packet/response/notification detail.
