# Versioned Mission API and deterministic state machine

`src/sphero_rvr_driver/mission_api.py` is the ROS-free Mission API contract for the canonical shoe-mapping command:

```text
Map the room and identify every shoe. Put it on a map.
```

The Mission API is deliberately narrow. It validates an allowlisted semantic mission and emits a deterministic mission command for downstream orchestration; it does not expose a generic ROS bridge and it rejects direct `/cmd_vel`, `/cmd_vel_motor`, motor, raw motor, and teleop requests.

## Supported version and mission type

Version:

```text
mission_api.v1
```

Supported mission type:

```text
semantic_room_shoe_mapping
```

Canonical command path:

```text
mission_api
  -> supervised_coordinator
  -> range_motion
  -> /cmd_vel
  -> collision_stop
```

The final `/cmd_vel_motor` path remains owned by the independent supervised/collision-stop graph. Mission API clients do not get arbitrary topic publishing.

## Request schema

`MissionRequest` contains:

- `api_version`: currently `mission_api.v1` only.
- `mission_id`: stable caller-provided id.
- `mission_type`: currently `semantic_room_shoe_mapping` only.
- `room_mapping`: semantic room mapping parameters:
  - `map_name`
  - `semantic_labels`, currently exactly `shoe`
  - `frame_id`, default `map`
  - `source_frame_id`, default `base_link`
  - `occupancy_resolution_m`, positive finite resolution
  - `require_artifact_references`, default `true`
- `safety`: bounded start/cancel/estop controls:
  - `start_requires_supervised_motion: true`
  - `cancel_supported: true`
  - `estop_supported: true`
  - `max_runtime_s > 0`
  - `max_segments > 0`
  - `allow_direct_ros_commands: false`
- `artifacts`: result contracts. Required kinds are `occupancy_map`, `semantic_map`, and `shoe_detections`.
- `requested_ros_topics` and `raw_ros_command`: validation tripwires. Unsafe/direct ROS requests are rejected before a `MissionCommand` is produced.

`build_canonical_shoe_mapping_request()` constructs the default request for the plain-English translator.

## Capability checks

`validate_mission_request()` requires `CapabilitySet` to confirm all of:

- `semantic_mapping`
- `slam_replay_or_live_mapping`
- `shoe_detection`
- `supervised_motion`
- `collision_stop`
- `estop`
- `artifacts`

Missing capabilities raise `MissionValidationError`; unsupported/unsafe missions never degrade into generic ROS commands.

## State machine

`MissionStateMachine` exposes these states:

- `IDLE`
- `VALIDATING`
- `MAPPING`
- `EXPLORING`
- `DETECTING`
- `FINALIZING`
- `COMPLETE`
- `PAUSED`
- `CANCELLED`
- `BLOCKED`
- `ESTOPPED`
- `FAILED`

The canonical success path is:

```text
IDLE
  --start_requested--> VALIDATING
  --validated--> MAPPING
  --mapping_started--> EXPLORING
  --exploration_started--> DETECTING
  --detection_started--> FINALIZING
  --finalize_started/complete--> COMPLETE
```

Cancel, estop, blocked, and failed paths latch terminal states. Once latched, later success events do not resurrect the mission. That gives start/cancel controls and status dashboards predictable behavior instead of callback roulette with wheels.

## Stable events, telemetry, and results

`MissionSnapshot.to_json_dict()` is the stable web/PWA and translator surface:

- `mission_id`
- `api_version`
- `state`
- `event_log`
- `telemetry`
- `result`

Telemetry includes:

- `state`
- `terminal`
- `cancel_supported`
- `estop_supported`
- bounded `max_runtime_s` and `max_segments`
- `command_path`
- `generic_ros_bridge: false`
- `direct_ros_commands_allowed: false`
- `required_artifacts`
- `result_artifacts`
- optional `range_motion_stop_reason`
- terminal `reason`

`complete()` validates references for every required artifact before returning a complete result. The result does not embed bulky artifacts; it carries stable references such as map YAML paths and detection JSON paths for downstream consumers.
