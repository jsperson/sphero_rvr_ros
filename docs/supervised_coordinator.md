# Deterministic supervised mapping/navigation coordinator

`SupervisedCoordinator` is the ROS-free mission coordinator contract for the shoe-mapping vertical slice. It sequences bounded mapping/exploration segments by emitting only `range_motion` goals; it never publishes directly to `/cmd_vel_motor`.

```text
Mission API / authenticated controls / read-only UI
  -> SupervisedCoordinator
  -> range_motion -> /cmd_vel -> collision_stop -> /cmd_vel_motor
  -> sphero_rvr_driver
```

The coordinator is deliberately not a motor controller. Its output is a `RangeMotionCommand` with:

- `channel: "range_motion"`
- `topic: "/range_motion/goal"`
- `motor_topic: null`
- a bounded `MotionGoal` containing direction, target clearance, max measured displacement, and timeout.

The independent collision-stop supervisor remains the only owner of final `/cmd_vel_motor` publication in supervised launches.

## ROS-free core contract

The tested core API is `sphero_rvr_driver.supervised_coordinator`:

- `CoordinatorConfig`: mission id, max segment count, segment target clearance, max measured displacement per segment, timeout, and command topic.
- `DeterministicSegmentSelector`: deterministic, single-pass segment selector. It walks fixed observable directions (`forward`, then `backward` by default), rejects clearances that cannot support a bounded segment, and never reuses a direction in the same mission.
- `SupervisedCoordinator`: mission state machine. It starts a segment, exposes the pending range-motion goal, consumes segment status, and latches fail-closed safety events.
- `CoordinatorTelemetry`: Mission API/read-only UI payload with phase, stop reason, active segment, completed segments, measured displacement, cancellation requirement, observable safety state, and the fixed command path.

This is intentionally no random bump-and-turn. Segment choice is deterministic and bounded from an observable clearance snapshot; future map-aware selectors should keep the same property.

Adaptive mission uses a separate LLM strategy layer but preserves this ownership rule:
the model supplies one typed bounded intent, `PhysicalAdaptiveMissionExecutor` converts
it to one live-route segment, and deterministic route and collision components
retain all velocity and stop authority. Adaptive mission does not call range-motion,
publish ROS, or create a second `/cmd_vel_motor` owner.

## State model

Phases:

- `IDLE`: no segment is active.
- `RUNNING_SEGMENT`: one range-motion segment is active and cancellation can deterministically target that segment.
- `COMPLETE`: the bounded segment budget has finished or no further deterministic segment remains after progress.
- `FAILED_CLOSED`: the mission is latched safe; no further range-motion goals are returned until a new coordinator instance/session is created.

Coordinator stop reasons:

- `complete`
- `no_segment_available`
- `range_motion_failed`
- `stop`
- `estop`
- `collision_stop`
- `shutdown`

Range-motion failures such as stale sensors, lost target, unsafe clearance, stalls, timeouts, odom/lidar disagreement, and driver faults are treated as `range_motion_failed` at mission level and preserve the underlying range-motion stop reason in telemetry.

## Safety and cancellation

STOP, ESTOP, `collision_stop`, and shutdown are independent mission events. Any of them immediately transitions to `FAILED_CLOSED`, clears the pending range-motion command, sets `cancellation_required: true`, and records an `observable_safety_state` matching the event.

Once failed-closed, later successful segment statuses do not resurrect the mission. That makes cancellation deterministic rather than “best effort unless a callback arrives late,” a tiny but important bit of robot sanity.

## Mission API and read-only UI telemetry

`CoordinatorTelemetry.to_dict()` is the JSON-friendly surface downstream services should expose:

```json
{
  "mission_id": "shoe-map-demo",
  "phase": "RUNNING_SEGMENT",
  "stop_reason": "none",
  "completed_segments": 0,
  "total_measured_displacement_m": 0.0,
  "active_segment": {
    "index": 0,
    "direction": "forward",
    "observed_clearance_m": 0.6,
    "target_clearance_m": 0.25,
    "max_displacement_m": 0.35,
    "timeout_s": 8.0
  },
  "range_motion_stop_reason": null,
  "observable_safety_state": "clear",
  "cancellation_required": false,
  "command_path": ["range_motion", "/cmd_vel", "collision_stop", "/cmd_vel_motor"],
  "cmd_vel_motor_publish_allowed": false,
  "safety": {
    "fail_closed": true,
    "stop_estop_collision_authoritative": true,
    "shutdown_cancels_active_segment": true
  }
}
```

Mission API may use this as a read/write control contract around start/cancel/status. Read-only UI should consume it directly and must not publish motion. Authenticated controls should route start/cancel through the Mission API, not through arbitrary `/cmd_vel` publishing.

## Current limits

- The shipped selector is intentionally simple: bounded `forward`/`backward` linear segments only. It is a safe deterministic seam for VS03, not a full SLAM frontier planner.
- Live execution remains motor-capable if range-motion is connected to a supervised launch. Do not run live range-motion goals without the physical/human gate.
- Replay assets without `/odom` can exercise interface/state-machine behavior, but cannot prove full odom/lidar disagreement handling.
