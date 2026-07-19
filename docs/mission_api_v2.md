# mission_api.v2 typed rover capability registry

`mission_api.v2` is the ROS-free contract for a general-purpose LLM-operable rover layer. The product contract is no longer the shoe-mapping vertical slice. Shoe mapping is preserved as a compatibility fixture expressed through generic tools.

```text
Human goal -> planner -> typed tool invocation -> deterministic runtime/adapters
           -> independent collision/STOP/ESTOP/driver boundary
```

The planner may be dynamic. Execution is not. The deterministic runtime treats model JSON as untrusted input, validates every `ToolInvocation` against the registry immediately before execution, and fails closed when any tool, schema, approval, budget, or availability check is invalid. This is the fail-closed boundary.

## Contracts

### MissionGoal

`MissionGoal` carries a generic objective, constraints, success criteria, execution mode, budgets, and requested artifacts. It has no shoe-specific fields. Object-specific constraints such as supported detector classes live in tool metadata and plugin configuration.

### ToolDefinition

Every capability is a `ToolDefinition` with:

- stable `tool_id` and `version`;
- JSON-friendly typed argument and result schema;
- preconditions;
- explicit availability: `available`, `unavailable`, or `unsupported`;
- timeout and cancellation semantics;
- safety class and approval class;
- resource ownership;
- documented effects.

`ToolDefinition` is the allowlist. No arbitrary ROS topics, raw Python callbacks, serial packets, motor commands, or generic bridge calls are represented as tools.

### ToolInvocation

`ToolInvocation` contains the planner-produced call envelope: correlation id, tool id/version, bounded arguments, optional approval grant, request time, and provenance. Arguments are recursively checked for unsafe ROS surface strings before schema validation continues.

### ToolResult

`ToolResult` contains correlation id via the original invocation, status, timestamps, structured observation/error, artifact references, and audit provenance. Results reference artifacts instead of embedding bulky maps/images/reports.

### MissionPlan

`MissionPlan` is an ordered set of invocations under a `MissionGoal`. Dependency edges are rejected until ordering semantics are implemented; ignored dependencies are not part of the contract. Planner-selected budgets are requests only: the deterministic runtime enforces trusted system ceilings for maximum steps, runtime, and travel where applicable. Unsupported tools, excess budgets, unbounded motion, stale approvals, and unavailable capabilities fail before execution or halt the deterministic runtime with an auditable terminal result.

## Initial tool lifecycle

1. Registry is built from installed deterministic adapters and detector plugins.
2. Planner emits typed JSON: `MissionGoal` plus `MissionPlan` invocations.
3. Runtime validates plan budgets.
4. Runtime validates each invocation immediately before execution:
   - tool id/version exists;
   - capability is available;
   - no direct ROS or motor surface appears in arguments;
   - arguments match schema and bounds;
   - required approval classes are fresh;
   - travel/runtime/step budgets remain inside trusted system ceilings;
   - a tool whose effective timeout cannot fit inside the remaining mission runtime is not started;
   - completed adapter observations and artifact refs match the declared result schema before crossing the typed boundary.
5. Adapter executes one deterministic project capability.
6. Runtime enforces the tool timeout, converts overrun to deterministic `timeout` with cancellation-cleanup audit state, then records `ToolResult` and audit entry.
7. Cancellation, timeout, BLOCKED, STOP, and ESTOP states latch and prevent later success resurrection.

## Approval classes

Current approval classes are:

- `none`: read-only, perception, localization, or artifact generation with no motor authority.
- `supervised_motion`: bounded motion/control tools that must pass through the supervised coordinator or range-motion path.

The runtime validates an `ApprovalGrant` against the tool approval class and its expiration at the time the invocation would start, not only at plan start. A stale or missing approval fails closed. `pause_cancel_stop_estop` is an authoritative safety-control surface and does not require a fresh motion approval to latch cancel/STOP/ESTOP. Physical motor authority remains separately gated by the v1 controls surface and independent robot-side STOP/ESTOP/collision supervisor until a v2 physical-control adapter is explicitly added.

## Initial capability surface

The default registry represents these deterministic tools:

- `map_localize`: replay/live map-localization workflow hook.
- `bounded_exploration_segment`: deterministic exploration segment through supervised motion.
- `move_to_clearance`: bounded range-motion primitive with caller-supplied clearance, speed, timeout, and max travel.
- `rotate_scan`: rotate/scan hook, explicitly reportable as unavailable when no safe adapter exists.
- `capture_observation`: replay/camera/lidar observation capture.
- `detect_objects`: detector plugin call with `object_class` constrained by installed detector classes.
- `project_detections_to_map`: map-frame semantic projection.
- `generate_semantic_artifacts`: semantic map/report/artifact reference generation.
- `query_status_telemetry`: read-only mission telemetry.
- `pause_cancel_stop_estop`: control semantics for pause/cancel/STOP/ESTOP propagation.

`move_to_clearance` is how a user goal like “move until four inches from the object” becomes executable without exposing direct raw movement. The tool requires `clearance_m`, `speed_mps`, `timeout_s`, and `max_travel_m`. The adapter implementation stays on the bounded path:

```text
range_motion -> /cmd_vel -> collision_stop -> /cmd_vel_motor
```

The final motor sink is not a planner surface; it remains owned by the independent safety graph. Internal path/topic names are documentation and adapter internals, not fields returned through `ToolResult`.

## Compatibility with mission_api.v1

`mission_api.v1` and the constrained VS09 shoe translator remain intact. `build_canonical_shoe_mapping_v2_plan()` expresses the canonical shoe request as a generic v2 goal/plan using `map_localize`, `bounded_exploration_segment`, `capture_observation`, `detect_objects(object_class="shoe")`, projection, and artifact generation.

That adapter is a conformance fixture, not the v2 product boundary. A different detector plugin can provide `object_class="backpack"` without changing the generic core.

## Runtime outcomes

Terminal runtime statuses are deterministic and auditable:

- `complete`
- `failed`
- `blocked`
- `cancelled`
- `timeout`
- `stopped`
- `estopped`

A terminal result stops later tool execution. There is no “continue anyway” path because that is how tiny robots become tiny lawsuits with wheels.

## Extension guide

To add a capability:

1. Add a deterministic adapter function that invokes one project capability. Do not expose a topic name or generic Python callable.
2. Add a `ToolDefinition` with schema bounds, result schema, preconditions, availability, timeout, cancellation semantics, safety class, approval class, resource ownership, and documented effects.
3. Keep unavailable hardware or unimplemented adapters as `unavailable`/`unsupported`; do not fake a passing result.
4. Add ROS-free tests for valid execution and every safety rejection: unknown tool, schema-invalid args, unsafe ROS strings, stale approvals, excess budgets, and unavailable capability.
5. Document the tool lifecycle and operator approvals here.

No arbitrary ROS topics, services, serial packets, direct movement aliases, or raw motor commands belong in `mission_api.v2`.
