# Mission API v2 rover tool registry

`src/sphero_rvr_driver/mission_api_v2.py` is the ROS-free, allowlisted rover capability/tool registry used by the iterative planner. It is a supervisory boundary, not a motor controller and not a generic ROS bridge.

## Contract

The registry exposes typed `ToolDefinition` entries and deterministic adapters. Planner/model output is accepted only as structured `ToolCall` objects and every call is validated before execution:

- tool name must exist in the registry;
- arguments must match the tool input schema;
- required capabilities must be available;
- physical execution requires physical approval;
- replay-only authorization cannot start physical execution;
- remaining tool-call, travel, segment, runtime, and iteration budgets must be sufficient;
- direct ROS, raw motor, shell, filesystem, credential, approval mutation, and ESTOP-clearing surfaces are rejected.

`MissionBudgets` is capped fail-closed: planner-controlled numeric strings, non-finite values, and excessive limits raise `MissionValidationError` instead of expanding authority.

## Default replay/mock tools

`build_default_rover_tool_registry()` registers deterministic offline tools for tests and replay demos:

- `create_room_map`
- `detect_objects`
- `project_semantic_map`
- `approach_clearance`
- `capture_observation`
- `report_artifacts`

The default adapters return artifact references only. They do not launch ROS, open the rover transport, publish `/cmd_vel`, mutate approvals, clear ESTOP, touch shell/filesystem credentials, or start physical motion.

## Result boundary

Adapters return `ToolResult`. The registry validates all result observations against the declared result schema before the result crosses back to the planner, including failed, partial, cancelled, and estopped statuses. Timeout and travel declarations are enforced after adapter execution. Observation and artifact payloads are data only; they cannot smuggle ROS internals such as `/cmd_vel` or command paths into planner authority.

## Relationship to mission_api.v1

`mission_api.v1` and the deterministic VS09 plain-English translator remain available as the offline canonical shoe-mapping fallback/test oracle. `mission_api.v2` generalizes the boundary to named rover tools so the planner can compose heterogeneous goals without becoming shoe-only or ROS-capable.
