# RVR MCP server

The RVR MCP server is a local-only stdio adapter over `mission_api.v2`. It exists so MCP clients can inspect the typed rover capability registry and request validated deterministic rover capabilities without becoming a ROS bridge.

```text
MCP client -> rvr_mcp_server stdio -> mission_api.v2 registry/runtime -> deterministic adapters -> robot-side STOP/ESTOP/collision/driver boundary
```

## Threat and authority boundary

The MCP layer treats every client argument, tool result, resource payload, and prompt-like string as untrusted data. It does not grant authority. It forwards only registry-defined `mission_api.v2` invocations to `DeterministicMissionRuntime`, which revalidates tool id/version, schema, availability, budgets, approval class, execution mode, STOP/ESTOP/cancellation status, freshness, and result boundaries immediately before or after deterministic execution.

The server deliberately does not expose:

- generic ROS discovery, publish, service, or action calls;
- raw `/cmd_vel`, `/cmd_vel_motor`, serial transport, raw motor, or device handles;
- shell, Python execution, credentials, unrestricted filesystem paths, or `file://` resources;
- LLM-authorized `clear_estop`, physical approval mutation, replay-to-physical escalation, or limit widening;
- a network listener, LAN/Tailscale bind, HTTP/SSE production endpoint, or live-hardware default.

STOP/cancel/ESTOP through MCP is supplementary control semantics only. Independent robot-side STOP/ESTOP/collision and physical power authority remain authoritative.

## Transport and protocol

First release transport is stdio JSON-RPC only. That avoids opening a socket and keeps the adapter suitable for local no-hardware CI.

- MCP protocol version documented/tested: `2024-11-05`.
- No external MCP SDK dependency is required for the server implementation; it implements the small stdio JSON-RPC subset used here (`initialize`, `tools/list`, `tools/call`, `resources/list`, `resources/read`, `shutdown`).
- Remote Streamable HTTP/SSE is out of scope. If added later it must be disabled by default, loopback-only, and reviewed separately.

Launch locally from a checkout or installed package:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.rvr_mcp_server
# or, after installation:
rvr_mcp_server
```

Example MCP client configuration shape, with placeholders only:

```json
{
  "mcpServers": {
    "sphero-rvr": {
      "command": "python3",
      "args": ["-m", "sphero_rvr_driver.rvr_mcp_server"],
      "env": {
        "PYTHONPATH": "/path/to/sphero_rvr_ros/src"
      }
    }
  }
}
```

## Tool mapping

The MCP tool list is derived from `CapabilityRegistry.definitions()`; argument schemas are the same objects carried by `ToolDefinition.argument_schema`. Adding or changing a capability in `mission_api.v2` changes discovery without duplicating policy in MCP-specific code.

Generic adapter tools:

- `rvr.list_capabilities` returns the registry and availability summary.
- `rvr.validate_plan` validates a bounded `mission_api.v2` plan without executing it.
- `rvr.submit_plan` executes a bounded plan or the canonical shoe-mapping fixture through the deterministic runtime.
- `rvr.mission_status` returns read-only server/mission/safety policy state.

Registry-derived tools are named:

```text
rvr.capability.<tool_id>
```

Examples include `rvr.capability.capture_observation`, `rvr.capability.detect_objects`, `rvr.capability.move_to_clearance`, and `rvr.capability.pause_cancel_stop_estop`. Unsupported or unavailable registry entries are discoverable but fail closed when invoked.

MCP clients cannot pass approval grants. For replay/mock no-hardware tests, the adapter may carry an internal supervised-motion replay grant. That grant is not physical authority and cannot be converted into physical execution.

## Resources

Read-only resources are explicit snapshots, not filesystem access:

- `rvr://capabilities/registry` — registry/version/availability/source SHA;
- `rvr://mission/health` — server/mission health and policy state;
- `rvr://mission/audit` — bounded MCP session-correlated audit manifest snapshots;
- `rvr://mission/artifacts` — explicit artifact references returned by validated tools.

Unknown resources such as `file://...`, `rvr://ros/topics`, or arbitrary artifact paths are rejected.

## Audit manifest

Each call records an audit manifest containing MCP session correlation, registry/API version, source SHA, proposed/rejected/executed calls, results, artifact references, and stop reason. The manifest records only structured call/result data and redacted error text; credentials and raw secret material are not included.

## Extension procedure

To expose a new rover capability through MCP:

1. Add the deterministic adapter and `ToolDefinition` once in `mission_api.v2` with bounded argument/result schemas, availability, timeout, cancellation, safety class, approval class, resource ownership, and effects.
2. Add ROS-free `mission_api.v2` runtime tests for valid execution and unsafe rejection.
3. Add or update MCP tests only for protocol behavior: discovery reflects the registry, stdio calls return structured MCP results, resources remain read-only, and adversarial inputs fail closed.
4. Do not add MCP-only safety policy copies, ROS topic names, raw motor aliases, filesystem paths, credentials, or physical approval mutation.

## Limitations

This is not remote production robotics infrastructure. It is local-only, stdio-only, no-hardware by default, and bounded to the current deterministic fake/replay adapters unless a later reviewed card adds physical-control integration. Tiny robots are charming; unaudited remote control surfaces are how charm gets expensive.
