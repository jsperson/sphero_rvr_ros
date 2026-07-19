"""Least-privilege MCP stdio adapter over mission_api.v2.

This module intentionally implements only a small local stdio JSON-RPC surface.
It is not a ROS bridge, not a network service, and not a path to direct motor
control.  Every executable capability is derived from the mission_api.v2
registry and routed through the deterministic runtime.
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional, Sequence

from .mission_api import MissionApiVersion, MissionValidationError
from .mission_api_v2 import (
    ApprovalGrant,
    CapabilityAvailability,
    CapabilityRegistry,
    DeterministicMissionRuntime,
    FakeCapabilityAdapters,
    MissionBudgets,
    MissionGoal,
    MissionPlan,
    MissionRuntimeResult,
    ToolDefinition,
    ToolInvocation,
    ToolResult,
    build_canonical_shoe_mapping_v2_plan,
    build_default_v2_registry,
)

PROTOCOL_VERSION = "2024-11-05"
SERVER_NAME = "sphero-rvr-mcp"
SERVER_VERSION = "0.1.0"
REGISTRY_VERSION = MissionApiVersion.V2.value
FORBIDDEN_MCP_ARGUMENT_KEYS = {
    "approval",
    "approval_grant",
    "physical_approval",
    "clear_estop",
    "filesystem_path",
    "path",
    "ros_topic",
    "ros_service",
    "ros_action",
    "shell",
    "python",
    "credential",
    "secret",
    "token",
}
RESOURCES = (
    "rvr://capabilities/registry",
    "rvr://mission/health",
    "rvr://mission/audit",
    "rvr://mission/artifacts",
)


@dataclass(frozen=True)
class McpCallContext:
    session_id: str
    client_name: str = "mcp-client"


@dataclass
class RvrMcpAdapter:
    registry: CapabilityRegistry = field(default_factory=lambda: build_default_v2_registry(detector_classes=("shoe", "backpack")))
    adapters: FakeCapabilityAdapters = field(default_factory=FakeCapabilityAdapters)
    source_sha: str = field(default_factory=lambda: _source_sha())
    now_s: float = 100.0
    replay_supervised_motion_enabled: bool = True

    def __post_init__(self) -> None:
        self._audit_events: list[dict[str, Any]] = []
        self._artifact_refs: dict[str, str] = {}
        self._last_status = "idle"
        self._shutdown = False

    def list_tools(self) -> list[dict[str, Any]]:
        tools: list[dict[str, Any]] = [
            {
                "name": "rvr.list_capabilities",
                "description": "Return the mission_api.v2 capability registry and availability summary.",
                "inputSchema": _schema({}, required=()),
            },
            {
                "name": "rvr.submit_plan",
                "description": "Validate and execute a bounded mission_api.v2 plan through the deterministic runtime.",
                "inputSchema": _submit_plan_schema(),
            },
            {
                "name": "rvr.validate_plan",
                "description": "Validate a bounded mission_api.v2 plan without returning authority to execute forbidden surfaces.",
                "inputSchema": _submit_plan_schema(),
            },
            {
                "name": "rvr.mission_status",
                "description": "Return read-only MCP/mission health, freshness, and safety-policy state.",
                "inputSchema": _schema({}, required=()),
            },
        ]
        for definition in self.registry.definitions():
            tools.append(
                {
                    "name": _mcp_tool_name(definition),
                    "description": _tool_description(definition),
                    "inputSchema": definition.argument_schema,
                    "annotations": {
                        "api_version": REGISTRY_VERSION,
                        "tool_id": definition.tool_id,
                        "tool_version": definition.version,
                        "availability": definition.availability.value,
                        "safety_class": definition.safety_class,
                        "approval_class": definition.approval_class,
                        "readOnlyHint": definition.approval_class == "none" and definition.safety_class == "read_only",
                        "destructiveHint": False,
                    },
                }
            )
        return tools

    def list_resources(self) -> list[dict[str, Any]]:
        return [
            {
                "uri": "rvr://capabilities/registry",
                "name": "Capability registry",
                "description": "Read-only mission_api.v2 registry/version/availability snapshot.",
                "mimeType": "application/json",
            },
            {
                "uri": "rvr://mission/health",
                "name": "Mission health",
                "description": "Read-only robot/mission health and freshness summary.",
                "mimeType": "application/json",
            },
            {
                "uri": "rvr://mission/audit",
                "name": "Mission audit stream snapshot",
                "description": "Bounded MCP/session-correlated audit events from this local server process.",
                "mimeType": "application/json",
            },
            {
                "uri": "rvr://mission/artifacts",
                "name": "Explicit artifact references",
                "description": "Artifact references returned by validated mission_api.v2 tools; no arbitrary file access.",
                "mimeType": "application/json",
            },
        ]

    def read_resource(self, uri: str) -> dict[str, Any]:
        if uri not in RESOURCES:
            raise MissionValidationError(f"unsupported MCP resource: {uri}")
        if uri == "rvr://capabilities/registry":
            return {
                "api_version": REGISTRY_VERSION,
                "registry_version": REGISTRY_VERSION,
                "source_sha": self.source_sha,
                "registry": self.registry.to_json_dict(),
                "forbidden_surfaces_exposed": False,
            }
        if uri == "rvr://mission/health":
            return self._mission_status_payload(McpCallContext("resource-read"))
        if uri == "rvr://mission/audit":
            return {"api_version": REGISTRY_VERSION, "events": list(self._audit_events[-100:])}
        return {"api_version": REGISTRY_VERSION, "artifact_refs": dict(self._artifact_refs)}

    def call_tool(self, name: str, arguments: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
        args = dict(arguments or {})
        context = McpCallContext(session_id=str(args.pop("client_session_id", "mcp-local-session") or "mcp-local-session"))
        try:
            _reject_forbidden_mcp_keys(args)
            if name == "rvr.list_capabilities":
                return self._record_payload("complete", context, {"registry": self.registry.to_json_dict(), "tools": self.list_tools()})
            if name == "rvr.mission_status":
                return self._record_payload("complete", context, self._mission_status_payload(context))
            if name == "rvr.validate_plan":
                plan = self._plan_from_arguments(args, context)
                runtime = self._runtime()
                runtime._validate_plan_budgets(plan)  # noqa: SLF001 - validation endpoint over the canonical runtime.
                elapsed_s = 0.0
                for invocation in plan.invocations:
                    definition = runtime._validate_invocation(invocation, plan, now_s=self.now_s + elapsed_s)  # noqa: SLF001
                    elapsed_s += _effective_timeout_from_arguments(definition, invocation.arguments)
                return self._record_payload("validated", context, {"plan": plan.to_json_dict()})
            if name == "rvr.submit_plan":
                plan = self._plan_from_arguments(args, context)
                return self._result_payload(self._runtime().execute_plan(plan), context)
            definition = self._definition_for_mcp_tool(name)
            if definition is None:
                raise MissionValidationError(f"unsupported MCP tool: {name}")
            if definition.availability is not CapabilityAvailability.AVAILABLE:
                raise MissionValidationError(f"tool {definition.tool_id} is {definition.availability.value}")
            invocation = ToolInvocation(
                correlation_id=f"mcp-{context.session_id}-{definition.tool_id}",
                tool_id=definition.tool_id,
                tool_version=definition.version,
                arguments=args,
                approval=self._approval_for(definition),
                requested_at_s=self.now_s,
                provenance={"mcp_server": SERVER_NAME, "mcp_session_id": context.session_id},
            )
            goal = MissionGoal(
                goal_id=f"mcp-{context.session_id}",
                objective=f"MCP request for {definition.tool_id}",
                success_criteria=("requested capability validates and returns a structured result",),
                execution_mode="replay",
                budgets=_single_call_budget(definition, args),
            )
            return self._result_payload(self._runtime().execute_plan(MissionPlan(goal=goal, invocations=(invocation,), plan_id=f"mcp-{context.session_id}-plan")), context)
        except MissionValidationError as exc:
            return self._rejected_payload(context, str(exc))
        except Exception as exc:  # pragma: no cover - defensive fail-closed boundary.
            return self._rejected_payload(context, f"unexpected MCP adapter error: {exc.__class__.__name__}")

    def handle_request(self, request: Mapping[str, Any]) -> Optional[dict[str, Any]]:
        request_id = request.get("id")
        try:
            _ensure_jsonable(request)
            if request.get("jsonrpc") != "2.0":
                return _error(request_id, -32600, "invalid JSON-RPC envelope")
            method = request.get("method")
            if "id" not in request:
                if method in {"notifications/initialized", "notifications/cancelled", "notifications/progress"}:
                    return None
                return None
            params = request.get("params", {})
            if params is None:
                params = {}
            if not isinstance(params, Mapping):
                return _error(request_id, -32602, "params must be an object")
            if method == "initialize":
                return _result(
                    request_id,
                    {
                        "protocolVersion": PROTOCOL_VERSION,
                        "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION},
                        "capabilities": {"tools": {}, "resources": {}},
                    },
                )
            if method == "tools/list":
                return _result(request_id, {"tools": self.list_tools()})
            if method == "resources/list":
                return _result(request_id, {"resources": self.list_resources()})
            if method == "resources/read":
                uri = str(params.get("uri", ""))
                payload = self.read_resource(uri)
                return _result(request_id, {"contents": [{"uri": uri, "mimeType": "application/json", "text": json.dumps(payload, sort_keys=True)}]})
            if method == "tools/call":
                name = str(params.get("name", ""))
                arguments = params.get("arguments", {})
                if not isinstance(arguments, Mapping):
                    return _error(request_id, -32602, "tool arguments must be an object")
                if self._definition_for_mcp_tool(name) is None and name not in {"rvr.list_capabilities", "rvr.submit_plan", "rvr.validate_plan", "rvr.mission_status"}:
                    return _error(request_id, -32602, f"unsupported MCP tool: {name}")
                payload = self.call_tool(name, arguments)
                return _result(request_id, {"content": [{"type": "text", "text": json.dumps(payload, sort_keys=True)}], "isError": payload.get("status") == "rejected"})
            if method == "shutdown":
                payload = self.shutdown("json-rpc shutdown")
                return _result(request_id, {"shutdown": True, "status": payload["status"]})
            return _error(request_id, -32601, f"method not found: {method}")
        except MissionValidationError as exc:
            return _error(request_id, -32602, str(exc))
        except (TypeError, ValueError):
            return _error(request_id, -32700, "malformed JSON/schema")

    def client_disconnected(self, reason: str) -> dict[str, Any]:
        context = McpCallContext("client-disconnect")
        payload = self._record_payload("cancelled", context, {"reason": reason, "audit_manifest": self._manifest(context, [], [], [], "client_disconnect")})
        return payload

    def shutdown(self, reason: str) -> dict[str, Any]:
        self._shutdown = True
        context = McpCallContext("server-shutdown")
        return self._record_payload("cancelled", context, {"reason": reason, "audit_manifest": self._manifest(context, [], [], [], "server_shutdown")})

    def _plan_from_arguments(self, args: Mapping[str, Any], context: McpCallContext) -> MissionPlan:
        if args.get("execution_mode") == "physical":
            raise MissionValidationError("MCP cannot start or authorize physical execution")
        if args.get("fixture") == "canonical_shoe_mapping":
            return build_canonical_shoe_mapping_v2_plan(goal_id=str(args.get("goal_id", "mcp-shoe-mapping")), approval=self._supervised_replay_approval(context))
        invocations_arg = args.get("invocations")
        if not isinstance(invocations_arg, Sequence) or isinstance(invocations_arg, (str, bytes)):
            raise MissionValidationError("submit_plan requires invocations array or an allowed fixture")
        budgets_arg = args.get("budgets", {})
        if not isinstance(budgets_arg, Mapping):
            raise MissionValidationError("budgets must be an object")
        goal = MissionGoal(
            goal_id=str(args.get("goal_id", f"mcp-{context.session_id}")),
            objective=str(args.get("objective", "MCP-submitted bounded mission_api.v2 plan")),
            success_criteria=tuple(str(item) for item in args.get("success_criteria", ("bounded MCP plan validates",))),
            constraints=args.get("constraints", {}) if isinstance(args.get("constraints", {}), Mapping) else {},
            execution_mode="replay",
            budgets=MissionBudgets(
                max_steps=int(budgets_arg.get("max_steps", max(1, len(invocations_arg)))),
                max_runtime_s=float(budgets_arg.get("max_runtime_s", 120.0)),
                max_travel_m=budgets_arg.get("max_travel_m"),
            ),
            requested_artifacts=tuple(str(item) for item in args.get("requested_artifacts", ())),
        )
        invocations: list[ToolInvocation] = []
        for index, item in enumerate(invocations_arg):
            if not isinstance(item, Mapping):
                raise MissionValidationError("each invocation must be an object")
            call_args = item.get("arguments", {})
            if not isinstance(call_args, Mapping):
                raise MissionValidationError("tool invocation arguments must be an object")
            _reject_forbidden_mcp_keys(call_args)
            definition = self.registry.require(str(item.get("tool_id", "")), str(item.get("tool_version", "1.0")))
            invocations.append(
                ToolInvocation(
                    correlation_id=str(item.get("correlation_id", f"mcp-{context.session_id}-{index}")),
                    tool_id=definition.tool_id,
                    tool_version=definition.version,
                    arguments=dict(call_args),
                    approval=self._approval_for(definition, context),
                    requested_at_s=self.now_s + float(index),
                    provenance={"mcp_server": SERVER_NAME, "mcp_session_id": context.session_id},
                )
            )
        return MissionPlan(goal=goal, invocations=tuple(invocations), plan_id=str(args.get("plan_id", f"mcp-{context.session_id}-plan")))

    def _definition_for_mcp_tool(self, name: str) -> Optional[ToolDefinition]:
        prefix = "rvr.capability."
        if not name.startswith(prefix):
            return None
        tool_id = name[len(prefix) :]
        for definition in self.registry.definitions():
            if definition.tool_id == tool_id:
                return definition
        return None

    def _runtime(self) -> DeterministicMissionRuntime:
        return DeterministicMissionRuntime(self.registry, self.adapters, now_s=self.now_s)

    def _approval_for(self, definition: ToolDefinition, context: Optional[McpCallContext] = None) -> Optional[ApprovalGrant]:
        if not definition.requires_approval():
            return None
        if not self.replay_supervised_motion_enabled or definition.approval_class != "supervised_motion":
            return None
        return self._supervised_replay_approval(context or McpCallContext("mcp-local-session"))

    def _supervised_replay_approval(self, context: McpCallContext) -> ApprovalGrant:
        return ApprovalGrant(
            approval_id=f"mcp-replay-{context.session_id}",
            approved_by="mcp-local-replay-supervisor",
            approved_at_s=self.now_s - 1.0,
            expires_at_s=self.now_s + 3600.0,
            approval_class="supervised_motion",
        )

    def _result_payload(self, result: MissionRuntimeResult, context: McpCallContext) -> dict[str, Any]:
        executed = [item for item in result.audit if item.get("status") == "complete"]
        rejected = [item for item in result.audit if item.get("status") != "complete"]
        artifacts: dict[str, str] = {}
        for tool_result in result.results:
            artifacts.update(tool_result.artifact_refs)
        self._artifact_refs.update(artifacts)
        result_payloads = [tool_result.to_json_dict() for tool_result in result.results]
        payload = {
            "api_version": REGISTRY_VERSION,
            "status": result.status.value,
            "plan": result.plan.to_json_dict(),
            "results": result_payloads,
            "audit": list(result.audit),
            "audit_manifest": self._manifest(
                context,
                result.audit,
                rejected,
                executed,
                result.status.value,
                artifacts=artifacts,
                results=result_payloads,
            ),
        }
        if result.results and result.results[-1].error:
            payload["error"] = dict(result.results[-1].error)
        self._last_status = result.status.value
        self._audit_events.append(payload["audit_manifest"])
        return payload

    def _rejected_payload(self, context: McpCallContext, message: str) -> dict[str, Any]:
        manifest = self._manifest(context, [], [{"reason": message}], [], "rejected")
        payload = {
            "api_version": REGISTRY_VERSION,
            "status": "rejected",
            "error": {"message": _redact(message)},
            "audit_manifest": manifest,
        }
        self._last_status = "rejected"
        self._audit_events.append(manifest)
        return payload

    def _record_payload(self, status: str, context: McpCallContext, payload: Mapping[str, Any]) -> dict[str, Any]:
        result = {"api_version": REGISTRY_VERSION, "status": status, **dict(payload)}
        manifest = result.get("audit_manifest") or self._manifest(context, [], [], [], status)
        result.setdefault("audit_manifest", manifest)
        self._last_status = status
        self._audit_events.append(dict(manifest))
        return result

    def _manifest(
        self,
        context: McpCallContext,
        proposed_calls: Sequence[Mapping[str, Any]],
        rejected_calls: Sequence[Mapping[str, Any]],
        executed_calls: Sequence[Mapping[str, Any]],
        stop_reason: str,
        *,
        artifacts: Optional[Mapping[str, str]] = None,
        results: Sequence[Mapping[str, Any]] = (),
    ) -> dict[str, Any]:
        return {
            "api_version": REGISTRY_VERSION,
            "mcp_server": SERVER_NAME,
            "mcp_session_id": context.session_id,
            "mcp_client": context.client_name,
            "registry_version": REGISTRY_VERSION,
            "source_sha": self.source_sha,
            "proposed_calls": [dict(item) for item in proposed_calls],
            "rejected_calls": [dict(item) for item in rejected_calls],
            "executed_calls": [dict(item) for item in executed_calls],
            "results": [dict(item) for item in results],
            "artifacts": dict(artifacts or {}),
            "stop_reason": stop_reason,
            "credential_material_present": False,
        }

    def _mission_status_payload(self, context: McpCallContext) -> dict[str, Any]:
        del context
        return {
            "api_version": REGISTRY_VERSION,
            "server": {"name": SERVER_NAME, "version": SERVER_VERSION, "transport": "stdio", "local_only": True},
            "mission": {"status": self._last_status, "fresh": True, "age_s": 0.0},
            "robot_health": {"summary": "mock/replay no-hardware MCP adapter", "direct_motor_route_exposed": False},
            "policy": {
                "mcp_can_clear_estop": False,
                "mcp_can_mutate_physical_approval": False,
                "mcp_can_start_physical": False,
                "mcp_can_expand_limits": False,
                "network_listener_enabled": False,
                "generic_ros_access_enabled": False,
                "shell_execution_enabled": False,
            },
            "artifact_refs": dict(self._artifact_refs),
        }


def run_stdio_server(adapter: Optional[RvrMcpAdapter] = None) -> None:
    server = adapter or RvrMcpAdapter()
    try:
        for line in sys.stdin:
            if not line.strip():
                continue
            try:
                request = json.loads(line)
                if not isinstance(request, Mapping):
                    response = _error(None, -32600, "request must be a JSON object")
                else:
                    response = server.handle_request(request)
            except json.JSONDecodeError:
                response = _error(None, -32700, "parse error")
            if response is not None:
                print(json.dumps(response, sort_keys=True), flush=True)
            if server._shutdown:  # noqa: SLF001 - local loop termination flag.
                break
    finally:
        if not server._shutdown:  # noqa: SLF001
            server.client_disconnected("stdio closed")


def main() -> None:
    run_stdio_server()


def _schema(properties: Mapping[str, Any], *, required: Optional[Sequence[str]] = None) -> dict[str, Any]:
    return {"type": "object", "properties": dict(properties), "required": list(properties if required is None else required), "additionalProperties": False}


def _submit_plan_schema() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "fixture": {"type": "string", "enum": ["canonical_shoe_mapping"]},
            "goal_id": {"type": "string"},
            "objective": {"type": "string"},
            "success_criteria": {"type": "array", "items": {"type": "string"}},
            "budgets": {"type": "object"},
            "invocations": {"type": "array", "items": {"type": "object"}},
            "client_session_id": {"type": "string"},
        },
        "required": [],
        "additionalProperties": True,
    }


def _mcp_tool_name(definition: ToolDefinition) -> str:
    return f"rvr.capability.{definition.tool_id}"


def _tool_description(definition: ToolDefinition) -> str:
    effects = "; ".join(definition.effects)
    return f"mission_api.v2 {definition.tool_id}@{definition.version}; availability={definition.availability.value}; safety_class={definition.safety_class}; approval_class={definition.approval_class}; effects={effects}"


def _single_call_budget(definition: ToolDefinition, args: Mapping[str, Any]) -> MissionBudgets:
    travel = args.get("max_travel_m") if definition.tool_id in {"move_to_clearance", "bounded_exploration_segment"} else None
    return MissionBudgets(max_steps=1, max_runtime_s=max(float(definition.timeout_s), 5.0), max_travel_m=travel)


def _effective_timeout_from_arguments(definition: ToolDefinition, args: Mapping[str, Any]) -> float:
    candidates = [float(definition.timeout_s)]
    for key in ("timeout_s", "segment_timeout_s"):
        value = args.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0:
            candidates.append(float(value))
    return min(candidates)


def _reject_forbidden_mcp_keys(args: Mapping[str, Any]) -> None:
    for key, value in args.items():
        lowered = str(key).lower()
        if lowered in FORBIDDEN_MCP_ARGUMENT_KEYS:
            raise MissionValidationError(f"MCP clients cannot provide forbidden authority or surface: {key}")
        if isinstance(value, Mapping):
            _reject_forbidden_mcp_keys(value)


def _source_sha() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL, timeout=2).strip()
    except Exception:
        return "unknown"


def _redact(message: str) -> str:
    redacted = str(message)
    for token in ("Bearer ", "token=", "secret=", "password=", "api_key="):
        if token in redacted:
            redacted = redacted.split(token)[0] + token + "<redacted>"
    return redacted


def _ensure_jsonable(value: Any) -> None:
    json.dumps(value)


def _result(request_id: Any, result: Mapping[str, Any]) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "result": dict(result)}


def _error(request_id: Any, code: int, message: str) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "error": {"code": code, "message": message}}


if __name__ == "__main__":
    main()
