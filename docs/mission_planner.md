# Iterative LLM planner over allowlisted rover tools

`src/sphero_rvr_driver/mission_planner.py` implements a provider-neutral, ROS-free supervisory planner loop above `mission_api.v2`.

## Shape

The loop is:

```text
natural-language goal
  -> bounded planner context
  -> PlannerProvider structured response
  -> mission_api.v2 validation
  -> deterministic tool adapter
  -> structured observation/artifact refs
  -> replan until terminal stop reason
```

The provider may request only registered typed tool calls. The planner never publishes ROS topics, never opens a rover transport, never owns high-rate control, never clears ESTOP, never approves its own physical gate, and never widens its own limits.

## Providers

`PlannerProvider` is a small protocol returning `PlannerProviderResponse` with:

- `decision`: `continue`, `complete`, or `reject`;
- `tool_calls`: allowlisted `ToolCall` entries;
- `message`: non-authoritative provider text.

`FakePlannerProvider` is deterministic and drives the replay/mock test suite. `OpenAICompatiblePlannerProvider` is optional and configured only from constructor values or environment variable names (`OPENAI_BASE_URL`, `OPENAI_MODEL`, and an API-key env name such as `OPENAI_API_KEY`). Missing live-provider credentials raise a clear validation error; the fake provider is never reported as live evidence.

## Planner context

The provider receives bounded context only:

- user goal;
- available tool definitions;
- capability and approval state;
- execution mode;
- bounded history/observations;
- remaining budgets;
- safety policy flags.

Tool observations are data, not authority. Provider messages that attempt direct ROS/motor/system access, policy bypass, approval mutation, ESTOP clearing, credential/shell access, or budget expansion are rejected before any tool executes.

## Manifest

Each run returns a stable provider-independent manifest with:

- goal;
- provider/model identifiers;
- registry version;
- exact source SHA;
- proposed, rejected, and executed calls;
- observations, decisions, and artifact references;
- stop reason;
- live-provider validation status.

Default validation is replay/mock/no-hardware. If a live provider is not already configured, record `live provider validation pending` and do not claim a live-model smoke happened. Tiny robot, large consequences.
