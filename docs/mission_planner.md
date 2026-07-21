# Iterative LLM planner over allowlisted rover tools

`src/sphero_rvr_driver/mission_planner.py` implements a provider-neutral, ROS-free supervisory planner loop above `mission_api.v2`. The rover planner default is now first-party OpenAI `gpt-5.6` through the Responses API, not GLM-5.2/OpenRouter; OpenRouter/GLM remains optional text-only compatibility only.

## Shape

The loop is:

```text
natural-language goal
  -> bounded planner context plus explicitly authorized image observations when present
  -> OpenAI supervisory planner / PlannerProvider structured response
  -> mission_api.v2 validation
  -> deterministic tool adapter
  -> structured observation/artifact refs
  -> replan until terminal stop reason
```

The provider may request only registered typed tool calls. The planner never publishes ROS topics, never opens a rover transport, never owns high-rate control, never clears ESTOP, never approves its own physical gate, and never widens its own limits. The preserved authority chain is: human goal -> OpenAI supervisory planner -> Mission API v2 allowlist -> deterministic bounded capabilities -> independent collision/STOP/ESTOP -> deterministic rover driver.

## Providers

`PlannerProvider` is a small protocol returning `PlannerProviderResponse` with:

- `decision`: `continue`, `complete`, or `reject`;
- `tool_calls`: allowlisted `ToolCall` entries;
- `message`: non-authoritative provider text.

Default rover planner config:

```text
provider: openai
model_id: gpt-5.6
api_surface: responses
credential environment: OPENAI_API_KEY
```

Capability evidence checked against current OpenAI developer docs during this migration:

- Models docs list GPT-5.6 / alias `gpt-5.6` and state latest OpenAI models support text and image input, text output, multilingual capabilities, and vision via the Responses API.
- Images and vision docs show the Responses API accepting `input_image` content by image URL, base64 data URL, or file ID for analysis.
- Function calling docs show Responses function tools defined by JSON Schema and `function_call` outputs.

`FakePlannerProvider` is deterministic and drives the replay/mock test suite. `OpenAICompatiblePlannerProvider` keeps its historical import name, but it is now explicitly a first-party OpenAI Responses adapter: it posts to `https://api.openai.com/v1/responses`, builds typed `mission_api_v2` function tools plus a structured `planner_terminal_decision` function tool, disables parallel tool calls, parses Responses `function_call` / refusal shapes, retries configured transient failures, and rejects OpenRouter or other endpoint mismatches before network I/O. It is configured only from constructor values or environment variable names (`OPENAI_BASE_URL`, `OPENAI_MODEL`, and an API-key env name such as `OPENAI_API_KEY`). Missing live-provider credentials raise a clear validation error; the fake provider is never reported as live evidence. GLM-5.2 through OpenRouter remains optional text-only compatibility and fails closed for image observations.

This rover planner config does not configure Hermes Kanban `coder`, `planner`, or `reviewer` agents, their models, or their credentials.

## Image observation boundary

Images stay behind typed observations. A caller must explicitly capture or replay an observation, mark it approved for planner use, provide bounded metadata, and pass only that approved image observation into `IterativeMissionPlanner.run(..., image_observations=(...))` or directly to the OpenAI Responses payload builder. The planner never receives raw camera device access, ROS graph access, filesystem access, shell access, UART access, or continuous video control.

Image observations fail closed before a provider call when they are not explicitly approved, missing an image URL/reference, unsupported MIME type, oversized, a raw camera/ROS graph reference such as `/dev/video0` or `/camera_node/image_raw`, or paired with prompt-injection/safety-bypass text.

Safe manifests preserve provider/model identity and bounded observation metadata, but never include image bytes, base64 data URLs, bearer tokens, Authorization headers, or credential values.

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
- API surface (`responses`, `scripted`, or another explicit provider-owned surface);
- registry version;
- exact source SHA;
- proposed, rejected, and executed calls;
- observations, decisions, and artifact references;
- stop reason;
- live-provider validation status.

Default validation is replay/mock/no-hardware. `scripts/rvr_openai_planner_smoke.py` is the no-hardware first-party live smoke: with `OPENAI_API_KEY` it runs the real OpenAI Responses provider against fake Mission API v2 adapters; without that credential it exits with a JSON `blocked` result and explicitly refuses to substitute OpenRouter or fake live-provider evidence. Tiny robot, large consequences.
