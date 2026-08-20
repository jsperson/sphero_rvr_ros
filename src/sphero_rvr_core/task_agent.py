"""Pure agent logic for the LLM task client: contract, validation, budget.

Everything here is decidable without ROS and without a model — it is the part of the
loop that decides whether a proposed tool call is allowed to touch the robot at all.
Keeping it pure is not tidiness: this is the layer that has to be trustworthy when
the thing on the other end is a language model that will, eventually, propose
something absurd. It is unit-testable against canned transcripts, so "what does the
client do when the model hallucinates a tool" is a test rather than a hope.

THE CONTRACT IS FAIL-CLOSED. Every reply must parse to exactly one of:
    {"tool": "<name>", "args": {...}}   -> execute that tool
    {"say": "<text>"}                   -> speak to the user; the instruction is done
Anything else is refused: an unknown tool, a missing args object, an argument the
schema does not name, a wrong type. The client never "figures out what it meant".
A model that cannot follow the contract gets exactly one reprompt carrying the
error, then the instruction ends — because a loop that reprompts forever is a bill
and a liveness bug, and on this project it would be a robot repeatedly acting on
garbage.

The tool schemas below are the ONLY tools that exist. They mirror task_node's three
services deliberately; if the two ever disagree, the node is authoritative and its
envelope rejects the call — which the client then feeds back verbatim so the model
can correct itself. That double check is on purpose: this file is a convenience for
the model, not a safety boundary. The safety boundary is the node, and beneath it
the collision supervisor.
"""

from dataclasses import dataclass, field
import json
from typing import Any, Optional


# name -> {arg: (type, required)}. Types are checked before anything reaches ROS.
TOOL_SCHEMAS: dict[str, dict[str, tuple[type, bool]]] = {
    "goto": {
        "x": ((int, float), True),
        "y": ((int, float), True),
    },
    "observe": {},
    "explore": {},
    "stop": {},
    "status": {},
    "query_semantic_map": {
        "label": ((str,), False),
        "near_x": ((int, float), False),
        "near_y": ((int, float), False),
        "radius_m": ((int, float), False),
        "min_confidence": ((int, float), False),
    },
    # The bridge round 1 (design_llm_verb_bridge_2026-08-20, consensus pins):
    "turn": {
        "degrees": ((int, float), True),
    },
    "where_am_i": {},
    "look_and_recognize": {
        "target": ((str,), True),
    },
}

#: CONTRACT-LAYER RANGE BOUNDS (consensus pin b): sanity lives at the schema,
#: safety lives at the admission. |degrees| <= 180 because a single in-place
#: turn never needs more (the long way round is a model mistake, not a plan),
#: and the bound is checked HERE so a nonsense number is refused before any ROS
#: call — while the supervisor's precise-turn admission remains the layer that
#: refuses a sane number at an unsafe moment. Both layers are named in
#: tests/test_task_agent.py so neither can quietly absorb the other's job.
ARG_BOUNDS: dict[str, dict[str, tuple[float, float]]] = {
    "turn": {"degrees": (-180.0, 180.0)},
}

#: String arguments that must be non-empty after strip: an empty target is a
#: question about nothing, refused at the contract like every unbounded thing.
NONEMPTY_STRINGS: dict[str, tuple[str, ...]] = {
    "look_and_recognize": ("target",),
}

SYSTEM_PROMPT = """You control a small ground robot through exactly nine tools.

Reply with ONE JSON object and nothing else. Either call a tool:
  {"tool": "goto", "args": {"x": 1.0, "y": 0.5}}
  {"tool": "observe", "args": {}}
  {"tool": "query_semantic_map", "args": {"label": "shoe"}}
  {"tool": "explore", "args": {}}
  {"tool": "stop", "args": {}}
  {"tool": "status", "args": {}}
or finish by speaking to the user:
  {"say": "There is a pink shoe about a metre ahead."}

Tools:
- goto(x, y): drive to a point in map coordinates, in metres. The robot starts near
  (0,0). +x is the direction the robot faced at startup; +y is to its left. Goals
  are limited to a few metres from the robot's current position.
- observe(): look with the camera and fold what is seen into the semantic map. Costs
  a real cloud call; use it when you need fresh information, not habitually.
- query_semantic_map(label, near_x, near_y, radius_m, min_confidence): ask what the
  robot already knows about. All arguments optional; no label means everything.
  Supply near_x, near_y and radius_m together or not at all.
- explore(): start a mission to explore and cover the whole room. IT RETURNS AS SOON
  AS THE MISSION STARTS, not when it finishes -- the drive takes minutes. Never say
  the room has been explored just because this returned ok; call status() to find out.
- stop(): end the mission and cancel the goal being driven. THIS IS NOT AN EMERGENCY
  STOP -- the robot coasts to a halt. If the user sounds like they want an emergency
  stop, do it and then tell them plainly that it was not one.
- status(): what the robot is doing right now. If the result says the status is STALE
  or unavailable, report THAT -- it means the robot has stopped reporting, which is
  worth more to the user than any older value.
- turn(degrees): turn in place using the robot's own heading controller. Positive is
  left (counter-clockwise), negative is right; degrees must be between -180 and 180
  (take the short way). The robot may REFUSE a turn near obstacles or while a stop
  is held -- read the message and either move first or tell the user.
- where_am_i(): the robot's position, heading, and a summary of the map it has
  built. Free and instant; call it whenever position matters.
- look_and_recognize(target): point the camera, take a photo, and ask a vision
  model whether the named thing is visible. The robot must be STATIONARY; it costs
  a real cloud call; the answer includes where in the frame and a bearing. If the
  result says the tool is disabled or refused, report that honestly -- never guess
  at what the camera would have seen.

Rules:
- One tool per reply. You will be given the result and may then call another.
- If a tool result says ok=false, read the message and correct your next call.
- When you have the answer, or the task is done, reply with {"say": ...}.
- Never invent a tool. Never add arguments that are not listed."""


class ContractError(ValueError):
    """A model reply the contract refuses. The message is shown to the model on the
    single reprompt, so it must read as an instruction, not an internal error."""


@dataclass
class Decision:
    """What the client should do next: run a tool, or stop and say something."""
    tool: Optional[str] = None
    args: dict = field(default_factory=dict)
    say: Optional[str] = None

    @property
    def is_final(self) -> bool:
        return self.say is not None


def parse_reply(text: str) -> Decision:
    """Turn a model reply into a Decision, or raise ContractError.

    Uses the same balanced-brace scan as every other model consumer here
    (`extract_json`), because models wrap JSON in prose and code fences, and a
    non-greedy regex silently returns an INNER object on nested input — the bug that
    made the semantic map report zero objects against a reply listing three.
    """
    from sphero_rvr_core.vlm_client import extract_json

    try:
        obj = extract_json(text)
    except ValueError:
        raise ContractError(
            "Your reply contained no JSON object. Reply with exactly one JSON "
            'object, e.g. {"say": "..."} or {"tool": "observe", "args": {}}.'
        )
    if not isinstance(obj, dict):
        raise ContractError("Your reply must be a JSON object.")

    if "say" in obj and "tool" not in obj:
        say = obj["say"]
        if not isinstance(say, str) or not say.strip():
            raise ContractError('"say" must be a non-empty string.')
        return Decision(say=say.strip())

    if "tool" not in obj:
        raise ContractError(
            'Your JSON had neither "tool" nor "say". Use {"tool": ..., "args": ...} '
            'to act or {"say": ...} to finish.'
        )
    if "say" in obj:
        raise ContractError(
            'Use either "tool" or "say", not both — one action per reply.'
        )

    name = obj["tool"]
    if not isinstance(name, str) or name not in TOOL_SCHEMAS:
        raise ContractError(
            f"There is no tool called {name!r}. The only tools are: "
            f"{', '.join(sorted(TOOL_SCHEMAS))}."
        )
    args = obj.get("args", {})
    if args is None:
        args = {}
    if not isinstance(args, dict):
        raise ContractError('"args" must be a JSON object.')
    return Decision(tool=name, args=validate_args(name, args))


def validate_args(tool: str, args: dict) -> dict:
    """Type- and name-check arguments against the tool's schema. Fails CLOSED on
    anything unrecognised: an argument we do not know is an argument we cannot
    bound, and silently dropping it would let a model believe it constrained
    something it did not."""
    schema = TOOL_SCHEMAS[tool]
    unknown = sorted(set(args) - set(schema))
    if unknown:
        raise ContractError(
            f"{tool} has no argument(s) {', '.join(unknown)}. Allowed: "
            f"{', '.join(sorted(schema)) or 'none'}."
        )
    out = {}
    for name, (types, required) in schema.items():
        if name not in args:
            if required:
                raise ContractError(f"{tool} requires the argument {name!r}.")
            continue
        value = args[name]
        # bool is an int subclass in Python; a boolean coordinate is a mistake, not
        # a number, and accepting it would put True into a pose.
        if isinstance(value, bool) or not isinstance(value, types):
            raise ContractError(
                f"{tool} argument {name!r} must be "
                f"{'a number' if types != (str,) else 'a string'}, got {value!r}."
            )
        out[name] = value
    for name, (lo, hi) in ARG_BOUNDS.get(tool, {}).items():
        if name in out and not (lo <= float(out[name]) <= hi):
            raise ContractError(
                f"{tool} argument {name!r} must be between {lo:g} and {hi:g} "
                f"(got {out[name]!r}). This is the contract's sanity bound; the "
                f"robot's own admission may still refuse a sane value at an "
                f"unsafe moment."
            )
    for name in NONEMPTY_STRINGS.get(tool, ()):
        if name in out and not out[name].strip():
            raise ContractError(f"{tool} argument {name!r} must not be empty.")
    if tool == "query_semantic_map":
        near = [k for k in ("near_x", "near_y", "radius_m") if k in out]
        if near and len(near) != 3:
            raise ContractError(
                "query_semantic_map needs near_x, near_y and radius_m together, "
                f"or none of them; you gave only {', '.join(near)}."
            )
    return out


@dataclass
class Budget:
    """A hard ceiling on what one English instruction may cost.

    Tool calls cost time, cloud money (observe) and, for goto, physical motion. An
    agent loop without a ceiling is the software equivalent of the explorer's
    93-goal thrash: it does not stop being wrong, it just keeps going. Exhaustion is
    a normal outcome and is reported, never silently retried.
    """
    max_tool_calls: int = 8
    used: int = 0

    def spend(self) -> None:
        if self.exhausted:
            raise ContractError("tool budget exhausted")
        self.used += 1

    @property
    def exhausted(self) -> bool:
        return self.used >= self.max_tool_calls

    @property
    def remaining(self) -> int:
        return max(0, self.max_tool_calls - self.used)


def build_user_turn(instruction: str, history: list) -> str:
    """The prompt for one turn: the instruction plus what the tools have answered.

    History is replayed as plain text rather than as provider-specific tool-call
    message types, so this works against any chat endpoint and stays honest about
    what the model has actually been told. Native tool-calling is the v2 upgrade.
    """
    lines = [f"Instruction: {instruction}"]
    if history:
        lines.append("")
        lines.append("Results of what you have done so far:")
        for call, result in history:
            lines.append(f"- called {call} -> {result}")
    lines.append("")
    lines.append("Reply with one JSON object.")
    return "\n".join(lines)


def describe_call(tool: str, args: dict) -> str:
    """Short, stable rendering of a call for the history and the transcript."""
    if not args:
        return f"{tool}()"
    inner = ", ".join(f"{k}={args[k]!r}" for k in sorted(args))
    return f"{tool}({inner})"


def run_instruction(instruction, ask_model, runner, budget, out=print):
    """Drive one English instruction to completion.

    Lives in the pure core rather than beside the ROS client because it IS the
    policy -- how many times a misbehaving model may retry, when an instruction
    stops, what the model is shown -- and this repo's rule is that anything
    testable without ROS belongs where it can be tested. `ask_model` and `runner`
    are injected, so the whole loop runs against canned transcripts with no network
    and no robot.

    THE REPROMPT RULE: a reply that breaks the contract earns exactly ONE correction
    carrying the error text, and a second consecutive failure ends the instruction.
    Models recover from a stated error surprisingly often; a model that cannot is
    not going to on the fifth attempt either, and every attempt is a paid call. The
    counter resets after any good reply, so one early stumble does not make a later
    unrelated one fatal.
    """
    history = []
    reprompted = False
    while True:
        if budget.exhausted:
            out(f"[budget] stopping after {budget.used} tool calls")
            return
        prompt = build_user_turn(instruction, history)
        reply = ask_model(SYSTEM_PROMPT, prompt)
        try:
            decision = parse_reply(reply)
        except ContractError as exc:
            if reprompted:
                out(f"[refused] {exc}")
                out("[refused] the model did not follow the tool contract; stopping")
                return
            reprompted = True
            out(f"[reprompt] {exc}")
            instruction = f"{instruction}\n\nYour previous reply was rejected: {exc}"
            continue
        reprompted = False
        if decision.is_final:
            out(f"robot> {decision.say}")
            return
        call = describe_call(decision.tool, decision.args)
        budget.spend()
        out(f"[tool {budget.used}/{budget.max_tool_calls}] {call}")
        # A failed tool result is RETURNED, not raised: an envelope refusal is
        # information the model must see and correct from, which is why
        # task_node's results carry readable reasons.
        result = runner.run(decision.tool, decision.args)
        out(f"[result] {result}")
        history.append((call, result))
