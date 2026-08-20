"""English instruction in, robot behaviour out — the Stage D demo client.

A CLI REPL: you type an instruction, a language model proposes one tool call at a
time, this client validates the proposal and executes it against `task_node`'s three
interfaces, feeds the result back, and loops until the model says it is done or the
budget runs out.

**STAGE D ACCEPTANCE, stated where it can be checked: DELETING THIS FILE CHANGES
NOTHING ABOUT THE ROBOT.** It is a client. It lives outside the node, nothing imports
it, no launch file starts it, and every capability it uses is a plain ROS service or
action that a human can call with `ros2 service call`. The robot does not gain
abilities by having a model attached; it gains an interface. That is the whole point
of the split, and it is why the tool surface was built first and separately.

WHAT THIS FILE MAY DO IS DELIBERATELY NARROW. It talks to exactly three interfaces —
`task/goto`, `task/observe`, `task/query_semantic_map` — and it holds no Twist, no
geometry_msgs, no publisher, no topic of its own. A model cannot ask it for anything
outside that set because `sphero_rvr_core.task_agent` refuses to parse such a
request, and even if it could, `task_node`'s envelope and then the collision
supervisor would still be in the way. Three layers, and the model is outside all of
them. `tests/test_task_node_safety.py` scans this file for the same forbidden names
as the node.

The provider is the one this repo already uses (Synthetic, key from
`~/.config/synthetic/api_key`), through the same `vlm_client` helpers as every other
model consumer here — a text model rather than a vision one. No new provider, no new
secret, no SDK. Native tool-calling APIs are the v2 upgrade, gated behind the same
door as a bespoke .srv package.

Usage (task_node and the rest of the stack must already be running):
    ros2 run sphero_rvr_driver task_client
    > what do you know about shoes?
    > drive half a metre forward
"""

import argparse
import json
import os
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from sphero_rvr_core.task_agent import Budget, availability_note, run_instruction


class ToolRunner(Node):
    """Executes validated tool calls against task_node. Owns no policy."""

    def __init__(self, timeout_s=180.0):
        super().__init__("task_client")
        self._timeout_s = timeout_s
        # F3. The rover keeps driving if this process dies mid-goto: Ctrl-C, or an
        # uncaught exception from the model call (a network blip is enough), would
        # exit without cancelling and leave the goal running for up to goal_timeout_s.
        # Held here so the shutdown path can always find and cancel it.
        self._active_goto = None
        self._goto = ActionClient(self, NavigateToPose, "task/goto")
        self._observe = self.create_client(Trigger, "task/observe")
        # THE MISSION TOOLS. Three more clients, no new machinery -- they are Triggers
        # exactly like observe, which is the payoff of task_node keeping them Triggers.
        self._mission = {
            name: self.create_client(Trigger, f"task/{name}")
            for name in ("explore", "stop", "status", "where_am_i")
        }
        self._query = self.create_client(Trigger, "task/query_semantic_map")
        # Bridge round 1: argument-carrying Triggers use the same typed-parameter
        # route as query (scalars via task_node's parameters, then the call).
        self._turn = self.create_client(Trigger, "task/turn")
        self._recognize = self.create_client(Trigger, "task/look_and_recognize")

    # Each runner returns the tool's own JSON result string. Failures are returned,
    # not raised: an envelope refusal is information the model must see and correct,
    # not an exception that ends the instruction.
    def probe_availability(self, wait_s=0.5):
        """tool name -> bool for every probeable interface, feeding the pure
        `availability_note` (search round 2 §5: the preamble kills the
        discovery tax at the root instead of letting each flight pay it)."""
        avail = {}
        for name, client in (("observe", self._observe),
                             ("query_semantic_map", self._query),
                             ("turn", self._turn),
                             ("look_and_recognize", self._recognize)):
            avail[name] = client.wait_for_service(timeout_sec=wait_s)
        for name, client in self._mission.items():
            avail[name] = client.wait_for_service(timeout_sec=wait_s)
        avail["goto"] = self._goto.wait_for_server(timeout_sec=wait_s)
        return avail

    def run(self, tool, args):
        if tool == "goto":
            return self._run_goto(args)
        if tool == "observe":
            return self._call(self._observe, "observe")
        if tool == "query_semantic_map":
            return self._run_query(args)
        if tool in ("explore", "stop", "status", "where_am_i"):
            return self._call(self._mission[tool], tool)
        if tool == "turn":
            return self._run_param_tool(self._turn, "turn",
                                        {"turn_degrees": float(args["degrees"])})
        if tool == "look_and_recognize":
            return self._run_param_tool(self._recognize, "look_and_recognize",
                                        {"recognition_target": str(args["target"])},
                                        timeout_s=110.0)
        return json.dumps({"ok": False, "message": f"no such tool {tool!r}"})

    def _spin_until(self, future, deadline):
        while not future.done():
            if time.monotonic() >= deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result()

    def _call(self, client, label):
        if not client.wait_for_service(timeout_sec=5.0):
            return json.dumps({"ok": False,
                               "message": f"{label} unavailable — is task_node running?"})
        future = client.call_async(Trigger.Request())
        result = self._spin_until(future, time.monotonic() + self._timeout_s)
        if result is None:
            return json.dumps({"ok": False, "message": f"{label} timed out"})
        return result.message

    def _run_query(self, args):
        """Arguments go through task_node's typed parameters (its interface note
        explains why they are scalars). Unset ones are reset each call so a stale
        filter from a previous question cannot silently narrow this one — a real
        effect observed while demoing the CLI."""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

        setter = self.create_client(SetParameters, "/task_node/set_parameters")
        if not setter.wait_for_service(timeout_sec=5.0):
            return json.dumps({"ok": False,
                               "message": "task_node parameters unavailable"})
        wanted = {
            "query_label": args.get("label", ""),
            "query_near_x": float(args.get("near_x", 0.0)),
            "query_near_y": float(args.get("near_y", 0.0)),
            "query_radius_m": float(args.get("radius_m", 0.0)),
            "query_min_confidence": float(args.get("min_confidence", 0.0)),
        }
        params = []
        for name, value in wanted.items():
            p = Parameter()
            p.name = name
            if isinstance(value, str):
                p.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                         string_value=value)
            else:
                p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                         double_value=value)
            params.append(p)
        request = SetParameters.Request(parameters=params)
        future = setter.call_async(request)
        response = self._spin_until(future, time.monotonic() + 10.0)
        if response is None:
            return json.dumps({"ok": False, "message": "setting query parameters timed out"})
        # F5. REFUSE rather than answer with unknown filters. These parameters are
        # the query, and a partially-applied set means a stale filter from an
        # earlier question silently narrows this one -- the exact bug the per-call
        # reset exists to prevent, so failing to notice it here would defeat the
        # whole mechanism.
        failed = [p.name for p, r in zip(params, getattr(response, "results", []))
                  if not getattr(r, "successful", False)]
        if failed or len(getattr(response, "results", [])) != len(params):
            return json.dumps({
                "ok": False,
                "message": ("could not set query parameter(s) "
                            f"{', '.join(failed) or '(no results returned)'} — "
                            "refusing the query rather than answering with a "
                            "possibly stale filter"),
            })
        return self._call(self._query, "query_semantic_map")

    def _run_param_tool(self, client, label, params_wanted, timeout_s=None):
        """The query pattern generalized: typed scalar parameters carry the
        arguments (task_node's interface doctrine), then the Trigger fires. Same
        refuse-on-partial-set rule as query — a half-applied argument is a call
        the model did not make."""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

        setter = self.create_client(SetParameters, "/task_node/set_parameters")
        if not setter.wait_for_service(timeout_sec=5.0):
            return json.dumps({"ok": False,
                               "message": "task_node parameters unavailable"})
        params = []
        for name, value in params_wanted.items():
            p = Parameter()
            p.name = name
            if isinstance(value, str):
                p.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                         string_value=value)
            else:
                p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                         double_value=float(value))
            params.append(p)
        response = self._spin_until(setter.call_async(
            SetParameters.Request(parameters=params)), time.monotonic() + 10.0)
        if response is None:
            return json.dumps({"ok": False,
                               "message": f"setting {label} parameters timed out"})
        failed = [p.name for p, r in zip(params, getattr(response, "results", []))
                  if not getattr(r, "successful", False)]
        if failed or len(getattr(response, "results", [])) != len(params):
            return json.dumps({"ok": False,
                               "message": f"could not set {label} parameter(s) "
                                          f"{failed or 'all'}; not calling"})
        if not client.wait_for_service(timeout_sec=5.0):
            return json.dumps({"ok": False,
                               "message": f"{label} unavailable — is task_node running?"})
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + (timeout_s or self._timeout_s)
        result = self._spin_until(future, deadline)
        if result is None:
            return json.dumps({"ok": False, "message": f"{label} timed out"})
        return result.message

    def _run_goto(self, args):
        if not self._goto.wait_for_server(timeout_sec=5.0):
            return json.dumps({"ok": False,
                               "message": "task/goto unavailable — is task_node running?"})
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(args["x"])
        goal.pose.pose.position.y = float(args["y"])
        goal.pose.pose.orientation.w = 1.0
        send = self._goto.send_goal_async(goal)
        handle = self._spin_until(send, time.monotonic() + 15.0)
        if handle is None:
            return json.dumps({"ok": False, "message": "task/goto did not acknowledge"})
        if not handle.accepted:
            # task_node refuses a second concurrent goto; say so in words the model
            # can act on rather than leaving it to infer.
            return json.dumps({"ok": False,
                               "message": "task/goto refused the goal (already driving?)"})
        self._active_goto = handle
        try:
            result = self._spin_until(handle.get_result_async(),
                                      time.monotonic() + self._timeout_s)
            if result is None:
                # F4. Do not claim "cancelled" on a fire-and-forget request. Spin
                # the cancel future and say honestly whether it was confirmed --
                # an unconfirmed cancel means the rover may still be driving, which
                # is exactly the thing a caller needs to know.
                confirmed = self._cancel_and_confirm(handle)
                return json.dumps({
                    "ok": False,
                    "message": ("goto timed out — cancel confirmed" if confirmed
                                else "goto timed out — CANCEL NOT CONFIRMED, the "
                                     "rover may still be moving"),
                })
            return result.result.error_msg or json.dumps({"ok": True, "message": "done"})
        finally:
            self._active_goto = None

    def _cancel_and_confirm(self, handle, timeout_s=5.0):
        """Cancel and actually wait for the acknowledgement. Returns True only when
        the cancel was accepted or the goal has ended."""
        try:
            future = handle.cancel_goal_async()
        except Exception:
            return False
        response = self._spin_until(future, time.monotonic() + timeout_s)
        if response is None:
            return False
        return len(getattr(response, "goals_canceling", [])) > 0

    def shutdown_safely(self):
        """Cancel anything still driving before this process goes away (F3)."""
        handle = self._active_goto
        if handle is None:
            return
        self.get_logger().warn(
            "task_client exiting with a goto in flight — cancelling it; a client "
            "that just exits leaves the rover driving."
        )
        self._cancel_and_confirm(handle)


def make_model_caller(base_url, api_key, model, max_tokens, timeout_s):
    from sphero_rvr_core.vlm_client import query_text

    def ask(system, prompt):
        return query_text(base_url, api_key, model, prompt, system=system,
                          max_tokens=max_tokens, timeout=timeout_s, json_mode=True)
    return ask


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--model", default="syn:large:text")
    ap.add_argument("--base-url", default="https://api.synthetic.new/v1")
    ap.add_argument("--api-key-file",
                    default=os.path.expanduser("~/.config/synthetic/api_key"))
    ap.add_argument("--max-tool-calls", type=int, default=8)
    # 1500, NOT 500: json_mode models reason before the JSON and a small cap
    # truncates to nothing — the synthetic-vlm trap, learned by the vision
    # path in 2026-08, re-learned HERE on flight 2 (2026-08-20) when call 11's
    # history-heavy prompt came back empty three times and crashed the client
    # mid-search. Third seam to learn it; the next model path checks BEFORE
    # flying. Deterministic reasoning-burn beyond this base is handled by
    # query_text's escalating-cap retry ladder (x3 after the first empty
    # reply) — raising this default wholesale was considered and declined
    # (2026-08-20 consensus); it remains the operator's override for a run
    # that wants a bigger base.
    ap.add_argument("--max-tokens", type=int, default=1500)
    ap.add_argument("--model-timeout-s", type=float, default=60.0)
    ap.add_argument("--tool-timeout-s", type=float, default=180.0)
    ap.add_argument("instruction", nargs="*",
                    help="run one instruction and exit; omit for an interactive REPL")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    try:
        with open(args.api_key_file) as f:
            key = f.read().strip()
    except OSError as exc:
        print(f"cannot read API key ({exc}). This client needs one; the ROBOT does "
              f"not — every tool it uses is callable with ros2 service call.")
        return 2

    rclpy.init()
    runner = ToolRunner(timeout_s=args.tool_timeout_s)
    ask = make_model_caller(args.base_url, key, args.model, args.max_tokens,
                            args.model_timeout_s)
    try:
        # One availability probe per process: interfaces this configuration
        # does not expose get named in the instruction preamble instead of
        # being discovered one failed call at a time. NOTE the honest limit:
        # this catches MISSING interfaces; a task_node whose backend is absent
        # still answers ok=false per call (its services always exist), and the
        # system prompt's do-not-retry-unavailable rule handles that case.
        note = availability_note(runner.probe_availability())
        if note:
            print(note.strip(), flush=True)
        if args.instruction:
            run_instruction(note + " ".join(args.instruction), ask, runner,
                            Budget(args.max_tool_calls))
        else:
            print("task_client — type an instruction, or Ctrl-D to quit.")
            while True:
                try:
                    line = input("> ").strip()
                except EOFError:
                    print()
                    break
                if not line:
                    continue
                # A fresh budget per instruction: the ceiling bounds one request, so
                # a long session is not one long unbounded agent.
                run_instruction(note + line, ask, runner, Budget(args.max_tool_calls))
    except KeyboardInterrupt:
        pass
    finally:
        # Order matters: cancel BEFORE destroying the node, or there is no longer a
        # client with which to cancel. This runs for Ctrl-C and for any uncaught
        # exception, including a model-call network failure mid-drive.
        try:
            runner.shutdown_safely()
        except Exception as exc:
            print(f"WARNING: could not cancel the in-flight goto ({exc}). "
                  "The rover may still be moving — use the STOP service.")
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
