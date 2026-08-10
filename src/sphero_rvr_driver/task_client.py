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

from sphero_rvr_core.task_agent import Budget, run_instruction


class ToolRunner(Node):
    """Executes validated tool calls against task_node. Owns no policy."""

    def __init__(self, timeout_s=180.0):
        super().__init__("task_client")
        self._timeout_s = timeout_s
        self._goto = ActionClient(self, NavigateToPose, "task/goto")
        self._observe = self.create_client(Trigger, "task/observe")
        self._query = self.create_client(Trigger, "task/query_semantic_map")

    # Each runner returns the tool's own JSON result string. Failures are returned,
    # not raised: an envelope refusal is information the model must see and correct,
    # not an exception that ends the instruction.
    def run(self, tool, args):
        if tool == "goto":
            return self._run_goto(args)
        if tool == "observe":
            return self._call(self._observe, "observe")
        if tool == "query_semantic_map":
            return self._run_query(args)
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
        if self._spin_until(future, time.monotonic() + 10.0) is None:
            return json.dumps({"ok": False, "message": "setting query parameters timed out"})
        return self._call(self._query, "query_semantic_map")

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
        result = self._spin_until(handle.get_result_async(),
                                  time.monotonic() + self._timeout_s)
        if result is None:
            handle.cancel_goal_async()
            return json.dumps({"ok": False, "message": "goto timed out — cancelled"})
        return result.result.error_msg or json.dumps({"ok": True, "message": "done"})


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
    ap.add_argument("--max-tokens", type=int, default=500)
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
        if args.instruction:
            run_instruction(" ".join(args.instruction), ask, runner,
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
                run_instruction(line, ask, runner, Budget(args.max_tool_calls))
    except KeyboardInterrupt:
        pass
    finally:
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
