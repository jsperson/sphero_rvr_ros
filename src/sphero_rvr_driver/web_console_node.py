"""The web console: a browser window onto the rover — map, chat, stop. A CLIENT.

**DELETING THIS FILE CHANGES NOTHING ABOUT THE ROBOT** — task_client's Stage D
acceptance test, inherited word for word, because this is the same client grown a
different face. Instructions run through the SAME `run_instruction` loop and the
SAME `ToolRunner` the CLI flights certified; the browser never sees the ROS graph
(no rosbridge — six curated HTTP doors, the task_node philosophy applied to the
web); and every capability used here is a plain service, action, or topic a human
can reach with ros2 CLI. The robot gains no ability by having a browser attached.

What this file owns is WIRING, deliberately nothing else: the pure core
(`sphero_rvr_core.web_console`, `web_console_http`) holds every decision that can
be wrong, under tests that run without ROS. Here live the subscriptions (/map,
/coverage_explorer/status, TF), the mission thread, and the stop path.

THE STOP PATH (batch A ruling 2): stop = cancel whatever `task/goto` is doing —
via the goto action's own standard cancel door, so it catches a chat goto, a CLI
goto, anyone's — then `task/stop` for the mission. task_node's own words about
what stop is NOT (an emergency stop) travel to the browser verbatim. After STOP,
the running instruction's remaining tool calls are refused with a readable
reason, so the model ends in words instead of driving on.

Run it beside a live stack (LAN-only, no auth — the ratified v1 scope):
    ros2 run sphero_rvr_driver web_console
    # then open http://<pi>:8088/ on anything with a browser

NOTE: creates a ToolRunner, whose node name is `task_client` — running the CLI
client at the same time puts two nodes of that name on the graph. Harmless to
ROS, confusing in `ros2 node list`; run one head at a time when it matters.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import rclpy
import tf2_ros
from action_msgs.srv import CancelGoal
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from sphero_rvr_core.task_agent import Budget, availability_note, run_instruction
from sphero_rvr_core.web_console import (EventBroker, build_state, classify_line,
                                         grid_to_png)
from sphero_rvr_core.web_console_http import make_server, shutdown_server
from sphero_rvr_driver.task_client import ToolRunner, make_model_caller

#: Matches map_server's and slam_toolbox's latched publisher, so the last map is
#: delivered on join instead of waiting for the next publish.
MAP_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class WebConsoleNode(Node):
    """Subscriptions and the stop clients. Spun by the executor thread; every
    public method here is called from HTTP threads and only reads under a lock
    or polls a future — the node is never spun from two places (the task_client
    discipline, kept)."""

    def __init__(self):
        super().__init__("web_console")
        self._lock = threading.Lock()
        self._map_msg = None
        self._map_meta = None
        self._map_png_cache = None      # (key, png_bytes)
        self._status_entry = None       # (monotonic_received, payload)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, MAP_QOS)
        self.create_subscription(String, "/coverage_explorer/status",
                                 self._on_status, 10)
        # The stop pair: the ratified goto action's own cancel door (standard
        # action protocol -- an empty goal id means "cancel all"), and task/stop.
        self._cancel_goto = self.create_client(
            CancelGoal, "task/goto/_action/cancel_goal")
        self._task_stop = self.create_client(Trigger, "task/stop")
        self._task_clear_map = self.create_client(Trigger, "task/clear_map")

    # ------------------------------------------------------------ callbacks

    def _on_map(self, msg):
        known = sum(1 for v in msg.data if v >= 0)
        meta = {
            "stamp": f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution_m": round(msg.info.resolution, 4),
            "origin": {"x": round(msg.info.origin.position.x, 3),
                       "y": round(msg.info.origin.position.y, 3)},
            "known_pct": round(100.0 * known / max(1, len(msg.data)), 1),
        }
        with self._lock:
            self._map_msg = msg
            self._map_meta = meta

    def _on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return                      # a malformed line is not evidence either way
        with self._lock:
            self._status_entry = (time.monotonic(), payload)

    # ------------------------------------------------------------ reads

    def pose(self):
        """{x, y, yaw_deg} in the map frame, or None — the same lookup and the
        same refusal-not-guess rule as task_node's where_am_i."""
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link",
                                                  rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {"x": round(t.x, 3), "y": round(t.y, 3),
                "yaw_deg": round(math.degrees(yaw), 1)}

    def status_entry(self):
        with self._lock:
            return self._status_entry

    def map_meta(self):
        with self._lock:
            return self._map_meta

    def map_png(self):
        """(png_bytes, meta) or None. Encoded at most once per distinct map —
        the cache key is the map's own identity, so a latched static map costs
        one encode ever and a growing SLAM map one per revision."""
        with self._lock:
            msg, meta = self._map_msg, self._map_meta
            cache = self._map_png_cache
        if msg is None:
            return None
        key = (meta["stamp"], meta["width"], meta["height"])
        if cache is not None and cache[0] == key:
            return cache[1], meta
        png = grid_to_png(msg.info.width, msg.info.height, msg.data)
        with self._lock:
            self._map_png_cache = (key, png)
        return png, meta

    # ------------------------------------------------------------ stop

    def _finish(self, future, timeout_s):
        """Poll a future the executor thread is completing (task_node's _await
        pattern; this thread never spins the node)."""
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.02)
        return future.result()

    def stop_everything(self):
        """Ruling 2's order: goto cancel first, then mission stop. Both results
        reported as they actually went — an unconfirmed cancel is SAID."""
        result = {}
        if self._cancel_goto.wait_for_service(timeout_sec=2.0):
            reply = self._finish(
                self._cancel_goto.call_async(CancelGoal.Request()), 5.0)
            if reply is None:
                result["goto_cancel"] = "requested but NOT CONFIRMED within 5s"
            else:
                n = len(reply.goals_canceling)
                result["goto_cancel"] = (f"cancelling {n} goal(s)" if n
                                         else "no goto was in flight")
        else:
            result["goto_cancel"] = ("goto server absent — nothing to cancel "
                                     "(is task_node running?)")
        if self._task_stop.wait_for_service(timeout_sec=2.0):
            reply = self._finish(self._task_stop.call_async(Trigger.Request()), 10.0)
            result["mission_stop"] = (reply.message if reply is not None
                                      else "task/stop did not answer within 10s")
        else:
            result["mission_stop"] = "task/stop unavailable — is task_node running?"
        return result

    def clear_map(self):
        """Forward to task/clear_map — the same door the model uses; the button
        adds no authority the chat does not have."""
        if not self._task_clear_map.wait_for_service(timeout_sec=2.0):
            return {"ok": False,
                    "message": "task/clear_map unavailable — is task_node running?"}
        # The clear sleeps a settle second inside task_node; allow for it.
        reply = self._finish(self._task_clear_map.call_async(Trigger.Request()), 20.0)
        if reply is None:
            return {"ok": False, "message": "clear_map did not answer within 20s"}
        try:
            return json.loads(reply.message)
        except ValueError:
            return {"ok": bool(reply.success), "message": reply.message}


class GuardedRunner:
    """After STOP, every tool call becomes a refusal the model can read and end
    on — the mission is not killed mid-thought (a fake [model-failure] would be
    a lie about what happened), it is told, and the budget bounds the rest."""

    def __init__(self, runner, stop_event):
        self._runner = runner
        self._stop = stop_event

    def run(self, tool, args):
        if self._stop.is_set():
            return json.dumps({"ok": False, "message":
                               "the operator pressed STOP — run no more tools; "
                               "report what you know so far"})
        return self._runner.run(tool, args)


class MissionManager:
    """One instruction at a time; this lock IS the 409 the design note promised."""

    def __init__(self, runner, ask, broker, max_tool_calls):
        self._runner = runner
        self._ask = ask
        self._broker = broker
        self._max_tool_calls = max_tool_calls
        self._busy = threading.Lock()
        self._stop_requested = threading.Event()
        self._progress = None
        self._thread = None

    def state(self):
        if self._busy.locked():
            return {"state": "running", "tool": self._progress}
        return {"state": "idle"}

    def start(self, text):
        if self._ask is None:
            return 503, {"ok": False, "error":
                         "no model API key on this host — the console can watch "
                         "but not instruct (map/status/stop still work)"}
        if not self._busy.acquire(blocking=False):
            return 409, {"ok": False, "error":
                         "an instruction is already running — one at a time"}
        self._stop_requested.clear()
        self._progress = None
        self._broker.publish({"type": "instruction", "text": text})
        self._thread = threading.Thread(target=self._work, args=(text,),
                                        daemon=True, name="mission")
        self._thread.start()
        return 202, {"ok": True}

    def request_stop(self):
        self._stop_requested.set()

    def join(self, timeout_s):
        thread = self._thread
        if thread is not None:
            thread.join(timeout=timeout_s)
            return not thread.is_alive()
        return True

    def _emit(self, line):
        event = classify_line(line)
        if event["type"] == "tool_call":
            self._progress = (event["n"], event["max"])
        self._broker.publish(event)

    def _work(self, text):
        try:
            # Per-instruction, not per-process: this server outlives stack
            # restarts, and a preamble describing last hour's graph is a lie.
            note = availability_note(self._runner.probe_availability(),
                                     self._runner.capability_reasons)
            if note:
                self._emit(note.strip())
            run_instruction(note + text, self._ask,
                            GuardedRunner(self._runner, self._stop_requested),
                            Budget(self._max_tool_calls), out=self._emit)
        except Exception as exc:  # noqa: BLE001 — the uniform R6a response
            self._broker.publish({"type": "error",
                                  "text": f"the mission thread failed: {exc}"})
        finally:
            try:
                # F3 inherited: never leave this frame holding a live goto.
                self._runner.shutdown_safely()
            except Exception:
                pass
            self._progress = None
            self._busy.release()
            self._broker.publish({"type": "mission_end"})


class ConsoleApp:
    """What the HTTP layer sees. Wiring only."""

    def __init__(self, node, mission, broker, photo_dir, static_dir,
                 status_max_age_s=3.0, heartbeat_s=15.0):
        self.broker = broker
        self.stopping = threading.Event()
        self.heartbeat_s = heartbeat_s
        self.photo_dir = photo_dir
        self.static_dir = static_dir
        self._node = node
        self._mission = mission
        self._status_max_age_s = status_max_age_s

    def map_png(self):
        return self._node.map_png()

    def state(self):
        return build_state(self._node.pose(), self._node.status_entry(),
                           time.monotonic(), self._status_max_age_s,
                           self._mission.state(), self._node.map_meta())

    def start_instruction(self, text):
        return self._mission.start(text)

    def clear_map(self):
        result = self._node.clear_map()
        self.broker.publish({"type": "map_cleared", **result})
        return result

    def stop(self):
        # Refuse further tools FIRST so the running mission cannot slip a new
        # goto in behind the cancel.
        self._mission.request_stop()
        result = self._node.stop_everything()
        payload = {"ok": True, **result,
                   "note": ("not an emergency stop — the robot coasts to a halt "
                            "and the collision supervisor is untouched")}
        self.broker.publish({"type": "stop", **payload})
        return payload

    def tick_forever(self):
        """The 1 Hz state tick. remember=False: ticks reach live clients but
        never flush the transcript out of the replay ring."""
        while not self.stopping.wait(1.0):
            self.broker.publish(self.state(), remember=False)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--photo-dir", default=os.path.expanduser("~/recognitions"))
    # The model flags mirror task_client exactly — same provider, same key file,
    # same defaults, same reasoning-burn doctrine on max-tokens.
    ap.add_argument("--model", default="syn:large:text")
    ap.add_argument("--base-url", default="https://api.synthetic.new/v1")
    ap.add_argument("--api-key-file",
                    default=os.path.expanduser("~/.config/synthetic/api_key"))
    ap.add_argument("--max-tool-calls", type=int, default=8)
    ap.add_argument("--max-tokens", type=int, default=1500)
    ap.add_argument("--model-timeout-s", type=float, default=60.0)
    ap.add_argument("--tool-timeout-s", type=float, default=180.0)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    ask = None
    try:
        with open(args.api_key_file) as fh:
            key = fh.read().strip()
        ask = make_model_caller(args.base_url, key, args.model,
                                args.max_tokens, args.model_timeout_s)
    except OSError as exc:
        # Watch-only is a real mode, not a crash: the console still shows the
        # map and status and the STOP button still works. Instructions answer
        # 503 with this same honesty.
        print(f"no model API key ({exc}) — starting in watch-only mode", flush=True)

    rclpy.init()
    node = WebConsoleNode()
    runner = ToolRunner(timeout_s=args.tool_timeout_s)
    executor = SingleThreadedExecutor()
    executor.add_node(node)             # the runner is spun by mission threads only
    spin_thread = threading.Thread(target=executor.spin, daemon=True, name="spin")
    spin_thread.start()

    broker = EventBroker()
    mission = MissionManager(runner, ask, broker, args.max_tool_calls)
    static_dir = os.path.join(os.path.dirname(__file__), "web_static")
    app = ConsoleApp(node, mission, broker, args.photo_dir, static_dir)
    tick_thread = threading.Thread(target=app.tick_forever, daemon=True,
                                   name="tick")
    tick_thread.start()

    server = make_server(args.host, args.port, app)
    node.get_logger().info(
        f"web console listening on http://{args.host}:{args.port}/ "
        f"({'chat enabled' if ask else 'WATCH-ONLY, no model key'})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # The owned teardown, in dependency order: stop serving, refuse the
        # mission further tools, give it a moment to end in words, and make
        # CERTAIN no goto outlives us before the nodes go away (F3 again —
        # a client that just exits leaves the rover driving).
        shutdown_server(server)
        mission.request_stop()
        clean = mission.join(3.0)
        if not clean:
            try:
                runner.shutdown_safely()
            except Exception as exc:
                print(f"WARNING: could not cancel the in-flight goto ({exc}); "
                      "use the STOP service.", flush=True)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
