#!/usr/bin/env python3
"""The web console's dev harness: REAL server + REAL pure core, fake everything else.

What is REAL here — and this is what makes it honest scaffolding rather than a
parallel implementation: `sphero_rvr_core.web_console_http.make_server` (the
production HTTP/SSE layer, byte-identical), `EventBroker`, `build_state`,
`classify_line`, `grid_to_png`. What is FAKED: the map (a drawn room), the pose
(a slow orbit so motion is visible), the mission status, and the transcript —
POSTing any instruction replays a canned mission covering every ending the chat
must render honestly (tool calls, a look card, model-failure, say, budget).

So the UI can be iterated anywhere — no ROS, no Pi, no model calls — and what
renders here renders identically against the robot, because everything between
the wire and the browser is the production code.

    python3 scripts/fake_web_console.py [--port 8090]
        [--static-dir src/sphero_rvr_driver/web_static] [--photo-dir DIR]

Point a browser at it; POST /api/instruction (or use the page's send box) to see
the canned mission; --photo-dir with a look.jpg in it makes the look card's
photo real. Built during batch D of the 2026-08-20 overnight, where it caught
the flexbox chat-crush bug before the robot ever served a look card.
"""

import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "src"))

from sphero_rvr_core.web_console import (EventBroker, build_state,  # noqa: E402
                                         classify_line, grid_to_png)
from sphero_rvr_core.web_console_http import (install_stop_handlers,
                                              make_server)  # noqa: E402

#: The canned mission: one line per transcript beat, every ending class present.
CANNED = (
    "[tool 1/8] where_am_i()",
    '[result] {"ok": true, "tool": "where_am_i", "x": 0.4, "y": -0.2, '
    '"yaw_deg": 33.0}',
    "[tool 2/8] look_and_recognize(target='dr pepper bottle')",
    '[result] {"target": "dr pepper bottle", "match": true, "identity": '
    '"unverified", "where_in_frame": "center", "confidence": 0.4, '
    '"description": "a clear bottle with a pink label near the storage bin", '
    '"photo_path": "/x/look.jpg", "map_pose": {"x": 0.4, "y": -0.2, '
    '"yaw_deg": 33.0}, "bearing_deg": -107.6, "bearing_relative_deg": -140.6, '
    '"range_m": 1.055, "range_source": "tof", "range_ambiguous": false, '
    '"stamp": "2026-08-20T22:00:00", "model": "syn:large:vision"}',
    "[model-failure] the api fell over mid-mission (canned)",
    "robot> I found a possible bottle at bearing -107.6.",
    "[budget] stopping after 8 tool calls — no final say (canned)",
)


def make_room(width, height):
    """A rectangular room with walls, some unknown fringe."""
    cells = []
    for row in range(height):
        for col in range(width):
            wall = ((row in (4, height - 5) and 6 < col < width - 6)
                    or (col in (6, width - 7) and 4 < row < height - 5))
            inside = 4 < row < height - 5 and 6 < col < width - 7
            cells.append(100 if wall else (0 if inside else -1))
    return cells


class FakeApp:
    """The app protocol the production server expects, backed by nothing."""

    def __init__(self, static_dir, photo_dir):
        self.broker = EventBroker()
        self.stopping = threading.Event()
        self.heartbeat_s = 15.0
        self.static_dir = static_dir
        self.photo_dir = photo_dir
        self._w, self._h = 92, 90
        self._png = grid_to_png(self._w, self._h, make_room(self._w, self._h))
        self._meta = {"stamp": "1.0", "width": self._w, "height": self._h,
                      "resolution_m": 0.05,
                      "origin": {"x": -2.3, "y": -2.25}, "known_pct": 87.5}
        self._t0 = time.time()

    def map_png(self):
        return self._png, self._meta

    def state(self):
        angle = (time.time() - self._t0) * 0.25
        pose = {"x": round(1.2 * math.cos(angle), 3),
                "y": round(0.9 * math.sin(angle), 3),
                "yaw_deg": round((math.degrees(angle) + 90.0) % 360.0 - 180.0, 1)}
        mission = (time.monotonic(), {"running": True, "armed": True,
                                      "goals_succeeded": 2, "done": False})
        return build_state(pose, mission, time.monotonic(), 3.0,
                           {"state": "idle"}, self._meta)

    def start_instruction(self, text):
        self.broker.publish({"type": "instruction", "text": text})
        for line in CANNED:
            self.broker.publish(classify_line(line))
        self.broker.publish({"type": "mission_end"})
        return 202, {"ok": True}

    def stop(self):
        payload = {"ok": True, "goto_cancel": "no goto was in flight (canned)",
                   "mission_stop": '{"message": "mission STOPPED (canned)"}',
                   "note": ("not an emergency stop — the robot coasts to a halt "
                            "and the collision supervisor is untouched")}
        self.broker.publish({"type": "stop", **payload})
        return payload


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--static-dir",
                    default=os.path.join(repo, "src", "sphero_rvr_driver",
                                         "web_static"))
    ap.add_argument("--photo-dir", default="/nonexistent")
    args = ap.parse_args()

    app = FakeApp(args.static_dir, args.photo_dir)

    def tick():
        while not app.stopping.wait(1.0):
            app.broker.publish(app.state(), remember=False)

    threading.Thread(target=tick, daemon=True).start()
    server = make_server("127.0.0.1", args.port, app)
    install_stop_handlers(server)          # the shipped stop path, exercised here
    print(f"fake console on http://127.0.0.1:{server.server_address[1]}/ "
          f"(static: {args.static_dir})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        app.stopping.set()
        server.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
