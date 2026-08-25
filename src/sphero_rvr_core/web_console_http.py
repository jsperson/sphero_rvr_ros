"""The web console's HTTP layer: six doors, stdlib only, no ROS anywhere in it.

Lives in the pure core because it IS testable without ROS — every SSE edge the
batch A consensus pinned (slow client, mid-stream disconnect, our own shutdown
path) gets exercised against a real socket in `tests/test_web_console_http.py`,
on any machine, with a fake app behind it. The node half only builds the app.

The server talks to an injected `app` object and knows nothing else:

    app.broker              EventBroker (fanout + replay)
    app.stopping            threading.Event — set by shutdown, drains SSE loops
    app.heartbeat_s         float — SSE keepalive cadence (and the bound on how
                            long a dead connection can hold a thread)
    app.static_dir          str | None — the page's files
    app.photo_dir           str — recognition photos, served CONFINED
    app.map_png()           -> (png_bytes, meta_dict) | None
    app.state()             -> dict (same shape as the 1 Hz tick)
    app.start_instruction(text) -> (http_status, payload_dict)
    app.stop()              -> payload_dict

WHY stdlib (batch A ruling 1): the Pi carries none of the async frameworks, our
only streaming need is one-way, and a hand-rolled SSE loop whose failure modes
are named and tested beats an overnight-invented dependency deployment. The
slow-client rule is structural: this file's threads write ONLY to their own
client's socket, draining their own bounded feed — `EventBroker.publish` never
blocks on anyone's network.
"""

import json
import os
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from sphero_rvr_core.web_console import SSE_HEARTBEAT, format_sse, safe_photo_path

_STATIC_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
}

#: What "/" serves when no static dir is present (batch C runs headless-first;
#: the real page lands in batch D and simply takes precedence).
_PLACEHOLDER = (b"<!doctype html><title>rvr web console</title>"
                b"<p>web console is up; the page arrives in a later batch. "
                b"The API lives under /api/.</p>")


class ConsoleHandler(BaseHTTPRequestHandler):
    server_version = "rvr-web-console/1"
    protocol_version = "HTTP/1.1"

    @property
    def app(self):
        return self.server.app

    def log_message(self, fmt, *args):
        # Quiet by design: per-request lines at 1 Hz map polls would be noise.
        # Failures speak through status codes the client renders honestly.
        pass

    # ------------------------------------------------------------------ util

    def _send(self, code, body, ctype="application/json", extra=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, sort_keys=True).encode("utf-8")
        elif isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    # ------------------------------------------------------------------ GET

    def do_GET(self):
        parts = urlsplit(self.path)
        path, query = parts.path, parse_qs(parts.query)
        if path == "/api/events":
            return self._events(query)
        if path == "/api/map.png":
            return self._map()
        if path == "/api/photo":
            return self._photo(query)
        if path.startswith("/api/"):
            return self._send(404, {"error": f"no such endpoint {path}"})
        return self._static(path)

    def _events(self, query):
        """The SSE stream. This thread serves exactly one client and touches
        exactly one socket — its own. A reconnecting EventSource resumes via
        Last-Event-ID (or ?since=N) into the broker's replay ring."""
        last_seen = 0
        header = self.headers.get("Last-Event-ID", "")
        since = (query.get("since") or [""])[0]
        if header.isdigit():
            last_seen = int(header)
        elif since.isdigit():
            last_seen = int(since)
        cid, feed = self.app.broker.register(last_seen_id=last_seen)
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            # No Content-Length on an endless body; close, never keep-alive.
            self.send_header("Connection", "close")
            self.end_headers()
            while not self.app.stopping.is_set():
                event = feed.get(timeout=self.app.heartbeat_s)
                # The heartbeat is a real write: it is HOW a dead connection
                # gets discovered on an otherwise idle stream, bounding this
                # thread's life to one heartbeat past the disconnect.
                self.wfile.write(format_sse(event) if event else SSE_HEARTBEAT)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # the client left; falling out of this frame IS the cleanup
        finally:
            self.app.broker.unregister(cid)

    def _map(self):
        result = self.app.map_png()
        if result is None:
            return self._send(503, {"error": "no map received yet"})
        png, meta = result
        return self._send(200, png, ctype="image/png",
                          extra={"X-Map-Meta": json.dumps(meta, sort_keys=True)})

    def _photo(self, query):
        name = (query.get("name") or [""])[0]
        path = safe_photo_path(self.app.photo_dir, name)
        if path is None:
            # One answer for absent, malformed, and escaping names alike: a
            # probe learns nothing about which fence it hit.
            return self._send(404, {"error": "no such photo"})
        with open(path, "rb") as fh:
            return self._send(200, fh.read(), ctype="image/jpeg")

    def _static(self, path):
        if path == "/":
            path = "/index.html"
        root = self.app.static_dir
        if root:
            root = os.path.realpath(root)
            candidate = os.path.realpath(os.path.join(root, path.lstrip("/")))
            # Same fence as photos: directly inside the root, or nothing.
            if os.path.dirname(candidate) == root and os.path.isfile(candidate):
                ext = os.path.splitext(candidate)[1]
                ctype = _STATIC_TYPES.get(ext)
                if ctype:
                    with open(candidate, "rb") as fh:
                        return self._send(200, fh.read(), ctype=ctype)
        if path == "/index.html":
            return self._send(200, _PLACEHOLDER, ctype="text/html; charset=utf-8")
        return self._send(404, {"error": "not found"})

    # ------------------------------------------------------------------ POST

    def do_POST(self):
        if self.path == "/api/instruction":
            return self._instruction()
        if self.path == "/api/stop":
            return self._send(200, self.app.stop())
        if self.path == "/api/map/clear":
            return self._send(200, self.app.clear_map())
        return self._send(404, {"error": f"no such endpoint {self.path}"})

    def _instruction(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(length).decode("utf-8"))
            text = body["text"]
        except (ValueError, KeyError, UnicodeDecodeError):
            return self._send(400, {"ok": False,
                                    "error": 'body must be JSON: {"text": "..."}'})
        if not isinstance(text, str) or not text.strip():
            return self._send(400, {"ok": False,
                                    "error": "instruction text must be non-empty"})
        code, payload = self.app.start_instruction(text.strip())
        return self._send(code, payload)


class ConsoleServer(ThreadingHTTPServer):
    # Daemon request threads: a straggler cannot hold the process open past the
    # owned shutdown below — the explore-launch teardown lesson, applied to us.
    daemon_threads = True
    allow_reuse_address = True


def make_server(host, port, app):
    server = ConsoleServer((host, port), ConsoleHandler)
    server.app = app
    return server


def shutdown_server(server, join_timeout_s=None):
    """The owned shutdown path (batch A pin), in order: raise `stopping` so every
    SSE loop exits at its next heartbeat, stop the accept loop, close the
    listening socket. Returns only when the accept loop has actually stopped."""
    server.app.stopping.set()
    server.shutdown()
    server.server_close()


def install_stop_handlers(server, signums=(signal.SIGTERM, signal.SIGINT)):
    """Make SIGTERM actually stop the console. D74.

    2026-08-25 teardown: this process survived a process-group SIGINT, a direct
    SIGINT and a SIGTERM, and was still answering HTTP 200 half a minute later.
    Python's DEFAULT SIGTERM disposition would have killed it -- so something had
    installed a handler, and in this process the only candidate is `rclpy.init()`,
    whose handlers ask the ROS context to shut down and know nothing about a
    blocking `serve_forever()` on the main thread. The signal was delivered,
    handled, and changed nothing.

    A SIGTERM-deaf process reparented to init is what systemd WAITS OUT for a full
    TimeoutStopUSec, which is why this is a shutdown-slowness fix.

    Two details that are load-bearing rather than stylistic:

    * `server.shutdown()` BLOCKS until the accept loop exits, and the accept loop
      is the thread the signal handler runs on. Calling it inline self-deadlocks,
      so the stop runs on its own thread and the handler returns immediately.
    * The previous handler is CHAINED, not replaced. rclpy still gets to shut its
      context down; we only add the half it was never going to do.

    Returns `serve_forever()` to its caller; the caller's own `finally` still owns
    the teardown (`shutdown_server`, which is what frees the port).
    """
    previous = {}

    def _stop(signum, frame):
        threading.Thread(target=server.shutdown, name="console-stop",
                         daemon=True).start()
        earlier = previous.get(signum)
        if callable(earlier):
            earlier(signum, frame)

    for signum in signums:
        try:
            previous[signum] = signal.signal(signum, _stop)
        except (ValueError, OSError):
            # not the main thread, or a platform without this signal: the caller
            # keeps whatever behaviour it had rather than losing the server.
            continue
    return server
