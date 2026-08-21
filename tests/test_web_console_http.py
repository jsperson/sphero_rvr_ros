"""The HTTP layer against real sockets — including the batch A consensus pins:
client-disconnect mid-stream leaks nothing, a slow client stalls no one, and our
own shutdown path completes with a live SSE client attached.
"""

import http.client
import json
import socket
import threading
import time

import pytest

from sphero_rvr_core.web_console import EventBroker, grid_to_png
from sphero_rvr_core.web_console_http import make_server, shutdown_server


class FakeApp:
    """The app protocol with recording endpoints and no ROS anywhere."""

    def __init__(self, static_dir=None, photo_dir="/nonexistent"):
        self.broker = EventBroker(feed_size=50)
        self.stopping = threading.Event()
        self.heartbeat_s = 0.2          # fast heartbeats so tests bound quickly
        self.static_dir = static_dir
        self.photo_dir = photo_dir
        self.map_result = None
        self.instructions = []
        self.stops = 0

    def map_png(self):
        return self.map_result

    def state(self):
        return {"type": "state"}

    def start_instruction(self, text):
        self.instructions.append(text)
        return 202, {"ok": True, "accepted": text}

    def stop(self):
        self.stops += 1
        return {"ok": True, "note": "not an emergency stop"}


@pytest.fixture()
def served(tmp_path):
    static = tmp_path / "static"
    static.mkdir()
    (static / "index.html").write_text("<html>the real page</html>")
    (static / "app.js").write_text("// js")
    (static / "secret.py").write_text("not servable")
    photos = tmp_path / "photos"
    photos.mkdir()
    (photos / "look_1.jpg").write_bytes(b"\xff\xd8jpeg-ish")

    app = FakeApp(static_dir=str(static), photo_dir=str(photos))
    server = make_server("127.0.0.1", 0, app)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    port = server.server_address[1]
    yield app, server, port
    shutdown_server(server)
    thread.join(timeout=3.0)
    assert not thread.is_alive()


def _get(port, path, headers=None):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=3.0)
    conn.request("GET", path, headers=headers or {})
    resp = conn.getresponse()
    body = resp.read()
    conn.close()
    return resp, body


def _post(port, path, payload):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=3.0)
    body = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
    conn.request("POST", path, body=body,
                 headers={"Content-Type": "application/json",
                          "Content-Length": str(len(body))})
    resp = conn.getresponse()
    out = resp.read()
    conn.close()
    return resp, out


# ---------------------------------------------------------------------------
# plain doors
# ---------------------------------------------------------------------------

def test_static_page_and_confinement(served):
    app, server, port = served
    resp, body = _get(port, "/")
    assert resp.status == 200 and b"the real page" in body
    resp, _ = _get(port, "/app.js")
    assert resp.status == 200
    # a servable-extension allowlist AND path confinement
    assert _get(port, "/secret.py")[0].status == 404
    assert _get(port, "/../setup.py")[0].status == 404
    assert _get(port, "/%2e%2e/setup.py")[0].status == 404


def test_placeholder_when_no_static_dir(served):
    app, server, port = served
    app.static_dir = None
    resp, body = _get(port, "/")
    assert resp.status == 200 and b"web console is up" in body


def test_map_503_then_200_with_meta(served):
    app, server, port = served
    resp, body = _get(port, "/api/map.png")
    assert resp.status == 503
    png = grid_to_png(2, 1, [0, 100])
    app.map_result = (png, {"stamp": "12.5", "resolution_m": 0.05})
    resp, body = _get(port, "/api/map.png")
    assert resp.status == 200
    assert resp.getheader("Content-Type") == "image/png"
    assert body == png
    meta = json.loads(resp.getheader("X-Map-Meta"))
    assert meta["resolution_m"] == 0.05


def test_instruction_endpoint_contract(served):
    app, server, port = served
    resp, body = _post(port, "/api/instruction", {"text": "look around"})
    assert resp.status == 202
    assert app.instructions == ["look around"]
    assert _post(port, "/api/instruction", {"text": "   "})[0].status == 400
    assert _post(port, "/api/instruction", {"wrong": "key"})[0].status == 400
    assert _post(port, "/api/instruction", b"not json")[0].status == 400
    assert app.instructions == ["look around"]     # rejects never reached the app


def test_stop_endpoint(served):
    app, server, port = served
    resp, body = _post(port, "/api/stop", {})
    assert resp.status == 200
    assert json.loads(body)["note"] == "not an emergency stop"
    assert app.stops == 1


def test_photo_served_confined(served):
    app, server, port = served
    resp, body = _get(port, "/api/photo?name=look_1.jpg")
    assert resp.status == 200 and body.startswith(b"\xff\xd8")
    for bad in ("missing.jpg", "../../etc/passwd", "look_1.png", ""):
        assert _get(port, f"/api/photo?name={bad}")[0].status == 404


def test_unknown_api_is_404(served):
    app, server, port = served
    assert _get(port, "/api/nope")[0].status == 404
    assert _post(port, "/api/nope", {})[0].status == 404


# ---------------------------------------------------------------------------
# the SSE stream and its pinned edges
# ---------------------------------------------------------------------------

def _open_sse(port, path="/api/events"):
    """A raw socket SSE client we can read incrementally and abandon rudely."""
    sock = socket.create_connection(("127.0.0.1", port), timeout=3.0)
    sock.sendall(f"GET {path} HTTP/1.1\r\nHost: x\r\n\r\n".encode())
    return sock


def _read_until(sock, needle, deadline_s=3.0):
    buf = b""
    end = time.monotonic() + deadline_s
    sock.settimeout(0.2)
    while time.monotonic() < end:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        buf += chunk
        if needle in buf:
            return buf
    raise AssertionError(f"{needle!r} not seen; got {buf!r}")


def test_sse_replay_then_live(served):
    app, server, port = served
    app.broker.publish({"type": "note", "text": "before"})
    sock = _open_sse(port)
    buf = _read_until(sock, b'"before"')
    assert b"text/event-stream" in buf
    app.broker.publish({"type": "say", "text": "hello there"})
    _read_until(sock, b'"hello there"')
    sock.close()


def test_sse_resumes_from_last_event_id(served):
    app, server, port = served
    for i in range(3):
        app.broker.publish({"type": "note", "text": f"n{i}"})
    sock = _open_sse(port, "/api/events?since=2")
    buf = _read_until(sock, b'"n2"')
    assert b'"n0"' not in buf and b'"n1"' not in buf
    sock.close()


def test_sse_disconnect_leaks_no_client(served):
    app, server, port = served
    sock = _open_sse(port)
    _read_until(sock, b"200 OK")
    assert app.broker.client_count == 1
    sock.close()                        # rude disconnect, mid-stream
    deadline = time.monotonic() + 3.0   # one heartbeat discovers the corpse
    while app.broker.client_count and time.monotonic() < deadline:
        time.sleep(0.05)
    assert app.broker.client_count == 0


def test_slow_sse_client_never_blocks_publish(served):
    app, server, port = served
    sock = _open_sse(port)
    _read_until(sock, b"200 OK")
    t0 = time.monotonic()
    for i in range(2000):               # feed is 50 deep; client reads nothing
        app.broker.publish({"type": "note", "text": f"flood {i}"})
    assert time.monotonic() - t0 < 1.0  # drop-oldest, publisher unstalled
    sock.close()


def test_shutdown_with_live_sse_client(served):
    """THE PIN: our own teardown, with a client attached, completes and drains."""
    app, server, port = served
    sock = _open_sse(port)
    _read_until(sock, b"200 OK")
    t0 = time.monotonic()
    shutdown_server(server)
    assert time.monotonic() - t0 < 2.0
    # the stream ends (EOF) rather than hanging the client forever
    sock.settimeout(2.0)
    while True:
        try:
            if sock.recv(4096) == b"":
                break
        except socket.timeout:
            raise AssertionError("SSE client saw no EOF after shutdown")
    sock.close()
    deadline = time.monotonic() + 2.0
    while app.broker.client_count and time.monotonic() < deadline:
        time.sleep(0.05)
    assert app.broker.client_count == 0


# ---------------------------------------------------------------------------
# the tick-vs-ring rule from the core change riding this batch
# ---------------------------------------------------------------------------

def test_unremembered_events_reach_live_clients_but_not_replay():
    broker = EventBroker()
    cid, feed = broker.register()
    broker.publish({"type": "state"}, remember=False)
    assert feed.get(timeout=0)["type"] == "state"       # live clients see it
    broker.unregister(cid)
    cid2, feed2 = broker.register()
    assert feed2.get(timeout=0) is None                 # replay does not
    broker.unregister(cid2)
