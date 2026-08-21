"""The web console's pure core: event typing, fanout, the map PNG, photo confinement.

Everything in this file runs without ROS, a network, a browser, or a robot, which is
why it lives in the core (this repo's rule: anything testable without ROS belongs
where it can be tested). The node half (`sphero_rvr_driver.web_console_node`) owns
sockets and subscriptions and NOTHING clever; every decision that can be wrong lives
here, under tests.

THE CLASSIFIER'S SEAM, stated: `task_agent.run_instruction` speaks through its
`out=` callback in stable one-line markers (`[tool n/m]`, `[result]`, `robot>`,
`[reprompt]`, `[refused]`, `[model-failure]`, `[budget]`). Classifying our own
strings is still an inference at a seam, so `tests/test_web_console.py` drives the
REAL loop and asserts every line it emits classifies to a non-note type — marker
drift breaks the build, never the browser silently. The loop itself is untouched:
the whole point of building the web console as a second head on the certified
client machinery is that nothing about the mission changes when a browser watches.
"""

import json
import os
import re
import struct
import threading
import zlib
from collections import deque

# ---------------------------------------------------------------------------
# transcript classification
# ---------------------------------------------------------------------------

_TOOL_RE = re.compile(r"^\[tool (\d+)/(\d+)\] (.+)$")

#: Marker prefix -> event type. Order matters only in that tool_call is regex-first.
_PREFIXES = (
    ("[result] ", "tool_result"),
    ("robot> ", "say"),
    ("[reprompt] ", "reprompt"),
    ("[refused] ", "refused"),
    ("[model-failure] ", "model_failure"),
    ("[budget] ", "budget"),
)

#: What a look card may carry to the browser. `photo_path` is deliberately NOT in
#: this list: the browser gets a basename and earns the bytes back through the
#: confined /api/photo door, never a filesystem path.
_LOOK_KEYS = (
    "target", "match", "identity", "where_in_frame", "confidence", "description",
    "map_pose", "bearing_deg", "bearing_relative_deg",
    "range_m", "range_source", "range_ambiguous", "stamp", "model",
)


def _try_json(text):
    try:
        data = json.loads(text)
    except ValueError:
        return None
    return data if isinstance(data, dict) else None


def extract_look(data):
    """A recognition result's card fields, or None if this dict is not one.

    Detection is by the result's own contract (`recognition.build_result`): a
    `match` verdict travelling WITH its `photo_path` provenance. A match=false
    result still gets a card — an honest "not it" with its photo is exactly the
    watch-item's inspection habit, and hiding it would be the UI deciding which
    verdicts the operator sees.
    """
    if not isinstance(data, dict) or "match" not in data or "photo_path" not in data:
        return None
    look = {k: data[k] for k in _LOOK_KEYS if k in data}
    look["photo"] = os.path.basename(str(data["photo_path"]))
    return look


def classify_line(line):
    """One `run_instruction` transcript line -> one typed event dict.

    Every line keeps its verbatim text under "text" — rendering may summarise,
    but the loop's own words are always in the event (endings rendered as what
    they are, batch A ratification). Unknown lines become type "note", shown
    rather than dropped: a marker this classifier does not know is information,
    not noise.
    """
    m = _TOOL_RE.match(line)
    if m:
        return {"type": "tool_call", "n": int(m.group(1)), "max": int(m.group(2)),
                "call": m.group(3), "text": line}
    for prefix, kind in _PREFIXES:
        if line.startswith(prefix):
            text = line[len(prefix):]
            event = {"type": kind, "text": text}
            if kind == "tool_result":
                data = _try_json(text)
                if data is not None:
                    event["data"] = data
                    look = extract_look(data)
                    if look is not None:
                        event["look"] = look
            return event
    return {"type": "note", "text": line}


# ---------------------------------------------------------------------------
# the state tick
# ---------------------------------------------------------------------------

def build_state(pose, mission_entry, now, max_age_s, chat, map_meta):
    """The 1 Hz state tick, with task/status's age-honesty inherited verbatim.

    `pose` arrives already null-honest (the node passes None when the TF lookup
    fails — never a held stale pose). `mission_entry` is (monotonic_received,
    payload) or None; an absent or aged status is REPORTED as absent or stale
    with its age, because a console that smooths silence into a live-looking
    report converts a stuck rover's most informative symptom into reassurance.
    """
    if mission_entry is None:
        mission = {"available": False, "stale": False,
                   "reason": "no mission status has ever been received"}
    else:
        at, payload = mission_entry
        age = now - at
        if age > max_age_s:
            mission = {"available": False, "stale": True,
                       "age_s": round(age, 1),
                       "reason": (f"mission status is STALE ({age:.1f}s old, "
                                  f"limit {max_age_s:.0f}s)"),
                       "last_known": payload}
        else:
            mission = {"available": True, "stale": False,
                       "age_s": round(age, 2), "data": payload}
    return {"type": "state", "pose": pose, "mission": mission,
            "chat": chat, "map": map_meta}


# ---------------------------------------------------------------------------
# SSE fanout
# ---------------------------------------------------------------------------

def format_sse(event):
    """One event -> SSE wire bytes. `id:` carries the broker id so a reconnecting
    EventSource resumes from where it was (Last-Event-ID -> replay)."""
    lines = []
    if "id" in event:
        lines.append(f"id: {event['id']}")
    lines.append("data: " + json.dumps(event, sort_keys=True))
    return ("\n".join(lines) + "\n\n").encode("utf-8")


#: Comment line per the SSE spec: ignored by EventSource, but a real socket write,
#: which is what bounds a dead connection's thread life on an otherwise idle stream.
SSE_HEARTBEAT = b": keepalive\n\n"


class ClientFeed:
    """One SSE client's bounded queue. Appending to a full deque(maxlen=...) drops
    from the opposite end — drop-OLDEST, by construction rather than by code. A
    drop shows up at the client as an id gap, never as a silent rewrite."""

    def __init__(self, maxlen=200):
        self._q = deque(maxlen=maxlen)
        self._cond = threading.Condition()

    def offer(self, event):
        with self._cond:
            self._q.append(event)
            self._cond.notify()

    def get(self, timeout=None):
        """Next event, or None on timeout (the caller's heartbeat moment)."""
        with self._cond:
            if not self._q:
                self._cond.wait(timeout)
            return self._q.popleft() if self._q else None


class EventBroker:
    """Fans events out to SSE clients and remembers the recent past.

    THE SLOW-CLIENT RULE, mechanism named (batch A consensus pin): publish()
    never touches a socket. It stamps an id, appends to the replay ring, and
    offers to bounded per-client queues; socket writes happen only in each
    connection's own request thread draining its own feed. A stuck reader
    therefore stalls exactly one thread — its own — and the 1 Hz tick and every
    other client are untouched.
    """

    def __init__(self, ring_size=500, feed_size=200):
        self._lock = threading.Lock()
        self._ring = deque(maxlen=ring_size)
        self._clients = {}
        self._next_event_id = 1
        self._next_client_id = 1
        self._feed_size = feed_size

    def publish(self, event):
        """Stamp, remember, fan out. Returns the stamped event."""
        with self._lock:
            event = dict(event)
            event["id"] = self._next_event_id
            self._next_event_id += 1
            self._ring.append(event)
            feeds = list(self._clients.values())
        for feed in feeds:
            feed.offer(event)
        return event

    def register(self, last_seen_id=0):
        """A new client: replay everything after `last_seen_id`, then live events.

        Replay and registration happen under ONE lock so no event can fall in the
        gap between them — published-after events reach the feed directly, and
        the replayed ones are already in it, in order.
        """
        feed = ClientFeed(maxlen=self._feed_size)
        with self._lock:
            cid = self._next_client_id
            self._next_client_id += 1
            for event in self._ring:
                if event["id"] > last_seen_id:
                    feed.offer(event)
            self._clients[cid] = feed
        return cid, feed

    def unregister(self, cid):
        with self._lock:
            self._clients.pop(cid, None)

    @property
    def client_count(self):
        with self._lock:
            return len(self._clients)


# ---------------------------------------------------------------------------
# the map PNG
# ---------------------------------------------------------------------------

#: Dark theme's floor tones. Unknown darker than free so the explored world reads
#: as carved out of the dark; occupied bright so walls draw the eye.
_UNKNOWN_RGB = (0x10, 0x12, 0x16)
_FREE_RGB = (0x24, 0x28, 0x2E)
_OCCUPIED_RGB = (0xE9, 0xEB, 0xEE)


def _build_palette():
    """256 palette entries so a raw occupancy byte IS its palette index: 0..100
    shade free->occupied linearly, -1 (byte 255) and every impossible value get
    the unknown tone."""
    pal = bytearray()
    for i in range(256):
        if i <= 100:
            t = i / 100.0
            rgb = tuple(round(f + (o - f) * t)
                        for f, o in zip(_FREE_RGB, _OCCUPIED_RGB))
        else:
            rgb = _UNKNOWN_RGB
        pal.extend(rgb)
    return bytes(pal)


_PALETTE = _build_palette()


def _chunk(tag, payload):
    body = tag + payload
    return (struct.pack(">I", len(payload)) + body
            + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))


def grid_to_png(width, height, data):
    """OccupancyGrid cells -> indexed-colour PNG bytes. Stdlib only, no cv2/numpy.

    Row 0 of an OccupancyGrid is the BOTTOM row (origin at the lower-left corner);
    PNG scanlines are top-first, so rows are written in reverse and the image
    displays map-correct (+y up) with no client-side flip to forget.
    """
    cells = list(data)
    if width <= 0 or height <= 0 or width * height != len(cells):
        raise ValueError(
            f"grid shape {width}x{height} does not match {len(cells)} cells")
    packed = bytes(v & 0xFF for v in cells)   # -1 (unknown) -> 255
    raw = bytearray()
    for row in range(height - 1, -1, -1):
        raw.append(0)                          # PNG filter type: None
        raw.extend(packed[row * width:(row + 1) * width])
    return b"".join([
        b"\x89PNG\r\n\x1a\n",
        _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 3, 0, 0, 0)),
        _chunk(b"PLTE", _PALETTE),
        _chunk(b"IDAT", zlib.compress(bytes(raw), 6)),
        _chunk(b"IEND", b""),
    ])


# ---------------------------------------------------------------------------
# photo confinement
# ---------------------------------------------------------------------------

_PHOTO_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*\.jpg$")


def safe_photo_path(photo_dir, name):
    """The photo file the browser may have, or None. Refusal is the default.

    Three fences, each sufficient alone: the name must be a plain `*.jpg`
    basename (no separators, no leading dot), the joined path must realpath to
    directly inside `photo_dir` (which kills symlink escapes AND any traversal
    the regex somehow missed), and the file must actually exist.
    """
    if not isinstance(name, str) or not _PHOTO_NAME_RE.match(name):
        return None
    if "/" in name or "\\" in name or ".." in name:
        return None
    root = os.path.realpath(photo_dir)
    path = os.path.realpath(os.path.join(root, name))
    if os.path.dirname(path) != root:
        return None
    if not os.path.isfile(path):
        return None
    return path
