# Design note: the web console (map + chat), from scratch — 2026-08-20 overnight

Scott's instruction, verbatim: "Let's start working on the web interface overnight.
Start from scratch. Do not use previous web interface code. The interface should be
simple to start. A map that is built during the run and a chat interface for the LLM
that guides the rover."

PM-ratified scope: two panes (live map + chat), STOP button labeled
convenience-not-safety, phone-first, presentation-only backend exposing EXACTLY
map/pose/chat/status/stop, LAN-only no-auth v1, dev loop against the sim rig,
morning demo = a rig mission visible in a browser.

FROM-SCRATCH COMPLIANCE: prior web attempts exist in history only as artifact
directories (`artifacts/web_interface_mvp/`, `artifacts/pi_web_mission_stack/`,
`artifacts/web_mission_console_map_first/` — PNG mockups by filename). Nothing was
read beyond filenames; nothing is imported. Everything below builds on the LIVE,
certified surfaces: `task_node`'s services, `task_client.ToolRunner`,
`task_agent.run_instruction`, `/map`, `/coverage_explorer/status`, TF.

## 1. What it is, in one paragraph

One small process on the Pi (`ros2 run sphero_rvr_driver web_console`) that is a
CLIENT of the task surface — the task_node philosophy applied to the web. Deleting
it changes nothing about the robot (task_client's own acceptance test, inherited
verbatim). It serves a single static page and six narrow endpoints; the browser
never sees the ROS graph. No rosbridge, no topic names in JavaScript, no new
authority anywhere: every instruction goes through the SAME `run_instruction` loop
and the SAME `ToolRunner` the CLI flights certified, and the model cannot reach
anything the CLI model can't.

## 2. Backend surface — every endpoint named with its ROS source

| Endpoint | Method | ROS source | Notes |
|---|---|---|---|
| `/` + static assets | GET | none | one HTML page, one CSS, one JS; no build step |
| `/api/map.png` | GET | `/map` (OccupancyGrid, TRANSIENT_LOCAL sub, latest held) | grid→PNG server-side (cv2, already on the Pi via the camera stack); headers carry stamp, resolution, origin, size; unknown=dark, free=floor tone, occupied=light |
| `/api/events` (SSE) | GET | see below | the one live stream; auto-reconnect with replay |
| `/api/instruction` | POST | `task/*` via the EXISTING `ToolRunner` + `run_instruction` | body `{"text": ...}`; 202 accepted / 409 busy — ONE instruction at a time, enforced server-side |
| `/api/stop` | POST | own in-flight goto cancel + `task/stop` (Trigger) | see §6 |
| `/api/photo?name=…` | GET | recognition photo files (`~/recognitions/`) | basename-only, confined to photo_dir; 404 outside it |

The SSE stream `/api/events` carries exactly two families:

* `state`, 1 Hz tick: `pose {x, y, yaw_deg} | null` (TF `map→base_link`, the same
  lookup task_node's `where_am_i` uses; lookup failure = `null`, never a stale
  pose — null-honesty), mission status (latest `/coverage_explorer/status` JSON
  **with its age**; absent/stale is REPORTED as absent/stale, the task/status
  doctrine), chat state (`idle | running {tool_n, tool_max}`), and the current
  map stamp + known% (so the client knows when to refetch the PNG).
* transcript events, as they happen (§5).

Server-Sent Events, not websockets, because our only streaming need is one-way;
instructions and stop are plain POSTs. `EventSource` reconnects natively on
phones, and with per-event ids the backend replays the transcript ring buffer
(last 500 events, in-memory, per-process) so a locked phone screen doesn't lose
the conversation.

## 3. The framework decision (needs consensus — it deviates from the brief's letter)

The brief said "small FastAPI/websocket class server". **The Pi has none of
fastapi, uvicorn, aiohttp, or websockets installed** (probed tonight over ssh,
all four ModuleNotFoundError). Installing them means either pip into system
Python (forbidden by standing rule) or inventing a venv-with-system-site-packages
deployment for rclpy overnight — a new mechanism, unreviewed, for a v1 that
doesn't need bidirectional streaming.

**Recommendation: stdlib only.** `http.server.ThreadingHTTPServer` + hand-rolled
SSE (SSE is ~10 lines: a header and `data:` lines on a kept-open response).
Zero new dependencies, zero deployment machinery, built-in-first. Honest limits,
stated: a threading server is right for 1–3 LAN clients and wrong for the open
internet — which is the ratified v1 scope anyway (LAN-only, no auth). FastAPI
remains the v2 door if the surface ever grows real routing/auth needs.

**The hand-rolled SSE's ugly edges, mechanisms NAMED and tested (consensus pin,
batch A ratification):**

* *Slow client cannot stall anyone:* the publisher never touches a socket. It
  appends to the ring and offers to BOUNDED per-client queues (drop-oldest on
  overflow); socket writes happen only in each connection's own request thread,
  draining its own queue. A stuck reader stalls exactly its own thread; the
  1 Hz tick and every other client are untouched. Because events carry ids, a
  drop is visible as an id gap, not a silent rewrite of history.
* *Disconnect leaks no thread:* a write to a closed socket raises in that
  client's own thread, which unregisters its queue in a `finally`. An IDLE
  stream still detects disconnect: a comment-line heartbeat every 15 s bounds
  how long a dead connection can hold a thread.
* *Server shutdown is owned:* `shutdown()` + close on SIGINT/SIGTERM, clients
  unregistered, the mission thread's `shutdown_safely` (cancel in-flight goto)
  runs BEFORE node teardown — the explore-launch teardown lesson applied to
  our own server, and tested rather than assumed.

## 4. Process architecture

Two rclpy nodes in one process, spun the way the certified code already spins:

* `web_console` node — the /map + /coverage_explorer/status subscriptions and the
  TF listener. Spun by a background executor thread, owns nothing movable.
* `ToolRunner` (imported from `task_client` — the ratified contract path) —
  created per process, spun ONLY inside the mission thread via its own
  `_spin_until`, exactly as the CLI does it. One mission thread at a time
  (a plain lock is the 409).

The model caller is `make_model_caller` reused as-is: same Synthetic key file on
the Pi, same `syn:large:text`, same sticky-escalation dict per process, same
1500-token base with the escalating ladder. The availability preamble
(`probe_availability` + `availability_note`) runs per instruction, not per
process — the backend outlives stack restarts, and a preamble describing last
hour's graph is a lie.

HTTP thread → (lock acquired) → mission thread runs
`run_instruction(note + text, ask, runner, Budget(8), out=emit)` → events fan out
to SSE subscribers → lock released, `mission_end` emitted. Process exit path
inherits `shutdown_safely` (cancel any in-flight goto before dying — F3).

## 5. Chat lifecycle semantics — endings rendered as what they are

`run_instruction`'s `out=` callback is the seam; the loop itself is UNTOUCHED.
Events are classified from the loop's own stable markers (`[tool n/m]`,
`[result]`, `robot>`, `[reprompt]`, `[refused]`, `[model-failure]`, `[budget]`).
Because classifying our own strings is still an inference at a seam, a pinned
test imports `task_agent`, runs a canned transcript through the real loop, and
asserts every marker classifies — marker drift breaks the build, not the UI.

Rendering rules, fixed:

* `say` → the robot's answer bubble. The ONLY thing styled as an answer.
* `[model-failure]` → a red card titled "model failure", the loop's own text
  verbatim. Never re-worded, never softened into "done".
* `[budget]` wordless ending → amber card: "budget exhausted without a final
  answer". `[refused]` → red card with the contract error.
* `tool_call` → a compact grey line ("tool 3/8 — turn(degrees=90)");
  `tool_result` → collapsible detail under it, raw JSON preserved.
* A `tool_result` whose JSON carries recognition fields (`match`, `identity`,
  `photo_path`, …) additionally renders THE LOOK CARD in-chat: photo thumbnail
  (via `/api/photo`, tap for full size) + match / identity / range_m +
  range_source (with the ambiguity flag when set) / bearing_deg &
  bearing_relative_deg / provenance line (pose, stamp, model id). The photo and
  the verdict travel together — the watch-item's inspection habit, built into
  the UI.
* Send box disabled while a mission runs (server enforces regardless); the
  running state shows "tool 3/8" from the state tick. Send re-enables on
  `mission_end`, whatever the ending was.

## 6. The STOP button

Top-right, always visible, one tap, no confirmation dialog (it's convenience;
a dialog is where taps go to die). Wiring, in order:

1. cancel the backend's OWN in-flight `task/goto` handle if any (the existing
   `_cancel_and_confirm` — mission/stop does not know about a chat goto; the
   client that sent it must cancel it);
2. call `task/stop` (Trigger) — disarms any coverage mission and cancels its goal.

Both results reported honestly in-chat as a system card, including task_node's
own sentence: "This is not an emergency stop: the robot coasts to a halt and the
collision supervisor is untouched." The button label is **Stop** with the
subtitle "not an e-stop" — the UI never claims a brake it doesn't have.

## 7. Layout sketch (phone-first, Apple-grade restraint)

Portrait phone:

```
┌────────────────────────────────┐
│ RVR        ● exploring   STOP  │  header: status pill + stop, always visible
├────────────────────────────────┤
│                                │
│         MAP  (≈40vh)           │  canvas: map PNG + pose triangle (heading),
│            ▲                   │  auto-fit, pinch-zoom/pan; "pose unknown"
│                                │  badge when pose is null
├────────────────────────────────┤
│  you> search the room for…     │
│  [tool 1/8] observe()       ▸  │  chat scroll region
│  ┌──────────┐ possible bottle  │
│  │ photo    │ match ✓ unverif. │  look card: photo + verdicts + provenance
│  │          │ 1.06 m @ −107.6° │
│  └──────────┘                  │
│  robot> I found a possible…    │
├────────────────────────────────┤
│ [ instruction…            ][→] │  send; disabled + "tool 3/8" while running
└────────────────────────────────┘
```

Landscape/desktop: map left (fills height), chat right (420 px column), same
components. Dark theme first (a rover console at night), system font stack, one
accent color, no icon library, no framework — vanilla JS, one file each of
HTML/CSS/JS. Map rendering: draw PNG to canvas, overlay pose triangle from the
1 Hz tick; refetch PNG only when the state tick's map stamp changes.

## 8. Update rates + load budget, and how load gets MEASURED

Budgeted: state tick ~300 B/s per client; map PNG only on stamp change (rig:
once — the rig map is static; flight: SLAM republish cadence, PNG tens of KB);
transcript events only when a mission speaks; photos on demand. Backend ROS
cost: two topic subscriptions + TF + per-instruction service calls — the same
order as one more `rig_mission.py` watcher.

Measured, counter method, pre-registered before batch E runs: on the rig, count
`/scan` messages and `/coverage_explorer/status` messages over 60 s windows —
(a) zero clients, (b) two clients (phone + desktop) with a mission running —
plus the rvr diagnostics write-counter rate across the same windows. Bar:
per-window message counts within 5% of the zero-client baseline. Numbers go in
the batch receipt, not adjectives.

## 9. Honest limits, stated up front

* On the RIG the map does not grow — `sim_closed_loop` serves a recorded map via
  map_server, no SLAM. The UI is source-agnostic (`/map` is `/map`); the growing
  map IS the flight case via slam_toolbox on the same topic. The morning demo
  shows a live mission (pose moving, goals, chat) on a served map; a growing map
  in the browser waits for the first flight use, and this note says so rather
  than bolting SLAM onto the rig at night.
* Look cards on the rig depend on the recognition node + camera being up (rover
  is parked; camera-while-parked is legal). If not up, the availability preamble
  already tells the model, and the demo shows the honest refusal path instead.
* Transcript ring is per-process memory; restart loses chat history (the
  mission bags/transcripts on disk remain the record of record).
* One browser client sending an instruction locks out the other until it ends —
  deliberate, it mirrors "one instruction at a time".

## 10. Files + batches

```
src/sphero_rvr_core/web_console.py      # pure: event classify/types, ring buffer,
                                        #   grid→PNG, photo-name confinement
src/sphero_rvr_driver/web_console_node.py  # node + HTTP server + mission thread
src/sphero_rvr_driver/web_static/       # index.html, app.css, app.js
tests/test_web_console.py               # incl. the marker-drift pin test
setup.py                                # console script `web_console`, package_data
```

No launch file in v1 — it runs beside the stack like task_client, and gains a
launch entry only when a flight wants it (same doctrine as the CLI).

Batches: **A** this note (consensus) → **B** pure core + tests → **C** node +
endpoints, curl-certified against the rig (no frontend yet) → **D** frontend →
**E** rig end-to-end: mission in a browser + the §8 measurement. Each batch a
commit with SHA to the PM; max one unreviewed.
