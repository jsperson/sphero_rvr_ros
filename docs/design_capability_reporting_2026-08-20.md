# Design note: task_node capability reporting — killing the discovery tax at the root

Status: RATIFIED (PM consensus, overnight 2026-08-20, ladder rung 2) with one
pin: the reply carries its own `stamp` — the node's clock at assembly — so a
consumer holding a stale snapshot can DETECT it. Consumed-in-seconds is the
design intent; the stamp makes violating it visible rather than trusted.

## The gap, as certified

The availability preamble (search round 2 §5) probes which of task_node's
interfaces EXIST and names the missing ones once per instruction, killing the
discovery tax for absent services. Its honest limit was stated the night it
landed: **task_node's services always exist — a tool whose BACKEND is absent
(semantic_map down, recognition node not launched, spin gateway missing) still
answers, per call, with ok=false.** The model pays one failed call per absent
backend per instruction to learn the same fact every time. Flight 5's leanest
run still carried that structural tax; the rig demo pays it on every look.

## The principle applied

Assert, don't infer, at seams: the ONLY component that knows whether a backend
is reachable is task_node itself — it holds the clients. Today each handler
discovers absence at call time and refuses honestly. Capability reporting is
the same knowledge, volunteered up front, by the owner.

## Built-in-first: what ROS already offers, and why none of it is this

Surveyed before inventing a field (the standing design rule):

* **Graph introspection** (`wait_for_service`, `server_is_ready`) — already
  used; it is exactly the exists-level the preamble has. It cannot see through
  task_node to the backends. Not sufficient.
* **Lifecycle nodes** — nav2's own pattern, and the rig's lifecycle_manager
  uses it. Node-granular, not tool-granular: `goto` readiness aggregates the
  nav action server AND a TF pose; `status` readiness is a freshness rule, not
  a node state. Making semantic_map/recognition lifecycle-managed is a real
  architectural change serving a different problem (orchestrated bringup), and
  still would not answer "which TOOLS work". Declined for this.
* **bond** (nav2's liveness heartbeat) — node-granular again, adds wiring per
  backend, and answers "is it alive", not "can this tool answer now". Declined.
* **/diagnostics** (`diagnostic_updater`) — the idiomatic self-reported-health
  channel, and rvr_node already publishes there. Right SHAPE of idea (the owner
  volunteers its health) but a periodic level topic: the preamble needs an
  on-demand snapshot at instruction start, and a 1 Hz level stream is the exact
  sampling pattern counters-not-levels warns about. A future passive mirror of
  the same predicates into /diagnostics is compatible and out of scope here.

So the idiom worth keeping is diagnostics' PRINCIPLE (owner reports health) on
task_node's own doctrine (everything is a Trigger with tool_result JSON) — the
tool surface is ours, so its readiness report lives on it.

## The interface (additive, Trigger-only doctrine kept)

One new service: `task/capabilities` (`std_srvs/Trigger`), answering the
standard `tool_result` JSON:

```json
{"ok": true, "tool": "capabilities", "stamp": "2026-08-20T22:41:03",
 "tools": {
   "goto":               {"ready": true},
   "turn":               {"ready": true},
   "observe":            {"ready": false, "why": "observe service absent (semantic_map not running?)"},
   "look_and_recognize": {"ready": false, "why": "recognition node not running"},
   "query_semantic_map": {"ready": false, "why": "no /semantic_map/objects publisher"},
   "explore":            {"ready": true},
   "stop":               {"ready": true},
   "status":             {"ready": false, "why": "mission status stale 41.2s (limit 3s)"},
   "where_am_i":         {"ready": false, "why": "no map->base_link transform"}
 }}
```

Per-tool readiness is determined by the SAME checks the handlers already make
at call time — client `service_is_ready()` / `server_is_ready()`, the status
age rule, the TF lookup — factored so the handler and the report cannot drift
apart (one predicate per tool, called by both). No new inference anywhere: the
report is assembled from the node's own clients at answer time.

**What this is NOT:** not a promise. A backend can die between the preamble
and the call; the per-call ok=false refusal remains the authority, unchanged,
fail-closed. Capability reporting reduces the tax; it never replaces refusals.
(Counters-not-levels noted: this is a level, snapshotted at instruction start,
consumed within seconds, and every consumer also gets the per-call truth — the
sampling-race argument that killed level-publishing at 1 Hz does not apply to
an on-demand snapshot backed by call-time refusals.)

## Client side (the payoff)

`ToolRunner.probe_availability` upgrades: after the existing exists-probe, call
`task/capabilities` when present and MERGE — a tool is announced unavailable if
it is missing OR reports ready=false, with the why carried into the preamble:

> Unavailable right now: observe (semantic_map not running), look_and_recognize
> (recognition node not running). Do not call them; report honestly instead.

`availability_note` already renders unavailable tools and the no-retry rule;
it gains the reasons. A task_node without the new service (older deploy)
degrades to today's behavior exactly — the merge treats "no capabilities
service" as "no backend information", not as "nothing works".

## Tests + certification

* Pure: the merge logic and note rendering (existing pattern in
  test_task_agent's preamble tests); the predicate table — one test per tool
  naming its predicate, so a new tool without a predicate fails loudly.
* Node-level, on the rig (chassis-free): rig has no camera/semantic stack —
  `task/capabilities` MUST report observe/look_and_recognize/query not-ready
  with reasons, goto/turn/explore/stop ready. Then kill task_node's view of one
  backend live (stop the spin gateway) and re-ask: turn flips to not-ready with
  the reason. The falsifier is the rig's own asymmetry.
* The bench bar: one full chat instruction on the rig that would previously
  have burned a failed look call now completes with zero ok=false tool results
  (the preamble carried the fact). Receipt: the transcript.

## Scope fence

No handler behavior changes. No new message types, no parameters, no topics.
One service in task_node, one predicate refactor per tool (call-site identical),
one client merge, tests. The web console needs NOTHING: it inherits through
ToolRunner, and its availability preamble line renders in-chat as it already
does.
