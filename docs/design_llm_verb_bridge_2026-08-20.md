# Design note: the LLM-verb bridge — extend the incumbent, adopt no framework

*Scott's direction (2026-08-20): "My hope is to use the LLM to assist with
this." Architecture ratified in principle: the LLM is the mission brain
composing certified verbs as TOOLS; it never enters the safety loop — it says
where, the supervisor/brake/watcher say whether. Read first, this note second,
consensus before any build.*

## The read's headline: the bridge already exists in this repo

Track 2 (Stage D + v2) built Scott's ratified architecture before it had
today's name, and it is flying-adjacent already:

- **task_node** — the tool surface as plain ROS interfaces: `task/goto`
  (NavigateToPose action with the trinity envelope), `task/explore`,
  `task/stop`, `task/status`, `task/observe`, `task/query_semantic_map`.
  The envelope and, beneath it, the collision supervisor are the safety
  boundary; the node is authoritative over any client.
- **task_agent (core, pure)** — the fail-closed contract: exactly-one-tool-or-
  say replies, schema-validated args, unknown-anything refused, ONE reprompt
  then the instruction ends, budgeted. Unit-tested against canned transcripts
  including hallucinated tools.
- **task_client** — the REPL loop on the project's existing provider, with the
  acceptance test this project should frame: **deleting the client changes
  nothing about the robot.** The model is outside all three layers.

That is the architecture Scott ratified, already reviewed, already tested. The
bridge question is therefore a GAP list, not a platform choice.

## The external candidates (keep-don't-rebuild shortlist), read 2026-08-20

- **ros-mcp-server**: exposes the WHOLE ROS graph to LLMs via rosbridge —
  topics, services, actions, parameters, auto-discovered; permissions are a
  *planned* feature. That generality is precisely what this project must not
  grant: an LLM that can publish `/cmd_vel_motor` has crossed every layer we
  built. Scoping it down would mean maintaining a deny-the-world config whose
  one mistake is a motor topic. **Verdict: not as-is.** Its useful idea — MCP
  as a transport so richer frontends (Claude-class) can drive — survives as a
  possible LATER wrapper around task_node's curated surface only.
- **RAI (RobotecAI)**: a full embodied-agent framework (multi-agent, ASR/TTS,
  perception, mission loops), mid-maturity. Adopting it re-homes the mission
  brain into someone else's loop — our fail-closed contract, budget,
  one-reprompt rule, and delete-changes-nothing property would all need
  re-proving inside a moving framework. **Verdict: not now.** Re-read if our
  needs outgrow the single-agent loop.

Built-in-first applies with the incumbent being OUR OWN certified layer:
config/extension of Track 2 first, external platforms only for measured gaps —
and the read found no gap those platforms fill that our layer cannot.

## The actual gaps (the bridge's build list, smallest first)

1. **Tool surface additions** — new schemas in task_agent + thin forwards in
   task_node, each inheriting the existing refusal machinery:
   - `look_and_recognize(target)` → the recognition node's service (GATED on
     the bench card passing; the tool lands disabled until then, the watcher-
     flip pattern).
   - `turn(degrees)` → the precise-turn gateway (already an action server with
     admission; the tool is a client, admission still refuses inadmissible
     turns — the LLM gets the refusal text like every other envelope answer).
   - `where_am_i()` → map pose + heading + a coarse map summary (the owner
     facts already on TF and /map; read-only).
2. **Multi-step missions on the existing loop** — the loop already iterates
   tool→result→tool; "search the room for X" becomes a PROMPT + the tools
   above, not choreography code. The recognition design note's composition
   paragraph becomes a system-prompt stanza. Bench-testable against canned
   transcripts first (the task_agent pattern), then rehearsal harness.
3. **Model upgrade behind the SAME contract** (the client's own named v2):
   native tool-calling APIs instead of JSON-in-prose, same schemas, same
   refusals. Provider stays the project's existing one until a consensus says
   otherwise; an MCP wrapper for external frontends is a separate, later
   decision with its own note.

## What does NOT change

The safety loop (supervisor, brakes, watcher, envelopes) — untouched, by
ratification. task_node stays authoritative; the trinity gate still refuses
unmapped/lethal goals no matter how eloquent the requester. The
delete-the-client test stays the acceptance bar for every bridge layer added.

## Proposed order (on consensus)

(1) `where_am_i` + `turn` tools (small, no gates needed beyond existing
admission); (2) `look_and_recognize` tool, landing disabled behind the bench
card; (3) the search prompt + canned-transcript tests; (4) model upgrade
round. Each its own reviewed batch; nothing starts before the consensus round
on this note.
