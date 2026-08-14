# Track 2 v2 — `explore`, `stop`, `status`: the rover takes orders in English

**Short by intent.** v1 (`goto`, `observe`, `query_semantic_map`) settled the hard
questions — where the safety boundary sits, who clamps whom, what a tool result looks
like. This note only covers what v2 adds, and the three tools it adds are thin wrappers
over machinery that already flies.

The measurable: **"explore the room" / "stop" / "status" work end-to-end through the LLM
path against the rehearsal harness.**

---

## 1. What is actually missing, and it is one thing

`goto` drives to a point. Nothing in the tool surface can start, end or ask about a
MISSION — and a mission is what the rover is for. The coverage explorer already has the
machinery and already exposes it as plain ROS services:

    /coverage_explorer/mission/start   Trigger   arms and begins
    /coverage_explorer/mission/stop    Trigger   disarms and drops the goal in flight

So `explore` and `stop` are forwarding, not invention, and they inherit D29's
gates-then-go discipline for free.

`status` is the one that needs something built. **The explorer publishes no live
status**: `~/report` is latched and only appears when a mission ENDS, and
`~/goal_generation` is an integer for a different purpose. So today the honest answer to
"what are you doing?" mid-mission is unavailable — and unavailable is exactly what a
status tool must never quietly render as "fine".

## 2. The explorer gains `~/status`, because the owner publishes the fact

One small publisher on the explorer, 1 Hz, JSON string: armed, done, outcome-so-far,
goals sent/succeeded/aborted (with the recovery split), cells covered, candidates
remaining, and whether a goal is in flight.

This is the standing rule rather than a new idea: **the component that knows the fact
publishes it.** The alternative — a status tool that infers mission state from goal
generations, topic liveness or timing — is the proxy-inference class that has produced
three defects in this project already. The explorer knows whether it is armed. It says
so.

`status` is then a read of that topic plus the supervisor's `/collision_stop/state`,
which is already live and already the authority on whether motion is being permitted.

**Staleness is reported, never smoothed.** If the status topic is older than a small
bound the tool says so and says how old, because "no status for 40 s" is the single most
informative thing a stuck rover can tell you, and a tool that hides it in favour of the
last good value is worse than no tool.

## 3. Interfaces

| tool | interface | type | forwards to |
|---|---|---|---|
| `explore` | `task/explore` | `Trigger` | `/coverage_explorer/mission/start` |
| `stop` | `task/stop` | `Trigger` | `/coverage_explorer/mission/stop` |
| `status` | `task/status` | `Trigger` | reads `~/status` + `/collision_stop/state` |

All three are `Trigger` because none needs an argument. The v1 note's reasoning about
custom `.srv` packages applies unchanged: a bespoke interface package is the honest v2
upgrade for tools that need typed arguments, and none of these do.

The safety boundary is unchanged and structural: this node still imports no
`geometry_msgs`, holds no `Twist`, publishes no velocity and names no `/cmd_vel*` topic.
`explore` and `stop` are service calls to another node; motion remains Nav2's, and
beneath it the collision supervisor remains the sole `/cmd_vel_motor` publisher.

## 4. `stop` MEANS MISSION-STOP, AND THE TOOL SAYS SO

This is the one place a plain-English word maps onto something narrower than a person
would assume, so it is written into the tool's own result rather than left to a doc.

`stop` disarms the mission and cancels the goal in flight. **It is not an emergency
stop.** The rover coasts to a halt through the normal command path; the collision
supervisor owns real stopping and this does not touch it. A model — or a person — who
says "stop" during an emergency must not believe they have hit a brake.

So the tool's success message states what it did in those terms, and the system prompt
tells the model to say it back to the user rather than reporting a bare "stopped". If a
hard stop is ever wanted from this surface it is a DIFFERENT tool wired to the
supervisor's own e-stop, and it is not in this batch.

## 5. What the model is told

Three tools appended to the existing prompt, with the same rules (one call per reply,
read `ok=false` and correct, never invent a tool). The additions worth their words:

* `explore()` — start a coverage mission of the whole room. Long-running: it returns as
  soon as the mission STARTS, not when it finishes. Use `status()` to follow it.
* `stop()` — end the mission and cancel the current goal. Not an emergency stop.
* `status()` — what the robot is doing right now.

**`explore` returning immediately is the subtlety most likely to produce a wrong-looking
demo**, because a model that expects a blocking call will report "I have explored the
room" the instant it returns. The prompt says so explicitly and `status` exists to be
the follow-up.

## 6. Acceptance

1. `ros2 service call /task/explore std_srvs/srv/Trigger` starts a mission with no LLM
   in the loop — the Stage D rule, unchanged: **delete the client and the robot still
   works.**
2. The three tools round-trip through `task_client` against the rehearsal harness:
   "explore the room" starts one, "status" reports it running, "stop" ends it.
3. A status read with a stale or absent topic reports UNAVAILABLE with the age, and
   never the last good value dressed as current.

## 7. Not in this batch

* An e-stop tool. Different authority, different failure consequences, its own review.
* Typed arguments (`explore(area)`, `status(verbosity)`) — no caller needs them yet.
* Native tool-calling APIs. Same door as v1: when the prompt-parsing loop actually
  costs us something.
