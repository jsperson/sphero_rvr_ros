# Persistent mission service

`rvr_mission_service` is the single local owner for durable Mission API execution
state. It retains session latches, cumulative runtime state, mission progress,
source/deployed SHAs, and an append-only SQLite event stream behind a user-only
Unix-domain socket.

This bounded slice provides the service lifecycle and state store only. Executor
health/capability binding, MCP/planner integration, and the user-facing client/CLI
are intentionally deferred to their successor tasks. The daemon currently starts
in replay mode; live authority is not exposed by this entry point.

## Start once

```bash
rvr_mission_service \
  --socket ~/.local/state/sphero-rvr/mission.sock \
  --database ~/.local/state/sphero-rvr/missions.sqlite3
```

The equivalent ROS launch is:

```bash
ros2 launch sphero_rvr_driver mission_service.launch.py
```

The owner lock is acquired before the database is opened or recovery runs. The
socket is mode `0600`, accepts one bounded JSON request per connection, and has no
TCP listener. Starting the daemon never starts or resumes a mission.

## Restart and recovery

On startup, every nonterminal persisted mission becomes `recovery_required`; its
session cancel latch is set and `auto_resume` remains false. Proposal, approval,
invocation, and the `running` transition are committed before runtime execution.
Any exception after that transition is treated as an unproven-quiescence failure:
the mission becomes `recovery_required` and the session is latched.

The event table rejects UPDATE and DELETE. Its records retain proposal, approval,
invocation, observation, artifact, terminal reason, source SHA, and deployed SHA
evidence needed to reconstruct execution.
