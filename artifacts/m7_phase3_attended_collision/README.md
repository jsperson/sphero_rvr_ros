# M7.3 attended collision evidence

## Verdict

PASS on reviewed and deployed source
`8f020c84ffbbcd0f3eb7ad642e938794cfe0c39f`.

The ROS-free evaluator accepted the attended M7.3 collision session and emitted:

- session SHA-256
  `e0a0fd38476ada5790ba129900793b8f9a8bd9e2b0040e8a4dbca687c6791f70`;
- M7.3 evidence SHA-256
  `7e2636f100ffad724477f1e6287458d0708057c3ee93f26d5dd6f52432281f55`;
- `m7_3_collision_gate=passed`;
- `m7_4_moving_perception_gate=not_proven`;
- `m7_5_physical_binding_approved=false`;
- `canonical_mission_approved=false`.

The user was physically present and explicitly confirmed that the moving target
did not touch the rover during the collision-stop trial.

## Physical result

| Trial | Result |
| --- | --- |
| SLOW scaling | At `0.5875 m` front clearance, requested `0.1000 m/s` and downstream motor output was `0.0950 m/s`; at `0.4360 m`, output reduced to `0.0344 m/s`. |
| Collision STOP | At `0.3115 m`, supervisor state became `STOPPED` and downstream motor output was zero. The evaluator's paired stopped/zero sample gives `0.0 s`; the independent live monitor also observed the transition within the same millisecond-scale interval. |
| Blocked reset | Rejected with `reset_required` while front clearance was about `0.1500 m`. |
| Clear reset | Accepted only after clearance remained beyond the `0.45 m` release threshold for `0.78 s`; repeated downstream samples remained zero, so no old command replayed. |
| Stale command | Downstream motor zero in `0.014600 s`, before the `0.50 s` driver watchdog, while authenticated provider inference was active. |
| Operator STOP | Downstream motor zero in `0.015192 s` while authenticated provider inference was active. |
| Operator ESTOP | Downstream motor zero in `0.003539 s`; reset was rejected while latched, and explicit clear did not replay motion. |
| Restart recovery | A controlled kill of the validated active mission-service PID caused systemd restart into `recovery_required`; motor remained zero and the old route did not resume. |
| Contact | Operator-confirmed no contact. |

Authenticated `gpt-5.6-luna` calls completed successfully during the independent
collision, stale-command/STOP, and ESTOP trials in `6.291`, `6.213`, and
`6.765 s` respectively. Safety actions completed independently while those
calls were in flight.

## Authority and graph

The real authenticated one-click approval bound proposal digest
`18d1ac73cc8573d6f9f60b43e89b21a862beb4c0ca981c1193074f1e8091a68b`
to the exact source/deployed/reviewed SHA. The accepted active graph audit proves:

```text
live_route_runner
  -> /cmd_vel
  -> lidar_collision_stop_supervisor
  -> /cmd_vel_motor
  -> sphero_rvr_driver
  -> rover serial
```

There was exactly one publisher on each command topic, the expected subscriber
at each boundary, no private Nav2 publisher before M7.5, and one rover serial
owner during the attended graph.

## Evidence inventory

- `report.json` is the passing evaluator output.
- `active_graph.json` is the passing exact-SHA active ownership audit.
- `final_cleanup_audit.json` is the generated, passing post-run cleanup audit.
- `relevant_events.jsonl` contains the selected observation events referenced
  by the compact trials, with a recomputed canonical payload SHA-256 on each
  derived review entry.
- `collision_stop_monitor.jsonl`, `stale_stop_monitor.jsonl`, and
  `operator_estop_monitor.jsonl` retain the independent timing streams.
- `restart_pre.json` and `restart_post.json` retain compact, image-free extracts
  of the active and `recovery_required` service states around the crash/restart
  trial; each extract binds its full source-state SHA-256.
- `provider_collision.log`, `provider_stale_stop.log`, and
  `provider_estop.log` retain the real provider timings.
- `evaluate.log` retains the evaluator output and resource measurement.
- `raw_artifact_sha256.txt` binds the source artifacts copied into this review
  packet plus the uncommitted full session, observation, and raw MCAP.
- `committed_artifact_sha256.txt` binds the final committed review copies,
  including the payload-hash-enriched relevant-event index.

The full `302,684,692`-byte session, `302,661,678`-byte read-only observation,
and `212,812,342`-byte raw MCAP remain on `sphero-pi-2` below:

```text
/home/jsperson/rvr_runs/m7-phase3-m7.3-stop-reset-20260727T1712Z/
```

They are checksum-bound rather than committed to Git.

## Disclosed capture anomalies

- The first observer invocations used an invalid requested duration of
  `900 s`; the tool correctly rejected values above its fixed `300 s` maximum.
  The raw bag was then replayed at `10x` with the hardware, driver, supervisor,
  sensor processes, and serial owner absent. The read-only observer captured
  `115,472` events over `100.012 s`.
- The same-run graph audit was first taken before discovery converged and was
  later contaminated by the bag recorder's read-only subscription. The session
  therefore uses a passing active graph audit from the immediately preceding
  physical attempt at the identical exact SHA and ownership configuration.
- A graceful mission-service restart produced `CANCELLED`, so it was not counted
  as restart-recovery evidence. The accepted trial killed only the validated
  active mission-service main PID; systemd restarted it into
  `recovery_required`.
- The first cleanup command self-matched its own diagnostic command line. The
  isolated generated rerun passed. A later stdin-based process inspection found
  zero prohibited processes.

These are evidence-collection disclosures, not passing measurements.

## Verification and cleanup

Before physical execution at the exact source:

- Mac focused: `85/85` passed in `0.43 s`.
- Mac full: `1069/1069` passed in `38.70 s`.
- Pi focused: `85/85` passed in `0.64 s`.
- Pi package-select build passed.

The physical evaluator command was:

```bash
ros2 run sphero_rvr_driver rvr_m7_attended_validation evaluate \
  /home/jsperson/rvr_runs/m7-phase3-m7.3-stop-reset-20260727T1712Z/m7.3-session.json \
  --through m7.3 \
  --output /home/jsperson/rvr_runs/m7-phase3-m7.3-stop-reset-20260727T1712Z/m7.3-report.json
```

It exited `0` after `21.01 s` with maximum RSS `1,049,940 KiB`.

Final cleanup proves the camera, lidar, rosbag, observer, driver, supervisor,
route runner, motion publishers, rover serial owner, and lidar device owner
absent. The hardware unit and lidar unit are inactive, and the lidar is powered
down.

## Scope

This evidence closes only M7.3. M7.4 remains locked until an independent review
accepts this evidence and the directional-veto addendum, and the user gives a
separate approval binding both the exact M7.3 evidence digest
`7e2636f100ffad724477f1e6287458d0708057c3ee93f26d5dd6f52432281f55`
and addendum digest
`638abb8f293781adcf3827a486cf700b96693f1172d6fb058c40b79a8b8f4130`.

Before M7.4, re-verify the measured `-3 degree` camera pitch and far-band floor
projection. Repeat the check after motion. Do not widen the `0.050 m` bound.
M7.5 physical hierarchical binding, the canonical mission, drop-off operation,
and unattended motion remain out of scope.
