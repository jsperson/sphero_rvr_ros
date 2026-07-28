# Milestone 7 Phase 3 attended safety and moving perception

## Scope and authority boundary

Phase 3 closes two sequential motor-capable entry gates:

1. M7.3 — attended collision slow/stop/manual-reset/no-contact validation.
2. M7.4 — attended live moving-perception validation.

The software slice begins from merged main
`1fc349305d3f809ea00896de81436c564a8c626d`. It adds a read-only observer,
graph audit, fail-closed session template, and ROS-free evaluator. It adds no
publisher, service, command source, serial transport, driver, motor-capable
launch, authority owner, or automatic activation.

The existing trusted path remains unchanged:

```text
reviewed route authority
  -> live_route_runner (sole /cmd_vel publisher)
  -> collision_stop_node (sole /cmd_vel_motor publisher)
  -> rvr_node (sole rover serial owner)
```

The observer subscribes to this graph after it has been separately approved and
started. Observing an active graph is not authority to start one.

M7.3 and M7.4 each require:

- independent review of the exact candidate SHA;
- source, deployed, and reviewed SHA equality;
- a separate explicit approval ID after the motor warning;
- an attended, level, bounded room with no stairs, ledges, or drop-offs;
- unchanged `0.10 m/s`, `0.4 rad/s`, and `0.50 s` maximum command lease;
- raw rosbag evidence, compact observation evidence, active graph audit, and
  generated final cleanup audit.

Approval for M7.3 does not approve M7.4. Neither approval grants M7.5 physical
hierarchical binding or the canonical mission. The passing M7.3-only report
emits an evidence SHA-256; the later M7.4 approval must bind that exact digest,
so M7.4 cannot be pre-approved independently of the reviewed collision result.
Every compact sample, veto, reset, restart, detection, and replan reference
must resolve to a checksum-verified event in the inline read-only observation.
The observation artifact records its canonical digest separately from its file
checksum. A reference set must also cover the required source roles: for
example, every moving sample binds collision state, requested and downstream
motor commands, odometry, scan, camera/lidar/localization/map status, and both
dynamic and static transforms.

## Non-executing preparation

On the reviewed exact source:

```bash
source /opt/ros/jazzy/setup.bash
source /home/jsperson/ros2_ws/install/setup.bash

ros2 run sphero_rvr_driver rvr_m7_attended_validation plan \
  --source-sha EXACT_CANDIDATE_SHA \
  --output /tmp/m7-phase3-plan.json

ros2 run sphero_rvr_driver rvr_m7_attended_validation template \
  --source-sha EXACT_CANDIDATE_SHA \
  --output /tmp/m7-phase3-session.json

ros2 run sphero_rvr_driver rvr_m7_attended_validation graph-audit \
  --source-sha EXACT_CANDIDATE_SHA \
  --source-repo /home/jsperson/ros2_ws/src/sphero_rvr_ros \
  --gate m7.3 --stage preflight \
  --output /tmp/m7-phase3-preflight.json
```

`plan` and `template` are ROS-free. `graph-audit` is read-only and gives each
direct ROS graph query a fixed three-second discovery window. The preflight
must show the driver, supervisor, route runner, motor publishers, and rover
serial owner absent. Stop here for independent review and explicit exact-SHA
M7.3 approval.

## WARNING: M7.3 can start the RVR motors

Do not perform this section until the user explicitly approves the reviewed
exact SHA after seeing this warning.

Use a broad rigid vertical obstacle centered ahead of the rover in a level
bounded room. Begin with more than `0.60 m` front clearance. Keep the operator
at the controls with STOP and ESTOP immediately available. Use only the
existing exact-SHA approval owner and supervised live-route path.

Record the active graph and all raw topics. In a separate terminal, the
read-only observer may be started:

```bash
ros2 run sphero_rvr_driver rvr_m7_attended_validation observe \
  --source-sha EXACT_CANDIDATE_SHA \
  --gate m7.3 --duration 120 \
  --output /tmp/m7-phase3-m7.3-observation.json

ros2 run sphero_rvr_driver rvr_m7_attended_validation graph-audit \
  --source-sha EXACT_CANDIDATE_SHA \
  --source-repo /home/jsperson/ros2_ws/src/sphero_rvr_ros \
  --gate m7.3 --stage active \
  --output /tmp/m7-phase3-m7.3-graph.json
```

The observer creates subscriptions only. It cannot publish a `Twist`, call a
service, open the rover serial device, approve a route, or start a launch.

M7.3 acceptance requires:

- physical `SLOW`: obstacle between `0.35 m` and `0.60 m`, requested forward
  motion remains nonzero, and motor output is reduced;
- physical `STOPPED`: after the supervisor first reports the obstacle at or
  inside `0.35 m`, motor output reaches zero within `0.30 s`, and no contact
  occurs;
- reset while still blocked is rejected;
- after clearance at or beyond `0.45 m` for at least `0.50 s`, manual reset is
  accepted but never replays the old command;
- stale command reaches motor zero within `0.35 s`, before the `0.50 s` driver
  watchdog, including while provider inference is held in flight;
- operator STOP and ESTOP each reach motor zero within `0.30 s`; ESTOP remains
  latched until explicit clear;
- collision, STOP, and ESTOP remain responsive while a provider call is
  deliberately held in flight;
- restarting an active mission produces `recovery_required`, motor zero, and
  never resumes the previous route;
- active graph audit proves one `/cmd_vel` publisher, one
  `/cmd_vel_motor` publisher/subscriber chain, exact publisher/subscriber node
  roles, the expected nodes, exact SHA, and rover serial ownership;
- the raw bag, observation, approvals, measurements, and final generated
  cleanup are checksum-bound.

After the trials, stop route authority, driver, supervisor, lidar, camera,
SLAM, rosbag, and observer. Generate cleanup with:

```bash
ros2 run sphero_rvr_driver rvr_m7_attended_validation cleanup-audit \
  --source-sha EXACT_CANDIDATE_SHA \
  --source-repo /home/jsperson/ros2_ws/src/sphero_rvr_ros \
  --output /tmp/m7-phase3-m7.3-cleanup.json
```

The evaluator recomputes every cleanup result from the retained Git, process,
ROS node/topic, publisher-count, and `fuser` command outputs; hand-entered
passing booleans are insufficient. It must prove all motion/sensor processes,
publishers, nodes, and device owners absent. Review M7.3 evidence before
requesting M7.4 approval:

```bash
ros2 run sphero_rvr_driver rvr_m7_attended_validation evaluate \
  /tmp/m7-phase3-session.json \
  --through m7.3 \
  --output /tmp/m7-phase3-m7.3-report.json
```

The M7.4 approval must bind the report's `m7_3_evidence_sha256`. It must also
bind the accepted M7.3 directional-veto addendum digest. The addendum must show
an attended, no-contact paired trial against one unchanged obstacle geometry:

- command toward the already-overlapping lidar return reaches zero downstream
  motor output;
- command away from that return reaches nonzero downstream motor output while
  the supervisor reports a positive moving-away point count;
- the command returns to motor zero after its bounded lease;
- the original M7.3 evidence digest remains unchanged.

The accepted digests are recorded in
`artifacts/m7_phase3_directional_addendum/README.md`. Neither the original M7.3
review nor a directional addendum alone grants M7.4 motion authority.

## Camera-pitch gate before M7.4

The M7.2 far floor-projection cell used `0.0424/0.0500 m`, about 85% of its
bound. Re-run the surveyed floor-contact pitch sweep before moving-perception
handling:

- use a target in the far `[0.85, 1.20] m` band;
- retain the unchanged `0.050 m` floor error bound;
- measured pitch must be within `0.5 degrees` of `-3 degrees`;
- checksum the sweep and floor-projection result.

If this check fails, stop. Do not widen the tolerance or start M7.4.

## WARNING: M7.4 can start the RVR motors

Do not perform this section until M7.3 is accepted and the user gives a second,
separate approval for the same reviewed exact SHA.

Start the same supervised graph with live lidar, SLAM, Camera 3, calibrated
transforms, and moving semantic perception. Use a bounded route through a
level, obstacle-controlled area. Record:

- at least five ordered compact samples and two samples during nonzero motor
  output;
- lidar age at most `0.30 s`, camera age at most `1.0 s`, localization age at
  most `0.30 s`, and semantic-map age at most `1.0 s` during motion;
- `map -> base_link`, `base_link -> camera_link`, and
  `base_link -> laser` availability;
- SLAM-authoritative valid localization moving by at least `0.05 m`;
- changing map revisions;
- at least one mapped point detection with method, uncertainty, source
  timestamps, calibration ID, map revision, and evidence IDs;
- `bearing_only` never containing a point;
- a newly stable track while moving and a geometry-free
  `new_stable_detection` replan-required event;
- stale lidar/localization/camera/map evidence reaching motor zero within
  `0.30 s`, including while a provider call is in flight.

After shutdown, repeat the far-band surveyed floor-contact pitch sweep. The
result must remain within `0.5 degrees` of `-3 degrees`, drift no more than
`0.5 degrees` from the pre-run check, and preserve the unchanged `0.050 m`
floor-projection bound.

Generate and checksum the final cleanup audit. The lidar must be powered down
when the goal finishes. Use the same `cleanup-audit` command with the M7.4
output path.

## Evaluation

The completed session is evaluated without ROS:

```bash
ros2 run sphero_rvr_driver rvr_m7_attended_validation evaluate \
  /tmp/m7-phase3-session.json \
  --through m7.4 \
  --output /tmp/m7-phase3-report.json
```

The evaluator recomputes the collision timing, slow scaling, reset behavior,
restart recovery, moving distance, new stable track, replan binding, freshness,
pitch drift, and point/bearing invariants. It also recomputes observation event
and payload digests, compact-event references, exact endpoint ownership, and
cleanup from raw command output. The manifest cannot change fixed thresholds
or approval limits. The M7.3-only evaluation requires only the M7.3 approval
and evidence. The complete evaluation additionally fails closed unless the
M7.4 approval binds the accepted M7.3 evidence digest and both active graph
audits, raw bags, and generated cleanup audits are present.

## Software verification

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_m7_attended_validation.py \
  tests/test_collision_stop.py \
  tests/test_collision_stop_contract.py \
  tests/test_stationary_perception.py \
  tests/test_camera_lidar_localization.py \
  tests/test_package_metadata.py
```

Passing software tests and a no-motion Pi preflight do not authorize M7.3 or
M7.4 physical motion.

## M7.3 physical result

The 2026-07-27 attended collision session passes on reviewed and deployed
source `8f020c84ffbbcd0f3eb7ad642e938794cfe0c39f`. That candidate is merged through
[PR 49](https://github.com/jsperson/sphero_rvr_ros/pull/49) at main
`54aebab9c60b6c6a205719552c4419c3f67aafa2`.

The evaluator accepted the exact trial set:

- real SLOW scaling at `0.5875 m` and `0.4360 m` front clearance;
- collision STOP at `0.3115 m`, downstream motor zero, and operator-confirmed
  no contact;
- rejected reset while blocked and accepted reset only after `0.78 s` clear,
  without replaying the old command;
- stale-command zero in `0.014600 s`, operator STOP zero in `0.015192 s`, and
  ESTOP zero in `0.003539 s`, each while a real authenticated provider call was
  in flight;
- ESTOP latch until explicit clear;
- crash/restart transition from `RUNNING` to `recovery_required`, motor zero,
  and no old-route resume;
- exact active publisher/subscriber ownership and final generated cleanup.

The accepted M7.3 evidence digest is
`7e2636f100ffad724477f1e6287458d0708057c3ee93f26d5dd6f52432281f55`.
The evaluator leaves M7.4 not proven and keeps M7.5 plus the canonical mission
unapproved. The complete committed evidence index is
[artifacts/m7_phase3_attended_collision/README.md](../artifacts/m7_phase3_attended_collision/README.md).
The roughly 203 MiB raw bag and 289 MiB read-only observation remain
checksum-bound on `sphero-pi-2`.

The camera, lidar, rosbag, observer, driver, supervisor, route runner, motion
publishers, and rover serial owner are absent after cleanup. The lidar is
powered down.

M7.4 required an independent review of this result and a separate approval
binding both the exact M7.3 evidence digest and the directional-addendum
digest. That condition was satisfied before any M7.4 motion.

## M7.4 physical result

The 2026-07-27/28 attended moving-perception session passes on the same
reviewed and deployed executable source
`8f020c84ffbbcd0f3eb7ad642e938794cfe0c39f`. Its separate approval binds:

- accepted M7.3 evidence
  `7e2636f100ffad724477f1e6287458d0708057c3ee93f26d5dd6f52432281f55`;
- accepted directional evidence
  `638abb8f293781adcf3827a486cf700b96693f1172d6fb058c40b79a8b8f4130`;
- unchanged `0.10 m/s`, `0.4 rad/s`, and `0.50 s` physical limits;
- an attended, bounded, level room without stairs, ledges, or drop-offs.

The evaluator accepted five ordered live samples, including three samples with
`0.05 m/s` downstream motor output and `0.051178355 m` maximum observed
displacement. Every sample retained fresh lidar, camera, localization, map, and
transform evidence. A floor-projected mapped detection retained its
calibration, source timestamps, mapped point, uncertainty, and evidence IDs.
Track `object-0007` was reconfirmed stable while motor output was nonzero and
caused a deterministic `new_stable_detection` replan event without model
geometry.

The fixed camera-pitch gate was not widened. Pre-run pitch was `-2.550°` at a
surveyed lidar-pivot range of `0.890 m`; post-run pitch was `-2.679°` at
`0.895 m`. Drift was `0.129°`, and the far floor-projection residuals were
`0.0047 m` and `0.0001 m` against the unchanged `0.050 m` bound.

The independent stale-sensor trial stopped lidar while the rover and a real
provider call were active. Source shutdown to first downstream zero took
`0.329830 s`, including the fixed `0.300 s` scan-age interval. The observed
stale-state evidence and first downstream zero were separated by `0.002553 s`,
which is the evaluator's response metric. No contact occurred.

Final generated cleanup found no camera, lidar, rosbag, driver, collision
supervisor, route runner, serial owner, or motion-topic publisher. The chassis
was confirmed off and the lidar was powered down. The complete committed index
and capture disclosures are in
[artifacts/m7_phase3_moving_perception/README.md](../artifacts/m7_phase3_moving_perception/README.md).
The complete raw session remains checksum-bound on `sphero-pi-2`.

The evaluator leaves M7.5, the canonical mission, and drop-off detection
unapproved. M7.4 does not grant physical hierarchical binding or unattended
motion authority.
