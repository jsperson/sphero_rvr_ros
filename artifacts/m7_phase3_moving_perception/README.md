# M7 Phase 3 moving-perception evidence

## Result

M7.4 passes on the 2026-07-27/28 attended physical session at executable
source `8f020c84ffbbcd0f3eb7ad642e938794cfe0c39f`. The approval is separate from
M7.3 and binds:

- accepted M7.3 evidence
  `7e2636f100ffad724477f1e6287458d0708057c3ee93f26d5dd6f52432281f55`;
- accepted directional addendum
  `638abb8f293781adcf3827a486cf700b96693f1172d6fb058c40b79a8b8f4130`;
- the same reviewed/deployed executable source;
- the fixed `0.10 m/s`, `0.4 rad/s`, and `0.50 s` physical limits;
- an attended level, bounded room without stairs, ledges, or drop-offs.

The generated report passes every M7.4 check while keeping M7.5, the canonical
mission, and drop-off detection explicitly unavailable.

## Accepted measurements

- Five ordered perception samples include three samples with downstream motor
  output at `0.05 m/s`.
- Live localization measured `0.051178355 m` maximum displacement during the
  recorder-backed route.
- Lidar, camera, localization, map, and required transforms were fresh at
  every accepted sample. The tightest localization age was `0.297610 s`
  against the unchanged `0.300 s` limit.
- A floor-projected mapped camera detection retained its calibration ID,
  evidence IDs, source timestamps, mapped point, and uncertainty.
- Track `object-0007` had one observation at compact map revision 130 and two
  observations at revision 132 while downstream motor output was nonzero,
  producing a server-side `new_stable_detection` replan event without model
  geometry.
- The pre-run camera-pitch check measured `-2.550°` at a surveyed lidar-pivot
  target range of `0.890 m`; the post-run check measured `-2.679°` at
  `0.895 m`. Drift was `0.129°`. Far floor-projection residuals were
  `0.0047 m` and `0.0001 m`, both inside the unchanged `0.050 m` bound.
- The stale-sensor trial stopped the lidar while motion and a real provider
  call were active. Source shutdown to first downstream zero was
  `0.329830 s`, including the fixed `0.300 s` scan-age interval. The observed
  stale-state evidence and first downstream zero were separated by
  `0.002553 s`, which is the evaluator's fixed-bound response metric.
- No physical contact was reported.
- Final generated cleanup found no driver, supervisor, route runner, camera,
  lidar, rosbag, serial owner, or motion-topic publisher. The chassis was
  subsequently confirmed off. This process/device audit did not prove physical
  lidar-motor de-energization; the operator later reported that the scanner was
  still spinning. The post-handoff correction invoked the upstream
  `/stop_motor` service, gracefully reaped the temporary driver, asserted the
  serial DTR stop state, and reverified zero lidar/camera/rosbag processes and
  zero lidar device owners. The operator then visually confirmed that the lidar
  stopped spinning.

## Evidence layout

- `approval.json` — exact-source, digest-bound M7.4 approval.
- `active_graph.json` — active endpoint ownership during the accepted run.
- `compact_evidence.json` — checksummed physical measurements and raw-artifact
  references used to build the session.
- `relevant_events.jsonl` — lossless 62-event index preserving raw payload
  hashes and evidence IDs.
- `report.json` — fail-closed evaluator result through M7.4.
- `evaluate.log` — evaluator output plus `/usr/bin/time -v` resource data.
- `final_cleanup_audit.json` — generated post-run process, ROS graph, publisher,
  and device-owner audit.
- `post_handoff_sensor_shutdown.json` — correction after the operator observed
  that ownerless cleanup had not physically stopped the lidar motor, plus the
  camera/storage growth audit.
- `raw_artifact_sha256.txt` — 445-entry inventory for the full raw Pi session,
  including excluded attempts and images.
- `committed_artifact_sha256.txt` — hashes of the compact files committed here.

The complete raw session remains on `sphero-pi-2` at
`/home/jsperson/rvr_runs/m7-phase3-m7.4-20260727T2330Z`. It includes the
`164206396`-byte combined session, the `13993662`-byte MCAP, full observations,
pitch snapshots/images, all trial logs, and the cleanup audit. The compact
index binds these files without committing large binary data.

## Disclosed without concealment

1. The first successful route moved `0.09613 m` but its observer started before
   rosbag capture, so it is not the accepted moving-perception artifact. Run 12
   repeated the gate with the MCAP active and moved `0.05118 m`.
2. Runs 2–10 were invalid capture attempts caused by buffered triggers,
   provider-call timing, ROS graph discovery overhead, and low-speed
   ridge/stall behavior. They failed closed or were cleaned up and are retained
   in the raw checksum inventory; none contributes an accepted sample.
3. Raw revision 131 crossed `object-0007` from one to two observations about
   `0.17 s` before the first nonzero motor sample. Revision 132 reconfirmed the
   stable track while the motor output was nonzero. The accepted claim is that
   the newly stable evidence was live and present during motion and triggered
   deterministic replanning, not that the exact threshold-crossing instant
   occurred after wheel motion began.
4. The stale trial's `0.329830 s` source-shutdown-to-zero duration is not
   represented as supervisor reaction time; it includes the configured
   `0.300 s` freshness interval. The evaluator separately records the
   `0.002553 s` stale-evidence-to-zero observation.
5. The operator later identified that the handheld centering laser was not
   centered in its own case. Surveyed forward distances remain valid and the
   pitch calculation uses the vertical target plane/floor-contact row rather
   than horizontal page centering. Horizontal centering is therefore not
   claimed as surveyed evidence.
6. A shell brace-expansion mistake invoked the final read-only cleanup audit
   three times. Every invocation reported the same quiescent safety state; the
   final generated audit is retained.
7. The initial handoff incorrectly equated an ownerless `/dev/rplidar` with a
   physically stopped scanner. The operator reported continued spin. A
   temporary upstream driver was started, `/stop_motor` succeeded, the driver
   was gracefully reaped, and DTR stop was asserted. Machine checks then found
   zero lidar processes and owners, and the operator visually confirmed the
   lidar stopped. Physical spin remains an operator-visible property rather
   than something the cleanup audit can infer.
8. The camera/rosbag audit found zero capture writers. Both `rvr_runs` and ROS
   log storage grew by zero bytes over an 8-second check; the filesystem was
   53% used with about 27 GB free.

## Scope boundary

This evidence proves the M7.4 attended moving-perception gate only. It does not
approve the M7.5 physical hierarchical binding, a canonical physical mission,
unattended driving, or operation near drop-offs.
