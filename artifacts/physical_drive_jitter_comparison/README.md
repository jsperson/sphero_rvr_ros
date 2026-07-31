# Physical drive jitter comparison

This is the compact evidence handoff for the physical drive-quality goal that
followed Milestone 7. It does not claim useful room traversal. It records the
diagnosed oscillation, the exact merged/deployed correction, synchronized
command evidence, the short physical comparison, operator observation, and
cleanup.

## Outcome

[PR #58](https://github.com/jsperson/sphero_rvr_ros/pull/58) merged and was
deployed at exact SHA `c8cbff35d156332806f0fe8d16b47b23514eac6d`.
The physical comparison mission
`m7-canonical-e90f7828e13843d981eab942b16751a4` finished normally after
35.523 seconds. The private Nav2 request changed angular sign once; neither
the supervised `/cmd_vel` request nor actual `/cmd_vel_motor` output changed
angular sign. The operator reported no contact and less jitter, with no
appreciable forward movement.

The result is deliberately narrow: the rapid left/right motor oscillation was
materially reduced, but effective forward driving remains work for the next
milestone.

## Preserved invariants

- maximum linear command: `0.10 m/s`;
- maximum angular command: `0.4 rad/s`;
- collision STOP/ESTOP, scan freshness, exact-SHA authority, mission lease,
  and command lease unchanged;
- `live_route_runner` remains the sole hierarchical `/cmd_vel` publisher;
- the collision supervisor remains the sole `/cmd_vel_motor` publisher.

## Durable evidence

The compact values and operator observation are in [report.json](report.json).
The full private trace remains on `sphero-pi-2` at the path and SHA-256 recorded
there. It is intentionally not committed because the bounded trace contains
high-rate runtime state and is 2.79 MB; its deterministic terminal summary is
captured in the report.

Browser recording:
`/Users/jsperson/.config/browser-harness/agent-workspace/recordings/m7-jitter-fix-short-comparison`.

Final cleanup verified the physical and telemetry units inactive, no driver or
sensor processes, no UART or lidar-device owner, downstream zero commands, and
the lidar `Stop motor` log entry. Camera evidence remains capped at 96 JPEGs
per evidence directory; the Pi retained about 27 GB free.
