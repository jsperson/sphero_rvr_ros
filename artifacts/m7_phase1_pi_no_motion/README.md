# Milestone 7 Phase 1 Pi no-motion evidence

These machine-readable reports were collected on `sphero-pi-2` from the
immutable executable source SHA
`9822ec6fe8c903191329ebdbb2646cac745e25ad`. The rover remained powered off.
No RVR driver, serial transport, live sensor, physical executor, or motion
authority was started.

- `wfd.json` is a 50-pass run of the project-owned deterministic WFD against
  the recorded Phase 1 SLAM map. It records the input checksums, map revision,
  frontier signatures, per-pass timings, and maximum process RSS.
- `graph.json` is a five-second observation of the replay-only ROS domain. It
  records active lifecycle states, action readiness, topic endpoints, motor
  samples, prohibited-process inspection, and serial-owner inspection.
- `environment.json` records the source archive, Pi identity, installed Nav2
  package versions, bounded launch result, and cleanup.

The repeated `/behavior_server` entries in `private_publishers` are separate
Nav2 behavior publisher endpoints. The allowed private-topic node set is
exactly `behavior_server` and `controller_server`; neither publishes directly
to `/cmd_vel` or `/cmd_vel_motor`.

The graph launch was deliberately bounded to 45 seconds with GNU `timeout`.
Exit `124` therefore denotes expiry of that observation window, not a pytest
result or a failed safety gate. The audit process exited successfully before
the outer bound, all launch descendants were reaped, and the serial devices
were rechecked with no owners.

These reports establish only M7.1. They do not authorize stationary live
sensing, a physical adapter, the RVR driver, serial transport, or motor output.
