# Phase 2 camera-to-map localization evidence

`recorded_calibration_fixture.json` is the ROS-free evidence fixture for
Milestone 6 Phase 2. It contains:

- the measured Pi Camera 3 intrinsics and camera/lidar mounts;
- one real paired camera/lidar observation from the primary recorded bag;
- the real scan returns around a broad target placed approximately 18 inches
  (`0.4572 m`) ahead of the rover;
- a calibrated analytic floor-contact case; and
- a deliberately ambiguous two-cluster association case.

The recorded pair's source timestamps differ by `35.038348 ms`, inside the
`100 ms` gate. With the 2026-07-27 tread-contact translation revision, the
evaluator selects scan indices `1..4`, reports a median lidar range of about
`0.44225 m`, and produces base-frame `(0.44667, -0.00144) m`: `0.01063 m`
from the approximate operator placement.
The range-dependent bound is `0.03 m + 0.04 × range`, capped at `0.08 m`;
it is about `0.0477 m` for this scan.
Because that placement was measured by the operator rather than a survey
instrument, this is a recorded replay software gate, not a physical accuracy
certification.

The translation revision is derived from lidar-origin distances of `0.200 m`
forward, `0.209 m` rearward, `0.213 m` right, and `0.235 m` left to the
tread-contact extents. It changes `base_link -> laser` translation to
`[0.0045, -0.0110, 0.1905] m`; the recorded yaw is unchanged.

The floor case is analytic geometry generated from the measured calibration.
It checks the implementation but is not represented as recorded floor-object
ground truth. The ambiguity case must return `bearing_only` with `point: null`.

Run the offline evaluator:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.camera_lidar_localization \
  artifacts/phase2_camera_lidar_localization/recorded_calibration_fixture.json
```

This artifact has no ROS publishers, live sensor access, or motor authority.
It does not prove async model-prefetch continuity and does not authorize a
physical run.
