# M7.2 surveyed stationary-localization evidence

## Verdict

PASS on exact source
`a9ebc70314f9f2caf5b971b4c07e623a0a8e1047`.

The ROS-free evaluator recomputed 18 point-producing observations and one
physical ambiguity control from compact camera/lidar evidence. Every reviewed
gate passed:

- three distinct surveyed configurations for both `lidar_range` and
  `floor_projection` in the near, mid, and far bands;
- camera/lidar synchronization no greater than 100 ms;
- pose age no greater than 150 ms;
- every point error within its unchanged provisional bound;
- the physical occluder control returned `bearing_only`,
  `reason=ambiguous_lidar_clusters`, and `point=null`;
- stationary, no-motion authority and final cleanup were both proven;
- no tolerance was widened.

## Physical result

| Method | Band | Samples | Maximum error | p95 error | Reviewed bound | Evaluator recommendation |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `lidar_range` | near | 3 | 0.0236 m | 0.0230 m | 0.0442–0.0444 m | tighten candidate to 0.034 m |
| `lidar_range` | mid | 3 | 0.0171 m | 0.0170 m | 0.0563–0.0564 m | tighten candidate to 0.028 m |
| `lidar_range` | far | 3 | 0.0304 m | 0.0300 m | 0.0683–0.0684 m | tighten candidate to 0.041 m |
| `floor_projection` | near | 3 | 0.0252 m | 0.0241 m | 0.0500 m | tighten candidate to 0.035 m |
| `floor_projection` | mid | 3 | 0.0165 m | 0.0159 m | 0.0500 m | tighten candidate to 0.026 m |
| `floor_projection` | far | 3 | 0.0424 m | 0.0419 m | 0.0500 m | retain provisional bound |

One fixed checkerboard at map coordinate `(0.3545, -0.0110) m` was reused
while the powered-down rover was manually placed at three lateral
configurations in each band. Coverage is keyed to target geometry relative to
each surveyed rover pose, not to target labels.

## Calibration finding

The first mid-distance diagnostic honestly failed `floor_projection`: the
visible face-to-floor contact missed surveyed truth by about 0.130 m under the
old zero-pitch camera assumption. A physical pitch sweep placed the minimum
near -3.04 degrees. The retained measured default is
`camera_pitch=-0.0523598775598299 rad` (-3.0 degrees).

All acceptance observations were then recaptured under the exact SHA which
publishes that corrected transform. The calibration diagnostic and the older
SHA observations are not counted as acceptance samples.

## Evidence inventory

- `survey_layout.json` records the fixed map, corrected sensor datums,
  calibrated camera pitch, actual rover poses, range bands, uncertainty, and
  ambiguity setup.
- `session.json` contains calibration/map identities, all 19 compact samples,
  exact provenance, the raw-artifact inventory, no-motion authority, and the
  generated final cleanup audit.
- `report.json` is the evaluator output; its top-level `passed` field is true.
- `raw_artifact_sha256.txt` binds ten retained Pi rosbag MCAP files, their
  `metadata.yaml` files, and their capture manifests without committing the
  roughly 2.6 GB of raw camera data.
- `final_cleanup_audit.json` proves the camera, lidar, rosbag, prohibited
  nodes, motion-topic publishers, and rover serial owners were absent after
  collection.

Raw bags remain on `sphero-pi-2` below
`/home/jsperson/rvr_runs/m7-phase2-*-a9ebc70/`.

## Safety and scope

The RVR driver, RVR serial transport, Nav2, collision supervisor, velocity
publishers, physical execution, and motor authority were never started.
RVR power and motors remained unavailable. Only the camera, lidar, static
survey transforms, read-only snapshot subscriber, and bounded rosbag recorder
were used.

One near-center shutdown required escalation through SIGTERM to SIGKILL for
the camera process. The recording and compact sample had already completed;
the independent post-shutdown audit then proved that the camera/lidar devices
were ownerless and every sensor/motion process and ROS node was absent. This is
recorded as a cleanup anomaly, not hidden as a clean process exit.

This evidence closes only stationary M7.2. It does not approve moving
perception, motor-capable M7.3/M7.4 work, or the canonical physical mission.
