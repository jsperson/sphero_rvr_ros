# Hierarchical exploration Phase 2 evidence

Phase 2 implements recorded/offline camera-to-map object localization. It is a
ROS-free geometry and evidence layer; it does not subscribe, publish, launch
live sensors, or expose motor authority.

## Implemented contract

`camera_lidar_localization.py` returns exactly one of three explicit methods:

- `lidar_range`: a reviewed image anchor is converted to a camera bearing,
  associated with one contiguous lidar cluster inside a `2°` angular gate,
  and transformed through the recorded sensor mounts and map pose;
- `floor_projection`: a bottom-center contact anchor is intersected with the
  floor using measured camera intrinsics, height, and camera-to-base
  transform; or
- `bearing_only`: a bounded cone with `point: null` whenever a point
  localization cannot be justified.

Camera/lidar source timestamps must differ by no more than `100 ms`.
Localization must be no more than `150 ms` from the image source timestamp.
Absent clusters, multiple eligible clusters, anchors outside the calibrated
image/floor region, rays above the floor horizon, and stale timestamps all
reject point localization.

Every result carries the method, source timestamps, calibration identity, map
revision, evidence IDs, and an uncertainty estimate. Lidar uncertainty names
pixel, range, camera/lidar extrinsic, synchronization, and robot-pose sources.
Floor uncertainty names pixel, camera extrinsic, floor-plane, and robot-pose
sources.

## Quantitative evidence

The committed fixture combines recorded evidence with explicitly labeled
calibrated/synthetic negative cases:

| Gate | Bound | Result |
|---|---:|---:|
| Camera/lidar source-time delta | `≤ 100 ms` | `35.038348 ms` |
| Recorded target point error against approximate 18-inch placement | `≤ 0.03 m + 0.04 × range`, capped at `0.08 m` (`0.04768 m` here) | `0.022725 m` |
| Calibrated analytic floor-geometry error | `≤ 0.05 m` | floating-point noise |
| Two eligible lidar clusters | point forbidden | `bearing_only`, `point: null` |

The recorded plane target comes from
`camera_lidar_bag_20260717T112416`; its image evidence and compact scan returns
are identified in the fixture. The operator placement is approximate, so the
result demonstrates deterministic replay association within the chosen gate,
not survey-grade physical accuracy. The floor case validates calibrated
geometry and is not mislabeled as a recorded floor-object observation.

## Safety and carryover

Phase 2 does not change the command chain, Nav2 bridge, collision supervisor,
or any live launch. It does not solve or claim to solve the roughly
`12.691 s` model-latency/short-hop problem carried from Phase 1. Lookahead
depth and provider latency remain Phase 3 work.

Before any physical phase, repeat and retain the Raspberry Pi no-motion WFD
and command-ownership graph evidence. Drop-off sensing remains absent, so
physical hierarchical exploration remains prohibited.

## Validation

Use the repository's bounded runner:

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_camera_lidar_localization.py \
  tests/test_shoe_map_projection.py
```

The evaluator can also print the complete structured result without ROS:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.camera_lidar_localization \
  artifacts/phase2_camera_lidar_localization/recorded_calibration_fixture.json
```
