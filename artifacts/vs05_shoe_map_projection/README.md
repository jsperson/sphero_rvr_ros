# VS05 shoe map projection artifact

Replay-first semantic observation fixture generated from the committed VS04 shoe-detector evaluation report.

Contents:

- `camera_info_fixture.json` — sanitized CameraInfo-shaped fixture for offline schema validation only. Runtime projection must use measured `/camera_node/camera_info` from the robot-local calibration file.
- `shoe_map_observations.json` — map-frame observation report from VS04 review-level detections, projected with explicit synthetic pose samples and measured camera mount defaults.

Generation command:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.shoe_map_projection \
  artifacts/vs04_shoe_detector_eval_bag/shoe_detector_evaluation.json \
  --camera-info-json artifacts/vs05_shoe_map_projection/camera_info_fixture.json \
  --pose 1784305462310414735,1.000,2.0,0.5 \
  --pose 1784305467260649666,1.050,2.0,0.5 \
  --pose 1784305472226237684,1.100,2.0,0.5 \
  --pose 1784305477157278055,1.150,2.0,0.5 \
  --pose 1784305482121548820,1.200,2.0,0.5 \
  --evidence-dir artifacts/vs04_shoe_detector_eval_bag/evidence_frames \
  --output artifacts/vs05_shoe_map_projection/shoe_map_observations.json
```

Result: 4 review-level observations spatially deduplicated into 1 track. The source replay bag has no accepted positive shoe detections, so this artifact validates projection/tracking/output plumbing rather than detector recall.
