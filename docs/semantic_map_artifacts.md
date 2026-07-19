# VS06 semantic map artifacts

`semantic_map_artifacts.py` is the replay-first final artifact generator for the canonical shoe-mapping vertical slice. It is ROS-free and consumes committed outputs from the preceding slices:

- VS02 occupancy map YAML/PGM (`artifacts/vs02_slam_replay_fixture_map/`)
- VS04 detector evaluation JSON and evidence-frame references (`artifacts/vs04_shoe_detector_eval_bag/`)
- VS05 map-frame shoe observations/tracks (`artifacts/vs05_shoe_map_projection/`)

The generator produces:

- `semantic_map.json` — canonical structured semantic map model
- `semantic_map.geojson` — GeoJSON `FeatureCollection` for UI/API map overlays
- `annotated_semantic_map.ppm` — occupancy map image with semantic shoe-track markers
- `coverage_uncertainty_report.md` — observed/inaccessible/occluded/uncertain coverage limits
- `mission_summary.md` — concise operator-facing summary

## Command

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.semantic_map_artifacts \
  --map-yaml artifacts/vs02_slam_replay_fixture_map/vs02_replay_fixture_map.yaml \
  --detector-evaluation-json artifacts/vs04_shoe_detector_eval_bag/shoe_detector_evaluation.json \
  --map-observations-json artifacts/vs05_shoe_map_projection/shoe_map_observations.json \
  --output-dir artifacts/vs06_semantic_map \
  --source-run-id canonical_replay_run \
  --source-map-id vs02_replay_fixture_map
```

Installed console entry point:

```bash
rvr_semantic_map_artifacts --map-yaml ... --detector-evaluation-json ... --map-observations-json ...
```

## Model boundaries

Each semantic object includes:

- class/status/confidence, with detector and projection confidence split out
- map pose (`x`, `y`, `yaw`, frame, timestamp)
- observation count and first/last observation timestamps
- evidence frame references
- source run/map IDs and source artifact paths
- uncertainty and coverage metadata

The output deliberately does not claim every shoe was found. It distinguishes observed coverage from inaccessible regions, occlusion, uncertain detections, and detector/projection confidence limits. Absence outside observed coverage is not evidence of no shoes. Tiny but load-bearing caveat, that one.
