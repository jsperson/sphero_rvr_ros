# Replay-first shoe detector baseline

This is the VS04 perception baseline for the canonical shoe-mapping vertical slice. It is deliberately narrow: a dependency-light heuristic that evaluates existing/replayable camera frames and emits structured JSON plus annotated evidence frames. It is not a general ML object detector.

## Safety boundary

- The evaluator reads PPM frames or a recorded ROS 2 bag.
- It does not publish `/cmd_vel`, call STOP/ESTOP services, start the RVR driver, start live sensors, or expose generic ROS control.
- Bag mode opens a recorded MCAP bag directly through `rosbag2_py`; it does not run `ros2 bag play`.
- Use the VS01 primary replay bag first:

```bash
rvr_shoe_detector_eval \
  --bag /home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag \
  --image-topic /camera_node/image_raw \
  --sample-count 5 \
  --output-dir ~/rvr_runs/shoe_detector_eval_$(date -u +%Y%m%dT%H%M%SZ)
```

If the bag metadata points at a compressed `.mcap.zstd` file but an uncompressed `bag_0.mcap` is also present, create a temporary copy/symlink of the bag directory with `compression_format` and `compression_mode` blank and `relative_file_paths: [bag_0.mcap]`; do not edit the preserved bag in place.

## Detector contract

The current detector is `floor_dark_blob_shoe_baseline_v1`:

- scans only the lower portion of an RGB frame;
- finds dark, low-chroma connected components;
- scores candidates by area, floor-adjacency, fill, and a shoe-like elongated aspect ratio;
- classifies scores using explicit thresholds:
  - `accepted`: confidence >= `--accept-threshold` (default `0.70`)
  - `review`: confidence >= `--review-threshold` (default `0.45`)
  - `rejected`: below review threshold

Output schema per detection:

```json
{
  "label": "shoe",
  "confidence": 0.72,
  "status": "accepted",
  "bbox": {"x": 10, "y": 20, "width": 30, "height": 12},
  "reasons": []
}
```

The report also includes `coverage_statement` and `known_failure_modes` so downstream projection/deduplication can carry confidence limits forward instead of treating detections as truth.

## Known limits

- The primary VS01 replay bag shows a checkerboard/calibration scene and provides no positive shoe examples. Zero accepted detections on that bag should be reported as no observed positives, not proof of recall.
- Occluded, partially visible, very bright, or motion-blurred shoes can be missed.
- Dark floor tools, cables, wheels, and checkerboard targets can become review-level false candidates.
- This baseline is for replay triage and semantic-map plumbing. Replace or wrap it before claiming robust real-world shoe recognition.

## Evidence artifacts

Each run writes:

```text
<output-dir>/
  shoe_detector_evaluation.json
  sampled_frames/              # bag mode only, PPM P6 frames
  evidence_frames/             # PPM P6 frames with accepted/review boxes
```

Evidence frame colors:

- red: accepted detection
- amber: review-level candidate

For quick PNG conversion on macOS:

```bash
for f in <output-dir>/evidence_frames/*.ppm; do
  sips -s format png "$f" --out "${f%.ppm}.png"
done
```
