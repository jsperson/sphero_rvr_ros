# Replay-first shoe map projection

VS05 projects VS04 camera detections into `map`-frame semantic observations without starting live motors or sensors. The implementation is ROS-import-free in `sphero_rvr_driver.shoe_map_projection`; runtime ROS adapters should pass measured `/camera_node/camera_info`, timestamped map poses/TF, and VS04 `shoe_detector_evaluation.json` detections into that library.

## Safety boundary

- Projection reads structured detector JSON, CameraInfo-like calibration, evidence-frame paths, and timestamped poses.
- It does not publish `/cmd_vel`, call STOP/ESTOP services, start `rvr_node`, start live camera/lidar, or run `ros2 bag play`.
- If a workflow needs live camera, lidar, mapping, or driver launch, use the existing physical/hardware gate from `STATUS.md` first.

## Inputs

Required semantic projection inputs:

- VS04 `shoe_detector_evaluation.json` with per-frame `frame_id`, `detections`, bbox/confidence/status, and evidence-frame naming.
- Measured CameraInfo from `/camera_node/camera_info`; empty intrinsics are rejected via `require_configured_camera_info()`.
- Measured static mount defaults from `docs/camera_lidar_calibration.md`: `base_link -> camera_link` translation `[0.0587375, -0.0301625, 0.114300]`, roll/yaw zero, pitch `-0.0523598775598299 rad`, and the `camera_link -> camera_optical_frame` optical convention.
- Timestamped robot poses in `map`: exact or interpolated inside the supplied pose history only. Requests before the first pose or after the last pose raise errors instead of extrapolating believable nonsense.

## Projection model

For each accepted/review detection, the library uses the bottom-center bbox footpoint as the ground contact estimate:

1. Build a pinhole ray from CameraInfo `K`: `[(u-cx)/fx, (v-cy)/fy, 1]` in `camera_optical_frame`.
2. Convert optical-frame ray to `camera_link` convention: x forward, y left, z up.
3. Apply measured `base_link -> camera_link` mount rotation/translation.
4. Intersect the ray with the `base_link` ground plane `z=0`.
5. Transform the base-frame point into `map` with the timestamp-matched pose.
6. Attach the VS04 confidence/status/reasons and evidence-frame reference.

## Tracking and deduplication

`ShoeObservationTracker` clusters map observations by configurable radius (`0.25 m` default):

- observations within the radius join the nearest existing track;
- each observation receives a stable `shoe_obs_####` id;
- each cluster receives a stable `shoe_track_####` id;
- track confidence is the maximum observation confidence;
- track position is confidence-weighted over member observations;
- repeated evidence references are preserved for downstream semantic-map artifacts.

## Output schema

`shoe_map_observations.json` reports:

```json
{
  "source_schema": "vs04_shoe_detector_evaluation",
  "frame": "map",
  "dedup_radius_m": 0.25,
  "track_count": 1,
  "observation_count": 2,
  "tracks": [
    {
      "track_id": "shoe_track_0001",
      "label": "shoe",
      "status": "accepted",
      "confidence": 0.83,
      "frame": "map",
      "position": {"x": 1.267, "y": 2.112, "timestamp_ns": 1784305462310414735},
      "observation_count": 2,
      "evidence": [{"frame_id": "bag_frame_0000_1784305462310414735", "path": "evidence_frames/bag_frame_0000_1784305462310414735_evidence.png"}]
    }
  ],
  "observations": []
}
```

## Uncertainty limits to carry downstream

Every report explicitly carries these limits:

- ground-plane assumption: each detection is projected from the image-footpoint ray to `z=0`;
- occlusion: partial views can shift the apparent contact point;
- pose drift: map coordinates inherit SLAM/odometry and timestamp-alignment error;
- calibration error: CameraInfo and camera TF errors directly move the output point;
- inaccessible regions: absent detections outside the camera view are not free-space evidence.

## Offline CLI shape

The CLI is for replay/schema validation with explicit measured inputs:

```bash
rvr_shoe_map_project artifacts/vs04_shoe_detector_eval_bag/shoe_detector_evaluation.json \
  --camera-info-json /path/to/measured_camera_info.json \
  --pose 1784305462310414735,1.0,2.0,0.5 \
  --evidence-dir artifacts/vs04_shoe_detector_eval_bag/evidence_frames \
  --output artifacts/shoe_map_observations.json
```

The JSON CameraInfo fixture must contain `width`, `height`, `k`/`K`, and `distortion_model`; it is not a substitute for the robot-local calibration artifact documented in `docs/camera_lidar_calibration.md`.
