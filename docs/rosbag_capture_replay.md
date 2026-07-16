# Safe rosbag2 capture, replay, and run manifests

This workflow records reusable lidar/camera/odom/TF/diagnostics data without launching hardware. The commands here build `ros2 bag` commands only: they do not start the RVR driver, lidar, camera, SLAM, teleop, `/cmd_vel`, or any motor-capable process.

## Safety boundary

- Default mode is dry-run. It prints the exact rosbag command, destination, topics, and safety warning, then exits.
- `--execute` is required before `ros2 bag record`, `ros2 bag play`, or `ros2 bag info` is invoked.
- Sensor and driver processes must be started separately under their own approval gate. This capture helper only subscribes to existing topics.
- Capture and replay reject `/cmd_vel`, raw motor, motor-control, teleop, and velocity-command-like topics by default.
- `--allow-unsafe-topics` exists only for developer analysis of already-recorded bags. Do not use it for live robot operation.

## Storage layout

Default captures are organized under `~/rvr_runs/<run-id>/`:

```text
~/rvr_runs/rvr-YYYYmmddTHHMMSSZ/
  rosbag/              # rosbag2 output directory
  run_manifest.json    # structured capture context
```

Use a descriptive run ID when the data has a known purpose:

```bash
rvr_rosbag_capture --run-id stationary_lidar_camera_2026-07-16
```

## Capture

Dry-run first:

```bash
rvr_rosbag_capture --run-id room_scan_001
```

Default capture topics:

```text
/scan
/camera/image_raw
/camera/camera_info
/odom
/tf
/tf_static
/diagnostics
```

To record only a specific subset, repeat `--topic` or comma-separate values:

```bash
rvr_rosbag_capture --run-id lidar_tf_only --topic /scan --topic /tf,/tf_static
```

To add future mission/event data without losing the defaults:

```bash
rvr_rosbag_capture --run-id semantic_pass_001 --extra-topic /mission/events --extra-topic /detector/objects
```

After reviewing the printed destination, topic list, command, and warning, execute explicitly:

```bash
rvr_rosbag_capture --execute --run-id room_scan_001 --notes "stationary lidar/camera sample; sensors started by separate approved terminal"
```

Stop capture with `Ctrl-C`. `ros2 bag record` handles bag closure; if the command exits non-zero during helper-managed startup, the helper removes the new per-run folder so failed dry artifacts do not masquerade as valid runs.

## Inspect

Inspection is hardware-free and defaults to dry-run:

```bash
rvr_rosbag_inspect ~/rvr_runs/room_scan_001/rosbag
rvr_rosbag_inspect --execute ~/rvr_runs/room_scan_001/rosbag
```

The manifest also stores a short `metadata.yaml` summary when the bag directory exists.

## Replay

Replay uses a non-motor topic allowlist and refuses motion topics by default:

```bash
rvr_rosbag_replay ~/rvr_runs/room_scan_001/rosbag
rvr_rosbag_replay --execute ~/rvr_runs/room_scan_001/rosbag
```

The default replay command is shaped like:

```bash
ros2 bag play ~/rvr_runs/room_scan_001/rosbag --topics /scan /camera/image_raw /camera/camera_info /odom /tf /tf_static /diagnostics
```

To replay a subset:

```bash
rvr_rosbag_replay ~/rvr_runs/room_scan_001/rosbag --topic /scan --topic /tf,/tf_static
```

Attempting to add `/cmd_vel` or motor-like topics fails unless `--allow-unsafe-topics` is passed. That override is for offline developer investigation only, and the manifest records the unsafe topics.

## Run manifest

`run_manifest.json` records:

- UTC timestamp and run ID;
- git SHA, branch, cleanliness, and status when available;
- host, OS, and ROS distro;
- capture/replay/inspect command and topic list;
- bag path and rosbag metadata summary;
- related map/log/artifact paths with deterministic SHA-256 inventories where practical;
- operator notes;
- whether hardware was active outside the helper;
- safety properties and any unsafe topic override.

Add related artifacts to the manifest with repeatable `--artifact` flags:

```bash
rvr_rosbag_capture --execute --run-id room_scan_001 --artifact ~/maps/room_scan.yaml --artifact ~/.local/state/sphero_rvr/rvr-console.log
```

## Cleanup

- Keep successful run folders intact until downstream mapping/perception work has consumed them.
- Remove failed or throwaway runs by deleting the whole per-run folder:

```bash
rm -rf ~/rvr_runs/<run-id>
```

- Do not hand-edit bag contents. If notes need correction, edit `run_manifest.json` or create a new manifest alongside the preserved bag.
