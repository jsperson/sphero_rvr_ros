# System validation and current-SHA corpus

This package has two validation layers, both run explicitly by an operator or development agent:

1. ROS-free unit/system gates that run on any Python host.
2. ROS 2/colcon checks that run in the Pi's Jazzy workspace and verify package install, entry points, launch-description construction, and deterministic process cleanup.

No gate on this page authorizes physical motion. Hardware-in-loop remains a separate evidence suite with an explicit operator gate.

## Validation commands

This repository does not use a hosted GitHub Actions workflow. Run the ROS-free checks locally with the bounded test runner:

```bash
python3 scripts/run_pytest_bounded.py --timeout 90 -- -vv
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv tests/test_system_validation.py
python -m sphero_rvr_driver.system_validation --repo-root .
```

After deployment, run the ROS package checks on the Pi from its workspace:

```bash
colcon build --symlink-install --packages-select sphero_rvr_driver
source install/setup.bash
ros2 pkg executables sphero_rvr_driver
ros2 launch sphero_rvr_driver supervised_rvr.launch.py --show-args
```

The system validation module exercises the fake route/collision corpus and latency gate without ROS, serial ports, sensors, or motors. The Pi checks above inspect the package and launch description but do not launch the rover.

## Bag-driven integration corpus

The synchronized current-SHA corpus is a no-motion Pi capture. It must be captured only after the target SHA has been deployed and verified on the Pi without launching motor-capable motion.

Required topics:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/camera_node/image_raw`
- `/camera_node/camera_info`
- `/diagnostics`
- `/collision_stop/state`
- `/mission_api/v2/live_route/request`
- `/mission_api/v2/live_route/status`

The manifest schema is `rvr-current-sha-corpus.v1` and records:

- exact `git_sha`
- no-motion flag
- environment (`host`, `ros_distro`, and non-secret hardware notes)
- topic counts and derived rates from rosbag2 `metadata.yaml`
- frame graph roots, at minimum `odom` and `base_link`
- clock basis
- cleanup evidence, including deterministic process termination and serial owner state

Minimal Python usage:

```python
from sphero_rvr_driver.system_validation import (
    build_current_sha_corpus_manifest,
    validate_current_sha_corpus_manifest,
)

manifest = build_current_sha_corpus_manifest(
    run_id="current-sha-no-motion",
    bag_path="/home/jsperson/rvr_runs/current-sha-no-motion/rosbag",
    git_sha="<deployed-sha>",
    environment={"host": "sphero-pi-2", "ros_distro": "jazzy", "hardware_motion": False},
    frame_graph={"odom": ["base_link"], "base_link": ["laser", "camera_link"]},
    cleanup={"processes_terminated": True, "serial_owners_after": []},
)
validate_current_sha_corpus_manifest(manifest, expected_sha="<deployed-sha>")
```

## Hardware-in-loop suite, not active here

The hardware-in-loop suite should be run as a separate approved card/run and should upload evidence with the same schema plus:

- operator approval id
- pre/post process list
- pre/post serial owners for `/dev/ttyAMA0`, `/dev/ttyUSB0`, and `/dev/rplidar`
- route/collision status logs
- emergency dispatch latency samples under contention
- rosbag path and checksums

Do not store credentials, raw personal identifiers, home Wi-Fi details, or unrelated camera imagery in the manifest.
