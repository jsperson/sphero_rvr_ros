# VS04 shoe detector replay evidence

Generated from the VS01 primary replay bag `/home/jsperson/rvr_runs/camera_lidar_bag_20260717T112416/bag` on `sphero-pi-2` with the new `sphero_rvr_driver.shoe_detector` evaluator.

The preserved bag metadata points at `bag_0.mcap.zstd` while an uncompressed `bag_0.mcap` is also present. For read-only extraction, the run used `/tmp/vs04_bag_uncompressed`, a temporary directory containing a symlink to the preserved uncompressed MCAP plus patched metadata. The preserved bag was not edited.

Result: 5 sampled replay frames, 0 accepted shoe detections at `accept=0.70`, `review=0.45`. The frames show a checkerboard/calibration scene, so this is negative/no-positive replay coverage, not a recall claim.
