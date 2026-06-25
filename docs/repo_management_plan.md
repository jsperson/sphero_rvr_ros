# Repository management plan

This repo currently has two dirty, divergent workspaces. Until the user says otherwise, treat the Mac clone as the source-of-truth workspace for commits and review, and treat the Pi checkout as a hardware validation/deployment workspace.

## 1. Authoritative development workspace

- **Default source of truth:** `/Users/jsperson/source/sphero_rvr_ros` on the Mac.
  - It uses the SSH remote `git@github.com:jsperson/sphero_rvr_ros.git`.
  - It is already aligned with `origin/main` and contains the large API-parity, safe-ROS-surface, test, and docs work.
  - It is the right place to curate commits, run ROS-free unit tests, and prepare reviewed branches/PRs.
- **Default Pi role:** `/home/jsperson/ros2_ws/src/sphero_rvr_ros` on `sphero-pi-2` is the deployment and hardware-validation checkout.
  - Use it for ROS 2 Jazzy builds, lidar/RVR integration, no-motion ROS checks, and approved live hardware smoke.
  - Do not let it silently become a second development trunk. Promote Pi work intentionally by exporting patches/branches back through the Mac source-of-truth flow.

## 2. Immediate cleanup actions

Do these before adding more feature work:

1. Freeze both dirty trees except for this management plan.
2. Keep the safety snapshot at `/Users/jsperson/.hermes/backups/sphero_rvr_ros-20260624-220709/` intact:
   - `mac/tracked.diff`, `mac/untracked.tgz`, `mac/status.txt`
   - `pi/tracked.diff`, `pi/untracked.tgz`, `pi/status.txt`
3. On the Mac, create a repo-management branch from current `main` before staging anything:
   ```bash
   cd /Users/jsperson/source/sphero_rvr_ros
   git switch -c chore/repo-management-plan
   ```
4. Commit this document by itself, or keep it as the first reviewed change, so the workflow is visible before feature commits land.
5. Inventory the two dirty trees from the snapshots and live status; classify each changed path as one of:
   - API parity/core driver work
   - safe ROS surface/odometry work
   - documentation/status updates
   - Pi-only lidar/mapping/calibration work
   - generated/runtime/local environment files
6. Reconcile one slice at a time using explicit patches, never broad staging.

Do **not**:

- run `git add .` in either workspace;
- overwrite either dirty tree with `git reset --hard`, `git checkout .`, `git clean -fd`, `rsync --delete`, or a blind copy from the other machine;
- push directly to `main` without review;
- launch motor-capable RVR commands as part of repo cleanup;
- treat macOS fake/unit test success as Pi ROS or live robot validation.

## 3. Reconciling the Mac and Pi dirty trees

Use the Mac as the merge bench and the snapshots as the safety net.

Recommended order:

1. **Preserve current state.** The existing snapshot is the rollback anchor. If either workspace changes materially before reconciliation, take a new timestamped snapshot first.
2. **Mac-first commit queue.** On the Mac, split current dirty work into small topic branches/commits. Use `git add -p` and path-specific staging only.
3. **Pi patch import.** For Pi-only work, inspect `pi/tracked.diff` and `pi/untracked.tgz`; apply only the intended files onto a fresh Mac branch with `git apply --check` / `git apply` or manual file copy from the extracted tarball.
4. **Conflict review.** For files changed in both places (`README.md`, `config/rvr.yaml`, `package.xml`, `src/sphero_rvr_driver/rvr_node.py`, odometry/tests/docs), compare Mac and Pi versions side-by-side and decide the winner per feature slice. Do not choose by timestamp.
5. **Pi reset only after merge.** Once a branch is reviewed and merged/pulled, reset the Pi checkout to the reviewed commit and redeploy from Git. The Pi should receive reviewed source, not hand-copied source-of-truth edits.

If Pi work must become authoritative for a slice, promote it explicitly: create a branch name for it, import its patch into the Mac repo, run review/tests there, then deploy back to Pi.

## 4. Branch naming strategy

Use short topic branches grouped by intent:

- `chore/repo-management-plan` — this workflow document.
- `docs/api-parity-handoff` — capability matrix, exposure policy, gap report, README/STATUS doc updates.
- `feat/core-api-parity` — core commands, responses, dispatcher, driver, fake transport, and parser/coverage tests.
- `feat/ros-safe-surfaces` — ROS node/config surfaces such as ambient light, motor temperatures, LEDs, reset services, diagnostics.
- `feat/odom-tf` — odometry implementation, `/odom`, TF, config, and tests.
- `feat/pi-lidar-mapping` — Pi-origin lidar, mapping launch/config, calibration scripts, workspace import files.
- `test/api-parity-coverage` — only if test-only commits need to be reviewed separately from implementation.

Prefer branch names that match PR scope. Avoid long-running catch-all branches named after the robot or date; those become junk drawers with wheels.

## 5. Commit and PR grouping plan

Keep each PR reviewable and deployable:

1. **Repo-management/doc bootstrap**
   - `docs/repo_management_plan.md`
   - Optional README docs-map link in a separate hunk if desired.
2. **API parity documentation ledger**
   - `docs/rvr_capability_matrix.md`
   - `docs/rvr_api_parity_scope.md`
   - `docs/rvr_api_gap_report.md`
   - `docs/rvr_notification_events.md`
   - README/STATUS references that describe the ledger.
3. **Core API parity implementation**
   - `src/sphero_rvr_core/*`
   - `tests/test_missing_command_builders.py`
   - parser/dispatcher/driver capability tests.
4. **Safe ROS operational surfaces**
   - `src/sphero_rvr_driver/rvr_node.py`
   - `src/sphero_rvr_driver/diagnostics.py`
   - `src/sphero_rvr_driver/led.py`
   - `config/rvr.yaml`, `package.xml`
   - `tests/test_ros_safe_surfaces.py` and node/config/diagnostics tests.
5. **Odometry and TF**
   - `src/sphero_rvr_driver/odometry.py`
   - `/odom` and TF pieces in `rvr_node.py`
   - `docs/rvr_odometry_tf_design.md`
   - `tests/test_odometry.py`.
6. **Pi lidar/mapping/calibration import**
   - `config/lidar.yaml`, `config/slam_toolbox.yaml`
   - `launch/lidar.launch.py`, `launch/mapping.launch.py`
   - `scripts/rvr_motion_calibration.py`
   - `workspace.repos`
   - ROS/Pi validation notes.

Every PR should include the smallest relevant validation output in its description: macOS fake/unit tests for code, Pi no-motion ROS checks for ROS packaging, and separately approved live smoke only when the branch truly needs hardware.

## 6. What stays uncommitted, generated, or ignored

Keep these out of source control:

- Python and build artifacts already covered by `.gitignore`: `.venv/`, `__pycache__/`, `*.py[cod]`, `.pytest_cache/`, `build/`, `dist/`, `*.egg-info/`, `install/`, `log/`.
- ROS workspace build outputs: `build/`, `install/`, `log/` under `~/ros2_ws`.
- Runtime logs such as `~/.local/state/sphero_rvr/*.log`.
- Local hardware/session captures, ad hoc maps, bag files, and calibration output unless they are deliberately promoted as small documented fixtures.
- Local credentials, host-specific shell setup, and environment files.

Potentially commit these only after review:

- ROS launch files, package configs, and calibration scripts that are part of repeatable operation.
- Small sample config files with safe defaults and no host secrets.
- Documentation describing validated hardware behavior and safety limits.

## 7. Pi deployment and validation process

After a branch is reviewed or intentionally selected for hardware validation:

1. On the Mac, push the branch or merge target to GitHub.
2. On the Pi, fetch the reviewed source instead of copying dirty files by hand:
   ```bash
   cd /home/jsperson/ros2_ws/src/sphero_rvr_ros
   git fetch origin
   git switch <reviewed-branch-or-main>
   git pull --ff-only
   ```
3. Run the no-motion ROS environment gate:
   ```bash
   cd /home/jsperson/ros2_ws
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install --packages-select sphero_rvr_driver
   source install/setup.bash
   ros2 pkg executables sphero_rvr_driver
   python3 - <<'PY'
   from sphero_rvr_driver.rvr_node import RVRNodeConfig
   print(RVRNodeConfig())
   PY
   ```
4. For live driver or motion-capable validation, require the project safety warning and explicit approval first:
   ```text
   WARNING: this can start the RVR motors
   ```
5. Keep live smoke narrow: topic/service listing, telemetry reads, TF echo, `/stop`, one conservative `cmd_vel` pulse only when approved, and final `/stop`. Mapping, TUI driving, and autonomy require separate approval scopes.
6. Record Pi validation results in the PR or `STATUS.md` only after they actually ran.

## 8. Rollback and snapshot process

Before risky reconciliation, deployment, or hardware work, create a timestamped snapshot with tracked diffs, untracked archive, status, branch, and remote info for each workspace.

Minimum snapshot shape:

```bash
STAMP=$(date -u +%Y%m%d-%H%M%S)
ROOT=/Users/jsperson/.hermes/backups/sphero_rvr_ros-$STAMP
mkdir -p "$ROOT/mac" "$ROOT/pi"

cd /Users/jsperson/source/sphero_rvr_ros
git status --short --branch > "$ROOT/mac/status.txt"
git diff > "$ROOT/mac/tracked.diff"
git ls-files --others --exclude-standard -z | tar --null -T - -czf "$ROOT/mac/untracked.tgz" 2>/dev/null || true
```

For the Pi, run the same commands in `/home/jsperson/ros2_ws/src/sphero_rvr_ros` and copy the results under `$ROOT/pi/`, or run them over SSH and store the output locally.

Rollback options:

- To recover tracked edits, apply the relevant `tracked.diff` onto the matching base commit with `git apply --check` then `git apply`.
- To recover untracked files, extract the matching `untracked.tgz` into a scratch directory first, inspect it, then copy only intended files.
- To abandon a failed deployment on the Pi, reset the Pi to the last reviewed Git commit and rebuild; do not reconstruct source from memory or terminal scrollback.

## 9. Decision checkpoints

Ask before changing the workflow if any of these become true:

- the user wants the Pi workspace to become the source of truth for a specific branch;
- dirty trees have changed since the `20260624-220709` snapshot and need a new safety snapshot;
- reconciliation requires discarding a file from either tree;
- live hardware validation is needed;
- a generated artifact looks useful enough to commit as a fixture or documented sample.
