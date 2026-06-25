# Branch, PR, and Pi hardware validation workflow

Use this workflow for every change that may need Raspberry Pi or live RVR validation. The goal is boring: one reviewed source of truth, reproducible Pi deploys, and no mystery edits stranded in dirty trees.

## Roles and source of truth

- **Mac development/review clone:** `/Users/jsperson/source/sphero_rvr_ros`
  - remote policy: SSH GitHub remote, `git@github.com:jsperson/sphero_rvr_ros.git`
  - use this clone, or a feature-branch worktree from it, for commits, review, and PR preparation.
- **Pi validation checkout:** `/home/jsperson/ros2_ws/src/sphero_rvr_ros` on `sphero-pi-2`
  - current known remote state: HTTPS GitHub remote; normalize this to SSH when Pi GitHub auth is ready.
  - use this checkout for ROS 2 Jazzy builds, no-motion ROS checks, lidar checks, and explicitly approved live hardware smoke.
  - do not let it become an unreviewed second trunk. Pi-origin work must come back as patches or commits and go through the same branch/PR path.

Remote normalization recommendation:

```bash
# Mac: keep SSH.
cd /Users/jsperson/source/sphero_rvr_ros
git remote set-url origin git@github.com:jsperson/sphero_rvr_ros.git

# Pi: normalize to the same SSH remote once the Pi has a GitHub deploy/user key configured.
ssh sphero-pi-2 'cd /home/jsperson/ros2_ws/src/sphero_rvr_ros && git remote set-url origin git@github.com:jsperson/sphero_rvr_ros.git'
```

Until Pi SSH auth is configured, HTTPS on the Pi is acceptable for fetch-only validation, but record that exception in the PR notes. Do not mix HTTPS push credentials and SSH push credentials across machines without saying which machine authored the branch.

## Start work from a clean branch or worktree

Default feature branch in the Mac clone:

```bash
cd /Users/jsperson/source/sphero_rvr_ros
git fetch origin
git switch main
git pull --ff-only
git switch -c feat/<short-topic>
```

Use a separate worktree when the main clone has unrelated dirty work:

```bash
cd /Users/jsperson/source/sphero_rvr_ros
git fetch origin
git worktree add ../sphero_rvr_ros_<short-topic> -b feat/<short-topic> origin/main
cd ../sphero_rvr_ros_<short-topic>
```

Before editing, snapshot any dirty workspace that matters:

```bash
STAMP=$(date -u +%Y%m%d-%H%M%S)
ROOT=/Users/jsperson/.hermes/backups/sphero_rvr_ros-$STAMP
mkdir -p "$ROOT/mac" "$ROOT/pi"

cd /Users/jsperson/source/sphero_rvr_ros
git status --short --branch > "$ROOT/mac/status.txt"
git rev-parse HEAD > "$ROOT/mac/head.txt"
git remote -v > "$ROOT/mac/remotes.txt"
git diff > "$ROOT/mac/tracked.diff"
git ls-files --others --exclude-standard -z | tar --null -T - -czf "$ROOT/mac/untracked.tgz" 2>/dev/null || true

ssh sphero-pi-2 'cd /home/jsperson/ros2_ws/src/sphero_rvr_ros && \
  mkdir -p /tmp/sphero-rvr-snapshot && \
  git status --short --branch > /tmp/sphero-rvr-snapshot/status.txt && \
  git rev-parse HEAD > /tmp/sphero-rvr-snapshot/head.txt && \
  git remote -v > /tmp/sphero-rvr-snapshot/remotes.txt && \
  git diff > /tmp/sphero-rvr-snapshot/tracked.diff && \
  git ls-files --others --exclude-standard -z | tar --null -T - -czf /tmp/sphero-rvr-snapshot/untracked.tgz 2>/dev/null || true'
scp sphero-pi-2:/tmp/sphero-rvr-snapshot/status.txt "$ROOT/pi/status.txt"
scp sphero-pi-2:/tmp/sphero-rvr-snapshot/head.txt "$ROOT/pi/head.txt"
scp sphero-pi-2:/tmp/sphero-rvr-snapshot/remotes.txt "$ROOT/pi/remotes.txt"
scp sphero-pi-2:/tmp/sphero-rvr-snapshot/tracked.diff "$ROOT/pi/tracked.diff"
scp sphero-pi-2:/tmp/sphero-rvr-snapshot/untracked.tgz "$ROOT/pi/untracked.tgz"
```

## Minimum pre-push checks

Run the checks that match the change. For docs-only changes:

```bash
git diff --check -- README.md STATUS.md docs
```

For Python/core/ROS adapter changes on the Mac:

```bash
python3 -m venv /tmp/sphero-rvr-ros-test
/tmp/sphero-rvr-ros-test/bin/python -m pip install -e '.[dev]'
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m pytest tests -q
PYTHONPATH=src /tmp/sphero-rvr-ros-test/bin/python -m compileall -q src scripts launch
git diff --check
```

For ROS packaging or Pi-only runtime paths, add the Pi no-motion gate after deploying the branch:

```bash
cd ~/ros2_ws
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

Launching the RVR driver, calling `/stop`, publishing `/cmd_vel`, running `rvr-console`, or using any live UART command is motor-capable work. Use the project warning first and wait for explicit approval:

```text
WARNING: this can start the RVR motors
```

## Deploy a branch to the Pi for validation

Preferred path: deploy from Git, not by copying dirty files.

```bash
# Mac
cd /Users/jsperson/source/sphero_rvr_ros
git status --short
git push -u origin feat/<short-topic>

# Pi
ssh sphero-pi-2 'cd /home/jsperson/ros2_ws/src/sphero_rvr_ros && \
  git fetch origin && \
  git switch feat/<short-topic> && \
  git pull --ff-only'
```

If the branch should validate against reviewed `main`, fast-forward the Pi instead:

```bash
ssh sphero-pi-2 'cd /home/jsperson/ros2_ws/src/sphero_rvr_ros && \
  git fetch origin && \
  git switch main && \
  git pull --ff-only'
```

Temporary file sync is allowed only for quick, uncommitted experiments when the PR cannot yet be pushed. Always dry-run first, exclude generated directories, and take a snapshot before syncing:

```bash
cd /Users/jsperson/source/sphero_rvr_ros
rsync -avzn \
  --exclude .git \
  --exclude .venv \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  --exclude build \
  --exclude install \
  --exclude log \
  ./ sphero-pi-2:/home/jsperson/ros2_ws/src/sphero_rvr_ros/

# If the dry-run is exactly what you intend, rerun without -n.
rsync -avz \
  --exclude .git \
  --exclude .venv \
  --exclude __pycache__ \
  --exclude .pytest_cache \
  --exclude build \
  --exclude install \
  --exclude log \
  ./ sphero-pi-2:/home/jsperson/ros2_ws/src/sphero_rvr_ros/
```

Do not use `rsync --delete` against either repo checkout unless a current snapshot exists and the delete list has been reviewed. That command is a broom with a chainsaw taped to it.

## Bring Pi-validated changes back

If validation only produced notes, update the PR description or `STATUS.md`; do not edit source on the Pi.

If a fix was made on the Pi during hardware work, export it as a patch and apply it on a Mac branch:

```bash
# Pi
cd /home/jsperson/ros2_ws/src/sphero_rvr_ros
git status --short
git diff > /tmp/sphero-rvr-pi-fix.patch
git ls-files --others --exclude-standard -z | tar --null -T - -czf /tmp/sphero-rvr-pi-untracked.tgz 2>/dev/null || true

# Mac
mkdir -p /tmp/sphero-rvr-pi-import
scp sphero-pi-2:/tmp/sphero-rvr-pi-fix.patch /tmp/sphero-rvr-pi-import/
scp sphero-pi-2:/tmp/sphero-rvr-pi-untracked.tgz /tmp/sphero-rvr-pi-import/
cd /Users/jsperson/source/sphero_rvr_ros
git switch -c fix/<pi-observed-issue> origin/main
git apply --check /tmp/sphero-rvr-pi-import/sphero-rvr-pi-fix.patch
git apply /tmp/sphero-rvr-pi-import/sphero-rvr-pi-fix.patch
mkdir -p /tmp/sphero-rvr-pi-import/untracked
tar -xzf /tmp/sphero-rvr-pi-import/sphero-rvr-pi-untracked.tgz -C /tmp/sphero-rvr-pi-import/untracked
```

Inspect untracked files from the extracted directory and copy only intentional source/config/docs files into the repo. Then run the normal Mac checks, commit, push, and open/update the PR.

## PR and validation note format

Every PR should include:

- branch name and scope;
- Mac checks run, with exact commands and pass/fail summary;
- Pi no-motion checks run, if applicable;
- live hardware smoke results, only if explicitly approved and actually performed;
- commit or branch deployed on the Pi;
- rollback snapshot path, if the Pi or a dirty workspace was touched;
- follow-up risks or deferred validation.

Suggested `STATUS.md` or PR note block:

```markdown
### Hardware validation: <branch-or-commit>

- Date/time UTC:
- Operator:
- Pi checkout path: `/home/jsperson/ros2_ws/src/sphero_rvr_ros`
- Pi branch/commit:
- Mac branch/commit:
- Snapshot: `/Users/jsperson/.hermes/backups/sphero_rvr_ros-<stamp>/`
- No-motion ROS gate:
  - `colcon build --symlink-install --packages-select sphero_rvr_driver`: pass/fail
  - `ros2 pkg executables sphero_rvr_driver`: pass/fail
  - `RVRNodeConfig` import: pass/fail
- Live hardware gate:
  - approval received after `WARNING: this can start the RVR motors`: yes/no/not run
  - checks run:
  - observed behavior:
  - final stop/off cleanup:
- Follow-ups:
```

Never write “validated on Pi” unless the note names the branch/commit and command output that proved it.

## Rollback

Rollback starts from the snapshot, not from vibes.

Recover tracked edits from a snapshot:

```bash
cd /Users/jsperson/source/sphero_rvr_ros
git switch -c recover/<topic> <snapshot-base-commit>
git apply --check /Users/jsperson/.hermes/backups/sphero_rvr_ros-<stamp>/mac/tracked.diff
git apply /Users/jsperson/.hermes/backups/sphero_rvr_ros-<stamp>/mac/tracked.diff
```

Inspect untracked snapshot files before restoring:

```bash
mkdir -p /tmp/sphero-rvr-restore
cd /tmp/sphero-rvr-restore
tar -xzf /Users/jsperson/.hermes/backups/sphero_rvr_ros-<stamp>/mac/untracked.tgz
find . -maxdepth 3 -type f | sort
```

Rollback a failed Pi deploy to the last reviewed Git commit:

```bash
ssh sphero-pi-2 'cd /home/jsperson/ros2_ws/src/sphero_rvr_ros && \
  git fetch origin && \
  git switch main && \
  git reset --hard origin/main && \
  git clean -fdx -- build install log && \
  cd /home/jsperson/ros2_ws && \
  source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install --packages-select sphero_rvr_driver'
```

Use `git reset --hard` or `git clean` only after confirming the snapshot exists and the target branch/commit is the reviewed rollback point. If local Pi changes need saving, export a patch first.

## Rules that prevent divergent dirty trees

- Feature branches and PRs are the unit of review. No direct pushes to `main` for hardware bring-up work.
- The Pi validates reviewed or intentionally exported code; it does not quietly become the canonical source.
- Mac fake/unit tests, Pi no-motion ROS checks, and live RVR smoke are separate validation labels.
- Every Pi validation run records branch/commit, command output summary, and rollback snapshot path.
- Broad staging (`git add .`), blind copies, and destructive syncs without snapshots are out. Future-you deserves fewer crime scenes.
