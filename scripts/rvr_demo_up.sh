#!/usr/bin/env bash
# The standing demo, started as ONE process tree so systemd can stop it as one.
# Mirrors docs/web_console.md's "morning-demo shape" exactly -- if that changes,
# this changes with it.
#
# `set -u` is deliberately NOT set before sourcing ROS: the ROS setup scripts
# reference unset variables and a strict shell dies there SILENTLY, which cost a
# watcher 2.5 hours of polling a corpse. Strictness resumes afterwards.
set -eo pipefail

WS="${RVR_ROS_WS:-$HOME/ros2_ws}"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "$WS/install/setup.bash"
set -u

# Children in this script's own process group; systemd's cgroup holds them all.
ros2 launch sphero_rvr_driver sim_closed_loop.launch.py start_coverage_explorer:=true &
RIG=$!
sleep 25
ros2 run sphero_rvr_driver task_node &
TASK=$!
sleep 8
ros2 run sphero_rvr_driver web_console &
CONSOLE=$!

# Forward a stop to all three rather than dying and orphaning them. The console
# obeys SIGTERM as of D74; the rig's launch and task_node are stopped here and
# swept by the unit's cgroup kill if any child outlives its parent.
terminate() {
  kill -TERM "$CONSOLE" "$TASK" "$RIG" 2>/dev/null || true
  wait "$CONSOLE" "$TASK" "$RIG" 2>/dev/null || true
  exit 0
}
trap terminate TERM INT

# Exit when the first of them exits, so a dead demo does not look alive.
wait -n "$RIG" "$TASK" "$CONSOLE"
terminate
