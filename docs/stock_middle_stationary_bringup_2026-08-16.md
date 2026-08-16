# The stock middle is alive on the robot's own Pi

**2026-08-16 evening. Chassis OFF. Rover parked on the floor. Binary `1583463`
(`prototype/stock-middle`). Nothing moved, and nothing could.**

This is the exhibit the §3a experiment card sits on. It reduces §3a to one word: *wheels*.

---

## What was proven, and what was not

Read this table before any other line in this document.

| claim | status |
|---|---|
| `controller_server` starts, configures, **activates** | **PROVEN** |
| `local_costmap` exists and **populates from real floor-level lidar** | **PROVEN** |
| `planner_server` active, global costmap takes a real SLAM map | **PROVEN** |
| `behavior_server` active **and subscribed to the local costmap** | **PROVEN** |
| planner produces a path to a real free-space goal | **PROVEN** |
| the D36-era cause of 2 ms recovery refusals is structurally gone | **PROVEN** |
| progress-checker verdicts, recovery success/failure, RPP path tracking, goal completion | **NOT PROVEN — cannot be, and is not claimed** |

**Why the second half cannot be claimed:** the chassis is off, so there is no `/odom` and
no `odom→base_link` transform. The bringup publishes a **static** one. With the rover
parked that transform *is* the truth — it publishes reality rather than fabricating input
— but it becomes fabricated input the instant anything motion-semantic reads it.

**The frozen-frame trap, named so nobody misreads a log:** with a static TF the robot's
pose never changes, so a progress checker will eventually declare no-progress on any goal
being pursued, and the BT will fire recoveries. **Those recoveries' outcomes would be
theatre** — artifacts of a frozen frame, not evidence about recoveries. This session
therefore used **plan-only** `compute_path_to_pose` requests and never sent a
`navigate_to_pose` goal.

The static publisher lives only in `launch/bringup_stationary_test.launch.py`, behind the
marker `STATIONARY_TEST_STATIC_ODOM`, with guard tests
(`tests/test_stationary_test_launch_is_not_flyable.py`) asserting that no flight launch
references it — by marker **and** by shape, since a marker can be deleted while the same
node is added by hand. That launch also cannot start `rvr_node`, the explorer, or the
decisive controller. Un-startable by construction.

---

## The graph, as it stood

```
/controller_server          active [3]
/planner_server             active [3]
/behavior_server            active [3]
/local_costmap/local_costmap
/global_costmap/global_costmap
/lifecycle_manager_stationary_test
/rplidar_node
/slam_toolbox               active [3]
```

## 1. The local costmap — the thing that did not exist

`explore.launch.py:36` documents the bespoke stack's central omission: dropping
`controller_server` drops Nav2's local costmap. **D36 measured recoveries refusing in 2 ms
against a costmap that was not there.**

Measured this session, from real lidar returns at floor level:

```
/local_costmap/costmap_raw   60 x 60 cells @ 0.05 m  =  3.0 x 3.0 m rolling window
/local_costmap/costmap_updates
    34 update messages in 20 s (~1.7 Hz, matching publish_frequency 2.0)
    up to 579 LETHAL cells in a single update
```

579 lethal cells out of 3600 is the room's furniture and walls, seen at the scan plane.
**The local costmap is alive and populated.** Both configured layers came up —
`scan_layer` (marks and clears) and `touch_layer` (marks, lidar-unclearable, ToF-cleared)
— which is D42's fix present in a running system rather than in a design note.

## 2. The behaviour server has something to collision-check against

```
/behavior_server  Subscribers:
    /local_costmap/costmap_raw          nav2_msgs/msg/Costmap
    /local_costmap/costmap_raw_updates  nav2_msgs/msg/CostmapUpdate
    /local_costmap/published_footprint  geometry_msgs/msg/PolygonStamped

/local_costmap/costmap_raw   Publisher count: 1   Subscription count: 1
```

One publisher, one subscriber, and in the bespoke stack the publisher did not exist.
**Nav2's Spin, BackUp, DriveOnHeading and Wait have been configured and running all along
— they were refusing because they had nothing to check against.** That cause is gone.

This is a *structural* claim, and it is the honest limit of it: the recoveries can now see
a costmap. Whether they *succeed* needs wheels.

## 3. SLAM, the global costmap, and a real plan

`slam_toolbox` (async, `mode: mapping`) built a map of the actual room from the actual
lidar while the rover sat still:

```
/map   74 x 143 cells @ 0.05 m  |  occupied 167   free 2703   unknown 7712
```

The global costmap's static layer had been warning `Can't update static costmap layer, no
map received` every 2 s — because nothing was publishing a map. **One second after SLAM
activated, that stopped**, replaced by:

```
[global_costmap]: StaticLayer: Resizing costmap to 74 X 143 at 0.050000 m/pix
```

Then a plan-only request to a free-space goal 0.6 m ahead:

```
ComputePathToPose -> SUCCEEDED   planning_time 0.001 s   error_code 0
```

**The planner produced a path over a costmap built from this room, in about a
millisecond.**

## 4. Two defects found on the way, both fixed

**`config/lean_nav2_stock.yaml` was never installable.** `setup.py` lists launch and
config files explicitly rather than globbing, and the stock config had been on this branch
since it was written without ever being in that list. `get_package_share_directory` could
not find it — **the stock-middle config could not have been loaded by anything, even
deliberately.** Real, reviewed, and unreachable. Fixed, with a guard test that walks
`launch/` and `config/` and asserts every file is in the manifest.

**`slam_toolbox` is a lifecycle node and does not self-activate.** Launched without
`autostart` it sits in `unconfigured`, publishing nothing, logging nothing after its
banner — a silent no-op that looks like a running node. Configured and activated
explicitly here; any future launch that includes it must handle its lifecycle.

## Reproducing it

```bash
# on the Pi, chassis OFF, rover parked
ros2 launch sphero_rvr_driver bringup_stationary_test.launch.py
ros2 launch sphero_rvr_driver mapping.launch.py start_lidar:=false   # then activate:
ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate
```

Teardown: `/stop_motor` **before** killing the lidar node — killing it does not stop the
motor — then SIGINT by explicit PID. Never `pkill -f` with a pattern that appears in your
own command line; it matches the SSH session and kills the shell mid-sequence. That
happened once tonight and cost a retry.

## What §3a needs now

Wheels, and the curve-derived angular constants that landed with this branch
(`rotate_to_heading_angular_vel: 3.55`, `min_rotational_vel: 3.55`,
`max_rotational_vel: 5.83`). Everything else in the stock middle has been shown to come
up, wire together, and do useful work on this robot's own hardware.
