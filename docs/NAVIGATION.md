# Navigation

How the robot gets a real `odom->base_link` TF (KISS-ICP), how to actually send it a goal
(Nav2), and a short writeup of the odometry approach that got replaced along the way
(FAST-LIO). See the [README](../README.md) for quick start and launch commands, and
[ARCHITECTURE.md](ARCHITECTURE.md) for the full sensor/data-flow pipeline this fits into.

## KISS-ICP (real odom source)

`src/kiss-icp` is a git submodule, an **unpatched** checkout of
[PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp). It's the lidar odometry this
project actually uses. An earlier attempt used
[spark-fast-lio](https://github.com/MIT-SPARK/spark-fast-lio) (tight IMU+lidar fusion)
instead; that's no longer part of this repo — see
[FAST-LIO, briefly](#fast-lio-briefly-legacy) below for why. KISS-ICP does pure lidar
scan-matching odometry with **no IMU fusion at all**, sidestepping that whole class of bug
by construction — "a LiDAR odometry pipeline that just works," per its own tagline.

**KISS-ICP now publishes the real `/robot1/odom` and `odom->base_link` TF** — it used to be
a disconnected side-pipeline in its own `odom_lidar` frame, purely for evaluating the
algorithm; it has since been wired directly into this project's actual navigation TF tree
(full rationale in `autonomous_exploration_plan.adoc`'s "Option A"). `slam_toolbox`
still does the actual scan-matching/mapping/loop-closure on top of it — KISS-ICP supplies
better, unbiased odometry (see [FAST-LIO, briefly](#fast-lio-briefly-legacy) for what it
replaced), not a replacement for SLAM itself.

Run it alongside `office_sim.launch.py`, before `nav_stack.launch.py`:

```bash
ros2 launch go2_office_sim kiss_icp.launch.py
```

`kiss_icp_node` is launched directly (not via kiss-icp's own `odometry.launch.py`) so it
can be namespaced under `/robot1` and have its `/tf`/`/tf_static` remapped, same as
`slam_toolbox` needs in `nav_stack.launch.py` — kiss_icp_node hardcodes those topics
absolute and isn't namespaced by its own upstream launch file. Key parameters, all set
explicitly rather than left at kiss-icp's own defaults:
- `base_frame: base_link`, `lidar_odom_frame: odom` — publishes directly into this
  project's real TF tree instead of its own separate `odom_lidar` frame.
- `invert_odom_tf: false` — kiss-icp's own default (`true`) silently broadcasts the TF
  backwards (`base_link->odom` instead of the standard `odom->base_link`); confirmed live
  via a TF dump, easy to miss since nothing errors, TF just silently reports the parent
  and child swapped.
- `publish_debug_clouds: true` — always on now, decoupled from kiss-icp's own bundled
  RViz window (which is permanently broken here anyway: its `rviz/kiss_icp.rviz` hardcodes
  `Fixed Frame: odom_lidar`, which no longer exists once `lidar_odom_frame` is renamed to
  `odom`). This is what feeds `office.rviz`'s `KissIcpLocalMap` display — the accumulated
  point cloud KISS-ICP registers each scan against, the closest thing it has to "a map"
  since it has no occupancy-grid output of its own.

![Office point cloud KISS-ICP has accumulated, top-down](media/kiss_icp_pointcloud_overview.png)
*`office.rviz`'s `KissIcpLocalMap` display, top-down over the whole office — desks, the
conference table, lounge seating, and other landmarks are all visible as point clusters.*

![The robot navigating among landmarks inside that same accumulated cloud](media/kiss_icp_pointcloud_robot_closeup.png)
*Closer in: the robot (yellow bounding box, red/green axis marker) among the office's
landmark clusters, still inside the same `KissIcpLocalMap` cloud.*

**Dependencies:** `ros-jazzy-sophus` and `robin-map-dev` — already covered by
[Installation](../README.md#installation)'s existing
`rosdep install --from-paths src --ignore-src -r -y`, nothing extra to add there.

**No per-point `time` field** (same lidar situation FAST-LIO dealt with) — KISS-ICP's own
handling is simpler still: it just logs `Field 't', 'timestamp', 'time_stamp', or 'time'
does not exist. Disabling scan deskewing` and proceeds without deskewing. No synthesis
needed, nothing to work around.

**Verified working:** with the robot genuinely stationary, `/robot1/odom`'s position held
at sub-millimetre noise and identity orientation (no tilt at all, over repeated samples).
Drove the robot forward and confirmed real tracked displacement, not just an idle topic —
including a full Nav2 goal walking the robot ~1.6 m to within 2 cm of target. Resource
usage is light too: ~50 MB RSS, low CPU, no growth over time. **One real trade-off**: its
`odom` TF publish rate tracks the lidar scan rate (~7-10 Hz) with real jitter (gaps up to
~0.3 s), tighter than the old fixed-50 Hz kinematic odometry — several `nav2_params.yaml`
`transform_tolerance` values were widened (0.2s -> 0.5s) to absorb this; see that file's
comments if tuning further.

**No loop closure.** KISS-ICP is odometry only — each scan registers against a local voxel
map via ICP, and the reported pose is just the accumulated chain of those registrations.
There's no pose graph, no place recognition, nothing that notices "I've been here before."
Small per-registration errors compound like any dead-reckoning method: expect good
short-term tracking with drift accumulating over time/distance, not a globally consistent
map on its own — that's exactly why `slam_toolbox` (loop closure, in 2D) still sits on top
of it rather than KISS-ICP replacing it outright.

---

## Nav2 (point-to-point navigation)

Nav2 is fixed and working — `nav_stack.launch.py` brings up `pointcloud_to_laserscan` +
`slam_toolbox` + the full Nav2 stack, all namespaced under `/robot1`. Getting here took
finding and fixing five real bugs this session (namespace mismatches, missing config for
Jazzy-only nodes, a broken behavior-tree XML path, and a false-positive collision-monitor
throttle from the walking gait's own legs) — see `autonomous_exploration_plan.adoc`
for the full writeup if any of this needs revisiting.

**Sending a goal**, once `nav_stack.launch.py`'s log ends with `Managed nodes are active`:

- **Click in RViz** — the `SetGoal` ("2D Nav Goal") tool in `office.rviz`'s toolbar
  publishes to `/robot1/goal_pose`, which `bt_navigator` auto-forwards to a
  `NavigateToPose` action call. Click, then drag before releasing to set the goal's facing
  angle.
- **From the command line** — `ros2 action` isn't installed in this project's setup (see
  [Installation](../README.md#installation)'s note on `ros2 run`), so
  `ros2 action send_goal` won't work. Use the bundled script instead:
  ```bash
  python3 ~/go2_nav/src/go2_office_sim/scripts/send_nav_goal.py --x 1.0 --y 0.0
  ```
  Optional `--yaw` (radians) and `--timeout` (seconds). Prints live feedback
  (`distance_remaining`, `recoveries`) and the final result, then exits.

Pick a goal inside already-mapped, open floor space, not hard against a wall — planning
very close to an obstacle is a known NavFn edge case that showed up during testing and is
unrelated to the odometry/TF fixes above. Don't run `keyboard_teleop.py` at the same time
as sending goals; both drive `/robot1/cmd_vel` and will fight each other.

**Not implemented yet:** `frontier_explorer.py` — the piece that would pick goals
automatically based on unexplored map area, making exploration actually autonomous rather
than one manually-placed goal at a time. See `autonomous_exploration_plan.adoc` for
the design (nearest-frontier heuristic, `nav2_simple_commander`-based) and
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

---

## FAST-LIO, briefly (legacy)

This project first tried [spark-fast-lio](https://github.com/MIT-SPARK/spark-fast-lio)
(a FAST-LIO2 fork with tight IMU+lidar fusion) instead of KISS-ICP. Two real bugs were
found and one was fixed (an unbounded ikd-Tree memory leak from a local-map box sized for
outdoor-scale maps, and the worst of an ESKF divergence caused by too short a gravity-init
averaging window), but a smaller residual orientation/position bias survived every fix
attempted and was confirmed unrelated to sensor noise (raw IMU data was bit-identical
throughout the init window — this simulated IMU has no noise model at all). Rather than
keep chasing it inside a third-party EKF implementation, the project switched to KISS-ICP
(above), which has no IMU fusion to get wrong in the first place. `spark-fast-lio` is not
part of this repo; it's kept locally (not published) in case this is ever revisited.
