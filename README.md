# Go2 Office Navigation

![Go2 driving around the simulated office](docs/media/go2_nav_office.gif)

A Unitree Go2 quadruped, simulated in Gazebo Harmonic inside a procedurally-generated
office built in Blender, carrying a head-mounted RGBD camera and a back-mounted 3D lidar.
Drivable by keyboard with full sensor visualization in RViz; SLAM mapping and **Nav2
point-to-point navigation both work** (KISS-ICP drives the real odometry — see
[KISS-ICP](#kiss-icp-real-odom-source)); you can send it a goal by
clicking in RViz or from the command line (see [Nav2](#nav2-point-to-point-navigation)).
Fully autonomous frontier exploration (the robot picking its own goals to map the whole
scene unattended) is not implemented yet — see [Known issues](#known-issues) for that and
for an open gait-stability bug.

```
Blender scene  ──export_sdf.py──>  Gazebo world + robot  ──sensors + TF──>  RViz / rosbag
(blender/)                          (src/go2_office_sim/)                   (visualize/record)
```

---

## Quick start

Each of these goes in its own terminal, left running:

```bash
# Terminal 1 — the simulation: world, robot, sensors, locomotion
ros2 launch go2_office_sim office_sim.launch.py

# Terminal 2 — lidar odometry (KISS-ICP) -- this is the real odom->base_link
# source now (see the KISS-ICP section below), so bring it up before SLAM/Nav2
ros2 launch go2_office_sim kiss_icp.launch.py

# Terminal 3 — SLAM + Nav2
ros2 launch go2_office_sim nav_stack.launch.py

# Terminal 4 — visualize: robot model, camera, lidar/map/costmaps, trajectory,
# and a "2D Nav Goal" button to send Nav2 goals by clicking
ros2 launch go2_office_sim rviz.launch.py

# Terminal 5 (optional) — manual teleop instead of Nav2
# (don't run this at the same time as sending Nav2 goals -- both drive cmd_vel)
python3 ~/go2_nav/src/go2_office_sim/scripts/keyboard_teleop.py
```

Terminals 1, 2, and 4 alone give you a fully drivable, visualized robot with manual
teleop (skip 3 and 5). Add Terminal 3 for SLAM mapping + Nav2 navigation — see
[Nav2](#nav2-point-to-point-navigation) for how to actually send it a goal once that's up
(wait for its log to end with `Managed nodes are active` first, same as any Nav2 bringup).
**Nav2 nodes are heavier than they look on a loaded machine** — if bringup stalls partway
(a node stuck "Configuring" with no further log output), that's very likely the machine
falling behind under CPU load rather than something misconfigured; letting Terminals 1-2
settle for a few seconds before starting Terminal 3 usually gives it enough headroom.

Any directory works for these — `ros2 launch` and `rviz2` resolve via the sourced ROS
environment, not your cwd. A fresh terminal is ready automatically; see
[Installation](#installation) for the one-time setup that makes that true.

**Starting over cleanly:** if something's gotten into a bad state (a launch died half-way,
a stale process is holding a topic), kill everything project-related before relaunching:
```bash
pkill -9 -f "gz sim|ros_gz_bridge|quadropted_controller/lib|go2_office_sim|kiss_icp_node|rviz2|slam_toolbox|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|collision_monitor|opennav_docking|route_server|smoother_server|pointcloud_to_laserscan_node|lifecycle_manager"
```

---

## Layout

```
go2_nav/
├── README.md                    this file
├── blender/                     scene authoring (Blender project) — see blender/README.md
│   ├── office.blend
│   ├── build_office.py          generates office.blend + scene_objects.json from scratch
│   ├── export_sdf.py            exports office.blend -> src/go2_office_sim/{models,worlds}
│   ├── scene_objects.json       semantic map: rooms, landmarks, labels, poses
│   └── assets/                  Poly Haven source models (CC0)
├── src/
│   ├── go2_ros2_sim_py/         Go2 URDF, IK locomotion — vendored + patched, see Patches
│   ├── kiss-icp/                lidar-only odometry (git submodule, unpatched)
│   └── go2_office_sim/          this project's own ROS package
│       ├── models/office_scene/ exported office (glTF meshes + SDF), from blender/export_sdf.py
│       ├── worlds/office.sdf
│       ├── launch/
│       │   ├── office_sim.launch.py   world + robot + sensors + locomotion — ✅ working
│       │   ├── rviz.launch.py         RViz with the /robot1/tf remap it needs — ✅ working
│       │   ├── kiss_icp.launch.py     KISS-ICP odometry, now the real odom->base_link
│       │   │                          source (see KISS-ICP section) — ✅ working
│       │   └── nav_stack.launch.py    SLAM + Nav2 — ✅ both working
│       ├── config/
│       │   ├── bridge.yaml                   ros_gz_bridge: RGBD camera + 3D lidar topics
│       │   ├── slam.yaml                     slam_toolbox (async)
│       │   ├── pointcloud_to_laserscan.yaml  3D lidar -> 2D /scan for SLAM + Nav2
│       │   ├── nav2_params.yaml              Nav2 stack config — ✅ working
│       │   └── office.rviz                   robot model, camera, lidar, map, costmaps,
│       │                                      odom trajectory, KISS-ICP accumulated cloud,
│       │                                      "2D Nav Goal" tool
│       └── scripts/
│           ├── keyboard_teleop.py     WASD + arrows teleop — ✅ working
│           ├── send_nav_goal.py       CLI NavigateToPose action client — ✅ working
│           ├── frontier_explorer.py   autonomous exploration — ⬜ not implemented yet
│           └── record_bag.sh          rosbag recording — ⬜ not implemented yet
├── docs/
│   └── autonomous_exploration_plan.adoc   the Nav2/KISS-ICP fix writeup + forward plan
│                                           for frontier_explorer.py — see Known issues
├── bags/                         rosbag output directory
└── install/ build/ log/          colcon artifacts (generated, not source)
```

---

## Architecture

**Scene.** `blender/build_office.py` procedurally generates a 20×20 m open-plan office
(desks, conference room, kitchenette, lounge, reception) plus navigation landmarks —
coloured/textured cubes, traffic cones, and five placed Poly Haven props (rubber duck,
chess set, baseball, school chair, display shelves). `blender/export_sdf.py` exports it to
glTF meshes + SDF with one Gazebo link per semantic landmark and primitive box collisions
(cheap and exact, since the scene is built from boxes/cylinders in the first place).
`blender/scene_objects.json` is the machine-readable semantic map — room bounds, door
positions, and every landmark's label/category/zone/pose — meant to be a planner's ground
truth for a query like *"go to the rubber duck."*

**Robot.** Locomotion is based on
[go2_ros2_sim_py](https://github.com/abutalipovvv/go2_ros2_sim_py) by abutalipovvv,
vendored into `src/go2_ros2_sim_py` with its original commit history preserved (via `git
subtree`) and our own changes on top — see [Patches](#patches-to-upstream) for what
changed. It ships a Go2 URDF/xacro and an IK trot-gait controller
(`quadropted_controller`) that walks the robot from `cmd_vel` — a real gait, not a
kinematic teleport. It also ships its own leg-kinematic odometry node
(`QuadrupedOdometryNode.py`), still running (renamed to publish `/robot1/odom_leg`) as a
comparison/fallback source, but it is **not** what SLAM/Nav2 actually consume — see
[KISS-ICP](#kiss-icp-real-odom-source) for the real `/robot1/odom`
source. `go1_description/` and `docker/` are excluded from the build via `COLCON_IGNORE`
(go1 depends on Classic Gazebo packages Jazzy dropped, and isn't used here).

**Sensors.** The upstream xacro already wired up a front camera and a lidar with native
gz-sim syntax; both were modified in place: the camera became a **RGBD** sensor
(848×480, 87° HFOV, 15 Hz) on the head facing forward, and the lidar was repositioned to a
**back mount** and upgraded to a real **3D** scan (900 azimuth × 40 elevation samples,
−7°…+52°, 0.1–25 m, 10 Hz). Both bridge to ROS via `config/bridge.yaml`.

**Namespace.** Everything (topics, TF, nodes) lives under `/robot1`, matching upstream's own
convention — `/robot1/cmd_vel`, `/robot1/odom`, `/robot1/tf`, `/robot1/camera/*`,
`/robot1/lidar/points`, etc.

**Pipeline** (all of this works end-to-end):

```
gz sim (office.sdf + Go2)
  ├─ rgbd_camera ──┐
  └─ gpu_lidar ────┴─> ros_gz_bridge ─┬─> /robot1/camera/{image,depth_image,points,camera_info}
                                       └─> /robot1/lidar/points ──────────────┐
                                                 │                            │
                           pointcloud_to_laserscan ─> /robot1/scan            │
                                                 │                       kiss_icp_node
                                                 │                            │
                                                 │              /robot1/odom, odom->base_link TF
                                                 │                            │
                                    slam_toolbox (async) <───────────────────-┘
                                                 │
                                    /robot1/map, map->odom TF
                                                 │
                                    Nav2 (costmaps, planner, controller) ──> /robot1/cmd_vel
```

KISS-ICP feeds `slam_toolbox` (which still does the actual scan-matching/mapping/loop
closure) rather than replacing it — see [KISS-ICP](#kiss-icp-real-odom-source)
for why, and `docs/autonomous_exploration_plan.adoc` for the full writeup of how this was
wired up and the three real bugs that had to be fixed to get Nav2 working at all.

---

## Installation

**See [INSTALL.md](INSTALL.md) for the full, from-scratch walkthrough** — ROS 2 Jazzy and
Gazebo Harmonic setup, every apt package this project needs, cloning, and building.
It's written to work on a genuinely clean Ubuntu 24.04 machine (verified in a clean Docker
container, not just "worked on the machine that already had stuff installed").

Short version, if you already have ROS 2 Jazzy + Gazebo Harmonic (`ros-jazzy-ros-gz`):

```bash
cd ~/go2_nav
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

`--symlink-install` means edits to Python scripts and config/launch/xacro files take effect
immediately on the next run — only C++/CMake changes need a rebuild. `ros2 run` and `ros2
lifecycle` are not available in this setup (the `ros2run` companion package isn't installed
by INSTALL.md, and nothing here needs it) — scripts are invoked directly with
`python3 <path>` instead of `ros2 run <pkg> <exe>`.

---

## Launch commands

| Command | What it does | Status |
|---|---|---|
| `ros2 launch go2_office_sim office_sim.launch.py` | Spawns the office world + Go2 with sensors and locomotion | ✅ |
| `python3 ~/go2_nav/src/go2_office_sim/scripts/keyboard_teleop.py` | WASD + arrow-key teleop (needs its own terminal, keyboard focus) | ✅ |
| `ros2 launch go2_office_sim rviz.launch.py` | RViz — robot model, camera, map, costmaps, odom trajectory, "2D Nav Goal" tool | ✅ |
| `ros2 launch go2_office_sim kiss_icp.launch.py` | KISS-ICP odometry — the real `/robot1/odom` + `odom->base_link` TF source | ✅ |
| `ros2 launch go2_office_sim nav_stack.launch.py` | SLAM + pointcloud_to_laserscan + Nav2 | ✅ |
| `python3 ~/go2_nav/src/go2_office_sim/scripts/send_nav_goal.py --x <x> --y <y>` | Send a one-off Nav2 goal from the CLI (`ros2 action` isn't installed) | ✅ |
| `blender -b blender/office.blend --python blender/export_sdf.py` | Re-export the scene after editing it in Blender | ✅ |
| `blender -b --python blender/build_office.py` | Regenerate the scene from scratch — **overwrites `office.blend`** | ✅ |

**Keyboard teleop controls:**

| Keys | Effect |
|---|---|
| `W` / `S` | strafe forward / back |
| `A` / `D` | strafe left / right (gentler than forward/back — upstream's own tuning) |
| `←` / `→` | yaw left / right |
| `↑` / `↓` | stand taller / crouch (stance height, clamped 0.15–0.32 m) |
| `Space` | stop |
| `q` | quit |

Auto-stops ~0.4s after the last keypress (no key-release event from a raw terminal, so this
is the safety net).

**Manual velocity publish** (useful for scripting/testing without the teleop's keyboard
focus requirement):
```bash
ros2 topic pub -r 10 /robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

**Process persistence note:** if you background any of these (`&`), use `setsid ... &
disown`, not `disown` alone — plain `disown` did not reliably survive process-group signals
sent to unrelated jobs during this project's development, leaving Gazebo/RViz orphaned.
Simplest is still just separate foreground terminals, one per command above.

---

## KISS-ICP (real odom source)

`src/kiss-icp` is a git submodule, an **unpatched** checkout of
[PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp). It's the lidar odometry this
project actually uses. An earlier attempt used
[spark-fast-lio](https://github.com/MIT-SPARK/spark-fast-lio) (tight IMU+lidar fusion)
instead; that's no longer part of this repo — see **FAST-LIO, briefly** near the end of
this file for why. KISS-ICP does pure lidar scan-matching odometry with **no IMU fusion at
all**, sidestepping that whole class of bug by construction — "a LiDAR odometry pipeline
that just works," per its own tagline.

**KISS-ICP now publishes the real `/robot1/odom` and `odom->base_link` TF** — it used to be
a disconnected side-pipeline in its own `odom_lidar` frame, purely for evaluating the
algorithm; it has since been wired directly into this project's actual navigation TF tree
(full rationale in `docs/autonomous_exploration_plan.adoc`'s "Option A"). `slam_toolbox`
still does the actual scan-matching/mapping/loop-closure on top of it — KISS-ICP supplies
better, unbiased odometry (see **FAST-LIO, briefly** for what it replaced), not a
replacement for SLAM itself.

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

**Dependencies:** `ros-jazzy-sophus` and `robin-map-dev` — already covered by
Installation's existing `rosdep install --from-paths src --ignore-src -r -y`, nothing
extra to add there.

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
throttle from the walking gait's own legs) — see `docs/autonomous_exploration_plan.adoc`
for the full writeup if any of this needs revisiting.

**Sending a goal**, once `nav_stack.launch.py`'s log ends with `Managed nodes are active`:

- **Click in RViz** — the `SetGoal` ("2D Nav Goal") tool in `office.rviz`'s toolbar
  publishes to `/robot1/goal_pose`, which `bt_navigator` auto-forwards to a
  `NavigateToPose` action call. Click, then drag before releasing to set the goal's facing
  angle.
- **From the command line** — `ros2 action` isn't installed in this project's setup (see
  [Installation](#installation)'s note on `ros2 run`), so `ros2 action send_goal` won't
  work. Use the bundled script instead:
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
than one manually-placed goal at a time. See `docs/autonomous_exploration_plan.adoc` for
the design (nearest-frontier heuristic, `nav2_simple_commander`-based) and
[Known issues](#known-issues).

---

## FAST-LIO, briefly

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

---

## Patches to upstream

`src/go2_ros2_sim_py` is vendored from
[go2_ros2_sim_py](https://github.com/abutalipovvv/go2_ros2_sim_py) by abutalipovvv, with
its original commit history preserved via `git subtree` and our changes on top as a single
commit — `git log -- src/go2_ros2_sim_py` shows the full history, our patch included. The
changes are small, and none of them are project-specific enough to warrant maintaining a
parallel xacro. Summary:

- **`go2_description/package.xml`** — removed a dead `gazebo_plugins` (Classic Gazebo)
  dependency that isn't used anywhere in the actual build (no `find_package` call for it)
  and doesn't exist for Jazzy, which dropped Classic support. Was blocking `rosdep`.
- **`go2_description/xacro/robot.xacro`** — repositioned the lidar joint from front
  (0.22, 0, 0.095) to back-mounted (−0.26, 0, 0.14); enlarged its visual/collision cylinder
  to roughly Livox Mid-360 proportions.
- **`go2_description/xacro/gazebo.xacro`** — camera sensor type `camera` → `rgbd_camera`
  (848×480, 87° HFOV, 15 Hz, depth clip 0.2–25 m — see the Inf/large-room note below);
  lidar upgraded from a single-ring 2D scan to a real 3D scan (900×40 samples, −7°…+52°,
  range 0.1–25 m); topic renamed `scan`→`lidar` since gz-sim's raw multi-ring range array is
  no longer a valid 2D `LaserScan` once there's more than one elevation ring — `/points` off
  that topic is the real payload. Also added an `initial_value` to every leg joint's
  `ros2_control` position state, matching `RobotController.py`'s real standing-stance
  target — the actual fix for the spawn tip-over bug (see Known Issues); without it every
  joint spawns at 0 and gets snapped toward the real target the instant the gait controller
  activates, with gravity already acting on the body.
- **`quadropted_controller/scripts/cmd_vel_pub.py`** — raised the hardcoded `angular.z`
  clamp from ±1.0 to ±2.0 rad/s. It was silently undoing Nav2's own
  `rotate_to_heading_angular_vel` tuning (`nav2_params.yaml` raises it to 1.6 rad/s, with
  `velocity_smoother` allowing up to 1.9) — every angular command still got capped back down
  to 1.0 here regardless, which is why the robot kept turning slowly even after every
  Nav2-side speed increase. `linear.x`/`linear.y` still go through this file's own nonlinear
  `multiply_and_limit` reshaping curve, untouched — live speed measurements there came out
  too noisy/contaminated (a stale Nav2 goal was still executing server-side from an earlier
  killed test script) to safely recalibrate that one.
- **`quadropted_controller/RobotController/RobotController.py`** —
  `velocity_callback` now zeroes `angular.x`/`angular.y` (roll/pitch) before they reach the
  gait controller. They were never meant to be user-commandable: `TrotStanceController
  .position_delta()` integrates them straight into stance foot position every tick with **no
  spring-back to zero**, and the idle check (`np.all(command.yaw_rate == 0)`) treats any
  nonzero roll/pitch as "must be walking" — so a held pitch command drove continuous,
  non-recovering foot drift indistinguishable from walking forward/backward. They're wired
  only for small automatic IMU-stabilization feedback (`use_imu` / `pid_controller.run
  (imu_roll, imu_pitch)`), not raw input. `linear.z` (otherwise completely dead downstream —
  `command.velocity[2]` is never read by any gait controller) was repurposed as a real,
  working stance-height adjust instead, which is what `keyboard_teleop.py`'s up/down arrows
  now drive.

`src/kiss-icp` (git submodule, see **KISS-ICP** above) is
[PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp) used **unpatched** — worth noting
given the contrast with `go2_ros2_sim_py` above.

Also, not an upstream patch but worth knowing: **`office_sim.launch.py`'s odometry node sets
`base_frame_id: "base_link"`**, not upstream's own default of `"base"`. `"base"` is a bare
string nothing else in the URDF's kinematic tree connects to — the tree is rooted at
`base_link` — so `odom -> base` was a dead end, and no sensor frame could ever be
transformed into `odom`/`map`. This one cost an entire debugging session before being
traced with a live TF dump.

---

## Known issues

- ~~Robot sometimes tips over within a few seconds of spawning~~ **fixed.** Root cause
  found by watching actual joint commands, not just the body's pose: every leg joint spawns
  at 0 (fully extended) by default, but the IK gait controller commands its real
  ~0.86-1.89 rad standing-stance target from the very first control tick regardless — so
  position control snaps all 12 joints from 0 toward that very different target at once,
  while gravity is already acting on the body. Confirmed live via `joint_states` sampling
  that the *commanded* stance was already correct and static within about 1s of spawn, yet
  the *body* kept visibly tipping for several more seconds afterward — this idle stance has
  no active balance feedback at all (legs just hold a fixed angle relative to the trunk), so
  once that initial snap perturbed the body even slightly, nothing corrected it and physics
  alone finished the fall. Fixed by giving every leg joint an `initial_value` in
  `go2_description/xacro/gazebo.xacro` matching that exact stance (so there's no snap to
  begin with), paired with `office_sim.launch.py`'s `z_pose` raised to 0.30 (empirically
  confirmed, not simply `RobotController.py`'s documented 0.25m — that undershot the real
  geometric clearance for these joint angles and caused instant ground interpenetration).
  Confirmed live with both fixes together: settles to an essentially perfectly level pose
  within 1-2s and stays rock solid over 20+s of testing.
- **Nav2 bringup can stall on a loaded machine.** A lifecycle node's `change_state` service
  response can get lost at the DDS layer under CPU contention (confirmed live: a node
  finished configuring internally but `lifecycle_manager` never got the acknowledgment,
  logged as `failed to send response ... at rmw_response.cpp:153`), leaving bringup stuck
  on that one node forever — no timeout, no retry. Gazebo's GUI alone can pin ~2 cores, and
  the worst spike is right at cold-start when Gazebo + KISS-ICP + `slam_toolbox` + all of
  Nav2's ~10 nodes initialize simultaneously. If bringup stalls with a node stuck
  "Configuring" and no further log output, kill and retry `nav_stack.launch.py` after
  letting `office_sim.launch.py`/`kiss_icp.launch.py` settle for a few seconds first — see
  the Quick start note.
- **`slam_toolbox` keeps its whole map/pose-graph in memory only, no disk persistence** —
  if it falls behind real-time for long enough (its own message-filter queue log-spams
  `discarding message because the queue is full`) it can die and silently restart from a
  blank map, which looks exactly like the map "resetting" rather than a process crash.
  Mitigated (not eliminated) via `slam.yaml`'s `throttle_scans: 2` (halves its per-scan
  compute cost) and a larger `scan_queue_size` (bigger shock absorber for transient stalls
  like loop-closure searches) — worth revisiting if it recurs. Saving the map to disk
  periodically (`map_saver_cli`, not yet wired into a script) would make this recoverable
  instead of just less likely; see `docs/autonomous_exploration_plan.adoc`'s Step 5.
- **`frontier_explorer.py` and `record_bag.sh` are unimplemented placeholders**, not
  oversights — full design for both (frontier detection + goal selection loop,
  `nav2_simple_commander`-based; a straightforward `ros2 bag record` one-liner) is written
  up in `docs/autonomous_exploration_plan.adoc`, just not built yet. Manual Nav2 goals and
  manual-teleop SLAM both work fully today in the meantime.
- **The 3D lidar's scan pattern isn't physically representative of a real Livox Mid-360** —
  `gpu_lidar` gives a uniform raster, not Mid-360's non-repetitive rosette pattern. Point
  *density* looks different from a real unit; the geometry (FOV, range) is right and it's
  fine for SLAM.
- **No headless rendering.** `eglInitialize` fails against `/dev/dri/card1` on this machine
  (driver/headless EGL mismatch), so both the RGBD camera and lidar — being *rendering*
  sensors — require a real X display attached (`DISPLAY=:1` here is the machine's actual
  local screen, confirmed via `Xorg` on `seat0`/`tty2` — not a remote or virtual session).
  `gz sim -s` alone will not produce sensor data on this box.
- **Depth camera far clip is 25 m**, not a "real" RealSense spec value — it was originally
  10 m and got extended after depth rendering went blank specifically in the office's larger
  rooms. Root cause: rays exceeding the far clip return `+Inf`, and RViz's `Image` display
  auto-normalizes min/max across the whole frame — one `Inf` in that computation poisons the
  normalization constant for *every* pixel, not just the out-of-range ones. 25 m covers the
  room's full diagonal with margin.

---

## License

This project's own original work (`src/go2_office_sim/`, Blender authoring scripts,
documentation) is MIT-licensed — see [LICENSE](LICENSE). It builds on third-party code and
assets under different terms; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the
full picture, including a real gap worth knowing about: `go2_ros2_sim_py` upstream has no
license file at all.
