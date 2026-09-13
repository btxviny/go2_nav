# Go2 Office Navigation

A Unitree Go2 quadruped, simulated in Gazebo Harmonic inside a procedurally-generated
office built in Blender, carrying a head-mounted RGBD camera and a back-mounted 3D lidar.
Currently drivable by keyboard with full sensor visualization in RViz; SLAM mapping works
manually; autonomous Nav2-driven exploration is scaffolded but blocked on an upstream bug
(see [Known issues](#known-issues)).

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

# Terminal 2 — drive it (click into this terminal first for keyboard focus)
python3 ~/go2_nav/src/go2_office_sim/scripts/keyboard_teleop.py

# Terminal 3 — visualize the robot itself: robot model, TF, camera, lidar point cloud
ros2 launch go2_office_sim rviz.launch.py

# Terminal 4 — lidar odometry (KISS-ICP), off the back lidar only
# (opens its own RViz window too — see the KISS-ICP section below for why)
ros2 launch go2_office_sim kiss_icp.launch.py
```

Terminal 4 is optional — 1–3 alone give you a fully drivable, visualized robot.

Any directory works for these — `ros2 launch` and `rviz2` resolve via the sourced ROS
environment, not your cwd. A fresh terminal is ready automatically; see
[Installation](#installation) for the one-time setup that makes that true.

**Starting over cleanly:** if something's gotten into a bad state (a launch died half-way,
a stale process is holding a topic), kill everything project-related before relaunching:
```bash
pkill -9 -f "gz sim|ros_gz_bridge|quadropted_controller/lib|go2_office_sim|kiss_icp_node|rviz2"
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
│       │   ├── kiss_icp.launch.py     KISS-ICP (lidar-only odometry) — ✅ working
│       │   └── nav_stack.launch.py    SLAM + Nav2 — SLAM ✅, Nav2 controller 🔴 broken
│       ├── config/
│       │   ├── bridge.yaml                   ros_gz_bridge: RGBD camera + 3D lidar topics
│       │   ├── slam.yaml                     slam_toolbox (async)
│       │   ├── pointcloud_to_laserscan.yaml  3D lidar -> 2D /scan for SLAM
│       │   ├── nav2_params.yaml              Nav2 stack config (controller currently broken)
│       │   └── office.rviz                   robot model, TF, camera, lidar point cloud, map
│       └── scripts/
│           ├── keyboard_teleop.py     WASD + arrows teleop — ✅ working
│           ├── frontier_explorer.py   autonomous exploration — ⬜ not implemented (blocked on Nav2)
│           └── record_bag.sh          rosbag recording — ⬜ not implemented (blocked on Nav2)
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
(`quadropted_controller`) that walks the robot from `cmd_vel` and publishes real odometry —
not a kinematic teleport. `go1_description/` and `docker/` are excluded from the build via
`COLCON_IGNORE` (go1 depends on Classic Gazebo packages Jazzy dropped, and isn't used here).

**Sensors.** The upstream xacro already wired up a front camera and a lidar with native
gz-sim syntax; both were modified in place: the camera became a **RGBD** sensor
(848×480, 87° HFOV, 15 Hz) on the head facing forward, and the lidar was repositioned to a
**back mount** and upgraded to a real **3D** scan (900 azimuth × 40 elevation samples,
−7°…+52°, 0.1–25 m, 10 Hz). Both bridge to ROS via `config/bridge.yaml`.

**Namespace.** Everything (topics, TF, nodes) lives under `/robot1`, matching upstream's own
convention — `/robot1/cmd_vel`, `/robot1/odom`, `/robot1/tf`, `/robot1/camera/*`,
`/robot1/lidar/points`, etc.

**Pipeline** (working parts only — see [Known issues](#known-issues) for Nav2):

```
gz sim (office.sdf + Go2)
  ├─ rgbd_camera ──┐
  └─ gpu_lidar ────┴─> ros_gz_bridge ─┬─> /robot1/camera/{image,depth_image,points,camera_info}
                                       └─> /robot1/lidar/points
                                                 │
                           pointcloud_to_laserscan ─> /robot1/scan
                                                 │
                                    slam_toolbox (async) ─> /robot1/map, map->odom TF
                                                 │
                                          [Nav2 — blocked, see below]
```

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
| `ros2 launch go2_office_sim rviz.launch.py` | RViz — robot model, TF, camera image, 3D lidar cloud, map | ✅ |
| `ros2 launch go2_office_sim nav_stack.launch.py` | SLAM + pointcloud_to_laserscan + Nav2 | SLAM ✅, Nav2 🔴 |
| `ros2 launch go2_office_sim kiss_icp.launch.py` | Lidar-only odometry (KISS-ICP) off the back lidar, own RViz window included | ✅ |
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

## KISS-ICP (lidar-only odometry)

`src/kiss-icp` is a git submodule, an **unpatched** checkout of
[PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp). It's the lidar odometry this
project actually uses. An earlier attempt used
[spark-fast-lio](https://github.com/MIT-SPARK/spark-fast-lio) (tight IMU+lidar fusion)
instead; that's no longer part of this repo — see **FAST-LIO, briefly** near the end of
this file for why. KISS-ICP does pure lidar scan-matching odometry with **no IMU fusion at
all**, sidestepping that whole class of bug by construction — "a LiDAR odometry pipeline
that just works," per its own tagline.

Run it alongside `office_sim.launch.py`:

```bash
ros2 launch go2_office_sim kiss_icp.launch.py
```

This launches `kiss_icp_node` against `/robot1/lidar/points`, plus KISS-ICP's own
preconfigured RViz window by default (pass `visualize:=false` to skip it). The wrapper
(`launch/kiss_icp.launch.py`) just points it at our topic — no config yaml of our own
needed. `base_frame` and `lidar_odom_frame` are both deliberately left at their own
defaults (empty / `odom_lidar`): `base_frame` empty means KISS-ICP publishes directly in
the lidar's own frame rather than needing a TF lookup to `base_link` — simplest possible
setup, and precise base_link alignment isn't needed to judge whether the algorithm itself
tracks cleanly. `lidar_odom_frame` matters more than it looks: KISS-ICP's own bundled
`rviz/kiss_icp.rviz` has `Fixed Frame: odom_lidar` **hardcoded** — renaming this frame via
the launch argument without also editing that file produces `Frame[odom_lidar] does not
exist` and a black RViz window (confirmed the hard way; don't rename it unless you also
fix the rviz config). Same **separate-window** situation as FAST-LIO applies here too and
for the same reason (`odom_lidar` is yet another frame disconnected from this project's
own `odom`/`base_link` tree) — this is why its own RViz window is launched by default
rather than trying to fold it into `office.rviz`.

**Dependencies:** `ros-jazzy-sophus` and `robin-map-dev` — already covered by
Installation's existing `rosdep install --from-paths src --ignore-src -r -y`, nothing
extra to add there.

**No per-point `time` field** (same lidar situation FAST-LIO dealt with) — KISS-ICP's own
handling is simpler still: it just logs `Field 't', 'timestamp', 'time_stamp', or 'time'
does not exist. Disabling scan deskewing` and proceeds without deskewing. No synthesis
needed, nothing to work around.

**Verified working:** with the robot genuinely stationary, `/kiss/odometry`'s position
held at sub-millimetre noise and identity orientation (no tilt at all, over repeated
samples). Drove the robot forward with a 4 s, 0.3 m/s `cmd_vel` burst and confirmed
`/kiss/odometry` tracked ~0.12 m of real displacement, not just an idle topic. Resource
usage is light too: ~50 MB RSS, low CPU, no growth over time.

**No loop closure.** KISS-ICP is odometry only — each scan registers against a local voxel
map via ICP, and the reported pose is just the accumulated chain of those registrations.
There's no pose graph, no place recognition, nothing that notices "I've been here before."
Small per-registration errors compound like any dead-reckoning method: expect good
short-term tracking with drift accumulating over time/distance, not a globally consistent
map. `slam_toolbox` (already in this project, via `nav_stack.launch.py`) does have loop
closure, in 2D, if that's ever needed instead.

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
  that topic is the real payload.
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

- **Nav2's `controller_server` does not come up.** Every launch fails during configure with
  `Couldn't load critics! Caught exception: No critics defined for FollowPath`, and
  `lifecycle_manager` aborts the whole Nav2 bringup. This survived switching controller
  plugins entirely (DWB → MPPI → Regulated Pure Pursuit, the last of which has no "critics"
  concept at all and still failed the same way), and byte-for-byte matching
  `nav2_bringup`'s own stock reference config. The params file `nav2_bringup` actually hands
  to `controller_server` at runtime was captured and diffed against a config proven to work
  when fed to an isolated `controller_server` directly — **identical content**, yet the real
  pipeline still loads `dwb_core::DWBLocalPlanner` instead of the configured plugin. The
  difference is in the process invocation itself (nav2_bringup's `RewrittenYaml` launches
  `controller_server` with two separate `--ros-args` blocks on the command line;
  reproducing that exact invocation by hand to confirm was the next step, not yet done).
  Until this is resolved, `nav_stack.launch.py`'s Nav2 half — and therefore autonomous
  frontier exploration and the rosbag-of-an-autonomous-run goal — stays blocked.
  `frontier_explorer.py` and `record_bag.sh` are unimplemented placeholders for this reason,
  not oversights.
- **SLAM and `pointcloud_to_laserscan` work independently of Nav2** — drive manually with
  the keyboard teleop and `/robot1/map` builds live in RViz. This is a legitimate, working
  path to a rosbag of manual SLAM exploration today, if that's useful before Nav2 is fixed.
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
