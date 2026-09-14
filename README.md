# Go2 Office Navigation

![Go2 driving around the simulated office](docs/media/go2_nav_office.gif)

A Unitree Go2 quadruped, simulated in Gazebo Harmonic inside a procedurally-generated
office built in Blender, carrying a head-mounted RGBD camera and a back-mounted 3D lidar.
Drivable by keyboard with full sensor visualization in RViz; SLAM mapping and **Nav2
point-to-point navigation both work** (KISS-ICP drives the real odometry — see
[Navigation](docs/NAVIGATION.md)); you can send it a goal by clicking in RViz or from the
command line (see [Navigation](docs/NAVIGATION.md#nav2-point-to-point-navigation)). Fully
autonomous frontier exploration (the robot picking its own goals to map the whole scene
unattended) is not implemented yet — see [Known issues](docs/KNOWN_ISSUES.md) for that and
for an open gait-stability bug.

```
Blender scene  ──export_sdf.py──>  Gazebo world + robot  ──sensors + TF──>  RViz / rosbag
(blender/)                          (src/go2_office_sim/)                   (visualize/record)
```

---

## Quick start

```bash
./src/go2_office_sim/scripts/run_stack.sh 
```

Or Each of these goes in its own terminal, left running:

```bash
# Terminal 1 — the simulation: world, robot, sensors, locomotion
ros2 launch go2_office_sim office_sim.launch.py

# Terminal 2 — lidar odometry (KISS-ICP) -- this is the real odom->base_link
# source now (see docs/NAVIGATION.md), so bring it up before SLAM/Nav2
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
[Navigation](docs/NAVIGATION.md#nav2-point-to-point-navigation) for how to actually send it
a goal once that's up (wait for its log to end with `Managed nodes are active` first, same
as any Nav2 bringup). **Nav2 nodes are heavier than they look on a loaded machine** — if
bringup stalls partway (a node stuck "Configuring" with no further log output), that's very
likely the machine falling behind under CPU load rather than something misconfigured;
letting Terminals 1-2 settle for a few seconds before starting Terminal 3 usually gives it
enough headroom.

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
├── README.md              this file
├── blender/               scene authoring (Blender project) — see blender/README.md
├── src/                    one folder per ROS package — standard colcon workspace layout
│   ├── go2_office_sim/     this project's own package — world, launch, config, scripts
│   ├── go2_description/    Go2 URDF/xacro — vendored + patched
│   ├── gazebo_sim/         upstream's own Gazebo bringup helpers — vendored, unpatched
│   ├── quadropted_controller/  IK trot-gait controller — vendored + patched
│   ├── quadropted_msgs/    custom msg/srv types for the controller — vendored, unpatched
│   └── kiss-icp/           lidar-only odometry (git submodule, unpatched)
├── docs/                   architecture, navigation, patches, known issues, media
├── bags/                   rosbag output directory
└── install/ build/ log/    colcon artifacts (generated, not source)
```

**Full annotated layout + architecture + data-flow pipeline diagram:
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).**

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

## Learn more

| Doc | Covers |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Full annotated repo layout; scene generation, robot, sensors, namespace, and the end-to-end data-flow pipeline |
| [docs/NAVIGATION.md](docs/NAVIGATION.md) | KISS-ICP odometry (the real `/robot1/odom` source), sending Nav2 goals, and the legacy FAST-LIO attempt it replaced |
| [docs/PATCHES.md](docs/PATCHES.md) | Exactly what changed vs. upstream `go2_ros2_sim_py`, and why |
| [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md) | Open bugs, with root causes |
| [docs/autonomous_exploration_plan.adoc](docs/autonomous_exploration_plan.adoc) | Design writeup for autonomous frontier exploration (not built yet) |
| [INSTALL.md](INSTALL.md) | Full from-scratch install walkthrough |
| [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) | Licensing for vendored/third-party code and assets |

---

## License

This project's own original work (`src/go2_office_sim/`, Blender authoring scripts,
documentation) is MIT-licensed — see [LICENSE](LICENSE). It builds on third-party code and
assets under different terms; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the
full picture, including a real gap worth knowing about: the vendored `go2_ros2_sim_py`
packages' upstream has no license file at all.
