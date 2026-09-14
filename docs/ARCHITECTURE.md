# Architecture

Full annotated repo layout, then how the scene/robot/sensors/pipeline fit together. See the
[README](../README.md) for quick start and launch commands.

## Layout

```
go2_nav/
├── README.md                    quick start + pointers into docs/
├── blender/                     scene authoring (Blender project) — see blender/README.md
│   ├── office.blend
│   ├── build_office.py          generates office.blend + scene_objects.json from scratch
│   ├── export_sdf.py            exports office.blend -> src/go2_office_sim/{models,worlds}
│   ├── scene_objects.json       semantic map: rooms, landmarks, labels, poses
│   └── assets/                  Poly Haven source models (CC0, not committed — see .gitignore)
├── src/                         one folder per ROS package — standard colcon workspace layout
│   ├── go2_office_sim/          this project's own package: world, robot bringup, SLAM/Nav2 glue
│   │   ├── models/office_scene/ exported office (glTF meshes + SDF), from blender/export_sdf.py
│   │   ├── worlds/office.sdf
│   │   ├── launch/
│   │   │   ├── office_sim.launch.py   world + robot + sensors + locomotion — ✅ working
│   │   │   ├── rviz.launch.py         RViz with the /robot1/tf remap it needs — ✅ working
│   │   │   ├── kiss_icp.launch.py     KISS-ICP odometry, real odom->base_link source — ✅ working
│   │   │   └── nav_stack.launch.py    SLAM + Nav2 — ✅ both working
│   │   ├── config/
│   │   │   ├── bridge.yaml                   ros_gz_bridge: RGBD camera + 3D lidar topics
│   │   │   ├── slam.yaml                     slam_toolbox (async)
│   │   │   ├── pointcloud_to_laserscan.yaml  3D lidar -> 2D /scan for SLAM + Nav2
│   │   │   ├── nav2_params.yaml              Nav2 stack config — ✅ working
│   │   │   └── office.rviz                   robot model, camera, lidar, map, costmaps,
│   │   │                                      odom trajectory, KISS-ICP accumulated cloud,
│   │   │                                      "2D Nav Goal" tool
│   │   └── scripts/
│   │       ├── keyboard_teleop.py     WASD + arrows teleop — ✅ working
│   │       ├── send_nav_goal.py       CLI NavigateToPose action client — ✅ working
│   │       ├── frontier_explorer.py   autonomous exploration — ⬜ not implemented yet
│   │       └── record_bag.sh          rosbag recording — ⬜ not implemented yet
│   ├── go2_description/         Go2 URDF/xacro (meshes, joints, sensors) — vendored + patched
│   ├── gazebo_sim/              upstream's own Gazebo bringup/launch helpers — vendored, unpatched
│   ├── quadropted_controller/   IK trot-gait controller: cmd_vel -> 12 leg joints — vendored + patched
│   ├── quadropted_msgs/         custom msg/srv types the controller uses — vendored, unpatched
│   └── kiss-icp/                lidar-only odometry (git submodule, unpatched)
│                                 ↑ the four packages above (except kiss-icp) all came from one
│                                   upstream repo, go2_ros2_sim_py — see PATCHES.md
├── docs/
│   ├── ARCHITECTURE.md          this file
│   ├── NAVIGATION.md            KISS-ICP odometry, sending Nav2 goals, legacy FAST-LIO writeup
│   ├── PATCHES.md               exactly what changed vs. upstream go2_ros2_sim_py, and why
│   ├── KNOWN_ISSUES.md          open bugs and their root causes
│   ├── autonomous_exploration_plan.adoc   the Nav2/KISS-ICP fix writeup + forward plan
│   │                                       for frontier_explorer.py — see KNOWN_ISSUES.md
│   ├── media/                   screenshots, gifs, the demo video
│   └── upstream/                original go2_ros2_sim_py README + media, kept for attribution
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
[go2_ros2_sim_py](https://github.com/abutalipovvv/go2_ros2_sim_py) by abutalipovvv —
originally one upstream repo, vendored via `git subtree` with its original commit history
preserved and our own changes on top (see [PATCHES.md](PATCHES.md) for what changed), then
flattened into four top-level packages here for a standard workspace layout:
`src/go2_description` (URDF/xacro), `src/gazebo_sim` (Gazebo bringup helpers),
`src/quadropted_controller` (the IK trot-gait controller), and `src/quadropted_msgs` (its
custom message types). The controller walks the robot from `cmd_vel` — a real gait, not a
kinematic teleport. It also ships its own leg-kinematic odometry node
(`QuadrupedOdometryNode.py`), still running (renamed to publish `/robot1/odom_leg`) as a
comparison/fallback source, but it is **not** what SLAM/Nav2 actually consume — see
[Navigation](NAVIGATION.md#kiss-icp-real-odom-source) for the real `/robot1/odom` source.
The upstream repo's own `go1_description/` package (unused — Go1 isn't the robot here, and
it also depended on Classic Gazebo packages Jazzy dropped) and `docker/` (its own
dev-container setup, superseded by this project's root `Dockerfile`) were dropped from the
build entirely — kept for reference under [`docs/upstream/`](upstream/).

**Sensors.** The upstream xacro already wired up a front camera and a lidar with native
gz-sim syntax; both were modified in place: the camera became a **RGBD** sensor
(848×480, 87° HFOV, 15 Hz) on the head facing forward, and the lidar was repositioned to a
**back mount** and upgraded to a real **3D** scan (900 azimuth × 40 elevation samples,
−7°…+52°, 0.1–15 m, 7 Hz). Both bridge to ROS via `config/bridge.yaml`.

**Sensor realism.** The simulated sensors started out effectively noise-free and
range-unlimited, which doesn't generalize to real hardware — both are now modelled, each
value cited to a real spec rather than guessed:
- **Lidar** — Gaussian range noise, stddev 0.02 m (Livox Mid-360's own published range
  precision, `livoxtech.com/mid-360/specs`), applied directly in `gazebo.xacro` since
  gz-sim's own lidar `<noise>`/`<range><max>` tags are confirmed to work correctly on this
  build (rays past `max` come back as literal `+Inf`). Max range cut from the datasheet's
  40 m down to 15 m to match `nav2_params.yaml`'s own `obstacle_max_range` (both costmaps
  already ignore returns past that), not because 40 m was unrealistic indoors.
- **IMU** — Gaussian noise + a fixed turn-on bias per axis on both angular velocity and
  linear acceleration, gz-sim's own official sensor-noise tutorial values (a generic MEMS
  IMU reference, not this project's own hardware — see `gazebo.xacro`'s comment). Previously
  the IMU had no noise model at all, which is part of why FAST-LIO's residual bias (see
  [NAVIGATION.md](NAVIGATION.md#fast-lio-briefly-legacy)) could never be pinned on sensor
  input.
- **Depth camera** — not currently modelled: gz-sim's own `depth_camera` `<clip>` element is
  confirmed live to be silently ignored on this build (`/depth_image` and `/points` keep
  returning finite values well past it regardless), and SDF has no range-dependent noise or
  edge-artifact model for depth at all. Real depth realism (D435-like range clip, noise,
  silhouette-edge smearing) would need a downstream ROS node reprocessing the raw depth
  image/cloud — see `gazebo.xacro`'s camera sensor comment. Not implemented: nothing
  downstream consumes camera depth for SLAM/Nav2 (lidar is the real range source), only RViz
  visualization.

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
closure) rather than replacing it — see [NAVIGATION.md](NAVIGATION.md#kiss-icp-real-odom-source)
for why, and `autonomous_exploration_plan.adoc` for the full writeup of how this was wired
up and the three real bugs that had to be fixed to get Nav2 working at all.
