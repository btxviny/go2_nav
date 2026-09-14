# Patches to upstream

What changed relative to the two vendored/submoduled upstream projects, and why. See the
[README](../README.md) for the overall project and [ARCHITECTURE.md](ARCHITECTURE.md) for
where these packages sit in the repo layout.

`src/go2_description`, `src/gazebo_sim`, `src/quadropted_controller`, and
`src/quadropted_msgs` are vendored from
[go2_ros2_sim_py](https://github.com/abutalipovvv/go2_ros2_sim_py) by abutalipovvv —
originally one nested `src/go2_ros2_sim_py/` folder holding all four packages, flattened
into these top-level packages for a standard colcon workspace layout (see
[ARCHITECTURE.md](ARCHITECTURE.md)). Commit history is preserved via `git subtree` with our
changes on top as a single commit — `git log -- src/go2_description src/gazebo_sim
src/quadropted_controller src/quadropted_msgs` shows the full history, our patch included.
The changes themselves are small, and none of them are project-specific enough to warrant
maintaining a parallel xacro. Summary:

- **`go2_description/package.xml`** — removed a dead `gazebo_plugins` (Classic Gazebo)
  dependency that isn't used anywhere in the actual build (no `find_package` call for it)
  and doesn't exist for Jazzy, which dropped Classic support. Was blocking `rosdep`.
- **`go2_description/xacro/robot.xacro`** — repositioned the lidar joint from front
  (0.22, 0, 0.095) to back-mounted (−0.26, 0, 0.14); enlarged its visual/collision cylinder
  to roughly Livox Mid-360 proportions.
- **`go2_description/xacro/gazebo.xacro`** — camera sensor type `camera` → `rgbd_camera`
  (848×480, 87° HFOV, 15 Hz, depth clip 0.2–25 m — see the Inf/large-room note in
  [KNOWN_ISSUES.md](KNOWN_ISSUES.md)); lidar upgraded from a single-ring 2D scan to a real
  3D scan (900×40 samples, −7°…+52°, range 0.1–25 m); topic renamed `scan`→`lidar` since
  gz-sim's raw multi-ring range array is no longer a valid 2D `LaserScan` once there's more
  than one elevation ring — `/points` off that topic is the real payload. Also added an
  `initial_value` to every leg joint's `ros2_control` position state, matching
  `RobotController.py`'s real standing-stance target — the actual fix for the spawn
  tip-over bug (see [KNOWN_ISSUES.md](KNOWN_ISSUES.md)); without it every joint spawns at 0
  and gets snapped toward the real target the instant the gait controller activates, with
  gravity already acting on the body.
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

`src/kiss-icp` (git submodule, see [NAVIGATION.md](NAVIGATION.md#kiss-icp-real-odom-source))
is [PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp) used **unpatched** — worth noting
given the contrast with the vendored packages above.

Also, not an upstream patch but worth knowing: **`office_sim.launch.py`'s odometry node sets
`base_frame_id: "base_link"`**, not upstream's own default of `"base"`. `"base"` is a bare
string nothing else in the URDF's kinematic tree connects to — the tree is rooted at
`base_link` — so `odom -> base` was a dead end, and no sensor frame could ever be
transformed into `odom`/`map`. This one cost an entire debugging session before being
traced with a live TF dump.
