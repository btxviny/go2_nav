# ROS2 basics, using this repo as the example

This is for a ROS2 newbie who wants to understand *why* this repo is laid out the way it
is, not just *how* to run it. Every concept below is explained using this project's own
real files, not a toy example — so once you get it, you already understand this repo. For
what this project's packages actually *do* (robot, sensors, navigation), see
[ARCHITECTURE.md](ARCHITECTURE.md); this file is about the ROS2/colcon machinery
underneath.

## Contents
- [The workspace: src/, build/, install/, log/](#the-workspace-src-build-install-log)
- [What a package is, and package.xml](#what-a-package-is-and-packagexml)
- [CMakeLists.txt — what actually gets built/installed](#cmakeliststxt--what-actually-gets-builtinstalled)
- [colcon build](#colcon-build)
- [setup.bash — why you have to "source" things](#setupbash--why-you-have-to-source-things)
- [Why kiss-icp is cloned into src/, but slam_toolbox and nav2 aren't](#why-kiss-icp-is-cloned-into-src-but-slam_toolbox-and-nav2-arent)
- [`ros2 run` and `ros2 launch`](#ros2-run-and-ros2-launch)
- [Common workflows](#common-workflows)

---

## The workspace: src/, build/, install/, log/

`~/go2_nav` is a **ROS2 workspace** — just a directory with a `src/` folder full of
packages, plus three more folders that `colcon build` (see below) generates for you:

```
go2_nav/
├── src/       your source code — the only one of these four you hand-edit or commit
├── build/     intermediate CMake/compiler working files, one subfolder per package
├── install/   the finished, ready-to-run result — what ros2 launch/run actually uses
└── log/       colcon's build logs, one timestamped folder per build
```

`build/`, `install/`, and `log/` are all **generated from `src/`** — colcon can rebuild
them from scratch at any time, so they're git-ignored (see `.gitignore`) and safe to
delete if something gets into a weird state:

```bash
rm -rf build install log
colcon build --symlink-install
```

This is a completely normal thing to do and fixes a surprising number of "it built before
and now it's broken" problems — you're not losing anything, just regenerating derived
files.

`src/` itself has one subfolder per **package** — this project's is deliberately flat
(see [ARCHITECTURE.md](ARCHITECTURE.md#layout)):

```
src/
├── go2_nav_bringup/          this project's own package
├── go2_description/         Go2 URDF/xacro
├── quadruped_controller/   the gait controller
├── quadruped_msgs/          custom message/service types
└── kiss-icp/ros/             lidar odometry (kiss-icp itself is a git submodule;
                               ros/ is the one subfolder of it that's an actual ROS package)
```

---

## What a package is, and package.xml

A ROS2 package is just: **a folder containing a file called `package.xml`.** That's the
entire definition — colcon finds packages by recursively searching `src/` for
`package.xml` files, however deeply nested they are. There's no registry, no central list
to edit; dropping a new folder with a `package.xml` into `src/` is enough to make it part
of the workspace.

Open [`src/go2_nav_bringup/package.xml`](../src/go2_nav_bringup/package.xml) and you'll see
the whole contract:

```xml
<package format="3">
  <name>go2_nav_bringup</name>
  <version>0.1.0</version>
  <description>...</description>
  <maintainer email="...">viny</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>go2_description</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  ...

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Piece by piece:
- **`<name>`** — must match the folder name by convention (not enforced, but everything
  assumes it — `ros2 launch go2_nav_bringup ...` looks up the package by this name, not by
  path).
- **`<buildtool_depend>ament_cmake</buildtool_depend>`** + **`<build_type>ament_cmake</build_type>`**
  — says "this package is built with a `CMakeLists.txt`". The other common value is
  `ament_python` (a `setup.py`-based package instead) — every package in this repo happens
  to be `ament_cmake`, even the pure-Python ones like `go2_nav_bringup` (its Python scripts
  are just installed as files by CMake — see below — rather than packaged the
  `ament_python` way).
- **`<exec_depend>`** — a *runtime* dependency: something that must be installed/built for
  this package to actually run, but isn't needed to compile it. `go2_nav_bringup` lists
  `go2_description`, `slam_toolbox`, `nav2_bringup`, etc. here because its launch files
  bring all of those up, even though `go2_nav_bringup` itself has no C++ code linking
  against them. This is exactly the list **`rosdep install --from-paths src --ignore-src -r -y`**
  reads to know what apt packages to install (see [INSTALL.md](INSTALL.md)) — that's
  the entire mechanism, there's no separate manually-maintained dependency list anywhere.
- **`<test_depend>`** — only needed for running tests/linting (`ament_lint_auto` etc.), not
  for normal use.

**A custom-message package's `package.xml` looks a bit different** — see
[`src/quadruped_msgs/package.xml`](../src/quadruped_msgs/package.xml): it adds
`<buildtool_depend>rosidl_default_generators</buildtool_depend>` and plain `<depend>`
tags (`geometry_msgs`, `std_msgs`) instead of `exec_depend`, because these dependencies are
needed at *build* time too — the `.msg`/`.srv` files get compiled into real C++/Python
code that `#include`s/`import`s those types directly (see CMakeLists.txt below).

---

## CMakeLists.txt — what actually gets built/installed

Every package here is `ament_cmake`, so every package also has a `CMakeLists.txt` — this
is the part that actually says what gets compiled and, just as importantly for a
Python-only package, **what gets installed and where**.
[`src/go2_nav_bringup/CMakeLists.txt`](../src/go2_nav_bringup/CMakeLists.txt) has no C++ at
all, and is a good minimal example:

```cmake
install(DIRECTORY
  worlds models config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/frontier_explorer.py
  scripts/keyboard_teleop.py
  scripts/send_nav_goal.py
  DESTINATION lib/${PROJECT_NAME}
)
```

Two different `install()` rules, two different destinations, and this distinction matters:
- **`worlds/`, `models/`, `config/`, `launch/`** go under `share/go2_nav_bringup/` — this is
  where `ros2 launch go2_nav_bringup office_sim.launch.py` finds the launch file, and where
  `get_package_share_directory('go2_nav_bringup')` (used all over this repo's launch files)
  resolves to. Non-executable data lives in `share/`.
- **The `.py` scripts** go under `lib/go2_nav_bringup/` — this is the convention for
  *executables*, whether they're compiled binaries or (as here) Python scripts with a
  `#!/usr/bin/env python3` shebang and the execute bit set. This is also exactly what
  `ros2 run <pkg> <executable>` looks up — see
  [`ros2 run` and `ros2 launch`](#ros2-run-and-ros2-launch) for what that gives you.

**A message package's CMakeLists.txt looks quite different** — see
[`src/quadruped_msgs/CMakeLists.txt`](../src/quadruped_msgs/CMakeLists.txt):
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotVelocity.msg" "msg/RobotGaitCommand.msg" ...
  "srv/RobotBehaviorCommand.srv"
  DEPENDENCIES geometry_msgs std_msgs
)
```
This one line is what turns a plain-text `.msg`/`.srv` file (just a list of field names and
types) into real, importable `RobotVelocity` Python/C++ classes during the build — which is
exactly why editing a `.msg` file requires a rebuild of that package before any code that
uses it will see the change (see [Common workflows](#common-workflows)).

---

## colcon build

`colcon build` is the tool that turns `src/` into `build/` + `install/`. Run from the
workspace root (`~/go2_nav`):

```bash
colcon build --symlink-install
```

What it does, in order:
1. **Discovers packages** — recursively scans `src/` for every `package.xml`.
2. **Figures out build order** from each package's dependencies (`quadruped_msgs` before
   `quadruped_controller`, since the controller imports the message types), and builds
   independent packages in parallel.
3. **For each package**, runs its build system (`cmake` + `make`/`ninja` for
   `ament_cmake`) in a scratch folder under `build/<package>/`, then installs the result
   into `install/<package>/`.

**`--symlink-install` matters a lot for a project like this one.** Without it, every
`install()`'d file is a real *copy* — so editing `src/go2_nav_bringup/launch/office_sim.launch.py`
would do nothing until you rebuilt. With it, `install/go2_nav_bringup/share/go2_nav_bringup/launch/office_sim.launch.py`
is a **symlink back into `src/`**, so edits to Python scripts, launch files, and
config/yaml/xacro files take effect on your *next run* — no rebuild needed. Only real
compiled code (C++, or anything CMake actually runs a compiler/generator on, like
`quadruped_msgs`'s `.msg` files) still needs a rebuild after editing. This project's own
[INSTALL.md](INSTALL.md) always uses `--symlink-install` for exactly this reason.

**Useful flags:**
- `--packages-select go2_nav_bringup` — build only that one package (and nothing else),
  much faster than rebuilding everything when you're iterating on one package.
- `--packages-up-to go2_nav_bringup` — build that package *and everything it depends on*,
  skipping packages nothing needs.
- Plain `colcon build` with no flags — (re)builds everything; colcon is smart enough to
  skip packages whose inputs haven't changed, so this is cheap to run often, not just the
  first time.

---

## setup.bash — why you have to "source" things

`colcon build` produces `install/setup.bash`, but building it isn't enough to *use* it —
your shell also needs to know where to find it. That's what "sourcing" does: it's not
running a program, it's loading a bunch of environment variables (`PATH`,
`AMENT_PREFIX_PATH`, `PYTHONPATH`, ...) into *your current shell*, so that commands like
`ros2 launch go2_nav_bringup ...` know which packages exist and where their files live.

There are actually **two layers**, sourced in order, and the order matters:

```bash
source /opt/ros/jazzy/setup.bash    # 1. the "underlay" — ROS2 itself + all the apt-installed
                                     #    packages (nav2_bringup, slam_toolbox, ros_gz_sim, ...)
source ~/go2_nav/install/setup.bash # 2. this workspace's "overlay" — layers go2_nav_bringup,
                                     #    go2_description, etc. on top
```

Each `setup.bash` internally sources the one "beneath" it, so step 2 alone actually pulls
in step 1 too — but only if step 1's install is the one it was built against. If you ever
see a package that "should" be there but isn't found, forgetting to source one of these two
(usually after opening a *new* terminal) is the most common cause.

This project's [INSTALL.md](INSTALL.md) has you add the underlay source line to
`~/.bashrc` once, so every new terminal gets `/opt/ros/jazzy/setup.bash` automatically —
but `~/go2_nav/install/setup.bash` (the overlay) still needs sourcing per-terminal after
each `colcon build`, unless you've also added that to your shell startup. If a terminal
was open *before* you last ran `colcon build`, its overlay is stale — open a fresh one or
re-source it.

---

## Why kiss-icp is cloned into src/, but slam_toolbox and nav2 aren't

This project depends on plenty of other ROS2 packages —
`slam_toolbox`, `nav2_bringup`, `navigation2`, `ros_gz_sim`, `pointcloud_to_laserscan`, and
more, all listed as `<exec_depend>`s in `go2_nav_bringup`'s `package.xml`. Only **one**
dependency, `kiss-icp`, actually has a folder under `src/`. Same underlay/overlay split
from [setup.bash](#setupbash--why-you-have-to-source-things) explains why, and it comes
down to one question for each dependency: **does a prebuilt binary package exist for it?**

- **`slam_toolbox`, `navigation2`, `nav2_bringup`, `ros_gz_sim`, etc. are all published as
  real `.deb` packages** on ROS2's package index — that's what
  `sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ...`
  in [INSTALL.md](INSTALL.md) actually installs. They land straight in the **underlay**
  (`/opt/ros/jazzy/`), fully built, already `source`d by `/opt/ros/jazzy/setup.bash`. This
  project never touches their source and never rebuilds them — `colcon build` doesn't even
  know they exist as source, only as things `rosdep`/apt already satisfied.
- **`kiss-icp` has no `ros-jazzy-kiss-icp` binary package at all** — check yourself with
  `apt-cache search kiss-icp` (comes back empty on this machine). PRBonn publishes it as
  source only. With no `.deb` to install, the only way to get it into this workspace is to
  **clone its source into `src/`** (as a git submodule here, since it's used unpatched —
  see [PATCHES.md](PATCHES.md)) and let `colcon build` compile it locally into this
  workspace's own `install/kiss_icp/`, i.e. the **overlay**.

Same logic explains the other vendored/subtree'd packages in `src/`
(`go2_description`, `quadruped_controller`, `quadruped_msgs`) — none of those has an
upstream binary release either, and on top of that this project actually **patches**
their source (see [PATCHES.md](PATCHES.md)), which would be impossible against a `.deb`
anyway. **The general rule:** if you can `apt install` it and never need to modify it, it
belongs in the underlay, not in `src/` — cloning a package into your workspace is for when
you need to build it yourself, whether because there's no binary release or because you're
patching it.

**The flip side of this rule, and a real example from this repo:** if a cloned package
turns out to satisfy *neither* reason — no patch needed, and nothing that actually runs
depends on it — it shouldn't be in `src/` at all. A fifth package, `gazebo_sim`, used to
sit alongside these three (all five came from the same upstream `go2_ros2_sim_py` repo,
before flattening — see [PATCHES.md](PATCHES.md)); it turned out to build no executable
and have no real runtime dependent anywhere in this project, just 136 MB of upstream's own
unused demo models along for the ride. It's been removed entirely rather than kept as a
fourth vendored package. Worth checking for on any package you're about to vendor: does
anything actually `get_package_share_directory()` or `<exec_depend>` it, or did it just
come along for the ride?

---

## `ros2 run` and `ros2 launch`

Both work in this project. `ros2 launch` always did — verified with `ros2 launch
go2_nav_bringup office_sim.launch.py --show-args`, which correctly lists that launch file's
arguments (`use_sim_time`, `x_pose`, `gz_args`, ...). `ros2 run` needed one more apt
package, since installed — verified live with `ros2 pkg executables go2_nav_bringup` and
`ros2 run go2_nav_bringup send_nav_goal.py --help`, both resolving correctly.

**Why `ros2 run` needed a separate install at all:** the `ros2` CLI is itself
plugin-based — every subcommand (`run`, `launch`, `pkg`, `topic`, `lifecycle`, ...) is its
own small apt package that registers a verb with the base `ros2cli` tool, rather than one
monolithic binary. `ros-jazzy-ros2launch` was already in
[INSTALL.md](INSTALL.md#3-gazebo-harmonic--project-packages)'s apt list (that's the only
reason `ros2 launch` worked from day one); `ros-jazzy-ros2run` has now been added right
next to it, in both `INSTALL.md` and the root `Dockerfile` that verifies it, so a fresh
clone gets both. `ros2 lifecycle` is the one verb still missing here — nothing installs
`ros-jazzy-ros2lifecycle`, since this project never drives Nav2's lifecycle nodes by hand
(`nav_stack.launch.py`'s own `lifecycle_manager` node does that automatically). Same fix
(`sudo apt install ros-jazzy-ros2lifecycle`) would apply if you ever need it.

**What `ros2 run` actually unlocks, concretely:**

- **Discoverability** — `ros2 pkg executables <pkg>` lists every runnable script/binary a
  package installs, without opening its `CMakeLists.txt` to check:
  ```
  $ ros2 pkg executables go2_nav_bringup
  go2_nav_bringup frontier_explorer.py
  go2_nav_bringup keyboard_teleop.py
  go2_nav_bringup record_bag.sh
  go2_nav_bringup run_stack.sh
  go2_nav_bringup send_nav_goal.py
  ```
- **Run by name, not by path** —
  `ros2 run go2_nav_bringup send_nav_goal.py --x 1.0 --y 0.0` instead of
  `python3 ~/go2_nav/src/go2_nav_bringup/scripts/send_nav_goal.py --x 1.0 --y 0.0`. Same
  "resolves via the sourced ROS environment, not your cwd" property `ros2 launch` already
  has (see [Quick start](../README.md#quick-start)) — works from any directory, and it
  always runs the currently-built `install/` copy rather than whatever `python3 <path>`
  happened to point at.
- **ROS's own generic CLI flags become available**, layered on top of whatever argparse
  flags a script defines itself — `ros2 run <pkg> <exe> --ros-args -r __ns:=/robot2 -p
  some_param:=1.0` overrides a node's namespace, name, parameters, or topic remaps from
  the command line, no launch-file edit required. A bare `python3 <path>` invocation has no
  idea what `--ros-args` even means; that flag only exists inside `ros2 run`/`ros2 launch`.
- **Every outside ROS2 tutorial or Stack Overflow answer assumes `ros2 run` works** — the
  main practical payoff for a newbie specifically: generic ROS2 learning material now
  applies to this repo's own packages verbatim, no mental translation into
  `python3 <path>` required.

None of this project's own launch files or scripts actually *require* `ros2 run` —
`office_sim.launch.py` and friends use `Node(...)` actions internally regardless, a third
mechanism separate from both `ros2 run` and calling a script directly. Installing
`ros2run` is a convenience for you at the terminal, not something this project depends on.

---

## Common workflows

**Fresh clone, first build:**
```bash
cd ~/go2_nav
rosdep install --from-paths src --ignore-src -r -y   # reads every package.xml's exec_depend
colcon build --symlink-install
source install/setup.bash    # or open a new terminal, if the underlay is in ~/.bashrc
```

**Edited a Python script, launch file, or config/yaml/xacro file:**
Nothing to do — with `--symlink-install`, the change is live on your *next* `ros2 launch`.
No rebuild, no re-source.

**Edited a C++ file, `CMakeLists.txt`, or `package.xml`:**
```bash
colcon build --symlink-install --packages-select <that package>
```
CMake needs to re-run and/or recompile; symlinks alone don't help here since something
actually has to be regenerated.

**Edited a `.msg` or `.srv` file** (e.g. `src/quadruped_msgs/msg/RobotVelocity.msg`):
Same as above — `rosidl_generate_interfaces` has to regenerate the Python/C++ classes.
Rebuild `quadruped_msgs`, and then anything that imports those types (`--packages-select
quadruped_msgs quadruped_controller`, or just plain `colcon build` to be safe) so it
picks up the new generated code.

**Added a new Python script to an existing package:**
Add an `install(PROGRAMS scripts/your_new_script.py ...)` entry to that package's
`CMakeLists.txt` (see `go2_nav_bringup`'s, above), then rebuild that package. Forgetting
this step is the classic "I added the file but `ros2 launch`/`python3 <path>` can't find
it in `install/`" bug — the file has to be `install()`'d explicitly, it doesn't happen
automatically just by existing in `src/`.

**Added a brand new package:**
Just `colcon build` — no flags needed. Since colcon discovers packages by scanning for
`package.xml`, a new folder under `src/` with one is picked up automatically on the next
build. Run `rosdep install` first if it declares any new dependencies.

**Something's broken and you're not sure why:**
```bash
rm -rf build install log
colcon build --symlink-install
```
Always safe (see [The workspace](#the-workspace-src-build-install-log)) — these three
folders are 100% derived from `src/`.

**Inspecting what's actually installed/available**, useful when a launch/import fails:
```bash
ros2 pkg list                     # every package colcon currently knows about
ros2 pkg prefix go2_nav_bringup    # where a specific package actually installed to
ros2 pkg executables go2_nav_bringup   # every runnable script/binary that package installs
ros2 launch go2_nav_bringup office_sim.launch.py --show-args   # what args a launch file takes
ros2 node list                    # nodes actually running right now
ros2 topic list                   # topics actually being published right now
```

**Running a single node/script directly**, instead of the full stack via `ros2 launch`
(see [`ros2 run` and `ros2 launch`](#ros2-run-and-ros2-launch) for what this gives you over
calling the script with `python3` directly):
```bash
ros2 run go2_nav_bringup send_nav_goal.py --x 1.0 --y 0.0
```
(`ros2 lifecycle` is the one CLI verb still not installed in this project's setup — see
above — nothing here needs it, since `nav_stack.launch.py`'s own `lifecycle_manager`
drives Nav2's lifecycle nodes automatically.)
