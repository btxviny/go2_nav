# Installation

A from-scratch walkthrough for a genuinely clean **Ubuntu 24.04 (Noble)** machine — no ROS,
no Gazebo, nothing pre-installed. This is the sequence verified in this project's own
Docker-based verification (see the end of this file); if you already have ROS 2 Jazzy and
Gazebo Harmonic set up, skip to [step 4](#4-clone-this-repo).

## 1. Locale

ROS 2 needs a UTF-8 locale. Skip if `locale` already reports `UTF-8`.

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## 2. ROS 2 Jazzy

```bash
sudo apt install -y curl gnupg
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
```

This is the current official method (a small `.deb` that installs the correct apt
keyring + sources entry) — see
[docs.ros.org's Jazzy install page](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
if this ever changes upstream.

## 3. Gazebo Harmonic + project packages

`ros-jazzy-navigation2`/`ros-jazzy-nav2-bringup`/`ros-jazzy-slam-toolbox` are for
`nav_stack.launch.py` — SLAM and Nav2 are both fully working; see the README's
"Nav2 (point-to-point navigation)" section for how to send it a goal once installed.

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
  ros-jazzy-twist-mux ros-jazzy-teleop-twist-keyboard ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-localization ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-ros2launch ros-jazzy-camera-info-manager ros-jazzy-rqt-robot-steering \
  ros-jazzy-tf-transformations ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
  ros-jazzy-sophus \
  git
```

`ros-jazzy-ros-gz` is Gazebo Harmonic (gz-sim 8) plus its ROS 2 bridge — that's the
"Gazebo" referenced throughout this project's docs.

`robin-map-dev` (needed by `kiss-icp`, see step 6) is intentionally **not** in this list —
`rosdep` picks it up automatically in step 6, which is exactly the point of using `rosdep`
rather than hand-maintaining every transitive dependency here.

## 4. Clone this repo

```bash
cd ~
git clone --recurse-submodules https://github.com/btxviny/go2_nav.git
cd go2_nav
```

`--recurse-submodules` matters: `src/kiss-icp` is a git submodule. (`src/go2_ros2_sim_py`
is not — its history is merged directly into this repo via `git subtree`, so it comes
along with a plain clone.) If you forgot the flag:

```bash
git submodule update --init --recursive
```

## 5. Source ROS 2 automatically

Add to `~/.bashrc` (skip if already present):

```bash
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f "$HOME/go2_nav/install/local_setup.bash" ]; then
    source "$HOME/go2_nav/install/local_setup.bash"
fi
```

Open a fresh terminal after this (or `source ~/.bashrc`) so `ros2`, `colcon`, etc. resolve.

## 6. rosdep + build

```bash
sudo rosdep init   # skip if you've used rosdep on this machine before -- errors "already exists", harmless
rosdep update

cd ~/go2_nav
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

This is also where `kiss-icp`'s real dependencies get resolved automatically:
`ros-jazzy-sophus` (already in step 3's list for clarity, but rosdep would've caught it
too) and `robin-map-dev` (from Ubuntu's `universe` repo — enable it first if disabled:
`sudo add-apt-repository universe`).

`--symlink-install` means edits to Python scripts and config/launch/xacro files take
effect immediately on the next run — only C++/CMake changes need a rebuild.

## 7. Run it

See the [top-level README's Quick Start](README.md#quick-start).

---

## Verifying these instructions (Docker)

This exact sequence (steps 1–6, minus anything GUI-specific) is meant to be run inside a
clean `ubuntu:24.04` container to catch drift between this document and reality — a
missing apt package, a renamed upstream URL, a rosdep key that stopped resolving. It only
verifies that `colcon build` succeeds, not a running GUI session (Gazebo/RViz rendering
needs a real X display and has its own machine-specific quirks — see the README's Known
issues). See `Dockerfile` at the repo root.

```bash
docker build -t go2_nav_install_check .
```

A successful build means everything above actually works end-to-end on a machine that had
nothing but Ubuntu 24.04 on it.
