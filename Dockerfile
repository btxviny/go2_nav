# Verifies INSTALL.md end-to-end on a genuinely clean machine: clones the real
# published repo and runs through the same steps a new user would, checking that
# `colcon build` succeeds. Does NOT verify a running GUI session (Gazebo/RViz need a
# real X display) -- see INSTALL.md's "Verifying these instructions" section.
#
# Build:  docker build -t go2_nav_install_check .
# A successful build IS the verification -- there's nothing to run afterward.

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# --- INSTALL.md step 1: locale ---
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# --- INSTALL.md step 2: ROS 2 Jazzy ---
RUN apt-get install -y curl gnupg software-properties-common \
    && add-apt-repository universe \
    && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep

# --- INSTALL.md step 3: Gazebo Harmonic + project packages ---
RUN apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-pointcloud-to-laserscan ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
    ros-jazzy-twist-mux ros-jazzy-teleop-twist-keyboard ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-localization ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-ros2launch ros-jazzy-ros2run ros-jazzy-camera-info-manager ros-jazzy-rqt-robot-steering \
    ros-jazzy-tf-transformations ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
    ros-jazzy-sophus \
    git

# --- INSTALL.md step 4: clone the real published repo ---
RUN git clone --recurse-submodules https://github.com/btxviny/go2_nav.git /root/go2_nav

# --- INSTALL.md step 6: rosdep + build ---
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash \
    && rosdep init \
    && rosdep update \
    && cd /root/go2_nav \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install

CMD ["/bin/bash"]
