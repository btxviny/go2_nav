"""KISS-ICP (lidar-only scan-matching odometry) against the Go2's back lidar.

Chosen as a replacement for spark-fast-lio (src/spark-fast-lio) after an unresolved
ESKF init bug there caused a small but persistent orientation/position bias with no
IMU noise to blame (confirmed: raw /robot1/imu_plugin/out sampled continuously through
the whole init window was bit-identical, no noise model active at all -- see README's
FAST-LIO section / Known issues). KISS-ICP does pure lidar scan-matching, no IMU fusion
at all, sidestepping that entire class of bug by construction.

This is a thin wrapper around kiss_icp's own ros/launch/odometry.launch.py with this
project's lidar topic wired in and its own rviz window enabled by default (same
standalone-window pattern as fastlio.rviz -- KISS-ICP's own "odom_lidar" frame is
independent of this project's odom/base_link tree, same root cause as always: see
rviz.launch.py's docstring). base_frame is deliberately left at its default (empty),
so KISS-ICP publishes odometry directly in the lidar's own frame rather than needing a
TF lookup to base_link -- simplest possible setup, and precise base_link alignment
isn't needed to judge whether the algorithm itself tracks cleanly.

lidar_odom_frame is deliberately left at its own default ("odom_lidar") too, even though
it's an independent frame that could in principle be renamed without conflicting with
anything -- kiss_icp's own bundled rviz/kiss_icp.rviz has "Fixed Frame: odom_lidar"
hardcoded, and overriding the frame name here without also touching that file produces
"Frame[odom_lidar] does not exist" and a black RViz window (confirmed the hard way).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    visualize = LaunchConfiguration('visualize')
    declare_visualize = DeclareLaunchArgument(
        'visualize', default_value='true',
        description="Launch KISS-ICP's own preconfigured RViz window")

    kiss_icp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('kiss_icp'), 'launch', 'odometry.launch.py')]),
        launch_arguments={
            'topic': '/robot1/lidar/points',
            'use_sim_time': use_sim_time,
            'visualize': visualize,
        }.items())

    return LaunchDescription([declare_use_sim_time, declare_visualize, kiss_icp])
