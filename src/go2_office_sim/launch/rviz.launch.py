"""RViz, remapped to the namespaced TF topics this setup actually publishes.

office_sim.launch.py's robot_state_publisher and odometry node both publish
under /robot1/tf and /robot1/tf_static (their remappings map the node's own
absolute /tf -> relative tf, resolved under the /robot1 namespace). RViz's
TF listener defaults to the bare, global /tf and /tf_static, which nothing in
this stack publishes to -- so every TF-dependent display (RobotModel, any
PointCloud2, the map) fails with "could not transform ... to odom" while
non-TF displays (a plain Image) work fine. This wrapper remaps rviz2 itself
rather than relying on everyone remembering the CLI flags.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('go2_office_sim'), 'config', 'office.rviz')

    rviz_config = LaunchConfiguration('rviz_config')
    declare_config = DeclareLaunchArgument(
        'rviz_config', default_value=default_config,
        description='Path to the .rviz config file')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', '/robot1/tf'),
            ('/tf_static', '/robot1/tf_static'),
        ],
        output='screen')

    return LaunchDescription([declare_config, declare_use_sim_time, rviz])
