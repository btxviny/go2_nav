"""SLAM + Nav2 for the Go2, layered on top of office_sim.launch.py.

Pipeline: /robot1/lidar/points --[pointcloud_to_laserscan]--> /robot1/scan
          --[slam_toolbox, async]--> /robot1/map + map->odom TF
          --[nav2_bringup navigation_launch.py]--> costmaps/planner/controller
          consuming /robot1/scan + /robot1/odom, commanding /robot1/cmd_vel.

Run office_sim.launch.py first (or alongside); this file assumes the robot is
already spawned and odom->base_link is already being published -- slam_toolbox
and Nav2 both need that TF chain alive before they can do anything useful.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = '/robot1'
    office_share = get_package_share_directory('go2_office_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    p2l_params = os.path.join(office_share, 'config', 'pointcloud_to_laserscan.yaml')
    slam_params = os.path.join(office_share, 'config', 'slam.yaml')
    nav2_params = os.path.join(office_share, 'config', 'nav2_params.yaml')

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node', namespace=namespace,
        parameters=[p2l_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', 'lidar/points'),   # -> /robot1/lidar/points
            ('scan', 'scan'),               # -> /robot1/scan
        ],
        output='screen')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time,
        }.items())
    # online_async_launch.py doesn't take a namespace arg; push one on instead
    from launch_ros.actions import PushRosNamespace
    from launch.actions import GroupAction
    slam_namespaced = GroupAction([PushRosNamespace(namespace), slam])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'True',
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'map_subscribe_transient_local': 'true',
        }.items())

    return LaunchDescription([
        declare_use_sim_time,
        pointcloud_to_laserscan,
        slam_namespaced,
        nav2,
    ])
