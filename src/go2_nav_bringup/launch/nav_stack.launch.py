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
    office_share = get_package_share_directory('go2_nav_bringup')

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
    from launch_ros.actions import PushRosNamespace, SetRemap
    from launch.actions import GroupAction
    # slam_toolbox itself hardcodes its map/map_metadata/tf publishers to
    # absolute topics in its own C++ source (confirmed live: `ros2 node info
    # /robot1/slam_toolbox` lists `/map`, not a namespace-relative `map`) --
    # PushRosNamespace alone can't touch an already-absolute topic string, so
    # without these SetRemaps slam_toolbox silently publishes at the root
    # namespace while everything downstream (global_costmap's static_layer,
    # this project's own remaps elsewhere e.g. rviz.launch.py) is listening
    # on /robot1/map and /robot1/tf -- global_costmap then never sees a map,
    # never gets a map->odom TF, and the planner hangs waiting on the `map`
    # frame forever. SetRemap is the standard launch_ros way to redirect a
    # hardcoded-absolute topic inside an included launch file we don't own.
    slam_namespaced = GroupAction([
        PushRosNamespace(namespace),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        SetRemap('/map', 'map'),
        SetRemap('/map_metadata', 'map_metadata'),
        slam,
    ])

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
    # navigation_launch.py never pushes a ROS namespace itself (normally
    # bringup_launch.py's job, which we bypass) -- without this wrapper every
    # Nav2 node runs at the root namespace while RewrittenYaml hands it
    # params keyed for /robot1/<node>, so nothing ever matches and every node
    # silently falls back to its internal defaults (this is the real cause of
    # the old "Couldn't load critics! No critics defined for FollowPath"
    # failure, not the bare-vs-fully-qualified-key theory in nav2_params.yaml).
    nav2_namespaced = GroupAction([PushRosNamespace(namespace), nav2])

    return LaunchDescription([
        declare_use_sim_time,
        pointcloud_to_laserscan,
        slam_namespaced,
        nav2_namespaced,
    ])
