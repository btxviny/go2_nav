"""Autonomous frontier exploration -- launches scripts/frontier_explorer.py as a proper node.

Run after nav_stack.launch.py is up and Nav2's lifecycle nodes are active (frontier_explorer.py
itself waits on that via BasicNavigator.waitUntilNav2Active(), but the map->base_link TF chain
needs to be alive too, which only happens once office_sim/odometry/nav_stack are all running).

frontier_explorer.py no longer namespaces or remaps /tf itself -- same division of
responsibility as slam_toolbox (nav_stack.launch.py) and kiss_icp_node (kiss_icp.launch.py):
it hardcodes absolute /tf, /tf_static, so PushRosNamespace alone can't touch it and this
GroupAction/SetRemap wrapper is required, not just cosmetic.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    namespace = '/robot1'

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # frontier_explorer.py's own tunables (frontier detection, info-gain/path-length
    # scoring, goal filtering, budgets, stuck recovery) -- see that file's config
    # dict for the full list and config/frontier_explorer.yaml for what each one
    # means and why. Same load-a-YAML-alongside-the-node pattern as kiss_icp.launch.py.
    default_config_file = os.path.join(
        get_package_share_directory('go2_nav_bringup'), 'config', 'frontier_explorer.yaml')

    frontier_explorer_node = Node(
        package='go2_nav_bringup', executable='frontier_explorer.py',
        name='frontier_explorer', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, default_config_file])

    frontier_explorer_namespaced = GroupAction([
        PushRosNamespace(namespace),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        frontier_explorer_node,
    ])

    return LaunchDescription([declare_use_sim_time, frontier_explorer_namespaced])
