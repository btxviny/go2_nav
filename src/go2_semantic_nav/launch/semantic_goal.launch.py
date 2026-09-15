"""Semantic text-query -> Nav2 goal -- launches scripts/semantic_goal.py as a proper node.

Run with the full stack up (office_sim + odometry + nav_stack) and a semantic map
already built by build_semantic_map.py:

    ros2 launch go2_semantic_nav semantic_goal.launch.py \\
        query:="orange cone" map_path:=/path/to/semantic_map.npz

Same GroupAction/PushRosNamespace/SetRemap wrapper as frontier_explorer.launch.py /
coverage_tour.launch.py, for the same reason: semantic_goal.py does a live
map->base_link TF lookup via tf2_ros.TransformListener, which always reads the
literal /tf, /tf_static topics regardless of node namespace.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    namespace = '/robot1'

    query = LaunchConfiguration('query')
    map_path = LaunchConfiguration('map_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_query = DeclareLaunchArgument('query')
    declare_map_path = DeclareLaunchArgument('map_path')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    semantic_goal_node = Node(
        package='go2_semantic_nav', executable='semantic_goal.py',
        name='semantic_goal', output='screen',
        arguments=['--query', query, '--map', map_path],
        parameters=[{'use_sim_time': use_sim_time}])

    semantic_goal_namespaced = GroupAction([
        PushRosNamespace(namespace),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        semantic_goal_node,
    ])

    return LaunchDescription([
        declare_query, declare_map_path, declare_use_sim_time, semantic_goal_namespaced,
    ])
