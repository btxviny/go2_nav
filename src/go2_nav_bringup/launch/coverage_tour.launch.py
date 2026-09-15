"""Hardcoded circular coverage tour -- launches scripts/coverage_tour.py as a proper node.

Independent of run_stack.sh/the main stack by design: run this on its own, in its own
terminal, after office_sim + odometry + nav_stack are already up (BasicNavigator.
waitUntilNav2Active() blocks until Nav2's lifecycle nodes are active, but the
map->base_link TF chain needs the rest of the stack running first too). It is not one
of run_stack.sh's flags -- coverage_tour.py owns its own bag recording (see its
docstring), so there is nothing else to launch alongside it.

    ros2 launch go2_nav_bringup coverage_tour.launch.py [bag_output:=/path/to/bag]

bag_output is optional -- omit it to let record_bag.sh pick its own default
(bags/<timestamp>).

Same GroupAction/PushRosNamespace/SetRemap wrapper as frontier_explorer.launch.py, for the
same reason: coverage_tour.py hardcodes absolute /tf, /tf_static (BasicNavigator's own TF
usage), so PushRosNamespace alone can't touch it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    namespace = '/robot1'

    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_output = LaunchConfiguration('bag_output')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_bag_output = DeclareLaunchArgument('bag_output', default_value='')

    coverage_tour_node = Node(
        package='go2_nav_bringup', executable='coverage_tour.py',
        name='coverage_tour', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'bag_output': bag_output}])

    coverage_tour_namespaced = GroupAction([
        PushRosNamespace(namespace),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        coverage_tour_node,
    ])

    return LaunchDescription([declare_use_sim_time, declare_bag_output, coverage_tour_namespaced])
