"""KISS-ICP (lidar-only scan-matching odometry) against the Go2's back lidar.

Chosen as a replacement for spark-fast-lio (src/spark-fast-lio) after an unresolved
ESKF init bug there caused a small but persistent orientation/position bias with no
IMU noise to blame (confirmed: raw /robot1/imu_plugin/out sampled continuously through
the whole init window was bit-identical, no noise model active at all -- see README's
FAST-LIO section / Known issues). KISS-ICP does pure lidar scan-matching, no IMU fusion
at all, sidestepping that entire class of bug by construction.

As of docs/autonomous_exploration_plan.adoc's "Option A", KISS-ICP is wired directly
into this project's real odom->base_link TF and /robot1/odom topic -- it is no longer
a disconnected side pipeline. `lidar_odom_frame` is renamed from kiss_icp's own default
"odom_lidar" to this project's real "odom" frame, and `base_frame` is set to "base_link"
(kiss_icp looks up the static laser_frame->base_link transform itself and composes the
odometry into that frame). `QuadrupedOdometryNode` (office_sim.launch.py) is still
running alongside this for comparison, but its own TF broadcast is disabled and its
topic renamed to `odom_leg` so the two sources don't fight over the same TF edge.

`kiss_icp_node` is constructed directly here instead of including kiss_icp's own
`odometry.launch.py`: that upstream file ties its "visualize" argument to BOTH
launching its own bundled rviz2 window AND the `publish_debug_clouds` node parameter
(the thing that actually makes it publish `kiss/local_map` -- the accumulated point
cloud, the closest thing KISS-ICP has to "a map" since it has no occupancy-grid output
of its own). That bundled rviz window is permanently broken here anyway (its
`rviz/kiss_icp.rviz` hardcodes `Fixed Frame: odom_lidar`, which no longer exists once
`lidar_odom_frame` is renamed to "odom" -- confirmed the hard way earlier in this
project's history). We want the debug clouds published (so `office.rviz`'s own
`KissIcpLocalMap` display can show them) without that separate, broken window, and
upstream's launch file has no way to decouple the two -- so `kiss_icp_node` is run
directly with the same parameters/config file `odometry.launch.py` would have used,
just with `publish_debug_clouds` always on and no `rviz2` node at all. Not a patch to
kiss_icp itself (still an unpatched checkout, see README) -- just choosing not to use
its launch-file wrapper.

kiss_icp_node itself isn't given a namespace or any /tf remapping by its own upstream
package, so this project's usual GroupAction([PushRosNamespace(...), SetRemap(...)])
pattern (same one nav_stack.launch.py needed for slam_toolbox) is required here too --
otherwise kiss_icp_node can't see this project's own /robot1/tf_static (needed for its
laser_frame->base_link lookup) and would publish its own TF/odometry at the root
namespace instead of /robot1.
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

    default_config_file = os.path.join(
        get_package_share_directory('kiss_icp'), 'config', 'config.yaml')

    kiss_icp_node = Node(
        package='kiss_icp', executable='kiss_icp_node', name='kiss_icp_node',
        output='screen',
        remappings=[('pointcloud_topic', '/robot1/lidar/points')],
        parameters=[
            {
                'base_frame': 'base_link',
                'lidar_odom_frame': 'odom',
                'publish_odom_tf': True,
                # kiss_icp's own default (True) broadcasts child->parent
                # (base_link->odom, confirmed live via a TF dump) instead of
                # the standard parent->child direction everything else in
                # this project's tree uses (odom->base_link, as
                # slam_toolbox/Nav2 expect).
                'invert_odom_tf': False,
                # always on now (was tied to a "visualize" flag upstream) --
                # publishes kiss/{frame,keypoints,local_map} for
                # office.rviz's KissIcpLocalMap display; see module docstring.
                'publish_debug_clouds': True,
                'use_sim_time': use_sim_time,
                'position_covariance': 0.1,
                'orientation_covariance': 0.1,
            },
            default_config_file,
        ])
    # same class of fix nav_stack.launch.py needed for slam_toolbox: kiss_icp_node
    # hardcodes /tf absolute and isn't namespaced by its own launch file, so
    # without this it can't see /robot1/tf_static (needed for its own
    # laser_frame->base_link lookup) and would publish at the root namespace.
    kiss_icp_namespaced = GroupAction([
        PushRosNamespace(namespace),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        # relative, not "/kiss/odometry": kiss_icp_node's own create_publisher
        # call uses a relative topic name (confirmed live -- with the
        # absolute-form rule this never matched and the topic stayed at
        # /robot1/kiss/odometry instead of remapping to /robot1/odom).
        SetRemap('kiss/odometry', 'odom'),
        kiss_icp_node,
    ])

    return LaunchDescription([declare_use_sim_time, kiss_icp_namespaced])
