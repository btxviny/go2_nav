"""spark_fast_lio (tightly-coupled LiDAR-IMU ESKF odometry) as an alternative to
KISS-ICP, selectable via run_stack.sh's --odom flag.

Same GroupAction([PushRosNamespace('/robot1'), SetRemap('/tf','tf'),
SetRemap('/tf_static','tf_static'), SetRemap(...), node]) pattern as
kiss_icp.launch.py -- spark_fast_lio's TransformBroadcaster/Buffer/Listener are
all constructed with no namespace of their own, and its `lidar`/`imu`/`odometry`
topics are plain relative names, so without this it would publish TF/odometry at
the root namespace instead of /robot1 and never see /robot1/tf_static for its own
base_frame->lidar_frame extrinsic lookup at startup.

Unlike kiss_icp, spark_fast_lio's own publishOdometry() already broadcasts TF in
the standard parent->child direction (map_frame -> child_frame_id, confirmed by
reading spark_fast_lio.cpp) -- no invert flag needed here.

`common.visualization_frame: "base"` (config/fast_lio.yaml) is what makes
child_frame_id resolve to base_frame ("base_link"), giving the same real
odom->base_link edge kiss_icp provides. `gravity_alignment.enable_gravity_alignment`
is explicitly forced False there (matching upstream's own
mapping_dcist_rrg.launch.yaml precedent for base-frame-connected setups): with
base_frame set, that subsystem otherwise blocks ALL odometry publishing until the
robot has moved for a while, which would make this backend not drop-in
interchangeable with KISS-ICP's immediate-from-stationary behavior. The separate,
still-needed IMU-init gravity-averaging fix (imu_processing.hpp's MAX_INI_COUNT,
bumped from 10 to 1000) is unrelated to this and is baked into the vendored
source directly -- see src/spark-fast-lio/spark_fast_lio/include/imu_processing.hpp.

spark_fast_lio's own point-cloud preprocessing only ships handlers for Livox AVIA
(needs a custom message type we don't have) and VELO16/OUST64/KMOUST64 (need
per-point ring/time fields this project's gz-sim gpu_lidar doesn't publish -- see
config/fast_lio.yaml's `preprocess.lidar_type: 5` comment). `GENERIC` (added by
this project's own patch to preprocess.h/preprocess.cpp) is what makes this sensor
usable at all.
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
        get_package_share_directory('go2_nav_bringup'), 'config', 'fast_lio.yaml')

    fast_lio_node = Node(
        package='spark_fast_lio', executable='spark_lio_mapping', name='spark_lio_mapping',
        output='screen',
        remappings=[
            ('lidar', '/robot1/lidar/points'),
            ('imu', '/robot1/imu_plugin/out'),
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            default_config_file,
        ])
    # same class of fix kiss_icp.launch.py needed: spark_lio_mapping hardcodes
    # /tf absolute and isn't namespaced by its own launch file, so without this
    # it can't see /robot1/tf_static (needed for its own base_frame->lidar_frame
    # lookup) and would publish at the root namespace instead of /robot1.
    fast_lio_namespaced = GroupAction([
        PushRosNamespace(namespace),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        # relative, not '/odometry': spark_fast_lio's create_publisher call uses
        # a relative topic name (same class of gotcha as kiss_icp's
        # 'kiss/odometry').
        SetRemap('odometry', 'odom'),
        fast_lio_node,
    ])

    return LaunchDescription([declare_use_sim_time, fast_lio_namespaced])
