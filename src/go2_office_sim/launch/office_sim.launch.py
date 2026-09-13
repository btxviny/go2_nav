"""Spawn the Go2 (RGBD head camera + back-mounted 3D lidar) in the office world.

Based on go2_ros2_sim_py's gazebo_sim/launch/launch_sim.launch.py, with:
  - our office.sdf instead of empty/cafe.world
  - spawn pose at the entrance (0, -9), clear of walls/furniture (see plan:
    rasterized collision clearance = 1.00 m there)
  - GZ_SIM_RESOURCE_PATH extended so model://office_scene resolves
  - our bridge.yaml added for the RGBD camera + 3D lidar
  - rviz/Nav2-bringup left out; those are separate launch files (Phase 3/4)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    namespace = '/robot1'
    name = 'robot1'

    go2_desc_share = get_package_share_directory('go2_description')
    office_share = get_package_share_directory('go2_office_sim')

    xacro_file = os.path.join(go2_desc_share, 'xacro', 'robot.xacro')
    world = os.path.join(office_share, 'worlds', 'office.sdf')
    bridge_yaml = os.path.join(office_share, 'config', 'bridge.yaml')

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/scan", "scan"),
        ("/odom", "odom"),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time,
        description='Use /clock from Gazebo')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='-9.0')
    z_pose = LaunchConfiguration('z_pose', default='0.4')
    declare_x = DeclareLaunchArgument('x_pose', default_value=x_pose)
    declare_y = DeclareLaunchArgument('y_pose', default_value=y_pose)
    declare_z = DeclareLaunchArgument('z_pose', default_value=z_pose)

    # office_scene's meshes live under go2_office_sim/models; the office.sdf's
    # <include><uri>model://office_scene</uri></include> needs this on the path.
    gazebo_sim_share = get_package_share_directory('gazebo_sim')
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(office_share, 'models') + ':' +
              os.path.join(gazebo_sim_share, 'models') + ':' +
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v3 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim', executable='create', namespace=namespace,
        arguments=[
            '-topic', f'{namespace}/robot_description',
            '-name', f'{namespace}/my_bot',
            '-x', x_pose, '-y', y_pose, '-z', z_pose,
        ],
        output='screen')

    robot_desc = xacro.process_file(xacro_file, mappings={'robot_name': name}).toxml()
    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', namespace=namespace, parameters=[params], remappings=remappings)

    # upstream's own bridge: IMU, TF, joint_states. The lidar's raw multi-ring
    # /robot1/lidar array is deliberately not bridged here -- only /points
    # (in bridge.yaml) is; see the note in gazebo.xacro.
    upstream_bridge_args = [
        f"{namespace}/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU",
        f"{namespace}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
        f"{namespace}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
    ]
    ros_gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=namespace, arguments=upstream_bridge_args, output='screen')

    ros_gz_bridge_clock = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    # our sensors: RGBD camera + 3D lidar points
    ros_gz_bridge_sensors = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}'],
        output='screen')

    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        namespace=namespace, arguments=["joint_state_broadcaster"], remappings=remappings)

    joint_group_controller = Node(
        package="controller_manager", executable="spawner",
        namespace=namespace, arguments=["joint_group_controller"],
        output="screen", remappings=remappings)

    controller = Node(
        package='quadropted_controller', executable='robot_controller_gazebo.py',
        name='quadruped_controller', namespace=namespace,
        output='screen', remappings=remappings)

    cmd_vel_pub = Node(
        package='quadropted_controller', executable='cmd_vel_pub.py',
        name='cmd_vel_pub', namespace=namespace, output='screen')

    odom = Node(
        package='quadropted_controller', executable='QuadrupedOdometryNode.py',
        name='odom', namespace=namespace, output='screen',
        parameters=[{
            "verbose": False, 'publish_rate': 50, 'open_loop': False,
            'has_imu_heading': True, 'is_gazebo': True,
            'imu_topic': f"/{namespace}/imu", 'base_frame_id': "base_link",
            # NOT "base" (upstream's default): the URDF's kinematic tree is
            # rooted at base_link (see robot.xacro); "base" is a bare string
            # with nothing published under it, so odom->base was a dead end
            # disconnected from every sensor frame. Verified via a live TF
            # dump against this exact launch file -- base_link never appeared
            # as a child anywhere, so it had no parent to reach odom/map through.
            'odom_frame_id': "odom", 'clock_topic': '/clock',
            'enable_odom_tf': True,
        }],
        remappings=remappings)

    return LaunchDescription([
        declare_use_sim_time, declare_x, declare_y, declare_z,
        set_resource_path,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_bridge_clock,
        ros_gz_bridge_sensors,
        joint_state_broadcaster,
        joint_group_controller,
        controller,
        cmd_vel_pub,
        odom,
    ])
