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
    # Reverted back to upstream's 0.4: live testing this session showed the
    # tip-over isn't a spawn-height/timing problem at all (see the
    # investigation notes on `controller` below) -- pitch grows gradually
    # over ~2-3 real seconds and then freezes, well after any landing impact
    # would have happened, and lowering this value (tried 0.28) made it
    # measurably worse, not better. Real cause is still open; don't re-tune
    # this value again without new evidence.
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
    # A pause-until-controller-ready mechanism was tried and reverted here:
    # spawning with the world paused and unpausing only once
    # `quadruped_controller` started didn't help either -- pose sampled every
    # 0.5s after unpause showed pitch climbing smoothly from ~17deg to
    # ~118deg over about 3 real seconds before freezing, i.e. the robot was
    # actively walking itself over well after standing up, not reacting to a
    # bad landing. That rules out both spawn height and startup timing as the
    # cause. Most likely next place to look: whether the trot gait
    # controller's idle/autoRest logic (TrotGaitController.step()) is
    # actually engaging with zero commanded velocity, or whether the
    # default_stance foot-location geometry (Robot.__init__'s body/legs
    # parameters vs this project's actual URDF leg lengths) produces an
    # off-balance standing pose that topples once the position controller
    # finishes moving the joints to it. Not yet root-caused.

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
            # KISS-ICP (kiss_icp.launch.py) now owns the real odom->base_link
            # TF and the /robot1/odom topic -- see
            # docs/autonomous_exploration_plan.adoc's "Option A". This node is
            # kept running (not deleted) as a leg-kinematic comparison/fallback
            # source, renamed off the topic and with its TF broadcast disabled
            # so the two odometry sources don't fight over the same TF edge.
            'enable_odom_tf': False,
        }],
        remappings=remappings + [('odom', 'odom_leg')])

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
