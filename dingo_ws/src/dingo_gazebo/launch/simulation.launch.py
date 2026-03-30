"""Gazebo Sim + Dingo URDF spawn (ROS 2 / gz-sim).

Starts gz with ``normal.world``, publishes ``robot_description``, spawns the model via
``ros_gz_sim create``, bridges sim clock, loads ``gz_ros2_control`` + position controllers
so the quadruped does not collapse under gravity.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = get_package_share_directory('dingo_description')
    gz_model_path = os.path.dirname(desc_share)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    dingo_share = get_package_share_directory('dingo_gazebo')
    world_path = os.path.join(dingo_share, 'launch', 'world', 'normal.world')
    ctrl_yaml_path = os.path.join(dingo_share, 'config', 'dingo_gz_ros2_control.yaml')

    robot_description_path = os.path.join(desc_share, 'urdf', 'dingo.urdf')
    robot_description_config = xacro.process_file(robot_description_path)
    desc_xml = robot_description_config.toxml().replace(
        '__DINGO_GZ_ROS2_CONTROL_YAML__',
        ctrl_yaml_path,
    )
    robot_description = {
        'robot_description': desc_xml,
        'use_sim_time': True,
    }

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    spawn_z = LaunchConfiguration('spawn_z')
    spawn_delay = LaunchConfiguration('spawn_delay')

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{
            'world': 'default',
            'name': 'dingo',
            'topic': 'robot_description',
            'x': 0.0,
            'y': 0.0,
            'z': spawn_z,
        }],
    )

    delayed_spawn = TimerAction(
        period=spawn_delay,
        actions=[spawn_entity],
    )

    ctrl_file = PathJoinSubstitution([
        FindPackageShare('dingo_gazebo'),
        'config',
        'dingo_gz_ros2_control.yaml',
    ])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    leg_joint_position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'leg_joint_position_controller',
            '--param-file',
            ctrl_file,
            '--controller-manager-timeout',
            '60',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Match initial_value in URDF (theta2 ~ 0.726 rad stand pose for all legs)
    pub_stand_pose = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'sleep 1.0 && ros2 topic pub --once /leg_joint_position_controller/commands '
            'std_msgs/msg/Float64MultiArray '
            '"{data: [0.0, 0.72646626, 0.0, 0.0, 0.72646626, 0.0, 0.0, 0.72646626, 0.0, 0.0, 0.72646626, 0.0]}"',
        ],
        output='screen',
    )

    # ros_gz_sim create often stays alive; OnProcessExit(create) never runs, so controllers
    # stayed unconfigured. Start spawners on a wall-clock delay after launch instead.
    controller_startup_seconds = LaunchConfiguration('controller_startup_seconds')

    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.45',
            description='Initial spawn height (m) above ground so the model clears the plane',
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='3.0',
            description='Seconds to wait before spawning (lets gz sim finish loading)',
        ),
        DeclareLaunchArgument(
            'controller_startup_seconds',
            default_value='8.0',
            description=(
                'Seconds after launch to spawn controllers (after gz + robot spawn). '
                'Increase if list_controllers shows unconfigured.'
            ),
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_model_path,
            prepend=True,
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[leg_joint_position_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_joint_position_spawner,
                on_exit=[pub_stand_pose],
            )
        ),
        TimerAction(
            period=controller_startup_seconds,
            actions=[joint_state_broadcaster_spawner],
        ),
        gz_sim,
        robot_state_publisher,
        bridge,
        delayed_spawn,
    ])
