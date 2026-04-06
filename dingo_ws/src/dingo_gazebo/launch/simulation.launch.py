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
    dingo_gazebo_share = get_package_share_directory('dingo_gazebo')
    dingo_share = get_package_share_directory('dingo')

    world_path = os.path.join(dingo_gazebo_share, 'launch', 'world', 'normal.world')
    ctrl_yaml_path = os.path.join(dingo_gazebo_share, 'config', 'dingo_gz_ros2_control.yaml')
    behaviors_yaml_path = os.path.join(dingo_share, 'config', 'behaviors.yaml')

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

    # -----------------------------
    # Launch arguments
    # -----------------------------
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_delay = LaunchConfiguration('spawn_delay')
    controller_startup_seconds = LaunchConfiguration('controller_startup_seconds')
    dingo_stack_startup_seconds = LaunchConfiguration('dingo_stack_startup_seconds')

    # -----------------------------
    # Gazebo
    # -----------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # -----------------------------
    # Robot description publisher
    # -----------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # -----------------------------
    # Gazebo <-> ROS clock bridge
    # -----------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen',
    )

    # -----------------------------
    # rosbridge for Android/WebSocket
    # -----------------------------
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
        }],
    )

    # -----------------------------
    # rosapi for topic/service discovery
    # -----------------------------
    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
    )

    # -----------------------------
    # Spawn robot into Gazebo
    # -----------------------------
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

    # -----------------------------
    # ros2_control controller config
    # -----------------------------
    ctrl_file = PathJoinSubstitution([
        FindPackageShare('dingo_gazebo'),
        'config',
        'dingo_gz_ros2_control.yaml',
    ])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout',
            '60',
        ],
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

    # Match initial_value in URDF so the model stands before driver takes over
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

    # -----------------------------
    # Dingo control stack
    # -----------------------------
    # dingo_driver.py:
    #   args = [is_sim, is_physical, use_imu]
    # In sim we want: is_sim=1, is_physical=0, use_imu=0
    dingo_driver = Node(
        package='dingo',
        executable='dingo_driver.py',
        name='dingo_driver',
        output='screen',
        arguments=['1', '0', '0'],
        parameters=[
            {
                'use_sim_time': True,
                'gz_leg_command_topic': '/dingo/trot_joint_commands',
            },
                   ],
    )

    dingo_mode_manager = Node(
        package='dingo_behaviors',
        executable='mode_manager.py',
        name='dingo_mode_manager',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
            }
        ],
    )

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
                'Increase if controllers remain unconfigured.'
            ),
        ),
        DeclareLaunchArgument(
            'dingo_stack_startup_seconds',
            default_value='10.0',
            description=(
                'Seconds after launch to start dingo_driver and dingo_mode_manager. '
                'Set later than controller startup so the controller is ready first.'
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

        # Start semantic control stack after controllers are up
        TimerAction(
            period=dingo_stack_startup_seconds,
            actions=[dingo_driver, dingo_mode_manager],
        ),

        gz_sim,
        robot_state_publisher,
        bridge,
        rosbridge,
        rosapi,
        delayed_spawn,
    ])