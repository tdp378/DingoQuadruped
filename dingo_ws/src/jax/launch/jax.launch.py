from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    is_sim = LaunchConfiguration('is_sim')
    is_physical = LaunchConfiguration('is_physical')
    use_joystick = LaunchConfiguration('use_joystick')
    use_keyboard = LaunchConfiguration('use_keyboard')
    use_imu = LaunchConfiguration('use_imu')

    return LaunchDescription([
        DeclareLaunchArgument('is_sim', default_value='0'),
        DeclareLaunchArgument('is_physical', default_value='1'),
        DeclareLaunchArgument('use_joystick', default_value='0'),
        DeclareLaunchArgument('use_keyboard', default_value='0'),
        DeclareLaunchArgument('use_imu', default_value='0'),

        Node(
            package='dingo_peripheral_interfacing',
            executable='jax_display_node.py',
            name='dingo_LCD_node',
            output='screen',
            condition=IfCondition(is_physical),
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'autorepeat_rate': 30.0}],
            condition=IfCondition(use_joystick),
        ),
        Node(
            package='dingo_input_interfacing',
            executable='Keyboard.py',
            name='keyboard_input_listener',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(use_keyboard),
        ),
        Node(
            package='dingo_behaviors',
            executable='mode_manager.py',
            name='dingo_mode_manager',
            output='screen',
        ),
        Node(
            package='dingo',
            executable='dingo_driver.py',
            name='dingo_driver',
            output='screen',
            arguments=[is_sim, is_physical, use_imu],
        ),
    ])
