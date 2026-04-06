from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dingo_behaviors',
            executable='mode_manager.py',
            name='dingo_mode_manager',
            output='screen',
            parameters=[
                {
                    'poses_file': '',
                    'mode_topic': '/dingo_mode',
                    'raw_walk_topic': '/dingo/trot_joint_commands',
                    'output_topic': '/leg_joint_position_controller/commands',
                    'joint_states_topic': '/joint_states',
                    'transition_steps': 50,
                }
            ]
        )
    ])