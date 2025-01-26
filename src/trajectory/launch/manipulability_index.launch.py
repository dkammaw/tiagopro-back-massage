from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_cpp',
            executable='manipulability',
            name='manipulability',
            output='screen'
        ),
        Node(
            package='trajectory',
            executable='state_receiver',
            name='state_receiver',
            output='screen'
        )
    ])