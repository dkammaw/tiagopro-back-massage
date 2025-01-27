from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_cpp',
            executable='redundancy_stepper',
            name='redundancy_stepper',
            output='screen',
        ),
        Node(
            package='trajectory',
            executable='state_receiver',
            name='state_receiver',
            output='screen'
        )
    ])
