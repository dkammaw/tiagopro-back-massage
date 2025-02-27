from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartesian_interpolation',
            executable='nullspace_explorer',
            name='nullspace_explorer',
            output='screen',
        ),
        Node(
            package='back_detection',
            executable='state_receiver',
            name='state_receiver',
            output='screen'
        )
    ])