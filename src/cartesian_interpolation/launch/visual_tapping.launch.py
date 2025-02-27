from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='back_detection',
            executable='back_detector',
            name='back_detector',
            output='screen'
        ),
        Node(
            package='cartesian_interpolation',
            executable='ik_solver',
            name='ik_solver',
            output='screen'
        )
    ])
