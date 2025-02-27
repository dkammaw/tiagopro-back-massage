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
            package='joint_interpolation',
            executable='joint_interpolator',
            name='joint_interpolator',
            output='screen'
        ),
        Node(
            package='back_detection',
            executable='state_receiver',
            name='state_receiver',
            output='screen'
        )
    ])
