from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory',
            executable='back_detector',
            name='back_detector',
            output='screen'
        ),
        Node(
            package='test_cpp',
            executable='test_interpolator',
            name='test_interpolator',
            output='screen'
        ),
        Node(
            package='trajectory',
            executable='state_receiver',
            name='state_receiver',
            output='screen'
        )
    ])
