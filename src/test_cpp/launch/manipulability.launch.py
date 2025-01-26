from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_cpp',
            executable='manipulability',
            output='screen',
            parameters=[
                {'use_sim_time': True},  # Set use_sim_time for the node
            ]
        )
    ])
