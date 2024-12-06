from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Create the launch description
    return LaunchDescription([
        Node(
            package="trajectory", 
            executable="move", 
            name="move",
            output="screen"
         )
     ])