from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Create the launch description
    return LaunchDescription([
        Node(
            package="trajectory", 
            executable="leftTapper", 
            name="leftTapper",
            output="screen"
         ),
        Node(
            package="trajectory", 
            executable="rightTapper", 
            name="rightTapper",
            output="screen"
        )
     ])
