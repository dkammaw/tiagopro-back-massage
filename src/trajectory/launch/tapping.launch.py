from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #Node(
            #package="trajectory", 
            #executable="impedance_gain_adjuster", 
            #name="impedance_gain_adjuster",
            #output="screen"
        #)
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

        