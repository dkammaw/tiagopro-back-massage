import os
from os import environ, pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration


from dataclasses import dataclass
from launch_pal.arg_utils import LaunchArgumentsBase, CommonArgs
from launch_ros.actions import Node


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    ''


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    my_controller = Node(
        package="trajectory", 
        executable="tapping", 
        name="tapping",
        output="screen"
    )
    launch_description.add_action(my_controller)

    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
