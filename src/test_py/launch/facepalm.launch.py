import os
from os import environ, pathsep
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.actions import CheckPublicSim

from launch_pal.arg_utils import LaunchArgumentsBase, CommonArgs
from launch_pal.robot_arguments import TiagoProArgs
from dataclasses import dataclass
from launch_ros.actions import Node


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    '''base_type: DeclareLaunchArgument = TiagoProArgs.base_type
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoProArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoProArgs.ft_sensor_left
    wrist_model_right: DeclareLaunchArgument = TiagoProArgs.wrist_model_right
    wrist_model_left: DeclareLaunchArgument = TiagoProArgs.wrist_model_left
    camera_model: DeclareLaunchArgument = TiagoProArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoProArgs.laser_model

    slam: DeclareLaunchArgument = DeclareLaunchArgument(
        "slam", default_value="False", description="Specify if launching SLAM Toolbox")
    navigation: DeclareLaunchArgument = CommonArgs.navigation
    moveit: DeclareLaunchArgument = CommonArgs.moveit
    world_name: DeclareLaunchArgument = CommonArgs.world_name
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim'''


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    '''# Set use_sim_time to True
    set_sim_time = SetLaunchConfiguration("use_sim_time", "True")
    launch_description.add_action(set_sim_time)

    # Shows error if is_public_sim is not set to True when using public simulation
    public_sim_check = CheckPublicSim()
    launch_description.add_action(public_sim_check)

    robot_name = "tiago_pro"
    packages = ["tiago_pro_description", "pal_sea_arm_description",
                "omni_base_description", "pal_pro_gripper_description"]

    model_path = get_model_paths(packages)

    gazebo_model_path_env_var = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH", model_path)

    gazebo = include_scoped_launch_py_description(
        pkg_name="pal_gazebo_worlds",
        paths=["launch", "pal_gazebo.launch.py"],
        env_vars=[gazebo_model_path_env_var],
        launch_arguments={
            "world_name":  launch_args.world_name,
            "model_paths": packages,
            "resource_paths": packages,
        })

    launch_description.add_action(gazebo)

    robot_spawn = include_scoped_launch_py_description(
        pkg_name="tiago_pro_gazebo",
        paths=["launch", "robot_spawn.launch.py"])

    launch_description.add_action(robot_spawn)

    tiago_bringup = include_scoped_launch_py_description(
        pkg_name="test_py", paths=["launch", "launch_robot_bringup.launch.py"],
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "arm_type_right": launch_args.arm_type_right,
            "arm_type_left": launch_args.arm_type_left,
            "end_effector_right": launch_args.end_effector_right,
            "end_effector_left": launch_args.end_effector_left,
            "ft_sensor_right": launch_args.ft_sensor_right,
            "ft_sensor_left": launch_args.ft_sensor_left,
            "wrist_model_right": launch_args.wrist_model_right,
            "wrist_model_left": launch_args.wrist_model_left,
            "laser_model": 'no-laser',
            "camera_model": launch_args.camera_model,
            "base_type": launch_args.base_type,
            "is_public_sim": launch_args.is_public_sim}
    )

    launch_description.add_action(tiago_bringup)'''

    my_controller = Node(
        package="test_py", 
        executable="facepalm", 
        name="facepalm",
        output="screen"
    )
    launch_description.add_action(my_controller)

    return


'''def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")

        model_paths += model_path

    if "GAZEBO_MODEL_PATH" in environ:
        model_paths += pathsep + environ["GAZEBO_MODEL_PATH"]

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ""
    for package_name in packages_names:
        if resource_paths != "":
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    if "GAZEBO_RESOURCE_PATH" in environ:
        resource_paths += pathsep + environ["GAZEBO_RESOURCE_PATH"]

    return resource_paths'''


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
