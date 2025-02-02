cmake_minimum_required(VERSION 3.8)
project(test_cpp)

# Set build type to Debug to enable debugging symbols
set(CMAKE_BUILD_TYPE Debug)

# Compiler flags (optional)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)


# Find MoveIt2 dependencies
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)

# Existing executables
add_executable(facepalm src/facepalm.cpp)
ament_target_dependencies(
    facepalm
    rclcpp
    rclcpp_action
    rclcpp_components
    builtin_interfaces 
    control_msgs
    trajectory_msgs
)

add_executable(scratch_calc src/scratch_calc.cpp)
ament_target_dependencies(
    scratch_calc
    rclcpp
    rclcpp_action
    rclcpp_components
    builtin_interfaces 
    control_msgs
    trajectory_msgs
    std_msgs
)

# New executable for manipulability
add_executable(manipulability src/manipulability.cpp)
ament_target_dependencies(
    manipulability
    rclcpp
    rclcpp_action
    rclcpp_components
    builtin_interfaces 
    control_msgs
    trajectory_msgs
    std_msgs
    moveit_core
    tf2_ros
    moveit_ros_planning_interface

)

add_executable(redundancy_stepper src/redundancy_stepper.cpp)
ament_target_dependencies(
    redundancy_stepper 
    rclcpp 
    rclcpp_action
    rclcpp_components
    builtin_interfaces
    geometry_msgs 
    moveit_msgs
    pluginlib
    kdl_parser
    orocos_kdl
    moveit_core
    std_msgs
    moveit_ros_planning_interface
)

add_executable(nullspace_explorer src/nullspace_explorer.cpp)
ament_target_dependencies(
    nullspace_explorer
    rclcpp 
    rclcpp_action
    rclcpp_components
    builtin_interfaces
    geometry_msgs 
    moveit_msgs
    pluginlib
    kdl_parser
    orocos_kdl
    moveit_core
    std_msgs
    moveit_ros_planning_interface
)

# Install executables
install(TARGETS
    facepalm
    scratch_calc
    manipulability
    redundancy_stepper
    nullspace_explorer
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
