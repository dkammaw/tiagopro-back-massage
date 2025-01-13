cmake_minimum_required(VERSION 3.8)
project(test_cpp)

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

# Find MoveIt2 dependencies
find_package(moveit_core REQUIRED)

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
)

# Install executables
install(TARGETS
    facepalm
    scratch_calc
    manipulability
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
