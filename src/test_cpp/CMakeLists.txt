cmake_minimum_required(VERSION 3.8)
project(test_cpp)


# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(builtin_interfaces REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Find MoveIt2 dependencies
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)

# Existing executables
add_executable(nullspace_explorer src/nullspace_explorer.cpp)

# Enable Precompiled Headers (AFTER defining the target)
target_precompile_headers(nullspace_explorer PRIVATE /ros2_ws/src/test_cpp/src/pch.h)

ament_target_dependencies(
    nullspace_explorer
    rclcpp 
    #builtin_interfaces
    moveit_core
    #std_msgs
    moveit_ros_planning_interface
)

# Install executables
install(TARGETS
    nullspace_explorer
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
