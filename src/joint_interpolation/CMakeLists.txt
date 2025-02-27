cmake_minimum_required(VERSION 3.8)
project(joint_interpolation)


# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Find MoveIt2 dependencies
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Create library for fk_solver
add_library(joint_nullspace_explorer_lib src/joint_nullspace_explorer.cpp)

# Existing executables
add_executable(joint_interpolator src/joint_interpolator.cpp)

target_link_libraries(joint_interpolator joint_nullspace_explorer_lib)



ament_target_dependencies(
    joint_interpolator
    rclcpp 
    moveit_core
    moveit_ros_planning_interface
    control_msgs
)

ament_target_dependencies(
    joint_nullspace_explorer_lib
    rclcpp 
    moveit_core
    moveit_ros_planning_interface
    control_msgs
)


# Install executables
install(TARGETS
    joint_interpolator
    joint_nullspace_explorer_lib
    DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(joint_interpolator PUBLIC include ${rclcpp_INCLUDE_DIRS})
target_include_directories(joint_nullspace_explorer_lib PUBLIC include ${rclcpp_INCLUDE_DIRS})


# Install launch files
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
