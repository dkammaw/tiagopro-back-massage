cmake_minimum_required(VERSION 3.8)
project(test_cpp)


# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Find MoveIt2 dependencies
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Erstelle eine Bibliothek für den FKSolver
add_library(fk_solver_lib src/fk_solver.cpp)
add_library(nullspace_explorer_lib src/nullspace_explorer.cpp)

# Existing executables
add_executable(ik_solver src/ik_solver.cpp)
add_executable(test_interpolator src/test_interpolator.cpp)

target_link_libraries(test_interpolator nullspace_explorer_lib)

ament_target_dependencies(
    ik_solver
    rclcpp 
    moveit_core
    moveit_ros_planning_interface
)

ament_target_dependencies(
    test_interpolator
    rclcpp 
    moveit_core
    moveit_ros_planning_interface
    control_msgs
)

ament_target_dependencies(
    nullspace_explorer_lib
    rclcpp 
    moveit_core
    moveit_ros_planning_interface
    control_msgs
)

ament_target_dependencies(
    fk_solver_lib
    rclcpp 
    moveit_core
    moveit_ros_planning_interface
    control_msgs
)

# Install executables
install(TARGETS
    test_interpolator
    nullspace_explorer_lib
    ik_solver
    fk_solver_lib
    DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(ik_solver PUBLIC include ${rclcpp_INCLUDE_DIRS})
target_include_directories(test_interpolator PUBLIC include ${rclcpp_INCLUDE_DIRS})
target_include_directories(nullspace_explorer_lib PUBLIC include ${rclcpp_INCLUDE_DIRS})
target_include_directories(fk_solver_lib PUBLIC include ${rclcpp_INCLUDE_DIRS})


# Install launch files
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
