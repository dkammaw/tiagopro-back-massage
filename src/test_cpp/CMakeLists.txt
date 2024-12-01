cmake_minimum_required(VERSION 3.8)
project(test_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

# Install SOURCE files
install(TARGETS
    facepalm
    scratch_calc
    DESTINATION lib/${PROJECT_NAME}
)

# Install LAUNCH files
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()