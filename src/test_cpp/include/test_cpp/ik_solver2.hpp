#ifndef IK_SOLVER_HPP
#define IK_SOLVER_HPP

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdlib>  
#include <iostream>  
#include <thread>  

class IKSolver : public rclcpp::Node
{
public:
    IKSolver();
    void initialize_move_group();
    void tapping_positions_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    geometry_msgs::msg::Pose interpolate_pose(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &end, double t, bool interpolate_orientation);
    void plan_cartesian_path(geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose target_pose, bool interpolate_orientation);
    void call_nullspace_exploration_node();
    void publish_markers(const std::vector<geometry_msgs::msg::Pose>& waypoints);

private:
    int id_counter = 1;
    int path_counter = 1;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tapping_positions_sub_;
};

#endif // IK_SOLVER_HPP