#ifndef FK_SOLVER_HPP
#define FK_SOLVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class FKSolver
{
public:
    explicit FKSolver(rclcpp::Node::SharedPtr node);
    void execute_trajectory(const std::vector<double>& joint_positions);

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_; 
};

#endif // FK_SOLVER_HPP
