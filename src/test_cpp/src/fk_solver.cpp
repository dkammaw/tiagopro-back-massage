#include "test_cpp/fk_solver.hpp"

FKSolver::FKSolver(rclcpp::Node::SharedPtr node)
    : node_(node), move_group_(node_, "arm_left") // Initialize MoveGroupInterface
{
    RCLCPP_INFO(node_->get_logger(), "MoveIt! MoveGroupInterface initialized for arm_left");
}

void FKSolver::execute_trajectory(const std::vector<double>& joint_positions)
{
    RCLCPP_INFO(node_->get_logger(), "Solving FK to best configuration in nullspace using MoveIt!...");
    
    move_group_.setJointValueTarget(joint_positions);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (!success)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to plan trajectory!");
        return;
    }

    // Execute the planned trajectory
    moveit::core::MoveItErrorCode result = move_group_.execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Nullspace motion executed successfully!");
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Nullspace motion execution failed!");
    }
}


