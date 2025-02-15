#ifndef FK_SOLVER_HPP
#define FK_SOLVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class FKSolver
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    explicit FKSolver(rclcpp::Node::SharedPtr node);

    void execute_trajectory(const std::vector<double>& joint_positions);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};

#endif // FK_SOLVER_HPP
