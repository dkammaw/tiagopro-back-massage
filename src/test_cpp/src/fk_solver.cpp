#include "fk_solver.hpp"

FKSolver::FKSolver(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        node_,
        "/arm_left_controller/follow_joint_trajectory"
    );

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available!");
    }
}

void FKSolver::execute_trajectory(const std::vector<double>& joint_positions)
{
    RCLCPP_INFO(node_->get_logger(), "Solving FK to best configuration in nullspace...");
    
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {
        "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint",
        "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint",
        "arm_left_7_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    point.time_from_start.sec = 2;
    point.time_from_start.nanosec = 0;

    goal_msg.trajectory.points.push_back(point);

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            RCLCPP_INFO(node_->get_logger(), "Nullspace motion executed successfully!");
        else
            RCLCPP_ERROR(node_->get_logger(), "Nullspace motion failed!");
    };

    send_goal_options.feedback_callback = 
    [this](GoalHandle::SharedPtr, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
        RCLCPP_INFO(node_->get_logger(), "Feedback received!");

        for (size_t i = 0; i < feedback->error.positions.size(); i++) {
            RCLCPP_INFO(node_->get_logger(), "Position error for joint %zu: %f", i + 1, feedback->error.positions[i]);
        }
    };

    auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
    if (future_goal_handle.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        RCLCPP_ERROR(node_->get_logger(), "Error: Goal was not accepted!");
        return;
    }
}

