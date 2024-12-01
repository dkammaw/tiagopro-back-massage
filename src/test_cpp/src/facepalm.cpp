#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std;
using namespace std::placeholders;

class FacepalmActionClient : public rclcpp::Node
{
    public:
    // using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
    using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using Duration = builtin_interfaces::msg::Duration;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    explicit FacepalmActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("facepalm", node_options)
    {
        string controller = "/arm_right_controller";

        this->client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            controller + "/follow_joint_trajectory");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&FacepalmActionClient::send_goal, this));
    }

    void send_goal()
    {
        this->timer_->cancel();
        RCLCPP_ERROR(this->get_logger(), "Sleeping");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        RCLCPP_ERROR(this->get_logger(), "Done Sleeping");

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto duration = Duration();
        duration.sec = 5;
        duration.nanosec = 0;

        auto target = JointTrajectoryPoint();
        target.time_from_start = duration;
        target.positions = {
            0.0,    // shoulder inline twist
            1.1,    // joint 1 perp ang
            0.0,    // joint 1 inline twist
            -1.57,  // joint 2 perp ang
            0.0,    // joint 2 inline twist
            0.5,    // joint 3 perp ang
            0.0     // joint 3 inline twist
        };

        duration.sec = 0;
        duration.nanosec = 500000000;

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory.points.push_back(target);
        goal.goal_time_tolerance = duration;
        goal.trajectory.joint_names = {
            "arm_right_1_joint",
            "arm_right_2_joint",
            "arm_right_3_joint",
            "arm_right_4_joint",
            "arm_right_5_joint",
            "arm_right_6_joint",
            "arm_right_7_joint"
        };
        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&FacepalmActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&FacepalmActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&FacepalmActionClient::result_callback, this, _1);
        
        this->client_ptr_->async_send_goal(goal, send_goal_options);
    }

    private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(GoalHandle::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server :(");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result :)");
        }
    }

    void feedback_callback(GoalHandle::SharedPtr, 
        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
        for (auto elem : feedback->error.positions){
            RCLCPP_INFO_STREAM(this->get_logger(), "position error" << elem);
        }
    }

    void result_callback(const GoalHandle::WrappedResult & result)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "result" << result.result->error_code);
        rclcpp::shutdown();
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FacepalmActionClient>());
    rclcpp::shutdown();
    return 0;
}








// namespace action_tutorials_cpp
// {
// class FibonacciActionClient : public rclcpp::Node
// {
// public:
//   using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
//   using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

//   ACTION_TUTORIALS_CPP_PUBLIC
//   explicit FibonacciActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
//   : Node("fibonacci_action_client", node_options)
//   {
//     this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
//       this->get_node_base_interface(),
//       this->get_node_graph_interface(),
//       this->get_node_logging_interface(),
//       this->get_node_waitables_interface(),
//       "fibonacci");

//     this->timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(500),
//       std::bind(&FibonacciActionClient::send_goal, this));
//   }

//   ACTION_TUTORIALS_CPP_PUBLIC
//   void send_goal()
//   {
//     using namespace std::placeholders;

//     this->timer_->cancel();

//     if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
//       RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//       rclcpp::shutdown();
//       return;
//     }

//     auto goal_msg = Fibonacci::Goal();
//     goal_msg.order = 10;

//     RCLCPP_INFO(this->get_logger(), "Sending goal");

//     auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
//     send_goal_options.goal_response_callback =
//       std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
//     send_goal_options.feedback_callback =
//       std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
//     send_goal_options.result_callback =
//       std::bind(&FibonacciActionClient::result_callback, this, _1);
//     this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//   }

// private:
//   rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   ACTION_TUTORIALS_CPP_LOCAL
//   void goal_response_callback(GoalHandleFibonacci::SharedPtr goal_handle)
//   {
//     if (!goal_handle) {
//       RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//     }
//   }

//   ACTION_TUTORIALS_CPP_LOCAL
//   void feedback_callback(
//     GoalHandleFibonacci::SharedPtr,
//     const std::shared_ptr<const Fibonacci::Feedback> feedback)
//   {
//     std::stringstream ss;
//     ss << "Next number in sequence received: ";
//     for (auto number : feedback->partial_sequence) {
//       ss << number << " ";
//     }
//     RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
//   }

//   ACTION_TUTORIALS_CPP_LOCAL
//   void result_callback(const GoalHandleFibonacci::WrappedResult & result)
//   {
//     switch (result.code) {
//       case rclcpp_action::ResultCode::SUCCEEDED:
//         break;
//       case rclcpp_action::ResultCode::ABORTED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//         return;
//       case rclcpp_action::ResultCode::CANCELED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//         return;
//       default:
//         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//         return;
//     }
//     std::stringstream ss;
//     ss << "Result received: ";
//     for (auto number : result.result->sequence) {
//       ss << number << " ";
//     }
//     RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
//     rclcpp::shutdown();
//   }
// };  // class FibonacciActionClient

// }  // namespace action_tutorials_cpp

// RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)