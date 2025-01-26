#include "rclcpp/rclcpp.hpp"
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>  
#include <moveit_msgs/msg/robot_state.hpp>

using namespace moveit::core;
using namespace moveit::planning_interface;

class ManipulabilityIndexExample : public rclcpp::Node
{
public:
  ManipulabilityIndexExample()
  : Node("manipulability_index_example")
  {
    RCLCPP_INFO(this->get_logger(), "Node initialized, planning...");

    // Subscribe to the robot state topic
    subscription_ = this->create_subscription<moveit_msgs::msg::RobotState>(
        "robot_state_topic", 10, std::bind(&ManipulabilityIndexExample::robot_state_callback, this, std::placeholders::_1));
  }

  void initialize_move_group()
  {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");

    kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(move_group_interface_->getRobotModel());

  }

  void robot_state_callback(const moveit_msgs::msg::RobotState::SharedPtr msg)
  {
      // Initialize RobotState using the robot model from MoveGroupInterface
      moveit::core::RobotState current_state(move_group_interface_->getRobotModel());

      // Convert the RobotState message to RobotState object using the robotStateMsgToRobotState function
      robotStateMsgToRobotState(*msg, current_state);

      // Now calculate the manipulability index
      double manipulability_index = 0.0;
      bool success = kinematics_metrics_->getManipulabilityIndex(current_state, "arm_left", manipulability_index, false);

      if (success)
      {
          RCLCPP_INFO(this->get_logger(), "Current Manipulability Index: %f", manipulability_index);
      }
      else
      {
          RCLCPP_ERROR(this->get_logger(), "Failed to calculate Manipulability Index");
      }
  }

private:
  rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr subscription_;  // Subscription to RobotState topic
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulabilityIndexExample>();
  node->initialize_move_group();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}




/*#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>

using namespace moveit::core;
using namespace moveit::planning_interface;

class ManipulabilityIndexExample : public rclcpp::Node
{
public:
  ManipulabilityIndexExample()
  : Node("manipulability_index_example")
  {
    RCLCPP_INFO(this->get_logger(), "Node initialized, planning...");
  }

  void initialize_move_group()
  {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");

    kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(move_group_interface_->getRobotModel());
  }

  void calculate_manipulability_index()
  {
    moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(20);

    double manipulability_index = 0.0;

    bool success = kinematics_metrics_->getManipulabilityIndex(*current_state, "arm_left", manipulability_index, false);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Current Manipulability Index: %f", manipulability_index);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to calculate Manipulability Index");
    }
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulabilityIndexExample>();

  node->initialize_move_group();
  node->calculate_manipulability_index();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}*/









