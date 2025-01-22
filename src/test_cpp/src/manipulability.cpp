#include <rclcpp/rclcpp.hpp>
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
    // Perform some initialization or configuration
    RCLCPP_INFO(this->get_logger(), "Node initialized, planning...");
  }

  void initialize_move_group()
  {
    static const std::string PLANNING_GROUP = "arm_left";

    // Initialize the MoveGroupInterface after the object is fully constructed
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);

    // Now that the MoveGroupInterface is initialized, update the KinematicsMetrics object with the robot model
    kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(move_group_interface_->getRobotModel());
  }

  void calculate_manipulability_index()
  {
    moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);

    double manipulability_index = 0.0;
    const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup("arm_left");

    bool success = kinematics_metrics_->getManipulabilityIndex(*current_state, joint_model_group, manipulability_index, false);

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
}
