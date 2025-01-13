#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit/kinematics_metrics/kinematics_metrics.h>


using namespace moveit::core;

class ManipulabilityIndexExample : public rclcpp::Node
{
public:
  ManipulabilityIndexExample()
  : Node("manipulability_index_example")
  {
    static const std::string PLANNING_GROUP = "arm_left";
    // Initialize MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    
    // Perform some initialization or configuration
    RCLCPP_INFO(this->get_logger(), "Node initialized, planning...");
    
    // Create an instance of KinematicsMetrics
    kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(move_group_interface_->getRobotModel());
  }

  void calculate_manipulability_index()
  {
    // Get the current robot state
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

    // Compute the Manipulability Index for the "arm_left" group
    double manipulability_index = 0.0;
    bool success = kinematics_metrics_->getManipulabilityIndex(current_state, "arm_left", manipulability_index, false);

    // Log the resulting
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
  std::shared_ptr<MoveGroupInterface> move_group_interface_;
  std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ManipulabilityIndexExample>());

  rclcpp::shutdown();
  return 0;
}
