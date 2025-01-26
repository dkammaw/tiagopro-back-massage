#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

class IKNode : public rclcpp::Node
{
public:
    IKNode() : Node("ik_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node initialized, planning...");
    }

    void initialize_move_group()
    {
        // Initialize the MoveGroupInterface
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");
        // Get the robot model using MoveGroupInterface
        moveit::core::RobotModelConstPtr robot_model = move_group_interface_->getRobotModel();
        
        // Example inputs
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.5;
        target_pose.position.y = 0.2;
        target_pose.position.z = 0.3;
        target_pose.orientation.w = 1.0;

        std::vector<double> ik_seed_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Initial guess
        double timeout = 1.0; // 1 second timeout
        std::vector<double> consistency_limits(ik_seed_state.size(), 0.1);
        std::vector<double> solution;
        moveit_msgs::msg::MoveItErrorCodes error_code;
        kinematics::KinematicsQueryOptions options;

        // Instantiate the kinematics solver
        kinematics_solver_ = getKinematicsSolver(robot_model);

        if (kinematics_solver_ && kinematics_solver_->searchPositionIK(target_pose, ik_seed_state, timeout, consistency_limits, solution, error_code, options))
        {
            RCLCPP_INFO(this->get_logger(), "IK solution found:");
            for (const auto &joint_value : solution)
            {
                RCLCPP_INFO(this->get_logger(), "Joint: %f", joint_value);
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "IK solution not found, error code: %d", error_code.val);
        }
    }
        
    std::shared_ptr<kinematics::KinematicsBase> getKinematicsSolver(const moveit::core::RobotModelConstPtr& robot_model)
    {
        try
        {
            // Load the KDL kinematics plugin using pluginlib
            pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader(
                "moveit_core", "kinematics::KinematicsBase");

            std::shared_ptr<kinematics::KinematicsBase> kinematics_solver =
                kinematics_loader.createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");

            // Define the frames for your robot (modify according to your setup)
            std::string base_frame = "base_link";
            std::string tip_frame = "arm_left_tool_link"; // Modify to the correct tip frame for your robot
            double search_discretization = 0.01;
            std::vector<std::string> tip_frames = {tip_frame};

            // Initialize the kinematics solver with the RobotModel
            if (kinematics_solver->initialize(
                    rclcpp::Node::SharedPtr(this), *robot_model, "arm_left", base_frame, tip_frames, search_discretization))
            {
                RCLCPP_INFO(this->get_logger(), "KDL Kinematics Plugin Loaded Successfully!");
                return kinematics_solver;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize KDL kinematics solver.");
                return nullptr;
            }
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while loading kinematics plugin: %s", ex.what());
            return nullptr;
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<kinematics::KinematicsBase> kinematics_solver_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKNode>();
  node->initialize_move_group();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}





