/*#include <moveit/planning_interface/planning_interface.h>
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
        target_pose.position.x = 0.0;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.0;

        std::vector<double> ik_seed_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Initial guess
        double timeout = 5.0; // 1 second timeout
        std::vector<double> consistency_limits(ik_seed_state.size(), 0.2);
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
}*/

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/robot_state.hpp>

class IKNode : public rclcpp::Node
{
public:
    IKNode() : Node("ik_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node initialized, planning...");

        // Initialize the subscriber to the RobotState topic
        joint_state_subscription_ = this->create_subscription<moveit_msgs::msg::RobotState>(
            "/robot_state_topic", 10, std::bind(&IKNode::robot_state_callback, this, std::placeholders::_1));

    }

    void initialize_move_group()
    {
        // Initialize the MoveGroupInterface
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");

        moveit::core::RobotModelConstPtr robot_model = move_group_interface_->getRobotModel();

        // Instantiate the kinematics solver
        kinematics_solver_ = getKinematicsSolver(robot_model);
    }

    void robot_state_callback(const moveit_msgs::msg::RobotState::SharedPtr msg)
    {
        // Directly copy the joint positions from the received RobotState
        std::vector<double> ik_seed_state = msg->joint_state.position;
    
        // Perform inverse kinematics
        perform_ik(ik_seed_state);
    }

    void perform_ik(const std::vector<double>& ik_seed_state)
    {
        // Target pose for inverse kinematics
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.058;  // -0.058
        target_pose.position.y = 0.739;  // 0.739
        target_pose.position.z = 0.392;  // 0.392
        target_pose.orientation.w = 1.0;

        double timeout = 5.0; // 5 second timeout
        std::vector<double> consistency_limits(ik_seed_state.size(), 0.5);
        std::vector<double> solution;
        moveit_msgs::msg::MoveItErrorCodes error_code;
        kinematics::KinematicsQueryOptions options;

        // Call the IK solver
        if (kinematics_solver_ && kinematics_solver_->searchPositionIK(target_pose, ik_seed_state, timeout, consistency_limits, solution, error_code, options))
        {
            RCLCPP_INFO(this->get_logger(), "IK solution found:");
            for (const auto& joint_value : solution)
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
    rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr joint_state_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKNode>();
    node->initialize_move_group();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}







