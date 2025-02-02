/*#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/SVD>

// Function to compute manipulability index
double computeManipulability(const Eigen::MatrixXd& jacobian)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.singularValues().prod();  // Product of singular values (measure of dexterity)
}

// Function to explore nullspace and return the best joint configuration
Eigen::VectorXd exploreNullspace(moveit::core::RobotState& robot_state, const moveit::core::JointModelGroup* jmg)
{
    robot_state.updateLinkTransforms();

    // Compute Jacobian for the end-effector
    Eigen::MatrixXd jacobian;
    if (!robot_state.getJacobian(jmg, jmg->getLinkModels().back(), Eigen::Vector3d::Zero(), jacobian, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute Jacobian.");
        return robot_state.getVariablePositions();
    }

    // Compute SVD to get nullspace
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullV);
    Eigen::Index rank = svd.rank();
    std::size_t ns_dim = svd.cols() - rank;
    if (ns_dim == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("nullspace_exploration"), "No nullspace available.");
        return robot_state.getVariablePositions();
    }

    Eigen::MatrixXd nullspace = svd.matrixV().rightCols(ns_dim);  // Nullspace basis vectors
    Eigen::VectorXd best_config = robot_state.getVariablePositions();
    double best_manipulability = computeManipulability(jacobian);

    // Explore configurations along each nullspace direction
    for (std::size_t i = 0; i < ns_dim; ++i)
    {
        for (double scale : {-0.1, 0.1})  // Small movements in positive & negative nullspace directions
        {
            Eigen::VectorXd new_config = best_config + scale * nullspace.col(i);
            robot_state.setVariablePositions(new_config);

            // Compute new Jacobian & manipulability index
            robot_state.updateLinkTransforms();
            if (!robot_state.getJacobian(jmg, jmg->getLinkModels().back(), Eigen::Vector3d::Zero(), jacobian, false))
                continue;

            double manipulability = computeManipulability(jacobian);
            if (manipulability > best_manipulability)
            {
                best_manipulability = manipulability;
                best_config = new_config;
            }
        }
    }

    return best_config;  // Return the best found joint configuration
}

// Main ROS2 node for testing nullspace exploration
class NullspaceExplorationNode : public rclcpp::Node
{
public:
    NullspaceExplorationNode() : Node("nullspace_exploration")
    {
        robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader.getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

        jmg_ = robot_model_->getJointModelGroup("arm_left");  // Change to match your robot's planning group
        if (!jmg_)
        {
            RCLCPP_ERROR(get_logger(), "Joint model group not found.");
            return;
        }

        explore_and_publish();
    }

    void explore_and_publish()
    {
        robot_state_->setToDefaultValues();  // Start from default position

        Eigen::VectorXd best_config = exploreNullspace(*robot_state_, jmg_);
        robot_state_->setVariablePositions(best_config);
        RCLCPP_INFO(get_logger(), "Best configuration found with highest manipulability.");
    }

private:
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* jmg_;
};

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NullspaceExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}*/

#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/SVD>

// Main ROS2 node for testing nullspace exploration
class NullspaceExplorationNode : public rclcpp::Node, public std::enable_shared_from_this<NullspaceExplorationNode>
{
public:
    NullspaceExplorationNode() : Node("nullspace_exploration")
    {
        robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader.getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

        jmg_ = robot_model_->getJointModelGroup("arm_left");  // Change to match your robot's planning group
        if (!jmg_)
        {
            RCLCPP_ERROR(get_logger(), "Joint model group not found.");
            return;
        }

        // Initialize MoveGroupInterface and KinematicsMetrics
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");
        kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(move_group_interface_->getRobotModel());

        explore_and_publish();
    }

    void explore_and_publish()
    {
        robot_state_->setToDefaultValues();  // Start from default position

        Eigen::VectorXd best_config = exploreNullspace(*robot_state_, jmg_);
        robot_state_->setVariablePositions(best_config.data());
        RCLCPP_INFO(get_logger(), "Best configuration found with highest manipulability.");
    }

private:
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* jmg_;
    std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

// Function to explore nullspace and return the best joint configuration
Eigen::VectorXd exploreNullspace(moveit::core::RobotState& robot_state, const moveit::core::JointModelGroup* jmg)
{
    robot_state.updateLinkTransforms();

    // Get the manipulability index for the current configuration
    double manipulability_index;
    bool success = kinematics_metrics_->getManipulabilityIndex(robot_state, jmg, manipulability_index, false);

    if (!success)
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute manipulability index.");
        return Eigen::VectorXd::Zero(robot_state.getVariableCount());  // Ensure valid return in case of failure
    }

    // Start with the current configuration
    Eigen::VectorXd best_config = Eigen::VectorXd::Map(robot_state.getVariablePositions().data(), robot_state.getVariableCount());  // Correct usage for Eigen::VectorXd
    double best_manipulability = manipulability_index;

    // Compute Jacobian for the end-effector
    Eigen::MatrixXd jacobian;
    if (!robot_state.getJacobian(jmg, jmg->getLinkModels().back(), Eigen::Vector3d::Zero(), jacobian, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute Jacobian.");
        return best_config;  // Return current config if Jacobian fails
    }

    // Compute the nullspace of the Jacobian using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullV);
    Eigen::Index rank = svd.rank();
    std::size_t ns_dim = svd.cols() - rank;
    if (ns_dim == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("nullspace_exploration"), "No nullspace available.");
        return best_config;  // Return current config if no nullspace
    }

    Eigen::MatrixXd nullspace = svd.matrixV().rightCols(ns_dim);  // Nullspace basis vectors

    // Explore configurations along each nullspace direction
    for (std::size_t i = 0; i < ns_dim; ++i)
    {
        for (double scale : {-0.1, 0.1})  // Small movements in positive & negative nullspace directions
        {
            Eigen::VectorXd new_config = best_config + scale * nullspace.col(i);
            robot_state.setVariablePositions(new_config.data());

            // Compute manipulability index for the new configuration
            robot_state.updateLinkTransforms();
            bool new_success = kinematics_metrics_->getManipulabilityIndex(robot_state, jmg, manipulability_index, false);
            if (!new_success)
                continue;

            // If the new configuration has better manipulability, update best_config
            if (manipulability_index > best_manipulability)
            {
                best_manipulability = manipulability_index;
                best_config = new_config;
            }
        }
    }

    return best_config;  // Return the best found joint configuration
}

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NullspaceExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

