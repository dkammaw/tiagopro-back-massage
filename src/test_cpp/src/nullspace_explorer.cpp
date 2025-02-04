#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/robot_state/conversions.h>  
#include <Eigen/SVD>

Eigen::VectorXd exploreNullspace(moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg, 
                                 std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics);

class NullspaceExplorationNode : public rclcpp::Node
{
public:
    NullspaceExplorationNode() : Node("nullspace_exploration")
    {
        RCLCPP_INFO(this->get_logger(), "Node initialized.");
        
        subscription_ = this->create_subscription<moveit_msgs::msg::RobotState>(
            "robot_state_topic", 10, std::bind(&NullspaceExplorationNode::robot_state_callback, this, std::placeholders::_1));
    }

    void initialize_move_group()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");
        kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(move_group_interface_->getRobotModel());
        robot_model_ = std::const_pointer_cast<moveit::core::RobotModel>(move_group_interface_->getRobotModel());
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        jmg_ = robot_model_->getJointModelGroup("arm_left");
        if (!jmg_)
        {
            RCLCPP_ERROR(get_logger(), "Joint model group not found.");
            return;
        }
    }

    void robot_state_callback(const moveit_msgs::msg::RobotState::SharedPtr msg)
    {
        moveit::core::RobotState current_state(robot_model_);
        robotStateMsgToRobotState(*msg, current_state);
        
        double manipulability_index = 0.0;
        if (kinematics_metrics_->getManipulabilityIndex(current_state, jmg_, manipulability_index, false))
        {
            RCLCPP_INFO(this->get_logger(), "Current Manipulability Index: %f", manipulability_index);
            explore_and_publish(current_state);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to calculate Manipulability Index");
        }
    }

    void explore_and_publish(moveit::core::RobotState& current_state)
    {
        Eigen::VectorXd best_config = exploreNullspace(current_state, jmg_, kinematics_metrics_);
        robot_state_->setVariablePositions(best_config.data());
        RCLCPP_INFO(get_logger(), "Best configuration found with highest manipulability.");
    }

private:
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* jmg_;
    std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr subscription_;
};

Eigen::VectorXd exploreNullspace(moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg, 
                                 std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics)
{
    std::vector<double> joint_values;
    robot_state.copyJointGroupPositions(jmg, joint_values);

    // Prepare the string for logging
    std::string joint_values_str = "Joint_values: [ ";
    for (size_t i = 0; i < joint_values.size(); ++i) {
        joint_values_str += std::to_string(joint_values[i]);
        if (i != joint_values.size() - 1) {
            joint_values_str += " "; // Add space between values
        }
    }
    joint_values_str += " ]";

    // Log the joint values using c_str() to convert to const char*
    RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), joint_values_str.c_str());


    double manipulability_index;
    if (!kinematics_metrics->getManipulabilityIndex(robot_state, jmg, manipulability_index, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute manipulability index.");
        return Eigen::VectorXd::Zero(robot_state.getVariableCount());
    }

    Eigen::VectorXd best_config(joint_values.size());
    for (size_t i = 0; i < joint_values.size(); ++i) {
        best_config(i) = joint_values[i];
    }

    // Log the best_config
    RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "Initial best_config: " << best_config);  // Assuming 7 joints



    double best_manipulability = manipulability_index;
    
    Eigen::MatrixXd jacobian;
    if (!robot_state.getJacobian(jmg, jmg->getLinkModels().back(), Eigen::Vector3d::Zero(), jacobian, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute Jacobian.");
        return best_config;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "Jacobian: \n" << jacobian << "\n");
    
    // Compute nullspace using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Index rank = svd.rank();
    std::size_t ns_dim = svd.cols() - rank;
    if (ns_dim == 0)
    {   
        RCLCPP_WARN(rclcpp::get_logger("nullspace_exploration"), "No nullspace available.");
        return best_config;
    } else {
        // Log the components of the SVD
        RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "U (Left Singular Vectors): \n" << svd.matrixU() << "\n");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "Singular Values: \n" << svd.singularValues() << "\n");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "V (Right Singular Vectors): \n" << svd.matrixV() << "\n");
        RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Nullspace available with dimension: %zu", ns_dim);
    }

    Eigen::MatrixXd nullspace = svd.matrixV().rightCols(ns_dim);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "Right Columns of Nullspace: \n" << nullspace<< "\n");
    
    // Explore nullspace directions
    for (std::size_t i = 0; i < ns_dim; ++i)
    {
        for (double scale : {-0.1, 0.1})
        {
            Eigen::VectorXd new_config = best_config + scale * nullspace.col(i);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "New_config: \n" << new_config<< "\n");
            
            //std::vector<double> new_config_vec(new_config.data(), new_config.data() + new_config.size());
            //robot_state.setVariablePositions(new_config_vec);
            robot_state.setVariablePositions(new_config.data());
            const std::vector<double>& joint_positions = robot_state.getVariablePositions();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "New_config_positions: \n" << joint_positions);

            robot_state.updateLinkTransforms();
            bool new_success = kinematics_metrics->getManipulabilityIndex(robot_state, jmg, manipulability_index, false);
            RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Current Manipulability Index: %f", manipulability_index);
            if (!new_success)
                continue;

            if (manipulability_index > best_manipulability)
            {
                best_manipulability = manipulability_index;
                best_config = new_config;
                RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "New best config: \n" << best_config);
                RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "New best manipulability index: %f", best_manipulability);
            }
        }
    }

    return best_config;
}

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NullspaceExplorationNode>();
    node->initialize_move_group();  // Initialize MoveGroupInterface separately
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



/*
#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <cmath>  // Für sqrt

// Forward declaration
Eigen::VectorXd exploreNullspace(moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg
                                 );

class NullspaceExplorationNode : public rclcpp::Node
{
public:
    NullspaceExplorationNode() : Node("nullspace_exploration")
    {
        RCLCPP_INFO(this->get_logger(), "Node initialized.");
    }

    void initialize_move_group()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");

        // Create kinematics metrics after move_group_interface_
        kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(move_group_interface_->getRobotModel());

        // Get the robot model and initialize robot state
        robot_model_ = std::const_pointer_cast<moveit::core::RobotModel>(move_group_interface_->getRobotModel());
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

        // Retrieve the JointModelGroup
        jmg_ = robot_model_->getJointModelGroup("arm_left");
        if (!jmg_)
        {
            RCLCPP_ERROR(get_logger(), "Joint model group not found.");
            return;
        }

        // Start exploration
        explore_and_publish();
    }

    void explore_and_publish()
    {
        robot_state_->setToDefaultValues();

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


double computeManipulabilityIndex(const Eigen::MatrixXd& jacobian)
{
    Eigen::MatrixXd JJt = jacobian * jacobian.transpose(); // J * J^T
    double determinant = JJt.determinant(); // Det(J * J^T)

    if (determinant < 0)  // Numerische Stabilitätsprüfung
    {
        return 0.0;  // Falls determinant < 0, setze auf 0 (sollte nicht passieren)
    }
    
    return std::sqrt(determinant);
}

// Function to explore nullspace and return the best joint configuration
Eigen::VectorXd exploreNullspace(moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg)
{
    std::vector<double> joint_values;
    robot_state.update();
    const std::vector<std::string>& joint_names = jmg->getVariableNames();
    robot_state.copyJointGroupPositions(jmg, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("state_logger"), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // Retrieve the joint positions correctly
    const double* positions_ptr = robot_state.getVariablePositions();
    std::vector<double> joint_positions(positions_ptr, positions_ptr + robot_state.getVariableCount());

    Eigen::VectorXd best_config = Eigen::Map<Eigen::VectorXd>(joint_positions.data(), joint_positions.size());
    //double best_manipulability = manipulability_index;

    // Compute Jacobian
    Eigen::MatrixXd jacobian;
    if (!robot_state.getJacobian(jmg, jmg->getLinkModels().back(), Eigen::Vector3d::Zero(), jacobian, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute Jacobian.");
        return best_config;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "Jacobian: \n" << jacobian << "\n");
    RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Best configuration found with highest manipulability: %f", computeManipulabilityIndex(jacobian));
    
    // Compute nullspace using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullV);
    Eigen::Index rank = svd.rank();
    std::size_t ns_dim = svd.cols() - rank;
    if (ns_dim == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("nullspace_exploration"), "No nullspace available.");
        return best_config;
    }

    Eigen::MatrixXd nullspace = svd.matrixV().rightCols(ns_dim);

    // Explore nullspace directions
    for (std::size_t i = 0; i < ns_dim; ++i)
    {
        for (double scale : {-0.1, 0.1})
        {
            Eigen::VectorXd new_config = best_config + scale * nullspace.col(i);
            std::vector<double> new_config_vec(new_config.data(), new_config.data() + new_config.size());
            robot_state.setVariablePositions(new_config_vec);

            robot_state.updateLinkTransforms();
            bool new_success = kinematics_metrics->getManipulabilityIndex(robot_state, jmg, manipulability_index, false);
            if (!new_success)
                continue;

            if (manipulability_index > best_manipulability)
            {
                best_manipulability = manipulability_index;
                best_config = new_config;
            }
        }
    }

    return best_config;
}

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NullspaceExplorationNode>();
    node->initialize_move_group();  // Initialize MoveGroupInterface separately
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}*/



