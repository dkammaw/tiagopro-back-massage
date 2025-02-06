/*
// SIMULATED ANNEALING

#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/robot_state/conversions.h>  
#include <Eigen/SVD>
#include <numeric>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <random>


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
        std::vector<double> best_config_vec(best_config.data(), best_config.data() + best_config.size());
        current_state.setJointGroupPositions(jmg_, best_config_vec);
        RCLCPP_INFO_STREAM(get_logger(), "Final best_config: \n" << best_config);  // Assuming 7 joints
        RCLCPP_INFO(get_logger(), "Updated robot state to best configuration found with highest manipulability.");
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
    Eigen::VectorXd best_config = Eigen::Map<Eigen::VectorXd>(joint_values.data(), joint_values.size());

    double best_manipulability;
    if (!kinematics_metrics->getManipulabilityIndex(robot_state, jmg, best_manipulability, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute manipulability index.");
        return best_config;
    }

    Eigen::MatrixXd jacobian;
    if (!robot_state.getJacobian(jmg, jmg->getLinkModels().back(), Eigen::Vector3d::Zero(), jacobian, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute Jacobian.");
        return best_config;
    }

    // Compute null space using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd nullspace = svd.matrixV().rightCols(svd.cols() - svd.rank());

    if (nullspace.cols() == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("nullspace_exploration"), "No nullspace available.");
        return best_config;
    } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "U (Left Singular Vectors): \n" << svd.matrixU() << "\n");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "Singular Values: \n" << svd.singularValues() << "\n");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "V (Right Singular Vectors): \n" << svd.matrixV() << "\n");
        RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Nullspace available with dimension: %zu", nullspace.cols());
    }


    // SIMULATED ANNEALING

    double temperature = 50.0;  // how much randomness and perturbation
    double cooling_rate = 0.98; // how quickly the temperature decreases
    double min_temperature = 1e-3;
    const int max_iterations = 5000;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> rand01(0.0, 1.0);

    RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Starting Simulated Annealing...");

    for (int iter = 0; iter < max_iterations && temperature > min_temperature; ++iter)
    {
        Eigen::VectorXd perturbation = nullspace * Eigen::VectorXd::Random(nullspace.cols()) * temperature * 0.8;
        Eigen::VectorXd new_config = best_config + perturbation;

        std::vector<double> new_config_vec(new_config.data(), new_config.data() + new_config.size());
        robot_state.setJointGroupPositions(jmg, new_config_vec);
        robot_state.updateLinkTransforms();

        if (!robot_state.satisfiesBounds(jmg))
        {
            continue;  // Skip invalid configurations
        }

        double new_manipulability;
        if (!kinematics_metrics->getManipulabilityIndex(robot_state, jmg, new_manipulability, false))
        {
            continue;  // Skip if manipulability could not be computed
        }

        double delta = new_manipulability - best_manipulability;

        if (delta > 0 || exp(delta / temperature) > rand01(gen))
        {
            best_config = new_config;
            best_manipulability = new_manipulability;
            RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Accepted new config with manipulability: %f", best_manipulability);
        }

        temperature *= cooling_rate;
    }

    RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Final best manipulability: %f", best_manipulability);
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


// GRID SEARCH

#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/robot_state/conversions.h>  
#include <Eigen/SVD>
#include <numeric>

Eigen::MatrixXd computeNullspace(const moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg);

Eigen::VectorXd exploreNullspace(moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg, 
                                 std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics);
                                 
Eigen::VectorXd gridSearch(moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg, 
                                 std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics,
                                 double best_manipulability, Eigen::VectorXd best_config, Eigen::MatrixXd nullspace);

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
        std::vector<double> best_config_vec(best_config.data(), best_config.data() + best_config.size());
        current_state.setJointGroupPositions(jmg_, best_config_vec);
        RCLCPP_INFO_STREAM(get_logger(), "Final best_config: \n" << best_config);  // Assuming 7 joints
        RCLCPP_INFO(get_logger(), "Updated robot state to best configuration found with highest manipulability.");
    }

private:
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* jmg_;
    std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr subscription_;
};

// Function to compute the nullspace matrix
Eigen::MatrixXd computeNullspace(const moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg)
{
    Eigen::MatrixXd jacobian;
    if (!robot_state.getJacobian(jmg, jmg->getLinkModels().back(), Eigen::Vector3d::Zero(), jacobian, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute Jacobian.");
        return Eigen::MatrixXd::Zero(jmg->getVariableCount(), jmg->getVariableCount());
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Index rank = svd.rank();
    std::size_t ns_dim = svd.cols() - rank;

    if (ns_dim == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("nullspace_exploration"), "No nullspace available.");
        return Eigen::MatrixXd::Zero(jmg->getVariableCount(), 1);
    } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "U (Left Singular Vectors): \n" << svd.matrixU() << "\n");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "Singular Values: \n" << svd.singularValues() << "\n");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "V (Right Singular Vectors): \n" << svd.matrixV() << "\n");
        RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Jacobian rank: %d, Nullspace dimension: %zu", rank, ns_dim);
    }

    Eigen::MatrixXd nullspace = svd.matrixV().rightCols(ns_dim);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("nullspace_exploration"), "Updated Nullspace: \n" << nullspace << "\n");

    return nullspace;
}

// Function to explore the nullspace
Eigen::VectorXd exploreNullspace(moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg, 
                                 std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics)
{
    std::vector<double> joint_values;
    robot_state.copyJointGroupPositions(jmg, joint_values);

    double manipulability_index;
    if (!kinematics_metrics->getManipulabilityIndex(robot_state, jmg, manipulability_index, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute manipulability index.");
        return Eigen::VectorXd::Zero(joint_values.size());
    }

    Eigen::VectorXd best_config = Eigen::Map<Eigen::VectorXd>(joint_values.data(), joint_values.size());
    double best_manipulability = manipulability_index;

    // Iteratively update nullspace and explore
    Eigen::MatrixXd nullspace = computeNullspace(robot_state, jmg);

    return gridSearch(robot_state, jmg, kinematics_metrics, best_manipulability, best_config, nullspace);

}

Eigen::VectorXd gridSearch(moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg, 
                                 std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics,
                                 double best_manipulability, Eigen::VectorXd best_config, Eigen::MatrixXd nullspace) {
    // Grid Search parameters
    double min_scale = -0.05;
    double max_scale = 0.05;
    double step_size = 0.005;

    for (std::size_t i = 0; i < static_cast<std::size_t>(nullspace.cols()); ++i)
    {
        for (double scale = min_scale; scale <= max_scale; scale += step_size)
        {
            Eigen::VectorXd new_config = best_config + scale * nullspace.col(i);

            std::vector<double> new_config_vec(new_config.data(), new_config.data() + new_config.size());
            robot_state.setJointGroupPositions(jmg, new_config_vec);
            if (!robot_state.satisfiesBounds(jmg))
                continue;  // Ignore configurations outside joint limits
            
            double manipulability_index;

            robot_state.updateLinkTransforms();
            bool new_success = kinematics_metrics->getManipulabilityIndex(robot_state, jmg, manipulability_index, false);
            RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Current Manipulability Index: %f", manipulability_index);
            if (!new_success)
                continue;

            if (manipulability_index > best_manipulability)
            {
                best_manipulability = manipulability_index;
                best_config = new_config;

                RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "New best manipulability index: %f", best_manipulability);
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Best configuration found with highest manipulability index: %f", best_manipulability);
    return best_config;
}


// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NullspaceExplorationNode>();
    node->initialize_move_group();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


