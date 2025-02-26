#include "test_cpp/nullspace_exploration.hpp"

// NullspaceExplorationNode Konstruktor
NullspaceExplorationNode::NullspaceExplorationNode() : Node("nullspace_exploration")
{
    RCLCPP_INFO(this->get_logger(), "Node initialized.");
}

// MoveGroup initialisieren
void NullspaceExplorationNode::initialize_move_group()
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


// Nullspace-Exploration und Ausgabe der besten Konfiguration
moveit::core::RobotState NullspaceExplorationNode::explore(moveit::core::RobotState& current_state)
{
    initialize_move_group();
    std::vector<double> joint_values;
    current_state.copyJointGroupPositions(jmg_, joint_values);
    
    double manipulability_index;
    if (kinematics_metrics_->getManipulabilityIndex(current_state, jmg_, manipulability_index, false))
    {
        RCLCPP_INFO(this->get_logger(), "Initial Manipulability Index: %f", manipulability_index);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to calculate Manipulability Index");
    }

    Eigen::VectorXd best_config = Eigen::Map<Eigen::VectorXd>(joint_values.data(), joint_values.size());


    // Start der rekursiven Nullspace-Exploration
    best_config = recursiveNullspaceExploration(current_state, jmg_, kinematics_metrics_, manipulability_index, best_config);


    std::vector<double> best_config_vec(best_config.data(), best_config.data() + best_config.size());
    current_state.setJointGroupPositions(jmg_, best_config_vec);

    RCLCPP_INFO_STREAM(get_logger(), "Final best_config: \n" << best_config);

    RCLCPP_INFO(this->get_logger(), "Updated robot state to best configuration found with highest manipulability.");
    
    return current_state;
}

// Berechnung der Nullspace-Matrix
Eigen::MatrixXd computeNullspace(const moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg)
{
    Eigen::MatrixXd full_jacobian;
    if (!robot_state.getJacobian(jmg, jmg->getLinkModels().back(), Eigen::Vector3d::Zero(), full_jacobian, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute Jacobian.");
        return Eigen::MatrixXd::Zero(jmg->getVariableCount(), jmg->getVariableCount());
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(full_jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Index rank = svd.rank();
    std::size_t ns_dim = svd.cols() - rank;

    if (ns_dim == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("nullspace_exploration"), "No nullspace available.");
        return Eigen::MatrixXd::Zero(jmg->getVariableCount(), 1);
    }

    Eigen::MatrixXd nullspace = svd.matrixV().rightCols(ns_dim);
    return nullspace;
}

// Rekursive Nullspace-Exploration
Eigen::VectorXd recursiveNullspaceExploration(
    moveit::core::RobotState& robot_state, 
    const moveit::core::JointModelGroup* jmg, 
    std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics, 
    double prev_manipulability, 
    Eigen::VectorXd prev_best_config)
{
    Eigen::MatrixXd nullspace = computeNullspace(robot_state, jmg);
    
    Eigen::VectorXd new_best_config = gridSearch(robot_state, jmg, kinematics_metrics, prev_manipulability, prev_best_config, nullspace);

    double new_manipulability;
    robot_state.setJointGroupPositions(jmg, std::vector<double>(new_best_config.data(), new_best_config.data() + new_best_config.size()));
    robot_state.updateLinkTransforms();

    if (!kinematics_metrics->getManipulabilityIndex(robot_state, jmg, new_manipulability, false))
    {
        RCLCPP_ERROR(rclcpp::get_logger("nullspace_exploration"), "Failed to compute manipulability index.");
        return prev_best_config;
    }

    if (new_manipulability <= prev_manipulability + 1e-6)
    {
        RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Manipulability improvement too small. Stopping exploration with highest manipulability index: %f", prev_manipulability);
        
        return prev_best_config;
    }

    return recursiveNullspaceExploration(robot_state, jmg, kinematics_metrics, new_manipulability, new_best_config);
}

// Grid Search zur Optimierung der Manipulierbarkeit
Eigen::VectorXd gridSearch(moveit::core::RobotState& robot_state, 
                           const moveit::core::JointModelGroup* jmg, 
                           std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics,
                           double best_manipulability, Eigen::VectorXd best_config, Eigen::MatrixXd nullspace)
{
    double min_scale = -0.01;
    double max_scale = 0.01;
    double step_size = 0.001;

    for (std::size_t i = 0; i < static_cast<std::size_t>(nullspace.cols()); ++i)
    {
        for (double scale = min_scale; scale <= max_scale; scale += step_size)
        {
            Eigen::VectorXd new_config = best_config + scale * nullspace.col(i);

            std::vector<double> new_config_vec(new_config.data(), new_config.data() + new_config.size());
            robot_state.setJointGroupPositions(jmg, new_config_vec);
            if (!robot_state.satisfiesBounds(jmg))
                continue;
            
            double manipulability_index;

            robot_state.updateLinkTransforms();
            bool new_success = kinematics_metrics->getManipulabilityIndex(robot_state, jmg, manipulability_index, false);
            // RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Current Manipulability Index: %f", manipulability_index);
            if (!new_success)
                continue;

            if (manipulability_index > best_manipulability)
            {
                best_manipulability = manipulability_index;
                best_config = new_config;

                // RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "New best manipulability index: %f", best_manipulability);
            }
        }
    }

    // RCLCPP_INFO(rclcpp::get_logger("nullspace_exploration"), "Best configuration found with highest manipulability index: %f", best_manipulability);
    return best_config;
}
