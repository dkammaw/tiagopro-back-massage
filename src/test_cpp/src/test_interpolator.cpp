#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "test_cpp/nullspace_exploration.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <mutex>
#include <atomic>
#include "rclcpp/callback_group.hpp"



class InterpolationTestNode : public rclcpp::Node
{
public:
    InterpolationTestNode() : Node("interpolation_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "Interpolation Test Node gestartet!");
        // Create callback groups
        callback_group_tapping_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_state_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create the options object to specify callback group
        rclcpp::SubscriptionOptions options_tapping;
        options_tapping.callback_group = callback_group_tapping_; // Use the created callback group

        rclcpp::SubscriptionOptions options_state;
        options_state.callback_group = callback_group_state_; // Use the created callback group

        // Create the subscriptions with callback group in options
        tapping_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/tapping_positions", 10,
            std::bind(&InterpolationTestNode::tapping_positions_callback, this, std::placeholders::_1),
            options_tapping); // Pass the options object here

        robot_state_sub_ = this->create_subscription<moveit_msgs::msg::RobotState>(
            "robot_state_topic", 10,
            std::bind(&InterpolationTestNode::robot_state_callback, this, std::placeholders::_1),
            options_state); // Pass the options object here
    }

    void initialize()
    {
        nullspace_explorer_ = std::make_shared<NullspaceExplorationNode>();
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "arm_left");
        robot_model_ = move_group_->getRobotModel();
        jmg_ = robot_model_->getJointModelGroup("arm_left");
    }
    
    void robot_state_callback(const moveit_msgs::msg::RobotState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for tapping_positions...");

        // Warten mit Timeout statt Blockierung
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_.wait_for(lock, std::chrono::seconds(50), [this]() { return tapping_positions_ready; })) {
            RCLCPP_WARN(this->get_logger(), "Timeout: tapping_positions nicht erhalten!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "tapping_positions verfügbar!");
        
        moveit::core::RobotState updated_state(robot_model_);
        moveit::core::robotStateMsgToRobotState(*msg, updated_state);
        interpolateAndExecute(updated_state);
    }

    void tapping_positions_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->data.size(); i += 3)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = msg->data[i] - 0.18; // transformation arm_left_tool_link -> tip_link
            pose.position.y = msg->data[i + 1];
            pose.position.z = msg->data[i + 2];

            tf2::Quaternion desired_orientation;
            desired_orientation.setRPY(M_PI / 2, 0, M_PI / 2);
            desired_orientation.normalize();

            // Assign the fixed desired orientation
            pose.orientation = tf2::toMsg(desired_orientation);

            tapping_positions.push_back(pose);
        }

        tapping_positions_ready = true;
        RCLCPP_INFO(this->get_logger(), "Received %zu tapping positions!", tapping_positions.size());

        // Benachrichtige den wartenden Thread, dass die tapping_positions jetzt verfügbar sind
        cv_.notify_one();
    }




private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::core::RobotModelConstPtr robot_model_;
    const moveit::core::JointModelGroup* jmg_;
    std::shared_ptr<NullspaceExplorationNode> nullspace_explorer_;
    std::vector<geometry_msgs::msg::Pose> tapping_positions;
    bool tapping_positions_ready = false;
    // Mutex und Bedingungsvariable für Synchronisation
    std::mutex mutex_;
    std::condition_variable cv_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tapping_sub_;
    rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr robot_state_sub_;

    rclcpp::CallbackGroup::SharedPtr callback_group_tapping_;
    rclcpp::CallbackGroup::SharedPtr callback_group_state_;
    



    moveit::core::RobotState solveBestIK(moveit::core::RobotState& start_state, geometry_msgs::msg::Pose target_pose){
        
        moveit::core::RobotState end_state(start_state);
        // Set pose target
        move_group_->setPoseTarget(target_pose);

        // Plan to target pose
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "IK solution found!");

            // Get the final planned joint values
            std::vector<double> planned_joint_values = plan.trajectory_.joint_trajectory.points.back().positions;

            // Apply the planned joint values to end_state
            end_state.setJointGroupPositions(jmg_, planned_joint_values);

            RCLCPP_INFO(this->get_logger(), "Stored IK solution in end_state.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "IK solution not found!");
        }

        return nullspace_explorer_->explore(end_state);
    }


    void interpolateAndExecute(moveit::core::RobotState& start_state)
    {

        // Define Cartesian target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.56;
        target_pose.position.y = -0.11;
        target_pose.position.z = 0.62;

        // Convert RPY (Roll = π/2, Pitch = 0, Yaw = π/2) to quaternion
        tf2::Quaternion q;
        q.setRPY(M_PI / 2, 0, M_PI / 2);
        target_pose.orientation = tf2::toMsg(q);

        moveit::core::RobotState end_state(start_state);
        
        for(int i = 0; i < tapping_positions.size(); i++) {

            RCLCPP_INFO(this->get_logger(), "Planning to position[%d]: x=%f, y=%f, z=%f", i, tapping_positions[i].position.x, tapping_positions[i].position.y, tapping_positions[i].position.z);

            end_state = solveBestIK(start_state, tapping_positions[i]);
            jointSpaceInterpolator(start_state, end_state);
            start_state = end_state;
        }
    }

    void jointSpaceInterpolator(moveit::core::RobotState& start_state, const moveit::core::RobotState& end_state) {
        std::vector<moveit::core::RobotStatePtr> interpolated_states;
        const int num_waypoints = 10;
        for (int i = 0; i <= num_waypoints; ++i)
        {
            double t = static_cast<double>(i) / num_waypoints;
            auto interpolated_state = std::make_shared<moveit::core::RobotState>(start_state);
            start_state.interpolate(end_state, t, *interpolated_state);
            interpolated_states.push_back(interpolated_state);
        }
        moveit_msgs::msg::RobotTrajectory trajectory;
        trajectory.joint_trajectory.joint_names = jmg_->getVariableNames();

        for (size_t i = 0; i < interpolated_states.size(); ++i)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            interpolated_states[i]->copyJointGroupPositions(jmg_, point.positions);
            point.time_from_start = rclcpp::Duration::from_seconds(i * 0.2);
            trajectory.joint_trajectory.points.push_back(point);
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan2;
        plan2.trajectory_ = trajectory;

        RCLCPP_INFO(this->get_logger(), "Führe interpolierte Bewegung aus...");
        // Apply global velocity scaling (optional)
        move_group_->setMaxVelocityScalingFactor(0.1);  // 30% of max speed
        move_group_->setMaxAccelerationScalingFactor(0.1);
        move_group_->execute(plan2);
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterpolationTestNode>();
    node->initialize();  // Erst nach shared_ptr-Erstellung aufrufen
    // MultiThreaded Executor erstellen
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);  // Node zum Executor hinzufügen
    executor.spin();  // Callbacks parallel ausführen lassen
    rclcpp::shutdown();
    return 0;
}





