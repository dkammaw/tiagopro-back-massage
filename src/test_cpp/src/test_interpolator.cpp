#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class InterpolationTestNode : public rclcpp::Node
{
public:
    InterpolationTestNode() : Node("interpolation_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "Interpolation Test Node gestartet!");
        // Abonnieren des Robot State Topics
        subscription_ = this->create_subscription<moveit_msgs::msg::RobotState>(
            "robot_state_topic", 10, std::bind(&InterpolationTestNode::robot_state_callback, this, std::placeholders::_1));
    }


    void initialize()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "arm_left");
        robot_model_ = move_group_->getRobotModel();
        joint_model_group_ = robot_model_->getJointModelGroup("arm_left");
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

    }

    // Callback zur Speicherung des aktuellen Roboterzustands
    void robot_state_callback(const moveit_msgs::msg::RobotState::SharedPtr msg)
    {
        moveit::core::RobotState updated_state(robot_model_);
        moveit::core::robotStateMsgToRobotState(*msg, updated_state);
    

        // Starte die Interpolation mit dem aktuellen Zustand
        interpolateAndExecute(updated_state);
    }


private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::core::RobotModelConstPtr robot_model_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::shared_ptr<moveit::core::RobotState> robot_state_;
    rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr subscription_;

    void interpolateAndExecute(const moveit::core::RobotState& start_state)
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
            end_state.setJointGroupPositions(joint_model_group_, planned_joint_values);

            RCLCPP_INFO(this->get_logger(), "Stored IK solution in end_state.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "IK solution not found!");
        }

        /*
        std::vector<double> target_joint_positions = {0.73, -0.77, -0.72, -1.89, -0.12, -0.4, -0.82};
        end_state.setJointGroupPositions(joint_model_group_, target_joint_positions);
        */


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
        trajectory.joint_trajectory.joint_names = joint_model_group_->getVariableNames();

        for (size_t i = 0; i < interpolated_states.size(); ++i)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            interpolated_states[i]->copyJointGroupPositions(joint_model_group_, point.positions);
            point.time_from_start = rclcpp::Duration::from_seconds(i * 0.2);
            trajectory.joint_trajectory.points.push_back(point);
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan2;
        plan2.trajectory_ = trajectory;


        RCLCPP_INFO(this->get_logger(), "Führe interpolierte Bewegung aus...");
        move_group_->execute(plan2);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterpolationTestNode>();
    node->initialize();  // Erst nach shared_ptr-Erstellung aufrufen
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




