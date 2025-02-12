#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

class IKSolver : public rclcpp::Node
{
public:
    IKSolver() : Node("ik_solver")
    {
        RCLCPP_INFO(this->get_logger(), "IK Solver Node Initialized.");
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rviz_visual_tools", 10);
    }

    void initialize_move_group()
    {
        move_group= std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");
        robot_model_ = std::const_pointer_cast<moveit::core::RobotModel>(move_group->getRobotModel());
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    }

    void plan_cartesian_path()
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Define a specific start pose
        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x = 0.55;  // Set the desired start position
        start_pose.position.y = -0.05;
        start_pose.position.z = 0.8;

        waypoints.push_back(start_pose);

        // Define target poses for Cartesian path
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z -= 0.2; // Move down
        waypoints.push_back(target_pose);

        target_pose.position.y -= 0.2; // Move right
        waypoints.push_back(target_pose);

        target_pose.position.z += 0.2; // Move up
        target_pose.position.y += 0.2; // Move left
        target_pose.position.x -= 0.2; // Move backward
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(this->get_logger(), "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);

        publish_markers(waypoints);
        
        // Plan the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        // Execute the plan
        move_group->execute(plan);
        RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully");

    }


private:
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    void publish_markers(const std::vector<geometry_msgs::msg::Pose>& waypoints)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Create marker for trajectory path
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "base_link";
        path_marker.header.stamp = this->now();
        path_marker.ns = "trajectory_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.01; // Line width
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0; // Green
        path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;

        // Add points to path marker
        for (const auto& pose : waypoints)
        {
            geometry_msgs::msg::Point point;
            point.x = pose.position.x;
            point.y = pose.position.y;
            point.z = pose.position.z;
            path_marker.points.push_back(point);
        }
        marker_array.markers.push_back(path_marker);

        // Create individual waypoints as spheres
        int id_counter = 1;
        for (const auto& pose : waypoints)
        {
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "base_link";
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "waypoints";
            waypoint_marker.id = id_counter++;
            waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
            waypoint_marker.scale.x = 0.05; // Sphere size
            waypoint_marker.scale.y = 0.05;
            waypoint_marker.scale.z = 0.05;
            waypoint_marker.color.r = 1.0; // Red
            waypoint_marker.color.g = 0.0;
            waypoint_marker.color.b = 0.0;
            waypoint_marker.color.a = 1.0;
            waypoint_marker.pose = pose;

            marker_array.markers.push_back(waypoint_marker);
        }

        // Publish markers
        marker_pub_->publish(marker_array);
    }
};

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKSolver>();
    node->initialize_move_group();
    node->plan_cartesian_path();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/*
#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

class IKSolver : public rclcpp::Node
{
public:
    IKSolver() : Node("ik_solver")
    {
        RCLCPP_INFO(this->get_logger(), "IK Solver Node Initialized.");
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        subscription_ = this->create_subscription<moveit_msgs::msg::RobotState>(
            "robot_state_topic", 10, std::bind(&IKSolver::robot_state_callback, this, std::placeholders::_1));
    }

    void initialize_move_group()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");
        robot_model_ = std::const_pointer_cast<moveit::core::RobotModel>(move_group_interface_->getRobotModel());
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    }

    void robot_state_callback(const moveit_msgs::msg::RobotState::SharedPtr msg)
    {
        moveit::core::RobotState current_state(robot_model_);
        robotStateMsgToRobotState(*msg, current_state);

        // Extract the start pose from the current robot state
        start_pose_ = current_state.getGlobalLinkTransform("base_link").translation();
        
        // Once the start pose is updated, you can call the Cartesian path function.
        plan_cartesian_path();
    }

    void plan_cartesian_path()
    {
        if (start_pose_.isZero()) {
            RCLCPP_WARN(this->get_logger(), "Start pose is not set yet.");
            return;
        }

        std::vector<geometry_msgs::msg::Pose> waypoints;
        
        geometry_msgs::msg::Pose start_pose;
        start_pose.position = start_pose_; // Use the pose extracted from the callback

        waypoints.push_back(start_pose);

        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z -= 0.2; // Move down
        waypoints.push_back(target_pose);

        target_pose.position.y -= 0.2; // Move right
        waypoints.push_back(target_pose);

        target_pose.position.z += 0.2; // Move up
        target_pose.position.y += 0.2; // Move left
        target_pose.position.x -= 0.2; // Move backward
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(this->get_logger(), "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);

        publish_markers(waypoints);
    }

private:
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr subscription_;
    geometry_msgs::msg::Vector3 start_pose_;  // Member variable to store the start pose

    void publish_markers(const std::vector<geometry_msgs::msg::Pose>& waypoints)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Create marker for trajectory path
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "base_link";
        path_marker.header.stamp = this->now();
        path_marker.ns = "trajectory_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.01; // Line width
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0; // Green
        path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;

        // Add points to path marker
        for (const auto& pose : waypoints)
        {
            geometry_msgs::msg::Point point;
            point.x = pose.position.x;
            point.y = pose.position.y;
            point.z = pose.position.z;
            path_marker.points.push_back(point);
        }
        marker_array.markers.push_back(path_marker);

        // Create individual waypoints as spheres
        int id_counter = 1;
        for (const auto& pose : waypoints)
        {
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "base_link";
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "waypoints";
            waypoint_marker.id = id_counter++;
            waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
            waypoint_marker.scale.x = 0.05; // Sphere size
            waypoint_marker.scale.y = 0.05;
            waypoint_marker.scale.z = 0.05;
            waypoint_marker.color.r = 1.0; // Red
            waypoint_marker.color.g = 0.0;
            waypoint_marker.color.b = 0.0;
            waypoint_marker.color.a = 1.0;
            waypoint_marker.pose = pose;

            marker_array.markers.push_back(waypoint_marker);
        }

        // Publish markers
        marker_pub_->publish(marker_array);
    }
};

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKSolver>();
    node->initialize_move_group();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/