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

        // Startpose definieren
        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x = 0.283;
        start_pose.position.y = 0.208;
        start_pose.position.z = 0.480;
        start_pose.orientation.w = 1.0;  // Neutraler Quaternion (kein Drehung)

        waypoints.push_back(start_pose);

        // 1. Waypoint (gleiche Orientierung)
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z += 0.1;
        target_pose.position.x += 0.1;
        waypoints.push_back(target_pose);

        // 2. Waypoint (gleiche Orientierung)
        target_pose.position.z += 0.1;
        target_pose.position.x += 0.1;
        waypoints.push_back(target_pose);

        // 3. Waypoint: Orientierung ändern (z.B. um 90° um Z-Achse drehen)
        target_pose.position.z += 0.1;
        target_pose.position.x += 0.1;
        
        tf2::Quaternion q;
        q.setRPY(M_PI / 2, 0, M_PI / 2);  // Roll=90°, Pitch=0, Yaw=90°
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();

        waypoints.push_back(target_pose);

        // Trajektorie berechnen
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0; 
        const double eef_step = 0.01;
        double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(this->get_logger(), "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);

        publish_markers(waypoints);

        // Plan erstellen und ausführen
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
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