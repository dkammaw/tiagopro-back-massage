#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <cmath>


class IKSolver : public rclcpp::Node
{
public:
    IKSolver() : Node("ik_solver")
    {
        RCLCPP_INFO(this->get_logger(), "IK Solver Node Initialized.");
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rviz_visual_tools", 10);
        tapping_positions_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/tapping_positions", 10, 
            std::bind(&IKSolver::tapping_positions_callback, this, std::placeholders::_1)
        );
    }

    void initialize_move_group()
    {
        move_group= std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_left");
        robot_model_ = std::const_pointer_cast<moveit::core::RobotModel>(move_group->getRobotModel());
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    }

    void tapping_positions_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Define start pose
        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x = 0.283;
        start_pose.position.y = 0.208;
        start_pose.position.z = 0.480;
        start_pose.orientation.w = 1.0;  // Neutral quaternion (no rotation)


        // Convert flat array into a vector of Pose objects
        std::vector<geometry_msgs::msg::Pose> positions;
        for (size_t i = 0; i < msg->data.size(); i += 3)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = msg->data[i] - 0.25;
            pose.position.y = msg->data[i + 1];
            pose.position.z = msg->data[i + 2];
            pose.orientation.w = 1.0;  // Keep orientation neutral for now

            positions.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Received %zu tapping positions:", positions.size());

        for (size_t i = 0; i < positions.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Planning to position[%zu]: x=%f, y=%f, z=%f", 
                        i, positions[i].position.x, positions[i].position.y, positions[i].position.z);

            if (i == 0) {
                plan_cartesian_path(start_pose, positions[i], true);
            } else {
                plan_cartesian_path(positions[i - 1], positions[i], false);
            }
        }
    }

    // Function for linear interpolation between two points
    geometry_msgs::msg::Pose interpolate_pose(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &end, double t, bool interpolate_orientation) {
        geometry_msgs::msg::Pose interpolated_pose;
        
        // Interpolate the position
        interpolated_pose.position.x = start.position.x + t * (end.position.x - start.position.x);
        interpolated_pose.position.y = start.position.y + t * (end.position.y - start.position.y);
        interpolated_pose.position.z = start.position.z + t * (end.position.z - start.position.z);
        
        if (interpolate_orientation) {
            // For orientation, interpolate the quaternion
            tf2::Quaternion start_q(start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
            tf2::Quaternion end_q(end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w);
            
            tf2::Quaternion interpolated_q = start_q.slerp(end_q, t);
            
            interpolated_pose.orientation.x = interpolated_q.x();
            interpolated_pose.orientation.y = interpolated_q.y();
            interpolated_pose.orientation.z = interpolated_q.z();
            interpolated_pose.orientation.w = interpolated_q.w();
        }
        
        return interpolated_pose;
    }

    // Plan the Cartesian path
    void plan_cartesian_path(geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose target_pose, bool interpolate_orientation) {

        std::vector<geometry_msgs::msg::Pose> waypoints;

        waypoints.push_back(start_pose);
        if (interpolate_orientation) {
            // Define orientaion for target pose
            tf2::Quaternion q;
            q.setRPY(M_PI / 2, 0, M_PI / 2);  // Roll=90°, Pitch=0, Yaw=90°
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            target_pose.orientation.w = q.w();
        }

        // Interpolate between the start and target poses with a step size
        int num_steps = 5;  // Adjust this value to change the number of waypoints
        for (int i = 1; i <= num_steps; ++i) {
            double t = static_cast<double>(i) / (num_steps + 1);
            geometry_msgs::msg::Pose interpolated_pose = interpolate_pose(start_pose, target_pose, t, interpolate_orientation);
            waypoints.push_back(interpolated_pose);
        }
        // Add the final target pose
        waypoints.push_back(target_pose);


        // Calculate trajectory
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0; 
        const double eef_step = 0.01;
        double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(this->get_logger(), "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);

        publish_markers(waypoints);

        // Create and execute the plan
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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tapping_positions_sub_;

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto ik_solver_node = std::make_shared<IKSolver>();
    ik_solver_node->initialize_move_group();
    rclcpp::spin(ik_solver_node);
    rclcpp::shutdown();
    return 0;
}