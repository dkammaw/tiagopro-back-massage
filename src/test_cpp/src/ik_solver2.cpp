#include "test_cpp/ik_solver2.hpp"

IKSolver::IKSolver() : Node("ik_solver")
{
    RCLCPP_INFO(this->get_logger(), "IK Solver Node Initialized.");
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rviz_visual_tools", 10);
    tapping_positions_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/tapping_positions", 10, 
        std::bind(&IKSolver::tapping_positions_callback, this, std::placeholders::_1)
    );
    // Abonnieren des Robot State Topics
    subscription_ = this->create_subscription<moveit_msgs::msg::RobotState>(
        "robot_state_topic", 10, std::bind(&NullspaceExplorationNode::robot_state_callback, this, std::placeholders::_1));

}

void IKSolver::initialize_move_group()
{
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "arm_left");
    robot_model_ = std::const_pointer_cast<moveit::core::RobotModel>(move_group->getRobotModel());
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
}

// Callback zur Speicherung des aktuellen Roboterzustands
void robot_state_callback(const moveit_msgs::msg::RobotState::SharedPtr msg)
{
    // Aktualisiere die Member-Variable `robot_state_`
    robotStateMsgToRobotState(*msg, *robot_state_);
}

// Zugriffsmethode für den aktuellen Roboterzustand
moveit::core::RobotState getCurrentRobotState() const
{
    return *robot_state_;  // Rückgabe einer Kopie
}

void IKSolver::tapping_positions_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // Define start pose
    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = 0.283;
    start_pose.position.y = 0.208;
    start_pose.position.z = 0.480;
    start_pose.orientation.w = 1.0;  // Neutral quaternion (no rotation)

    tf2::Quaternion desired_orientation;
    desired_orientation.setRPY(M_PI / 2, 0, M_PI / 2);
    desired_orientation.normalize();

    // Convert flat array into a vector of Pose objects
    std::vector<geometry_msgs::msg::Pose> positions;

    for (size_t i = 0; i < msg->data.size(); i += 3)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = msg->data[i] - 0.18; // transformation arm_left_tool_link -> tip_link
        pose.position.y = msg->data[i + 1];
        pose.position.z = msg->data[i + 2];

        // Assign the fixed desired orientation
        pose.orientation = tf2::toMsg(desired_orientation);

        positions.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Received %zu tapping positions:", positions.size());

    for (size_t i = 0; i < positions.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Planning to position[%zu]: x=%f, y=%f, z=%f", 
                    i, positions[i].position.x, positions[i].position.y, positions[i].position.z);

        plan_cartesian_path(i == 0 ? start_pose : positions[i - 1], positions[i], i == 0);
    }
}


void IKSolver::plan_cartesian_path(geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose target_pose, bool interpolate_orientation) {


    // 3. RobotState für IK-Suche erstellen
    moveit::core::RobotState target_state(robot_model_);

    // 4. Inverse Kinematik berechnen
    bool found_ik = target_state.setFromIK(joint_model_group, target_pose);

    #target_state = nullspace_explorer(target_state)

    moveit::core::RobotState start_state = getCurrentRobotState()

    auto interpolated_state = std::make_shared<moveit::core::RobotState>(*start_state);
    start_state->interpolate(target_state, t, *interpolated_state);


}


void IKSolver::publish_markers(const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Create marker for trajectory path
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "base_footprint";
    path_marker.header.stamp = this->now();
    path_marker.ns = "trajectory_path";
    path_marker.id = path_counter++;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.012; // Line width
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0; 
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;

    // Add points to path marker
    for (const auto& pose : waypoints)
    {
        geometry_msgs::msg::Point point;
        point.x = pose.position.x + 0.16;
        point.y = pose.position.y;
        point.z = pose.position.z;
        path_marker.points.push_back(point);
    }
    marker_array.markers.push_back(path_marker);

    // Create individual waypoints as spheres
    for (const auto& pose : waypoints)
    {
        visualization_msgs::msg::Marker waypoint_marker;
        waypoint_marker.header.frame_id = "base_footprint";
        waypoint_marker.header.stamp = this->now();
        waypoint_marker.ns = "waypoints";
        waypoint_marker.id = id_counter++;
        waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
        waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
        waypoint_marker.scale.x = 0.012; // Sphere size
        waypoint_marker.scale.y = 0.012;
        waypoint_marker.scale.z = 0.012;
        waypoint_marker.color.r = 0.0; 
        waypoint_marker.color.g = 0.0;
        waypoint_marker.color.b = 1.0;
        waypoint_marker.color.a = 1.0;
        waypoint_marker.pose = pose;
        waypoint_marker.pose.position.x += 0.16;
        marker_array.markers.push_back(waypoint_marker);
    }

    // Publish markers
    marker_pub_->publish(marker_array);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto ik_solver_node = std::make_shared<IKSolver>();
    ik_solver_node->initialize_move_group();
    rclcpp::spin(ik_solver_node);
    rclcpp::shutdown();
    return 0;
}