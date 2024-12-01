#define ARM_SEG_LEN     0.337 //m
#define J1_FROM_FRONT   0.149 //m
#define END_EFF_LEN     0.3   //m
#define SHOULDER_H      0.656 //m
#define J1_ANG_OFF      0.5 //rad
#define STEP_PERC       0.1 //PU


#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std;
using namespace std::placeholders;

struct vec2 {
    double x, y;
    vec2() {};
    vec2(double x_, double y_){
        x = x_;
        y = y_;
    }

    vec2 operator* (const double m)
    {
        return vec2(x*m, y*m);
    }

    vec2 operator/ (const double d)
    {
        return vec2(x/d, y/d);
    }

    vec2 operator- (const vec2& v2)
    {
        return vec2(x-v2.x, y-v2.y);
    }

    vec2 operator+ (const vec2& v2)
    {
        return vec2(x+v2.x, y+v2.y);
    }
};

class ScratchCalc : public rclcpp::Node
{
public:
    using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using Duration = builtin_interfaces::msg::Duration;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    explicit ScratchCalc(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("scratch_calc", node_options)
    {
        string controller = "/arm_right_controller";

        this->client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            controller + "/follow_joint_trajectory");

        RCLCPP_INFO(this->get_logger(), "initialize action client");

        subscriber_dist_ = this->create_subscription<std_msgs::msg::Float64>("robot_gui/dist", 10, std::bind(&ScratchCalc::receiveDist, this, _1));
        subscriber_speed_ = this->create_subscription<std_msgs::msg::Float64>("robot_gui/speed", 10, std::bind(&ScratchCalc::receiveSpeed, this, _1));
        subscriber_top_ = this->create_subscription<std_msgs::msg::Float64>("robot_gui/top", 10, std::bind(&ScratchCalc::receiveTop, this, _1));
        subscriber_bot_ = this->create_subscription<std_msgs::msg::Float64>("robot_gui/bot", 10, std::bind(&ScratchCalc::receiveBot, this, _1));
        subscriber_start_ = this->create_subscription<std_msgs::msg::Bool>("robot_gui/start", 10, std::bind(&ScratchCalc::receiveStart, this, _1));
        subscriber_home_ = this->create_subscription<std_msgs::msg::Bool>("robot_gui/home", 10, std::bind(&ScratchCalc::receiveHome, this, _1));

        RCLCPP_INFO(this->get_logger(), "initialize subscribers");

        publisher_start_ = this->create_publisher<std_msgs::msg::Bool>("scratch_calc/start", 10);
        RCLCPP_INFO(this->get_logger(), "initialize publisher");

        goal = FollowJointTrajectory::Goal();
        goal.goal_time_tolerance = getDuration(0.5);
        goal.trajectory.joint_names = {
            "arm_right_1_joint",
            "arm_right_2_joint",
            "arm_right_3_joint",
            "arm_right_4_joint",
            "arm_right_5_joint",
            "arm_right_6_joint",
            "arm_right_7_joint"
        };


        initPath();
        addPointToPath(vec2(outDist, botHeight), 5);
        addLineToPath(vec2(outDist, botHeight), vec2(outDist, topHeight), 5);
        addLineToPath(vec2(outDist, topHeight), vec2(outDist, botHeight), 5);

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ScratchCalc::send_goal, this));
    }
    void send_goal();

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr     subscriber_dist_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr     subscriber_speed_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr     subscriber_top_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr     subscriber_bot_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr        subscriber_start_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr        subscriber_home_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        publisher_start_;

    rclcpp::TimerBase::SharedPtr timer_;

    double botHeight = 0.55;
    double topHeight = 0.75;

    double homeDist = 0.5;
    double outDist = 0.73;
    double timeSpend = 5;
    double time;

    bool continueScratching = false;

    vec2 curPose;

    FollowJointTrajectory::Goal goal;
    GoalHandle::SharedPtr goal_handle;

    void goal_response_callback(GoalHandle::SharedPtr goal_handler);
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);
    void result_callback(const GoalHandle::WrappedResult & result);

    void cancelGoal();

    double getTimeSpend(double yPerc, double totalTime, double yPercStep);
    builtin_interfaces::msg::Duration getDuration(double time);

    vec2 getPose(vector<double> j);
    std::vector<double> getAngles(vec2 pose);

    void initPath();
    void addPointToPath(vec2 pos, double timeSpend);
    void addLineToPath(vec2 initPos, vec2 endPos, double timeSpend);

    void receiveDist(const std_msgs::msg::Float64::SharedPtr msg);
    void receiveSpeed(const std_msgs::msg::Float64::SharedPtr msg);
    void receiveTop(const std_msgs::msg::Float64::SharedPtr msg);
    void receiveBot(const std_msgs::msg::Float64::SharedPtr msg);
    void receiveStart(const std_msgs::msg::Bool::SharedPtr msg);
    void receiveHome(const std_msgs::msg::Bool::SharedPtr msg);

    void publishStart(bool start);
};