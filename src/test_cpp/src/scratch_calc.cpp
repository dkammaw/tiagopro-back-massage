#include "scratch_calc.hpp"

using namespace std;
using namespace std::placeholders;

builtin_interfaces::msg::Duration ScratchCalc::getDuration(double time)
{
    auto duration = Duration();
    duration.sec = (int)(time);
    duration.nanosec =  (int)((time-std::floor(time)) * 1000000000);
    return duration;
}

void ScratchCalc::initPath()
{
    goal.trajectory.points.clear();
    time = 0;
}

void ScratchCalc::addLineToPath(vec2 initPos, vec2 endPos, double timeSpend)
{
    auto target = JointTrajectoryPoint();
    double i = 0;
    while (i < 1){
        time += getTimeSpend(i, timeSpend, STEP_PERC);
        target.time_from_start = getDuration(time);

        vec2 move = (endPos - initPos) * i + initPos;
        target.positions = getAngles(move);

        goal.trajectory.points.push_back(target);
        i += STEP_PERC;
    }
}

void ScratchCalc::addPointToPath(vec2 pos, double timeSpend)
{
    auto target = JointTrajectoryPoint();
    time += timeSpend;
    target.time_from_start = getDuration(time);
    target.positions = getAngles(pos);
    goal.trajectory.points.push_back(target);
}

void ScratchCalc::send_goal()
{
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ScratchCalc::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&ScratchCalc::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&ScratchCalc::result_callback, this, _1);
    
    this->client_ptr_->async_send_goal(goal, send_goal_options);
}

void ScratchCalc::goal_response_callback(GoalHandle::SharedPtr goal_handler)
{
    goal_handle = goal_handler;
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server :(");
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ScratchCalc::send_goal, this));
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result :)");
    }
}

void ScratchCalc::feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
{
    curPose = getPose(feedback->actual.positions);
}

void ScratchCalc::result_callback(const GoalHandle::WrappedResult & result)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "result" << result.result->error_code);
    if (continueScratching){
        initPath();
        addLineToPath(vec2(outDist, botHeight), vec2(outDist, topHeight), timeSpend);
        addLineToPath(vec2(outDist, topHeight), vec2(outDist, botHeight), timeSpend);
        send_goal();
    } else {
        publishStart(false);
    }
    // setPath(0);
    // send_goal();
    // rclcpp::shutdown();
}

void ScratchCalc::cancelGoal()
{
    if (goal_handle) {
        auto status = goal_handle->get_status();
        if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
            RCLCPP_INFO(this->get_logger(), "cancelling goal");
            auto cancel_result_future = client_ptr_->async_cancel_goal(goal_handle);
            RCLCPP_INFO(this->get_logger(), "successfully cancelled goal");
        }
    }
}

std::vector<double> ScratchCalc::getAngles(vec2 pose)
{
    double x = pose.x + J1_FROM_FRONT - END_EFF_LEN;
    double y = SHOULDER_H - pose.y;
    
    RCLCPP_INFO_STREAM(this->get_logger(), "x " << pose.x << " y " << pose.y);
    RCLCPP_INFO_STREAM(this->get_logger(), "x " << x << " y " << y);

    double r = sqrt(x*x + y*y);
    RCLCPP_INFO_STREAM(this->get_logger(), "radius " << r);

    if (r > 2*ARM_SEG_LEN || r < 0) {
        throw runtime_error("radius is invalid");
    }

    double cos_angle2 = 1 - pow(r, 2) / (2 * pow(ARM_SEG_LEN, 2));
    double a2 = acos(cos_angle2);

    double sin_angle1 = ARM_SEG_LEN * (sin(a2) / r);
    double a1 = -asin(sin_angle1) - atan2(y, x) + J1_ANG_OFF;

    a2 = a2 - M_PI;
    double a3 = a1 - a2 - J1_ANG_OFF;

    vector<double> joints = {0.0, a1, 0.0, a2, 0.0, a3, 0.0};
    return joints;
}

vec2 ScratchCalc::getPose(vector<double> j)
{
    double x = ARM_SEG_LEN * cos(j[1] + J1_ANG_OFF) + 
                ARM_SEG_LEN * cos(j[3] + j[1] + J1_ANG_OFF) + 
                END_EFF_LEN * cos(j[5] + j[3] + j[1] + J1_ANG_OFF);
    
    double y = ARM_SEG_LEN * sin(j[1] + J1_ANG_OFF) + 
                ARM_SEG_LEN * sin(j[3] + j[1] + J1_ANG_OFF) + 
                END_EFF_LEN * sin(j[5] + j[3] + j[1] + J1_ANG_OFF);

    RCLCPP_INFO_STREAM(this->get_logger(), "curPose: " << x - J1_FROM_FRONT << " " << y + SHOULDER_H);

    return vec2(x - J1_FROM_FRONT, y + SHOULDER_H);
}

double ScratchCalc::getTimeSpend(double yPerc, double totalTime, double yPercStep)
{
    double yInit = yPerc - yPercStep;
    double timeEnd = (cos(M_PI*yPerc) * sin(M_PI*yPerc)) / M_PI + yPerc;
    double timeInit = (cos(M_PI*yInit) * sin(M_PI*yInit)) / M_PI + yInit;
    return (timeEnd - timeInit) * totalTime;
}

void ScratchCalc::receiveDist(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received Dist");
    outDist = msg->data;
    cancelGoal();

    initPath();
    addPointToPath(vec2(outDist, curPose.y), 5);
    continueScratching = false;
    send_goal();
}

void ScratchCalc::receiveSpeed(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received Speed");
    timeSpend = msg->data;
}

void ScratchCalc::receiveTop(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received Top");
    topHeight = msg->data;
}

void ScratchCalc::receiveBot(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received Bot");
    botHeight = msg->data;
}

void ScratchCalc::receiveStart(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received Start");
    initPath();
    if (!msg->data){
        cancelGoal();
        continueScratching = false;
        return;
    }

    if (abs(curPose.x - outDist) < 0.05){
        RCLCPP_INFO(this->get_logger(), "on path");
        RCLCPP_INFO_STREAM(this->get_logger(), "curPose" << curPose.x << curPose.y);

        addLineToPath(curPose, vec2(outDist, topHeight), timeSpend);
        RCLCPP_INFO(this->get_logger(), "added Line1");
        addLineToPath(vec2(outDist, topHeight), vec2(outDist, botHeight), timeSpend);
        RCLCPP_INFO(this->get_logger(), "added Line2");
    } else {
        RCLCPP_INFO(this->get_logger(), "off path");

        addPointToPath(vec2(outDist, curPose.y), 5);
        RCLCPP_INFO(this->get_logger(), "added point");
        addLineToPath(vec2(outDist, curPose.y), vec2(outDist, topHeight), timeSpend);
        RCLCPP_INFO(this->get_logger(), "added Line1");
        addLineToPath(vec2(outDist, topHeight), vec2(outDist, botHeight), timeSpend);
        RCLCPP_INFO(this->get_logger(), "added Line2");
    }
    continueScratching = true;
    send_goal();
}

void ScratchCalc::receiveHome(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "received Home");
    cancelGoal();
    initPath();
    if (!msg->data){
        addPointToPath(vec2(homeDist, SHOULDER_H), 5);
    } else {
        addPointToPath(vec2(outDist, SHOULDER_H), 5);
    }
    continueScratching = false;
    send_goal();
}

void ScratchCalc::publishStart(bool start)
{
    auto msg = std_msgs::msg::Bool();
    msg.data = start;
    publisher_start_->publish(msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScratchCalc>());
    rclcpp::shutdown();
    return 0;
}






// void ScratchCalc::addLineToPath(double initPos, double endPos, double timeSpend)
// {
//     auto target = JointTrajectoryPoint();
//     double i = 0;
//     while (i < 1){
//         time += getTimeSpend(i, timeSpend, STEP_PERC);
//         target.time_from_start = getDuration(time);
//         double height = (endPos - initPos) * i + initPos;
//         armAngles j = getAngles(dist, height);
//         target.positions = {
//             0.0,    // shoulder inline twist
//             j.a1,   // joint 1 perp ang
//             0.0,    // joint 1 inline twist
//             j.a2,   // joint 2 perp ang
//             0.0,    // joint 2 inline twist
//             j.a3,   // joint 3 perp ang
//             0.0     // joint 3 inline twist
//         };
//         goal.trajectory.points.push_back(target);
//         i += STEP_PERC;
//     }
// }

// void ScratchCalc::addPointToPath(double pos, double time)
// {
//     auto target = JointTrajectoryPoint();
//     target.time_from_start = getDuration(time);
//     armAngles j = getAngles(dist, botHeight);
//     target.positions = {
//         0.0,    // shoulder inline twist
//         j.a1,   // joint 1 perp ang
//         0.0,    // joint 1 inline twist
//         j.a2,   // joint 2 perp ang
//         0.0,    // joint 2 inline twist
//         j.a3,   // joint 3 perp ang
//         0.0     // joint 3 inline twist
//     };
//     goal.trajectory.points.push_back(target);
// }

// armAngles ScratchCalc::getAngles(double x, double y)
// {
//     double x_ = x + J1_FROM_FRONT - END_EFF_LEN;
//     double y_ = SHOULDER_H - y;

//     double r = sqrt(x_*x_ + y_*y_);
//     if (r > 2*ARM_SEG_LEN || r < 0) {
//         throw runtime_error("radius is invalid");
//     }

//     double cos_angle2 = 1 - pow(r, 2) / (2 * pow(ARM_SEG_LEN, 2));
//     double a2 = acos(cos_angle2);

//     double sin_angle1 = ARM_SEG_LEN * (sin(a2) / r);
//     double a1 = -asin(sin_angle1) - atan2(y_, x_) + J1_ANG_OFF;

//     a2 = a2 - M_PI;
//     double a3 = a1 - a2 - J1_ANG_OFF;

//     return armAngles(a1, a2, a3);
// }


/* TODO:
pause scratch when moving to a new distance
send shutdown message when qt gets shut down (to both nodes)
get all data from qt
*/