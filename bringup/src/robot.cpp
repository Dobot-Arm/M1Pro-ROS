/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <bringup/robot.h>

M1ProRobot::M1ProRobot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false)
    , goal_{}
    , control_nh_(nh)
    , trajectory_duration_(1.0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}

M1ProRobot::~M1ProRobot()
{
    ROS_INFO("~M1ProRobot");
}

void M1ProRobot::init()
{
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.1.6");

    trajectory_duration_ = control_nh_.param("trajectory_duration", 0.3);
    ROS_INFO("trajectory_duration : %0.2f", trajectory_duration_);

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/EnableRobot", &M1ProRobot::enableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DisableRobot", &M1ProRobot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ClearError", &M1ProRobot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ResetRobot", &M1ProRobot::resetRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SpeedFactor", &M1ProRobot::speedFactor, this));
    // server_tbl_.push_back(
    //     control_nh_.advertiseService("/bringup/srv/DigitalOutputs", &M1ProRobot::DigitalOutputs, this));    // DO
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetErrorID", &M1ProRobot::getErrorID, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/User", &M1ProRobot::user, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Tool", &M1ProRobot::tool, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RobotMode", &M1ProRobot::robotMode, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/PayLoad", &M1ProRobot::payload, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DO", &M1ProRobot::DO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DOExecute", &M1ProRobot::DOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ToolDO", &M1ProRobot::toolDO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ToolDOExecute", &M1ProRobot::toolDOExecute, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AO", &M1ProRobot::AO, this));   // 四轴无此指令
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AOExecute", &M1ProRobot::AOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AccJ", &M1ProRobot::accJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AccL", &M1ProRobot::accL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SpeedJ", &M1ProRobot::speedJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SpeedL", &M1ProRobot::speedL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Arch", &M1ProRobot::arch, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/CP", &M1ProRobot::cp, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/LimZ", &M1ProRobot::limZ, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/SetArmOrientation", &M1ProRobot::setArmOrientation, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetPayload", &M1ProRobot::SetPayload, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/PositiveSolution", &M1ProRobot::PositiveSolution, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/InverseSolution", &M1ProRobot::InverseSolution, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/PowerOn", &M1ProRobot::powerOn, this));   //
    // 四轴无此指令
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RunScript", &M1ProRobot::runScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StopScript", &M1ProRobot::stopScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/PauseScript", &M1ProRobot::PauseScript, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/ContinueScript", &M1ProRobot::continueScript, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetSafeSkin", &M1ProRobot::setSafeSkin, this));
    // // 四轴无此指令
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/SetObstacleAvoid", &M1ProRobot::setObstacleAvoid, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/SetCollisionLevel", &M1ProRobot::setCollisionLevel, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetAngle", &M1ProRobot::GetAngle, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetPose", &M1ProRobot::GetPose, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/EmergencyStop", &M1ProRobot::emergencyStop, this));

    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovJ", &M1ProRobot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovL", &M1ProRobot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/JointMovJ", &M1ProRobot::jointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Jump", &M1ProRobot::jump, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovJ", &M1ProRobot::relMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovL", &M1ProRobot::relMovL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovLIO", &M1ProRobot::movLIO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovJIO", &M1ProRobot::movJIO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Arc", &M1ProRobot::arc, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Circle", &M1ProRobot::circle, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovJUser", &M1ProRobot::RelMovJUser, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovLUser", &M1ProRobot::RelMovLUser, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelJointMovJ", &M1ProRobot::RelJointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovJExt", &M1ProRobot::MovJExt, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ServoJ", &M1ProRobot::servoJ, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ServoP", &M1ProRobot::servoP, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Sync", &M1ProRobot::sync, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartTrace", &M1ProRobot::startTrace, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartPath", &M1ProRobot::startPath, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartFCTrace", &M1ProRobot::startFCTrace,
    // this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MoveJog", &M1ProRobot::moveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StopmoveJog", &M1ProRobot::stopmoveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SyncAll", &M1ProRobot::SyncAll, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Wait", &M1ProRobot::Wait, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Continue", &M1ProRobot::Continue, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Pause", &M1ProRobot::Pause, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ModbusCreate", &M1ProRobot::modbusCreate, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ModbusClose", &M1ProRobot::modbusClose, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetInBits", &M1ProRobot::getInBits, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetInRegs", &M1ProRobot::getInRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetCoils", &M1ProRobot::getCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetCoils", &M1ProRobot::setCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetHoldRegs", &M1ProRobot::getHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetHoldRegs", &M1ProRobot::setHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ToolDI", &M1ProRobot::ToolDI, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DI", &M1ProRobot::DI, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DOGroup", &M1ProRobot::DOGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/BrakeControl", &M1ProRobot::brakeControl, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartDrag", &M1ProRobot::startDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StopDrag", &M1ProRobot::stopDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/LoadSwitch", &M1ProRobot::loadSwitch, this));

    // registerGoalCallback(boost::bind(&M1ProRobot::goalHandle, this, _1));
    // registerCancelCallback(boost::bind(&M1ProRobot::cancelHandle, this, _1));
    start();
}

// void M1ProRobot::feedbackHandle(const ros::TimerEvent& tm,
//                                 ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     control_msgs::FollowJointTrajectoryFeedback feedback;

//     double current_joints[6];
//     getJointState(current_joints);

//     for (uint32_t i = 0; i < 6; i++)
//     {
//         feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
//         feedback.actual.positions.push_back(current_joints[i]);
//         feedback.desired.positions.push_back(goal_[i]);
//     }

//     handle.publishFeedback(feedback);
// }

// void M1ProRobot::moveHandle(const ros::TimerEvent& tm,
//                             ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

//     if (index_ < trajectory->trajectory.points.size())
//     {
//         auto point = trajectory->trajectory.points[index_].positions;
//         double tmp[6];
//         for (uint32_t i = 0; i < 4; i++)
//         {
//             tmp[i] = point[i] * 180.0 / 3.1415926;
//         }

//         bringup::ServoJ srv;
//         srv.request.j1 = tmp[0];
//         srv.request.j2 = tmp[1];
//         srv.request.j3 = tmp[2];
//         srv.request.j4 = tmp[3];
//         servoJ(srv.request, srv.response);
//         index_++;
//     }
//     else
//     {
// #define OFFSET_VAL 0.01
//         double current_joints[6];
//         getJointState(current_joints);
//         if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
//             (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
//             (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
//             (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL))
//         {
//             timer_.stop();
//             movj_timer_.stop();
//             handle.setSucceeded();
//         }
//     }
// }

// void M1ProRobot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     index_ = 0;
//     for (uint32_t i = 0; i < 6; i++)
//     {
//         goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
//     }
//     timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&M1ProRobot::feedbackHandle, this, _1, handle));
//     movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),
//                                           boost::bind(&M1ProRobot::moveHandle, this, _1, handle));
//     timer_.start();
//     movj_timer_.start();
//     handle.setAccepted();
// }

// void M1ProRobot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     timer_.stop();
//     movj_timer_.stop();
//     handle.setSucceeded();
// }

void M1ProRobot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

bool M1ProRobot::isEnable() const
{
    return commander_->isEnable();
}

bool M1ProRobot::isConnected() const
{
    return commander_->isConnected();
}

void M1ProRobot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *----------------------------------------------------------------------------------------------------------------------
 */

bool M1ProRobot::enableRobot(bringup::EnableRobot::Request& request, bringup::EnableRobot::Response& response)
{
    try
    {
        char cmd[1000];
        std::string str{ "EnableRobot(" };
        for (int i = 0; i < request.args.size(); i++)
        {
            if (i == request.args.size() - 1)
            {
                str = str + std::to_string(request.args[i]);
                break;
            }
            str = str + std::to_string(request.args[i]) + ",";
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const std::exception& err)
    {
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::disableRobot(bringup::DisableRobot::Request& request, bringup::DisableRobot::Response& response)
{
    try
    {
        const char* cmd = "DisableRobot()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const std::exception& err)
    {
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::clearError(bringup::ClearError::Request& request, bringup::ClearError::Response& response)
{
    try
    {
        const char* cmd = "ClearError()";
        commander_->dashboardDoCmd(cmd, response.res);
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::resetRobot(bringup::ResetRobot::Request& request, bringup::ResetRobot::Response& response)
{
    try
    {
        const char* cmd = "ResetRobot()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::speedFactor(bringup::SpeedFactor::Request& request, bringup::SpeedFactor::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedFactor(%d)", request.ratio);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*  DO
bool M1ProRobot::DigitalOutputs(bringup::DigitalOutputs::Request& request,
                                bringup::DigitalOutputs::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DigitalOutputs(%d,%d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

bool M1ProRobot::getErrorID(bringup::GetErrorID::Request& request, bringup::GetErrorID::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "GetErrorID()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::user(bringup::User::Request& request, bringup::User::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "User(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::tool(bringup::Tool::Request& request, bringup::Tool::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Tool(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::robotMode(bringup::RobotMode::Request& request, bringup::RobotMode::Response& response)
{
    try
    {
        const char* cmd = "RobotMode()";

        std::vector<std::string> result;
        commander_->dashboardDoCmd(cmd, response.res, result);
        if (result.empty())
            throw std::logic_error("robotMode : Empty string");

        response.mode = str2Int(result[0].c_str());
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
    catch (const std::exception& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::payload(bringup::PayLoad::Request& request, bringup::PayLoad::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "PayLoad(%0.3f, %0.3f)", request.weight, request.inertia);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::DO(bringup::DO::Request& request, bringup::DO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::DOExecute(bringup::DOExecute::Request& request, bringup::DOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DOExecute(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::toolDO(bringup::ToolDO::Request& request, bringup::ToolDO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDO(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::toolDOExecute(bringup::ToolDOExecute::Request& request, bringup::ToolDOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDOExecute(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
bool M1ProRobot::AO(bringup::AO::Request& request, bringup::AO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

/*
bool M1ProRobot::AOExecute(bringup::AOExecute::Request& request, bringup::AOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AOExecute(%d, %0.3f)", request.index, static_cast<float>(request.value));
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

bool M1ProRobot::accJ(bringup::AccJ::Request& request, bringup::AccJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccJ(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::accL(bringup::AccL::Request& request, bringup::AccL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccL(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::speedJ(bringup::SpeedJ::Request& request, bringup::SpeedJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedJ(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::speedL(bringup::SpeedL::Request& request, bringup::SpeedL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedL(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::arch(bringup::Arch::Request& request, bringup::Arch::Response& response)
{
    try
    {
        char cmd[100];
        if (request.cpValue.size() == 0)
        {
            sprintf(cmd, "Arch(%d)", request.index);
        }
        else
        {
            sprintf(cmd, "Arch(%d,%s)", request.index, request.cpValue[0].c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::cp(bringup::CP::Request& request, bringup::CP::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "CP(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::limZ(bringup::LimZ::Request& request, bringup::LimZ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "LimZ(%d)", request.value);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::setArmOrientation(bringup::SetArmOrientation::Request& request,
                                   bringup::SetArmOrientation::Response& response)
{
    try
    {
        char cmd[100];
        if (request.UorD == 0 && request.ForN == 0 && request.Config6 == 0)
        {
            sprintf(cmd, "SetArmOrientation(%d)", request.LorR);
        }
        else
        {
            sprintf(cmd, "SetArmOrientation(%d,%d,%d,%d)", request.LorR, request.UorD, request.ForN, request.Config6);
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::SetPayload(bringup::SetPayload::Request& request, bringup::SetPayload::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetPayload(%f)", request.load);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::PositiveSolution(bringup::PositiveSolution::Request& request,
                                  bringup::PositiveSolution::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "PositiveSolution(%0.3f,%0.3f,%0.3f,%0.3f,%d,%d)", request.offset1, request.offset2,
                request.offset3, request.offset4, request.user, request.tool);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::InverseSolution(bringup::InverseSolution::Request& request,
                                 bringup::InverseSolution::Response& response)
{
    try
    {
        char cmd[100];
        if (request.JointNear == "")
        {
            sprintf(cmd, "InverseSolution(%0.3f,%0.3f,%0.3f,%0.3f,%d,%d)", request.offset1, request.offset2,
                    request.offset3, request.offset4, request.user, request.tool);
        }
        else
        {
            sprintf(cmd, "InverseSolution(%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%s)", request.offset1, request.offset2,
                    request.offset3, request.offset4, request.user, request.tool, request.isJointNear,
                    request.JointNear.c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
bool M1ProRobot::powerOn(bringup::PowerOn::Request& request, bringup::PowerOn::Response& response)
{
    try
    {
        const char* cmd = "PowerOn()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

bool M1ProRobot::runScript(bringup::RunScript::Request& request, bringup::RunScript::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RunScript(%s)", request.projectName.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::stopScript(bringup::StopScript::Request& request, bringup::StopScript::Response& response)
{
    try
    {
        const char* cmd = "StopScript()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::PauseScript(bringup::PauseScript::Request& request, bringup::PauseScript::Response& response)
{
    try
    {
        const char* cmd = "PauseScript()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::modbusCreate(bringup::ModbusCreate::Request& request, bringup::ModbusCreate::Response& response)
{
    try
    {
        char cmd[300];
        std::vector<std::string> result;
        if (request.is_rtu.size() == 0)
        {
            snprintf(cmd, sizeof(cmd), "ModbusCreate(%s,%d,%d)", request.ip.c_str(), request.port, request.slave_id);
        }
        else
        {
            snprintf(cmd, sizeof(cmd), "ModbusCreate(%s,%d,%d,%d)", request.ip.c_str(), request.port, request.slave_id,
                     request.is_rtu[0]);
        }
        commander_->dashboardDoCmd(cmd, response.res, result);
        if (result.size() != 2)
            throw std::logic_error("Haven't recv any result");

        response.res = str2Int(result[0].c_str());
        response.index = str2Int(result[1].c_str());
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        response.index = -1;
        return false;
    }
    catch (const std::exception& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        response.index = -1;
        return false;
    }
}

bool M1ProRobot::modbusClose(bringup::ModbusClose::Request& request, bringup::ModbusClose::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ModbusClose(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::getInBits(bringup::GetInBits::Request& request, bringup::GetInBits::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "GetInBits(%d,%d,%d)", request.index, request.addr, request.count);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
bool M1ProRobot::getInRegs(bringup::GetInRegs::Request& request, bringup::GetInRegs::Response& response)
{
    try
    {
        char cmd[100];
        if (request.valType.size() == 0)
        {
            snprintf(cmd, sizeof(cmd), "GetInRegs(%d,%d,%d)", request.index, request.addr, request.count);
        }
        else
        {
            snprintf(cmd, sizeof(cmd), "GetInRegs(%d,%d,%d,%s)", request.index, request.addr, request.count,
                     request.valType[0].c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::getCoils(bringup::GetCoils::Request& request, bringup::GetCoils::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "GetCoils(%d,%d,%d)", request.index, request.addr, request.count);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::setCoils(bringup::SetCoils::Request& request, bringup::SetCoils::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "SetCoils(%d,%d,%d,%s)", request.index, request.addr, request.count,
                 request.valTab.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::getHoldRegs(bringup::GetHoldRegs::Request& request, bringup::GetHoldRegs::Response& response)
{
    try
    {
        char cmd[100];
        if (request.valType.size() == 0)
        {
            snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d)", request.index, request.addr, request.count);
        }
        else
        {
            snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d,%s)", request.index, request.addr, request.count,
                     request.valType[0].c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::setHoldRegs(bringup::SetHoldRegs::Request& request, bringup::SetHoldRegs::Response& response)
{
    try
    {
        char cmd[100];
        if (request.valType.size() == 0)
        {
            snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s)", request.index, request.addr, request.count,
                     request.valTab.c_str());
        }
        else
        {
            snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s,%s)", request.index, request.addr, request.count,
                     request.valTab.c_str(), request.valType[0].c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::DI(bringup::DI::Request& request, bringup::DI::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "DI(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::ToolDI(bringup::ToolDI::Request& request, bringup::ToolDI::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ToolDI(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::DOGroup(bringup::DOGroup::Request& request, bringup::DOGroup::Response& response)
{
    try
    {
        char cmd[1000];
        std::string str{ "DOGroup(" };
        for (int i = 0; i < request.args.size(); i++)
        {
            if (i == request.args.size() - 1)
            {
                str = str + std::to_string(request.args[i]);
                break;
            }
            str = str + std::to_string(request.args[i]) + ",";
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::brakeControl(bringup::BrakeControl::Request& request, bringup::BrakeControl::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "BrakeControl(%d,%d)", request.axisID, request.value);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::startDrag(bringup::StartDrag::Request& request, bringup::StartDrag::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "StartDrag()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::stopDrag(bringup::StopDrag::Request& request, bringup::StopDrag::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "StopDrag()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::loadSwitch(bringup::LoadSwitch::Request& request, bringup::LoadSwitch::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "LoadSwitch(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

// bool M1ProRobot::setHoldRegs(bringup::SetHoldRegs::Request& request, bringup::SetHoldRegs::Response&
// response)
// {
//     try
//     {
//         char cmd[200];
//         std::vector<std::string> result;
//         snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s,%s)", request.index, request.addr, request.count,
//                  request.regs.c_str(), request.type.c_str());
//         commander_->dashboardDoCmd(cmd, response.res, result);
//         if (result.empty())
//             throw std::logic_error("Haven't recv any result");

//         response.res = str2Int(result[0].c_str());
//         return true;
//     }
//     catch (const TcpClientException& err)
//     {
//         ROS_ERROR("%s", err.what());
//         response.res = -1;
//         return false;
//     }
//     catch (const std::exception& err)
//     {
//         ROS_ERROR("%s", err.what());
//         response.res = -1;
//         return false;
//     }
// }

bool M1ProRobot::continueScript(bringup::ContinueScript::Request& request, bringup::ContinueScript::Response& response)
{
    try
    {
        const char* cmd = "ContinueScript()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
bool M1ProRobot::setSafeSkin(bringup::SetSafeSkin::Request& request, bringup::SetSafeSkin::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetSafeSkin(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

bool M1ProRobot::setObstacleAvoid(bringup::SetObstacleAvoid::Request& request,
                                  bringup::SetObstacleAvoid::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetObstacleAvoid(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::setCollisionLevel(bringup::SetCollisionLevel::Request& request,
                                   bringup::SetCollisionLevel::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetCollisionLevel(%d)", request.level);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::GetAngle(bringup::GetAngle::Request& request, bringup::GetAngle::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "getAngle()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::GetPose(bringup::GetPose::Request& request, bringup::GetPose::Response& response)
{
    try
    {
        char cmd[100];
        if (request.user.size() == 0)
        {
            sprintf(cmd, "GetPose()");
        }
        else
        {
            sprintf(cmd, "GetPose(%d,%d)", request.user[0], request.tool[0]);
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::emergencyStop(bringup::EmergencyStop::Request& request, bringup::EmergencyStop::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "EmergencyStop()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::sync(bringup::Sync::Request& request, bringup::Sync::Response& response)
{
    try
    {
        const char* cmd = "Sync()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool M1ProRobot::movJ(bringup::MovJ::Request& request, bringup::MovJ::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovJ(%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.r);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::movL(bringup::MovL::Request& request, bringup::MovL::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovL(%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.r);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

// bool M1ProRobot::servoJ(bringup::ServoJ::Request& request, bringup::ServoJ::Response& response)
// {
//     try
//     {
//         char cmd[100];
//         sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f)", request.j1, request.j2, request.j3, request.j4);
//         commander_->motionDoCmd(cmd, response.res);
//         response.res = 0;
//         return true;
//     }
//     catch (const TcpClientException& err)
//     {
//         ROS_ERROR("%s", err.what());
//         response.res = -1;
//         return false;
//     }
// }

bool M1ProRobot::jump(bringup::Jump::Request& request, bringup::Jump::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Jump(%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::arc(bringup::Arc::Request& request, bringup::Arc::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.x1, request.y1, request.z1,
                request.r1, request.x2, request.y2, request.z2, request.r2);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::circle(bringup::Circle::Request& request, bringup::Circle::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "Circle(%d, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.count, request.x1,
                request.y1, request.z1, request.r1, request.x2, request.y2, request.z2, request.r2);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::RelMovJUser(bringup::RelMovJUser::Request& request, bringup::RelMovJUser::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "RelMovJUser(%0.3f,%0.3f,%0.3f,%0.3f,%d", request.x, request.y, request.z, request.r,
                request.user);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::RelMovLUser(bringup::RelMovLUser::Request& request, bringup::RelMovLUser::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "RelMovLUser(%0.3f,%0.3f,%0.3f,%0.3f,%d", request.x, request.y, request.z, request.r,
                request.user);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::RelJointMovJ(bringup::RelJointMovJ::Request& request, bringup::RelJointMovJ::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "RelJointMovJ(%0.3f,%0.3f,%0.3f,%0.3f", request.offset1, request.offset2, request.offset3,
                request.offset4);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::MovJExt(bringup::MovJExt::Request& request, bringup::MovJExt::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovJExt(%0.3f", request.offset);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

// bool M1ProRobot::servoP(bringup::ServoP::Request& request, bringup::ServoP::Response& response)
// {
//     try
//     {
//         char cmd[100];
//         sprintf(cmd, "ServoP(%0.3f,%0.3f,%0.3f,%0.3f)", request.x, request.y, request.z, request.r);
//         commander_->motionDoCmd(cmd, response.res);
//         return true;
//     }
//     catch (const TcpClientException& err)
//     {
//         ROS_ERROR("%s", err.what());
//         response.res = -1;
//         return false;
//     }
// }

bool M1ProRobot::relMovJ(bringup::RelMovJ::Request& request, bringup::RelMovJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelMovJ(%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::relMovL(bringup::RelMovL::Request& request, bringup::RelMovL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelMovL(%0.3f,%0.3f,%0.3f)", request.x, request.y, request.z);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::jointMovJ(bringup::JointMovJ::Request& request, bringup::JointMovJ::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "jointMovJ(%0.3f,%0.3f,%0.3f,%0.3f", request.j1, request.j2, request.j3, request.j4);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::movLIO(bringup::MovLIO::Request& request, bringup::MovLIO::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovLIO(%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.r);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::movJIO(bringup::MovJIO::Request& request, bringup::MovJIO::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovJIO(%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.r);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
bool M1ProRobot::startTrace(bringup::StartTrace::Request& request, bringup::StartTrace::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartTrace(%s)", request.trace_name.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

/*
bool M1ProRobot::startPath(bringup::StartPath::Request& request, bringup::StartPath::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartPath(%s,%d,%d)", request.trace_name.c_str(), request.const_val, request.cart);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

/*
bool M1ProRobot::startFCTrace(bringup::StartFCTrace::Request& request, bringup::StartFCTrace::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartFCTrace(%s)", request.trace_name.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

bool M1ProRobot::moveJog(bringup::MoveJog::Request& request, bringup::MoveJog::Response& response)
{
    try
    {
        char cmd[100];
        std::string str = "MoveJog(" + std::string(request.axisID);
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::stopmoveJog(bringup::StopmoveJog::Request& request, bringup::StopmoveJog::Response& response)
{
    try
    {
        char cmd[100] = "MoveJog()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::SyncAll(bringup::SyncAll::Request& request, bringup::SyncAll::Response& response)
{
    try
    {
        char cmd[100] = "SyncAll()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
bool M1ProRobot::Wait(bringup::Wait::Request& request, bringup::Wait::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Wait(%d)", request.time);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::Continue(bringup::Continues::Request& request, bringup::Continues::Response& response)
{
    try
    {
        char cmd[100] = "Continue()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::Pause(bringup::Pause::Request& request, bringup::Pause::Response& response)
{
    try
    {
        char cmd[100] = "Pause()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

int M1ProRobot::str2Int(const char* val)
{
    char* err;
    int mode = (int)strtol(val, &err, 10);
    if (*err != 0)
        throw std::logic_error(std::string("Invalid value : ") + val);
    return mode;
}
