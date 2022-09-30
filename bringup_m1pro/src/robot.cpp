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

    server_tbl_.push_back(control_nh_.advertiseService("srv/EnableRobot", &M1ProRobot::enableRobot, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("srv/DisableRobot", &M1ProRobot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/ClearError", &M1ProRobot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/ResetRobot", &M1ProRobot::resetRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/SpeedFactor", &M1ProRobot::speedFactor, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/User", &M1ProRobot::user, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/Tool", &M1ProRobot::tool, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/RobotMode", &M1ProRobot::robotMode, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/PayLoad", &M1ProRobot::payload, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/DO", &M1ProRobot::DO, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/DOExecute", &M1ProRobot::DOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/ToolDO", &M1ProRobot::toolDO, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("srv/ToolDOExecute", &M1ProRobot::toolDOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/AO", &M1ProRobot::AO, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/AOExecute", &M1ProRobot::AOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/AccJ", &M1ProRobot::accJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/AccL", &M1ProRobot::accL, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/SpeedJ", &M1ProRobot::speedJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/SpeedL", &M1ProRobot::speedL, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/Arch", &M1ProRobot::arch, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/CP", &M1ProRobot::cp, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/LimZ", &M1ProRobot::limZ, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("srv/SetArmOrientation", &M1ProRobot::setArmOrientation, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/PowerOn", &M1ProRobot::powerOn, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/RunScript", &M1ProRobot::runScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/StopScript", &M1ProRobot::stopScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/PauseScript", &M1ProRobot::pauseScript, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("srv/ContinueScript", &M1ProRobot::continueScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/SetSafeSkin", &M1ProRobot::setSafeSkin, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("srv/SetObstacleAvoid", &M1ProRobot::setObstacleAvoid, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("srv/SetCollisionLevel", &M1ProRobot::setCollisionLevel, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("srv/EmergencyStop", &M1ProRobot::emergencyStop, this));

    server_tbl_.push_back(control_nh_.advertiseService("srv/MovJ", &M1ProRobot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/MovL", &M1ProRobot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/JointMovJ", &M1ProRobot::jointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/Jump", &M1ProRobot::jump, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/RelMovJ", &M1ProRobot::relMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/RelMovL", &M1ProRobot::relMovL, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/Arc", &M1ProRobot::arc, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/Circle", &M1ProRobot::circle, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/ServoJ", &M1ProRobot::servoJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/ServoP", &M1ProRobot::servoP, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/Sync", &M1ProRobot::sync, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/StartTrace", &M1ProRobot::startTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/StartPath", &M1ProRobot::startPath, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("srv/StartFCTrace", &M1ProRobot::startFCTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/MoveJog", &M1ProRobot::moveJog, this));

    // server_tbl_.push_back(
    //     control_nh_.advertiseService("srv/ModbusCreate", &M1ProRobot::modbusCreate, this));
    // server_tbl_.push_back(control_nh_.advertiseService("srv/SetHoldRegs", &M1ProRobot::setHoldRegs, this));

    registerGoalCallback(boost::bind(&M1ProRobot::goalHandle, this, _1));
    registerCancelCallback(boost::bind(&M1ProRobot::cancelHandle, this, _1));
    start();
}

void M1ProRobot::feedbackHandle(const ros::TimerEvent& tm,
                              ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryFeedback feedback;

    double current_joints[6];
    getJointState(current_joints);

    for (uint32_t i = 0; i < 6; i++)
    {
        feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback.actual.positions.push_back(current_joints[i]);
        feedback.desired.positions.push_back(goal_[i]);
    }

    handle.publishFeedback(feedback);
}

void M1ProRobot::moveHandle(const ros::TimerEvent& tm,
                          ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

    if (index_ < trajectory->trajectory.points.size())
    {
        auto point = trajectory->trajectory.points[index_].positions;
        double tmp[6];
        for (uint32_t i = 0; i < 4; i++)
        {
            tmp[i] = point[i] * 180.0 / 3.1415926;
        }

        bringup::ServoJ srv;
        srv.request.j1 = tmp[0];
        srv.request.j2 = tmp[1];
        srv.request.j3 = tmp[2];
        srv.request.j4 = tmp[3];
        servoJ(srv.request, srv.response);
        index_++;
    }
    else
    {
#define OFFSET_VAL 0.01
        double current_joints[6];
        getJointState(current_joints);
        if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
            (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
            (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
            (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL))
        {
            timer_.stop();
            movj_timer_.stop();
            handle.setSucceeded();
        }
    }
}

void M1ProRobot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++)
    {
        goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&M1ProRobot::feedbackHandle, this, _1, handle));
    movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),
                                          boost::bind(&M1ProRobot::moveHandle, this, _1, handle));
    timer_.start();
    movj_timer_.start();
    handle.setAccepted();
}

void M1ProRobot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}

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
        const char* cmd = "EnableRobot()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const std::exception& err)
    {
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::disableRobot(bringup::DisableRobot::Request& request,
                            bringup::DisableRobot::Response& response)
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

bool M1ProRobot::toolDOExecute(bringup::ToolDOExecute::Request& request,
                             bringup::ToolDOExecute::Response& response)
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

bool M1ProRobot::AOExecute(bringup::AOExecute::Request& request, bringup::AOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %0.3f)", request.index, static_cast<float>(request.value));
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
        sprintf(cmd, "Arch(%d)", request.index);
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
        sprintf(cmd, "SetArmOrientation(%d,%d,%d,%d)", request.LorR, request.UorD, request.ForN, request.Config6);
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

bool M1ProRobot::pauseScript(bringup::PauseScript::Request& request, bringup::PauseScript::Response& response)
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

bool M1ProRobot::continueScript(bringup::ContinueScript::Request& request,
                              bringup::ContinueScript::Response& response)
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

bool M1ProRobot::emergencyStop(bringup::EmergencyStop::Request& request,
                             bringup::EmergencyStop::Response& response)
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
        char result[50];
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
        char cmd[100];
        sprintf(cmd, "MovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.x, request.y, request.z, request.a, request.b,
                request.c);
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
        char cmd[100];
        sprintf(cmd, "MovL(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.x, request.y, request.z, request.a, request.b,
                request.c);
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

bool M1ProRobot::servoJ(bringup::ServoJ::Request& request, bringup::ServoJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f)", request.j1, request.j2, request.j3, request.j4);
        commander_->motionDoCmd(cmd, response.res);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool M1ProRobot::jump(bringup::Jump::Request& request, bringup::Jump::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Jump(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4, request.offset5, request.offset6);
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
        char cmd[100];
        sprintf(cmd, "Arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.x1,
                request.y1, request.z1, request.rx1, request.ry1, request.rz1, request.x2, request.y2, request.z2,
                request.rx2, request.ry2, request.rz2);
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
        char cmd[100];
        sprintf(cmd, "Circle(%d, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
                request.count, request.x1, request.y1, request.z1, request.rx1, request.ry1, request.rz1, request.x2,
                request.y2, request.z2, request.rx2, request.ry2, request.rz2);
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

bool M1ProRobot::servoP(bringup::ServoP::Request& request, bringup::ServoP::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ServoP(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.x, request.y, request.z, request.a,
                request.b, request.c);
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

bool M1ProRobot::relMovJ(bringup::RelMovJ::Request& request, bringup::RelMovJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4, request.offset5, request.offset6);
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
        char cmd[100];
        sprintf(cmd, "JointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.j1, request.j2, request.j3, request.j4,
                request.j5, request.j6);
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

bool M1ProRobot::startFCTrace(bringup::StartFCTrace::Request& request,
                            bringup::StartFCTrace::Response& response)
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

bool M1ProRobot::moveJog(bringup::MoveJog::Request& request, bringup::MoveJog::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "MoveJog(%s)", request.axisID.c_str());
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
