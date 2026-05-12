/*
本程序功能：
    1、订阅单机 UAVControlState
    2、额外订阅单机 local_odom，用于在终端同步显示本机位姿与速度
    3、以终端状态面板形式显示 sunray_uav_control 的核心输入、状态和参数
    4、让 sunray_uav_control 本体只负责控制逻辑，不负责打印
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlState.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include "agent_key_helper.hpp"

namespace
{
constexpr double kRadToDeg = 57.29577951308232;
const char *kAnsiReset = "\033[0m";
const char *kAnsiTitle = "\033[1;36m";
const char *kAnsiLabel = "\033[1;32m";
const char *kAnsiWarn = "\033[1;33m";
const char *kAnsiBad = "\033[1;31m";
const char *kAnsiGood = "\033[1;32m";
const char *kAnsiValue = "\033[1;37m";

struct CachedState
{
    sunray_msgs::UAVControlState state{};
    nav_msgs::Odometry local_odom{};
    bool has_state{false};
    bool has_local_odom{false};
    ros::Time receive_time{0.0};
};

CachedState g_state;
std::mutex g_mutex;

std::string formatDouble(const double value, const int precision = 2)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

std::string colorText(const std::string &text, const char *color)
{
    return std::string(color) + text + kAnsiReset;
}

std::string okText(const bool ok)
{
    return ok ? colorText("正常", kAnsiGood) : colorText("异常", kAnsiBad);
}

double yawFromOdom(const nav_msgs::Odometry &odom)
{
    const auto &q = odom.pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string fsmName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UAVControlState::OFF:
        return "OFF";
    case sunray_msgs::UAVControlState::INIT:
        return "INIT";
    case sunray_msgs::UAVControlState::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlState::HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlState::RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlState::LAND:
        return "LAND";
    case sunray_msgs::UAVControlState::MOVE:
        return "MOVE";
    case sunray_msgs::UAVControlState::EMERGENCY_KILL:
        return "KILL";
    default:
        return "UNKNOWN";
    }
}

std::string cmdName(const uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::UAVControlCMD::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlCMD::LAND:
        return "LAND";
    case sunray_msgs::UAVControlCMD::RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlCMD::KILL:
        return "KILL";
    case sunray_msgs::UAVControlCMD::HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlCMD::MOVE_POINT:
        return "MOVE_POINT";
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY:
        return "MOVE_VELOCITY";
    case sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY:
        return "MOVE_TRAJECTORY";
    case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
        return "MOVE_POINT_BODY";
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY:
        return "MOVE_VELOCITY_BODY";
    case sunray_msgs::UAVControlCMD::MOVE_POINT_WGS84:
        return "MOVE_POINT_WGS84";
    default:
        return "UNDEFINE";
    }
}

std::string sourceName(const uint8_t source)
{
    switch (source)
    {
    case sunray_msgs::UAVControlCMD::SUNRAY_STATION:
        return "SUNRAY_STATION";
    case sunray_msgs::UAVControlCMD::RC_CONTROLLER:
        return "RC_CONTROLLER";
    case sunray_msgs::UAVControlCMD::TERMINAL:
        return "TERMINAL";
    case sunray_msgs::UAVControlCMD::SWARM_CONTROL:
        return "SWARM_CONTROL";
    case sunray_msgs::UAVControlCMD::PLANNING:
        return "PLANNING";
    case sunray_msgs::UAVControlCMD::EXAMPLE_DEMO:
        return "EXAMPLE_DEMO";
    default:
        return "UNDEFINE";
    }
}

std::string yawModeName(const uint8_t mode)
{
    switch (mode)
    {
    case sunray_msgs::UAVControlCMD::KEEP_YAW:
        return "KEEP_YAW";
    case sunray_msgs::UAVControlCMD::SET_YAW:
        return "SET_YAW";
    case sunray_msgs::UAVControlCMD::SET_YAWRATE:
        return "SET_YAWRATE";
    default:
        return "UNKNOWN";
    }
}

std::string landTypeName(const uint8_t land_type)
{
    switch (land_type)
    {
    case 0:
        return "CTRL_LAND";
    case 1:
        return "PX4_AUTOLAND";
    default:
        return "UNKNOWN";
    }
}

std::string coloredFsmName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UAVControlState::OFF:
    case sunray_msgs::UAVControlState::INIT:
        return colorText(fsmName(state), kAnsiValue);
    case sunray_msgs::UAVControlState::HOVER:
        return colorText("HOVER", kAnsiGood);
    case sunray_msgs::UAVControlState::TAKEOFF:
    case sunray_msgs::UAVControlState::RETURN:
    case sunray_msgs::UAVControlState::LAND:
    case sunray_msgs::UAVControlState::MOVE:
        return colorText(fsmName(state), kAnsiWarn);
    case sunray_msgs::UAVControlState::EMERGENCY_KILL:
        return colorText("KILL", kAnsiBad);
    default:
        return colorText(fsmName(state), kAnsiBad);
    }
}

std::string coloredCmdName(const uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::UAVControlCMD::HOVER:
        return colorText("HOVER", kAnsiGood);
    case sunray_msgs::UAVControlCMD::TAKEOFF:
    case sunray_msgs::UAVControlCMD::LAND:
    case sunray_msgs::UAVControlCMD::RETURN:
    case sunray_msgs::UAVControlCMD::MOVE_POINT:
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY:
    case sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY:
    case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY:
    case sunray_msgs::UAVControlCMD::MOVE_POINT_WGS84:
        return colorText(cmdName(cmd), kAnsiWarn);
    case sunray_msgs::UAVControlCMD::KILL:
        return colorText("KILL", kAnsiBad);
    default:
        return colorText(cmdName(cmd), kAnsiValue);
    }
}

std::string stateTopicText(const std::string &agent_key)
{
    return colorText(agent_key + "/sunray/uav_control/control_state", kAnsiValue);
}

std::string odomTopicText(const std::string &agent_key)
{
    return colorText(agent_key + "/sunray/localization/local_odom", kAnsiValue);
}

std::string cmdTopicText(const std::string &agent_key)
{
    return colorText(agent_key + "/sunray/uav_control/control_cmd", kAnsiValue);
}

std::string landConfigText(const sunray_msgs::UAVControlState &state)
{
    std::ostringstream ss;
    ss << "type = " << landTypeName(state.land_type)
       << "  vmax = " << formatDouble(state.land_max_velocity) << " m/s";
    return ss.str();
}

std::string homePointText(const sunray_msgs::UAVControlState &state)
{
    std::ostringstream ss;
    ss << "x = " << std::setw(6) << formatDouble(state.home_point.x)
       << " m  y = " << std::setw(6) << formatDouble(state.home_point.y)
       << " m  z = " << std::setw(6) << formatDouble(state.home_point.z) << " m";
    return ss.str();
}

std::string odomPoseText(const nav_msgs::Odometry &odom)
{
    const auto &p = odom.pose.pose.position;
    const double yaw = yawFromOdom(odom);
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "x = " << std::setw(6) << p.x << " m"
       << "  y = " << std::setw(6) << p.y << " m"
       << "  z = " << std::setw(6) << p.z << " m"
       << "  yaw = " << std::setw(6) << yaw * kRadToDeg << " deg";
    return ss.str();
}

std::string odomVelText(const nav_msgs::Odometry &odom)
{
    const auto &v = odom.twist.twist.linear;
    const auto &w = odom.twist.twist.angular;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "vx = " << std::setw(6) << v.x << " m/s"
       << "  vy = " << std::setw(6) << v.y << " m/s"
       << "  vz = " << std::setw(6) << v.z << " m/s"
       << "  wz = " << std::setw(6) << w.z * kRadToDeg << " deg/s";
    return ss.str();
}

std::string rawControlInputText(const sunray_msgs::UAVControlCMD &cmd)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    switch (cmd.control_cmd)
    {
    case sunray_msgs::UAVControlCMD::TAKEOFF:
    case sunray_msgs::UAVControlCMD::LAND:
    case sunray_msgs::UAVControlCMD::RETURN:
    case sunray_msgs::UAVControlCMD::KILL:
    case sunray_msgs::UAVControlCMD::HOVER:
        return "无额外输入参数";
    case sunray_msgs::UAVControlCMD::MOVE_POINT:
        ss << "desired_pos = (" << cmd.desired_pos.x << ", " << cmd.desired_pos.y << ", "
           << cmd.desired_pos.z << ") m  yaw = " << cmd.desired_yaw * kRadToDeg << " deg";
        return ss.str();
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY:
        ss << "desired_vel = (" << cmd.desired_vel.x << ", " << cmd.desired_vel.y << ", "
           << cmd.desired_vel.z << ") m/s";
        if (cmd.fixed_height > 0.0)
        {
            ss << "  fixed_height = " << cmd.fixed_height << " m";
        }
        ss << "  yaw_mode = " << yawModeName(cmd.yaw_mode);
        return ss.str();
    case sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY:
        ss << "pos = (" << cmd.desired_pos.x << ", " << cmd.desired_pos.y << ", "
           << cmd.desired_pos.z << ")"
           << "  vel = (" << cmd.desired_vel.x << ", " << cmd.desired_vel.y << ", "
           << cmd.desired_vel.z << ")"
           << "  acc = (" << cmd.desired_acc.x << ", " << cmd.desired_acc.y << ", "
           << cmd.desired_acc.z << ")";
        return ss.str();
    case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
        ss << "body_xy = (" << cmd.desired_body_xy_pos.x << ", " << cmd.desired_body_xy_pos.y
           << ") m  fixed_height = " << cmd.fixed_height << " m";
        return ss.str();
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY:
        ss << "body_vel_xy = (" << cmd.desired_body_xy_vel.x << ", " << cmd.desired_body_xy_vel.y
           << ") m/s  fixed_height = " << cmd.fixed_height << " m";
        if (cmd.yaw_mode == sunray_msgs::UAVControlCMD::SET_YAW)
        {
            ss << "  yaw = " << cmd.desired_yaw * kRadToDeg << " deg";
        }
        else if (cmd.yaw_mode == sunray_msgs::UAVControlCMD::SET_YAWRATE)
        {
            ss << "  yaw_rate = " << cmd.desired_yaw_rate * kRadToDeg << " deg/s";
        }
        return ss.str();
    case sunray_msgs::UAVControlCMD::MOVE_POINT_WGS84:
        ss << "wgs84 = (" << cmd.desired_wgs84_pos.latitude << ", "
           << cmd.desired_wgs84_pos.longitude << ", " << cmd.desired_wgs84_pos.altitude
           << ")  yaw = " << cmd.desired_yaw * kRadToDeg << " deg";
        return ss.str();
    default:
        return "无有效原始输入";
    }
}

std::string buildPanel(const std::string &agent_key, const CachedState &cached)
{
    const sunray_msgs::UAVControlState &state = cached.state;
    const sunray_msgs::UAVControlCMD &cmd = state.last_cmd;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << kAnsiTitle << "=================== UAV 控制状态面板 | " << agent_key
       << " ===================" << kAnsiReset << "\n";

    ss << kAnsiLabel << " 基本状态 " << kAnsiReset
       << "状态话题（发布） -> " << stateTopicText(agent_key) << "\n";
    ss << "          "
       << "FSM = " << coloredFsmName(state.control_state)
       << "  里程计有效 = " << okText(state.odometry_valid)
       << "  里程计超时 = " << okText(!state.odometry_lost) << "\n";

    ss << kAnsiLabel << " 本机位姿 " << kAnsiReset
       << "位姿话题（订阅） -> " << odomTopicText(agent_key) << "\n";
    if (cached.has_local_odom)
    {
        ss << "          " << odomPoseText(cached.local_odom) << "\n";
        ss << "          " << odomVelText(cached.local_odom) << "\n";
    }
    else
    {
        ss << "          " << colorText("等待 local_odom...", kAnsiWarn) << "\n";
    }

    ss << kAnsiLabel << " 控制输入 " << kAnsiReset
       << "指令话题（订阅） -> " << cmdTopicText(agent_key) << "\n";
    ss << "          "
       << "来源 = " << colorText(sourceName(cmd.cmd_source), kAnsiValue)
       << "  控制指令 = " << coloredCmdName(cmd.control_cmd)
       << "  Yaw模式 = " << colorText(yawModeName(cmd.yaw_mode), kAnsiValue) << "\n";
    ss << "          " << "原始输入 -> " << rawControlInputText(cmd) << "\n";

    ss << kAnsiLabel << " 起降参数 " << kAnsiReset << "\n";
    ss << "          "
       << "takeoff_h = " << formatDouble(state.takeoff_relative_height)
       << " m  takeoff_vmax = " << formatDouble(state.takeoff_max_velocity) << " m/s\n";
    ss << "          "
       << "land -> " << landConfigText(state) << "\n";
    ss << "          "
       << "home -> " << homePointText(state) << "\n";

    return ss.str();
}

void stateCallback(const sunray_msgs::UAVControlState::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_state.state = *msg;
    g_state.has_state = true;
    g_state.receive_time = ros::Time::now();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_state.local_odom = *msg;
    g_state.has_local_odom = true;
}

void printPanel(const ros::TimerEvent &, const std::string &agent_key, const double stale_timeout)
{
    CachedState cached;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        cached = g_state;
    }

    std::cout << "\033[2J\033[H";
    if (!cached.has_state)
    {
        std::cout << colorText("UAV 控制状态面板", kAnsiTitle) << "\n";
        std::cout << "等待 UAVControlState...\n";
        std::cout.flush();
        return;
    }

    const ros::Time now = ros::Time::now();
    if ((now - cached.receive_time).toSec() > stale_timeout)
    {
        std::cout << colorText(agent_key + " 状态超时", kAnsiBad) << "\n";
        std::cout.flush();
        return;
    }

    std::cout << buildPanel(agent_key, cached) << "\n";
    std::cout.flush();
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control_monitor_node");
    ros::NodeHandle nh("~");

    bool use_private_agent_key = false;
    double print_hz = 2.0;
    double stale_timeout = 1.0;
    nh.param("use_private_agent_key", use_private_agent_key, false);
    nh.param("print_hz", print_hz, print_hz);
    nh.param("stale_timeout", stale_timeout, stale_timeout);

    const std::string agent_key = use_private_agent_key
                                      ? sunray_common::get_agent_key_from_private()
                                      : sunray_common::get_agent_key_from_global();
    const std::string state_topic = agent_key + "/sunray/uav_control/control_state";
    const std::string odom_topic = agent_key + "/sunray/localization/local_odom";

    ros::Subscriber state_sub = nh.subscribe<sunray_msgs::UAVControlState>(
        state_topic, 20, stateCallback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        odom_topic, 20, odomCallback);

    ROS_INFO("uav_control_monitor_node subscribe state: %s", state_topic.c_str());
    ROS_INFO("uav_control_monitor_node subscribe odom : %s", odom_topic.c_str());

    ros::Timer print_timer = nh.createTimer(
        ros::Duration(1.0 / std::max(0.1, print_hz)),
        [agent_key, stale_timeout](const ros::TimerEvent &event) {
            printPanel(event, agent_key, stale_timeout);
        });

    ros::spin();
    return 0;
}
