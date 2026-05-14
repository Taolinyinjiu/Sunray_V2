/*
本程序功能：
    1、订阅 UGVControlState
    2、以终端状态面板形式显示 sunray_ugv_control 的核心输入、状态和输出
    3、让 ugv_control_fsm 本体只负责控制逻辑，不负责打印
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVControlState.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

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
    sunray_msgs::UGVControlState state{};
    ros::Time receive_time{0.0};
};

std::map<int, CachedState> g_states;
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

std::string driveTypeName(const uint8_t drive_type)
{
    switch (drive_type)
    {
    case sunray_msgs::UGVControlState::DRIVE_MECANUM:
        return "麦克纳姆";
    case sunray_msgs::UGVControlState::DRIVE_DIFFERENTIAL:
        return "差速";
    default:
        return "未知";
    }
}

std::string fsmName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UGVControlState::FSM_INIT:
        return "INIT";
    case sunray_msgs::UGVControlState::FSM_HOLD:
        return "HOLD";
    case sunray_msgs::UGVControlState::FSM_RETURN:
        return "RETURN";
    case sunray_msgs::UGVControlState::FSM_MOVE:
        return "MOVE";
    default:
        return "UNKNOWN";
    }
}

std::string cmdName(const uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::UGVControlCMD::HOLD:
        return "HOLD";
    case sunray_msgs::UGVControlCMD::RETURN:
        return "RETURN";
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
        return "MOVE_POINT";
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
        return "MOVE_VELOCITY";
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
        return "MOVE_VELOCITY_BODY";
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
        return "MOVE_WGS84";
    default:
        return "UNDEFINE";
    }
}

std::string sourceName(const uint8_t source)
{
    switch (source)
    {
    case sunray_msgs::UGVControlCMD::SUNRAY_STATION:
        return "SUNRAY_STATION";
    case sunray_msgs::UGVControlCMD::RC_CONTROLLER:
        return "RC_CONTROLLER";
    case sunray_msgs::UGVControlCMD::TERMINAL:
        return "TERMINAL";
    case sunray_msgs::UGVControlCMD::CONTROL_CMD:
        return "CONTROL_CMD";
    case sunray_msgs::UGVControlCMD::SWARM_CONTROL:
        return "SWARM_CONTROL";
    case sunray_msgs::UGVControlCMD::PLANNING:
        return "PLANNING";
    case sunray_msgs::UGVControlCMD::EXAMPLE_DEMO:
        return "EXAMPLE_DEMO";
    default:
        return "UNDEFINE";
    }
}

std::string coloredFsmName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UGVControlState::FSM_HOLD:
        return colorText("HOLD", kAnsiGood);
    case sunray_msgs::UGVControlState::FSM_RETURN:
    case sunray_msgs::UGVControlState::FSM_MOVE:
        return colorText(fsmName(state), kAnsiWarn);
    case sunray_msgs::UGVControlState::FSM_INIT:
    default:
        return colorText(fsmName(state), kAnsiValue);
    }
}

std::string coloredCmdName(const uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::UGVControlCMD::HOLD:
        return colorText("HOLD", kAnsiGood);
    case sunray_msgs::UGVControlCMD::RETURN:
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
        return colorText(cmdName(cmd), kAnsiWarn);
    default:
        return colorText(cmdName(cmd), kAnsiValue);
    }
}

std::string currentTargetText(const sunray_msgs::UGVControlState &state)
{
    if (!state.target_valid)
    {
        return "无明确目标点";
    }

    std::ostringstream ss;
    ss << "x = " << std::setw(6) << formatDouble(state.target_pos.x)
       << " m  y = " << std::setw(6) << formatDouble(state.target_pos.y)
       << " m  yaw = " << std::setw(6) << formatDouble(state.target_yaw * kRadToDeg) << " deg";
    return ss.str();
}

std::string rawControlInputText(const sunray_msgs::UGVControlCMD &cmd)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    switch (cmd.control_cmd)
    {
    case sunray_msgs::UGVControlCMD::HOLD:
        return "HOLD：无额外输入参数";
    case sunray_msgs::UGVControlCMD::RETURN:
        return "RETURN：使用内部返航点";
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
        ss << "desired_pos = (" << cmd.desired_pos.x << ", " << cmd.desired_pos.y << ", " << cmd.desired_pos.z
           << ") m  desired_yaw = " << cmd.desired_yaw * kRadToDeg << " deg";
        return ss.str();
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
        ss << "desired_vel = (" << cmd.desired_vel.x << ", " << cmd.desired_vel.y << ", " << cmd.desired_vel.z
           << ") m/s  desired_yaw = " << cmd.desired_yaw * kRadToDeg << " deg";
        return ss.str();
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
        ss << "cmd_vel.linear = (" << cmd.cmd_vel.linear.x << ", " << cmd.cmd_vel.linear.y << ", "
           << cmd.cmd_vel.linear.z << ") m/s  cmd_vel.angular.z = " << cmd.cmd_vel.angular.z * kRadToDeg
           << " deg/s";
        return ss.str();
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
        ss << "desired_wgs84_pos = (" << cmd.desired_wgs84_pos.latitude << ", "
           << cmd.desired_wgs84_pos.longitude << ", " << cmd.desired_wgs84_pos.altitude
           << ")  desired_yaw = " << cmd.desired_yaw * kRadToDeg << " deg";
        return ss.str();
    default:
        return "无有效原始输入";
    }
}

std::string odomTopicText(const sunray_msgs::UGVControlState &state)
{
    const std::string agent_prefix = "/" + state.agent_name + std::to_string(static_cast<int>(state.agent_id));
    const std::string odom_topic = agent_prefix + "/sunray/localization/local_odom";
    return colorText(odom_topic, kAnsiValue);
}

std::string cmdTopicText(const sunray_msgs::UGVControlState &state)
{
    const std::string agent_prefix = "/" + state.agent_name + std::to_string(static_cast<int>(state.agent_id));
    const std::string cmd_topic = agent_prefix + "/sunray/ugv_control/control_cmd";
    return colorText(cmd_topic, kAnsiValue);
}

std::string cmdVelTopicText(const sunray_msgs::UGVControlState &state)
{
    const std::string agent_prefix = "/" + state.agent_name + std::to_string(static_cast<int>(state.agent_id));
    const std::string cmd_vel_topic = agent_prefix + "/sunray/ugv_control/cmd_vel";
    return colorText(cmd_vel_topic, kAnsiValue);
}

std::string fsmStateTopicText(const sunray_msgs::UGVControlState &state)
{
    const std::string agent_prefix = "/" + state.agent_name + std::to_string(static_cast<int>(state.agent_id));
    const std::string state_topic = agent_prefix + "/sunray/ugv_control/control_state";
    return colorText(state_topic, kAnsiValue);
}

std::string buildPanel(const sunray_msgs::UGVControlState &state)
{
    const geometry_msgs::Point &pos = state.self_odom.pose.pose.position;
    const double yaw = yawFromOdom(state.self_odom);
    const std::string agent_label = "/" + state.agent_name + std::to_string(static_cast<int>(state.agent_id));

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << kAnsiTitle << "=================== UGV 控制状态面板 | " << agent_label
       << " ===================" << kAnsiReset << "\n";

    ss << kAnsiLabel << " 基本状态 " << kAnsiReset
       << "状态话题（发布） -> " << fsmStateTopicText(state) << "\n";

    ss << "          "
       << "Name = " << state.agent_name
       << "  ID = " << static_cast<int>(state.agent_id)
       << "  底盘 = " << colorText(driveTypeName(state.drive_type), kAnsiValue)
       << "  FSM = " << coloredFsmName(state.fsm_state) << "\n";

    ss << kAnsiLabel << " 本机位姿 " << kAnsiReset
       << "位姿话题（订阅） -> " << odomTopicText(state) << "\n";

    ss << "          "
       << "ODOM状态 = " << okText(state.odom_valid)
       << "  地理围栏 = " << okText(state.inside_geo_fence) << "\n";

    ss << "           "
       << "x = " << std::setw(5) << pos.x << " m"
       << "  y = " << std::setw(5) << pos.y << " m"
       << "  z = " << std::setw(5) << pos.z << " m"
       << "  yaw = " << std::setw(5) << yaw * kRadToDeg << " deg\n";

    ss << kAnsiLabel << " 控制输入 " << kAnsiReset
       << "指令话题（订阅） -> " << cmdTopicText(state) << "\n";

    ss << "          "
       << "指令状态 = " << okText(state.control_cmd_valid)
       << "  来源 = " << colorText(sourceName(state.active_ugv_control_cmd.cmd_source), kAnsiValue)
       << "  控制指令 = " << coloredCmdName(state.active_ugv_control_cmd.control_cmd) << "\n";

    ss << "           "
       << "原始输入 -> " << rawControlInputText(state.active_ugv_control_cmd) << "\n";

    ss << "           "
       << "控制目标 -> " << currentTargetText(state) << "\n";

    ss << kAnsiLabel << " 控制输出 " << kAnsiReset
       << "速度话题（发布） -> " << cmdVelTopicText(state) << "\n";

    ss << "           "
       << "vx = " << std::setw(6) << state.controller_cmd_vel.linear.x << " m/s"
       << "  vy = " << std::setw(6) << state.controller_cmd_vel.linear.y << " m/s"
       << "  wz = " << std::setw(6) << state.controller_cmd_vel.angular.z * kRadToDeg << " deg/s\n";

    return ss.str();
}

void stateCallback(const sunray_msgs::UGVControlState::ConstPtr &msg)
{
    if (msg->agent_id == 0)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    CachedState &cached = g_states[static_cast<int>(msg->agent_id)];
    cached.state = *msg;
    cached.receive_time = ros::Time::now();
}

void printPanel(const ros::TimerEvent &, const double stale_timeout)
{
    std::map<int, CachedState> states;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        states = g_states;
    }

    std::cout << "\033[2J\033[H";
    if (states.empty())
    {
        std::cout << colorText("UGV 控制状态面板", kAnsiTitle) << "\n";
        std::cout << "等待 UGVControlState...\n";
        return;
    }

    const ros::Time now = ros::Time::now();
    for (const auto &item : states)
    {
        if ((now - item.second.receive_time).toSec() > stale_timeout)
        {
            std::cout << colorText("ugv" + std::to_string(item.first) + " 状态超时", kAnsiBad) << "\n";
            continue;
        }
        std::cout << buildPanel(item.second.state) << "\n";
    }
    std::cout.flush();
}
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_control_monitor_node");
    ros::NodeHandle nh("~");

    std::string agent_name = "ugv";
    int agent_num = 1;
    double print_hz = 2.0;
    double stale_timeout = 1.0;
    nh.param("agent_name", agent_name, agent_name);
    nh.param("agent_num", agent_num, agent_num);
    nh.param("print_hz", print_hz, print_hz);
    nh.param("stale_timeout", stale_timeout, stale_timeout);

    std::vector<ros::Subscriber> state_subs;
    state_subs.reserve(static_cast<size_t>(std::max(1, agent_num)));
    for (int agent_id = 1; agent_id <= std::max(1, agent_num); ++agent_id)
    {
        const std::string state_topic =
            "/" + agent_name + std::to_string(agent_id) + "/sunray/ugv_control/control_state";
        state_subs.push_back(nh.subscribe(state_topic, 20, stateCallback));
        ROS_INFO("ugv_control_monitor_node subscribe: %s", state_topic.c_str());
    }

    ros::Timer print_timer = nh.createTimer(
        ros::Duration(1.0 / std::max(0.1, print_hz)),
        [stale_timeout](const ros::TimerEvent &event) { printPanel(event, stale_timeout); });

    ros::spin();
    return 0;
}
