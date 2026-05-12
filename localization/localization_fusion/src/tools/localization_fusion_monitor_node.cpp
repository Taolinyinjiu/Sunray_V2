/*
本程序功能：
    1. 订阅 {agent_key}/sunray/localization/odom_state 话题
    2. 订阅 {agent_key}/sunray/localization/local_odom 话题
    3. 以终端状态面板形式显示 localization_fusion 的核心状态与位姿输出
    4. 让 localization_fusion 本体只负责融合逻辑，不负责打印
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/OdomState.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "agent_key_helper.hpp"

namespace
{

const char *kAnsiReset = "\033[0m";
const char *kAnsiTitle = "\033[1;36m";
const char *kAnsiLabel = "\033[1;32m";
const char *kAnsiWarn  = "\033[1;33m";
const char *kAnsiBad   = "\033[1;31m";
const char *kAnsiGood  = "\033[1;32m";
const char *kAnsiValue = "\033[1;37m";

constexpr double kRadToDeg = 57.29577951308232;

struct CachedState
{
    sunray_msgs::OdomState state{};
    nav_msgs::Odometry     local_odom{};
    bool                   has_local{false};
    ros::Time              receive_time{0.0};
};

std::map<int, CachedState> g_states;
std::mutex g_mutex;

std::string colorText(const std::string &text, const char *color)
{
    return std::string(color) + text + kAnsiReset;
}

std::string okText(const bool ok)
{
    return ok ? colorText("正常", kAnsiGood) : colorText("异常", kAnsiBad);
}

std::string sourceName(const uint8_t source)
{
    switch (source)
    {
    case sunray_msgs::OdomState::VIOBOT:       return "VIOBOT";
    case sunray_msgs::OdomState::MOCAP:        return "MOCAP";
    case sunray_msgs::OdomState::VINS:         return "VINS";
    case sunray_msgs::OdomState::GAZEBO:       return "GAZEBO";
    case sunray_msgs::OdomState::GAZEBO_ARUCO: return "GAZEBO_ARUCO";
    case sunray_msgs::OdomState::PENGYU_SIM:   return "PENGYU_SIM";
    case sunray_msgs::OdomState::FASTLIO_EFK:  return "FASTLIO_EKF";
    default:                                   return "UNKNOWN";
    }
}

std::string modeName(const uint8_t mode)
{
    switch (mode)
    {
    case sunray_msgs::OdomState::LOCAL:            return "LOCAL";
    case sunray_msgs::OdomState::GLOBAL:           return "GLOBAL";
    case sunray_msgs::OdomState::LOCAL_AND_GLOBAL: return "LOCAL_AND_GLOBAL";
    case sunray_msgs::OdomState::LOCAL_WITH_ARUCO: return "LOCAL_WITH_ARUCO";
    default:                                       return "UNKNOWN";
    }
}

std::string coloredModeName(const uint8_t mode)
{
    switch (mode)
    {
    case sunray_msgs::OdomState::LOCAL:
    case sunray_msgs::OdomState::GLOBAL:
        return colorText(modeName(mode), kAnsiGood);
    case sunray_msgs::OdomState::LOCAL_AND_GLOBAL:
    case sunray_msgs::OdomState::LOCAL_WITH_ARUCO:
        return colorText(modeName(mode), kAnsiWarn);
    default:
        return colorText(modeName(mode), kAnsiBad);
    }
}

std::string stampText(const ros::Time &stamp)
{
    if (stamp.isZero())
        return colorText("未收到", kAnsiBad);
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << stamp.toSec() << " s";
    return colorText(ss.str(), kAnsiValue);
}

double yawFromOdom(const nav_msgs::Odometry &odom)
{
    const auto &q = odom.pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string odomPoseText(const nav_msgs::Odometry &odom)
{
    const auto &p = odom.pose.pose.position;
    const double yaw = yawFromOdom(odom);
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "x = " << std::setw(7) << p.x << " m"
       << "  y = " << std::setw(7) << p.y << " m"
       << "  z = " << std::setw(7) << p.z << " m"
       << "  yaw = " << std::setw(7) << yaw * kRadToDeg << " deg";
    return colorText(ss.str(), kAnsiValue);
}

std::string odomVelText(const nav_msgs::Odometry &odom)
{
    const auto &v = odom.twist.twist.linear;
    const double wz = odom.twist.twist.angular.z;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "vx = " << std::setw(6) << v.x << " m/s"
       << "  vy = " << std::setw(6) << v.y << " m/s"
       << "  vz = " << std::setw(6) << v.z << " m/s"
       << "  wz = " << std::setw(6) << wz * kRadToDeg << " deg/s";
    return colorText(ss.str(), kAnsiValue);
}

std::string buildPanel(const std::string &agent_key, const CachedState &cached)
{
    const sunray_msgs::OdomState &state = cached.state;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss << colorText("========== Localization Fusion 状态面板 | " + agent_key + " ==========", kAnsiTitle) << "\n";

    ss << colorText(" 定位源    ", kAnsiLabel)
       << "来源 = " << colorText(sourceName(state.external_source), kAnsiValue)
       << "  模式 = " << coloredModeName(state.localization_mode) << "\n";

    ss << colorText(" 里程计    ", kAnsiLabel)
       << "状态 = " << okText(state.odometry_valid)
       << "  更新频率 = " << colorText(std::to_string(static_cast<int>(state.odometry_update_hz + 0.5f)) + " Hz", kAnsiValue)
       << "  最新时间戳 = " << stampText(state.odometry_received_stamp) << "\n";

    if (state.localization_mode == sunray_msgs::OdomState::LOCAL_AND_GLOBAL ||
        state.localization_mode == sunray_msgs::OdomState::LOCAL_WITH_ARUCO)
    {
        ss << colorText(" 重定位    ", kAnsiLabel)
           << "状态 = " << okText(state.relocalization_valid)
           << "  最新时间戳 = " << stampText(state.relocalization_received_stamp) << "\n";
    }

    ss << colorText(" local_odom", kAnsiLabel) << " ";
    if (cached.has_local)
    {
        ss << "frame = " << colorText(cached.local_odom.header.frame_id, kAnsiValue) << "\n";
        ss << "           位置: " << odomPoseText(cached.local_odom) << "\n";
        ss << "           速度: " << odomVelText(cached.local_odom) << "\n";
    }
    else
    {
        ss << colorText("等待数据...", kAnsiWarn) << "\n";
    }

    return ss.str();
}

void stateCallback(const sunray_msgs::OdomState::ConstPtr &msg, const int agent_id)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    CachedState &cached = g_states[agent_id];
    cached.state = *msg;
    cached.receive_time = ros::Time::now();
}

void localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, const int agent_id)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    CachedState &cached = g_states[agent_id];
    cached.local_odom = *msg;
    cached.has_local = true;
}

void printPanel(const ros::TimerEvent &,
                const std::vector<std::string> &agent_keys,
                const double stale_timeout)
{
    std::map<int, CachedState> states;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        states = g_states;
    }

    std::cout << "\033[2J\033[H";
    if (states.empty())
    {
        std::cout << colorText("Localization Fusion 状态面板", kAnsiTitle) << "\n";
        std::cout << "等待 OdomState 消息...\n";
        std::cout.flush();
        return;
    }

    const ros::Time now = ros::Time::now();
    for (const auto &item : states)
    {
        const int id = item.first;
        const std::string &key = (id >= 1 && id <= static_cast<int>(agent_keys.size()))
                                     ? agent_keys[static_cast<size_t>(id - 1)]
                                     : "agent" + std::to_string(id);
        if ((now - item.second.receive_time).toSec() > stale_timeout)
        {
            std::cout << colorText(key + " 状态超时", kAnsiBad) << "\n";
            continue;
        }
        std::cout << buildPanel(key, item.second) << "\n";
    }
    std::cout.flush();
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_fusion_monitor_node");
    ros::NodeHandle nh("~");

    std::string agent_name = "uav";
    int agent_num = 1;
    double print_hz = 2.0;
    double stale_timeout = 1.0;
    nh.param("agent_name", agent_name, agent_name);
    nh.param("agent_num", agent_num, agent_num);
    nh.param("print_hz", print_hz, print_hz);
    nh.param("stale_timeout", stale_timeout, stale_timeout);

    const int num = std::max(1, agent_num);
    std::vector<std::string> agent_keys;
    std::vector<ros::Subscriber> subs;
    agent_keys.reserve(static_cast<size_t>(num));
    subs.reserve(static_cast<size_t>(num * 3));

    for (int id = 1; id <= num; ++id)
    {
        const std::string key = sunray_common::normalize_agent_key(agent_name + std::to_string(id));
        agent_keys.push_back(key);

        subs.push_back(nh.subscribe<sunray_msgs::OdomState>(
            key + "/sunray/localization/odom_state", 10,
            [id](const sunray_msgs::OdomState::ConstPtr &msg) { stateCallback(msg, id); }));

        subs.push_back(nh.subscribe<nav_msgs::Odometry>(
            key + "/sunray/localization/local_odom", 10,
            [id](const nav_msgs::Odometry::ConstPtr &msg) { localOdomCallback(msg, id); }));

        ROS_INFO("localization_fusion_monitor subscribe: %s", (key + "/sunray/localization/[odom_state|local_odom]").c_str());
    }

    ros::Timer print_timer = nh.createTimer(
        ros::Duration(1.0 / std::max(0.1, print_hz)),
        [&agent_keys, stale_timeout](const ros::TimerEvent &e) {
            printPanel(e, agent_keys, stale_timeout);
        });

    ros::spin();
    return 0;
}
