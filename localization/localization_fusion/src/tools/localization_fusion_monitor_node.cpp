/*
本程序功能：
    1. 读取 agent_name 和 agent_num，自动生成 /{agent_name}{id}
    2. 订阅每个智能体的 sunray/localization/odom_state 话题
    3. 以终端状态面板形式显示 localization_fusion 的核心状态与位姿输出
    4. 让 localization_fusion 本体只负责融合逻辑，不负责打印
*/
#include <ros/ros.h>
#include <ros/package.h>
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
#include "localization_fusion_utils.hpp"

namespace {

const char *kAnsiReset = "\033[0m";
const char *kAnsiTitle = "\033[1;36m";
const char *kAnsiLabel = "\033[1;32m";
const char *kAnsiWarn  = "\033[1;33m";
const char *kAnsiBad   = "\033[1;31m";
const char *kAnsiGood  = "\033[1;32m";
const char *kAnsiValue = "\033[1;37m";

constexpr double kRadToDeg = 57.29577951308232;

struct EulerAngle
{
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

struct CachedState
{
    sunray_msgs::OdomState state{};
    bool                   has_state{false};
    ros::Time              receive_time{0.0};
};

struct AgentTopics
{
    std::string external_odom_topic;
    std::string relocalization_topic;
    std::string local_odom_topic;
    std::string global_odom_topic;
    std::string odom_state_topic;
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

std::string relocalizationTopicText(const sunray_msgs::OdomState &state)
{
    if (state.subtopic_name_external_relocalization.empty())
    {
        return colorText("未配置", kAnsiWarn);
    }
    return colorText(state.subtopic_name_external_relocalization, kAnsiValue);
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

std::string topicValueText(const std::string &topic)
{
    return colorText(topic.empty() ? "-" : topic, kAnsiValue);
}

std::string localOdomTopicFromAgentKey(const std::string &agent_key)
{
    return agent_key + "/sunray/localization/local_odom";
}

std::string globalOdomTopicFromAgentKey(const std::string &agent_key)
{
    return agent_key + "/sunray/localization/global_odom";
}

std::string odomStateTopicFromAgentKey(const std::string &agent_key)
{
    return agent_key + "/sunray/localization/odom_state";
}

AgentTopics buildAgentTopics(const std::string &agent_key,
                             const SourceConfig &source_config)
{
    AgentTopics topics;
    topics.external_odom_topic =
        sunray_common::replace_agent_key(source_config.odometry_topic, agent_key);
    topics.relocalization_topic =
        sunray_common::replace_agent_key(source_config.relocalization_topic, agent_key);
    topics.local_odom_topic = localOdomTopicFromAgentKey(agent_key);
    topics.global_odom_topic = globalOdomTopicFromAgentKey(agent_key);
    topics.odom_state_topic = odomStateTopicFromAgentKey(agent_key);
    return topics;
}

EulerAngle eulerFromOdom(const nav_msgs::Odometry &odom)
{
    const auto &q = odom.pose.pose.orientation;
    EulerAngle euler;

    euler.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                            1.0 - 2.0 * (q.x * q.x + q.y * q.y));

    const double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sin_pitch) >= 1.0)
    {
        euler.pitch = std::copysign(M_PI / 2.0, sin_pitch);
    }
    else
    {
        euler.pitch = std::asin(sin_pitch);
    }

    euler.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return euler;
}

std::string odomFrameText(const nav_msgs::Odometry &odom)
{
    std::ostringstream ss;
    ss << "frame_id = " << odom.header.frame_id
       << "  child_frame_id = " << (odom.child_frame_id.empty() ? "-" : odom.child_frame_id);
    return colorText(ss.str(), kAnsiValue);
}

std::string odomPositionText(const nav_msgs::Odometry &odom)
{
    const auto &p = odom.pose.pose.position;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "x = " << std::setw(7) << p.x << " m"
       << "  y = " << std::setw(7) << p.y << " m"
       << "  z = " << std::setw(7) << p.z << " m";
    return colorText(ss.str(), kAnsiValue);
}

std::string odomVelText(const nav_msgs::Odometry &odom)
{
    const auto &v = odom.twist.twist.linear;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "vx = " << std::setw(6) << v.x << " m/s"
       << "  vy = " << std::setw(6) << v.y << " m/s"
       << "  vz = " << std::setw(6) << v.z << " m/s";
    return colorText(ss.str(), kAnsiValue);
}

std::string odomAttitudeText(const nav_msgs::Odometry &odom)
{
    const EulerAngle euler = eulerFromOdom(odom);
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "roll = " << std::setw(7) << euler.roll * kRadToDeg << " deg"
       << "  pitch = " << std::setw(7) << euler.pitch * kRadToDeg << " deg"
       << "  yaw = " << std::setw(7) << euler.yaw * kRadToDeg << " deg";
    return colorText(ss.str(), kAnsiValue);
}

std::string buildPanel(const std::string &agent_key,
                       const AgentTopics &topics,
                       const CachedState &cached)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss << colorText("============== Localization Fusion 状态面板 | " + agent_key + " ==============",
                    kAnsiTitle)
       << "\n";

    if (!cached.has_state)
    {
        ss << colorText(" 订阅话题  ", kAnsiLabel)
           << "odom状态话题: " << topicValueText(topics.odom_state_topic) << "\n";
        ss << colorText(" 基本信息  ", kAnsiLabel) << colorText("等待 OdomState...", kAnsiWarn)
           << "\n";
        return ss.str();
    }

    const sunray_msgs::OdomState &state = cached.state;

    ss << colorText(" 订阅话题  ", kAnsiLabel)
       << "外部定位源（局部）话题: " << topicValueText(state.subtopic_name_external_odom) << "\n";
    ss << "           "
       << "外部定位源（重定位）话题: " << relocalizationTopicText(state) << "\n";
    ss << colorText(" 发布话题  ", kAnsiLabel)
       << "局部odom话题: " << topicValueText(state.pubtopic_name_local_odom) << "\n";
    ss << "           "
       << "全局odom话题: " << topicValueText(state.pubtopic_name_global_odom) << "\n";

    ss << colorText(" 基本信息  ", kAnsiLabel)
       << "外部定位源 = " << colorText(sourceName(state.external_source), kAnsiValue)
       << "  odom状态 = " << okText(state.odometry_valid)
       << "  频率 = "
       << colorText(std::to_string(static_cast<int>(state.odometry_update_hz + 0.5f)) + " Hz",
                    kAnsiValue)
       << "\n";

    ss << "           "
       << "global_frame = " << topicValueText(state.global_frame_name)
       << "  local_frame = " << topicValueText(state.local_frame_name)
       << "  base_frame = " << topicValueText(state.base_frame_name)
       << "\n";

    ss << colorText(" 局部odom  ", kAnsiLabel);
    if (!state.local_odom.header.stamp.isZero())
    {
        ss << "frame: " << odomFrameText(state.local_odom) << "\n";
        ss << "           位置 : " << odomPositionText(state.local_odom) << "\n";
        ss << "           速度 : " << odomVelText(state.local_odom) << "\n";
        ss << "           姿态 : " << odomAttitudeText(state.local_odom) << "\n";
    }
    else
    {
        ss << "           " << colorText("等待数据...", kAnsiWarn) << "\n";
    }

    ss << colorText(" 全局odom  ", kAnsiLabel);
    if (!state.global_odom.header.stamp.isZero())
    {
        ss << "frame: " << odomFrameText(state.global_odom) << "\n";
        ss << "           位置 : " << odomPositionText(state.global_odom) << "\n";
        ss << "           速度 : " << odomVelText(state.global_odom) << "\n";
        ss << "           姿态 : " << odomAttitudeText(state.global_odom) << "\n";
    }
    else
    {
        ss << "           " << colorText("等待数据...", kAnsiWarn) << "\n";
    }

    return ss.str();
}

void stateCallback(const sunray_msgs::OdomState::ConstPtr &msg, const int agent_id)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    CachedState &cached = g_states[agent_id];
    cached.state = *msg;
    cached.has_state = true;
    cached.receive_time = ros::Time::now();
}

void printPanel(const ros::TimerEvent &,
                const std::vector<std::string> &agent_keys,
                const std::vector<AgentTopics> &agent_topics,
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
        const AgentTopics topics = (id >= 1 && id <= static_cast<int>(agent_topics.size()))
                                       ? agent_topics[static_cast<size_t>(id - 1)]
                                       : buildAgentTopics(key, SourceConfig{});
        if (item.second.has_state && (now - item.second.receive_time).toSec() > stale_timeout)
        {
            std::cout << colorText(key + " 状态超时", kAnsiBad) << "\n";
            continue;
        }
        std::cout << buildPanel(key, topics, item.second);
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
    int source_id = 3;
    std::string config_yamlfile_path;
    double print_hz = 5.0;
    double stale_timeout = 1.0;
    nh.param("agent_name", agent_name, agent_name);
    nh.param("agent_num", agent_num, agent_num);
    nh.param("source_id", source_id, source_id);
    nh.param("config_yamlfile_path", config_yamlfile_path,
             std::string(ros::package::getPath("localization_fusion") + "/config/localization_sources.yaml"));
    nh.param("print_hz", print_hz, print_hz);
    nh.param("stale_timeout", stale_timeout, stale_timeout);

    SourceConfig source_config;
    try
    {
        source_config = load_config_from_yaml(config_yamlfile_path, source_id);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("localization_fusion_monitor failed to load config: %s", e.what());
        return 1;
    }

    const int num = std::max(1, agent_num);
    std::vector<std::string> agent_keys;
    std::vector<AgentTopics> agent_topics;
    std::vector<ros::Subscriber> subs;
    agent_keys.reserve(static_cast<size_t>(num));
    agent_topics.reserve(static_cast<size_t>(num));
    subs.reserve(static_cast<size_t>(num));

    for (int id = 1; id <= num; ++id)
    {
        const std::string key = sunray_common::normalize_agent_key(agent_name + std::to_string(id));
        const AgentTopics topics = buildAgentTopics(key, source_config);
        agent_keys.push_back(key);
        agent_topics.push_back(topics);
        g_states[id] = CachedState{};

        subs.push_back(nh.subscribe<sunray_msgs::OdomState>(
            topics.odom_state_topic, 10,
            [id](const sunray_msgs::OdomState::ConstPtr &msg) { stateCallback(msg, id); }));

        ROS_INFO("localization_fusion_monitor subscribe: %s", topics.odom_state_topic.c_str());
    }

    ros::Timer print_timer = nh.createTimer(
        ros::Duration(1.0 / std::max(0.1, print_hz)),
        [&agent_keys, &agent_topics, stale_timeout](const ros::TimerEvent &e) {
            printPanel(e, agent_keys, agent_topics, stale_timeout);
        });

    ros::spin();
    return 0;
}
