/*
本文件功能：
    1、订阅所有无人机发布的 UAVSwarmState
    2、按 agent_id 汇总后，在一个终端中统一打印集群状态面板
*/
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/Formation.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVSwarmCMD.h>
#include <sunray_msgs/UAVSwarmState.h>

namespace
{

constexpr const char *kColorReset = "\033[0m";
constexpr const char *kColorCyan = "\033[1;36m";
constexpr const char *kColorGreen = "\033[32m";
constexpr const char *kColorYellow = "\033[33m";
constexpr const char *kColorRed = "\033[31m";
constexpr const char *kColorWhite = "\033[37m";
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

std::string colorText(const std::string &text, const char *color)
{
    return std::string(color) + text + kColorReset;
}

std::string formatBool(const bool value)
{
    return value ? colorText("正常", kColorGreen) : colorText("异常", kColorRed);
}

std::string formatDouble(const double value, const int precision = 2)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

std::string formatTargetAgentId(const uint8_t agent_id)
{
    const std::string id_text = std::to_string(static_cast<int>(agent_id));
    return agent_id == 99U ? colorText(id_text, kColorGreen) : colorText(id_text, kColorWhite);
}

double yawFromOdom(const nav_msgs::Odometry &odom)
{
    const geometry_msgs::Quaternion &q = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

const char *swarmStateToString(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UAVSwarmState::INIT:
        return "INIT";
    case sunray_msgs::UAVSwarmState::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVSwarmState::LAND:
        return "LAND";
    case sunray_msgs::UAVSwarmState::RETURN_HOME:
        return "RETURN_HOME";
    case sunray_msgs::UAVSwarmState::ARRIVED:
        return "ARRIVED";
    case sunray_msgs::UAVSwarmState::SWARM_STATIC_FORMATION:
        return "SWARM_STATIC_FORMATION";
    case sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION:
        return "SWARM_DYNAMIC_FORMATION";
    case sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION_PREPARE:
        return "SWARM_DYNAMIC_FORMATION_PREPARE";
    default:
        return "UNKNOWN";
    }
}

const char *swarmCmdToString(const uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::UAVSwarmCMD::SWARM_TAKEOFF:
        return "SWARM_TAKEOFF";
    case sunray_msgs::UAVSwarmCMD::SWARM_LAND:
        return "SWARM_LAND";
    case sunray_msgs::UAVSwarmCMD::SWARM_HOVER:
        return "SWARM_HOVER";
    case sunray_msgs::UAVSwarmCMD::SWARM_RETURN:
        return "SWARM_RETURN";
    case sunray_msgs::UAVSwarmCMD::SWARM_FORMATION:
        return "SWARM_FORMATION";
    default:
        return "UNDEFINE";
    }
}

const char *formationTypeToString(const uint8_t formation_type)
{
    switch (formation_type)
    {
    case sunray_msgs::Formation::STATIC_KEEP_FORMATION:
        return "STATIC_KEEP";
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        return "STATIC_LINE";
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        return "STATIC_POLYGON";
    case sunray_msgs::Formation::STATIC_FORMATION_RANDOM:
        return "STATIC_RANDOM";
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        return "STATIC_CUSTOM";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        return "DYNAMIC_RING";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        return "DYNAMIC_POLYGON";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        return "DYNAMIC_LEMNISCATE";
    default:
        return "UNDEFINE";
    }
}

const char *uavControlCmdToString(const uint8_t cmd)
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

std::string agentPrefix(const std::string &agent_name, const sunray_msgs::UAVSwarmState &state)
{
    return "/" + agent_name + std::to_string(static_cast<int>(state.agent_id));
}

std::string swarmStateTopicText(const std::string &swarm_state_topic)
{
    return colorText(swarm_state_topic, kColorWhite);
}

std::string localOdomTopicText(const std::string &agent_name, const sunray_msgs::UAVSwarmState &state)
{
    return colorText(agentPrefix(agent_name, state) + "/sunray/localization/local_odom", kColorWhite);
}

std::string peerOdomTopicText(const std::string &agent_name, const sunray_msgs::UAVSwarmState &state)
{
    const int self_id = static_cast<int>(state.agent_id);
    const int swarm_num = static_cast<int>(state.swarm_num);
    if (swarm_num <= 1)
    {
        return colorText("无邻居", kColorYellow);
    }

    std::ostringstream range_ss;
    bool has_range = false;
    auto appendRange = [&](const int begin, const int end) {
        if (begin > end)
        {
            return;
        }

        if (has_range)
        {
            range_ss << ",";
        }

        range_ss << begin;
        if (begin != end)
        {
            range_ss << "-" << end;
        }
        has_range = true;
    };

    appendRange(1, self_id - 1);
    appendRange(self_id + 1, swarm_num);

    return colorText("/" + agent_name + "[" + range_ss.str() + "]/sunray/localization/local_odom", kColorWhite);
}

std::string swarmCmdTopicText()
{
    return colorText("/sunray/swarm/uav_swarm_cmd", kColorWhite);
}

std::string controlCmdTopicText(const std::string &agent_name, const sunray_msgs::UAVSwarmState &state)
{
    return colorText(agentPrefix(agent_name, state) + "/sunray/uav_control/control_cmd", kColorWhite);
}

std::string formatFormationCommand(const sunray_msgs::Formation &formation_cmd)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss << formationTypeToString(formation_cmd.formation_type)
       << "  leader=(" << formation_cmd.leader_pos.x << ", " << formation_cmd.leader_pos.y << ", "
       << formation_cmd.leader_pos.z << ")"
       << "  yaw=" << static_cast<double>(formation_cmd.leader_yaw) * kRadToDeg << " deg";

    switch (formation_cmd.formation_type)
    {
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        ss << "  spacing=" << formation_cmd.static_line_spacing
           << "  angle=" << static_cast<double>(formation_cmd.static_line_angle) << " deg";
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        ss << "  spacing=" << formation_cmd.static_polygon_spacing;
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_RANDOM:
        ss << "  rule=safe_random";
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        ss << "  custom_num=" << formation_cmd.custom_offsets_pos.size();
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        ss << "  radius=" << formation_cmd.dynamic_ring_radius
           << "  speed=" << formation_cmd.dynamic_ring_move_speed
           << "  time=" << formation_cmd.dynamic_time << " s";
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        ss << "  spacing=" << formation_cmd.dynamic_polygon_spacing
           << "  speed=" << formation_cmd.dynamic_polygon_move_speed
           << "  time=" << formation_cmd.dynamic_time << " s";
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        ss << "  x_radius=" << formation_cmd.dynamic_lemniscate_x_radius
           << "  y_radius=" << formation_cmd.dynamic_lemniscate_y_radius
           << "  speed=" << formation_cmd.dynamic_lemniscate_move_speed
           << "  time=" << formation_cmd.dynamic_time << " s";
        break;
    default:
        break;
    }

    return ss.str();
}

std::string formatTarget(const sunray_msgs::UAVSwarmState &state)
{
    if (!state.target_valid)
    {
        return colorText("无有效目标点", kColorYellow);
    }

    std::ostringstream ss;
    ss << "valid=true  pos=(" << formatDouble(state.target_pos.x) << ", " << formatDouble(state.target_pos.y) << ", "
       << formatDouble(state.target_pos.z) << ") yaw=" << formatDouble(state.target_yaw * kRadToDeg) << " deg";
    return ss.str();
}

std::string formatUavCommand(const sunray_msgs::UAVControlCMD &cmd)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "控制指令 = " << colorText(uavControlCmdToString(cmd.control_cmd), kColorYellow);

    switch (cmd.control_cmd)
    {
    case sunray_msgs::UAVControlCMD::TAKEOFF:
        ss << "  原始输入 -> TAKEOFF";
        break;
    case sunray_msgs::UAVControlCMD::LAND:
        ss << "  原始输入 -> LAND";
        break;
    case sunray_msgs::UAVControlCMD::RETURN:
        ss << "  原始输入 -> RETURN";
        break;
    case sunray_msgs::UAVControlCMD::KILL:
        ss << "  原始输入 -> KILL";
        break;
    case sunray_msgs::UAVControlCMD::HOVER:
        ss << "  原始输入 -> HOVER";
        break;
    case sunray_msgs::UAVControlCMD::MOVE_POINT:
        ss << "  原始输入 -> desired_pos=(" << cmd.desired_pos.x << ", " << cmd.desired_pos.y << ", "
           << cmd.desired_pos.z << ") m  yaw=" << cmd.desired_yaw * kRadToDeg << " deg";
        break;
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY:
        ss << "  原始输入 -> desired_vel=(" << cmd.desired_vel.x << ", " << cmd.desired_vel.y << ", "
           << cmd.desired_vel.z << ") m/s  yaw=" << cmd.desired_yaw * kRadToDeg << " deg";
        break;
    case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
        ss << "  原始输入 -> body_pos=(" << cmd.desired_body_xy_pos.x << ", " << cmd.desired_body_xy_pos.y
           << ") m  fixed_height=" << cmd.fixed_height << " m  yaw=" << cmd.desired_yaw * kRadToDeg << " deg";
        break;
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY:
        ss << "  原始输入 -> body_vel=(" << cmd.desired_body_xy_vel.x << ", " << cmd.desired_body_xy_vel.y
           << ") m/s  fixed_height=" << cmd.fixed_height << " m  yaw=" << cmd.desired_yaw * kRadToDeg << " deg";
        break;
    case sunray_msgs::UAVControlCMD::MOVE_POINT_WGS84:
        ss << "  原始输入 -> wgs84=(" << cmd.desired_wgs84_pos.latitude << ", "
           << cmd.desired_wgs84_pos.longitude << ", " << cmd.desired_wgs84_pos.altitude << ")";
        break;
    default:
        break;
    }

    return ss.str();
}

std::string formatSwarmControlUavStatus(const sunray_msgs::UAVSwarmState &swarm_state,
                                        const std::string &agent_name,
                                        const std::string &swarm_state_topic)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << kColorCyan << "================ UAV 集群控制状态面板 | " << agent_name
       << static_cast<int>(swarm_state.agent_id) << " ================" << kColorReset << '\n';

    ss << " 基本状态  状态话题（发布） -> " << swarmStateTopicText(swarm_state_topic) << '\n';
    ss << "           Name = " << agent_name
       << "  ID = " << static_cast<int>(swarm_state.agent_id)
       << "  集群数量 = " << swarm_state.swarm_num
       << "  集群FSM = " << colorText(swarmStateToString(swarm_state.fsm_state), kColorYellow) << '\n';

    ss << " 集群位姿  本机位姿话题（订阅） -> " << localOdomTopicText(agent_name, swarm_state) << '\n';
    ss << "           邻居位姿话题（订阅） -> " << peerOdomTopicText(agent_name, swarm_state) << '\n';
    ss << "           本机ODOM状态 = " << formatBool(swarm_state.self_odom_ready)
       << "  邻居ODOM状态 = "
       << (swarm_state.peers_odom_ready
               ? colorText("正常", kColorGreen)
               : colorText("异常", kColorRed) + "(" + std::to_string(swarm_state.ready_peer_num) + "/" +
                     std::to_string(static_cast<int>(std::max<uint32_t>(swarm_state.swarm_num, 1) - 1)) + ")")
       << '\n';

    if (swarm_state.self_odom_ready)
    {
        const geometry_msgs::Point &pos = swarm_state.self_odom.pose.pose.position;
        ss << "           本机odom信息 -> x = " << formatDouble(pos.x)
           << " m  y = " << formatDouble(pos.y)
           << " m  z = " << formatDouble(pos.z)
           << " m  yaw = " << formatDouble(yawFromOdom(swarm_state.self_odom) * kRadToDeg)
           << " deg\n";
    }
    else
    {
        ss << "           无有效本机位姿\n";
    }

    ss << " 集群输入  集群指令话题（订阅） -> " << swarmCmdTopicText() << '\n';
    ss << "           外部命令 = " << colorText(swarmCmdToString(swarm_state.swarm_cmd.swarm_cmd), kColorYellow)
       << "  目标ID = " << formatTargetAgentId(swarm_state.swarm_cmd.agent_id) << '\n';

    if (swarm_state.swarm_cmd.swarm_cmd == sunray_msgs::UAVSwarmCMD::SWARM_FORMATION)
    {
        ss << "           阵型参数 -> " << formatFormationCommand(swarm_state.swarm_cmd.formation_cmd) << '\n';
    }

    ss << "           控制目标 -> " << formatTarget(swarm_state) << '\n';

    ss << " 控制输出  控制指令话题（发布） -> " << controlCmdTopicText(agent_name, swarm_state) << '\n';
    ss << "           " << formatUavCommand(swarm_state.uav_cmd) << '\n';
    return ss.str();
}

struct CachedState
{
    sunray_msgs::UAVSwarmState state{};
    ros::Time receive_time{0.0};
};

class SwarmControlUavMonitorNode
{
  public:
    SwarmControlUavMonitorNode()
        : nh_(), private_nh_("~")
    {
        private_nh_.param("swarm_state_topic", swarm_state_topic_, std::string("/sunray/swarm/uav_swarm_state"));
        private_nh_.param("agent_name", agent_name_, std::string("uav"));
        private_nh_.param("display_hz", display_hz_, 1.0);
        private_nh_.param("stale_timeout", stale_timeout_, 1.0);
        private_nh_.param("clear_screen", clear_screen_, true);

        display_hz_ = std::max(0.1, display_hz_);
        stale_timeout_ = std::max(0.1, stale_timeout_);

        swarm_state_sub_ =
            nh_.subscribe(swarm_state_topic_, 100, &SwarmControlUavMonitorNode::swarmStateCallback, this);
        print_timer_ = nh_.createTimer(ros::Duration(1.0 / display_hz_),
                                       &SwarmControlUavMonitorNode::printTimerCallback, this);
    }

  private:
    void swarmStateCallback(const sunray_msgs::UAVSwarmState::ConstPtr &msg)
    {
        if (msg->agent_id == 0)
        {
            return;
        }

        CachedState &cached_state = states_[static_cast<int>(msg->agent_id)];
        cached_state.state = *msg;
        cached_state.receive_time = ros::Time::now();
    }

    void printTimerCallback(const ros::TimerEvent &)
    {
        std::ostringstream ss;
        if (clear_screen_)
        {
            ss << "\033[2J\033[H";
        }

        if (states_.empty())
        {
            ss << kColorRed << "等待 UAVSwarmState..." << kColorReset << '\n';
            std::cout << ss.str() << std::flush;
            return;
        }

        const ros::Time now = ros::Time::now();
        for (const auto &item : states_)
        {
            const CachedState &cached_state = item.second;
            const double age = (now - cached_state.receive_time).toSec();
            if (age > stale_timeout_)
            {
                ss << kColorRed << agent_name_ << item.first << " 状态超时" << kColorReset << '\n';
                continue;
            }

            ss << formatSwarmControlUavStatus(cached_state.state, agent_name_, swarm_state_topic_);
        }

        std::cout << ss.str() << std::flush;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber swarm_state_sub_{};
    ros::Timer print_timer_{};

    std::string swarm_state_topic_{"/sunray/swarm/uav_swarm_state"};
    std::string agent_name_{"uav"};
    double display_hz_{1.0};
    double stale_timeout_{1.0};
    bool clear_screen_{true};
    std::map<int, CachedState> states_{};
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_control_uav_monitor_node");
    SwarmControlUavMonitorNode node;
    ros::spin();
    return 0;
}
