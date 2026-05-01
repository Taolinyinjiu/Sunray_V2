/*
本文件功能：
    1、订阅所有无人机发布的 UAVSwarmState
    2、按 uav_id 汇总后，在一个终端中统一打印集群状态面板
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

std::string panelTitle(const int agent_id)
{
    return std::string("无人机集群控制状态面板 /uav") + std::to_string(agent_id);
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

std::string formatPeerOdomState(const sunray_msgs::UAVSwarmState &swarm_state)
{
    const uint32_t expected_peer_num = swarm_state.swarm_num > 0 ? swarm_state.swarm_num - 1U : 0U;
    std::ostringstream ss;
    ss << "邻居ODOM(" << swarm_state.ready_peer_num << "/" << expected_peer_num << ")";
    return swarm_state.peers_odom_ready ? ss.str() : colorText(ss.str(), kColorRed);
}

std::string formatSelfOdomState(const bool ready)
{
    return ready ? std::string("本机ODOM(正常)") : colorText("本机ODOM(异常)", kColorRed);
}

std::string pointText(const std::string &name, const geometry_msgs::Point &pos, const float yaw)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << name << "=(" << pos.x << ", " << pos.y << ", " << pos.z << ", "
       << static_cast<double>(yaw) * kRadToDeg << " deg)";
    return ss.str();
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

std::string formationParamSummary(const sunray_msgs::Formation &formation_cmd)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss << " 阵型参数 | 类型=" << colorText(formationTypeToString(formation_cmd.formation_type), kColorYellow)
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
        ss << "  time=" << formation_cmd.dynamic_time << " s"
           << "  radius=" << formation_cmd.dynamic_ring_radius
           << "  speed=" << formation_cmd.dynamic_ring_move_speed;
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        ss << "  time=" << formation_cmd.dynamic_time << " s"
           << "  spacing=" << formation_cmd.dynamic_polygon_spacing
           << "  speed=" << formation_cmd.dynamic_polygon_move_speed;
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        ss << "  time=" << formation_cmd.dynamic_time << " s"
           << "  x_radius=" << formation_cmd.dynamic_lemniscate_x_radius
           << "  y_radius=" << formation_cmd.dynamic_lemniscate_y_radius
           << "  speed=" << formation_cmd.dynamic_lemniscate_move_speed;
        break;
    default:
        break;
    }

    return ss.str();
}

std::string controlOutputSummary(const sunray_msgs::UAVControlCMD &uav_cmd)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << " 发布指令 | 模式=" << colorText(uavControlCmdToString(uav_cmd.control_cmd), kColorYellow);

    if (uav_cmd.control_cmd == sunray_msgs::UAVControlCMD::MOVE_VELOCITY)
    {
        ss << "  vx=" << uav_cmd.desired_vel.x
           << "  vy=" << uav_cmd.desired_vel.y
           << "  vz=" << uav_cmd.desired_vel.z
           << "  yaw=" << static_cast<double>(uav_cmd.desired_yaw) * kRadToDeg << " deg";
    }
    else if (uav_cmd.control_cmd == sunray_msgs::UAVControlCMD::MOVE_POINT)
    {
        ss << "  x=" << uav_cmd.desired_pos.x
           << "  y=" << uav_cmd.desired_pos.y
           << "  z=" << uav_cmd.desired_pos.z
           << "  yaw=" << static_cast<double>(uav_cmd.desired_yaw) * kRadToDeg << " deg";
    }

    return ss.str();
}

std::string formatSwarmControlUavStatus(const sunray_msgs::UAVSwarmState &swarm_state)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << kColorCyan << panelTitle(static_cast<int>(swarm_state.agent_id)) << kColorReset << '\n'
       << " 基本状态 | 集群数量=" << swarm_state.swarm_num
       << "  本机ID=" << static_cast<int>(swarm_state.agent_id)
       << "  " << formatSelfOdomState(swarm_state.self_odom_ready)
       << "  " << formatPeerOdomState(swarm_state) << '\n';

    if (swarm_state.self_odom_ready)
    {
        ss << " 自机位姿 | x=" << swarm_state.self_odom.pose.pose.position.x
           << "  y=" << swarm_state.self_odom.pose.pose.position.y
           << "  z=" << swarm_state.self_odom.pose.pose.position.z
           << "  yaw=" << yawFromOdom(swarm_state.self_odom) * kRadToDeg << " deg" << '\n';
    }
    else
    {
        ss << " 自机位姿 | " << colorText("无有效 odom", kColorYellow) << '\n';
    }

    ss << " 控制状态 | 集群FSM=" << colorText(swarmStateToString(swarm_state.fsm_state), kColorYellow);
    if (swarm_state.target_valid && swarm_state.fsm_state == sunray_msgs::UAVSwarmState::ARRIVED)
    {
        ss << "  " << pointText("悬停点", swarm_state.target_pos, swarm_state.target_yaw);
    }
    else if (swarm_state.target_valid && swarm_state.fsm_state == sunray_msgs::UAVSwarmState::RETURN_HOME)
    {
        ss << "  " << pointText("返航点", swarm_state.target_pos, swarm_state.target_yaw);
    }
    ss << '\n';

    ss << " 输入命令 | 目标ID=" << formatTargetAgentId(swarm_state.swarm_cmd.agent_id)
       << "  命令=" << colorText(swarmCmdToString(swarm_state.swarm_cmd.swarm_cmd), kColorYellow) << '\n';

    if (swarm_state.swarm_cmd.swarm_cmd == sunray_msgs::UAVSwarmCMD::SWARM_FORMATION)
    {
        ss << formationParamSummary(swarm_state.swarm_cmd.formation_cmd) << '\n';
    }

    if (swarm_state.fsm_state == sunray_msgs::UAVSwarmState::SWARM_STATIC_FORMATION ||
        swarm_state.fsm_state == sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION)
    {
        if (swarm_state.target_valid)
        {
            ss << " 目标点   | " << pointText("goal", swarm_state.target_pos, swarm_state.target_yaw) << '\n';
        }
        else
        {
            ss << " 目标点   | " << colorText("无有效目标点", kColorYellow) << '\n';
        }
    }

    ss << controlOutputSummary(swarm_state.uav_cmd) << '\n';
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

        ss << kColorCyan << "无人机集群控制集中监控 | topic=" << swarm_state_topic_
           << " | online=" << onlineCount() << "/" << states_.size() << kColorReset << "\n\n";

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
                ss << kColorRed << "无人机集群控制状态面板 /uav" << item.first
                   << "  状态超时 age=" << std::fixed << std::setprecision(2) << age << " s" << kColorReset
                   << "\n\n";
                continue;
            }

            ss << formatSwarmControlUavStatus(cached_state.state) << '\n';
        }

        std::cout << ss.str() << std::flush;
    }

    size_t onlineCount() const
    {
        const ros::Time now = ros::Time::now();
        size_t count = 0;
        for (const auto &item : states_)
        {
            if ((now - item.second.receive_time).toSec() <= stale_timeout_)
            {
                ++count;
            }
        }
        return count;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber swarm_state_sub_{};
    ros::Timer print_timer_{};

    std::string swarm_state_topic_{"/sunray/swarm/uav_swarm_state"};
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
