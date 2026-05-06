/*
本程序功能：
    1、订阅所有无人车发布的 UGVSwarmState
    2、在一个终端中集中显示 UGV 集群控制关键状态，避免多个节点打印互相刷屏
*/
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sunray_msgs/Formation.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVSwarmCMD.h>
#include <sunray_msgs/UGVSwarmState.h>

namespace
{

constexpr const char *kColorReset = "\033[0m";
constexpr const char *kColorGreen = "\033[1;32m";
constexpr const char *kColorRed = "\033[1;31m";
constexpr const char *kColorYellow = "\033[1;33m";
constexpr const char *kColorCyan = "\033[1;36m";
constexpr const char *kColorWhite = "\033[1;37m";
constexpr double kRadToDeg = 57.29577951308232;

struct CachedState
{
    sunray_msgs::UGVSwarmState state{};
    ros::Time receive_time{0.0};
};

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

double yawFromOdom(const nav_msgs::Odometry &odom)
{
    const geometry_msgs::Quaternion &q = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

const char *swarmStateName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UGVSwarmState::INIT:
        return "INIT";
    case sunray_msgs::UGVSwarmState::RETURN_HOME:
        return "RETURN_HOME";
    case sunray_msgs::UGVSwarmState::ARRIVED:
        return "ARRIVED";
    case sunray_msgs::UGVSwarmState::SWARM_STATIC_FORMATION:
        return "SWARM_STATIC_FORMATION";
    case sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION:
        return "SWARM_DYNAMIC_FORMATION";
    case sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION_PREPARE:
        return "SWARM_DYNAMIC_FORMATION_PREPARE";
    default:
        return "UNKNOWN";
    }
}

const char *swarmCmdName(const uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::UGVSwarmCMD::SWARM_HOLD:
        return "SWARM_HOLD";
    case sunray_msgs::UGVSwarmCMD::SWARM_RETURN:
        return "SWARM_RETURN";
    case sunray_msgs::UGVSwarmCMD::SWARM_FORMATION:
        return "SWARM_FORMATION";
    default:
        return "UNDEFINE";
    }
}

const char *formationName(const uint8_t formation_type)
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
        return "UNKNOWN";
    }
}

const char *controlCmdName(const uint8_t cmd)
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

std::string agentPrefix(const std::string &agent_name, const sunray_msgs::UGVSwarmState &state)
{
    return "/" + agent_name + std::to_string(static_cast<int>(state.agent_id));
}

std::string swarmStateTopicText()
{
    return colorText("/sunray/swarm/ugv_swarm_state", kColorWhite);
}

std::string localOdomTopicText(const std::string &agent_name, const sunray_msgs::UGVSwarmState &state)
{
    return colorText(agentPrefix(agent_name, state) + "/sunray/localization/local_odom", kColorWhite);
}

std::string peerOdomTopicText(const std::string &agent_name, const sunray_msgs::UGVSwarmState &state)
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
    return colorText("/sunray/swarm/ugv_swarm_cmd", kColorWhite);
}

std::string controlCmdTopicText(const std::string &agent_name, const sunray_msgs::UGVSwarmState &state)
{
    return colorText(agentPrefix(agent_name, state) + "/sunray/ugv_control/control_cmd", kColorWhite);
}

std::string formatSwarmCmdTargetId(const uint8_t agent_id)
{
    if (agent_id == 99U)
    {
        return colorText("99", kColorGreen);
    }
    return std::to_string(static_cast<int>(agent_id));
}

std::string formatTarget(const sunray_msgs::UGVSwarmState &state)
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

std::string formatFormationCommand(const sunray_msgs::Formation &formation)
{
    std::ostringstream ss;
    ss << formationName(formation.formation_type)
       << "  leader=(" << formatDouble(formation.leader_pos.x) << ", "
       << formatDouble(formation.leader_pos.y) << ", " << formatDouble(formation.leader_pos.z)
       << ")  yaw=" << formatDouble(formation.leader_yaw * kRadToDeg) << " deg";

    switch (formation.formation_type)
    {
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        ss << "  spacing=" << formatDouble(formation.static_line_spacing)
           << "  angle=" << formatDouble(formation.static_line_angle) << " deg";
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        ss << "  spacing=" << formatDouble(formation.static_polygon_spacing);
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        ss << "  radius=" << formatDouble(formation.dynamic_ring_radius)
           << "  speed=" << formatDouble(formation.dynamic_ring_move_speed)
           << "  time=" << formatDouble(formation.dynamic_time) << " s";
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        ss << "  spacing=" << formatDouble(formation.dynamic_polygon_spacing)
           << "  speed=" << formatDouble(formation.dynamic_polygon_move_speed)
           << "  time=" << formatDouble(formation.dynamic_time) << " s";
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        ss << "  x_radius=" << formatDouble(formation.dynamic_lemniscate_x_radius)
           << "  y_radius=" << formatDouble(formation.dynamic_lemniscate_y_radius)
           << "  speed=" << formatDouble(formation.dynamic_lemniscate_move_speed)
           << "  time=" << formatDouble(formation.dynamic_time) << " s";
        break;
    default:
        break;
    }

    return ss.str();
}

std::string formatUgvCommand(const sunray_msgs::UGVControlCMD &cmd)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "控制指令 = " << colorText(controlCmdName(cmd.control_cmd), kColorYellow);

    switch (cmd.control_cmd)
    {
    case sunray_msgs::UGVControlCMD::HOLD:
        ss << "  原始输入 -> HOLD";
        break;
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
        ss << "  原始输入 -> desired_pos=(" << cmd.desired_pos.x << ", " << cmd.desired_pos.y << ", "
           << cmd.desired_pos.z << ") m  yaw=" << cmd.desired_yaw * kRadToDeg << " deg";
        break;
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
        ss << "  原始输入 -> desired_vel=(" << cmd.desired_vel.x << ", " << cmd.desired_vel.y << ", "
           << cmd.desired_vel.z << ") m/s  yaw=" << cmd.desired_yaw * kRadToDeg << " deg";
        break;
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
        ss << "  原始输入 -> cmd_vel=(" << cmd.cmd_vel.linear.x << ", " << cmd.cmd_vel.linear.y << ", "
           << cmd.cmd_vel.angular.z * kRadToDeg << " deg/s)";
        break;
    default:
        break;
    }

    return ss.str();
}

std::string formatUgvStatus(const sunray_msgs::UGVSwarmState &state, const std::string &agent_name)
{
    std::ostringstream ss;
    ss << kColorCyan << "================ UGV 集群控制状态面板 | " << agent_name << static_cast<int>(state.agent_id)
       << " ================" << kColorReset << '\n';

    ss << " 基本状态  状态话题（发布） -> " << swarmStateTopicText() << '\n';
    ss << "           Name = " << agent_name
       << "  ID = " << static_cast<int>(state.agent_id)
       << "  集群数量 = " << state.swarm_num
       << "  集群FSM = " << colorText(swarmStateName(state.fsm_state), kColorYellow) << '\n';

    ss << " 集群位姿  本机位姿话题（订阅） -> " << localOdomTopicText(agent_name, state) << '\n';
    ss << "           邻居位姿话题（订阅） -> " << peerOdomTopicText(agent_name, state) << '\n';
    ss << "           本机ODOM状态 = " << formatBool(state.self_odom_ready)
       << "  邻居ODOM状态 = "
       << (state.peers_odom_ready ? colorText("正常", kColorGreen)
                                  : colorText("异常", kColorRed) + "(" +
                                        std::to_string(state.ready_peer_num) + "/" +
                                        std::to_string(static_cast<int>(std::max<uint32_t>(state.swarm_num, 1) - 1)) +
                                        ")")
       << '\n';

    if (state.self_odom_ready)
    {
        const geometry_msgs::Point &pos = state.self_odom.pose.pose.position;
        ss << "           本机odom信息 -> x = " << formatDouble(pos.x)
           << " m  y = " << formatDouble(pos.y)
           << " m  z = " << formatDouble(pos.z)
           << " m  yaw = " << formatDouble(yawFromOdom(state.self_odom) * kRadToDeg)
           << " deg\n";
    }
    else
    {
        ss << "           无有效本机位姿\n";
    }

    ss << " 集群输入  集群指令话题（订阅） -> " << swarmCmdTopicText() << '\n';
    ss << "           外部命令 = " << colorText(swarmCmdName(state.swarm_cmd.swarm_cmd), kColorYellow)
       << "  目标ID = " << formatSwarmCmdTargetId(state.swarm_cmd.agent_id) << '\n';

    if (state.swarm_cmd.swarm_cmd == sunray_msgs::UGVSwarmCMD::SWARM_FORMATION)
    {
        ss << "           阵型参数 -> " << formatFormationCommand(state.swarm_cmd.formation_cmd) << '\n';
    }

    ss << "           控制目标 -> " << formatTarget(state) << '\n';

    ss << " 控制输出  控制指令话题（发布） -> " << controlCmdTopicText(agent_name, state) << '\n';
    ss << "           " << formatUgvCommand(state.ugv_cmd) << '\n';
    return ss.str();
}

class SwarmControlUgvMonitorNode
{
  public:
    SwarmControlUgvMonitorNode()
        : nh_(), private_nh_("~")
    {
        private_nh_.param("swarm_state_topic", swarm_state_topic_, std::string("/sunray/swarm/ugv_swarm_state"));
        private_nh_.param("agent_name", agent_name_, std::string("ugv"));
        private_nh_.param("display_hz", display_hz_, 1.0);
        private_nh_.param("stale_timeout", stale_timeout_, 1.0);
        private_nh_.param("clear_screen", clear_screen_, true);

        display_hz_ = std::max(0.2, display_hz_);
        swarm_state_sub_ = nh_.subscribe(swarm_state_topic_, 100, &SwarmControlUgvMonitorNode::swarmStateCallback, this);
        display_timer_ = nh_.createTimer(ros::Duration(1.0 / display_hz_),
                                         &SwarmControlUgvMonitorNode::displayTimerCallback, this);
    }

  private:
    void swarmStateCallback(const sunray_msgs::UGVSwarmState::ConstPtr &msg)
    {
        if (msg->agent_id == 0)
        {
            return;
        }

        CachedState &cache = states_[static_cast<int>(msg->agent_id)];
        cache.state = *msg;
        cache.receive_time = ros::Time::now();
    }

    void displayTimerCallback(const ros::TimerEvent &)
    {
        if (clear_screen_)
        {
            std::cout << "\033[2J\033[H";
        }

        const ros::Time now = ros::Time::now();
        if (states_.empty())
        {
            std::cout << kColorRed << "等待 UGVSwarmState..." << kColorReset << std::endl;
            return;
        }

        for (const auto &item : states_)
        {
            const CachedState &cache = item.second;
            if ((now - cache.receive_time).toSec() > stale_timeout_)
            {
                std::cout << kColorRed << agent_name_ << item.first << " 状态超时" << kColorReset << std::endl;
                continue;
            }
            std::cout << formatUgvStatus(cache.state, agent_name_);
        }
        std::cout.flush();
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber swarm_state_sub_;
    ros::Timer display_timer_;
    std::string swarm_state_topic_;
    std::string agent_name_{"ugv"};
    double display_hz_{1.0};
    double stale_timeout_{1.0};
    bool clear_screen_{true};
    std::map<int, CachedState> states_;
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_control_ugv_monitor_node");
    SwarmControlUgvMonitorNode node;
    ros::spin();
    return 0;
}
