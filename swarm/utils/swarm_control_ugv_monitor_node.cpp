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

std::string formatTarget(const sunray_msgs::UGVSwarmState &state)
{
    if (!state.target_valid)
    {
        return colorText("无有效目标点", kColorYellow);
    }

    std::ostringstream ss;
    ss << "valid=true  pos=(" << formatDouble(state.target_pos.x) << ", " << formatDouble(state.target_pos.y) << ", "
       << formatDouble(state.target_pos.z) << ") yaw=" << formatDouble(state.target_yaw * 180.0 / M_PI) << " deg";
    return ss.str();
}

std::string formatUgvStatus(const sunray_msgs::UGVSwarmState &state)
{
    std::ostringstream ss;
    ss << kColorCyan << "================ UGV 集群控制状态面板 | ugv" << static_cast<int>(state.agent_id)
       << " ================" << kColorReset << '\n';

    ss << " 基本状态 | 集群数量=" << state.swarm_num << "  本机ID=" << static_cast<int>(state.agent_id)
       << "  本机ODOM=" << formatBool(state.self_odom_ready) << "  邻居ODOM="
       << (state.peers_odom_ready ? colorText("正常", kColorGreen)
                                  : colorText("异常", kColorRed) + "(" +
                                        std::to_string(state.ready_peer_num) + ")")
       << '\n';

    if (state.self_odom_ready)
    {
        const geometry_msgs::Point &pos = state.self_odom.pose.pose.position;
        ss << " 本机位姿 | x=" << formatDouble(pos.x) << "  y=" << formatDouble(pos.y)
           << "  z=" << formatDouble(pos.z) << "  yaw=" << formatDouble(yawFromOdom(state.self_odom) * 180.0 / M_PI)
           << " deg\n";
    }
    else
    {
        ss << " 本机位姿 | 无有效odom\n";
    }

    ss << " 控制状态 | 集群FSM=" << colorText(swarmStateName(state.fsm_state), kColorYellow)
       << "  外部命令=" << swarmCmdName(state.swarm_cmd.swarm_cmd);
    if (state.swarm_cmd.agent_id == 99U)
    {
        ss << "  目标ID=" << colorText("99", kColorGreen);
    }
    else
    {
        ss << "  目标ID=" << static_cast<int>(state.swarm_cmd.agent_id);
    }
    ss << '\n';

    if (state.swarm_cmd.swarm_cmd == sunray_msgs::UGVSwarmCMD::SWARM_FORMATION)
    {
        ss << " 阵型参数 | " << formationName(state.swarm_cmd.formation_cmd.formation_type)
           << "  leader=(" << formatDouble(state.swarm_cmd.formation_cmd.leader_pos.x) << ", "
           << formatDouble(state.swarm_cmd.formation_cmd.leader_pos.y) << ", "
           << formatDouble(state.swarm_cmd.formation_cmd.leader_pos.z) << ")\n";
    }

    ss << " 目标点   | " << formatTarget(state) << '\n';
    ss << " 发布指令 | 模式=" << colorText(controlCmdName(state.ugv_cmd.control_cmd), kColorYellow);
    if (state.ugv_cmd.control_cmd == sunray_msgs::UGVControlCMD::MOVE_VELOCITY)
    {
        ss << "  vel=(" << formatDouble(state.ugv_cmd.desired_vel.x) << ", "
           << formatDouble(state.ugv_cmd.desired_vel.y) << ")  yaw="
           << formatDouble(state.ugv_cmd.desired_yaw * 180.0 / M_PI) << " deg";
    }
    ss << '\n';
    return ss.str();
}

class SwarmControlUgvMonitorNode
{
  public:
    SwarmControlUgvMonitorNode()
        : nh_(), private_nh_("~")
    {
        private_nh_.param("swarm_state_topic", swarm_state_topic_, std::string("/sunray/swarm/ugv_swarm_state"));
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
                std::cout << kColorRed << "ugv" << item.first << " 状态超时" << kColorReset << std::endl;
                continue;
            }
            std::cout << formatUgvStatus(cache.state);
        }
        std::cout.flush();
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber swarm_state_sub_;
    ros::Timer display_timer_;
    std::string swarm_state_topic_;
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
