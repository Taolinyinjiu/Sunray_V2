/*
本程序功能：
    1、通过终端交互发布 UGVSwarmCMD，便于联调无人车集群控制模块
    2、覆盖 HOLD / RETURN / FORMATION 三类 UGV 集群指令
    3、当发布编队指令时，先显示当前参数，再按需修改某一项
*/
#include "sunray_log.hpp"

#include <geometry_msgs/Point.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <ros/ros.h>
#include <signal.h>
#include <sstream>
#include <string>
#include <sunray_msgs/Formation.h>
#include <sunray_msgs/UGVSwarmCMD.h>
#include <vector>

namespace
{

constexpr const char *kColorReset = "\033[0m";
constexpr const char *kColorTitle = "\033[1;36m";
constexpr const char *kColorItem = "\033[1;32m";
constexpr const char *kColorWarn = "\033[1;33m";
constexpr const char *kColorError = "\033[1;31m";
constexpr const char *kColorInfo = "\033[1;34m";

struct TerminalSession
{
    int agent_id{99};
    sunray_msgs::Formation formation_cmd{};
};

void sigintHandler(int)
{
    SUNRAY_INFO("ugv_swarm_cmd_pub_terminal exit...");
    ros::shutdown();
}

std::string trim(const std::string &text)
{
    const size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return "";
    }
    const size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

template <typename T>
std::string formatValue(const T &value)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << value;
    return ss.str();
}

template <>
std::string formatValue<int>(const int &value)
{
    return std::to_string(value);
}

template <typename T>
bool parseValue(const std::string &text, T &value)
{
    std::istringstream ss(text);
    ss >> value;
    return ss && ss.eof();
}

template <typename T>
T promptValue(const std::string &label, const T &default_value)
{
    while (ros::ok())
    {
        std::cout << kColorInfo << label << " [" << formatValue(default_value) << "] : " << kColorReset << std::flush;
        std::string line;
        if (!std::getline(std::cin, line))
        {
            ros::shutdown();
            return default_value;
        }

        const std::string input = trim(line);
        if (input.empty())
        {
            return default_value;
        }

        T value{};
        if (parseValue(input, value))
        {
            return value;
        }

        std::cout << kColorError << "输入无效，请重新输入。" << kColorReset << std::endl;
    }
    return default_value;
}

void printKeyValue(const std::string &key, const std::string &value)
{
    std::cout << "  " << std::left << std::setw(24) << key << " : " << value << std::endl;
}

const char *formationTypeName(const uint8_t formation_type)
{
    switch (formation_type)
    {
    case sunray_msgs::Formation::STATIC_KEEP_FORMATION:
        return "STATIC_KEEP_FORMATION";
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        return "STATIC_FORMATION_LINE";
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        return "STATIC_FORMATION_POLYGON";
    case sunray_msgs::Formation::STATIC_FORMATION_RANDOM:
        return "STATIC_FORMATION_RANDOM";
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        return "STATIC_FORMATION_CUSTOM";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        return "DYNAMIC_FORMATION_RING";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        return "DYNAMIC_FORMATION_POLYGON";
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        return "DYNAMIC_FORMATION_LEMNISCATE";
    default:
        return "UNKNOWN";
    }
}

const char *swarmCmdName(const uint8_t swarm_cmd)
{
    switch (swarm_cmd)
    {
    case sunray_msgs::UGVSwarmCMD::SWARM_HOLD:
        return "SWARM_HOLD";
    case sunray_msgs::UGVSwarmCMD::SWARM_RETURN:
        return "SWARM_RETURN";
    case sunray_msgs::UGVSwarmCMD::SWARM_FORMATION:
        return "SWARM_FORMATION";
    default:
        return "UNKNOWN";
    }
}

void printMainMenu(const std::string &topic_name, const TerminalSession &session)
{
    std::cout << std::endl;
    std::cout << kColorTitle << "================ UGV 集群指令终端 ================" << kColorReset << std::endl;
    printKeyValue("发布话题", topic_name);
    printKeyValue("默认目标ID", std::to_string(session.agent_id));
    printKeyValue("发送方式", "固定单次发送");
    std::cout << kColorItem << "  1" << kColorReset << "  SWARM_HOLD        停车/保持" << std::endl;
    std::cout << kColorItem << "  2" << kColorReset << "  SWARM_RETURN      返航" << std::endl;
    std::cout << kColorItem << "  3" << kColorReset << "  SWARM_FORMATION   编队" << std::endl;
    std::cout << kColorWarn << "  0" << kColorReset << "  EXIT              退出" << std::endl;
}

void printFormationMenu()
{
    std::cout << std::endl;
    std::cout << kColorTitle << "================ 阵型类型 ================" << kColorReset << std::endl;
    std::cout << kColorItem << "  0" << kColorReset << "  STATIC_KEEP_FORMATION" << std::endl;
    std::cout << kColorItem << "  1" << kColorReset << "  STATIC_FORMATION_LINE" << std::endl;
    std::cout << kColorItem << "  2" << kColorReset << "  STATIC_FORMATION_POLYGON" << std::endl;
    std::cout << kColorItem << "  3" << kColorReset << "  STATIC_FORMATION_RANDOM" << std::endl;
    std::cout << kColorItem << "  9" << kColorReset << "  STATIC_FORMATION_CUSTOM" << std::endl;
    std::cout << kColorItem << " 11" << kColorReset << "  DYNAMIC_FORMATION_RING" << std::endl;
    std::cout << kColorItem << " 12" << kColorReset << "  DYNAMIC_FORMATION_POLYGON" << std::endl;
    std::cout << kColorItem << " 13" << kColorReset << "  DYNAMIC_FORMATION_LEMNISCATE" << std::endl;
}

void printFormationSummary(const sunray_msgs::Formation &cmd)
{
    std::cout << std::endl;
    std::cout << kColorTitle << "================ 当前阵型参数 ================" << kColorReset << std::endl;
    printKeyValue("formation_type", formationTypeName(cmd.formation_type));
    printKeyValue("leader_pos",
                  "(" + formatValue(cmd.leader_pos.x) + ", " + formatValue(cmd.leader_pos.y) + ", " +
                      formatValue(cmd.leader_pos.z) + ")");
    printKeyValue("leader_yaw", formatValue(cmd.leader_yaw) + " rad");

    switch (cmd.formation_type)
    {
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        printKeyValue("line_spacing", formatValue(cmd.static_line_spacing));
        printKeyValue("line_angle", formatValue(cmd.static_line_angle) + " deg");
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        printKeyValue("polygon_spacing", formatValue(cmd.static_polygon_spacing));
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        printKeyValue("custom_offset_num", std::to_string(cmd.custom_offsets_pos.size()));
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        printKeyValue("dynamic_time", formatValue(cmd.dynamic_time) + " s");
        printKeyValue("ring_radius", formatValue(cmd.dynamic_ring_radius));
        printKeyValue("ring_speed", formatValue(cmd.dynamic_ring_move_speed));
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        printKeyValue("dynamic_time", formatValue(cmd.dynamic_time) + " s");
        printKeyValue("polygon_spacing", formatValue(cmd.dynamic_polygon_spacing));
        printKeyValue("polygon_speed", formatValue(cmd.dynamic_polygon_move_speed));
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        printKeyValue("dynamic_time", formatValue(cmd.dynamic_time) + " s");
        printKeyValue("lemniscate_x_radius", formatValue(cmd.dynamic_lemniscate_x_radius));
        printKeyValue("lemniscate_y_radius", formatValue(cmd.dynamic_lemniscate_y_radius));
        printKeyValue("lemniscate_speed", formatValue(cmd.dynamic_lemniscate_move_speed));
        break;
    default:
        break;
    }
}

void promptCustomOffsets(sunray_msgs::Formation &cmd)
{
    const int offset_num = promptValue<int>("自定义偏移数量", static_cast<int>(cmd.custom_offsets_pos.size()));
    if (offset_num <= 0)
    {
        cmd.custom_offsets_pos.clear();
        cmd.custom_offsets_yaw.clear();
        return;
    }

    cmd.custom_offsets_pos.clear();
    cmd.custom_offsets_yaw.clear();
    for (int i = 0; i < offset_num; ++i)
    {
        geometry_msgs::Point p;
        std::cout << kColorTitle << "agent_id=" << (i + 1) << kColorReset << std::endl;
        p.x = promptValue<double>("offset_x", 0.0);
        p.y = promptValue<double>("offset_y", 0.0);
        p.z = promptValue<double>("offset_z", 0.0);
        cmd.custom_offsets_pos.push_back(p);
        cmd.custom_offsets_yaw.push_back(promptValue<float>("offset_yaw(rad)", 0.0f));
    }
}

void editFormationParam(sunray_msgs::Formation &cmd)
{
    std::cout << kColorTitle << "---------------- 可修改参数 ----------------" << kColorReset << std::endl;
    std::cout << "  1  leader_x\n  2  leader_y\n  3  leader_z\n  4  leader_yaw\n";
    std::cout << "  5  当前阵型尺寸/时间参数\n";
    const int field = promptValue<int>("请选择要修改的参数编号", 1);

    if (field == 1)
    {
        cmd.leader_pos.x = promptValue<double>("leader_x", cmd.leader_pos.x);
        return;
    }
    if (field == 2)
    {
        cmd.leader_pos.y = promptValue<double>("leader_y", cmd.leader_pos.y);
        return;
    }
    if (field == 3)
    {
        cmd.leader_pos.z = promptValue<double>("leader_z", cmd.leader_pos.z);
        return;
    }
    if (field == 4)
    {
        cmd.leader_yaw = promptValue<float>("leader_yaw(rad)", cmd.leader_yaw);
        return;
    }
    if (field != 5)
    {
        return;
    }

    switch (cmd.formation_type)
    {
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        cmd.static_line_spacing = promptValue<float>("line_spacing", cmd.static_line_spacing);
        cmd.static_line_angle = promptValue<float>("line_angle(deg)", cmd.static_line_angle);
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        cmd.static_polygon_spacing = promptValue<float>("polygon_spacing", cmd.static_polygon_spacing);
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        promptCustomOffsets(cmd);
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        cmd.dynamic_time = promptValue<float>("dynamic_time(s)", cmd.dynamic_time);
        cmd.dynamic_ring_radius = promptValue<float>("ring_radius", cmd.dynamic_ring_radius);
        cmd.dynamic_ring_move_speed = promptValue<float>("ring_speed", cmd.dynamic_ring_move_speed);
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        cmd.dynamic_time = promptValue<float>("dynamic_time(s)", cmd.dynamic_time);
        cmd.dynamic_polygon_spacing = promptValue<float>("polygon_spacing", cmd.dynamic_polygon_spacing);
        cmd.dynamic_polygon_move_speed = promptValue<float>("polygon_speed", cmd.dynamic_polygon_move_speed);
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        cmd.dynamic_time = promptValue<float>("dynamic_time(s)", cmd.dynamic_time);
        cmd.dynamic_lemniscate_x_radius = promptValue<float>("lemniscate_x_radius", cmd.dynamic_lemniscate_x_radius);
        cmd.dynamic_lemniscate_y_radius = promptValue<float>("lemniscate_y_radius", cmd.dynamic_lemniscate_y_radius);
        cmd.dynamic_lemniscate_move_speed =
            promptValue<float>("lemniscate_speed", cmd.dynamic_lemniscate_move_speed);
        break;
    default:
        break;
    }
}

void promptFormationParams(sunray_msgs::Formation &cmd)
{
    printFormationMenu();
    cmd.formation_type = static_cast<uint8_t>(promptValue<int>("请选择阵型类型", cmd.formation_type));

    while (ros::ok())
    {
        printFormationSummary(cmd);
        const int edit = promptValue<int>("是否修改参数？0=直接发送 1=修改", 0);
        if (edit == 0)
        {
            return;
        }
        editFormationParam(cmd);
    }
}

void loadDefaults(ros::NodeHandle &nh, std::string &topic_name, TerminalSession &session)
{
    nh.param("swarm_cmd_topic", topic_name, std::string("/sunray/swarm/ugv_swarm_cmd"));
    nh.param("default_target_agent_id", session.agent_id, 99);

    int default_formation_type = static_cast<int>(sunray_msgs::Formation::STATIC_FORMATION_LINE);
    nh.param("default_formation_type", default_formation_type, default_formation_type);
    session.formation_cmd.formation_type = static_cast<uint8_t>(default_formation_type);
    nh.param("default_leader_x", session.formation_cmd.leader_pos.x, 0.0 /*米*/);
    nh.param("default_leader_y", session.formation_cmd.leader_pos.y, 0.0 /*米*/);
    nh.param("default_leader_z", session.formation_cmd.leader_pos.z, 0.0 /*米*/);
    nh.param("default_leader_yaw", session.formation_cmd.leader_yaw, 0.0f /*rad*/);
    nh.param("default_dynamic_time", session.formation_cmd.dynamic_time, 10.0f /*秒*/);
    nh.param("default_static_line_spacing", session.formation_cmd.static_line_spacing, 1.5f /*米*/);
    nh.param("default_static_line_angle", session.formation_cmd.static_line_angle, 0.0f /*deg*/);
    nh.param("default_static_polygon_spacing", session.formation_cmd.static_polygon_spacing, 2.0f /*米*/);
    nh.param("default_dynamic_ring_radius", session.formation_cmd.dynamic_ring_radius, 2.0f /*米*/);
    nh.param("default_dynamic_ring_move_speed", session.formation_cmd.dynamic_ring_move_speed, 0.4f /*米/秒*/);
    nh.param("default_dynamic_polygon_spacing", session.formation_cmd.dynamic_polygon_spacing, 2.0f /*米*/);
    nh.param("default_dynamic_polygon_move_speed", session.formation_cmd.dynamic_polygon_move_speed, 0.4f /*米/秒*/);
    nh.param("default_dynamic_lemniscate_x_radius",
             session.formation_cmd.dynamic_lemniscate_x_radius,
             3.0f /*米*/);
    nh.param("default_dynamic_lemniscate_y_radius",
             session.formation_cmd.dynamic_lemniscate_y_radius,
             2.0f /*米*/);
    nh.param("default_dynamic_lemniscate_move_speed",
             session.formation_cmd.dynamic_lemniscate_move_speed,
             0.4f /*米/秒*/);
}

void publishCommand(ros::Publisher &pub, sunray_msgs::UGVSwarmCMD &msg)
{
    msg.header.stamp = ros::Time::now();
    msg.formation_cmd.header.stamp = msg.header.stamp;
    pub.publish(msg);
    ros::spinOnce();

    SUNRAY_INFO("publish UGVSwarmCMD: agent_id={} swarm_cmd={} formation_type={}",
                static_cast<int>(msg.agent_id),
                swarmCmdName(msg.swarm_cmd),
                formationTypeName(msg.formation_cmd.formation_type));
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_swarm_cmd_pub_terminal");
    ros::NodeHandle nh("~");
    signal(SIGINT, sigintHandler);

    std::string topic_name;
    TerminalSession session;
    loadDefaults(nh, topic_name, session);

    ros::Publisher pub = nh.advertise<sunray_msgs::UGVSwarmCMD>(topic_name, 10);
    SUNRAY_INFO("ugv_swarm_cmd_pub_terminal ready: topic={} default_target_agent_id={}",
                topic_name,
                session.agent_id);

    while (ros::ok())
    {
        ros::spinOnce();
        printMainMenu(topic_name, session);

        const int menu = promptValue<int>("请选择要发布的命令", 3);
        if (menu == 0)
        {
            break;
        }

        sunray_msgs::UGVSwarmCMD msg;
        msg.cmd_source = sunray_msgs::UGVSwarmCMD::TERMINAL;
        msg.agent_id = static_cast<uint8_t>(session.agent_id);

        if (menu == 1)
        {
            msg.swarm_cmd = sunray_msgs::UGVSwarmCMD::SWARM_HOLD;
        }
        else if (menu == 2)
        {
            msg.swarm_cmd = sunray_msgs::UGVSwarmCMD::SWARM_RETURN;
        }
        else if (menu == 3)
        {
            msg.swarm_cmd = sunray_msgs::UGVSwarmCMD::SWARM_FORMATION;
            msg.formation_cmd = session.formation_cmd;
            promptFormationParams(msg.formation_cmd);
            session.formation_cmd = msg.formation_cmd;
        }
        else
        {
            std::cout << kColorError << "无效命令编号，请重新选择。" << kColorReset << std::endl;
            continue;
        }

        publishCommand(pub, msg);
    }

    return 0;
}
