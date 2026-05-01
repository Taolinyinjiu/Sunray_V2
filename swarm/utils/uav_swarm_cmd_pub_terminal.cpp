/*
本程序功能：
    1、通过终端交互发布 UAVSwarmCMD，便于联调无人机集群控制模块
    2、覆盖起飞/降落/悬停/返航/编队等全部集群指令
    3、当发布编队指令时，先显示当前参数，再按需修改某一项
    4、默认广播到全部无人机（agent_id=99），默认单次发送
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
#include <sunray_msgs/UAVSwarmCMD.h>
#include <vector>

using std::cin;
using std::cout;
using std::endl;

static void SigintHandler(int)
{
    SUNRAY_INFO("uav_swarm_cmd_pub_terminal exit...");
    ros::shutdown();
}

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
    int agent_id = 99;
    sunray_msgs::Formation formation_cmd{};
};

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

std::string formatValue(const double value)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << value;
    return oss.str();
}

std::string formatValue(const float value)
{
    return formatValue(static_cast<double>(value));
}

template <typename T>
std::string formatValue(const T &value)
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

template <typename T>
bool parseValue(const std::string &text, T &value)
{
    std::istringstream iss(text);
    iss >> value;
    return iss && iss.eof();
}

template <>
bool parseValue<std::string>(const std::string &text, std::string &value)
{
    value = text;
    return true;
}

template <typename T>
T promptValue(const std::string &label, const T &default_value)
{
    while (ros::ok())
    {
        cout << kColorInfo << label << " [" << formatValue(default_value) << "] : " << kColorReset << std::flush;

        std::string line;
        if (!std::getline(cin, line))
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

        cout << kColorError << "输入无效，请重新输入。" << kColorReset << endl;
    }

    return default_value;
}

void printKeyValue(const std::string &key, const std::string &value)
{
    cout << "  " << std::left << std::setw(20) << key << " : " << value << endl;
}

const char *swarmCmdName(const uint8_t swarm_cmd)
{
    switch (swarm_cmd)
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
        return "UNKNOWN";
    }
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

void printMainMenu(const std::string &topic_name, const TerminalSession &session)
{
    cout << endl;
    cout << kColorTitle << "================ UAV 集群指令终端 ================" << kColorReset << endl;
    printKeyValue("发布话题", topic_name);
    printKeyValue("默认目标ID", formatValue(session.agent_id));
    printKeyValue("发送方式", "固定单次发送");
    cout << kColorItem << "  1" << kColorReset << "  SWARM_TAKEOFF      起飞" << endl;
    cout << kColorItem << "  2" << kColorReset << "  SWARM_LAND         降落" << endl;
    cout << kColorItem << "  3" << kColorReset << "  SWARM_HOVER        悬停" << endl;
    cout << kColorItem << "  4" << kColorReset << "  SWARM_RETURN       返航" << endl;
    cout << kColorItem << "  5" << kColorReset << "  SWARM_FORMATION    编队" << endl;
    cout << kColorWarn << "  0" << kColorReset << "  EXIT               退出" << endl;
}

void printFormationMenu()
{
    cout << endl;
    cout << kColorTitle << "================ 阵型类型 ================" << kColorReset << endl;
    cout << kColorItem << "  0" << kColorReset << "  STATIC_KEEP_FORMATION    保持当前相对队形" << endl;
    cout << kColorItem << "  1" << kColorReset << "  STATIC_FORMATION_LINE    静态直线阵" << endl;
    cout << kColorItem << "  2" << kColorReset << "  STATIC_FORMATION_POLYGON 静态正多边形" << endl;
    cout << kColorItem << "  3" << kColorReset << "  STATIC_FORMATION_RANDOM  静态随机阵" << endl;
    cout << kColorItem << "  9" << kColorReset << "  STATIC_FORMATION_CUSTOM   静态自定义" << endl;
    cout << kColorItem << " 11" << kColorReset << "  DYNAMIC_FORMATION_RING    动态圆环" << endl;
    cout << kColorItem << " 12" << kColorReset << "  DYNAMIC_FORMATION_POLYGON 动态正多边形边运动" << endl;
    cout << kColorItem << " 13" << kColorReset << "  DYNAMIC_FORMATION_LEMNISCATE 动态 8 字轨迹" << endl;
}

void promptStaticCustomOffsets(sunray_msgs::Formation &formation_cmd)
{
    const int default_offset_num = static_cast<int>(formation_cmd.custom_offsets_pos.size());
    const int offset_num = promptValue<int>("自定义偏移数量", default_offset_num > 0 ? default_offset_num : 3);

    if (offset_num <= 0)
    {
        cout << kColorWarn << "偏移数量无效，保持空数组。" << kColorReset << endl;
        formation_cmd.custom_offsets_pos.clear();
        formation_cmd.custom_offsets_yaw.clear();
        return;
    }

    std::vector<geometry_msgs::Point> old_offsets_pos = formation_cmd.custom_offsets_pos;
    std::vector<float> old_offsets_yaw = formation_cmd.custom_offsets_yaw;

    formation_cmd.custom_offsets_pos.clear();
    formation_cmd.custom_offsets_yaw.clear();
    formation_cmd.custom_offsets_pos.reserve(static_cast<size_t>(offset_num));
    formation_cmd.custom_offsets_yaw.reserve(static_cast<size_t>(offset_num));

    for (int i = 0; i < offset_num; ++i)
    {
        geometry_msgs::Point point;
        const geometry_msgs::Point default_point =
            i < static_cast<int>(old_offsets_pos.size()) ? old_offsets_pos[static_cast<size_t>(i)] : geometry_msgs::Point{};
        const float default_yaw =
            i < static_cast<int>(old_offsets_yaw.size()) ? old_offsets_yaw[static_cast<size_t>(i)] : 0.0f;

        cout << endl;
        cout << kColorTitle << "---------------- 自定义偏移 agent_id=" << (i + 1)
             << " ----------------" << kColorReset << endl;
        point.x = promptValue<double>("offset_x", default_point.x);
        point.y = promptValue<double>("offset_y", default_point.y);
        point.z = promptValue<double>("offset_z", default_point.z);
        const float offset_yaw = promptValue<float>("offset_yaw(rad)", default_yaw);

        formation_cmd.custom_offsets_pos.push_back(point);
        formation_cmd.custom_offsets_yaw.push_back(offset_yaw);
    }
}

void printFormationParamSummary(const sunray_msgs::Formation &formation_cmd)
{
    cout << endl;
    cout << kColorTitle << "================ 当前阵型参数 ================" << kColorReset << endl;
    printKeyValue("formation_type", formationTypeName(formation_cmd.formation_type));
    printKeyValue("leader_pos",
                  "(" + formatValue(formation_cmd.leader_pos.x) + ", " +
                      formatValue(formation_cmd.leader_pos.y) + ", " +
                      formatValue(formation_cmd.leader_pos.z) + ")");
    printKeyValue("leader_yaw", formatValue(static_cast<double>(formation_cmd.leader_yaw)) + " rad");

    switch (formation_cmd.formation_type)
    {
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        printKeyValue("line_spacing", formatValue(static_cast<double>(formation_cmd.static_line_spacing)));
        printKeyValue("line_angle", formatValue(static_cast<double>(formation_cmd.static_line_angle)) + " deg");
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        printKeyValue("polygon_spacing", formatValue(static_cast<double>(formation_cmd.static_polygon_spacing)));
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_RANDOM:
        printKeyValue("random_rule", "safe_field + pair_distance");
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        printKeyValue("custom_offset_num", formatValue(static_cast<int>(formation_cmd.custom_offsets_pos.size())));
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        printKeyValue("dynamic_time", formatValue(static_cast<double>(formation_cmd.dynamic_time)) + " s");
        printKeyValue("ring_radius", formatValue(static_cast<double>(formation_cmd.dynamic_ring_radius)));
        printKeyValue("ring_speed", formatValue(static_cast<double>(formation_cmd.dynamic_ring_move_speed)));
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        printKeyValue("dynamic_time", formatValue(static_cast<double>(formation_cmd.dynamic_time)) + " s");
        printKeyValue("polygon_spacing", formatValue(static_cast<double>(formation_cmd.dynamic_polygon_spacing)));
        printKeyValue("polygon_speed", formatValue(static_cast<double>(formation_cmd.dynamic_polygon_move_speed)));
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        printKeyValue("dynamic_time", formatValue(static_cast<double>(formation_cmd.dynamic_time)) + " s");
        printKeyValue("lemniscate_x_radius",
                      formatValue(static_cast<double>(formation_cmd.dynamic_lemniscate_x_radius)));
        printKeyValue("lemniscate_y_radius",
                      formatValue(static_cast<double>(formation_cmd.dynamic_lemniscate_y_radius)));
        printKeyValue("lemniscate_speed",
                      formatValue(static_cast<double>(formation_cmd.dynamic_lemniscate_move_speed)));
        break;
    default:
        break;
    }
}

void printFormationEditMenu(const sunray_msgs::Formation &formation_cmd)
{
    cout << endl;
    cout << kColorTitle << "---------------- 可修改参数 ----------------" << kColorReset << endl;
    cout << kColorItem << "  1" << kColorReset << "  虚拟领机 x" << endl;
    cout << kColorItem << "  2" << kColorReset << "  虚拟领机 y" << endl;
    cout << kColorItem << "  3" << kColorReset << "  虚拟领机 z" << endl;
    cout << kColorItem << "  4" << kColorReset << "  虚拟领机 yaw" << endl;

    switch (formation_cmd.formation_type)
    {
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        cout << kColorItem << "  5" << kColorReset << "  line_spacing" << endl;
        cout << kColorItem << "  6" << kColorReset << "  line_angle" << endl;
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        cout << kColorItem << "  5" << kColorReset << "  polygon_spacing" << endl;
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        cout << kColorItem << "  5" << kColorReset << "  custom_offsets" << endl;
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        cout << kColorItem << "  5" << kColorReset << "  dynamic_time" << endl;
        cout << kColorItem << "  6" << kColorReset << "  ring_radius" << endl;
        cout << kColorItem << "  7" << kColorReset << "  ring_speed" << endl;
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        cout << kColorItem << "  5" << kColorReset << "  dynamic_time" << endl;
        cout << kColorItem << "  6" << kColorReset << "  polygon_spacing" << endl;
        cout << kColorItem << "  7" << kColorReset << "  polygon_speed" << endl;
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        cout << kColorItem << "  5" << kColorReset << "  dynamic_time" << endl;
        cout << kColorItem << "  6" << kColorReset << "  lemniscate_x_radius" << endl;
        cout << kColorItem << "  7" << kColorReset << "  lemniscate_y_radius" << endl;
        cout << kColorItem << "  8" << kColorReset << "  lemniscate_speed" << endl;
        break;
    default:
        break;
    }
}

void editFormationParam(sunray_msgs::Formation &formation_cmd)
{
    printFormationEditMenu(formation_cmd);
    const int field = promptValue<int>("请选择要修改的参数编号", 1);

    switch (field)
    {
    case 1:
        formation_cmd.leader_pos.x = promptValue<double>("虚拟领机 x", formation_cmd.leader_pos.x);
        return;
    case 2:
        formation_cmd.leader_pos.y = promptValue<double>("虚拟领机 y", formation_cmd.leader_pos.y);
        return;
    case 3:
        formation_cmd.leader_pos.z = promptValue<double>("虚拟领机 z", formation_cmd.leader_pos.z);
        return;
    case 4:
        formation_cmd.leader_yaw = promptValue<float>("虚拟领机 yaw(rad)", formation_cmd.leader_yaw);
        return;
    default:
        break;
    }

    switch (formation_cmd.formation_type)
    {
    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        if (field == 5)
        {
            formation_cmd.static_line_spacing =
                promptValue<float>("line_spacing", formation_cmd.static_line_spacing);
            return;
        }
        if (field == 6)
        {
            formation_cmd.static_line_angle =
                promptValue<float>("line_angle(deg)", formation_cmd.static_line_angle);
            return;
        }
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        if (field == 5)
        {
            formation_cmd.static_polygon_spacing =
                promptValue<float>("polygon_spacing", formation_cmd.static_polygon_spacing);
            return;
        }
        break;
    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        if (field == 5)
        {
            promptStaticCustomOffsets(formation_cmd);
            return;
        }
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        if (field == 5)
        {
            formation_cmd.dynamic_time = promptValue<float>("dynamic_time(s)", formation_cmd.dynamic_time);
            return;
        }
        if (field == 6)
        {
            formation_cmd.dynamic_ring_radius =
                promptValue<float>("ring_radius", formation_cmd.dynamic_ring_radius);
            return;
        }
        if (field == 7)
        {
            formation_cmd.dynamic_ring_move_speed =
                promptValue<float>("ring_speed", formation_cmd.dynamic_ring_move_speed);
            return;
        }
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        if (field == 5)
        {
            formation_cmd.dynamic_time = promptValue<float>("dynamic_time(s)", formation_cmd.dynamic_time);
            return;
        }
        if (field == 6)
        {
            formation_cmd.dynamic_polygon_spacing =
                promptValue<float>("polygon_spacing", formation_cmd.dynamic_polygon_spacing);
            return;
        }
        if (field == 7)
        {
            formation_cmd.dynamic_polygon_move_speed =
                promptValue<float>("polygon_speed", formation_cmd.dynamic_polygon_move_speed);
            return;
        }
        break;
    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        if (field == 5)
        {
            formation_cmd.dynamic_time = promptValue<float>("dynamic_time(s)", formation_cmd.dynamic_time);
            return;
        }
        if (field == 6)
        {
            formation_cmd.dynamic_lemniscate_x_radius =
                promptValue<float>("lemniscate_x_radius", formation_cmd.dynamic_lemniscate_x_radius);
            return;
        }
        if (field == 7)
        {
            formation_cmd.dynamic_lemniscate_y_radius =
                promptValue<float>("lemniscate_y_radius", formation_cmd.dynamic_lemniscate_y_radius);
            return;
        }
        if (field == 8)
        {
            formation_cmd.dynamic_lemniscate_move_speed =
                promptValue<float>("lemniscate_speed", formation_cmd.dynamic_lemniscate_move_speed);
            return;
        }
        break;
    default:
        break;
    }

    cout << kColorError << "参数编号无效，请重新选择。" << kColorReset << endl;
}

void promptFormationParams(sunray_msgs::Formation &formation_cmd)
{
    while (ros::ok())
    {
        printFormationMenu();
        formation_cmd.formation_type =
            static_cast<uint8_t>(promptValue<int>("formation_type", static_cast<int>(formation_cmd.formation_type)));

        while (ros::ok())
        {
            printFormationParamSummary(formation_cmd);
            cout << kColorWarn << "回车默认直接发送；如需修改，只改你关心的那一项。" << kColorReset << endl;
            const int action = promptValue<int>("0直接发送  1修改参数  2重选阵型", 0);

            if (action == 0)
            {
                return;
            }

            if (action == 1)
            {
                editFormationParam(formation_cmd);
                continue;
            }

            if (action == 2)
            {
                break;
            }

            cout << kColorError << "操作编号无效，请重新选择。" << kColorReset << endl;
        }
    }
}

void printCommandSummary(const sunray_msgs::UAVSwarmCMD &msg)
{
    cout << endl;
    cout << kColorTitle << "================ 待发送指令摘要 ================" << kColorReset << endl;
    printKeyValue("agent_id", formatValue(static_cast<int>(msg.agent_id)));
    printKeyValue("swarm_cmd", swarmCmdName(msg.swarm_cmd));
    printKeyValue("cmd_source", "TERMINAL");

    if (msg.swarm_cmd == sunray_msgs::UAVSwarmCMD::SWARM_FORMATION)
    {
        printFormationParamSummary(msg.formation_cmd);
    }
}

void loadDefaults(ros::NodeHandle &nh, std::string &swarm_cmd_topic, TerminalSession &session)
{
    nh.param<std::string>("swarm_cmd_topic", swarm_cmd_topic, "/sunray/swarm/uav_swarm_cmd");
    nh.param("default_agent_id", session.agent_id, 99);

    int default_formation_type = static_cast<int>(sunray_msgs::Formation::STATIC_FORMATION_LINE);
    nh.param("default_formation_type", default_formation_type, default_formation_type);
    session.formation_cmd.formation_type = static_cast<uint8_t>(default_formation_type);
    nh.param("default_leader_x", session.formation_cmd.leader_pos.x, 0.0 /*米*/);
    nh.param("default_leader_y", session.formation_cmd.leader_pos.y, 0.0 /*米*/);
    nh.param("default_leader_z", session.formation_cmd.leader_pos.z, 1.5 /*米*/);
    nh.param("default_leader_yaw", session.formation_cmd.leader_yaw, 0.0f /*rad*/);
    nh.param("default_dynamic_time", session.formation_cmd.dynamic_time, 10.0f /*秒*/);

    nh.param("default_static_line_spacing", session.formation_cmd.static_line_spacing, 1.5f /*米*/);
    nh.param("default_static_line_angle", session.formation_cmd.static_line_angle, 0.0f /*deg*/);
    nh.param("default_static_polygon_spacing", session.formation_cmd.static_polygon_spacing, 2.0f /*米*/);

    nh.param("default_dynamic_ring_radius", session.formation_cmd.dynamic_ring_radius, 2.0f /*米*/);
    nh.param("default_dynamic_ring_move_speed", session.formation_cmd.dynamic_ring_move_speed, 0.5f /*米/秒*/);
    nh.param("default_dynamic_polygon_spacing", session.formation_cmd.dynamic_polygon_spacing, 2.0f /*米*/);
    nh.param("default_dynamic_polygon_move_speed", session.formation_cmd.dynamic_polygon_move_speed, 0.5f /*米/秒*/);
    nh.param("default_dynamic_lemniscate_x_radius",
             session.formation_cmd.dynamic_lemniscate_x_radius,
             3.0f /*米*/);
    nh.param("default_dynamic_lemniscate_y_radius",
             session.formation_cmd.dynamic_lemniscate_y_radius,
             2.0f /*米*/);
    nh.param("default_dynamic_lemniscate_move_speed",
             session.formation_cmd.dynamic_lemniscate_move_speed,
             0.5f /*米/秒*/);
}

void publishCommand(ros::Publisher &swarm_pub, sunray_msgs::UAVSwarmCMD &msg)
{
    msg.header.stamp = ros::Time::now();
    msg.formation_cmd.header.stamp = msg.header.stamp;
    swarm_pub.publish(msg);
    ros::spinOnce();

    SUNRAY_INFO("publish UAVSwarmCMD: agent_id={} swarm_cmd={} formation_type={}",
                static_cast<int>(msg.agent_id),
                swarmCmdName(msg.swarm_cmd),
                formationTypeName(msg.formation_cmd.formation_type));
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_swarm_cmd_pub_terminal");
    ros::NodeHandle nh("~");
    signal(SIGINT, SigintHandler);

    std::string swarm_cmd_topic;
    TerminalSession session;
    loadDefaults(nh, swarm_cmd_topic, session);

    ros::Publisher swarm_pub = nh.advertise<sunray_msgs::UAVSwarmCMD>(swarm_cmd_topic, 10);

    SUNRAY_INFO("uav_swarm_cmd_pub_terminal ready: topic={} default_agent_id={} send_mode=single",
                swarm_cmd_topic,
                session.agent_id);

    while (ros::ok())
    {
        ros::spinOnce();
        printMainMenu(swarm_cmd_topic, session);

        const int menu = promptValue<int>("请选择要发布的命令", 5);
        if (menu == 0)
        {
            break;
        }

        sunray_msgs::UAVSwarmCMD msg;
        msg.cmd_source = sunray_msgs::UAVSwarmCMD::TERMINAL;
        msg.agent_id = static_cast<uint8_t>(session.agent_id);

        switch (menu)
        {
        case 1:
            msg.swarm_cmd = sunray_msgs::UAVSwarmCMD::SWARM_TAKEOFF;
            break;

        case 2:
            msg.swarm_cmd = sunray_msgs::UAVSwarmCMD::SWARM_LAND;
            break;

        case 3:
            msg.swarm_cmd = sunray_msgs::UAVSwarmCMD::SWARM_HOVER;
            break;

        case 4:
            msg.swarm_cmd = sunray_msgs::UAVSwarmCMD::SWARM_RETURN;
            break;

        case 5:
            msg.swarm_cmd = sunray_msgs::UAVSwarmCMD::SWARM_FORMATION;
            msg.formation_cmd = session.formation_cmd;
            promptFormationParams(msg.formation_cmd);
            session.formation_cmd = msg.formation_cmd;
            break;

        default:
            cout << kColorError << "无效命令编号，请重新选择。" << kColorReset << endl;
            continue;
        }

        printCommandSummary(msg);
        publishCommand(swarm_pub, msg);
    }

    return 0;
}
