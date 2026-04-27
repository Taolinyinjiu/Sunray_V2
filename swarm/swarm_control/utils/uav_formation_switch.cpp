/*
本程序功能：
    1、实现命令行集群控制工具，通过键盘输入发布 UAVSwarmCMD 指令
    2、支持 ring/line/column/v_shape/wedge 阵型切换，以及 expand/contract 阵型尺度调节
    3、支持 takeoff/land/hover/return 和 leader 目标点输入，便于联调集群行为
*/
#include <iostream>
#include <limits>
#include <ros/ros.h>
#include <signal.h>
#include <sunray_msgs/UAVSwarmCMD.h>

using std::cin;
using std::cout;
using std::endl;

static void SigintHandler(int)
{
    ROS_INFO("[uav_formation_switch] exit...");
    ros::shutdown();
}

namespace
{

void publishUAVSwarmCMD(ros::Publisher &pub, uint8_t swarm_cmd,
                        uint8_t formation = sunray_msgs::UAVSwarmCMD::KEEP_FORMATION, float formation_param = 0.0f,
                        double x = 0.0, double y = 0.0, double z = 0.0, double yaw = 0.0)
{
    sunray_msgs::UAVSwarmCMD msg;
    msg.header.stamp = ros::Time::now();
    msg.cmd_source = sunray_msgs::UAVSwarmCMD::EXTERNAL_MODULE;
    msg.swarm_cmd = swarm_cmd;
    msg.formation = formation;
    msg.formation_param = formation_param;
    msg.leader_pos.x = x;
    msg.leader_pos.y = y;
    msg.leader_pos.z = z;
    msg.leader_yaw = yaw;
    pub.publish(msg);
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_formation_switch");
    ros::NodeHandle nh("~");
    signal(SIGINT, SigintHandler);

    ros::Publisher swarm_pub = nh.advertise<sunray_msgs::UAVSwarmCMD>("/sunray/swarm/uav_swarm_cmd", 10);

    while (ros::ok())
    {
        ros::spinOnce();
        cout << "================ 集群键盘控制 ================" << endl;
        cout << "阵型指令:" << endl;
        cout << "  1  ring     (圆环阵型)" << endl;
        cout << "  4  line     (一字横队)" << endl;
        cout << "  5  column   (纵队)" << endl;
        cout << "  6  v_shape  (V形编队)" << endl;
        cout << "  7  wedge    (楔形编队)" << endl;
        cout << "动作指令:" << endl;
        cout << "  2  expand   (扩散)" << endl;
        cout << "  3  contract (聚拢)" << endl;
        cout << "控制指令:" << endl;
        cout << "  101 takeoff   起飞" << endl;
        cout << "  102 land      降落" << endl;
        cout << "  103 hover     悬停" << endl;
        cout << "  105 return    返航" << endl;
        cout << "移动指令:" << endl;
        cout << "  201 move leader  (输入 leader 目标位置)" << endl;
        cout << "请输入指令: ";

        int cmd = 0;
        cin >> cmd;
        if (!cin)
        {
            cin.clear();
            cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            cout << "输入无效，请输入整数指令" << endl;
            continue;
        }

        switch (cmd)
        {
        case 1:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION, sunray_msgs::UAVSwarmCMD::RING);
            break;
        case 2:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION, sunray_msgs::UAVSwarmCMD::EXPAND);
            break;
        case 3:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UAVSwarmCMD::CONTRACT);
            break;
        case 4:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION, sunray_msgs::UAVSwarmCMD::LINE);
            break;
        case 5:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UAVSwarmCMD::COLUMN);
            break;
        case 6:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UAVSwarmCMD::V_SHAPE);
            break;
        case 7:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION, sunray_msgs::UAVSwarmCMD::WEDGE);
            break;
        case 101:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_TAKEOFF);
            break;
        case 102:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_LAND);
            break;
        case 103:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_HOVER);
            break;
        case 105:
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_RETURN);
            break;
        case 201: {
            cout << "输入目标点 x y z yaw (空格分隔): ";
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double yaw = 0.0;
            cin >> x >> y >> z >> yaw;
            publishUAVSwarmCMD(swarm_pub, sunray_msgs::UAVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UAVSwarmCMD::KEEP_FORMATION, 0.0f, x, y, z, yaw);
            break;
        }
        default:
            cout << "无效指令" << endl;
            break;
        }

        ros::Duration(0.5).sleep();
    }

    return 0;
}
