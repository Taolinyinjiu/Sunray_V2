/*
本程序功能：
    1、实现命令行集群控制工具，通过键盘输入发布 UGVSwarmCMD 指令
    2、支持 ring/line/column/v_shape/wedge 阵型切换，以及 expand/contract 阵型尺度调节
    3、支持 hold/return 和 leader 目标点输入，便于联调 UGV 集群行为
*/
#include <iostream>
#include <limits>
#include <ros/ros.h>
#include <signal.h>
#include <sunray_msgs/UGVSwarmCMD.h>

using std::cin;
using std::cout;
using std::endl;

static void SigintHandler(int)
{
    ROS_INFO("[ugv_formation_switch] exit...");
    ros::shutdown();
}

namespace
{

void publishUGVSwarmCMD(ros::Publisher &pub, uint8_t swarm_cmd,
                        uint8_t formation = sunray_msgs::UGVSwarmCMD::KEEP_FORMATION,
                        float formation_param = 0.0f, double x = 0.0, double y = 0.0,
                        double z = 0.0, double yaw = 0.0)
{
    sunray_msgs::UGVSwarmCMD msg;
    msg.header.stamp = ros::Time::now();
    msg.cmd_source = sunray_msgs::UGVSwarmCMD::EXTERNAL_MODULE;
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
    ros::init(argc, argv, "ugv_formation_switch");
    ros::NodeHandle nh("~");
    signal(SIGINT, SigintHandler);

    ros::Publisher swarm_pub = nh.advertise<sunray_msgs::UGVSwarmCMD>("/sunray/swarm/ugv_swarm_cmd", 10);

    while (ros::ok())
    {
        ros::spinOnce();
        cout << "================ UGV 集群键盘控制 ================" << endl;
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
        cout << "  101 hold      静止保持" << endl;
        cout << "  102 return    返航" << endl;
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
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UGVSwarmCMD::RING);
            break;
        case 2:
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UGVSwarmCMD::EXPAND);
            break;
        case 3:
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UGVSwarmCMD::CONTRACT);
            break;
        case 4:
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UGVSwarmCMD::LINE);
            break;
        case 5:
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UGVSwarmCMD::COLUMN);
            break;
        case 6:
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UGVSwarmCMD::V_SHAPE);
            break;
        case 7:
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UGVSwarmCMD::WEDGE);
            break;
        case 101:
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_HOLD);
            break;
        case 102:
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_RETURN);
            break;
        case 201:
        {
            cout << "输入目标点 x y z yaw (空格分隔): ";
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double yaw = 0.0;
            cin >> x >> y >> z >> yaw;
            publishUGVSwarmCMD(swarm_pub, sunray_msgs::UGVSwarmCMD::SWARM_FORMATION,
                               sunray_msgs::UGVSwarmCMD::KEEP_FORMATION, 0.0f, x, y, z, yaw);
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
