/*
本程序功能：
    1.实现无人机终端控制命令的发布
    2.提供起飞、降落、返航、三个点位移动命令
    3.用于测试 sunray_msgs/UAVControlCMD 控制命令
*/

#include <ros/ros.h>
#include <sunray_msgs/UAVControlCMD.h>

#include <iostream>
#include <limits>
#include <string>

static void printMenu() {
    std::cout << "\n===== UAV Terminal Control =====" << std::endl;
    std::cout << "1 - TAKEOFF" << std::endl;
    std::cout << "2 - LAND" << std::endl;
    std::cout << "3 - RETURN" << std::endl;
    std::cout << "4 - MOVE_POINT 1 (local)" << std::endl;
    std::cout << "5 - MOVE_POINT 2 (body)" << std::endl;
    std::cout << "6 - MOVE_POINT 3 (wgs84)" << std::endl;
    std::cout << "0 - quit" << std::endl;
    std::cout << "Please input: ";
}

static bool readPoint(const std::string& title, double& x, double& y, double& z) {
    std::cout << title << std::endl;
    std::cout << "Enter desired position x y z: ";
    if (!(std::cin >> x >> y >> z)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid coordinate input");
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "terminal_control_node");
    ros::NodeHandle nh("~");

    int uav_id = 1;
    nh.param("uav_id", uav_id, 1);
    std::string topic = "/uav" + std::to_string(uav_id) + "/sunray/uav_control_cmd";

    ros::Publisher control_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic, 1);
    ros::Duration(0.5).sleep();

    ROS_INFO("terminal_control_node started, publishing to %s", topic.c_str());

    while (ros::ok()) {
        printMenu();
        int input = -1;
        if (!(std::cin >> input)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            ROS_WARN("Invalid input, please enter a number");
            continue;
        }

        if (input == 0) {
            ROS_INFO("Exiting terminal_control_node");
            break;
        }

        sunray_msgs::UAVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_source = sunray_msgs::UAVControlCMD::TERMINAL;
        cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
        cmd.fixed_height = false;

        switch (input) {
        case 1:
            cmd.control_cmd = sunray_msgs::UAVControlCMD::TAKEOFF;
            ROS_INFO("Publish TAKEOFF");
            break;
        case 2:
            cmd.control_cmd = sunray_msgs::UAVControlCMD::LAND;
            ROS_INFO("Publish LAND");
            break;
        case 3:
            cmd.control_cmd = sunray_msgs::UAVControlCMD::RETURN;
            ROS_INFO("Publish RETURN");
            break;
        case 4: {
            double x, y, z;
            std::string title = "MOVE_POINT LOCAL " + std::to_string(input - 3);
            if (!readPoint(title, x, y, z)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT;
            cmd.desired_pos.x = x;
            cmd.desired_pos.y = y;
            cmd.desired_pos.z = z;
            ROS_INFO("Publish %s [%.2f, %.2f, %.2f]", title.c_str(), x, y, z);
            break;
        }
        case 5: {
            double x, y, z;
            std::string title = "MOVE_POINT BODY" + std::to_string(input - 3);
            if (!readPoint(title, x, y, z)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT_BODY;
            cmd.desired_pos.x = x;
            cmd.desired_pos.y = y;
            cmd.desired_pos.z = z;
            ROS_INFO("Publish %s [%.2f, %.2f, %.2f]", title.c_str(), x, y, z);
            break;
        }
        case 6:
        default:
            ROS_WARN("Unsupported command: %d", input);
            continue;
        }

        control_pub.publish(cmd);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return 0;
}
