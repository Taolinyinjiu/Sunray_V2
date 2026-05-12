/*
本程序功能：
    1.实现无人机终端控制命令的发布
    2.提供起飞、降落、返航、三个点位移动命令
    3.用于测试 sunray_msgs/UAVControlCMD 控制命令
*/

#include <ros/ros.h>
#include <sunray_msgs/UAVControlCMD.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
double degToRad(const double deg) {
    return deg * M_PI / 180.0;
}
}

static void printMenu() {
    std::cout << "\n===== UAV Terminal Control =====" << std::endl;
    std::cout << "Units: position[m], velocity[m/s], height[m], duration[s], yaw input[deg] -> publish[rad]" << std::endl;
    std::cout << "1 - TAKEOFF" << std::endl;
    std::cout << "2 - LAND" << std::endl;
    std::cout << "3 - RETURN" << std::endl;
    std::cout << "4 - MOVE_POINT 1 (local)" << std::endl;
    std::cout << "5 - MOVE_POINT 2 (body)" << std::endl;
    std::cout << "6 - MOVE_VELOCITY (local, timed)" << std::endl;
    std::cout << "7 - MOVE_VELOCITY_BODY (body, timed)" << std::endl;
    std::cout << "0 - quit" << std::endl;
    std::cout << "Please input: ";
}

static bool readPoint(const std::string& title, double& x, double& y, double& z, double& yaw_deg) {
    std::cout << title << std::endl;
    std::cout << "Enter desired position x[m] y[m] z[m] yaw[deg]: ";
    if (!(std::cin >> x >> y >> z >> yaw_deg)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid coordinate input");
        return false;
    }
    return true;
}

static bool readBodyPoint(const std::string& title, double& x, double& y, double& height, double& yaw_deg) {
    std::cout << title << std::endl;
    std::cout << "Enter desired body x[m] y[m] fixed_height[m] yaw[deg]: ";
    if (!(std::cin >> x >> y >> height >> yaw_deg)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid body point input");
        return false;
    }
    return true;
}

static bool readVelocity(const std::string& title,
                         double& vx,
                         double& vy,
                         double& vz,
                         double& yaw_deg,
                         double& duration) {
    std::cout << title << std::endl;
    std::cout << "Enter desired velocity vx[m/s] vy[m/s] vz[m/s] yaw[deg] duration[s]: ";
    if (!(std::cin >> vx >> vy >> vz >> yaw_deg >> duration) || duration <= 0.0) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid velocity input");
        return false;
    }
    return true;
}

static bool readBodyVelocity(const std::string& title,
                             double& vx,
                             double& vy,
                             double& height,
                             double& yaw_deg,
                             double& duration) {
    std::cout << title << std::endl;
    std::cout << "Enter desired body vx[m/s] vy[m/s] fixed_height[m] yaw[deg] duration[s]: ";
    if (!(std::cin >> vx >> vy >> height >> yaw_deg >> duration) || duration <= 0.0) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid body velocity input");
        return false;
    }
    return true;
}

static void publishTimedVelocityCommand(ros::Publisher& control_pub,
                                        const sunray_msgs::UAVControlCMD& cmd,
                                        double duration_s) {
    ros::Rate rate(20.0);
    const ros::Time end_time = ros::Time::now() + ros::Duration(duration_s);

    while (ros::ok() && ros::Time::now() < end_time) {
        sunray_msgs::UAVControlCMD timed_cmd = cmd;
        timed_cmd.header.stamp = ros::Time::now();
        control_pub.publish(timed_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    sunray_msgs::UAVControlCMD hover_cmd;
    hover_cmd.header.stamp = ros::Time::now();
    hover_cmd.cmd_source = sunray_msgs::UAVControlCMD::TERMINAL;
    hover_cmd.control_cmd = sunray_msgs::UAVControlCMD::HOVER;
    hover_cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    hover_cmd.desired_yaw = 0.0;
    for (int i = 0; i < 5 && ros::ok(); ++i) {
        hover_cmd.header.stamp = ros::Time::now();
        control_pub.publish(hover_cmd);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "terminal_control_node");
    ros::NodeHandle nh("~");

    int uav_id = 1;
    nh.param("uav_id", uav_id, 1);
    std::string topic = "/uav" + std::to_string(uav_id) + "/sunray/uav_control/control_cmd";

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
        cmd.fixed_height = 0.0;
        bool timed_velocity_cmd = false;
        double velocity_duration_s = 0.0;

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
            double x, y, z, yaw_deg;
            std::string title = "MOVE_POINT LOCAL " + std::to_string(input - 3);
            if (!readPoint(title, x, y, z, yaw_deg)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT;
            cmd.desired_pos.x = x;
            cmd.desired_pos.y = y;
            cmd.desired_pos.z = z;
            cmd.desired_yaw = degToRad(yaw_deg);
            cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
            ROS_INFO("Publish %s [x=%.2fm, y=%.2fm, z=%.2fm, yaw=%.2fdeg -> %.3frad]",
                     title.c_str(),
                     x,
                     y,
                     z,
                     yaw_deg,
                     cmd.desired_yaw);
            break;
        }
        case 5: {
            double x, y, z, yaw_deg;
            std::string title = "MOVE_POINT BODY" + std::to_string(input - 3);
            if (!readBodyPoint(title, x, y, z, yaw_deg)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT_BODY;
            cmd.desired_body_xy_pos.x = x;
            cmd.desired_body_xy_pos.y = y;
            cmd.fixed_height = z;
            cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
            cmd.desired_yaw = degToRad(yaw_deg);
            ROS_INFO("Publish %s [x=%.2fm, y=%.2fm, fixed_height=%.2fm, yaw=%.2fdeg -> %.3frad]",
                     title.c_str(),
                     x,
                     y,
                     z,
                     yaw_deg,
                     cmd.desired_yaw);
            break;
        }
        case 6: {
            double vx, vy, vz, yaw_deg, duration;
            const std::string title = "MOVE_VELOCITY LOCAL";
            if (!readVelocity(title, vx, vy, vz, yaw_deg, duration)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY;
            cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
            cmd.desired_yaw = degToRad(yaw_deg);
            cmd.desired_vel.x = vx;
            cmd.desired_vel.y = vy;
            cmd.desired_vel.z = vz;
            timed_velocity_cmd = true;
            velocity_duration_s = duration;
            ROS_INFO("Publish %s [vx=%.2fm/s, vy=%.2fm/s, vz=%.2fm/s, yaw=%.2fdeg -> %.3frad] for %.2fs",
                     title.c_str(),
                     vx,
                     vy,
                     vz,
                     yaw_deg,
                     cmd.desired_yaw,
                     duration);
            break;
        }
        case 7: {
            double vx, vy, height, yaw_deg, duration;
            const std::string title = "MOVE_VELOCITY BODY";
            if (!readBodyVelocity(title, vx, vy, height, yaw_deg, duration)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY;
            cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
            cmd.desired_yaw = degToRad(yaw_deg);
            cmd.desired_body_xy_vel.x = vx;
            cmd.desired_body_xy_vel.y = vy;
            cmd.fixed_height = height;
            timed_velocity_cmd = true;
            velocity_duration_s = duration;
            ROS_INFO("Publish %s [vx=%.2fm/s, vy=%.2fm/s, fixed_height=%.2fm, yaw=%.2fdeg -> %.3frad] for %.2fs",
                     title.c_str(),
                     vx,
                     vy,
                     height,
                     yaw_deg,
                     cmd.desired_yaw,
                     duration);
            break;
        }
        default:
            ROS_WARN("Unsupported command: %d", input);
            continue;
        }

        if (timed_velocity_cmd) {
            publishTimedVelocityCommand(control_pub, cmd, velocity_duration_s);
            ROS_INFO("Velocity command finished, switched to HOVER");
        } else {
            control_pub.publish(cmd);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }

    return 0;
}
