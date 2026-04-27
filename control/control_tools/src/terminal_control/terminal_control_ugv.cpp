/*
本程序功能：
    1. 实现无人车终端控制命令的发布
    2. 提供 HOLD、RETURN、点位、速度、车体系速度指令
    3. 用于测试 sunray_msgs/UGVControlCMD 控制命令
*/

#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>

#include <iostream>
#include <limits>
#include <string>

static void print_menu() {
    std::cout << "\n===== UGV Terminal Control =====" << std::endl;
    std::cout << "1 - HOLD" << std::endl;
    std::cout << "2 - RETURN" << std::endl;
    std::cout << "3 - MOVE_POINT" << std::endl;
    std::cout << "4 - MOVE_VELOCITY (ENU, timed)" << std::endl;
    std::cout << "5 - MOVE_VELOCITY_BODY (body, timed)" << std::endl;
    std::cout << "0 - quit" << std::endl;
    std::cout << "Please input: ";
}

static bool read_point(double& x, double& y, double& yaw_deg) {
    std::cout << "Enter desired x y yaw_deg: ";
    if (!(std::cin >> x >> y >> yaw_deg)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid point input");
        return false;
    }
    return true;
}

static bool read_velocity(double& vx, double& vy, double& yaw_deg, double& duration) {
    std::cout << "Enter desired vx vy yaw_deg duration(s): ";
    if (!(std::cin >> vx >> vy >> yaw_deg >> duration) || duration <= 0.0) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid velocity input");
        return false;
    }
    return true;
}

static bool read_body_velocity(double& vx, double& vy, double& wz, double& duration) {
    std::cout << "Enter desired body_vx body_vy body_wz duration(s): ";
    if (!(std::cin >> vx >> vy >> wz >> duration) || duration <= 0.0) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid body velocity input");
        return false;
    }
    return true;
}

static double deg_to_rad(const double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

static void publish_once(ros::Publisher& control_pub, sunray_msgs::UGVControlCMD cmd) {
    ros::Rate rate(20.0);
    for (int i = 0; i < 3 && ros::ok(); ++i) {
        cmd.header.stamp = ros::Time::now();
        control_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
}

static void publish_timed_command(ros::Publisher& control_pub,
                                  const sunray_msgs::UGVControlCMD& cmd,
                                  const double duration_s) {
    ros::Rate rate(20.0);
    const ros::Time end_time = ros::Time::now() + ros::Duration(duration_s);

    while (ros::ok() && ros::Time::now() < end_time) {
        sunray_msgs::UGVControlCMD timed_cmd = cmd;
        timed_cmd.header.stamp = ros::Time::now();
        control_pub.publish(timed_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    sunray_msgs::UGVControlCMD hold_cmd;
    hold_cmd.header.stamp = ros::Time::now();
    hold_cmd.cmd_source = sunray_msgs::UGVControlCMD::CMD_SOURCE_MODULE;
    hold_cmd.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
    publish_once(control_pub, hold_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "terminal_control_ugv_node");
    ros::NodeHandle nh("~");

    std::string ugv_name = "ugv";
    int ugv_id = 1;
    nh.param("ugv_name", ugv_name, ugv_name);
    nh.param("ugv_id", ugv_id, ugv_id);

    const std::string topic = "/" + ugv_name + std::to_string(ugv_id) + "/sunray/ugv_control/control_cmd";
    ros::Publisher control_pub = nh.advertise<sunray_msgs::UGVControlCMD>(topic, 1);
    ros::Duration(0.5).sleep();

    ROS_INFO("terminal_control_ugv_node started, publishing to %s", topic.c_str());

    while (ros::ok()) {
        print_menu();

        int input = -1;
        if (!(std::cin >> input)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            ROS_WARN("Invalid input, please enter a number");
            continue;
        }

        if (input == 0) {
            ROS_INFO("Exiting terminal_control_ugv_node");
            break;
        }

        sunray_msgs::UGVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_source = sunray_msgs::UGVControlCMD::CMD_SOURCE_MODULE;
        bool timed_cmd = false;
        double duration_s = 0.0;

        switch (input) {
        case 1:
            cmd.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
            ROS_INFO("Publish HOLD");
            publish_once(control_pub, cmd);
            break;

        case 2:
            cmd.control_cmd = sunray_msgs::UGVControlCMD::RETURN;
            ROS_INFO("Publish RETURN");
            publish_once(control_pub, cmd);
            break;

        case 3: {
            double x = 0.0;
            double y = 0.0;
            double yaw_deg = 0.0;
            if (!read_point(x, y, yaw_deg)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
            cmd.desired_pos.x = x;
            cmd.desired_pos.y = y;
            cmd.desired_pos.z = 0.0;
            cmd.desired_yaw = deg_to_rad(yaw_deg);
            ROS_INFO("Publish MOVE_POINT [x=%.2f y=%.2f yaw=%.2f deg]", x, y, yaw_deg);
            publish_once(control_pub, cmd);
            break;
        }

        case 4: {
            double vx = 0.0;
            double vy = 0.0;
            double yaw_deg = 0.0;
            if (!read_velocity(vx, vy, yaw_deg, duration_s)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY;
            cmd.desired_vel.x = vx;
            cmd.desired_vel.y = vy;
            cmd.desired_vel.z = 0.0;
            cmd.desired_yaw = deg_to_rad(yaw_deg);
            timed_cmd = true;
            ROS_INFO("Publish MOVE_VELOCITY [vx=%.2f vy=%.2f yaw=%.2f deg] for %.2fs",
                     vx,
                     vy,
                     yaw_deg,
                     duration_s);
            break;
        }

        case 5: {
            double vx = 0.0;
            double vy = 0.0;
            double wz = 0.0;
            if (!read_body_velocity(vx, vy, wz, duration_s)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY;
            cmd.desired_linear.x = vx;
            cmd.desired_linear.y = vy;
            cmd.desired_linear.z = 0.0;
            cmd.desired_angular.x = 0.0;
            cmd.desired_angular.y = 0.0;
            cmd.desired_angular.z = wz;
            timed_cmd = true;
            ROS_INFO("Publish MOVE_VELOCITY_BODY [vx=%.2f vy=%.2f wz=%.2f] for %.2fs",
                     vx,
                     vy,
                     wz,
                     duration_s);
            break;
        }

        default:
            ROS_WARN("Unsupported command: %d", input);
            continue;
        }

        if (timed_cmd) {
            publish_timed_command(control_pub, cmd, duration_s);
            ROS_INFO("Timed command finished, switched to HOLD");
        }
    }

    return 0;
}
