/*
本程序功能：
    1.实现无人车终端控制命令的发布
    2.提供停止、返航、点位移动、速度控制等命令
    3.用于测试 sunray_msgs/UGVControlCMD 控制命令
*/

#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>

#include <iostream>
#include <limits>
#include <cmath>
#include <string>

namespace {
enum class DriveType {
    MECANUM = 1,
    DIFFERENTIAL = 2,
};

double degToRad(const double deg) {
    return deg * M_PI / 180.0;
}

const char* driveTypeName(const DriveType drive_type) {
    return drive_type == DriveType::MECANUM ? "mecanum" : "differential";
}
}

static void printMenu(const DriveType drive_type) {
    std::cout << "\n===== UGV Terminal Control =====" << std::endl;
    std::cout << "Drive type: " << driveTypeName(drive_type) << std::endl;
    std::cout << "Units: position[m], velocity[m/s], duration[s], yaw input[deg] -> publish[rad], angular_z input[deg/s] -> publish[rad/s]" << std::endl;
    std::cout << "1 - HOLD (停止)" << std::endl;
    std::cout << "2 - RETURN (返航)" << std::endl;
    std::cout << "3 - MOVE_POINT (点位置控制)" << std::endl;
    if (drive_type == DriveType::MECANUM) {
        std::cout << "4 - MOVE_VELOCITY (速度控制)" << std::endl;
        std::cout << "5 - MOVE_VELOCITY_BODY (机体系速度控制)" << std::endl;
    } else {
        std::cout << "4 - MOVE_VELOCITY_BODY (机体系速度控制, vx+wz)" << std::endl;
    }
    std::cout << "0 - quit" << std::endl;
    std::cout << "Please input: ";
}

static bool selectDriveType(DriveType& drive_type) {
    std::cout << "\n===== Select UGV Drive Type =====" << std::endl;
    std::cout << "1 - MECANUM (麦克纳姆轮)" << std::endl;
    std::cout << "2 - DIFFERENTIAL (差速轮)" << std::endl;
    std::cout << "Please input drive type: ";

    int input = 0;
    if (!(std::cin >> input) || (input != 1 && input != 2)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid drive type input");
        return false;
    }

    drive_type = static_cast<DriveType>(input);
    return true;
}

static bool readPoint(const std::string& title, double& x, double& y, double& yaw) {
    std::cout << title << std::endl;
    std::cout << "Enter desired position x[m] y[m] yaw[deg]: ";
    if (!(std::cin >> x >> y >> yaw)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid coordinate input");
        return false;
    }
    return true;
}

static bool readVelocity(const std::string& title, double& vx, double& vy, double& yaw, double& duration) {
    std::cout << title << std::endl;
    std::cout << "Enter desired velocity vx[m/s] vy[m/s] yaw[deg] duration[s]: ";
    if (!(std::cin >> vx >> vy >> yaw >> duration) || duration <= 0.0) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid velocity input");
        return false;
    }
    return true;
}

static bool readBodyVelocity(const std::string& title, double& vx, double& vy, double& wz, double& duration) {
    std::cout << title << std::endl;
    std::cout << "Enter desired body velocity vx[m/s] vy[m/s] angular_z[deg/s] duration[s]: ";
    if (!(std::cin >> vx >> vy >> wz >> duration) || duration <= 0.0) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid body velocity input");
        return false;
    }
    return true;
}

static bool readDifferentialBodyVelocity(const std::string& title, double& vx, double& wz, double& duration) {
    std::cout << title << std::endl;
    std::cout << "Enter desired body velocity vx[m/s] angular_z[deg/s] duration[s]: ";
    if (!(std::cin >> vx >> wz >> duration) || duration <= 0.0) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_WARN("Invalid differential body velocity input");
        return false;
    }
    return true;
}

static void publishTimedVelocityCommand(ros::Publisher& control_pub, const sunray_msgs::UGVControlCMD& cmd, double duration_s) {
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
    hold_cmd.cmd_source = 3; // 终端控制
    hold_cmd.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
    for (int i = 0; i < 5 && ros::ok(); ++i) {
        hold_cmd.header.stamp = ros::Time::now();
        control_pub.publish(hold_cmd);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ugv_terminal_control_node");
    ros::NodeHandle nh("~");

    std::string ugv_id = "ugv_1";
    nh.param("ugv_id", ugv_id, std::string("ugv_1"));
    std::string topic = "/" + ugv_id + "/sunray/ugv_control/control_cmd";

    ros::Publisher control_pub = nh.advertise<sunray_msgs::UGVControlCMD>(topic, 1);
    ros::Duration(0.5).sleep();

    ROS_INFO("ugv_terminal_control_node started, publishing to %s", topic.c_str());

    DriveType drive_type = DriveType::DIFFERENTIAL;
    while (ros::ok() && !selectDriveType(drive_type)) {
    }
    if (!ros::ok()) {
        return 0;
    }
    ROS_INFO("Selected drive type: %s", driveTypeName(drive_type));

    while (ros::ok()) {
        printMenu(drive_type);
        int input = -1;
        if (!(std::cin >> input)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            ROS_WARN("Invalid input, please enter a number");
            continue;
        }

        if (input == 0) {
            ROS_INFO("Exiting ugv_terminal_control_node");
            break;
        }

        sunray_msgs::UGVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_source = 3; // 终端控制
        bool timed_velocity_cmd = false;
        double velocity_duration_s = 0.0;

        if (input == 1) {
            cmd.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
            ROS_INFO("Publish HOLD");
        } else if (input == 2) {
            cmd.control_cmd = sunray_msgs::UGVControlCMD::RETURN;
            ROS_INFO("Publish RETURN");
        } else if (input == 3) {
            double x, y, yaw;
            std::string title = "MOVE_POINT";
            if (!readPoint(title, x, y, yaw)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
            cmd.desired_pos.x = x;
            cmd.desired_pos.y = y;
            cmd.desired_pos.z = 0.0;
            cmd.desired_yaw = degToRad(yaw);
            ROS_INFO("Publish %s [x=%.2fm, y=%.2fm, yaw=%.2fdeg -> %.3frad]",
                     title.c_str(),
                     x,
                     y,
                     yaw,
                     cmd.desired_yaw);
        } else if (drive_type == DriveType::MECANUM && input == 4) {
            double vx, vy, yaw, duration;
            const std::string title = "MOVE_VELOCITY";
            if (!readVelocity(title, vx, vy, yaw, duration)) {
                continue;
            }
            cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY;
            cmd.desired_vel.x = vx;
            cmd.desired_vel.y = vy;
            cmd.desired_vel.z = 0.0;
            cmd.desired_yaw = degToRad(yaw);
            timed_velocity_cmd = true;
            velocity_duration_s = duration;
            ROS_INFO("Publish %s [vx=%.2fm/s, vy=%.2fm/s, yaw=%.2fdeg -> %.3frad] for %.2fs",
                     title.c_str(),
                     vx,
                     vy,
                     yaw,
                     cmd.desired_yaw,
                     duration);
        } else if ((drive_type == DriveType::MECANUM && input == 5) ||
                   (drive_type == DriveType::DIFFERENTIAL && input == 4)) {
            double vx, vy, wz, duration;
            const std::string title = "MOVE_VELOCITY_BODY";
            if (drive_type == DriveType::DIFFERENTIAL) {
                if (!readDifferentialBodyVelocity(title, vx, wz, duration)) {
                    continue;
                }
                vy = 0.0;
            } else {
                if (!readBodyVelocity(title, vx, vy, wz, duration)) {
                    continue;
                }
            }
            cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY;
            cmd.desired_linear.x = vx;
            cmd.desired_linear.y = (drive_type == DriveType::DIFFERENTIAL) ? 0.0 : vy;
            cmd.desired_linear.z = 0.0;
            cmd.desired_angular.x = 0.0;
            cmd.desired_angular.y = 0.0;
            cmd.desired_angular.z = degToRad(wz);
            timed_velocity_cmd = true;
            velocity_duration_s = duration;
            if (drive_type == DriveType::DIFFERENTIAL) {
                ROS_INFO("Publish %s [vx=%.2fm/s, wz=%.2fdeg/s -> %.3frad/s] for %.2fs",
                         title.c_str(),
                         vx,
                         wz,
                         cmd.desired_angular.z,
                         duration);
            } else {
                ROS_INFO("Publish %s [vx=%.2fm/s, vy=%.2fm/s, wz=%.2fdeg/s -> %.3frad/s] for %.2fs",
                         title.c_str(),
                         vx,
                         vy,
                         wz,
                         cmd.desired_angular.z,
                         duration);
            }
        } else {
            ROS_WARN("Unsupported command: %d", input);
            continue;
        }

        if (timed_velocity_cmd) {
            publishTimedVelocityCommand(control_pub, cmd, velocity_duration_s);
            ROS_INFO("Velocity command finished, switched to HOLD");
        } else {
            control_pub.publish(cmd);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }

    return 0;
}
