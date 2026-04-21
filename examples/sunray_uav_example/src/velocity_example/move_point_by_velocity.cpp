/**
 * @file move_point_by_velocity.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> hover -> move 1 point by velocity -> land
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Surnay无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物
 * 运行结果：
 * sunray系列无人机在当前位置起飞，达到指定高度后悬停5s，然后通过MOVE_VELOCITY速度指令
 * 在local坐标系下飞向指定位置，悬停5s后降落
 *
 * [补充说明]：
 * 这个文件用于教学/演示如何使用一个简单的PD反馈控制器，将位置误差转换为速度指令，
 * 从而通过MOVE_VELOCITY接口完成单点位置控制。
 */

// 首先我们需要引入ros头文件，用于向ros注册节点，订阅sunray相关话题与发布控制指令
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include <Eigen/Dense>
#include <cmath>
#include <ros/ros.h>
// 我们需要引入sunray相关消息，用于发布sunray无人机控制指令，以及检测无人机当前状态
#include <string>
#include <string_uav_namespace_utils.hpp>
#include <sunray_msgs/Px4State.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlFSMState.h>
// 我们需要引入sunray_fsm头文件，用于获取无人机当前状态
#include <statemachine/sunray_state_types.hpp>

// 本示例example只进行一次定点飞行，因此使用宏定义在此处定义目标点
#define TARGET_X 1.0
#define TARGET_Y 0.0
#define TARGET_Z 0.6
#define TARGET_YAW 0.0

// 简单PD速度控制器参数
const double K_P_XY = 1.0;
const double K_D_XY = 0.4;
const double K_P_Z = 1.0;
const double K_D_Z = 0.3;
const double MAX_VEL_XY = 1.0;
const double MAX_VEL_Z = 0.6;
const double POS_TOL_XY = 0.10;
const double POS_TOL_Z = 0.10;

// 如果系统还未准备好的等待时间
const double WAIT_FOR_READY_TIME = 10;
// 无人机准备就绪后准备起飞的等待时间
const double READY_TO_TAKEOFF_TIME = 3.0;
// 起飞后的第一次悬停时间
const double TAKEOFF_HOVER_TIME = 5.0;
// 到达目标点后的第二次悬停时间
const double TARGET_HOVER_TIME = 5.0;

// 无人机当前状态
sunray_fsm::SunrayState current_state = sunray_fsm::SunrayState::OFF;
// 当前PX4状态
sunray_msgs::Px4State current_px4_state;
bool has_px4_state = false;

namespace {

double clamp_abs(double value, double limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

sunray_msgs::UAVControlCMD make_basic_command(uint8_t control_cmd) {
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::CONTROL_CMD;
    cmd.control_cmd = control_cmd;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
    return cmd;
}

sunray_msgs::UAVControlCMD make_velocity_command(const Eigen::Vector3d& desired_velocity,
                                                 double yaw) {
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::CONTROL_CMD;
    cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    cmd.desired_vel.x = desired_velocity.x();
    cmd.desired_vel.y = desired_velocity.y();
    cmd.desired_vel.z = desired_velocity.z();
    cmd.desired_yaw = yaw;
    cmd.desired_yaw_rate = 0.0;
    return cmd;
}

}  // namespace

// 无人机当前状态回调函数
void Sunray_FSM_State_Callback(const sunray_msgs::UAVControlFSMState::ConstPtr& msg) {
    // 由于ros中的msg在传递时，使用的是uint8类型，而我们定义的current_state是sunray_fsm::SunrayState类型
    // 因此我们需要做一次类型转换
    // 这里的static_cast是安全的，因为sunray_fsm::SunrayState是一个枚举类型值与msg定义的值是一致的
    current_state = static_cast<sunray_fsm::SunrayState>(msg->sunray_fsm_state);
}

// PX4状态回调函数
void Px4_State_Callback(const sunray_msgs::Px4State::ConstPtr& msg) {
    current_px4_state = *msg;
    has_px4_state = true;
}

// 节点主函数
int main(int argc, char** argv) {
    // 向ros系统注册当前节点
    ros::init(argc, argv, "move_point_by_velocity_example_node");
    // 创建一个全局的ros节点句柄，为后续注册ros话题订阅者和发布者做准备
    ros::NodeHandle nh;
    // 使用ROS_INFO输出节点启动信息
    ROS_INFO("move_point_by_velocity_example_node started");
    ROS_INFO("sunray_uav will takeoff, hover, move to one target point by velocity, then land");
    // 使用ros时间戳来检测是否到达了指定的时间
    ros::Time time_node_start = ros::Time::now();
    ros::Time time_ready_start = ros::Time(0);
    ros::Time time_first_hover_start = ros::Time(0);
    ros::Time time_second_hover_start = ros::Time(0);
    // 读取宏定义的目标点位和Yaw角
    const Eigen::Vector3d target_position(TARGET_X, TARGET_Y, TARGET_Z);
    const double target_yaw = TARGET_YAW;
    bool velocity_control_started = false;
    // 初始化检测变量
    bool target_reached = false;
    bool land_command_sent = false;
    // 在终端中打印数据
    ROS_INFO("Move target set to [%.2f, %.2f, %.2f], yaw = %.2f rad.",
             target_position.x(),
             target_position.y(),
             target_position.z(),
             target_yaw);

    // 读取ros空间中的参数，确定当前无人机的命名标识符
    std::string uav_namespace;
    std::string uav_name;
    int uav_id;
    // 如果本节点先于mavros节点或者Gazebo仿真节点启动，则需要等待参数服务器中的uav_name和uav_id参数准备好
    bool system_ready = false;
    ros::Rate wait_rate(10.0);
    while (ros::ok() && !system_ready) {
        const bool has_uav_name = nh.getParam("/uav_name", uav_name);
        const bool has_uav_id = nh.getParam("/uav_id", uav_id);
        // 如果uav_name和uav_id参数都已经准备好，则系统准备就绪
        if (has_uav_name && !uav_name.empty() && has_uav_id) {
            system_ready = true;
            uav_namespace = sunray_common::normalize_uav_ns(uav_name + std::to_string(uav_id));
            ROS_INFO("System ready: namespace = %s", uav_namespace.c_str());
            break;
        }
        // 如果等待时间超过阈值，则系统准备失败，节点结束，需要重新启动
        if ((ros::Time::now() - time_node_start).toSec() > WAIT_FOR_READY_TIME) {
            ROS_FATAL("System long time not ready.");
            ros::shutdown();
            return 1;
        }
        wait_rate.sleep();
    }

    // 创建ros话题发布者
    ros::Publisher cmd_pub =
        nh.advertise<sunray_msgs::UAVControlCMD>(uav_namespace + "/sunray/uav_control_cmd", 10);
    // 创建ros话题订阅者，用于接收无人机当前状态
    ros::Subscriber fsm_sub = nh.subscribe<sunray_msgs::UAVControlFSMState>(
        uav_namespace + "/sunray/fsm/state", 10, Sunray_FSM_State_Callback);
    // 创建PX4状态订阅者，用于获取当前位置与速度，供PD控制器反馈使用
    ros::Subscriber px4_state_sub = nh.subscribe<sunray_msgs::Px4State>(
        uav_namespace + "/sunray/px4_state", 10, Px4_State_Callback);

    // 等待PX4状态准备好，否则无法进行速度闭环控制
    // connect标识符用于描述px4飞控是否响应mavros心跳包
    ros::Time time_px4_wait_start = ros::Time::now();
    while (ros::ok() && (!has_px4_state || !current_px4_state.connected)) {
        ros::spinOnce();
        if ((ros::Time::now() - time_px4_wait_start).toSec() > WAIT_FOR_READY_TIME) {
            ROS_FATAL("PX4 state long time not ready.");
            ros::shutdown();
            return 1;
        }
        wait_rate.sleep();
    }

    ros::Rate loop_rate(20.0);
    while (ros::ok()) {
        // 调用ros::spinOnce()来处理ros订阅者的回调函数
        ros::spinOnce();
        if (current_state == sunray_fsm::SunrayState::INIT) {
            // 如果已经发送过降落指令，并重新回到了INIT状态，则说明本次任务已经完成
            if (land_command_sent) {
                ROS_INFO("Mission completed, landed at target point.");
                ros::shutdown();
                return 0;
            }
            // 当系统处于INIT状态时，设置ready时间为第一次检测到INIT状态的时间
            if (time_ready_start == ros::Time(0)) {
                time_ready_start = ros::Time::now();
            } else if ((ros::Time::now() - time_ready_start).toSec() > READY_TO_TAKEOFF_TIME) {
                sunray_msgs::UAVControlCMD cmd =
                    make_basic_command(sunray_msgs::UAVControlCMD::TAKEOFF);
                cmd_pub.publish(cmd);
            }
        } else if (!velocity_control_started) {
            // 第一次进入HOVER，表示起飞完成，先悬停一段时间再开始速度控制
            if (current_state == sunray_fsm::SunrayState::HOVER) {
                if (time_first_hover_start == ros::Time(0)) {
                    time_first_hover_start = ros::Time::now();
                    ROS_INFO("Takeoff completed, hover for %.1fs before velocity control.",
                             TAKEOFF_HOVER_TIME);
                } else if ((ros::Time::now() - time_first_hover_start).toSec() >
                           TAKEOFF_HOVER_TIME) {
                    velocity_control_started = true;
                    ROS_INFO("Start MOVE_VELOCITY PD control to target point.");
                }
            } else {
                time_first_hover_start = ros::Time(0);
            }
        } else if (!target_reached) {
            // 使用一个简单的PD控制器：速度指令 = Kp * 位置误差 - Kd * 当前速度
            const double current_x = current_px4_state.local_pose.position.x;
            const double current_y = current_px4_state.local_pose.position.y;
            const double current_z = current_px4_state.local_pose.position.z;
            const double current_vx = current_px4_state.local_velocity.linear.x;
            const double current_vy = current_px4_state.local_velocity.linear.y;
            const double current_vz = current_px4_state.local_velocity.linear.z;

            const double dx = target_position.x() - current_x;
            const double dy = target_position.y() - current_y;
            const double dz = target_position.z() - current_z;

            const double distance_xy = std::hypot(dx, dy);
            if (distance_xy < POS_TOL_XY && std::fabs(dz) < POS_TOL_Z) {
                target_reached = true;
                time_second_hover_start = ros::Time(0);
                ROS_INFO("Target point reached, switching to HOVER.");
                sunray_msgs::UAVControlCMD cmd =
                    make_basic_command(sunray_msgs::UAVControlCMD::HOVER);
                cmd_pub.publish(cmd);
            } else {
                const Eigen::Vector3d desired_velocity(
                    clamp_abs(K_P_XY * dx - K_D_XY * current_vx, MAX_VEL_XY),
                    clamp_abs(K_P_XY * dy - K_D_XY * current_vy, MAX_VEL_XY),
                    clamp_abs(K_P_Z * dz - K_D_Z * current_vz, MAX_VEL_Z));
                sunray_msgs::UAVControlCMD cmd =
                    make_velocity_command(desired_velocity, target_yaw);
                cmd_pub.publish(cmd);
            }
        } else if (!land_command_sent) {
            // 到达目标点后持续请求HOVER，等状态切到HOVER后再悬停并降落
            if (current_state != sunray_fsm::SunrayState::HOVER) {
                sunray_msgs::UAVControlCMD cmd =
                    make_basic_command(sunray_msgs::UAVControlCMD::HOVER);
                cmd_pub.publish(cmd);
            } else if (time_second_hover_start == ros::Time(0)) {
                time_second_hover_start = ros::Time::now();
                ROS_INFO("Target point reached, hover for %.1fs before landing.",
                         TARGET_HOVER_TIME);
            } else if ((ros::Time::now() - time_second_hover_start).toSec() >
                       TARGET_HOVER_TIME) {
                sunray_msgs::UAVControlCMD cmd =
                    make_basic_command(sunray_msgs::UAVControlCMD::LAND);
                cmd_pub.publish(cmd);
                land_command_sent = true;
                ROS_INFO("Target hover completed, landing.");
            }
        }
        loop_rate.sleep();
    }
}
