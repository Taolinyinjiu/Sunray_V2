/**
 * @file takeoff_land.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> hover -> land
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Surnay无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物
 * [启动顺序]：为了保持example示例的简洁，我们并没有完整的启动流程保护措施，
 * 运行结果：
 * sunray系列无人机在当前位置起飞，达到指定高度后悬停10s，然后降落
 * [补充说明]:
 * 如需修改起飞降落的相关参数，请修改control文件夹中
 * sunray_uav_control文件夹下config文件夹中的sunray_control_config.yaml中对应参数
 *
 */

// 首先我们需要引入ros头文件，用于向ros注册节点，订阅sunray相关话题与发布控制指令
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include <ros/ros.h>
// 我们需要引入sunray相关消息，用于发布sunray无人机控制指令，以及检测无人机当前状态
#include <string>
#include <string_uav_namespace_utils.hpp>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlFSMState.h>
// 我们需要引入sunray_fsm头文件，用于获取无人机当前状态
#include <statemachine/sunray_state_types.hpp>

namespace {

sunray_msgs::UAVControlCMD make_basic_command(uint8_t control_cmd) {
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::CONTROL_CMD;
    cmd.control_cmd = control_cmd;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
    return cmd;
}

}  // namespace

// 如果系统还未准备好的等待时间
const double WAIT_FOR_READY_TIME = 10;
// 无人机准备就绪后准备起飞的等待时间
const double READY_TO_TAKEOFF_TIME = 3.0;
// 起飞成功后的悬停时间
const double HOVER_TIME = 10.0;

// 无人机当前状态
sunray_fsm::SunrayState current_state = sunray_fsm::SunrayState::OFF;

// 无人机当前状态回调函数
void Sunray_FSM_State_Callback(const sunray_msgs::UAVControlFSMState::ConstPtr& msg) {
    // 由于ros中的msg在传递时，使用的是uint8类型，而我们定义的current_state是sunray_fsm::SunrayState类型
    // 因此我们需要做一次类型转换
    // 这里的static_cast是安全的，因为sunray_fsm::SunrayState是一个枚举类型值与msg定义的值是一致的
    current_state = static_cast<sunray_fsm::SunrayState>(msg->sunray_fsm_state);
}

// 节点主函数
int main(int argc, char** argv) {
    // 向ros系统注册当前节点，节点名为surnay_uav_example_node
    ros::init(argc, argv, "surnay_uav_example_node");
    // 创建一个全局的ros节点句柄，为后续注册ros话题订阅者和发布者做准备
    ros::NodeHandle nh;
    // 使用ROS_INFO输出节点启动信息
    ROS_INFO("sunray_uav_example_node started");
    ROS_INFO("sunray_uav will takeoff, hover for 10s, then land");
    // 使用ros时间戳来检测是否到达了指定的时间
    ros::Time time_node_start = ros::Time::now();
    ros::Time time_ready_start = ros::Time(0);
    ros::Time time_hover_start = ros::Time(0);
    bool land_command_sent = false;
    // 读取ros空间中的参数，确定当前无人机的命名标识符
    std::string uav_namespace;
    std::string uav_name;
    int uav_id;
    // 如果本节点先于mavros或者无人机控制节点启动，则需要等待参数服务器中的uav_name和uav_id参数准备好
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
    ros::Rate loop_rate(20.0);
    while (ros::ok()) {
        // 调用ros::spinOnce()来处理ros订阅者的回调函数
        ros::spinOnce();
        if (current_state == sunray_fsm::SunrayState::INIT) {
            // 如果已经发送过降落指令，并重新回到了INIT状态，则说明本次任务已经完成
            if (land_command_sent) {
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
        } else if (current_state == sunray_fsm::SunrayState::HOVER) {
            // 如果当前处于HOVER状态，说明无人机起飞成功，并且进入到了悬停状态
            if (time_hover_start == ros::Time(0)) {
                time_hover_start = ros::Time::now();
            } else if ((ros::Time::now() - time_hover_start).toSec() > HOVER_TIME) {
                sunray_msgs::UAVControlCMD cmd =
                    make_basic_command(sunray_msgs::UAVControlCMD::LAND);
                cmd_pub.publish(cmd);
                land_command_sent = true;
            }
        }
        loop_rate.sleep();
    }
}
