/**
 * @file move_point_by_trajectory.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> hover -> move 1 point by trajectory -> land
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Surnay无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物
 * 运行结果：
 * sunray系列无人机在当前位置起飞，达到指定高度后悬停5s，然后通过MOVE_TRAJECTORY轨迹指令
 * 在local坐标系下飞向指定位置，悬停5s后降落
 *
 * [补充说明]：
 * 这个demo用于表示如何使用五次多项式轨迹生成器，为MOVE_TRAJECTORY接口连续提供
 * 位置、速度、加速度前馈，从而完成一次定点轨迹控制
 */

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
// 我们需要引入一个轨迹生成器，用于在两个点之间生成一条轨迹
#include <utils/quintic_curve.hpp>
// 本示例example只进行一次定点飞行，因此使用宏定义在此处定义目标点
#define TARGET_X 1.0
#define TARGET_Y 0.0
#define TARGET_Z 0.6
#define TARGET_YAW 0.0

// 轨迹生成器参数
const double TRAJECTORY_MAX_VEL = 0.8;
const double POS_TOL_XY = 0.10;
const double POS_TOL_Z = 0.10;
const double VEL_TOL = 0.10;

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

sunray_msgs::UAVControlCMD make_basic_command(uint8_t control_cmd) {
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::CONTROL_CMD;
    cmd.control_cmd = control_cmd;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
    return cmd;
}

sunray_msgs::UAVControlCMD make_trajectory_command(const curve::QuinticCurveState& traj_state,
                                                   double yaw) {
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::CONTROL_CMD;
    cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    cmd.desired_pos.x = traj_state.position.x();
    cmd.desired_pos.y = traj_state.position.y();
    cmd.desired_pos.z = traj_state.position.z();
    cmd.desired_vel.x = traj_state.velocity.x();
    cmd.desired_vel.y = traj_state.velocity.y();
    cmd.desired_vel.z = traj_state.velocity.z();
    cmd.desired_acc.x = traj_state.acceleration.x();
    cmd.desired_acc.y = traj_state.acceleration.y();
    cmd.desired_acc.z = traj_state.acceleration.z();
    cmd.desired_jerk.x = 0.0;
    cmd.desired_jerk.y = 0.0;
    cmd.desired_jerk.z = 0.0;
    cmd.desired_yaw = yaw;
    cmd.desired_yaw_rate = 0.0;
    return cmd;
}

}  // namespace

// 无人机当前状态回调函数
void Sunray_FSM_State_Callback(const sunray_msgs::UAVControlFSMState::ConstPtr& msg) {
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
    ros::init(argc, argv, "move_point_by_trajectory_example_node");
    // 创建一个全局的ros节点句柄，为后续注册ros话题订阅者和发布者做准备
    ros::NodeHandle nh;
    // 使用ROS_INFO输出节点启动信息
    ROS_INFO("move_point_by_trajectory_example_node started");
    ROS_INFO("sunray_uav will takeoff, hover, move to one target point by trajectory, then land");
    // 使用ros时间戳来检测是否到达了指定的时间
    ros::Time time_node_start = ros::Time::now();
    ros::Time time_ready_start = ros::Time(0);
    ros::Time time_first_hover_start = ros::Time(0);
    ros::Time time_second_hover_start = ros::Time(0);
    const Eigen::Vector3d target_position(TARGET_X, TARGET_Y, TARGET_Z);
    const double target_yaw = TARGET_YAW;
    bool trajectory_started = false;
    bool target_reached = false;
    bool land_command_sent = false;
    curve::QuinticCurve quintic_curve;
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
    // 创建PX4状态订阅者，用于获取当前位置和速度，作为轨迹生成器的初始条件
    ros::Subscriber px4_state_sub = nh.subscribe<sunray_msgs::Px4State>(
        uav_namespace + "/sunray/px4_state", 10, Px4_State_Callback);

    // 等待PX4状态准备好，否则无法初始化轨迹起点
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
        } else if (!trajectory_started) {
            // 第一次进入HOVER，表示起飞完成，先悬停一段时间再开始轨迹控制
            if (current_state == sunray_fsm::SunrayState::HOVER) {
                if (time_first_hover_start == ros::Time(0)) {
                    time_first_hover_start = ros::Time::now();
                    ROS_INFO("Takeoff completed, hover for %.1fs before trajectory control.",
                             TAKEOFF_HOVER_TIME);
                } else if ((ros::Time::now() - time_first_hover_start).toSec() >
                           TAKEOFF_HOVER_TIME) {
                    const Eigen::Vector3d start_position(
                        current_px4_state.local_pose.position.x,
                        current_px4_state.local_pose.position.y,
                        current_px4_state.local_pose.position.z);
                    const Eigen::Vector3d start_velocity(
                        current_px4_state.local_velocity.linear.x,
                        current_px4_state.local_velocity.linear.y,
                        current_px4_state.local_velocity.linear.z);
                    quintic_curve.clear();
                    quintic_curve.set_start_trajpoint(start_position, start_velocity);
                    quintic_curve.set_end_trajpoint(target_position, Eigen::Vector3d::Zero());
                    quintic_curve.set_curve_maxvel(TRAJECTORY_MAX_VEL);
                    trajectory_started = true;
                    ROS_INFO("Start MOVE_TRAJECTORY control to target point.");
                }
            } else {
                time_first_hover_start = ros::Time(0);
            }
        } else if (!target_reached) {
            curve::QuinticCurveState traj_state = quintic_curve.get_result();
            if (traj_state.valid) {
                sunray_msgs::UAVControlCMD cmd =
                    make_trajectory_command(traj_state, target_yaw);
                cmd_pub.publish(cmd);
            }

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
            const double speed_norm = std::sqrt(current_vx * current_vx +
                                                current_vy * current_vy +
                                                current_vz * current_vz);

            if (distance_xy < POS_TOL_XY &&
                std::fabs(dz) < POS_TOL_Z &&
                speed_norm < VEL_TOL) {
                target_reached = true;
                time_second_hover_start = ros::Time(0);
                ROS_INFO("Target point reached, switching to HOVER.");
                sunray_msgs::UAVControlCMD cmd =
                    make_basic_command(sunray_msgs::UAVControlCMD::HOVER);
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
