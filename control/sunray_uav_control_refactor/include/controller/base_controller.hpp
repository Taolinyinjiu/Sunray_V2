/**
 * @file base_controller.hpp
 * @author Taolinyinjiu@Yun-Drone Tech
 * @brief
 * 本文件旨在抽象出一个Sunray项目中控制器所需要遵循的基类控制器，从而将Sunray_FSM彻底解耦为一个纯粹的任务调度管理器，也就是说Sunyray_FSM对具体的控制并不关心
 * 那么我们就能够将Sunray_FSM抽象为一个对多类型无人机通用的任务调度框架
 * base_controller 不做与控制相关的实现
 * @version 0.1
 * @date 2026-03-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "control_data_types/uav_state_estimate.hpp"
#include "control_data_types/controller_desired_types.hpp"
#include "mavros_helper/mavros_helper.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/timer.h>

namespace uav_control {

class BaseController {
  public:
    explicit BaseController(ros::NodeHandle nh)
        : nh_(nh), config_list(true), mavros_helper_(nh_, config_list) {
    }  // 显式的构造函数，子类继承需显式调用
    virtual ~BaseController() = default;
    // 控制器初始化,构造话题发布者发布控制器状态
    virtual bool init() = 0;
    // 检查控制器是否就绪
    virtual bool is_ready() = 0;
    // 传递里程计信息
    virtual void set_current_odom(const control_common::UAVStateEstimate& odom);
    // 调用这个函数，就会使能vision_pose接口，所以直接传递频率
    virtual void set_fuse_frequency(double fuse_odom_frequency);
    // ---------------------运动相关接口-----------------------
    // 触发起飞，参数为起飞高度和最大起飞速度
    virtual bool takeoff(double relative_takeoff_height, double max_takeoff_velocity) = 0;
    // 触发降落，参数为降落类型和最大降落速度
    virtual bool land(bool land_type, double max_land_velocity) = 0;
    // 在当前点悬停(运动过程触发立即停止并进入悬停)
    virtual bool hover() = 0;
    // 运动到某一点
    virtual bool move_point(controller_data_types::TargetPoint_t point) = 0;
    // 以速度控制的方式运动
    virtual bool move_velocity(controller_data_types::TargetVelocity_t velocity) = 0;
    // 控制无人机跟踪轨迹点
    virtual bool move_trajectory(controller_data_types::TargetTrajectoryPoint_t trajpoint) = 0;
    // 运动到机体系的某一点
    virtual bool move_point_body(controller_data_types::TargetPoint_t point) = 0;
    // 以机体系速度的方式运动
    virtual bool move_velocity_body(controller_data_types::TargetVelocity_t velocity) = 0;
    // 移动到WGS84下的某一点
    virtual bool move_point_wgs84(controller_data_types::TargetPoint_t point) = 0;
    // ---------------------运动状态查询接口-----------------------
    virtual bool is_takeoff_complete() = 0;
    virtual bool is_land_complete() = 0;

  protected:
    // ---------------------定时器回调函数---------------------
    void publish_timer_cb(const ros::TimerEvent&);
    void vision_fuse_timer_cb(const ros::TimerEvent&);
    // -------------------ros句柄与辅助工具-----------------
    ros::NodeHandle nh_;  // 缓存ros句柄，但是实际上没有什么用
    MavrosHelper_ConfigList config_list;
    MavrosHelper mavros_helper_;
    // --------------------px4状态-----------------------
    control_common::FlightMode px4_mode_ = control_common::FlightMode::Undefined;
    bool px4_arm_ = false;
    double fuse_odom_frequency_ = 0.0;  // 是否要使用vision_pose接口融合里程计
    // --------------------里程计状态---------------------
    control_common::UAVStateEstimate uav_odometry_;
    geometry_msgs::PoseStamped vision_pose_;
    // --------------------期望状态--------------------
    controller_data_types::TargetTrajectoryPoint_t desired_state_;
    // ros话题发布者
    ros::Publisher controller_state_pub_;  // 发布controller状态话题数据
    ros::Publisher px4_state_pub_;         // 发布px4_state话题数据
    // ros定时器
    ros::Timer pub_timer_;          // 用于发布话题数据的定时器
    ros::Timer vision_pose_timer_;  // 用于发布vision_pose的定时器
};

}  // namespace uav_control
