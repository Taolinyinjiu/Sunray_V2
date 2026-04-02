#pragma once

#include "control_data_types/uav_state_estimate.hpp"
#include "control_data_types/controller_desired_types.hpp"

class Controller_Interface {
  public:
    virtual ~Controller_Interface() = default;
    // ------------声明周期相关----------------
    virtual bool init() = 0;  // 使用显式的 init代替构造函数
    virtual bool is_ready() = 0;
    // -------------状态注入---------------
    virtual void set_current_odom(const control_common::UAVStateEstimate& odom) = 0;
    // -------------运动相关接口-------------
    // 触发起飞，参数为起飞高度和最大起飞速度
    virtual bool takeoff(double relative_takeoff_height, double max_takeoff_velocity) = 0;
    // 触发降落，参数为降落类型和最大降落速度
    virtual bool land(bool land_type, double max_land_velocity) = 0;
    // 在当前点悬停(运动过程触发立即停止并进入悬停)
    virtual bool hover() = 0;
    // 紧急上锁
    virtual bool emergency_kill() = 0;
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
    // ---------------------起降状态查询接口-----------------------
    virtual bool is_takeoff_complete() = 0;
    virtual bool is_land_complete() = 0;
    // ---------------------控制器状态发布接口----------------------
    virtual void pub_controller_state() = 0;
    // 为了将Sunray_FSM与Mavros/PX4解耦，这里还是决定，参数由控制器自己读取yaml文件而不是函数传递

  protected:
};
