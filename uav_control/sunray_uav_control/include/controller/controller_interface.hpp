#pragma once

#include "control_data_types/controller_desired_types.hpp"
#include "control_data_types/takeoff_failure_reason.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include <cstdint>
#include <geographic_msgs/GeoPoint.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

class Controller_Interface {
  public:
    // 析构函数
    virtual ~Controller_Interface() = default;
    
    // ------------声明周期相关----------------
    virtual bool init() = 0;  // 使用显式的 init代替构造函数
    virtual bool is_ready() = 0; // 获取当前控制器状态，控制器就绪是FSM从OFF状态切换为INIT状态的条件之一
    
    // -------------状态注入---------------
    // 将获取的里程计数据注入控制器
    virtual void set_current_odom(const control_common::UAVStateEstimate& odom) = 0;
    // 将原始外部里程计注入 PX4 EKF 转发缓存。
    // 默认空实现，避免不需要 fuse_odom 的控制器被迫维护额外状态。
    virtual void set_external_odom_for_fusion(const control_common::UAVStateEstimate& odom) {
        (void)odom;
    }
    
    // -------------模式切换---------------
    // 本函数在状态机为INIT模式时触发,要求切换px4的模式为position模式,切断setpoint流传输 
    virtual void set_position_mode() = 0;
    
    // -------------运动相关接口-------------
    // 触发起飞，参数为起飞高度和最大起飞速度
    virtual bool takeoff(double relative_takeoff_height, double max_takeoff_velocity) = 0;
    // 触发降落，参数为降落类型和最大降落速度
    virtual bool land(bool land_type, double max_land_velocity) = 0;
    // 设置悬停点，参数为当前位置
    virtual bool set_hover_point(control_common::UAVStateEstimate current_odom) = 0;
    // 将悬停点设置为最近一次 move_point 的目标（而非里程计）。
    // 适用于 POINT_COMPLETED 之后的转移，避免到达判断阶段的位置误差被锁死为悬停点。
    // 当控制器没有可用的 move_point 目标时返回 false，调用侧应回退到 set_hover_point()。
    virtual bool set_hover_point_to_last_target() {
        return false;
    }
    // 切换为悬停状态
    virtual bool hover() = 0;
    // 紧急上锁
    virtual bool emergency_kill() = 0;
    // 运动到某一点
    virtual bool move_point(controller_data_types::TargetPoint_t point) = 0;
    // 以速度控制的方式运动
    virtual bool move_velocity(controller_data_types::TargetVelocity_t velocity) = 0;
    // 控制无人机跟踪轨迹点
    virtual bool move_trajectory(controller_data_types::TargetTrajectoryPoint_t trajpoint) = 0;
    // 运动到机体系的某一点, 其中 yaw 仍按惯性系绝对角解释
    virtual bool move_point_body(controller_data_types::TargetBodyPoint_t point) = 0;
    // 以机体系速度的方式运动, 其中 yaw 仍按惯性系绝对角解释, yaw_rate 为角速度
    virtual bool move_velocity_body(controller_data_types::TargetBodyVelocity_t velocity) = 0;
    // WGS84预留接口：当前FSM运行路径不会调用，保留用于外部API兼容和未来实现。
    virtual bool move_point_wgs84(geographic_msgs::GeoPoint point) = 0;
    // ---------------------起降状态查询接口-----------------------
    // [先占个位置] 查询起飞状态
    virtual void get_takeoff_status(){};
    // [先占个位置] 查询降落状态
    virtual void get_land_status(){};

    virtual void reset_takeoff_status() {}
    virtual bool is_takeoff_complete() = 0;
    virtual bool is_takeoff_failed() const {
        return false;
    }
    virtual control_common::TakeoffFailureReason takeoff_failure_reason() const {
        return control_common::TakeoffFailureReason::None;
    }
    virtual bool is_land_complete() = 0;
    virtual bool is_point_complete() = 0;
    virtual uint8_t current_px4_landed_state() const {
        return 0;
    }
    // ---------------------底层控制输出查询接口-----------------------
    virtual bool get_last_position_target(mavros_msgs::PositionTarget& msg) const {
        (void)msg;
        return false;
    }
    virtual bool get_last_attitude_target(mavros_msgs::AttitudeTarget& msg) const {
        (void)msg;
        return false;
    }
  protected:
};
