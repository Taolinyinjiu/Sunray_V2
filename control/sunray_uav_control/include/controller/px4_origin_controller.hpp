/**
 * @file px4_origin_controller.hpp
 * @brief 基于px4原生控制环的控制器，从Controller_Interface派生
 * @details
    1. 引入Mavros_Helper类用于读取px4数据
    2. 向Sunray_FSM提供规范的函数实现
    3. 向Sunray_FSM提供规范的运动控制与状态查询接口
    4. 修改构造函数为仅存储ros句柄，显示调用init()函数进行初始化
    5. 初始化检查使用void类型，如果检查失败会通过抛出异常的方式终止程序运行
       运行时检查函数使用bool类型，检查失败通过返回false警告
    6. TODO: [ ] 实现public中需要的函数
             [-] 实现pub_timer_的初始化
             [ ] 实现px4_mode和px4_arm的同步(从mavros_helper中读取数据)
             [ ] desired_state_初始化
             [ ] 考虑是否为不同的定位源引入不同的参数配置？比如EV_Delay之类？
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-30
 * @version 0.2
 *
 *
 */

#pragma once
#include "controller/controller_interface.hpp"
#include "mavros_helper/mavros_helper.hpp"
#include "utils/arrival_helper.hpp"
#include "utils/reference_limit_helper.hpp"
#include <ros/node_handle.h>
#include <string>
#include <atomic>
#include <mutex>
#include "utils/quintic_curve.hpp"

class PX4_OriginController : public Controller_Interface {
  public:
    PX4_OriginController(ros::NodeHandle& nh);
    ~PX4_OriginController() override = default;
    // ------------声明周期相关----------------
    bool init() override;  // init()读取config.yaml文件中的相关配置
    bool is_ready() override;  // is_ready()检查各项指标是否正常，比如里程计频率，融合参数等等
    // -------------状态注入---------------
    void set_current_odom(const control_common::UAVStateEstimate& odom) override;
    // -------------运动相关接口------------
    // 在地面，保持setpoint流，该函数用于状态机INIT阶段保持setpoint流持续发布，设置为-100的z轴推力
    void set_position_mode() override;
    // 触发起飞，参数为起飞高度和最大起飞速度
    bool takeoff(double relative_takeoff_height, double max_takeoff_velocity) override;
    // 触发降落，参数为降落类型和最大降落速度
    bool land(bool land_type, double max_land_velocity) override;
    // 设置悬停点
    bool set_hover_point(control_common::UAVStateEstimate current_odom) override;
    // 将悬停点设置为最近一次 move_point 的目标点（避免被到达判断阶段的位置误差污染）
    bool set_hover_point_to_last_target() override;
    // 进入悬停状态
    bool hover() override;
    // 紧急上锁
    bool emergency_kill() override;
    // 运动到某一点
    bool move_point(controller_data_types::TargetPoint_t point) override;
    // 以速度控制的方式运动
    bool move_velocity(controller_data_types::TargetVelocity_t velocity) override;
    // 控制无人机跟踪轨迹点
    bool move_trajectory(controller_data_types::TargetTrajectoryPoint_t trajpoint) override;
    // 运动到机体系的某一点
    bool move_point_body(controller_data_types::TargetBodyPoint_t point) override;
    // 以机体系速度的方式运动
    bool move_velocity_body(controller_data_types::TargetBodyVelocity_t velocity) override;
    // 移动到WGS84下的某一点
    bool move_point_wgs84(geographic_msgs::GeoPoint point) override;
    // ---------------------起降状态查询接口-----------------------
    bool is_takeoff_complete() override;
    bool is_land_complete() override;
    bool is_point_complete() override;
    bool get_last_position_target(mavros_msgs::PositionTarget& msg) const override;
  private:
    enum class MotionCurveOwner {
        None = 0,
        MovePoint,
        Takeoff,
        Land,
    };

    // ----------------------配置相关-----------------------
    int fuse_odom_type_{0};
    double fuse_odom_frequency_{0.0};
    // ----------------------检查相关-----------------------
    // void 类型 + throw后缀 = 整个启动过程只会执行一次，抛出异常则终止启动
    // bool 类型 表示运行过程中检查，会反复调用maybe
    void load_and_validate_config_or_throw();  // 读取并校验yaml文件配置的参数

    bool check_px4_basic_state();      // 检查px4飞控的状态
    bool check_mavros_stream_ready();  // 检查mavros_helper是否在稳定的更新数据
    bool move_point_impl(controller_data_types::TargetPoint_t point,
                         bool preserve_body_point_context);
    void clear_motion_curve();
    void begin_motion_curve(MotionCurveOwner owner);
    void reset_point_motion_context();
    void clear_cached_setpoint();
    controller_data_types::TargetTrajectoryPoint_t make_hold_desired_state(
        const control_common::UAVStateEstimate& odom) const;
    void publish_hold_setpoint(const Eigen::Vector3d& position, double yaw);
    void reset_after_landing_success();
    double update_limited_yaw_target(double target_yaw, const ros::Time& now);
    void warn_if_trajectory_exceeds_limits(
        const controller_data_types::TargetTrajectoryPoint_t& trajpoint) const;
    void cache_local_setpoint(const control_common::Mavros_SetpointLocal& setpoint);
    std::atomic<bool> controller_ready_{false};
    // ---------------------定时器回调函数---------------------
    void pub_px4_state_timer_cb(const ros::TimerEvent&);    // 发布PX4State话题
    void pub_vision_fuse_timer_cb(const ros::TimerEvent&);  // 发布vision_fuse
    // -------------------ros句柄与mavros辅助类-----------------
    ros::NodeHandle nh_;
    MavrosHelper mavros_helper_;
    // ---------------------共享曲线辅助类-----------------
    curve::QuinticCurve motion_curve_;
    MotionCurveOwner motion_curve_owner_{MotionCurveOwner::None};
    // --------------------px4状态-----------------------
    control_common::FlightMode px4_mode_ = control_common::FlightMode::Undefined;
    control_common::LandedState px4_land_ = control_common::LandedState::Undefined;
    bool px4_arm_ = false;
    double pub_px4_state_freq_ = 100.0;  // 考虑简单点，在这里硬编码px4state的发布频率
    // 初始时刻的yaw角(弧度 rad)，用于起飞锁定
    double takeoff_yaw_ = 0.0;
    double land_yaw_ = 0.0;
    // 地面高度
    double takeoff_ground_height = 0.0;
    // 开始切换offboard的触发时间
    ros::Time start_checkout_offboard_time_{ros::Time(0)};
    ros::Time last_checkout_offboard_time_{ros::Time(0)};
    // arm解锁成功的触发时间
    ros::Time last_arm_time_{ros::Time(0)};
    // 到达稳定判定参数：takeoff/move_point 共用配置，误差定义由调用侧决定
    arrival_helper::Config arrival_judge_config_{};
    Eigen::Vector3d max_velocity_{Eigen::Vector3d::Ones()};
    double max_yaw_rate_rad_s_{0.5};
    // 降落阶段开始的时间
    ros::Time start_land_time_{ros::Time(0)};
    bool land_near_ground_ = false;
    ros::Time land_touchground_time_{ros::Time(0)};
    arrival_helper::State takeoff_arrival_state_{};
    arrival_helper::State point_arrival_state_{};
    reference_limit_helper::YawReferenceState yaw_reference_state_{};
    bool point_target_initialized_{false}; 
    bool body_point_target_initialized_{false};
    // -----------------缓存状态----------------
    control_common::Mavros_SetpointLocal last_setpoint_{};
    mutable std::mutex last_setpoint_mutex_;
    controller_data_types::TargetPoint_t last_point_;
    controller_data_types::TargetBodyPoint_t last_point_body_;
    Eigen::Vector3d hover_point_{Eigen::Vector3d::Zero()};  // hover 悬停点（与 linear 对齐）
    double hover_yaw_{0.0};
    // -------------------起降状态标志位--------------
    std::atomic<bool> takeoff_complete_{false};
    std::atomic<bool> land_complete_{false};
    std::atomic<bool> point_complete_{false};
    // --------------------里程计状态---------------------
    control_common::UAVStateEstimate uav_odometry_;
    std::atomic<bool> has_uav_odometry_{false};
    std::atomic<bool> can_fuse_{false};
    ros::Time last_odom_timestamp_{ros::Time(0)};
    // --------------------期望状态--------------------
    controller_data_types::TargetTrajectoryPoint_t desired_state_;
    // ros定时器
    ros::Timer pub_px4_state_timer;     // 用于发布话题数据的定时器
    ros::Timer pub_vision_pose_timer_;  // 用于发布vision_pose的定时器
};
