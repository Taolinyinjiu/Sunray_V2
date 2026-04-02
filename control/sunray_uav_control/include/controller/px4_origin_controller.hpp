/**
 * @file px4_origin_controller.hpp
 * @brief 基于px4原生控制环的控制器，从Controller_Interface派生
 * @details
    1. 引入Mavros_Helper类用于读取px4数据
    2. 引入Mavros_ParamManager类用于读取+修改 px4参数
    3. 向Sunray_FSM提供规范的函数实现
    4. 发布controller_state话题，属于自定义消息sunray_msgs::UAVControllerState
    5. 修改构造函数为仅存储ros句柄，显示调用init()函数进行初始化
    6. 初始化检查使用void类型，如果检查失败会通过抛出异常的方式终止程序运行
       运行时检查函数使用bool类型，检查失败通过返回false警告
    7. TODO: [ ] 实现public中需要的函数
             [-] 实现controller_state_pub_的初始化
             [-] 实现pub_timer_的初始化
             [ ] 实现px4_mode和px4_arm的同步(从mavros_helper中读取数据)
             [ ] desired_state_初始化
             [-] 引入Mavros_ParamManager类，修改EV_CTRL以及相关参数
             [ ] 修改Mavros_ParamManager类名为MavrosParam
             [ ] 引入px4_nsh_simulator 实现重启ekf2或者将ekf2重启放在Mavros_Param或者Helper？
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
#include <px4_param_manager/px4_param_manager.h>
#include <ros/node_handle.h>
#include <string>
#include "utils/quintic_curve.hpp"
// 在这里构造参数结构体？对于原生控制器，要检查什么参数呢
struct Original_Param {
    // 1, vision_pose
    int fuse_odom_type{0};
    double fuse_odom_frequency{0.0};
    // 2. 暂时没想到
};

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
    // 触发起飞，参数为起飞高度和最大起飞速度
    bool takeoff(double relative_takeoff_height, double max_takeoff_velocity) override;
    // 触发降落，参数为降落类型和最大降落速度
    bool land(bool land_type, double max_land_velocity) override;
    // 在当前点悬停(运动过程触发立即停止并进入悬停)
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
    bool move_point_body(controller_data_types::TargetPoint_t point) override;
    // 以机体系速度的方式运动
    bool move_velocity_body(controller_data_types::TargetVelocity_t velocity) override;
    // 移动到WGS84下的某一点
    bool move_point_wgs84(controller_data_types::TargetPoint_t point) override;
    // ---------------------起降状态查询接口-----------------------
    bool is_takeoff_complete() override;
    bool is_land_complete() override;
    // ----------------------控制器状态话题更新函数-----------------
    void pub_controller_state() override;  // 发布controller_state
  private:
    // ----------------------配置相关-----------------------
    std::string config_yamlfile_path_;
    std::string uav_ns_;
    Original_Param config_param_;
    // ----------------------检查相关-----------------------
    // void 类型 + throw后缀 = 整个启动过程只会执行一次，抛出异常则终止启动
    // bool 类型 表示运行过程中检查，会反复调用maybe
    void load_and_validate_config_or_throw();   // 读取并校验yaml文件配置的参数
    void ensure_fusion_param_ready_or_throw();  // 检查外部里程计融合参数是否正确

    bool check_px4_basic_state();      // 检查px4飞控的状态
    bool check_mavros_stream_ready();  // 检查mavros_helper是否在稳定的更新数据
    bool check_odom_freshness();       // 检查外部里程计数据是否新鲜
    bool check_odom_for_fusion(
        control_common::UAVStateEstimate& fuse_odom);  // 检查本次用于融合的里程计是否有效
    bool controller_ready_ = false;
    // ---------------------定时器回调函数---------------------
    void pub_px4_state_timer_cb(const ros::TimerEvent&);    // 发布PX4State话题
    void pub_vision_fuse_timer_cb(const ros::TimerEvent&);  // 发布vision_fuse
    // -------------------ros句柄与mavros辅助类-----------------
    ros::NodeHandle nh_;
    MavrosHelper mavros_helper_;
    PX4_ParamManager mavros_param_;
    // ---------------------曲线辅助类-----------------
    curve::QuinticCurve quint_curve_;
    // --------------------px4状态-----------------------
    control_common::FlightMode px4_mode_ = control_common::FlightMode::Undefined;
    control_common::LandedState px4_land_ = control_common::LandedState::Undefined;
    bool px4_arm_ = false;
    double pub_px4_state_freq_ = 100.0;  // 考虑简单点，在这里硬编码px4state的发布频率
    bool param_state_ = false;
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
    // 起飞成功判断阈值,保持至少2s的稳定，算到达期望位置
    double takeoff_success_keep_time_s = 2;
    ros::Time start_checkout_takeoff_success_time_{ros::Time(0)};
    // 降落阶段开始的时间
    ros::Time start_land_time_{ros::Time(0)};
    bool land_near_ground_ = false;
    ros::Time land_touchground_time_{ros::Time(0)};
    // move稳定的时间
    ros::Time start_move_arrive_time_{ros::Time(0)};
    bool move_point_arrive_state_{false};
    // -----------------缓存状态----------------
    control_common::Mavros_SetpointLocal last_setpoint_{};
    controller_data_types::TargetPoint_t last_point_;
    controller_data_types::TargetPoint_t last_point_body_;

    // -------------------起降状态标志位--------------
    bool takeoff_complete_{false};
    bool land_complete_{false};
    // --------------------里程计状态---------------------
    control_common::UAVStateEstimate uav_odometry_;
    bool has_uav_odometry_{false};
    // --------------------期望状态--------------------
    controller_data_types::TargetTrajectoryPoint_t desired_state_;
    // ros话题发布者
    ros::Publisher controller_state_pub_;  // 发布controller状态话题数据
    // ros定时器
    ros::Timer pub_px4_state_timer;     // 用于发布话题数据的定时器
    ros::Timer pub_vision_pose_timer_;  // 用于发布vision_pose的定时器
};
