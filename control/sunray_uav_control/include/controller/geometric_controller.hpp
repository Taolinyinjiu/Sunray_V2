#pragma once

#include "controller/controller_interface.hpp"
#include "controller/takeoff_land_utils.hpp"
#include "core_algorithm/geometric_attitude_control.hpp"
#include "mavros_helper/mavros_helper.hpp"
#include "utils/arrival_helper.hpp"
#include "utils/reference_limit_helper.hpp"
#include "utils/quintic_curve.hpp"
#include "utils/yaw_curve.hpp"
#include <ros/node_handle.h>
#include <ros/time.h>
#include <string>
#include <atomic>
#include <Eigen/Dense>

/**
 * @file geometric_controller.hpp
 * @brief 几何控制器适配层
 *
 * 继承 Controller_Interface，将 Geometric_AttitudeControl 核心算法封装为
 * 符合 Sunray 框架接口规范的控制器。
 *
 * 与 PX4_LinearAttitude_Controller 的最大差异：
 *   - 发布 AttitudeTarget，可按配置选择 Attitude 或 BodyRate 模式
 *   - calculateControl 不需要 IMU 输入
 *   - move_trajectory 真正实现（直接将 trajpoint 传入核心算法）
 *   - takeoff 使用平滑爬升曲线，move_point 使用位置参考平滑
 *
 * 故意不引入的依赖：
 *   - PX4 参数管理职责（已迁移至 system_check 方向）
 */
class Geometric_Controller : public Controller_Interface {
  public:
    explicit Geometric_Controller(ros::NodeHandle& nh);
    ~Geometric_Controller() = default;

    // ------------生命周期相关----------------
    bool init() override;      // 读取 config.yaml，初始化 mavros_helper、定时器
    bool is_ready() override;  // 检查里程计新鲜度、mavros 连通性等

    // -------------状态注入---------------
    void set_current_odom(const control_common::UAVStateEstimate& odom) override;

    // -------------运动相关接口------------
    // 在地面保持 setpoint 流，推力置 0
    void set_position_mode() override;
    // 触发起飞（简化版：直接以目标高度点跟踪，不使用五次项曲线）
    bool takeoff(double relative_takeoff_height, double max_takeoff_velocity) override;
    // 触发降落
    bool land(bool land_type, double max_land_velocity) override;
    // 设置悬停点
    bool set_hover_point(control_common::UAVStateEstimate current_odom) override;
    // 进入悬停状态
    bool hover() override;
    // 紧急上锁
    bool emergency_kill() override;
    // 运动到某一点（世界系）
    bool move_point(controller_data_types::TargetPoint_t point) override;
    // 以速度控制运动（stub，暂不实现）
    bool move_velocity(controller_data_types::TargetVelocity_t velocity) override;
    // 跟踪轨迹点（几何控制器的核心运动接口，真正实现）
    bool move_trajectory(controller_data_types::TargetTrajectoryPoint_t trajpoint) override;
    // 运动到机体系某点（stub）
    bool move_point_body(controller_data_types::TargetBodyPoint_t point) override;
    // 以机体系速度运动（stub）
    bool move_velocity_body(controller_data_types::TargetBodyVelocity_t velocity) override;
    // 移动到 WGS84 坐标（stub）
    bool move_point_wgs84(geographic_msgs::GeoPoint point) override;

    // ---------------------起降状态查询接口-----------------------
    bool is_takeoff_complete() override;
    bool is_land_complete() override;
    bool is_point_complete() override;

  private:
    enum class AttitudeCommandMode : uint8_t {
        Attitude = 0,
        BodyRate = 1,
    };

    enum class MotionCurveOwner : uint8_t {
        None = 0,
        MovePoint = 1,
        Takeoff = 2,
    };

    // ----------------------配置相关-----------------------
    std::string config_yamlfile_path_;
    Geometric_AttitudeControl_Param_t geometric_controller_param_;
    Geometric_AttitudeControl controller_;
    AttitudeCommandMode attitude_command_mode_{AttitudeCommandMode::BodyRate};
    int fuse_odom_type{0};
    double fuse_odom_frequency{0.0};
    Eigen::Vector3d max_velocity_{Eigen::Vector3d::Ones()};
    double max_yaw_rate_rad_s_{0.5};

    // ----------------------检查相关-----------------------
    // void 类型 + throw 后缀 = 整个启动过程只执行一次，抛出异常则终止启动
    void load_and_validate_config_or_throw();  // 读取并校验 yaml 文件配置

    // bool 类型 = 运行过程中反复检查
    bool check_px4_basic_state();      // 检查 px4 飞控基础状态
    bool check_mavros_stream_ready();  // 检查 mavros_helper 数据流是否稳定
    bool has_valid_imu_data();
    Eigen::Vector3d get_world_acc_from_imu();
    void seed_rc_thrust_filter(takeoff_land::RCThrustFilterState& state,
                               double thrust,
                               const ros::Time& now);
    double update_rc_thrust_filter(takeoff_land::RCThrustFilterState& state,
                                   double target_thrust,
                                   double tau_s,
                                   const ros::Time& now);
    void maybe_rebase_takeoff_curve_start();
    void update_hover_reference(const Eigen::Vector3d& hover_point,
                                double hover_yaw,
                                const char* reason);
    bool move_point_impl(controller_data_types::TargetPoint_t point,
                         bool preserve_body_point_context);
    void clear_motion_curve();
    void begin_motion_curve(MotionCurveOwner owner);
    void reset_point_motion_context();
    double update_limited_yaw_target(double target_yaw, const ros::Time& now);
    double integrate_limited_yaw_rate(double yaw_rate_cmd, const ros::Time& now);
    void warn_if_trajectory_exceeds_limits(
        const controller_data_types::TargetTrajectoryPoint_t& trajpoint) const;
    void reset_stage_thrust_filters();
    bool
    publish_trajectory_setpoint(const controller_data_types::TargetTrajectoryPoint_t& trajpoint,
                                ThrustCommandPolicy thrust_policy =
                                    ThrustCommandPolicy::UseEstimatedAnchor);

    std::atomic<bool> controller_ready_{false};

    // ---------------------定时器回调函数---------------------
    void pub_px4_state_timer_cb(const ros::TimerEvent&);    // 定时发布 PX4State
    void pub_vision_fuse_timer_cb(const ros::TimerEvent&);  // 定时发布 vision_pose

    // -------------------ros 句柄与 mavros 辅助类-----------------
    ros::NodeHandle nh_;
    MavrosHelper mavros_helper_;

    // --------------------到达判断
    // takeoff/move_point 共用配置，调用侧决定误差如何构造。
    arrival_helper::Config arrival_judge_config_{};

    // --------------------px4 状态缓存-----------------------
    double pub_px4_state_freq_{100.0};  // px4state 话题发布频率（Hz）

    // 起降阶段共享的地面参考高度
    double ground_height_ref_{0.0};

    // 切换 Offboard 模式的计时上下文
    ros::Time start_checkout_offboard_time_{ros::Time(0)};
    ros::Time last_checkout_offboard_time_{ros::Time(0)};

    takeoff_land::TakeoffTuning takeoff_tuning_{};
    takeoff_land::TakeoffState takeoff_state_{};

    // 共享轨迹曲线：takeoff/move_point 轮流占用，通过 owner 显式管理生命周期。
    curve::QuinticCurve motion_curve_;
    MotionCurveOwner motion_curve_owner_{MotionCurveOwner::None};
    curve::QuinticYawCurve yaw_curve_;

    // 降落阶段相关状态
    takeoff_land::LandingTuning landing_tuning_{};
    takeoff_land::LandingState landing_state_{};

    arrival_helper::State takeoff_arrival_state_{};
    arrival_helper::State point_arrival_state_{};
    reference_limit_helper::YawReferenceState yaw_reference_state_{};
    bool point_target_initialized_{false};
    bool body_point_target_initialized_{false};

    // -----------------缓存状态----------------
    control_common::Mavros_SetpointAttitude last_setpoint_;
    controller_data_types::TargetPoint_t last_point_;           // move_point 上次目标
    controller_data_types::TargetBodyPoint_t last_point_body_;  // move_point_body 上次目标
    Eigen::Vector3d hover_point{Eigen::Vector3d::Zero()};       // hover 悬停点（与 linear 对齐）
    double hover_yaw_{0.0};

    // -------------------起降状态与 move_point 到位标志位--------------
    std::atomic<bool> takeoff_complete_{false};
    std::atomic<bool> land_complete_{false};
    std::atomic<bool> point_complete_{false};

    // --------------------里程计状态---------------------
    control_common::UAVStateEstimate uav_odometry_;
    std::atomic<bool> has_uav_odometry_{false};
    std::atomic<bool> has_imu_{false};
    std::atomic<bool> can_fuse_{false};
    ros::Time last_odom_timestamp_{ros::Time(0)};
    bool imu_acc_is_specific_force_{true};

    // --------------------期望状态--------------------
    controller_data_types::TargetTrajectoryPoint_t desired_state_;

    // ----------------------ros 定时器-----------------------
    ros::Timer pub_px4_state_timer_;    // 周期发布 PX4State
    ros::Timer pub_vision_pose_timer_;  // 周期发布 vision_pose
};
