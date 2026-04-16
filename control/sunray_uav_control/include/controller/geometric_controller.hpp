#pragma once

#include "controller/controller_interface.hpp"
#include "core_algorithm/geometric_attitude_control.hpp"
#include "mavros_helper/mavros_helper.hpp"
#include "utils/quintic_curve.hpp"
#include <ros/node_handle.h>
#include <ros/time.h>
#include <string>
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
 *   - takeoff 不依赖 QuinticCurve，直接以目标高度点跟踪
 *
 * 故意不引入的依赖：
 *   - PX4_ParamManager（EKF2 参数管理）
 *   - curve::QuinticCurve（五次项轨迹曲线）
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
    void on_ground_keep_setpoint() override;
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

    // ----------------------控制器状态话题更新函数-----------------
    void pub_controller_state() override;  // stub，暂返回

  private:
    enum class AttitudeCommandMode : uint8_t {
        Attitude = 0,
        BodyRate = 1,
    };

    struct VelocityTrajectoryState {
        bool active{false};
        bool hold_fixed_height{false};
        double fixed_height{0.0};
        ros::Time last_cmd_stamp{ros::Time(0)};
        ros::Time segment_start_time{ros::Time(0)};
        double segment_duration{0.0};
        controller_data_types::TargetTrajectoryPoint_t segment_start;
        controller_data_types::TargetTrajectoryPoint_t segment_end;
    };

    struct RCThrustFilterState {
        bool initialized{false};
        double thrust{0.0};
        ros::Time last_update{ros::Time(0)};
    };

    // ----------------------配置相关-----------------------
    std::string config_yamlfile_path_;
    std::string uav_ns_;
    Geometric_AttitudeControl_Param_t geometric_controller_param_;
    Geometric_AttitudeControl controller_;
    AttitudeCommandMode attitude_command_mode_{AttitudeCommandMode::BodyRate};
    int fuse_odom_type{0};
    double fuse_odom_frequency{0.0};
    double velocity_ref_acc_xy_{1.5};
    double velocity_ref_acc_z_{0.8};

    // ----------------------检查相关-----------------------
    // void 类型 + throw 后缀 = 整个启动过程只执行一次，抛出异常则终止启动
    void load_and_validate_config_or_throw();   // 读取并校验 yaml 文件配置
    void ensure_fusion_param_ready_or_throw();  // 检查外部里程计融合参数是否就绪

    // bool 类型 = 运行过程中反复检查
    bool check_px4_basic_state();      // 检查 px4 飞控基础状态
    bool check_mavros_stream_ready();  // 检查 mavros_helper 数据流是否稳定
    bool check_odom_freshness();       // 检查外部里程计数据是否新鲜
    bool check_odom_for_fusion(
        control_common::UAVStateEstimate& fuse_odom);  // 检查本次融合里程计是否有效
    bool has_valid_imu_data() ;
    Eigen::Vector3d get_world_acc_from_imu() ;
    void reset_velocity_trajectory_state();
    controller_data_types::TargetTrajectoryPoint_t
    update_velocity_trajectory_reference(const Eigen::Vector3d& target_velocity_world,
                                         double target_yaw,
                                         double target_yaw_rate,
                                         const ros::Time& cmd_stamp,
                                         bool hold_fixed_height = false,
                                         double fixed_height = 0.0);
    void seed_rc_thrust_filter(RCThrustFilterState& state, double thrust, const ros::Time& now);
    double update_rc_thrust_filter(RCThrustFilterState& state,
                                   double target_thrust,
                                   double tau_s,
                                   const ros::Time& now);
    double get_takeoff_warmup_duration() const;
    double compute_takeoff_warmup_target_thrust(double target_thrust, double elapsed_s) const;
    void maybe_rebase_takeoff_curve_start();
    void update_hover_reference(const Eigen::Vector3d& hover_point,
                                double hover_yaw,
                                const char* reason);
    void reset_stage_thrust_filters();
    bool publish_trajectory_setpoint(
        const controller_data_types::TargetTrajectoryPoint_t& trajpoint,
        ThrustCommandPolicy thrust_policy = ThrustCommandPolicy::Auto);

    bool controller_ready_{false};

    // ---------------------定时器回调函数---------------------
    void pub_px4_state_timer_cb(const ros::TimerEvent&);    // 定时发布 PX4State
    void pub_vision_fuse_timer_cb(const ros::TimerEvent&);  // 定时发布 vision_pose

    // -------------------ros 句柄与 mavros 辅助类-----------------
    ros::NodeHandle nh_;
    MavrosHelper mavros_helper_;

    // --------------------px4 状态缓存-----------------------
    control_common::FlightMode px4_mode_{control_common::FlightMode::Undefined};
    control_common::LandedState px4_land_{control_common::LandedState::Undefined};
    bool px4_arm_{false};
    double pub_px4_state_freq_{100.0};  // px4state 话题发布频率（Hz）

    // 起飞 / 降落时锁定的 yaw 角（弧度）
    double takeoff_yaw_{0.0};
    double land_yaw_{0.0};

    // 起飞前记录的地面高度
    double takeoff_ground_height_{0.0};

    // 切换 Offboard 模式的计时上下文
    ros::Time start_checkout_offboard_time_{ros::Time(0)};
    ros::Time last_checkout_offboard_time_{ros::Time(0)};

    // 解锁后电机暖机阶段计时
    ros::Time last_arm_time_{ros::Time(0)};
    double motors_speedup_time_{2.0};  // 兼容旧配置：未显式指定 ramp_time 时作为缓升阶段时长
    double takeoff_idle_thrust_{0.08};
    double takeoff_idle_hold_time_{0.0};
    double takeoff_ramp_time_{2.0};
    double takeoff_thrust_filter_tau_{0.40};
    double takeoff_thrust_handoff_time_{0.30};
    RCThrustFilterState takeoff_thrust_filter_;
    double arrival_judge_stabile_time_s_{0.5};
    double arrival_pos_stabile_err_m_{0.15};
    double arrival_vel_stabile_err_mps_{0.15};

    // 五次项起飞曲线（平滑爬升，避免直接发送目标位置引起的阶跃响应）
    curve::QuinticCurve quint_curve_;

    // 起飞到位判断开始计时
    ros::Time start_checkout_takeoff_success_time_{ros::Time(0)};
    bool takeoff_curve_started_{false};

    // 降落阶段相关状态
    ros::Time landing_time_{ros::Time(0)};     // 首次进入 land() 的时刻
    ros::Time start_land_time_{ros::Time(0)};  // 靠近地面开始的计时
    bool land_near_ground_{false};
    ros::Time land_touchground_time_{ros::Time(0)};  // 触地并等待上锁的计时
    double land_max_velocity_{0.3};  // 降落最大速度（首次进入 land() 时由参数写入）
    double land_slow_height_m_{0.35};
    double land_near_ground_height_m_{0.18};
    double land_near_ground_velocity_{0.10};
    double land_touchdown_velocity_{0.04};
    double land_position_lookahead_time_{0.35};
    double land_thrust_filter_tau_{0.20};
    double land_near_ground_thrust_max_{0.31};
    double land_touchdown_thrust_max_{0.05};
    RCThrustFilterState land_thrust_filter_;

    // move_point 到位计时
    ros::Time start_move_arrive_time_{ros::Time(0)};
    bool move_point_arrive_state_{false};

    // -----------------缓存状态----------------
    control_common::Mavros_SetpointAttitude last_setpoint_;
    controller_data_types::TargetPoint_t last_point_;       // move_point 上次目标
    controller_data_types::TargetBodyPoint_t last_point_body_;  // move_point_body 上次目标
    Eigen::Vector3d land_point_{Eigen::Vector3d::Zero()};   // 降落时锁定的 xy 位置
    Eigen::Vector3d hover_point{Eigen::Vector3d::Zero()};  // hover 悬停点（与 linear 对齐）
    double hover_yaw_{0.0};

    // -------------------起降状态与 move_point 到位标志位--------------
    bool takeoff_complete_{false};
    bool land_complete_{false};
    bool point_complete_{false};

    // --------------------里程计状态---------------------
    control_common::UAVStateEstimate uav_odometry_;
    bool has_uav_odometry_{false};
    bool has_imu_{false};
    bool imu_acc_is_specific_force_{true};

    // --------------------期望状态（用于调试/日志）--------------------
    controller_data_types::TargetTrajectoryPoint_t desired_state_;
    VelocityTrajectoryState velocity_traj_state_;

    // ----------------------ros 话题发布者-----------------------
    ros::Publisher controller_state_pub_;

    // ----------------------ros 定时器-----------------------
    ros::Timer pub_px4_state_timer_;    // 周期发布 PX4State
    ros::Timer pub_vision_pose_timer_;  // 周期发布 vision_pose
};
