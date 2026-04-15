/**
 * @file geometric_attitude_control.hpp
 * @brief Sunray控制器核心算法
 *
 * 主体控制逻辑与 ROS 解耦，输入为轨迹点 + 里程计，输出为归一化推力 + BodyRate。
 * 为桥接悬停推力估计器，会局部依赖 ros::Time 相关类型。
 * 移植自 ecbf_bodyrate/geometric_controller，保留位置环、姿态环两级结构。
 *
 * 内部四元数约定：Eigen::Vector4d(qw, qx, qy, qz)
 * 公共接口四元数约定：Eigen::Quaterniond（与 Sunray 其他模块一致）
 */

#pragma once

#include "control_data_types/controller_desired_types.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include "controller/core_algorithm/attitude_slover.hpp"
#include "controller/core_algorithm/hoverthrust_estimator.hpp"
#include <Eigen/Dense>
#include <memory>

// ═══════════════════════════════════════════════════════════
// 控制类型枚举
// ═══════════════════════════════════════════════════════════
enum class Control_Type : uint8_t { Undefine_ = 0, Point_, Velocity_, Trajectory_ };

// ═══════════════════════════════════════════════════════════
// 控制器参数结构体
// ═══════════════════════════════════════════════════════════
struct Geometric_AttitudeControl_Param_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ── 位置环 PID 参数 ──────────────────────────────────────
    Eigen::Vector3d pos_kp{Eigen::Vector3d::Zero()};
    Eigen::Vector3d pos_ki{Eigen::Vector3d::Zero()};
    Eigen::Vector3d pos_kd{Eigen::Vector3d::Zero()};

    // ── 速度环 PID 参数 ──────────────────────────────────────
    Eigen::Vector3d vel_kp{Eigen::Vector3d::Zero()};
    Eigen::Vector3d vel_ki{Eigen::Vector3d::Zero()};
    Eigen::Vector3d vel_kd{Eigen::Vector3d::Zero()};

    double max_acc{9.0};                            // 位置环加速度输出限幅 (m/s²)

    // ── 姿态环参数 ──────────────────────────────────────────
    double attitude_tau{0.1};  // 姿态控制时间常数：越小响应越快，太小会振荡
    int attitude_type{0};      // 0 = 四元数误差法, 1 = 几何 SO3 法 (Lee 2010)

    // ── 物理参数 ────────────────────────────────────────────
    double drone_mass{1.5};  // 无人机质量 (kg)
    double gravity{9.8};     // 重力加速度 (m/s²)

    // ── 推力模型参数 ────────────────────────────────────────
    // hover_thrust_init: 线性回退模型的锚点，也是估计器初始化值
    double hover_thrust_init{0.35};
    int hover_thrust_estimator_type{1};  // 0 = LowPass, 1 = RLS, 2 = Kalman

    // ── 转子阻力补偿 ────────────────────────────────────────
    // 对应 ecbf_bodyrate 中的 D_ 向量，不确定时保持零向量
    Eigen::Vector3d D{Eigen::Vector3d::Zero()};

    // ── 三轴积分限幅 ────────────────────────────────────────
    // 按轴限制 pos/vel 积分环节的输出加速度，防止积分饱和
    Eigen::Vector3d int_max_pos{Eigen::Vector3d::Constant(2.0)};
    Eigen::Vector3d int_max_vel{Eigen::Vector3d::Constant(2.0)};
    double controller_hz{100.0};  // 控制器运行频率 (Hz)，用于计算积分步长 dt
};

// ═══════════════════════════════════════════════════════════
// 输出结构体
// 字段与 Linear_AttitudeControl_Output_t 保持一致，方便上层统一处理
// ═══════════════════════════════════════════════════════════

struct Geometric_AttitudeControl_Output_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};  // 期望姿态
    Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};  // 机体角速度 [ωx, ωy, ωz] (rad/s)
    double thrust{0.0};                                  // 归一化推力，范围 [0, 0.95]
};

enum class ThrustCommandPolicy : uint8_t {
    Auto = 0,
    ForceLinear,
};

// ═══════════════════════════════════════════════════════════
// 主控制器类
// ═══════════════════════════════════════════════════════════

class Geometric_AttitudeControl {
  public:
    Geometric_AttitudeControl() = default;
    ~Geometric_AttitudeControl() = default;

    // 加载参数，同时根据 param.attitude_type 实例化对应的姿态子控制器
    void load_param(const Geometric_AttitudeControl_Param_t& param);

    // 主控制计算接口
    // 输入：期望轨迹点 + 当前里程计
    // 输出：期望姿态 + 机体角速度 + 归一化推力
    Geometric_AttitudeControl_Output_t
    calculateControl(const controller_data_types::TargetTrajectoryPoint_t& des_state,
                     const control_common::UAVStateEstimate& current_odom,
                     ThrustCommandPolicy thrust_policy = ThrustCommandPolicy::Auto);

    // 纯速度控制接口
    // 输入：期望速度 + 当前里程计
    // 输出：期望姿态 + 机体角速度 + 归一化推力
    Geometric_AttitudeControl_Output_t
    calculateVelocityControl(const controller_data_types::TargetVelocity_t& des_state,
                             const control_common::UAVStateEstimate& current_odom,
                             ThrustCommandPolicy thrust_policy = ThrustCommandPolicy::Auto);

    // 向悬停推力估计器注入观测数据
    void feed_thrust_estimator(const thrust_estimator::Input_t& input);

    // 重置三轴积分状态（起飞/降落/切换模式时调用，防止积分饱和残留）
    void reset_integral() {
        integral_pos_.setZero();
        integral_vel_.setZero();
        last_pos_error_.setZero();
        last_vel_error_.setZero();
    }

    // 仅重置 z 轴积分与误差缓存，避免高度环残留影响下一段运动
    void reset_vertical_integral() {
        integral_pos_.z() = 0.0;
        integral_vel_.z() = 0.0;
        last_pos_error_.z() = 0.0;
        last_vel_error_.z() = 0.0;
    }

    // 初始化期望 yaw 缓存（起飞锁定 yaw 后调用）
    void set_initial_yaw(double yaw_rad) {
        last_desired_yaw_ = yaw_rad;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    enum class ThrustModelMode : uint8_t {
        LinearFallback = 0,
        LinearForced,
        LowPassEstimator,
        RLSEstimator,
        KalmanEstimator,
    };

    Geometric_AttitudeControl_Param_t param_;

    // 由 attitude_type 决定运行时实例类型（0: Quaternion_Solver, 1: SO3_Solver）
    std::shared_ptr<Attitude_Slover> att_controller_;

    // 悬停推力估计器：由 core 持有，controller 仅负责注入观测数据
    std::unique_ptr<thrust_estimator::HoverThrustEstimator> hover_thrust_estimator_;

    // 三轴位置/速度误差积分状态
    Eigen::Vector3d integral_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d integral_vel_{Eigen::Vector3d::Zero()};

    // 三轴位置/速度误差缓存（用于 D 项）
    Eigen::Vector3d last_pos_error_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d last_vel_error_{Eigen::Vector3d::Zero()};

    // 期望 yaw 缓存：每次控制更新都会同步为 des_state.yaw
    // 主要用于与外部接口保持一致，并保留 set_initial_yaw() 的初始化入口
    double last_desired_yaw_{0.0};
    Control_Type last_control_type_{Control_Type::Undefine_};

    // 推力估计更新使用上一拍已输出的推力命令，保持与 controller 发布时序一致
    double last_thrust_cmd_{0.0};
    double pending_thrust_cmd_{0.0};
    bool has_pending_thrust_cmd_{false};
    ThrustModelMode last_thrust_model_mode_{ThrustModelMode::LinearFallback};
    bool thrust_model_mode_initialized_{false};

    // ── 内部计算函数 ─────────────────────────────────────────────────────────
    // 对应 ecbf_bodyrate 中 geometric_controller.cpp 的各私有方法

    void advance_thrust_command_history();
    double map_thrust_acc_to_hover_anchor(double thrust_acc) const;
    double compose_thrust_command(double thrust_acc, ThrustCommandPolicy thrust_policy);
    ThrustModelMode get_hover_estimator_mode() const;
    static const char* thrust_model_mode_name(ThrustModelMode mode);
    void log_thrust_model_usage(ThrustModelMode mode, double thrust_cmd, double hover_thrust);

    // 位置 PD 控制器：由位置误差和速度误差计算加速度反馈量，并对输出限幅
    Eigen::Vector3d poscontroller(const Eigen::Vector3d& pos_error,
                                  const Eigen::Vector3d& vel_error);

    // 综合位置反馈、前馈加速度、转子阻力补偿，生成最终期望加速度向量
    Eigen::Vector3d controlPosition(const Eigen::Vector3d& target_pos,
                                    const Eigen::Vector3d& target_vel,
                                    const Eigen::Vector3d& target_acc,
                                    const Eigen::Vector3d& mav_pos,
                                    const Eigen::Vector3d& mav_vel,
                                    double mav_yaw);

    // 纯速度控制：不引入轨迹加速度前馈，只由速度误差生成期望加速度
    Eigen::Vector3d controlVelocity(const Eigen::Vector3d& target_vel,
                                    const Eigen::Vector3d& mav_vel,
                                    double target_yaw);

    // 由期望加速度调用姿态子控制器，计算 body rate 和归一化推力
    // bodyrate_cmd: [ωx, ωy, ωz, thrust_acc]，第4维为 a_des·zb (m/s²)
    void computeBodyRateCmd(Eigen::Vector4d& bodyrate_cmd,
                            const Eigen::Vector3d& a_des,
                            const Eigen::Quaterniond& curr_att,
                            double mav_yaw);

    // 由期望加速度方向和偏航角计算期望姿态四元数
    static Eigen::Quaterniond acc2quaternion(const Eigen::Vector3d& vector_acc, double yaw);
};
