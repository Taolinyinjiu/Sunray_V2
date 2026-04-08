/**
 * @file geometric_attitude_control.hpp
 * @brief 几何控制器核心算法
 *
 * 与 ROS 无关，输入为轨迹点 + 里程计，输出为归一化推力 + BodyRate。
 * 移植自 ecbf_bodyrate/geometric_controller，保留位置环、姿态环两级结构。
 *
 * 内部四元数约定：Eigen::Vector4d(qw, qx, qy, qz)
 * 公共接口四元数约定：Eigen::Quaterniond（与 Sunray 其他模块一致）
 */

#pragma once

#include "control_data_types/controller_desired_types.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include <Eigen/Dense>
#include <memory>

// ═══════════════════════════════════════════════════════════
// 参数结构体
// ═══════════════════════════════════════════════════════════

struct Geometric_AttitudeControl_Param_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ── 位置环增益 ──────────────────────────────────────────
    // 对应 ecbf_bodyrate 中的 Kpos_x/y/z 和 Kvel_x/y/z
    Eigen::Vector3d Kpos{Eigen::Vector3d::Zero()};  // 位置比例增益 [x, y, z]
    Eigen::Vector3d Kvel{Eigen::Vector3d::Zero()};  // 速度比例增益 [x, y, z]
    double max_acc{9.0};                            // 位置环加速度输出限幅 (m/s²)

    // ── 姿态环参数 ──────────────────────────────────────────
    double attctrl_tau{0.1};  // 姿态控制时间常数：越小响应越快，太小会振荡
    int ctrl_mode{0};         // 0 = 四元数误差法, 1 = 几何 SO3 法 (Lee 2010)

    // ── 物理参数 ────────────────────────────────────────────
    double drone_mass{1.5};  // 无人机质量 (kg)
    double gravity{9.8};     // 重力加速度 (m/s²)

    // ── 推力归一化映射 ──────────────────────────────────────
    // 公式：thrust_normalized = norm_thrust_const * (drone_mass * thrust_acc) + norm_thrust_offset
    // !! 这两个参数与机型强相关，更换机型时必须重新标定 !!
    // 默认值来自 ecbf_bodyrate 对 Iris 机型的实验拟合
    double norm_thrust_const{0.038};
    double norm_thrust_offset{0.1};

    // ── 转子阻力补偿 ────────────────────────────────────────
    // 对应 ecbf_bodyrate 中的 D_ 向量，不确定时保持零向量
    Eigen::Vector3d D{Eigen::Vector3d::Zero()};

    // ── Z 轴位置积分 ────────────────────────────────────────
    // 用于消除推力模型未标定时产生的 Z 轴稳态悬停误差
    // 若推力模型已精确标定，可将 pos_ki_z 设为 0 禁用此项
    double pos_ki_z{0.0};         // Z 轴积分增益，建议从 0.2 开始调节
    double int_max_z{2.0};        // Z 轴积分项输出限幅 (m/s²)，防止积分饱和
    double controller_hz{100.0};  // 控制器运行频率 (Hz)，用于计算积分步长 dt
};

// ═══════════════════════════════════════════════════════════
// 输出结构体
// 字段与 Linear_AttitudeControl_Output_t 保持一致，方便上层统一处理
// ═══════════════════════════════════════════════════════════

struct Geometric_AttitudeControl_Output_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};  // 期望姿态（用于可视化/调试）
    Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};  // 机体角速度 [ωx, ωy, ωz] (rad/s)
    double thrust{0.0};                                  // 归一化推力，范围 [0, 0.95]
};

// ═══════════════════════════════════════════════════════════
// 姿态控制子模块
// 对应 ecbf_bodyrate 中的 Control 抽象基类 + 两个具体实现
// 内部使用 Vector4d(qw, qx, qy, qz) 四元数约定
// ═══════════════════════════════════════════════════════════

// 抽象基类：定义姿态子控制器的统一接口
class GeometricAttitudeSubCtrl {
  public:
    virtual ~GeometricAttitudeSubCtrl() = default;

    // 输入当前姿态、期望姿态、期望加速度、期望加加速度，计算期望角速度和推力
    virtual void update(const Eigen::Vector4d& curr_att,
                        const Eigen::Vector4d& ref_att,
                        const Eigen::Vector3d& ref_acc,
                        const Eigen::Vector3d& ref_jerk) = 0;

    Eigen::Vector3d get_desired_rate() const {
        return desired_rate_;
    }
    Eigen::Vector3d get_desired_thrust() const {
        return desired_thrust_;
    }

  protected:
    Eigen::Vector3d desired_rate_{Eigen::Vector3d::Zero()};  // 输出：期望机体角速度
    Eigen::Vector3d desired_thrust_{
        Eigen::Vector3d::Zero()};  // 输出：期望推力方向（z 分量为有效值）
};

// 四元数误差姿态控制（简单，推荐首选）
// 对应 ecbf_bodyrate 中的 NonlinearAttitudeControl
// 参考：Brescianini et al., "Nonlinear quadrocopter attitude control", ETH Zurich, 2013
class QuaternionAttitudeCtrl : public GeometricAttitudeSubCtrl {
  public:
    explicit QuaternionAttitudeCtrl(double attctrl_tau);
    void update(const Eigen::Vector4d& curr_att,
                const Eigen::Vector4d& ref_att,
                const Eigen::Vector3d& ref_acc,
                const Eigen::Vector3d& ref_jerk) override;

  private:
    double attctrl_tau_{0.1};
};

// SO3 几何姿态控制（更精确）
// 对应 ecbf_bodyrate 中的 NonlinearGeometricControl
// 参考：Lee et al., "Geometric tracking control of a quadrotor UAV on SE(3)", CDC 2010
class SO3AttitudeCtrl : public GeometricAttitudeSubCtrl {
  public:
    explicit SO3AttitudeCtrl(double attctrl_tau);
    void update(const Eigen::Vector4d& curr_att,
                const Eigen::Vector4d& ref_att,
                const Eigen::Vector3d& ref_acc,
                const Eigen::Vector3d& ref_jerk) override;

  private:
    double attctrl_tau_{0.1};
};

// ═══════════════════════════════════════════════════════════
// 主控制器类
// ═══════════════════════════════════════════════════════════

class Geometric_AttitudeControl {
  public:
    Geometric_AttitudeControl() = default;
    ~Geometric_AttitudeControl() = default;

    // 加载参数，同时根据 param.ctrl_mode 实例化对应的姿态子控制器
    void load_param(const Geometric_AttitudeControl_Param_t& param);

    // 主控制计算接口
    // 输入：期望轨迹点 + 当前里程计
    // 输出：期望姿态 + 机体角速度 + 归一化推力
    Geometric_AttitudeControl_Output_t
    calculateControl(const controller_data_types::TargetTrajectoryPoint_t& des_state,
                     const control_common::UAVStateEstimate& current_odom);

    // 重置 Z 轴积分状态（起飞/降落/切换模式时调用，防止积分饱和残留）
    void reset_integral() {
        integral_z_ = 0.0;
    }

    // 初始化期望 yaw 缓存（起飞锁定 yaw 后调用，防止首次 move_point 时 yaw 跳变到 0）
    void set_initial_yaw(double yaw_rad) {
        last_desired_yaw_ = yaw_rad;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    Geometric_AttitudeControl_Param_t param_;

    // 由 ctrl_mode 决定运行时实例类型（0: QuaternionAttitudeCtrl, 1: SO3AttitudeCtrl）
    std::shared_ptr<GeometricAttitudeSubCtrl> att_controller_;

    // Z 轴位置积分状态（单位：m·s，乘以 pos_ki_z 后得到 m/s² 的加速度补偿量）
    double integral_z_{0.0};

    // 期望 yaw 缓存：仅当上层显式设置 des_state.yaw 时更新，否则保持上次值
    // 初始为 0，应在起飞锁定 yaw 后通过 set_initial_yaw() 同步
    double last_desired_yaw_{0.0};

    // ── 内部计算函数 ─────────────────────────────────────────────────────────
    // 对应 ecbf_bodyrate 中 geometric_controller.cpp 的各私有方法

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

    // 由期望加速度调用姿态子控制器，计算 body rate 和归一化推力
    // bodyrate_cmd: [ωx, ωy, ωz, thrust_normalized]，以 Vector4d 形式传出
    void computeBodyRateCmd(Eigen::Vector4d& bodyrate_cmd,
                            const Eigen::Vector3d& a_des,
                            const Eigen::Vector4d& curr_att_vec,
                            double mav_yaw,
                            const Eigen::Vector3d& target_jerk);

    // 由期望加速度方向和偏航角计算期望姿态四元数（内部 Vector4d 约定）
    static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d& vector_acc, double yaw);
};
