// TODO：测试工作完成后，选择一个子类作为geometric_controller的默认推力实现，移除本文件
// expect: core_algorithm文件夹下仅留一个geometric_control.hpp文件
#pragma once

#include <Eigen/Dense>
#include <deque>
#include <ros/time.h>
#include <string>

namespace thrust_estimator {

// ─────────────────────────────────────────────────────────────────────────
// 统一输入：悬停推力估计器在更新推力估计量时,需要的输入量
// ─────────────────────────────────────────────────────────────────────────
struct Input_t {
    ros::Time stamp{ros::Time(0)};
    Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d velocity_w{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration_w{Eigen::Vector3d::Zero()};
    double thrust_cmd{0.0};  // 当前发送给 PX4 的归一化 collective thrust
};

struct Param_t {
    double hover_thrust; // 归一化悬停推力
    double gravity; // 重力加速度 [正值,无方向性]
    double hover_thrust_min{0.05};
    double hover_thrust_max{0.80};

    bool ekf_onlyhover_estimate{true};
    double ekf_Q{1e-4};
    double ekf_R{0.05};
    double ekf_P0{0.1};
    double ekf_P_min{1e-6};
    double ekf_P_max{10.0};
    double ekf_delay_min_s{0.035};
    double ekf_delay_max_s{0.045};
    double ekf_innovation_gate{6.0};
    double ekf_min_thrust_cmd{0.05};
    double ekf_max_thrust_cmd{0.90};
    double ekf_min_tilt_cos_hover{0.50};
    double ekf_min_tilt_cos_move{0.85};
    double ekf_max_abs_acc_z_hover{5.0};
    double ekf_max_abs_acc_z_move{3.0};
    double ekf_convergence_p_threshold{0.02};
    double ekf_convergence_hold_s{1.0};
    bool ekf_adaptive_R_enabled{false};
    double ekf_R_min{0.005};
    double ekf_R_max{1.0};
};

// ─────────────────────────────────────────────────────────────────────────
// 抽象基类：统一 hover thrust 估计接口，由 control 层持有基类指针
// ─────────────────────────────────────────────────────────────────────────
class HoverThrustEstimator {
  public:
    virtual ~HoverThrustEstimator() = default;
    // 导入参数
    virtual void load_param(const Param_t& param);
    // 用外部提供的悬停油门估计值显式重置当前锚点
    virtual void seed_hover_thrust(double hover_thrust);
    // 返回估计器是否仅支持在悬停状态下进行悬停推力估计
    // 如果返回 true，则外层只在悬停时调用 update()
    // 如果返回 false，则外层在所有时刻都调用 update()
    virtual bool should_estimate_onlyhover() const {
        return onlyhover_estimate_;
    }
    // 更新悬停推力估计
    virtual void update(const Input_t& data_input) = 0;
    // 获取当前悬停推力估计值
    virtual double get_hover_thrust() const = 0;
    // RLS 默认视为可被控制层采纳，EKF 会覆盖该接口。
    virtual bool converged() const {
        return true;
    }

  protected:
    double hover_thrust_min_{0.05};
    double hover_thrust_max_{0.80};

    double hover_thrust_{0.0};
    double gravity_{9.81};
    bool onlyhover_estimate_{false};  // true: 仅在悬停状态下估计; false: 支持任意时刻估计
};

// ─────────────────────────────────────────────────────────────────────────
// RLS 最小二乘法 悬停推力估计器
// ─────────────────────────────────────────────────────────────────────────
class RLS_HoverThrustEstimator : public HoverThrustEstimator {
  public:
    // RLS 也仅在悬停段更新，避免机动过程将姿态/加速度耦合误差注入悬停推力估计
    explicit RLS_HoverThrustEstimator(){
        onlyhover_estimate_ = true;
    }

    void seed_hover_thrust(double hover_thrust) override;
    void update(const Input_t& input) override;
    double get_hover_thrust() const override;

  private:
    std::deque<Input_t> input_history_;

    const double rho2_ = 0.998;  // 遗忘因子，决定动态性能
    double thr2acc_{0.0}; // thr2acc_ = param_.gravity / param_.hover_thrust;
    double P_ = 1e6;
};

// ─────────────────────────────────────────────────────────────────────────
// EKF 基于加速度观测的悬停推力估计器
// ─────────────────────────────────────────────────────────────────────────
class EKF_HoverThrustEstimator : public HoverThrustEstimator {
  public:
    explicit EKF_HoverThrustEstimator() {
        onlyhover_estimate_ = true;
    }

    void load_param(const Param_t& param) override;
    void seed_hover_thrust(double hover_thrust) override;
    void update(const Input_t& input) override;
    double get_hover_thrust() const override;
    bool converged() const override;

  private:
    struct UpdateStatus {
        ros::Time stamp{ros::Time(0)};
        bool updated{false};
        bool rejected{false};
        bool converged{false};
        std::string reject_reason;

        double hover_thrust{0.0};
        double P{0.0};
        double Q{0.0};
        double R{0.0};
        double K{0.0};
        double innovation{0.0};
        double innovation_cov{0.0};
        double nis{0.0};
        double measurement_y{0.0};
        double measurement_pred{0.0};
        double H{0.0};
        double input_thrust_cmd{0.0};
        double delayed_thrust_cmd{0.0};
        double tilt_cos{0.0};
        double acc_z{0.0};
        double velocity_z{0.0};
        double sample_delay{0.0};
    };

    std::deque<Input_t> input_history_;

    ros::Time last_input_stamp_{ros::Time(0)};
    ros::Time last_update_stamp_{ros::Time(0)};
    ros::Time converged_since_{ros::Time(0)};

    double P_{0.1};
    double P0_{0.1};
    double Q_{1e-4};
    double R_{0.05};
    double R_min_{0.005};
    double R_max_{1.0};

    double delay_min_s_{0.035};
    double delay_max_s_{0.045};

    double p_min_{1e-6};
    double p_max_{10.0};

    double innovation_gate_{6.0};

    double min_thrust_cmd_{0.05};
    double max_thrust_cmd_{0.90};
    double min_tilt_cos_hover_{0.50};
    double min_tilt_cos_move_{0.85};
    double max_abs_acc_z_hover_{5.0};
    double max_abs_acc_z_move_{3.0};

    double convergence_p_threshold_{0.02};
    double convergence_hold_s_{1.0};

    bool converged_{false};
    bool adaptive_R_enabled_{false};

    UpdateStatus debug_{};

    void reset_debug();
    void reject_update(const std::string& reason);
    void update_convergence(const ros::Time& stamp, bool accepted);
    void update_R(double innovation, bool accepted);
};
};  // namespace thrust_estimator
