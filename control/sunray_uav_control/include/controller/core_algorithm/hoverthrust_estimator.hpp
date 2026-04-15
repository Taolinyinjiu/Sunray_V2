#pragma once

#include <Eigen/Dense>
#include <deque>
#include <ros/time.h>

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
};

// ─────────────────────────────────────────────────────────────────────────
// 抽象基类：统一 hover thrust 估计接口，由 control 层持有基类指针
// ─────────────────────────────────────────────────────────────────────────
class HoverThrustEstimator {
  public:
    virtual ~HoverThrustEstimator() = default;
    // 导入参数
    virtual void load_param(const Param_t& param);
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

  protected:
    double hover_thrust_min_{0.05};
    double hover_thrust_max_{0.80};

    double hover_thrust_{0.0};
    double gravity_{9.81};
    bool onlyhover_estimate_{false};  // true: 仅在悬停状态下估计; false: 支持任意时刻估计
};

// ─────────────────────────────────────────────────────────────────────────
// 一阶低通RC滤波 悬停推力估计器
// ─────────────────────────────────────────────────────────────────────────
class LowPass_HoverThrustEstimator : public HoverThrustEstimator {
  public:
    // 低通法仅支持在悬停状态下估计，外层应只在悬停时调用 update()
    explicit LowPass_HoverThrustEstimator(){
        onlyhover_estimate_ = true;
    }

    void update(const Input_t& input) override;
    double get_hover_thrust() const override;

  private:
    ros::Time last_stamp_{ros::Time(0)};
    double adapt_tau_s_{5.0};
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

    void update(const Input_t& input) override;
    double get_hover_thrust() const override;

  private:
    std::deque<Input_t> input_history_;

    const double rho2_ = 0.998;  // 遗忘因子，决定动态性能
    double thr2acc_; // thr2acc_ = param_.gravity / param_.hover_thrust;
    double P_ = 1e6;
};

// ─────────────────────────────────────────────────────────────────────────
// 卡尓曼滤波 悬停推力估计器
// ─────────────────────────────────────────────────────────────────────────
class Kalman_HoverThrustEstimator : public HoverThrustEstimator {
  public:
    // Kalman 估计仅支持在悬停状态下估计，外层应只在悬停时调用 update()
    explicit Kalman_HoverThrustEstimator(){
        onlyhover_estimate_ = true;
    }

    void update(const Input_t& input) override;
    double get_hover_thrust() const override;

  private:
    ros::Time last_stamp_{ros::Time(0)};
    double Q_{1e-4};
    double R_{1e-2};
    double P_{1e-1};
};
};  // namespace thrust_estimator
