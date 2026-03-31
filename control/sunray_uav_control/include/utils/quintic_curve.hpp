#pragma once

#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <cmath>
#include <ros/time.h>

namespace curve {

// 五次多项式曲线计算结果
struct QuinticCurveState {
    bool valid{false};                                      ///< 输入与计算结果是否有效
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};      ///< 位置 p(t), m
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};      ///< 速度 v(t), m/s
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};  ///< 加速度 a(t), m/s^2
};
// 轨迹计算的约束类型
enum class ConstarintType { Undefined = 0, Time, Vel };

class QuinticCurve {
  public:
    ~QuinticCurve() = default;
    // 得到轨迹的计算结果
    QuinticCurveState get_result();
    // 设置起点和终点
    void set_start_trajpoint(Eigen::Vector3d pos, Eigen::Vector3d vel);
    void set_end_trajpoint(Eigen::Vector3d pos, Eigen::Vector3d vel);
    Eigen::Vector3d get_start_position();
    Eigen::Vector3d get_end_position();
    // 设置约束项，1. 运动时间 2.最大运动速度 任选其一
    void set_curve_time(double time);
    void set_curve_maxvel(double vel);
    // 重置曲线参数
    void clear();
    // 提供曲线状态查询接口
    bool is_ready();

  private:
    // 根据一维运动的起点/终点位置与速度，以及给定总时间T，求一条五次多项式的系数。
    // 公式： p(t)=c0+c1 t+c2 t^2+c3 t^3+c4 t^4+c5 t^5
    // 默认起点与终点加速度为0
    static std::array<double, 6>
    solve_quintic_1d(double p0, double v0, double p1, double v1, double T);
    // 求解单轴位置
    static double eval_pos_1d(const std::array<double, 6>& c, double t);
    // 求解单轴速度
    static double eval_vel_1d(const std::array<double, 6>& c, double t);
    // 求解单轴加速度
    static double eval_acc_1d(const std::array<double, 6>& c, double t);
    // 根据输入的时间T，估计三维五次项曲线的最大规划速度
    double estimate_max_speed(double T, int samples = 200) const;
    // 根据最大速度限制，求解五次项曲线的最小时间T
    bool solve_time_from_maxvel(double vmax_limit, double& T_out) const;
    // --------------------缓存变量-------------------
    QuinticCurveState result_;
    ros::Time start_time_{ros::Time(0)};
    Eigen::Vector3d start_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d start_velocity_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d stop_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d stop_velocity_{Eigen::Vector3d::Zero()};
    ConstarintType curve_constraint_type_ = ConstarintType::Undefined;
    double curve_time_{0.0};
    double curve_maxvel_{0.0};
};
// 设置起点的位置和速度
inline void QuinticCurve::set_start_trajpoint(Eigen::Vector3d pos, Eigen::Vector3d vel) {
    start_position_ = pos;
    start_velocity_ = vel;
}
// 得到起点的位置
inline Eigen::Vector3d QuinticCurve::get_start_position() {
    return start_position_;
}
// 设置终点的位置和速度
inline void QuinticCurve::set_end_trajpoint(Eigen::Vector3d pos, Eigen::Vector3d vel) {
    stop_position_ = pos;
    stop_velocity_ = vel;
}
// 得到终点的位置
inline Eigen::Vector3d QuinticCurve::get_end_position() {
    return stop_position_;
}
// 设置轨迹的时间
inline void QuinticCurve::set_curve_time(double time) {
    curve_constraint_type_ = ConstarintType::Time;
    curve_time_ = time;
}
// 设置轨迹的最大速度
inline void QuinticCurve::set_curve_maxvel(double vel) {
    curve_constraint_type_ = ConstarintType::Vel;
    curve_maxvel_ = vel;
}
// 求解单轴的位置
inline double QuinticCurve::eval_pos_1d(const std::array<double, 6>& c, double t) {
    return c[0] + c[1] * t + c[2] * t * t + c[3] * t * t * t + c[4] * t * t * t * t +
           c[5] * t * t * t * t * t;
}
// 求解单轴速度
inline double QuinticCurve::eval_vel_1d(const std::array<double, 6>& c, double t) {
    return c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t * t + 4.0 * c[4] * t * t * t +
           5.0 * c[5] * t * t * t * t;
}
// 求解单轴加速度
inline double QuinticCurve::eval_acc_1d(const std::array<double, 6>& c, double t) {
    return 2.0 * c[2] + 6.0 * c[3] * t + 12.0 * c[4] * t * t + 20.0 * c[5] * t * t * t;
}
// 清除参数
inline void QuinticCurve::clear() {
    start_time_ = ros::Time(0);
    start_position_.setZero();
    start_velocity_.setZero();
    stop_position_.setZero();
    stop_velocity_.setZero();
    curve_constraint_type_ = ConstarintType::Undefined;
    curve_time_ = 0.0;
    curve_maxvel_ = 0.0;
    result_ = QuinticCurveState{};
}

inline bool QuinticCurve::is_ready() {
    return start_time_ != ros::Time(0);
}

inline QuinticCurveState QuinticCurve::get_result() {
    result_.valid = false;
    // 首先判断是否为初次进入函数
    if (start_time_ == ros::Time(0)) {
        start_time_ = ros::Time::now();  // 用当前时间戳作为开始时间戳
    }

    // Vel constraint -> solve curve_time_ once (or each call if start/end changed externally)
    if (curve_constraint_type_ ==
        ConstarintType::Vel) {  // 如果约束的类型是最大速度，其实是使用最大速度求解时间
        double solved_T = 0.0;
        if (!solve_time_from_maxvel(curve_maxvel_, solved_T)) {
            return result_;
        }
        curve_time_ = solved_T;
        // 得到运动时间，修改为Time
        curve_constraint_type_ = ConstarintType::Time;
    }

    if (curve_constraint_type_ != ConstarintType::Time || curve_time_ <= 1e-6) {
        return result_;
    }

    const double dt = (ros::Time::now() - start_time_).toSec();
    const double t = std::min(std::max(0.0, dt), curve_time_);
    const double T = curve_time_;

    const auto cx = solve_quintic_1d(
        start_position_.x(), start_velocity_.x(), stop_position_.x(), stop_velocity_.x(), T);
    const auto cy = solve_quintic_1d(
        start_position_.y(), start_velocity_.y(), stop_position_.y(), stop_velocity_.y(), T);
    const auto cz = solve_quintic_1d(
        start_position_.z(), start_velocity_.z(), stop_position_.z(), stop_velocity_.z(), T);

    result_.position.x() = eval_pos_1d(cx, t);
    result_.position.y() = eval_pos_1d(cy, t);
    result_.position.z() = eval_pos_1d(cz, t);

    result_.velocity.x() = eval_vel_1d(cx, t);
    result_.velocity.y() = eval_vel_1d(cy, t);
    result_.velocity.z() = eval_vel_1d(cz, t);

    result_.acceleration.x() = eval_acc_1d(cx, t);
    result_.acceleration.y() = eval_acc_1d(cy, t);
    result_.acceleration.z() = eval_acc_1d(cz, t);

    result_.valid = true;
    return result_;
}

// 求解五次项曲线的参数
inline std::array<double, 6>
QuinticCurve::solve_quintic_1d(double p0, double v0, double p1, double v1, double T) {
    std::array<double, 6> c{};
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    c[0] = p0;
    c[1] = v0;
    c[2] = 0.0;  // a0 = 0 => c2 = a0/2 = 0

    // From terminal constraints
    const double dp = p1 - (p0 + v0 * T);
    const double dv = v1 - v0;

    // [ T3    T4     T5 ] [c3] = [dp]
    // [3T2   4T3    5T4] [c4]   [dv]
    // [6T    12T2   20T3][c5]   [0 ]
    Eigen::Matrix3d A;
    A << T3, T4, T5, 3.0 * T2, 4.0 * T3, 5.0 * T4, 6.0 * T, 12.0 * T2, 20.0 * T3;

    Eigen::Vector3d b(dp, dv, 0.0);
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

    c[3] = x(0);
    c[4] = x(1);
    c[5] = x(2);
    return c;
}

// 根据输入的时间T，估计三维五次项曲线的最大规划速度
inline double QuinticCurve::estimate_max_speed(double T, int samples) const {
    const auto cx = solve_quintic_1d(
        start_position_.x(), start_velocity_.x(), stop_position_.x(), stop_velocity_.x(), T);
    const auto cy = solve_quintic_1d(
        start_position_.y(), start_velocity_.y(), stop_position_.y(), stop_velocity_.y(), T);
    const auto cz = solve_quintic_1d(
        start_position_.z(), start_velocity_.z(), stop_position_.z(), stop_velocity_.z(), T);

    double vmax = 0.0;
    for (int i = 0; i <= samples; ++i) {
        const double t = T * static_cast<double>(i) / static_cast<double>(samples);
        const double vx = eval_vel_1d(cx, t);
        const double vy = eval_vel_1d(cy, t);
        const double vz = eval_vel_1d(cz, t);
        const double v = std::sqrt(vx * vx + vy * vy + vz * vz);
        vmax = std::max(vmax, v);
    }
    return vmax;
}

// 根据最大速度限制，求解五次项曲线的最小时间T
inline bool QuinticCurve::solve_time_from_maxvel(double vmax_limit, double& T_out) const {
    constexpr double kEps = 1e-6;
    constexpr double kTMin = 0.05;
    constexpr double kTMax = 120.0;

    if (vmax_limit <= kEps)
        return false;

    const double dist = (stop_position_ - start_position_).norm();
    double T_low =
        std::max(kTMin, 0.2 * dist / std::max(vmax_limit, kEps));        // aggressive lower guess
    double T_high = std::max(kTMin, dist / std::max(vmax_limit, kEps));  // nominal guess

    // Ensure high is feasible
    double v_high = estimate_max_speed(T_high);
    int expand_iter = 0;
    while (v_high > vmax_limit && T_high < kTMax && expand_iter < 40) {
        T_low = T_high;
        T_high *= 1.8;
        v_high = estimate_max_speed(T_high);
        ++expand_iter;
    }
    if (v_high > vmax_limit)
        return false;  // cannot find feasible within limit

    // maybe low already feasible -> tighten low down a bit
    double v_low = estimate_max_speed(T_low);
    if (v_low <= vmax_limit) {
        // shrink to bracket infeasible/feasible if possible
        for (int i = 0; i < 20; ++i) {
            const double T_try = std::max(kTMin, T_low * 0.7);
            const double v_try = estimate_max_speed(T_try);
            if (v_try > vmax_limit) {
                T_low = T_try;
                break;
            }
            T_low = T_try;
        }
    }

    // bisection: want low infeasible, high feasible
    for (int i = 0; i < 50; ++i) {
        const double T_mid = 0.5 * (T_low + T_high);
        const double v_mid = estimate_max_speed(T_mid);
        if (v_mid > vmax_limit) {
            T_low = T_mid;
        } else {
            T_high = T_mid;
        }
        if (std::abs(T_high - T_low) < 1e-3)
            break;
    }

    T_out = T_high;  // feasible side
    return true;
}

}  // namespace curve
