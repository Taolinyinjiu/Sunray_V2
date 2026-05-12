#pragma once

#include "utils/orientation_utils.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <ros/time.h>

namespace curve {

struct YawCurveState {
    bool valid{false};
    double yaw{0.0};
    double yaw_rate{0.0};
    double yaw_acceleration{0.0};
};

class QuinticYawCurve {
  public:
    void set_start_yawpoint(double yaw_rad, double yaw_rate_rad_s);
    void set_end_yawpoint(double yaw_rad, double yaw_rate_rad_s);
    void set_curve_time(double time_s);
    void set_curve_avgvel(double yaw_rate_avg_rad_s);
    void set_curve_maxvel(double yaw_rate_limit_rad_s);
    void clear();
    bool is_ready() const;
    bool matches_target(double yaw_rad, double eps = 1e-6) const;
    YawCurveState get_result(const ros::Time& now);

  private:
    static std::array<double, 6>
    solve_quintic_1d(double p0, double v0, double p1, double v1, double T);
    static double eval_pos_1d(const std::array<double, 6>& c, double t);
    static double eval_vel_1d(const std::array<double, 6>& c, double t);
    static double eval_acc_1d(const std::array<double, 6>& c, double t);
    double estimate_max_speed(double T, int samples = 200) const;
    bool solve_time_from_maxvel(double vmax_limit, double& T_out) const;
    static double unwrap_target_yaw(double start_yaw, double target_yaw);

    YawCurveState result_{};
    ros::Time start_time_{ros::Time(0)};
    double start_yaw_{0.0};
    double start_yaw_rate_{0.0};
    double stop_yaw_{0.0};
    double stop_yaw_rate_{0.0};
    double curve_time_{0.0};
    double curve_avgvel_{0.0};
    double curve_maxvel_{0.0};
    bool use_time_constraint_{false};
    bool use_avgvel_constraint_{false};
    bool use_vel_constraint_{false};
    double commanded_target_yaw_{0.0};
    bool has_commanded_target_{false};
};

inline void QuinticYawCurve::set_start_yawpoint(double yaw_rad, double yaw_rate_rad_s) {
    start_yaw_ = yaw_rad;
    start_yaw_rate_ = yaw_rate_rad_s;
}

inline void QuinticYawCurve::set_end_yawpoint(double yaw_rad, double yaw_rate_rad_s) {
    commanded_target_yaw_ = normalize_angle_rad(yaw_rad);
    has_commanded_target_ = true;
    stop_yaw_ = unwrap_target_yaw(start_yaw_, yaw_rad);
    stop_yaw_rate_ = yaw_rate_rad_s;
}

inline void QuinticYawCurve::set_curve_time(double time_s) {
    use_time_constraint_ = true;
    use_avgvel_constraint_ = false;
    use_vel_constraint_ = false;
    curve_time_ = time_s;
}

inline void QuinticYawCurve::set_curve_avgvel(double yaw_rate_avg_rad_s) {
    use_time_constraint_ = false;
    use_avgvel_constraint_ = true;
    use_vel_constraint_ = false;
    curve_avgvel_ = yaw_rate_avg_rad_s;
}

inline void QuinticYawCurve::set_curve_maxvel(double yaw_rate_limit_rad_s) {
    use_time_constraint_ = false;
    use_avgvel_constraint_ = false;
    use_vel_constraint_ = true;
    curve_maxvel_ = yaw_rate_limit_rad_s;
}

inline void QuinticYawCurve::clear() {
    result_ = YawCurveState{};
    start_time_ = ros::Time(0);
    start_yaw_ = 0.0;
    start_yaw_rate_ = 0.0;
    stop_yaw_ = 0.0;
    stop_yaw_rate_ = 0.0;
    curve_time_ = 0.0;
    curve_avgvel_ = 0.0;
    curve_maxvel_ = 0.0;
    use_time_constraint_ = false;
    use_avgvel_constraint_ = false;
    use_vel_constraint_ = false;
    commanded_target_yaw_ = 0.0;
    has_commanded_target_ = false;
}

inline bool QuinticYawCurve::is_ready() const {
    return use_time_constraint_ || use_avgvel_constraint_ || use_vel_constraint_;
}

inline bool QuinticYawCurve::matches_target(double yaw_rad, double eps) const {
    return has_commanded_target_ &&
           std::abs(normalize_angle_rad(commanded_target_yaw_ - normalize_angle_rad(yaw_rad))) <= eps;
}

inline std::array<double, 6>
QuinticYawCurve::solve_quintic_1d(double p0, double v0, double p1, double v1, double T) {
    std::array<double, 6> c{};
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    c[0] = p0;
    c[1] = v0;
    c[2] = 0.0;

    const double dp = p1 - (p0 + v0 * T);
    const double dv = v1 - v0;

    Eigen::Matrix3d A;
    A << T3, T4, T5, 3.0 * T2, 4.0 * T3, 5.0 * T4, 6.0 * T, 12.0 * T2, 20.0 * T3;
    Eigen::Vector3d b(dp, dv, 0.0);
    const Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

    c[3] = x(0);
    c[4] = x(1);
    c[5] = x(2);
    return c;
}

inline double QuinticYawCurve::eval_pos_1d(const std::array<double, 6>& c, double t) {
    return c[0] + c[1] * t + c[2] * t * t + c[3] * t * t * t + c[4] * t * t * t * t +
           c[5] * t * t * t * t * t;
}

inline double QuinticYawCurve::eval_vel_1d(const std::array<double, 6>& c, double t) {
    return c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t * t + 4.0 * c[4] * t * t * t +
           5.0 * c[5] * t * t * t * t;
}

inline double QuinticYawCurve::eval_acc_1d(const std::array<double, 6>& c, double t) {
    return 2.0 * c[2] + 6.0 * c[3] * t + 12.0 * c[4] * t * t + 20.0 * c[5] * t * t * t;
}

inline double QuinticYawCurve::estimate_max_speed(double T, int samples) const {
    const auto c = solve_quintic_1d(start_yaw_, start_yaw_rate_, stop_yaw_, stop_yaw_rate_, T);
    double vmax = 0.0;
    for (int i = 0; i <= samples; ++i) {
        const double t = T * static_cast<double>(i) / static_cast<double>(samples);
        vmax = std::max(vmax, std::abs(eval_vel_1d(c, t)));
    }
    return vmax;
}

inline bool QuinticYawCurve::solve_time_from_maxvel(double vmax_limit, double& T_out) const {
    constexpr double kEps = 1e-6;
    constexpr double kTMin = 0.05;
    constexpr double kTMax = 120.0;

    if (vmax_limit <= kEps) {
        return false;
    }

    const double dist = std::abs(stop_yaw_ - start_yaw_);
    double T_low = std::max(kTMin, 0.2 * dist / std::max(vmax_limit, kEps));
    double T_high = std::max(kTMin, dist / std::max(vmax_limit, kEps));

    double v_high = estimate_max_speed(T_high);
    int expand_iter = 0;
    while (v_high > vmax_limit && T_high < kTMax && expand_iter < 40) {
        T_low = T_high;
        T_high *= 1.8;
        v_high = estimate_max_speed(T_high);
        ++expand_iter;
    }
    if (v_high > vmax_limit) {
        return false;
    }

    double v_low = estimate_max_speed(T_low);
    if (v_low <= vmax_limit) {
        for (int i = 0; i < 20; ++i) {
            const double T_try = std::max(kTMin, T_low * 0.7);
            const double v_try = estimate_max_speed(T_try);
            if (v_try <= vmax_limit) {
                T_low = T_try;
                v_low = v_try;
            } else {
                break;
            }
        }
    }

    for (int i = 0; i < 60; ++i) {
        const double T_mid = 0.5 * (T_low + T_high);
        const double v_mid = estimate_max_speed(T_mid);
        if (v_mid > vmax_limit) {
            T_low = T_mid;
        } else {
            T_high = T_mid;
        }
    }

    T_out = T_high;
    return true;
}

inline double QuinticYawCurve::unwrap_target_yaw(double start_yaw, double target_yaw) {
    return start_yaw + normalize_angle_rad(target_yaw - start_yaw);
}

inline YawCurveState QuinticYawCurve::get_result(const ros::Time& now) {
    result_ = YawCurveState{};
    if (start_time_ == ros::Time(0)) {
        start_time_ = now;
    }

    if (use_avgvel_constraint_) {
        constexpr double kEps = 1e-6;
        constexpr double kMinCurveTime = 0.05;
        if (curve_avgvel_ <= kEps) {
            return result_;
        }
        curve_time_ = std::max(kMinCurveTime, std::abs(stop_yaw_ - start_yaw_) / curve_avgvel_);
        use_time_constraint_ = true;
        use_avgvel_constraint_ = false;
    }

    if (use_vel_constraint_) {
        double solved_T = 0.0;
        if (!solve_time_from_maxvel(curve_maxvel_, solved_T)) {
            return result_;
        }
        curve_time_ = solved_T;
        use_time_constraint_ = true;
        use_vel_constraint_ = false;
    }

    if (!use_time_constraint_ || curve_time_ <= 1e-6) {
        return result_;
    }

    const double dt = (now - start_time_).toSec();
    const double t = std::clamp(dt, 0.0, curve_time_);
    const auto c =
        solve_quintic_1d(start_yaw_, start_yaw_rate_, stop_yaw_, stop_yaw_rate_, curve_time_);

    result_.yaw = normalize_angle_rad(eval_pos_1d(c, t));
    result_.yaw_rate = eval_vel_1d(c, t);
    result_.yaw_acceleration = eval_acc_1d(c, t);
    result_.valid = true;
    return result_;
}

}  // namespace curve
