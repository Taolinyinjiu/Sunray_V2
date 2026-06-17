#pragma once

#include <Eigen/Dense>
#include <ros/time.h>
#include <algorithm>
#include <cstdint>

namespace takeoff_land {

// ─────────────────────────────────────────────────────────────────────────────
// Sunray 托管的几何控制起降使用 AccFF 数据结构。
// 设计要点见 ai_result/accff_takeoff_landing_todo.md (v2)
// ─────────────────────────────────────────────────────────────────────────────

enum class TakeoffPhaseAccFF : uint8_t { PreLift = 0, AirborneCurve = 1, Hold = 2 };
enum class LandingPhaseAccFF : uint8_t { HighDescent = 0, NearGround = 1, TouchdownRelease = 2 };
enum class LandingNearGroundTrigger : uint8_t { None = 0, Height = 1, GroundEffectStall = 2 };

struct TakeoffAccFFTuning {
    double a_start_mps2{2.0};
    double a_target_mps2{10.3};
    double ramp_time_s{1.2};
    double jerk_max_mps3{8.0};
    double a_min_mps2{0.0};
    double a_max_mps2{15.0};
    double hover_thrust_reference{0.375};
    double hover_thrust_gain_min{0.65};
    double thrust_margin{0.04};
    double odom_ahead_pos_tolerance_m{0.04};
    double odom_ahead_vz_tolerance_mps{0.05};
    double odom_thrust_scale_min{0.72};
};

struct TakeoffAccFFState {
    TakeoffPhaseAccFF phase{TakeoffPhaseAccFF::PreLift};
    ros::Time phase_start{ros::Time(0)};
    ros::Time last_update{ros::Time(0)};
    double a_ff_prev{0.0};
    double yaw{0.0};
    bool curve_started{false};

    void reset() {
        phase = TakeoffPhaseAccFF::PreLift;
        phase_start = ros::Time(0);
        last_update = ros::Time(0);
        a_ff_prev = 0.0;
        yaw = 0.0;
        curve_started = false;
    }
};

struct LandingAccFFTuning {
    double near_ground_h_m{0.10};
    double near_ground_vz_mps{0.10};
    double a_touchdown_mps2{5.4};
    double ramp_time_s{0.6};
    double jerk_max_mps3{8.0};
    double a_min_mps2{0.0};
    double a_max_mps2{12.0};
    bool   touchdown_landed_state{true};
    double touchdown_h_settle_m{0.05};
    double touchdown_v_settle_mps{0.05};
    double touchdown_dwell_s{0.30};
    double ground_effect_release_h_m{0.25};
    double ground_effect_stall_vz_mps{0.05};
    double ground_effect_dwell_s{0.40};
    double xy_slow_error_m{0.12};
    double xy_hold_error_m{0.25};
    double xy_hold_timeout_s{1.50};
};

struct LandingAccFFState {
    LandingPhaseAccFF phase{LandingPhaseAccFF::HighDescent};
    LandingNearGroundTrigger near_ground_trigger{LandingNearGroundTrigger::None};
    ros::Time phase_start{ros::Time(0)};
    ros::Time last_update{ros::Time(0)};
    ros::Time settle_start{ros::Time(0)};
    ros::Time touchdown_start{ros::Time(0)};
    ros::Time ground_effect_stall_start{ros::Time(0)};
    ros::Time xy_hold_start{ros::Time(0)};
    Eigen::Vector3d locked_xy_z{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double max_velocity{0.30};
    double anchor_thrust{0.0};
    double a_ff_prev{9.8};
    double near_ground_entry_h_m{0.10};
    double release_progress{1.0};
    double xy_error_m{0.0};
    double desc_speed_mps{0.0};
    double a_ff_now_mps2{0.0};
    double thrust_release_limit{0.0};
    double setpoint_thrust{0.0};
    double height_above_ground_m{0.0};
    bool entered{false};

    void reset() {
        phase = LandingPhaseAccFF::HighDescent;
        near_ground_trigger = LandingNearGroundTrigger::None;
        phase_start = ros::Time(0);
        last_update = ros::Time(0);
        settle_start = ros::Time(0);
        touchdown_start = ros::Time(0);
        ground_effect_stall_start = ros::Time(0);
        xy_hold_start = ros::Time(0);
        locked_xy_z.setZero();
        yaw = 0.0;
        max_velocity = 0.30;
        anchor_thrust = 0.0;
        a_ff_prev = 9.8;
        near_ground_entry_h_m = 0.10;
        release_progress = 1.0;
        xy_error_m = 0.0;
        desc_speed_mps = 0.0;
        a_ff_now_mps2 = 0.0;
        thrust_release_limit = 0.0;
        setpoint_thrust = 0.0;
        height_above_ground_m = 0.0;
        entered = false;
    }
};

// jerk-bounded S-curve 单步推进:a_prev 朝 a_target 移动一步,
// 单步幅值不超过 jerk_max*dt,最后 clamp 到 [a_min, a_max]。
// dt 由调用方按 last_update 自管;异常 dt 直接返回 clamp(a_target)。
inline double advance_s_curve_acc(double a_prev,
                                  double a_target,
                                  double dt,
                                  double jerk_max,
                                  double a_min,
                                  double a_max) {
    if (!(dt > 0.0) || !(jerk_max > 0.0)) {
        return std::clamp(a_target, a_min, a_max);
    }
    const double max_step = jerk_max * dt;
    const double delta = std::clamp(a_target - a_prev, -max_step, max_step);
    return std::clamp(a_prev + delta, a_min, a_max);
}

}  // namespace takeoff_land
