#pragma once

#include <Eigen/Dense>
#include <ros/time.h>

namespace takeoff_land {

struct RCThrustFilterState {
    bool initialized{false};
    double thrust{0.0};
    ros::Time last_update{ros::Time(0)};
};

struct TakeoffTuning {
    double idle_thrust{0.08};
    double openloop_boost{0.025};
    double liftoff_detect_height_m{0.03};
    double liftoff_detect_vz_mps{0.08};
    double ramp_time{1.5};
    double thrust_tau{0.30};
};

struct TakeoffState {
    double yaw{0.0};
    ros::Time arm_time{ros::Time(0)};
    bool curve_started{false};
    RCThrustFilterState thrust_filter{};

    void reset() {
        yaw = 0.0;
        arm_time = ros::Time(0);
        curve_started = false;
        thrust_filter = RCThrustFilterState{};
    }
};

struct LandingTuning {
    double slow_height_m{0.35};
    double near_ground_height_m{0.18};
    double near_ground_velocity{0.10};
    double touchdown_velocity{0.04};
    double position_lookahead_time{0.35};
    double slow_release_tau{0.90};
    double fast_release_tau{0.12};
    double touchdown_detect_height_m{0.20};
    double fast_release_height_m{0.05};
    double odom_settle_threshold_m{0.05};
    double odom_settle_window_s{0.15};
    double soft_touchdown_thrust_ratio{0.90};
    double shutdown_thrust{0.02};
};

struct LandingState {
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    Eigen::Vector3d fast_release_reference_position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double max_velocity{0.30};
    ros::Time start_time{ros::Time(0)};
    ros::Time slow_release_start_time{ros::Time(0)};
    ros::Time fast_release_reference_time{ros::Time(0)};
    ros::Time fast_release_start_time{ros::Time(0)};
    bool slow_release_started{false};
    bool fast_release_started{false};
    RCThrustFilterState thrust_filter{};

    void reset() {
        point.setZero();
        fast_release_reference_position.setZero();
        yaw = 0.0;
        max_velocity = 0.30;
        start_time = ros::Time(0);
        slow_release_start_time = ros::Time(0);
        fast_release_reference_time = ros::Time(0);
        fast_release_start_time = ros::Time(0);
        slow_release_started = false;
        fast_release_started = false;
        thrust_filter = RCThrustFilterState{};
    }
};

}  // namespace takeoff_land
