/**
 * @file arrival_helper.hpp
 * @brief 到达稳定判定辅助工具
 */
#pragma once

#include <ros/time.h>

namespace arrival_helper {
// 配置参数结构体
struct Config {
    double stable_time_s{0.5};                // 位置误差与速度误差都满足时判断稳定时间
    double pos_err_m{0.15};                   // 位置误差
    double vel_err_mps{0.15};                 // 速度误差
    double max_pos_err_m{0.0};                // 最大位置误差
    double vel_only_long_time_factor{3.0};    // 满足最大位置误差和速度误差时的判断稳定时间
    double vel_only_force_time_factor{10.0};  // 只满足速度误差时的判断稳定时间
};
// 到达状态结构体
struct State {
    ros::Time both_ok_since{ros::Time(0)};
    ros::Time vel_ok_since{ros::Time(0)};
};

// 内联函数，更新状态变量并检测是否达到
inline bool update_and_check(State& state,
                             const Config& config,
                             double pos_err,
                             double vel_err,
                             const ros::Time& now) {
    // 更新位置误差与速度误差是否满足
    const bool pos_ok = pos_err < config.pos_err_m;
    const bool vel_ok = vel_err < config.vel_err_mps;
    // 如果位置误差与速度误差都满足
    if (pos_ok && vel_ok) {
        // 并且此时的both状态变量还未进行赋值
        if (state.both_ok_since == ros::Time(0)) {
            // 则置位为当前值
            state.both_ok_since = now;
        }
    } else {
        // 如果不能同时满足，则重置both状态变量
        state.both_ok_since = ros::Time(0);
    }
    // 如果速度误差满足
    if (vel_ok) {
        // 并且当前状态中速度误差满足的时间戳还未置位,则置为当前时间戳
        if (state.vel_ok_since == ros::Time(0)) {
            state.vel_ok_since = now;
        }
    } else {
        // 反之,则重置
        state.vel_ok_since = ros::Time(0);
    }
    // 根据到位设计的理念,决定当前如何输出到位状态
    const bool both_ok_long_enough = state.both_ok_since != ros::Time(0) &&
                                     (now - state.both_ok_since).toSec() >= config.stable_time_s;

    const bool vel_only_within_max_pos =
        config.max_pos_err_m > 0.0 && pos_err < config.max_pos_err_m &&
        state.vel_ok_since != ros::Time(0) &&
        (now - state.vel_ok_since).toSec() >=
            config.vel_only_long_time_factor * config.stable_time_s;

    const bool vel_only_force_complete =
        state.vel_ok_since != ros::Time(0) &&
        (now - state.vel_ok_since).toSec() >=
            config.vel_only_force_time_factor * config.stable_time_s;

    return both_ok_long_enough || vel_only_within_max_pos || vel_only_force_complete;
}

}  // namespace arrival_helper
