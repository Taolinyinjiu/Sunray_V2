#pragma once

#include <ros/time.h>

namespace arrival_helper {

struct Config {
    double stable_time_s{0.5};
    double pos_err_m{0.15};
    double vel_err_mps{0.15};
    double vel_only_max_pos_err_m{0.0};
    bool require_pos_ok_before_vel_only{false};
};

struct State {
    ros::Time both_ok_since{ros::Time(0)};
    ros::Time vel_ok_since{ros::Time(0)};
    bool has_seen_pos_ok{false};
};

inline bool update_and_check(State& state,
                             const Config& config,
                             double pos_err,
                             double vel_err,
                             const ros::Time& now) {
    const bool pos_ok = pos_err < config.pos_err_m;
    const bool vel_ok = vel_err < config.vel_err_mps;

    if (pos_ok) {
        state.has_seen_pos_ok = true;
    }

    if (pos_ok && vel_ok) {
        if (state.both_ok_since == ros::Time(0)) {
            state.both_ok_since = now;
        }
    } else {
        state.both_ok_since = ros::Time(0);
    }

    const bool vel_only_pos_ok = config.vel_only_max_pos_err_m <= 0.0 ||
                                 pos_err < config.vel_only_max_pos_err_m;
    const bool vel_only_armed = !config.require_pos_ok_before_vel_only || state.has_seen_pos_ok;
    if (vel_ok && vel_only_pos_ok && vel_only_armed) {
        if (state.vel_ok_since == ros::Time(0)) {
            state.vel_ok_since = now;
        }
    } else {
        state.vel_ok_since = ros::Time(0);
    }

    return (state.both_ok_since != ros::Time(0) &&
            (now - state.both_ok_since).toSec() >= config.stable_time_s) ||
           (state.vel_ok_since != ros::Time(0) &&
            (now - state.vel_ok_since).toSec() >= 3.0 * config.stable_time_s);
}

}  // namespace arrival_helper
