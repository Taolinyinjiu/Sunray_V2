#pragma once

#include <chrono>
#include <cstdint>

#include <ros/time.h>

namespace sunray_logger {

inline uint64_t rosTimeToUs(const ros::Time& time) {
    return static_cast<uint64_t>(time.sec) * 1000000ULL + static_cast<uint64_t>(time.nsec) / 1000ULL;
}

inline uint64_t rosWallTimeToUs(const ros::WallTime& time) {
    return static_cast<uint64_t>(time.sec) * 1000000ULL + static_cast<uint64_t>(time.nsec) / 1000ULL;
}

inline uint64_t rosSteadyTimeToUs(const ros::SteadyTime& time) {
    return static_cast<uint64_t>(time.sec) * 1000000ULL + static_cast<uint64_t>(time.nsec) / 1000ULL;
}

inline uint64_t steadyTimeToUs() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

inline uint64_t wallTimeNowToUs() {
    return rosWallTimeToUs(ros::WallTime::now());
}

}  // namespace sunray_logger
