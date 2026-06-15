#pragma once

#include <Eigen/Geometry>
#include <cmath>

/// @brief 弧度转角度
/// @param rad 弧度值
/// @return 角度值
inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

/// @brief 角度转弧度
/// @param deg 角度值
/// @return 弧度值
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

/// @brief 对角度（弧度）做归一化，映射到 [-pi, pi]
/// @param rad 输入弧度
/// @return 归一化后的弧度
inline double normalize_angle_rad(double rad) {
    while (rad > M_PI) {
        rad -= 2.0 * M_PI;
    }
    while (rad < -M_PI) {
        rad += 2.0 * M_PI;
    }
    return rad;
}

/// @brief 对角度（度）做归一化，映射到 [-180, 180]
/// @param deg 输入角度
/// @return 归一化后的角度
inline double normalize_angle_deg(double deg) {
    while (deg > 180.0) {
        deg -= 360.0;
    }
    while (deg < -180.0) {
        deg += 360.0;
    }
    return deg;
}

/// @brief 由四元数提取偏航角（弧度）
/// @param q 姿态四元数（w, x, y, z）
/// @return 偏航角（弧度）
inline double quaternion_to_yaw_rad(const Eigen::Quaterniond& q) {
    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

/// @brief 由四元数提取偏航角（角度）
/// @param q 姿态四元数（w, x, y, z）
/// @return 偏航角（度）
inline double quaternion_to_yaw_deg(const Eigen::Quaterniond& q) {
    return rad2deg(quaternion_to_yaw_rad(q));
}
