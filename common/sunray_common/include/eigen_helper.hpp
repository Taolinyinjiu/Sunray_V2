#pragma once

#include "Eigen/Dense"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

namespace eigen_helper{

inline double get_yaw_from_orientation(const Eigen::Quaterniond& orientation) {
    return std::atan2(2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
                      1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
}

inline Eigen::Vector3d get_euler_from_orientation(const Eigen::Quaterniond& q) {
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    double pitch;
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return Eigen::Vector3d(roll, pitch, yaw);
}

inline double wrap_angle(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

inline geometry_msgs::Point to_ros_point(const Eigen::Vector3d& vector) {
    geometry_msgs::Point point;
    point.x = vector.x();
    point.y = vector.y();
    point.z = vector.z();
    return point;
}

inline geometry_msgs::Vector3 to_ros_vector3(const Eigen::Vector3d& vector) {
    geometry_msgs::Vector3 msg;
    msg.x = vector.x();
    msg.y = vector.y();
    msg.z = vector.z();
    return msg;
}

inline geometry_msgs::Quaternion to_ros_quaternion(const Eigen::Quaterniond& quat) {
    geometry_msgs::Quaternion msg;
    msg.w = quat.w();
    msg.x = quat.x();
    msg.y = quat.y();
    msg.z = quat.z();
    return msg;
}

inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
    Eigen::Vector4d quat;
    double tr = R.trace();
    if (tr > 0.0){
        double S = std::sqrt(tr + 1.0) * 2.0;  // S=4*qw
        quat(0) = 0.25 * S;
        quat(1) = (R(2, 1) - R(1, 2)) / S;
        quat(2) = (R(0, 2) - R(2, 0)) / S;
        quat(3) = (R(1, 0) - R(0, 1)) / S;
    }
    else if ((R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2))){
        double S = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
        quat(0) = (R(2, 1) - R(1, 2)) / S;
        quat(1) = 0.25 * S;
        quat(2) = (R(0, 1) + R(1, 0)) / S;
        quat(3) = (R(0, 2) + R(2, 0)) / S;
    }
    else if (R(1, 1) > R(2, 2)){
        double S = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
        quat(0) = (R(0, 2) - R(2, 0)) / S;
        quat(1) = (R(0, 1) + R(1, 0)) / S;
        quat(2) = 0.25 * S;
        quat(3) = (R(1, 2) + R(2, 1)) / S;
    }
    else{
        double S = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
        quat(0) = (R(1, 0) - R(0, 1)) / S;
        quat(1) = (R(0, 2) + R(2, 0)) / S;
        quat(2) = (R(1, 2) + R(2, 1)) / S;
        quat(3) = 0.25 * S;
    }
    return quat;
}
}
