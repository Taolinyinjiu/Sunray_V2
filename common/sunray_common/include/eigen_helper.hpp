#pragma once

#include "Eigen/Dense"

namespace eigen_helper{

inline double get_yaw_from_orientation(const Eigen::Quaterniond& orientation) {
    return atan2(2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
                 1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
}

inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
    Eigen::Vector4d quat;
    double tr = R.trace();
    if (tr > 0.0){
        double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
        quat(0) = 0.25 * S;
        quat(1) = (R(2, 1) - R(1, 2)) / S;
        quat(2) = (R(0, 2) - R(2, 0)) / S;
        quat(3) = (R(1, 0) - R(0, 1)) / S;
    }
    else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))){
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
        quat(0) = (R(2, 1) - R(1, 2)) / S;
        quat(1) = 0.25 * S;
        quat(2) = (R(0, 1) + R(1, 0)) / S;
        quat(3) = (R(0, 2) + R(2, 0)) / S;
    }
    else if (R(1, 1) > R(2, 2)){
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
        quat(0) = (R(0, 2) - R(2, 0)) / S;
        quat(1) = (R(0, 1) + R(1, 0)) / S;
        quat(2) = 0.25 * S;
        quat(3) = (R(1, 2) + R(2, 1)) / S;
    }
    else{
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
        quat(0) = (R(1, 0) - R(0, 1)) / S;
        quat(1) = (R(0, 2) + R(2, 0)) / S;
        quat(2) = (R(1, 2) + R(2, 1)) / S;
        quat(3) = 0.25 * S;
    }
    return quat;
}
}
