/**
 * 几何控制器数学工具
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>

// 将三维向量转换为对应的 3×3 反对称矩阵（叉积算子，hat map）
static Eigen::Matrix3d matrix_hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    // Sanity checks on M
    m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
    return m;
}

// 从 3×3 反对称矩阵中提取对应的三维向量（hat map 的逆运算）
static Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d& m) {
    Eigen::Vector3d v;
    // TODO: Sanity checks if m is skew symmetric
    v << m(7), m(2), m(3);
    return v;
}

// 将 ROS geometry_msgs::Point 类型转换为 Eigen::Vector3d
inline Eigen::Vector3d toEigen(const geometry_msgs::Point& p) {
    Eigen::Vector3d ev3(p.x, p.y, p.z);
    return ev3;
}

// 将 ROS geometry_msgs::Vector3 类型转换为 Eigen::Vector3d
inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3& v3) {
    Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
    return ev3;
}

// 计算两个四元数的乘积，约定格式为 Vector4d(qw, qx, qy, qz)
inline Eigen::Vector4d quatMultiplication(const Eigen::Vector4d& q, const Eigen::Vector4d& p) {
    Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
        p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
        p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}

// 将四元数 Vector4d(qw, qx, qy, qz) 转换为对应的 3×3 旋转矩阵
inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d& q) {
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
        2 * q(1) * q(2) - 2 * q(0) * q(3), 2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
        2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
        q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}

// 将 3×3 旋转矩阵转换为四元数 Vector4d(qw, qx, qy, qz)，使用 Shepperd 方法避免数值奇异
inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d& R) {
    Eigen::Vector4d quat;
    double tr = R.trace();
    if (tr > 0.0) {
        double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
        quat(0) = 0.25 * S;
        quat(1) = (R(2, 1) - R(1, 2)) / S;
        quat(2) = (R(0, 2) - R(2, 0)) / S;
        quat(3) = (R(1, 0) - R(0, 1)) / S;
    } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
        quat(0) = (R(2, 1) - R(1, 2)) / S;
        quat(1) = 0.25 * S;
        quat(2) = (R(0, 1) + R(1, 0)) / S;
        quat(3) = (R(0, 2) + R(2, 0)) / S;
    } else if (R(1, 1) > R(2, 2)) {
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
        quat(0) = (R(0, 2) - R(2, 0)) / S;
        quat(1) = (R(0, 1) + R(1, 0)) / S;
        quat(2) = 0.25 * S;
        quat(3) = (R(1, 2) + R(2, 1)) / S;
    } else {
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
        quat(0) = (R(1, 0) - R(0, 1)) / S;
        quat(1) = (R(0, 2) + R(2, 0)) / S;
        quat(2) = (R(1, 2) + R(2, 1)) / S;
        quat(3) = 0.25 * S;
    }
    return quat;
}
