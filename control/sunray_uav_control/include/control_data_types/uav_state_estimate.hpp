#pragma once

#include <cmath>
#include <string>

#include <Eigen/Dense>
#include "utils/geometry_eigen_conversions.hpp"
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

namespace uav_control {
/**
 * @brief 无人机状态估计数据容器（Odometry <-> Eigen 的轻量桥接类型）。
 *
 * 该结构体用于在控制模块内部统一保存无人机状态，并提供与
 * `nav_msgs::Odometry` 的双向转换能力。它不负责“估计算法”本身，
 * 仅负责状态数据组织与基础合法性校验。
 *
 * 状态语义约定：
 * - `position` / `velocity` 处于 `header.frame_id` 对应的坐标系；
 * - `bodyrates` 为机体系角速度（rad/s）；
 * - `orientation` 表示机体系相对世界系姿态（四元数）；
 * - `coordinate_frame` 为内部简化后的坐标系枚举。
 */
struct UAVStateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 默认构造函数。
     *
     * 初始化结果：
     * - `timestamp = ros::Time::now()`
     * - `coordinate_frame = INVALID`
     * - `child_frame_id = "body"`
     * - 线速度/位置/角速度置零，姿态置单位四元数
     */
    UAVStateEstimate();

    /**
     * @brief 从 ROS 里程计消息构造内部状态。
     * @param state_estimate_msg 输入里程计消息。
     *
     * 行为说明：
     * - 读取 `header.stamp` 到 `timestamp`；
     * - 解析 `header.frame_id` 到内部 `coordinate_frame`；
     * - 若 `child_frame_id` 为空则回退为 `"body"`；
     * - 若四元数分量均有限且范数足够大，则归一化处理。
     */
    UAVStateEstimate(const nav_msgs::Odometry& state_estimate_msg);

    /**
     * @brief 转换为 ROS 里程计消息。
     * @return `nav_msgs::Odometry` 消息副本。
     *
     * 输出规则：
     * - `WORLD -> "world"`，`LOCAL -> "local"`，`INVALID -> "invalid"`；
     * - `child_frame_id` 为空时回退 `"body"`；
     * - 其余字段按成员值直接填充。
     */
    nav_msgs::Odometry toRosMessage() const;

    /**
     * @brief 状态有效性检查。
     * @return true 表示状态可用；false 表示状态非法。
     *
     * 判据：
     * - `coordinate_frame` 不能为 `INVALID`；
     * - `position` / `velocity` / `bodyrates` 每个分量均为有限值；
     * - `orientation` 四元数每个分量为有限值；
     * - `orientation` 范数接近 1（容差 `1e-3`）。
     */
    bool isValid() const;

    ros::Time timestamp;  ///< 状态时间戳，对应 `Odometry.header.stamp`。

    /**
     * @brief 内部坐标系标签。
     *
     * 说明：
     * - `WORLD`：通常对应 `world`/`map`；
     * - `LOCAL`：通常对应 `local`/`odom`；
     * - `INVALID`：无法识别或未初始化。
     */
    enum class CoordinateFrame { INVALID, WORLD, LOCAL } coordinate_frame;

    std::string child_frame_id;  ///< 子坐标系，一般为机体系（如 `body`/`base_link`）。
    Eigen::Vector3d position;    ///< 位置向量（m）。
    Eigen::Vector3d velocity;    ///< 线速度向量（m/s）。
    Eigen::Vector3d bodyrates;   ///< 机体系角速度（rad/s）。
    Eigen::Quaterniond orientation;  ///< 姿态四元数（建议单位四元数）。
};
}  // namespace uav_control

namespace uav_control {
namespace detail {
inline std::string normalizeFrameId(const std::string& frame_id) {
    if (!frame_id.empty() && frame_id.front() == '/') {
        return frame_id.substr(1);
    }
    return frame_id;
}

/**
 * @brief 解析 ROS `frame_id` 到内部坐标系枚举。
 * @param frame_id 输入 frame 名称。
 * @return 解析后的坐标系枚举；不识别时返回 `INVALID`。
 */
inline uav_control::UAVStateEstimate::CoordinateFrame parseCoordinateFrame(const std::string& frame_id) {
    const std::string normalized = normalizeFrameId(frame_id);
    if (normalized == "world" || normalized == "map") {
        return uav_control::UAVStateEstimate::CoordinateFrame::WORLD;
    }
    if (normalized == "local" || normalized == "odom") {
        return uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    }
    return uav_control::UAVStateEstimate::CoordinateFrame::INVALID;
}

/**
 * @brief 判断三维向量是否全分量有限（非 NaN/Inf）。
 * @param v 输入向量。
 * @return true 表示全分量有限。
 */
inline bool isFiniteVector(const Eigen::Vector3d& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

/**
 * @brief 判断四元数是否全分量有限（非 NaN/Inf）。
 * @param q 输入四元数。
 * @return true 表示全分量有限。
 */
inline bool isFiniteQuaternion(const Eigen::Quaterniond& q) {
    return std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
}
}  // namespace detail

inline UAVStateEstimate::UAVStateEstimate()
    : timestamp(ros::Time::now()), coordinate_frame(CoordinateFrame::INVALID),
      child_frame_id("body"),
      position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()),
      bodyrates(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond::Identity()) {}

inline UAVStateEstimate::UAVStateEstimate(const nav_msgs::Odometry& state_estimate_msg) {
    timestamp = state_estimate_msg.header.stamp;
    coordinate_frame = detail::parseCoordinateFrame(state_estimate_msg.header.frame_id);
    child_frame_id = state_estimate_msg.child_frame_id.empty() ? "body" : state_estimate_msg.child_frame_id;
    position = geometryToEigen(state_estimate_msg.pose.pose.position);
    velocity = geometryToEigen(state_estimate_msg.twist.twist.linear);
    bodyrates = geometryToEigen(state_estimate_msg.twist.twist.angular);
    orientation = geometryToEigen(state_estimate_msg.pose.pose.orientation);
    // 若输入四元数有效，则在接收边界上做一次归一化，减少上游数值漂移影响。
    if (detail::isFiniteQuaternion(orientation)) {
        const double qnorm = orientation.norm();
        if (qnorm > 1e-9) {
            orientation.normalize();
        }
    }
}

inline nav_msgs::Odometry UAVStateEstimate::toRosMessage() const {
    nav_msgs::Odometry msg;
    msg.header.stamp = timestamp;
    switch (coordinate_frame) {
    case CoordinateFrame::WORLD:
        msg.header.frame_id = "world";
        break;
    case CoordinateFrame::LOCAL:
        msg.header.frame_id = "local";
        break;
    default:
        msg.header.frame_id = "invalid";
        break;
    }
    msg.child_frame_id = child_frame_id.empty() ? "body" : child_frame_id;
    msg.pose.pose.position = vectorToPoint(eigenToGeometry(position));
    msg.twist.twist.linear = eigenToGeometry(velocity);
    msg.pose.pose.orientation = eigenToGeometry(orientation);
    msg.twist.twist.angular = eigenToGeometry(bodyrates);
    return msg;
}

inline bool UAVStateEstimate::isValid() const {
    if (coordinate_frame == CoordinateFrame::INVALID) {
        return false;
    }
    if (!detail::isFiniteVector(position)) {
        return false;
    }
    if (!detail::isFiniteVector(velocity)) {
        return false;
    }
    if (!detail::isFiniteQuaternion(orientation)) {
        return false;
    }
    if (!detail::isFiniteVector(bodyrates)) {
        return false;
    }

    const double qnorm = orientation.norm();
    // 单位四元数容差校验，避免将明显异常姿态传入控制链路。
    if (std::abs(qnorm - 1.0) > 1e-3) {
        return false;
    }

    return true;
}

}  // namespace uav_control
