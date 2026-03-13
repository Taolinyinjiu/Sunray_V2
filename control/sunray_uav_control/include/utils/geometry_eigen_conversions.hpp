#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace uav_control {
/**
 * @brief `geometry_msgs` 与 Eigen 之间的常用几何类型转换接口集合。
 *
 * 说明：
 * - 本头文件仅声明转换函数，具体实现位于对应 `.cpp`；
 * - 函数本身不做坐标系语义推断，调用方需保证输入输出处于期望坐标系；
 * - 单位保持与输入消息一致（通常位置 m、速度 m/s、角速度 rad/s）。
 */

/**
 * @name Quaternion
 * @brief 四元数互转接口。
 */
///@{
/**
 * @brief ROS 四元数转 Eigen 四元数。
 * @param vec_ros 输入 `geometry_msgs::Quaternion`。
 * @return 对应的 `Eigen::Quaterniond`。
 */
Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion &vec_ros);

/**
 * @brief Eigen 四元数转 ROS 四元数。
 * @param vec_eigen 输入 `Eigen::Quaterniond`。
 * @return 对应的 `geometry_msgs::Quaternion`。
 */
geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond &vec_eigen);
///@}

/**
 * @name Vector And Point
 * @brief 三维向量/点互转接口。
 */
///@{
/**
 * @brief ROS 三维向量转 Eigen 向量。
 * @param vec_ros 输入 `geometry_msgs::Vector3`。
 * @return 对应的 `Eigen::Vector3d`。
 */
Eigen::Vector3d geometryToEigen(const geometry_msgs::Vector3 &vec_ros);

/**
 * @brief ROS 点转 Eigen 向量。
 * @param vec_ros 输入 `geometry_msgs::Point`。
 * @return 对应的 `Eigen::Vector3d`（按点坐标填充）。
 */
Eigen::Vector3d geometryToEigen(const geometry_msgs::Point &vec_ros);

/**
 * @brief ROS 带时间戳点转 Eigen 向量。
 * @param vec_ros 输入 `geometry_msgs::PointStamped`。
 * @return 对应的 `Eigen::Vector3d`（仅取 `point` 字段）。
 */
Eigen::Vector3d geometryToEigen(const geometry_msgs::PointStamped &vec_ros);

/**
 * @brief Eigen 向量转 ROS 三维向量。
 * @param vec_eigen 输入 `Eigen::Vector3d`。
 * @return 对应的 `geometry_msgs::Vector3`。
 */
geometry_msgs::Vector3 eigenToGeometry(const Eigen::Vector3d &vec_eigen);

/**
 * @brief ROS 三维向量类型转换为 ROS 点类型。
 * @param vector 输入 `geometry_msgs::Vector3`。
 * @return 对应的 `geometry_msgs::Point`（按 xyz 直接映射）。
 */
geometry_msgs::Point vectorToPoint(const geometry_msgs::Vector3 &vector);
///@}

/**
 * @name Pose
 * @brief 位姿消息到 Eigen 仿射变换的转换接口。
 */
///@{
/**
 * @brief ROS 位姿转 Eigen 仿射变换。
 * @param pose_ros 输入 `geometry_msgs::Pose`。
 * @return 对应的 `Eigen::Affine3d`。
 *
 * 通常平移来自 position，旋转来自 orientation。
 */
Eigen::Affine3d geometryToEigen(const geometry_msgs::Pose &pose_ros);

/**
 * @brief ROS 带时间戳位姿转 Eigen 仿射变换。
 * @param pose_ros 输入 `geometry_msgs::PoseStamped`。
 * @return 对应的 `Eigen::Affine3d`（仅取 `pose` 字段）。
 */
Eigen::Affine3d geometryToEigen(const geometry_msgs::PoseStamped &pose_ros);
///@}
} // namespace uav_control
