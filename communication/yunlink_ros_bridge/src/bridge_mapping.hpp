/** @file @brief ROS 消息到 YunLink snapshot 的字段映射接口。 */
#pragma once

#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVCommandExecutionStatus.h>
#include <sunray_msgs/Vector2.h>
#include <yunlink/runtime/runtime.hpp>

/** @brief 映射 ROS Header。 @param msg ROS Header。 @return YunLink HeaderSnapshot。 */
yunlink::HeaderSnapshot mapHeader(const std_msgs::Header& msg);
/** @brief 映射 Sunray 二维向量。 @param msg Sunray Vector2。 @return YunLink Vector2f。 */
yunlink::Vector2f mapVector2(const sunray_msgs::Vector2& msg);
/** @brief 映射四元数。 @param msg ROS Quaternion。 @return YunLink Quaternionf。 */
yunlink::Quaternionf mapQuaternion(const geometry_msgs::Quaternion& msg);
/** @brief 映射地理坐标。 @param msg ROS GeoPoint。 @return YunLink GeoPointSnapshot。 */
yunlink::GeoPointSnapshot mapGeoPoint(const geographic_msgs::GeoPoint& msg);
/** @brief 映射位姿。 @param msg ROS Pose。 @return YunLink PoseSnapshot。 */
yunlink::PoseSnapshot mapPose(const geometry_msgs::Pose& msg);
/** @brief 映射速度。 @param msg ROS Twist。 @return YunLink TwistSnapshot。 */
yunlink::TwistSnapshot mapTwist(const geometry_msgs::Twist& msg);
/** @brief 映射坐标变换。 @param msg ROS TransformStamped。 @return YunLink TransformSnapshot。 */
yunlink::TransformSnapshot mapTransform(const geometry_msgs::TransformStamped& msg);
/** @brief 映射通用里程计。 @param msg ROS Odometry。 @return YunLink OdometrySnapshot。 */
yunlink::OdometrySnapshot mapOdometry(const nav_msgs::Odometry& msg);
/** @brief 映射本地里程计。 @param msg ROS Odometry。 @return YunLink LocalOdomSnapshot。 */
yunlink::LocalOdomSnapshot mapLocalOdom(const nav_msgs::Odometry& msg);
/** @brief 映射 Sunray 控制命令。 @param msg Sunray UAVControlCMD。 @return YunLink UavControlCmdSnapshot。 */
yunlink::UavControlCmdSnapshot mapControlCmd(const sunray_msgs::UAVControlCMD& msg);
/** @brief 映射控制侧命令执行状态。 @param msg Sunray UAVCommandExecutionStatus。 @return YunLink CommandExecutionStatusSnapshot。 */
yunlink::CommandExecutionStatusSnapshot
mapCommandExecutionStatus(const sunray_msgs::UAVCommandExecutionStatus& msg);
/** @brief 转成 float。 @tparam T 输入数值类型。 @param value 输入值。 @return float 值。 */
template <typename T>
float toFloat(T value) {
    return static_cast<float>(value);
}

/** @brief 映射三维向量。 @tparam RosVector3T 含 x/y/z 字段的 ROS 向量类型。 @param msg ROS 向量。 @return YunLink Vector3f。 */
template <typename RosVector3T>
yunlink::Vector3f mapVector3(const RosVector3T& msg) {
    yunlink::Vector3f out{};
    out.x = toFloat(msg.x);
    out.y = toFloat(msg.y);
    out.z = toFloat(msg.z);
    return out;
}

/** @brief 映射位置目标。 @tparam PositionTargetT MAVROS 位置目标类型。 @param msg ROS 位置目标。 @return YunLink PositionTargetSnapshot。 */
template <typename PositionTargetT>
yunlink::PositionTargetSnapshot mapPositionTarget(const PositionTargetT& msg) {
    yunlink::PositionTargetSnapshot out{};
    out.header = mapHeader(msg.header);
    out.coordinate_frame = msg.coordinate_frame;
    out.type_mask = msg.type_mask;
    out.position_m = mapVector3(msg.position);
    out.velocity_mps = mapVector3(msg.velocity);
    out.acceleration_or_force = mapVector3(msg.acceleration_or_force);
    out.yaw_rad = msg.yaw;
    out.yaw_rate_radps = msg.yaw_rate;
    return out;
}

/** @brief 映射姿态目标。 @tparam AttitudeTargetT MAVROS 姿态目标类型。 @param msg ROS 姿态目标。 @return YunLink AttitudeTargetSnapshot。 */
template <typename AttitudeTargetT>
yunlink::AttitudeTargetSnapshot mapAttitudeTarget(const AttitudeTargetT& msg) {
    yunlink::AttitudeTargetSnapshot out{};
    out.header = mapHeader(msg.header);
    out.type_mask = msg.type_mask;
    out.orientation = mapQuaternion(msg.orientation);
    out.body_rate_radps = mapVector3(msg.body_rate);
    out.thrust = msg.thrust;
    return out;
}
