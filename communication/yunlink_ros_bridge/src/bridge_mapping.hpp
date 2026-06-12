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

yunlink::HeaderSnapshot mapHeader(const std_msgs::Header& msg);
yunlink::Vector2f mapVector2(const sunray_msgs::Vector2& msg);
yunlink::Quaternionf mapQuaternion(const geometry_msgs::Quaternion& msg);
yunlink::GeoPointSnapshot mapGeoPoint(const geographic_msgs::GeoPoint& msg);
yunlink::PoseSnapshot mapPose(const geometry_msgs::Pose& msg);
yunlink::TwistSnapshot mapTwist(const geometry_msgs::Twist& msg);
yunlink::TransformSnapshot mapTransform(const geometry_msgs::TransformStamped& msg);
yunlink::OdometrySnapshot mapOdometry(const nav_msgs::Odometry& msg);
yunlink::LocalOdomSnapshot mapLocalOdom(const nav_msgs::Odometry& msg);
yunlink::UavControlCmdSnapshot mapControlCmd(const sunray_msgs::UAVControlCMD& msg);
yunlink::CommandExecutionStatusSnapshot
mapCommandExecutionStatus(const sunray_msgs::UAVCommandExecutionStatus& msg);
template <typename T>
float toFloat(T value) {
    return static_cast<float>(value);
}

template <typename RosVector3T>
yunlink::Vector3f mapVector3(const RosVector3T& msg) {
    yunlink::Vector3f out{};
    out.x = toFloat(msg.x);
    out.y = toFloat(msg.y);
    out.z = toFloat(msg.z);
    return out;
}

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
