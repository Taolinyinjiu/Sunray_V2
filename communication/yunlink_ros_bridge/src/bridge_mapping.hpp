#pragma once

#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <sunray_msgs/UAVControlCMD.h>
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
yunlink::PositionTargetSnapshot mapPositionTarget(const mavros_msgs::PositionTarget& msg);
yunlink::AttitudeTargetSnapshot mapAttitudeTarget(const mavros_msgs::AttitudeTarget& msg);

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
