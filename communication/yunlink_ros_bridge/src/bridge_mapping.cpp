#include "bridge_mapping.hpp"

yunlink::HeaderSnapshot mapHeader(const std_msgs::Header& msg) {
    yunlink::HeaderSnapshot out{};
    out.frame_id = msg.frame_id;
    if (!msg.stamp.isZero()) {
        out.stamp_ns =
            static_cast<uint64_t>(msg.stamp.sec) * 1000000000ULL + static_cast<uint64_t>(msg.stamp.nsec);
    }
    return out;
}

yunlink::Vector2f mapVector2(const sunray_msgs::Vector2& msg) {
    yunlink::Vector2f out{};
    out.x = msg.x;
    out.y = msg.y;
    return out;
}

yunlink::Quaternionf mapQuaternion(const geometry_msgs::Quaternion& msg) {
    yunlink::Quaternionf out{};
    out.x = toFloat(msg.x);
    out.y = toFloat(msg.y);
    out.z = toFloat(msg.z);
    out.w = toFloat(msg.w);
    return out;
}

yunlink::GeoPointSnapshot mapGeoPoint(const geographic_msgs::GeoPoint& msg) {
    yunlink::GeoPointSnapshot out{};
    out.latitude_deg = msg.latitude;
    out.longitude_deg = msg.longitude;
    out.altitude_m = msg.altitude;
    return out;
}

yunlink::PoseSnapshot mapPose(const geometry_msgs::Pose& msg) {
    yunlink::PoseSnapshot out{};
    out.position_m = mapVector3(msg.position);
    out.orientation = mapQuaternion(msg.orientation);
    return out;
}

yunlink::TwistSnapshot mapTwist(const geometry_msgs::Twist& msg) {
    yunlink::TwistSnapshot out{};
    out.linear_mps = mapVector3(msg.linear);
    out.angular_radps = mapVector3(msg.angular);
    return out;
}

yunlink::TransformSnapshot mapTransform(const geometry_msgs::TransformStamped& msg) {
    yunlink::TransformSnapshot out{};
    out.header = mapHeader(msg.header);
    out.child_frame_id = msg.child_frame_id;
    out.translation_m = mapVector3(msg.transform.translation);
    out.rotation = mapQuaternion(msg.transform.rotation);
    return out;
}

yunlink::OdometrySnapshot mapOdometry(const nav_msgs::Odometry& msg) {
    yunlink::OdometrySnapshot out{};
    out.header = mapHeader(msg.header);
    out.child_frame_id = msg.child_frame_id;
    out.pose = mapPose(msg.pose.pose);
    out.twist = mapTwist(msg.twist.twist);
    for (size_t i = 0; i < out.pose_covariance.size(); ++i) {
        out.pose_covariance[i] = msg.pose.covariance[i];
        out.twist_covariance[i] = msg.twist.covariance[i];
    }
    return out;
}

yunlink::LocalOdomSnapshot mapLocalOdom(const nav_msgs::Odometry& msg) {
    yunlink::LocalOdomSnapshot out{};
    out.header = mapHeader(msg.header);
    out.child_frame_id = msg.child_frame_id;
    out.pose = mapPose(msg.pose.pose);
    out.twist = mapTwist(msg.twist.twist);
    for (size_t i = 0; i < out.pose_covariance.size(); ++i) {
        out.pose_covariance[i] = msg.pose.covariance[i];
        out.twist_covariance[i] = msg.twist.covariance[i];
    }
    return out;
}

yunlink::UavControlCmdSnapshot mapControlCmd(const sunray_msgs::UAVControlCMD& msg) {
    yunlink::UavControlCmdSnapshot out{};
    out.header = mapHeader(msg.header);
    out.cmd_source = msg.cmd_source;
    out.control_cmd = msg.control_cmd;
    out.desired_pos_m = mapVector3(msg.desired_pos);
    out.desired_vel_mps = mapVector3(msg.desired_vel);
    out.desired_acc_mps2 = mapVector3(msg.desired_acc);
    out.desired_jerk = mapVector3(msg.desired_jerk);
    out.desired_body_xy_pos_m = mapVector2(msg.desired_body_xy_pos);
    out.desired_body_xy_vel_mps = mapVector2(msg.desired_body_xy_vel);
    out.fixed_height_m = msg.fixed_height;
    out.desired_wgs84_pos = mapGeoPoint(msg.desired_wgs84_pos);
    out.yaw_mode = msg.yaw_mode;
    out.desired_yaw_rad = msg.desired_yaw;
    out.desired_yaw_rate_radps = msg.desired_yaw_rate;
    return out;
}
