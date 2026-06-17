/** @file @brief ROS 消息到 YunLink snapshot 的字段映射实现。 */
#include "bridge_mapping.hpp"

/// 本文件只做字段转换，不推断就绪、完成或控制权限。
/// 将 ROS Header 的 frame/stamp 转成 YunLink 通用头。
yunlink::HeaderSnapshot mapHeader(const std_msgs::Header& msg) {
    yunlink::HeaderSnapshot out{};
    out.frame_id = msg.frame_id;
    if (!msg.stamp.isZero()) {
        out.stamp_ns =
            static_cast<uint64_t>(msg.stamp.sec) * 1000000000ULL + static_cast<uint64_t>(msg.stamp.nsec);
    }
    return out;
}

/// 将 Sunray Vector2 转成 YunLink 二维向量。
yunlink::Vector2f mapVector2(const sunray_msgs::Vector2& msg) {
    yunlink::Vector2f out{};
    out.x = msg.x;
    out.y = msg.y;
    return out;
}

/// 将 ROS 四元数转成 YunLink 四元数。
yunlink::Quaternionf mapQuaternion(const geometry_msgs::Quaternion& msg) {
    yunlink::Quaternionf out{};
    out.x = toFloat(msg.x);
    out.y = toFloat(msg.y);
    out.z = toFloat(msg.z);
    out.w = toFloat(msg.w);
    return out;
}

/// 将 ROS 地理坐标转成 YunLink 地理坐标 snapshot。
yunlink::GeoPointSnapshot mapGeoPoint(const geographic_msgs::GeoPoint& msg) {
    yunlink::GeoPointSnapshot out{};
    out.latitude_deg = msg.latitude;
    out.longitude_deg = msg.longitude;
    out.altitude_m = msg.altitude;
    return out;
}

/// 将 ROS 位姿转成 YunLink 位姿 snapshot。
yunlink::PoseSnapshot mapPose(const geometry_msgs::Pose& msg) {
    yunlink::PoseSnapshot out{};
    out.position_m = mapVector3(msg.position);
    out.orientation = mapQuaternion(msg.orientation);
    return out;
}

/// 将 ROS 线速度/角速度转成 YunLink twist snapshot。
yunlink::TwistSnapshot mapTwist(const geometry_msgs::Twist& msg) {
    yunlink::TwistSnapshot out{};
    out.linear_mps = mapVector3(msg.linear);
    out.angular_radps = mapVector3(msg.angular);
    return out;
}

/// 将 ROS 坐标变换转成 YunLink transform snapshot。
yunlink::TransformSnapshot mapTransform(const geometry_msgs::TransformStamped& msg) {
    yunlink::TransformSnapshot out{};
    out.header = mapHeader(msg.header);
    out.child_frame_id = msg.child_frame_id;
    out.translation_m = mapVector3(msg.transform.translation);
    out.rotation = mapQuaternion(msg.transform.rotation);
    return out;
}

/// 将 ROS 里程计完整转成 YunLink odometry snapshot。
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

/// 将本地里程计转成 YunLink local odom snapshot。
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

/// 将 Sunray 控制命令转成 monitor 可读的 YunLink snapshot。
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

/// 将控制侧发布的命令执行状态转成 YunLink 状态 snapshot。
yunlink::CommandExecutionStatusSnapshot
mapCommandExecutionStatus(const sunray_msgs::UAVCommandExecutionStatus& msg) {
    yunlink::CommandExecutionStatusSnapshot out{};
    out.header = mapHeader(msg.header);
    out.agent_name = msg.agent_name;
    out.agent_id = msg.agent_id;
    // envelope ID 经控制链路透传回来，用于把状态匹配回原始命令。
    out.session_id = msg.yunlink_session_id;
    out.command_message_id = msg.yunlink_message_id;
    out.command_correlation_id = msg.yunlink_correlation_id;
    out.command_kind = static_cast<yunlink::CommandKind>(msg.command_kind);
    out.execution_state = msg.execution_state;
    out.progress_percent = msg.progress_percent;
    out.active = msg.active;
    out.terminal = msg.terminal;
    out.success = msg.success;
    out.result_code = msg.result_code;
    out.detail = msg.detail;
    out.control_state = msg.control_state;
    out.px4_landed_state = msg.px4_landed_state;
    out.ready_for_takeoff = msg.ready_for_takeoff;
    out.ready_for_land = msg.ready_for_land;
    out.busy_reason = msg.busy_reason;
    return out;
}
