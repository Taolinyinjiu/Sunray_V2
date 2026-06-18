/** @file @brief Sunray ROS 状态话题到 YunLink snapshot 的转发实现。 */
#include "bridge_node.hpp"

#include "bridge_mapping.hpp"

/// topic 回调只镜像状态，权威控制状态仍在 Sunray 包内。
/// 转发本地里程计到 YunLink。
void YunlinkRosBridgeNode::onLocalOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    const ros::Time receive_time = msg->header.stamp.isZero() ? now : msg->header.stamp;
    {
        std::lock_guard<std::mutex> lock(diag_mu_);
        recordRosToYunlinkEvent(&local_odom_diag_, receive_time);
    }
    if (!shouldPublishRosToYunlink("local_odom", now.isZero() ? receive_time : now)) {
        return;
    }
    const yunlink::LocalOdomSnapshot payload = mapLocalOdom(*msg);
    publishSnapshot("local_odom", &yunlink::Runtime::publish_local_odom, payload);
}

/// 转发定位状态并同步诊断里程计 topic 元信息。
void YunlinkRosBridgeNode::onOdomState(const sunray_msgs::OdomState::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    const ros::Time receive_time = msg->header.stamp.isZero() ? now : msg->header.stamp;
    {
        std::lock_guard<std::mutex> lock(diag_mu_);
        std::string detail = "external_source=" + std::to_string(msg->external_source) +
                             " odometry_valid=" + std::string(msg->odometry_valid ? "true" : "false");
        recordRosToYunlinkEvent(&odom_state_diag_, receive_time, detail);
        external_odom_diag_.detail = "source_topic=" + msg->subtopic_name_external_odom;
        if (!msg->subtopic_name_external_relocalization.empty()) {
            external_odom_diag_.detail += " relocalization=" + msg->subtopic_name_external_relocalization;
        }
        local_odom_diag_.topic = msg->pubtopic_name_local_odom;
        local_odom_diag_.configured = !msg->pubtopic_name_local_odom.empty();
    }
    if (!shouldPublishRosToYunlink("odom_state", now.isZero() ? receive_time : now)) {
        return;
    }
    yunlink::OdomStateSnapshot payload{};
    payload.header = mapHeader(msg->header);
    payload.external_source = msg->external_source;
    payload.subtopic_name_external_odom = msg->subtopic_name_external_odom;
    payload.odometry_valid = msg->odometry_valid;
    payload.odometry_update_hz = msg->odometry_update_hz;
    payload.subtopic_name_external_relocalization = msg->subtopic_name_external_relocalization;
    payload.pubtopic_name_local_odom = msg->pubtopic_name_local_odom;
    payload.pubtopic_name_global_odom = msg->pubtopic_name_global_odom;
    payload.local_odom = mapOdometry(msg->local_odom);
    payload.global_odom = mapOdometry(msg->global_odom);
    payload.world_frame_name = msg->world_frame_name;
    payload.global_frame_name = msg->global_frame_name;
    payload.local_frame_name = msg->local_frame_name;
    payload.base_frame_name = msg->base_frame_name;
    payload.world_to_global_tf = mapTransform(msg->world_to_global_tf);
    payload.global_to_local_tf = mapTransform(msg->global_to_local_tf);
    payload.local_to_base_tf = mapTransform(msg->local_to_base_tf);
    publishSnapshot("odom_state", &yunlink::Runtime::publish_odom_state, payload);
}

/// 转发 Sunray 控制命令 snapshot，包含 bridge 下发和本地发布的命令。
void YunlinkRosBridgeNode::onControlCmd(const sunray_msgs::UAVControlCMD::ConstPtr& msg) {
    // 同步 bridge 转发命令和本地 Sunray 命令到同一条 snapshot 流。
    const ros::Time now = ros::Time::now();
    const ros::Time receive_time = msg->header.stamp.isZero() ? now : msg->header.stamp;
    {
        std::lock_guard<std::mutex> lock(diag_mu_);
        recordRosToYunlinkEvent(&uav_control_cmd_diag_,
                                receive_time,
                                "control_cmd=" + std::to_string(msg->control_cmd));
    }
    if (!shouldPublishRosToYunlink("uav_control_cmd", now.isZero() ? receive_time : now)) {
        return;
    }
    const auto payload = mapControlCmd(*msg);
    publishSnapshot("uav_control_cmd", &yunlink::Runtime::publish_uav_control_cmd, payload);
}

/// 转发控制状态 snapshot，不在 bridge 内判断任务是否完成。
void YunlinkRosBridgeNode::onControlState(const sunray_msgs::UAVControlState::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    const ros::Time receive_time = msg->header.stamp.isZero() ? now : msg->header.stamp;
    {
        std::lock_guard<std::mutex> lock(diag_mu_);
        recordRosToYunlinkEvent(&uav_control_state_diag_,
                                receive_time,
                                "fsm=" + std::to_string(msg->control_state));
    }
    if (!shouldPublishRosToYunlink("control_state", now.isZero() ? receive_time : now)) {
        return;
    }
    yunlink::UavControlStateSnapshot payload{};
    payload.header = mapHeader(msg->header);
    payload.agent_name = msg->agent_name;
    payload.agent_id = msg->agent_id;
    payload.controller_types = msg->controller_types;
    payload.takeoff_relative_height_m = msg->takeoff_relative_height;
    payload.takeoff_max_velocity_mps = msg->takeoff_max_velocity;
    payload.land_type = msg->land_type;
    payload.land_max_velocity_mps = msg->land_max_velocity;
    payload.home_point_m = mapVector3(msg->home_point);
    payload.control_state = msg->control_state;
    payload.last_cmd = mapControlCmd(msg->last_cmd);
    payload.self_odom = mapOdometry(msg->self_odom);
    payload.odometry_lost = msg->odometry_lost;
    payload.odometry_valid = msg->odometry_valid;
    payload.controller_output_type = msg->controller_output_type;
    payload.position_target = mapPositionTarget(msg->position_target);
    payload.attitude_target = mapAttitudeTarget(msg->attitude_target);
    publishSnapshot("uav_control_state", &yunlink::Runtime::publish_uav_control_state, payload);
}

/// 转发控制侧命令执行状态，保留与原 YunLink 命令的关联 ID。
void YunlinkRosBridgeNode::onCommandExecutionStatus(
    const sunray_msgs::UAVCommandExecutionStatus::ConstPtr& msg) {
    // 执行结果权威来源是 sunray_uav_control；bridge 只做 snapshot 转换。
    const ros::Time now = ros::Time::now();
    const ros::Time receive_time = msg->header.stamp.isZero() ? now : msg->header.stamp;
    if (!shouldPublishRosToYunlink("command_execution_status", now.isZero() ? receive_time : now)) {
        return;
    }
    const auto payload = mapCommandExecutionStatus(*msg);
    publishSnapshot("command_execution_status",
                    &yunlink::Runtime::publish_command_execution_status,
                    payload);
}

/// 转发 PX4 状态，并缓存高度供机体系速度命令使用。
void YunlinkRosBridgeNode::onPx4State(const sunray_msgs::Px4State::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    const ros::Time receive_time = msg->header.stamp.isZero() ? now : msg->header.stamp;
    {
        std::lock_guard<std::mutex> lock(px4_state_mu_);
        latest_px4_height_m_ = static_cast<float>(msg->local_pose.position.z);
        has_px4_height_ = true;
    }
    {
        std::lock_guard<std::mutex> lock(diag_mu_);
        recordRosToYunlinkEvent(&px4_state_diag_,
                                receive_time,
                                "armed=" + std::string(msg->armed ? "true" : "false") +
                                    " mode=" + std::to_string(msg->flight_mode));
    }
    if (!shouldPublishRosToYunlink("px4_state", now.isZero() ? receive_time : now)) {
        return;
    }

    yunlink::Px4StateSnapshot payload{};
    payload.header = mapHeader(msg->header);
    payload.connected = msg->connected;
    payload.rc_available = msg->rc_available;
    payload.armed = msg->armed;
    payload.flight_mode = msg->flight_mode;
    payload.system_status = msg->system_status;
    payload.landed_state = msg->landed_state;
    payload.battery_voltage_v = msg->battery_voltage_v;
    payload.battery_current_a = msg->battery_current_a;
    payload.battery_percentage = msg->battery_percentage;
    payload.fcu_load = msg->fcu_load;
    payload.external_pose = mapPose(msg->external_pose);
    payload.external_velocity = mapTwist(msg->external_velocity);
    payload.local_pose = mapPose(msg->local_pose);
    payload.local_velocity = mapTwist(msg->local_velocity);
    payload.setpoint_coordinate_frame = msg->setpoint_coordinate_frame;
    payload.setpoint_local_type_mask = msg->setpoint_local_type_mask;
    payload.pos_setpoint_m = mapVector3(msg->pos_setpoint);
    payload.vel_setpoint_mps = mapVector3(msg->vel_setpoint);
    payload.acc_setpoint_mps2 = mapVector3(msg->acc_setpoint);
    payload.yaw_setpoint_rad = msg->yaw_setpoint;
    payload.yaw_rate_setpoint_radps = msg->yaw_rate_setpoint;
    payload.setpoint_att_type_mask = msg->setpoint_att_type_mask;
    payload.orientation_setpoint = mapQuaternion(msg->orientation_setpoint);
    payload.body_rate_setpoint_radps = mapVector3(msg->body_rate_setpoint);
    payload.thrust_setpoint = msg->thrust_setpoint;
    payload.satellites = msg->satellites;
    payload.gps_status = msg->gps_status;
    payload.gps_service = msg->gps_service;
    payload.latitude_deg = msg->latitude;
    payload.longitude_deg = msg->longitude;
    payload.altitude_m = msg->altitude;
    payload.latitude_raw_deg = msg->latitude_raw;
    payload.longitude_raw_deg = msg->longitude_raw;
    payload.altitude_amsl_m = msg->altitude_amsl;
    publishSnapshot("px4_state", &yunlink::Runtime::publish_px4_state, payload);
}
