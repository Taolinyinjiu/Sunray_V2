#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sunray_msgs/OdomState.h>
#include <sunray_msgs/Px4State.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlState.h>
#include <yunlink/runtime/runtime.hpp>

namespace {

template <typename T>
float toFloat(T value) {
    return static_cast<float>(value);
}

yunlink::HeaderSnapshot mapHeader(const std_msgs::Header& msg) {
    yunlink::HeaderSnapshot out{};
    out.frame_id = msg.frame_id;
    return out;
}

yunlink::Vector2f mapVector2(const sunray_msgs::Vector2& msg) {
    yunlink::Vector2f out{};
    out.x = msg.x;
    out.y = msg.y;
    return out;
}

template <typename RosVector3T>
yunlink::Vector3f mapVector3(const RosVector3T& msg) {
    yunlink::Vector3f out{};
    out.x = toFloat(msg.x);
    out.y = toFloat(msg.y);
    out.z = toFloat(msg.z);
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

yunlink::PositionTargetSnapshot mapPositionTarget(const mavros_msgs::PositionTarget& msg) {
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

yunlink::AttitudeTargetSnapshot mapAttitudeTarget(const mavros_msgs::AttitudeTarget& msg) {
    yunlink::AttitudeTargetSnapshot out{};
    out.header = mapHeader(msg.header);
    out.type_mask = msg.type_mask;
    out.orientation = mapQuaternion(msg.orientation);
    out.body_rate_radps = mapVector3(msg.body_rate);
    out.thrust = msg.thrust;
    return out;
}

class YunlinkRosBridgeNode {
  public:
    YunlinkRosBridgeNode() : nh_(), pnh_("~") {
        loadParams();
        startRuntime();
        setupSubscribers();
        setupReconnectTimer();
    }

    ~YunlinkRosBridgeNode() {
        runtime_.stop();
    }

  private:
    void loadParams() {
        pnh_.param<std::string>("local_odom_topic",
                                local_odom_topic_,
                                "/uav1/sunray/localization/local_odom");
        pnh_.param<std::string>("odom_state_topic",
                                odom_state_topic_,
                                "/uav1/sunray/localization/odom_state");
        pnh_.param<std::string>("control_state_topic",
                                control_state_topic_,
                                "/uav1/sunray/uav_control/control_state");
        pnh_.param<std::string>("control_cmd_topic",
                                control_cmd_topic_,
                                "/uav1/sunray/uav_control/control_cmd");
        pnh_.param<std::string>("mavros_state_topic", mavros_state_topic_, "/uav1/mavros/state");
        pnh_.param<std::string>("px4_state_topic", px4_state_topic_, "/uav1/sunray/px4_state");

        pnh_.param<std::string>("remote_ip", remote_ip_, std::string());
        pnh_.param<int>("remote_tcp_port", remote_tcp_port_, 0);
        pnh_.param<int>("udp_bind_port", udp_bind_port_, 9696);
        pnh_.param<int>("udp_target_port", udp_target_port_, 9898);
        pnh_.param<int>("tcp_listen_port", tcp_listen_port_, 9696);
        pnh_.param<int>("agent_id", agent_id_, 1);
        pnh_.param<std::string>("shared_secret", shared_secret_, "yunlink-default-secret");
        pnh_.param<std::string>("node_name", node_name_, std::string("yunlink_ros_bridge_node"));
    }

    void startRuntime() {
        yunlink::RuntimeConfig cfg;
        cfg.udp_bind_port = clampPort(udp_bind_port_);
        cfg.udp_target_port = clampPort(udp_target_port_);
        cfg.tcp_listen_port = clampPort(tcp_listen_port_);
        cfg.shared_secret = shared_secret_;
        cfg.self_identity.agent_type = yunlink::AgentType::kUav;
        cfg.self_identity.agent_id = static_cast<uint32_t>(std::max(agent_id_, 0));
        cfg.self_identity.role = yunlink::EndpointRole::kVehicle;

        const auto ec = runtime_.start(cfg);
        if (ec != yunlink::ErrorCode::kOk) {
            ROS_FATAL("yunlink runtime start failed, error code=%u", static_cast<unsigned>(ec));
            ros::shutdown();
            return;
        }

        ROS_INFO("yunlink runtime started: udp_bind=%u udp_target=%u tcp_listen=%u agent_id=%d",
                 cfg.udp_bind_port,
                 cfg.udp_target_port,
                 cfg.tcp_listen_port,
                 agent_id_);
    }

    void setupSubscribers() {
        control_cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>(control_cmd_topic_, 20);
        local_odom_sub_ =
            nh_.subscribe(local_odom_topic_, 20, &YunlinkRosBridgeNode::onLocalOdom, this);
        odom_state_sub_ =
            nh_.subscribe(odom_state_topic_, 20, &YunlinkRosBridgeNode::onOdomState, this);
        control_state_sub_ =
            nh_.subscribe(control_state_topic_, 20, &YunlinkRosBridgeNode::onControlState, this);
        mavros_state_sub_ =
            nh_.subscribe(mavros_state_topic_, 20, &YunlinkRosBridgeNode::onMavrosState, this);
        px4_state_sub_ =
            nh_.subscribe(px4_state_topic_, 20, &YunlinkRosBridgeNode::onPx4State, this);

        takeoff_command_sub_token_ = runtime_.command_subscriber().subscribe_takeoff(
            [this](const yunlink::InboundCommandView<yunlink::TakeoffCommand>& view) {
                onTakeoffCommand(view);
            });
        land_command_sub_token_ = runtime_.command_subscriber().subscribe_land(
            [this](const yunlink::InboundCommandView<yunlink::LandCommand>& view) {
                onLandCommand(view);
            });
        return_command_sub_token_ = runtime_.command_subscriber().subscribe_return(
            [this](const yunlink::InboundCommandView<yunlink::ReturnCommand>& view) {
                onReturnCommand(view);
            });
        goto_command_sub_token_ = runtime_.command_subscriber().subscribe_goto(
            [this](const yunlink::InboundCommandView<yunlink::GotoCommand>& view) {
                onGotoCommand(view);
            });
        velocity_setpoint_command_sub_token_ =
            runtime_.command_subscriber().subscribe_velocity_setpoint(
                [this](const yunlink::InboundCommandView<yunlink::VelocitySetpointCommand>& view) {
                    onVelocitySetpointCommand(view);
                });

        ROS_INFO("yunlink_ros_bridge advertise control_cmd: %s", control_cmd_topic_.c_str());
        ROS_INFO("yunlink_ros_bridge subscribe local_odom: %s", local_odom_topic_.c_str());
        ROS_INFO("yunlink_ros_bridge subscribe odom_state: %s", odom_state_topic_.c_str());
        ROS_INFO("yunlink_ros_bridge subscribe control_state: %s", control_state_topic_.c_str());
        ROS_INFO("yunlink_ros_bridge subscribe mavros_state: %s", mavros_state_topic_.c_str());
        ROS_INFO("yunlink_ros_bridge subscribe px4_state: %s", px4_state_topic_.c_str());
        ROS_INFO("yunlink_ros_bridge subscribe yunlink commands: takeoff land return goto velocity_setpoint");
    }

    void setupReconnectTimer() {
        reconnect_timer_ = nh_.createTimer(
            ros::Duration(1.0), &YunlinkRosBridgeNode::onReconnectTimer, this, false, true);
    }

    static uint16_t clampPort(int port) {
        if (port < 0) {
            return 0;
        }
        if (port > 65535) {
            return 65535;
        }
        return static_cast<uint16_t>(port);
    }

    bool ensurePeerReady() {
        if (peer_ready_) {
            return true;
        }

        yunlink::SessionDescriptor active_session{};
        if (runtime_.session_server().find_active_session(&active_session)) {
            peer_id_ = active_session.peer.id;
            session_id_ = active_session.session_id;
            peer_ready_ = true;
            ROS_INFO_THROTTLE(5.0,
                              "yunlink_ros_bridge reusing active session peer_id=%s session_id=%lu",
                              peer_id_.c_str(),
                              static_cast<unsigned long>(session_id_));
            return true;
        }

        if (remote_ip_.empty() || remote_tcp_port_ <= 0) {
            ROS_WARN_THROTTLE(5.0,
                              "yunlink_ros_bridge has no remote_ip/remote_tcp_port and no inbound active session yet");
            return false;
        }

        std::string peer_id;
        const auto connect_ec =
            runtime_.tcp_clients().connect_peer(remote_ip_, clampPort(remote_tcp_port_), &peer_id);
        if (connect_ec != yunlink::ErrorCode::kOk) {
            ROS_WARN_THROTTLE(5.0,
                              "yunlink_ros_bridge connect_peer failed, ip=%s port=%d ec=%u",
                              remote_ip_.c_str(),
                              remote_tcp_port_,
                              static_cast<unsigned>(connect_ec));
            return false;
        }

        const uint64_t session_id = runtime_.session_client().open_active_session(peer_id, node_name_);
        if (session_id == 0) {
            ROS_WARN_THROTTLE(5.0,
                              "yunlink_ros_bridge open_active_session failed, peer_id=%s",
                              peer_id.c_str());
            return false;
        }

        peer_id_ = peer_id;
        session_id_ = session_id;
        peer_ready_ = true;
        ROS_INFO("yunlink_ros_bridge connected peer_id=%s session_id=%lu",
                 peer_id_.c_str(),
                 static_cast<unsigned long>(session_id_));
        return true;
    }

    yunlink::TargetSelector targetSelector() const {
        return yunlink::TargetSelector::broadcast(yunlink::AgentType::kGroundStation);
    }

    void onReconnectTimer(const ros::TimerEvent&) {
        if (!peer_ready_) {
            (void)ensurePeerReady();
            return;
        }

        yunlink::SessionDescriptor session{};
        if (!runtime_.session_server().describe_session(peer_id_, session_id_, &session) ||
            session.state != yunlink::SessionState::kActive) {
            peer_ready_ = false;
            session_id_ = 0;
            peer_id_.clear();
            ROS_WARN_THROTTLE(5.0, "yunlink_ros_bridge peer session is no longer active, retrying");
        }
    }

    template <typename PublishFn, typename PayloadT>
    void publishSnapshot(const char* name, PublishFn publish_fn, const PayloadT& payload) {
        if (!ensurePeerReady()) {
            return;
        }

        const auto ec = (runtime_.*publish_fn)(peer_id_, targetSelector(), payload, session_id_);
        if (ec != yunlink::ErrorCode::kOk) {
            peer_ready_ = false;
            ROS_WARN_THROTTLE(5.0,
                              "yunlink_ros_bridge publish %s failed, ec=%u",
                              name,
                              static_cast<unsigned>(ec));
        }
    }

    sunray_msgs::UAVControlCMD makeBaseControlCmd(uint8_t control_cmd) const {
        sunray_msgs::UAVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd_source = sunray_msgs::UAVControlCMD::SUNRAY_STATION;
        cmd.control_cmd = control_cmd;
        return cmd;
    }

    void publishControlCmd(const char* name,
                           const sunray_msgs::UAVControlCMD& cmd,
                           uint64_t session_id,
                           uint64_t correlation_id,
                           uint64_t message_id,
                           const std::string& detail = std::string()) {
        control_cmd_pub_.publish(cmd);

        std::string line = std::string("yunlink_ros_bridge forwarded ") + name + " to " +
                           control_cmd_topic_ + " session_id=" + std::to_string(session_id) +
                           " correlation_id=" + std::to_string(correlation_id) + " msg_id=" +
                           std::to_string(message_id);
        if (!detail.empty()) {
            line += " " + detail;
        }
        ROS_INFO("%s", line.c_str());
    }

    float latestPx4Height(bool* has_height = nullptr) const {
        std::lock_guard<std::mutex> lock(px4_state_mu_);
        if (has_height != nullptr) {
            *has_height = has_px4_height_;
        }
        return latest_px4_height_m_;
    }

    void onTakeoffCommand(const yunlink::InboundCommandView<yunlink::TakeoffCommand>& view) {
        sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(sunray_msgs::UAVControlCMD::TAKEOFF);
        publishControlCmd("takeoff",
                          cmd,
                          view.inbound.envelope.session_id,
                          view.inbound.envelope.correlation_id,
                          view.inbound.envelope.message_id,
                          "relative_height_m=" +
                              std::to_string(view.payload.relative_height_m) +
                              " max_velocity_mps=" +
                              std::to_string(view.payload.max_velocity_mps));
    }

    void onLandCommand(const yunlink::InboundCommandView<yunlink::LandCommand>& view) {
        sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(sunray_msgs::UAVControlCMD::LAND);
        publishControlCmd("land",
                          cmd,
                          view.inbound.envelope.session_id,
                          view.inbound.envelope.correlation_id,
                          view.inbound.envelope.message_id,
                          "max_velocity_mps=" + std::to_string(view.payload.max_velocity_mps));
    }

    void onReturnCommand(const yunlink::InboundCommandView<yunlink::ReturnCommand>& view) {
        sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(sunray_msgs::UAVControlCMD::RETURN);
        publishControlCmd("return",
                          cmd,
                          view.inbound.envelope.session_id,
                          view.inbound.envelope.correlation_id,
                          view.inbound.envelope.message_id,
                          "loiter_before_return_s=" +
                              std::to_string(view.payload.loiter_before_return_s));
    }

    void onGotoCommand(const yunlink::InboundCommandView<yunlink::GotoCommand>& view) {
        sunray_msgs::UAVControlCMD cmd =
            makeBaseControlCmd(sunray_msgs::UAVControlCMD::MOVE_POINT);
        cmd.desired_pos.x = view.payload.x_m;
        cmd.desired_pos.y = view.payload.y_m;
        cmd.desired_pos.z = view.payload.z_m;
        cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
        cmd.desired_yaw = view.payload.yaw_rad;

        publishControlCmd("goto",
                          cmd,
                          view.inbound.envelope.session_id,
                          view.inbound.envelope.correlation_id,
                          view.inbound.envelope.message_id,
                          "x=" + std::to_string(view.payload.x_m) +
                              " y=" + std::to_string(view.payload.y_m) +
                              " z=" + std::to_string(view.payload.z_m) +
                              " yaw=" + std::to_string(view.payload.yaw_rad));
    }

    void onVelocitySetpointCommand(
        const yunlink::InboundCommandView<yunlink::VelocitySetpointCommand>& view) {
        sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(
            view.payload.body_frame ? sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY
                                    : sunray_msgs::UAVControlCMD::MOVE_VELOCITY);
        cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAWRATE;
        cmd.desired_yaw_rate = view.payload.yaw_rate_radps;

        std::string detail = "body_frame=" + std::string(view.payload.body_frame ? "true" : "false") +
                             " vx=" + std::to_string(view.payload.vx_mps) +
                             " vy=" + std::to_string(view.payload.vy_mps) +
                             " vz=" + std::to_string(view.payload.vz_mps) +
                             " yaw_rate=" + std::to_string(view.payload.yaw_rate_radps);

        if (view.payload.body_frame) {
            bool has_height = false;
            cmd.desired_body_xy_vel.x = view.payload.vx_mps;
            cmd.desired_body_xy_vel.y = view.payload.vy_mps;
            cmd.fixed_height = latestPx4Height(&has_height);
            if (!has_height) {
                ROS_WARN_THROTTLE(5.0,
                                  "yunlink_ros_bridge has not received px4_state yet, body velocity fallback fixed_height=0");
            }
            if (std::fabs(view.payload.vz_mps) > 1e-4F) {
                ROS_WARN_THROTTLE(5.0,
                                  "yunlink_ros_bridge body velocity ignores vz_mps=%f and keeps fixed_height=%f",
                                  view.payload.vz_mps,
                                  cmd.fixed_height);
            }
            detail += " fixed_height=" + std::to_string(cmd.fixed_height);
        } else {
            cmd.desired_vel.x = view.payload.vx_mps;
            cmd.desired_vel.y = view.payload.vy_mps;
            cmd.desired_vel.z = view.payload.vz_mps;
        }

        publishControlCmd("velocity_setpoint",
                          cmd,
                          view.inbound.envelope.session_id,
                          view.inbound.envelope.correlation_id,
                          view.inbound.envelope.message_id,
                          detail);
    }

    void onLocalOdom(const nav_msgs::Odometry::ConstPtr& msg) {
        const yunlink::LocalOdomSnapshot payload = mapLocalOdom(*msg);
        publishSnapshot("local_odom", &yunlink::Runtime::publish_local_odom, payload);
    }

    void onOdomState(const sunray_msgs::OdomState::ConstPtr& msg) {
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

    void onControlState(const sunray_msgs::UAVControlState::ConstPtr& msg) {
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

    void onMavrosState(const mavros_msgs::State::ConstPtr& msg) {
        yunlink::MavrosStateSnapshot payload{};
        payload.header = mapHeader(msg->header);
        payload.connected = msg->connected;
        payload.armed = msg->armed;
        payload.guided = msg->guided;
        payload.manual_input = msg->manual_input;
        payload.mode = msg->mode;
        payload.system_status = msg->system_status;
        publishSnapshot("mavros_state", &yunlink::Runtime::publish_mavros_state, payload);
    }

    void onPx4State(const sunray_msgs::Px4State::ConstPtr& msg) {
        {
            std::lock_guard<std::mutex> lock(px4_state_mu_);
            latest_px4_height_m_ = static_cast<float>(msg->local_pose.position.z);
            has_px4_height_ = true;
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

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber local_odom_sub_;
    ros::Subscriber odom_state_sub_;
    ros::Subscriber control_state_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Subscriber px4_state_sub_;
    ros::Publisher control_cmd_pub_;
    ros::Timer reconnect_timer_;

    yunlink::Runtime runtime_;

    std::string local_odom_topic_;
    std::string odom_state_topic_;
    std::string control_state_topic_;
    std::string control_cmd_topic_;
    std::string mavros_state_topic_;
    std::string px4_state_topic_;

    std::string remote_ip_;
    std::string shared_secret_;
    std::string node_name_;
    std::string peer_id_;

    int remote_tcp_port_{0};
    int udp_bind_port_{9696};
    int udp_target_port_{9898};
    int tcp_listen_port_{9696};
    int agent_id_{1};

    size_t takeoff_command_sub_token_{0};
    size_t land_command_sub_token_{0};
    size_t return_command_sub_token_{0};
    size_t goto_command_sub_token_{0};
    size_t velocity_setpoint_command_sub_token_{0};

    uint64_t session_id_{0};
    bool peer_ready_{false};

    mutable std::mutex px4_state_mu_;
    float latest_px4_height_m_{0.0F};
    bool has_px4_height_{false};
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "yunlink_ros_bridge_node");
    YunlinkRosBridgeNode node;
    ros::spin();
    return 0;
}
