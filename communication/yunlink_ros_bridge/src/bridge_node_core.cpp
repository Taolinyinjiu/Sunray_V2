#include "bridge_node.hpp"

#include <algorithm>

YunlinkRosBridgeNode::YunlinkRosBridgeNode() : nh_(), pnh_("~") {
    loadParams();
    startRuntime();
    setupSubscribers();
    setupReconnectTimer();
}

YunlinkRosBridgeNode::~YunlinkRosBridgeNode() {
    runtime_.stop();
}

void YunlinkRosBridgeNode::loadParams() {
    pnh_.param<std::string>("local_odom_topic",
                            params_.local_odom_topic,
                            params_.local_odom_topic);
    pnh_.param<std::string>("odom_state_topic",
                            params_.odom_state_topic,
                            params_.odom_state_topic);
    pnh_.param<std::string>("control_state_topic",
                            params_.control_state_topic,
                            params_.control_state_topic);
    pnh_.param<std::string>("control_cmd_topic",
                            params_.control_cmd_topic,
                            params_.control_cmd_topic);
    pnh_.param<std::string>("mavros_state_topic",
                            params_.mavros_state_topic,
                            params_.mavros_state_topic);
    pnh_.param<std::string>("px4_state_topic", params_.px4_state_topic, params_.px4_state_topic);
    pnh_.param<std::string>("remote_ip", params_.remote_ip, params_.remote_ip);
    pnh_.param<int>("remote_tcp_port", params_.remote_tcp_port, params_.remote_tcp_port);
    pnh_.param<int>("udp_bind_port", params_.udp_bind_port, params_.udp_bind_port);
    pnh_.param<int>("udp_target_port", params_.udp_target_port, params_.udp_target_port);
    pnh_.param<int>("tcp_listen_port", params_.tcp_listen_port, params_.tcp_listen_port);
    pnh_.param<int>("agent_id", params_.agent_id, params_.agent_id);
    pnh_.param<std::string>("shared_secret", params_.shared_secret, params_.shared_secret);
    pnh_.param<std::string>("node_name", params_.node_name, params_.node_name);
}

void YunlinkRosBridgeNode::startRuntime() {
    yunlink::RuntimeConfig cfg;
    cfg.udp_bind_port = clampPort(params_.udp_bind_port);
    cfg.udp_target_port = clampPort(params_.udp_target_port);
    cfg.tcp_listen_port = clampPort(params_.tcp_listen_port);
    cfg.shared_secret = params_.shared_secret;
    cfg.self_identity.agent_type = yunlink::AgentType::kUav;
    cfg.self_identity.agent_id = static_cast<uint32_t>(std::max(params_.agent_id, 0));
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
             params_.agent_id);
}

void YunlinkRosBridgeNode::setupSubscribers() {
    control_cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>(params_.control_cmd_topic, 20);
    local_odom_sub_ =
        nh_.subscribe(params_.local_odom_topic, 20, &YunlinkRosBridgeNode::onLocalOdom, this);
    odom_state_sub_ =
        nh_.subscribe(params_.odom_state_topic, 20, &YunlinkRosBridgeNode::onOdomState, this);
    control_state_sub_ =
        nh_.subscribe(params_.control_state_topic, 20, &YunlinkRosBridgeNode::onControlState, this);
    mavros_state_sub_ =
        nh_.subscribe(params_.mavros_state_topic, 20, &YunlinkRosBridgeNode::onMavrosState, this);
    px4_state_sub_ =
        nh_.subscribe(params_.px4_state_topic, 20, &YunlinkRosBridgeNode::onPx4State, this);
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

    ROS_INFO("yunlink_ros_bridge advertise control_cmd: %s", params_.control_cmd_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe local_odom: %s", params_.local_odom_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe odom_state: %s", params_.odom_state_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe control_state: %s", params_.control_state_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe mavros_state: %s", params_.mavros_state_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe px4_state: %s", params_.px4_state_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe yunlink commands: takeoff land return goto velocity_setpoint");
}

void YunlinkRosBridgeNode::setupReconnectTimer() {
    reconnect_timer_ = nh_.createTimer(
        ros::Duration(1.0), &YunlinkRosBridgeNode::onReconnectTimer, this, false, true);
}

uint16_t YunlinkRosBridgeNode::clampPort(int port) {
    if (port < 0) {
        return 0;
    }
    if (port > 65535) {
        return 65535;
    }
    return static_cast<uint16_t>(port);
}

bool YunlinkRosBridgeNode::ensurePeerReady() {
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

    if (params_.remote_ip.empty() || params_.remote_tcp_port <= 0) {
        ROS_WARN_THROTTLE(5.0,
                          "yunlink_ros_bridge has no remote_ip/remote_tcp_port and no inbound active session yet");
        return false;
    }

    std::string peer_id;
    const auto connect_ec =
        runtime_.tcp_clients().connect_peer(params_.remote_ip, clampPort(params_.remote_tcp_port), &peer_id);
    if (connect_ec != yunlink::ErrorCode::kOk) {
        ROS_WARN_THROTTLE(5.0,
                          "yunlink_ros_bridge connect_peer failed, ip=%s port=%d ec=%u",
                          params_.remote_ip.c_str(),
                          params_.remote_tcp_port,
                          static_cast<unsigned>(connect_ec));
        return false;
    }

    const uint64_t session_id = runtime_.session_client().open_active_session(peer_id, params_.node_name);
    if (session_id == 0) {
        ROS_WARN_THROTTLE(5.0, "yunlink_ros_bridge open_active_session failed, peer_id=%s", peer_id.c_str());
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

yunlink::TargetSelector YunlinkRosBridgeNode::targetSelector() const {
    return yunlink::TargetSelector::broadcast(yunlink::AgentType::kGroundStation);
}

void YunlinkRosBridgeNode::onReconnectTimer(const ros::TimerEvent&) {
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

float YunlinkRosBridgeNode::latestPx4Height(bool* has_height) const {
    std::lock_guard<std::mutex> lock(px4_state_mu_);
    if (has_height != nullptr) {
        *has_height = has_px4_height_;
    }
    return latest_px4_height_m_;
}

sunray_msgs::UAVControlCMD YunlinkRosBridgeNode::makeBaseControlCmd(uint8_t control_cmd) const {
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::SUNRAY_STATION;
    cmd.control_cmd = control_cmd;
    return cmd;
}

void YunlinkRosBridgeNode::publishControlCmd(const char* name,
                                             const sunray_msgs::UAVControlCMD& cmd,
                                             uint64_t session_id,
                                             uint64_t correlation_id,
                                             uint64_t message_id,
                                             const std::string& detail) {
    control_cmd_pub_.publish(cmd);

    std::string line = std::string("yunlink_ros_bridge forwarded ") + name + " to " +
                       params_.control_cmd_topic + " session_id=" + std::to_string(session_id) +
                       " correlation_id=" + std::to_string(correlation_id) + " msg_id=" +
                       std::to_string(message_id);
    if (!detail.empty()) {
        line += " " + detail;
    }
    ROS_INFO("%s", line.c_str());
}
