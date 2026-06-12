#include "bridge_node.hpp"

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
        ROS_INFO_THROTTLE(5.0,
                          "yunlink_ros_bridge waiting for inbound monitor session; "
                          "active dialing disabled remote_ip='%s' remote_tcp_port=%d",
                          params_.remote_ip.c_str(),
                          params_.remote_tcp_port);
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
