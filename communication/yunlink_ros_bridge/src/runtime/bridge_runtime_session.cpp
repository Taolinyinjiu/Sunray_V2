/** @file @brief YunLink peer session 发现、连接和重连维护实现。 */
#include "bridge_node.hpp"

/// 将配置端口限制在 uint16_t 范围内。
uint16_t YunlinkRosBridgeNode::clampPort(int port) {
    if (port < 0) {
        return 0;
    }
    if (port > 65535) {
        return 65535;
    }
    return static_cast<uint16_t>(port);
}

/// 确保有可用 YunLink peer session；优先复用入站连接，再按配置主动拨号。
bool YunlinkRosBridgeNode::ensurePeerReady() {
    if (peer_ready_) {
        return true;
    }

    // 优先复用已有入站 session，再考虑主动拨号。
    // 这样可以兼容 monitor 手动连接 bridge 的流程。
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
        std::lock_guard<std::mutex> lock(diag_mu_);
        last_session_error_ = "waiting inbound session";
        last_error_time_ = ros::Time::now();
        ROS_INFO_THROTTLE(5.0,
                          "yunlink_ros_bridge waiting for inbound monitor session; "
                          "active dialing disabled remote_ip='%s' remote_tcp_port=%d",
                          params_.remote_ip.c_str(),
                          params_.remote_tcp_port);
        return false;
    }

    std::string peer_id;
    {
        std::lock_guard<std::mutex> lock(diag_mu_);
        connect_attempt_count_ += 1;
    }
    const auto connect_ec =
        runtime_.tcp_clients().connect_peer(params_.remote_ip, clampPort(params_.remote_tcp_port), &peer_id);
    if (connect_ec != yunlink::ErrorCode::kOk) {
        {
            std::lock_guard<std::mutex> lock(diag_mu_);
            last_connect_error_ = "connect_peer failed ec=" + std::to_string(static_cast<unsigned>(connect_ec));
            last_error_time_ = ros::Time::now();
        }
        ROS_WARN_THROTTLE(5.0,
                          "yunlink_ros_bridge connect_peer failed, ip=%s port=%d ec=%u",
                          params_.remote_ip.c_str(),
                          params_.remote_tcp_port,
                          static_cast<unsigned>(connect_ec));
        return false;
    }

    const uint64_t session_id = runtime_.session_client().open_active_session(peer_id, params_.node_name);
    if (session_id == 0) {
        {
            std::lock_guard<std::mutex> lock(diag_mu_);
            last_session_error_ = "open_active_session failed peer_id=" + peer_id;
            last_error_time_ = ros::Time::now();
        }
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

/// 当前 bridge 默认向地面站广播 snapshot。
yunlink::TargetSelector YunlinkRosBridgeNode::targetSelector() const {
    return yunlink::TargetSelector::broadcast(yunlink::AgentType::kGroundStation);
}

/// 定期检查 session 活性，失效后清空缓存等待下一轮重连。
void YunlinkRosBridgeNode::onReconnectTimer(const ros::TimerEvent&) {
    if (!peer_ready_) {
        (void)ensurePeerReady();
        return;
    }

    yunlink::SessionDescriptor session{};
    if (!runtime_.session_server().describe_session(peer_id_, session_id_, &session) ||
        session.state != yunlink::SessionState::kActive) {
        {
            std::lock_guard<std::mutex> lock(diag_mu_);
            session_lost_count_ += 1;
            last_session_error_ = "peer session is no longer active";
            last_error_time_ = ros::Time::now();
        }
        peer_ready_ = false;
        session_id_ = 0;
        peer_id_.clear();
        ROS_WARN_THROTTLE(5.0, "yunlink_ros_bridge peer session is no longer active, retrying");
    }
}
