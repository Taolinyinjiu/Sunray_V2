/** @file @brief bridge runtime 初始化、参数加载和 ROS/YunLink 订阅设置实现。 */
#include "bridge_node.hpp"

#include <algorithm>
#include <sstream>

/// 按固定顺序初始化参数、诊断、YunLink runtime、ROS IO 和后台任务。
YunlinkRosBridgeNode::YunlinkRosBridgeNode() : nh_(), pnh_("~") {
    loadParams();
    setupDiagnosticState();
    setupMonitorState();
    startRuntime();
    setupEndpointDiscovery();
    setupSystemServiceClients();
    setupSubscribers();
    startSystemServiceWorker();
    setupReconnectTimer();
}

/// 停止后台 worker、端点发现和 YunLink runtime。
YunlinkRosBridgeNode::~YunlinkRosBridgeNode() {
    stopSystemServiceWorker();
    endpoint_discovery_.stop();
    runtime_.stop();
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
    cfg.qos_policy = makeRuntimeQosPolicy();
    cfg.qos_channel_policies = makeRuntimeQosChannelPolicies();

    const auto ec = runtime_.start(cfg);
    if (ec != yunlink::ErrorCode::kOk) {
        {
            std::lock_guard<std::mutex> lock(diag_mu_);
            runtime_started_ = false;
            last_connect_error_ = "runtime start failed ec=" + std::to_string(static_cast<unsigned>(ec));
            last_error_time_ = ros::Time::now();
        }
        ROS_FATAL("yunlink runtime start failed, error code=%u", static_cast<unsigned>(ec));
        ros::shutdown();
        return;
    }
    {
        std::lock_guard<std::mutex> lock(diag_mu_);
        runtime_started_ = true;
    }

    ROS_INFO("yunlink runtime started: udp_bind=%u udp_target=%u tcp_listen=%u agent_id=%d",
             cfg.udp_bind_port, cfg.udp_target_port, cfg.tcp_listen_port, params_.agent_id);
    logQosConfig();
    if (params_.remote_ip.empty() || params_.remote_tcp_port <= 0) {
        // 被动模式是默认 monitor 工作流：bridge 监听端口，
        // endpoint discovery 告诉 monitor 应该连接哪里。
        ROS_INFO("yunlink_ros_bridge passive connection mode: waiting for monitor inbound session on tcp_listen=%u",
                 cfg.tcp_listen_port);
    } else {
        ROS_INFO("yunlink_ros_bridge active dialing enabled: remote=%s:%d",
                 params_.remote_ip.c_str(), params_.remote_tcp_port);
    }
}

/// 配置 UDP 端点发现广播，让 monitor 能发现被动监听的 bridge。
void YunlinkRosBridgeNode::setupEndpointDiscovery() {
    if (!params_.enable_endpoint_discovery) {
        ROS_INFO("yunlink_ros_bridge endpoint discovery disabled");
        return;
    }

    std::string error;
    if (!endpoint_discovery_.configure(params_, &error)) {
        ROS_WARN("yunlink_ros_bridge endpoint discovery init failed: %s", error.c_str());
        return;
    }

    ROS_INFO("yunlink_ros_bridge endpoint discovery ready: %s target=%s:%d endpoint_id=%s",
             endpoint_discovery_.display_name().c_str(),
             params_.discovery_target_ip.c_str(),
             params_.discovery_port,
             endpoint_discovery_.endpoint_id().c_str());
}

/// 创建重连、诊断发布和端点发现定时器。
void YunlinkRosBridgeNode::setupReconnectTimer() {
    reconnect_timer_ = nh_.createTimer(
        ros::Duration(1.0), &YunlinkRosBridgeNode::onReconnectTimer, this, false, true);
    const double hz = std::max(0.1, params_.diagnostic_publish_rate_hz);
    diagnostic_timer_ = nh_.createTimer(
        ros::Duration(1.0 / hz), &YunlinkRosBridgeNode::onDiagnosticTimer, this, false, true);
    if (params_.enable_endpoint_discovery && endpoint_discovery_.enabled()) {
        const double period_sec =
            std::max(0.1, static_cast<double>(params_.discovery_period_ms) / 1000.0);
        endpoint_discovery_timer_ = nh_.createTimer(
            ros::Duration(period_sec),
            &YunlinkRosBridgeNode::onEndpointDiscoveryTimer,
            this,
            false,
            true);
    }
}
