/** @file @brief bridge runtime 初始化、参数加载和 ROS/YunLink 订阅设置实现。 */
#include "bridge_node.hpp"

#include <algorithm>
#include <cstdlib>

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

/// 从 ROS 私有参数加载 bridge 配置，保留单机默认值作为 fallback。
void YunlinkRosBridgeNode::loadParams() {
    // 默认值对应单机启动路径；仿真、真机或多机布局通过 YAML/launch 参数覆盖。
    pnh_.param<std::string>("local_odom_topic",
                            params_.local_odom_topic,
                            params_.local_odom_topic);
    pnh_.param<std::string>("odom_state_topic",
                            params_.odom_state_topic,
                            params_.odom_state_topic);
    pnh_.param<std::string>("control_state_topic",
                            params_.control_state_topic,
                            params_.control_state_topic);
    pnh_.param<std::string>("command_execution_status_topic",
                            params_.command_execution_status_topic,
                            params_.command_execution_status_topic);
    pnh_.param<std::string>("control_cmd_topic",
                            params_.control_cmd_topic,
                            params_.control_cmd_topic);
    pnh_.param<std::string>("px4_state_topic", params_.px4_state_topic, params_.px4_state_topic);
    pnh_.param<std::string>(
        "external_odom_topic", params_.external_odom_topic, params_.external_odom_topic);
    pnh_.param<std::string>(
        "global_odom_topic", params_.global_odom_topic, params_.global_odom_topic);
    pnh_.param<bool>(
        "enable_system_services", params_.enable_system_services, params_.enable_system_services);
    pnh_.param<bool>("enable_runtime_diagnostics",
                     params_.enable_runtime_diagnostics,
                     params_.enable_runtime_diagnostics);
    pnh_.param<std::string>("sunray_system_ns", params_.sunray_system_ns, params_.sunray_system_ns);
    pnh_.param<double>("system_service_timeout_sec",
                       params_.system_service_timeout_sec,
                       params_.system_service_timeout_sec);
    pnh_.param<double>("diagnostic_publish_rate_hz",
                       params_.diagnostic_publish_rate_hz,
                       params_.diagnostic_publish_rate_hz);
    pnh_.param<int>("diagnostic_stale_timeout_ms",
                    params_.diagnostic_stale_timeout_ms,
                    params_.diagnostic_stale_timeout_ms);
    pnh_.param<std::string>("monitor_diagnostic_topic",
                            params_.monitor_diagnostic_topic,
                            params_.monitor_diagnostic_topic);
    pnh_.param<bool>("default_start_with_terminal",
                     params_.default_start_with_terminal,
                     params_.default_start_with_terminal);
    pnh_.param<std::string>("remote_ip", params_.remote_ip, params_.remote_ip);
    pnh_.param<int>("remote_tcp_port", params_.remote_tcp_port, params_.remote_tcp_port);
    pnh_.param<int>("udp_bind_port", params_.udp_bind_port, params_.udp_bind_port);
    pnh_.param<int>("udp_target_port", params_.udp_target_port, params_.udp_target_port);
    pnh_.param<int>("tcp_listen_port", params_.tcp_listen_port, params_.tcp_listen_port);
    pnh_.param<int>("agent_id", params_.agent_id, params_.agent_id);
    pnh_.param<std::string>("shared_secret", params_.shared_secret, params_.shared_secret);
    pnh_.param<std::string>("node_name", params_.node_name, params_.node_name);
    pnh_.param<bool>("enable_endpoint_discovery",
                     params_.enable_endpoint_discovery,
                     params_.enable_endpoint_discovery);
    pnh_.param<int>("discovery_port", params_.discovery_port, params_.discovery_port);
    pnh_.param<std::string>("discovery_target_ip",
                            params_.discovery_target_ip,
                            params_.discovery_target_ip);
    pnh_.param<int>("discovery_period_ms",
                    params_.discovery_period_ms,
                    params_.discovery_period_ms);
    pnh_.param<std::string>("endpoint_name_prefix",
                            params_.endpoint_name_prefix,
                            params_.endpoint_name_prefix);
    const char* home_env = std::getenv("HOME");
    const std::string default_endpoint_id_file =
        home_env != nullptr ? std::string(home_env) + "/.config/yunlink_ros_bridge/endpoint_id"
                            : std::string("endpoint_id");
    params_.endpoint_id_file = default_endpoint_id_file;
    pnh_.param<std::string>("endpoint_id_file",
                            params_.endpoint_id_file,
                            params_.endpoint_id_file);
}

/// 启动 YunLink runtime，并记录主动拨号或被动监听模式。
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

/// 建立 ROS topic 订阅/发布和 YunLink command/system-service 订阅。
void YunlinkRosBridgeNode::setupSubscribers() {
    control_cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>(params_.control_cmd_topic, 20);
    monitor_diagnostic_pub_ =
        nh_.advertise<diagnostic_msgs::DiagnosticArray>(params_.monitor_diagnostic_topic, 1, true);
    ros::TransportHints latest_hints;
    latest_hints.tcpNoDelay();
    // 状态 snapshot 只关心最新值，不需要积压历史消息。
    local_odom_sub_ =
        nh_.subscribe(params_.local_odom_topic, 1, &YunlinkRosBridgeNode::onLocalOdom, this, latest_hints);
    odom_state_sub_ =
        nh_.subscribe(params_.odom_state_topic, 1, &YunlinkRosBridgeNode::onOdomState, this, latest_hints);
    control_cmd_sub_ =
        nh_.subscribe(params_.control_cmd_topic, 1, &YunlinkRosBridgeNode::onControlCmd, this, latest_hints);
    control_state_sub_ =
        nh_.subscribe(params_.control_state_topic, 1, &YunlinkRosBridgeNode::onControlState, this, latest_hints);
    command_execution_status_sub_ = nh_.subscribe(params_.command_execution_status_topic,
                                                  1,
                                                  &YunlinkRosBridgeNode::onCommandExecutionStatus,
                                                  this,
                                                  latest_hints);
    px4_state_sub_ =
        nh_.subscribe(params_.px4_state_topic, 1, &YunlinkRosBridgeNode::onPx4State, this, latest_hints);
    if (params_.enable_runtime_diagnostics && !params_.external_odom_topic.empty()) {
        external_odom_diag_sub_ = nh_.subscribe(params_.external_odom_topic,
                                                1,
                                                &YunlinkRosBridgeNode::onExternalOdomDiagnostic,
                                                this,
                                                latest_hints);
    }
    if (params_.enable_runtime_diagnostics && !params_.global_odom_topic.empty()) {
        global_odom_diag_sub_ = nh_.subscribe(params_.global_odom_topic,
                                              1,
                                              &YunlinkRosBridgeNode::onGlobalOdomDiagnostic,
                                              this,
                                              latest_hints);
    }
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
    if (params_.enable_system_services) {
        feature_list_request_sub_token_ =
            runtime_.system_service_subscriber().subscribe_feature_list_requests(
                [this](const yunlink::InboundSystemServiceRequestView<yunlink::FeatureListRequest>& view) {
                    onFeatureListRequest(view);
                });
        feature_get_request_sub_token_ =
            runtime_.system_service_subscriber().subscribe_feature_get_requests(
                [this](const yunlink::InboundSystemServiceRequestView<yunlink::FeatureGetRequest>& view) {
                    onFeatureGetRequest(view);
                });
        feature_start_request_sub_token_ =
            runtime_.system_service_subscriber().subscribe_feature_start_requests(
                [this](const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStartRequest>& view) {
                    onFeatureStartRequest(view);
                });
        feature_stop_request_sub_token_ =
            runtime_.system_service_subscriber().subscribe_feature_stop_requests(
                [this](const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStopRequest>& view) {
                    onFeatureStopRequest(view);
                });
    }

    ROS_INFO("yunlink_ros_bridge advertise control_cmd: %s", params_.control_cmd_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe local_odom: %s", params_.local_odom_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe odom_state: %s", params_.odom_state_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe control_cmd: %s", params_.control_cmd_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe control_state: %s", params_.control_state_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe command_execution_status: %s",
             params_.command_execution_status_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe px4_state: %s", params_.px4_state_topic.c_str());
    if (params_.enable_runtime_diagnostics) {
        ROS_INFO("yunlink_ros_bridge diagnostic external_odom: %s",
                 params_.external_odom_topic.empty() ? "<empty>" : params_.external_odom_topic.c_str());
        ROS_INFO("yunlink_ros_bridge diagnostic global_odom: %s",
                 params_.global_odom_topic.empty() ? "<empty>" : params_.global_odom_topic.c_str());
    }
    ROS_INFO("yunlink_ros_bridge monitor diagnostics topic: %s",
             params_.monitor_diagnostic_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe yunlink commands: takeoff land return goto velocity_setpoint");
    if (params_.enable_system_services) {
        ROS_INFO("yunlink_ros_bridge subscribe yunlink system services: feature_list feature_get feature_start feature_stop");
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
