/** @file @brief bridge runtime 参数和 QoS 配置类型。 */
#pragma once

#include <map>
#include <string>

/** @brief bridge 的 ROS topic、YunLink 端口、诊断和发现参数集合。 */
struct BridgeParams {
    std::string local_odom_topic{"/uav1/sunray/localization/local_odom"};
    std::string odom_state_topic{"/uav1/sunray/localization/odom_state"};
    std::string control_state_topic{"/uav1/sunray/uav_control/control_state"};
    std::string command_execution_status_topic{"/uav1/sunray/uav_control/command_execution_status"};
    std::string control_cmd_topic{"/uav1/sunray/uav_control/control_cmd"};
    std::string px4_state_topic{"/uav1/sunray/px4_state"};
    std::string external_odom_topic;
    std::string global_odom_topic{"/uav1/sunray/localization/global_odom"};
    bool enable_system_services{true};
    bool enable_runtime_diagnostics{true};
    std::string sunray_system_ns{"/sunray_system"};
    double system_service_timeout_sec{3.0};
    double diagnostic_publish_rate_hz{2.0};
    int diagnostic_stale_timeout_ms{1000};
    std::string monitor_diagnostic_topic{"/yunlink_ros_bridge/monitor_diagnostics"};
    bool default_start_with_terminal{false};
    std::string remote_ip;
    std::string shared_secret{"yunlink-default-secret"};
    std::string node_name{"yunlink_ros_bridge_node"};
    int remote_tcp_port{0};
    int udp_bind_port{9696};
    int udp_target_port{9898};
    int tcp_listen_port{9696};
    int agent_id{1};
    bool enable_endpoint_discovery{true};
    int discovery_port{9966};
    std::string discovery_target_ip{"255.255.255.255"};
    int discovery_period_ms{1000};
    std::string endpoint_name_prefix{"yundrone_uav"};
    std::string endpoint_id_file;
    std::string qos_profile{"balanced"};
    bool qos_udp_fallback_to_tcp{true};
};

/** @brief bridge 语义通道的 QoS 和 ROS 层覆盖参数。 */
struct BridgeChannelQosConfig {
    std::string qos_class;
    std::string transport;
    int queue_size{0};
    bool has_queue_size{false};
    bool tcp_no_delay{true};
    bool has_tcp_no_delay{false};
    double timeout_sec{0.0};
    bool has_timeout_sec{false};
    double publish_hz{0.0};
    bool has_publish_hz{false};
    bool configured{false};
};

/** @brief 按方向保存 YAML 中的 qos_channels 覆盖项。 */
struct BridgeQosChannels {
    std::map<std::string, BridgeChannelQosConfig> ros_to_yunlink;
    std::map<std::string, BridgeChannelQosConfig> yunlink_to_ros;
    std::map<std::string, BridgeChannelQosConfig> system_services;
};
