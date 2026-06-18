/** @file @brief bridge QoS profile、通道覆盖项和发送节流实现。 */
#include "bridge_node.hpp"

#include <cctype>
#include <vector>

namespace {

struct RuntimeQosPolicySpec {
    const char* channel;
    yunlink::MessageFamily family;
    uint16_t message_type;
    yunlink::QosClass default_qos;
    yunlink::TransportPreference default_transport;
};

/// 标准化 QoS token，供文件级解析 helper 使用。
std::string normalize_qos_token_local(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (char ch : value) {
        if (ch == '-') {
            out.push_back('_');
        } else {
            out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
        }
    }
    return out;
}

/// 将字符串 QoS class 转换为 libyunlink enum。
yunlink::QosClass parse_qos_class_or_default(const std::string& value,
                                             yunlink::QosClass fallback) {
    const std::string token = normalize_qos_token_local(value);
    if (token == "reliable_ordered") {
        return yunlink::QosClass::kReliableOrdered;
    }
    if (token == "reliable_latest") {
        return yunlink::QosClass::kReliableLatest;
    }
    if (token == "best_effort") {
        return yunlink::QosClass::kBestEffort;
    }
    if (token == "bulk") {
        return yunlink::QosClass::kBulk;
    }
    return fallback;
}

/// 将字符串 transport 转换为 libyunlink enum。
yunlink::TransportPreference
parse_transport_or_default(const std::string& value, yunlink::TransportPreference fallback) {
    const std::string token = normalize_qos_token_local(value);
    if (token == "tcp") {
        return yunlink::TransportPreference::kTcp;
    }
    if (token == "udp") {
        return yunlink::TransportPreference::kUdp;
    }
    return fallback;
}

}  // namespace

/// 统一大小写和分隔符，允许 YAML 中使用 low-latency / low_latency 等写法。
std::string YunlinkRosBridgeNode::normalizeQosToken(const std::string& value) {
    return normalize_qos_token_local(value);
}

/// 格式化 QoS profile 名称。
const char* YunlinkRosBridgeNode::qosProfileName(yunlink::QosProfile profile) {
    switch (profile) {
    case yunlink::QosProfile::kReliable:
        return "reliable";
    case yunlink::QosProfile::kBalanced:
        return "balanced";
    case yunlink::QosProfile::kLowLatency:
        return "low_latency";
    }
    return "unknown";
}

/// 格式化传输偏好名称。
const char* YunlinkRosBridgeNode::transportName(yunlink::TransportPreference transport) {
    switch (transport) {
    case yunlink::TransportPreference::kTcp:
        return "tcp";
    case yunlink::TransportPreference::kUdp:
        return "udp";
    }
    return "unknown";
}

/// 将 bridge 参数转换成 libyunlink RuntimeConfig 的 QoS policy。
yunlink::RuntimeQosPolicy YunlinkRosBridgeNode::makeRuntimeQosPolicy() const {
    yunlink::RuntimeQosPolicy policy;
    const std::string profile = normalizeQosToken(params_.qos_profile);
    if (profile == "reliable" || profile == "1") {
        policy.profile = yunlink::QosProfile::kReliable;
        policy.reliable_ordered = yunlink::TransportPreference::kTcp;
        policy.reliable_latest = yunlink::TransportPreference::kTcp;
        policy.best_effort = yunlink::TransportPreference::kTcp;
        policy.bulk = yunlink::TransportPreference::kTcp;
    } else if (profile == "low_latency" || profile == "lowlatency" || profile == "3") {
        policy.profile = yunlink::QosProfile::kLowLatency;
        policy.reliable_ordered = yunlink::TransportPreference::kTcp;
        policy.reliable_latest = yunlink::TransportPreference::kUdp;
        policy.best_effort = yunlink::TransportPreference::kUdp;
        policy.bulk = yunlink::TransportPreference::kUdp;
    } else {
        policy.profile = yunlink::QosProfile::kBalanced;
        policy.reliable_ordered = yunlink::TransportPreference::kTcp;
        policy.reliable_latest = yunlink::TransportPreference::kTcp;
        policy.best_effort = yunlink::TransportPreference::kUdp;
        policy.bulk = yunlink::TransportPreference::kUdp;
    }
    policy.udp_fallback_to_tcp = params_.qos_udp_fallback_to_tcp;
    return policy;
}

/// 将 bridge 语义通道覆盖转换成 runtime message-level QoS policy。
std::vector<yunlink::RuntimeQosChannelPolicy>
YunlinkRosBridgeNode::makeRuntimeQosChannelPolicies() const {
    std::vector<yunlink::RuntimeQosChannelPolicy> policies;
    using Family = yunlink::MessageFamily;
    using Qos = yunlink::QosClass;
    using Transport = yunlink::TransportPreference;
    const RuntimeQosPolicySpec specs[] = {
        {"local_odom", Family::kStateSnapshot, static_cast<uint16_t>(yunlink::StateSnapshotType::kLocalOdom),
         Qos::kReliableLatest, Transport::kTcp},
        {"odom_state", Family::kStateSnapshot, static_cast<uint16_t>(yunlink::StateSnapshotType::kOdomState),
         Qos::kReliableLatest, Transport::kTcp},
        {"control_state", Family::kStateSnapshot,
         static_cast<uint16_t>(yunlink::StateSnapshotType::kUavControlState), Qos::kReliableLatest, Transport::kTcp},
        {"uav_control_cmd", Family::kStateSnapshot,
         static_cast<uint16_t>(yunlink::StateSnapshotType::kUavControlCmd), Qos::kReliableLatest, Transport::kTcp},
        {"command_execution_status", Family::kStateSnapshot,
         static_cast<uint16_t>(yunlink::StateSnapshotType::kCommandExecutionStatus),
         Qos::kReliableLatest, Transport::kTcp},
        {"px4_state", Family::kStateSnapshot, static_cast<uint16_t>(yunlink::StateSnapshotType::kPx4State),
         Qos::kReliableLatest, Transport::kTcp},
        {"runtime_diagnostic", Family::kStateSnapshot,
         static_cast<uint16_t>(yunlink::StateSnapshotType::kSunrayRuntimeDiagnostic),
         Qos::kReliableLatest, Transport::kTcp},
        {"vehicle_event", Family::kStateEvent, static_cast<uint16_t>(yunlink::StateEventType::kVehicleEvent),
         Qos::kBestEffort, Transport::kUdp},
    };

    for (const RuntimeQosPolicySpec& spec : specs) {
        const auto it = qos_channels_.ros_to_yunlink.find(spec.channel);
        if (it == qos_channels_.ros_to_yunlink.end()) {
            continue;
        }
        const BridgeChannelQosConfig& cfg = it->second;
        if (cfg.qos_class.empty() && cfg.transport.empty()) {
            continue;
        }
        yunlink::RuntimeQosChannelPolicy policy;
        policy.message_family = spec.family;
        policy.message_type = spec.message_type;
        policy.qos_class = parse_qos_class_or_default(cfg.qos_class, spec.default_qos);
        policy.transport = parse_transport_or_default(cfg.transport, spec.default_transport);
        policies.push_back(policy);
    }
    return policies;
}

/// 校验 QoS profile、通道 token，以及关键通道不可降级约束。
bool YunlinkRosBridgeNode::validateQosChannels(std::string* error) const {
    const std::string profile = normalizeQosToken(params_.qos_profile);
    if (profile != "reliable" && profile != "balanced" && profile != "low_latency" &&
        profile != "lowlatency" && profile != "1" && profile != "2" && profile != "3") {
        *error = "qos_profile must be reliable, balanced, low_latency, or numeric alias 1/2/3";
        return false;
    }

    const auto valid_qos_class = [](const std::string& value) {
        return value.empty() || value == "reliable_ordered" || value == "reliable_latest" ||
               value == "best_effort" || value == "bulk";
    };
    const auto valid_transport = [](const std::string& value) {
        return value.empty() || value == "tcp" || value == "udp";
    };
    const auto validate_group = [&](const std::map<std::string, BridgeChannelQosConfig>& group,
                                    const char* group_name) {
        for (const auto& entry : group) {
            const std::string qos_class = normalizeQosToken(entry.second.qos_class);
            const std::string transport = normalizeQosToken(entry.second.transport);
            if (!valid_qos_class(qos_class)) {
                *error = std::string(group_name) + "." + entry.first + " has unsupported qos_class";
                return false;
            }
            if (!valid_transport(transport)) {
                *error = std::string(group_name) + "." + entry.first + " has unsupported transport";
                return false;
            }
        }
        return true;
    };
    if (!validate_group(qos_channels_.ros_to_yunlink, "ros_to_yunlink") ||
        !validate_group(qos_channels_.yunlink_to_ros, "yunlink_to_ros") ||
        !validate_group(qos_channels_.system_services, "system_services")) {
        return false;
    }

    const auto require_reliable_tcp = [&](const std::map<std::string, BridgeChannelQosConfig>& group,
                                          const char* group_name,
                                          const std::vector<std::string>& channels) {
        for (const auto& channel : channels) {
            const auto it = group.find(channel);
            if (it == group.end()) {
                continue;
            }
            const std::string qos_class = normalizeQosToken(it->second.qos_class);
            const std::string transport = normalizeQosToken(it->second.transport);
            if ((!qos_class.empty() && qos_class != "reliable_ordered") ||
                (!transport.empty() && transport != "tcp")) {
                *error = std::string(group_name) + "." + channel + " requires reliable_ordered/tcp";
                return false;
            }
        }
        return true;
    };

    return require_reliable_tcp(qos_channels_.yunlink_to_ros,
                                "yunlink_to_ros",
                                {"takeoff", "takeoff_command", "land", "land_command", "return",
                                 "return_command", "goto", "goto_command", "velocity_setpoint",
                                 "velocity_setpoint_command", "command_result", "session", "authority"}) &&
           require_reliable_tcp(qos_channels_.ros_to_yunlink,
                                "ros_to_yunlink",
                                {"command_result", "session", "authority"}) &&
           require_reliable_tcp(qos_channels_.system_services,
                                "system_services",
                                {"feature_list", "feature_get", "feature_start", "feature_stop"});
}
