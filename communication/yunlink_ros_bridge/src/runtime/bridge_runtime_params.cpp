/** @file @brief bridge ROS 参数和 YAML QoS 覆盖项加载实现。 */
#include "bridge_node.hpp"

#include <cstdlib>

#include <XmlRpcValue.h>

namespace {

/// 判断 XmlRpcValue 是否包含指定 member。
bool has_member(const XmlRpc::XmlRpcValue& value, const char* key) {
    return value.getType() == XmlRpc::XmlRpcValue::TypeStruct && value.hasMember(key);
}

/// 从 XmlRpcValue 中读取字符串参数。
bool read_xml_string(const XmlRpc::XmlRpcValue& value, const char* key, std::string* out) {
    if (!has_member(value, key)) {
        return true;
    }
    const XmlRpc::XmlRpcValue& item = value[key];
    if (item.getType() != XmlRpc::XmlRpcValue::TypeString) {
        return false;
    }
    *out = static_cast<std::string>(item);
    return true;
}

/// 从 XmlRpcValue 中读取整数参数。
bool read_xml_int(const XmlRpc::XmlRpcValue& value, const char* key, int* out) {
    if (!has_member(value, key)) {
        return true;
    }
    const XmlRpc::XmlRpcValue& item = value[key];
    if (item.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        return false;
    }
    *out = static_cast<int>(item);
    return true;
}

/// 从 XmlRpcValue 中读取 bool 参数。
bool read_xml_bool(const XmlRpc::XmlRpcValue& value, const char* key, bool* out) {
    if (!has_member(value, key)) {
        return true;
    }
    const XmlRpc::XmlRpcValue& item = value[key];
    if (item.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
        return false;
    }
    *out = static_cast<bool>(item);
    return true;
}

/// 从 XmlRpcValue 中读取 double 参数，兼容 YAML 整数写法。
bool read_xml_double(const XmlRpc::XmlRpcValue& value, const char* key, double* out) {
    if (!has_member(value, key)) {
        return true;
    }
    const XmlRpc::XmlRpcValue& item = value[key];
    if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        *out = static_cast<double>(item);
        return true;
    }
    if (item.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        *out = static_cast<int>(item);
        return true;
    }
    return false;
}

/// 读取一个 qos_channels 分组。
bool read_qos_group(const XmlRpc::XmlRpcValue& root,
                    const char* group_name,
                    std::map<std::string, BridgeChannelQosConfig>* out,
                    std::string* error) {
    if (!has_member(root, group_name)) {
        return true;
    }
    const XmlRpc::XmlRpcValue& group = root[group_name];
    if (group.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        *error = std::string("qos_channels.") + group_name + " must be a map";
        return false;
    }
    for (auto it = group.begin(); it != group.end(); ++it) {
        const std::string channel = it->first;
        const XmlRpc::XmlRpcValue& value = it->second;
        if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            *error = std::string("qos_channels.") + group_name + "." + channel + " must be a map";
            return false;
        }

        BridgeChannelQosConfig cfg;
        cfg.configured = true;
        if (!read_xml_string(value, "qos_class", &cfg.qos_class)) {
            *error = std::string("qos_channels.") + group_name + "." + channel + ".qos_class must be a string";
            return false;
        }
        if (!read_xml_string(value, "transport", &cfg.transport)) {
            *error = std::string("qos_channels.") + group_name + "." + channel + ".transport must be a string";
            return false;
        }
        if (has_member(value, "queue_size")) {
            cfg.has_queue_size = true;
            if (!read_xml_int(value, "queue_size", &cfg.queue_size) || cfg.queue_size <= 0) {
                *error = std::string("qos_channels.") + group_name + "." + channel +
                         ".queue_size must be a positive integer";
                return false;
            }
        }
        if (has_member(value, "tcp_no_delay")) {
            cfg.has_tcp_no_delay = true;
            if (!read_xml_bool(value, "tcp_no_delay", &cfg.tcp_no_delay)) {
                *error = std::string("qos_channels.") + group_name + "." + channel +
                         ".tcp_no_delay must be a bool";
                return false;
            }
        }
        if (has_member(value, "timeout_sec")) {
            cfg.has_timeout_sec = true;
            if (!read_xml_double(value, "timeout_sec", &cfg.timeout_sec) || cfg.timeout_sec < 0.0) {
                *error = std::string("qos_channels.") + group_name + "." + channel +
                         ".timeout_sec must be a non-negative number";
                return false;
            }
        }
        if (has_member(value, "publish_hz")) {
            cfg.has_publish_hz = true;
            if (!read_xml_double(value, "publish_hz", &cfg.publish_hz) || cfg.publish_hz < 0.0) {
                *error = std::string("qos_channels.") + group_name + "." + channel +
                         ".publish_hz must be a non-negative number";
                return false;
            }
        }
        (*out)[channel] = cfg;
    }
    return true;
}

}  // namespace

/// 从 ROS 私有参数加载 bridge 配置，保留单机默认值作为 fallback。
void YunlinkRosBridgeNode::loadParams() {
    // 默认值对应单机启动路径；仿真、真机或多机布局通过 YAML/launch 参数覆盖。
    pnh_.param<std::string>("local_odom_topic", params_.local_odom_topic, params_.local_odom_topic);
    pnh_.param<std::string>("odom_state_topic", params_.odom_state_topic, params_.odom_state_topic);
    pnh_.param<std::string>("control_state_topic", params_.control_state_topic, params_.control_state_topic);
    pnh_.param<std::string>("command_execution_status_topic",
                            params_.command_execution_status_topic,
                            params_.command_execution_status_topic);
    pnh_.param<std::string>("control_cmd_topic", params_.control_cmd_topic, params_.control_cmd_topic);
    pnh_.param<std::string>("px4_state_topic", params_.px4_state_topic, params_.px4_state_topic);
    pnh_.param<std::string>("external_odom_topic", params_.external_odom_topic, params_.external_odom_topic);
    pnh_.param<std::string>("global_odom_topic", params_.global_odom_topic, params_.global_odom_topic);
    pnh_.param<bool>("enable_system_services", params_.enable_system_services, params_.enable_system_services);
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
    pnh_.param<std::string>("discovery_target_ip", params_.discovery_target_ip, params_.discovery_target_ip);
    pnh_.param<int>("discovery_period_ms", params_.discovery_period_ms, params_.discovery_period_ms);
    pnh_.param<std::string>("endpoint_name_prefix", params_.endpoint_name_prefix, params_.endpoint_name_prefix);
    const char* home_env = std::getenv("HOME");
    params_.endpoint_id_file = home_env != nullptr
                                   ? std::string(home_env) + "/.config/yunlink_ros_bridge/endpoint_id"
                                   : std::string("endpoint_id");
    pnh_.param<std::string>("endpoint_id_file", params_.endpoint_id_file, params_.endpoint_id_file);
    if (!loadQosParams()) {
        ros::shutdown();
    }
}

/// 读取 YunLink QoS profile 和 bridge 语义通道覆盖项。
bool YunlinkRosBridgeNode::loadQosParams() {
    XmlRpc::XmlRpcValue profile_value;
    if (pnh_.getParam("qos_profile", profile_value)) {
        if (profile_value.getType() == XmlRpc::XmlRpcValue::TypeString) {
            params_.qos_profile = static_cast<std::string>(profile_value);
        } else if (profile_value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            const int profile_id = static_cast<int>(profile_value);
            if (profile_id == 1) {
                params_.qos_profile = "reliable";
            } else if (profile_id == 2) {
                params_.qos_profile = "balanced";
            } else if (profile_id == 3) {
                params_.qos_profile = "low_latency";
            } else {
                ROS_FATAL("invalid qos config: qos_profile numeric alias must be 1, 2, or 3");
                return false;
            }
        } else {
            ROS_FATAL("invalid qos config: qos_profile must be a string or numeric alias");
            return false;
        }
    }
    pnh_.param<bool>("qos_udp_fallback_to_tcp",
                     params_.qos_udp_fallback_to_tcp,
                     params_.qos_udp_fallback_to_tcp);

    XmlRpc::XmlRpcValue channels;
    if (pnh_.getParam("qos_channels", channels)) {
        if (channels.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_FATAL("invalid qos config: qos_channels must be a map");
            return false;
        }
        std::string error;
        if (!read_qos_group(channels, "ros_to_yunlink", &qos_channels_.ros_to_yunlink, &error) ||
            !read_qos_group(channels, "yunlink_to_ros", &qos_channels_.yunlink_to_ros, &error) ||
            !read_qos_group(channels, "system_services", &qos_channels_.system_services, &error)) {
            ROS_FATAL("invalid qos config: %s", error.c_str());
            return false;
        }
    }

    std::string error;
    if (!validateQosChannels(&error)) {
        ROS_FATAL("invalid qos config: %s", error.c_str());
        return false;
    }
    return true;
}
