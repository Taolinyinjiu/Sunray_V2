/** @file @brief bridge QoS 通道查询、ROS 参数派生和日志摘要实现。 */
#include "bridge_node.hpp"

#include <sstream>

/// 查找通道配置。
BridgeChannelQosConfig
YunlinkRosBridgeNode::channelQos(const std::map<std::string, BridgeChannelQosConfig>& group,
                                 const char* channel) const {
    const auto it = group.find(channel);
    return it == group.end() ? BridgeChannelQosConfig{} : it->second;
}

/// 获取通道 ROS queue_size，未配置时使用 fallback。
int YunlinkRosBridgeNode::queueSize(const std::map<std::string, BridgeChannelQosConfig>& group,
                                    const char* channel,
                                    int fallback) const {
    const auto cfg = channelQos(group, channel);
    return cfg.has_queue_size ? cfg.queue_size : fallback;
}

/// 获取通道 ROS tcp_no_delay，未配置时使用 fallback。
bool YunlinkRosBridgeNode::tcpNoDelay(const std::map<std::string, BridgeChannelQosConfig>& group,
                                      const char* channel,
                                      bool fallback) const {
    const auto cfg = channelQos(group, channel);
    return cfg.has_tcp_no_delay ? cfg.tcp_no_delay : fallback;
}

/// 获取通道发往 YunLink 的最高频率，0 表示不节流。
double YunlinkRosBridgeNode::publishHz(const std::map<std::string, BridgeChannelQosConfig>& group,
                                       const char* channel,
                                       double fallback) const {
    const auto cfg = channelQos(group, channel);
    return cfg.has_publish_hz ? cfg.publish_hz : fallback;
}

/// 判断 ROS->YunLink 通道是否到达 YAML 配置的发送周期。
bool YunlinkRosBridgeNode::shouldPublishRosToYunlink(const char* channel, const ros::Time& now) {
    if (channel == nullptr) {
        return true;
    }
    const double hz = publishHz(qos_channels_.ros_to_yunlink, channel, 0.0);
    if (hz <= 0.0) {
        return true;
    }

    const std::string key(channel);
    std::lock_guard<std::mutex> lock(diag_mu_);
    auto& last_time = last_ros_to_yunlink_publish_time_[key];
    if (!last_time.isZero() && (now - last_time).toSec() < (1.0 / hz)) {
        return false;
    }
    last_time = now;
    return true;
}

/// 输出 QoS 配置摘要。
void YunlinkRosBridgeNode::logQosConfig() const {
    const auto policy = makeRuntimeQosPolicy();
    ROS_INFO("yunlink_ros_bridge qos profile=%s udp_fallback_to_tcp=%s route_phase=runtime_qos",
             qosProfileName(policy.profile),
             policy.udp_fallback_to_tcp ? "true" : "false");
    ROS_INFO("yunlink_ros_bridge qos policy: reliable_ordered=%s reliable_latest=%s best_effort=%s bulk=%s",
             transportName(policy.reliable_ordered),
             transportName(policy.reliable_latest),
             transportName(policy.best_effort),
             transportName(policy.bulk));

    const auto log_group = [](const char* group_name,
                              const std::map<std::string, BridgeChannelQosConfig>& group) {
        for (const auto& entry : group) {
            const auto& cfg = entry.second;
            std::ostringstream oss;
            oss << "yunlink_ros_bridge qos channel " << group_name << "." << entry.first;
            if (!cfg.qos_class.empty()) {
                oss << " qos_class=" << cfg.qos_class;
            }
            if (!cfg.transport.empty()) {
                oss << " transport=" << cfg.transport;
            }
            if (cfg.has_queue_size) {
                oss << " queue_size=" << cfg.queue_size;
            }
            if (cfg.has_tcp_no_delay) {
                oss << " tcp_no_delay=" << (cfg.tcp_no_delay ? "true" : "false");
            }
            if (cfg.has_timeout_sec) {
                oss << " timeout_sec=" << cfg.timeout_sec;
            }
            if (cfg.has_publish_hz) {
                oss << " publish_hz=" << cfg.publish_hz;
            }
            ROS_INFO("%s", oss.str().c_str());
        }
    };
    log_group("ros_to_yunlink", qos_channels_.ros_to_yunlink);
    log_group("yunlink_to_ros", qos_channels_.yunlink_to_ros);
    log_group("system_services", qos_channels_.system_services);
    ROS_INFO("yunlink_ros_bridge qos runtime channel policies=%zu",
             makeRuntimeQosChannelPolicies().size());
}
