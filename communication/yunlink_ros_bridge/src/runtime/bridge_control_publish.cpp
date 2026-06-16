#include "bridge_node.hpp"

#include "bridge_mapping.hpp"

float YunlinkRosBridgeNode::latestPx4Height(bool* has_height) const {
    std::lock_guard<std::mutex> lock(px4_state_mu_);
    if (has_height != nullptr) {
        *has_height = has_px4_height_;
    }
    return latest_px4_height_m_;
}

void YunlinkRosBridgeNode::publishControlCmd(const char* name,
                                             const sunray_msgs::UAVControlCMD& cmd,
                                             const yunlink_msgs::CommandMeta& meta,
                                             const std::string& detail) {
    uint64_t tracking_token = cmd.tracking_token;
    if (tracking_token == 0) {
        tracking_token = next_tracking_token_.fetch_add(1, std::memory_order_relaxed);
    }
    {
        std::lock_guard<std::mutex> lock(command_meta_mu_);
        command_meta_by_token_[tracking_token] = meta;
    }
    const sunray_msgs::UAVControlCMD routed_cmd = applyTrackingToken(cmd, tracking_token);
    control_cmd_pub_.publish(routed_cmd);
    recordYunlinkToRosEvent(name, detail);

    std::string line = std::string("yunlink_ros_bridge forwarded ") + name + " to " +
                       params_.control_cmd_topic + " session_id=" + std::to_string(meta.session_id) +
                       " correlation_id=" + std::to_string(meta.correlation_id) + " msg_id=" +
                       std::to_string(meta.message_id) + " tracking_token=" +
                       std::to_string(routed_cmd.tracking_token);
    if (!detail.empty()) {
        line += " " + detail;
    }
    ROS_INFO("%s", line.c_str());
}
