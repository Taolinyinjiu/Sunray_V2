#include "bridge_node.hpp"

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
    sunray_msgs::UAVControlCMD routed_cmd = cmd;
    routed_cmd.yunlink_session_id = session_id;
    routed_cmd.yunlink_message_id = message_id;
    routed_cmd.yunlink_correlation_id = correlation_id;
    control_cmd_pub_.publish(routed_cmd);
    recordYunlinkToRosEvent(name, detail);

    std::string line = std::string("yunlink_ros_bridge forwarded ") + name + " to " +
                       params_.control_cmd_topic + " session_id=" + std::to_string(session_id) +
                       " correlation_id=" + std::to_string(correlation_id) + " msg_id=" +
                       std::to_string(message_id);
    if (!detail.empty()) {
        line += " " + detail;
    }
    ROS_INFO("%s", line.c_str());
}
