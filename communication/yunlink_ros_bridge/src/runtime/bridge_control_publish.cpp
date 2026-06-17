/** @file @brief YunLink 命令发布到 Sunray control_cmd 的实现。 */
#include "bridge_node.hpp"

/// 读取最近一次 PX4 高度，供机体系速度命令填充 fixed_height。
float YunlinkRosBridgeNode::latestPx4Height(bool* has_height) const {
    std::lock_guard<std::mutex> lock(px4_state_mu_);
    if (has_height != nullptr) {
        *has_height = has_px4_height_;
    }
    return latest_px4_height_m_;
}

/// 创建带时间戳和来源字段的 Sunray 控制命令基础对象。
sunray_msgs::UAVControlCMD YunlinkRosBridgeNode::makeBaseControlCmd(uint8_t control_cmd) const {
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::SUNRAY_STATION;
    cmd.control_cmd = control_cmd;
    return cmd;
}

/// 发布 Sunray 控制命令，并把 YunLink envelope ID 透传到控制链路。
void YunlinkRosBridgeNode::publishControlCmd(const char* name,
                                             const sunray_msgs::UAVControlCMD& cmd,
                                             uint64_t session_id,
                                             uint64_t correlation_id,
                                             uint64_t message_id,
                                             const std::string& detail) {
    sunray_msgs::UAVControlCMD routed_cmd = cmd;
    // 保留 YunLink envelope 身份信息并透传进 Sunray 控制链路，
    // 这样后续执行状态才能和这条命令关联起来。
    routed_cmd.yunlink_session_id = session_id;
    routed_cmd.yunlink_message_id = message_id;
    routed_cmd.yunlink_correlation_id = correlation_id;
    const uint32_t subscriber_count = control_cmd_pub_.getNumSubscribers();
    {
        std::lock_guard<std::mutex> lock(diag_mu_);
        yunlink_to_ros_command_count_ += 1;
        yunlink_to_ros_publish_count_ += 1;
        // ROS publish 本身是 fire-and-forget；没有订阅者是本地能立即发现的
        // “命令到不了 sunray_uav_control” 信号。
        if (subscriber_count == 0) {
            recordPublishFailureUnlocked("YunLink->ROS",
                                         name == nullptr ? "" : name,
                                         0,
                                         "no subscribers on " + params_.control_cmd_topic,
                                         ros::Time::now());
        }
    }
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
