/** @file @brief bridge 转发失败与发布结果诊断统计实现。 */
#include "bridge_node.hpp"

/// 记录发布失败，并把失败归因到 ROS->YunLink 或 YunLink->ROS 方向。
void YunlinkRosBridgeNode::recordPublishFailureUnlocked(const std::string& direction,
                                                        const std::string& key,
                                                        uint32_t error_code,
                                                        const std::string& detail,
                                                        const ros::Time& event_time) {
    // 调用方已经持有 diag_mu_。按方向拆分计数，便于 TUI 区分
    // ROS 输入转发失败和 YunLink 命令下发失败。
    if (direction == "ROS->YunLink") {
        ros_to_yunlink_fail_count_ += 1;
        for (auto* item : {&external_odom_diag_,
                           &odom_state_diag_,
                           &local_odom_diag_,
                           &global_odom_diag_,
                           &uav_control_cmd_diag_,
                           &uav_control_state_diag_,
                           &px4_state_diag_}) {
            if (item->key == key) {
                item->publish_fail_count += 1;
                break;
            }
        }
    } else if (direction == "YunLink->ROS") {
        yunlink_to_ros_fail_count_ += 1;
        for (auto* item : {&takeoff_diag_,
                           &land_diag_,
                           &return_diag_,
                           &goto_diag_,
                           &velocity_setpoint_diag_,
                           &feature_list_diag_,
                           &feature_get_diag_,
                           &feature_start_diag_,
                           &feature_stop_diag_}) {
            if (item->key == key) {
                item->fail_count += 1;
                break;
            }
        }
    }
    last_publish_error_ = direction + " " + key + " " + detail;
    last_fail_direction_ = direction;
    last_fail_key_ = key;
    last_fail_error_code_ = error_code;
    last_fail_detail_ = detail;
    last_error_time_ = event_time;
    appendRecentEventUnlocked(direction == "YunLink->ROS" ? BridgeFlowDirection::kYunlinkToRos
                                                          : BridgeFlowDirection::kRosToYunlink,
                              key,
                              "publish_fail " + detail,
                              event_time);
}

/// 处理 runtime 发布结果，失败时清空 peer_ready 触发后续重连。
void YunlinkRosBridgeNode::recordRosToYunlinkPublishResult(const char* name,
                                                           yunlink::ErrorCode ec) {
    const std::string key = name == nullptr ? "" : name;
    std::lock_guard<std::mutex> lock(diag_mu_);
    if (ec == yunlink::ErrorCode::kOk) {
        ros_to_yunlink_publish_count_ += 1;
        return;
    }
    if (ec == yunlink::ErrorCode::kRuntimeStopped) {
        return;
    }
    // 传输发布失败通常表示缓存的 peer/session 已过期。
    // 交给重连定时器或下一次发布路径重新解析 session。
    peer_ready_ = false;
    recordPublishFailureUnlocked("ROS->YunLink",
                                 key,
                                 static_cast<uint32_t>(ec),
                                 "runtime publish failed",
                                 ros::Time::now());
}
