/** @file @brief YunLink runtime 诊断 snapshot 与 ROS DiagnosticArray 发布实现。 */
#include "bridge_node.hpp"

#include "diagnostics/bridge_diagnostic_helpers.hpp"

namespace {

/// 抑制不会影响当前运行安全性的配置/空闲状态 summary 告警。
bool suppress_summary_warn(const yunlink::SunrayTopicDiagnosticSnapshot& item) {
    return item.status == "UNCONFIGURED" ||
           (item.key == "uav_control_cmd" && item.status == "WAIT_MESSAGE");
}

/// 拼接状态和详情，供 summary 使用。
std::string join_status_detail(const std::string& status, const std::string& detail) {
    return detail.empty() ? status : status + ": " + detail;
}

}  // namespace

/// 发布 YunLink runtime diagnostic snapshot，并同步发布本地 ROS DiagnosticArray。
void YunlinkRosBridgeNode::publishRuntimeDiagnosticSnapshot() {
    const ros::Time now = ros::Time::now();
    if (params_.enable_runtime_diagnostics) {
        yunlink::SunrayRuntimeDiagnosticSnapshot payload{};
        payload.header.frame_id = "sunray_runtime_diagnostic";
        payload.header.stamp_ns = static_cast<uint64_t>(now.toNSec());
        payload.agent_key = "/uav" + std::to_string(std::max(params_.agent_id, 0));
        payload.stale_timeout_ms = static_cast<uint32_t>(std::max(params_.diagnostic_stale_timeout_ms, 0));

        std::string worst = "OK";
        std::string summary = "all monitored topics OK";
        {
            std::lock_guard<std::mutex> lock(diag_mu_);
            refreshTopicPublisherCounts();

            payload.runtime_started = runtime_started_;
            payload.peer_ready = peer_ready_;
            payload.session_state = peer_ready_ ? "OK" : "WAIT";
            payload.last_connect_error = last_connect_error_;
            payload.last_session_error = last_session_error_;
            payload.last_publish_error = last_publish_error_;
            payload.last_error_age_ms = bridge_diag_age_ms(now, last_error_time_);
            payload.connect_attempt_count = connect_attempt_count_;
            payload.session_lost_count = session_lost_count_;
            payload.ros_to_yunlink_publish_count = ros_to_yunlink_publish_count_;
            payload.ros_to_yunlink_fail_count = ros_to_yunlink_fail_count_;
            payload.yunlink_to_ros_command_count = yunlink_to_ros_command_count_;
            payload.yunlink_to_ros_publish_count = yunlink_to_ros_publish_count_;
            payload.yunlink_to_ros_fail_count = yunlink_to_ros_fail_count_;
            payload.last_fail_direction = last_fail_direction_;
            payload.last_fail_key = last_fail_key_;
            payload.last_fail_error_code = last_fail_error_code_;
            payload.last_fail_detail = last_fail_detail_;

            payload.external_odom = makeTopicDiagnosticSnapshot(external_odom_diag_, now);
            payload.odom_state = makeTopicDiagnosticSnapshot(odom_state_diag_, now);
            payload.local_odom = makeTopicDiagnosticSnapshot(local_odom_diag_, now);
            payload.global_odom = makeTopicDiagnosticSnapshot(global_odom_diag_, now);
            payload.uav_control_cmd = makeTopicDiagnosticSnapshot(uav_control_cmd_diag_, now);
            payload.uav_control_state = makeTopicDiagnosticSnapshot(uav_control_state_diag_, now);
            payload.px4_state = makeTopicDiagnosticSnapshot(px4_state_diag_, now);

            for (const auto* item : {&payload.external_odom,
                                     &payload.odom_state,
                                     &payload.local_odom,
                                     &payload.global_odom,
                                     &payload.uav_control_cmd,
                                     &payload.uav_control_state,
                                     &payload.px4_state}) {
                if (suppress_summary_warn(*item)) {
                    continue;
                }
                if (item->status == "ERROR") {
                    worst = "ERROR";
                    summary = join_status_detail(item->key, item->status);
                    break;
                }
                if (item->status == "STALE" || item->status == "NO_PUBLISHER" ||
                    item->status == "WAIT_MESSAGE" || item->status == "UNCONFIGURED") {
                    if (worst != "ERROR") {
                        worst = "WARN";
                        summary = join_status_detail(item->key, item->status);
                    }
                }
            }
            if (!peer_ready_ && worst != "ERROR") {
                worst = "WARN";
                summary = "YunLink session WAIT";
            }
            if (bridge_diag_recent_forwarding_failure(now, last_error_time_) && worst != "ERROR") {
                worst = "WARN";
                summary = "bridge forwarding publish failure";
            }
        }

        payload.worst_level = worst;
        payload.summary = summary;
        publishSnapshot("sunray_runtime_diagnostic",
                        &yunlink::Runtime::publish_sunray_runtime_diagnostic,
                        payload);
        if (worst != "OK") {
            ROS_WARN_THROTTLE(5.0, "yunlink_ros_bridge diagnostic summary: %s", summary.c_str());
        }
    }
    publishMonitorDiagnosticArray(now);
}

/// 诊断定时器入口。
void YunlinkRosBridgeNode::onDiagnosticTimer(const ros::TimerEvent&) {
    publishRuntimeDiagnosticSnapshot();
}
