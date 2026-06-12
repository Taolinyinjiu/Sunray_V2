#include "bridge_node.hpp"

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

namespace {

bool suppress_summary_warn(const yunlink::SunrayTopicDiagnosticSnapshot& item) {
    return item.key == "uav_control_cmd" && item.status == "WAIT_MESSAGE";
}

uint32_t to_age_ms(const ros::Time& now, const ros::Time& then) {
    if (then.isZero() || now < then) {
        return 0;
    }
    return static_cast<uint32_t>((now - then).toSec() * 1000.0);
}

std::string join_status_detail(const std::string& status, const std::string& detail) {
    return detail.empty() ? status : status + ": " + detail;
}

uint8_t level_for_status(const std::string& status) {
    if (status == "OK") {
        return diagnostic_msgs::DiagnosticStatus::OK;
    }
    if (status == "ERROR") {
        return diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    return diagnostic_msgs::DiagnosticStatus::WARN;
}

diagnostic_msgs::KeyValue make_kv(const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
}

std::string bool_text(bool value) {
    return value ? "true" : "false";
}

std::string direction_text(YunlinkRosBridgeNode::BridgeFlowDirection direction) {
    return direction == YunlinkRosBridgeNode::BridgeFlowDirection::kRosToYunlink ? "ROS->YunLink"
                                                                                 : "YunLink->ROS";
}

}  // namespace

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

void YunlinkRosBridgeNode::publishMonitorDiagnosticArray(const ros::Time& now) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = now;
    array.header.frame_id = "/uav" + std::to_string(std::max(params_.agent_id, 0));

    std::lock_guard<std::mutex> lock(diag_mu_);
    refreshTopicPublisherCounts();

    const auto append_ros_status = [&](const TopicDiagnosticRuntime& runtime) {
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "yunlink_ros_bridge/ros_to_yunlink/" + runtime.key;
        status.hardware_id = params_.node_name;
        status.message = statusForTopicDiagnostic(runtime, now, params_.diagnostic_stale_timeout_ms);
        status.level = level_for_status(status.message);
        status.values.push_back(make_kv("direction", "ROS->YunLink"));
        status.values.push_back(make_kv("key", runtime.key));
        status.values.push_back(make_kv("label", runtime.label));
        status.values.push_back(make_kv("topic", runtime.topic));
        status.values.push_back(make_kv("configured", bool_text(runtime.configured)));
        status.values.push_back(make_kv("has_message", bool_text(runtime.has_message)));
        status.values.push_back(make_kv("publisher_count", std::to_string(runtime.publisher_count)));
        status.values.push_back(make_kv("message_count", std::to_string(runtime.message_count)));
        status.values.push_back(make_kv("hz", std::to_string(runtime.hz)));
        status.values.push_back(make_kv("age_ms", std::to_string(to_age_ms(now, runtime.last_receive_time))));
        status.values.push_back(make_kv("detail", runtime.detail));
        array.status.push_back(status);
    };

    const auto append_cmd_status = [&](const FlowDiagnosticRuntime& runtime) {
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "yunlink_ros_bridge/yunlink_to_ros/" + runtime.key;
        status.hardware_id = params_.node_name;
        status.message = statusForMonitorFlow(true,
                                              runtime.has_message,
                                              1,
                                              runtime.last_receive_time,
                                              now,
                                              params_.diagnostic_stale_timeout_ms,
                                              false,
                                              runtime.stale_by_age);
        status.level = level_for_status(status.message);
        status.values.push_back(make_kv("direction", "YunLink->ROS"));
        status.values.push_back(make_kv("key", runtime.key));
        status.values.push_back(make_kv("label", runtime.label));
        status.values.push_back(make_kv("configured", "true"));
        status.values.push_back(make_kv("has_message", bool_text(runtime.has_message)));
        status.values.push_back(make_kv("message_count", std::to_string(runtime.message_count)));
        status.values.push_back(make_kv("hz", std::to_string(runtime.hz)));
        status.values.push_back(make_kv("age_ms", std::to_string(to_age_ms(now, runtime.last_receive_time))));
        status.values.push_back(make_kv("detail", runtime.detail));
        array.status.push_back(status);
    };

    append_ros_status(local_odom_diag_);
    append_ros_status(odom_state_diag_);
    append_ros_status(uav_control_cmd_diag_);
    append_ros_status(uav_control_state_diag_);
    append_ros_status(px4_state_diag_);
    append_cmd_status(takeoff_diag_);
    append_cmd_status(land_diag_);
    append_cmd_status(return_diag_);
    append_cmd_status(goto_diag_);
    append_cmd_status(velocity_setpoint_diag_);
    append_cmd_status(feature_list_diag_);
    append_cmd_status(feature_get_diag_);
    append_cmd_status(feature_start_diag_);
    append_cmd_status(feature_stop_diag_);

    diagnostic_msgs::DiagnosticStatus conn;
    conn.name = "yunlink_ros_bridge/connection";
    conn.hardware_id = params_.node_name;
    conn.message = peer_ready_ ? "OK" : "WAIT";
    conn.level = peer_ready_ ? diagnostic_msgs::DiagnosticStatus::OK
                             : diagnostic_msgs::DiagnosticStatus::WARN;
    conn.values.push_back(make_kv("udp_bind_port", std::to_string(params_.udp_bind_port)));
    conn.values.push_back(make_kv("udp_target_port", std::to_string(params_.udp_target_port)));
    conn.values.push_back(make_kv("tcp_listen_port", std::to_string(params_.tcp_listen_port)));
    conn.values.push_back(make_kv("remote_ip", params_.remote_ip));
    conn.values.push_back(make_kv("remote_tcp_port", std::to_string(params_.remote_tcp_port)));
    conn.values.push_back(make_kv("peer_id", peer_id_.empty() ? "WAIT" : peer_id_));
    conn.values.push_back(make_kv("session_id", session_id_ == 0 ? "WAIT" : std::to_string(session_id_)));
    array.status.push_back(conn);

    for (size_t i = 0; i < recent_events_.size(); ++i) {
        const BridgeEvent& event = recent_events_[recent_events_.size() - 1 - i];
        diagnostic_msgs::DiagnosticStatus item;
        item.name = "yunlink_ros_bridge/recent_event/" + std::to_string(i);
        item.hardware_id = params_.node_name;
        item.level = diagnostic_msgs::DiagnosticStatus::OK;
        item.message = direction_text(event.direction) + " " + event.key;
        item.values.push_back(make_kv("direction", direction_text(event.direction)));
        item.values.push_back(make_kv("key", event.key));
        item.values.push_back(make_kv("stamp_sec", std::to_string(event.time.toSec())));
        item.values.push_back(make_kv("age_ms", std::to_string(to_age_ms(now, event.time))));
        item.values.push_back(make_kv("detail", event.detail));
        array.status.push_back(item);
    }

    monitor_diagnostic_pub_.publish(array);
}

void YunlinkRosBridgeNode::onDiagnosticTimer(const ros::TimerEvent&) {
    publishRuntimeDiagnosticSnapshot();
}
