/** @file @brief 本地 ROS DiagnosticArray 监控消息组装和发布实现。 */
#include "bridge_node.hpp"

#include <algorithm>

#include <diagnostic_msgs/DiagnosticStatus.h>

#include "diagnostics/bridge_diagnostic_helpers.hpp"

/// 组装给 TUI 使用的 ROS DiagnosticArray。
void YunlinkRosBridgeNode::publishMonitorDiagnosticArray(const ros::Time& now) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = now;
    array.header.frame_id = "/uav" + std::to_string(std::max(params_.agent_id, 0));

    std::lock_guard<std::mutex> lock(diag_mu_);
    refreshTopicPublisherCounts();

    const auto append_ros_status = [&](const TopicDiagnosticRuntime& runtime) {
        const auto qos = channelQos(qos_channels_.ros_to_yunlink, runtime.key.c_str());
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "yunlink_ros_bridge/ros_to_yunlink/" + runtime.key;
        status.hardware_id = params_.node_name;
        status.message = statusForTopicDiagnostic(runtime, now, params_.diagnostic_stale_timeout_ms);
        status.level = bridge_diag_level_for_status(status.message);
        status.values.push_back(bridge_diag_kv("direction", "ROS->YunLink"));
        status.values.push_back(bridge_diag_kv("key", runtime.key));
        status.values.push_back(bridge_diag_kv("label", runtime.label));
        status.values.push_back(bridge_diag_kv("topic", runtime.topic));
        status.values.push_back(bridge_diag_kv("configured", bridge_diag_bool_text(runtime.configured)));
        status.values.push_back(bridge_diag_kv("has_message", bridge_diag_bool_text(runtime.has_message)));
        status.values.push_back(bridge_diag_kv("publisher_count", std::to_string(runtime.publisher_count)));
        status.values.push_back(bridge_diag_kv("message_count", std::to_string(runtime.message_count)));
        status.values.push_back(bridge_diag_kv("hz", std::to_string(runtime.hz)));
        status.values.push_back(
            bridge_diag_kv("age_ms", std::to_string(bridge_diag_age_ms(now, runtime.last_receive_time))));
        status.values.push_back(bridge_diag_kv("last_transition", runtime.last_transition));
        status.values.push_back(bridge_diag_kv(
            "last_transition_age_ms", std::to_string(bridge_diag_age_ms(now, runtime.last_transition_time))));
        status.values.push_back(bridge_diag_kv("publish_fail_count", std::to_string(runtime.publish_fail_count)));
        status.values.push_back(bridge_diag_kv("expected_min_hz", std::to_string(runtime.expected_min_hz)));
        status.values.push_back(bridge_diag_kv("sparse", bridge_diag_bool_text(runtime.sparse)));
        status.values.push_back(bridge_diag_kv("qos_class", qos.qos_class));
        status.values.push_back(bridge_diag_kv("transport", qos.transport));
        status.values.push_back(bridge_diag_kv("queue_size", std::to_string(qos.has_queue_size ? qos.queue_size : 1)));
        status.values.push_back(
            bridge_diag_kv("tcp_no_delay", bridge_diag_bool_text(qos.has_tcp_no_delay ? qos.tcp_no_delay : true)));
        status.values.push_back(
            bridge_diag_kv("publish_hz", std::to_string(qos.has_publish_hz ? qos.publish_hz : 0.0)));
        status.values.push_back(bridge_diag_kv("detail", runtime.detail));
        array.status.push_back(status);
    };

    const auto qos_for_flow = [this](const FlowDiagnosticRuntime& runtime) {
        BridgeChannelQosConfig qos = channelQos(qos_channels_.yunlink_to_ros, runtime.key.c_str());
        if (!qos.configured) {
            qos = channelQos(qos_channels_.yunlink_to_ros, (runtime.key + "_command").c_str());
        }
        return qos.configured ? qos : channelQos(qos_channels_.system_services, runtime.key.c_str());
    };
    const auto append_cmd_status = [&](const FlowDiagnosticRuntime& runtime) {
        const auto qos = qos_for_flow(runtime);
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
        status.level = bridge_diag_level_for_status(status.message);
        status.values.push_back(bridge_diag_kv("direction", "YunLink->ROS"));
        status.values.push_back(bridge_diag_kv("key", runtime.key));
        status.values.push_back(bridge_diag_kv("label", runtime.label));
        status.values.push_back(bridge_diag_kv("configured", "true"));
        status.values.push_back(bridge_diag_kv("has_message", bridge_diag_bool_text(runtime.has_message)));
        status.values.push_back(bridge_diag_kv("message_count", std::to_string(runtime.message_count)));
        status.values.push_back(bridge_diag_kv("publish_count", std::to_string(runtime.publish_count)));
        status.values.push_back(bridge_diag_kv("fail_count", std::to_string(runtime.fail_count)));
        status.values.push_back(bridge_diag_kv("hz", std::to_string(runtime.hz)));
        status.values.push_back(
            bridge_diag_kv("age_ms", std::to_string(bridge_diag_age_ms(now, runtime.last_receive_time))));
        status.values.push_back(bridge_diag_kv("qos_class", qos.qos_class));
        status.values.push_back(bridge_diag_kv("transport", qos.transport));
        if (qos.has_queue_size) {
            status.values.push_back(bridge_diag_kv("queue_size", std::to_string(qos.queue_size)));
        }
        if (qos.has_timeout_sec) {
            status.values.push_back(bridge_diag_kv("timeout_sec", std::to_string(qos.timeout_sec)));
        }
        status.values.push_back(bridge_diag_kv("detail", runtime.detail));
        array.status.push_back(status);
    };

    for (const auto* topic :
         {&local_odom_diag_, &odom_state_diag_, &uav_control_cmd_diag_, &uav_control_state_diag_, &px4_state_diag_}) {
        append_ros_status(*topic);
    }
    for (const auto* flow : {&takeoff_diag_, &land_diag_, &return_diag_, &goto_diag_,
                             &velocity_setpoint_diag_, &feature_list_diag_, &feature_get_diag_,
                             &feature_start_diag_, &feature_stop_diag_}) {
        append_cmd_status(*flow);
    }

    diagnostic_msgs::DiagnosticStatus conn;
    conn.name = "yunlink_ros_bridge/connection";
    conn.hardware_id = params_.node_name;
    conn.message = peer_ready_ ? "OK" : "WAIT";
    conn.level = peer_ready_ ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
    conn.values.push_back(bridge_diag_kv("udp_bind_port", std::to_string(params_.udp_bind_port)));
    conn.values.push_back(bridge_diag_kv("udp_target_port", std::to_string(params_.udp_target_port)));
    conn.values.push_back(bridge_diag_kv("tcp_listen_port", std::to_string(params_.tcp_listen_port)));
    conn.values.push_back(bridge_diag_kv("remote_ip", params_.remote_ip));
    conn.values.push_back(bridge_diag_kv("remote_tcp_port", std::to_string(params_.remote_tcp_port)));
    conn.values.push_back(bridge_diag_kv("qos_profile", qosProfileName(makeRuntimeQosPolicy().profile)));
    conn.values.push_back(bridge_diag_kv("qos_udp_fallback_to_tcp", bridge_diag_bool_text(params_.qos_udp_fallback_to_tcp)));
    conn.values.push_back(bridge_diag_kv("qos_route_phase", "runtime_qos"));
    conn.values.push_back(bridge_diag_kv("runtime_started", bridge_diag_bool_text(runtime_started_)));
    conn.values.push_back(bridge_diag_kv("peer_ready", bridge_diag_bool_text(peer_ready_)));
    conn.values.push_back(bridge_diag_kv("peer_id", peer_id_.empty() ? "WAIT" : peer_id_));
    conn.values.push_back(bridge_diag_kv("session_id", session_id_ == 0 ? "WAIT" : std::to_string(session_id_)));
    conn.values.push_back(bridge_diag_kv("last_connect_error", last_connect_error_));
    conn.values.push_back(bridge_diag_kv("last_session_error", last_session_error_));
    conn.values.push_back(bridge_diag_kv("last_publish_error", last_publish_error_));
    conn.values.push_back(bridge_diag_kv("last_error_age_ms", std::to_string(bridge_diag_age_ms(now, last_error_time_))));
    conn.values.push_back(bridge_diag_kv("connect_attempt_count", std::to_string(connect_attempt_count_)));
    conn.values.push_back(bridge_diag_kv("session_lost_count", std::to_string(session_lost_count_)));
    array.status.push_back(conn);

    diagnostic_msgs::DiagnosticStatus forwarding;
    forwarding.name = "yunlink_ros_bridge/forwarding";
    forwarding.hardware_id = params_.node_name;
    forwarding.message = bridge_diag_recent_forwarding_failure(now, last_error_time_) ? "PUBLISH_FAIL" : "OK";
    forwarding.level = forwarding.message == "OK" ? diagnostic_msgs::DiagnosticStatus::OK
                                                   : diagnostic_msgs::DiagnosticStatus::WARN;
    forwarding.values.push_back(bridge_diag_kv("ros_to_yunlink_publish_count", std::to_string(ros_to_yunlink_publish_count_)));
    forwarding.values.push_back(bridge_diag_kv("ros_to_yunlink_fail_count", std::to_string(ros_to_yunlink_fail_count_)));
    forwarding.values.push_back(bridge_diag_kv("yunlink_to_ros_command_count", std::to_string(yunlink_to_ros_command_count_)));
    forwarding.values.push_back(bridge_diag_kv("yunlink_to_ros_publish_count", std::to_string(yunlink_to_ros_publish_count_)));
    forwarding.values.push_back(bridge_diag_kv("yunlink_to_ros_fail_count", std::to_string(yunlink_to_ros_fail_count_)));
    forwarding.values.push_back(bridge_diag_kv("last_fail_direction", last_fail_direction_));
    forwarding.values.push_back(bridge_diag_kv("last_fail_key", last_fail_key_));
    forwarding.values.push_back(bridge_diag_kv("last_fail_error_code", std::to_string(last_fail_error_code_)));
    forwarding.values.push_back(bridge_diag_kv("last_fail_detail", last_fail_detail_));
    array.status.push_back(forwarding);

    for (size_t i = 0; i < recent_events_.size(); ++i) {
        const BridgeEvent& event = recent_events_[recent_events_.size() - 1 - i];
        diagnostic_msgs::DiagnosticStatus item;
        item.name = "yunlink_ros_bridge/recent_event/" + std::to_string(i);
        item.hardware_id = params_.node_name;
        item.level = diagnostic_msgs::DiagnosticStatus::OK;
        item.message = bridge_diag_direction_text(event.direction) + " " + event.key;
        item.values.push_back(bridge_diag_kv("direction", bridge_diag_direction_text(event.direction)));
        item.values.push_back(bridge_diag_kv("key", event.key));
        item.values.push_back(bridge_diag_kv("stamp_sec", std::to_string(event.time.toSec())));
        item.values.push_back(bridge_diag_kv("age_ms", std::to_string(bridge_diag_age_ms(now, event.time))));
        item.values.push_back(bridge_diag_kv("detail", event.detail));
        array.status.push_back(item);
    }

    monitor_diagnostic_pub_.publish(array);
}
