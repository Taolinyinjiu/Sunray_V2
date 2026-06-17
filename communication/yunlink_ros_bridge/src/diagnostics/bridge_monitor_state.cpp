/** @file @brief bridge monitor 诊断状态构建与事件记录实现。 */
#include "bridge_node.hpp"

#include <algorithm>

namespace {

/// 计算从 then 到 now 的毫秒年龄，时间无效时返回 0。
uint32_t to_age_ms(const ros::Time& now, const ros::Time& then) {
    if (then.isZero() || now < then) {
        return 0;
    }
    return static_cast<uint32_t>((now - then).toSec() * 1000.0);
}

}  // namespace

/// 初始化 ROS->YunLink 方向 topic 诊断元信息。
void YunlinkRosBridgeNode::setupDiagnosticState() {
    external_odom_diag_.key = "external_odom";
    external_odom_diag_.label = "external_odom";
    external_odom_diag_.topic = params_.external_odom_topic;
    external_odom_diag_.configured = !params_.external_odom_topic.empty();

    odom_state_diag_.key = "odom_state";
    odom_state_diag_.label = "odom_state";
    odom_state_diag_.topic = params_.odom_state_topic;
    odom_state_diag_.configured = !params_.odom_state_topic.empty();

    local_odom_diag_.key = "local_odom";
    local_odom_diag_.label = "local_odom";
    local_odom_diag_.topic = params_.local_odom_topic;
    local_odom_diag_.configured = !params_.local_odom_topic.empty();

    global_odom_diag_.key = "global_odom";
    global_odom_diag_.label = "global_odom";
    global_odom_diag_.topic = params_.global_odom_topic;
    global_odom_diag_.configured = !params_.global_odom_topic.empty();

    uav_control_cmd_diag_.key = "uav_control_cmd";
    uav_control_cmd_diag_.label = "uav_control_cmd";
    uav_control_cmd_diag_.topic = params_.control_cmd_topic;
    uav_control_cmd_diag_.configured = !params_.control_cmd_topic.empty();
    uav_control_cmd_diag_.sparse = true;

    uav_control_state_diag_.key = "uav_control_state";
    uav_control_state_diag_.label = "uav_control_state";
    uav_control_state_diag_.topic = params_.control_state_topic;
    uav_control_state_diag_.configured = !params_.control_state_topic.empty();

    px4_state_diag_.key = "px4_state";
    px4_state_diag_.label = "px4_state";
    px4_state_diag_.topic = params_.px4_state_topic;
    px4_state_diag_.configured = !params_.px4_state_topic.empty();
}

/// 初始化 YunLink->ROS 方向命令和 system-service 诊断元信息。
void YunlinkRosBridgeNode::setupMonitorState() {
    takeoff_diag_.key = "takeoff";
    takeoff_diag_.label = "takeoff";
    land_diag_.key = "land";
    land_diag_.label = "land";
    return_diag_.key = "return";
    return_diag_.label = "return";
    goto_diag_.key = "goto";
    goto_diag_.label = "goto";
    velocity_setpoint_diag_.key = "velocity_setpoint";
    velocity_setpoint_diag_.label = "velocity_setpoint";
    velocity_setpoint_diag_.stale_by_age = true;
    feature_list_diag_.key = "feature_list";
    feature_list_diag_.label = "feature_list";
    feature_get_diag_.key = "feature_get";
    feature_get_diag_.label = "feature_get";
    feature_start_diag_.key = "feature_start";
    feature_start_diag_.label = "feature_start";
    feature_stop_diag_.key = "feature_stop";
    feature_stop_diag_.label = "feature_stop";
}

/// 更新单个 topic 的消息计数、时间戳和估算频率。
void YunlinkRosBridgeNode::updateTopicDiagnostic(TopicDiagnosticRuntime* runtime,
                                                 const ros::Time& receive_time) {
    if (runtime == nullptr) {
        return;
    }
    runtime->configured = !runtime->topic.empty();
    runtime->has_message = true;
    runtime->message_count += 1;
    runtime->previous_receive_time = runtime->last_receive_time;
    runtime->last_receive_time = receive_time;
    if (!runtime->previous_receive_time.isZero() && receive_time > runtime->previous_receive_time) {
        const double dt = (receive_time - runtime->previous_receive_time).toSec();
        if (dt > 1e-6) {
            runtime->hz = 1.0 / dt;
        }
    }
}

/// 记录 ROS->YunLink 方向事件，必要时写入最近事件列表。
void YunlinkRosBridgeNode::recordRosToYunlinkEvent(TopicDiagnosticRuntime* runtime,
                                                   const ros::Time& receive_time,
                                                   const std::string& detail,
                                                   bool log_event) {
    if (runtime == nullptr) {
        return;
    }
    updateTopicDiagnostic(runtime, receive_time);
    if (!detail.empty()) {
        runtime->detail = detail;
    }
    const bool should_log = log_event &&
                            (runtime->message_count <= 3 ||
                             runtime->last_event_log_time.isZero() ||
                             (receive_time - runtime->last_event_log_time).toSec() >= 1.0);
    if (should_log) {
        runtime->last_event_log_time = receive_time;
        appendRecentEventUnlocked(BridgeFlowDirection::kRosToYunlink,
                                  runtime->key,
                                  runtime->detail,
                                  receive_time);
    }
}

/// 记录 YunLink->ROS 方向事件，用于命令转发和服务请求诊断。
void YunlinkRosBridgeNode::recordYunlinkToRosEvent(const char* key, const std::string& detail) {
    if (key == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(diag_mu_);
    FlowDiagnosticRuntime* runtime = nullptr;
    const std::string name(key);
    if (name == "takeoff") {
        runtime = &takeoff_diag_;
    } else if (name == "land") {
        runtime = &land_diag_;
    } else if (name == "return") {
        runtime = &return_diag_;
    } else if (name == "goto") {
        runtime = &goto_diag_;
    } else if (name == "velocity_setpoint") {
        runtime = &velocity_setpoint_diag_;
    } else if (name == "feature_list") {
        runtime = &feature_list_diag_;
    } else if (name == "feature_get") {
        runtime = &feature_get_diag_;
    } else if (name == "feature_start") {
        runtime = &feature_start_diag_;
    } else if (name == "feature_stop") {
        runtime = &feature_stop_diag_;
    }
    if (runtime == nullptr) {
        return;
    }

    const ros::Time event_time = ros::Time::now();
    runtime->has_message = true;
    runtime->message_count += 1;
    runtime->publish_count += 1;
    runtime->previous_receive_time = runtime->last_receive_time;
    runtime->last_receive_time = event_time;
    runtime->detail = detail;
    if (!runtime->previous_receive_time.isZero() && event_time > runtime->previous_receive_time) {
        const double dt = (event_time - runtime->previous_receive_time).toSec();
        if (dt > 1e-6) {
            runtime->hz = 1.0 / dt;
        }
    }
    appendRecentEventUnlocked(BridgeFlowDirection::kYunlinkToRos, runtime->key, detail, event_time);
}

/// 追加最近事件并限制列表长度，避免诊断消息无限增长。
void YunlinkRosBridgeNode::appendRecentEventUnlocked(BridgeFlowDirection direction,
                                                     const std::string& key,
                                                     const std::string& detail,
                                                     const ros::Time& event_time) {
    recent_events_.push_back(BridgeEvent{event_time, direction, key, detail});
    while (recent_events_.size() > 8) {
        recent_events_.pop_front();
    }
}

/// 根据 topic 配置、publisher、消息到达时间计算监控状态。
std::string YunlinkRosBridgeNode::statusForTopicDiagnostic(const TopicDiagnosticRuntime& runtime,
                                                           const ros::Time& now,
                                                           int stale_timeout_ms) {
    return statusForMonitorFlow(runtime.configured,
                                runtime.has_message,
                                runtime.publisher_count,
                                runtime.last_receive_time,
                                now,
                                stale_timeout_ms,
                                true,
                                !runtime.sparse);
}

/// 复用同一套状态规则计算 ROS->YunLink 和 YunLink->ROS 方向状态。
std::string YunlinkRosBridgeNode::statusForMonitorFlow(bool configured,
                                                       bool has_message,
                                                       uint32_t publisher_count,
                                                       const ros::Time& last_receive_time,
                                                       const ros::Time& now,
                                                       int stale_timeout_ms,
                                                       bool require_publisher,
                                                       bool stale_by_age) {
    if (!configured) {
        return "UNCONFIGURED";
    }
    if (require_publisher && publisher_count == 0) {
        return "NO_PUBLISHER";
    }
    if (!has_message) {
        return "WAIT_MESSAGE";
    }
    if (stale_by_age && to_age_ms(now, last_receive_time) >
                            static_cast<uint32_t>(std::max(stale_timeout_ms, 0))) {
        return "STALE";
    }
    return "OK";
}

/// 将内部 topic 诊断状态打包成 YunLink runtime diagnostic snapshot。
yunlink::SunrayTopicDiagnosticSnapshot
YunlinkRosBridgeNode::makeTopicDiagnosticSnapshot(const TopicDiagnosticRuntime& runtime,
                                                  const ros::Time& now) const {
    yunlink::SunrayTopicDiagnosticSnapshot snapshot{};
    snapshot.key = runtime.key;
    snapshot.topic = runtime.topic;
    snapshot.configured = runtime.configured;
    snapshot.has_message = runtime.has_message;
    snapshot.publisher_count = runtime.publisher_count;
    snapshot.message_count = runtime.message_count;
    snapshot.hz = static_cast<float>(runtime.hz);
    snapshot.age_ms = to_age_ms(now, runtime.last_receive_time);
    snapshot.stale = !runtime.sparse && snapshot.has_message &&
                     snapshot.age_ms > static_cast<uint32_t>(std::max(params_.diagnostic_stale_timeout_ms, 0));
    snapshot.status = statusForTopicDiagnostic(runtime, now, params_.diagnostic_stale_timeout_ms);
    snapshot.detail = runtime.detail;
    snapshot.last_transition = runtime.last_transition;
    snapshot.last_transition_age_ms = to_age_ms(now, runtime.last_transition_time);
    snapshot.publish_fail_count = runtime.publish_fail_count;
    snapshot.expected_min_hz = static_cast<float>(runtime.expected_min_hz);
    snapshot.sparse = runtime.sparse;
    return snapshot;
}

/// 记录外部里程计诊断输入，不单独写最近事件。
void YunlinkRosBridgeNode::onExternalOdomDiagnostic(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(diag_mu_);
    recordRosToYunlinkEvent(&external_odom_diag_,
                            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp,
                            external_odom_diag_.detail,
                            false);
}

/// 记录全局里程计诊断输入，不单独写最近事件。
void YunlinkRosBridgeNode::onGlobalOdomDiagnostic(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(diag_mu_);
    recordRosToYunlinkEvent(&global_odom_diag_,
                            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp,
                            global_odom_diag_.detail,
                            false);
}

/// 刷新 publisher 数量并记录 topic 状态变化。
void YunlinkRosBridgeNode::refreshTopicPublisherCounts() {
    external_odom_diag_.publisher_count = external_odom_diag_sub_.getNumPublishers();
    odom_state_diag_.publisher_count = odom_state_sub_.getNumPublishers();
    local_odom_diag_.publisher_count = local_odom_sub_.getNumPublishers();
    global_odom_diag_.publisher_count = global_odom_diag_sub_.getNumPublishers();
    uav_control_cmd_diag_.publisher_count = control_cmd_sub_.getNumPublishers();
    uav_control_state_diag_.publisher_count = control_state_sub_.getNumPublishers();
    px4_state_diag_.publisher_count = px4_state_sub_.getNumPublishers();
    const ros::Time now = ros::Time::now();
    for (auto* item : {&external_odom_diag_,
                       &odom_state_diag_,
                       &local_odom_diag_,
                       &global_odom_diag_,
                       &uav_control_cmd_diag_,
                       &uav_control_state_diag_,
                       &px4_state_diag_}) {
        const std::string status =
            statusForTopicDiagnostic(*item, now, params_.diagnostic_stale_timeout_ms);
        if (item->last_status.empty()) {
            item->last_status = status;
            continue;
        }
        if (item->last_status != status) {
            item->last_transition = item->last_status + "->" + status;
            item->last_status = status;
            item->last_transition_time = now;
            appendRecentEventUnlocked(BridgeFlowDirection::kRosToYunlink,
                                      item->key,
                                      "status " + item->last_transition,
                                      now);
        }
    }
}
