#include "bridge_tui_node.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <diagnostic_msgs/DiagnosticStatus.h>

namespace {

const char* kAnsiReset = "\033[0m";
const char* kAnsiGood = "\033[1;32m";
const char* kAnsiWarn = "\033[1;33m";
const char* kAnsiBad = "\033[1;31m";
const char* kAnsiTitle = "\033[1;36m";

}  // namespace

BridgeTuiNode::BridgeTuiNode() : nh_(), pnh_("~") {
    setupDefaults();
    loadParams();
    setupRos();
}

void BridgeTuiNode::setupDefaults() {
    ros_to_yunlink_["local_odom"] = FlowRow{"local_odom", "local_odom"};
    ros_to_yunlink_["odom_state"] = FlowRow{"odom_state", "odom_state"};
    ros_to_yunlink_["uav_control_cmd"] = FlowRow{"uav_control_cmd", "uav_control_cmd"};
    ros_to_yunlink_["uav_control_state"] = FlowRow{"uav_control_state", "uav_control_state"};
    ros_to_yunlink_["px4_state"] = FlowRow{"px4_state", "px4_state"};

    yunlink_to_ros_["takeoff"] = FlowRow{"takeoff", "takeoff"};
    yunlink_to_ros_["land"] = FlowRow{"land", "land"};
    yunlink_to_ros_["return"] = FlowRow{"return", "return"};
    yunlink_to_ros_["goto"] = FlowRow{"goto", "goto"};
    yunlink_to_ros_["velocity_setpoint"] = FlowRow{"velocity_setpoint", "velocity_setpoint"};
    yunlink_to_ros_["feature_list"] = FlowRow{"feature_list", "feature_list"};
    yunlink_to_ros_["feature_get"] = FlowRow{"feature_get", "feature_get"};
    yunlink_to_ros_["feature_start"] = FlowRow{"feature_start", "feature_start"};
    yunlink_to_ros_["feature_stop"] = FlowRow{"feature_stop", "feature_stop"};
}

void BridgeTuiNode::loadParams() {
    pnh_.param<std::string>("monitor_diagnostic_topic", diagnostic_topic_, diagnostic_topic_);
    pnh_.param<double>("print_hz", print_hz_, print_hz_);
    pnh_.param<int>("stale_timeout_ms", stale_timeout_ms_, stale_timeout_ms_);
}

void BridgeTuiNode::setupRos() {
    diagnostic_sub_ =
        nh_.subscribe(diagnostic_topic_, 1, &BridgeTuiNode::onDiagnostics, this, ros::TransportHints().tcpNoDelay());
    print_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, print_hz_)),
                                   &BridgeTuiNode::onPrintTimer,
                                   this,
                                   false,
                                   true);
}

void BridgeTuiNode::resetSnapshotLocked() {
    recent_events_.clear();
    connection_ = ConnectionState{};
    for (auto& item : ros_to_yunlink_) {
        item.second.status = "WAIT_MESSAGE";
        item.second.topic = "-";
        item.second.detail.clear();
        item.second.hz = 0.0;
        item.second.age_ms = 0;
        item.second.count = 0;
        item.second.publisher_count = 0;
        item.second.configured = true;
        item.second.has_message = false;
    }
    for (auto& item : yunlink_to_ros_) {
        item.second.status = "WAIT_MESSAGE";
        item.second.detail.clear();
        item.second.hz = 0.0;
        item.second.age_ms = 0;
        item.second.count = 0;
        item.second.publisher_count = 0;
        item.second.configured = true;
        item.second.has_message = false;
        item.second.topic = "-";
    }
}

void BridgeTuiNode::onDiagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    last_snapshot_time_ = msg->header.stamp;
    agent_key_ = msg->header.frame_id.empty() ? "/uav?" : msg->header.frame_id;
    resetSnapshotLocked();

    for (const auto& status : msg->status) {
        if (status.name.find("yunlink_ros_bridge/ros_to_yunlink/") == 0) {
            const std::string key = findValue(status, "key");
            auto it = ros_to_yunlink_.find(key);
            if (it != ros_to_yunlink_.end()) {
                updateFlowRow(&it->second, status);
            }
            continue;
        }
        if (status.name.find("yunlink_ros_bridge/yunlink_to_ros/") == 0) {
            const std::string key = findValue(status, "key");
            auto it = yunlink_to_ros_.find(key);
            if (it != yunlink_to_ros_.end()) {
                updateFlowRow(&it->second, status);
            }
            continue;
        }
        if (status.name == "yunlink_ros_bridge/connection") {
            connection_.status = status.message;
            connection_.udp_bind_port = findValue(status, "udp_bind_port");
            connection_.udp_target_port = findValue(status, "udp_target_port");
            connection_.tcp_listen_port = findValue(status, "tcp_listen_port");
            connection_.remote_ip = findValue(status, "remote_ip");
            connection_.remote_tcp_port = findValue(status, "remote_tcp_port");
            connection_.peer_id = findValue(status, "peer_id");
            connection_.session_id = findValue(status, "session_id");
            continue;
        }
        if (status.name.find("yunlink_ros_bridge/recent_event/") == 0) {
            EventRow row;
            row.direction = findValue(status, "direction");
            row.key = findValue(status, "key");
            row.detail = findValue(status, "detail");
            row.stamp_sec = std::atof(findValue(status, "stamp_sec").c_str());
            row.age_ms = static_cast<uint64_t>(std::strtoull(findValue(status, "age_ms").c_str(), nullptr, 10));
            recent_events_.push_back(row);
        }
    }
}

void BridgeTuiNode::updateFlowRow(FlowRow* row, const diagnostic_msgs::DiagnosticStatus& status) {
    if (row == nullptr) {
        return;
    }
    row->status = status.message;
    row->label = findValue(status, "label");
    row->topic = findValue(status, "topic");
    row->detail = findValue(status, "detail");
    row->hz = std::atof(findValue(status, "hz").c_str());
    row->age_ms = static_cast<uint64_t>(std::strtoull(findValue(status, "age_ms").c_str(), nullptr, 10));
    row->count = static_cast<uint64_t>(std::strtoull(findValue(status, "message_count").c_str(), nullptr, 10));
    row->publisher_count =
        static_cast<uint32_t>(std::strtoul(findValue(status, "publisher_count").c_str(), nullptr, 10));
    row->configured = findValue(status, "configured") != "false";
    row->has_message = findValue(status, "has_message") == "true";
}

std::string BridgeTuiNode::findValue(const diagnostic_msgs::DiagnosticStatus& status, const char* key) {
    for (const auto& item : status.values) {
        if (item.key == key) {
            return item.value;
        }
    }
    return "";
}

void BridgeTuiNode::onPrintTimer(const ros::TimerEvent&) {
    std::cout << "\033[2J\033[H" << buildPanel() << std::flush;
}

std::string BridgeTuiNode::buildPanel() const {
    std::lock_guard<std::mutex> lock(mu_);
    std::ostringstream ss;
    ss << colorText("================ YunLink ROS Bridge 监听面板 | " + agent_key_ + " ================\n",
                    kAnsiTitle);
    ss << "\n连接状态\n";
    ss << "  udp_bind=" << connection_.udp_bind_port << "  udp_target=" << connection_.udp_target_port
       << "  tcp_listen=" << connection_.tcp_listen_port << "\n";
    ss << "  remote=" << connection_.remote_ip << ":" << connection_.remote_tcp_port
       << "  peer=" << connection_.peer_id << "  session=" << connection_.session_id
       << "  status=" << colorStatus(connection_.status) << "\n";

    ss << "\nSunray_v2 -> YunLink\n";
    for (const auto& item : ros_to_yunlink_) {
        const FlowRow& row = item.second;
        ss << "  " << std::left << std::setw(18) << row.label << " "
           << std::setw(16) << colorStatus(row.status) << " hz=" << std::setw(6) << formatHz(row.hz)
           << " age=" << std::setw(8) << formatAge(row.age_ms, row.has_message)
           << " count=" << row.count;
        if (!row.topic.empty() && row.topic != "-") {
            ss << " topic=" << row.topic;
        }
        if (!row.detail.empty()) {
            ss << " detail=" << row.detail;
        }
        ss << "\n";
    }

    ss << "\nYunLink -> Sunray_v2\n";
    for (const auto& item : yunlink_to_ros_) {
        const FlowRow& row = item.second;
        ss << "  " << std::left << std::setw(18) << row.label << " "
           << std::setw(16) << colorStatus(row.status) << " hz=" << std::setw(6) << formatHz(row.hz)
           << " age=" << std::setw(8) << formatAge(row.age_ms, row.has_message)
           << " count=" << row.count;
        if (!row.detail.empty()) {
            ss << " detail=" << row.detail;
        }
        ss << "\n";
    }

    ss << "\n最近事件\n";
    if (recent_events_.empty()) {
        ss << "  -\n";
    } else {
        for (const auto& event : recent_events_) {
            ss << "  [" << formatStamp(event.stamp_sec) << "] " << event.direction << " " << event.key;
            if (!event.detail.empty()) {
                ss << " " << event.detail;
            }
            ss << "\n";
        }
    }
    return ss.str();
}

std::string BridgeTuiNode::formatAge(uint64_t age_ms, bool has_message) {
    if (!has_message) {
        return "-";
    }
    if (age_ms >= 1000) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << (static_cast<double>(age_ms) / 1000.0) << "s";
        return ss.str();
    }
    return std::to_string(age_ms) + "ms";
}

std::string BridgeTuiNode::formatHz(double hz) {
    if (hz <= 0.0) {
        return "0.0";
    }
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << hz;
    return ss.str();
}

std::string BridgeTuiNode::formatStamp(double stamp_sec) {
    if (stamp_sec <= 0.0) {
        return "-";
    }
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << stamp_sec;
    return ss.str();
}

std::string BridgeTuiNode::colorStatus(const std::string& status) {
    if (status == "OK") {
        return colorText(status, kAnsiGood);
    }
    if (status == "ERROR" || status == "UNCONFIGURED") {
        return colorText(status, kAnsiBad);
    }
    return colorText(status, kAnsiWarn);
}

std::string BridgeTuiNode::colorText(const std::string& text, const char* color) {
    return std::string(color) + text + kAnsiReset;
}
