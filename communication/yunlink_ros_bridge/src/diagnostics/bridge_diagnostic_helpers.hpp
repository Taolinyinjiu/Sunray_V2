/** @file @brief bridge 诊断发布共用的小型格式化 helper。 */
#pragma once

#include <cstdint>
#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>

#include "bridge_node.hpp"

/// 计算诊断时间年龄，时间无效时返回 0。
inline uint32_t bridge_diag_age_ms(const ros::Time& now, const ros::Time& then) {
    if (then.isZero() || now < then) {
        return 0;
    }
    return static_cast<uint32_t>((now - then).toSec() * 1000.0);
}

/// 将文本状态映射到 ROS diagnostic level。
inline uint8_t bridge_diag_level_for_status(const std::string& status) {
    if (status == "OK") {
        return diagnostic_msgs::DiagnosticStatus::OK;
    }
    if (status == "ERROR") {
        return diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    return diagnostic_msgs::DiagnosticStatus::WARN;
}

/// 创建 DiagnosticStatus 的 key/value 项。
inline diagnostic_msgs::KeyValue bridge_diag_kv(const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
}

/// 将 bool 格式化成诊断字段文本。
inline std::string bridge_diag_bool_text(bool value) {
    return value ? "true" : "false";
}

/// 将 bridge 方向枚举格式化成诊断字段文本。
inline std::string bridge_diag_direction_text(YunlinkRosBridgeNode::BridgeFlowDirection direction) {
    return direction == YunlinkRosBridgeNode::BridgeFlowDirection::kRosToYunlink ? "ROS->YunLink"
                                                                                 : "YunLink->ROS";
}

/// 判断最近是否发生过转发失败。
inline bool bridge_diag_recent_forwarding_failure(const ros::Time& now,
                                                  const ros::Time& last_error_time) {
    return !last_error_time.isZero() && now >= last_error_time &&
           (now - last_error_time).toSec() <= 5.0;
}
