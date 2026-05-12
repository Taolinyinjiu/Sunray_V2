/**
 * @file agent_key_helper.hpp
 * @brief 提供 agent_key 的规范化、加载与占位符替换工具函数
 * @version 0.2
 * @date 2026-05-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <string>
#include <ros/ros.h>

namespace sunray_common {

// 规范化 agent_key，确保格式为 /agent_name + agent_id
inline std::string normalize_agent_key(std::string agent_key) {
    if (agent_key.empty()) {
        return agent_key;
    }
    if (agent_key.front() != '/') {
        agent_key.insert(agent_key.begin(), '/');
    }
    if (agent_key.size() > 1 && agent_key.back() == '/') {
        agent_key.pop_back();
    }
    return agent_key;
}

/* get_agent_key_from_*   use example 

bool use_private_agent_key = false;
private_nh.param("use_private_agent_key", use_private_agent_key, false);
agent_key_ = use_private_agent_key
    ? sunray_common::get_agent_key_from_private()
    : sunray_common::get_agent_key_from_global();

*/
// 从全局参数空间读取 agent_name 与 agent_id 组成 agent_key
// 适用于实际部署场景，统一通过全局参数命名智能体
inline std::string get_agent_key_from_global() {
    ros::NodeHandle global_nh;
    std::string agent_name;
    int agent_id = 0;

    global_nh.getParam("agent_name", agent_name);
    global_nh.getParam("agent_id", agent_id);

    if (agent_name.empty()) {
        throw std::runtime_error("agent_name is empty in global param space");
    }
    return normalize_agent_key(agent_name + std::to_string(agent_id));
}

// 从节点私有参数读取 ~agent_name 与 ~agent_id 组成 agent_key
// 适用于调试或需要单独指定标识符的场景
inline std::string get_agent_key_from_private() {
    ros::NodeHandle private_nh("~");
    std::string agent_name;
    int agent_id = 0;

    if (!private_nh.getParam("agent_name", agent_name) ||
        !private_nh.getParam("agent_id", agent_id)) {
        throw std::runtime_error("~agent_name or ~agent_id not found in private params");
    }

    if (agent_name.empty()) {
        throw std::runtime_error("agent_name is empty in private params");
    }
    return normalize_agent_key(agent_name + std::to_string(agent_id));
}

// 将字符串中的 “${agent_key}” 占位符替换为实际的 agent_key
inline std::string replace_agent_key(std::string input_string, const std::string& agent_key) {
    const std::string placeholder = "${agent_key}";
    if (input_string.empty() || agent_key.empty()) {
        return "";
    }
    if (input_string.find(placeholder) == std::string::npos) {
        return input_string;
    }
    std::size_t pos = 0;
    while ((pos = input_string.find(placeholder, pos)) != std::string::npos) {
        input_string.replace(pos, placeholder.length(), agent_key);
        pos += agent_key.length();
    }
    return input_string;
}

};  // namespace sunray_common
