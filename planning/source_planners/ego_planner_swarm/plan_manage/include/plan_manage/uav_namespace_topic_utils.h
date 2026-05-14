#pragma once

#include <ros/ros.h>

#include <stdexcept>
#include <string>

#include "agent_key_helper.hpp"

namespace ego_planner {

struct Planner_Config_t_ {
  std::string odom_topic;
  std::string depth_topic;
  std::string pose_topic;
  std::string cloud_topic;
  std::string extrinsic_topic;
};

inline std::string loadRequiredStringParamOrThrow(ros::NodeHandle& nh,
                                                  const std::string& param_name) {
  std::string value;
  if (!nh.getParam(param_name, value)) {
    throw std::runtime_error("missing param " + param_name);
  }
  if (value.empty()) {
    throw std::runtime_error(param_name + " cannot be empty");
  }
  return value;
}

inline int loadRequiredIntParamOrThrow(ros::NodeHandle& nh,
                                       const std::string& param_name) {
  int value = 0;
  if (!nh.getParam(param_name, value)) {
    throw std::runtime_error("missing param " + param_name);
  }
  return value;
}

inline bool usePrivateAgentKey(ros::NodeHandle& private_nh) {
  bool use_private_agent_key = false;
  private_nh.param("use_private_agent_key", use_private_agent_key, false);
  return use_private_agent_key;
}

inline std::string loadAgentNameOrThrow(ros::NodeHandle& private_nh) {
  if (usePrivateAgentKey(private_nh)) {
    return loadRequiredStringParamOrThrow(private_nh, "agent_name");
  }

  ros::NodeHandle global_nh;
  return loadRequiredStringParamOrThrow(global_nh, "agent_name");
}

inline int loadAgentIdOrThrow(ros::NodeHandle& private_nh) {
  if (usePrivateAgentKey(private_nh)) {
    return loadRequiredIntParamOrThrow(private_nh, "agent_id");
  }

  ros::NodeHandle global_nh;
  return loadRequiredIntParamOrThrow(global_nh, "agent_id");
}

inline std::string makeAgentKeyOrThrow(const std::string& agent_name, const int agent_id) {
  if (agent_name.empty()) {
    throw std::runtime_error("agent_name cannot be empty");
  }
  if (agent_id <= 0) {
    throw std::runtime_error("agent_id cannot <= 0");
  }
  return sunray_common::normalize_agent_key(agent_name + std::to_string(agent_id));
}

inline std::string loadAgentKeyOrThrow(ros::NodeHandle& private_nh) {
  return usePrivateAgentKey(private_nh) ? sunray_common::get_agent_key_from_private()
                                        : sunray_common::get_agent_key_from_global();
}

inline std::string loadUavNamespaceOrThrow(ros::NodeHandle& nh) {
  return loadAgentKeyOrThrow(nh);
}

inline std::string loadUavNamespaceOrThrow(ros::NodeHandle&, ros::NodeHandle& private_nh) {
  return loadAgentKeyOrThrow(private_nh);
}

inline std::string expandUavTopic(const std::string& raw_topic, const std::string& uav_ns) {
  if (raw_topic.empty()) {
    return raw_topic;
  }
  return sunray_common::replace_agent_key(raw_topic, uav_ns);
}

inline std::string loadExpandedTopicParamOrThrow(ros::NodeHandle& nh,
                                                 const std::string& param_name,
                                                 const std::string& uav_ns) {
  std::string raw_topic;
  if (!nh.getParam(param_name, raw_topic)) {
    throw std::runtime_error("missing param " + param_name);
  }
  if (raw_topic.empty()) {
    throw std::runtime_error(param_name + " cannot be empty");
  }
  return expandUavTopic(raw_topic, uav_ns);
}

inline Planner_Config_t_ loadPlannerConfigOrThrow(ros::NodeHandle& nh,
                                                  const std::string& uav_ns) {
  Planner_Config_t_ planner_config;
  planner_config.odom_topic =
      loadExpandedTopicParamOrThrow(nh, "planner_config/odom_topic", uav_ns);
  planner_config.depth_topic =
      loadExpandedTopicParamOrThrow(nh, "planner_config/depth_topic", uav_ns);
  planner_config.pose_topic =
      loadExpandedTopicParamOrThrow(nh, "planner_config/pose_topic", uav_ns);
  planner_config.cloud_topic =
      loadExpandedTopicParamOrThrow(nh, "planner_config/cloud_topic", uav_ns);
  planner_config.extrinsic_topic =
      loadExpandedTopicParamOrThrow(nh, "planner_config/extrinsic_topic", uav_ns);
  return planner_config;
}

inline std::string makePlannerTopic(std::string suffix, const std::string& uav_ns) {
  const std::string normalized_uav_ns = sunray_common::normalize_agent_key(uav_ns);
  if (normalized_uav_ns.empty()) {
    return normalized_uav_ns;
  }

  if (!suffix.empty() && suffix.front() == '/') {
    suffix.erase(0, 1);
  }

  const std::string planner_topic_prefix = normalized_uav_ns + "/sunray/planning/ego_planner";
  if (suffix.empty()) {
    return planner_topic_prefix;
  }
  return planner_topic_prefix + "/" + suffix;
}

}  // namespace ego_planner
