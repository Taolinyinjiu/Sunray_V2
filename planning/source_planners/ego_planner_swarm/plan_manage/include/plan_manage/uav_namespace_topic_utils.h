#pragma once

#include <ros/ros.h>

#include <stdexcept>
#include <string>

#include "string_uav_namespace_utils.hpp"

namespace ego_planner {

struct Planner_Config_t_ {
  std::string odom_topic;
  std::string depth_topic;
  std::string pose_topic;
  std::string cloud_topic;
  std::string extrinsic_topic;
};

inline std::string loadRequiredGlobalStringParamOrThrow(ros::NodeHandle& nh,
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

inline int loadRequiredGlobalIntParamOrThrow(ros::NodeHandle& nh,
                                             const std::string& param_name) {
  int value = 0;
  if (!nh.getParam(param_name, value)) {
    throw std::runtime_error("missing param " + param_name);
  }
  return value;
}

inline std::string loadUavNamespaceOrThrow(ros::NodeHandle& nh) {
  const std::string uav_name = loadRequiredGlobalStringParamOrThrow(nh, "/uav_name");
  const int uav_id = loadRequiredGlobalIntParamOrThrow(nh, "/uav_id");
  if (uav_id <= 0) {
    throw std::runtime_error("/uav_id cannot <= 0");
  }
  return sunray_common::normalize_uav_ns(uav_name + std::to_string(uav_id));
}

inline std::string loadUavNamespaceOrThrow(ros::NodeHandle& nh, ros::NodeHandle&) {
  return loadUavNamespaceOrThrow(nh);
}

inline std::string expandUavTopic(const std::string& raw_topic, const std::string& uav_ns) {
  if (raw_topic.empty()) {
    return raw_topic;
  }
  return sunray_common::replace_uav_ns(raw_topic, uav_ns);
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
  const std::string normalized_uav_ns = sunray_common::normalize_uav_ns(uav_ns);
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
