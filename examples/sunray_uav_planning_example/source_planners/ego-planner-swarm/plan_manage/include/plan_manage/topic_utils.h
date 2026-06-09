#pragma once

#include <ros/ros.h>

#include <string>

namespace ego_planner {

inline std::string normalizeTopicPrefix(std::string prefix) {
  if (prefix.empty()) {
    return prefix;
  }
  if (prefix.front() != '/') {
    prefix.insert(prefix.begin(), '/');
  }
  while (prefix.size() > 1 && prefix.back() == '/') {
    prefix.pop_back();
  }
  return prefix;
}

inline std::string loadTopicPrefix(const ros::NodeHandle& nh) {
  std::string prefix;
  nh.param<std::string>("planner_prefix", prefix, "");
  return normalizeTopicPrefix(prefix);
}

inline std::string makePrefixedTopic(const std::string& prefix, const std::string& topic) {
  if (topic.empty()) {
    return prefix;
  }

  std::string normalized_topic = topic;
  if (normalized_topic.front() != '/') {
    normalized_topic.insert(normalized_topic.begin(), '/');
  }

  if (prefix.empty()) {
    return normalized_topic;
  }
  return prefix + normalized_topic;
}

}  // namespace ego_planner
