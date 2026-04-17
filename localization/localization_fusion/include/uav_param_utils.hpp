#pragma once

#include "string_uav_namespace_utils.hpp"
#include <ros/ros.h>
#include <stdexcept>
#include <string>

namespace localization_fusion {

inline std::string load_uav_namespace_or_throw(ros::NodeHandle& nh) {
    ros::NodeHandle private_nh("~");

    std::string uav_name;
    int uav_id = 0;

    if (private_nh.getParam("uav_name", uav_name)) {
        if (uav_name.empty()) {
            throw std::runtime_error("~uav_name cannot be empty");
        }
    } else if (nh.getParam("/uav_name", uav_name)) {
        if (uav_name.empty()) {
            throw std::runtime_error("/uav_name cannot be empty");
        }
    } else {
        throw std::runtime_error("missing param ~uav_name or /uav_name");
    }

    if (private_nh.getParam("uav_id", uav_id)) {
        if (uav_id <= 0) {
            throw std::runtime_error("~uav_id cannot <= 0");
        }
    } else if (nh.getParam("/uav_id", uav_id)) {
        if (uav_id <= 0) {
            throw std::runtime_error("/uav_id cannot <= 0");
        }
    } else {
        throw std::runtime_error("missing param ~uav_id or /uav_id");
    }

    return sunray_common::normalize_uav_ns(uav_name + std::to_string(uav_id));
}

}  // namespace localization_fusion
