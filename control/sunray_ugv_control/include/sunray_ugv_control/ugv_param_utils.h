#pragma once

#include "string_uav_namespace_utils.hpp"
#include <ros/ros.h>
#include <stdexcept>
#include <string>

namespace sunray_ugv_control {

inline std::string replace_ugv_ns(std::string input, const std::string& ugv_ns) {
    return sunray_common::replace_string(std::move(input), "${ugv_ns}", ugv_ns);
}

inline std::string load_ugv_namespace_or_throw(ros::NodeHandle& nh) {
    ros::NodeHandle private_nh("~");

    std::string ugv_ns;
    if (private_nh.getParam("ugv_ns", ugv_ns) || nh.getParam("/ugv_ns", ugv_ns)) {
        ugv_ns = sunray_common::normalize_uav_ns(std::move(ugv_ns));
        if (ugv_ns.empty()) {
            throw std::runtime_error("ugv namespace cannot be empty");
        }
        return ugv_ns;
    }

    std::string ugv_name;
    int ugv_id = 0;

    if (private_nh.getParam("ugv_name", ugv_name)) {
        if (ugv_name.empty()) {
            throw std::runtime_error("~ugv_name cannot be empty");
        }
    } else if (nh.getParam("/ugv_name", ugv_name)) {
        if (ugv_name.empty()) {
            throw std::runtime_error("/ugv_name cannot be empty");
        }
    } else {
        throw std::runtime_error("missing param ~ugv_name or /ugv_name");
    }

    if (private_nh.getParam("ugv_id", ugv_id)) {
        if (ugv_id <= 0) {
            throw std::runtime_error("~ugv_id cannot <= 0");
        }
    } else if (nh.getParam("/ugv_id", ugv_id)) {
        if (ugv_id <= 0) {
            throw std::runtime_error("/ugv_id cannot <= 0");
        }
    } else {
        throw std::runtime_error("missing param ~ugv_id or /ugv_id");
    }

    return sunray_common::normalize_uav_ns(ugv_name + std::to_string(ugv_id));
}

}  // namespace sunray_ugv_control
