#include "planner_interface/ego_planner.hpp"

#include "string_uav_namespace_utils.hpp"
#include <ros/this_node.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

void EgoPlanner::set_nodehandle(ros::NodeHandle& nh) { nh_ = &nh; }

bool EgoPlanner::load_param() {
    position_cmd_topic_ =
        sunray_common::replace_uav_ns(load_config_from_yaml(config_yamlfile_path_), uav_ns_);

    if (position_cmd_topic_.empty()) {
        throw std::runtime_error("ego planner config position_cmd_topic is empty");
    }
    return true;
}

void EgoPlanner::init() {
    snapshot_.ready = false;
    if (nh_ != nullptr) {
        ros::NodeHandle private_nh("~");
        const std::string node_name = ros::this_node::getName();
        if (!private_nh.getParam("config_yamlfile_path", config_yamlfile_path_)) {
            throw std::runtime_error("missing param " + node_name + "/config_yamlfile_path");
        }
        uav_ns_ = load_uav_namespace_or_throw(*nh_);
        snapshot_.ready = load_param();
    }
    snapshot_.planner_state = snapshot_.ready ? PlannerExecState::WAIT_TARGET
                                              : PlannerExecState::INIT;
    snapshot_.planner_state_string = snapshot_.ready ? "WAIT_TARGET" : "INIT";
}

bool EgoPlanner::is_ready() const { return snapshot_.ready; }

bool EgoPlanner::send_goal(const PlanningTarget& target) {
    target_ = target;
    snapshot_.goal_active = true;
    snapshot_.ready = true;
    snapshot_.planner_state = PlannerExecState::GENERATE;
    snapshot_.planner_state_string = "GENERATE";
    snapshot_.last_goal_stamp = ros::Time::now();
    return true;
}

PlannerPositionCommand EgoPlanner::get_position_cmd() { return position_cmd_; }

PlannerSnapshot EgoPlanner::get_snapshot() const { return snapshot_; }

std::string EgoPlanner::load_uav_namespace_or_throw(ros::NodeHandle& nh) {
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

std::string EgoPlanner::load_config_from_yaml(const std::string& yaml_path) {
    if (yaml_path.empty()) {
        throw std::invalid_argument("yaml_path cannot be empty");
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("failed to load yaml file '" + yaml_path + "': " + e.what());
    }

    const YAML::Node planner_list = root["planner_list"];
    if (!planner_list || !planner_list.IsMap()) {
        throw std::runtime_error("the yaml file '" + yaml_path +
                                 "' is missing a valid planner_list map");
    }

    const YAML::Node ego_node = planner_list["EGO"];
    if (!ego_node || !ego_node.IsMap()) {
        throw std::runtime_error("the yaml file '" + yaml_path +
                                 "' is missing planner_list/EGO");
    }

    if (!ego_node["position_cmd_topic"]) {
        throw std::runtime_error("the yaml file '" + yaml_path +
                                 "' is missing planner_list/EGO/position_cmd_topic");
    }

    const std::string position_cmd_topic = ego_node["position_cmd_topic"].as<std::string>();
    if (position_cmd_topic.empty()) {
        throw std::runtime_error("planner_list/EGO/position_cmd_topic cannot be empty");
    }
    return position_cmd_topic;
}
