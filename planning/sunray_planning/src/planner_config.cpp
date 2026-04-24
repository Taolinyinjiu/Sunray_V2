#include "planner_config.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "string_uav_namespace_utils.hpp"

namespace {
PlannerRuntimeConfig parse_planner_config(const std::string& planner_name,
                                          const YAML::Node& planner_node,
                                          const std::string& uav_ns) {
    if (!planner_node || !planner_node.IsMap()) {
        throw std::runtime_error("planner config for '" + planner_name + "' is invalid");
    }

    PlannerRuntimeConfig config;
    config.planner_name = planner_name;
    config.planner_type =
        normalize_planner_type(planner_node["planner_type"] ? planner_node["planner_type"].as<std::string>()
                                                            : planner_name);
    config.planner_id = planner_node["planner_id"] ? planner_node["planner_id"].as<int>() : -1;

    if (!planner_node["goal_topic"] || !planner_node["position_cmd_topic"]) {
        throw std::runtime_error("planner '" + planner_name +
                                 "' is missing goal_topic or position_cmd_topic");
    }

    config.goal_topic =
        sunray_common::replace_uav_ns(planner_node["goal_topic"].as<std::string>(), uav_ns);
    config.position_cmd_topic =
        sunray_common::replace_uav_ns(planner_node["position_cmd_topic"].as<std::string>(), uav_ns);

    if (config.goal_topic.empty() || config.position_cmd_topic.empty()) {
        throw std::runtime_error("planner '" + planner_name + "' has empty runtime topic after parsing");
    }

    if (planner_node["goal_frame_id"]) {
        config.goal_frame_id = planner_node["goal_frame_id"].as<std::string>();
    }
    if (planner_node["planner_state_topic"]) {
        const std::string raw_state_topic = planner_node["planner_state_topic"].as<std::string>();
        config.planner_state_topic =
            raw_state_topic.empty() ? std::string() : sunray_common::replace_uav_ns(raw_state_topic, uav_ns);
    }
    if (planner_node["cmd_timeout_sec"]) {
        config.cmd_timeout_sec = planner_node["cmd_timeout_sec"].as<double>();
    }
    if (planner_node["state_timeout_sec"]) {
        config.state_timeout_sec = planner_node["state_timeout_sec"].as<double>();
    }

    return config;
}
}  // namespace

PlannerRuntimeConfig load_selected_planner_config(const std::string& config_yamlfile_path,
                                                  const std::string& planner_type_filter,
                                                  const int planner_id_filter,
                                                  const std::string& uav_ns) {
    if (config_yamlfile_path.empty()) {
        throw std::invalid_argument("config_yamlfile_path cannot be empty");
    }
    if (uav_ns.empty()) {
        throw std::invalid_argument("uav_ns cannot be empty");
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(config_yamlfile_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("failed to load planner config '" + config_yamlfile_path +
                                 "': " + e.what());
    }

    const YAML::Node planner_list = root["planner_list"];
    if (!planner_list || !planner_list.IsMap()) {
        throw std::runtime_error("planner config '" + config_yamlfile_path +
                                 "' is missing a valid planner_list");
    }

    const std::string normalized_type_filter = normalize_planner_type(planner_type_filter);
    PlannerRuntimeConfig first_config;
    bool has_first_config = false;

    for (YAML::const_iterator it = planner_list.begin(); it != planner_list.end(); ++it) {
        const std::string planner_name = it->first.as<std::string>();
        const PlannerRuntimeConfig config = parse_planner_config(planner_name, it->second, uav_ns);

        if (!has_first_config) {
            first_config = config;
            has_first_config = true;
        }

        if (!normalized_type_filter.empty() && config.planner_type == normalized_type_filter) {
            return config;
        }
        if (normalized_type_filter.empty() && planner_id_filter >= 0 &&
            config.planner_id == planner_id_filter) {
            return config;
        }
    }

    if (!normalized_type_filter.empty()) {
        throw std::runtime_error("planner type '" + normalized_type_filter +
                                 "' was not found in config '" + config_yamlfile_path + "'");
    }
    if (planner_id_filter >= 0) {
        throw std::runtime_error("planner_id '" + std::to_string(planner_id_filter) +
                                 "' was not found in config '" + config_yamlfile_path + "'");
    }
    if (!has_first_config) {
        throw std::runtime_error("planner config '" + config_yamlfile_path + "' contains no planners");
    }

    return first_config;
}
