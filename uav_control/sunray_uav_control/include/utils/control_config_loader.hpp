#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace sunray_config {

struct ControlConfigPaths {
    std::string base_path;
    std::string airframe_path;
    std::string merged_path_label;
    std::string airframe_type;
    bool use_airframe_overlay{false};
};

ControlConfigPaths get_control_config_paths_or_throw();

YAML::Node load_control_config_or_throw(const std::string& context);

YAML::Node load_control_config_or_throw(const std::string& context, ControlConfigPaths* paths_out);

}  // namespace sunray_config
