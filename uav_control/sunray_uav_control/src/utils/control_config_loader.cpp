#include "utils/control_config_loader.hpp"

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <stdexcept>

namespace sunray_config {
namespace {

YAML::Node clone_node(const YAML::Node& node) {
    if (!node) {
        return YAML::Node();
    }
    return YAML::Clone(node);
}

YAML::Node merge_overlay(YAML::Node base, const YAML::Node& overlay) {
    if (!overlay || overlay.IsNull()) {
        return base;
    }
    if (!base.IsMap() || !overlay.IsMap()) {
        return clone_node(overlay);
    }

    for (const auto& item : overlay) {
        const std::string key = item.first.as<std::string>();
        const YAML::Node value = item.second;
        if (base[key] && base[key].IsMap() && value.IsMap()) {
            base[key] = merge_overlay(base[key], value);
        } else {
            base[key] = clone_node(value);
        }
    }

    return base;
}

YAML::Node load_yaml_file_or_throw(const std::string& path, const std::string& label) {
    try {
        return YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load " + label + " yaml file '" + path + "': " +
                                 e.what());
    }
}

}  // namespace

ControlConfigPaths get_control_config_paths_or_throw() {
    ros::NodeHandle private_nh("~");
    ControlConfigPaths paths;
    private_nh.param<std::string>("airframe_type", paths.airframe_type, "");

    const bool has_base = private_nh.getParam("config_base_path", paths.base_path);
    const bool has_airframe = private_nh.getParam("airframe_config_path", paths.airframe_path);

    if (has_base || has_airframe) {
        if (!has_base || paths.base_path.empty()) {
            throw std::runtime_error("missing or empty param " + ros::this_node::getName() +
                                     "/config_base_path");
        }
        if (!has_airframe || paths.airframe_path.empty()) {
            throw std::runtime_error("missing or empty param " + ros::this_node::getName() +
                                     "/airframe_config_path");
        }
        if (paths.airframe_type.empty()) {
            throw std::runtime_error("missing or empty param " + ros::this_node::getName() +
                                     "/airframe_type");
        }

        paths.use_airframe_overlay = true;
        paths.merged_path_label = paths.base_path + " + " + paths.airframe_path;
        return paths;
    }

    if (!private_nh.getParam("config_yamlfile_path", paths.base_path) || paths.base_path.empty()) {
        throw std::runtime_error("missing or empty param " + ros::this_node::getName() +
                                 "/config_yamlfile_path");
    }

    paths.use_airframe_overlay = false;
    paths.merged_path_label = paths.base_path;
    return paths;
}

YAML::Node load_control_config_or_throw(const std::string& context) {
    return load_control_config_or_throw(context, nullptr);
}

YAML::Node load_control_config_or_throw(const std::string& context, ControlConfigPaths* paths_out) {
    const ControlConfigPaths paths = get_control_config_paths_or_throw();
    if (paths_out != nullptr) {
        *paths_out = paths;
    }
    YAML::Node root = load_yaml_file_or_throw(paths.base_path, "base control config");

    if (paths.use_airframe_overlay) {
        const YAML::Node overlay =
            load_yaml_file_or_throw(paths.airframe_path, "airframe control config");
        root = merge_overlay(root, overlay);
        ROS_INFO_STREAM("[" << context << "] airframe_type=" << paths.airframe_type
                            << ", config_base=" << paths.base_path
                            << ", airframe_config=" << paths.airframe_path);
    } else {
        ROS_INFO_STREAM("[" << context << "] config=" << paths.base_path);
    }

    return root;
}

}  // namespace sunray_config
