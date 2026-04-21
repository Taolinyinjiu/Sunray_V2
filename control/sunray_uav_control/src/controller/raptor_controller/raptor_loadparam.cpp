#include "controller/raptor_controller.hpp"
#include <algorithm>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

void Raptor_Controller::load_and_validate_config_or_throw() {
    YAML::Node root;
    try {
        root = YAML::LoadFile(config_yamlfile_path_);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load yaml file '" + config_yamlfile_path_ +
                                 "': " + e.what());
    }

    const YAML::Node basic_param = root["basic_param"];
    if (!basic_param || !basic_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_yamlfile_path_ +
                                 "' is missing a valid 'basic_param' map");
    }

    if (!basic_param["fuse_odom_type"]) {
        throw std::runtime_error("missing param 'basic_param.fuse_odom_type'");
    }
    fuse_odom_type = basic_param["fuse_odom_type"].as<int>();
    if (!basic_param["fuse_odom_frequency"]) {
        throw std::runtime_error("missing param 'basic_param.fuse_odom_frequency'");
    }
    fuse_odom_frequency = basic_param["fuse_odom_frequency"].as<double>();
    if (fuse_odom_type != 0 && fuse_odom_type != 1 && fuse_odom_type != 2) {
        throw std::runtime_error("param 'basic_param.fuse_odom_type' must be 0, 1 or 2");
    }
    if (fuse_odom_frequency <= 0.0) {
        throw std::runtime_error("param 'basic_param.fuse_odom_frequency' must > 0");
    }
    fuse_odom_frequency = std::clamp(fuse_odom_frequency, 10.0, 200.0);

    const YAML::Node arrival_judge_param = root["arrival_judge_param"];
    if (!arrival_judge_param || !arrival_judge_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_yamlfile_path_ +
                                 "' is missing a valid 'arrival_judge_param' map");
    }
    if (!arrival_judge_param["judge_stabile_time_s"]) {
        throw std::runtime_error("missing param 'arrival_judge_param.judge_stabile_time_s'");
    }
    if (!arrival_judge_param["pos_stabile_err_m"]) {
        throw std::runtime_error("missing param 'arrival_judge_param.pos_stabile_err_m'");
    }
    if (!arrival_judge_param["vel_stabile_err_mps"]) {
        throw std::runtime_error("missing param 'arrival_judge_param.vel_stabile_err_mps'");
    }

    arrival_judge_config_.stable_time_s = arrival_judge_param["judge_stabile_time_s"].as<double>();
    arrival_judge_config_.pos_err_m = arrival_judge_param["pos_stabile_err_m"].as<double>();
    arrival_judge_config_.vel_err_mps = arrival_judge_param["vel_stabile_err_mps"].as<double>();

    if (arrival_judge_config_.stable_time_s <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.judge_stabile_time_s' must > 0");
    }
    if (arrival_judge_config_.pos_err_m <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.pos_stabile_err_m' must > 0");
    }
    if (arrival_judge_config_.vel_err_mps <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.vel_stabile_err_mps' must > 0");
    }
}

void Raptor_Controller::ensure_fusion_param_ready_or_throw() {
    if (fuse_odom_type == 0) {
        return;
    }
    // Raptor_Controller 不依赖 PX4_ParamManager，外部里程计融合参数由使用者预先配置。
}
