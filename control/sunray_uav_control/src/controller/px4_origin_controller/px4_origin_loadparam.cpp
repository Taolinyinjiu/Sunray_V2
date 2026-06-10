#include "controller/px4_origin_controller.hpp"
#include "utils/control_config_loader.hpp"
#include "utils/orientation_utils.hpp"
#include <algorithm>
#include <stdexcept>

void PX4_OriginController::load_and_validate_config_or_throw() {
    sunray_config::ControlConfigPaths config_paths;
    YAML::Node root =
        sunray_config::load_control_config_or_throw("PX4_OriginController", &config_paths);
    const std::string config_label = config_paths.merged_path_label;

    const YAML::Node basic_param = root["basic_param"];
    if (!basic_param || !basic_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_label +
                                 "' is missing a valid 'basic_param' map");
    }

    if (!basic_param["fuse_odom_type"]) {
        throw std::runtime_error("missing param 'basic_param.fuse_odom_type'");
    }
    fuse_odom_type_ = basic_param["fuse_odom_type"].as<int>();

    if (!basic_param["fuse_odom_frequency"]) {
        throw std::runtime_error("missing param 'basic_param.fuse_odom_frequency'");
    }
    fuse_odom_frequency_ = basic_param["fuse_odom_frequency"].as<double>();

    if (fuse_odom_type_ != 0 && fuse_odom_type_ != 1 && fuse_odom_type_ != 2) {
        throw std::runtime_error("param 'basic_param.fuse_odom_type' must be 0, 1 or 2");
    }
    if (fuse_odom_frequency_ <= 0.0) {
        throw std::runtime_error("param 'basic_param.fuse_odom_frequency' must > 0");
    }
    fuse_odom_frequency_ = std::clamp(fuse_odom_frequency_, 10.0, 200.0);

    const YAML::Node arrival_judge_param = root["arrival_judge_param"];
    if (!arrival_judge_param || !arrival_judge_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_label +
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
    if (!arrival_judge_param["max_pos_err_m"]) {
        throw std::runtime_error("missing param 'arrival_judge_param.max_pos_err_m'");
    }
    if (!arrival_judge_param["yaw_stabile_err_deg"]) {
        throw std::runtime_error("missing param 'arrival_judge_param.yaw_stabile_err_deg'");
    }
    if (!arrival_judge_param["yaw_rate_stabile_err_deg_s"]) {
        throw std::runtime_error("missing param 'arrival_judge_param.yaw_rate_stabile_err_deg_s'");
    }

    arrival_judge_config_.stable_time_s = arrival_judge_param["judge_stabile_time_s"].as<double>();
    arrival_judge_config_.pos_err_m = arrival_judge_param["pos_stabile_err_m"].as<double>();
    arrival_judge_config_.vel_err_mps = arrival_judge_param["vel_stabile_err_mps"].as<double>();
    arrival_judge_config_.max_pos_err_m = arrival_judge_param["max_pos_err_m"].as<double>();
    arrival_judge_config_.yaw_err_rad =
        deg2rad(arrival_judge_param["yaw_stabile_err_deg"].as<double>());
    arrival_judge_config_.yaw_rate_err_rad_s =
        deg2rad(arrival_judge_param["yaw_rate_stabile_err_deg_s"].as<double>());

    if (arrival_judge_config_.stable_time_s <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.judge_stabile_time_s' must > 0");
    }
    if (arrival_judge_config_.pos_err_m <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.pos_stabile_err_m' must > 0");
    }
    if (arrival_judge_config_.max_pos_err_m <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.max_pos_err_m' must > 0");
    }
    if (arrival_judge_config_.yaw_err_rad <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.yaw_stabile_err_deg' must > 0");
    }
    if (arrival_judge_config_.yaw_rate_err_rad_s <= 0.0) {
        throw std::runtime_error(
            "param 'arrival_judge_param.yaw_rate_stabile_err_deg_s' must > 0");
    }

    const YAML::Node velocity_param = root["velocity_param"];
    if (!velocity_param || !velocity_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_label +
                                 "' is missing a valid 'velocity_param' map");
    }
    const YAML::Node max_velocity = velocity_param["max_velocity"];
    if (!max_velocity || !max_velocity.IsMap() || !max_velocity["x_vel"] || !max_velocity["y_vel"] ||
        !max_velocity["z_vel"]) {
        throw std::runtime_error("missing param 'velocity_param.max_velocity.(x_vel|y_vel|z_vel)'");
    }

    max_velocity_.x() = max_velocity["x_vel"].as<double>();
    max_velocity_.y() = max_velocity["y_vel"].as<double>();
    max_velocity_.z() = max_velocity["z_vel"].as<double>();
    if (max_velocity_.x() <= 0.0 || max_velocity_.y() <= 0.0 || max_velocity_.z() <= 0.0) {
        throw std::runtime_error("param 'velocity_param.max_velocity.*' must > 0");
    }

    if (!velocity_param["yaw_rate"]) {
        throw std::runtime_error("missing param 'velocity_param.yaw_rate'");
    }
    max_yaw_rate_rad_s_ = deg2rad(velocity_param["yaw_rate"].as<double>());
    if (max_yaw_rate_rad_s_ <= 0.0) {
        throw std::runtime_error("param 'velocity_param.yaw_rate' must > 0");
    }
}
