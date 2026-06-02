#include "controller/geometric_controller.hpp"
#include "utils/orientation_utils.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <stdexcept>

void Geometric_Controller::load_and_validate_config_or_throw() {
    // 读取 geometric controller 自身需要的参数。
    // 那些已经改为“类内默认值”的细调项，不再从 yaml 中加载。
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

    // -------------------- basic_param --------------------
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
    if (!basic_param["gravity"]) {
        throw std::runtime_error("missing param 'basic_param.gravity'");
    }
    geometric_controller_param_.gravity = basic_param["gravity"].as<double>();
    if (!basic_param["mass_kg"]) {
        throw std::runtime_error("missing param 'basic_param.mass_kg'");
    }
    geometric_controller_param_.drone_mass = basic_param["mass_kg"].as<double>();
    if (geometric_controller_param_.gravity <= 0.0) {
        throw std::runtime_error("param 'basic_param.gravity' must > 0");
    }
    if (geometric_controller_param_.drone_mass <= 0.0) {
        throw std::runtime_error("param 'basic_param.mass_kg' must > 0");
    }

    // 悬停推力初始化值直接来自 basic_param.hover_thrust_percent。
    if (!basic_param["hover_thrust_percent"]) {
        throw std::runtime_error("missing param 'basic_param.hover_thrust_percent'");
    }
    geometric_controller_param_.hover_thrust_init =
        std::clamp(basic_param["hover_thrust_percent"].as<double>(), 0.05, 0.80);
    geometric_controller_param_.hover_thrust_min = 0.05;
    geometric_controller_param_.hover_thrust_max = 0.80;

    // ---------------- arrival_judge_param ----------------
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

    // ------------------- velocity_param -------------------
    const YAML::Node velocity_param = root["velocity_param"];
    if (!velocity_param || !velocity_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_yamlfile_path_ +
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

    // ------------- sunray_controller_param --------------
    const YAML::Node controller_param = root["sunray_controller_param"];
    if (!controller_param || !controller_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_yamlfile_path_ +
                                 "' is missing a valid 'sunray_controller_param' map");
    }

    if (!controller_param["control_type"]) {
        throw std::runtime_error("missing param 'sunray_controller_param.control_type'");
    }
    const int control_type = controller_param["control_type"].as<int>();
    if (control_type != 0 && control_type != 1) {
        throw std::runtime_error(
            "param 'sunray_controller_param.control_type' must be 0 (attitude+thrust) or 1 "
            "(bodyrate+thrust) for Geometric_Controller");
    }
    attitude_command_mode_ =
        (control_type == 0) ? AttitudeCommandMode::Attitude : AttitudeCommandMode::BodyRate;

    if (!controller_param["attitude_tau"]) {
        throw std::runtime_error("missing param 'sunray_controller_param.attitude_tau'");
    }
    geometric_controller_param_.attitude_tau = controller_param["attitude_tau"].as<double>();
    if (geometric_controller_param_.attitude_tau <= 0.0) {
        throw std::runtime_error("param 'sunray_controller_param.attitude_tau' must > 0");
    }

    // controller_update_frequency 由 basic_param 统一提供，这里只取值，不重复约束状态机语义。
    if (basic_param["controller_update_frequency"]) {
        geometric_controller_param_.controller_hz =
            basic_param["controller_update_frequency"].as<double>();
        if (geometric_controller_param_.controller_hz <= 0.0) {
            throw std::runtime_error("param 'basic_param.controller_update_frequency' must > 0");
        }
    }

    const auto load_vec3 = [&](const char* key, Eigen::Vector3d& out) {
        if (!controller_param[key] || !controller_param[key].IsSequence() ||
            controller_param[key].size() != 3) {
            throw std::runtime_error(std::string("missing or invalid param 'sunray_controller_param.") +
                                     key + "' (expected sequence of 3 values)");
        }
        out.x() = controller_param[key][0].as<double>();
        out.y() = controller_param[key][1].as<double>();
        out.z() = controller_param[key][2].as<double>();
    };

    load_vec3("pos_kp", geometric_controller_param_.pos_kp);
    load_vec3("pos_ki", geometric_controller_param_.pos_ki);
    load_vec3("pos_kd", geometric_controller_param_.pos_kd);
    load_vec3("vel_kp", geometric_controller_param_.vel_kp);
    load_vec3("vel_ki", geometric_controller_param_.vel_ki);
    load_vec3("vel_kd", geometric_controller_param_.vel_kd);

    if (controller_param["max_acc"]) {
        geometric_controller_param_.max_acc = controller_param["max_acc"].as<double>();
        if (geometric_controller_param_.max_acc <= 0.0) {
            throw std::runtime_error("param 'sunray_controller_param.max_acc' must > 0");
        }
    }
    if (controller_param["max_d_acc"]) {
        geometric_controller_param_.max_d_acc = controller_param["max_d_acc"].as<double>();
        if (geometric_controller_param_.max_d_acc <= 0.0) {
            throw std::runtime_error("param 'sunray_controller_param.max_d_acc' must > 0");
        }
    } else {
        // 未配置时默认与 max_acc 一致，保持原有行为
        geometric_controller_param_.max_d_acc = geometric_controller_param_.max_acc;
    }
    if (controller_param["hover_thrust_estimator_type"]) {
        geometric_controller_param_.hover_thrust_estimator_type =
            controller_param["hover_thrust_estimator_type"].as<int>();
        if (geometric_controller_param_.hover_thrust_estimator_type < 0 ||
            geometric_controller_param_.hover_thrust_estimator_type > 1) {
            throw std::runtime_error(
                "param 'sunray_controller_param.hover_thrust_estimator_type' must be 0 (RLS) or 1 (EKFAccel)");
        }
    }

    // ---------------- takeoff_land_type 路由 ----------------
    if (!controller_param["takeoff_land_type"]) {
        throw std::runtime_error("missing param 'sunray_controller_param.takeoff_land_type'");
    }
    takeoff_land_type_ = controller_param["takeoff_land_type"].as<int>();
    if (takeoff_land_type_ != 0 && takeoff_land_type_ != 1) {
        throw std::runtime_error(
            "param 'sunray_controller_param.takeoff_land_type' must be 0 (direct thrust) or 1 (AccFF)");
    }

    // ---------------- AccFF 起降参数:由 gravity / hover_thrust_init 推导 ----------------
    // 设计原则:yaml 仅暴露 takeoff_land_type 开关,AccFF 内部参数全部从基础物理量推导,
    // 避免向用户暴露 ramp/jerk/touchdown 这类纯调试维度。详见 ai_result/accff_takeoff_landing_todo.md。
    const double gravity = geometric_controller_param_.gravity;
    const double hover_init = std::clamp(geometric_controller_param_.hover_thrust_init, 0.05, 0.80);

    // 起飞:a_start 对应 thrust ≈ 0.08(略高于 idle,远低于悬停);a_target = g + 0.5。
    takeoff_accff_tuning_.a_start_mps2 = std::clamp(gravity * 0.08 / hover_init, 0.5, gravity);
    takeoff_accff_tuning_.a_target_mps2 = gravity + 0.5;
    takeoff_accff_tuning_.ramp_time_s = 1.2;
    takeoff_accff_tuning_.jerk_max_mps3 = 8.0;
    takeoff_accff_tuning_.a_min_mps2 = 0.0;
    takeoff_accff_tuning_.a_max_mps2 = gravity + 5.0;
    takeoff_accff_tuning_.liftoff_detect_h_m = 0.05;
    takeoff_accff_tuning_.liftoff_detect_vz_mps = 0.15;

    // 降落:a_touchdown 接近 g 的稳态值,让 NearGround 大部分时间维持悬停推力,
    // 只在末段做温和净减速,避免出现 a_touchdown=0.55g 时的"NearGround 一进入就自由落体"。
    landing_accff_tuning_.near_ground_h_m = 0.25;
    landing_accff_tuning_.near_ground_vz_mps = 0.10;
    landing_accff_tuning_.a_touchdown_mps2 = 0.85 * gravity;
    landing_accff_tuning_.ramp_time_s = 2.0;
    landing_accff_tuning_.jerk_max_mps3 = 1.0;
    landing_accff_tuning_.a_min_mps2 = 0.0;
    landing_accff_tuning_.a_max_mps2 = gravity + 2.0;
    landing_accff_tuning_.touchdown_landed_state = true;
    landing_accff_tuning_.touchdown_h_settle_m = 0.05;
    landing_accff_tuning_.touchdown_v_settle_mps = 0.05;
    landing_accff_tuning_.touchdown_dwell_s = 0.30;

    // 推导值的健全性校验:防止 gravity/hover_thrust_init 极端取值后失稳
    if (!(takeoff_accff_tuning_.a_start_mps2 < takeoff_accff_tuning_.a_target_mps2 &&
          takeoff_accff_tuning_.a_target_mps2 < takeoff_accff_tuning_.a_max_mps2)) {
        throw std::runtime_error(
            "derived takeoff_accff acc values are inconsistent (check gravity / hover_thrust_percent)");
    }
    if (!(landing_accff_tuning_.a_touchdown_mps2 > landing_accff_tuning_.a_min_mps2 &&
          landing_accff_tuning_.a_touchdown_mps2 < gravity)) {
        throw std::runtime_error(
            "derived landing_accff a_touchdown is inconsistent (check gravity)");
    }
}
