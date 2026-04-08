#include "controller/geometric_controller.hpp"
#include "string_uav_namespace_utils.hpp"
#include <sunray_msgs/UAVControllerState.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// 构造函数
// ─────────────────────────────────────────────────────────────────────────────
Geometric_Controller::Geometric_Controller(ros::NodeHandle& nh) : nh_(nh), mavros_helper_(nh_) {
    std::string node_name = ros::this_node::getName();
    ros::NodeHandle private_nh_("~");

    if (private_nh_.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        if (config_yamlfile_path_.empty()) {
            throw std::runtime_error("yaml_path cannot be empty");
        }
    } else {
        throw std::runtime_error("missing param " + node_name + "/config_yamlfile_path");
    }

    std::string uav_name;
    int uav_id;
    if (nh_.getParam("/uav_name", uav_name)) {
        if (uav_name.empty()) {
            throw std::runtime_error("uav_name cannot be empty");
        }
    } else {
        throw std::runtime_error("missing param /uav_name");
    }
    if (!nh_.getParam("/uav_id", uav_id)) {
        throw std::runtime_error("missing param /uav_id");
    }

    uav_ns_ = uav_name + std::to_string(uav_id);
    uav_ns_ = sunray_common::normalize_uav_ns(uav_ns_);
}

// ─────────────────────────────────────────────────────────────────────────────
// 生命周期
// ─────────────────────────────────────────────────────────────────────────────
bool Geometric_Controller::init() {
    // 加载参数并校验
    load_and_validate_config_or_throw();
    // 将参数加载到核心算法
    controller_.load_param(geometric_controller_param_);

    // 初始化 mavros_helper
    MavrosHelper_ConfigList config_list(true);
    if (!mavros_helper_.init(config_list)) {
        throw std::runtime_error("mavros_helper init failed");
    }

    // 若需要融合外部里程计，检查 EKF2 参数并注册发布定时器
    if (fuse_odom_type != 0) {
        ensure_fusion_param_ready_or_throw();
        mavros_helper_.set_vision_fuse_type(fuse_odom_type);
        pub_vision_pose_timer_ = nh_.createTimer(ros::Duration(1.0 / fuse_odom_frequency),
                                                 &Geometric_Controller::pub_vision_fuse_timer_cb,
                                                 this);
    }

    // 初始化话题发布者
    controller_state_pub_ =
        nh_.advertise<sunray_msgs::UAVControllerState>(uav_ns_ + "/sunray/controller_state", 10);

    // 启动 PX4State 发布定时器
    pub_px4_state_timer_ = nh_.createTimer(ros::Duration(1.0 / pub_px4_state_freq_),
                                           &Geometric_Controller::pub_px4_state_timer_cb,
                                           this);
    return true;
}

bool Geometric_Controller::is_ready() {
    ros::Time now = ros::Time::now();

    if (uav_odometry_.timestamp == ros::Time(0)) {
        ROS_INFO("odom msg lost");
        return false;
    } else if ((now - uav_odometry_.timestamp).toSec() > 0.5) {
        ROS_INFO("odom msg timeout");
        return false;
    }

    control_common::Mavros_State mavros_state = mavros_helper_.get_state();
    if (mavros_state.connected != true) {
        ROS_INFO("mavros not connect");
        return false;
    }

    bool mavros_ready = mavros_helper_.is_ready();
    if (!mavros_ready) {
        ROS_INFO("mavros helper not ready");
        return false;
    }

    controller_ready_ = true;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 状态注入
// ─────────────────────────────────────────────────────────────────────────────
void Geometric_Controller::set_current_odom(const control_common::UAVStateEstimate& odom) {
    uav_odometry_ = odom;
    has_uav_odometry_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 运动相关接口
// ─────────────────────────────────────────────────────────────────────────────
void Geometric_Controller::on_ground_keep_setpoint() {
    control_common::Mavros_SetpointAttitude current_setpoint;
    current_setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
    current_setpoint.body_rate = Eigen::Vector3d::Zero();
    current_setpoint.thrust = 0.0;
    mavros_helper_.pub_attitude_setpoint(current_setpoint);
}

bool Geometric_Controller::takeoff(double relative_takeoff_height, double max_takeoff_velocity) {
    // 如果本轮之前已经降落，清除降落标志
    if (land_complete_ == true) {
        land_complete_ = false;
    }
    // 控制器未就绪则直接返回
    if (controller_ready_ == false) {
        return false;
    }
    // 起飞已完成时退化为悬停
    if (takeoff_complete_ == true) {
        return hover();
    }

    ros::Time now = ros::Time::now();

    // ── 首次进入起飞流程：初始化五次项曲线与 yaw 锁存 ───────────────────────
    if (!quint_curve_.is_ready()) {
        // 起点：当前位置，速度为零
        quint_curve_.set_start_trajpoint(uav_odometry_.position, Eigen::Vector3d::Zero());
        // 终点：当前位置 + 相对起飞高度，速度为零
        quint_curve_.set_end_trajpoint(uav_odometry_.position +
                                           Eigen::Vector3d(0.0, 0.0, relative_takeoff_height),
                                       Eigen::Vector3d::Zero());
        // 由最大起飞速度反推运动时间，使曲线平滑且有速度上限
        quint_curve_.set_curve_maxvel(max_takeoff_velocity);
        // 锁存起飞时刻的 yaw 角和地面高度
        takeoff_yaw_ = mavros_helper_.get_yaw_rad();
        takeoff_ground_height_ = uav_odometry_.position.z();
        // 注意：quint_curve_ 并不在此处记录开始时间，
        // 而是以第一次调用 get_result() 时的时刻作为曲线起点
    }

    control_common::Mavros_State px4_state = mavros_helper_.get_state();

    // ── 阶段 1：切换 Offboard 模式 ──────────────────────────────────────────
    if (px4_state.flight_mode != control_common::FlightMode::Offboard) {
        // 切换前持续发送零指令以满足 Offboard 2 Hz 最低频率要求
        control_common::Mavros_SetpointAttitude setpoint_cmd;
        setpoint_cmd.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
        setpoint_cmd.body_rate = Eigen::Vector3d::Zero();
        setpoint_cmd.thrust = 0.0;
        mavros_helper_.pub_attitude_setpoint(setpoint_cmd);

        if (start_checkout_offboard_time_ == ros::Time(0)) {
            start_checkout_offboard_time_ = now;
            last_checkout_offboard_time_ = ros::Time(0);
        }
        // 每 0.3s 请求一次切换
        if (last_checkout_offboard_time_ == ros::Time(0) ||
            (now - last_checkout_offboard_time_).toSec() >= 0.3) {
            mavros_helper_.set_px4_mode(control_common::FlightMode::Offboard);
            last_checkout_offboard_time_ = now;
        }
        if ((now - start_checkout_offboard_time_).toSec() > 3.0) {
            // TODO: 超过 3s 仍未进入 Offboard 的错误处理
        }
        return false;
    }

    // ── 阶段 2：解锁（成功进入 Offboard，清理切换上下文）──────────────────────
    start_checkout_offboard_time_ = ros::Time(0);
    last_checkout_offboard_time_ = ros::Time(0);

    if (px4_state.armed == false) {
        mavros_helper_.set_arm(true);
        return false;
    }

    // ── 阶段 3：电机暖机（motors speedup）──────────────────────────────────
    // 在无人机离地前保持在地面高度，等待推力建立，避免曲线计时提前开始
    if (last_arm_time_ == ros::Time(0)) {
        last_arm_time_ = now;
    }
    if ((now - last_arm_time_).toSec() < motors_speedup_time_) {
        controller_data_types::TargetTrajectoryPoint_t des_state;
        des_state.position = Eigen::Vector3d(quint_curve_.get_start_position().x(),
                                             quint_curve_.get_start_position().y(),
                                             takeoff_ground_height_);
        des_state.velocity = Eigen::Vector3d::Zero();
        des_state.acceleration = Eigen::Vector3d::Zero();
        des_state.jerk = Eigen::Vector3d::Zero();
        des_state.yaw = takeoff_yaw_;
        des_state.yaw_rate = 0.0;

        auto output = controller_.calculateControl(des_state, uav_odometry_);

        control_common::Mavros_SetpointAttitude setpoint;
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
        setpoint.body_rate = output.bodyrates;
        setpoint.thrust = output.thrust;
        mavros_helper_.pub_attitude_setpoint(setpoint);
        last_setpoint_ = setpoint;
        return false;
    }

    // ── 阶段 4：五次项曲线平滑爬升 ───────────────────────────────────────
    // get_result() 在首次调用时开始计时，输出连续的位置/速度/加速度轨迹
    // 曲线结束后输出值保持在终点，控制器自然收敛悬停
    {
        curve::QuinticCurveState curve_result = quint_curve_.get_result();

        controller_data_types::TargetTrajectoryPoint_t des_state;
        des_state.position = curve_result.position;
        des_state.velocity = curve_result.velocity;
        des_state.acceleration = curve_result.acceleration;
        des_state.jerk = Eigen::Vector3d::Zero();
        des_state.yaw = takeoff_yaw_;
        des_state.yaw_rate = 0.0;

        auto output = controller_.calculateControl(des_state, uav_odometry_);

        control_common::Mavros_SetpointAttitude setpoint;
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
        setpoint.body_rate = output.bodyrates;
        setpoint.thrust = output.thrust;
        mavros_helper_.pub_attitude_setpoint(setpoint);
        last_setpoint_ = setpoint;

        // 到位判断：与曲线终点的位置误差 < 0.3m 且速度 < 0.15m/s，持续稳定才算成功
        const double pos_err = (uav_odometry_.position - quint_curve_.get_end_position()).norm();
        const double vel_err = uav_odometry_.velocity.norm();

        if (pos_err < 0.3 && vel_err < 0.15) {
            if (start_checkout_takeoff_success_time_ == ros::Time(0)) {
                start_checkout_takeoff_success_time_ = now;
            }
            if ((now - start_checkout_takeoff_success_time_).toSec() >
                takeoff_success_keep_time_s) {
                takeoff_complete_ = true;
                // hover_point 设为起飞终点
                hover_point = quint_curve_.get_start_position();
                hover_point.z() += relative_takeoff_height;
                // 清理起飞上下文
                start_checkout_offboard_time_ = ros::Time(0);
                last_checkout_offboard_time_ = ros::Time(0);
                last_arm_time_ = ros::Time(0);
                start_checkout_takeoff_success_time_ = ros::Time(0);
                // 起飞完成后重置积分，避免残留量影响后续 hover/move
                controller_.reset_integral();
                quint_curve_.clear();
                return true;
            }
        } else {
            start_checkout_takeoff_success_time_ = ros::Time(0);
        }
    }
    return false;
}

bool Geometric_Controller::land(bool land_type, double max_land_velocity) {
    // 进入降落流程时重置积分，防止下降阶段因残留积分产生额外推力
    if (landing_time_ == ros::Time(0)) {
        controller_.reset_integral();
    }
    // land_type == 1：切换为 PX4 自带 AutoLand 模式
    if (land_type == 1) {
        mavros_helper_.set_px4_mode(control_common::FlightMode::AutoLand);
        bool land_state =
            mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
        return land_state;
    }

    ros::Time now = ros::Time::now();

    if (land_complete_ == true) {
        return true;
    }

    // 首次进入降落流程：锁定当前位置、yaw 和最大降落速度
    if (landing_time_ == ros::Time(0)) {
        landing_time_ = now;
        land_point_ = uav_odometry_.position;
        land_yaw_ = mavros_helper_.get_yaw_rad();
        land_max_velocity_ = max_land_velocity;
    }

    // 时基降落：目标 z 随时间线性下降
    double elapsed = (now - landing_time_).toSec();
    double target_z = land_point_.z() - land_max_velocity_ * elapsed;
    // 防止目标高度无限下降，限制在地面以下 0.2m
    target_z = std::max(target_z, takeoff_ground_height_ - 0.2);

    controller_data_types::TargetTrajectoryPoint_t des_state;
    des_state.position = Eigen::Vector3d(land_point_.x(), land_point_.y(), target_z);
    des_state.velocity = Eigen::Vector3d(0.0, 0.0, -land_max_velocity_);
    des_state.acceleration = Eigen::Vector3d::Zero();
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = land_yaw_;
    des_state.yaw_rate = 0.0;

    auto output = controller_.calculateControl(des_state, uav_odometry_);

    control_common::Mavros_SetpointAttitude setpoint;
    setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
    setpoint.body_rate = output.bodyrates;
    setpoint.thrust = output.thrust;

    // 触地判断：靠近地面 + 三轴速度均低
    bool near_ground = uav_odometry_.position.z() <= takeoff_ground_height_ + 0.05;
    const bool velocity_low = std::abs(uav_odometry_.velocity.x()) < 0.1 &&
                              std::abs(uav_odometry_.velocity.y()) < 0.1 &&
                              std::abs(uav_odometry_.velocity.z()) < 0.1;

    bool landed_by_velocity = false;
    if (near_ground && velocity_low) {
        if (start_land_time_ == ros::Time(0)) {
            start_land_time_ = now;
        } else if ((now - start_land_time_).toSec() > 2.0) {
            landed_by_velocity = true;
        }
    } else {
        start_land_time_ = ros::Time(0);
    }

    control_common::LandedState px4_land_state = mavros_helper_.get_state().landed_state;
    bool px4_landed = (px4_land_state == control_common::LandedState::OnGround);
    const bool landed_detected = px4_landed || landed_by_velocity;

    // 靠近地面或已触地时压制推力上限，防止弹跳
    if (landed_by_velocity) {
        setpoint.thrust = std::clamp(setpoint.thrust, 0.02, 0.05);
    } else if (near_ground) {
        setpoint.thrust = std::clamp(setpoint.thrust, 0.02, 0.31);
    }

    if (landed_detected && landed_by_velocity) {
        if (land_touchground_time_ == ros::Time(0)) {
            land_touchground_time_ = now;
        }
        if ((now - land_touchground_time_).toSec() > 1.0) {
            mavros_helper_.set_arm(false);
            if (mavros_helper_.get_state().armed == false) {
                // 上锁成功，清理降落上下文
                land_complete_ = true;
                takeoff_complete_ = false;
                start_land_time_ = ros::Time(0);
                land_near_ground_ = false;
                land_touchground_time_ = ros::Time(0);
                landing_time_ = ros::Time(0);
            }
            return land_complete_;
        }
    } else {
        land_touchground_time_ = ros::Time(0);
    }

    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;
    return false;
}

bool Geometric_Controller::hover() {
    controller_data_types::TargetTrajectoryPoint_t des_state;
    des_state.position = hover_point;
    des_state.velocity = Eigen::Vector3d::Zero();
    des_state.acceleration = Eigen::Vector3d::Zero();
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = takeoff_yaw_;
    des_state.yaw_rate = 0.0;

    auto output = controller_.calculateControl(des_state, uav_odometry_);

    control_common::Mavros_SetpointAttitude setpoint;
    setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
    setpoint.body_rate = output.bodyrates;
    setpoint.thrust = output.thrust;
    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;
    return true;
}

bool Geometric_Controller::emergency_kill() {
    return mavros_helper_.emergency_kill();
}

bool Geometric_Controller::move_point(controller_data_types::TargetPoint_t point) {
    ros::Time now = ros::Time::now();

    // 新目标检测
    constexpr double kNewTargetPosEps = 1e-3;
    bool is_new_target = false;
    const double dp = (point.position - last_point_.position).norm();
    if (dp > kNewTargetPosEps) {
        is_new_target = true;
    }
    if (is_new_target) {
        move_point_arrive_state_ = false;
        start_move_arrive_time_ = ros::Time(0);
        last_point_ = point;
    }

    controller_data_types::TargetTrajectoryPoint_t des_state;
    des_state.position = point.position;
    des_state.velocity = Eigen::Vector3d::Zero();
    des_state.acceleration = Eigen::Vector3d::Zero();
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = point.yaw;
    des_state.yaw_rate = 0.0;

    auto output = controller_.calculateControl(des_state, uav_odometry_);

    control_common::Mavros_SetpointAttitude setpoint;
    setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
    setpoint.body_rate = output.bodyrates;
    setpoint.thrust = output.thrust;
    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;

    // 到位检查：位置误差 < 0.15m 且速度误差 < 0.1m/s，持续 0.5s
    double pos_err = (uav_odometry_.position - point.position).norm();
    double vel_err = uav_odometry_.velocity.norm();
    ROS_INFO("pos_err : %f", pos_err);
    ROS_INFO("vel_err : %f", vel_err);

    if (pos_err < 0.15 && vel_err < 0.1) {
        if (start_move_arrive_time_ == ros::Time(0)) {
            start_move_arrive_time_ = now;
        }
        if ((now - start_move_arrive_time_).toSec() > 0.5) {
            move_point_arrive_state_ = true;
            point_complete_ = true;
            hover_point = last_point_.position;
        }
    } else {
        start_move_arrive_time_ = ros::Time(0);
        point_complete_ = false;
    }
    return move_point_arrive_state_;
}

bool Geometric_Controller::move_velocity(controller_data_types::TargetVelocity_t velocity) {
    return false;
}

bool Geometric_Controller::move_trajectory(
    controller_data_types::TargetTrajectoryPoint_t trajpoint) {
    // 几何控制器的核心运动接口。
    // 将完整的轨迹点（位置 / 速度 / 加速度 / 加加速度 / yaw）直接送入核心算法，
    // 由两级控制环（位置 PD + 姿态子控制器）输出 body rate 和归一化推力。
    desired_state_ = trajpoint;

    auto output = controller_.calculateControl(trajpoint, uav_odometry_);

    control_common::Mavros_SetpointAttitude setpoint;
    setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
    setpoint.body_rate = output.bodyrates;
    setpoint.thrust = output.thrust;
    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;
    return true;
}

bool Geometric_Controller::move_point_body(controller_data_types::TargetPoint_t point) {
    return false;
}

bool Geometric_Controller::move_velocity_body(controller_data_types::TargetVelocity_t velocity) {
    return false;
}

bool Geometric_Controller::move_point_wgs84(geographic_msgs::GeoPoint point) {
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// 起降状态查询接口
// ─────────────────────────────────────────────────────────────────────────────
bool Geometric_Controller::is_takeoff_complete() {
    return takeoff_complete_;
}

bool Geometric_Controller::is_land_complete() {
    return land_complete_;
}

bool Geometric_Controller::is_point_complete() {
    return point_complete_;
}

// ─────────────────────────────────────────────────────────────────────────────
// 控制器状态话题更新（stub，待后续填充）
// ─────────────────────────────────────────────────────────────────────────────
void Geometric_Controller::pub_controller_state() {
    return;
}

// ═════════════════════════════════════════════════════════════════════════════
// 私有函数
// ═════════════════════════════════════════════════════════════════════════════

void Geometric_Controller::load_and_validate_config_or_throw() {
    YAML::Node root;
    try {
        root = YAML::LoadFile(config_yamlfile_path_);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load yaml file '" + config_yamlfile_path_ +
                                 "': " + e.what());
    }

    // ──── basic_param ─────────────────────────────────────────────────────────
    const YAML::Node basic_param = root["basic_param"];
    if (!basic_param || !basic_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_yamlfile_path_ +
                                 "' is missing a valid 'basic_param' map");
    }

    if (!basic_param["fuse_odom_type"]) {
        throw std::runtime_error("missing param 'fuse_odom_type'");
    }
    fuse_odom_type = basic_param["fuse_odom_type"].as<int>();

    if (!basic_param["fuse_odom_frequency"]) {
        throw std::runtime_error("missing param 'fuse_odom_frequency'");
    }
    fuse_odom_frequency = basic_param["fuse_odom_frequency"].as<double>();

    if (fuse_odom_type != 0 && fuse_odom_type != 1 && fuse_odom_type != 2) {
        throw std::runtime_error("param 'fuse_odom_type' must be 0, 1 or 2");
    }
    fuse_odom_frequency = std::max(10.0, fuse_odom_frequency);
    fuse_odom_frequency = std::min(200.0, fuse_odom_frequency);

    if (!basic_param["gravity"]) {
        throw std::runtime_error("missing param 'gravity'");
    }
    geometric_controller_param_.gravity = basic_param["gravity"].as<double>();

    if (!basic_param["mass_kg"]) {
        throw std::runtime_error("missing param 'mass_kg'");
    }
    geometric_controller_param_.drone_mass = basic_param["mass_kg"].as<double>();

    // ──── geometric_controller_param ──────────────────────────────────────────
    const YAML::Node geo_param = root["geometric_controller_param"];
    if (!geo_param || !geo_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_yamlfile_path_ +
                                 "' is missing a valid 'geometric_controller_param' map");
    }

    if (!geo_param["ctrl_mode"]) {
        throw std::runtime_error("missing param 'ctrl_mode'");
    }
    geometric_controller_param_.ctrl_mode = geo_param["ctrl_mode"].as<int>();
    if (geometric_controller_param_.ctrl_mode != 0 && geometric_controller_param_.ctrl_mode != 1) {
        throw std::runtime_error("param 'ctrl_mode' must be 0 (quaternion) or 1 (SO3)");
    }

    if (!geo_param["attctrl_tau"]) {
        throw std::runtime_error("missing param 'attctrl_tau'");
    }
    geometric_controller_param_.attctrl_tau = geo_param["attctrl_tau"].as<double>();

    if (!geo_param["norm_thrust_const"]) {
        throw std::runtime_error("missing param 'norm_thrust_const'");
    }
    geometric_controller_param_.norm_thrust_const = geo_param["norm_thrust_const"].as<double>();

    if (!geo_param["norm_thrust_offset"]) {
        throw std::runtime_error("missing param 'norm_thrust_offset'");
    }
    geometric_controller_param_.norm_thrust_offset = geo_param["norm_thrust_offset"].as<double>();

    // controller_hz 可选，缺省保持结构体默认值 100.0
    if (geo_param["controller_hz"]) {
        geometric_controller_param_.controller_hz = geo_param["controller_hz"].as<double>();
    }

    if (!geo_param["max_acc"]) {
        throw std::runtime_error("missing param 'max_acc'");
    }
    geometric_controller_param_.max_acc = geo_param["max_acc"].as<double>();

    // ──── gain ────────────────────────────────────────────────────────────────
    const YAML::Node gain_param = geo_param["gain"];
    if (!gain_param || !gain_param.IsMap()) {
        throw std::runtime_error("missing valid 'gain' map under 'geometric_controller_param'");
    }

    if (!gain_param["Kpos"] || !gain_param["Kpos"].IsSequence() || gain_param["Kpos"].size() != 3) {
        throw std::runtime_error(
            "missing or invalid param 'gain/Kpos' (expected sequence of 3 values)");
    }
    geometric_controller_param_.Kpos.x() = gain_param["Kpos"][0].as<double>();
    geometric_controller_param_.Kpos.y() = gain_param["Kpos"][1].as<double>();
    geometric_controller_param_.Kpos.z() = gain_param["Kpos"][2].as<double>();

    if (!gain_param["Kvel"] || !gain_param["Kvel"].IsSequence() || gain_param["Kvel"].size() != 3) {
        throw std::runtime_error(
            "missing or invalid param 'gain/Kvel' (expected sequence of 3 values)");
    }
    geometric_controller_param_.Kvel.x() = gain_param["Kvel"][0].as<double>();
    geometric_controller_param_.Kvel.y() = gain_param["Kvel"][1].as<double>();
    geometric_controller_param_.Kvel.z() = gain_param["Kvel"][2].as<double>();

    // Z 轴积分参数（可选，缺省时使用结构体默认值 0.0，即禁用积分）
    if (gain_param["pos_ki_z"]) {
        geometric_controller_param_.pos_ki_z = gain_param["pos_ki_z"].as<double>();
    }
    if (gain_param["int_max_z"]) {
        geometric_controller_param_.int_max_z = gain_param["int_max_z"].as<double>();
    }

    // ──── drag ────────────────────────────────────────────────────────────────
    const YAML::Node drag_param = geo_param["drag"];
    if (!drag_param || !drag_param.IsMap()) {
        throw std::runtime_error("missing valid 'drag' map under 'geometric_controller_param'");
    }

    if (!drag_param["D"] || !drag_param["D"].IsSequence() || drag_param["D"].size() != 3) {
        throw std::runtime_error(
            "missing or invalid param 'drag/D' (expected sequence of 3 values)");
    }
    geometric_controller_param_.D.x() = drag_param["D"][0].as<double>();
    geometric_controller_param_.D.y() = drag_param["D"][1].as<double>();
    geometric_controller_param_.D.z() = drag_param["D"][2].as<double>();
}

void Geometric_Controller::ensure_fusion_param_ready_or_throw() {
    if (fuse_odom_type == 0) {
        return;  // 不做视觉融合，无需检查
    }
    // Geometric_Controller 不依赖 PX4_ParamManager，无法自动读写 EKF2 参数。
    // 使用者需在起飞前手动确认以下 PX4 参数已正确设置：
    //   EKF2_EV_CTRL : enable_horizontal_position + enable_vertical_position + enable_yaw
    //   EKF2_HGT_REF : vision
    // 这里仅打印告警，由操作员自行确认。
    ROS_WARN("[Geometric_Controller] fuse_odom_type=%d: EKF2 fusion params cannot be verified "
             "automatically (PX4_ParamManager not used). Please ensure EKF2_EV_CTRL and "
             "EKF2_HGT_REF are correctly configured on the flight controller before flight.",
             fuse_odom_type);
}

bool Geometric_Controller::check_px4_basic_state() {
    const control_common::Mavros_State st = mavros_helper_.get_state();
    const bool mode_ok = (st.flight_mode == control_common::FlightMode::Posctl);
    const bool land_ok = (st.landed_state == control_common::LandedState::OnGround);
    const bool arm_ok = (st.armed == false);
    return mode_ok && land_ok && arm_ok;
}

bool Geometric_Controller::check_mavros_stream_ready() {
    return mavros_helper_.is_ready();
}

bool Geometric_Controller::check_odom_freshness() {
    return ((ros::Time::now() - uav_odometry_.timestamp).toSec() < 0.15);
}

bool Geometric_Controller::check_odom_for_fusion(control_common::UAVStateEstimate& fuse_odom) {
    // 1. 时间戳有效
    if (fuse_odom.timestamp.isZero()) {
        return false;
    }
    // 2. 新鲜度
    if (!check_odom_freshness()) {
        return false;
    }
    // 3. 数值有界
    auto finite3 = [](double a, double b, double c) {
        return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
    };
    if (!finite3(fuse_odom.position.x(), fuse_odom.position.y(), fuse_odom.position.z()))
        return false;
    if (!finite3(fuse_odom.velocity.x(), fuse_odom.velocity.y(), fuse_odom.velocity.z()))
        return false;
    if (!finite3(fuse_odom.bodyrate.x(), fuse_odom.bodyrate.y(), fuse_odom.bodyrate.z()))
        return false;
    // 4. 四元数范数检查
    const double qn = std::sqrt(fuse_odom.orientation.w() * fuse_odom.orientation.w() +
                                fuse_odom.orientation.x() * fuse_odom.orientation.x() +
                                fuse_odom.orientation.y() * fuse_odom.orientation.y() +
                                fuse_odom.orientation.z() * fuse_odom.orientation.z());
    if (qn < 1e-6)
        return false;  // 退化四元数
    if (std::fabs(qn - 1.0) > 0.2)
        return false;  // 偏离单位四元数过大
    return true;
}

void Geometric_Controller::pub_px4_state_timer_cb(const ros::TimerEvent&) {
    mavros_helper_.pub_px4_state();
}

void Geometric_Controller::pub_vision_fuse_timer_cb(const ros::TimerEvent&) {
    if (has_uav_odometry_) {
        mavros_helper_.pub_vision_pose(uav_odometry_);
    }
}
