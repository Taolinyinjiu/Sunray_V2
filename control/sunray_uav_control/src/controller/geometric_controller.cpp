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
    has_imu_ = has_valid_imu_data();
}

// ─────────────────────────────────────────────────────────────────────────────
// 运动相关接口
// ─────────────────────────────────────────────────────────────────────────────
void Geometric_Controller::on_ground_keep_setpoint() {
    reset_velocity_trajectory_state();
    reset_stage_thrust_filters();
    land_near_ground_ = false;
    control_common::Mavros_SetpointAttitude current_setpoint;
    if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
        current_setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                                control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                                control_common::Mavros_SetpointAttitude::IgnoreYawRate;
        current_setpoint.orientation =
            has_uav_odometry_ ? uav_odometry_.orientation : Eigen::Quaterniond::Identity();
        current_setpoint.body_rate = Eigen::Vector3d::Zero();
    } else {
        current_setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
        current_setpoint.body_rate = Eigen::Vector3d::Zero();
    }
    current_setpoint.thrust = 0.0;
    mavros_helper_.pub_attitude_setpoint(current_setpoint);
    last_setpoint_ = current_setpoint;
}

void Geometric_Controller::seed_rc_thrust_filter(RCThrustFilterState& state,
                                                 double thrust,
                                                 const ros::Time& now) {
    state.initialized = true;
    state.thrust = std::clamp(thrust, 0.0, 0.95);
    state.last_update = now;
}

double Geometric_Controller::update_rc_thrust_filter(RCThrustFilterState& state,
                                                     double target_thrust,
                                                     double tau_s,
                                                     const ros::Time& now) {
    const double clamped_target = std::clamp(target_thrust, 0.0, 0.95);
    if (tau_s <= 1e-4) {
        seed_rc_thrust_filter(state, clamped_target, now);
        return clamped_target;
    }

    if (!state.initialized) {
        seed_rc_thrust_filter(state, clamped_target, now);
        return state.thrust;
    }

    if (state.last_update.isZero()) {
        state.last_update = now;
        return state.thrust;
    }

    const double dt = (now - state.last_update).toSec();
    state.last_update = now;
    if (dt <= 1e-6) {
        return state.thrust;
    }

    const double alpha = std::clamp(dt / (tau_s + dt), 0.0, 1.0);
    state.thrust = std::clamp(state.thrust + alpha * (clamped_target - state.thrust), 0.0, 0.95);
    return state.thrust;
}

double Geometric_Controller::get_takeoff_warmup_duration() const {
    return std::max(0.0, takeoff_idle_hold_time_) + std::max(0.0, takeoff_ramp_time_);
}

double Geometric_Controller::compute_takeoff_warmup_target_thrust(double target_thrust,
                                                                  double elapsed_s) const {
    const double idle_thrust = std::clamp(takeoff_idle_thrust_, 0.0, 0.95);
    const double climb_target = std::clamp(std::max(target_thrust, idle_thrust), 0.0, 0.95);

    if (elapsed_s <= takeoff_idle_hold_time_) {
        return idle_thrust;
    }

    const double ramp_time = std::max(1e-3, takeoff_ramp_time_);
    const double ramp_elapsed = elapsed_s - takeoff_idle_hold_time_;
    const double blend = std::clamp(ramp_elapsed / ramp_time, 0.0, 1.0);
    return idle_thrust + blend * (climb_target - idle_thrust);
}

void Geometric_Controller::maybe_rebase_takeoff_curve_start() {
    if (takeoff_curve_started_ || !quint_curve_.is_ready()) {
        return;
    }

    const Eigen::Vector3d original_start = quint_curve_.get_start_position();
    const Eigen::Vector3d original_end = quint_curve_.get_end_position();
    const Eigen::Vector3d current_pos = uav_odometry_.position;
    const Eigen::Vector3d current_vel = uav_odometry_.velocity;
    const Eigen::Vector3d delta = current_pos - original_start;

    if (delta.norm() > 0.03 || current_vel.norm() > 0.05) {
        ROS_WARN("takeoff curve rebase dp=(%.3f, %.3f, %.3f) v=(%.3f, %.3f, %.3f) end=(%.3f, %.3f, %.3f)",
                 delta.x(),
                 delta.y(),
                 delta.z(),
                 current_vel.x(),
                 current_vel.y(),
                 current_vel.z(),
                 original_end.x(),
                 original_end.y(),
                 original_end.z());
    } else {
        ROS_INFO("takeoff curve rebase dp=(%.3f, %.3f, %.3f)",
                 delta.x(),
                 delta.y(),
                 delta.z());
    }

    quint_curve_.set_start_trajpoint(current_pos, current_vel);
    takeoff_curve_started_ = true;
}

void Geometric_Controller::update_hover_reference(const Eigen::Vector3d& hover_point_target,
                                                 double hover_yaw_target,
                                                 const char* reason) {
    const Eigen::Vector3d old_hover_point = hover_point;
    const double old_hover_yaw = hover_yaw_;
    hover_point = hover_point_target;
    hover_yaw_ = hover_yaw_target;

    const Eigen::Vector3d dp = hover_point - old_hover_point;
    const double dyaw = hover_yaw_ - old_hover_yaw;
    const Eigen::Vector3d pos_err = hover_point - uav_odometry_.position;
    ROS_INFO("hover_ref[%s] old=(%.3f, %.3f, %.3f, yaw=%.3f) new=(%.3f, %.3f, %.3f, yaw=%.3f) d=(%.3f, %.3f, %.3f, dyaw=%.3f) cur=(%.3f, %.3f, %.3f) err=(%.3f, %.3f, %.3f)",
             reason,
             old_hover_point.x(),
             old_hover_point.y(),
             old_hover_point.z(),
             old_hover_yaw,
             hover_point.x(),
             hover_point.y(),
             hover_point.z(),
             hover_yaw_,
             dp.x(),
             dp.y(),
             dp.z(),
             dyaw,
             uav_odometry_.position.x(),
             uav_odometry_.position.y(),
             uav_odometry_.position.z(),
             pos_err.x(),
             pos_err.y(),
             pos_err.z());
}

void Geometric_Controller::reset_stage_thrust_filters() {
    takeoff_thrust_filter_ = RCThrustFilterState{};
    land_thrust_filter_ = RCThrustFilterState{};
}

bool Geometric_Controller::takeoff(double relative_takeoff_height, double max_takeoff_velocity) {
    reset_velocity_trajectory_state();
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
        // 锁存起飞时刻的 yaw 角和地面高度，这里从uav_odometry_获取yaw角，可以不向px4融合里程计
        takeoff_yaw_ = uav_odometry_.get_yaw();
        takeoff_ground_height_ = uav_odometry_.position.z();
        // 将起飞 yaw 同步到核心算法缓存，防止后续 move_point 未显式设置 yaw 时默认归零
        controller_.set_initial_yaw(takeoff_yaw_);
        takeoff_curve_started_ = false;
        // 注意：quint_curve_ 并不在此处记录开始时间，
        // 而是以第一次调用 get_result() 时的时刻作为曲线起点
    }

    control_common::Mavros_State px4_state = mavros_helper_.get_state();

    // ── 阶段 1：切换 Offboard 模式 ──────────────────────────────────────────
    if (px4_state.flight_mode != control_common::FlightMode::Offboard) {
        reset_stage_thrust_filters();
        takeoff_curve_started_ = false;
        // 切换前持续发送零指令以满足 Offboard 2 Hz 最低频率要求
        control_common::Mavros_SetpointAttitude setpoint_cmd;
        if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
            setpoint_cmd.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                                control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                                control_common::Mavros_SetpointAttitude::IgnoreYawRate;
            setpoint_cmd.orientation =
                has_uav_odometry_ ? uav_odometry_.orientation : Eigen::Quaterniond::Identity();
            setpoint_cmd.body_rate = Eigen::Vector3d::Zero();
        } else {
            setpoint_cmd.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
            setpoint_cmd.body_rate = Eigen::Vector3d::Zero();
        }
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
        reset_stage_thrust_filters();
        takeoff_curve_started_ = false;
        mavros_helper_.set_arm(true);
        return false;
    }

    // ── 阶段 3：电机暖机（motors speedup）──────────────────────────────────
    // 在无人机离地前保持在地面高度，等待推力建立，避免曲线计时提前开始
    if (last_arm_time_ == ros::Time(0)) {
        last_arm_time_ = now;
        seed_rc_thrust_filter(takeoff_thrust_filter_, takeoff_idle_thrust_, now);
    }
    const double takeoff_elapsed = (now - last_arm_time_).toSec();
    const double takeoff_warmup_duration = get_takeoff_warmup_duration();
    if (takeoff_elapsed < takeoff_warmup_duration) {
        controller_data_types::TargetTrajectoryPoint_t des_state;
        des_state.position = Eigen::Vector3d(quint_curve_.get_start_position().x(),
                                             quint_curve_.get_start_position().y(),
                                             takeoff_ground_height_);
        des_state.velocity = Eigen::Vector3d::Zero();
        des_state.acceleration = Eigen::Vector3d::Zero();
        des_state.jerk = Eigen::Vector3d::Zero();
        des_state.yaw = takeoff_yaw_;
        des_state.yaw_rate = 0.0;

        // 起飞暖机阶段强制使用线性推力模型，同时不更新推力估计器。
        auto output = controller_.calculateControl(des_state,
                                                   uav_odometry_,
                                                   ThrustCommandPolicy::ForceLinear);
#ifdef DEBUG
        ROS_INFO_THROTTLE(1.0, "est:skip tk0 z=%.2f vz=%.2f u=%.3f",
                          uav_odometry_.position.z(),
                          uav_odometry_.velocity.z(),
                          output.thrust);
#endif

        control_common::Mavros_SetpointAttitude setpoint;
        if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
            setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                            control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                            control_common::Mavros_SetpointAttitude::IgnoreYawRate;
            setpoint.orientation = output.orientation;
            setpoint.body_rate = Eigen::Vector3d::Zero();
        } else {
            setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
            setpoint.body_rate = output.bodyrates;
        }
        const double warmup_target_thrust =
            compute_takeoff_warmup_target_thrust(output.thrust, takeoff_elapsed);
        setpoint.thrust = update_rc_thrust_filter(takeoff_thrust_filter_,
                                                  warmup_target_thrust,
                                                  takeoff_thrust_filter_tau_,
                                                  now);
        mavros_helper_.pub_attitude_setpoint(setpoint);
        last_setpoint_ = setpoint;
        return false;
    }

    // ── 阶段 4：五次项曲线平滑爬升 ───────────────────────────────────────
    // get_result() 在首次调用时开始计时，输出连续的位置/速度/加速度轨迹
    // 曲线结束后输出值保持在终点，控制器自然收敛悬停
    {
        maybe_rebase_takeoff_curve_start();
        curve::QuinticCurveState curve_result = quint_curve_.get_result();

        controller_data_types::TargetTrajectoryPoint_t des_state;
        des_state.position = curve_result.position;
        des_state.velocity = curve_result.velocity;
        des_state.acceleration = curve_result.acceleration;
        des_state.jerk = Eigen::Vector3d::Zero();
        des_state.yaw = takeoff_yaw_;
        des_state.yaw_rate = 0.0;

        // 起飞爬升阶段强制使用线性推力模型，同时不更新推力估计器。
        auto output = controller_.calculateControl(des_state,
                                                   uav_odometry_,
                                                   ThrustCommandPolicy::ForceLinear);
#ifdef DEBUG
        ROS_INFO_THROTTLE(1.0, "est:skip tk1 z=%.2f vz=%.2f az_ref=%.2f u=%.3f",
                          uav_odometry_.position.z(),
                          uav_odometry_.velocity.z(),
                          curve_result.acceleration.z(),
                          output.thrust);
#endif

        control_common::Mavros_SetpointAttitude setpoint;
        if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
            setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                            control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                            control_common::Mavros_SetpointAttitude::IgnoreYawRate;
            setpoint.orientation = output.orientation;
            setpoint.body_rate = Eigen::Vector3d::Zero();
        } else {
            setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
            setpoint.body_rate = output.bodyrates;
        }
        const bool use_takeoff_handoff_filter =
            takeoff_thrust_filter_.initialized &&
            takeoff_elapsed < takeoff_warmup_duration + takeoff_thrust_handoff_time_;
        if (use_takeoff_handoff_filter) {
            setpoint.thrust = update_rc_thrust_filter(takeoff_thrust_filter_,
                                                      output.thrust,
                                                      takeoff_thrust_filter_tau_,
                                                      now);
        } else {
            takeoff_thrust_filter_ = RCThrustFilterState{};
            setpoint.thrust = output.thrust;
        }
        mavros_helper_.pub_attitude_setpoint(setpoint);
        last_setpoint_ = setpoint;

        // 到位判断：使用统一的到达稳定参数，而不是为 takeoff / move_point 分别写死阈值
        const double pos_err = (uav_odometry_.position - quint_curve_.get_end_position()).norm();
        const double vel_err = uav_odometry_.velocity.norm();

        if (pos_err < arrival_pos_stabile_err_m_ && vel_err < arrival_vel_stabile_err_mps_) {
            if (start_checkout_takeoff_success_time_ == ros::Time(0)) {
                start_checkout_takeoff_success_time_ = now;
            }
            if ((now - start_checkout_takeoff_success_time_).toSec() >
                arrival_judge_stabile_time_s_) {
                takeoff_complete_ = true;
                // hover_point 设为起飞轨迹终点，避免曲线起点重对齐后与终点语义脱节
                update_hover_reference(quint_curve_.get_end_position(),
                                       takeoff_yaw_,
                                       "takeoff_complete");
                // 清理起飞上下文
                start_checkout_offboard_time_ = ros::Time(0);
                last_checkout_offboard_time_ = ros::Time(0);
                last_arm_time_ = ros::Time(0);
                start_checkout_takeoff_success_time_ = ros::Time(0);
                takeoff_curve_started_ = false;
                reset_stage_thrust_filters();
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
    reset_velocity_trajectory_state();
    takeoff_thrust_filter_ = RCThrustFilterState{};
    // 进入降落流程时重置积分，防止下降阶段因残留积分产生额外推力
    if (landing_time_ == ros::Time(0)) {
        controller_.reset_integral();
    }
    // land_type == 1：切换为 PX4 自带 AutoLand 模式
    if (land_type == 1) {
        reset_stage_thrust_filters();
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
        land_near_ground_ = false;
        land_thrust_filter_ = RCThrustFilterState{};
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

    // 降落阶段强制使用线性推力模型，同时不更新推力估计器。
    auto output = controller_.calculateControl(des_state,
                                               uav_odometry_,
                                               ThrustCommandPolicy::ForceLinear);
#ifdef DEBUG
    ROS_INFO_THROTTLE(1.0, "est:skip land z=%.2f vz=%.2f u=%.3f",
                      uav_odometry_.position.z(),
                      uav_odometry_.velocity.z(),
                      output.thrust);
#endif

    control_common::Mavros_SetpointAttitude setpoint;
    if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                        control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                        control_common::Mavros_SetpointAttitude::IgnoreYawRate;
        setpoint.orientation = output.orientation;
        setpoint.body_rate = Eigen::Vector3d::Zero();
    } else {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
        setpoint.body_rate = output.bodyrates;
    }
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

    if (near_ground && !land_near_ground_) {
        land_near_ground_ = true;
        const double seed_thrust =
            (last_setpoint_.thrust > 0.0) ? last_setpoint_.thrust : setpoint.thrust;
        seed_rc_thrust_filter(land_thrust_filter_, seed_thrust, now);
    }

    // 靠近地面或已触地时对推力做 RC 平滑收敛，减少压地/反弹时的突变。
    if (land_near_ground_ || landed_by_velocity) {
        const double thrust_cap =
            landed_by_velocity ? land_touchdown_thrust_max_ : land_near_ground_thrust_max_;
        const double thrust_target = std::clamp(setpoint.thrust, 0.02, thrust_cap);
        setpoint.thrust = update_rc_thrust_filter(land_thrust_filter_,
                                                  thrust_target,
                                                  land_thrust_filter_tau_,
                                                  now);
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
                reset_stage_thrust_filters();
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

bool Geometric_Controller::set_hover_point(control_common::UAVStateEstimate current_odom) {
    update_hover_reference(current_odom.position, current_odom.get_yaw(), "set_hover_point");
    return true;
}

bool Geometric_Controller::hover() {
    reset_velocity_trajectory_state();
    controller_data_types::TargetTrajectoryPoint_t des_state;
    des_state.position = hover_point;
    des_state.velocity = Eigen::Vector3d::Zero();
    des_state.acceleration = Eigen::Vector3d::Zero();
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = hover_yaw_;
    des_state.yaw_rate = 0.0;

    ROS_INFO_THROTTLE(1.0,
                      "hover track tgt=(%.3f, %.3f, %.3f, yaw=%.3f) cur=(%.3f, %.3f, %.3f) err=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f)",
                      hover_point.x(),
                      hover_point.y(),
                      hover_point.z(),
                      hover_yaw_,
                      uav_odometry_.position.x(),
                      uav_odometry_.position.y(),
                      uav_odometry_.position.z(),
                      hover_point.x() - uav_odometry_.position.x(),
                      hover_point.y() - uav_odometry_.position.y(),
                      hover_point.z() - uav_odometry_.position.z(),
                      uav_odometry_.velocity.x(),
                      uav_odometry_.velocity.y(),
                      uav_odometry_.velocity.z());

    auto output =
        controller_.calculateControl(des_state, uav_odometry_, ThrustCommandPolicy::Auto);
    if (has_valid_imu_data()) {
        thrust_estimator::Input_t estimator_input;
        estimator_input.stamp = mavros_helper_.get_imu_data().stamp;
        estimator_input.attitude = uav_odometry_.orientation;
        estimator_input.velocity_w = uav_odometry_.velocity;
        estimator_input.acceleration_w = get_world_acc_from_imu();
#ifdef DEBUG
        ROS_INFO_THROTTLE(1.0, "est:feed hover az=%.2f vz=%.2f u=%.3f",
                          estimator_input.acceleration_w.z(),
                          estimator_input.velocity_w.z(),
                          output.thrust);
#endif
        controller_.feed_thrust_estimator(estimator_input);
    }

    control_common::Mavros_SetpointAttitude setpoint;
    if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                        control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                        control_common::Mavros_SetpointAttitude::IgnoreYawRate;
        setpoint.orientation = output.orientation;
        setpoint.body_rate = Eigen::Vector3d::Zero();
    } else {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
        setpoint.body_rate = output.bodyrates;
    }
    setpoint.thrust = output.thrust;
    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;
    return true;
}

bool Geometric_Controller::emergency_kill() {
    reset_velocity_trajectory_state();
    return mavros_helper_.emergency_kill();
}

bool Geometric_Controller::move_point(controller_data_types::TargetPoint_t point) {
    reset_velocity_trajectory_state();
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
        point_complete_ = false;
        start_move_arrive_time_ = ros::Time(0);
        controller_.reset_vertical_integral();
        last_point_ = point;
    }

    controller_data_types::TargetTrajectoryPoint_t des_state;
    des_state.position = point.position;
    des_state.velocity = Eigen::Vector3d::Zero();
    des_state.acceleration = Eigen::Vector3d::Zero();
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = point.yaw;
    des_state.yaw_rate = 0.0;

    auto output =
        controller_.calculateControl(des_state, uav_odometry_, ThrustCommandPolicy::Auto);

    control_common::Mavros_SetpointAttitude setpoint;
    if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                        control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                        control_common::Mavros_SetpointAttitude::IgnoreYawRate;
        setpoint.orientation = output.orientation;
        setpoint.body_rate = Eigen::Vector3d::Zero();
    } else {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
        setpoint.body_rate = output.bodyrates;
    }
    setpoint.thrust = output.thrust;
    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;

    // 到位检查：与起飞共享统一的 arrival_judge_param
    double pos_err = (uav_odometry_.position - point.position).norm();
    double vel_err = uav_odometry_.velocity.norm();
    // ROS_INFO("pos_err : %f", pos_err);
    // ROS_INFO("vel_err : %f", vel_err);

    if (pos_err < arrival_pos_stabile_err_m_ && vel_err < arrival_vel_stabile_err_mps_) {
        if (start_move_arrive_time_ == ros::Time(0)) {
            start_move_arrive_time_ = now;
        }
        if ((now - start_move_arrive_time_).toSec() > arrival_judge_stabile_time_s_) {
            move_point_arrive_state_ = true;
            point_complete_ = true;
            update_hover_reference(last_point_.position, last_point_.yaw, "move_point_arrive");
        }
    } else {
        start_move_arrive_time_ = ros::Time(0);
        point_complete_ = false;
    }
    return move_point_arrive_state_;
}

bool Geometric_Controller::move_velocity(controller_data_types::TargetVelocity_t velocity) {
    reset_velocity_trajectory_state();

    auto output =
        controller_.calculateVelocityControl(velocity, uav_odometry_, ThrustCommandPolicy::Auto);

    control_common::Mavros_SetpointAttitude setpoint;
    if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                        control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                        control_common::Mavros_SetpointAttitude::IgnoreYawRate;
        setpoint.orientation = output.orientation;
        setpoint.body_rate = Eigen::Vector3d::Zero();
    } else {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
        setpoint.body_rate = output.bodyrates;
    }
    setpoint.thrust = output.thrust;
    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;
    return true;
}

bool Geometric_Controller::move_trajectory(
    controller_data_types::TargetTrajectoryPoint_t trajpoint) {
    reset_velocity_trajectory_state();
    // 几何控制器的核心运动接口。
    // 将完整的轨迹点（位置 / 速度 / 加速度 / 加加速度 / yaw）直接送入核心算法，
    // 由两级控制环（位置 PD + 姿态子控制器）输出 body rate 和归一化推力。
    return publish_trajectory_setpoint(trajpoint, ThrustCommandPolicy::Auto);
}

bool Geometric_Controller::move_point_body(controller_data_types::TargetBodyPoint_t point) {
    reset_velocity_trajectory_state();
    constexpr double kPosEps = 1e-3;
    constexpr double kYawEps = 1e-3;
    bool is_new_body_target = false;

    const double dxy = (point.position_xy - last_point_body_.position_xy).norm();
    if (dxy > kPosEps || std::abs(point.fixed_height - last_point_body_.fixed_height) > kPosEps) {
        is_new_body_target = true;
    }
    if (!is_new_body_target && std::abs(point.yaw - last_point_body_.yaw) > kYawEps) {
        is_new_body_target = true;
    }

    if (is_new_body_target) {
        const double yaw = uav_odometry_.get_yaw();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        const Eigen::Vector2d p_b = point.position_xy;
        Eigen::Vector2d delta_w;
        delta_w.x() = c * p_b.x() - s * p_b.y();
        delta_w.y() = s * p_b.x() + c * p_b.y();

        controller_data_types::TargetPoint_t world_point;
        world_point.position.x() = uav_odometry_.position.x() + delta_w.x();
        world_point.position.y() = uav_odometry_.position.y() + delta_w.y();
        world_point.position.z() = point.fixed_height;
        world_point.yaw = yaw + point.yaw;

        last_point_ = world_point;
        last_point_body_ = point;
    }

    return move_point(last_point_);
}

bool Geometric_Controller::move_velocity_body(controller_data_types::TargetBodyVelocity_t velocity) {
    const double yaw = uav_odometry_.get_yaw();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    Eigen::Vector2d v_w_xy;
    v_w_xy.x() = c * velocity.velocity_xy.x() - s * velocity.velocity_xy.y();
    v_w_xy.y() = s * velocity.velocity_xy.x() + c * velocity.velocity_xy.y();

    const auto trajpoint = update_velocity_trajectory_reference(
        Eigen::Vector3d(v_w_xy.x(), v_w_xy.y(), 0.0),
        yaw + velocity.yaw,
        velocity.yaw_rate,
        velocity.stamp,
        true,
        velocity.fixed_height);
    return publish_trajectory_setpoint(trajpoint, ThrustCommandPolicy::Auto);
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

void Geometric_Controller::reset_velocity_trajectory_state() {
    velocity_traj_state_.active = false;
    velocity_traj_state_.hold_fixed_height = false;
    velocity_traj_state_.fixed_height = 0.0;
    velocity_traj_state_.last_cmd_stamp = ros::Time(0);
    velocity_traj_state_.segment_start_time = ros::Time(0);
    velocity_traj_state_.segment_duration = 0.0;
    velocity_traj_state_.segment_start = controller_data_types::TargetTrajectoryPoint_t{};
    velocity_traj_state_.segment_end = controller_data_types::TargetTrajectoryPoint_t{};
}

controller_data_types::TargetTrajectoryPoint_t
Geometric_Controller::update_velocity_trajectory_reference(
    const Eigen::Vector3d& target_velocity_world,
    double target_yaw,
    double target_yaw_rate,
    const ros::Time& cmd_stamp,
    bool hold_fixed_height,
    double fixed_height) {

    const ros::Time now = ros::Time::now();
    Eigen::Vector3d desired_velocity = target_velocity_world;
    if (hold_fixed_height) {
        desired_velocity.z() = 0.0;
    }
    const double default_dt =
        std::clamp(1.0 / std::max(1.0, geometric_controller_param_.controller_hz), 0.02, 0.05);
    const bool mode_changed = velocity_traj_state_.hold_fixed_height != hold_fixed_height;
    const auto limit_velocity_step =
        [&](const Eigen::Vector3d& current_velocity, const Eigen::Vector3d& target_velocity, double dt) {
            Eigen::Vector3d velocity_delta = target_velocity - current_velocity;

            Eigen::Vector2d velocity_delta_xy = velocity_delta.head<2>();
            const double max_dv_xy = std::max(0.05, velocity_ref_acc_xy_) * dt;
            const double dv_xy_norm = velocity_delta_xy.norm();
            if (dv_xy_norm > max_dv_xy && dv_xy_norm > 1e-6) {
                velocity_delta_xy *= max_dv_xy / dv_xy_norm;
            }
            velocity_delta.head<2>() = velocity_delta_xy;

            const double max_dv_z = std::max(0.05, velocity_ref_acc_z_) * dt;
            velocity_delta.z() = std::clamp(velocity_delta.z(), -max_dv_z, max_dv_z);
            return current_velocity + velocity_delta;
        };

    const auto eval_segment = [&](const ros::Time& query_time) {
        controller_data_types::TargetTrajectoryPoint_t ref = velocity_traj_state_.segment_start;
        ref.yaw = target_yaw;
        ref.yaw_rate = target_yaw_rate;

        const double duration = velocity_traj_state_.segment_duration;
        const double hold_time = std::max(duration, default_dt);
        if (!std::isfinite(duration) || duration <= 1e-4 ||
            velocity_traj_state_.segment_start_time.isZero()) {
            ref.position = velocity_traj_state_.segment_end.position;
            ref.velocity = Eigen::Vector3d::Zero();
            ref.acceleration = Eigen::Vector3d::Zero();
            ref.jerk = Eigen::Vector3d::Zero();
            ref.yaw_rate = 0.0;
            return ref;
        }

        const double elapsed = (query_time - velocity_traj_state_.segment_start_time).toSec();
        if (elapsed <= 0.0) {
            ref.acceleration =
                (velocity_traj_state_.segment_end.velocity -
                 velocity_traj_state_.segment_start.velocity) /
                duration;
            ref.jerk = Eigen::Vector3d::Zero();
            return ref;
        }
        if (elapsed >= duration + hold_time) {
            ref.position = velocity_traj_state_.segment_end.position;
            ref.velocity = Eigen::Vector3d::Zero();
            ref.acceleration = Eigen::Vector3d::Zero();
            ref.jerk = Eigen::Vector3d::Zero();
            ref.yaw_rate = 0.0;
            return ref;
        }
        if (elapsed >= duration) {
            const double coast_time = elapsed - duration;
            ref.position =
                velocity_traj_state_.segment_end.position +
                velocity_traj_state_.segment_end.velocity * coast_time;
            ref.velocity = velocity_traj_state_.segment_end.velocity;
            ref.acceleration = Eigen::Vector3d::Zero();
            ref.jerk = Eigen::Vector3d::Zero();
            if (hold_fixed_height) {
                ref.position.z() = velocity_traj_state_.fixed_height;
                ref.velocity.z() = 0.0;
            }
            return ref;
        }

        const Eigen::Vector3d a =
            (velocity_traj_state_.segment_end.velocity -
             velocity_traj_state_.segment_start.velocity) /
            duration;
        ref.position = velocity_traj_state_.segment_start.position +
                       velocity_traj_state_.segment_start.velocity * elapsed +
                       0.5 * a * elapsed * elapsed;
        ref.velocity = velocity_traj_state_.segment_start.velocity + a * elapsed;
        ref.acceleration = a;
        ref.jerk = Eigen::Vector3d::Zero();
        if (hold_fixed_height) {
            ref.position.z() = velocity_traj_state_.fixed_height;
            ref.velocity.z() = 0.0;
            ref.acceleration.z() = 0.0;
        }
        return ref;
    };

    if (!velocity_traj_state_.active || mode_changed) {
        velocity_traj_state_.active = true;
        velocity_traj_state_.hold_fixed_height = hold_fixed_height;
        velocity_traj_state_.fixed_height =
            hold_fixed_height ? fixed_height : uav_odometry_.position.z();
        velocity_traj_state_.last_cmd_stamp = cmd_stamp.isZero() ? now : cmd_stamp;
        velocity_traj_state_.segment_start_time = now;
        velocity_traj_state_.segment_duration = default_dt;
        velocity_traj_state_.segment_start.position = uav_odometry_.position;
        velocity_traj_state_.segment_start.velocity = uav_odometry_.velocity;
        velocity_traj_state_.segment_start.acceleration = Eigen::Vector3d::Zero();
        velocity_traj_state_.segment_start.jerk = Eigen::Vector3d::Zero();
        velocity_traj_state_.segment_start.yaw = target_yaw;
        velocity_traj_state_.segment_start.yaw_rate = target_yaw_rate;
        velocity_traj_state_.segment_end = velocity_traj_state_.segment_start;
        if (hold_fixed_height) {
            velocity_traj_state_.segment_start.position.z() = velocity_traj_state_.fixed_height;
            velocity_traj_state_.segment_start.velocity.z() = 0.0;
        }
        velocity_traj_state_.segment_end.velocity = limit_velocity_step(
            velocity_traj_state_.segment_start.velocity, desired_velocity, default_dt);
        velocity_traj_state_.segment_end.position =
            velocity_traj_state_.segment_start.position +
            0.5 * (velocity_traj_state_.segment_start.velocity +
                   velocity_traj_state_.segment_end.velocity) *
                default_dt;
        velocity_traj_state_.segment_end.acceleration =
            (velocity_traj_state_.segment_end.velocity -
             velocity_traj_state_.segment_start.velocity) /
            default_dt;
        velocity_traj_state_.segment_end.jerk = Eigen::Vector3d::Zero();
        velocity_traj_state_.segment_end.yaw = target_yaw;
        velocity_traj_state_.segment_end.yaw_rate = target_yaw_rate;
        if (hold_fixed_height) {
            velocity_traj_state_.segment_end.position.z() = velocity_traj_state_.fixed_height;
            velocity_traj_state_.segment_end.velocity.z() = 0.0;
            velocity_traj_state_.segment_end.acceleration.z() = 0.0;
        }
        controller_.reset_vertical_integral();
        desired_state_ = eval_segment(now);
        return desired_state_;
    }

    const bool has_new_cmd =
        !cmd_stamp.isZero() &&
        (velocity_traj_state_.last_cmd_stamp.isZero() || cmd_stamp > velocity_traj_state_.last_cmd_stamp);

    if (has_new_cmd) {
        controller_data_types::TargetTrajectoryPoint_t start_ref = eval_segment(now);
        double dt = (cmd_stamp - velocity_traj_state_.last_cmd_stamp).toSec();
        if (!std::isfinite(dt) || dt <= 1e-4) {
            dt = default_dt;
        }
        dt = std::clamp(dt, 0.01, 0.20);

        velocity_traj_state_.fixed_height =
            hold_fixed_height ? fixed_height : uav_odometry_.position.z();
        velocity_traj_state_.segment_start_time = now;
        velocity_traj_state_.segment_duration = dt;
        velocity_traj_state_.segment_start = start_ref;
        velocity_traj_state_.segment_start.yaw = target_yaw;
        velocity_traj_state_.segment_start.yaw_rate = target_yaw_rate;
        velocity_traj_state_.segment_end = velocity_traj_state_.segment_start;
        velocity_traj_state_.segment_end.velocity =
            limit_velocity_step(velocity_traj_state_.segment_start.velocity, desired_velocity, dt);
        velocity_traj_state_.segment_end.position =
            velocity_traj_state_.segment_start.position +
            0.5 * (velocity_traj_state_.segment_start.velocity +
                   velocity_traj_state_.segment_end.velocity) *
                dt;
        velocity_traj_state_.segment_end.acceleration =
            (velocity_traj_state_.segment_end.velocity -
             velocity_traj_state_.segment_start.velocity) /
            dt;
        velocity_traj_state_.segment_end.jerk = Eigen::Vector3d::Zero();
        velocity_traj_state_.segment_end.yaw = target_yaw;
        velocity_traj_state_.segment_end.yaw_rate = target_yaw_rate;
        if (hold_fixed_height) {
            velocity_traj_state_.segment_start.position.z() = velocity_traj_state_.fixed_height;
            velocity_traj_state_.segment_end.position.z() = velocity_traj_state_.fixed_height;
            velocity_traj_state_.segment_start.velocity.z() = 0.0;
            velocity_traj_state_.segment_end.velocity.z() = 0.0;
            velocity_traj_state_.segment_start.acceleration.z() = 0.0;
            velocity_traj_state_.segment_end.acceleration.z() = 0.0;
        }
        velocity_traj_state_.last_cmd_stamp = cmd_stamp;
    }

    desired_state_ = eval_segment(now);
    return desired_state_;
}

bool Geometric_Controller::publish_trajectory_setpoint(
    const controller_data_types::TargetTrajectoryPoint_t& trajpoint,
    ThrustCommandPolicy thrust_policy) {
    desired_state_ = trajpoint;

    auto output = controller_.calculateControl(trajpoint, uav_odometry_, thrust_policy);

    control_common::Mavros_SetpointAttitude setpoint;
    if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                        control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                        control_common::Mavros_SetpointAttitude::IgnoreYawRate;
        setpoint.orientation = output.orientation;
        setpoint.body_rate = Eigen::Vector3d::Zero();
    } else {
        setpoint.mask = control_common::Mavros_SetpointAttitude::IgnoreAttitude;
        setpoint.body_rate = output.bodyrates;
    }
    setpoint.thrust = output.thrust;
    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;
    return true;
}

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

    const YAML::Node hover_thrust_node =
        basic_param["hover_thrust_precent"] ? basic_param["hover_thrust_precent"]
                                             : basic_param["hover_thrust_percent"];
    const bool has_basic_hover_thrust = static_cast<bool>(hover_thrust_node);
    if (has_basic_hover_thrust) {
        geometric_controller_param_.hover_thrust_init =
            std::clamp(hover_thrust_node.as<double>(), 0.05, 0.80);
    }

    const YAML::Node takeoff_land_param = root["takeoff_land_param"];
    if (takeoff_land_param && takeoff_land_param.IsMap()) {
        bool has_takeoff_ramp_time = false;
        if (takeoff_land_param["motors_speedup_time"]) {
            motors_speedup_time_ = std::max(0.0, takeoff_land_param["motors_speedup_time"].as<double>());
        }
        if (takeoff_land_param["takeoff_idle_thrust"]) {
            takeoff_idle_thrust_ =
                std::clamp(takeoff_land_param["takeoff_idle_thrust"].as<double>(), 0.0, 0.95);
        }
        if (takeoff_land_param["takeoff_idle_hold_time"]) {
            takeoff_idle_hold_time_ =
                std::max(0.0, takeoff_land_param["takeoff_idle_hold_time"].as<double>());
        }
        if (takeoff_land_param["takeoff_ramp_time"]) {
            takeoff_ramp_time_ =
                std::max(1e-3, takeoff_land_param["takeoff_ramp_time"].as<double>());
            has_takeoff_ramp_time = true;
        }
        if (takeoff_land_param["takeoff_thrust_filter_tau"]) {
            takeoff_thrust_filter_tau_ =
                std::max(1e-3, takeoff_land_param["takeoff_thrust_filter_tau"].as<double>());
        }
        if (takeoff_land_param["takeoff_thrust_handoff_time"]) {
            takeoff_thrust_handoff_time_ =
                std::max(0.0, takeoff_land_param["takeoff_thrust_handoff_time"].as<double>());
        }
        if (takeoff_land_param["land_thrust_filter_tau"]) {
            land_thrust_filter_tau_ =
                std::max(1e-3, takeoff_land_param["land_thrust_filter_tau"].as<double>());
        }
        if (takeoff_land_param["land_near_ground_thrust_max"]) {
            land_near_ground_thrust_max_ =
                std::clamp(takeoff_land_param["land_near_ground_thrust_max"].as<double>(), 0.02, 0.95);
        }
        if (takeoff_land_param["land_touchdown_thrust_max"]) {
            land_touchdown_thrust_max_ =
                std::clamp(takeoff_land_param["land_touchdown_thrust_max"].as<double>(), 0.02, 0.95);
        }
        if (!has_takeoff_ramp_time) {
            takeoff_ramp_time_ = std::max(1e-3, motors_speedup_time_);
        }
        land_touchdown_thrust_max_ =
            std::min(land_touchdown_thrust_max_, land_near_ground_thrust_max_);
    }

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

    arrival_judge_stabile_time_s_ =
        arrival_judge_param["judge_stabile_time_s"].as<double>();
    arrival_pos_stabile_err_m_ = arrival_judge_param["pos_stabile_err_m"].as<double>();
    arrival_vel_stabile_err_mps_ =
        arrival_judge_param["vel_stabile_err_mps"].as<double>();

    if (arrival_judge_stabile_time_s_ <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.judge_stabile_time_s' must > 0");
    }
    if (arrival_pos_stabile_err_m_ <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.pos_stabile_err_m' must > 0");
    }
    if (arrival_vel_stabile_err_mps_ <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.vel_stabile_err_mps' must > 0");
    }

    // ──── sunray_controller_param ─────────────────────────────────────────────
    const YAML::Node geo_param = root["sunray_controller_param"];
    if (!geo_param || !geo_param.IsMap()) {
        throw std::runtime_error("yaml '" + config_yamlfile_path_ +
                                 "' is missing a valid 'sunray_controller_param' map");
    }

    if (!geo_param["control_type"]) {
        throw std::runtime_error("missing param 'control_type'");
    }
    const int control_type = geo_param["control_type"].as<int>();
    if (control_type != 0 && control_type != 1) {
        throw std::runtime_error(
            "param 'control_type' must be 0 (attitude+thrust) or 1 (bodyrate+thrust) for "
            "Geometric_Controller");
    }
    attitude_command_mode_ = (control_type == 0) ? AttitudeCommandMode::Attitude
                                                 : AttitudeCommandMode::BodyRate;

    if (!geo_param["attitude_type"]) {
        throw std::runtime_error("missing param 'attitude_type'");
    }
    geometric_controller_param_.attitude_type = geo_param["attitude_type"].as<int>();
    if (geometric_controller_param_.attitude_type != 0 &&
        geometric_controller_param_.attitude_type != 1) {
        throw std::runtime_error("param 'attitude_type' must be 0 (quaternion) or 1 (SO3)");
    }

    if (!geo_param["attitude_tau"]) {
        throw std::runtime_error("missing param 'attitude_tau'");
    }
    geometric_controller_param_.attitude_tau = geo_param["attitude_tau"].as<double>();

    // 配置与运行频率保持一致，供 PID 积分/微分计算 dt 使用
    if (basic_param["controller_update_frequency"]) {
        geometric_controller_param_.controller_hz =
            basic_param["controller_update_frequency"].as<double>();
    }

    const auto load_vec3 = [&](const char* key, Eigen::Vector3d& out) {
        if (!geo_param[key] || !geo_param[key].IsSequence() || geo_param[key].size() != 3) {
            throw std::runtime_error(std::string("missing or invalid param '") + key +
                                     "' (expected sequence of 3 values)");
        }
        out.x() = geo_param[key][0].as<double>();
        out.y() = geo_param[key][1].as<double>();
        out.z() = geo_param[key][2].as<double>();
    };

    load_vec3("pos_kp", geometric_controller_param_.pos_kp);
    load_vec3("pos_ki", geometric_controller_param_.pos_ki);
    load_vec3("pos_kd", geometric_controller_param_.pos_kd);
    load_vec3("vel_kp", geometric_controller_param_.vel_kp);
    load_vec3("vel_ki", geometric_controller_param_.vel_ki);
    load_vec3("vel_kd", geometric_controller_param_.vel_kd);

    // 兼容：允许在新配置中按需覆盖以下进阶参数
    if (geo_param["max_acc"]) {
        geometric_controller_param_.max_acc = geo_param["max_acc"].as<double>();
    }
    if (geo_param["velocity_ref_acc_xy"]) {
        velocity_ref_acc_xy_ = std::max(0.05, geo_param["velocity_ref_acc_xy"].as<double>());
    }
    if (geo_param["velocity_ref_acc_z"]) {
        velocity_ref_acc_z_ = std::max(0.05, geo_param["velocity_ref_acc_z"].as<double>());
    }
    if (!has_basic_hover_thrust && geo_param["hover_thrust_init"]) {
        geometric_controller_param_.hover_thrust_init =
            std::clamp(geo_param["hover_thrust_init"].as<double>(), 0.05, 0.80);
    }
    if (geo_param["hover_thrust_estimator_type"]) {
        geometric_controller_param_.hover_thrust_estimator_type =
            geo_param["hover_thrust_estimator_type"].as<int>();
        if (geometric_controller_param_.hover_thrust_estimator_type < 0 ||
            geometric_controller_param_.hover_thrust_estimator_type > 2) {
            throw std::runtime_error("param 'hover_thrust_estimator_type' must be 0, 1 or 2");
        }
    }
    if (geo_param["imu_acc_is_specific_force"]) {
        imu_acc_is_specific_force_ = geo_param["imu_acc_is_specific_force"].as<bool>();
    }
    if (geo_param["int_max_pos"] && geo_param["int_max_pos"].IsSequence() &&
        geo_param["int_max_pos"].size() == 3) {
        geometric_controller_param_.int_max_pos.x() = geo_param["int_max_pos"][0].as<double>();
        geometric_controller_param_.int_max_pos.y() = geo_param["int_max_pos"][1].as<double>();
        geometric_controller_param_.int_max_pos.z() = geo_param["int_max_pos"][2].as<double>();
    }
    if (geo_param["int_max_vel"] && geo_param["int_max_vel"].IsSequence() &&
        geo_param["int_max_vel"].size() == 3) {
        geometric_controller_param_.int_max_vel.x() = geo_param["int_max_vel"][0].as<double>();
        geometric_controller_param_.int_max_vel.y() = geo_param["int_max_vel"][1].as<double>();
        geometric_controller_param_.int_max_vel.z() = geo_param["int_max_vel"][2].as<double>();
    }
    if (geo_param["drag"] && geo_param["drag"].IsSequence() && geo_param["drag"].size() == 3) {
        geometric_controller_param_.D.x() = geo_param["drag"][0].as<double>();
        geometric_controller_param_.D.y() = geo_param["drag"][1].as<double>();
        geometric_controller_param_.D.z() = geo_param["drag"][2].as<double>();
    }
}

bool Geometric_Controller::has_valid_imu_data()  {
    control_common::Mavros_IMU imu_data = mavros_helper_.get_imu_data();
    if (imu_data.stamp.isZero()) {
        return false;
    }
    const bool fresh = (ros::Time::now() - imu_data.stamp).toSec() < 0.15;
    if (!fresh) {
        return false;
    }
    return std::isfinite(imu_data.accelection.x()) && std::isfinite(imu_data.accelection.y()) &&
           std::isfinite(imu_data.accelection.z());
}

Eigen::Vector3d Geometric_Controller::get_world_acc_from_imu()  {
    const control_common::Mavros_IMU imu_data = mavros_helper_.get_imu_data();
    const Eigen::Matrix3d Rwb = uav_odometry_.orientation.toRotationMatrix();
    const Eigen::Vector3d acc_body = imu_data.accelection;
    if (imu_acc_is_specific_force_) {
        // specific force f = a - g -> a = R*f + g_w, g_w=(0,0,-g)
        return Rwb * acc_body + Eigen::Vector3d(0.0, 0.0, -geometric_controller_param_.gravity);
    }
    // 已去重力的 linear acceleration，直接旋转到世界系
    return Rwb * acc_body;
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
