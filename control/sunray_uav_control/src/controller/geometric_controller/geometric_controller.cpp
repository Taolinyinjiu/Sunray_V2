#include "controller/geometric_controller.hpp"
#include "eigen_helper.hpp"
#include "utils/body_frame_reference_helper.hpp"
#include "utils/orientation_utils.hpp"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// 构造函数
// ─────────────────────────────────────────────────────────────────────────────
Geometric_Controller::Geometric_Controller(ros::NodeHandle& nh) : nh_(nh) {
    std::string node_name = ros::this_node::getName();
    ros::NodeHandle private_nh_("~");

    if (private_nh_.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        if (config_yamlfile_path_.empty()) {
            throw std::runtime_error("yaml_path cannot be empty");
        }
    } else {
        throw std::runtime_error("missing param " + node_name + "/config_yamlfile_path");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 生命周期
// ─────────────────────────────────────────────────────────────────────────────
/*  init()函数负责初始化
    1. 加载yaml文件中的参数并校验参数是否正确，这一步出错会使用抛出异常的方式结束ros节点
    2. 初始化controller核心算法并注入需要的参数
    3. 初始化mavros_helper用于读取px4数据与发布控制命令
    4. 根据参数决定是否初始化定时器，用于融合里程计数据到px4
    5. 初始化发布PX4State定时器
*/
bool Geometric_Controller::init() {
    // 加载参数并校验
    load_and_validate_config_or_throw();
    // 将参数加载到核心算法
    controller_.load_param(geometric_controller_param_);

    // 初始化 mavros_helper
    mavros_helper_.init();

    // 若需要融合外部里程计，注册发布定时器
    if (fuse_odom_type != 0) {
        mavros_helper_.set_vision_fuse_type(fuse_odom_type);
        pub_vision_pose_timer_ = nh_.createTimer(ros::Duration(1.0 / fuse_odom_frequency),
                                                 &Geometric_Controller::pub_vision_fuse_timer_cb,
                                                 this);
    }

    // 启动 PX4State 发布定时器
    pub_px4_state_timer_ = nh_.createTimer(ros::Duration(1.0 / pub_px4_state_freq_),
                                           &Geometric_Controller::pub_px4_state_timer_cb,
                                           this);
    return true;
}

/*  is_ready()函数用于状态机确认控制器当前是否完成飞行前的准备工作，包含两个方面
    1. 里程计数据是否稳定
    2. px4飞控数据流是否稳定
    3. MAVROS/PX4 数据流是否满足控制器运行条件
*/
bool Geometric_Controller::is_ready() {
    control_common::Mavros_State mavros_state = mavros_helper_.get_state();
    if (mavros_state.connected != true) {
        return false;
    }

    bool mavros_ready = mavros_helper_.is_ready();
    if (!mavros_ready) {
        return false;
    }

    controller_ready_.store(true, std::memory_order_relaxed);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 状态注入
// ─────────────────────────────────────────────────────────────────────────────
void Geometric_Controller::set_current_odom(const control_common::UAVStateEstimate& odom) {
    uav_odometry_ = odom;
    const bool has_valid_timestamp = !odom.timestamp.isZero();
    const bool is_new_odom = has_valid_timestamp && odom.timestamp != last_odom_timestamp_;
    has_uav_odometry_.store(has_valid_timestamp, std::memory_order_relaxed);
    if (is_new_odom) {
        last_odom_timestamp_ = odom.timestamp;
        can_fuse_.store(true, std::memory_order_relaxed);
    }
    has_imu_.store(has_valid_imu_data(), std::memory_order_relaxed);
}

// ─────────────────────────────────────────────────────────────────────────────
// 运动相关接口
// ─────────────────────────────────────────────────────────────────────────────

// 任务流程描述为 takoff ： [mode]position -> [mode]offboard setpoint 流持续发送
//              land  :  [mode]offboard -> [mode]position setpoint 流停止发送
// 因此我们提供一个set_postion_mode函数用于设置为position模式
void Geometric_Controller::set_position_mode() {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    yaw_curve_.clear();
    control_common::Mavros_State state = mavros_helper_.get_state();
    if (state.flight_mode != control_common::FlightMode::Posctl) {
        mavros_helper_.set_px4_mode(control_common::FlightMode::Posctl);
    }
}

// 起飞阶段推力RC滤波器，将突变推力切换为斜坡递增推力
void Geometric_Controller::seed_rc_thrust_filter(takeoff_land::RCThrustFilterState& state,
                                                 double thrust,
                                                 const ros::Time& now) {
    state.initialized = true;
    state.thrust = std::clamp(thrust, 0.0, 0.95);
    state.last_update = now;
}

double Geometric_Controller::update_rc_thrust_filter(takeoff_land::RCThrustFilterState& state,
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

void Geometric_Controller::maybe_rebase_takeoff_curve_start() {
    if (takeoff_state_.curve_started || motion_curve_owner_ != MotionCurveOwner::Takeoff ||
        !motion_curve_.is_ready()) {
        return;
    }

    const Eigen::Vector3d current_pos = uav_odometry_.position;
    const Eigen::Vector3d current_vel = uav_odometry_.velocity;

    motion_curve_.set_start_trajpoint(current_pos, current_vel);
    takeoff_state_.curve_started = true;
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
    (void)reason;
    (void)dp;
    (void)dyaw;
    (void)pos_err;
}

void Geometric_Controller::clear_motion_curve() {
    motion_curve_.clear();
    motion_curve_owner_ = MotionCurveOwner::None;
}

void Geometric_Controller::begin_motion_curve(MotionCurveOwner owner) {
    motion_curve_.clear();
    motion_curve_owner_ = owner;
}

void Geometric_Controller::reset_point_motion_context() {
    if (motion_curve_owner_ == MotionCurveOwner::MovePoint) {
        clear_motion_curve();
    }
    point_arrival_state_ = arrival_helper::State{};
    point_complete_.store(false, std::memory_order_relaxed);
    point_target_initialized_ = false;
    body_point_target_initialized_ = false;
    last_point_ = controller_data_types::TargetPoint_t{};
    last_point_body_ = controller_data_types::TargetBodyPoint_t{};
}

double Geometric_Controller::update_limited_yaw_target(double target_yaw, const ros::Time& now) {
    constexpr double kYawTargetEps = 1e-4;
    const double current_yaw = uav_odometry_.get_yaw();
    const double current_yaw_rate = uav_odometry_.bodyrate.z();
    const double normalized_target = normalize_angle_rad(target_yaw);

    if (!yaw_curve_.is_ready() || !yaw_curve_.matches_target(normalized_target, kYawTargetEps)) {
        yaw_curve_.set_start_yawpoint(current_yaw, current_yaw_rate);
        yaw_curve_.set_end_yawpoint(normalized_target, 0.0);
        yaw_curve_.set_curve_avgvel(max_yaw_rate_rad_s_);
    }

    const curve::YawCurveState current_ref = yaw_curve_.get_result(now);
    return current_ref.valid ? current_ref.yaw : current_yaw;
}

double Geometric_Controller::integrate_limited_yaw_rate(double yaw_rate_cmd, const ros::Time& now) {
    return reference_limit_helper::integrate_yaw_rate_command(
        yaw_reference_state_, yaw_rate_cmd, uav_odometry_.get_yaw(), max_yaw_rate_rad_s_, now);
}

void Geometric_Controller::warn_if_trajectory_exceeds_limits(
    const controller_data_types::TargetTrajectoryPoint_t& trajpoint) const {
    if (!reference_limit_helper::trajectory_reference_exceeds_limits(
            trajpoint.velocity, trajpoint.yaw_rate, max_velocity_, max_yaw_rate_rad_s_)) {
        return;
    }

    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[Geometric_Controller] trajectory reference exceeds velocity_param limits: vel="
                                   << trajpoint.velocity.transpose() << " max="
                                   << max_velocity_.transpose() << " yaw_rate="
                                   << trajpoint.yaw_rate << " max_yaw_rate="
                                   << max_yaw_rate_rad_s_);
}

bool Geometric_Controller::move_point_impl(controller_data_types::TargetPoint_t point,
                                           bool preserve_body_point_context) {
    constexpr double kNewTargetPosEps = 1e-3;
    if (!preserve_body_point_context) {
        body_point_target_initialized_ = false;
    }

    const bool is_new_target =
        !point_target_initialized_ ||
        (point.position - last_point_.position).norm() > kNewTargetPosEps ||
        std::abs(normalize_angle_rad(point.yaw - last_point_.yaw)) > 1e-3;
    const bool need_rebuild_curve =
        is_new_target || motion_curve_owner_ != MotionCurveOwner::MovePoint;
    if (need_rebuild_curve) {
        point_arrival_state_ = arrival_helper::State{};
        point_complete_.store(false, std::memory_order_relaxed);
        controller_.reset_vertical_integral();
        last_point_ = point;
        point_target_initialized_ = true;
        begin_motion_curve(MotionCurveOwner::MovePoint);
        motion_curve_.set_start_trajpoint(uav_odometry_.position, uav_odometry_.velocity);
        motion_curve_.set_end_trajpoint(point.position, Eigen::Vector3d::Zero());
        motion_curve_.set_curve_maxvel(reference_limit_helper::compute_point_curve_maxvel(
            uav_odometry_.position, point.position, max_velocity_));
    }

    const ros::Time now = ros::Time::now();
    const curve::QuinticCurveState curve_result = motion_curve_.get_result();
    controller_data_types::TargetTrajectoryPoint_t des_state;
    // Keep geometric point mode close to the historical "position hold" semantics:
    // only smooth the position reference, and avoid quintic velocity/acceleration
    // feedforward that can make the point interface much more aggressive.
    des_state.position = curve_result.valid ? curve_result.position : point.position;
    des_state.velocity = Eigen::Vector3d::Zero();
    des_state.acceleration = Eigen::Vector3d::Zero();
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = update_limited_yaw_target(point.yaw, now);
    des_state.yaw_rate = 0.0;

    publish_trajectory_setpoint(des_state, ThrustCommandPolicy::UseEstimatedAnchor);

    if (point_complete_.load(std::memory_order_relaxed)) {
        return true;
    }

    const double pos_err = (uav_odometry_.position - last_point_.position).norm();
    const double vel_err = uav_odometry_.velocity.norm();
    const double yaw_err =
        std::abs(normalize_angle_rad(last_point_.yaw - uav_odometry_.get_yaw()));
    const double yaw_rate_err = std::abs(uav_odometry_.bodyrate.z());
    if (!arrival_helper::update_pose_and_check(point_arrival_state_,
                                               arrival_judge_config_,
                                               pos_err,
                                               vel_err,
                                               yaw_err,
                                               yaw_rate_err,
                                               now)) {
        point_complete_.store(false, std::memory_order_relaxed);
        return false;
    }

    point_complete_.store(true, std::memory_order_relaxed);
    update_hover_reference(last_point_.position, last_point_.yaw, "move_point_arrive");
    return true;
}

void Geometric_Controller::reset_stage_thrust_filters() {
    takeoff_state_.thrust_filter = takeoff_land::RCThrustFilterState{};
    landing_state_.thrust_filter = takeoff_land::RCThrustFilterState{};
}

bool Geometric_Controller::takeoff(double relative_takeoff_height, double max_takeoff_velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    yaw_curve_.clear();
    // 如果本轮之前已经降落，清除降落标志
    if (land_complete_.load(std::memory_order_relaxed)) {
        land_complete_.store(false, std::memory_order_relaxed);
    }
    // 控制器未就绪则直接返回
    if (!controller_ready_.load(std::memory_order_relaxed)) {
        return false;
    }
    // 起飞已完成时退化为悬停
    if (takeoff_complete_.load(std::memory_order_relaxed)) {
        return hover();
    }

    ros::Time now = ros::Time::now();

    // ── 首次进入起飞流程：初始化五次项曲线与 yaw 锁存 ───────────────────────
    if (motion_curve_owner_ != MotionCurveOwner::Takeoff) {
        takeoff_arrival_state_ = arrival_helper::State{};
        // 起点：当前位置，速度为零
        begin_motion_curve(MotionCurveOwner::Takeoff);
        motion_curve_.set_start_trajpoint(uav_odometry_.position, Eigen::Vector3d::Zero());
        // 终点：当前位置 + 相对起飞高度，速度为零
        motion_curve_.set_end_trajpoint(uav_odometry_.position +
                                           Eigen::Vector3d(0.0, 0.0, relative_takeoff_height),
                                       Eigen::Vector3d::Zero());
        // 由最大起飞速度反推运动时间，使曲线平滑且有速度上限
        motion_curve_.set_curve_maxvel(max_takeoff_velocity);
        // 锁存起飞时刻的 yaw 角和地面高度，这里从uav_odometry_获取yaw角，可以不向px4融合里程计
        takeoff_state_.yaw = uav_odometry_.get_yaw();
        ground_height_ref_ = uav_odometry_.position.z();
        // 将起飞 yaw 同步到核心算法缓存，防止后续 move_point 未显式设置 yaw 时默认归零
        controller_.set_initial_yaw(takeoff_state_.yaw);
        takeoff_state_.curve_started = false;
        // 注意：quint_curve_ 并不在此处记录开始时间，
        // 而是以第一次调用 get_result() 时的时刻作为曲线起点
    }

    control_common::Mavros_State px4_state = mavros_helper_.get_state();

    // ── 阶段 1：切换 Offboard 模式 ──────────────────────────────────────────
    if (px4_state.flight_mode != control_common::FlightMode::Offboard) {
        reset_stage_thrust_filters();
        takeoff_state_.curve_started = false;
        takeoff_state_.arm_time = ros::Time(0);
        takeoff_arrival_state_ = arrival_helper::State{};
        // 切换前持续发送零指令以满足 Offboard 2 Hz 最低频率要求
        control_common::Mavros_SetpointAttitude setpoint_cmd;
        if (attitude_command_mode_ == AttitudeCommandMode::Attitude) {
            setpoint_cmd.mask = control_common::Mavros_SetpointAttitude::IgnoreRollRate |
                                control_common::Mavros_SetpointAttitude::IgnorePitchRate |
                                control_common::Mavros_SetpointAttitude::IgnoreYawRate;
            setpoint_cmd.orientation = has_uav_odometry_.load(std::memory_order_relaxed)
                                           ? uav_odometry_.orientation
                                           : Eigen::Quaterniond::Identity();
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
        takeoff_state_.curve_started = false;
        takeoff_state_.arm_time = ros::Time(0);
        takeoff_arrival_state_ = arrival_helper::State{};
        mavros_helper_.set_arm(true);
        return false;
    }

    // ── 阶段 3：起飞前开环推力缓升 ───────────────────────────────────────────
    // 在无人机离地前保持地面高度，推力从 idle 平滑拉向 hover_thrust_init+boost。
    // 这一段不直接使用控制器推力输出，避免悬停线性化推力在地面阶段过早注入。
    if (takeoff_state_.arm_time == ros::Time(0)) {
        takeoff_state_.arm_time = now;
        seed_rc_thrust_filter(takeoff_state_.thrust_filter, takeoff_tuning_.idle_thrust, now);
    }
    const double takeoff_elapsed = (now - takeoff_state_.arm_time).toSec();
    const double ramp_time = std::max(1e-3, takeoff_tuning_.ramp_time);
    const double rel_height = uav_odometry_.position.z() - ground_height_ref_;
    const bool airborne = rel_height > takeoff_tuning_.liftoff_detect_height_m ||
                          uav_odometry_.velocity.z() > takeoff_tuning_.liftoff_detect_vz_mps;
    if (!airborne) {
        controller_data_types::TargetTrajectoryPoint_t des_state;
        des_state.position = Eigen::Vector3d(motion_curve_.get_start_position().x(),
                                             motion_curve_.get_start_position().y(),
                                             ground_height_ref_);
        des_state.velocity = Eigen::Vector3d::Zero();
        des_state.acceleration = Eigen::Vector3d::Zero();
        des_state.jerk = Eigen::Vector3d::Zero();
        des_state.yaw = takeoff_state_.yaw;
        des_state.yaw_rate = 0.0;

        // 起飞暖机阶段强制使用线性推力模型，同时不更新推力估计器。
        auto output = controller_.calculateControl(
            des_state, uav_odometry_, ThrustCommandPolicy::UseFixedAnchor);
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
        const double openloop_takeoff_thrust =
            std::clamp(geometric_controller_param_.hover_thrust_init + takeoff_tuning_.openloop_boost,
                       takeoff_tuning_.idle_thrust,
                       0.95);
        const double ramp_ratio = std::clamp(takeoff_elapsed / ramp_time, 0.0, 1.0);
        const double target_thrust =
            takeoff_tuning_.idle_thrust +
            ramp_ratio * (openloop_takeoff_thrust - takeoff_tuning_.idle_thrust);
        setpoint.thrust = update_rc_thrust_filter(
            takeoff_state_.thrust_filter, target_thrust, takeoff_tuning_.thrust_tau, now);
        mavros_helper_.pub_attitude_setpoint(setpoint);
        last_setpoint_ = setpoint;
        return false;
    }

    // 已离地，后续直接使用几何控制器输出，不再经过起飞阶段 RC 推力滤波。
    takeoff_state_.thrust_filter = takeoff_land::RCThrustFilterState{};

    // ── 阶段 4：五次项曲线平滑爬升 ───────────────────────────────────────
    // get_result() 在首次调用时开始计时，输出连续的位置/速度/加速度轨迹
    // 曲线结束后输出值保持在终点，控制器自然收敛悬停
    {
        maybe_rebase_takeoff_curve_start();
        curve::QuinticCurveState curve_result = motion_curve_.get_result();

        controller_data_types::TargetTrajectoryPoint_t des_state;
        des_state.position = curve_result.position;
        des_state.velocity = curve_result.velocity;
        des_state.acceleration = curve_result.acceleration;
        des_state.jerk = Eigen::Vector3d::Zero();
        des_state.yaw = takeoff_state_.yaw;
        des_state.yaw_rate = 0.0;

        // 起飞爬升阶段强制使用线性推力模型，同时不更新推力估计器。
        auto output = controller_.calculateControl(
            des_state, uav_odometry_, ThrustCommandPolicy::UseFixedAnchor);
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

        const double pos_err = (uav_odometry_.position - motion_curve_.get_end_position()).norm();
        const double vel_err = uav_odometry_.velocity.norm();
        if (!arrival_helper::update_and_check(
                takeoff_arrival_state_, arrival_judge_config_, pos_err, vel_err, now)) {
            return false;
        }

        takeoff_complete_.store(true, std::memory_order_relaxed);
        if (last_setpoint_.thrust > 0.05) {
            controller_.seed_hover_thrust_estimator(last_setpoint_.thrust);
        }
        update_hover_reference(
            motion_curve_.get_end_position(), takeoff_state_.yaw, "takeoff_complete");
        start_checkout_offboard_time_ = ros::Time(0);
        last_checkout_offboard_time_ = ros::Time(0);
        takeoff_arrival_state_ = arrival_helper::State{};
        takeoff_state_.reset();
        reset_stage_thrust_filters();
        controller_.reset_integral();
        clear_motion_curve();
        return true;
    }
}

bool Geometric_Controller::land(bool land_type, double max_land_velocity) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    yaw_curve_.clear();
    takeoff_state_.thrust_filter = takeoff_land::RCThrustFilterState{};
    // 进入降落流程时重置积分，防止下降阶段因残留积分产生额外推力
    if (landing_state_.start_time == ros::Time(0)) {
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

    if (land_complete_.load(std::memory_order_relaxed)) {
        return true;
    }

    // 首次进入降落流程：锁定当前位置、yaw 和最大降落速度
    if (landing_state_.start_time == ros::Time(0)) {
        landing_state_.reset();
        landing_state_.start_time = now;
        landing_state_.point = uav_odometry_.position;
        landing_state_.yaw = mavros_helper_.get_yaw_rad();
        landing_state_.max_velocity = max_land_velocity;
    }

    const double height_above_ground =
        std::max(0.0, uav_odometry_.position.z() - ground_height_ref_);
    const double slow_height =
        std::max(landing_tuning_.slow_height_m, landing_tuning_.near_ground_height_m + 0.05);
    const double far_descent_speed =
        std::max(landing_tuning_.touchdown_velocity, std::max(0.05, landing_state_.max_velocity));
    const double near_descent_speed =
        std::clamp(landing_tuning_.near_ground_velocity,
                   landing_tuning_.touchdown_velocity,
                   far_descent_speed);
    const double touchdown_descent_speed =
        std::clamp(landing_tuning_.touchdown_velocity, 0.01, near_descent_speed);

    double desired_descent_speed = far_descent_speed;
    if (height_above_ground <= landing_tuning_.near_ground_height_m) {
        const double blend =
            std::clamp(height_above_ground / std::max(landing_tuning_.near_ground_height_m, 1e-3),
                       0.0,
                       1.0);
        desired_descent_speed =
            touchdown_descent_speed + blend * (near_descent_speed - touchdown_descent_speed);
    } else if (height_above_ground <= slow_height) {
        const double blend =
            std::clamp((height_above_ground - landing_tuning_.near_ground_height_m) /
                           std::max(slow_height - landing_tuning_.near_ground_height_m, 1e-3),
                       0.0,
                       1.0);
        desired_descent_speed =
            near_descent_speed + blend * (far_descent_speed - near_descent_speed);
    }

    const control_common::LandedState px4_land_state = mavros_helper_.get_state().landed_state;
    const bool px4_landed = (px4_land_state == control_common::LandedState::OnGround);
    const bool slow_release_candidate =
        height_above_ground <= landing_tuning_.touchdown_detect_height_m;
    if (!landing_state_.slow_release_started && slow_release_candidate) {
        landing_state_.slow_release_started = true;
        landing_state_.slow_release_start_time = now;
        controller_.reset_vertical_integral();
        const double seed_thrust = (last_setpoint_.thrust > 0.0)
                                       ? last_setpoint_.thrust
                                       : geometric_controller_param_.hover_thrust_init;
        seed_rc_thrust_filter(landing_state_.thrust_filter, seed_thrust, now);
    }

    if (landing_state_.slow_release_started && !landing_state_.fast_release_started) {
        if (height_above_ground <= landing_tuning_.fast_release_height_m) {
            if (landing_state_.fast_release_reference_time == ros::Time(0)) {
                landing_state_.fast_release_reference_time = now;
                landing_state_.fast_release_reference_position = uav_odometry_.position;
            } else {
                const double odom_delta =
                    (uav_odometry_.position - landing_state_.fast_release_reference_position).norm();
                const double observe_dt =
                    (now - landing_state_.fast_release_reference_time).toSec();
                if (observe_dt >= landing_tuning_.odom_settle_window_s) {
                    if (odom_delta <= landing_tuning_.odom_settle_threshold_m) {
                        landing_state_.fast_release_started = true;
                        landing_state_.fast_release_start_time = now;
                        controller_.reset_vertical_integral();
                    } else {
                        landing_state_.fast_release_reference_time = now;
                        landing_state_.fast_release_reference_position = uav_odometry_.position;
                    }
                }
            }
        } else {
            landing_state_.fast_release_reference_time = ros::Time(0);
            landing_state_.fast_release_reference_position = Eigen::Vector3d::Zero();
        }
    }

    const double lookahead_time = std::max(0.05, landing_tuning_.position_lookahead_time);
    double target_z = std::max(uav_odometry_.position.z() - desired_descent_speed * lookahead_time,
                               ground_height_ref_);
    double target_vz = -desired_descent_speed;
    if (landing_state_.slow_release_started) {
        target_z = std::max(uav_odometry_.position.z(), ground_height_ref_);
        target_vz = 0.0;
    }

    controller_data_types::TargetTrajectoryPoint_t des_state;
    des_state.position =
        Eigen::Vector3d(landing_state_.point.x(), landing_state_.point.y(), target_z);
    des_state.velocity = Eigen::Vector3d(0.0, 0.0, target_vz);
    des_state.acceleration = Eigen::Vector3d::Zero();
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = landing_state_.yaw;
    des_state.yaw_rate = 0.0;

    // 降落阶段强制使用线性推力模型，同时不更新推力估计器。
    auto output =
        controller_.calculateControl(des_state, uav_odometry_, ThrustCommandPolicy::UseFixedAnchor);
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

    if (!landing_state_.slow_release_started) {
        setpoint.thrust = output.thrust;
    } else {
        const double soft_touchdown_thrust =
            std::clamp(landing_tuning_.soft_touchdown_thrust_ratio *
                           geometric_controller_param_.hover_thrust_init,
                       0.20,
                       0.36);
        const double thrust_target = landing_state_.fast_release_started
                                         ? landing_tuning_.shutdown_thrust
                                         : soft_touchdown_thrust;
        const double thrust_tau = landing_state_.fast_release_started
                                      ? landing_tuning_.fast_release_tau
                                      : landing_tuning_.slow_release_tau;
        setpoint.thrust = update_rc_thrust_filter(
            landing_state_.thrust_filter, thrust_target, thrust_tau, now);
    }

    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;

    if (landing_state_.fast_release_started) {
        const double fast_release_elapsed = (now - landing_state_.fast_release_start_time).toSec();
        if (px4_landed || fast_release_elapsed > 1.0) {
            mavros_helper_.set_arm(false);
            if (mavros_helper_.get_state().armed == false) {
                // 上锁成功，清理降落上下文
                land_complete_.store(true, std::memory_order_relaxed);
                takeoff_complete_.store(false, std::memory_order_relaxed);
                landing_state_.reset();
                reset_stage_thrust_filters();
                return true;
            }
        }
    }
    return false;
}

bool Geometric_Controller::set_hover_point(control_common::UAVStateEstimate current_odom) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    yaw_curve_.clear();
    update_hover_reference(current_odom.position, current_odom.get_yaw(), "set_hover_point");
    return true;
}

bool Geometric_Controller::hover() {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    yaw_curve_.clear();
    controller_data_types::TargetTrajectoryPoint_t des_state;
    des_state.position = hover_point;
    des_state.velocity = Eigen::Vector3d::Zero();
    des_state.acceleration = Eigen::Vector3d::Zero();
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = hover_yaw_;
    des_state.yaw_rate = 0.0;

    auto output =
        controller_.calculateControl(des_state, uav_odometry_, ThrustCommandPolicy::UseEstimatedAnchor);

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
    if (has_valid_imu_data()) {
        thrust_estimator::Input_t estimator_input;
        estimator_input.stamp = mavros_helper_.get_imu_data().stamp;
        estimator_input.attitude = uav_odometry_.orientation;
        estimator_input.velocity_w = uav_odometry_.velocity;
        estimator_input.acceleration_w = get_world_acc_from_imu();
        estimator_input.thrust_cmd = setpoint.thrust;
        controller_.feed_thrust_estimator(estimator_input);
    }
    return true;
}

bool Geometric_Controller::emergency_kill() {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    yaw_curve_.clear();
    return mavros_helper_.emergency_kill();
}

bool Geometric_Controller::move_point(controller_data_types::TargetPoint_t point) {
    return move_point_impl(point, false);
}

bool Geometric_Controller::move_velocity(controller_data_types::TargetVelocity_t velocity) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_curve_.clear();
    const bool fixed_height_active = velocity.fixed_height > 0.0;
    const ros::Time now = velocity.stamp.isZero() ? ros::Time::now() : velocity.stamp;
    velocity.velocity =
        reference_limit_helper::clamp_velocity_per_axis(velocity.velocity, max_velocity_);
    if (fixed_height_active) {
        velocity.velocity.z() = 0.0;
    }
    if (std::abs(velocity.yaw_rate) > 1e-6) {
        yaw_curve_.clear();
        velocity.yaw_rate =
            reference_limit_helper::clamp_yaw_rate(velocity.yaw_rate, max_yaw_rate_rad_s_);
        velocity.yaw = integrate_limited_yaw_rate(velocity.yaw_rate, now);
    } else {
        velocity.yaw = update_limited_yaw_target(velocity.yaw, now);
        velocity.yaw_rate = 0.0;
    }

    auto output =
        controller_.calculateVelocityControl(
            velocity, uav_odometry_, ThrustCommandPolicy::UseEstimatedAnchor);

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
    desired_state_.position = uav_odometry_.position;
    if (fixed_height_active) {
        desired_state_.position.z() = velocity.fixed_height;
    }
    desired_state_.velocity = velocity.velocity;
    desired_state_.acceleration = Eigen::Vector3d::Zero();
    desired_state_.jerk = Eigen::Vector3d::Zero();
    desired_state_.yaw = velocity.yaw;
    desired_state_.yaw_rate = velocity.yaw_rate;
    return true;
}

bool Geometric_Controller::move_trajectory(
    controller_data_types::TargetTrajectoryPoint_t trajpoint) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    yaw_curve_.clear();
    warn_if_trajectory_exceeds_limits(trajpoint);
    // 几何控制器的核心运动接口。
    // 将完整的轨迹点（位置 / 速度 / 加速度 / 加加速度 / yaw）直接送入核心算法，
    // 由两级控制环（位置 PD + 姿态子控制器）输出 body rate 和归一化推力。
    return publish_trajectory_setpoint(trajpoint, ThrustCommandPolicy::UseEstimatedAnchor);
}

bool Geometric_Controller::move_point_body(controller_data_types::TargetBodyPoint_t point) {
    constexpr double kPosEps = 1e-3;
    constexpr double kYawEps = 1e-3;
    bool is_new_body_target = !body_point_target_initialized_;
    controller_data_types::TargetPoint_t world_point = last_point_;

    const double dxy = (point.position_xy - last_point_body_.position_xy).norm();
    if (dxy > kPosEps || std::abs(point.fixed_height - last_point_body_.fixed_height) > kPosEps) {
        is_new_body_target = true;
    }
    if (!is_new_body_target && std::abs(point.yaw - last_point_body_.yaw) > kYawEps) {
        is_new_body_target = true;
    }

    if (is_new_body_target) {
        world_point = body_frame_reference_helper::to_world_point(uav_odometry_, point);

        last_point_body_ = point;
        body_point_target_initialized_ = true;
    }

    return move_point_impl(world_point, true);
}

bool Geometric_Controller::move_velocity_body(
    controller_data_types::TargetBodyVelocity_t velocity) {
    const controller_data_types::TargetVelocity_t world_velocity =
        body_frame_reference_helper::to_world_velocity(uav_odometry_, velocity);

    const bool accept = move_velocity(world_velocity);
    desired_state_.position = uav_odometry_.position;
    desired_state_.position.z() = velocity.fixed_height;
    desired_state_.velocity = world_velocity.velocity;
    desired_state_.acceleration = Eigen::Vector3d::Zero();
    desired_state_.jerk = Eigen::Vector3d::Zero();
    desired_state_.yaw = world_velocity.yaw;
    desired_state_.yaw_rate = world_velocity.yaw_rate;
    return accept;
}

bool Geometric_Controller::move_point_wgs84(geographic_msgs::GeoPoint point) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    yaw_curve_.clear();
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// 起降状态查询接口
// ─────────────────────────────────────────────────────────────────────────────
bool Geometric_Controller::is_takeoff_complete() {
    return takeoff_complete_.load(std::memory_order_relaxed);
}

bool Geometric_Controller::is_land_complete() {
    return land_complete_.load(std::memory_order_relaxed);
}

bool Geometric_Controller::is_point_complete() {
    return point_complete_.load(std::memory_order_relaxed);
}

// ═════════════════════════════════════════════════════════════════════════════
// 私有函数
// ═════════════════════════════════════════════════════════════════════════════

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

bool Geometric_Controller::has_valid_imu_data() {
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

Eigen::Vector3d Geometric_Controller::get_world_acc_from_imu() {
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

bool Geometric_Controller::check_mavros_stream_ready() {
    return mavros_helper_.is_ready();
}

void Geometric_Controller::pub_px4_state_timer_cb(const ros::TimerEvent&) {
    mavros_helper_.pub_px4_state();
}

void Geometric_Controller::pub_vision_fuse_timer_cb(const ros::TimerEvent&) {
    if (!can_fuse_.exchange(false, std::memory_order_relaxed)) {
        return;
    }
    mavros_helper_.pub_vision_pose(uav_odometry_);
}
