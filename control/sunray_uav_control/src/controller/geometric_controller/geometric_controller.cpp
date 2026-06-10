#include "controller/geometric_controller.hpp"
#include "eigen_helper.hpp"
#include "utils/control_config_loader.hpp"
#include "utils/body_frame_reference_helper.hpp"
#include "utils/orientation_utils.hpp"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace {

mavros_msgs::AttitudeTarget to_attitude_target_msg(
    const control_common::Mavros_SetpointAttitude& setpoint) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = setpoint.timestamp.isZero() ? ros::Time::now() : setpoint.timestamp;
    msg.type_mask = setpoint.mask;
    msg.orientation.x = setpoint.orientation.x();
    msg.orientation.y = setpoint.orientation.y();
    msg.orientation.z = setpoint.orientation.z();
    msg.orientation.w = setpoint.orientation.w();
    msg.body_rate.x = setpoint.body_rate.x();
    msg.body_rate.y = setpoint.body_rate.y();
    msg.body_rate.z = setpoint.body_rate.z();
    msg.thrust = setpoint.thrust;
    return msg;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// 构造函数
// ─────────────────────────────────────────────────────────────────────────────
Geometric_Controller::Geometric_Controller(ros::NodeHandle& nh) : nh_(nh) {
    (void)sunray_config::get_control_config_paths_or_throw();
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
// 读取最新的px4 setpoint
// ─────────────────────────────────────────────────────────────────────────────
bool Geometric_Controller::get_last_attitude_target(mavros_msgs::AttitudeTarget& msg) const {
    std::lock_guard<std::mutex> lock(last_setpoint_mutex_);
    if (!last_setpoint_.valid) {
        return false;
    }
    msg = to_attitude_target_msg(last_setpoint_);
    return true;
}

void Geometric_Controller::cache_attitude_setpoint(
    const control_common::Mavros_SetpointAttitude& setpoint) {
    std::lock_guard<std::mutex> lock(last_setpoint_mutex_);
    last_setpoint_ = setpoint;
    last_setpoint_.timestamp = ros::Time::now();
    last_setpoint_.valid = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 读取当前px4的推力，并将该数据输出给推力估计器
// ─────────────────────────────────────────────────────────────────────────────
void Geometric_Controller::feed_thrust_estimator_from_setpoint(
    const control_common::Mavros_SetpointAttitude& setpoint,
    bool is_hover_context) {
    if (!has_valid_imu_data()) {
        return;
    }
    if (controller_.thrust_estimator_should_estimate_onlyhover() && !is_hover_context) {
        return;
    }

    thrust_estimator::Input_t estimator_input;
    estimator_input.stamp = mavros_helper_.get_imu_data().stamp;
    estimator_input.attitude = uav_odometry_.orientation;
    estimator_input.velocity_w = uav_odometry_.velocity;
    estimator_input.acceleration_w = get_world_acc_from_imu();
    estimator_input.thrust_cmd = setpoint.thrust;
    controller_.feed_thrust_estimator(estimator_input);
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
    reset_takeoff_land_contexts();
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

// 决定是否需要重置起飞曲线的起点
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

// 清除运动曲线
void Geometric_Controller::clear_motion_curve() {
    motion_curve_.clear();
    motion_curve_owner_ = MotionCurveOwner::None;
}

// 重新开始运动曲线，并以形参决定曲线的所有权
void Geometric_Controller::begin_motion_curve(MotionCurveOwner owner) {
    motion_curve_.clear();
    motion_curve_owner_ = owner;
}

// 重置move_point阶段的上下文
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
// 更新yaw角目标
double Geometric_Controller::update_limited_yaw_target(double target_yaw, const ros::Time& now) {
    return reference_limit_helper::update_slewed_yaw_target(
        yaw_reference_state_, target_yaw, uav_odometry_.get_yaw(), max_yaw_rate_rad_s_, now);
}
// 更新yaw角速度目标
double Geometric_Controller::integrate_limited_yaw_rate(double yaw_rate_cmd, const ros::Time& now) {
    return reference_limit_helper::integrate_yaw_rate_command(
        yaw_reference_state_, yaw_rate_cmd, uav_odometry_.get_yaw(), max_yaw_rate_rad_s_, now);
}
// 警告如果传入的轨迹点属性超过了限制值
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
// 更新move_point控制阶段
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

    // 曲线运行中继续跟踪平滑参考，避免中途直接跳到目标点产生位置阶跃。
    // 只有 arrival_judge 通过且曲线已经结束时，move_point 才算真正完成。
    // 曲线走完或无效时，切换为目标点定点保持，前馈归零。
    const bool curve_still_running = curve_result.valid && !motion_curve_.is_finished();
    if (curve_still_running) {
        des_state.position = curve_result.position;
        des_state.velocity = curve_result.velocity;
        // des_state.acceleration = curve_result.acceleration;  // 暂时注释，观察曲线加速度是否合理
        des_state.acceleration = Eigen::Vector3d::Zero();
    } else {
        des_state.position = last_point_.position;
        des_state.velocity = Eigen::Vector3d::Zero();
        des_state.acceleration = Eigen::Vector3d::Zero();
    }
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
    const bool arrival_ok = arrival_helper::update_pose_and_check(point_arrival_state_,
                                                                  arrival_judge_config_,
                                                                  pos_err,
                                                                  vel_err,
                                                                  yaw_err,
                                                                  yaw_rate_err,
                                                                  now);
    if (!arrival_ok || curve_still_running) {
        point_complete_.store(false, std::memory_order_relaxed);
        return false;
    }

    point_complete_.store(true, std::memory_order_relaxed);
    update_hover_reference(last_point_.position, last_point_.yaw, "move_point_arrive");
    return true;
}

// 重置rc推力滤波器状态
void Geometric_Controller::reset_stage_thrust_filters() {
    takeoff_state_.thrust_filter = takeoff_land::RCThrustFilterState{};
    landing_state_.thrust_filter = takeoff_land::RCThrustFilterState{};
}

// 基于直接推力的起飞设计
bool Geometric_Controller::takeoff_direct_thrust(double relative_takeoff_height, double max_takeoff_velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
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
        cache_attitude_setpoint(setpoint_cmd);

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
        cache_attitude_setpoint(setpoint);
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
        cache_attitude_setpoint(setpoint);

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

// 基于直接推力估计的降落策略
bool Geometric_Controller::land_direct_thrust(double max_land_velocity) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    takeoff_state_.thrust_filter = takeoff_land::RCThrustFilterState{};
    // 进入降落流程时重置积分，防止下降阶段因残留积分产生额外推力
    if (landing_state_.start_time == ros::Time(0)) {
        controller_.reset_integral();
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
    cache_attitude_setpoint(setpoint);

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

bool Geometric_Controller::land_px4_autoland() {
    reset_stage_thrust_filters();
    mavros_helper_.set_px4_mode(control_common::FlightMode::AutoLand);
    return mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
}

void Geometric_Controller::reset_takeoff_land_contexts() {
    takeoff_state_.reset();
    landing_state_.reset();
    takeoff_accff_state_.reset();
    landing_accff_state_.reset();
    reset_stage_thrust_filters();
    takeoff_arrival_state_ = arrival_helper::State{};
    // 清掉 land_accff 入口设置的 anchor override,
    // 起飞 PreLift 必须使用 yaml 静态 hover_thrust_init 作为锚点。
    controller_.clear_fixed_anchor_override();
}

void Geometric_Controller::maybe_rebase_takeoff_curve_start_accff() {
    if (takeoff_accff_state_.curve_started ||
        motion_curve_owner_ != MotionCurveOwner::Takeoff ||
        !has_uav_odometry_.load(std::memory_order_relaxed)) {
        return;
    }
    motion_curve_.set_start_trajpoint(uav_odometry_.position, uav_odometry_.velocity);
    takeoff_accff_state_.curve_started = true;
}

bool Geometric_Controller::takeoff_accff(double relative_takeoff_height,
                                         double max_takeoff_velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    if (land_complete_.load(std::memory_order_relaxed)) {
        land_complete_.store(false, std::memory_order_relaxed);
    }
    if (!controller_ready_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (takeoff_complete_.load(std::memory_order_relaxed)) {
        return hover();
    }

    const ros::Time now = ros::Time::now();
    const double nominal_dt =
        1.0 / std::max(1.0, geometric_controller_param_.controller_hz);

    // ── 首次进入：初始化曲线、锁存起飞参数与 PreLift 状态 ────────────────────
    if (motion_curve_owner_ != MotionCurveOwner::Takeoff) {
        takeoff_arrival_state_ = arrival_helper::State{};
        begin_motion_curve(MotionCurveOwner::Takeoff);
        motion_curve_.set_start_trajpoint(uav_odometry_.position, Eigen::Vector3d::Zero());
        motion_curve_.set_end_trajpoint(uav_odometry_.position +
                                            Eigen::Vector3d(0.0, 0.0, relative_takeoff_height),
                                        Eigen::Vector3d::Zero());
        motion_curve_.set_curve_maxvel(max_takeoff_velocity);
        ground_height_ref_ = uav_odometry_.position.z();
        takeoff_accff_state_.reset();
        takeoff_accff_state_.yaw = uav_odometry_.get_yaw();
        controller_.set_initial_yaw(takeoff_accff_state_.yaw);
    }

    const control_common::Mavros_State px4_state = mavros_helper_.get_state();

    // ── 阶段 1：切 Offboard,保持零推力 ──────────────────────────────────────
    if (px4_state.flight_mode != control_common::FlightMode::Offboard) {
        takeoff_accff_state_.phase = takeoff_land::TakeoffPhaseAccFF::PreLift;
        takeoff_accff_state_.phase_start = ros::Time(0);
        takeoff_accff_state_.last_update = ros::Time(0);
        takeoff_accff_state_.a_ff_prev = takeoff_accff_tuning_.a_start_mps2;
        takeoff_accff_state_.curve_started = false;

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
        cache_attitude_setpoint(setpoint_cmd);

        if (start_checkout_offboard_time_ == ros::Time(0)) {
            start_checkout_offboard_time_ = now;
            last_checkout_offboard_time_ = ros::Time(0);
        }
        if (last_checkout_offboard_time_ == ros::Time(0) ||
            (now - last_checkout_offboard_time_).toSec() >= 0.3) {
            mavros_helper_.set_px4_mode(control_common::FlightMode::Offboard);
            last_checkout_offboard_time_ = now;
        }
        return false;
    }

    start_checkout_offboard_time_ = ros::Time(0);
    last_checkout_offboard_time_ = ros::Time(0);

    // ── 阶段 2：解锁 ────────────────────────────────────────────────────────
    if (px4_state.armed == false) {
        takeoff_accff_state_.phase = takeoff_land::TakeoffPhaseAccFF::PreLift;
        takeoff_accff_state_.phase_start = ros::Time(0);
        takeoff_accff_state_.last_update = ros::Time(0);
        takeoff_accff_state_.a_ff_prev = takeoff_accff_tuning_.a_start_mps2;
        takeoff_accff_state_.curve_started = false;
        takeoff_arrival_state_ = arrival_helper::State{};
        mavros_helper_.set_arm(true);
        return false;
    }

    const double rel_height = uav_odometry_.position.z() - ground_height_ref_;
    const bool airborne = rel_height > takeoff_accff_tuning_.liftoff_detect_h_m ||
                          uav_odometry_.velocity.z() > takeoff_accff_tuning_.liftoff_detect_vz_mps;

    // ── 阶段 3：PreLift  jerk-bounded S-curve a_start → a_target ────────────
    if (takeoff_accff_state_.phase == takeoff_land::TakeoffPhaseAccFF::PreLift) {
        if (takeoff_accff_state_.phase_start == ros::Time(0)) {
            takeoff_accff_state_.phase_start = now;
            takeoff_accff_state_.last_update = now;
            takeoff_accff_state_.a_ff_prev = takeoff_accff_tuning_.a_start_mps2;
        }

        if (airborne) {
            ROS_INFO("[takeoff_accff] PreLift -> AirborneCurve, rel_h=%.3f vz=%.3f",
                     rel_height, uav_odometry_.velocity.z());
            takeoff_accff_state_.phase = takeoff_land::TakeoffPhaseAccFF::AirborneCurve;
            takeoff_accff_state_.phase_start = now;
            takeoff_accff_state_.last_update = now;
            takeoff_accff_state_.curve_started = false;
        } else {
            double dt = (now - takeoff_accff_state_.last_update).toSec();
            if (!(dt > 0.0) || dt > 5.0 * nominal_dt) {
                dt = nominal_dt;
            }
            takeoff_accff_state_.last_update = now;
            const double a_ff = takeoff_land::advance_s_curve_acc(
                takeoff_accff_state_.a_ff_prev,
                takeoff_accff_tuning_.a_target_mps2,
                dt,
                takeoff_accff_tuning_.jerk_max_mps3,
                takeoff_accff_tuning_.a_min_mps2,
                takeoff_accff_tuning_.a_max_mps2);
            takeoff_accff_state_.a_ff_prev = a_ff;

            controller_data_types::TargetTrajectoryPoint_t des_state;
            des_state.position = Eigen::Vector3d(motion_curve_.get_start_position().x(),
                                                 motion_curve_.get_start_position().y(),
                                                 ground_height_ref_);
            des_state.velocity = Eigen::Vector3d::Zero();
            des_state.acceleration =
                Eigen::Vector3d(0.0, 0.0, a_ff - geometric_controller_param_.gravity);
            des_state.jerk = Eigen::Vector3d::Zero();
            des_state.yaw = takeoff_accff_state_.yaw;
            des_state.yaw_rate = 0.0;

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
            cache_attitude_setpoint(setpoint);

            ROS_INFO_THROTTLE(0.5,
                              "[takeoff_accff] PreLift a_ff=%.2f thrust=%.3f rel_h=%.3f vz=%.3f",
                              a_ff, setpoint.thrust, rel_height, uav_odometry_.velocity.z());

            const double elapsed = (now - takeoff_accff_state_.phase_start).toSec();
            if (elapsed > 1.5 * takeoff_accff_tuning_.ramp_time_s &&
                takeoff_accff_state_.a_ff_prev <
                    0.99 * takeoff_accff_tuning_.a_target_mps2) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[takeoff_accff] PreLift ramp 未达 a_target,可能需要增大 jerk_max_mps3 或 ramp_time_s");
            }
            return false;
        }
    }

    // ── 阶段 4：AirborneCurve  rebase + 五次项曲线 + UseFixedAnchor ─────────
    {
        maybe_rebase_takeoff_curve_start_accff();
        const curve::QuinticCurveState curve_result = motion_curve_.get_result();

        controller_data_types::TargetTrajectoryPoint_t des_state;
        des_state.position = curve_result.position;
        des_state.velocity = curve_result.velocity;
        des_state.acceleration = curve_result.acceleration;
        des_state.jerk = Eigen::Vector3d::Zero();
        des_state.yaw = takeoff_accff_state_.yaw;
        des_state.yaw_rate = 0.0;

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
        cache_attitude_setpoint(setpoint);

        // AirborneCurve 稳定爬升段开始喂 estimator,让其在 hover 之前就开始收敛。
        // 触发条件:rel_height > 0.30 且 |vz - target_vz| < 0.20。
        // is_hover_context=true 强制绕过 estimator 的 onlyhover 守卫——
        // 爬升中段物理状态(姿态接近 level、加速度温和)与 hover 接近,数据可信。
        // 这样 hover_thrust_percent 写错时也能在起飞过程中收敛,
        // 避免飞机持续超调爬到目标高度之上。
        const double rel_h = uav_odometry_.position.z() - ground_height_ref_;
        const double vz_err = std::abs(uav_odometry_.velocity.z() - curve_result.velocity.z());
        if (rel_h > 0.30 && vz_err < 0.20) {
            feed_thrust_estimator_from_setpoint(setpoint, /*is_hover_context=*/true);
        }

        const double pos_err =
            (uav_odometry_.position - motion_curve_.get_end_position()).norm();
        const double vel_err = uav_odometry_.velocity.norm();
        if (!arrival_helper::update_and_check(
                takeoff_arrival_state_, arrival_judge_config_, pos_err, vel_err, now)) {
            return false;
        }

        ROS_INFO("[takeoff_accff] AirborneCurve complete -> Hold (estimator hover_thrust=%.3f)",
                 controller_.get_accepted_hover_thrust());
        takeoff_complete_.store(true, std::memory_order_relaxed);
        // 不再 seed estimator:AirborneCurve 段已经持续喂数据,收敛值就是当前可信状态,
        // 强行 seed 当前 setpoint.thrust 反而会重置收敛进度。
        update_hover_reference(motion_curve_.get_end_position(),
                               takeoff_accff_state_.yaw,
                               "takeoff_accff_complete");
        start_checkout_offboard_time_ = ros::Time(0);
        last_checkout_offboard_time_ = ros::Time(0);
        controller_.reset_integral();
        clear_motion_curve();
        reset_takeoff_land_contexts();
        return true;
    }
}

bool Geometric_Controller::land_accff(double max_land_velocity) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    takeoff_state_.thrust_filter = takeoff_land::RCThrustFilterState{};

    const ros::Time now = ros::Time::now();
    const double nominal_dt =
        1.0 / std::max(1.0, geometric_controller_param_.controller_hz);

    if (land_complete_.load(std::memory_order_relaxed)) {
        return true;
    }

    if (!landing_accff_state_.entered) {
        controller_.reset_integral();
        landing_accff_state_.reset();
        landing_accff_state_.entered = true;
        landing_accff_state_.phase = takeoff_land::LandingPhaseAccFF::HighDescent;
        landing_accff_state_.phase_start = now;
        landing_accff_state_.last_update = now;
        landing_accff_state_.locked_xy_z = uav_odometry_.position;
        // yaw 与 hover 段保持同源,避免 mavros_helper get_yaw_rad 与 hover_yaw_ 不一致导致降落瞬间 yaw 跳变
        landing_accff_state_.yaw = hover_yaw_;
        landing_accff_state_.max_velocity = max_land_velocity;
        landing_accff_state_.a_ff_prev = geometric_controller_param_.gravity;
        // 降落入口 snapshot estimator 收敛值作为 UseFixedAnchor 锚点,
        // 替代 yaml 静态 hover_thrust_init。这样降落锚点会自动反映载荷/电池实际状态,
        // 不再因 yaml hover_thrust_percent 偏差导致"降不下来"或"硬墩"。
        const double snapshot = controller_.get_accepted_hover_thrust();
        controller_.set_fixed_anchor_override(snapshot);
        ROS_INFO("[land_accff] anchor snapshot from estimator = %.3f (yaml init = %.3f)",
                 snapshot, geometric_controller_param_.hover_thrust_init);
    }

    const double height_above_ground =
        std::max(0.0, uav_odometry_.position.z() - ground_height_ref_);
    const control_common::LandedState px4_land_state =
        mavros_helper_.get_state().landed_state;
    const bool px4_landed = (px4_land_state == control_common::LandedState::OnGround);

    // ── 阶段切换:HighDescent → NearGround ────────────────────────────────
    if (landing_accff_state_.phase == takeoff_land::LandingPhaseAccFF::HighDescent &&
        height_above_ground < landing_accff_tuning_.near_ground_h_m) {
        ROS_INFO("[land_accff] HighDescent -> NearGround, h=%.3f", height_above_ground);
        landing_accff_state_.phase = takeoff_land::LandingPhaseAccFF::NearGround;
        landing_accff_state_.phase_start = now;
        landing_accff_state_.last_update = now;
        landing_accff_state_.a_ff_prev = geometric_controller_param_.gravity;
        controller_.reset_vertical_integral();
    }

    // ── 阶段切换:NearGround → TouchdownRelease ───────────────────────────
    if (landing_accff_state_.phase == takeoff_land::LandingPhaseAccFF::NearGround) {
        bool should_touchdown = false;
        if (landing_accff_tuning_.touchdown_landed_state && px4_landed) {
            should_touchdown = true;
        }
        const bool settle_now =
            height_above_ground < landing_accff_tuning_.touchdown_h_settle_m &&
            uav_odometry_.velocity.norm() < landing_accff_tuning_.touchdown_v_settle_mps;
        if (settle_now) {
            if (landing_accff_state_.settle_start == ros::Time(0)) {
                landing_accff_state_.settle_start = now;
            } else if ((now - landing_accff_state_.settle_start).toSec() >=
                       landing_accff_tuning_.touchdown_dwell_s) {
                should_touchdown = true;
            }
        } else {
            landing_accff_state_.settle_start = ros::Time(0);
        }
        if (should_touchdown) {
            ROS_INFO("[land_accff] NearGround -> TouchdownRelease, h=%.3f v=%.3f px4_landed=%d",
                     height_above_ground, uav_odometry_.velocity.norm(),
                     static_cast<int>(px4_landed));
            landing_accff_state_.phase = takeoff_land::LandingPhaseAccFF::TouchdownRelease;
            landing_accff_state_.phase_start = now;
            landing_accff_state_.last_update = now;
            landing_accff_state_.touchdown_start = now;
        }
    }

    // ── 阶段:TouchdownRelease  旁路 thrust=0 + disarm ─────────────────────
    if (landing_accff_state_.phase == takeoff_land::LandingPhaseAccFF::TouchdownRelease) {
        controller_data_types::TargetTrajectoryPoint_t des_state;
        des_state.position = Eigen::Vector3d(landing_accff_state_.locked_xy_z.x(),
                                             landing_accff_state_.locked_xy_z.y(),
                                             ground_height_ref_);
        des_state.velocity = Eigen::Vector3d::Zero();
        des_state.acceleration = Eigen::Vector3d::Zero();
        des_state.jerk = Eigen::Vector3d::Zero();
        des_state.yaw = landing_accff_state_.yaw;
        des_state.yaw_rate = 0.0;
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
        setpoint.thrust = 0.0;
        mavros_helper_.pub_attitude_setpoint(setpoint);
        cache_attitude_setpoint(setpoint);

        const double touchdown_elapsed =
            (now - landing_accff_state_.touchdown_start).toSec();
        if (px4_landed || touchdown_elapsed > 1.0) {
            mavros_helper_.set_arm(false);
            if (mavros_helper_.get_state().armed == false) {
                land_complete_.store(true, std::memory_order_relaxed);
                takeoff_complete_.store(false, std::memory_order_relaxed);
                reset_takeoff_land_contexts();
                return true;
            }
        }
        return false;
    }

    // ── 阶段:HighDescent / NearGround  通过 calculateControl 输出 ─────────
    // v5 设计:整段 vz_ref(h) 与 a_ff(h) 共用同一条五次平滑曲线 (smootherstep),
    //         端点 0/1/2 阶导数全连续,从 (vz=-far_v, a=g) 单调过渡到 (vz=-near_v, a=a_touchdown)。
    //         phase 仍区分 HighDescent / NearGround,但仅用于触地判定窗口,不影响 vz_ref / a_ff 计算。
    controller_data_types::TargetTrajectoryPoint_t des_state;
    const double lookahead_time = 0.35;
    const double far_v  = std::max(0.05, landing_accff_state_.max_velocity);
    const double near_v = landing_accff_tuning_.near_ground_vz_mps;
    const double h_high = std::max(landing_accff_tuning_.near_ground_h_m + 0.10,
                                   2.0 * landing_accff_tuning_.near_ground_h_m);
    const double gravity = geometric_controller_param_.gravity;

    // 归一化高度 u ∈ [0, 1]:u=1 在 h_high 之上,u=0 在触地
    const double u = std::clamp(height_above_ground / std::max(h_high, 1e-3), 0.0, 1.0);
    // smootherstep: s(u) = 6u^5 - 15u^4 + 10u^3
    // 性质: s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
    const double s = u * u * u * (u * (u * 6.0 - 15.0) + 10.0);

    const double desc_speed = near_v + s * (far_v - near_v);

    // a_ff 同样按 s 在 a_touchdown 与 g 之间插值;u=1 时 a_ff=g(不引入 acc_ref),u=0 时 a_ff=a_touchdown。
    // 进一步用 jerk-bounded ramp 平滑跨帧跳变(防止 h 抖动反映到 a_ff)。
    const double a_target = landing_accff_tuning_.a_touchdown_mps2 +
                            s * (gravity - landing_accff_tuning_.a_touchdown_mps2);
    double dt = (now - landing_accff_state_.last_update).toSec();
    if (!(dt > 0.0) || dt > 5.0 * nominal_dt) {
        dt = nominal_dt;
    }
    landing_accff_state_.last_update = now;
    const double a_ff_now = takeoff_land::advance_s_curve_acc(
        landing_accff_state_.a_ff_prev,
        a_target,
        dt,
        landing_accff_tuning_.jerk_max_mps3,
        landing_accff_tuning_.a_min_mps2,
        landing_accff_tuning_.a_max_mps2);
    landing_accff_state_.a_ff_prev = a_ff_now;

    // target_z 略高于当前高度(约 0.05 m),让 pos_error_z = +0.05 给一个小刹车,
    // 抵消 vel_kp 不足以完全压住 -0.30 m/s 时的超速倾向。
    // 不再用 lookahead 减偏(lookahead 会让 pos_error 持续为负,反而推动加速)。
    constexpr double kPosBrakeOffsetM = 0.05;
    const double target_z =
        std::max(uav_odometry_.position.z() + kPosBrakeOffsetM, ground_height_ref_);
    des_state.position = Eigen::Vector3d(landing_accff_state_.locked_xy_z.x(),
                                         landing_accff_state_.locked_xy_z.y(),
                                         target_z);
    des_state.velocity = Eigen::Vector3d(0.0, 0.0, -desc_speed);
    des_state.acceleration = Eigen::Vector3d(0.0, 0.0, a_ff_now - gravity);
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = landing_accff_state_.yaw;
    des_state.yaw_rate = 0.0;

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
    cache_attitude_setpoint(setpoint);

    if (landing_accff_state_.phase == takeoff_land::LandingPhaseAccFF::NearGround) {
        ROS_INFO_THROTTLE(0.5,
                          "[land_accff] NearGround a_ff=%.2f thrust=%.3f h=%.3f vz=%.3f",
                          a_ff_now, setpoint.thrust, height_above_ground,
                          uav_odometry_.velocity.z());
    }
    return false;
}

bool Geometric_Controller::takeoff(double relative_takeoff_height, double max_takeoff_velocity) {
    if (takeoff_land_type_ == 0) {
        return takeoff_direct_thrust(relative_takeoff_height, max_takeoff_velocity);
    }
    return takeoff_accff(relative_takeoff_height, max_takeoff_velocity);
}

bool Geometric_Controller::land(bool land_type, double max_land_velocity) {
    //  选择使用px4的auto_land方式降落
    if (land_type) {
        reset_takeoff_land_contexts();
        return land_px4_autoland();
    }
    // 选择使用直接推力估计降落
    if (takeoff_land_type_ == 0) {
        return land_direct_thrust(max_land_velocity);
    }
    // 选择使用基于加速度反馈的推力降落
    return land_accff(max_land_velocity);
}

bool Geometric_Controller::set_hover_point(control_common::UAVStateEstimate current_odom) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    update_hover_reference(current_odom.position, current_odom.get_yaw(), "set_hover_point");
    return true;
}

bool Geometric_Controller::set_hover_point_to_last_target() {
    if (!point_target_initialized_) {
        return false;
    }
    // 先抓快照，因为 reset_point_motion_context() 会把 last_point_ 清掉
    const controller_data_types::TargetPoint_t target_snapshot = last_point_;
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    update_hover_reference(target_snapshot.position, target_snapshot.yaw,
                           "set_hover_point_to_last_target");
    return true;
}

bool Geometric_Controller::hover() {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    // hover 入口顺带清掉起降残留状态,防止 A/B 路径互相污染。
    // 注意: takeoff_complete_/land_complete_ 等高层状态由调用方维护,这里不动。
    takeoff_accff_state_.reset();
    landing_accff_state_.reset();
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
    cache_attitude_setpoint(setpoint);
    feed_thrust_estimator_from_setpoint(setpoint, true);
    return true;
}

bool Geometric_Controller::emergency_kill() {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    reset_takeoff_land_contexts();
    return mavros_helper_.emergency_kill();
}

bool Geometric_Controller::move_point(controller_data_types::TargetPoint_t point) {
    return move_point_impl(point, false);
}

bool Geometric_Controller::move_velocity(controller_data_types::TargetVelocity_t velocity) {
    clear_motion_curve();
    reset_point_motion_context();
    const bool fixed_height_active = velocity.fixed_height > 0.0;
    const ros::Time now = velocity.stamp.isZero() ? ros::Time::now() : velocity.stamp;
    velocity.velocity =
        reference_limit_helper::clamp_velocity_per_axis(velocity.velocity, max_velocity_);
    if (fixed_height_active) {
        velocity.velocity.z() = 0.0;
    }
    if (std::abs(velocity.yaw_rate) > 1e-6) {
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
    cache_attitude_setpoint(setpoint);
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
    cache_attitude_setpoint(setpoint);
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
