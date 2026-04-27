#include "controller/raptor_controller.hpp"
#include "eigen_helper.hpp"
#include "utils/uav_param_utils.hpp"
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <stdexcept>
#include <sunray_msgs/UAVControllerState.h>

namespace {

constexpr double kPubPx4StateFreq = 100.0;
constexpr uint8_t kRaptorExternalModeIndex = 1;
constexpr double kModeRequestIntervalS = 0.3;
constexpr double kMinCurveVelocity = 0.05;
constexpr double kLandLookaheadTimeS = 0.2;
constexpr double kLandSlowHeightM = 0.20;
constexpr double kLandTouchdownHeightM = 0.05;
constexpr double kLandTouchdownSpeedMps = 0.06;
constexpr double kLandTouchdownDetectVelocityMps = 0.10;
constexpr double kLandMinDurationS = 2.0;
constexpr double kLandDisarmHoldS = 1.0;

}  // namespace

Raptor_Controller::Raptor_Controller(ros::NodeHandle& nh) : nh_(nh), mavros_helper_(nh_) {
    std::string node_name = ros::this_node::getName();
    ros::NodeHandle private_nh_("~");

    if (private_nh_.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        if (config_yamlfile_path_.empty()) {
            throw std::runtime_error("yaml_path cannot be empty");
        }
    } else {
        throw std::runtime_error("missing param " + node_name + "/config_yamlfile_path");
    }

    uav_ns_ = sunray_control::load_uav_namespace_or_throw(nh_);
}

bool Raptor_Controller::init() {
    load_and_validate_config_or_throw();

    MavrosHelper_ConfigList config_list(true);
    if (!mavros_helper_.init(config_list)) {
        throw std::runtime_error("mavros_helper init failed");
    }

    if (fuse_odom_type != 0) {
        ensure_fusion_param_ready_or_throw();
        mavros_helper_.set_vision_fuse_type(fuse_odom_type);
        pub_vision_pose_timer_ = nh_.createTimer(ros::Duration(1.0 / fuse_odom_frequency),
                                                 &Raptor_Controller::pub_vision_fuse_timer_cb,
                                                 this);
    }

    controller_state_pub_ =
        nh_.advertise<sunray_msgs::UAVControllerState>(uav_ns_ + "/sunray/controller_state", 10);
    pub_px4_state_timer_ = nh_.createTimer(ros::Duration(1.0 / kPubPx4StateFreq),
                                           &Raptor_Controller::pub_px4_state_timer_cb,
                                           this);
    return true;
}

bool Raptor_Controller::is_ready() {
    const control_common::Mavros_State mavros_state = mavros_helper_.get_state();
    if (!mavros_state.connected) {
        return false;
    }
    if (!mavros_helper_.is_ready()) {
        return false;
    }

    controller_ready_.store(true, std::memory_order_relaxed);
    return true;
}

void Raptor_Controller::set_current_odom(const control_common::UAVStateEstimate& odom) {
    uav_odometry_ = odom;
    const bool has_valid_timestamp = !odom.timestamp.isZero();
    const bool is_new_odom = has_valid_timestamp && odom.timestamp != last_odom_timestamp_;
    has_uav_odometry_.store(has_valid_timestamp, std::memory_order_relaxed);
    if (is_new_odom) {
        last_odom_timestamp_ = odom.timestamp;
        can_fuse_.store(true, std::memory_order_relaxed);
    }
}

void Raptor_Controller::set_position_mode() {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    clear_cached_setpoint();
    const control_common::Mavros_State state = mavros_helper_.get_state();
    if (state.flight_mode != control_common::FlightMode::Posctl) {
        mavros_helper_.set_px4_mode(control_common::FlightMode::Posctl);
    }
}

double Raptor_Controller::compute_move_point_curve_maxvel(const Eigen::Vector3d& start_position,
                                                          const Eigen::Vector3d& goal_position) const {
    return reference_limit_helper::compute_point_curve_maxvel(
        start_position, goal_position, max_move_velocity_, kMinCurveVelocity);
}

double Raptor_Controller::update_limited_yaw_target(double target_yaw, const ros::Time& now) {
    return reference_limit_helper::update_slewed_yaw_target(
        yaw_reference_state_, target_yaw, uav_odometry_.get_yaw(), max_yaw_rate_rad_s_, now);
}

void Raptor_Controller::warn_if_trajectory_exceeds_limits(
    const controller_data_types::TargetTrajectoryPoint_t& trajpoint) const {
    if (!reference_limit_helper::trajectory_reference_exceeds_limits(
            trajpoint.velocity, trajpoint.yaw_rate, max_move_velocity_, max_yaw_rate_rad_s_)) {
        return;
    }

    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[Raptor_Controller][" << uav_ns_
                                << "] trajectory reference exceeds velocity_param limits: vel="
                                << trajpoint.velocity.transpose() << " max="
                                << max_move_velocity_.transpose() << " yaw_rate="
                                << trajpoint.yaw_rate << " max_yaw_rate="
                                << max_yaw_rate_rad_s_);
}

bool Raptor_Controller::publish_trajectory_setpoint(
    const controller_data_types::TargetTrajectoryPoint_t& trajpoint,
    bool preserve_point_context) {
    if (!has_uav_odometry_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (!preserve_point_context) {
        reset_point_motion_context();
    }

    control_common::Mavros_SetpointLocal setpoint;
    setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    setpoint.position = trajpoint.position;
    setpoint.velocity = trajpoint.velocity;
    setpoint.accel_or_force = trajpoint.acceleration;
    setpoint.yaw = trajpoint.yaw;
    setpoint.yaw_rate = trajpoint.yaw_rate;
    mavros_helper_.pub_local_setpoint(setpoint);
    cache_local_setpoint(setpoint);
    desired_state_ = trajpoint;
    return true;
}

bool Raptor_Controller::move_point_impl(controller_data_types::TargetPoint_t point,
                                        bool preserve_body_point_context) {
    if (!has_uav_odometry_.load(std::memory_order_relaxed)) {
        return false;
    }

    constexpr double kNewTargetPosEps = 1e-3;
    if (!preserve_body_point_context) {
        body_point_target_initialized_ = false;
    }

    const bool is_new_target = !point_target_initialized_ ||
                               (point.position - last_point_.position).norm() > kNewTargetPosEps;
    if (is_new_target) {
        const Eigen::Vector3d start_position =
            last_setpoint_.valid ? last_setpoint_.position : uav_odometry_.position;
        const Eigen::Vector3d start_velocity =
            last_setpoint_.valid ? last_setpoint_.velocity : uav_odometry_.velocity;
        move_point_curve_.clear();
        last_point_ = point;
        point_target_initialized_ = true;
        point_arrival_state_ = arrival_helper::State{};
        point_complete_.store(false, std::memory_order_relaxed);
        move_point_curve_.set_start_trajpoint(start_position, start_velocity);
        move_point_curve_.set_end_trajpoint(point.position, Eigen::Vector3d::Zero());
        move_point_curve_.set_curve_maxvel(
            compute_move_point_curve_maxvel(start_position, point.position));
    }

    curve::QuinticCurveState curve_result = move_point_curve_.get_result();
    controller_data_types::TargetTrajectoryPoint_t trajpoint;
    // Keep point mode close to historical "position hold" semantics:
    // smooth only the position reference, and avoid quintic velocity/acceleration
    // feedforward that can make move_point much more aggressive.
    trajpoint.position = curve_result.valid ? curve_result.position : point.position;
    trajpoint.velocity = Eigen::Vector3d::Zero();
    trajpoint.acceleration = Eigen::Vector3d::Zero();
    trajpoint.jerk = Eigen::Vector3d::Zero();
    trajpoint.yaw = update_limited_yaw_target(point.yaw, ros::Time::now());
    trajpoint.yaw_rate = 0.0;
    if (!publish_trajectory_setpoint(trajpoint, true)) {
        return false;
    }

    if (point_complete_.load(std::memory_order_relaxed)) {
        return true;
    }

    const ros::Time now = ros::Time::now();
    const double pos_err = (uav_odometry_.position - last_point_.position).norm();
    const double vel_err = uav_odometry_.velocity.norm();
    if (!arrival_helper::update_and_check(
            point_arrival_state_, arrival_judge_config_, pos_err, vel_err, now)) {
        point_complete_.store(false, std::memory_order_relaxed);
        return false;
    }

    point_complete_.store(true, std::memory_order_relaxed);
    hover_point_ = last_point_.position;
    hover_yaw_ = last_point_.yaw;
    return true;
}

void Raptor_Controller::reset_point_motion_context() {
    move_point_curve_.clear();
    point_arrival_state_ = arrival_helper::State{};
    point_complete_.store(false, std::memory_order_relaxed);
    point_target_initialized_ = false;
    body_point_target_initialized_ = false;
    last_point_ = controller_data_types::TargetPoint_t{};
    last_point_body_ = controller_data_types::TargetBodyPoint_t{};
}

void Raptor_Controller::clear_cached_setpoint() {
    last_setpoint_ = control_common::Mavros_SetpointLocal{};
    desired_state_ = has_uav_odometry_.load(std::memory_order_relaxed)
                         ? make_hold_desired_state(uav_odometry_)
                         : controller_data_types::TargetTrajectoryPoint_t{};
}

void Raptor_Controller::cache_local_setpoint(const control_common::Mavros_SetpointLocal& setpoint) {
    last_setpoint_ = setpoint;
    last_setpoint_.timestamp = ros::Time::now();
    last_setpoint_.valid = true;

    desired_state_.position = setpoint.position;
    desired_state_.velocity = setpoint.velocity;
    desired_state_.acceleration = setpoint.accel_or_force;
    desired_state_.jerk = Eigen::Vector3d::Zero();
    desired_state_.yaw = setpoint.yaw;
    desired_state_.yaw_rate = setpoint.yaw_rate;
}

controller_data_types::TargetTrajectoryPoint_t Raptor_Controller::make_hold_desired_state(
    const control_common::UAVStateEstimate& odom) const {
    controller_data_types::TargetTrajectoryPoint_t desired_state;
    desired_state.position = odom.position;
    desired_state.yaw = eigen_helper::get_yaw_from_orientation(odom.orientation);
    return desired_state;
}

void Raptor_Controller::publish_hold_setpoint(double yaw) {
    control_common::Mavros_SetpointLocal setpoint;
    setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    setpoint.position = last_setpoint_.valid ? last_setpoint_.position : uav_odometry_.position;
    setpoint.velocity = Eigen::Vector3d::Zero();
    setpoint.accel_or_force = Eigen::Vector3d::Zero();
    setpoint.yaw = yaw;
    setpoint.yaw_rate = 0.0;
    mavros_helper_.pub_local_setpoint(setpoint);
    cache_local_setpoint(setpoint);
}

bool Raptor_Controller::ensure_raptor_mode(const ros::Time& now, double yaw) {
    const control_common::Mavros_State px4_state = mavros_helper_.get_state();
    if (px4_state.flight_mode == control_common::FlightMode::Raptor) {
        last_checkout_raptor_time_ = ros::Time(0);
        return true;
    }

    publish_hold_setpoint(yaw);

    if (last_checkout_raptor_time_ == ros::Time(0) ||
        (now - last_checkout_raptor_time_).toSec() >= kModeRequestIntervalS) {
        mavros_helper_.set_px4_external_mode(kRaptorExternalModeIndex);
        last_checkout_raptor_time_ = now;
    }
    return false;
}

bool Raptor_Controller::takeoff(double relative_takeoff_height, double max_takeoff_velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    if (land_complete_.load(std::memory_order_relaxed)) {
        land_complete_.store(false, std::memory_order_relaxed);
    }
    if (!controller_ready_.load(std::memory_order_relaxed) ||
        !has_uav_odometry_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (takeoff_complete_.load(std::memory_order_relaxed)) {
        return hover();
    }

    const ros::Time now = ros::Time::now();
    if (!quint_curve_.is_ready()) {
        takeoff_arrival_state_ = arrival_helper::State{};
        quint_curve_.set_start_trajpoint(uav_odometry_.position, Eigen::Vector3d::Zero());
        quint_curve_.set_end_trajpoint(uav_odometry_.position +
                                           Eigen::Vector3d(0.0, 0.0, relative_takeoff_height),
                                       Eigen::Vector3d::Zero());
        quint_curve_.set_curve_maxvel(std::max(kMinCurveVelocity, std::abs(max_takeoff_velocity)));
        takeoff_yaw_ = uav_odometry_.get_yaw();
        ground_height_ref_ = uav_odometry_.position.z();
    }

    if (!ensure_raptor_mode(now, takeoff_yaw_)) {
        takeoff_arrival_state_ = arrival_helper::State{};
        return false;
    }

    if (!mavros_helper_.get_state().armed) {
        takeoff_arrival_state_ = arrival_helper::State{};
        publish_hold_setpoint(takeoff_yaw_);
        mavros_helper_.set_arm(true);
        return false;
    }

    const curve::QuinticCurveState curve_result = quint_curve_.get_result();
    control_common::Mavros_SetpointLocal setpoint;
    setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    setpoint.position = curve_result.position;
    setpoint.velocity = curve_result.velocity;
    setpoint.accel_or_force = curve_result.acceleration;
    setpoint.yaw = takeoff_yaw_;
    setpoint.yaw_rate = 0.0;
    mavros_helper_.pub_local_setpoint(setpoint);
    cache_local_setpoint(setpoint);

    const double pos_err = (uav_odometry_.position - quint_curve_.get_end_position()).norm();
    const double vel_err = uav_odometry_.velocity.norm();
    if (!arrival_helper::update_and_check(
            takeoff_arrival_state_, arrival_judge_config_, pos_err, vel_err, now)) {
        return false;
    }

    takeoff_complete_.store(true, std::memory_order_relaxed);
    hover_point_ = quint_curve_.get_end_position();
    hover_yaw_ = takeoff_yaw_;
    takeoff_arrival_state_ = arrival_helper::State{};
    quint_curve_.clear();
    return true;
}

bool Raptor_Controller::land(bool land_type, double max_land_velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    if (land_type == 1) {
        mavros_helper_.set_px4_mode(control_common::FlightMode::AutoLand);
        const bool landed =
            mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
        if (landed) {
            land_complete_.store(true, std::memory_order_relaxed);
            takeoff_complete_.store(false, std::memory_order_relaxed);
            start_land_time_ = ros::Time(0);
            land_touchground_time_ = ros::Time(0);
        }
        return landed;
    }

    if (!controller_ready_.load(std::memory_order_relaxed) ||
        !has_uav_odometry_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (land_complete_.load(std::memory_order_relaxed)) {
        return true;
    }

    const ros::Time now = ros::Time::now();
    if (start_land_time_ == ros::Time(0)) {
        start_land_time_ = now;
        land_touchground_time_ = ros::Time(0);
        land_yaw_ = uav_odometry_.get_yaw();
        hover_point_ = last_setpoint_.valid ? last_setpoint_.position : uav_odometry_.position;
    }

    if (!ensure_raptor_mode(now, land_yaw_)) {
        return false;
    }

    const control_common::Mavros_State px4_state = mavros_helper_.get_state();
    const double height_above_ground =
        std::max(0.0, uav_odometry_.position.z() - ground_height_ref_);
    if (!px4_state.armed) {
        if (height_above_ground <= kLandTouchdownHeightM) {
            land_complete_.store(true, std::memory_order_relaxed);
            takeoff_complete_.store(false, std::memory_order_relaxed);
            start_land_time_ = ros::Time(0);
            land_touchground_time_ = ros::Time(0);
            return true;
        }

        return false;
    }

    const double descend_rate = std::max(kMinCurveVelocity, std::abs(max_land_velocity));
    const double clamped_far_speed = std::max(descend_rate, kLandTouchdownSpeedMps);

    double descend_speed = clamped_far_speed;
    if (height_above_ground < kLandSlowHeightM) {
        const double blend = std::clamp((height_above_ground - kLandTouchdownHeightM) /
                                            std::max(kLandSlowHeightM - kLandTouchdownHeightM, 1e-3),
                                        0.0,
                                        1.0);
        const double smooth_blend = blend * blend * (3.0 - 2.0 * blend);
        descend_speed = kLandTouchdownSpeedMps +
                        smooth_blend * (clamped_far_speed - kLandTouchdownSpeedMps);
    }

    const double target_velocity_z = -descend_speed;

    controller_data_types::TargetTrajectoryPoint_t des_state;
    des_state.position = hover_point_;
    des_state.position.z() =
        std::max(ground_height_ref_,
                 uav_odometry_.position.z() + target_velocity_z * kLandLookaheadTimeS);
    des_state.velocity = Eigen::Vector3d(0.0, 0.0, target_velocity_z);
    des_state.acceleration = Eigen::Vector3d::Zero();
    des_state.jerk = Eigen::Vector3d::Zero();
    des_state.yaw = land_yaw_;
    des_state.yaw_rate = 0.0;
    move_trajectory(des_state);

    const bool near_ground = height_above_ground <= kLandTouchdownHeightM;
    const bool velocity_low =
        std::abs(uav_odometry_.velocity.z()) <= kLandTouchdownDetectVelocityMps;
    const bool land_duration_ok = (now - start_land_time_).toSec() >= kLandMinDurationS;
    const bool touchdown_candidate = land_duration_ok && near_ground && velocity_low;

    if (touchdown_candidate) {
        if (land_touchground_time_ == ros::Time(0)) {
            land_touchground_time_ = now;
        }
        if ((now - land_touchground_time_).toSec() >= kLandDisarmHoldS) {
            mavros_helper_.set_arm(false);
            if (!mavros_helper_.get_state().armed) {
                land_complete_.store(true, std::memory_order_relaxed);
                takeoff_complete_.store(false, std::memory_order_relaxed);
                start_land_time_ = ros::Time(0);
                land_touchground_time_ = ros::Time(0);
                quint_curve_.clear();
                return true;
            }
        }
    } else {
        land_touchground_time_ = ros::Time(0);
    }

    return false;
}

bool Raptor_Controller::set_hover_point(control_common::UAVStateEstimate current_odom) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    hover_point_ = current_odom.position;
    hover_yaw_ = current_odom.get_yaw();
    return true;
}

bool Raptor_Controller::hover() {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    if (!has_uav_odometry_.load(std::memory_order_relaxed)) {
        return false;
    }

    control_common::Mavros_SetpointLocal setpoint;
    setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    setpoint.position = hover_point_;
    setpoint.velocity = Eigen::Vector3d::Zero();
    setpoint.accel_or_force = Eigen::Vector3d::Zero();
    setpoint.yaw = hover_yaw_;
    setpoint.yaw_rate = 0.0;
    mavros_helper_.pub_local_setpoint(setpoint);
    cache_local_setpoint(setpoint);
    return true;
}

bool Raptor_Controller::emergency_kill() {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    clear_cached_setpoint();
    return mavros_helper_.emergency_kill();
}

bool Raptor_Controller::move_point(controller_data_types::TargetPoint_t point) {
    return move_point_impl(point, false);
}

bool Raptor_Controller::move_velocity(controller_data_types::TargetVelocity_t velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    (void)velocity;
    ROS_WARN_THROTTLE(
        1.0,
        "[Raptor_Controller][%s] move_velocity is not supported because RAPTOR cannot express a pure velocity interface",
        uav_ns_.c_str());
    return false;
}

bool Raptor_Controller::move_trajectory(
    controller_data_types::TargetTrajectoryPoint_t trajpoint) {
    yaw_reference_state_.reset();
    warn_if_trajectory_exceeds_limits(trajpoint);
    return publish_trajectory_setpoint(trajpoint, false);
}

bool Raptor_Controller::move_point_body(controller_data_types::TargetBodyPoint_t point) {
    if (!has_uav_odometry_.load(std::memory_order_relaxed)) {
        return false;
    }

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
        const double yaw = uav_odometry_.get_yaw();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        Eigen::Vector2d delta_w;
        delta_w.x() = c * point.position_xy.x() - s * point.position_xy.y();
        delta_w.y() = s * point.position_xy.x() + c * point.position_xy.y();

        world_point.position.x() = uav_odometry_.position.x() + delta_w.x();
        world_point.position.y() = uav_odometry_.position.y() + delta_w.y();
        world_point.position.z() = point.fixed_height;
        world_point.yaw = yaw + point.yaw;
        last_point_body_ = point;
        body_point_target_initialized_ = true;
    }

    return move_point_impl(world_point, true);
}

bool Raptor_Controller::move_velocity_body(controller_data_types::TargetBodyVelocity_t velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    (void)velocity;
    ROS_WARN_THROTTLE(
        1.0,
        "[Raptor_Controller][%s] move_velocity_body is not supported because RAPTOR cannot express a pure velocity interface",
        uav_ns_.c_str());
    return false;
}

bool Raptor_Controller::move_point_wgs84(geographic_msgs::GeoPoint point) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    (void)point;
    return false;
}

bool Raptor_Controller::is_takeoff_complete() {
    return takeoff_complete_.load(std::memory_order_relaxed);
}

bool Raptor_Controller::is_land_complete() {
    return land_complete_.load(std::memory_order_relaxed);
}

bool Raptor_Controller::is_point_complete() {
    return point_complete_.load(std::memory_order_relaxed);
}

void Raptor_Controller::pub_controller_state() {
    update_log_snapshot();

    if (!has_uav_odometry_.load(std::memory_order_relaxed)) {
        return;
    }

    const control_common::Mavros_SetpointLocal px4_local_target = mavros_helper_.get_target_local();
    const bool has_controller_setpoint = last_setpoint_.valid;
    const control_common::Mavros_SetpointLocal controller_local_output =
        has_controller_setpoint ? last_setpoint_ : control_common::Mavros_SetpointLocal{};
    const controller_data_types::TargetTrajectoryPoint_t desired_state =
        has_controller_setpoint ? desired_state_ : make_hold_desired_state(uav_odometry_);

    sunray_msgs::UAVControllerState msg;
    msg.header.stamp =
        uav_odometry_.timestamp.isZero() ? ros::Time::now() : uav_odometry_.timestamp;
    msg.reference_frame = sunray_msgs::UAVControllerState::FRAME_LOCAL;
    msg.controller_type = sunray_msgs::UAVControllerState::RL_RAPTOR_CONTROLLER;

    msg.desired_pos = eigen_helper::to_ros_point(desired_state.position);
    msg.desired_vel = eigen_helper::to_ros_vector3(desired_state.velocity);
    msg.desired_acc = eigen_helper::to_ros_vector3(desired_state.acceleration);
    msg.desired_yaw = desired_state.yaw;
    msg.desired_yawrate = desired_state.yaw_rate;

    msg.current_pos = eigen_helper::to_ros_point(uav_odometry_.position);
    msg.current_vel = eigen_helper::to_ros_vector3(uav_odometry_.velocity);
    msg.current_attitude = eigen_helper::to_ros_quaternion(uav_odometry_.orientation);
    msg.current_bodyrate = eigen_helper::to_ros_vector3(uav_odometry_.bodyrate);
    msg.current_yaw = eigen_helper::get_yaw_from_orientation(uav_odometry_.orientation);

    msg.pos_error = eigen_helper::to_ros_vector3(desired_state.position - uav_odometry_.position);
    msg.vel_error = eigen_helper::to_ros_vector3(desired_state.velocity - uav_odometry_.velocity);
    msg.yaw_error = eigen_helper::wrap_angle(desired_state.yaw - msg.current_yaw);

    msg.position_from_ctrl = eigen_helper::to_ros_vector3(controller_local_output.position);
    msg.velocity_from_ctrl = eigen_helper::to_ros_vector3(controller_local_output.velocity);
    msg.acceleration_from_ctrl =
        eigen_helper::to_ros_vector3(controller_local_output.accel_or_force);
    msg.yaw_from_ctrl = controller_local_output.yaw;
    msg.yawrate_from_ctrl = controller_local_output.yaw_rate;

    msg.position_from_px4 = eigen_helper::to_ros_vector3(px4_local_target.position);
    msg.velocity_from_px4 = eigen_helper::to_ros_vector3(px4_local_target.velocity);
    msg.acceleration_from_px4 = eigen_helper::to_ros_vector3(px4_local_target.accel_or_force);
    msg.yaw_from_px4 = px4_local_target.yaw;
    msg.yawrate_from_px4 = px4_local_target.yaw_rate;

    controller_state_pub_.publish(msg);
}

void Raptor_Controller::update_log_snapshot() {
    LogSnapshot snapshot;
    snapshot.controller_ready = controller_ready_.load(std::memory_order_relaxed);
    snapshot.takeoff_complete = takeoff_complete_.load(std::memory_order_relaxed);
    snapshot.land_complete = land_complete_.load(std::memory_order_relaxed);
    snapshot.point_complete = point_complete_.load(std::memory_order_relaxed);
    snapshot.has_uav_odometry = has_uav_odometry_.load(std::memory_order_relaxed);
    snapshot.can_fuse = can_fuse_.load(std::memory_order_relaxed);
    snapshot.px4_state = mavros_helper_.get_state();
    snapshot.odom = uav_odometry_;
    snapshot.px4_local_target = mavros_helper_.get_target_local();
    snapshot.last_setpoint = last_setpoint_;
    snapshot.desired_state = desired_state_;
    snapshot.hover_point = hover_point_;
    snapshot.hover_yaw = hover_yaw_;

    std::lock_guard<std::mutex> lk(log_snapshot_mutex_);
    log_snapshot_ = snapshot;
}

Raptor_Controller::LogSnapshot Raptor_Controller::get_log_snapshot() const {
    std::lock_guard<std::mutex> lk(log_snapshot_mutex_);
    return log_snapshot_;
}

void Raptor_Controller::pub_px4_state_timer_cb(const ros::TimerEvent&) {
    mavros_helper_.pub_px4_state();
}

void Raptor_Controller::pub_vision_fuse_timer_cb(const ros::TimerEvent&) {
    if (!can_fuse_.exchange(false, std::memory_order_relaxed)) {
        return;
    }
    mavros_helper_.pub_vision_pose(uav_odometry_);
}
