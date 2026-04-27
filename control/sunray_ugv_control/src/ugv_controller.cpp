#include "sunray_ugv_control/ugv_controller.h"

namespace sunray_ugv_control {

UGVController::PIDAxis::PIDAxis(const PIDGains& gains) : gains_(gains) {}

void UGVController::PIDAxis::reset() {
    integral_ = 0.0;
    last_error_ = 0.0;
    initialized_ = false;
}

double UGVController::PIDAxis::update(const double error, const double dt) {
    const double safe_dt = std::max(dt, 1e-3);
    integral_ += error * safe_dt;

    double derivative = 0.0;
    if (initialized_) {
        derivative = (error - last_error_) / safe_dt;
    } else {
        initialized_ = true;
    }

    last_error_ = error;
    return gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
}

UGVController::UGVController(ros::NodeHandle& nh,
                             const UGVControllerConfig& config,
                             const std::string& state_topic)
    : nh_(nh),
      config_(config),
      point_x_pid_(config.point_x),
      point_y_pid_(config.point_y),
      point_yaw_pid_(config.point_yaw),
      vel_x_pid_(config.vel_x),
      vel_y_pid_(config.vel_y),
      vel_yaw_pid_(config.vel_yaw) {
    state_pub_ = nh_.advertise<sunray_msgs::UGVControllerState>(state_topic, 10);
    state_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.state_pub_frequency)),
                                   &UGVController::status_timer_callback,
                                   this);
    last_update_stamp_ = ros::Time::now();
}

void UGVController::set_current_state(const UGVKinematicState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_ = state;
}

void UGVController::set_mode(const uint8_t control_mode) {
    if (control_mode_ == control_mode) {
        return;
    }

    control_mode_ = control_mode;
    point_x_pid_.reset();
    point_y_pid_.reset();
    point_yaw_pid_.reset();
    vel_x_pid_.reset();
    vel_y_pid_.reset();
    vel_yaw_pid_.reset();
    last_update_stamp_ = ros::Time::now();
}

double UGVController::compute_dt() {
    const ros::Time now = ros::Time::now();
    const double dt = (now - last_update_stamp_).toSec();
    last_update_stamp_ = now;
    return std::max(dt, 1e-3);
}

geometry_msgs::Twist UGVController::apply_platform_limits(const geometry_msgs::Twist& raw_cmd,
                                                          const bool direct_body_command) const {
    geometry_msgs::Twist output = raw_cmd;
    output.linear.x = clamp(output.linear.x, -config_.max_linear_x, config_.max_linear_x);
    output.linear.y = clamp(output.linear.y, -config_.max_linear_y, config_.max_linear_y);
    output.angular.z = clamp(output.angular.z, -config_.max_angular_z, config_.max_angular_z);

    if (config_.ugv_type == sunray_msgs::UGVControllerState::DIFFERENTIAL) {
        // 差速底盘无法直接执行车体侧向速度，
        // 因此把这部分需求折算为角速度，尽量让目标仍然可达。
        const double lateral_compensation =
            direct_body_command ? 0.0 : config_.lateral_to_yaw_gain * output.linear.y;
        output.linear.y = 0.0;
        output.angular.z = clamp(output.angular.z + lateral_compensation,
                                 -config_.max_angular_z,
                                 config_.max_angular_z);
    }

    return output;
}

geometry_msgs::Twist UGVController::hold() {
    std::lock_guard<std::mutex> lock(mutex_);
    set_mode(sunray_msgs::UGVControllerState::CONTROL_HOLD);
    // HOLD 会把当前位置冻结为参考点，并输出零速度。
    desired_pos_ = current_state_.position;
    desired_vel_ = geometry_msgs::Vector3{};
    desired_linear_ = geometry_msgs::Vector3{};
    desired_angular_ = geometry_msgs::Vector3{};
    desired_yaw_ = current_state_.yaw;
    last_cmd_ = geometry_msgs::Twist{};
    return last_cmd_;
}

geometry_msgs::Twist UGVController::move_point(const sunray_msgs::UGVControlCMD& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    set_mode(sunray_msgs::UGVControllerState::CONTROL_POINT);
    const double dt = compute_dt();

    // 位置控制在车体系闭环，前向/侧向误差能直接对应到底盘运动方向。
    const Vector2 pos_error_enu{cmd.desired_pos.x - current_state_.position.x,
                                cmd.desired_pos.y - current_state_.position.y};
    const Vector2 pos_error_body = enu_to_body(pos_error_enu, current_state_.yaw);
    const double yaw_error = normalize_angle(cmd.desired_yaw - current_state_.yaw);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = point_x_pid_.update(pos_error_body.x, dt);
    cmd_vel.linear.y = point_y_pid_.update(pos_error_body.y, dt);
    cmd_vel.angular.z = point_yaw_pid_.update(yaw_error, dt);

    desired_pos_ = cmd.desired_pos;
    desired_vel_ = geometry_msgs::Vector3{};
    desired_linear_ = geometry_msgs::Vector3{};
    desired_angular_ = geometry_msgs::Vector3{};
    desired_yaw_ = cmd.desired_yaw;
    last_cmd_ = apply_platform_limits(cmd_vel, false);
    return last_cmd_;
}

geometry_msgs::Twist UGVController::move_velocity(const sunray_msgs::UGVControlCMD& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    set_mode(sunray_msgs::UGVControllerState::CONTROL_VELOCITY);
    const double dt = compute_dt();

    // 速度指令以 ENU 给出，因此先把期望值和反馈值都转换到车体系，
    // 再进入 PID，这样底盘控制始终在同一坐标系下工作。
    const Vector2 desired_vel_enu{cmd.desired_vel.x, cmd.desired_vel.y};
    const Vector2 desired_vel_body = enu_to_body(desired_vel_enu, current_state_.yaw);
    const Vector2 current_vel_enu{current_state_.velocity.x, current_state_.velocity.y};
    const Vector2 current_vel_body = enu_to_body(current_vel_enu, current_state_.yaw);
    const double yaw_error = normalize_angle(cmd.desired_yaw - current_state_.yaw);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x =
        desired_vel_body.x + vel_x_pid_.update(desired_vel_body.x - current_vel_body.x, dt);
    cmd_vel.linear.y =
        desired_vel_body.y + vel_y_pid_.update(desired_vel_body.y - current_vel_body.y, dt);
    cmd_vel.angular.z = vel_yaw_pid_.update(yaw_error, dt);

    desired_pos_ = current_state_.position;
    desired_vel_ = cmd.desired_vel;
    desired_linear_.x = desired_vel_body.x;
    desired_linear_.y = desired_vel_body.y;
    desired_linear_.z = 0.0;
    desired_angular_ = geometry_msgs::Vector3{};
    desired_yaw_ = cmd.desired_yaw;
    last_cmd_ = apply_platform_limits(cmd_vel, false);
    return last_cmd_;
}

geometry_msgs::Twist UGVController::move_velocity_body(const sunray_msgs::UGVControlCMD& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    set_mode(sunray_msgs::UGVControllerState::CONTROL_VELOCITY_BODY);
    compute_dt();

    // 车体系速度模式本身已经是底盘坐标系指令，因此直接下发，不再做 PID 整形。
    desired_pos_ = current_state_.position;
    desired_linear_ = cmd.desired_linear;
    desired_angular_ = cmd.desired_angular;
    desired_yaw_ = current_state_.yaw;

    const Vector2 desired_body{cmd.desired_linear.x, cmd.desired_linear.y};
    const Vector2 desired_enu = body_to_enu(desired_body, current_state_.yaw);
    desired_vel_.x = desired_enu.x;
    desired_vel_.y = desired_enu.y;
    desired_vel_.z = cmd.desired_linear.z;

    geometry_msgs::Twist body_cmd;
    body_cmd.linear = cmd.desired_linear;
    body_cmd.angular = cmd.desired_angular;
    last_cmd_ = apply_platform_limits(body_cmd, true);
    return last_cmd_;
}

bool UGVController::reached_point(const sunray_msgs::UGVControlCMD& cmd) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const double dx = cmd.desired_pos.x - current_state_.position.x;
    const double dy = cmd.desired_pos.y - current_state_.position.y;
    const double pos_error = std::hypot(dx, dy);
    const double yaw_error = std::fabs(normalize_angle(cmd.desired_yaw - current_state_.yaw));
    return pos_error <= config_.goal_pos_tolerance && yaw_error <= config_.goal_yaw_tolerance;
}

sunray_msgs::UGVControllerState UGVController::build_status_message(const ros::Time& stamp) const {
    sunray_msgs::UGVControllerState msg;
    msg.header.stamp = stamp;
    msg.reference_frame = sunray_msgs::UGVControllerState::FRAME_BODY;
    msg.ugv_type = static_cast<uint8_t>(config_.ugv_type);
    msg.control_mode = control_mode_;

    msg.desired_pos = desired_pos_;
    msg.desired_vel = desired_vel_;
    msg.desired_linear = desired_linear_;
    msg.desired_angular = desired_angular_;
    msg.desired_yaw = desired_yaw_;

    msg.current_pos = current_state_.position;
    msg.current_vel = current_state_.velocity;
    msg.current_yaw = current_state_.yaw;

    // 该状态消息主要给 UI 和调试使用，因此同时暴露最近目标与当前瞬时误差。
    msg.pos_error.x = desired_pos_.x - current_state_.position.x;
    msg.pos_error.y = desired_pos_.y - current_state_.position.y;
    msg.pos_error.z = desired_pos_.z - current_state_.position.z;

    msg.vel_error.x = desired_vel_.x - current_state_.velocity.x;
    msg.vel_error.y = desired_vel_.y - current_state_.velocity.y;
    msg.vel_error.z = desired_vel_.z - current_state_.velocity.z;
    msg.yaw_error = normalize_angle(desired_yaw_ - current_state_.yaw);
    msg.cmd_vel = last_cmd_;
    return msg;
}

sunray_msgs::UGVControllerState UGVController::get_status_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return build_status_message(ros::Time::now());
}

void UGVController::status_timer_callback(const ros::TimerEvent&) {
    state_pub_.publish(get_status_snapshot());
}

}  // namespace sunray_ugv_control
