#include "differential_controller.h"

#include <cmath>

namespace sunray_ugv_control {

DifferentialController::DifferentialController(ros::NodeHandle& nh) : UGVControllerBase(nh) {
  nh_.param<bool>("enable_reverse", enable_reverse_, true);
  nh_.param<double>("reverse_angle_threshold", reverse_angle_threshold_, 2.09);
  nh_.param<double>("reverse_distance_threshold", reverse_distance_threshold_, 0.8);
  nh_.param<double>("reverse_speed_ratio", reverse_speed_ratio_, 0.6);
  nh_.param<double>("final_yaw_distance_threshold", final_yaw_distance_threshold_, 0.05);
}

DifferentialController::~DifferentialController() {
}

geometry_msgs::Twist DifferentialController::move_point(const sunray_msgs::UGVControlCMD& cmd) {
  geometry_msgs::Twist twist;

  const Eigen::Vector3d desired_pos(cmd.desired_pos.x, cmd.desired_pos.y, cmd.desired_pos.z);
  const Eigen::Vector3d pos_error = desired_pos - current_state_.pos;
  const Eigen::Vector2d pos_error_xy(pos_error.x(), pos_error.y());
  const double distance = pos_error_xy.norm();
  const double pos_tolerance = 0.03;

  if (distance <= final_yaw_distance_threshold_) {
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.angular.z = clamp(params_.kp_angular * wrap_angle(cmd.desired_yaw - current_state_.yaw),
                            -params_.max_angular_vel,
                            params_.max_angular_vel);
    return twist;
  }

  const double target_heading = std::atan2(pos_error.y(), pos_error.x());
  const double heading_error = wrap_angle(target_heading - current_state_.yaw);
  const bool reverse_allowed = enable_reverse_ &&
                               distance <= reverse_distance_threshold_ &&
                               std::fabs(heading_error) >= reverse_angle_threshold_;

  double linear_vel = clamp(params_.kp_linear * distance, 0.0, params_.max_linear_vel);
  double angular_error = heading_error;

  if (reverse_allowed) {
    angular_error = wrap_angle(heading_error - std::copysign(M_PI, heading_error));
    linear_vel = -clamp(linear_vel * reverse_speed_ratio_, 0.0, params_.max_linear_vel);
  } else {
    if (std::fabs(heading_error) > M_PI / 2.0) {
      linear_vel = 0.0;
    } else {
      linear_vel *= std::max(0.0, std::cos(heading_error));
    }
  }

  twist.linear.x = (distance > pos_tolerance) ? linear_vel : 0.0;
  twist.linear.y = 0.0;
  twist.angular.z = clamp(params_.kp_angular * angular_error,
                          -params_.max_angular_vel,
                          params_.max_angular_vel);
  return twist;
}

geometry_msgs::Twist DifferentialController::move_velocity(const sunray_msgs::UGVControlCMD& cmd) {
  geometry_msgs::Twist twist;

  const Eigen::Vector2d desired_vel_world(cmd.desired_vel.x, cmd.desired_vel.y);
  const Eigen::Vector2d desired_vel_body = world_to_body(desired_vel_world, current_state_.yaw);
  twist.linear.x = clamp(desired_vel_body.x(), -params_.max_linear_vel, params_.max_linear_vel);
  twist.linear.y = 0.0;

  const double yaw_error = wrap_angle(cmd.desired_yaw - current_state_.yaw);
  twist.angular.z = clamp(params_.kp_angular * yaw_error,
                          -params_.max_angular_vel,
                          params_.max_angular_vel);
  return twist;
}

bool DifferentialController::supports_world_velocity() const {
  return false;
}

bool DifferentialController::supports_lateral_velocity() const {
  return false;
}

}  // namespace sunray_ugv_control
