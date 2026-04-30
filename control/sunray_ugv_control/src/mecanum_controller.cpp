#include "sunray_ugv_control/mecanum_controller.h"

#include <cmath>

namespace sunray_ugv_control {

MecanumController::MecanumController(ros::NodeHandle& nh) : UGVControllerBase(nh) {
}

MecanumController::~MecanumController() {
}

geometry_msgs::Twist MecanumController::move_point(const sunray_msgs::UGVControlCMD& cmd) {
  geometry_msgs::Twist twist;

  const Eigen::Vector3d desired_pos(cmd.desired_pos.x, cmd.desired_pos.y, cmd.desired_pos.z);
  const Eigen::Vector3d pos_error = desired_pos - current_state_.pos;
  const Eigen::Vector2d body_error = world_to_body(Eigen::Vector2d(pos_error.x(), pos_error.y()), current_state_.yaw);
  twist.linear.x = params_.kp_linear * body_error.x();
  twist.linear.y = params_.kp_linear * body_error.y();

  const double linear_vel = std::sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y);
  if (linear_vel > params_.max_linear_vel) {
    twist.linear.x = twist.linear.x * params_.max_linear_vel / linear_vel;
    twist.linear.y = twist.linear.y * params_.max_linear_vel / linear_vel;
  }

  const double yaw_error = wrap_angle(cmd.desired_yaw - current_state_.yaw);
  twist.angular.z = clamp(params_.kp_angular * yaw_error,
                          -params_.max_angular_vel,
                          params_.max_angular_vel);
  return twist;
}

geometry_msgs::Twist MecanumController::move_velocity(const sunray_msgs::UGVControlCMD& cmd) {
  geometry_msgs::Twist twist;

  const Eigen::Vector2d desired_vel_world(cmd.desired_vel.x, cmd.desired_vel.y);
  const Eigen::Vector2d desired_vel_body = world_to_body(desired_vel_world, current_state_.yaw);
  twist.linear.x = desired_vel_body.x();
  twist.linear.y = desired_vel_body.y();

  const double yaw_error = wrap_angle(cmd.desired_yaw - current_state_.yaw);
  twist.angular.z = params_.kp_angular * yaw_error;

  const double linear_vel = std::sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y);
  if (linear_vel > params_.max_linear_vel) {
    twist.linear.x = twist.linear.x * params_.max_linear_vel / linear_vel;
    twist.linear.y = twist.linear.y * params_.max_linear_vel / linear_vel;
  }
  twist.angular.z = clamp(twist.angular.z, -params_.max_angular_vel, params_.max_angular_vel);

  return twist;
}

bool MecanumController::supports_world_velocity() const {
  return true;
}

bool MecanumController::supports_lateral_velocity() const {
  return true;
}

}  // namespace sunray_ugv_control
