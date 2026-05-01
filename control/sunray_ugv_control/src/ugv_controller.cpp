#include "sunray_ugv_control/ugv_controller.h"

#include <algorithm>
#include <cmath>

namespace sunray_ugv_control {

UGVControllerBase::UGVControllerBase(ros::NodeHandle& nh) : nh_(nh) {
  current_state_.pos.setZero();
  current_state_.vel.setZero();
  current_state_.yaw = 0.0;

  init_params();

  pub_controller_state_ = nh_.advertise<sunray_msgs::UGVControllerState>("ugv_controller_state", 10);
  status_timer_ = nh_.createTimer(ros::Duration(0.01), &UGVControllerBase::status_timer_callback, this);
}

UGVControllerBase::~UGVControllerBase() {
}

void UGVControllerBase::set_current_state(const Eigen::Vector3d& pos,
                                          const Eigen::Vector3d& vel,
                                          const double yaw) {
  current_state_.pos = pos;
  current_state_.vel = vel;
  current_state_.yaw = yaw;
}

void UGVControllerBase::pub_ugv_controller_status() {
  sunray_msgs::UGVControllerState state_msg;
  state_msg.header.stamp = ros::Time::now();
  state_msg.current_pos.x = current_state_.pos.x();
  state_msg.current_pos.y = current_state_.pos.y();
  state_msg.current_pos.z = current_state_.pos.z();
  state_msg.current_vel.x = current_state_.vel.x();
  state_msg.current_vel.y = current_state_.vel.y();
  state_msg.current_vel.z = current_state_.vel.z();
  state_msg.current_yaw = current_state_.yaw;
  pub_controller_state_.publish(state_msg);
}

double UGVControllerBase::wrap_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double UGVControllerBase::clamp(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(value, upper));
}

Eigen::Vector2d UGVControllerBase::world_to_body(const Eigen::Vector2d& world_vec, const double yaw) {
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return Eigen::Vector2d(cos_yaw * world_vec.x() + sin_yaw * world_vec.y(),
                         -sin_yaw * world_vec.x() + cos_yaw * world_vec.y());
}

void UGVControllerBase::init_params() {
  nh_.param<double>("kp_linear", params_.kp_linear, 0.5);
  nh_.param<double>("kp_angular", params_.kp_angular, 1.0);
  nh_.param<double>("max_linear_vel", params_.max_linear_vel, 1.0);
  nh_.param<double>("max_angular_vel", params_.max_angular_vel, 1.0);
}

void UGVControllerBase::status_timer_callback(const ros::TimerEvent& event) {
  (void)event;
  pub_ugv_controller_status();
}

}  // namespace sunray_ugv_control
