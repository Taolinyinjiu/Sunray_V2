#include "ugv_controller.h"

#include <algorithm>
#include <cmath>

namespace sunray_ugv_control {

UGVControllerBase::UGVControllerBase(ros::NodeHandle& nh) : nh_(nh) {
  current_state_.pos.setZero();
  current_state_.vel.setZero();
  current_state_.yaw = 0.0;

  init_params();
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
  params_ = UGVControllerParams::loadFromRos(nh_);
  ROS_INFO("UGV controller params: kp_linear=%.3f, kp_angular=%.3f, "
           "max_linear_vel=%.3fm/s, max_angular_vel=%.3frad/s",
           params_.kp_linear,
           params_.kp_angular,
           params_.max_linear_vel,
           params_.max_angular_vel);
}

}  // namespace sunray_ugv_control
