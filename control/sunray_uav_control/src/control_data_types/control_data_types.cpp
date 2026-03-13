#include "control_data_types/control_data_types.h"

namespace uav_control {

void TrajectoryPoint::channel_enable(ValidMask item) {
  valid_mask |= static_cast<uint32_t>(item);
}

void TrajectoryPoint::channel_disable(ValidMask item) {
  valid_mask &= ~static_cast<uint32_t>(item);
}

bool TrajectoryPoint::is_channel_enabled(ValidMask item) const {
  return (valid_mask & static_cast<uint32_t>(item)) != 0U;
}

void TrajectoryPoint::clear_all() {
  valid_mask = static_cast<uint32_t>(ValidMask::UNDEFINED);
  position = Eigen::Vector3d::Zero();
  velocity = Eigen::Vector3d::Zero();
  acceleration = Eigen::Vector3d::Zero();
  jerk = Eigen::Vector3d::Zero();
  snap = Eigen::Vector3d::Zero();
  yaw = 0.0;
  yaw_rate = 0.0;
  yaw_acc = 0.0;
}

void TrajectoryPoint::infer_valid_mask_from_nonzero() {
  valid_mask = static_cast<uint32_t>(ValidMask::UNDEFINED);
  if (!position.isZero()) {
    channel_enable(ValidMask::POSITION);
  }
  if (!velocity.isZero()) {
    channel_enable(ValidMask::VELOCITY);
  }
  if (!acceleration.isZero()) {
    channel_enable(ValidMask::ACCELERATION);
  }
  if (!jerk.isZero()) {
    channel_enable(ValidMask::JERK);
  }
  if (!snap.isZero()) {
    channel_enable(ValidMask::SNAP);
  }
  if (yaw != 0.0) {
    channel_enable(ValidMask::YAW);
  }
  if (yaw_rate != 0.0) {
    channel_enable(ValidMask::YAW_RATE);
  }
  if (yaw_acc != 0.0) {
    channel_enable(ValidMask::YAW_ACC);
  }
}

void TrajectoryPoint::set_position(const Eigen::Vector3d &value) {
  position = value;
  channel_enable(ValidMask::POSITION);
}

void TrajectoryPoint::clear_position() {
  position = Eigen::Vector3d::Zero();
  channel_disable(ValidMask::POSITION);
}

void TrajectoryPoint::set_velocity(const Eigen::Vector3d &value) {
  velocity = value;
  channel_enable(ValidMask::VELOCITY);
}

void TrajectoryPoint::clear_velocity() {
  velocity = Eigen::Vector3d::Zero();
  channel_disable(ValidMask::VELOCITY);
}

void TrajectoryPoint::set_acceleration(const Eigen::Vector3d &value) {
  acceleration = value;
  channel_enable(ValidMask::ACCELERATION);
}

void TrajectoryPoint::clear_acceleration() {
  acceleration = Eigen::Vector3d::Zero();
  channel_disable(ValidMask::ACCELERATION);
}

void TrajectoryPoint::set_jerk(const Eigen::Vector3d &value) {
  jerk = value;
  channel_enable(ValidMask::JERK);
}

void TrajectoryPoint::clear_jerk() {
  jerk = Eigen::Vector3d::Zero();
  channel_disable(ValidMask::JERK);
}

void TrajectoryPoint::set_snap(const Eigen::Vector3d &value) {
  snap = value;
  channel_enable(ValidMask::SNAP);
}

void TrajectoryPoint::clear_snap() {
  snap = Eigen::Vector3d::Zero();
  channel_disable(ValidMask::SNAP);
}

void TrajectoryPoint::set_yaw(double value) {
  yaw = value;
  channel_enable(ValidMask::YAW);
}

void TrajectoryPoint::clear_yaw() {
  yaw = 0.0;
  channel_disable(ValidMask::YAW);
}

void TrajectoryPoint::set_yaw_rate(double value) {
  yaw_rate = value;
  channel_enable(ValidMask::YAW_RATE);
}

void TrajectoryPoint::clear_yaw_rate() {
  yaw_rate = 0.0;
  channel_disable(ValidMask::YAW_RATE);
}

void TrajectoryPoint::set_yaw_acc(double value) {
  yaw_acc = value;
  channel_enable(ValidMask::YAW_ACC);
}

void TrajectoryPoint::clear_yaw_acc() {
  yaw_acc = 0.0;
  channel_disable(ValidMask::YAW_ACC);
}

void ControllerOutput::channel_enable(ControllerOutputMask item) {
  output_mask |= static_cast<uint32_t>(item);
}

void ControllerOutput::channel_disable(ControllerOutputMask item) {
  output_mask &= ~static_cast<uint32_t>(item);
}

bool ControllerOutput::is_channel_enabled(ControllerOutputMask item) const {
  return (output_mask & static_cast<uint32_t>(item)) != 0U;
}

void ControllerOutput::clear_all(void) {
  output_mask = static_cast<uint32_t>(ControllerOutputMask::UNDEFINED);
  position = Eigen::Vector3d::Zero();
  velocity = Eigen::Vector3d::Zero();
  acceleration_or_force = Eigen::Vector3d::Zero();
  yaw = 0.0;
  yaw_rate = 0.0;
  attitude = Eigen::Quaterniond::Identity();
  body_rate = Eigen::Vector3d::Zero();
  thrust = 0.0;
}

} // namespace uav_control
