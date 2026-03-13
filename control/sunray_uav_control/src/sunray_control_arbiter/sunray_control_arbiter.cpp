#include "sunray_control_arbiter/sunray_control_arbiter.h"

#include <algorithm>
#include <cmath>

namespace uav_control {
namespace {
std::string trimLeadingSlash(const std::string &ns) {
  if (ns.empty()) {
    return ns;
  }
  if (ns[0] == '/') {
    return ns.substr(1);
  }
  return ns;
}

double quaternionToYaw(const Eigen::Quaterniond &q) {
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string frameIdFromState(
    const UAVStateEstimate &state_estimate) {
  switch (state_estimate.coordinate_frame) {
  case UAVStateEstimate::CoordinateFrame::WORLD:
    return "world";
  case UAVStateEstimate::CoordinateFrame::LOCAL:
    return "local";
  default:
    return "world";
  }
}
} // namespace

bool Sunray_Control_Arbiter::init(ros::NodeHandle &nh) {
  nh.param("arbiter/output_timeout_s", config_.output_timeout_s,
           config_.output_timeout_s);
  nh.param("arbiter/publish_timeout_s", config_.publish_timeout_s,
           config_.publish_timeout_s);
  nh.param("arbiter/max_velocity_xy_mps", config_.max_velocity_xy_mps,
           config_.max_velocity_xy_mps);
  nh.param("arbiter/max_velocity_z_mps", config_.max_velocity_z_mps,
           config_.max_velocity_z_mps);
  nh.param("arbiter/min_thrust", config_.min_thrust, config_.min_thrust);
  nh.param("arbiter/max_thrust", config_.max_thrust, config_.max_thrust);
  nh.param("arbiter/prefer_position_target_raw",
           config_.prefer_position_target_raw,
           config_.prefer_position_target_raw);
  nh.param("arbiter/allow_publish_without_state",
           config_.allow_publish_without_state,
           config_.allow_publish_without_state);

  if (!(config_.output_timeout_s > 0.0) || !(config_.publish_timeout_s > 0.0) ||
      !(config_.max_velocity_xy_mps > 0.0) ||
      !(config_.max_velocity_z_mps > 0.0) ||
      (config_.max_thrust < config_.min_thrust)) {
    ROS_ERROR("[SunrayArbiter] invalid params");
    initialized_ = false;
    return false;
  }

  resolved_uav_ns_ = resolve_uav_namespace(nh);
  const std::string ns_prefix =
      resolved_uav_ns_.empty() ? std::string("") : ("/" + resolved_uav_ns_);
  const std::string mavros_prefix = ns_prefix + "/mavros";

  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
      mavros_prefix + "/setpoint_position/local", 10);
  velocity_pub_ = nh.advertise<geometry_msgs::TwistStamped>(
      mavros_prefix + "/setpoint_velocity/cmd_vel", 10);
  attitude_raw_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(
      mavros_prefix + "/setpoint_raw/attitude", 10);
  local_raw_pub_ = nh.advertise<mavros_msgs::PositionTarget>(
      mavros_prefix + "/setpoint_raw/local", 10);

  clear_all();
  initialized_ = true;
  last_publish_time_ = ros::Time(0);
  ROS_INFO("[SunrayArbiter] initialized, uav_ns='%s', mavros_prefix='%s'",
           resolved_uav_ns_.c_str(), mavros_prefix.c_str());
  return true;
}

void Sunray_Control_Arbiter::set_fsm_state(sunray_fsm::SunrayState state) {
  fsm_state_ = state;
}

void Sunray_Control_Arbiter::set_uav_state(
    const UAVStateEstimate &state) {
  current_state_ = state;
}

void Sunray_Control_Arbiter::submit(
    ControlSource source, const ControllerOutput &output,
    const ros::Time &stamp, uint8_t priority) {
  if (!is_valid_source(source)) {
    ROS_WARN_THROTTLE(1.0, "[SunrayArbiter] submit ignored: invalid source");
    return;
  }
  CandidateSlot &slot = slots_[source_index(source)];
  slot.output = output;
  slot.priority = priority;
  slot.stamp = stamp;
  slot.active = true;
}

void Sunray_Control_Arbiter::clear(ControlSource source) {
  if (!is_valid_source(source)) {
    return;
  }
  slots_[source_index(source)] = CandidateSlot{};
}

void Sunray_Control_Arbiter::clear_all() {
  for (CandidateSlot &slot : slots_) {
    slot = CandidateSlot{};
  }
}

bool Sunray_Control_Arbiter::arbitrate_and_publish() {
  if (!initialized_) {
    ROS_WARN_THROTTLE(1.0, "[SunrayArbiter] not initialized");
    return false;
  }
  const ros::Time now = ros::Time::now();
  if (!last_publish_time_.isZero()) {
    const double publish_gap_s = (now - last_publish_time_).toSec();
    if (publish_gap_s > config_.publish_timeout_s) {
      ROS_WARN_THROTTLE(1.0,
                        "[SunrayArbiter] publish gap exceeds timeout: %.3f s",
                        publish_gap_s);
    }
  }

  if (!config_.allow_publish_without_state && !current_state_.isValid()) {
    ROS_WARN_THROTTLE(1.0, "[SunrayArbiter] current state invalid, publish blocked");
    return false;
  }

  ControllerOutput selected;
  ControlSource selected_source = ControlSource::PX4_POSITION;
  if (!select_candidate(&selected, &selected_source)) {
    ROS_WARN_THROTTLE(1.0, "[SunrayArbiter] no fresh candidate available");
    return false;
  }
  if (!validate_output(selected)) {
    ROS_WARN_THROTTLE(1.0, "[SunrayArbiter] selected candidate invalid");
    return false;
  }
  clamp_output(&selected);
  if (!publish_output(selected)) {
    return false;
  }

  (void)selected_source;
  last_publish_time_ = now;
  return true;
}

bool Sunray_Control_Arbiter::is_valid_source(ControlSource source) {
  const std::size_t idx = static_cast<std::size_t>(source);
  return idx < static_cast<std::size_t>(ControlSource::COUNT);
}

std::size_t Sunray_Control_Arbiter::source_index(ControlSource source) {
  return static_cast<std::size_t>(source);
}

bool Sunray_Control_Arbiter::select_candidate(
    ControllerOutput *selected, ControlSource *source) const {
  if (selected == nullptr || source == nullptr) {
    return false;
  }

  const ros::Time now = ros::Time::now();
  const bool emergency_mode =
      (fsm_state_ == sunray_fsm::SunrayState::EMERGENCY_LAND);

  bool found = false;
  bool best_is_emergency = false;
  uint8_t best_priority = 0;
  ros::Time best_stamp;
  std::size_t best_idx = 0;

  for (std::size_t i = 0; i < slots_.size(); ++i) {
    const CandidateSlot &slot = slots_[i];
    if (!slot.active || !is_slot_fresh(slot, now)) {
      continue;
    }

    const ControlSource slot_source = static_cast<ControlSource>(i);
    const bool is_emergency_source = (slot_source == ControlSource::EMERGENCY);
    if (emergency_mode && !is_emergency_source) {
      continue;
    }

    if (!found) {
      found = true;
      best_idx = i;
      best_priority = slot.priority;
      best_stamp = slot.stamp;
      best_is_emergency = is_emergency_source;
      continue;
    }

    bool replace = false;
    if (is_emergency_source != best_is_emergency) {
      replace = is_emergency_source;
    } else if (slot.priority != best_priority) {
      replace = (slot.priority > best_priority);
    } else {
      replace = (slot.stamp > best_stamp);
    }

    if (replace) {
      best_idx = i;
      best_priority = slot.priority;
      best_stamp = slot.stamp;
      best_is_emergency = is_emergency_source;
    }
  }

  if (!found) {
    return false;
  }

  *selected = slots_[best_idx].output;
  *source = static_cast<ControlSource>(best_idx);
  return true;
}

bool Sunray_Control_Arbiter::validate_output(
    const ControllerOutput &output) const {
  const bool has_pos = output.is_channel_enabled(ControllerOutputMask::POSITION);
  const bool has_vel = output.is_channel_enabled(ControllerOutputMask::VELOCITY);
  const bool has_att = output.is_channel_enabled(ControllerOutputMask::ATTITUDE);
  const bool has_thr = output.is_channel_enabled(ControllerOutputMask::THRUST);
  if (!(has_pos || has_vel || has_att || has_thr)) {
    return false;
  }

  auto finiteVec = [](const Eigen::Vector3d &v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  };
  auto finiteQuat = [](const Eigen::Quaterniond &q) {
    return std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
           std::isfinite(q.z());
  };

  if (has_pos && !finiteVec(output.position)) {
    return false;
  }
  if (has_vel && !finiteVec(output.velocity)) {
    return false;
  }
  if (has_att) {
    if (!finiteQuat(output.attitude)) {
      return false;
    }
    const double qnorm = output.attitude.norm();
    if (std::abs(qnorm - 1.0) > 1e-2) {
      return false;
    }
  }
  if (has_thr && !std::isfinite(output.thrust)) {
    return false;
  }
  return true;
}

void Sunray_Control_Arbiter::clamp_output(
    ControllerOutput *output) const {
  if (output == nullptr) {
    return;
  }

  if (output->is_channel_enabled(ControllerOutputMask::VELOCITY)) {
    const double vx = output->velocity.x();
    const double vy = output->velocity.y();
    const double xy_norm = std::sqrt(vx * vx + vy * vy);
    if (xy_norm > config_.max_velocity_xy_mps && xy_norm > 1e-9) {
      const double scale = config_.max_velocity_xy_mps / xy_norm;
      output->velocity.x() *= scale;
      output->velocity.y() *= scale;
    }
    output->velocity.z() = std::max(
        -config_.max_velocity_z_mps,
        std::min(config_.max_velocity_z_mps, output->velocity.z()));
  }

  if (output->is_channel_enabled(ControllerOutputMask::THRUST)) {
    output->thrust =
        std::max(config_.min_thrust, std::min(config_.max_thrust, output->thrust));
  }

  if (output->is_channel_enabled(ControllerOutputMask::ATTITUDE)) {
    const double qnorm = output->attitude.norm();
    if (qnorm > 1e-9 && std::isfinite(qnorm)) {
      output->attitude.normalize();
    } else {
      output->attitude = Eigen::Quaterniond::Identity();
    }
  }
}

bool Sunray_Control_Arbiter::is_slot_fresh(const CandidateSlot &slot,
                                           const ros::Time &now) const {
  if (!slot.active || slot.stamp.isZero()) {
    return false;
  }
  const double age_s = (now - slot.stamp).toSec();
  return age_s >= 0.0 && age_s <= config_.output_timeout_s;
}

bool Sunray_Control_Arbiter::publish_output(
    const ControllerOutput &output) {
  const bool has_pos = output.is_channel_enabled(ControllerOutputMask::POSITION);
  const bool has_vel = output.is_channel_enabled(ControllerOutputMask::VELOCITY);
  const bool has_att = output.is_channel_enabled(ControllerOutputMask::ATTITUDE);
  const bool has_thr = output.is_channel_enabled(ControllerOutputMask::THRUST);

  if (config_.prefer_position_target_raw && (has_pos || has_vel)) {
    return publish_position_target_raw(output);
  }
  if (has_att || has_thr) {
    return publish_attitude_setpoint(output);
  }
  if (has_pos) {
    return publish_pose_setpoint(output);
  }
  if (has_vel) {
    return publish_velocity_setpoint(output);
  }
  return false;
}

bool Sunray_Control_Arbiter::publish_pose_setpoint(
    const ControllerOutput &output) {
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frameIdFromState(current_state_);

  msg.pose.position.x = output.position.x();
  msg.pose.position.y = output.position.y();
  msg.pose.position.z = output.position.z();

  const Eigen::Quaterniond q =
      output.is_channel_enabled(ControllerOutputMask::ATTITUDE)
          ? output.attitude
          : current_state_.orientation;
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();

  pose_pub_.publish(msg);
  return true;
}

bool Sunray_Control_Arbiter::publish_velocity_setpoint(
    const ControllerOutput &output) {
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frameIdFromState(current_state_);
  msg.twist.linear.x = output.velocity.x();
  msg.twist.linear.y = output.velocity.y();
  msg.twist.linear.z = output.velocity.z();
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;
  velocity_pub_.publish(msg);
  return true;
}

bool Sunray_Control_Arbiter::publish_attitude_setpoint(
    const ControllerOutput &output) {
  mavros_msgs::AttitudeTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frameIdFromState(current_state_);
  msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                  mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                  mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
  msg.body_rate.x = 0.0;
  msg.body_rate.y = 0.0;
  msg.body_rate.z = 0.0;

  if (output.is_channel_enabled(ControllerOutputMask::ATTITUDE)) {
    msg.orientation.w = output.attitude.w();
    msg.orientation.x = output.attitude.x();
    msg.orientation.y = output.attitude.y();
    msg.orientation.z = output.attitude.z();
  } else {
    msg.type_mask |= mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  }

  if (output.is_channel_enabled(ControllerOutputMask::THRUST)) {
    msg.thrust = static_cast<float>(output.thrust);
  } else {
    msg.type_mask |= mavros_msgs::AttitudeTarget::IGNORE_THRUST;
  }

  attitude_raw_pub_.publish(msg);
  return true;
}

bool Sunray_Control_Arbiter::publish_position_target_raw(
    const ControllerOutput &output) {
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frameIdFromState(current_state_);
  msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = make_position_target_type_mask(output);

  if (output.is_channel_enabled(ControllerOutputMask::POSITION)) {
    msg.position.x = output.position.x();
    msg.position.y = output.position.y();
    msg.position.z = output.position.z();
  }

  if (output.is_channel_enabled(ControllerOutputMask::VELOCITY)) {
    msg.velocity.x = output.velocity.x();
    msg.velocity.y = output.velocity.y();
    msg.velocity.z = output.velocity.z();
  }

  msg.acceleration_or_force.x = 0.0;
  msg.acceleration_or_force.y = 0.0;
  msg.acceleration_or_force.z = 0.0;

  if (output.is_channel_enabled(ControllerOutputMask::ATTITUDE)) {
    msg.yaw = static_cast<float>(quaternionToYaw(output.attitude));
  } else {
    msg.yaw = 0.0F;
  }
  msg.yaw_rate = 0.0F;

  local_raw_pub_.publish(msg);
  return true;
}

uint16_t Sunray_Control_Arbiter::make_position_target_type_mask(
    const ControllerOutput &output) const {
  uint16_t type_mask = 0;

  if (!output.is_channel_enabled(ControllerOutputMask::POSITION)) {
    type_mask |= mavros_msgs::PositionTarget::IGNORE_PX;
    type_mask |= mavros_msgs::PositionTarget::IGNORE_PY;
    type_mask |= mavros_msgs::PositionTarget::IGNORE_PZ;
  }

  if (!output.is_channel_enabled(ControllerOutputMask::VELOCITY)) {
    type_mask |= mavros_msgs::PositionTarget::IGNORE_VX;
    type_mask |= mavros_msgs::PositionTarget::IGNORE_VY;
    type_mask |= mavros_msgs::PositionTarget::IGNORE_VZ;
  }

  type_mask |= mavros_msgs::PositionTarget::IGNORE_AFX;
  type_mask |= mavros_msgs::PositionTarget::IGNORE_AFY;
  type_mask |= mavros_msgs::PositionTarget::IGNORE_AFZ;

  if (!output.is_channel_enabled(ControllerOutputMask::ATTITUDE)) {
    type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
  }
  type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

  return type_mask;
}

std::string Sunray_Control_Arbiter::resolve_uav_namespace(
    ros::NodeHandle &nh) const {
  std::string key;
  std::string uav_ns;
  if (nh.searchParam("uav_ns", key) && nh.getParam(key, uav_ns) &&
      !uav_ns.empty()) {
    return trimLeadingSlash(uav_ns);
  }

  std::string uav_name;
  int uav_id = 0;
  bool ok_name = false;
  bool ok_id = false;

  if (nh.searchParam("uav_name", key)) {
    ok_name = nh.getParam(key, uav_name) && !uav_name.empty();
  }
  if (nh.searchParam("uav_id", key)) {
    ok_id = nh.getParam(key, uav_id);
  }
  if (ok_name && ok_id) {
    return trimLeadingSlash(uav_name + std::to_string(uav_id));
  }

  ROS_WARN(
      "[SunrayArbiter] failed to resolve uav namespace from params "
      "(uav_ns/uav_name/uav_id), fallback to global /mavros");
  return "";
}

} // namespace uav_control
