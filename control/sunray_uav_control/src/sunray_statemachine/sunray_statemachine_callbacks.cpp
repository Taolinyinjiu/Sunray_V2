#include "sunray_statemachine/sunray_statemachine.h"

#include <cmath>

namespace {
double yaw_from_quat(const Eigen::Quaterniond &q) {
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}
} // namespace

namespace sunray_fsm {

void Sunray_StateMachine::takeoff_cmd_cb(
    const uav_control::TakeoffCmd::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  if (msg->takeoff_relative_height > 0.0) {
    fsm_param_config_.takeoff_height_m = msg->takeoff_relative_height;
  }
  if (msg->takeoff_max_velocity > 0.0) {
    fsm_param_config_.takeoff_max_vel_mps = msg->takeoff_max_velocity;
  }
  (void)handle_event(SunrayEvent::TAKEOFF_REQUEST);
}

bool Sunray_StateMachine::takeoff_srv_cb(
    uav_control::Takeoff::Request &req,
    uav_control::Takeoff::Response &res) {
  if (req.takeoff_relative_height > 0.0) {
    fsm_param_config_.takeoff_height_m = req.takeoff_relative_height;
  }
  if (req.takeoff_max_velocity > 0.0) {
    fsm_param_config_.takeoff_max_vel_mps = req.takeoff_max_velocity;
  }

  const bool ok = handle_event(SunrayEvent::TAKEOFF_REQUEST);
  res.accepted = ok;
  res.message = ok ? "takeoff request accepted" : "takeoff request rejected";
  return true;
}

void Sunray_StateMachine::land_cmd_cb(
    const uav_control::LandCmd::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  if (msg->land_type >= 0) {
    fsm_param_config_.land_type = msg->land_type;
  }
  if (msg->land_max_velocity > 0.0) {
    fsm_param_config_.land_max_vel_mps = msg->land_max_velocity;
  }
  (void)handle_event(SunrayEvent::LAND_REQUEST);
}

bool Sunray_StateMachine::land_srv_cb(uav_control::Land::Request &req,
                                      uav_control::Land::Response &res) {
  if (req.land_type >= 0) {
    fsm_param_config_.land_type = req.land_type;
  }
  if (req.land_max_velocity > 0.0) {
    fsm_param_config_.land_max_vel_mps = req.land_max_velocity;
  }

  const bool ok = handle_event(SunrayEvent::LAND_REQUEST);
  res.accepted = ok;
  res.message = ok ? "land request accepted" : "land request rejected";
  return true;
}

void Sunray_StateMachine::return_cmd_cb(
    const uav_control::ReturnHomeCmd::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  if (msg->land_max_velocity > 0.0) {
    fsm_param_config_.land_max_vel_mps = msg->land_max_velocity;
  }
  // TODO: target_pose/use_takeoff_homepoint can be handled by controller/mission layer.
  (void)handle_event(SunrayEvent::RETURN_REQUEST);
}

bool Sunray_StateMachine::return_srv_cb(
    uav_control::ReturnHome::Request &req,
    uav_control::ReturnHome::Response &res) {
  (void)req;
  const bool ok = handle_event(SunrayEvent::RETURN_REQUEST);
  res.accepted = ok;
  res.message = ok ? "return request accepted" : "return request rejected";
  return true;
}

void Sunray_StateMachine::position_cmd_cb(
    const uav_control::PositionCmd::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  const auto controller = get_controller();
  if (!controller) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] position_cmd ignored: no controller");
    return;
  }
  uav_control::TrajectoryPoint traj;
  traj.set_position(Eigen::Vector3d(msg->target_position.x,
                                    msg->target_position.y,
                                    msg->target_position.z));
  traj.set_yaw(msg->yaw);
  (void)controller->set_trajectory(traj);
  (void)handle_event(SunrayEvent::ENTER_POSITION_CONTROL);
}

bool Sunray_StateMachine::position_srv_cb(
    uav_control::PositionRequest::Request &req,
    uav_control::PositionRequest::Response &res) {
  (void)req;
  const bool ok = handle_event(SunrayEvent::ENTER_POSITION_CONTROL);
  res.accepted = ok;
  res.message = ok ? "position request accepted" : "position request rejected";
  return true;
}

void Sunray_StateMachine::velocity_cmd_cb(
    const uav_control::VelocityCmd::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  const auto controller = get_controller();
  if (!controller) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] velocity_cmd ignored: no controller");
    return;
  }
  uav_control::TrajectoryPoint traj;
  if (msg->position_ctrl) {
    traj.set_position(Eigen::Vector3d(msg->target_position.x,
                                      msg->target_position.y,
                                      msg->target_position.z));
  }
  traj.set_velocity(Eigen::Vector3d(msg->target_linear_velocity.x,
                                    msg->target_linear_velocity.y,
                                    msg->target_linear_velocity.z));
  traj.set_yaw_rate(msg->target_angular_velocity.z);
  (void)controller->set_trajectory(traj);
  (void)handle_event(SunrayEvent::ENTER_VELOCITY_CONTROL);
}

void Sunray_StateMachine::attitude_cmd_cb(
    const uav_control::AttitudeCmd::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  const auto controller = get_controller();
  if (!controller) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] attitude_cmd ignored: no controller");
    return;
  }
  double yaw = msg->yaw;
  if (!msg->yaw_ctrl_types) { // relative yaw
    const auto &state = controller->get_current_state();
    yaw += yaw_from_quat(state.orientation);
  }
  uav_control::TrajectoryPoint traj;
  traj.set_yaw(yaw);
  (void)controller->set_trajectory(traj);
  (void)handle_event(SunrayEvent::ENTER_ATTITUDE_CONTROL);
}

void Sunray_StateMachine::trajectory_cmd_cb(
    const uav_control::TrajectoryCmd::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  const auto controller = get_controller();
  if (!controller) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] trajectory_cmd ignored: no controller");
    return;
  }
  uav_control::TrajectoryPoint traj;
  traj.time_from_start = msg->time_from_start;
  traj.set_position(Eigen::Vector3d(msg->position.x, msg->position.y,
                                    msg->position.z));
  traj.set_velocity(Eigen::Vector3d(msg->velocity.x, msg->velocity.y,
                                    msg->velocity.z));
  traj.set_acceleration(Eigen::Vector3d(msg->acceleration.x,
                                        msg->acceleration.y,
                                        msg->acceleration.z));
  traj.set_jerk(
      Eigen::Vector3d(msg->jerk.x, msg->jerk.y, msg->jerk.z));
  traj.set_snap(
      Eigen::Vector3d(msg->snap.x, msg->snap.y, msg->snap.z));
  traj.set_yaw(msg->yaw);
  traj.set_yaw_rate(msg->yaw_rate);
  traj.set_yaw_acc(msg->yaw_acc);
  (void)controller->set_trajectory(traj);
  (void)handle_event(SunrayEvent::ENTER_TRAJECTORY_CONTROL);
}

void Sunray_StateMachine::complex_cmd_cb(
    const uav_control::ComplexCmd::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  // TODO: direct MAVROS passthrough is not implemented in FSM yet.
  ROS_WARN_THROTTLE(1.0,
                    "[SunrayFSM] complex_cmd received but passthrough not wired");
  (void)handle_event(SunrayEvent::ENTER_COMPLEX_CONTROL);
}

} // namespace sunray_fsm
