#include "ugv_control_fsm.h"

#include "differential_controller.h"
#include "mecanum_controller.h"

#include <cmath>

namespace sunray_ugv_control {

namespace {
double wrapAngle(const double angle) {
  double wrapped = angle;
  while (wrapped > M_PI) wrapped -= 2.0 * M_PI;
  while (wrapped < -M_PI) wrapped += 2.0 * M_PI;
  return wrapped;
}

}  // namespace

UGVControlFSM::UGVControlFSM(ros::NodeHandle& nh) : nh_(nh) {
  // 初始化状态
  current_state_ = INIT;
  current_pos_.setZero();
  current_vel_.setZero();
  current_yaw_ = 0.0;
  have_odom_ = false;
  current_odom_ = nav_msgs::Odometry();
  hold_point_.setZero();
  hold_yaw_ = 0.0;
  hold_target_valid_ = false;
  diagnostic_level_ = sunray_msgs::UGVControlState::DIAGNOSTIC_OK;
  diagnostic_msg_ = "OK";

  config_ = UGVControlConfig::loadFromRos(nh_);
  agent_prefix_ = "/" + config_.agent_name + std::to_string(config_.agent_id);

  if (config_.drive_type == 2) {
    controller_.reset(new DifferentialController(nh_));
  } else {
    if (config_.drive_type != 1) {
      ROS_WARN("%s[CONFIG] invalid drive_type=%d, fallback to mecanum(1).",
               agent_log_prefix().c_str(),
               config_.drive_type);
      config_.drive_type = 1;
    }
    controller_.reset(new MecanumController(nh_));
  }
  print_config();

  // 初始化订阅器
  sub_odom_ = nh_.subscribe(agent_prefix_ + "/sunray/localization/local_odom", 10, &UGVControlFSM::odom_callback, this);
  sub_odom_state_ = nh_.subscribe(agent_prefix_ + "/sunray/localization/odom_state", 10, &UGVControlFSM::odom_state_callback, this);
  sub_control_cmd_ = nh_.subscribe(agent_prefix_ + "/sunray/ugv_control/control_cmd", 10, &UGVControlFSM::control_cmd_callback, this);

  // 初始化发布器
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(agent_prefix_ + "/sunray/ugv_control/cmd_vel", 10);
  pub_fsm_state_ = nh_.advertise<sunray_msgs::UGVControlState>(agent_prefix_ + "/sunray/ugv_control/control_state", 10);
  pub_debug_ = nh_.advertise<sunray_msgs::UGVControlCMD>(agent_prefix_ + "/sunray/ugv_control_debug", 10);

  // 启动定时器
  control_timer_ = nh_.createTimer(ros::Duration(0.01), &UGVControlFSM::control_timer_callback, this);
  geo_fence_timer_ = nh_.createTimer(ros::Duration(0.1), &UGVControlFSM::geo_fence_timer_callback, this);
}

UGVControlFSM::~UGVControlFSM() {
}

void UGVControlFSM::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  // 提取位置、速度和偏航角
  Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Vector3d vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

  // 从四元数提取偏航角
  double yaw = 0.0;
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

  // 更新控制器状态
  current_pos_ = pos;
  current_vel_ = vel;
  current_yaw_ = yaw;
  have_odom_ = true;
  current_odom_ = *msg;
  controller_->set_current_state(pos, vel, yaw);
}

void UGVControlFSM::odom_state_callback(const sunray_msgs::OdomState::ConstPtr& msg) {
  // 处理里程计状态信息
  // 这里可以根据需要添加逻辑
}

void UGVControlFSM::control_cmd_callback(const sunray_msgs::UGVControlCMD::ConstPtr& msg) {
  // 更新控制指令
  ugv_control_cmd_ = *msg;

  // 根据控制指令切换状态
  switch (msg->control_cmd) {
    case sunray_msgs::UGVControlCMD::HOLD:
      clear_diagnostic("HOLD command accepted");
      capture_hold_target_from_current_state();
      current_state_ = HOLD;
      break;
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
      clear_diagnostic("MOVE_POINT command accepted");
      current_state_ = MOVE;
      break;
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
      clear_diagnostic("velocity command accepted");
      current_state_ = MOVE;
      break;
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
      set_diagnostic(sunray_msgs::UGVControlState::DIAGNOSTIC_WARN,
                     "MOVE_WGS84 is reserved and not executed by sunray_ugv_control");
      current_state_ = MOVE;
      break;
    default:
      set_diagnostic(sunray_msgs::UGVControlState::DIAGNOSTIC_ERROR,
                     "unknown UGVControlCMD.control_cmd, command ignored");
      break;
  }
}

void UGVControlFSM::control_timer_callback(const ros::TimerEvent& event) {
  if (config_.enable_geo_fence_protection && have_odom_ && !is_inside_geo_fence() && current_state_ != HOLD) {
    ROS_WARN_THROTTLE(1.0,
                      "%s[GEO_FENCE] rejected: current position is outside fence, switch to HOLD.",
                      agent_log_prefix().c_str());
    set_diagnostic(sunray_msgs::UGVControlState::DIAGNOSTIC_ERROR,
                   "outside geo fence, switched to HOLD");
    switch_to_hold(false);
  }

  // 处理状态机逻辑
  switch (current_state_) {
    case INIT:
      process_init();
      break;
    case HOLD:
      process_hold();
      break;
    case MOVE:
      process_move();
      break;
    default:
      break;
  }

  // 发布状态和调试信息
  publish_fsm_state();
  publish_debug();
}

void UGVControlFSM::geo_fence_timer_callback(const ros::TimerEvent& event) {
  // 这里可以添加地理围栏检测逻辑
  // 由于需要当前位置信息，实际检测逻辑会在control_timer_callback中处理
}

void UGVControlFSM::process_init() {
  // INIT状态：do nothing
}

void UGVControlFSM::process_hold() {
  // HOLD状态：持续发布零速度，确保底盘保持停止
  last_cmd_vel = geometry_msgs::Twist();
  pub_cmd_vel_.publish(last_cmd_vel);
}

void UGVControlFSM::process_move() {
  geometry_msgs::Twist twist;

  // 检查命令是否超时
  if (is_velocity_command(ugv_control_cmd_.control_cmd)) {
    if (ugv_control_cmd_.header.stamp.isZero()) {
      ROS_WARN_THROTTLE(1.0,
                        "%s[%s] rejected: header.stamp is zero, switch to HOLD.",
                        agent_log_prefix().c_str(),
                        command_name(ugv_control_cmd_.control_cmd).c_str());
      set_diagnostic(sunray_msgs::UGVControlState::DIAGNOSTIC_ERROR,
                     "velocity command header.stamp is zero, switched to HOLD");
      switch_to_hold(false);
      return;
    }

    const ros::Duration time_since_cmd = ros::Time::now() - ugv_control_cmd_.header.stamp;
    if (time_since_cmd.toSec() > config_.wait_velcmd_time) {
      ROS_WARN_THROTTLE(1.0,
                        "%s[%s] timeout: %.2fs > %.2fs, switch to HOLD.",
                        agent_log_prefix().c_str(),
                        command_name(ugv_control_cmd_.control_cmd).c_str(),
                        time_since_cmd.toSec(),
                        config_.wait_velcmd_time);
      set_diagnostic(sunray_msgs::UGVControlState::DIAGNOSTIC_ERROR,
                     "velocity command timeout, switched to HOLD");
      switch_to_hold(false);
      return;
    }
  }

  // 根据控制命令类型处理
  switch (ugv_control_cmd_.control_cmd) {
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
      if (is_point_reached(ugv_control_cmd_)) {
        switch_to_hold();
        break;
      }
      twist = controller_->move_point(ugv_control_cmd_);
      last_cmd_vel = twist;
      pub_cmd_vel_.publish(twist);
      break;
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
      if (!controller_->supports_world_velocity()) {
        ROS_WARN_THROTTLE(1.0,
                          "%s[%s] rejected: differential drive does not support world-frame velocity, "
                          "use MOVE_POINT or MOVE_VELOCITY_BODY with vx/wz, switch to HOLD.",
                          agent_log_prefix().c_str(),
                          command_name(ugv_control_cmd_.control_cmd).c_str());
        set_diagnostic(sunray_msgs::UGVControlState::DIAGNOSTIC_ERROR,
                       "differential drive does not support MOVE_VELOCITY, switched to HOLD");
        switch_to_hold(false);
        break;
      }
      twist = controller_->move_velocity(ugv_control_cmd_);
      last_cmd_vel = twist;
      pub_cmd_vel_.publish(twist);
      break;
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
      // MOVE_VELOCITY_BODY 直接复用 geometry_msgs/Twist，与底盘 cmd_vel 语义一致。
      twist = ugv_control_cmd_.cmd_vel;
      if (!controller_->supports_lateral_velocity()) {
        if (std::fabs(twist.linear.y) > 1.0e-6) {
          ROS_WARN_THROTTLE(1.0,
                            "%s[%s] degraded: differential drive ignores linear.y=%.3f and uses 0.0.",
                            agent_log_prefix().c_str(),
                            command_name(ugv_control_cmd_.control_cmd).c_str(),
                            twist.linear.y);
          set_diagnostic(sunray_msgs::UGVControlState::DIAGNOSTIC_WARN,
                         "differential drive ignores MOVE_VELOCITY_BODY linear.y");
        }
        twist.linear.y = 0.0;
      }
      last_cmd_vel = twist;
      pub_cmd_vel_.publish(twist);
      break;
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
      // 保留接口，不做处理
      set_diagnostic(sunray_msgs::UGVControlState::DIAGNOSTIC_WARN,
                     "MOVE_WGS84 is reserved and not executed by sunray_ugv_control");
      break;
    default:
      set_diagnostic(sunray_msgs::UGVControlState::DIAGNOSTIC_ERROR,
                     "unknown command in MOVE state, switched to HOLD");
      switch_to_hold(false);
      break;
  }
}

bool UGVControlFSM::is_point_reached(const sunray_msgs::UGVControlCMD& cmd) const {
  if (!have_odom_) {
    return false;
  }

  const Eigen::Vector3d desired_pos(cmd.desired_pos.x, cmd.desired_pos.y, cmd.desired_pos.z);
  const double pos_error = (desired_pos - current_pos_).head<2>().norm();
  const double yaw_error = std::fabs(wrapAngle(cmd.desired_yaw - current_yaw_));
  return pos_error <= config_.point_pos_tolerance && yaw_error <= config_.point_yaw_tolerance;
}

bool UGVControlFSM::is_inside_geo_fence() const {
  if (!have_odom_) {
    return true;
  }

  return current_pos_.x() >= config_.fence_min.x() && current_pos_.x() <= config_.fence_max.x() &&
         current_pos_.y() >= config_.fence_min.y() && current_pos_.y() <= config_.fence_max.y() &&
         current_pos_.z() >= config_.fence_min.z() && current_pos_.z() <= config_.fence_max.z();
}

bool UGVControlFSM::is_velocity_command(const uint8_t control_cmd) const {
  return control_cmd == sunray_msgs::UGVControlCMD::MOVE_VELOCITY ||
         control_cmd == sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY;
}

std::string UGVControlFSM::agent_log_prefix() const {
  return "[" + config_.agent_name + std::to_string(config_.agent_id) + "]";
}

std::string UGVControlFSM::command_name(const uint8_t control_cmd) const {
  switch (control_cmd) {
    case sunray_msgs::UGVControlCMD::UNDEFINE:
      return "UNDEFINE";
    case sunray_msgs::UGVControlCMD::HOLD:
      return "HOLD";
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
      return "MOVE_POINT";
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
      return "MOVE_VELOCITY";
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
      return "MOVE_VELOCITY_BODY";
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
      return "MOVE_WGS84";
    default:
      return "UNKNOWN";
  }
}

void UGVControlFSM::print_config() const {
  ROS_INFO("UGV control config: agent=%s%d, drive_type=%d(%s), wait_velcmd_time=%.2fs, "
           "point_tolerance=(pos %.3fm, yaw %.3frad), geo_fence_protection=%s, "
           "fence_min=(%.2f, %.2f, %.2f), fence_max=(%.2f, %.2f, %.2f)",
           config_.agent_name.c_str(),
           config_.agent_id,
           config_.drive_type,
           config_.driveTypeName().c_str(),
           config_.wait_velcmd_time,
           config_.point_pos_tolerance,
           config_.point_yaw_tolerance,
           config_.enable_geo_fence_protection ? "true" : "false",
           config_.fence_min.x(),
           config_.fence_min.y(),
           config_.fence_min.z(),
           config_.fence_max.x(),
           config_.fence_max.y(),
           config_.fence_max.z());
}

void UGVControlFSM::set_diagnostic(const uint8_t level, const std::string& msg) {
  diagnostic_level_ = level;
  diagnostic_msg_ = msg;
}

void UGVControlFSM::clear_diagnostic(const std::string& msg) {
  diagnostic_level_ = sunray_msgs::UGVControlState::DIAGNOSTIC_OK;
  diagnostic_msg_ = msg;
}

void UGVControlFSM::capture_hold_target_from_current_state() {
  if (!have_odom_) {
    return;
  }
  set_hold_target(current_pos_, current_yaw_);
}

void UGVControlFSM::set_hold_target(const Eigen::Vector3d& pos, const double yaw) {
  hold_point_ = pos;
  hold_yaw_ = yaw;
  hold_target_valid_ = true;
}

void UGVControlFSM::switch_to_hold(const bool keep_move_point_target) {
  // 对于点位控制，到点后保持原目标点更直观；其他场景退回当前点。
  if (keep_move_point_target && ugv_control_cmd_.control_cmd == sunray_msgs::UGVControlCMD::MOVE_POINT) {
    set_hold_target(Eigen::Vector3d(ugv_control_cmd_.desired_pos.x,
                                    ugv_control_cmd_.desired_pos.y,
                                    ugv_control_cmd_.desired_pos.z),
                    ugv_control_cmd_.desired_yaw);
  } else {
    capture_hold_target_from_current_state();
  }
  current_state_ = HOLD;
  ugv_control_cmd_.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
  last_cmd_vel = geometry_msgs::Twist();
  pub_cmd_vel_.publish(last_cmd_vel);
}

void UGVControlFSM::publish_fsm_state() {
  sunray_msgs::UGVControlState state_msg;
  state_msg.header.stamp = ros::Time::now();

  state_msg.agent_name = config_.agent_name;
  state_msg.agent_id = static_cast<uint8_t>(config_.agent_id);
  state_msg.drive_type = (config_.drive_type == 1) ? sunray_msgs::UGVControlState::DRIVE_MECANUM
                                                   : sunray_msgs::UGVControlState::DRIVE_DIFFERENTIAL;

  switch (current_state_) {
    case INIT:
      state_msg.fsm_state = sunray_msgs::UGVControlState::FSM_INIT;
      break;
    case HOLD:
      state_msg.fsm_state = sunray_msgs::UGVControlState::FSM_HOLD;
      break;
    case MOVE:
      state_msg.fsm_state = sunray_msgs::UGVControlState::FSM_MOVE;
      break;
    default:
      state_msg.fsm_state = sunray_msgs::UGVControlState::FSM_INIT;
      break;
  }

  state_msg.active_ugv_control_cmd = ugv_control_cmd_;
  state_msg.self_odom = current_odom_;

  state_msg.target_valid = false;
  if (current_state_ == MOVE && ugv_control_cmd_.control_cmd == sunray_msgs::UGVControlCMD::MOVE_POINT) {
    state_msg.target_valid = true;
    state_msg.target_pos = ugv_control_cmd_.desired_pos;
    state_msg.target_yaw = ugv_control_cmd_.desired_yaw;
  } else if (current_state_ == HOLD && hold_target_valid_) {
    state_msg.target_valid = true;
    state_msg.target_pos.x = hold_point_.x();
    state_msg.target_pos.y = hold_point_.y();
    state_msg.target_pos.z = hold_point_.z();
    state_msg.target_yaw = hold_yaw_;
  }

  state_msg.controller_cmd_vel = last_cmd_vel;

  state_msg.odom_valid = have_odom_;
  state_msg.control_cmd_valid = ugv_control_cmd_.control_cmd != sunray_msgs::UGVControlCMD::UNDEFINE;
  state_msg.inside_geo_fence = is_inside_geo_fence();
  state_msg.diagnostic_level = diagnostic_level_;
  state_msg.diagnostic_msg = diagnostic_msg_;

  state_msg.geo_fence_min.x = config_.fence_min.x();
  state_msg.geo_fence_min.y = config_.fence_min.y();
  state_msg.geo_fence_min.z = config_.fence_min.z();
  state_msg.geo_fence_max.x = config_.fence_max.x();
  state_msg.geo_fence_max.y = config_.fence_max.y();
  state_msg.geo_fence_max.z = config_.fence_max.z();

  pub_fsm_state_.publish(state_msg);
}

void UGVControlFSM::publish_debug() {
  // 发布调试信息
  pub_debug_.publish(ugv_control_cmd_);
}

} // namespace sunray_ugv_control
