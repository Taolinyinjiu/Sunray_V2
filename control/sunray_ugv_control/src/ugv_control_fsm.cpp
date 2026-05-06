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

  // 机器人身份参数：统一使用 agent_name + agent_id 生成话题前缀，例如 /ugv1。
  nh_.param<std::string>("agent_name", agent_name_, "ugv");
  nh_.param<int>("agent_id", agent_id_, 1);
  if (agent_id_ < 1) {
    ROS_WARN("agent_id=%d is invalid, fallback to 1.", agent_id_);
    agent_id_ = 1;
  }
  agent_prefix_ = "/" + agent_name_ + std::to_string(agent_id_);

  nh_.param<double>("wait_velcmd_time", WAIT_VELCMD_TIME_, 5.0);
  nh_.param<double>("point_pos_tolerance", point_pos_tolerance_, 0.05);
  nh_.param<double>("point_yaw_tolerance", point_yaw_tolerance_, 0.10);

  drive_type_ = 2;
  nh_.param<int>("drive_type", drive_type_, 2);
  if (drive_type_ == 2) {
    drive_type_name_ = "differential";
    controller_.reset(new DifferentialController(nh_));
  } else {
    if (drive_type_ != 1) {
      ROS_WARN("Unknown drive_type=%d, fallback to mecanum(1).", drive_type_);
    }
    drive_type_name_ = "mecanum";
    controller_.reset(new MecanumController(nh_));
  }

  // 初始化地理围栏
  nh_.param<double>("fence_min_x", fence_min_.x(), -10.0);
  nh_.param<double>("fence_min_y", fence_min_.y(), -10.0);
  nh_.param<double>("fence_min_z", fence_min_.z(), -1.0);
  nh_.param<double>("fence_max_x", fence_max_.x(), 10.0);
  nh_.param<double>("fence_max_y", fence_max_.y(), 10.0);
  nh_.param<double>("fence_max_z", fence_max_.z(), 1.0);

  // 初始化返航点
  return_point_.setZero();
  return_yaw_ = 0.0;

  // 初始化订阅器
  sub_odom_ = nh_.subscribe(agent_prefix_ + "/sunray/localization/local_odom", 10, &UGVControlFSM::odom_callback, this);
  sub_odom_status_ = nh_.subscribe(agent_prefix_ + "/sunray/localization/odom_status", 10, &UGVControlFSM::odom_status_callback, this);
  sub_control_cmd_ = nh_.subscribe(agent_prefix_ + "/sunray/ugv_control/control_cmd", 10, &UGVControlFSM::control_cmd_callback, this);

  // 初始化发布器
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(agent_prefix_ + "/sunray/ugv_control/cmd_vel", 10);
  pub_fsm_state_ = nh_.advertise<sunray_msgs::UGVControlFSMState>(agent_prefix_ + "/sunray/ugv_control/ugv_control_fsm_state", 10);
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

void UGVControlFSM::odom_status_callback(const sunray_msgs::OdomStatus::ConstPtr& msg) {
  // 处理里程计状态信息
  // 这里可以根据需要添加逻辑
}

void UGVControlFSM::control_cmd_callback(const sunray_msgs::UGVControlCMD::ConstPtr& msg) {
  // 更新控制指令
  ugv_control_cmd_ = *msg;

  // 根据控制指令切换状态
  switch (msg->control_cmd) {
    case sunray_msgs::UGVControlCMD::HOLD:
      capture_hold_target_from_current_state();
      current_state_ = HOLD;
      break;
    case sunray_msgs::UGVControlCMD::RETURN:
      current_state_ = RETURN;
      break;
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
      current_state_ = MOVE;
      break;
    default:
      break;
  }
}

void UGVControlFSM::control_timer_callback(const ros::TimerEvent& event) {
  // 处理状态机逻辑
  switch (current_state_) {
    case INIT:
      process_init();
      break;
    case HOLD:
      process_hold();
      break;
    case RETURN:
      process_return();
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

void UGVControlFSM::process_return() {
  // RETURN状态：移动到返航点
  sunray_msgs::UGVControlCMD return_cmd;
  return_cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
  return_cmd.desired_pos.x = return_point_.x();
  return_cmd.desired_pos.y = return_point_.y();
  return_cmd.desired_pos.z = return_point_.z();
  return_cmd.desired_yaw = return_yaw_;

  // 计算控制量
  geometry_msgs::Twist twist = controller_->move_point(return_cmd);
  last_cmd_vel = twist;
  pub_cmd_vel_.publish(twist);

  // 检查是否到达目标点
  // 这里需要根据实际情况添加到达检测逻辑
  // 简化处理：假设到达目标点后切换到HOLD状态
  // if (到达目标点) {
  //   current_state_ = HOLD;
  // }
}

void UGVControlFSM::process_move() {
  geometry_msgs::Twist twist;

  // 检查命令是否超时
  ros::Duration time_since_cmd = ros::Time::now() - ugv_control_cmd_.header.stamp;
  if ((ugv_control_cmd_.control_cmd == sunray_msgs::UGVControlCMD::MOVE_VELOCITY ||
       ugv_control_cmd_.control_cmd == sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY) &&
      time_since_cmd.toSec() > WAIT_VELCMD_TIME_) {
    switch_to_hold();
    return;
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
                          "differential_drive does not support VELOCITY (world-frame vx/vy). "
                          "Use POINT or VELOCITY_BODY with vx/wz instead.");
        switch_to_hold();
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
                            "differential_drive ignores VELOCITY_BODY linear.y=%.3f and uses 0.0 instead.",
                            twist.linear.y);
        }
        twist.linear.y = 0.0;
      }
      last_cmd_vel = twist;
      pub_cmd_vel_.publish(twist);
      break;
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
      // 保留接口，不做处理
      break;
    default:
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
  return pos_error <= point_pos_tolerance_ && yaw_error <= point_yaw_tolerance_;
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

void UGVControlFSM::switch_to_hold() {
  // 对于点位控制，到点后保持原目标点更直观；其他场景退回当前点。
  if (ugv_control_cmd_.control_cmd == sunray_msgs::UGVControlCMD::MOVE_POINT) {
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
  sunray_msgs::UGVControlFSMState state_msg;
  state_msg.header.stamp = ros::Time::now();

  state_msg.agent_name = agent_name_;
  state_msg.agent_id = static_cast<uint8_t>(agent_id_);
  state_msg.drive_type = (drive_type_ == 1) ? sunray_msgs::UGVControlFSMState::DRIVE_MECANUM
                                            : sunray_msgs::UGVControlFSMState::DRIVE_DIFFERENTIAL;

  switch (current_state_) {
    case INIT:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::FSM_INIT;
      break;
    case HOLD:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::FSM_HOLD;
      break;
    case RETURN:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::FSM_RETURN;
      break;
    case MOVE:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::FSM_MOVE;
      break;
    default:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::FSM_INIT;
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
  } else if (current_state_ == RETURN) {
    state_msg.target_valid = true;
    state_msg.target_pos.x = return_point_.x();
    state_msg.target_pos.y = return_point_.y();
    state_msg.target_pos.z = return_point_.z();
    state_msg.target_yaw = return_yaw_;
  }

  state_msg.controller_cmd_vel = last_cmd_vel;

  state_msg.odom_valid = have_odom_;
  state_msg.control_cmd_valid = ugv_control_cmd_.control_cmd != sunray_msgs::UGVControlCMD::UNDEFINE;
  state_msg.inside_geo_fence =
      !have_odom_ ||
      (current_pos_.x() >= fence_min_.x() && current_pos_.x() <= fence_max_.x() &&
       current_pos_.y() >= fence_min_.y() && current_pos_.y() <= fence_max_.y() &&
       current_pos_.z() >= fence_min_.z() && current_pos_.z() <= fence_max_.z());

  state_msg.geo_fence_min.x = fence_min_.x();
  state_msg.geo_fence_min.y = fence_min_.y();
  state_msg.geo_fence_min.z = fence_min_.z();
  state_msg.geo_fence_max.x = fence_max_.x();
  state_msg.geo_fence_max.y = fence_max_.y();
  state_msg.geo_fence_max.z = fence_max_.z();

  pub_fsm_state_.publish(state_msg);
}

void UGVControlFSM::publish_debug() {
  // 发布调试信息
  pub_debug_.publish(ugv_control_cmd_);
}

} // namespace sunray_ugv_control
