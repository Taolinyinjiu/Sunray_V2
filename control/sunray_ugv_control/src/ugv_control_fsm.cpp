#include "sunray_ugv_control/ugv_control_fsm.h"

namespace sunray_ugv_control {

UGVControlFSM::UGVControlFSM(ros::NodeHandle& nh) : nh_(nh), controller_(nh) {
  // 初始化状态
  current_state_ = INIT;

  // 初始化参数
  WAIT_POSCMD_TIME_ = 5.0; // 位置命令超时时间（秒）
  WAIT_VELCMD_TIME_ = 5.0; // 速度命令超时时间（秒）

  // 读取ugv_id参数
  nh_.param<std::string>("ugv_id", ugv_id_, "ugv_1");

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
  sub_odom_ = nh_.subscribe("/" + ugv_id_ + "/sunray/odom", 10, &UGVControlFSM::odom_callback, this);
  sub_odom_status_ = nh_.subscribe("/" + ugv_id_ + "/sunray/odom_status", 10, &UGVControlFSM::odom_status_callback, this);
  sub_control_cmd_ = nh_.subscribe("/" + ugv_id_ + "/sunray/ugv_control/control_cmd", 10, &UGVControlFSM::control_cmd_callback, this);

  // 初始化发布器
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/" + ugv_id_ + "/sunray/cmd_vel", 10);
  pub_fsm_state_ = nh_.advertise<sunray_msgs::UGVControlFSMState>("/" + ugv_id_ + "/sunray/ugv_control_fsm_state", 10);
  pub_debug_ = nh_.advertise<sunray_msgs::UGVControlCMD>("/" + ugv_id_ + "/sunray/ugv_control_debug", 10);

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
  controller_.set_current_state(pos, vel, yaw);
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
      current_state_ = HOLD;
      break;
    case sunray_msgs::UGVControlCMD::RETURN:
      current_state_ = RETURN;
      break;
    case sunray_msgs::UGVControlCMD::POINT:
    case sunray_msgs::UGVControlCMD::VELOCITY:
    case sunray_msgs::UGVControlCMD::VELOCITY_BODY:
    case sunray_msgs::UGVControlCMD::WGS84:
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
  // HOLD状态：do nothing，不发布任何指令
}

void UGVControlFSM::process_return() {
  // RETURN状态：移动到返航点
  sunray_msgs::UGVControlCMD return_cmd;
  return_cmd.control_cmd = sunray_msgs::UGVControlCMD::POINT;
  return_cmd.desired_pos.x = return_point_.x();
  return_cmd.desired_pos.y = return_point_.y();
  return_cmd.desired_pos.z = return_point_.z();
  return_cmd.desired_yaw = return_yaw_;

  // 计算控制量
  geometry_msgs::Twist twist = controller_.move_point(return_cmd);
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
  if ((ugv_control_cmd_.control_cmd == sunray_msgs::UGVControlCMD::POINT && time_since_cmd.toSec() > WAIT_POSCMD_TIME_) ||
      ((ugv_control_cmd_.control_cmd == sunray_msgs::UGVControlCMD::VELOCITY || 
        ugv_control_cmd_.control_cmd == sunray_msgs::UGVControlCMD::VELOCITY_BODY) && 
       time_since_cmd.toSec() > WAIT_VELCMD_TIME_)) {
    current_state_ = HOLD;
    return;
  }

  // 根据控制命令类型处理
  switch (ugv_control_cmd_.control_cmd) {
    case sunray_msgs::UGVControlCMD::POINT:
      twist = controller_.move_point(ugv_control_cmd_);
      pub_cmd_vel_.publish(twist);
      break;
    case sunray_msgs::UGVControlCMD::VELOCITY:
      twist = controller_.move_velocity(ugv_control_cmd_);
      pub_cmd_vel_.publish(twist);
      break;
    case sunray_msgs::UGVControlCMD::VELOCITY_BODY:
      // 直接发布机体系速度命令
      twist.linear.x = ugv_control_cmd_.desired_linear.x;
      twist.linear.y = ugv_control_cmd_.desired_linear.y;
      twist.angular.z = ugv_control_cmd_.desired_angular.z;
      pub_cmd_vel_.publish(twist);
      break;
    case sunray_msgs::UGVControlCMD::WGS84:
      // 保留接口，不做处理
      break;
    default:
      break;
  }
}

void UGVControlFSM::publish_fsm_state() {
  sunray_msgs::UGVControlFSMState state_msg;
  state_msg.header.stamp = ros::Time::now();
  
  // 设置当前状态
  switch (current_state_) {
    case INIT:
      state_msg.state = sunray_msgs::UGVControlFSMState::INIT;
      break;
    case HOLD:
      state_msg.state = sunray_msgs::UGVControlFSMState::HOLD;
      break;
    case RETURN:
      state_msg.state = sunray_msgs::UGVControlFSMState::RETURN;
      break;
    case MOVE:
      state_msg.state = sunray_msgs::UGVControlFSMState::MOVE;
      break;
    default:
      state_msg.state = sunray_msgs::UGVControlFSMState::INIT;
      break;
  }

  pub_fsm_state_.publish(state_msg);
}

void UGVControlFSM::publish_debug() {
  // 发布调试信息
  pub_debug_.publish(ugv_control_cmd_);
}

} // namespace sunray_ugv_control