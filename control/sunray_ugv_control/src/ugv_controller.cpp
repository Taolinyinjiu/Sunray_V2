#include "sunray_ugv_control/ugv_controller.h"

namespace sunray_ugv_control {

UGVController::UGVController(ros::NodeHandle& nh) : nh_(nh) {
  // 初始化参数
  init_params();

  // 初始化发布器
  pub_controller_state_ = nh_.advertise<sunray_msgs::UGVControllerState>("ugv_controller_state", 10);

  // 启动状态发布定时器（100Hz）
  status_timer_ = nh_.createTimer(ros::Duration(0.01), &UGVController::status_timer_callback, this);
}

UGVController::~UGVController() {
}

void UGVController::init_params() {
  // 读取控制器类型
  std::string controller_type_str;
  nh_.param<std::string>("controller_type", controller_type_str, "mecanum_wheel");
  if (controller_type_str == "differential_drive") {
    controller_type_ = DIFFERENTIAL_DRIVE;
  } else {
    controller_type_ = MECANUM_WHEEL;
  }

  // 读取控制器参数
  nh_.param<double>("kp_linear", params_.kp_linear, 0.5);
  nh_.param<double>("kp_angular", params_.kp_angular, 1.0);
  nh_.param<double>("max_linear_vel", params_.max_linear_vel, 1.0);
  nh_.param<double>("max_angular_vel", params_.max_angular_vel, 1.0);
}

void UGVController::set_current_state(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw) {
  current_state_.pos = pos;
  current_state_.vel = vel;
  current_state_.yaw = yaw;
}

geometry_msgs::Twist UGVController::move_point(const sunray_msgs::UGVControlCMD& cmd) {
  geometry_msgs::Twist twist;

  // 计算位置误差
  Eigen::Vector3d desired_pos(cmd.desired_pos.x, cmd.desired_pos.y, cmd.desired_pos.z);
  Eigen::Vector3d pos_error = desired_pos - current_state_.pos;

  // 计算偏航角误差
  double yaw_error = cmd.desired_yaw - current_state_.yaw;
  // 归一化到[-pi, pi]
  while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

  // P控制计算速度
  double linear_vel = params_.kp_linear * pos_error.head<2>().norm();
  double angular_vel = params_.kp_angular * yaw_error;

  // 限制速度
  linear_vel = std::min(linear_vel, params_.max_linear_vel);
  angular_vel = std::min(std::max(angular_vel, -params_.max_angular_vel), params_.max_angular_vel);

  // 计算方向
  if (pos_error.head<2>().norm() > 0.01) {
    twist.linear.x = linear_vel * pos_error.x() / pos_error.head<2>().norm();
    twist.linear.y = linear_vel * pos_error.y() / pos_error.head<2>().norm();
  } else {
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
  }
  twist.angular.z = angular_vel;

  return twist;
}

geometry_msgs::Twist UGVController::move_velocity(const sunray_msgs::UGVControlCMD& cmd) {
  geometry_msgs::Twist twist;

  // 直接使用期望速度
  twist.linear.x = cmd.desired_vel.x;
  twist.linear.y = cmd.desired_vel.y;
  twist.angular.z = cmd.desired_yaw;

  // 限制速度
  double linear_vel = sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2));
  if (linear_vel > params_.max_linear_vel) {
    twist.linear.x = twist.linear.x * params_.max_linear_vel / linear_vel;
    twist.linear.y = twist.linear.y * params_.max_linear_vel / linear_vel;
  }
  twist.angular.z = std::min(std::max(twist.angular.z, -params_.max_angular_vel), params_.max_angular_vel);

  return twist;
}

void UGVController::pub_ugv_controller_status() {
  sunray_msgs::UGVControllerState state_msg;
  state_msg.header.stamp = ros::Time::now();
  state_msg.pos.x = current_state_.pos.x();
  state_msg.pos.y = current_state_.pos.y();
  state_msg.pos.z = current_state_.pos.z();
  state_msg.vel.x = current_state_.vel.x();
  state_msg.vel.y = current_state_.vel.y();
  state_msg.vel.z = current_state_.vel.z();
  state_msg.yaw = current_state_.yaw;
  pub_controller_state_.publish(state_msg);
}

void UGVController::status_timer_callback(const ros::TimerEvent& event) {
  pub_ugv_controller_status();
}

} // namespace sunray_ugv_control