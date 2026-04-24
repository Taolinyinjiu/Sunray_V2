#include "sunray_ugv_control/ugv_control_fsm.h"

#include "sunray_ugv_control/differential_controller.h"
#include "sunray_ugv_control/mecanum_controller.h"

#include <iomanip>

namespace sunray_ugv_control {

namespace {
double wrapAngle(const double angle) {
  double wrapped = angle;
  while (wrapped > M_PI) wrapped -= 2.0 * M_PI;
  while (wrapped < -M_PI) wrapped += 2.0 * M_PI;
  return wrapped;
}

double radToDeg(const double angle) {
  return angle * 180.0 / M_PI;
}
}  // namespace

UGVControlFSM::UGVControlFSM(ros::NodeHandle& nh) : nh_(nh) {
  // 初始化状态
  current_state_ = INIT;
  current_pos_.setZero();
  current_yaw_ = 0.0;
  have_odom_ = false;

  // 读取ugv_id参数
  nh_.param<std::string>("ugv_id", ugv_id_, "ugv_1");
  nh_.param<std::string>("localization_ns", localization_ns_, "uav1");
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
  sub_odom_ = nh_.subscribe("/" + localization_ns_ + "/sunray/localization/local_odom", 10, &UGVControlFSM::odom_callback, this);
  sub_odom_status_ = nh_.subscribe("/" + localization_ns_ + "/sunray/localization/odom_status", 10, &UGVControlFSM::odom_status_callback, this);
  sub_control_cmd_ = nh_.subscribe("/" + ugv_id_ + "/sunray/ugv_control/control_cmd", 10, &UGVControlFSM::control_cmd_callback, this);

  // 初始化发布器
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/" + ugv_id_ + "/sunray/cmd_vel", 10);
  pub_fsm_state_ = nh_.advertise<sunray_msgs::UGVControlFSMState>("/" + ugv_id_ + "/sunray/ugv_control_fsm_state", 10);
  pub_debug_ = nh_.advertise<sunray_msgs::UGVControlCMD>("/" + ugv_id_ + "/sunray/ugv_control_debug", 10);

  // 启动定时器
  control_timer_ = nh_.createTimer(ros::Duration(0.01), &UGVControlFSM::control_timer_callback, this);
  geo_fence_timer_ = nh_.createTimer(ros::Duration(0.1), &UGVControlFSM::geo_fence_timer_callback, this);
  
  // 初始化状态信息打印定时器
  double status_print_frequency = 1.0;  // 1Hz
  status_print_timer_ = nh_.createTimer(ros::Duration(1.0 / status_print_frequency), &UGVControlFSM::print_status_info, this);
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
  current_yaw_ = yaw;
  have_odom_ = true;
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
    current_state_ = HOLD;
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
      // Differential drive only supports body vx + yaw_rate.
      twist.linear.x = ugv_control_cmd_.desired_linear.x;
      twist.linear.y = ugv_control_cmd_.desired_linear.y;
      twist.angular.z = ugv_control_cmd_.desired_angular.z;
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

void UGVControlFSM::switch_to_hold() {
  current_state_ = HOLD;
  ugv_control_cmd_.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
  last_cmd_vel = geometry_msgs::Twist();
  pub_cmd_vel_.publish(last_cmd_vel);
}

void UGVControlFSM::publish_fsm_state() {
  sunray_msgs::UGVControlFSMState state_msg;
  state_msg.header.stamp = ros::Time::now();
  
  // 设置当前状态
  switch (current_state_) {
    case INIT:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::INIT;
      break;
    case HOLD:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::HOLD;
      break;
    case RETURN:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::RETURN;
      break;
    case MOVE:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::MOVE;
      break;
    default:
      state_msg.fsm_state = sunray_msgs::UGVControlFSMState::INIT;
      break;
  }

  // 设置上一条收到的指令
  state_msg.ugv_control_cmd = ugv_control_cmd_;

  // 设置下发到底层的控制指令
  state_msg.cmd_vel = last_cmd_vel;

  pub_fsm_state_.publish(state_msg);
}

void UGVControlFSM::publish_debug() {
  // 发布调试信息
  pub_debug_.publish(ugv_control_cmd_);
}

void UGVControlFSM::print_status_info(const ros::TimerEvent& event) {
  (void)event;

  // 清除屏幕
  std::cout << "\033[2J\033[H";
  std::cout << std::fixed << std::setprecision(2);
  
  // 打印标题
  std::cout << "\033[1;36m==========================================\033[0m" << std::endl;
  std::cout << "\033[1;36m           UGV Control FSM Status Info       \033[0m" << std::endl;
  std::cout << "\033[1;36m==========================================\033[0m" << std::endl;
  
  // 打印时间
  ros::Time current_time = ros::Time::now();
  std::cout << "\033[1;32m[Time]\033[0m: " << current_time.toSec() << std::endl;
  
  // 打印UGV ID
  std::cout << "\033[1;32m[UGV ID]\033[0m: " << ugv_id_ << std::endl;
  std::cout << "\033[1;32m[Localization NS]\033[0m: " << localization_ns_ << std::endl;

  // 打印底盘信息
  std::cout << "\033[1;32m[Drive Base]\033[0m: " << std::endl;
  std::cout << "  Drive Type: " << drive_type_ << " (" << drive_type_name_ << ")" << std::endl;
  std::cout << "  Supports World Velocity: " << (controller_->supports_world_velocity() ? "YES" : "NO") << std::endl;
  std::cout << "  Supports Lateral Velocity: " << (controller_->supports_lateral_velocity() ? "YES" : "NO") << std::endl;
  
  // 打印FSM状态
  std::cout << "\033[1;32m[FSM State]\033[0m: ";
  switch (current_state_) {
    case INIT:
      std::cout << "\033[1;34mINIT\033[0m" << std::endl;
      break;
    case HOLD:
      std::cout << "\033[1;33mHOLD\033[0m" << std::endl;
      break;
    case RETURN:
      std::cout << "\033[1;31mRETURN\033[0m" << std::endl;
      break;
    case MOVE:
      std::cout << "\033[1;32mMOVE\033[0m" << std::endl;
      break;
    default:
      std::cout << "\033[1;33mUNKNOWN\033[0m" << std::endl;
      break;
  }

  // 打印当前状态估计
  std::cout << "\033[1;32m[Odom State]\033[0m: " << std::endl;
  std::cout << "  Odom Ready: " << (have_odom_ ? "YES" : "NO") << std::endl;
  std::cout << "  Current Pos: (" << current_pos_.x() << ", " << current_pos_.y() << ", " << current_pos_.z() << ") m" << std::endl;
  std::cout << "  Current Yaw: " << current_yaw_ << " rad (" << radToDeg(current_yaw_) << " deg)" << std::endl;
  std::cout << "  Last Cmd Vel: vx=" << last_cmd_vel.linear.x
            << " m/s, vy=" << last_cmd_vel.linear.y
            << " m/s, wz=" << last_cmd_vel.angular.z << " rad/s" << std::endl;
  
  // 打印控制指令信息
  std::cout << "\033[1;32m[Control Cmd]\033[0m: " << std::endl;
  std::cout << "  Cmd Source: " << ugv_control_cmd_.cmd_source << std::endl;
  std::cout << "  Control Cmd: ";
  switch (ugv_control_cmd_.control_cmd) {
    case sunray_msgs::UGVControlCMD::HOLD:
      std::cout << "HOLD" << std::endl;
      break;
    case sunray_msgs::UGVControlCMD::RETURN:
      std::cout << "RETURN" << std::endl;
      break;
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
      std::cout << "MOVE_POINT" << std::endl;
      std::cout << "  Desired Pos: (" << ugv_control_cmd_.desired_pos.x << ", " << ugv_control_cmd_.desired_pos.y << ", " << ugv_control_cmd_.desired_pos.z << ")" << std::endl;
      std::cout << "  Desired Yaw: " << ugv_control_cmd_.desired_yaw << " rad (" << radToDeg(ugv_control_cmd_.desired_yaw) << " deg)" << std::endl;
      if (have_odom_) {
        const double dx = ugv_control_cmd_.desired_pos.x - current_pos_.x();
        const double dy = ugv_control_cmd_.desired_pos.y - current_pos_.y();
        const double dist = std::sqrt(dx * dx + dy * dy);
        const double yaw_error = wrapAngle(ugv_control_cmd_.desired_yaw - current_yaw_);
        const bool pos_reached = dist <= point_pos_tolerance_;
        const bool yaw_reached = std::fabs(yaw_error) <= point_yaw_tolerance_;
        std::cout << "  Distance To Target: " << dist << " m" << std::endl;
        std::cout << "  Point Error: dx=" << dx << " m, dy=" << dy
                  << " m, dyaw=" << yaw_error << " rad ("
                  << radToDeg(yaw_error) << " deg)" << std::endl;
        std::cout << "  Reached Check: pos=" << (pos_reached ? "YES" : "NO")
                  << " (tol=" << point_pos_tolerance_ << " m), yaw="
                  << (yaw_reached ? "YES" : "NO") << " (tol="
                  << point_yaw_tolerance_ << " rad)" << std::endl;
      }
      break;
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
      std::cout << "MOVE_VELOCITY" << std::endl;
      std::cout << "  Desired Vel: (" << ugv_control_cmd_.desired_vel.x << ", " << ugv_control_cmd_.desired_vel.y << ", " << ugv_control_cmd_.desired_vel.z << ")" << std::endl;
      std::cout << "  Desired Yaw: " << ugv_control_cmd_.desired_yaw << " rad (" << radToDeg(ugv_control_cmd_.desired_yaw) << " deg)" << std::endl;
      break;
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
      std::cout << "MOVE_VELOCITY_BODY" << std::endl;
      std::cout << "  Desired Linear: (" << ugv_control_cmd_.desired_linear.x << ", " << ugv_control_cmd_.desired_linear.y << ", " << ugv_control_cmd_.desired_linear.z << ")" << std::endl;
      std::cout << "  Desired Angular: (" << ugv_control_cmd_.desired_angular.x << ", " << ugv_control_cmd_.desired_angular.y << ", " << ugv_control_cmd_.desired_angular.z << ")" << std::endl;
      break;
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
      std::cout << "MOVE_WGS84" << std::endl;
      std::cout << "  Lat: " << ugv_control_cmd_.lat << ", Lon: " << ugv_control_cmd_.lon << ", Alt: " << ugv_control_cmd_.alt << std::endl;
      break;
    default:
      std::cout << "UNKNOWN" << std::endl;
      break;
  }
  
  // 打印返航点信息
  std::cout << "\033[1;32m[Return Point]\033[0m: (" << return_point_.x() << ", " << return_point_.y() << ", " << return_point_.z() << ")" << std::endl;
  std::cout << "  Return Yaw: " << return_yaw_ << std::endl;
  
  // 打印地理围栏信息
  std::cout << "\033[1;32m[Geo Fence]\033[0m: " << std::endl;
  std::cout << "  Min: (" << fence_min_.x() << ", " << fence_min_.y() << ", " << fence_min_.z() << ")" << std::endl;
  std::cout << "  Max: (" << fence_max_.x() << ", " << fence_max_.y() << ", " << fence_max_.z() << ")" << std::endl;
  
  // 打印时间参数
  std::cout << "\033[1;32m[Time Params]\033[0m: " << std::endl;
  std::cout << "  Wait Vel Cmd Time: " << WAIT_VELCMD_TIME_ << "s" << std::endl;
  
  // 打印分隔线
  std::cout << "\033[1;36m==========================================\033[0m" << std::endl;
  std::cout << "\033[1;36mPress Ctrl+C to exit\033[0m" << std::endl;
  std::cout << std::defaultfloat;
}

} // namespace sunray_ugv_control
