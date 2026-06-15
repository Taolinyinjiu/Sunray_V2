#ifndef UGV_CONTROL_FSM_H
#define UGV_CONTROL_FSM_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/OdomState.h>
#include <sunray_msgs/UGVControlState.h>
#include <memory>
#include <Eigen/Eigen>
#include <cstdint>
#include <string>
#include "ugv_control_config.h"
#include "ugv_controller.h"

namespace sunray_ugv_control {

class UGVControlFSM {
public:
  UGVControlFSM(ros::NodeHandle& nh);
  ~UGVControlFSM();

private:
  // 状态枚举
  enum State {
    INIT,
    HOLD,
    MOVE
  };

  // ROS节点
  ros::NodeHandle nh_;
  UGVControlConfig config_;
  std::string agent_prefix_;  // 统一话题前缀，例如 /ugv1
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_odom_state_;
  ros::Subscriber sub_control_cmd_;
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_fsm_state_;
  ros::Publisher pub_debug_;

  // 控制器
  std::unique_ptr<UGVController> controller_;

  // 当前状态
  State current_state_;
  Eigen::Vector3d current_pos_;
  Eigen::Vector3d current_vel_;
  double current_yaw_;
  bool have_odom_;
  nav_msgs::Odometry current_odom_;

  // 控制指令
  sunray_msgs::UGVControlCMD ugv_control_cmd_;
  uint8_t diagnostic_level_;
  std::string diagnostic_msg_;

  // 最后发布的控制指令
  geometry_msgs::Twist last_cmd_vel;

  // HOLD 状态下对外公布的停车参考点
  Eigen::Vector3d hold_point_;
  double hold_yaw_;
  bool hold_target_valid_;

  // 定时器
  ros::Timer control_timer_;
  ros::Timer geo_fence_timer_;

  // 回调函数
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void odom_state_callback(const sunray_msgs::OdomState::ConstPtr& msg);
  void control_cmd_callback(const sunray_msgs::UGVControlCMD::ConstPtr& msg);
  void control_timer_callback(const ros::TimerEvent& event);
  void geo_fence_timer_callback(const ros::TimerEvent& event);

  // 状态机处理函数
  void process_init();
  void process_hold();
  void process_move();

  // 辅助函数
  bool is_point_reached(const sunray_msgs::UGVControlCMD& cmd) const;
  bool is_inside_geo_fence() const;
  bool is_velocity_command(uint8_t control_cmd) const;
  std::string agent_log_prefix() const;
  std::string command_name(uint8_t control_cmd) const;
  void print_config() const;
  void set_diagnostic(uint8_t level, const std::string& msg);
  void clear_diagnostic(const std::string& msg = "OK");
  void capture_hold_target_from_current_state();
  void set_hold_target(const Eigen::Vector3d& pos, double yaw);
  void switch_to_hold(bool keep_move_point_target = true);
  void publish_fsm_state();
  void publish_debug();
};

} // namespace sunray_ugv_control

#endif // UGV_CONTROL_FSM_H
