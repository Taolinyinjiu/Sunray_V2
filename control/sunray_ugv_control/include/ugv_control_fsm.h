#ifndef UGV_CONTROL_FSM_H
#define UGV_CONTROL_FSM_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/OdomStatus.h>
#include <sunray_msgs/UGVControlFSMState.h>
#include <memory>
#include <Eigen/Eigen>
#include <string>
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
    RETURN,
    MOVE
  };

  // ROS节点
  ros::NodeHandle nh_;
  std::string agent_name_;    // 机器人类型名，例如 ugv
  int agent_id_;              // 机器人编号，从1开始
  std::string agent_prefix_;  // 统一话题前缀，例如 /ugv1
  int drive_type_;
  std::string drive_type_name_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_odom_status_;
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

  // 最后发布的控制指令
  geometry_msgs::Twist last_cmd_vel;

  // HOLD 状态下对外公布的停车参考点
  Eigen::Vector3d hold_point_;
  double hold_yaw_;
  bool hold_target_valid_;

  // 返航点
  Eigen::Vector3d return_point_;
  double return_yaw_;

  // 地理围栏
  Eigen::Vector3d fence_min_;
  Eigen::Vector3d fence_max_;

  // 时间参数
  double WAIT_VELCMD_TIME_;
  double point_pos_tolerance_;
  double point_yaw_tolerance_;

  // 定时器
  ros::Timer control_timer_;
  ros::Timer geo_fence_timer_;

  // 回调函数
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void odom_status_callback(const sunray_msgs::OdomStatus::ConstPtr& msg);
  void control_cmd_callback(const sunray_msgs::UGVControlCMD::ConstPtr& msg);
  void control_timer_callback(const ros::TimerEvent& event);
  void geo_fence_timer_callback(const ros::TimerEvent& event);

  // 状态机处理函数
  void process_init();
  void process_hold();
  void process_return();
  void process_move();

  // 辅助函数
  bool is_point_reached(const sunray_msgs::UGVControlCMD& cmd) const;
  void capture_hold_target_from_current_state();
  void set_hold_target(const Eigen::Vector3d& pos, double yaw);
  void switch_to_hold();
  void publish_fsm_state();
  void publish_debug();
};

} // namespace sunray_ugv_control

#endif // UGV_CONTROL_FSM_H
