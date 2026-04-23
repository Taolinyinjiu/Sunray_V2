#ifndef UGV_CONTROL_FSM_H
#define UGV_CONTROL_FSM_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/OdomStatus.h>
#include <sunray_msgs/UGVControlFSMState.h>
#include <Eigen/Eigen>
#include "sunray_ugv_control/ugv_controller.h"
#include "sunray_ugv_control/ugv_control_utils.h"

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
  std::string ugv_id_; // 无人车id
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_odom_status_;
  ros::Subscriber sub_control_cmd_;
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_fsm_state_;
  ros::Publisher pub_debug_;

  // 控制器
  UGVController controller_;

  // 当前状态
  State current_state_;

  // 控制指令
  sunray_msgs::UGVControlCMD ugv_control_cmd_;
  
  // 最后发布的控制指令
  geometry_msgs::Twist last_cmd_vel;

  // 返航点
  Eigen::Vector3d return_point_;
  double return_yaw_;

  // 地理围栏
  Eigen::Vector3d fence_min_;
  Eigen::Vector3d fence_max_;

  // 时间参数
  double WAIT_POSCMD_TIME_;
  double WAIT_VELCMD_TIME_;

  // 定时器
  ros::Timer control_timer_;
  ros::Timer geo_fence_timer_;
  ros::Timer status_print_timer_;

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
  void publish_fsm_state();
  void publish_debug();
  void print_status_info();
};

} // namespace sunray_ugv_control

#endif // UGV_CONTROL_FSM_H