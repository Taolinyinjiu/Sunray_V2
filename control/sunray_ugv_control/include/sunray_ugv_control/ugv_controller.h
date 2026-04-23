#ifndef UGV_CONTROLLER_H
#define UGV_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVControllerState.h>
#include <Eigen/Eigen>

namespace sunray_ugv_control {

class UGVController {
public:
  UGVController(ros::NodeHandle& nh);
  ~UGVController();

  // 设置当前状态
  void set_current_state(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw);

  // 移动到指定点（P控制）
  geometry_msgs::Twist move_point(const sunray_msgs::UGVControlCMD& cmd);

  // 速度控制（坐标转换）
  geometry_msgs::Twist move_velocity(const sunray_msgs::UGVControlCMD& cmd);

  // 发布控制器状态
  void pub_ugv_controller_status();

private:
  // 控制器类型
  enum ControllerType {
    DIFFERENTIAL_DRIVE,
    MECANUM_WHEEL
  };

  // 状态结构体
  struct State {
    Eigen::Vector3d pos;    // 当前位置
    Eigen::Vector3d vel;    // 当前速度
    double yaw;             // 当前偏航角
  };

  // 控制器参数
  struct ControllerParams {
    double kp_linear;       // 线性位置P增益
    double kp_angular;      // 角度位置P增益
    double max_linear_vel;  // 最大线速度
    double max_angular_vel; // 最大角速度
  };

  // ROS节点
  ros::NodeHandle nh_;
  ros::Publisher pub_controller_state_;

  // 控制器类型
  ControllerType controller_type_;

  // 当前状态
  State current_state_;

  // 控制器参数
  ControllerParams params_;

  // 定时器
  ros::Timer status_timer_;

  // 初始化参数
  void init_params();

  // 定时器回调函数
  void status_timer_callback(const ros::TimerEvent& event);
};

} // namespace sunray_ugv_control

#endif // UGV_CONTROLLER_H