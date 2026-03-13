#pragma once

#include <ros/time.h>
#include <string>

namespace sunray_fsm {

struct SunrayFSM_ParamConfig {
// -------------------基本参数-----------------------
  std::string uav_name{"uav"};
  int uav_id{1};
  double mass_kg{0.96};
  double gravity{0.98};

  int controller_type{0};             
	double controller_update_hz{100.0}; 

  std::string odom_topic_name{"/sunray/odom"};
  bool fuse_odom_to_px4{false};
	int fuse_odom_type{0};
  double fuse_odom_frequency_hz{50.0};

  double low_voltage_v{13.2};
  int low_voltage_action{0};

  bool control_with_no_rc{false};
  int lost_with_rc_action{1};

  bool arm_with_code{true};
  bool takeoff_with_code{true};
  bool check_flip{false};

  double fence_x_max{4.0}, fence_x_min{-4.0};
  double fence_y_max{4.0}, fence_y_min{-4.0};
  double fence_z_max{2.0}, fence_z_min{0.0};

  double timeout_odom_s{0.5};
  double timeout_rc_s{0.5};
  double timeout_control_hb_s{0.5};
  double timeout_imu_s{0.5};
  double timeout_battery_s{0.5};

  double error_tolerance_pos_x_m{0.05};
  double error_tolerance_pos_y_m{0.05};
  double error_tolerance_pos_z_m{0.05};

  double max_velocity_x_mps{3.0};
  double max_velocity_y_mps{3.0};
  double max_velocity_z_mps{1.0};

  double max_velocity_with_rc_x_mps{1.0};
  double max_velocity_with_rc_y_mps{1.0};
  double max_velocity_with_rc_z_mps{1.0};

  double tilt_angle_max_deg{20.0};

  int land_type{1};


  double takeoff_height_m{0.6};       // 条件关心
  double takeoff_max_vel_mps{0.5};    // 条件关心
  double land_max_vel_mps{0.5};
};

struct OffboardRetryConfig {
  ros::Time last_set_mode_req_time{};
  ros::Time last_arm_req_time{};
  double set_mode_retry_interval_s{1.0};
  double arm_retry_interval_s{1.0};
};


/**
 * @brief Sunray 状态机主状态集合。
 * @see https://yundrone.feishu.cn/wiki/MIz4w0vGQiAvwkkXtkJcLbEsn5d
 */
enum class SunrayState {
  OFF = 0,            ///< 待机/未激活状态。
  TAKEOFF,            ///< 起飞过程状态。
  HOVER,              ///< 悬停状态（主稳态）。
  RETURN,             ///< 返航状态
  LAND,               ///< 降落过程状态。
  EMERGENCY_LAND,     ///< 紧急降落状态。
  POSITION_CONTROL,   ///< 位置控制模式
  VELOCITY_CONTROL,   ///< 速度控制模式。
  ATTITUDE_CONTROL,   ///< 姿态控制模式。
  COMPLEX_CONTROL,    ///< 复合控制模式。
  TRAJECTORY_CONTROL, ///< 轨迹控制模式
};

/**
 * @brief 触发状态机转移的事件集合。
 */
enum class SunrayEvent {
  TAKEOFF_REQUEST = 0,      ///< 请求起飞。
  TAKEOFF_COMPLETED,        ///< 起飞完成。
  LAND_REQUEST,             ///< 请求降落。
  LAND_COMPLETED,           ///< 降落完成。
  EMERGENCY_REQUEST,        ///< 请求紧急降落。
  EMERGENCY_COMPLETED,      ///< 紧急降落完成。
  RETURN_REQUEST,           ///< 请求返航。
  RETURN_COMPLETED,         ///< 返航完成。
  WATCHDOG_ERROR,           ///< 看门狗异常。
  ENTER_POSITION_CONTROL,   ///< 进入位置控制
  ENTER_VELOCITY_CONTROL,   ///< 进入速度控制。
  ENTER_ATTITUDE_CONTROL,   ///< 进入姿态控制
  ENTER_COMPLEX_CONTROL,    ///< 进入复合控制
  ENTER_TRAJECTORY_CONTROL, ///< 进入轨迹控制
  POSITION_COMPLETED,       ///< 位置控制完成
  VELOCITY_COMPLETED,       ///< 速度控制完成
  ATTITUDE_COMPLETED,       ///< 姿态控制完成
  COMPLEX_COMPLETED,        ///< 复合控制完成
  TRAJECTORY_COMPLETED,     ///< 轨迹执行完成。
};


} // namespace sunray_fsm
