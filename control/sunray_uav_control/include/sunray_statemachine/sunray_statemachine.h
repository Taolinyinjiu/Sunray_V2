#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <uav_control/AttitudeCmd.h>
#include <uav_control/ComplexCmd.h>
#include <uav_control/LandCmd.h>
#include <uav_control/PositionCmd.h>
#include <uav_control/ReturnHomeCmd.h>
#include <uav_control/TakeoffCmd.h>
#include <uav_control/TrajectoryCmd.h>
#include <uav_control/VelocityCmd.h>
#include <uav_control/Land.h>
#include <uav_control/PositionRequest.h>
#include <uav_control/ReturnHome.h>
#include <uav_control/Takeoff.h>

#include <px4_bridge/px4_data_reader.h>
#include <px4_bridge/px4_param_manager.h>

#include "sunray_statemachine_datatypes.h"

#include "controller/base_controller/base_controller.hpp"
#include "sunray_control_arbiter/sunray_control_arbiter.h"

namespace sunray_fsm {

/**
 * @class Sunray_StateMachine
 * @brief Sunray 飞控任务状态机。
 *
 * 主要职责：
 * - 维护当前主状态；
 * - 接收事件并执行状态转移；
 * - 管理单一控制器实例，保证各状态控制链路一致性；
 * - 在 OFF 阶段为起飞做前置条件检查。
 * -
 * 安全检查分为两部分，起飞前和起飞后，起飞前需要结合里程计来源进行PX4参数审查，起飞后主要观察里程计稳定性
 */
class Sunray_StateMachine {
  /** -----------------公开函数------外部可以调用使用------------------ */
public:
  /**
   * @brief 构造状态机。
   * @param nh ROS 节点句柄。
   */
  explicit Sunray_StateMachine(ros::NodeHandle &nh);

  /**
   * @brief 状态机事件处理入口
   * @param event 输入事件。
   * @return true 发生有效转移；false 事件被忽略或不满足转移条件。
   */
  bool handle_event(SunrayEvent event);

  /**
   * @brief 状态机周期更新函数。
   * @note 按当前状态调用控制器的 takeoff/land/emergency_land/update。
   */
  void update();

  /**
   * @brief 获取当前状态。
   * @return 当前状态枚举值。
   */
  SunrayState get_current_state() const;

  /**
   * @brief 状态枚举转字符串。
   * @param state 状态值。
   * @return 对应字符串常量。
   */
  static const char *to_string(SunrayState state);

  /**
   * @brief 事件枚举转字符串。
   * @param event 事件值。
   * @return 对应字符串常量。
   */
  static const char *to_string(SunrayEvent event);

  /** -----------------私有函数/变量------状态机内部使用------------------ */
private:
  /**
   * @brief 根据参数选择控制器类型，注册到状态机中
   * @param controller_types 控制器类型
   * @return true 注册成功；false 注册失败。
   * @note 状态机所有状态共享同一个控制器对象，以避免多控制器状态不一致问题。
   */
  bool register_controller(int controller_types);
	// 控制器更新函数，包含（控制器里程计注入+控制器期望位置更新+控制器输出量更新）
  void controller_update_timer_cb(const ros::TimerEvent &);
	// 里程计向px4融合函数，包含（根据里程计类型选择通道向px4融合）
  void odom_fuse_timer_cb(const ros::TimerEvent &);
  void external_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
  void publish_external_odom_to_px4();
  void takeoff_cmd_cb(const uav_control::TakeoffCmd::ConstPtr &msg);
  void land_cmd_cb(const uav_control::LandCmd::ConstPtr &msg);
  void return_cmd_cb(const uav_control::ReturnHomeCmd::ConstPtr &msg);
  void position_cmd_cb(const uav_control::PositionCmd::ConstPtr &msg);
  void velocity_cmd_cb(const uav_control::VelocityCmd::ConstPtr &msg);
  void attitude_cmd_cb(const uav_control::AttitudeCmd::ConstPtr &msg);
  void trajectory_cmd_cb(const uav_control::TrajectoryCmd::ConstPtr &msg);
  void complex_cmd_cb(const uav_control::ComplexCmd::ConstPtr &msg);
  bool takeoff_srv_cb(uav_control::Takeoff::Request &req,
                      uav_control::Takeoff::Response &res);
  bool land_srv_cb(uav_control::Land::Request &req,
                   uav_control::Land::Response &res);
  bool return_srv_cb(uav_control::ReturnHome::Request &req,
                     uav_control::ReturnHome::Response &res);
  bool position_srv_cb(uav_control::PositionRequest::Request &req,
                       uav_control::PositionRequest::Response &res);

  /**
   * @brief 解析 UAV 命名空间（uav_ns 或 uav_name+uav_id）。
   * @return 例如 "uav1"，失败返回空字符串。
   */
  std::string resolve_uav_namespace() const;

  /**
   * @brief 在飞行相关状态确保 OFFBOARD + ARM。
   * @return true 当前已满足；false 尚未满足。# 
   */
  bool ensure_offboard_and_arm();

  /**
   * @brief 当前状态是否需要飞控处于 OFFBOARD/ARM。
   */
  bool requires_offboard() const;

  /**
   * @brief 起飞/解锁前 安全检查
   * @return true 可解锁/起飞；false 禁止解锁/起飞。
   */
  bool check_health_preflight();

  /**
   * @brief 飞行过程中 安全检查
   * @return true 可继续；false 紧急降落。
   */
  bool check_health();

  /**
   * @brief 判定是否满足起飞条件。
   * @return true 可起飞；false 不可起飞。
   */
  bool can_takeoff() const;

  /**
   * @brief OFF 阶段的里程计来源合法性检查。
   * @return true 当前可用于起飞前门控；false 表示外部里程计不可用。
   */
  bool validate_offstage_odometry_source() const;

  /**
   * @brief 执行状态转移，作为public中handle_event函数的底层实现
   * @param next_state 目标状态。
   * @return true 转移成功（含自环）。
   */
  bool transition_to(SunrayState next_state);

  /**
   * @brief 获取已注册的控制器实例。
   * @return 控制器智能指针；未注册时返回 nullptr。
   */
  std::shared_ptr<uav_control::Base_Controller> get_controller() const;

  /** -----------------基础参数---------------------- */
  ros::NodeHandle nh_;                 ///< ROS 节点句柄。
	ros::NodeHandle ctrl_nh_;							///< 向外暴露的控制接口命名空间
  std::string uav_ns_;                  ///< 无人机命名空间参数
  SunrayState fsm_current_state_;          ///< 状态机当前状态。
  SunrayFSM_ParamConfig fsm_param_config_; // yaml文件中写入的参数

  /** -----------------里程计相关参数---------------------- */
  ros::Subscriber external_odom_sub_;
  uav_control::UAVStateEstimate latest_external_odom_;
  nav_msgs::Odometry latest_external_odom_msg_;
  // 根据fsm_param_config_参数中的fuse_odom_to_px4变量，决定是否将里程计与px4进行同步
  // 如果里程计有协方差，使用odometry接口，如果没有协方差，使用vision_pose接口
  ros::Publisher odom_to_px4_vision_pose_pub_;
  ros::Publisher odom_to_px4_odometry_pub_;
  ros::Timer odom_fuse_timer_;

  /** -----------------px4数据读取与参数管理---------------------- */
  PX4_DataReader px4_data_reader_;
  PX4_ParamManager px4_param_manager_;

  /** -----------------MAVROS offboard/arming 接管相关--------------------- */
  ros::ServiceClient px4_arming_client_;   ///< /<uav_ns>/mavros/cmd/arming
  ros::ServiceClient px4_set_mode_client_; ///< /<uav_ns>/mavros/set_mode
	OffboardRetryConfig px4_offboard_retry_state_;	///< 该结构体作用为切换px4模式为offboard时的重试参数，在构造时硬编码，不需要再次赋值

  /** -----------------控制器相关--------------------- */
  std::shared_ptr<uav_control::Base_Controller>
      sunray_controller_; ///< 全局唯一控制器实例。
  ros::Timer controller_update_timer_;
  uav_control::Sunray_Control_Arbiter arbiter_; ///< 控制输出仲裁与发布层。

	/** -----------------向外暴露的控制接口--------------------- */

	/** -----------------------话题订阅者--------------------- */
	ros::Subscriber takeoff_cmd_sub_;
	ros::Subscriber land_cmd_sub_;
	ros::Subscriber return_cmd_sub_;
	ros::Subscriber position_cmd_sub_;
	ros::Subscriber velocity_cmd_sub_;
	ros::Subscriber attitude_cmd_sub_;
	ros::Subscriber trajectory_cmd_sub_;
	ros::Subscriber complex_cmd_sub_;
	
	/** -----------------------服务提供者--------------------- */
	ros::ServiceServer takeoff_srv_;
	ros::ServiceServer land_srv_;
	ros::ServiceServer return_srv_;
	ros::ServiceServer position_srv_;
	
	// 加把锁防止同时触发
	std::mutex event_mutex_;
	std::deque<SunrayEvent> pending_events_;
};

} // namespace sunray_fsm
