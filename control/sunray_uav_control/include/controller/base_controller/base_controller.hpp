/**
 * @class Base_Controller
 * @brief 无人机控制器抽象基类。
 *
 * @details
 * 该类定义了 Sunray 控制链路中“控制器组件”的统一接口契约，职责边界如下：
 * - 由上层状态机/任务层进行任务决策与模式切换；
 * - 控制器接收当前状态、目标参考与模式命令，并在 `update()` 中输出控制量；
 * - 具体控制律（位置环、速度环、姿态环、轨迹跟踪等）由子类实现。
 *
 * 典型调用顺序：
 * 1. `load_param()`：加载控制参数；
 * 2. 周期性注入状态：`set_current_odom()` / `set_px4_attitude()`；
 * 3. 按任务切换模式：`set_takeoff_mode()` / `set_land_mode()` /
 * `set_emergency_mode()`；
 * 4. 周期性检查控制器状态：`get_controller_state()`
 * 5. 周期调用 `update()` 获取控制输出。
 *
 * @note 该基类侧重“接口一致性与公共状态容器”，不负责高层任务规划。
 */

#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

#include "control_data_types/control_data_types.h"
#include "control_data_types/uav_state_estimate.hpp"

namespace uav_control {

class Base_Controller {
public:
  Base_Controller() = default;
  virtual ~Base_Controller() {} // 必须为虚析构

  /**
   * @brief 以相对高度方式切换到起飞模式。
   * @param relative_takeoff_height 相对当前高度的起飞增量，单位 m。
   * @param max_takeoff_velocity 起飞阶段最大速度，单位 m/s
   * @return true 切换成功；false 参数非法或状态不允许。
   * @note 该参数语义为“相对高度”，不是绝对世界高度。
   */
  virtual bool set_takeoff_mode(double relative_takeoff_height,
                                double max_takeoff_velocity);

  /**
   * @brief 设置飞控解锁状态。
   * @param arm_state true 表示已解锁；false 表示未解锁。
   * @return true 设置成功。
   */
  virtual bool set_px4_arm_state(bool arm_state);

  /**
   * @brief 切换到降落模式。
   * @return true 切换成功；false 切换失败。
   * @details 默认语义为“进入降落控制阶段”，降落目标由任务层与目标输入约定决定。
   */
  virtual bool set_land_mode(void);

  /**
   * @brief 切换到紧急降落模式。
   * @return true 切换成功；false 切换失败。
   * @details 紧急降落通常用于健康检查失败、链路异常等紧急场景。
   */
  virtual bool set_emergency_mode(void);

  /**
   * @brief 更新无人机当前状态估计（里程计侧）。
   * @param current_state 当前状态估计。
   * @return true 写入成功。
   */
  virtual bool set_current_odom(const UAVStateEstimate &current_state);

  /**
   * @brief 获取无人机当前状态估计（里程计侧）。
   * @return 当前状态估计只读引用。
   */
  virtual const UAVStateEstimate &get_current_state() const;

  /**
   * @brief 更新 PX4 侧姿态输入（IMU）。
   * @param imu_msg IMU 消息。
   * @return true 写入成功。
   * @note 该接口与 `set_current_odom()` 的数据源可不同，保留解耦设计。
   */
  virtual bool set_px4_attitude(const sensor_msgs::Imu &imu_msg);

  /**
   * @brief 更新 PX4 降落检测器状态
   * @param land_status
   * @return true 写入成功。
   */
  virtual bool set_px4_land_status(const bool land_status);

  /**
   * @brief 设置控制参考输入（轨迹点）。
   * @param trajectory 期望轨迹点（位置/速度/加速度/yaw 等）。
   * @return true 写入成功。
   */
  virtual bool set_trajectory(const TrajectoryPoint &trajectory);

  /**
   * @brief 获取控制器内部状态机状态。
   * @return 当前控制器状态。
   */
  virtual ControllerState get_controller_state() const;

  /**
   * @brief 判定起飞任务是否完成。
   * @return true 表示完成；false 表示未完成。
   * @note 建议子类按控制器特性（高度、速度、稳定时间）重写。
   */
  virtual bool is_takeoff_completed() const;

  /**
   * @brief 判定降落任务是否完成。
   * @return true 表示完成；false 表示未完成。
   * @note 建议子类按控制器特性（高度接地、速度阈值）重写。
   */
  virtual bool is_land_completed() const;

  /**
   * @brief 判定紧急降落任务是否完成。
   * @return true 表示完成；false 表示未完成。
   */
  virtual bool is_emergency_completed() const;

  /**
   * @brief 控制器通用就绪状态检查。
   * @return true 控制器已就绪；false 控制器还未就绪。
   * @details 建议覆盖检查项包括参数合法性、输入时效性、状态有效性等。
   */
  virtual bool is_ready() const;

  /**
   * @brief 控制律核心更新函数。
   * @return 控制输出（由 output mask 声明有效通道）。
   * @note 该函数应满足“可周期调用、无副作用泄漏、对无效输入可安全退化”。
   */
  virtual ControllerOutput update(void) = 0;

protected:
  /** ---------------基本参数----------------- */

  /** @brief 控制器就绪状态。 */
  bool controller_ready_ = false;

  /** @brief PX4 解锁状态。 */
  bool px4_arm_state_ = false;

  /** @brief 误差容限数组，通常为 `{x_tol, y_tol, z_tol}`。 */
  Eigen::Vector3d error_tolerance_ = Eigen::Vector3d(0.2,0.2,0.2);

  /** @brief 三轴最大速度参数，通常为`x_vel,y_vel,z_vel` */
  Eigen::Vector3d velocity_max_ = Eigen::Vector3d(2.0,2.0,2.0);

  /** ---------------地面参数----------------- */

  /** @brief 地面参考高度（m），通常由首次有效里程计/起飞时刻锁存。 */
  double ground_reference_z_ = 0.0;

  /** @brief 地面参考高度是否已初始化。 */
  bool ground_reference_initialized_ = false;

  /** ---------------起飞参数----------------- */
  /** @brief 起飞状态上下文 */
  bool takeoff_initialized_ = false;
	
	/** @brief 起飞时的位置 */
	Eigen::Vector3d home_position_;

  /** @brief 起飞期望位置（m）。 */
  Eigen::Vector3d takeoff_expect_position_;

  /** @brief 起飞过程中最大速度 */
  double takeoff_max_velocity_ = 0.5;

  /** @brief 计算出来的理论运动时间，小于 @param takeoff_singlecurve_limit_time
   * 则切换为多段曲线拼接式的起飞模式 */
  double takeoff_singlecurve_limit_time_ = 3.0;
	
	double takeoff_singlecurve_time_ = 0.0;

  /** @brief 起飞完成判定所需保持时间（s）。 */
  double takeoff_success_time_ = 3.0;
	
	/** @brief 起飞开始时间戳。 */
  ros::Time takeoff_start_time_ = ros::Time(0);

  /** @brief 起飞稳定区间开始时间戳。 */
  ros::Time takeoff_holdstart_time_ = ros::Time(0);

  /** @brief 起飞稳定区间最近保持时间戳。 */
  ros::Time takeoff_holdkeep_time_ = ros::Time(0);

  /** ---------------降落参数----------------- */
  /** @brief 降落类型 0:基于五次项曲线实现的降落 1:px4.auto_land */
  uint8_t land_type_ = 0;
	
	/*** @brief px4传入的降落检测状态 */	
	bool px4_land_status_ = false;

  /** @brief 降落状态上下文 */
  bool land_initialized_ = false;

  /** @brief 降落期望参考位置（m）。 */
  Eigen::Vector3d land_expect_position_;

  /** @brief 降落过程中最大速度 */
  double land_max_velocity_ = -0.5;

  /** @brief 限制运动时间 */
  double land_singlecurve_limit_time_ = 3.0;
	/*** @brief 计算出来的理论运动时间 */
	double land_singlecurve_time_ = 0.0;
	
  /** @brief 降落完成判定所需保持时间（s）。 */
  double land_success_time_ = 3.0;

			/** @brief 降落开始时间戳。 */
  ros::Time land_start_time_ = ros::Time(0);

  /** @brief 降落稳定区间开始时间戳。 */
  ros::Time land_holdstart_time_ = ros::Time(0);

  /** @brief 降落稳定区间最近保持时间戳。 */
  ros::Time land_holdkeep_time_ = ros::Time(0);

  /** ---------------运动参数----------------- */
  /** @brief 当前控制参考轨迹点。 */
  TrajectoryPoint trajectory_;

  /** @brief 控制器内部状态机当前状态。 */
  ControllerState controller_state_ = ControllerState::UNDEFINED;

  /** @brief 当前状态估计（主输入状态）。 */
  UAVStateEstimate uav_current_state_;

  /** @brief PX4 姿态输入（可与里程计来源解耦）。 */
  Eigen::Quaterniond px4_attitude_ = Eigen::Quaterniond::Identity();
};

} // namespace uav_control
