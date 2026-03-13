/**
 * @file control_data_types.h
 * @brief Sunray 控制链路通用数据类型定义。
 *
 * @details
 * 本文件集中定义控制状态枚举、控制器输入轨迹点和控制器标准输出，
 * 用于状态机、控制器和执行层之间的数据交换。
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/duration.h"
#include <Eigen/Dense>
#include <cstdint>
#include <ros/time.h>
#include <string>
#include <vector>

namespace uav_control {

/**
 * @brief 任务状态机状态定义（面向 FSM 的顶层状态）。
 */
enum class ControlState {
  UNDEFINED,
  OFF,
  TAKEOFF,
  LAND,
  EMERGENCY_LAND,
  RETURN,
  HOVER,
  POSITION_CONTROL,
  VELOCITY_CONTROL,
  ATTITUDE_CONTROL,
  COMPLEX_CONTROL,
  TRAJECTORY_CONTROL
};

/**
 * @brief 控制器内部状态定义（面向控制律阶段切换）。
 *
 * @details
 * 与 `ControlState` 区分：该枚举聚焦于控制器本体行为，
 * 用于控制器内部在起飞/悬停/移动/降落等阶段选择不同控制输出策略。
 */
enum class ControllerState {
  UNDEFINED,
  OFF,
  TAKEOFF,
  LAND,
  EMERGENCY_LAND,
  HOVER,
  MOVE
};

// 控制器标准的输入(全量轨迹)，根据不同的任务情况构造不同的轨迹参数（构造指的是FSM进行构造）
// controller.get_input(std::vector<uav_control::TrajectoryPoint> tarjectory)

/* 1. 队列优先级和抢占优先级
队列优先级指的是，同时有两个控制量到达，一个是ego_planner这种外部导航算法输出的轨迹，另一个是像遥控器/地面站这种输出的命令，本质上都会转换成控制器的标准轨迹输入
但是我们认为这两者之间是有优先级的，举例
当ego_planner进行导航时，我们突然希望他做一些别的事情，此时来自遥控器/地面站的控制量会打断原有的ego_planner的控制


抢占优先级指的是，同一队列优先级的控制量，最新的控制量会覆盖原来的控制量，举例
在地面站指点飞行，无人机进行位置控制模式，此时在运动过程中，无人机尚未到达目标点，地面站设置停止，希望在当前位置悬停，切换到HOVER
此时，新的控制量会打断旧的控制量
*/
/**
 * @brief 单个时刻的轨迹参考点。
 *
 * @details
 * 该结构用于控制器输入层，支持位置/速度/加速度/yaw 等多通道混合指令。
 * 通过 `valid_mask` 显式声明“哪些字段有效”，从而避免“数值为 0 被误判为无效”的语义歧义。
 */
struct TrajectoryPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Duration time_from_start;                         ///< 相对轨迹起点时间。
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};     ///< 位置（m）。
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};     ///< 速度（m/s）。
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()}; ///< 加速度（m/s^2）。
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};         ///< 加加速度（m/s^3）。
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};         ///< 三阶导（m/s^4）。
  double yaw{0.0};                                       ///< 偏航角（rad）。
  double yaw_rate{0.0};                                  ///< 偏航角速度（rad/s）。
  double yaw_acc{0.0};                                   ///< 偏航角加速度（rad/s^2）。
  uint32_t trajectory_id{0};                             ///< 轨迹 ID（用于抢占/覆盖识别）。

  /**
   * @brief 轨迹字段有效位掩码。
   *
   * @note
   * - 位被置位表示该字段应被控制器消费；
   * - 位未置位表示该字段应被忽略，不论字段数值是否为 0。
   */
  enum class ValidMask : uint32_t {
    UNDEFINED = 0U,      ///< 无有效字段。
    POSITION = 1U << 0,  ///< `position` 字段有效。
    VELOCITY = 1U << 1,  ///< `velocity` 字段有效。
    ACCELERATION = 1U << 2, ///< `acceleration` 字段有效。
    JERK = 1U << 3,      ///< `jerk` 字段有效。
    SNAP = 1U << 4,      ///< `snap` 字段有效。
    YAW = 1U << 5,       ///< `yaw` 字段有效。
    YAW_RATE = 1U << 6,  ///< `yaw_rate` 字段有效。
    YAW_ACC = 1U << 7    ///< `yaw_acc` 字段有效。
  };

  /**
   * @brief 使能轨迹字段有效位。
   * @param item 需要使能的掩码项。
   */
  void channel_enable(ValidMask item);

  /**
   * @brief 关闭轨迹字段有效位。
   * @param item 需要关闭的掩码项。
   */
  void channel_disable(ValidMask item);

  /**
   * @brief 查询某个轨迹字段是否被显式声明为有效。
   * @param item 掩码项。
   * @return true 有效；false 无效。
   */
  bool is_channel_enabled(ValidMask item) const;

  /**
   * @brief 清空所有轨迹字段与有效位。
   */
  void clear_all();

  /**
   * @brief 按“非零字段”推断 `valid_mask`（兼容旧调用）。
   *
   * @details
   * 用于平滑迁移旧接口：如果上游尚未显式设置 `valid_mask`，
   * 可以通过该函数把非零字段转换为有效位。
   *
   * @warning
   * 该函数会把“数值等于 0 的有效指令”推断为无效，
   * 因此新代码建议使用 `set_xxx()` 显式声明有效位。
   */
  void infer_valid_mask_from_nonzero();

  /**
   * @brief 设置位置并置位 `POSITION`。
   * @param value 位置参考值（m）。
   */
  void set_position(const Eigen::Vector3d &value);

  /**
   * @brief 清除位置并清位 `POSITION`。
   */
  void clear_position();

  /**
   * @brief 设置速度并置位 `VELOCITY`。
   * @param value 速度参考值（m/s）。
   */
  void set_velocity(const Eigen::Vector3d &value);

  /**
   * @brief 清除速度并清位 `VELOCITY`。
   */
  void clear_velocity();

  /**
   * @brief 设置加速度并置位 `ACCELERATION`。
   * @param value 加速度参考值（m/s^2）。
   */
  void set_acceleration(const Eigen::Vector3d &value);

  /**
   * @brief 清除加速度并清位 `ACCELERATION`。
   */
  void clear_acceleration();

  /**
   * @brief 设置 jerk 并置位 `JERK`。
   * @param value jerk 参考值（m/s^3）。
   */
  void set_jerk(const Eigen::Vector3d &value);

  /**
   * @brief 清除 jerk 并清位 `JERK`。
   */
  void clear_jerk();

  /**
   * @brief 设置 snap 并置位 `SNAP`。
   * @param value snap 参考值（m/s^4）。
   */
  void set_snap(const Eigen::Vector3d &value);

  /**
   * @brief 清除 snap 并清位 `SNAP`。
   */
  void clear_snap();

  /**
   * @brief 设置偏航角并置位 `YAW`。
   * @param value 偏航角（rad）。
   */
  void set_yaw(double value);

  /**
   * @brief 清除偏航角并清位 `YAW`。
   */
  void clear_yaw();

  /**
   * @brief 设置偏航角速度并置位 `YAW_RATE`。
   * @param value 偏航角速度（rad/s）。
   */
  void set_yaw_rate(double value);

  /**
   * @brief 清除偏航角速度并清位 `YAW_RATE`。
   */
  void clear_yaw_rate();

  /**
   * @brief 设置偏航角加速度并置位 `YAW_ACC`。
   * @param value 偏航角加速度（rad/s^2）。
   */
  void set_yaw_acc(double value);

  /**
   * @brief 清除偏航角加速度并清位 `YAW_ACC`。
   */
  void clear_yaw_acc();

  uint32_t valid_mask = static_cast<uint32_t>(ValidMask::UNDEFINED); ///< 字段有效位集合。
  // 来源优先级与抢占优先级
  // uint8_t source_priority;
  // uint8_t preemption_priority;
};

/**
 * @brief 控制输出掩码位定义。
 */
enum class ControllerOutputMask : uint32_t {
  UNDEFINED = 0U,
  POSITION = 1U << 0, ///< position 字段有效
  VELOCITY = 1U << 1, ///< velocity 字段有效
  ACCELERATION = 1U << 2,
  FORCE = 1U << 3,
  YAW = 1U << 4,
  YAW_RATE = 1U << 5,
  ATTITUDE = 1U << 6, ///< attitude 字段有效
  BODY_RATE = 1U << 7,
  THRUST = 1U << 8 ///< thrust 字段有效
};

// ControllerOutput output;
// output.channel_enable(uav_control::ControllerOutputMase);

/**
 * @brief 控制器标准输出。
 * @note 各字段是否有效由 output_mask 指示，FSM/执行器应按掩码消费数据。
 */
struct ControllerOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControllerOutput() = default;
  ~ControllerOutput() = default;

  /**
   * @brief 使能某个输出子项。
   * @param item 需要使能的掩码位。
   */
  void channel_enable(ControllerOutputMask item);

  /**
   * @brief 关闭某个输出子项。
   * @param item 需要关闭的掩码位。
   */
  void channel_disable(ControllerOutputMask item);

  /**
   * @brief 判断某个输出子项是否已使能。
   * @param item 掩码位。
   * @return true 已使能；false 未使能。
   */
  bool is_channel_enabled(ControllerOutputMask item) const;

  /**
   * @brief 清除所有输出字段并重置 `output_mask`。
   */
  void clear_all(void);

  // PositionTarget 对应字段
  Eigen::Vector3d position = Eigen::Vector3d::Zero(); ///< POSITION
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); ///< VELOCITY
  Eigen::Vector3d acceleration_or_force =
      Eigen::Vector3d::Zero(); ///< ACCELERATION / FORCE
  double yaw = 0.0;            ///< YAW
  double yaw_rate = 0.0;       ///< YAW_RATE

  // AttitudeTarget 对应字段
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity(); ///< ATTITUDE
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();          ///< BODY_RATE
  double thrust = 0.0;                                          ///< THRUST

  uint32_t output_mask = static_cast<uint32_t>(ControllerOutputMask::UNDEFINED);
};

// 是否应当在这里表示出控制器的期望输出，或者说控制器的期望输出是什么样的？

}; // namespace uav_control
