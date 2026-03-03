/**
 * @file px4_data_types.h
 * @brief PX4 数据类型定义（状态/估计/里程计等）。
 *
 * 本文件定义 PX4 管理模块对外使用的数据结构体与枚举，
 * 以语义化结构替代上层模块中的散乱原始字段。
 */

#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <string>

namespace px4_data_types {

/**
 * @brief PX4 数据类型命名空间。
 */

/**
 * @brief PX4 飞行模式枚举。
 *
 * 说明：
 * - 枚举顺序与 MAVROS State.mode 的常见映射保持一致；
 * - 用于上层状态机与日志输出的语义化表达。
 */
enum class FlightMode : uint8_t {
  kUndefined = 0,
  kManual,
  kAcro,
  kAltctl,
  kPosctl,
  kOffboard,
  kStabilized,
  kRattitude,
  kAutoMission,
  kAutoLoiter,
  kAutoRtl,
  kAutoLand,
  kAutoRtgs,
  kAutoReady,
  kAutoTakeoff
};
/**
 * @brief PX4 着地检测状态枚举。
 *
 * 说明：
 * - 枚举顺序与 MAVROS ExtendedState 的 landed_state 常见定义一致；
 * - 用于任务逻辑中“在地/空中/起降阶段”判断。
 */
enum class LandedState : uint8_t {
  kUndefined = 0,
  kOnGround,
  kInAir,
  kTakeoff,
  kLanding
};
/**
 * @brief 飞行器当前系统状态快照。
 *
 * 目的：
 * - 聚合连接状态、电池、飞行模式等高频状态；
 * - 作为上层控制与显示模块的统一输入。
 */
struct SystemState {
  uint8_t uav_id = 0;
  std::string uav_name = "null";
  bool connected = false;
  bool armed = false;
  bool rc_input = false;
  uint8_t system_load = 0;
  float voltage = 0.0f;
  float current = 0.0f;
  float percent = 0.0f;
  FlightMode flight_mode = FlightMode::kUndefined;
  LandedState landed_state = LandedState::kUndefined;
};
/**
 * @brief EKF2 估计器能力状态。
 *
 * 说明：
 * - `state_codes` 保存原始状态位；
 * - `allow_*` 字段为业务层常用的能力判定结果。
 */
struct Ekf2State {
  uint32_t state_codes = 0;
  bool allow_stabilize = false;
  bool allow_altitude = false;
  bool allow_position = false;
};
/**
 * @brief 光流原始数据（直接对应 mavros_msgs::OpticalFlowRad）。
 *
 * 目的：
 * - 保留传感器侧原始积分量与质量信息；
 * - 作为后续速度解算与滤波的输入。
 */
struct OpticalFlow {
  double timestamp;
  uint8_t quality;
  uint32_t integration_time_us;
  float integrated_x;
  float integrated_y;
  float integrated_xgyro;
  float integrated_ygyro;
  float integrated_zgyro;
  uint32_t time_delta_distance_us;
  float distance;
};
/**
 * @brief 位姿数据（位置 + 姿态）。
 */
struct Pose {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};
/**
 * @brief 速度数据（线速度 + 角速度）。
 */
struct Velocity {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};
/**
 * @brief 里程计数据（时间戳 + 位姿 + 速度）。
 *
 * 目的：
 * - 作为轨迹控制与记录模块的统一里程计载体；
 * - 便于在模块间传递完整运动状态。
 */
struct Odometry {
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

} // namespace px4_data_types
