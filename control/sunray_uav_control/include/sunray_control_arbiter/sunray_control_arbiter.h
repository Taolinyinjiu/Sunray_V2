#pragma once

/**
 * @file sunray_control_arbiter.h
 * @brief Sunray 控制仲裁层声明（控制器输出 -> MAVROS 发布）。
 */

#include <array>
#include <cstdint>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

#include "controller/base_controller/base_controller.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include "sunray_statemachine/sunray_statemachine_datatypes.h"

namespace uav_control {

/**
 * @class Sunray_Control_Arbiter
 * @brief 控制输出仲裁与发布层。
 *
 * 设计目标：
 * - 与控制器同级，接收多个控制器候选输出；
 * - 基于优先级/时效/合法性进行仲裁，输出唯一控制指令；
 * - 统一执行安全门控（超时、限幅、mask 合法性检查）；
 * - 将仲裁结果映射并发布到 MAVROS 话题。
 */
class Sunray_Control_Arbiter {
public:
  /**
   * @brief 控制输出来源类型（用于固定仲裁槽位）。
   */
  enum class ControlSource : uint8_t {
    PX4_POSITION = 0,  ///< PX4 位置控制器输出
    PX4_VELOCITY = 1,  ///< PX4 速度控制器输出
    SUNRAY_ATTITUDE = 2, ///< Sunray 姿态控制器输出
    RAPTOR = 3,        ///< 强化学习控制器输出
    EMERGENCY = 4,     ///< 紧急控制输出（最高优先级候选）
    EXTERNAL = 5,      ///< 外部算法注入输出
    COUNT = 6
  };

  /**
   * @brief 仲裁层参数。
   */
  struct Config {
    double output_timeout_s{0.2};     ///< 候选输出超时阈值（秒）
    double publish_timeout_s{0.2};    ///< 最近一次仲裁结果允许发布的超时阈值（秒）
    double max_velocity_xy_mps{3.0};  ///< XY 速度限幅（m/s）
    double max_velocity_z_mps{1.5};   ///< Z 速度限幅（m/s）
    double min_thrust{0.0};           ///< 推力下限
    double max_thrust{1.0};           ///< 推力上限
    bool prefer_position_target_raw{true}; ///< 优先使用 PositionTarget 发布
    bool allow_publish_without_state{false}; ///< true 时允许无状态发布（不建议）
  };

  /**
   * @brief 候选控制输出缓存槽位。
   */
  struct CandidateSlot {
    ControllerOutput output;  ///< 控制输出
    ros::Time stamp;                       ///< 输出时间戳
    uint8_t priority{0};                   ///< 优先级（值越大优先级越高）
    bool active{false};                    ///< 槽位是否有效
  };

  Sunray_Control_Arbiter() = default;

  /**
   * @brief 初始化仲裁层（参数加载 + 发布器初始化）。
   * @param nh 节点句柄。
   * @return true 初始化成功。
   */
  bool init(ros::NodeHandle &nh);

  /**
   * @brief 更新状态机状态（用于策略切换，例如紧急态强制接管）。
   * @param state Sunray 主状态。
   */
  void set_fsm_state(sunray_fsm::SunrayState state);

  /**
   * @brief 更新当前无人机状态估计（用于安全门控和消息生成）。
   * @param state 当前状态估计。
   */
  void set_uav_state(const UAVStateEstimate &state);

  /**
   * @brief 提交某来源候选输出。
   * @param source 输出来源。
   * @param output 控制输出。
   * @param stamp 输出时间戳（默认 now）。
   * @param priority 优先级（默认 0）。
   */
  void submit(ControlSource source, const ControllerOutput &output,
              const ros::Time &stamp = ros::Time::now(), uint8_t priority = 0);

  /**
   * @brief 清空某来源槽位。
   * @param source 输出来源。
   */
  void clear(ControlSource source);

  /**
   * @brief 清空全部候选输出。
   */
  void clear_all();

  /**
   * @brief 执行一次仲裁并发布到 MAVROS 话题。
   * @return true 发布成功；false 表示无可用输出或校验失败。
   */
  bool arbitrate_and_publish();

  /**
   * @brief 获取当前配置。
   */
  const Config &config() const { return config_; }

private:
  /**
   * @brief 判断来源是否是合法枚举值。
   */
  static bool is_valid_source(ControlSource source);

  /**
   * @brief 将来源枚举转索引。
   */
  static std::size_t source_index(ControlSource source);

  /**
   * @brief 执行仲裁，得到最终发布输出。
   * @param selected 仲裁输出。
   * @param source 被选中来源。
   * @return true 表示存在可用输出。
   */
  bool select_candidate(ControllerOutput *selected,
                        ControlSource *source) const;

  /**
   * @brief 校验候选输出是否合法。
   */
  bool validate_output(const ControllerOutput &output) const;

  /**
   * @brief 对输出执行限幅处理。
   */
  void clamp_output(ControllerOutput *output) const;

  /**
   * @brief 判断槽位是否超时。
   */
  bool is_slot_fresh(const CandidateSlot &slot, const ros::Time &now) const;

  /**
   * @brief 根据 output_mask 发布到对应 MAVROS 话题。
   * @return true 发布成功。
   */
  bool publish_output(const ControllerOutput &output);

  /**
   * @brief 发布 PoseStamped 到 `/mavros/setpoint_position/local`。
   */
  bool publish_pose_setpoint(const ControllerOutput &output);

  /**
   * @brief 发布 TwistStamped 到 `/mavros/setpoint_velocity/cmd_vel`。
   */
  bool publish_velocity_setpoint(const ControllerOutput &output);

  /**
   * @brief 发布 AttitudeTarget 到 `/mavros/setpoint_raw/attitude`。
   */
  bool publish_attitude_setpoint(const ControllerOutput &output);

  /**
   * @brief 发布 PositionTarget 到 `/mavros/setpoint_raw/local`。
   */
  bool publish_position_target_raw(const ControllerOutput &output);

  /**
   * @brief 生成 PositionTarget 的 type_mask。
   */
  uint16_t make_position_target_type_mask(
      const ControllerOutput &output) const;

  /**
   * @brief 从参数服务器解析 UAV 命名空间。
   *
   * 解析顺序：
   * 1) `uav_ns`
   * 2) `uav_name` + `uav_id` 拼接
   *
   * @param nh 节点句柄。
   * @return 解析结果（不带前导 '/'，例如 `uav1`）。失败返回空字符串。
   */
  std::string resolve_uav_namespace(ros::NodeHandle &nh) const;

  Config config_{}; ///< 仲裁参数缓存
  bool initialized_{false}; ///< 初始化完成标志
  sunray_fsm::SunrayState fsm_state_{static_cast<sunray_fsm::SunrayState>(0)}; ///< 当前状态机状态（0 对应 OFF）
  UAVStateEstimate current_state_{}; ///< 当前状态估计缓存
  ros::Time last_publish_time_{}; ///< 最近一次发布时刻
  std::string resolved_uav_ns_; ///< 动态解析出的 UAV 命名空间（例如 uav1）

  std::array<CandidateSlot, static_cast<std::size_t>(ControlSource::COUNT)>
      slots_{}; ///< 候选输出槽位

  ros::Publisher pose_pub_;          ///< /mavros/setpoint_position/local
  ros::Publisher velocity_pub_;      ///< /mavros/setpoint_velocity/cmd_vel
  ros::Publisher attitude_raw_pub_;  ///< /mavros/setpoint_raw/attitude
  ros::Publisher local_raw_pub_;     ///< /mavros/setpoint_raw/local
};

} // namespace uav_control
